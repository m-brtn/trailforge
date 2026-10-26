#!/usr/bin/env python3
"""Trace a vehicle blueprint from the concept sheet (external_assets/images/Cars.png).

For one car column of the sheet: crop the side / top / front / rear cells, segment the car from the uniform grey
background, fit the wheel circles (side view: scale reference and axle positions), classify the pixels into colour
classes (paint, accent, roof, dark trim / glass, lamps) and extract the profiles the body loft needs. Hand-placed
corrections live in tools/cars/<id>.keypoints.json (pixel coordinates of the cell), never in this file.

Output: assets/vehicles/<id>/blueprint.json (body space, meters: +Z forward, +X left, +Y up) and debug overlays
/tmp/trailforge_blueprint_<id>_<view>.png.

Usage (venv with numpy, Pillow, opencv-python-headless; see tools/cars/requirements.txt):
  python3 tools/cars/trace_blueprint.py trailforge_suv [--root .] [--debug-dir /tmp]
"""

import argparse
import json
import math
import os
import sys

import cv2
import numpy as np
from PIL import Image

VIEWS = ("side", "top", "front", "rear")


# ----------------------------------------------------------------------------------------------- image helpers


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def poly_mask(shape, polys, value=1):
    m = np.zeros(shape[:2], np.uint8)
    for p in polys:
        pts = np.array(p, np.float32)
        if len(pts) == 2:  # rectangle [[x0, y0], [x1, y1]]
            (x0, y0), (x1, y1) = pts
            pts = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], np.float32)
        cv2.fillPoly(m, [np.round(pts).astype(np.int32)], value)
    return m


def fill_holes(m):
    h, w = m.shape
    ff = m.copy()
    mask = np.zeros((h + 2, w + 2), np.uint8)
    for seed in ((0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1)):
        if ff[seed[1], seed[0]] == 0:
            cv2.floodFill(ff, mask, seed, 1)
    return (m | (1 - ff)).astype(np.uint8)


def background_model(cell, cfg):
    """Smooth background (quadratic in x, y) fitted to the grey, low-chroma pixels of the cell."""
    g = cell.mean(2)
    h, w = g.shape
    chroma = cell.max(2) - cell.min(2)
    yy, xx = np.mgrid[0:h, 0:w]
    xn, yn = xx / w, yy / h
    cand = (chroma < 10) & (g > 45) & (g < 90)
    A = np.stack([np.ones(h * w), xn.ravel(), yn.ravel(), xn.ravel() ** 2, yn.ravel() ** 2, (xn * yn).ravel()], 1)
    sel = cand.ravel()
    for _ in range(3):  # refit without outliers (car pixels that happen to be grey)
        coef, *_ = np.linalg.lstsq(A[sel], g.ravel()[sel], rcond=None)
        res = g.ravel() - A @ coef
        sel = sel & (np.abs(res) < 6)
    return (A @ coef).reshape(h, w)


def segment(cell, cfg, ignore_mask):
    """Car mask: pixels that differ from the background model; the contact shadow (smooth, neutral, dark, below the
    wheel centres) is removed; largest component, holes filled."""
    bgc = cfg["background"]
    g = cell.mean(2)
    chroma = cell.max(2) - cell.min(2)
    d = g - background_model(cell, cfg)
    fg = ((d < -bgc["darker"]) | (d > bgc["brighter"]) | (chroma > bgc["maxChroma"])).astype(np.uint8)
    # Contact shadow: flat dark band; its local standard deviation is tiny compared with tread, rims and outlines.
    mean = cv2.blur(g, (9, 9))
    var = cv2.blur(g * g, (9, 9)) - mean * mean
    smooth = ((np.sqrt(np.maximum(var, 0)) < 2.5) & (chroma < 8) & (d < -8)).astype(np.uint8)
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    ys = np.nonzero(largest(fg))[0]
    top0, bottom = (ys.min(), ys.max()) if len(ys) else (0, fg.shape[0])
    n, lab, st, _ = cv2.connectedComponentsWithStats(smooth, connectivity=4)
    shadow = np.zeros_like(fg)
    for i in range(1, n):
        top, hgt = st[i, cv2.CC_STAT_TOP], st[i, cv2.CC_STAT_HEIGHT]
        if top + hgt >= bottom - 0.10 * (bottom - top0) and st[i, cv2.CC_STAT_AREA] > 150 and st[i, cv2.CC_STAT_WIDTH] > 3 * hgt:
            shadow[lab == i] = 1
    # Grow the shadow over its soft rim (the 9 px window makes the penumbra look textured): neutral, darker than
    # the floor, and directly above or below the smooth core.
    rim = cv2.dilate(shadow, np.ones((19, 3), np.uint8)) & (chroma < 10).astype(np.uint8) & (d < -4).astype(np.uint8)
    shadow = shadow | rim
    fg = fg & (1 - shadow)
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    if ignore_mask is not None:
        fg = fg & (1 - ignore_mask)
    return fg, shadow, d


def largest(m):
    n, lab, st, _ = cv2.connectedComponentsWithStats(m.astype(np.uint8), connectivity=8)
    if n <= 1:
        return m * 0
    big = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    return (lab == big).astype(np.uint8)


def classify(cell, palette):
    """Nearest palette colour in Lab per pixel. Returns label indices into `palette` names and the distance."""
    lab = cv2.cvtColor(cell.astype(np.uint8), cv2.COLOR_RGB2LAB).astype(np.float32)
    names = list(palette.keys())
    best = np.full(lab.shape[:2], 1e9, np.float32)
    idx = np.zeros(lab.shape[:2], np.int32)
    for i, n in enumerate(names):
        for rgb in palette[n]:
            ref = cv2.cvtColor(np.array([[rgb]], np.uint8), cv2.COLOR_RGB2LAB).astype(np.float32)[0, 0]
            dist = np.sqrt(((lab - ref) ** 2).sum(2))
            better = dist < best
            best[better] = dist[better]
            idx[better] = i
    return names, idx, best


# ----------------------------------------------------------------------------------------------- wheels


def fit_circle(pts):
    pts = np.asarray(pts, np.float64)
    A = np.stack([2 * pts[:, 0], 2 * pts[:, 1], np.ones(len(pts))], 1)
    b = (pts ** 2).sum(1)
    (cx, cy, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    return cx, cy, math.sqrt(c + cx * cx + cy * cy)


def side_wheels(cell, mask, hint):
    """Wheel circles of the side view: Hough for the approximate centres, then a least-squares circle through the
    outer tire edge found along rays in the lower front and rear quadrants (the top is inside the arch, the bottom
    in the contact shadow)."""
    """Hub centre: Hough circle of the rim (strong concentric edges). Tire radius: along rays over the lower half,
    the outermost pixel that is dark and textured (tread; the contact shadow is smooth, the floor is lighter);
    median over the rays."""
    g = cv2.GaussianBlur(cv2.cvtColor(cell.astype(np.uint8), cv2.COLOR_RGB2GRAY), (5, 5), 1.5)
    rmin, rmax = hint["rimRadiusPx"]
    cs = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, dp=1, minDist=4 * rmax, param1=60, param2=20, minRadius=rmin,
                          maxRadius=rmax)
    if cs is None:
        raise SystemExit("side view: no rim circles found; give side.wheels in the keypoints")
    # Strongest candidate in the front and the rear half of the car (rims sit in the lower part of the mask).
    ys = np.nonzero(mask)[0]
    low = ys.min() + 0.5 * (ys.max() - ys.min())
    xs = np.nonzero(mask)[1]
    mid = 0.5 * (xs.min() + xs.max())
    cands = []
    for half in (lambda x: x < mid, lambda x: x >= mid):
        c = [c for c in cs[0].tolist() if half(c[0]) and c[1] > low]
        if not c:
            raise SystemExit("side view: rim missing in one half; give side.wheels in the keypoints")
        cands.append(c[0])
    gray = cell.mean(2)
    mean = cv2.blur(gray, (5, 5))
    sd = np.sqrt(np.maximum(cv2.blur(gray * gray, (5, 5)) - mean * mean, 0))
    tire = (gray < 62) & (sd > 3.0)
    out = []
    for cx, cy, rr in cands:
        radii = []
        for deg in np.arange(15, 166, 2.0):
            a = math.radians(deg)
            last = None
            for r in np.arange(1.2 * rr, 2.6 * rr, 0.5):
                x, y = int(round(cx + r * math.cos(a))), int(round(cy + r * math.sin(a)))
                if not (0 <= x < gray.shape[1] and 0 <= y < gray.shape[0]):
                    break
                if tire[y, x]:
                    last = r
                elif last is not None and r - last > 4:
                    break
            if last is not None:
                radii.append(last)
        radii = np.array(radii)
        out.append({"cx": cx, "cy": cy, "r": float(np.median(radii)), "rimR": rr,
                    "rSpread": float(np.percentile(radii, 75) - np.percentile(radii, 25))})
    return out


# ----------------------------------------------------------------------------------------------- profiles


def column_runs(col):
    """(start, end) runs of True in a 1-D boolean array."""
    runs = []
    inr = False
    for i, v in enumerate(col):
        if v and not inr:
            s, inr = i, True
        elif not v and inr:
            runs.append((s, i))
            inr = False
    if inr:
        runs.append((s, len(col)))
    return runs


def smooth_profile(v, k=5):
    v = np.asarray(v, np.float64)
    out = v.copy()
    n = len(v)
    for i in range(n):
        w = v[max(0, i - k // 2): i + k // 2 + 1]
        w = w[np.isfinite(w)]
        out[i] = np.median(w) if len(w) else np.nan
    return out


def simplify(points, eps):
    """Douglas-Peucker on an open polyline (list of (a, b))."""
    if len(points) < 3:
        return list(points)
    pts = np.array(points, np.float32).reshape(-1, 1, 2)
    return [tuple(map(float, p)) for p in cv2.approxPolyDP(pts, eps, False).reshape(-1, 2)]


def contours(mask, eps_px, min_area_px):
    cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out = []
    for c in cs:
        if cv2.contourArea(c) < min_area_px:
            continue
        a = cv2.approxPolyDP(c, eps_px, True).reshape(-1, 2)
        if len(a) >= 3:
            out.append(a.astype(np.float64))
    return out


# ----------------------------------------------------------------------------------------------- per-view tracing


class View:
    """One cell of the sheet with its masks and class map (pixel coordinates of the cell)."""

    def __init__(self, name, cell, cfg, kp):
        self.name = name
        self.cell = cell
        vk = kp.get(name, {})
        self.ignore = poly_mask(cell.shape, vk.get("ignore", []))
        self.accessory = poly_mask(cell.shape, vk.get("accessories", []))
        fg, self.shadow, _ = segment(cell, cfg, None)
        fg = fg & (1 - poly_mask(cell.shape, vk.get("shadow", [])))  # hand-marked ground shadow
        full = fill_holes(largest(fg))
        self.ground = None
        self.full = full  # everything drawn (for display)
        self.silhouette = full & (1 - self.ignore)  # compared with the render (antennas, roof spare ignored)
        self.body = largest(self.silhouette & (1 - self.accessory))  # what the body loft is traced from
        self.class_names, idx, dist = classify(cell, kp["palette"])
        self.overrides = [(poly_mask(cell.shape, [o["poly"]]), self.class_names.index(o["class"]))
                          for o in vk.get("classOverride", [])]  # hand-marked colour regions (roof under the rack)
        self.regions = [(poly_mask(cell.shape, [o["poly"]]), self.class_names.index(o["class"]))
                        for o in vk.get("paint", [])]  # hand-drawn paint regions with straight edges, drawn last
        for m, c in self.overrides:
            idx = np.where(m > 0, c, idx)
        self.cls = np.where(self.silhouette > 0, idx, -1)
        self.cls_dist = dist
        self.paint = paint_classes(self)

    def cmask(self, name, within=None):
        if name not in self.class_names:
            return np.zeros(self.body.shape, np.uint8)
        m = (self.cls == self.class_names.index(name)).astype(np.uint8)
        return m & (within if within is not None else self.silhouette)


def nearest_fill(cls, holes):
    """Classes of the `holes` pixels (and of every pixel without a class) taken from the nearest pixel that has one."""
    known = (cls >= 0) & (holes == 0)
    if not known.any():
        return cls
    _, lab = cv2.distanceTransformWithLabels((1 - known).astype(np.uint8), cv2.DIST_L2, 5,
                                             labelType=cv2.DIST_LABEL_PIXEL)
    ys, xs = np.nonzero(known)
    lut = np.zeros(lab.max() + 1, cls.dtype)
    lut[lab[ys, xs]] = cls[ys, xs]
    return np.where(known, cls, lut[lab])


def paint_classes(v, k=7, min_area=90):
    """Paint regions of one view for the body colours (cell pixels, a class everywhere): the colour classes with the
    accessories and ignored parts (rack, cargo, mirrors, bull bar, antennas) replaced by the nearest body colour,
    the hand-marked overrides put back, then cleaned: two k x k majority passes and no region smaller than
    `min_area` pixels (handles, seams, badges, line work), which are filled from around them. The hand-drawn
    "paint" polygons of the view (main panels, cladding lines) are drawn over the result in order."""
    cls = nearest_fill(v.cls, (v.accessory | v.ignore) & v.silhouette)
    for m, c in v.overrides:
        cls = np.where(m > 0, c, cls)
    every = np.ones(cls.shape, np.uint8)
    for _ in range(2):
        cls = mode_filter(cls, every, k)
    for _ in range(2):
        small = np.zeros(cls.shape, np.uint8)
        for c in range(int(cls.max()) + 1):
            n, lab, st, _ = cv2.connectedComponentsWithStats((cls == c).astype(np.uint8), connectivity=4)
            tiny = np.nonzero(st[:, cv2.CC_STAT_AREA] < min_area)[0]
            tiny = tiny[tiny > 0]
            if len(tiny):
                small |= np.isin(lab, tiny).astype(np.uint8)
        if not small.any():
            break
        cls = nearest_fill(cls, small)
    if "accent" in v.class_names:  # accent stripes are drawn thinner than the majority window: kept from the sheet
        a = v.class_names.index("accent")
        raw = ((v.cls == a) & (v.accessory == 0) & (v.ignore == 0)).astype(np.uint8)
        raw = cv2.morphologyEx(raw, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, st, _ = cv2.connectedComponentsWithStats(raw, connectivity=4)
        keep = np.nonzero(st[:, cv2.CC_STAT_AREA] >= min_area // 3)[0]
        cls = np.where(np.isin(lab, keep[keep > 0]), a, cls)
    for m, c in v.regions:
        cls = np.where(m > 0, c, cls)
    return cls


def col_extent(mask, x):
    ys = np.nonzero(mask[:, x])[0]
    return (ys.min(), ys.max()) if len(ys) else (None, None)


def row_extent(mask, y):
    xs = np.nonzero(mask[y, :])[0]
    return (xs.min(), xs.max()) if len(xs) else (None, None)


def trace_side(v, kp, wheels):
    """Per-column profiles of the side view (pixel coordinates): body top, body bottom outside the arches, the
    arch opening and flare outer edge over each wheel, greenhouse (glass top / belt), cladding top."""
    h, w = v.body.shape
    gray = v.cell.mean(2)
    dark = v.cmask("dark", v.body)
    paint = v.cmask("paint", v.body) | v.cmask("accent", v.body) | v.cmask("white", v.body)
    xs = [x for x in range(w) if v.body[:, x].any()]
    prof = {k: np.full(w, np.nan) for k in ("top", "bottom", "glassTop", "belt", "cladTop", "archOpen", "flareTop")}
    for x in xs:
        top, bot = col_extent(v.body, x)
        prof["top"][x] = top
        # Over a wheel: walk up from the tire (circle) through the dark gap to the flare, then to the paint.
        over = None
        for wh in wheels:
            if abs(x - wh["cx"]) < wh["r"] * 1.35:
                over = wh
        if over is not None:
            dx = x - over["cx"]
            r = over["r"]
            tire_top = over["cy"] - math.sqrt(max(r * r - dx * dx, 0.0)) if abs(dx) < r else over["cy"]
            y = int(tire_top) - 1
            while y > top and gray[y, x] < kp["side"].get("archVoidGray", 24):
                y -= 1
            prof["archOpen"][x] = y
            yy = y
            while yy > top and not paint[yy, x]:
                yy -= 1
            prof["flareTop"][x] = yy
        else:
            prof["bottom"][x] = bot
        # Greenhouse: from the top, the first dark run of >= 4 px followed by paint is the glass; its lower end is
        # the belt.
        col_d = dark[:, x].astype(bool)
        col_p = paint[:, x].astype(bool)
        y = top
        glass_top = None
        while y < bot:
            if col_d[y] and col_d[y: y + 4].all() and glass_top is None:
                glass_top = y
            if glass_top is not None and col_p[y] and col_p[y: y + 4].all():
                if y - glass_top >= 6:
                    prof["glassTop"][x] = glass_top
                    prof["belt"][x] = y
                break
            if glass_top is None and y - top > 0.45 * (bot - top):
                break
            y += 1
        # Cladding: walking up from the bottom (outside the arches), the first paint pixel.
        if over is None:
            y = bot
            while y > top and not col_p[y]:
                y -= 1
            prof["cladTop"][x] = y
    # Door cut lines: thin, near-vertical dark lines through the painted side between the arches.
    x_lo = wheels[0]["cx"] + 0.9 * wheels[0]["r"]
    x_hi = wheels[1]["cx"] - 0.9 * wheels[1]["r"]
    belt = np.nanmedian(prof["belt"][int(x_lo):int(x_hi)])
    clad = np.nanmedian(prof["cladTop"][int(x_lo):int(x_hi)])
    y0, y1 = int(belt + 0.15 * (clad - belt)), int(clad - 0.1 * (clad - belt))
    lum = gray[y0:y1, :]
    score = np.full(w, 0.0)
    for x in range(int(x_lo), int(x_hi)):
        # Darker than both neighbours two pixels away over most of the band.
        d = np.minimum(lum[:, x - 2] if x >= 2 else lum[:, x], lum[:, x + 2] if x + 2 < w else lum[:, x]) - lum[:, x]
        score[x] = float(np.mean(d > 6))
    cuts = []
    for x in range(int(x_lo) + 1, int(x_hi) - 1):
        if score[x] > 0.55 and score[x] >= score[x - 1] and score[x] >= score[x + 1]:
            if not cuts or x - cuts[-1] > 6:
                cuts.append(x)
    prof["doorCuts"] = cuts
    prof["doorBand"] = (y0, y1)
    return prof


def trace_plan(v):
    """Top view (car facing left): per column the extents of the whole body (flares and tires included; mirrors
    are accessories) and of the painted body (no dark cladding, flares, bumpers or tires), in pixels. The drop
    shadow falls on the image-bottom side, so the full extent is measured on the top edge only."""
    h, w = v.body.shape
    painted = (v.cmask("paint", v.body) | v.cmask("accent", v.body) | v.cmask("white", v.body))
    painted = cv2.morphologyEx(painted, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    out = {k: np.full(w, np.nan) for k in ("x0", "x1", "p0", "p1")}
    for x in range(w):
        a, b = col_extent(v.body, x)
        if a is not None:
            out["x0"][x], out["x1"][x] = a, b
        a, b = col_extent(painted, x)
        if a is not None:
            out["p0"][x], out["p1"][x] = a, b
    pc = 0.5 * (out["p0"] + out["p1"])
    ok = np.isfinite(pc)
    xs = np.nonzero(ok)[0]
    mid = (xs > xs.min() + 0.3 * (xs.max() - xs.min())) & (xs < xs.min() + 0.7 * (xs.max() - xs.min()))
    out["centre"] = float(np.median(pc[xs[mid]]))
    return out


def trace_section(v):
    """Front / rear view: per row the body extents (pixels)."""
    h, w = v.body.shape
    out = {"l": np.full(h, np.nan), "r": np.full(h, np.nan)}
    for y in range(h):
        a, b = row_extent(v.body, y)
        if a is not None:
            out["l"][y], out["r"][y] = a, b
    return out


# ----------------------------------------------------------------------------------------------- overlays

CLASS_COLORS = {"paint": (60, 200, 60), "accent": (255, 140, 0), "white": (240, 240, 240), "red": (230, 30, 30),
                "dark": (40, 40, 160), "blue": (40, 140, 255), "lamp": (255, 255, 120)}


def overlay_base(v, alpha=0.55):
    img = v.cell.copy()
    cl = np.zeros_like(img)
    for i, n in enumerate(v.class_names):
        cl[v.cls == i] = CLASS_COLORS.get(n, (255, 0, 255))
    m = v.silhouette > 0
    img[m] = img[m] * (1 - alpha) + cl[m] * alpha
    img[(v.ignore > 0) & (v.full > 0)] *= 0.35
    edge = v.body - cv2.erode(v.body, np.ones((3, 3), np.uint8))
    img[edge > 0] = (255, 0, 255)
    return img


def save_overlay(img, path, scale=2):
    big = cv2.resize(np.clip(img, 0, 255).astype(np.uint8), None, fx=scale, fy=scale,
                     interpolation=cv2.INTER_NEAREST)
    Image.fromarray(big).save(path)
    return path


def draw_profile(img, xs, ys, color):
    pts = [(int(x), int(round(y))) for x, y in zip(xs, ys) if np.isfinite(y)]
    for p in pts:
        cv2.circle(img, p, 0, color, -1)


# ----------------------------------------------------------------------------------------------- blueprint


def clean(v, k=7):
    """Median-smoothed copy of a per-column profile (NaN where unknown)."""
    return smooth_profile(v, k)


def polyline(us, vs, eps):
    pts = [(float(u), float(v)) for u, v in zip(us, vs) if np.isfinite(u) and np.isfinite(v)]
    # Split at gaps (consecutive samples further apart than 3 columns).
    out, run = [], []
    for p in pts:
        if run and abs(p[0] - run[-1][0]) > 0.03:
            out.append(run)
            run = []
        run.append(p)
    if run:
        out.append(run)
    return [simplify(r, eps) for r in out if len(r) >= 2]


def rnd(x, n=4):
    return round(float(x), n)


def rpts(pts):
    return [[rnd(a), rnd(b)] for a, b in pts]


def build(cfg, kp, root, debug_dir):
    im = np.asarray(Image.open(os.path.join(root, cfg["image"])).convert("RGB")).astype(np.float32)
    veh = load_json(os.path.join(root, kp["vehicle"]))
    x0, x1 = cfg["columns"][kp["column"]]
    V = {}
    for n in VIEWS:
        y0, y1 = cfg["rows"][n]
        V[n] = View(n, im[y0:y1, x0:x1], cfg, kp)
    report = {}

    # ---- side: wheels, scale, frame
    s = V["side"]
    wh = kp["side"].get("wheels") or side_wheels(s.cell, s.body, kp["side"])
    wh = sorted(wh, key=lambda e: e["cx"])  # front wheel first (the car faces left)
    radius = float(veh["axles"]["front"]["wheel"]["radius"])
    D = 2.0 * radius
    ppm = np.mean([2.0 * e["r"] for e in wh]) / D
    x_mid = 0.5 * (wh[0]["cx"] + wh[1]["cx"])
    y_axle = 0.5 * (wh[0]["cy"] + wh[1]["cy"])
    ground_px = y_axle + D / 2 * ppm
    for vv in (s,):  # clip the contact-shadow residue under the tires
        for m in (vv.silhouette, vv.body, vv.full):
            m[int(math.ceil(ground_px)) + 1:, :] = 0
            yy, xx = np.mgrid[0:m.shape[0], 0:m.shape[1]]
            for e in wh:  # below the hub the tire is the circle; the corners next to the contact patch are shadow
                box = (np.abs(xx - e["cx"]) <= e["r"] + 2) & (yy > e["cy"])
                m[box & (np.hypot(xx - e["cx"], yy - e["cy"]) > e["r"] + 1.5)] = 0
    zf = lambda xp: (x_mid - xp) / ppm  # noqa: E731
    yf = lambda yp: (y_axle - yp) / ppm  # noqa: E731
    wb_concept = (wh[1]["cx"] - wh[0]["cx"]) / ppm
    report["side"] = {
        "pxPerM": rnd(ppm, 2), "tireRadiusPx": [rnd(e["r"], 1) for e in wh],
        "tireRadiusSpreadPx": [rnd(e.get("rSpread", 0), 1) for e in wh],
        "axleHeightDiffM": rnd((wh[0]["cy"] - wh[1]["cy"]) / ppm, 3),
        "frontRearTireDiameterRatio": rnd(wh[0]["r"] / wh[1]["r"], 3),
        "conceptWheelBase": rnd(wb_concept, 3), "physicsWheelBase": veh["geometry"]["wheelBase"],
    }
    p = trace_side(s, kp, wh)
    cols = np.arange(s.body.shape[1])
    Z = zf(cols)
    side = {"frame": "z from the wheelbase midpoint, y from the axle line (concept scale; the model maps the axles onto the physics wheels)"}
    eps = 0.004
    top = clean(p["top"], 5)
    side["top"] = rpts(polyline(Z[::-1], yf(top)[::-1], eps)[0]) if np.isfinite(top).any() else []
    # Bottom outside the arches, glass, belt, cladding: segments ordered rear -> front (z increasing).
    for key, k in (("bottom", 5), ("cladTop", 9)):
        prof = clean(p[key], k)
        side[key] = [rpts(seg) for seg in polyline(Z[::-1], yf(prof)[::-1], eps)]
    # Belt and glass top of the side windows: straight lines fitted (with outlier rejection) over the columns
    # between the arches where a glass run was found; the windshield and backlight follow from the top profile.
    gh = (cols > wh[0]["cx"]) & (cols < wh[1]["cx"])
    for key in ("belt", "glassTop"):
        ok = gh & np.isfinite(p[key])
        if key == "glassTop":  # side windows only: the belt fit's range (the windshield top slopes down)
            ok &= (Z >= side["belt"][0][0]) & (Z <= side["belt"][1][0])
            ok &= yf(top) > np.nanmax(yf(top)) - 0.10  # under the roof, not the windshield
        zz, yy = Z[ok], yf(p[key][ok])
        keep = np.ones(len(zz), bool)
        for _ in range(5):
            a, b = np.polyfit(zz[keep], yy[keep], 1)
            res = np.abs(yy - (a * zz + b))
            keep = res < max(2.5 * np.median(res[keep]), 0.01)
        z0, z1 = float(zz[keep].min()), float(zz[keep].max())
        side[key] = [[rnd(z0), rnd(a * z0 + b)], [rnd(z1), rnd(a * z1 + b)]]
        report["side"][key + "FitInliers"] = f"{int(keep.sum())}/{len(keep)}"
    arches = []
    for i, e in enumerate(wh):
        sel = np.abs(cols - e["cx"]) < e["r"] * 1.35
        a = clean(np.where(sel, p["archOpen"], np.nan), 5)
        f = clean(np.where(sel, p["flareTop"], np.nan), 5)
        arches.append({
            "axle": "front" if i == 0 else "rear",
            "center": [rnd(zf(e["cx"])), rnd(yf(e["cy"]))],
            "radius": rnd(e["r"] / ppm),
            "opening": rpts(polyline(Z[::-1], yf(a)[::-1], eps)[0]),
            "flareOuter": rpts(polyline(Z[::-1], yf(f)[::-1], eps)[0]),
        })
    side["arches"] = arches
    side["doorCuts"] = sorted({rnd(zf(x)) for x in p["doorCuts"] + kp["side"].get("extraDoorCutsPx", [])},
                              reverse=True)
    ys, xs = np.nonzero(s.body)
    side["extent"] = {"zFront": rnd(zf(xs.min())), "zRear": rnd(zf(xs.max())), "yTop": rnd(yf(ys.min())),
                      "yBottom": rnd(yf(ys.max()))}
    ys, xs = np.nonzero(s.silhouette)
    side["silhouette"] = sil_polys(s.silhouette, lambda P: [(zf(a), yf(b)) for a, b in P])
    side["ignore"] = ignore_polys(kp, "side", lambda P: [(zf(a), yf(b)) for a, b in P])
    wheel_mask = np.zeros_like(s.body)
    for e in wh:
        cv2.circle(wheel_mask, (int(round(e["cx"])), int(round(e["cy"]))), int(round(e["r"] + 2)), 1, -1)
    side["regions"] = regions(s, lambda P: [(zf(a), yf(b)) for a, b in P], s.body & (1 - wheel_mask))
    side["wheels"] = [{"center": [rnd(zf(e["cx"])), rnd(yf(e["cy"]))], "radius": rnd(e["r"] / ppm)} for e in wh]
    body_len_px = (lambda xs_: xs_.max() - xs_.min())(np.nonzero(s.body)[1])
    roof_px = np.nanmin(top)

    # ---- front / rear: ground row, centre, scale from the body height (side view governs heights)
    sections = {}
    for n in ("front", "rear"):
        v = V[n]
        ys, xs = np.nonzero(v.body)
        g_px = ys.max()
        low = v.body[int(g_px - 0.12 * (g_px - ys.min())): int(g_px) + 1]
        lx = np.nonzero(low.any(0))[0]
        cx = 0.5 * (lx.min() + lx.max())
        ppm_v = (g_px - ys.min()) / ((y_axle - roof_px) / ppm + D / 2)
        tire_w = (lx.max() - lx.min()) / ppm_v
        xf_ = (lambda c, q: (lambda xp: (xp - c) / q))(cx, ppm_v)
        if n == "rear":  # looking forward from behind: image right is the car's right (-X)
            xf_ = (lambda c, q: (lambda xp: -(xp - c) / q))(cx, ppm_v)
        yv = (lambda g, q: (lambda yp: (g - yp) / q - D / 2))(g_px, ppm_v)
        sec = trace_section(v)
        rows = np.arange(v.body.shape[0])
        half = 0.5 * (sec["r"] - sec["l"]) / ppm_v
        offc = (0.5 * (sec["r"] + sec["l"]) - cx) / ppm_v
        sections[n] = {
            "pxPerM": rnd(ppm_v, 2),
            "halfWidth": rpts(simplify([(float(yv(r)), float(hw)) for r, hw in zip(rows, half) if np.isfinite(hw)],
                                       eps)),
            "silhouette": sil_polys(v.silhouette, lambda P, f=xf_, g=yv: [(f(a), g(b)) for a, b in P]),
            "ignore": ignore_polys(kp, n, lambda P, f=xf_, g=yv: [(f(a), g(b)) for a, b in P]),
            "regions": regions(v, lambda P, f=xf_, g=yv: [(f(a), g(b)) for a, b in P], v.body),
            "_tireHalf": 0.5 * tire_w,
            "_grid": (v, symmetric_rows(v.body, cx), cx, ppm_v, g_px),
        }
        report[n] = {"pxPerM": rnd(ppm_v, 2), "scaleVsSide": rnd(ppm_v / ppm, 3),
                     "outerTireWidth": rnd(tire_w, 3),
                     "physicsOuterTireWidth": rnd(veh["axles"]["front"]["track"] + veh["axles"]["front"]["wheel"]["width"], 3),
                     "maxHalfWidth": rnd(np.nanmax(half), 3),
                     "centreOffsetMaxM": rnd(np.nanmax(np.abs(offc[np.isfinite(offc)])), 3)}

    # ---- top: plan half-widths; the body length is matched to the side view (side governs length)
    v = V["top"]
    ys, xs = np.nonzero(v.body)
    side_front, side_rear = side["extent"]["zFront"], side["extent"]["zRear"]
    ppm_t = (xs.max() - xs.min()) / (side_front - side_rear)
    pl = trace_plan(v)
    cy_t = pl["centre"]
    zt = (lambda a, q: (lambda xp: side_front - (xp - a) / q))(xs.min(), ppm_t)
    xt = (lambda c, q: (lambda yp: (yp - c) / q))(cy_t, ppm_t)  # image down = car left (+X)
    colz = zt(np.arange(v.body.shape[1]))
    full = np.array([(cy_t - col_extent(v.silhouette, x)[0]) / ppm_t if v.silhouette[:, x].any() else np.nan
                     for x in range(v.body.shape[1])])
    painted = smooth_profile((cy_t - pl["x0"]) / ppm_t, 5)
    # Tire outer half-width in the plan: the widest full extent within a tire radius of each concept axle.
    axle_z = [zf(e["cx"]) for e in wh]
    near = np.zeros_like(colz, bool)
    for az in axle_z:
        near |= np.abs(colz - az) < 0.8 * D / 2
    tire_half_top = float(np.nanmax(np.where(near, full, np.nan)))
    plan = {
        "pxPerM": rnd(ppm_t, 2),
        "halfWidth": rpts(simplify([(float(z), float(hw)) for z, hw in zip(colz[::-1], painted[::-1])
                                    if np.isfinite(hw)], eps)),
        "fullHalfWidth": rpts(simplify([(float(z), float(hw)) for z, hw in zip(colz[::-1], full[::-1])
                                        if np.isfinite(hw)], eps)),
        "tireHalfWidth": rnd(tire_half_top),
        "silhouette": sil_polys(v.silhouette, lambda P: [(zt(a), xt(b)) for a, b in P]),
        "ignore": ignore_polys(kp, "top", lambda P: [(zt(a), xt(b)) for a, b in P]),
        "regions": regions(v, lambda P: [(zt(a), xt(b)) for a, b in P], v.body),
    }
    report["top"] = {"pxPerM": rnd(ppm_t, 2), "scaleVsSide": rnd(ppm_t / ppm, 3),
                     "maxPaintedHalfWidth": rnd(np.nanmax(painted), 3),
                     "tireOuterHalfWidth": rnd(tire_half_top, 3),
                     "physicsTireOuterHalfWidth": rnd(0.5 * (veh["axles"]["front"]["track"] +
                                                             veh["axles"]["front"]["wheel"]["width"]), 4),
                     "conceptAxleZ": [rnd(a, 3) for a in axle_z]}
    # Front / rear x: the views are drawn at their own scale; their tire outer half-width is matched to the plan
    # (top view governs widths), heights keep the height-matched scale.
    for n in ("front", "rear"):
        kx = tire_half_top / sections[n]["_tireHalf"]
        report[n]["xScaleToPlan"] = rnd(kx, 3)
        for key in ("halfWidth",):
            sections[n][key] = [[a, rnd(b * kx)] for a, b in sections[n][key]]
        for key in ("silhouette", "ignore"):
            sections[n][key] = [[[rnd(a * kx), b] for a, b in poly] for poly in sections[n][key]]
        sections[n]["regions"] = {c: [[[rnd(a * kx), b] for a, b in poly] for poly in polys]
                                  for c, polys in sections[n]["regions"].items()}
        del sections[n]["_tireHalf"]
        gv, ghull, gcx, gq, gg = sections[n].pop("_grid")
        sgn = 1.0 if n == "front" else -1.0
        sections[n]["grid"] = grid(gv, ghull, gv.paint, -sgn * gcx * kx / gq, sgn * kx / gq, gg / gq - D / 2, -1.0 / gq)
    # Top: symmetric about the centre line, from the image-top half (the drop shadow falls on the other side).
    h_t = v.body.shape[0]
    top_i = np.array([col_extent(v.body, x)[0] if v.body[:, x].any() else np.nan for x in range(v.body.shape[1])])
    ii = np.arange(h_t)[:, None]
    hw_rows = (cy_t - top_i)[None, :]
    hull_t = (np.abs(ii - cy_t) <= hw_rows).astype(np.uint8)
    mirror_i = np.clip(np.round(np.where(ii > cy_t, 2 * cy_t - ii, ii)).astype(int), 0, h_t - 1)
    cls_t = v.paint[mirror_i, np.arange(v.body.shape[1])[None, :]]
    plan["grid"] = grid(v, hull_t, cls_t, side_front + xs.min() / ppm_t, -1.0 / ppm_t, -cy_t / ppm_t, 1.0 / ppm_t)
    side["grid"] = grid(s, side_hull(s, wh, p, kp), s.paint, x_mid / ppm, -1.0 / ppm, y_axle / ppm, -1.0 / ppm)
    def box_m(view, box):
        g = (sections[view] if view in sections else side if view == "side" else plan)["grid"]
        (x0, y0), (x1, y1) = box
        us = sorted([g["u0"] + g["du"] * x0, g["u0"] + g["du"] * x1])
        vs = sorted([g["v0"] + g["dv"] * y0, g["v0"] + g["dv"] * y1])
        return [rnd(us[0]), rnd(vs[0]), rnd(us[1]), rnd(vs[1])]

    lights = [{"view": L["view"], "kind": L["kind"], "box": box_m(L["view"], L["box"])} for L in kp.get("lights", [])]
    # Where the accessories sit (hand-placed boxes on the sheet, converted like the lights).
    anchors = [{"name": A["name"], "view": A["view"], "box": box_m(A["view"], A["box"])} for A in kp.get("anchors", [])]

    bp = {
        "schema": 1,
        "id": kp["id"],
        "source": {"image": cfg["image"], "column": kp["column"], "keypoints": f"tools/cars/{kp['id']}.keypoints.json",
                   "tool": "tools/cars/trace_blueprint.py"},
        "scale": {"tireDiameter": rnd(D), "conceptWheelBase": rnd(wb_concept, 3),
                  "restHubY": kp["restHubY"]},
        "side": side, "top": plan, "front": sections["front"], "rear": sections["rear"], "lights": lights,
        "anchors": anchors,
        "report": report,
    }
    if debug_dir:
        debug_overlays(V, wh, p, debug_dir, kp["id"])
    return bp


# ----------------------------------------------------------------------------------------------- hull grids

CLASS_LETTERS = {"paint": "p", "accent": "a", "white": "w", "red": "r", "dark": "d"}


def mode_filter(cls, hull, k=5):
    """Most frequent class in a k x k window (inside the hull): removes seams, handles and speckles thinner than
    about k / 2 pixels, which the body mesh cannot resolve anyway."""
    n = int(cls.max()) + 1
    best, score = np.zeros_like(cls), np.full(cls.shape, -1.0, np.float32)
    for c in range(n):
        s = cv2.boxFilter(((cls == c) & (hull > 0)).astype(np.float32), -1, (k, k), normalize=False)
        upd = s > score
        best[upd], score[upd] = c, s[upd]
    return np.where(hull > 0, best, -1)


def rle_rows(letters):
    rows = []
    for r in letters:
        out, run, prev = [], 0, None
        for ch in r:
            if ch == prev:
                run += 1
            else:
                if prev is not None:
                    out.append(f"{run}{prev}")
                prev, run = ch, 1
        out.append(f"{run}{prev}")
        rows.append("".join(out))
    return rows


def grid(v, hull, cls, u0, du, v0, dv):
    """One view as a class grid on the cell's pixels: cell (row i, column j) is at u = u0 + du * j,
    v = v0 + dv * i (meters, concept frame); '.' outside the body hull."""
    names = v.class_names
    cl = np.where(cls < 0, names.index("paint"), cls)
    lut = np.array([CLASS_LETTERS.get(n, n[0]) for n in names] + ["."])
    letters = lut[np.where(hull > 0, cl, len(names))]
    return {"u0": rnd(u0, 5), "du": rnd(du, 6), "v0": rnd(v0, 5), "dv": rnd(dv, 6), "width": int(hull.shape[1]),
            "height": int(hull.shape[0]), "rows": rle_rows(letters)}


def side_hull(s, wh, p, kp):
    """Side body without the tires and the dark void between tire and arch: the arch openings are cut where the
    concept shows them."""
    h, w = s.body.shape
    hull = s.body.copy()
    yy, xx = np.mgrid[0:h, 0:w]
    gray = cv2.GaussianBlur(s.cell.mean(2), (3, 3), 0)
    void = gray < kp["side"].get("archVoidGray", 24) + 6
    for e in wh:
        d = np.hypot(xx - e["cx"], yy - e["cy"])
        disc = d <= e["r"] + 1.5
        cand = (disc | (void & (d < 1.45 * e["r"]))).astype(np.uint8)
        n, lab = cv2.connectedComponents(cand, connectivity=8)
        ids = np.unique(lab[disc & (cand > 0)])
        hull[np.isin(lab, ids[ids > 0])] = 0
    hull = cv2.morphologyEx(hull, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return largest(hull)


def symmetric_rows(mask, cx):
    """Front / rear: keep a pixel only if its mirror about the centre column is body too (one-sided parts such as
    a snorkel or a ladder do not widen the hull)."""
    h, w = mask.shape
    j = np.arange(w)
    jm = np.clip(np.round(2 * cx - j).astype(int), 0, w - 1)
    valid = (2 * cx - j >= 0) & (2 * cx - j <= w - 1)
    return (mask & mask[:, jm] & valid[None, :]).astype(np.uint8)


def ignore_polys(kp, view, conv):
    """Regions the silhouette comparison skips: the hand-marked ignores (antennas, roof spare) and the ground shadow
    between the tires (it may hide underbody parts, so neither presence nor absence can be checked there)."""
    out = []
    for p in kp.get(view, {}).get("ignore", []) + kp.get(view, {}).get("shadow", []):
        if len(p) == 2:
            (x0, y0), (x1, y1) = p
            p = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
        out.append(rpts(conv(p)))
    return out


def sil_polys(mask, conv):
    return [rpts(conv(c)) for c in contours(mask, 1.0, 200)]


def regions(v, conv, within):
    out = {}
    for n in v.class_names:
        if n == "paint":
            continue
        m = v.cmask(n, within)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        polys = [rpts(conv(c)) for c in contours(m, 0.8, 10)]
        if polys:
            out[n] = polys
    return out


def debug_overlays(V, wh, p, out_dir, vid):
    S = 3
    for n, v in V.items():
        img = cv2.resize(overlay_base(v, 0.35).astype(np.uint8), None, fx=S, fy=S, interpolation=cv2.INTER_NEAREST)
        if n == "side":
            cols = np.arange(len(p["top"]))
            for key, col in (("top", (255, 0, 0)), ("bottom", (255, 0, 0)), ("glassTop", (0, 255, 255)),
                             ("belt", (255, 255, 0)), ("cladTop", (255, 0, 255)), ("archOpen", (0, 255, 0)),
                             ("flareTop", (255, 128, 0))):
                prof = clean(p[key], 5)
                for x, y in zip(cols, prof):
                    if np.isfinite(y):
                        cv2.circle(img, (int(x * S + 1), int(y * S + 1)), 2, col, -1)
            for e in wh:
                cv2.circle(img, (int(e["cx"] * S), int(e["cy"] * S)), int(e["r"] * S), (255, 255, 255), 2)
        path = os.path.join(out_dir, f"trailforge_blueprint_{vid}_{n}.png")
        Image.fromarray(img).save(path)
        print("overlay", path)
        pc = np.zeros_like(v.cell)
        for i, cn in enumerate(v.class_names):
            pc[v.paint == i] = CLASS_COLORS.get(cn, (255, 0, 255))
        path = os.path.join(out_dir, f"trailforge_blueprint_{vid}_{n}_paint.png")
        Image.fromarray(np.concatenate([v.cell, pc], 1).astype(np.uint8)).resize(
            (v.cell.shape[1] * 4, v.cell.shape[0] * 2), Image.NEAREST).save(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("vehicle_id")
    ap.add_argument("--root", default=os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..")))
    ap.add_argument("--debug-dir", default="/tmp")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    cfg = load_json(os.path.join(a.root, "tools", "cars", "sheet.json"))
    kp = load_json(os.path.join(a.root, "tools", "cars", f"{a.vehicle_id}.keypoints.json"))
    bp = build(cfg, kp, a.root, a.debug_dir)
    out = a.out or os.path.join(a.root, "assets", "vehicles", a.vehicle_id, "blueprint.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(bp, f, separators=(",", ":"))
        f.write("\n")
    print(json.dumps(bp["report"], indent=1))
    print("wrote", out, os.path.getsize(out), "bytes")


if __name__ == "__main__":
    main()
