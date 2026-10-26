"""Generates the Suncoast Raceway level (assets/levels/suncoast_raceway) from real data: the circuit, drag strip, short
course connector and paddock access are the OpenStreetMap raceway ways of the venue (tools/levels/data/suncoast_raceway/osm.osm, ODbL), the
ground is the Copernicus GLO-30 DEM (dem.json there); both are fetched by tools/levels/fetch_osm_track.py.

Real: the plan geometry of every asphalt way (WGS84 -> local east / south metres about LAT0 / LON0), the circuit
length that follows from it, the venue fence, the farmland around it, the start line and the two sector lines
(positions read from the published track map, nearest circuit point). Chosen here (not in OSM): the track width
(12 m; published 12 to 15 m), the driving direction (the order the circuit way was drawn in), the asphalt surface
(a quadratic fitted to the smoothed DEM: the site is flat, the DSM includes the trees), run-offs, curbs, barriers,
the pit lane and all buildings (nothing of it is mapped).

Headless:
  /Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup --python-exit-code 1 \\
      --python tools/levels/blender/make_suncoast_raceway.py -- [--blend build/levels/suncoast_raceway.blend] \\
      [--out assets/levels/suncoast_raceway]
Numbers only (no Blender scene; Blender's bundled python with numpy):
  /Applications/Blender.app/Contents/Resources/5.3/python/bin/python3.13 tools/levels/blender/make_suncoast_raceway.py --plan
"""

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tf_level as tl  # noqa: E402

LEVEL_ID = "suncoast_raceway"
SEED = 3517
HALF, PLAY = 512, 480
N = 2 * HALF + 1
ORIGIN = -float(HALF)
SOURCE = os.path.join(tl.REPO, "tools", "levels", "data", LEVEL_ID)

LAT0, LON0 = 35.02546, 33.80017  # level origin (the middle of the circuit)
CIRCUIT_WAY, SHORT_WAY, ACCESS_WAY, FENCE_WAY = "734690065", "734690066", "734691195", "734690068"
WIDTH = 12.0          # circuit, drag strip and short course connector (OSM has no width tag)
ACCESS_WIDTH = 6.0    # paddock access to the car park
PIT_WIDTH = 8.0
PIT_OFFSET = 14.5     # pit lane center line from the circuit center line along the start straight
CONTROL_SPACING = 6.0
# Start / finish and the two sector lines: positions on the published map of the real circuit (full course), snapped to the
# nearest circuit point.
START_AT = (-59.0, -123.5)
SECTOR_AT = [(76.0, -21.0), (-7.0, 154.5)]


def smoothstep(a, b, x):
    t = np.clip((np.asarray(x, float) - a) / (b - a), 0.0, 1.0)
    return t * t * (3 - 2 * t)


# ------------------------------------------------------------------------------------------------------- sources
def metres_per_degree(lat):
    p = math.radians(lat)
    return (111132.954 - 559.822 * math.cos(2 * p) + 1.175 * math.cos(4 * p),
            111412.84 * math.cos(p) - 93.5 * math.cos(3 * p))


MLAT, MLON = metres_per_degree(LAT0)


def to_local(lat, lon):
    """Game x (east) and z (south) in m about LAT0 / LON0 (local tangent plane; < 1 cm error over the level)."""
    return (lon - LON0) * MLON, -(lat - LAT0) * MLAT


def load_osm():
    root = ET.parse(os.path.join(SOURCE, "osm.osm")).getroot()
    nodes = {n.get("id"): to_local(float(n.get("lat")), float(n.get("lon"))) for n in root.iter("node")}
    ways, tags = {}, {}
    for w in root.iter("way"):
        ways[w.get("id")] = [nd.get("ref") for nd in w.iter("nd")]
        tags[w.get("id")] = {t.get("k"): t.get("v") for t in w.iter("tag")}
    return nodes, ways, tags


def load_dem():
    d = json.load(open(os.path.join(SOURCE, "dem.json")))
    h = np.array(d["heights"], float)

    def at(x, z):
        lat = LAT0 - np.asarray(z, float) / MLAT
        lon = LON0 + np.asarray(x, float) / MLON
        r = np.clip((d["lat0"] - lat) / d["dlat"], 0, h.shape[0] - 1.001)
        c = np.clip((lon - d["lon0"]) / d["dlon"], 0, h.shape[1] - 1.001)
        r0, c0 = r.astype(int), c.astype(int)
        u, v = c - c0, r - r0
        return (h[r0, c0] * (1 - u) + h[r0, c0 + 1] * u) * (1 - v) + (h[r0 + 1, c0] * (1 - u) + h[r0 + 1, c0 + 1] * u) * v
    return at


# --------------------------------------------------------------------------------------------------------- curves
def centripetal(points, closed, step=0.5):
    """Centripetal Catmull-Rom through points (no overshoot or cusps at the sparse OSM nodes), every ~step m."""
    p = np.asarray(points, float)
    if not closed:
        p = np.vstack([2 * p[0] - p[1], p, 2 * p[-1] - p[-2]])
    n = len(p)
    out = []
    rng = range(n) if closed else range(1, n - 2)
    for i in rng:
        p0, p1, p2, p3 = p[i - 1], p[i], p[(i + 1) % n], p[(i + 2) % n]
        t1 = np.linalg.norm(p1 - p0) ** 0.5
        t2 = t1 + np.linalg.norm(p2 - p1) ** 0.5
        t3 = t2 + np.linalg.norm(p3 - p2) ** 0.5
        m = max(2, int(np.linalg.norm(p2 - p1) / step))
        t = np.linspace(t1, t2, m, endpoint=False)[:, None]
        a1 = (t1 - t) / t1 * p0 + t / t1 * p1
        a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
        a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
        b1 = (t2 - t) / t2 * a1 + t / t2 * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
        out.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    if not closed:
        out.append(p[-2:-1])
    return np.vstack(out)


def resample(P, closed, step=1.0):
    E = np.vstack([P, P[:1]]) if closed else P
    d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(E, axis=0).T))])
    m = max(2, int(round(d[-1] / step)))
    s = np.arange(m) * (d[-1] / m) if closed else np.linspace(0.0, d[-1], m + 1)
    return np.interp(s, d, E[:, 0]), np.interp(s, d, E[:, 1]), d[-1]


def curvature(x, z, closed, half=4):
    dx, dz = tl.tangents(x, z, closed)
    a = np.unwrap(np.arctan2(dx, dz))
    if closed:
        da = np.roll(a, -half) - np.roll(a, half)
        k = -((da + np.pi) % (2 * np.pi) - np.pi) / (2 * half)
        k = np.convolve(np.concatenate([k[-7:], k, k[:7]]), np.ones(15) / 15, "same")[7:-7]
    else:
        k = -np.gradient(a)
    return k  # + for left turns


def polyline_length(x, z, closed):
    if closed:
        x, z = np.append(x, x[0]), np.append(z, z[0])
    return float(np.sum(np.hypot(np.diff(x), np.diff(z))))


# ----------------------------------------------------------------------------------------------------------- plan
def plan():
    nodes, ways, tags = load_osm()
    dem = load_dem()
    main = ways[CIRCUIT_WAY]
    j6 = main[-1]
    k6 = main.index(j6)  # the way: drag strip J0 -> J6, then the full course from J6 back to J6
    circuit_nodes = np.array([nodes[r] for r in main[k6:-1]])
    osm_length = polyline_length(circuit_nodes[:, 0], circuit_nodes[:, 1], True)
    cx, cz, length = resample(centripetal(circuit_nodes, True), True, 1.0)
    i0 = int(np.argmin(np.hypot(cx - START_AT[0], cz - START_AT[1])))
    # The circuit starts at the start / finish line.
    cx, cz = np.roll(cx, -i0), np.roll(cz, -i0)
    cdx, cdz = tl.tangents(cx, cz, True)
    ck = curvature(cx, cz, True)
    m = len(cx)

    def open_road(refs):
        x, z, ln = resample(centripetal(np.array([nodes[r] for r in refs]), False), False, 1.0)
        return x, z, ln

    drag = open_road(main[:k6 + 1])
    short = open_road(ways[SHORT_WAY])
    access = open_road(ways[ACCESS_WAY])

    # Pit lane: outside the start straight (the side towards the car park, north = -z), offset smoothly from the
    # circuit line between s = -150 and +90 m.
    pit_side = 1.0 if cdx[0] > 0 else -1.0  # left normal (dz, -dx) dotted with north (0, -1) is dx
    sp = np.arange(-150, 91)
    ramp = smoothstep(-150, -95, sp) * smoothstep(90, 40, sp)
    ii = sp % m
    off = pit_side * PIT_OFFSET * ramp
    px, pz = cx[ii] + cdz[ii] * off, cz[ii] - cdx[ii] * off
    pit = resample(np.stack([px, pz], 1), False, 1.0)

    # Track surface: a quadratic fitted to the smoothed DEM along every asphalt way (the site is flat; the DSM has
    # the tree canopy in it). The ground around follows the DEM.
    xs = ORIGIN + np.arange(N, dtype=float)
    X, Z = np.meshgrid(xs, xs)
    D = dem(X, Z)
    for _ in range(3):  # box blur, radius 10 m (~Gaussian sigma 10 m)
        c = np.cumsum(np.pad(D, ((0, 0), (11, 10)), mode="edge"), axis=1)
        D = (c[:, 21:] - c[:, :-21]) / 21.0
        c = np.cumsum(np.pad(D, ((11, 10), (0, 0)), mode="edge"), axis=0)
        D = (c[21:, :] - c[:-21, :]) / 21.0
    D = D[:N, :N]
    fx = np.concatenate([cx, drag[0], short[0], access[0], pit[0]])
    fz = np.concatenate([cz, drag[1], short[1], access[1], pit[1]])
    fh = tl_bilinear(D, fx, fz)
    A = np.stack([np.ones_like(fx), fx, fz, fx * fx, fx * fz, fz * fz], 1) / np.array([1, 300, 300, 9e4, 9e4, 9e4])
    coef, *_ = np.linalg.lstsq(A, fh, rcond=None)
    coef = coef / np.array([1, 300, 300, 9e4, 9e4, 9e4])
    fit_rms = float(np.sqrt(np.mean((A @ (coef * np.array([1, 300, 300, 9e4, 9e4, 9e4])) - fh) ** 2)))

    def S(x, z):
        x, z = np.asarray(x, float), np.asarray(z, float)
        return coef[0] + coef[1] * x + coef[2] * z + coef[3] * x * x + coef[4] * x * z + coef[5] * z * z

    def grad(x, z):
        return coef[1] + 2 * coef[3] * x + coef[4] * z, coef[2] + coef[4] * x + 2 * coef[5] * z

    fence = np.array([nodes[r] for r in ways[FENCE_WAY]])
    inside = tl.point_in_polygon(X, Z, fence)
    # Inside the fence the ground is the fitted surface with a third of the DEM relief; outside, the DEM within
    # +-3 m of the surface (canopy and hedges clipped); blended over ~25 m across the fence; fine noise everywhere.
    w_in = inside.astype(float)
    for _ in range(2):
        c = np.cumsum(np.pad(w_in, ((0, 0), (13, 12)), mode="edge"), axis=1)
        w_in = (c[:, 25:] - c[:, :-25]) / 25.0
        c = np.cumsum(np.pad(w_in, ((13, 12), (0, 0)), mode="edge"), axis=0)
        w_in = (c[25:, :] - c[:-25, :]) / 25.0
    w_in = w_in[:N, :N]
    SG = S(X, Z)
    rel = np.clip(D - SG, -3.0, 3.0)
    base = SG + rel * (1.0 - 0.67 * w_in) + 0.12 * (tl.fbm(X / 9.0, Z / 9.0, 3, SEED) - 0.5)

    def rows_for(x, z, closed):
        ln = polyline_length(x, z, closed)
        count = max(3, int(round(ln / CONTROL_SPACING)))
        idx = np.round(np.arange(count) * (len(x) / count)).astype(int) if closed else \
            np.round(np.linspace(0, len(x) - 1, count + 1)).astype(int)
        x_, z_ = x[idx], z[idx]
        gx, gz = grad(x_, z_)
        dx, dz = tl.tangents(x_, z_, closed)
        # Bank = the cross slope of the surface (bank > 0: the left edge (dz, -dx) higher).
        bank = np.arctan(gx * dz + gz * -dx)
        rows = tl.catmull_rom_bezier(np.stack([x_, z_, S(x_, z_)], 1), closed)
        return np.hstack([rows, bank[:, None]])

    roads = {
        "circuit": (rows_for(cx, cz, True), True, "circuit", WIDTH),
        "drag_strip": (rows_for(drag[0], drag[1], False), False, "road", WIDTH),
        "short_course": (rows_for(short[0], short[1], False), False, "road", WIDTH),
        "paddock_access": (rows_for(access[0], access[1], False), False, "road", ACCESS_WIDTH),
        "pit_lane": (rows_for(pit[0], pit[1], False), False, "road", PIT_WIDTH),
    }

    # Base surfaces: grass inside the fence (dry patches of dirt), farmland (OSM landuse) ploughed dirt, grass with
    # dirt patches beyond; gravel traps outside the corners, asphalt run-off outside the faster ones.
    cxs = ORIGIN + 0.5 + np.arange(N - 1)
    CX, CZ = np.meshgrid(cxs, cxs)
    cls = np.full((N - 1, N - 1), tl.CLASSES.index("grass"), np.uint8)
    cls[tl.fbm(CX / 24.0, CZ / 24.0, 3, SEED + 3) > 0.64] = tl.CLASSES.index("dirt")
    farm = np.zeros(CX.shape, bool)
    for wid, t in tags.items():
        if t.get("landuse") == "farmland" and ways[wid][0] == ways[wid][-1]:
            poly = [nodes[r] for r in ways[wid] if r in nodes]
            if len(poly) > 3:
                farm |= tl.point_in_polygon(CX, CZ, poly)
    in_fence_c = tl.point_in_polygon(CX, CZ, fence)
    farm &= ~in_fence_c
    # Fields: ploughed soil with fallow (grass) parts.
    cls[farm & (tl.fbm(CX / 60.0, CZ / 60.0, 3, SEED + 5) > 0.47)] = tl.CLASSES.index("dirt")
    cls[farm & (tl.fbm(CX / 60.0, CZ / 60.0, 3, SEED + 5) <= 0.47)] = tl.CLASSES.index("grass")

    # Nearest circuit sample per cell (within 40 m): signed offset u (+ left) and distance d.
    best = np.full(CX.shape, np.inf)
    bk = np.full(CX.shape, -1, np.int64)
    for i in range(m):
        i0_, i1_ = np.searchsorted(cxs, cx[i] - 40), np.searchsorted(cxs, cx[i] + 40)
        j0_, j1_ = np.searchsorted(cxs, cz[i] - 40), np.searchsorted(cxs, cz[i] + 40)
        d2 = (cxs[None, i0_:i1_] - cx[i]) ** 2 + (cxs[j0_:j1_, None] - cz[i]) ** 2
        win = best[j0_:j1_, i0_:i1_]
        better = d2 < win
        win[better] = d2[better]
        bk[j0_:j1_, i0_:i1_][better] = i
    kk = np.where(bk >= 0, bk, 0)
    U = (CX - cx[kk]) * cdz[kk] - (CZ - cz[kk]) * cdx[kk]
    hw = 0.5 * WIDTH
    kabs = np.abs(ck)
    # Corner zones: curvature over 1/110 m, widened by 25 m along the track (entry braking and exit).
    zone = np.zeros(m)
    for i in np.where(kabs > 1.0 / 110.0)[0]:
        zone[np.arange(i - 25, i + 26) % m] = np.maximum(zone[np.arange(i - 25, i + 26) % m], kabs[i])
    turn_sign = np.sign(np.convolve(np.concatenate([ck[-25:], ck, ck[:25]]), np.ones(51), "same")[25:-25])
    outside = (bk >= 0) & (np.sign(U) == -turn_sign[kk]) & (zone[kk] > 0)
    trap_w = np.clip(300.0 * zone[kk], 7.0, 16.0)
    au = np.abs(U)
    gravel = outside & (zone[kk] > 1.0 / 45.0) & (au > hw + 1.2) & (au < hw + 1.2 + trap_w)
    runoff = outside & (zone[kk] <= 1.0 / 45.0) & (au > hw) & (au < hw + 6.0)
    cls[runoff] = tl.CLASSES.index("asphalt")
    cls[gravel] = tl.CLASSES.index("gravel")

    return {
        "base": base.astype(np.float32), "classes": cls, "roads": roads, "length": length, "osm_length": osm_length,
        "dense": (cx, cz, cdx, cdz, ck), "zone": zone, "trap_w": trap_w, "fence": fence, "fit_rms": fit_rms,
        "S": S, "pit_side": pit_side, "drag": drag, "short": short, "access": access, "pit": pit, "farm": farm,
        "in_fence": in_fence_c, "coef": coef,
    }


def tl_bilinear(grid, x, z):
    fi = np.clip(np.asarray(x) - ORIGIN, 0, N - 1.001)
    fj = np.clip(np.asarray(z) - ORIGIN, 0, N - 1.001)
    i, j = fi.astype(int), fj.astype(int)
    u, v = fi - i, fj - j
    return (grid[j, i] * (1 - u) + grid[j, i + 1] * u) * (1 - v) + (grid[j + 1, i] * (1 - u) + grid[j + 1, i + 1] * u) * v


# ------------------------------------------------------------------------------------------------------- markings
def export_length(rows, closed):
    """Length of the road as the game measures it: the polyline of the exported (rounded) 2 m samples."""
    s = tl.sample_bezier(tl.round_bezier(rows), closed, tl.SAMPLE_SPACING)
    x, z = np.round(s["x"], 3), np.round(s["z"], 3)
    return polyline_length(x, z, closed)


def markings(P):
    """Circuit road markings (game s along the exported samples): edge lines, the start line and grid, curbs at the
    apexes (inside) and corner exits (outside), none where another road joins."""
    cx, cz, cdx, cdz, ck = P["dense"]
    m = len(cx)
    scale = export_length(P["roads"]["circuit"][0], True) / P["length"]
    others = np.concatenate([np.stack(P[k][:2], 1) for k in ("drag", "short", "access", "pit")])
    hw = 0.5 * WIDTH

    def clear(i, side):
        x, z = cx[i] + cdz[i] * side * (hw + 0.5), cz[i] - cdx[i] * side * (hw + 0.5)
        return np.hypot(others[:, 0] - x, others[:, 1] - z).min() > 0.5 * WIDTH + 4.0

    kabs = np.abs(ck)
    curbs = []
    peaks = [i for i in range(m) if kabs[i] > 1.0 / 70.0 and kabs[i] == kabs[np.arange(i - 20, i + 21) % m].max()]
    for i in peaks:
        lo = i
        while kabs[lo % m] > 0.5 * kabs[i] and i - lo < 60:
            lo -= 1
        hi = i
        while kabs[hi % m] > 0.5 * kabs[i] and hi - i < 60:
            hi += 1
        side = 1.0 if ck[i] > 0 else -1.0
        span = hi - lo
        exit0 = i + int(0.15 * span)
        for a, b, sd in ((i - int(0.25 * span) - 3, i + int(0.25 * span) + 3, side), (exit0, min(hi + 8, exit0 + 30), -side)):
            run = [k for k in range(a, b + 1) if clear(k % m, sd)]
            if len(run) < 6:
                continue
            # Contiguous pieces only.
            start = prev = run[0]
            for k in run[1:] + [None]:
                if k is None or k != prev + 1:
                    if prev - start >= 5:
                        curbs.append([tl.r3((start % m) * scale), tl.r3(((prev % m) + 1) * scale), sd])
                    start = k
                prev = k if k is not None else prev
    return {"edgeLine": 0.2, "edgeInset": 0.3, "startLine": 0.0, "gridSlots": 12, "gridPitch": 8.0, "curbs": curbs}


# ------------------------------------------------------------------------------------------------------ placements
TREE_MODELS = {
    "conifer": ["fir_tree_01/fir_tree_01_a", "fir_tree_01/fir_tree_01_b", "fir_tree_01/fir_tree_01_c"],
    "sapling": ["fir_sapling_medium/fir_sapling_medium_a", "fir_sapling_medium/fir_sapling_medium_b",
                "fir_sapling_medium/fir_sapling_medium_c"],
    "broadleaf": ["tree_small_02/tree_small_02", "island_tree_01/island_tree_01"],
}
ROCK_MODELS = ["rocks/boulder_01", "rocks/boulder_02", "rocks/boulder_05"]
ROCK_SIZE = {"rocks/boulder_01": (1.25, 1.0, 1.83), "rocks/boulder_02": (2.51, 0.86, 1.23),
             "rocks/boulder_05": (1.36, 0.53, 0.74)}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def placements(P, roads_export):
    cx, cz, cdx, cdz, ck = P["dense"]
    m = len(cx)
    hw = 0.5 * WIDTH
    out = []

    def at(i, u):
        i %= m
        return cx[i] + cdz[i] * u, cz[i] - cdx[i] * u, math.atan2(cdx[i], cdz[i])

    def footprint_gap(p):
        fx, fz = tl.footprint(p)
        u = np.linspace(-fx, fx, max(2, int(2 * fx) + 1))
        v = np.linspace(-fz, fz, max(2, int(2 * fz) + 1))
        U = np.concatenate([u, u, np.full(len(v), -fx), np.full(len(v), fx)])
        V = np.concatenate([np.full(len(u), -fz), np.full(len(u), fz), v, v])
        c, sn = math.cos(p["yaw"]), math.sin(p["yaw"])
        return float(tl.road_edge_gap(roads_export, p["x"] + c * U + sn * V, p["z"] - sn * U + c * V).min())

    taken = []

    def kit(kind, x, z, yaw, w=0.0, d=0.0, h=0.0, variant=0, floors=1, force=False):
        p = {"kind": "building", "type": kind, "x": tl.r3(x), "z": tl.r3(z), "yaw": tl.r4(wrap(yaw)),
             "w": float(w), "d": float(d), "h": float(h), "floors": floors, "variant": variant}
        need = 3.0 if kind in tl.STRUCTURES else 0.3
        fx, fz = tl.footprint(p)
        r = 0.75 * min(fx, fz) + 0.25 * max(fx, fz)  # overlap test: a circle between the inner and outer one
        if not force:
            if footprint_gap(p) < need + 0.05:
                return False
            if any(math.hypot(x - tx, z - tz) < r + tr for tx, tz, tr in taken):
                return False
        out.append(p)
        taken.append((x, z, r))
        return True

    side = P["pit_side"]
    # Start / finish: gantry with the start lights over the line.
    x, z, yaw = at(0, 0.0)
    kit("start_gantry", x, z, yaw, w=WIDTH + 2.0, h=6.0, force=True)
    # Pit wall between the circuit and the pit lane, garages and race control beyond the pit lane facing it.
    for s in range(-88, 36, 4):
        x, z, yaw = at(s, side * (hw + 1.7))
        kit("barrier", x, z, yaw - 0.5 * math.pi, w=4.0)
    face = yaw_to(-side)  # garages face the pit lane (towards the circuit)
    for k, s in enumerate(range(-80, 30, 13)):
        x, z, yaw = at(s, side * (PIT_OFFSET + 0.5 * PIT_WIDTH + 3.5 + 6.0))
        if k == 4:
            kit("race_control", x, z, yaw + face, w=12.0, d=9.0)
        else:
            kit("garage", x, z, yaw + face, w=11.0, d=12.0, h=5.0, variant=0)
    # Grandstands across the start straight and outside the long sweeper of the west loop.
    for s, u in ((-48, -(hw + 4.8 + 6.0)), (-14, -(hw + 4.8 + 6.0))):
        x, z, yaw = at(s, side * u)
        kit("grandstand", x, z, yaw + yaw_to(side), w=30.0, d=10.0)
    for s in range(-100, 60, 4):
        x, z, yaw = at(s, -side * (hw + 1.6))
        kit("barrier", x, z, yaw - 0.5 * math.pi, w=4.0)

    # Corners: tyre walls at the outer edge of the gravel traps, guard rail beyond the asphalt run-offs.
    zone, kabs = P["zone"], np.abs(ck)
    turn = np.sign(np.convolve(np.concatenate([ck[-25:], ck, ck[:25]]), np.ones(51), "same")[25:-25])
    for i in range(0, m, 6):
        if zone[i] <= 0:
            continue
        o = -turn[i]
        if zone[i] > 1.0 / 45.0:
            tw = float(np.clip(300.0 * zone[i], 7.0, 16.0))
            x, z, yaw = at(i, o * (hw + 1.2 + tw + 1.0))
            kit("tyre_wall", x, z, yaw - 0.5 * math.pi, w=6.0, variant=(i // 6) % 2)
        else:
            x, z, yaw = at(i, o * (hw + 6.0 + 1.0))
            kit("barrier", x, z, yaw - 0.5 * math.pi, w=6.0, variant=1)
    # Marshal posts on the outside before the corners, flags at the start line.
    for i in range(30, m, 160):
        o = -turn[(i + 40) % m]
        x, z, yaw = at(i, o * (hw + 6.5))
        kit("marshal_post", x, z, yaw + yaw_to(-o))
    for k, s_flag in enumerate((-20, 20)):
        x, z, yaw = at(s_flag, -side * (hw + 3.2))
        kit("flag", x, z, yaw, h=6.0, variant=k)

    props = list(out)

    # Trees: pine woods and broadleaf clumps outside the fence and off the fields; a few inside the fence well away
    # from the asphalt. Rocks: a scatter of boulders at the wood edges.
    step = 6.0
    g = np.arange(-PLAY + 6, PLAY - 6, step)
    GX, GZ = np.meshgrid(g, g)
    kk = np.arange(GX.size).reshape(GX.shape)
    jx = (tl._hash(kk, np.zeros_like(kk), 31) - 0.5) * step * 0.9
    jz = (tl._hash(kk, np.zeros_like(kk), 32) - 0.5) * step * 0.9
    TX, TZ = (GX + jx).ravel(), (GZ + jz).ravel()
    woods = tl.fbm(TX / 90.0, TZ / 90.0, 3, SEED + 41)
    fence = P["fence"]
    inf = tl.point_in_polygon(TX, TZ, fence)
    ci = np.clip(np.round(TX - ORIGIN - 0.5).astype(int), 0, N - 2)
    cj = np.clip(np.round(TZ - ORIGIN - 0.5).astype(int), 0, N - 2)
    farm = P["farm"][cj, ci]
    density = np.where(inf, 0.06, np.where(farm, 0.02, 0.25 + 0.6 * smoothstep(0.45, 0.62, woods)))
    pick = tl._hash(kk.ravel(), np.ones(kk.size, np.int64), 33) < density
    TX, TZ, inf, W = TX[pick], TZ[pick], inf[pick], woods[pick]
    gap = tl.road_edge_gap(roads_export, TX, TZ)
    keep = gap > np.where(inf, 30.0, 12.0)
    for p in props:
        keep &= np.hypot(TX - p["x"], TZ - p["z"]) > 14.0
    for k, (x, z, wv) in enumerate(zip(TX[keep], TZ[keep], W[keep])):
        hsh = tl.hash01(k, 51)
        kind = ("conifer" if hsh < 0.7 else "sapling") if wv > 0.55 else ("broadleaf" if hsh < 0.6 else "conifer")
        models = TREE_MODELS[kind]
        model = models[int(tl.hash01(k, 52) * len(models)) % len(models)]
        out.append({"kind": "tree", "model": model, "x": tl.r3(x), "z": tl.r3(z),
                    "yaw": tl.r4(wrap(2 * math.pi * tl.hash01(k, 53))), "scale": tl.r3(0.75 + 0.4 * tl.hash01(k, 54)),
                    "width": tl.r3(0.9 + 0.2 * tl.hash01(k, 55))})
    k, rocks = 0, 0
    while k < 6000 and rocks < 70:
        x, z = -PLAY + 10 + (2 * PLAY - 20) * tl.hash01(k, 61), -PLAY + 10 + (2 * PLAY - 20) * tl.hash01(k, 62)
        k += 1
        if tl.point_in_polygon(np.array([x]), np.array([z]), fence)[0]:
            continue
        model = ROCK_MODELS[k % len(ROCK_MODELS)]
        s = 0.7 + 0.8 * tl.hash01(k, 63)
        size = ROCK_SIZE[model]
        if tl.road_edge_gap(roads_export, np.array([x]), np.array([z]))[0] - 0.5 * max(size[0], size[2]) * s * 2 < 8.0:
            continue
        out.append({"kind": "rock", "model": model, "x": tl.r3(x), "z": tl.r3(z), "yaw": tl.r4(wrap(6.283 * tl.hash01(k, 64))),
                    "scale": [tl.r3(s), tl.r3(s * 0.9), tl.r3(s)], "sink": 0.3})
        rocks += 1
    return out


def yaw_to(u):
    """Yaw offset from the track heading to a front (+Z) facing the left (u > 0) or right side."""
    return 0.5 * math.pi if u > 0 else -0.5 * math.pi


def markers(P):
    cx, cz, cdx, cdz, ck = P["dense"]
    m = len(cx)
    hw = 0.5 * WIDTH

    def gate(i, sector=False):
        g = {"x": tl.r3(cx[i]), "z": tl.r3(cz[i]), "yaw": tl.r4(math.atan2(cdx[i], cdz[i])), "halfWidth": hw + 3.0}
        if sector:
            g["sector"] = True
        return g

    sectors = [int(np.argmin(np.hypot(cx - x, cz - z))) for x, z in SECTOR_AT]
    # Anti-cut checkpoints in the loops the drag strip and the short course connector bypass: the middle of the
    # longest stretch between two junctions in each third of the lap, away from the other roads.
    others = np.concatenate([np.stack(P[k][:2], 1) for k in ("drag", "short", "access", "pit")])
    near = np.array([np.hypot(others[:, 0] - x, others[:, 1] - z).min() for x, z in zip(cx, cz)])
    cps = []
    for a, b in ((60, sectors[0]), (sectors[0], sectors[1]), (sectors[1], m - 60)):
        seg = np.arange(a + 20, b - 20)
        if len(seg):
            cps.append(int(seg[np.argmax(near[seg])]))
    idx = sorted([(0, False)] + [(i, True) for i in sectors] + [(i, False) for i in cps])
    gates = [gate(i, s) for i, s in idx]
    # Spawn: pole position of the grid (left, behind the start line).
    i = (-8) % m
    spawn = {"x": tl.r3(cx[i] + cdz[i] * 0.24 * WIDTH), "z": tl.r3(cz[i] - cdx[i] * 0.24 * WIDTH),
             "yaw": tl.r4(math.atan2(cdx[i], cdz[i]))}

    def near_pt(x, z):
        k = int(np.argmin(np.hypot(cx - x, cz - z)))
        return [tl.r3(cx[k]), tl.r3(cz[k])]

    side = P["pit_side"]
    px, pz = cx[-30] + cdz[-30] * side * PIT_OFFSET * 1.0, cz[-30] - cdx[-30] * side * PIT_OFFSET
    dx_, dz_ = P["drag"][0], P["drag"][1]
    hp = int(np.argmax(np.abs(ck)))
    pois = [
        {"id": 1, "name": "Start / Finish", "kind": "start", "x": tl.r3(cx[0]), "z": tl.r3(cz[0]), "at": near_pt(cx[0], cz[0])},
        {"id": 2, "name": "Pit Lane", "kind": "pit", "x": tl.r3(px), "z": tl.r3(pz), "at": [tl.r3(px), tl.r3(pz)]},
        {"id": 3, "name": "Drag Strip", "kind": "drag", "x": tl.r3(dx_[0]), "z": tl.r3(dz_[0]),
         "at": [tl.r3(dx_[3]), tl.r3(dz_[3])]},
        {"id": 4, "name": "Hairpin", "kind": "corner", "x": tl.r3(cx[hp]), "z": tl.r3(cz[hp]), "at": near_pt(cx[hp], cz[hp])},
        {"id": 5, "name": "Sector 2", "kind": "sector", "x": tl.r3(cx[sectors[0]]), "z": tl.r3(cz[sectors[0]]),
         "at": near_pt(cx[sectors[0]], cz[sectors[0]])},
        {"id": 6, "name": "Sector 3", "kind": "sector", "x": tl.r3(cx[sectors[1]]), "z": tl.r3(cz[sectors[1]]),
         "at": near_pt(cx[sectors[1]], cz[sectors[1]])},
    ]
    return spawn, gates, pois


def report(P):
    cx, cz, cdx, cdz, ck = P["dense"]
    S = P["S"]
    h = S(cx, cz)
    grade = np.abs(np.diff(np.append(h, h[0])))
    kabs = np.abs(ck)
    print(f"TF: OSM circuit polyline {P['osm_length']:.0f} m, smoothed center line {P['length']:.0f} m "
          f"(published 2210 m), drag strip {P['drag'][2]:.0f} m, short course connector {P['short'][2]:.0f} m, "
          f"pit lane {P['pit'][2]:.0f} m")
    print(f"TF: track surface {h.min():.2f}..{h.max():.2f} m (quadratic fit to the DEM, rms {P['fit_rms']:.2f} m), "
          f"max grade {grade.max() * 100:.2f} %, tightest radius {1.0 / kabs.max():.1f} m, "
          f"terrain {P['base'].min():.1f}..{P['base'].max():.1f} m")
    cl = P["classes"]
    print("TF: cells", {c: int((cl == k).sum()) for k, c in enumerate(tl.CLASSES) if (cl == k).any()})


def build(P, blend, out):
    import bpy  # noqa: PLC0415
    import export_level  # noqa: PLC0415

    tl.clear_scene()
    sc = bpy.context.scene
    sc["tf_level"] = LEVEL_ID
    sc["tf_samples"], sc["tf_origin"], sc["tf_cell"] = N, ORIGIN, 1.0
    tl.build_terrain(P["base"], P["classes"], ORIGIN, 1.0, P["base"], P["classes"])
    mk = markings(P)
    for name, (rows, closed, kind, width) in P["roads"].items():
        tl.build_curve("TRACK_" + name, tl.round_bezier(rows), closed, kind, width, ["asphalt"] * len(rows),
                       mk if name == "circuit" else None)
    heights, classes, worst = tl.carve_scene()
    print(f"TF: carved, largest change at the carve reach {worst:.2f} m; {len(mk['curbs'])} curbs")
    lo, hi = math.floor(float(heights.min())) - 1.0, math.ceil(float(heights.max())) + 1.0
    sc["tf_height_min"], sc["tf_height_max"] = lo, hi

    roads = []
    for o in sorted(bpy.data.collections["TRACKS"].objects, key=lambda o: o.name):
        rows, closed, kind, width, surf, name, _ = tl.read_curve(o)
        s = tl.sample_bezier(rows, closed, tl.SAMPLE_SPACING)
        roads.append({"name": name, "kind": kind, "width": width,
                      "samples": [[x, z, b] for x, z, b in zip(s["x"], s["z"], s["bank"])]})
    pl = placements(P, roads)
    hgrid = heights.astype(np.float64)

    def ground(x, z):
        return float(hgrid[int(round(z - ORIGIN)), int(round(x - ORIGIN))])

    for k, p in enumerate(pl):
        tl.placement_object(f"{p['kind']}_{k:05d}", p, ground(p["x"], p["z"]))
    spawn, gates, pois = markers(P)
    mcol = tl.collection("MARKERS")
    tl.empty("SPAWN", mcol, spawn["x"], spawn["z"], ground(spawn["x"], spawn["z"]) + 1.0, spawn["yaw"], "ARROWS", 4.0)
    for k, cp in enumerate(gates):
        o = tl.empty(f"CP_{k:02d}", mcol, cp["x"], cp["z"], ground(cp["x"], cp["z"]), cp["yaw"], "CUBE", 1.0,
                     (cp["halfWidth"], 0.2, 2.0))
        o["tf_half_width"] = cp["halfWidth"]
        if cp.get("sector"):
            o["tf_sector"] = True
    for p in pois:
        o = tl.empty(f"POI_{p['id']:02d}", mcol, p["x"], p["z"], ground(p["x"], p["z"]) + 2.0, 0.0, "SPHERE", 3.0)
        o["tf_id"], o["tf_name"], o["tf_poi_kind"], o["tf_at"] = p["id"], p["name"], p["kind"], p["at"]
    counts = {}
    for p in pl:
        key = p.get("type", p["kind"])
        counts[key] = counts.get(key, 0) + 1
    print("TF: placements", dict(sorted(counts.items())), "gates", len(gates))
    if blend:
        os.makedirs(os.path.dirname(os.path.abspath(blend)), exist_ok=True)
        bpy.context.preferences.filepaths.save_version = 0  # no .blend1 backup
        bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(blend), compress=True)
        print(f"TF: saved {blend}")
    if out:
        export_level.run(os.path.abspath(out))


def main():
    in_blender = os.path.basename(sys.argv[0]).lower().startswith("blender")
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else ([] if in_blender else sys.argv[1:])
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", action="store_true", help="print the numbers only (no Blender)")
    ap.add_argument("--blend", default=os.path.join(tl.REPO, "build", "levels", LEVEL_ID + ".blend"))
    ap.add_argument("--out", default=os.path.join(tl.REPO, "assets", "levels", LEVEL_ID))
    a = ap.parse_args(argv)
    P = plan()
    report(P)
    if a.plan:
        mk = markings(P)
        tot = sum((c[1] - c[0]) % P["length"] for c in mk["curbs"])
        print(f"TF: {len(mk['curbs'])} curbs, {tot:.0f} m")
        spawn, gates, pois = markers(P)
        print("TF: gates", [(g["x"], g["z"], g.get("sector", False)) for g in gates])
    else:
        build(P, a.blend, a.out)


if __name__ == "__main__":
    main()
