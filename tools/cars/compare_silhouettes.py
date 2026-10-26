#!/usr/bin/env python3
"""Silhouette IoU of the procedural vehicle model against the traced concept views.

The model comes from trailforge_model_export (OBJ in body space at rest on flat ground). The concept silhouettes come
from assets/vehicles/<id>/blueprint.json (concept frame: z from the wheelbase midpoint, y from the axle line) and are
placed on the exported model's axles: y + the mean rest hub height, z mapped like the body generator (concept axles
onto the physics axles, overhangs unscaled). Both are rasterised orthographically per view (side from +X, top from
+Y, front from +Z, rear from -Z); the blueprint's `ignore` regions (antennas, roof spare: drawn but not modelled) are
masked out of both. Writes concept / model / overlap PNGs per view.

Usage: python3 tools/cars/compare_silhouettes.py model.obj blueprint.json [--out /tmp/prefix] [--no-warp]
"""

import argparse
import json
import re

import cv2
import numpy as np
from PIL import Image

PPM = 200.0  # raster pixels per meter
BOUNDS = {  # (u range, v range) in meters per view; u to the right, v up in the PNG
    "side": ((-2.9, 2.9), (-1.0, 1.6)),   # u = -z (front on the left, as in the concept), v = y
    "top": ((-2.9, 2.9), (-1.4, 1.4)),    # u = -z, v = -x (car left at the bottom, as in the concept)
    "front": ((-1.4, 1.4), (-1.0, 1.6)),  # u = x (car left on the right), v = y
    "rear": ((-1.4, 1.4), (-1.0, 1.6)),   # u = -x, v = y
}


def load_obj(path):
    objs, cur, verts, faces = {}, None, [], []
    for line in open(path, encoding="utf-8"):
        if line.startswith("o "):
            cur = line[2:].strip()
            objs[cur] = []
        elif line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(x) - 1 for x in line.split()[1:4]]
            objs[cur].append(idx)
    V = np.array(verts, np.float64)
    return V, {k: np.array(v, np.int64).reshape(-1, 3) for k, v in objs.items()}


def to_px(view, u, v):
    (u0, u1), (v0, v1) = BOUNDS[view]
    return np.stack([(u - u0) * PPM, (v1 - v) * PPM], -1)


def canvas(view):
    (u0, u1), (v0, v1) = BOUNDS[view]
    return np.zeros((int((v1 - v0) * PPM), int((u1 - u0) * PPM)), np.uint8)


def uv_body(view, p):
    """Body-space points (N, 3) -> view (u, v)."""
    x, y, z = p[..., 0], p[..., 1], p[..., 2]
    return {"side": (-z, y), "top": (-z, -x), "front": (x, y), "rear": (-x, y)}[view]


def raster_tris(view, V, F):
    m = canvas(view)
    u, v = uv_body(view, V[F])
    px = np.round(to_px(view, u, v) * 4).astype(np.int32)  # 2 bits of sub-pixel precision
    for tri in px:  # one by one: fillPoly over many contours would apply the even-odd rule across triangles
        cv2.fillConvexPoly(m, tri, 1, lineType=cv2.LINE_8, shift=2)
    return m


def raster_polys(view, polys):
    m = canvas(view)
    pts = [np.round(to_px(view, *np.array(p, np.float64).T) * 4).astype(np.int32) for p in polys if len(p) >= 3]
    if pts:
        cv2.fillPoly(m, pts, 1, shift=2)
    return m


def concept_polys(bp, view, key, y_off, zmap):
    """Concept polygons of one view mapped to (u, v) meters."""
    out = []
    for poly in bp[view].get(key, []):
        p = np.array(poly, np.float64)
        if view == "side":  # (z, y)
            u, v = -zmap(p[:, 0]), p[:, 1] + y_off
        elif view == "top":  # (z, x)
            u, v = -zmap(p[:, 0]), -p[:, 1]
        elif view == "front":  # (x, y)
            u, v = p[:, 0], p[:, 1] + y_off
        else:  # rear: (x, y), u = -x
            u, v = -p[:, 0], p[:, 1] + y_off
        out.append(np.stack([u, v], 1))
    return out


def raster_uv(view, polys):
    m = canvas(view)
    pts = [np.round(to_px(view, p[:, 0], p[:, 1]) * 4).astype(np.int32) for p in polys if len(p) >= 3]
    if pts:
        cv2.fillPoly(m, pts, 1, shift=2)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("obj")
    ap.add_argument("blueprint")
    ap.add_argument("--out", default=None, help="PNG prefix (concept red, model cyan, overlap white)")
    ap.add_argument("--no-warp", action="store_true", help="keep the concept wheelbase (no axle mapping)")
    a = ap.parse_args()
    bp = json.load(open(a.blueprint, encoding="utf-8"))
    V, objs = load_obj(a.obj)
    hubs = []
    for k, F in objs.items():
        if re.fullmatch(r"wheel_\d", k):
            P = V[F.reshape(-1)]
            hubs.append(0.5 * (P.min(0) + P.max(0)))
    hubs = np.array(hubs)
    y_off = float(np.mean(hubs[:, 1]))
    zA_p = 0.5 * (hubs[:, 2].max() - hubs[:, 2].min())
    zA_c = 0.5 * bp["scale"]["conceptWheelBase"]

    def zmap(zc):
        if a.no_warp:
            return zc
        zc = np.asarray(zc)
        inner = np.abs(zc) <= zA_c
        return np.where(inner, zc * zA_p / zA_c, np.sign(zc) * (zA_p + np.abs(zc) - zA_c))

    allF = np.concatenate(list(objs.values()))
    res = {}
    for view in ("side", "front", "rear", "top"):
        model = raster_tris(view, V, allF)
        concept = raster_uv(view, concept_polys(bp, view, "silhouette", y_off, zmap))
        ign = raster_uv(view, concept_polys(bp, view, "ignore", y_off, zmap))
        keep = ign == 0
        inter = int(((model > 0) & (concept > 0) & keep).sum())
        union = int((((model > 0) | (concept > 0)) & keep).sum())
        iou = inter / union if union else 0.0
        res[view] = round(iou, 4)
        if a.out:
            img = np.zeros(model.shape + (3,), np.uint8)
            img[(concept > 0) & keep] = (220, 40, 40)
            img[(model > 0) & keep] = (40, 200, 220)
            img[(model > 0) & (concept > 0) & keep] = (235, 235, 235)
            img[~keep & ((model > 0) | (concept > 0))] = (90, 90, 90)
            Image.fromarray(img).save(f"{a.out}_{view}.png")
    print(json.dumps({"iou": res, "restHubY": round(y_off, 4), "axleZ": round(float(zA_p), 4),
                      "conceptAxleZ": zA_c, "warp": not a.no_warp}))


if __name__ == "__main__":
    main()
