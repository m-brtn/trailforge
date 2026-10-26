"""Shared code of the level Blender pipeline (README "Editing a level in Blender").

Game files of a baked level (assets/levels/<id>/, level.json "baked"):
  heightmap.pgm  16-bit P5, samples^2 heights, row j = game z (origin + j * cell), column i = game x; the range is
                 layout.json grid heightMin..heightMax (0 .. 65535)
  surfaces.pgm   8-bit P5, (samples - 1)^2 cells, value = surface class (CLASSES)
  layout.json    roads (2 m center line samples x, z, bank + the Bezier for Blender), water, placements, spawn,
                 checkpoints, POIs
Blender scene (coordinates: game x, z, height -> Blender x, -z, z; game yaw = rotation_euler.z, the forward of an
object is its local -Y like a vehicle body):
  TERRAIN        grid mesh, vertex j * samples + i; colour attribute tf_surface (CORNER, palette below), point
                 attribute tf_base_height and face attribute tf_base_surface (the terrain before carve_track.py)
  TRACKS         Bezier curves; tilt = bank (rad, + left edge higher), custom properties tf_kind (circuit, trail,
                 road), tf_width (m), tf_segment_surface (class name per control point: the segment that starts there)
  WATER          flat meshes, one face each: the polygon and the water level (render only in the game)
  PLACEMENTS     empties: tf_kind tree / rock / building, tf_ref model ref (vegetation library) or kit type, sizes
  MARKERS        SPAWN, CP_00.. (CP_00 = start / finish line, tf_half_width), POI_..
Pure numpy except the functions that take or build Blender data (bpy is imported lazily).
"""

import json
import math
import os

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))

CLASSES = ["asphalt", "gravel", "dirt", "mud", "grass", "sand", "rock", "snow"]  # src/track/LevelFiles.h order
PALETTE = np.array([[38, 38, 42], [168, 158, 136], [128, 92, 60], [64, 44, 28], [76, 128, 52], [224, 204, 150],
                    [120, 120, 124], [246, 246, 250]], np.uint8)
SAMPLE_SPACING = 2.0  # m between exported road samples
CARVE_SPACING = 1.0   # m between the samples the carve uses
CUT_SLOPE = 0.65      # rise per m of the cut / fill banks beside a carved road
CARVE_REACH = 18.0    # m beyond the road edge the carve may change the terrain
SHOULDER = 1.5      # m of flat ground at the road height beside the edge
VERGE = 1.0           # m of verge (dirt; gravel beside asphalt) beside the road surface


# ------------------------------------------------------------------------------------------------------------ files
def write_pgm(path, arr):
    arr = np.asarray(arr)
    h, w = arr.shape
    wide = arr.dtype != np.uint8
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n{65535 if wide else 255}\n".encode())
        f.write(arr.astype(">u2").tobytes() if wide else arr.astype(np.uint8).tobytes())


def read_pgm(path):
    data = open(path, "rb").read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            while data[pos:pos + 1] not in (b"\n", b""):
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    if tokens[0] != b"P5":
        raise ValueError(f"{path}: not a binary PGM")
    w, h, maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])
    pos += 1
    if maxv > 255:
        return np.frombuffer(data, ">u2", w * h, pos).reshape(h, w).astype(np.uint16)
    return np.frombuffer(data, np.uint8, w * h, pos).reshape(h, w).copy()


def r3(v):
    return float(round(float(v), 3)) + 0.0


def r4(v):
    return float(round(float(v), 4)) + 0.0


def dump_json(value, indent=0):
    """Stable, diff-friendly JSON: objects one key per line, arrays of numbers inline, other arrays one item per line."""
    pad, inner = " " * indent, " " * (indent + 1)
    if isinstance(value, dict):
        if not value:
            return "{}"
        items = [f'{inner}{json.dumps(k)}: {dump_json(v, indent + 1)}' for k, v in value.items()]
        return "{\n" + ",\n".join(items) + "\n" + pad + "}"
    if isinstance(value, (list, tuple)):
        if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
            return "[" + ", ".join(dump_json(v) for v in value) + "]"
        if all(isinstance(v, dict) and all(not isinstance(x, (dict, list)) or
                                           all(isinstance(y, (int, float)) for y in x) for x in v.values())
               for v in value):
            return "[\n" + ",\n".join(inner + json.dumps(v, separators=(", ", ": ")) for v in value) + "\n" + pad + "]"
        return "[\n" + ",\n".join(inner + dump_json(v, indent + 1) for v in value) + "\n" + pad + "]"
    if isinstance(value, float):
        if value == int(value) and abs(value) < 1e15:
            return str(int(value))
        return repr(value)
    return json.dumps(value)


# ----------------------------------------------------------------------------------------------------------- noise
def _hash(i, j, seed):
    h = (i.astype(np.int64) * 374761393 + j.astype(np.int64) * 668265263 + seed * 2147483647) & 0xFFFFFFFF
    h = ((h ^ (h >> 13)) * 1274126177) & 0xFFFFFFFF
    return ((h ^ (h >> 16)) & 0xFFFFFF).astype(np.float64) / float(0xFFFFFF)


def value_noise(x, z, seed):
    i, j = np.floor(x), np.floor(z)
    fx, fz = x - i, z - j
    ux, uz = fx * fx * (3 - 2 * fx), fz * fz * (3 - 2 * fz)
    i, j = i.astype(np.int64), j.astype(np.int64)
    a, b = _hash(i, j, seed), _hash(i + 1, j, seed)
    c, d = _hash(i, j + 1, seed), _hash(i + 1, j + 1, seed)
    return (a + (b - a) * ux) + ((c + (d - c) * ux) - (a + (b - a) * ux)) * uz


def fbm(x, z, octaves, seed):
    total, amp, norm = 0.0, 0.5, 0.0
    for o in range(octaves):
        total = total + amp * value_noise(x * (2 ** o), z * (2 ** o), seed + 17 * o)
        norm += amp
        amp *= 0.5
    return total / norm


def hash01(k, seed):
    return float(_hash(np.array([k]), np.array([seed]), 977)[0])


# ---------------------------------------------------------------------------------------------------------- curves
def catmull_rom_bezier(points, closed=True):
    """Bezier control rows [x, z, h, lx, lz, lh, rx, rz, rh] through points (N x 3) with Catmull-Rom tangents."""
    p = np.asarray(points, float)
    n = len(p)
    prev = np.roll(p, 1, 0) if closed else np.vstack([p[:1], p[:-1]])
    nxt = np.roll(p, -1, 0) if closed else np.vstack([p[1:], p[-1:]])
    t = (nxt - prev) * 0.5
    if not closed:
        t[0], t[-1] = p[1] - p[0], p[-1] - p[-2]
    return np.hstack([p, p - t / 3.0, p + t / 3.0]).reshape(n, 9)


def round_bezier(rows):
    return [[r3(v) for v in row[:9]] + [r4(row[9])] for row in rows]


def sample_bezier(rows, closed, spacing, sub=64):
    """Samples at equal horizontal arc length: arrays x, z, h, bank and the control segment of each sample.
    rows: [x, z, h, lx, lz, lh, rx, rz, rh, tilt] (game coordinates). Tilt is linear in the segment parameter."""
    rows = np.asarray(rows, float)
    n = len(rows)
    segs = n if closed else n - 1
    t = np.linspace(0.0, 1.0, sub, endpoint=False)
    pts, tilts, seg = [], [], []
    for k in range(segs):
        a, b = rows[k], rows[(k + 1) % n]
        p0, p1, p2, p3 = a[0:3], a[6:9], b[3:6], b[0:3]
        u = 1.0 - t
        c = (u ** 3)[:, None] * p0 + (3 * u * u * t)[:, None] * p1 + (3 * u * t * t)[:, None] * p2 + (t ** 3)[:, None] * p3
        pts.append(c)
        tilts.append(a[9] + (b[9] - a[9]) * t)
        seg.append(np.full(sub, k))
    pts.append(rows[0:1, 0:3] if closed else rows[-1:, 0:3])
    tilts.append(np.array([rows[0, 9] if closed else rows[-1, 9]]))
    seg.append(np.array([segs - 1]))
    P, T, S = np.vstack(pts), np.concatenate(tilts), np.concatenate(seg)
    d = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])))])
    total = d[-1]
    count = max(2, int(round(total / spacing)))
    s = np.arange(count) * (total / count) if closed else np.linspace(0.0, total, count + 1)
    out = {"x": np.interp(s, d, P[:, 0]), "z": np.interp(s, d, P[:, 1]), "h": np.interp(s, d, P[:, 2]),
           "bank": np.interp(s, d, T), "s": s, "length": total}
    idx = np.clip(np.searchsorted(d, s, side="right") - 1, 0, len(S) - 1)
    out["seg"] = S[idx]
    return out


def tangents(x, z, closed):
    if closed:
        dx, dz = np.roll(x, -1) - np.roll(x, 1), np.roll(z, -1) - np.roll(z, 1)
    else:
        dx, dz = np.gradient(x), np.gradient(z)
    ln = np.maximum(np.hypot(dx, dz), 1e-6)
    return dx / ln, dz / ln


# ----------------------------------------------------------------------------------------------------------- carve
def carve(base_h, base_cls, roads, origin, cell=1.0):
    """Terrain and surface classes with the roads cut in: the road surface follows the curve height and bank, cut /
    fill banks at CUT_SLOPE meet the base terrain, the road class inside the width, a verge beside it, dirt on the
    fresh earthworks. roads: dicts {x, z, h, bank, width, cls (per sample, class index), closed} at CARVE_SPACING.
    Returns (heights float32, classes uint8, worst mismatch at the carve reach in m)."""
    n = base_h.shape[0]
    xs = origin + np.arange(n) * cell
    cx = origin + (np.arange(n - 1) + 0.5) * cell
    X = np.concatenate([r["x"] for r in roads])
    Z = np.concatenate([r["z"] for r in roads])
    H = np.concatenate([r["h"] for r in roads])
    B = np.concatenate([r["bank"] for r in roads])
    W = np.concatenate([np.full(len(r["x"]), 0.5 * r["width"]) for r in roads])
    C = np.concatenate([np.asarray(r["cls"], np.int64) for r in roads])
    R = np.concatenate([np.full(len(r["x"]), q) for q, r in enumerate(roads)])
    PRIO = np.array([bool(r.get("priority", False)) for r in roads])
    DX, DZ, GH, GB = [], [], [], []
    for r in roads:
        dx, dz = tangents(r["x"], r["z"], r["closed"])
        DX.append(dx)
        DZ.append(dz)
        # Height and bank change per m along the road: the surface between samples is extrapolated linearly from
        # the nearest one, so neighbouring samples meet without a step (smooth to second order).
        ds = np.maximum(np.hypot(np.gradient(r["x"]), np.gradient(r["z"])), 1e-6)
        if r["closed"]:
            ds = np.maximum(0.5 * np.hypot(np.roll(r["x"], -1) - np.roll(r["x"], 1), np.roll(r["z"], -1) - np.roll(r["z"], 1)), 1e-6)
            GH.append(0.5 * (np.roll(r["h"], -1) - np.roll(r["h"], 1)) / ds)
            GB.append(0.5 * (np.roll(r["bank"], -1) - np.roll(r["bank"], 1)) / ds)
        else:
            GH.append(np.gradient(r["h"]) / ds)
            GB.append(np.gradient(r["bank"]) / ds)
    DX, DZ = np.concatenate(DX), np.concatenate(DZ)
    GH, GB = np.concatenate(GH), np.concatenate(GB)

    def nearest_of(coords, m, ks):
        best = np.full((m, m), np.inf)
        bk = np.full((m, m), -1, np.int64)
        for k in ks:
            reach = W[k] + CARVE_REACH
            i0, i1 = np.searchsorted(coords, X[k] - reach), np.searchsorted(coords, X[k] + reach)
            j0, j1 = np.searchsorted(coords, Z[k] - reach), np.searchsorted(coords, Z[k] + reach)
            if i0 >= i1 or j0 >= j1:
                continue
            d2 = (coords[None, i0:i1] - X[k]) ** 2 + (coords[j0:j1, None] - Z[k]) ** 2
            win = best[j0:j1, i0:i1]
            better = d2 < win
            win[better] = d2[better]
            bk[j0:j1, i0:i1][better] = k
        return best, bk

    def frame(coords, best, bk):
        k = np.where(bk >= 0, bk, 0)
        gx, gz = coords[None, :] - X[k], coords[:, None] - Z[k]
        u = gx * DZ[k] - gz * DX[k]  # + left of the direction of travel
        along = gx * DX[k] + gz * DZ[k]
        # Past the end of an open road the distance is the one to the end sample (a round cap).
        a = np.clip(along, -0.6 * CARVE_SPACING, 0.6 * CARVE_SPACING)
        d = np.sqrt(np.maximum(best - a * a, 0.0))
        return k, u, d, along

    def nearest(coords, m):
        """Per cell the sample of the road it is deepest inside (or nearest the edge of); inside a priority road
        (the circuit) that road wins, so junctions and overlapping roads leave no seams in it."""
        key = np.full((m, m), np.inf)
        best = np.full((m, m), np.inf)
        bk = np.full((m, m), -1, np.int64)
        for q in range(len(roads)):
            b, kk = nearest_of(coords, m, np.nonzero(R == q)[0])
            k, _, d, _ = frame(coords, b, kk)
            gap = np.where(kk >= 0, d - W[k], np.inf)
            kq = np.where(PRIO[q] & (gap <= 0.0), gap - 1.0e3, gap)
            better = kq < key
            key[better], best[better], bk[better] = kq[better], b[better], kk[better]
        return best, bk

    best, bk = nearest(xs, n)
    k, u, d, along = frame(xs, best, bk)
    inside = (bk >= 0) & (d <= W[k] + CARVE_REACH)
    target = H[k] + GH[k] * along + np.tan(B[k] + GB[k] * along) * np.clip(u, -W[k], W[k])
    # A flat shoulder 1.5 m wide beside the road (curbs sit on it; the 1 m height grid keeps the bank off the road).
    e = np.maximum(0.0, d - W[k] - SHOULDER)
    h = np.where(inside, np.clip(base_h, target - CUT_SLOPE * e, target + CUT_SLOPE * e), base_h)
    rim = inside & (d > W[k] + CARVE_REACH - 1.0)
    worst = float(np.max(np.abs(h - base_h)[rim])) if rim.any() else 0.0

    best_c, bk_c = nearest(cx, n - 1)
    kc, uc, dc, _ = frame(cx, best_c, bk_c)
    cls = base_cls.copy()
    moved = np.abs(h - base_h)
    moved_c = np.maximum.reduce([moved[:-1, :-1], moved[1:, :-1], moved[:-1, 1:], moved[1:, 1:]])
    has = bk_c >= 0
    road = has & (dc <= W[kc])
    road_cls = C[kc]
    # Beside asphalt the base classes stay (a circuit paints its run-offs into the base surface).
    verge = has & ~road & (dc <= W[kc] + VERGE) & (road_cls != CLASSES.index("asphalt"))
    earth = has & ~road & ~verge & (moved_c > 0.35) & (road_cls != CLASSES.index("asphalt"))
    verge_cls = np.where(road_cls == CLASSES.index("asphalt"), CLASSES.index("gravel"),
                         np.where(road_cls == CLASSES.index("mud"), CLASSES.index("mud"), CLASSES.index("dirt")))
    cls[earth] = CLASSES.index("dirt")
    cls[verge] = verge_cls[verge]
    cls[road] = road_cls[road]
    return h.astype(np.float32), cls.astype(np.uint8), worst


def road_for_carve(rows, closed, width, segment_surface):
    s = sample_bezier(rows, closed, CARVE_SPACING)
    cls = np.array([CLASSES.index(segment_surface[int(g) % len(segment_surface)]) for g in s["seg"]])
    return {"x": s["x"], "z": s["z"], "h": s["h"], "bank": s["bank"], "width": width, "cls": cls, "closed": closed}


# ------------------------------------------------------------------------------------------------------ validation
def road_edge_gap(roads, px, pz):
    """Distance from points to the nearest road edge (m, negative on the road). roads: export dicts."""
    px, pz = np.atleast_1d(np.asarray(px, float)), np.atleast_1d(np.asarray(pz, float))
    best = np.full(px.shape, np.inf)
    for r in roads:
        sx, sz = np.array([p[0] for p in r["samples"]]), np.array([p[1] for p in r["samples"]])
        for k in range(0, len(px), 256):
            d = np.hypot(px[k:k + 256, None] - sx[None, :], pz[k:k + 256, None] - sz[None, :]).min(axis=1)
            best[k:k + 256] = np.minimum(best[k:k + 256], d - 0.5 * r["width"])
    return best


def point_in_polygon(px, pz, poly):
    px, pz = np.asarray(px, float), np.asarray(pz, float)
    inside = np.zeros(px.shape, bool)
    n = len(poly)
    for a in range(n):
        x0, z0 = poly[a]
        x1, z1 = poly[(a + 1) % n]
        cross = ((z0 > pz) != (z1 > pz)) & (px < (x1 - x0) * (pz - z0) / ((z1 - z0) + 1e-12) + x0)
        inside ^= cross
    return inside


STRUCTURES = ("house", "fuel_canopy", "fuel_shop", "garage", "lighthouse", "radio_mast", "radio_hut", "ruin_wall",
              "ruin_tower", "ruin_gate", "crusher", "silo", "grandstand", "race_control")


def footprint(p):
    """Half extents (x along the width, z along the depth) of a kit placement, about its bounds in the game."""
    t, w, d = p["type"], p.get("w", 0.0), p.get("d", 0.0)
    table = {"barrier": (0.5 * (w or 4.0), 0.3), "tyre_wall": (0.5 * (w or 6.0), 0.33), "flag": (0.25, 0.25),
             "bench": (0.5 * (w or 1.8), 0.25), "tent": (1.3, 1.5), "signpost": (0.3, 0.3),
             "start_gantry": (0.5 * w + 0.7, 0.45), "grandstand": (0.5 * w + 0.3, 0.5 * d + 0.4),
             "marshal_post": (1.1, 1.1)}
    return table.get(t, (0.5 * w + 0.3, 0.5 * d + 0.3))


def validate(layout, heights, play_half):
    """The layout rules of the game (Level checkBakedLayout), approximated with point / circle footprints.
    Returns a list of issue strings."""
    issues = []
    roads = layout["roads"]
    grid = layout["grid"]
    n, origin = grid["samples"], grid["origin"]

    def height(x, z):
        i = np.clip((np.asarray(x) - origin), 0, n - 1.001)
        j = np.clip((np.asarray(z) - origin), 0, n - 1.001)
        i0, j0 = i.astype(int), j.astype(int)
        u, v = i - i0, j - j0
        h00, h10 = heights[j0, i0], heights[j0, i0 + 1]
        h01, h11 = heights[j0 + 1, i0], heights[j0 + 1, i0 + 1]
        return (h00 * (1 - u) + h10 * u) * (1 - v) + (h01 * (1 - u) + h11 * u) * v

    def water_level(x, z):
        out = np.full(np.shape(x), -np.inf)
        for w in layout["water"]:
            inside = point_in_polygon(x, z, w["polygon"])
            out = np.where(inside, np.maximum(out, w["level"]), out)
        return out

    pl = layout["placements"]
    if pl:
        px = np.array([p["x"] for p in pl])
        pz = np.array([p["z"] for p in pl])
        gap = road_edge_gap(roads, px, pz)
        for p, g in zip(pl, gap):
            if abs(p["x"]) > play_half or abs(p["z"]) > play_half:
                issues.append(f"{p['kind']} at ({p['x']}, {p['z']}) is outside the play area")
            if p["kind"] in ("tree", "rock"):
                r = 0.5 * max(p.get("scale", [1, 1, 1])[0], p.get("scale", [1, 1, 1])[2]) * 2.0 \
                    if p["kind"] == "rock" else 0.6
                if g - r < 1.0:
                    issues.append(f"{p['kind']} at ({p['x']}, {p['z']}) is {g - r:.2f} m from a road edge")
            else:
                need = 3.0 if p["type"] in STRUCTURES else 0.3
                fx, fz = footprint(p)
                if p["type"] == "start_gantry":  # spans the road: only the pillars stand beside it
                    pts = [(sgn * (fx - 0.45), 0.0) for sgn in (-1.0, 1.0)]
                    fx = fz = 0.45
                else:
                    pts = [(0.0, 0.0)]
                gmin = np.inf
                for ox, oz in pts:
                    u = np.linspace(-fx, fx, max(2, int(2 * fx) + 1))
                    v = np.linspace(-fz, fz, max(2, int(2 * fz) + 1))
                    U = np.concatenate([u, u, np.full(len(v), -fx), np.full(len(v), fx)]) + ox
                    V = np.concatenate([np.full(len(u), -fz), np.full(len(u), fz), v, v]) + oz
                    c, sn = math.cos(p["yaw"]), math.sin(p["yaw"])
                    gmin = min(gmin, float(road_edge_gap(roads, p["x"] + c * U + sn * V, p["z"] - sn * U + c * V).min()))
                if gmin < need - 1e-6:
                    issues.append(f"{p['type']} at ({p['x']}, {p['z']}) is {gmin:.2f} m from a road edge (min {need})")
        wl = water_level(px, pz)
        for p, w in zip(pl, wl):
            if p["kind"] == "building" and p["type"] in STRUCTURES and w > -np.inf:
                issues.append(f"{p['type']} at ({p['x']}, {p['z']}) stands in water")
    circuit = [r for r in roads if r["kind"] == "circuit"]
    if layout["checkpoints"] and len(circuit) != 1:
        issues.append("checkpoints need exactly one road of kind circuit")
    if circuit:
        c = circuit[0]
        sx, sz = np.array([p[0] for p in c["samples"]]), np.array([p[1] for p in c["samples"]])
        for k, g in enumerate(layout["checkpoints"]):
            d = np.hypot(sx - g["x"], sz - g["z"]).min()
            if d > 2.0 or g["halfWidth"] < 0.5 * c["width"]:
                issues.append(f"checkpoint {k} is {d:.1f} m off the circuit or narrower than the road")
        sp = layout["spawn"]
        if np.hypot(sx - sp["x"], sz - sp["z"]).min() > 3.0:
            issues.append("spawn is not on the circuit")
        depth = water_level(sx, sz) - height(sx, sz)
        if np.nanmax(np.where(np.isfinite(depth), depth, -1.0)) > 0.45:
            issues.append(f"the circuit is {np.nanmax(depth):.2f} m under water (max 0.45 m)")
    return issues


# ---------------------------------------------------------------------------------------------- Blender scene I/O
def _bpy():
    import bpy  # noqa: PLC0415
    return bpy


def collection(name, clear=False):
    bpy = _bpy()
    col = bpy.data.collections.get(name)
    if col is None:
        col = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(col)
    if clear:
        for o in list(col.objects):
            bpy.data.objects.remove(o, do_unlink=True)
    return col


def to_blender(x, z, h=0.0):
    return (float(x), float(-z), float(h))


def surface_material():
    bpy = _bpy()
    mat = bpy.data.materials.get("TF_surface")
    if mat:
        return mat
    mat = bpy.data.materials.new("TF_surface")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    bsdf = next(nd for nd in nodes if nd.type == "BSDF_PRINCIPLED")
    attr = nodes.new("ShaderNodeAttribute")
    attr.attribute_name = "tf_surface"
    mat.node_tree.links.new(attr.outputs[0], bsdf.inputs[0])
    return mat


def build_terrain(heights, classes, origin, cell, base_heights=None, base_classes=None):
    """TERRAIN grid mesh (replaces an existing one)."""
    bpy = _bpy()
    n = heights.shape[0]
    old = bpy.data.objects.get("TERRAIN")
    if old:
        bpy.data.objects.remove(old, do_unlink=True)
    mesh = bpy.data.meshes.new("TERRAIN")
    xs = origin + np.arange(n) * cell
    co = np.empty((n, n, 3), np.float32)
    co[:, :, 0] = xs[None, :]
    co[:, :, 1] = -xs[:, None]
    co[:, :, 2] = heights
    mesh.vertices.add(n * n)
    mesh.vertices.foreach_set("co", co.ravel())
    m = n - 1
    j, i = np.mgrid[0:m, 0:m]
    a = (j * n + i).ravel()
    loops = np.stack([a, a + n, a + n + 1, a + 1], axis=1).ravel()  # a, d, c, b: +Z normals in Blender
    mesh.loops.add(len(loops))
    mesh.loops.foreach_set("vertex_index", loops.astype(np.int32))
    mesh.polygons.add(m * m)
    mesh.polygons.foreach_set("loop_start", (np.arange(m * m) * 4).astype(np.int32))
    mesh.update(calc_edges=True)
    set_surface_colors(mesh, classes)
    bh = mesh.attributes.new("tf_base_height", "FLOAT", "POINT")
    bh.data.foreach_set("value", np.asarray(heights if base_heights is None else base_heights, np.float32).ravel())
    bs = mesh.attributes.new("tf_base_surface", "INT", "FACE")
    bs.data.foreach_set("value", np.asarray(classes if base_classes is None else base_classes, np.int32).ravel())
    mesh.materials.append(surface_material())
    obj = bpy.data.objects.new("TERRAIN", mesh)
    collection("TERRAIN").objects.link(obj)
    return obj


def set_surface_colors(mesh, classes):
    attr = mesh.color_attributes.get("tf_surface") or mesh.color_attributes.new("tf_surface", "BYTE_COLOR", "CORNER")
    rgb = PALETTE[np.asarray(classes, np.int64).ravel()].astype(np.float32) / 255.0
    rgba = np.concatenate([rgb, np.ones((len(rgb), 1), np.float32)], axis=1)
    attr.data.foreach_set("color_srgb", np.repeat(rgba, 4, axis=0).ravel())
    mesh.color_attributes.active_color = attr


def read_terrain(obj, n, origin, cell):
    """(heights n x n float64, classes (n-1) x (n-1) uint8, base heights, base classes) of the TERRAIN mesh."""
    mesh = obj.data
    if len(mesh.vertices) != n * n or len(mesh.polygons) != (n - 1) ** 2:
        raise ValueError(f"TERRAIN must stay a {n} x {n} grid (vertices {len(mesh.vertices)}, faces {len(mesh.polygons)})")
    co = np.empty(n * n * 3, np.float32)
    mesh.vertices.foreach_get("co", co)
    co = co.reshape(n, n, 3)
    xs = origin + np.arange(n) * cell
    if np.abs(co[:, :, 0] - xs[None, :]).max() > 1e-3 or np.abs(co[:, :, 1] + xs[:, None]).max() > 1e-3:
        raise ValueError("TERRAIN vertices moved horizontally: only their height (Blender z) may change")
    attr = mesh.color_attributes.get("tf_surface")
    if attr is None or attr.domain != "CORNER":
        raise ValueError("TERRAIN has no CORNER colour attribute tf_surface")
    col = np.empty(len(attr.data) * 4, np.float32)
    attr.data.foreach_get("color_srgb", col)
    col = col.reshape(-1, 4, 4)[:, :, :3].mean(axis=1) * 255.0
    d = ((col[:, None, :] - PALETTE[None, :, :].astype(np.float32)) ** 2).sum(axis=2)
    classes = d.argmin(axis=1).astype(np.uint8).reshape(n - 1, n - 1)
    out = [co[:, :, 2].astype(np.float64), classes]
    bh = mesh.attributes.get("tf_base_height")
    bs = mesh.attributes.get("tf_base_surface")
    if bh is not None and bs is not None:
        b = np.empty(n * n, np.float32)
        bh.data.foreach_get("value", b)
        c = np.empty((n - 1) ** 2, np.int32)
        bs.data.foreach_get("value", c)
        out += [b.reshape(n, n).astype(np.float64), c.reshape(n - 1, n - 1).astype(np.uint8)]
    else:
        out += [None, None]
    return out


def write_terrain_heights(obj, heights, classes):
    mesh = obj.data
    n = heights.shape[0]
    co = np.empty(n * n * 3, np.float32)
    mesh.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    co[:, 2] = np.asarray(heights, np.float32).ravel()
    mesh.vertices.foreach_set("co", co.ravel())
    set_surface_colors(mesh, classes)
    mesh.update()


def build_curve(name, rows, closed, kind, width, segment_surface, markings=None, aligned=True):
    """Bezier curve in TRACKS from rows [x, z, h, lx, lz, lh, rx, rz, rh, tilt] (game coordinates). aligned: ALIGNED
    handles (Blender then moves one handle of each point onto the line of the other); False keeps the rows exactly
    (FREE handles, the importer: a re-export gives the same file)."""
    bpy = _bpy()
    old = bpy.data.objects.get(name)
    if old:
        bpy.data.objects.remove(old, do_unlink=True)
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.twist_mode = "Z_UP"
    cu.extrude = 0.5 * width
    sp = cu.splines.new("BEZIER")
    sp.bezier_points.add(len(rows) - 1)
    for bp, r in zip(sp.bezier_points, rows):
        bp.handle_left_type = bp.handle_right_type = "FREE"
        bp.co = to_blender(r[0], r[1], r[2])
        bp.handle_left = to_blender(r[3], r[4], r[5])
        bp.handle_right = to_blender(r[6], r[7], r[8])
        bp.tilt = float(r[9])
        bp.radius = 1.0
    for bp in sp.bezier_points:
        if aligned:
            bp.handle_left_type = bp.handle_right_type = "ALIGNED"
    sp.use_cyclic_u = closed
    obj = bpy.data.objects.new(name, cu)
    obj["tf_kind"] = kind
    obj["tf_width"] = float(width)
    obj["tf_segment_surface"] = list(segment_surface)
    if markings:
        obj["tf_markings"] = json.dumps(markings, sort_keys=True)
    obj.display_type = "WIRE"
    collection("TRACKS").objects.link(obj)
    return obj


def read_curve(obj):
    """(rows rounded like the export, closed, kind, width, segment surfaces, name, markings) of a TRACKS curve.
    markings: the tf_markings JSON string (edge lines, start line, grid, curbs; game layout road "markings") or None."""
    sp = obj.data.splines[0]
    if sp.type != "BEZIER":
        raise ValueError(f"{obj.name}: the track must be one Bezier spline")
    mw = obj.matrix_world
    rows = []
    for bp in sp.bezier_points:
        c, l, r = mw @ bp.co, mw @ bp.handle_left, mw @ bp.handle_right
        rows.append([c.x, -c.y, c.z, l.x, -l.y, l.z, r.x, -r.y, r.z, bp.tilt])
    surf = list(obj.get("tf_segment_surface", []))
    if len(surf) != len(rows):
        # Points added in Blender: each new point takes the surface of the nearest original control point.
        surf = (surf or ["dirt"]) * len(rows)
        surf = surf[:len(rows)]
    for s in surf:
        if s not in CLASSES:
            raise ValueError(f"{obj.name}: tf_segment_surface '{s}' is not a surface class {CLASSES}")
    name = obj.name[len("TRACK_"):] if obj.name.startswith("TRACK_") else obj.name
    mk = obj.get("tf_markings")
    markings = json.loads(mk) if mk else None
    return (round_bezier(rows), bool(sp.use_cyclic_u), str(obj.get("tf_kind", "road")),
            float(obj.get("tf_width", 8.0)), surf, name, markings)


def empty(name, col, x, z, h, yaw, display="PLAIN_AXES", size=1.0, scale=None):
    bpy = _bpy()
    o = bpy.data.objects.new(name, None)
    o.empty_display_type = display
    o.empty_display_size = size
    o.location = to_blender(x, z, h)
    o.rotation_euler = (0.0, 0.0, float(yaw))
    if scale:
        o.scale = scale
    col.objects.link(o)
    return o


def yaw_of(obj):
    """Game yaw of an empty: its rotation about Blender z (placements and markers are not parented)."""
    return obj.rotation_euler.z


def game_xz(obj):
    return obj.location.x, -obj.location.y


# ------------------------------------------------------------------------------------------------ export / import
def _placement_from_object(o):
    kind = str(o.get("tf_kind", ""))
    x, z = game_xz(o)
    p = {"kind": kind}
    if kind in ("tree", "rock"):
        p["model"] = str(o.get("tf_ref", ""))
    elif kind == "building":
        p["type"] = str(o.get("tf_ref", ""))
    else:
        raise ValueError(f"{o.name}: tf_kind must be tree, rock or building")
    p["x"], p["z"], p["yaw"] = r3(x), r3(z), r4(yaw_of(o))
    if kind == "tree":
        p["scale"] = r3(o.get("tf_scale", 1.0))
        p["width"] = r3(o.get("tf_width", 1.0))
    elif kind == "rock":
        p["scale"] = [r3(v) for v in o.get("tf_scale_xyz", (1.0, 1.0, 1.0))]
        p["sink"] = r3(o.get("tf_sink", 0.3))
    else:
        for key in ("w", "d", "h"):
            p[key] = r3(o.get("tf_" + key, 0.0))
        p["floors"] = int(o.get("tf_floors", 1))
        p["variant"] = int(o.get("tf_variant", 0))
    return p


def scene_layout():
    """(layout dict, heights n x n, classes) from the open scene."""
    bpy = _bpy()
    sc = bpy.context.scene
    n, origin, cell = int(sc["tf_samples"]), float(sc["tf_origin"]), float(sc.get("tf_cell", 1.0))
    terrain = bpy.data.objects.get("TERRAIN")
    if terrain is None:
        raise ValueError("no TERRAIN object")
    heights, classes, _, _ = read_terrain(terrain, n, origin, cell)
    lo, hi = float(sc.get("tf_height_min", 0.0)), float(sc.get("tf_height_max", 0.0))
    if not (hi > lo and heights.min() >= lo and heights.max() <= hi):
        lo, hi = math.floor(heights.min()) - 1.0, math.ceil(heights.max()) + 1.0
        sc["tf_height_min"], sc["tf_height_max"] = lo, hi
    layout = {"format": 1, "grid": {"samples": n, "cell": cell, "origin": origin, "heightMin": lo, "heightMax": hi}}
    roads = []
    col = bpy.data.collections.get("TRACKS")
    for o in sorted(col.objects if col else [], key=lambda o: o.name):
        if o.type != "CURVE":
            continue
        rows, closed, kind, width, surf, name, markings = read_curve(o)
        s = sample_bezier(rows, closed, SAMPLE_SPACING)
        samples = [[r3(x), r3(z), r4(b)] for x, z, b in zip(s["x"], s["z"], s["bank"])]
        road = {"name": name, "kind": kind, "width": r3(width), "closed": closed, "samples": samples,
                "bezier": rows, "surfaces": surf}
        if markings:
            road["markings"] = markings
        roads.append(road)
    layout["roads"] = roads
    water = []
    col = bpy.data.collections.get("WATER")
    for o in sorted(col.objects if col else [], key=lambda o: o.name):
        if o.type != "MESH" or not o.data.polygons:
            continue
        mw = o.matrix_world
        poly = o.data.polygons[0]
        pts = [mw @ o.data.vertices[v].co for v in poly.vertices]
        name = o.name[len("WATER_"):] if o.name.startswith("WATER_") else o.name
        water.append({"name": name, "level": r3(sum(p.z for p in pts) / len(pts)),
                      "polygon": [[r3(p.x), r3(-p.y)] for p in pts]})
    layout["water"] = water
    col = bpy.data.collections.get("PLACEMENTS")
    layout["placements"] = [_placement_from_object(o) for o in sorted(col.objects if col else [], key=lambda o: o.name)
                            if o.type == "EMPTY"]
    spawn = bpy.data.objects.get("SPAWN")
    if spawn is None:
        raise ValueError("no SPAWN empty")
    x, z = game_xz(spawn)
    layout["spawn"] = {"x": r3(x), "z": r3(z), "yaw": r4(yaw_of(spawn))}
    cps, pois = [], []
    col = bpy.data.collections.get("MARKERS")
    for o in sorted(col.objects if col else [], key=lambda o: o.name):
        x, z = game_xz(o)
        if o.name.startswith("CP_"):
            cp = {"x": r3(x), "z": r3(z), "yaw": r4(yaw_of(o)), "halfWidth": r3(o.get("tf_half_width", 8.0))}
            if o.get("tf_sector", False):
                cp["sector"] = True
            cps.append(cp)
        elif o.name.startswith("POI_"):
            at = o.get("tf_at", (x, z))
            pois.append({"id": int(o.get("tf_id", len(pois) + 1)), "name": str(o.get("tf_name", o.name[4:])),
                         "kind": str(o.get("tf_poi_kind", "")), "x": r3(x), "z": r3(z), "at": [r3(at[0]), r3(at[1])]})
    layout["checkpoints"] = cps
    layout["pois"] = pois
    return layout, heights, classes


def write_level_files(out_dir, layout, heights, classes):
    os.makedirs(out_dir, exist_ok=True)
    g = layout["grid"]
    lo, hi = g["heightMin"], g["heightMax"]
    q = np.clip(np.round((np.asarray(heights, np.float64) - lo) / (hi - lo) * 65535.0), 0, 65535).astype(np.uint16)
    write_pgm(os.path.join(out_dir, "heightmap.pgm"), q)
    write_pgm(os.path.join(out_dir, "surfaces.pgm"), np.asarray(classes, np.uint8))
    with open(os.path.join(out_dir, "layout.json"), "w", newline="\n") as f:
        f.write(dump_json(layout) + "\n")


def read_level_files(level_dir):
    layout = json.load(open(os.path.join(level_dir, "layout.json")))
    g = layout["grid"]
    q = read_pgm(os.path.join(level_dir, "heightmap.pgm"))
    heights = g["heightMin"] + q.astype(np.float64) * ((g["heightMax"] - g["heightMin"]) / 65535.0)
    classes = read_pgm(os.path.join(level_dir, "surfaces.pgm"))
    return layout, heights, classes


def clear_scene():
    bpy = _bpy()
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    for c in list(bpy.data.collections):
        bpy.data.collections.remove(c)
    for block in (bpy.data.meshes, bpy.data.curves, bpy.data.cameras, bpy.data.lights):
        for d in list(block):
            block.remove(d)


def placement_object(name, p, h):
    col = collection("PLACEMENTS")
    kind = p["kind"]
    if kind == "tree":
        o = empty(name, col, p["x"], p["z"], h, p["yaw"], "SINGLE_ARROW", 12.0 * p["scale"])
        o["tf_ref"], o["tf_scale"], o["tf_width"] = p["model"], p["scale"], p["width"]
    elif kind == "rock":
        o = empty(name, col, p["x"], p["z"], h, p["yaw"], "SPHERE", 1.0, tuple(p["scale"]))
        o["tf_ref"], o["tf_scale_xyz"], o["tf_sink"] = p["model"], list(p["scale"]), p["sink"]
    else:
        w, d, hh = max(p["w"], 0.5), max(p["d"], 0.5), max(p["h"], 1.0)
        o = empty(name, col, p["x"], p["z"], h, p["yaw"], "CUBE", 1.0, (0.5 * w, 0.5 * d, 0.5 * hh))
        o["tf_ref"] = p["type"]
        for key in ("w", "d", "h"):
            o["tf_" + key] = p[key]
        o["tf_floors"], o["tf_variant"] = p["floors"], p["variant"]
    o["tf_kind"] = kind
    return o


def build_scene(layout, heights, classes, level_id, base_heights=None, base_classes=None):
    """The whole level scene from game data (replaces everything in the open file)."""
    bpy = _bpy()
    clear_scene()
    g = layout["grid"]
    n, origin, cell = g["samples"], g["origin"], g["cell"]
    sc = bpy.context.scene
    sc["tf_level"] = level_id
    sc["tf_samples"], sc["tf_origin"], sc["tf_cell"] = n, origin, cell
    sc["tf_height_min"], sc["tf_height_max"] = g["heightMin"], g["heightMax"]
    sc.unit_settings.system = "METRIC"
    build_terrain(np.asarray(heights, np.float32), classes, origin, cell, base_heights, base_classes)
    for r in layout["roads"]:
        build_curve("TRACK_" + r["name"], r["bezier"], r["closed"], r["kind"], r["width"], r["surfaces"],
                    r.get("markings"), aligned=False)
    wcol = collection("WATER")
    for w in layout["water"]:
        mesh = bpy.data.meshes.new("WATER_" + w["name"])
        mesh.from_pydata([to_blender(x, z, w["level"]) for x, z in w["polygon"]], [],
                         [list(range(len(w["polygon"])))])
        o = bpy.data.objects.new("WATER_" + w["name"], mesh)
        o.display_type = "WIRE"
        wcol.objects.link(o)

    def ground(x, z):
        i = min(max(int(round(x - origin)), 0), n - 1)
        j = min(max(int(round(z - origin)), 0), n - 1)
        return float(heights[j][i])

    for k, p in enumerate(layout["placements"]):
        placement_object(f"{p['kind']}_{k:05d}", p, ground(p["x"], p["z"]))
    mcol = collection("MARKERS")
    sp = layout["spawn"]
    empty("SPAWN", mcol, sp["x"], sp["z"], ground(sp["x"], sp["z"]) + 1.0, sp["yaw"], "ARROWS", 4.0)
    for k, cp in enumerate(layout["checkpoints"]):
        o = empty(f"CP_{k:02d}", mcol, cp["x"], cp["z"], ground(cp["x"], cp["z"]), cp["yaw"], "CUBE", 1.0,
                  (cp["halfWidth"], 0.2, 2.0))
        o["tf_half_width"] = cp["halfWidth"]
        if cp.get("sector"):
            o["tf_sector"] = True
    for p in layout["pois"]:
        o = empty(f"POI_{p['id']:02d}", mcol, p["x"], p["z"], ground(p["x"], p["z"]) + 2.0, 0.0, "SPHERE", 3.0)
        o["tf_id"], o["tf_name"], o["tf_poi_kind"], o["tf_at"] = p["id"], p["name"], p["kind"], list(p["at"])


def carve_scene():
    """Re-carves the TERRAIN from the TRACKS curves over tf_base_height / tf_base_surface (carve_track.py)."""
    bpy = _bpy()
    sc = bpy.context.scene
    n, origin, cell = int(sc["tf_samples"]), float(sc["tf_origin"]), float(sc.get("tf_cell", 1.0))
    terrain = bpy.data.objects["TERRAIN"]
    _, _, base_h, base_c = read_terrain(terrain, n, origin, cell)
    if base_h is None:
        raise ValueError("TERRAIN has no tf_base_height / tf_base_surface attributes")
    roads = []
    for o in sorted(bpy.data.collections["TRACKS"].objects, key=lambda o: o.name):
        if o.type != "CURVE":
            continue
        rows, closed, kind, width, surf, _, _ = read_curve(o)
        roads.append(road_for_carve(rows, closed, width, surf))
        roads[-1]["priority"] = kind == "circuit"
    h, c, worst = carve(base_h, base_c, roads, origin, cell)
    write_terrain_heights(terrain, h, c)
    return h, c, worst
