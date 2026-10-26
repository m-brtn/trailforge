"""Builds a starting .blend for a Blender-authored vehicle body (README "Vehicle data format", body.glb).

Run headless:
  /Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup \
      --python tools/cars/blender/make_body_blend.py -- --vehicle trailforge_suv --out /tmp/trailforge_suv_body.blend \
      [--export /tmp/body_src.glb]

Scene (vehicle-local game axes mapped to Blender: game (x, y, z) = Blender (x, -z, y); the vehicle faces -Y, its
left is +X, origin = the vehicle body origin the physics wheels are placed from):
  REF_references  Cars.png reference planes (side, top, front, rear) at blueprint scale; not exported
  PROXY_wheels    physics-size wheels at rest, and wire ghosts at full compression; not exported
  CUT_arches      wheelhouse cutters per wheel: stadium of radius (tire reach over the steering lock + 25 mm)
                  swept from full droop to full compression (rest length - minLength from vehicle.json, at the lowest
                  garage ride height), from the tub wall outward; Boolean DIFFERENCE on the body panels; not exported
  BODY            the parametric base body: named parts, one material slot per contract role, SPARE_rear seat
  COL             convex collision pieces COL_*: the base body split along the wheel-sweep boundaries (tub between
                  the wheels, overhangs, sides between the arches, the band above the arches, the cabin), the convex
                  hull of each cell
Export with tools/cars/blender/export_body.py (validates and converts with trailforge_vehiclec).
"""

import argparse
import math
import os
import sys

import bmesh
import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tf_vehicle  # noqa: E402
from tf_vehicle import blender as B  # noqa: E402


def srgb(r, g, b):
    return (r ** 2.2, g ** 2.2, b ** 2.2)


# Contract roles -> Principled settings (linear colours). Colours follow the SUV livery (src/render/StyleKit.cpp).
MATERIALS = {
    "paint_primary": dict(color=srgb(0.45, 0.50, 0.32), rough=0.32, coat=1.0),
    "paint_secondary": dict(color=srgb(0.90, 0.90, 0.87), rough=0.36, coat=1.0),
    "paint_accent": dict(color=srgb(0.96, 0.50, 0.13), rough=0.34, coat=1.0),
    "cladding": dict(color=(0.17 ** 2.2,) * 3, rough=0.78),
    "plastic": dict(color=(0.06,) * 3, rough=0.6),
    "chrome": dict(color=(0.92,) * 3, rough=0.05, metal=1.0),
    "metal": dict(color=(0.16 ** 2.2,) * 3, rough=0.48, metal=0.35),
    "glass": dict(color=(0.012, 0.016, 0.020), rough=0.04, transmission=1.0, ior=1.5),
    "rubber": dict(color=(0.032, 0.032, 0.034), rough=0.85),
    "lens": dict(color=(0.10 ** 2.2,) * 3, rough=0.12, metal=0.6),
    "light_head": dict(color=(1.0, 0.95, 0.88), rough=0.2, emission=(1.0, 0.95, 0.88)),
    "light_tail": dict(color=(1.0, 0.04, 0.02), rough=0.2, emission=(1.0, 0.04, 0.02)),
    "light_brake": dict(color=(1.0, 0.04, 0.02), rough=0.2, emission=(1.0, 0.04, 0.02)),
    "light_reverse": dict(color=(0.95, 0.95, 0.95), rough=0.2, emission=(0.95, 0.95, 0.95)),
    "light_indicator": dict(color=(1.0, 0.45, 0.02), rough=0.2, emission=(1.0, 0.45, 0.02)),
    "liner": dict(color=(0.12 ** 2.2,) * 3, rough=0.92),
    "underbody": dict(color=(0.20 ** 2.2,) * 3, rough=0.88),
}


def make_materials():
    mats = {}
    for name, p in MATERIALS.items():
        m = bpy.data.materials.new(name)
        m.use_nodes = True
        b = m.node_tree.nodes["Principled BSDF"]
        b.inputs["Base Color"].default_value = (*p["color"], 1.0)
        b.inputs["Roughness"].default_value = p.get("rough", 0.5)
        b.inputs["Metallic"].default_value = p.get("metal", 0.0)
        b.inputs["Coat Weight"].default_value = p.get("coat", 0.0)
        b.inputs["Coat Roughness"].default_value = 0.05
        b.inputs["Transmission Weight"].default_value = p.get("transmission", 0.0)
        b.inputs["IOR"].default_value = p.get("ior", 1.5)
        if "emission" in p:
            b.inputs["Emission Color"].default_value = (*p["emission"], 1.0)
            b.inputs["Emission Strength"].default_value = 1.0
        m["tf_role"] = name  # exported as material extras; the converter also accepts the name itself
        m.diffuse_color = (*p["color"], 1.0)
        mats[name] = m
    return mats


def collection(name, parent=None):
    c = bpy.data.collections.new(name)
    (parent or bpy.context.scene.collection).children.link(c)
    return c


def mesh_object(name, bm, col, mat, parent=None, smooth_angle=None):
    me = bpy.data.meshes.new(name)
    bm.normal_update()
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    col.objects.link(o)
    if mat is not None:
        me.materials.append(mat)
    if smooth_angle is not None:
        me.shade_smooth()
        me.set_sharp_from_angle(angle=math.radians(smooth_angle))
    if parent is not None:
        o.parent = parent
    return o


def prism(profile, half_width, x_center=0.0, taper=None):
    """Side profile [(z, y), ...] (game frame, any winding) extruded across x = x_center +- half width; taper(y)
    returns the half width at height y (tumblehome) when given."""
    bm = bmesh.new()
    hw = taper or (lambda y: half_width)
    left = [bm.verts.new(B(x_center + hw(y), y, z)) for z, y in profile]
    right = [bm.verts.new(B(x_center - hw(y), y, z)) for z, y in profile]
    bm.faces.new(left)
    bm.faces.new(list(reversed(right)))
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((left[j], left[i], right[i], right[j]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


def box(center, half):
    """Axis-aligned box in the game frame."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts:
        g = (center[0] + v.co.x * 2 * half[0], center[1] + v.co.y * 2 * half[1], center[2] + v.co.z * 2 * half[2])
        v.co = Vector(B(*g))
    return bm


def hull(points):
    """Convex hull of game-frame points."""
    bm = bmesh.new()
    for p in points:
        bm.verts.new(B(*p))
    bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=0.002)
    res = bmesh.ops.convex_hull(bm, input=list(bm.verts))
    drop = {g for g in list(res["geom_interior"]) + list(res["geom_unused"]) if isinstance(g, bmesh.types.BMVert)}
    bmesh.ops.delete(bm, geom=list(drop), context="VERTS")
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


def bevel(o, width, segments=2, angle=35.0):
    m = o.modifiers.new("bevel", "BEVEL")
    m.width = width
    m.segments = segments
    m.limit_method = "ANGLE"
    m.angle_limit = math.radians(angle)
    m.harden_normals = True
    return m


def cut_arches(o, cutters):
    for c in cutters:
        b = o.modifiers.new(f"cut_{c.name}", "BOOLEAN")
        b.object = c
        b.operation = "DIFFERENCE"
        b.solver = "EXACT"
        b.material_mode = "TRANSFER"
    # booleans before any bevel
    for i, m in enumerate([m.name for m in o.modifiers if m.name.startswith("cut_")]):
        with bpy.context.temp_override(object=o):
            bpy.ops.object.modifier_move_to_index(modifier=m, index=i)


# ------------------------------------------------------------------------------------------------ scene parts

def references(v, col):
    """Cars.png cells as image planes at blueprint scale (concept frame mapped like BlueprintBody)."""
    bp = v.blueprint
    sheet_path = os.path.join(tf_vehicle.REPO, "tools", "cars", "sheet.json")
    import json
    sheet = json.load(open(sheet_path))
    image_path = os.path.join(tf_vehicle.REPO, sheet["image"])
    if not bp or not os.path.exists(image_path):
        print("TF: no blueprint or Cars.png, skipping the reference planes")
        return
    img = bpy.data.images.load(image_path)
    W, H = img.size
    column = sheet["columns"][bp["source"]["column"]]
    hub = v.rest_hub
    ground = min(w.hub_rest - w.radius for w in v.wheels) - 0.005

    def plane(view, to_game):
        g = bp[view]["grid"]
        x0, y0 = column[0], sheet["rows"][view][0]
        # columns at the cell edges and, for views along z, at the concept axles (piecewise like bodyZ)
        js = [0.0, float(g["width"])]
        if view in ("side", "top"):
            for zc in (v.concept_axle_z, -v.concept_axle_z):
                j = (zc - g["u0"]) / g["du"]
                if 0 < j < g["width"]:
                    js.append(j)
        js.sort()
        bm = bmesh.new()
        uv_layer = bm.loops.layers.uv.new()
        rows = [0.0, float(g["height"])]
        grid = [[None] * len(js) for _ in rows]
        for a, i in enumerate(rows):
            for b, j in enumerate(js):
                u, vv = g["u0"] + g["du"] * j, g["v0"] + g["dv"] * i
                grid[a][b] = (bm.verts.new(B(*to_game(u, vv))), ((x0 + j) / W, 1.0 - (y0 + i) / H))
        for b in range(len(js) - 1):
            quad = [grid[0][b], grid[0][b + 1], grid[1][b + 1], grid[1][b]]
            f = bm.faces.new([q[0] for q in quad])
            for loop, q in zip(f.loops, quad):
                loop[uv_layer].uv = q[1]
        mat = bpy.data.materials.new(f"REF_{view}")
        mat.use_nodes = True
        nt = mat.node_tree
        nt.nodes.remove(nt.nodes["Principled BSDF"])
        tex = nt.nodes.new("ShaderNodeTexImage")
        tex.image = img
        em = nt.nodes.new("ShaderNodeEmission")
        nt.links.new(tex.outputs["Color"], em.inputs["Color"])
        nt.links.new(em.outputs["Emission"], nt.nodes["Material Output"].inputs["Surface"])
        o = mesh_object(f"REF_{view}", bm, col, mat)
        o.hide_select = True
        return o

    plane("side", lambda u, vv: (-1.7, vv + hub, v.body_z(u)))
    plane("top", lambda u, vv: (vv, ground - 0.3, v.body_z(u)))
    plane("front", lambda u, vv: (u, vv + hub, v.body_z(bp["side"]["extent"]["zFront"]) + 1.5))
    plane("rear", lambda u, vv: (u, vv + hub, v.body_z(bp["side"]["extent"]["zRear"]) - 1.5))


def proxy_wheels(v, col, mats):
    for w in v.wheels:
        for tag, y, wire in (("rest", w.hub_rest, False), ("bump", w.hub_high, True)):
            bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=w.radius, depth=w.width, location=B(w.x, y, w.z),
                                                rotation=(0.0, math.pi / 2, 0.0))
            o = bpy.context.active_object
            o.name = f"PROXY_wheel_{w.tag}_{tag}"
            for c in o.users_collection:
                c.objects.unlink(o)
            col.objects.link(o)
            o.data.materials.append(mats["rubber"])
            o.hide_select = True
            if wire:
                o.display_type = "WIRE"
                o.hide_render = True


def arch_cutters(v, col, mats):
    """Per wheel: stadium (arch radius around the hub line from full droop to full compression) extruded from the tub
    wall (inner_x) far outward."""
    cutters = []
    x_in = v.inner_x
    for w in v.wheels:
        R = w.arch_radius
        sx = 1.0 if w.x > 0 else -1.0
        prof = []
        n = 24
        for k in range(n + 1):  # top half circle around the highest hub
            t = math.pi * k / n
            prof.append((w.z + R * math.cos(t), w.hub_high + R * math.sin(t)))
        prof.append((w.z - R, w.hub_low - R - 0.6))
        prof.append((w.z + R, w.hub_low - R - 0.6))
        x0, x1 = x_in, abs(w.x) + 1.0
        bm = prism(prof, 0.5 * (x1 - x0), sx * 0.5 * (x0 + x1))
        o = mesh_object(f"CUT_arch_{w.tag}", bm, col, mats["liner"])
        o.display_type = "WIRE"
        o.hide_render = True
        o.hide_select = True
        cutters.append(o)
    return cutters


class Body:
    """Parametric base body from the blueprint landmarks (game frame, m)."""

    def __init__(self, v):
        bp = v.blueprint
        ext = bp["side"]["extent"]
        self.zF = v.body_z(ext["zFront"]) - 0.02
        self.zR = v.body_z(ext["zRear"]) + 0.02
        self.roof = v.body_y(ext["yTop"]) - 0.02
        self.half = bp["report"]["top"]["maxPaintedHalfWidth"] - 0.01
        fr = [w for w in v.wheels if w.index < 2][0]
        rr = [w for w in v.wheels if w.index >= 2][0]
        self.front_wheel, self.rear_wheel = fr, rr
        arch_top = max(w.arch_top for w in v.wheels)
        self.deck = max(v.body_y(0.81), arch_top + 0.06)      # belt / hood deck over the arches
        self.sill = v.body_y(0.07)                             # sill between the wheels
        self.floor = self.sill - 0.04
        self.cowl = v.body_z(0.84)                             # windshield base
        self.roof_front = v.body_z(0.21)
        self.roof_rear = v.body_z(-1.60)
        self.back = v.body_z(-1.95)
        self.door_cuts = [v.body_z(z) for z in bp["side"]["doorCuts"]]
        self.belt_half = self.half - 0.07
        self.roof_half = self.half - 0.30
        self.nose = v.body_y(0.64)


def build_body(v, col, mats, cutters):
    k = Body(v)
    parts = {}
    sm = 40.0
    # ---- lower body: nose, hood deck, belt deck, rear face; approach / departure chamfers
    lower = [
        (k.zR, k.floor + 0.14), (k.zR + 0.25, k.floor), (k.zF - 0.30, k.floor), (k.zF, k.floor + 0.16),
        (k.zF, k.nose), (k.zF - 0.12, k.deck - 0.05), (k.cowl, k.deck), (k.zR + 0.02, k.deck + 0.01),
        (k.zR, k.deck - 0.02),
    ]
    body = mesh_object("body", prism(lower, k.half), col, mats["paint_primary"], smooth_angle=sm)
    cut_arches(body, cutters)
    bevel(body, 0.03)
    parts["body"] = body
    # ---- hood panel, slightly proud of the deck
    hood_prof = [(k.cowl - 0.02, k.deck + 0.004), (k.zF - 0.14, k.deck - 0.046), (k.zF - 0.14, k.deck - 0.03),
                 (k.cowl - 0.02, k.deck + 0.02)]
    hood = mesh_object("hood", prism(hood_prof, k.half - 0.16), col, mats["paint_primary"], parent=body, smooth_angle=sm)
    bevel(hood, 0.008)
    # ---- greenhouse: the glass volume, painted pillars, white roof
    def gh(y):
        t = (y - k.deck) / max(k.roof - k.deck, 1e-6)
        return k.belt_half + (k.roof_half - k.belt_half) * t
    cab = [(k.cowl, k.deck), (k.roof_front, k.roof - 0.03), (k.roof_rear, k.roof - 0.03), (k.back, k.deck + 0.22),
           (k.back - 0.02, k.deck)]
    glass = mesh_object("glass", prism(cab, 0.0, taper=gh), col, mats["glass"], parent=body, smooth_angle=sm)
    bevel(glass, 0.02)
    roof_prof = [(k.roof_front - 0.03, k.roof - 0.04), (k.roof_rear + 0.02, k.roof - 0.04), (k.roof_rear + 0.04, k.roof),
                 (k.roof_front - 0.06, k.roof)]
    roof = mesh_object("roof", prism(roof_prof, k.roof_half + 0.015), col, mats["paint_secondary"], parent=body,
                       smooth_angle=sm)
    bevel(roof, 0.012)

    def pillar(name, z_low, z_high, width):
        pts = []
        for z, y in ((z_low, k.deck), (z_high, k.roof - 0.035)):
            for sx in (1, -1):
                for dz in (-width, width):
                    for dx in (0.0, 0.012):
                        pts.append((sx * (gh(y) + dx), y, z + dz))
        return mesh_object(name, hull(pts), col, mats["paint_primary"], parent=body, smooth_angle=sm)
    pillar("pillar_a", k.cowl - 0.02, k.roof_front + 0.02, 0.035)
    pillar("pillar_b", k.door_cuts[1], k.door_cuts[1], 0.04)
    pillar("pillar_c", k.door_cuts[2] - 0.05, k.door_cuts[2] - 0.05, 0.045)
    pillar("pillar_d", k.back + 0.03, k.roof_rear - 0.04, 0.05)
    # ---- doors: skins proud of the body side between the traced shut lines, sill to belt
    cuts = k.door_cuts  # front to rear
    for sx, side in ((1, "left"), (-1, "right")):
        for name, z0, z1 in (("door_front", cuts[1] + 0.006, cuts[0] - 0.006), ("door_rear", cuts[2] + 0.006, cuts[1] - 0.006)):
            prof = [(z0, k.sill + 0.06), (z1, k.sill + 0.06), (z1, k.deck - 0.03), (z0, k.deck - 0.03)]
            d = mesh_object(f"{name}_{side}", prism(prof, 0.006, sx * (k.half + 0.004)), col, mats["paint_primary"],
                            parent=body, smooth_angle=sm)
            cut_arches(d, cutters)
            bevel(d, 0.003, 1)
        # side stripe in the accent colour under the belt
        stripe = [(k.zR + 0.05, k.deck - 0.10), (k.zF - 0.20, k.deck - 0.13), (k.zF - 0.20, k.deck - 0.11),
                  (k.zR + 0.05, k.deck - 0.08)]
        s = mesh_object(f"stripe_{side}", prism(stripe, 0.004, sx * (k.half + 0.012)), col, mats["paint_accent"], parent=body)
        cut_arches(s, cutters)
        # lower cladding between the arches and around the sills
        fw, rw = k.front_wheel, k.rear_wheel
        clad = [(rw.z + rw.arch_radius + 0.01, k.floor + 0.02), (fw.z - fw.arch_radius - 0.01, k.floor + 0.02),
                (fw.z - fw.arch_radius - 0.01, k.sill + 0.05), (rw.z + rw.arch_radius + 0.01, k.sill + 0.05)]
        c = mesh_object(f"cladding_{side}", prism(clad, 0.02, sx * (k.half + 0.01)), col, mats["cladding"], parent=body,
                        smooth_angle=sm)
        bevel(c, 0.008)
        # fender flares: a band around each arch opening, outside the cutter, out to the tire face + margin
        for w in (fw, rw) if sx > 0 else (v.wheels[1], v.wheels[3]):
            R, band = w.arch_radius + 0.004, 0.065
            outer = []
            inner = []
            n = 20
            for i in range(n + 1):
                t = math.pi * i / n
                outer.append((w.z + (R + band) * math.cos(t), w.hub_high + (R + band) * math.sin(t)))
                inner.append((w.z + R * math.cos(t), w.hub_high + R * math.sin(t)))
            foot = k.floor + 0.04
            prof = [(w.z + R + band, foot)] + outer + [(w.z - R - band, foot), (w.z - R, foot)] + list(reversed(inner)) + [(w.z + R, foot)]
            x0, x1 = k.half - 0.05, w.outer_x
            f = mesh_object(f"flare_{'front' if w.index < 2 else 'rear'}_{side}", prism(prof, 0.5 * (x1 - x0), sx * 0.5 * (x0 + x1)),
                            col, mats["cladding"], parent=body, smooth_angle=sm)
            bevel(f, 0.01)
        # mirror on the door, behind the A pillar
        mz = k.cowl - 0.12
        my = k.deck + 0.14
        mx = sx * (k.belt_half + 0.13)
        m = mesh_object(f"mirror_{side}", box((mx, my, mz), (0.10, 0.07, 0.035)), col, mats["plastic"],
                        parent=parts["body"], smooth_angle=sm)
        bevel(m, 0.015)
        arm = mesh_object(f"mirror_arm_{side}", box((sx * (k.belt_half + 0.03), my - 0.03, mz), (0.04, 0.015, 0.015)),
                          col, mats["plastic"], parent=m)
        mirror_glass = mesh_object(f"mirror_glass_{side}", box((mx, my, mz - 0.037), (0.085, 0.055, 0.003)), col,
                                   mats["chrome"], parent=m)
    # ---- front: bumper, grille, lamps, indicators, skid plate
    fb_z = k.zF + 0.02
    bumper_f = mesh_object("bumper_f", box((0.0, k.floor + 0.12, fb_z - 0.08), (k.half - 0.04, 0.11, 0.10)), col,
                           mats["metal"], parent=body, smooth_angle=sm)
    bevel(bumper_f, 0.02)
    mesh_object("grille", box((0.0, k.nose - 0.14, k.zF + 0.004), (0.38, 0.10, 0.006)), col, mats["plastic"], parent=body)
    for l in v.blueprint["lights"]:
        xs = sorted((l["box"][0], l["box"][2]))
        ys = sorted((v.body_y(l["box"][1]), v.body_y(l["box"][3])))
        side = "left" if xs[0] + xs[1] > 0 else "right"
        c = (0.5 * (xs[0] + xs[1]), 0.5 * (ys[0] + ys[1]))
        h = (0.5 * (xs[1] - xs[0]), 0.5 * (ys[1] - ys[0]))
        if l["view"] == "front" and l["kind"] == "head":
            mesh_object(f"lights_head_{side}", box((c[0], c[1], k.zF + 0.008), (h[0], h[1], 0.01)), col,
                        mats["light_head"], parent=body)
            sx = 1 if side == "left" else -1
            mesh_object(f"lights_indicator_{side}", box((sx * (k.half - 0.03), c[1], k.zF - 0.04), (0.012, 0.035, 0.04)),
                        col, mats["light_indicator"], parent=body)
        elif l["view"] == "rear" and l["kind"] in ("tail", "brake"):
            mesh_object(f"lights_tail_{side}", box((c[0], c[1], k.zR - 0.008), (h[0], h[1], 0.01)), col,
                        mats["light_tail"], parent=body)
    skid_z0 = k.front_wheel.z + k.front_wheel.arch_radius + 0.05
    skid = mesh_object("skid_plate", box((0.0, k.floor + 0.01, 0.5 * (skid_z0 + k.zF - 0.1)), (0.45, 0.012, 0.5 * (k.zF - 0.1 - skid_z0))),
                       col, mats["metal"], parent=bumper_f)
    # ---- rear: bumper, reverse lamps, third brake light, spare carrier and seat
    bumper_r = mesh_object("bumper_r", box((0.0, k.floor + 0.12, k.zR + 0.06), (k.half - 0.04, 0.11, 0.09)), col,
                           mats["metal"], parent=body, smooth_angle=sm)
    bevel(bumper_r, 0.02)
    for sx, side in ((1, "left"), (-1, "right")):
        mesh_object(f"lights_reverse_{side}", box((sx * 0.55, k.floor + 0.16, k.zR - 0.035), (0.05, 0.02, 0.008)), col,
                    mats["light_reverse"], parent=bumper_r)
    mesh_object("lights_brake", box((0.0, k.roof - 0.06, k.back - 0.01), (0.18, 0.015, 0.01)), col, mats["light_brake"],
                parent=body)
    rr = k.rear_wheel.radius
    spare_y = k.floor + 0.12 + 0.11 + rr + 0.03
    seat_z = k.zR - 0.07
    carrier = mesh_object("spare_carrier", box((0.0, spare_y, 0.5 * (k.zR + seat_z)), (0.12, 0.12, 0.5 * (k.zR - seat_z) + 0.01)),
                          col, mats["metal"], parent=body)
    spare = bpy.data.objects.new("SPARE_rear", None)
    spare.empty_display_type = "SINGLE_ARROW"
    spare.empty_display_size = 0.4
    spare.location = B(0.0, spare_y, seat_z)
    # the arrow (local +Z) points backwards (game -Z = Blender +Y)
    spare.rotation_euler = (-math.pi / 2, 0.0, 0.0)
    col.objects.link(spare)
    spare.parent = carrier
    # ---- roof rack: two rails and three cross bars
    rack_y = k.roof + 0.05
    rack = []
    for sx in (1, -1):
        rack.append(box((sx * (k.roof_half - 0.05), rack_y, 0.5 * (k.roof_front + k.roof_rear)), (0.02, 0.02, 0.5 * (k.roof_front - k.roof_rear) - 0.05)))
    for z in (k.roof_front - 0.15, 0.5 * (k.roof_front + k.roof_rear), k.roof_rear + 0.15):
        rack.append(box((0.0, rack_y + 0.025, z), (k.roof_half - 0.05, 0.012, 0.02)))
    for sx in (1, -1):
        for z in (k.roof_front - 0.05, k.roof_rear + 0.05):
            rack.append(box((sx * (k.roof_half - 0.05), 0.5 * (k.roof + rack_y), z), (0.015, 0.03, 0.015)))
    bm = bmesh.new()
    for part in rack:
        me = bpy.data.meshes.new("tmp")
        part.to_mesh(me)
        part.free()
        bm.from_mesh(me)
        bpy.data.meshes.remove(me)
    mesh_object("roof_rack", bm, col, mats["metal"], parent=roof)
    # ---- underbody: flat floor between the tub walls
    mesh_object("underbody", box((0.0, k.floor - 0.005, 0.5 * (k.zF + k.zR)), (v.inner_x - 0.02, 0.005, 0.5 * (k.zF - k.zR) - 0.35)),
                col, mats["underbody"], parent=body)
    return k


def collision_pieces(v, k, body_col, col_col):
    """Convex decomposition along the wheel-sweep boundaries: clip the evaluated body (colliding parts) to each cell
    and take the convex hull of what is inside."""
    dg = bpy.context.evaluated_depsgraph_get()
    src = bmesh.new()
    for o in body_col.all_objects:
        if o.type != "MESH" or "mirror" in o.name:
            continue
        me = o.evaluated_get(dg).to_mesh()
        me.transform(o.matrix_world)
        src.from_mesh(me)
        o.evaluated_get(dg).to_mesh_clear()
    fw, rw = k.front_wheel, k.rear_wheel
    xi = v.inner_x - 0.005
    top_lo = max(w.arch_top for w in v.wheels) + 0.002
    zf, zr = fw.z + fw.arch_radius + 0.002, rw.z - rw.arch_radius - 0.002
    zfs, zrs = fw.z - fw.arch_radius - 0.002, rw.z + rw.arch_radius + 0.002
    big = 10.0
    cells = {  # name: (x range, y range, z range) in the game frame
        "COL_tub": ((-xi, xi), (-big, top_lo), (-big, big)),
        "COL_front": ((-big, big), (-big, top_lo), (zf, big)),
        "COL_rear": ((-big, big), (-big, top_lo), (-big, zr)),
        "COL_side_left": ((xi, big), (-big, top_lo), (zrs, zfs)),
        "COL_side_right": ((-big, -xi), (-big, top_lo), (zrs, zfs)),
        "COL_band": ((-big, big), (top_lo, k.deck + 0.03), (-big, big)),
        "COL_cabin": ((-big, big), (k.deck + 0.03, big), (-big, big)),
    }
    pieces = []
    for name, (xr, yr, zr_) in cells.items():
        bm = src.copy()
        planes = [((xr[0], 0, 0), (-1, 0, 0)), ((xr[1], 0, 0), (1, 0, 0)), ((0, yr[0], 0), (0, -1, 0)),
                  ((0, yr[1], 0), (0, 1, 0)), ((0, 0, zr_[0]), (0, 0, -1)), ((0, 0, zr_[1]), (0, 0, 1))]
        for co, no in planes:
            if max(abs(c) for c in co) >= big:
                continue
            geom = list(bm.verts) + list(bm.edges) + list(bm.faces)
            bmesh.ops.bisect_plane(bm, geom=geom, dist=1e-5, plane_co=Vector(B(*co)), plane_no=Vector(B(*no)),
                                   clear_outer=True)
        pts = [tuple(vv.co) for vv in bm.verts]
        bm.free()
        if len(pts) < 4:
            print(f"TF: {name} is empty")
            continue
        game = [(p[0], p[2], -p[1]) for p in pts]
        out = hull(game)
        bmesh.ops.dissolve_degenerate(out, dist=0.001, edges=list(out.edges))
        bmesh.ops.triangulate(out, faces=list(out.faces))
        o = mesh_object(name, out, col_col, None)
        o.display_type = "WIRE"
        o.hide_render = True
        pieces.append(o)
    src.free()
    return pieces


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="trailforge_suv")
    ap.add_argument("--out", required=True, help=".blend to write")
    ap.add_argument("--export", help="also export the source glTF (BODY + COL) here")
    args = ap.parse_args(argv)

    v = tf_vehicle.Vehicle(args.vehicle)
    print(v.summary())
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene["tf_vehicle"] = v.json_path if args.vehicle.endswith(".json") or os.sep in args.vehicle else args.vehicle
    mats = make_materials()
    refs = collection("REF_references")
    proxies = collection("PROXY_wheels")
    cut = collection("CUT_arches")
    body_col = collection("BODY")
    col_col = collection("COL")
    references(v, refs)
    proxy_wheels(v, proxies, mats)
    cutters = arch_cutters(v, cut, mats)
    k = build_body(v, body_col, mats, cutters)
    pieces = collision_pieces(v, k, body_col, col_col)
    print(f"TF: {len(body_col.all_objects)} body objects, {len(pieces)} collision pieces")
    # light rig + camera for quick looks (not exported)
    sun = bpy.data.objects.new("REF_sun", bpy.data.lights.new("REF_sun", "SUN"))
    refs.objects.link(sun)
    sun.rotation_euler = (math.radians(50), math.radians(10), math.radians(140))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(args.out))
    print("TF: wrote", args.out)
    if args.export:
        import export_body
        export_body.export_source(os.path.abspath(args.export))


if __name__ == "__main__":
    main()
