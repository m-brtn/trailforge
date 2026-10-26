"""Exports the BODY and COL collections of a vehicle body .blend as glTF and converts it into the game's body.glb with
trailforge_vehiclec (validation: part names, material roles, convex COL_* pieces, wheel-sweep clearance for every
kit and garage ride height, visual vs collision bounds). Nothing is written to OUT when validation fails.

Headless:
  /Applications/Blender.app/Contents/MacOS/Blender -b BODY.blend --python tools/cars/blender/export_body.py -- \
      --out assets/vehicles/<id>/body.glb [--vehicle <id> | PATH/vehicle.json] [--source /tmp/body_src.glb] \
      [--converter PATH]
--vehicle defaults to the one the .blend was generated for (scene property tf_vehicle).
From a live Blender session (e.g. over MCP):
  import sys; sys.path.insert(0, "<repo>/tools/cars/blender"); import export_body
  export_body.run(out="<repo>/assets/vehicles/<id>/body.glb")
"""

import argparse
import os
import subprocess
import sys
import tempfile

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tf_vehicle  # noqa: E402

EXPORT_COLLECTIONS = ("BODY", "COL")


def export_source(path):
    """Selected objects = everything in BODY and COL (helpers REF_ / PROXY_ / CUT_ stay out); modifiers applied."""
    if bpy.context.object and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.object.select_all(action="DESELECT")
    n = 0
    for name in EXPORT_COLLECTIONS:
        col = bpy.data.collections.get(name)
        if not col:
            continue
        for o in list(col.all_objects):  # hiding / selecting rebuilds all_objects: iterate a copy
            o.hide_set(False)
            o.hide_select = False
            o.select_set(True)
            n += 1
    bpy.ops.export_scene.gltf(filepath=path, export_format="GLB", use_selection=True, export_apply=True,
                              export_yup=True, export_extras=True, export_materials="EXPORT", export_normals=True,
                              export_texcoords=False, export_animations=False, export_cameras=False,
                              export_lights=False)
    print(f"TF: exported {n} objects to {path}")
    return path


def find_converter(repo):
    for preset in ("release", "debug", "headless", "package"):
        p = os.path.join(repo, "build", preset, "trailforge_vehiclec")
        if os.path.exists(p):
            return p
    return None


def run(out, vehicle=None, source=None, converter=None, repo=tf_vehicle.REPO):
    v = tf_vehicle.Vehicle(vehicle or bpy.context.scene.get("tf_vehicle", "trailforge_suv"), repo)
    vehicle_json = v.json_path
    source = source or os.path.join(tempfile.gettempdir(), f"{v.id}_body_src.glb")
    export_source(source)
    converter = converter or find_converter(repo)
    if not converter:
        print("TF: trailforge_vehiclec not built (cmake --build --preset release --target trailforge_vehiclec)")
        return 2
    r = subprocess.run([converter, "--vehicle", vehicle_json, source, os.path.abspath(out)], capture_output=True, text=True)
    print(r.stdout + r.stderr)
    return r.returncode


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--vehicle")
    ap.add_argument("--source")
    ap.add_argument("--converter")
    ap.add_argument("--repo", default=tf_vehicle.REPO)
    a = ap.parse_args(argv)
    code = run(a.out, a.vehicle, a.source, a.converter, a.repo)
    sys.exit(code)


if __name__ == "__main__":
    main()
