"""Builds a level .blend from the game files of a baked level (heightmap.pgm, surfaces.pgm, layout.json).
Exporting the result again without edits gives the same files byte for byte (CTest level_blender_roundtrip).
The imported terrain is its own carve base (tf_base_height): carve_track.py then cuts edited tracks into the
terrain as it was exported. The generator's .blend (make_<level>.py) keeps the uncarved base instead.

Headless:
  /Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup --python-exit-code 1 \\
      --python tools/levels/blender/import_level.py -- --level assets/levels/<id> --out build/levels/<id>.blend
"""

import argparse
import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tf_level  # noqa: E402


def run(level, out=None):
    layout, heights, classes = tf_level.read_level_files(level)
    tf_level.build_scene(layout, heights, classes, os.path.basename(os.path.normpath(level)))
    if out:
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        bpy.context.preferences.filepaths.save_version = 0  # no .blend1 backup
        bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(out), compress=True)
        print(f"TF: imported {level} -> {out}")


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", required=True)
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    run(os.path.abspath(a.level), a.out)


if __name__ == "__main__":
    main()
