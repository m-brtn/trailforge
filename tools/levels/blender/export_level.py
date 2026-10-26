"""Exports the open level .blend to the game files heightmap.pgm, surfaces.pgm and layout.json (tf_level.py has the
format). The export is a plain dump of the scene: carving the tracks into the terrain is an explicit step
(carve_track.py, also run by the generator). The layout is validated with the game's rules first (nothing on a road,
structures out of the water, gates on the circuit, the circuit at most 0.45 m under water, everything in the play
area); nothing is written when it fails unless --force.

Headless:
  /Applications/Blender.app/Contents/MacOS/Blender -b LEVEL.blend --python-exit-code 1 \\
      --python tools/levels/blender/export_level.py -- --out assets/levels/<id> [--force]
From a live Blender session (e.g. over MCP):
  import sys; sys.path.insert(0, "<repo>/tools/levels/blender"); import export_level
  export_level.run(out="<repo>/assets/levels/<id>")
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tf_level  # noqa: E402


def play_half(out):
    try:
        return float(json.load(open(os.path.join(out, "level.json"))).get("playHalfExtent", 1e9))
    except (OSError, ValueError):
        return 1e9


def run(out, force=False):
    layout, heights, classes = tf_level.scene_layout()
    issues = tf_level.validate(layout, heights, play_half(out))
    for issue in issues:
        print("TF: layout:", issue)
    if issues and not force:
        raise RuntimeError(f"{len(issues)} layout issues, nothing written (--force writes anyway)")
    tf_level.write_level_files(out, layout, heights, classes)
    roads = ", ".join(f"{r['name']} {len(r['samples'])} samples" for r in layout["roads"])
    print(f"TF: exported {out}: grid {layout['grid']['samples']}, roads [{roads}], {len(layout['water'])} water, "
          f"{len(layout['placements'])} placements, {len(layout['checkpoints'])} checkpoints, "
          f"{len(layout['pois'])} POIs, {len(issues)} issues")
    return layout


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args(argv)
    run(os.path.abspath(a.out), a.force)


if __name__ == "__main__":
    main()
