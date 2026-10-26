"""Cuts the TRACKS curves of the open level .blend into the TERRAIN again, from the terrain before carving
(attributes tf_base_height / tf_base_surface): the road follows the curve height and tilt (bank), cut / fill banks
meet the base terrain, the road surface comes from tf_segment_surface. Run it after editing a curve, then export.

Headless (saves the file):
  /Applications/Blender.app/Contents/MacOS/Blender -b LEVEL.blend --python-exit-code 1 \\
      --python tools/levels/blender/carve_track.py -- --save
From a live Blender session:
  import sys; sys.path.insert(0, "<repo>/tools/levels/blender"); import carve_track; carve_track.run()
"""

import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tf_level  # noqa: E402


def run(save=False):
    _, _, worst = tf_level.carve_scene()
    print(f"TF: carved the tracks; largest change at the carve reach {worst:.2f} m (0 = the banks meet the terrain)")
    if save:
        bpy.ops.wm.save_mainfile(compress=True)


if __name__ == "__main__":
    run(save="--save" in sys.argv)
