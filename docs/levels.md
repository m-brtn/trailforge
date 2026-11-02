# Levels

The proving ground, Exploration Peaks, Suncoast Raceway, editing a level in Blender, drift scoring, the map and the attract drive. All documentation: [index](README.md).

## Proving ground (~100×100 m)

Surrounded by an earth berm and concrete walls. South of its south wall is the walled mini handling track
(see below).

- **Start** — concrete pad (z ≈ −40).
- **Lane A** (center):
  - 0.18 m bumps every 3 m;
  - diagonal waves ±0.38 m — wheels lift diagonally on them;
  - **low-grip section**: a hard surface with an effective peak μ of 0.42 (lateral 0.40). Not mud or ice — just a
    low coefficient.
- **Lane B** (right of the start, −X): hills of 12°, 20°, 28° and 35°, 1.8 m high.
- **Lane C** (left, +X):
  - 18° side slope;
  - steps of 0.15 / 0.30 / 0.45 m;
  - a field of 28 rocks and two boulders. Rocks are angular convex chunks (a subdivided icosahedron cut by random
    planes), flat-shaded; the collision mesh is the same triangles.
- **Skid strip** (east edge, x 39–45, z −48…48, teleport "Skid strip (concrete)"): a flat 6 × 96 m concrete strip
  for braking and wheelspin on concrete; its north end meets the sand area directly (teleport "Sand to concrete"
  starts on the sand facing south over the boundary).
- **Sand area** (east meadow, x 28–45, z 48–84, teleport "Sand area (dunes)"): the rigid `Sand` surface (peak μ
  0.55/0.52), flat next to the skid strip and with low dunes (crossing ridges up to ~0.9 m) in the north part.
  There is **no sinkage**: sand is only a lower-μ rigid surface with a light sandy material.
- **Mud bath and west strip** (west edge, x −44.5…−38.5, teleport "Mud bath to concrete"): a flat 6 × 12 m `Mud`
  patch (z 34–46) that ends directly on a 30 m concrete strip (z 4–34), for driving out of mud onto concrete.
- **Bump strip** (west edge south of the west strip, x −43…−38, teleport "Bump strip 0.35 m", starting at z −43):
  two 0.35 m high, 2.5 m long round mounds of packed dirt, at z −24 across both wheel tracks and at z −10 under the
  left wheels only (`tools/scenarios/bump_check.txt`).
- Teleports "Meadow (grass)" (open grass east of the mud path) and "Puddle (splash)" (on the mud path just before a
  puddle) are spawn points for the wheel spray checks.
- **Wheat test patch** (west meadow, x −46…−28, z 56…76, teleport "Wheat patch (visual)"): render-only wheat. The
  ground under it is the ordinary `Grass` surface; the stalks have no collision and no drag, they only bend under the
  wheel contacts and in the trample mask. The real field comes with a later level.

- **Mud & puddles showcase** (north, z ≈ 46–86, teleport "Mud & puddles showcase"): a grass meadow with a
  winding mud path, puddle bowls in the terrain and a few rocks. The mud is the rigid `Mud` surface (peak μ 0.35/0.32),
  the meadow is `Grass` (peak μ 0.50/0.48). There is **no sinkage**: puddles are dips in the hard ground under a
  still water level, whose buoyancy and drag act on the vehicle ([Water forces](physics.md#water-forces)). The ruts you see are tire marks, which are render-only too
  (see [Tire marks](rendering.md#tire-marks-visual-only)). Deformable terrain is a later stage.

- **Mini handling track** (`src/track/MiniTrack`, walled area x −185…185, z −232…−51 south of the proving ground,
  the last teleport, "Mini handling track", `teleport mini` in scenarios): a 1044 m asphalt circuit, 10 m wide, driven
  counter-clockwise from the map's view (east on the start / finish straight). Sections: a ~330 m start / finish
  straight (the EV reaches 138 km/h in the probe), a R13 hairpin, a chicane
  (8 m offset over 44 m), a R18 double-apex corner, a R32 constant-radius corner (steady lateral g window), a 150 m
  slalom (±2.5 m) on the back straight and a R80 fast sweeper; the west half is 1.8 m higher than the east half
  (smooth ramps). Flat red / white curbs (5 cm, 1.2 m wide) on the corners and the chicane, white edge lines, a
  checkered start line, a pit wall along the straight, grass runoff, gravel traps (the rigid `Gravel` surface, no
  sinkage) outside the hairpins and the fast corners, and tyre stacks (octagonal prisms, collision) behind them.
  The circuit and the curbs are the `Asphalt` surface (μ 1.00); the curb colors and the edge lines are render-only
  paint per triangle.
  A **lap timer** (`src/sim/LapTimer`: start / finish line and two sector gates, crossing times interpolated
  within a physics tick, a lap counts only through the gates in order; the same timer as on
  [Suncoast Raceway](#suncoast-raceway-race-circuit)) shows the lap panel below the compass once a lap was started:
  `LAP n`, the running time, last and best lap, the sector times. Teleporting (or R) restarts the lap; best laps
  driven by the player are stored per vehicle like on the circuit.
  Probe laps (`trailforge_probe minitrack`, autopilot driver, best clean flying lap, `Asphalt`): SUV 0:59.35
  (114 km/h, 0.92 g), EV 0:58.51 (145 km/h, 0.90 g), `builtin` 1:04.18 (115 km/h, 0.73 g), `rwd_test` 0:58.86
  (124 km/h, 0.95 g) (kit bodies, box collision, v0.24.0 brakes; on the earlier `Concrete` circuit 0:59.69 / 1:00.83
  / 1:04.22 / 0:59.41; with the v0.23.0 brakes 1:04.57 / 1:01.25 / 1:06.51 / 0:58.62). These are the autopilot's laps, not the vehicles' limits: with a higher lateral budget it overdrives the
  hairpin entry (oversteer) and leaves the track.

Rendering and collision are built from the same triangles (`TrackGeometry`). Each triangle has its own material.
The outer backdrop (hills, trees beyond the walls, distant mountains) and grass are render-only and cannot be reached.
Grip is computed per wheel: `SetCombineFriction` takes the material under the contact patch
and multiplies the tire curve by the surface coefficients — longitudinal and lateral separately.

The tire curve peaks at μ 1.00 (longitudinal, slip ratio 0.10; sliding 0.80) and 1.00 (lateral, 6°; sliding 0.78);
the surface multipliers therefore are the effective peak μ:

| Surface | Longitudinal | Lateral | Was (tire 1.20 / 1.15 × old multiplier) |
|---|---|---|---|
| Asphalt | 1.00 | 1.00 | new (mini track, level roads) |
| Concrete | 0.95 | 0.95 | 1.20 / 1.15 |
| Rock | 0.75 | 0.75 | 1.08 / 1.04 |
| Packed dirt | 0.70 | 0.68 | 1.02 / 0.98 |
| Gravel (loose) | 0.62 | 0.58 | new: peak slip × 2, slide drop × 0.4, rolling resistance 0.015 |
| Sand | 0.55 | 0.52 | new in v0.3.0 |
| Grass | 0.50 | 0.48 | 0.96 / 0.90 |
| Mud | 0.35 | 0.32 | 0.72 / 0.63 |
| Low grip (hard) | 0.42 | 0.40 | 0.42 / 0.40 (unchanged) |

These are plausible dry-concrete-to-mud ranges for an all-terrain tire, not measured data. Every surface is rigid:
mud, sand and grass are only lower friction, with no sinkage, rutting or bulldozing. Measured with
`trailforge_probe surfaces` (120 Hz, full throttle): straight launches keep grip on concrete (max slip 0.27 m/s,
0–30 km/h 1.52 s) and spin on grass (7.3 m/s, 0–30 km/h 2.20 s) and mud (9.4 m/s, 3.07 s); 50 → 0 km/h takes
13.7 m on concrete, 18.5 m on dirt, 25.4 m on grass and 36.2 m on mud (without ABS). In low range a full-throttle
hill start on 15° mud climbs slowly with the wheels spinning (14.1 s to the plateau, since the slip speed floor; it
failed before), 20° mud is beyond the friction limit (μ 0.35 < tan 20°), and the 20° mud ramp is climbed only with
momentum. Traction control results are under [Driver aids](vehicles.md#driver-aids).

**Asphalt and gravel** (`SurfaceId::Asphalt` / `Gravel`, keys `"asphalt"` / `"gravel"`). Sources: dry asphalt and
concrete peak μ 0.8–0.9 for passenger tires on a locked-wheel tester and up to ~1.0–1.1 for modern road tires (Wong,
*Theory of Ground Vehicles*, table 1.3; Bosch *Automotive Handbook*), gravel peak ~0.6 with a slide value ~0.55
(Wong, table 1.3); rolling resistance coefficients ~0.010–0.015 on paved roads and ~0.02–0.03 on gravel (Wong,
Gillespie, *Fundamentals of Vehicle Dynamics*). Asphalt is 5 % above the concrete pad (1.00 / 1.00) and otherwise
behaves like concrete (rubber marks, squeal, no spray). Gravel is 0.62 / 0.58 with a loose-surface tire response:
the slip axis of both curves is stretched × 2 (peak at slip ratio 0.20 and ~12°), and past the peak the drop toward
the sliding value is cut to 40 % (slide / peak ≈ 0.92), because a sliding tire digs into the loose layer; the friction
ellipse uses the stretched peak. These two numbers are assumptions chosen to match the qualitative behavior (a
wide, flat peak), not fitted data. Gravel adds a rolling resistance force of 0.015 × wheel load (the gravel value
minus a paved one) at each contact patch against the patch's rolling direction, faded in below 0.5 m/s so it never
pushes a stopped wheel; paved rolling resistance is still not modeled. Gravel is rigid: no sinkage, no ruts in the
collision, no bulldozing. Measured with `trailforge_probe surfacecmp` (120 Hz; 100-0 km/h ABS off / on, max lateral
g at 35 % steer, 60 km/h 5 s coast loss):

| Vehicle | Asphalt | Concrete | Gravel |
|---|---|---|---|
| SUV | 48.9 / 40.9 m, 0.88 g, −6.8 km/h | 51.5 / 42.8 m, 0.84 g, −6.8 km/h | 65.2 / 61.9 m, 0.47 g, −9.0 km/h (295 N) |
| EV | 46.9 / 38.0 m, 0.93 g, −21.2 km/h | 50.3 / 39.9 m, 0.89 g, −21.2 km/h | 61.3 / 57.6 m, 0.56 g, −23.6 km/h (390 N) |
| `builtin` | 46.6 / 41.7 m, 0.91 g, −4.7 km/h | 50.7 / 43.6 m, 0.87 g, −4.7 km/h | 66.6 / 63.2 m, 0.55 g, −6.9 km/h (295 N) |
| `rwd_test` | 45.8 / 40.4 m, 0.90 g, −7.5 km/h | 48.8 / 42.0 m, 0.86 g, −7.5 km/h | 62.6 / 60.1 m, 0.51 g, −9.6 km/h (245 N) |

The coast loss on asphalt and concrete is drag, wheel damping and (EV) regeneration only. CTest
`asphalt_gravel_surfaces` checks the table, the data keys, 80-0 km/h stops on the three lanes and the gravel rolling
resistance.

Teleport points are in the Tuning window (17 on the proving ground; the map shows them and the mini track).

## Exploration Peaks (open-world level)

The first level, after `external_assets/images/Exploration.png`. Chosen with the LEVEL row of the start menu
(EXPLORATION PEAKS / PROVING GROUND, saved as `level =` in settings.cfg) or `--level ID`; the proving ground stays as
the development level and is what scenarios and `--demo` start on.

- **Data:** `assets/levels/exploration_peaks/level.json` — elevation control points and peaks, coast polygons and
  beaches, 2 lakes, 4 rivers, 11 spline roads (width, banking, trail flag, `surface`: `dirt` (default), `gravel`
  or `asphalt`; the main roads Valley Road, Coast Road and Canyon Road are gravel, the trails packed dirt), forests, a wheat field, flats for the POI
  sites, 13 POIs, the spawn. `Level` bakes it at load (about 0.4 s in Release on the M-series Mac) into a 2561 × 2561 grid at 1 m
  (2.56 × 2.56 km, play area ±1000 m), heights −22 … 297 m: base relief, coast (sea cliffs / beaches, seabed),
  flats, lakes (bed, shore band, outline smoothed), river channels, roads (flattened and banked, cut and fill with
  shoulders), then the per-cell surface (the road's own surface on the road, rock above 40° over ~4 m, grass, sand,
  mud on wet banks and in dirt road ruts, snow above a noisy snow line), render masks, ~22.1k trees (thinning out over the last ~200 m before the bake edge), ~1.2k rocks, logs, cliff faces and scree
  boulders, and the POI buildings
  ([Buildings](rendering.md#buildings)).
- **Tree style:** Settings → TREES. STYLIZED (the default) bakes every tree as the procedural tiered fir
  (`buildPineMeshes`, the same pines that stand in when the model library is missing; trunks collide as procedural
  prisms in the terrain tiles); DETAILED uses the CC0 conifer, sapling and broadleaf models of `vegetation`. Bushes,
  ferns, debris, rocks, logs, cliffs and statics are models in both. A change bakes the current level again (the
  vehicle returns to the spawn). A model that fails to load is skipped and named in the log
  (`Level::modelError()`); only a library with no loadable model falls back to procedural trees and rocks.
  `tools/scenarios/tree_style.txt` captures both on Exploration Peaks; CTest `level_tree_style`.
- **Steep ground:** no straight walls from the bake itself. A lake whose surroundings are lower than its water keeps
  a 12 m lip that falls away as an uneven 15–27° embankment; inside the outline the bed shelves down at most ~27°,
  and gorge walls rise from it at most ~63°. Steep river banks (`bankSlope` ≥ 1.5, the canyon) have a scree apron of
  1.5–5.5 m at ~37° at the water, spurs and gullies (the slope varies ±25 % along the wall), ledges every ~16 m of
  height, a bank line that wanders a few meters off the spline and a rim rounded by a smooth minimum. Cut banks
  within 40 m of a road are relaxed to about 45° (switchback legs used to meet in a wall). Road profiles follow the
  land: the grade-limited profile is the cheapest one under a cost that prefers cut to fill (fill above the ground
  costs 3x, beyond 1.5 m 11x; `level_math::gradeLimitedProfile`), so a climb steeper than the road's `maxGrade` is cut
  into its upper end instead of raising an embankment far ahead of it, and a road end on a POI flat that the profile
  reaches within a 6 m cut meets the flat. Shoulders blend back into the ground at 1:3 at their steepest on fill and
  1:2 in a cut (steeper only for fills and cuts beyond ~10 m, within the 48 m reach). CTest `level_road_embankment`
  compares the bake with a bake without roads; `tools/scenarios/road_views.txt` shows the former worst spots. The coast cap fades out
  toward its 400 m reach (it used to cut a 64 m cliff into the northern range). Cliff faces (CC0 models with their
  collision meshes) clad walls of 46° or more every ~7 m, scree boulders (rock models, collision meshes) lie at the
  wall feet where the ground within 6 m rises by more than 6 m. CTest `level_slopes`: no dry cell steeper than 80°
  beyond 0.04 % (measured 0.000 %) and no single 1 m step of 8 m or more outside the waterfalls (largest 7.3 m, a
  canyon ledge). `trailforge_probe levelslope` prints the slope statistics and writes a slope map.
- **Physics:** the grid is split into 255 m Jolt `HeightFieldShape` tiles (`TerrainTiles`) with per-sample
  materials; tiles within 420 m are built on a worker thread (removed beyond 620 m), a tile within 110 m that is
  not ready yet is built synchronously. Tree trunks and rocks are per-tile mesh colliders. The rendered terrain, the props and the colliders
  come from the same grid and triangles. Every prop piece (bridge decks and ramps, piers, buildings) is wound
  outward, because Jolt's wheel casts ignore back faces (CTest `level_prop_winding`); both vehicles cross both
  bridges in both directions on four wheels without chassis contact (`level_bridges`). Buildings and props are
  placed clear of the road corridors (`level_placement`).
- **Water:** rivers, the lakes and the sea are flat, still surfaces over rigid ground; they push the vehicle with
  buoyancy and drag ([Water forces](physics.md#water-forces)). Fords are shallow dips the vehicle drives through on
  rigid ground. Deep water: when the body origin stays below the water surface, or the vehicle is afloat (buoyancy
  ≥ 80 % of its weight), for 1 s the vehicle is put on the nearest road with the message "Deep water: vehicle
  submerged, recovered to the nearest road" (or "... vehicle afloat, ..."). The play limit (±1000 m) shows a warning 40 m before it and recovers the vehicle to
  the nearest road 8 m past it. R resets to the nearest road point, not to the spawn.
- **Rendering:** quadtree terrain LOD over the same height grid (64 × 64 patch, split by distance and by a height
  error; `terrainDetail` per preset), splat weights (road paint with worn shoulders, rock, sand, snow) over the grass
  / mud / wet / wheat masks, shaded with photo-scanned materials ([Terrain textures](rendering.md#terrain-textures)); steep rock
  gets warped side projections (no stacked texture repeats), dipping sedimentary beds (grey, buff and iron-stained
  tones, small ledges in the normal) and large tone patches; under the tree crowns (a canopy weight from the placed
  trees in the splat alpha) the floor turns into dark needle and leaf litter with less sky light; 4 tree LODs (crossed cards beyond the tree detail distance: 380 / 500 / 650 / 900 m per
  preset, the mesh LODs switch at 0.145 / 0.37 of it; the last 12% before each switch is a dithered cross-fade of
  both LODs, so trees do not pop; drawn out to 2 / 3 / 3.5 / 4.5 km per preset); grass, flowers, wheat, trees, rocks
  and props keep clear of every road corridor and of bridge decks with their ramps (a 1.5–3 m fade from the edge,
  CTest `level_grass_clear`); a render-only horizon ring outside the bake out to 24 km (ridged hills that continue the
  bake edge, a northern range from ~1 km out; the ring rounds from the bake's square into a circle over the first
  ~4 km, the coast continues the bake's own sea / land along the edge with noisy bays, capes and small islands, and
  an outer coast 15–21 km out sinks into the open sea; the edge heights are averaged over a window that grows with
  the distance and the material starts from the bake's ground colors at the edge, then a per-pixel grass / forest /
  rock / snow / sand palette from noise tinted to the edge's mean tone; snow follows the level's own snow line, so
  the bake square does not read from above) and the open sea
  to 80 km; longer shadow cascades. Camera-relative rendering was not needed: at ±1280 m a float position step is
  ~1.2e-4 m and reversed-Z keeps the depth precision.
- **Map (M / LB):** see [Map, waypoint and compass](#map-waypoint-and-compass). No fast travel: teleports are in the
  F1 panel only.
- **Frame times** (2880 × 1800, Metal, the display caps at 120 FPS): High 112–120 FPS on the roads, in the village and
  at the quarry, 95 FPS looking across the Alpine Lake; Ultra 89 FPS in the village. Details in the devlog. v0.14.0 distances (grass 110 m, tree detail 650 m on High)
  against the v0.13.0 ones (45 m, 380 m), GPU ms with VSync off (medians of 3, a background VM running): forest
  view High 6.90 vs 6.85, Ultra 11.3 vs 9.2; village view High 7.6–11.0 vs 6.9–10.5 (noisy).

## Suncoast Raceway (race circuit)

A fictional circuit name on a real layout: the course is traced from OpenStreetMap data of a real circuit and its
ground from a DEM. LEVEL row of the start menu (SUNCOAST RACEWAY), `--level suncoast_raceway`, scenario `level
suncoast_raceway`. Map data © OpenStreetMap contributors, ODbL; heights from the Copernicus GLO-30 DEM (credits and
licences in `assets/levels/suncoast_raceway/CREDITS.md`; the OSM way ids, the bounding box and the hashes are in
`tools/levels/data/suncoast_raceway/sources.lock.json`).

- **What is real:** the plan geometry of every asphalt way of the venue (the full course with the drag strip, the
  short course connector, the paddock access), the venue fence and the farmland around it, converted from WGS84 to
  local metres (east = +X, north = −Z) about the middle of the circuit. Measured full course: 1979 m along the OSM
  polyline, 1989 m along the smoothed center line in the game (centripetal Catmull-Rom through the nodes; tightest
  radius 14.6 m). Published length of the real circuit: 2.21 km (11 turns, 12–15 m wide), so the mapped course is
  10 % shorter than the published figure; the course shape matches the published full-course map, which is the
  closed loop of the OSM ways (no invented sections). Drag strip 423 m (OSM spine), short course connector 369 m.
- **Not mapped, placed by the generator:** the width (OSM has no width tags; 12 m, published 12–15 m), the driving
  direction (clockwise on the map), the start line and the two sector lines (after the published map), painted edge
  lines and a 12-slot start grid, red / white curbs at the apexes and exits (33 curbs, 770 m), gravel traps (the
  gravel surface) outside the tighter corners and asphalt run-off elsewhere, grass beyond, tyre walls and armco
  barriers, the pit lane (8 m, north of the start straight) with 8 garages and race control, two grandstands, 12
  marshal posts, the start gantry with five start lights, flags, and trees and rocks only outside the track
  boundary.
- **Elevation:** the DEM tile cropped to the venue (`dem.json`, 30 m samples, 43–57 m above the geoid) is
  the terrain; the track surface is a quadratic least-squares fit to the smoothed DEM along every asphalt way
  (rms 0.21 m, 48.76–50.55 m, max grade 0.8 %), with the bank = the fit's cross slope. The DEM is a surface model
  of 30 m cells with decimetre noise: it cannot show real bumps or camber, so the circuit is smooth by
  construction.
- **Surface:** asphalt is the `Asphalt` surface (μ 1.00) drawn with the asphalt layer, gravel traps the `Gravel`
  surface (μ 0.62 / 0.58, loose curve, rolling resistance), no mud, no water, no jumps. CTest `level_baked_track_smooth` casts rays onto the physics
  tiles and props every 0.5 m along and across the circuit up to 0.6 m inside the edges (107,778 points): the
  height deviates from the chord between the points 1 m before and after by at most 4.1 mm (the 4 mm painted start
  line; limit 6 mm), max grade 0.9 %. The carve keeps a 1.5 m flat shoulder beside the edge (the 1 m height grid
  would otherwise bring the cut bank onto the asphalt).
- **Markings and curbs** (layout.json road `markings`): edge lines, the start line and grid brackets are 4 mm paint
  quads, curbs are 1.3 m wide prisms, 4.5 cm high, sunk 5 cm into the ground; all of them are prop triangles
  (`LinePaint`, `CurbRed`, `CurbWhite` styles, the `Asphalt` surface), drawn and collided as the same triangles.
- **Timing:** gate 0 is the start / finish line, then checkpoints and the two sector lines (`"sector": true`) in
  driving order, 6 gates. The lap panel (below the compass): lap number and checkpoints, the running time, last and
  best lap (the best of the session and of `best_laps.cfg`), S1 … S3 of the running lap (purple: a session-best
  sector) and of the last lap. A lap counts only through every checkpoint in order; R during a lap puts the vehicle
  4 m past the last gate passed (the lap keeps running); teleports, placing, rebuilds and handing over between the
  autopilot and the player stop the lap. Best laps driven by the player are stored per level and vehicle in
  `best_laps.cfg` in the user data directory (`level vehicle seconds` lines).
- **Autopilot laps** (`trailforge_probe lap`, the attract-drive autopilot at the level's `attractRoute` pace: 130
  km/h cruise, 5 m/s² in curves, 4 m/s² planned braking, full throttle allowed): SUV 2:16.25 (S1 39.77, S2 61.93,
  S3 34.55; 100.5 km/h max, 7.7 m/s² peak lateral), EV 2:14.32 (100.6 km/h, 7.5 m/s²) on the `Asphalt` surface
  (2:16.32 / 2:14.39 when the circuit was `Concrete`), both 100 % of the ticks on
  four asphalt wheels, no resets (flying laps; CTest `level_baked_lap` / `level_baked_lap_ev` drive the first lap).
  A gentle driver, not the vehicles' limit.
- **Size:** 4.0 MB of plain files (heightmap 2.1 MB, surfaces 1.0 MB, layout 0.66 MB, source data 0.38 MB), not in
  Git LFS; bake ~45 ms.

### Editing a level in Blender

A baked level (`level.json` `"baked": {"heightmap", "surfaces", "layout"}`) is three plain files: `heightmap.pgm`
(16-bit P5, samples² heights over `layout.json` `grid.heightMin..heightMax`), `surfaces.pgm` (8-bit P5, one surface
class per 1 m cell: asphalt, gravel, dirt, mud, grass, sand, rock, snow) and `layout.json` (roads as 2 m center line
samples with bank plus the Bezier they came from, painted `markings`, water polygons, placements, spawn, checkpoints
with `sector`, POIs). `Level` loads them as they are: no relief, carve or class is recomputed in the game, and the
physics tiles, the render terrain and the prop triangles come from them. The Blender scripts (run only with Blender
5.3 Beta; `tools/levels/blender/`, the format in `tf_level.py`) turn them into a scene and back:

```sh
B=/Applications/Blender.app/Contents/MacOS/Blender
# level files -> .blend (TERRAIN mesh with a surface colour attribute, TRACKS curves, WATER, PLACEMENTS, MARKERS)
$B -b --factory-startup --python-exit-code 1 --python tools/levels/blender/import_level.py -- \
   --level assets/levels/suncoast_raceway --out build/levels/suncoast_raceway.blend
# after editing a track curve: cut the tracks into the terrain again (from the uncarved base heights)
$B -b build/levels/suncoast_raceway.blend --python-exit-code 1 --python tools/levels/blender/carve_track.py -- --save
# .blend -> level files (validated: placements off the roads and out of water, gates on the circuit, play area)
$B -b build/levels/suncoast_raceway.blend --python-exit-code 1 --python tools/levels/blender/export_level.py -- \
   --out assets/levels/suncoast_raceway
# the whole level again from the raw data (OSM + DEM): make_suncoast_raceway.py; numbers only: --plan
$B -b --factory-startup --python-exit-code 1 --python tools/levels/blender/make_suncoast_raceway.py
```

In Blender: move the TRACKS curve points or handles (tilt = bank; custom properties `tf_width`, `tf_kind`,
`tf_segment_surface`, `tf_markings`), paint the `tf_surface` colour attribute of TERRAIN with the palette in
`tf_level.py`, sculpt TERRAIN, move or duplicate PLACEMENTS empties (`tf_kind`, `tf_ref`), move the MARKERS
(`SPAWN`, `CP_00` = start / finish, `CP_nn` checkpoints with `tf_half_width` and `tf_sector`, `POI_nn`). From a live
session the same steps are `import carve_track; carve_track.run()` and `import export_level;
export_level.run(out=...)` after `sys.path.insert(0, "<repo>/tools/levels/blender")`. Imported curves keep FREE
handles (exact); the generator uses ALIGNED ones. CTest `level_blender_roundtrip` (where Blender is installed)
imports the shipped level, exports it without edits and requires the same three files byte for byte and the same
bake (`trailforge_level_tests level_baked_compare`). The raw data is fetched by `tools/levels/fetch_osm_track.py`
(venv: `tools/levels/requirements.txt`), which compares it with `tools/levels/data/suncoast_raceway/sources.lock.json` (sha256; `--check`
verifies the committed files, `--update-lock` accepts new OSM edits).

## Drift scoring

A small judge (`src/sim/DriftScore.h`, `DriftScorer`, in `trailforge_sim`) fed with the body motion of every
physics tick; it only reads simulation data and never feeds back.

- **Score:** body slip angle = angle between the body heading and its horizontal velocity (body origin). A tick
  counts as drifting at |slip| ≥ `minAngleDeg` and ≥ `minSpeedKmh` with at least two wheels on the ground inside the
  course `area`. The first drifting tick starts a run; each drifting tick adds `pointsScale` × min(|slip|,
  `maxScoredAngleDeg`) × km/h × dt × the multiplier of the clipping zone the vehicle is in (1 outside).
- **End of a run:** completed after `linkSeconds` without drifting (drifts closer together are linked) or on leaving
  the area; failed (0 points) on a spin (|slip| > `spinAngleDeg` above 3 m/s) or a body impact approaching at ≥
  `wallSpeed` m/s (the chassis contact listener of the impact sounds: walls, barriers, props, a rollover); a reset,
  teleport, placing or rebuild drops the run uncounted.
- **Clipping zones:** a polygon, or a line (a wall face from a to b) with a `reach` in m on both sides; a zone counts
  while the body origin or the rear bumper point (origin − forward × (wheel base / 2 + 1 m)) is inside it;
  overlapping zones: the largest multiplier. No zones: no bonus.
- **Per level:** `level.json` `"drift"` (every key optional; an object without `"enabled"` is enabled; missing:
  scoring off):

  ```json
  "drift": {"enabled": true, "minSpeedKmh": 20, "minAngleDeg": 12, "spinAngleDeg": 100, "maxScoredAngleDeg": 70,
            "linkSeconds": 1.5, "wallSpeed": 1.5, "pointsScale": 0.1,
            "area": [[x, z], [x, z], [x, z]],
            "zones": [{"name": "Outer wall T1", "multiplier": 2, "line": [[x0, z0], [x1, z1]], "reach": 2.5},
                      {"name": "Clip 1", "multiplier": 1.5, "polygon": [[x, z], [x, z], [x, z]]}]}
  ```

  Bad values fail the level load with a message (`parseDriftConfig`). The proving ground enables it in code for the
  mini handling track (`miniTrackDrift()`): area = the walled rectangle of the mini track, zone "Outer zone" ×2 (the
  outer 3.7 m band of the asphalt and edge over the steady part of the 32 m 180° corner), zone "Apex" ×1.5 (a line
  on the inner edge over the middle third of that arc, reach 2.5 m).
- **Switch:** F1 → Simulation → Drift scoring (World setting / On / Off) and the scenario command `drift
  on|off|level`; the override lasts the session.
- **HUD:** a panel below the lap panel while scoring is active and the vehicle is on the course: slip angle, speed,
  the running score, the zone and its multiplier, the last result (or LOST SPIN / WALL) and the best score.
- **Best scores:** completed runs driven by the player (not the autopilot) are stored per level and vehicle in
  `best_drift.cfg` in the user data directory (`level vehicle points` lines, highest kept).
- **Probe** `trailforge_probe drift`: 24 scripted attempts per vehicle on the mini track's 32 m corner (target slip
  20 / 30°, entry 40–65 km/h, power-over or handbrake entry; a simple counter-steer and throttle controller, TC off).
  A scripted driver, not the vehicles' limit. Measured on the `Asphalt` surface (2026-10-26): rwd_test best 842
  points (65 km/h handbrake, max 32.1°, 6.0 s of drift), 17 of 24 attempts scored, 1 spin, 6 wall hits; builtin
  best 353 (50 km/h handbrake, 86.3°), 15 scored, 2 spins, 5 walls; SUV best 364 (45 km/h handbrake, 24.2°), 15
  scored, 0 spins, 3 walls; EV best 462 (40 km/h handbrake, 53.6°), 7 scored, 8 spins, 4 walls. The best runs
  spend 0–0.3 s in the clipping zones: this driver holds a tighter line than the outer band.

## Map, waypoint and compass

**Full-screen map** (M, gamepad LB, or MAP in the pause menu; the simulation pauses while it is open). Drawn from the
level data, not from a screenshot: a 2048² raster of Exploration Peaks (1.09 m per pixel over the play area plus a
12% margin, darkened outside the ±1000 m limit) with hill shading from the north-west, 10 m contours (every 50 m
stronger), water by depth, surfaces (sand, rock, snow, mud, wheat), forest crowns and rocks; on top as vector lines
roads and trails by class (cased), rivers, bridges, the play boundary, the 13 POIs as numbered red circles with names
(a label that would cover another one is left out; the list names every POI), the vehicle arrow, the waypoint with a
dashed line from the vehicle, a scale bar (1 / 2 / 5 × 10ⁿ m) and a north arrow. The side panel lists the POIs with
their kind and distance (style after `external_assets/images/Exploration.png`). The proving ground gets a 1024²
top-down raster of its own track triangles (0.14 m per pixel, 1 m contours, puddles) with its 16 features as POIs.
The raster is built once per world on a worker thread (`buildLevelMap` / `buildProvingGroundMap` in
`src/app/MapData.cpp`, no SDL; Release on the M-series Mac: Exploration Peaks 15–17 ms, proving ground 9–11 ms),
uploaded as one mipmapped texture, and the CPU copy is freed. Lines and labels are drawn as vectors every frame, so
they stay sharp on HiDPI at every zoom.

| Map control | Keyboard / mouse | Gamepad |
|---|---|---|
| Zoom (around the pointer / the center) | wheel, Q / E, PgDn / PgUp | LT / RT |
| Pan | drag, W A S D or arrows | left stick (shows a crosshair) |
| Set / clear the waypoint (snaps to a POI within 16 pt; a click on the waypoint clears it) | left click; Space / Enter at the map center | A at the crosshair (map center) |
| Center on the vehicle | C | Y or RB |
| Center on a POI | click its row in the list | — |
| Close | M, Esc | LB, B |

There is no fast travel: the map only sets a waypoint. Teleports stay in the F1 panel (development).

**Compass bar** (top of the screen, Settings → MAP → COMPASS BAR): ±90° around the heading, ticks every 15°, N / NE /
E … labels, POIs within 900 m (120 m on the proving ground) as small numbered circles, the waypoint as a yellow pin
(clamped to the edge with an arrow when it is behind) and its horizontal distance below the tape.

**Minimap** (bottom right, Settings → MAP → MINIMAP on / off, MINIMAP SIZE small / medium / large = radius 8.5 / 11 /
14.5% of the window height, MINIMAP ORIENTATION turns with the vehicle (default) / north up): the same raster as a
textured circle showing 180 m around the vehicle (45 m on the proving ground), roads, rivers, POIs, the waypoint (on
the rim with a pointer when outside), a dashed guidance line from the vehicle to the waypoint, N on the rim and the
waypoint distance. Settings are saved in `settings.cfg` (`minimap`, `minimap_size`, `minimap_north_up`, `compass`).
Cost (Exploration Peaks, 2880 × 1800, High, VSync off, three alternating 3 s pairs in one process): GPU 15.24 / 15.05
/ 15.32 ms with the large minimap and the compass, 14.65 / 14.94 / 15.70 ms without — no difference above the
±0.5 ms noise of the scene.

Pure logic (no SDL) in `src/app/MapView.h`: world ↔ screen transform with rotation, zoom around a point, bearings
(clockwise from north, +X east, −Z north), compass placement, distance formatting, scale bar steps, waypoint toggle
(CTest `map_view_math`); `map_level_raster` checks the raster (water samples blue, dry samples not, POIs, lines,
build time). `tools/scenarios/map_check.txt` opens the map from gameplay and from the pause menu, zooms, drags,
centers, sets / clears / sets the waypoint by click, drives at it with the distance falling, uses the virtual pad (LB,
RT, left stick, A, B), switches the minimap options and opens the proving-ground map;
`/tmp/trailforge_map_*.png`.

## Attract drive (start menu)

Behind the intro and the start menu the vehicle drives a looped route by itself (`src/sim/Autopilot.h`, in
trailforge_sim): pure pursuit steering on the route (lookahead 7–16 m with speed, in tight curves at most 0.6 × the
curve radius, so it does not cut corners), a speed profile from the route
curvature (2 m/s² lateral, braking 1.6 m/s² before curves, accelerating 1.0 m/s² after them, slower downhill) and a
PI speed controller (throttle capped at 0.65 and reduced with steering). Its only output is a `DriverIntent`
(throttle, brake, steering) that goes through the same pedal smoothing as the player's; there are no forces and no
velocity setting. Routes: Exploration Peaks `attractRoute` in level.json ("Glacier Creek loop": Valley Road from
the Canyon Road junction past Alpine Lake, through the Glacier Creek ford and down to the village → Coast Road →
Ruins Track → Canyon Road, 3322 m at 40 km/h, junction corners rounded in the bake), proving ground: a 195 m rounded
rectangle around lane A. On a level the menu puts the vehicle on the route start facing along the route (when the
menu opens and after a level switch; the spawn faces a climb), and the attract drive uses 4H on a part-time
transfer case, ABS and traction control on and no lockers; Start puts the vehicle back on the spawn with its default
transfer mode and the aids of the settings. The vehicle page and the garage hold the vehicle with the brake. When
the vehicle is stuck (> 10 s below 0.6 m/s), off the route (> 12 m for 4 s) or on its side (2.5 s), it is recreated
at the route start like the R reset; Start and Quit to menu put it on the spawn. The menu camera follows the
smoothed heading (3.5 s) and swings from behind to the side and back over 80 s. CTest `autopilot_loops`: one full
loop without a reset for the gasoline SUV in 2H and 4H and the EV on both worlds (Exploration Peaks from the route
start: 327-329 s, max 42.3 km/h, max 0.99 m off the line; 2.0 m on the proving ground where it joins the loop from
the start pad).
