# Testing and diagnostics

`trailforge_probe` measurements, scenario mode (`--script`), the F1 diagnostics and the verification results. All documentation: [index](README.md).

## Headless measurements (`trailforge_probe`)

`build/<preset>/trailforge_probe` — headless measurements:
- `brake`, `course`, `bench` — braking, course drive, step cost at 60/120/240 Hz;
- `corner [steer] [collisionSteps]` — steady-state cornering on a flat pad, average loads;
- `demo` — loads along the `--demo` trajectory in 1 s windows (Σload, Σ(F·ŷ), bump stop hits);
- `hillbrake` — holding on climbs with the brake;
- `susp [feature throttle low secs steer]` — per-wheel suspension length, compression and hub height in body space
  over a track feature (default 2, the diagonal twister), body-terrain contact ticks;
- `bump all|quick [hold|cruise|coast|full]` — straight runs over one bump on the flat pad for every shipped vehicle
  and `builtin` (or `TRAILFORGE_VEHICLE`): 1 m kicker, 2.5 m mound and 60° step, under both wheel tracks or the left
  ones, 0.20 / 0.30 / 0.40 m at 20 / 30 / 40 km/h (`quick`: 0.30 / 0.35 / 0.40 m at 30 km/h); from 1 m before the
  bump the driver holds the speed, freezes the throttle, coasts or floors it; prints CoM rise, airtime, roll / pitch,
  suspension speed, hard stop and rim hits, body hits and rollovers per run and a `RESULT` line;
  `bump trace OFFSET H KMH DRIVER SHAPE` prints one run per tick;
- `compare` — the EV vs SUV table: launch, brake 50-0 / 100-0 with and without ABS (regen share), lift-off, corner,
  climbs on dirt 20° and mud 15° per drive mode, the twister, steady-speed energy (electric only);
- `minitrack [vehicle.json ...]` — scripted laps of the mini handling track for the SUV, the EV, `builtin` and
  `rwd_test` (or `TRAILFORGE_VEHICLE`, plus any extra definitions on the command line): the autopilot drives a standing
  and a flying lap per lateral-acceleration budget of its speed plan (5 … 10 m/s², `TRAILFORGE_MINITRACK_BUDGET`
  for one), prints lap and sector times, max speed, max lateral g, steady g in the constant-radius corner, line
  error and time off the asphalt, and a `RESULT` line with the best clean flying lap (`TRAILFORGE_MINITRACK_TRACE=1`
  prints a trace);
- `brakedetail` — a 100 km/h concrete stop with ABS off and on, averaged per axle while above 5 km/h: pedal / ABS
  pressure, brake torque, load, |Fx|, μ of the curve × surface, Fx / (μ load) (how much of the available grip is used)
  and slip;
- `lap [level.json] [laps]` — autopilot laps of a circuit level (default Suncoast Raceway, 2 laps) at its
  `attractRoute` pace: lap and sector times, max speed, max lateral acceleration (from the body path, ~0.2 s
  smoothing), share of ticks with all four wheels on asphalt, resets; exit code 1 without the laps;
  `TF_LAP_TRACE=1` in the environment prints the position, speed and the autopilot's intent every 0.5 s;
- `skid` — tire slip, squeal and mark response for launches, brakes and turns on the surface lanes and a flat pad;
  `launch [lane throttle]` — the same per tick for one launch (`PROBE_LOW_RANGE`, `PROBE_HZ`,
  `PROBE_NO_STATIC_GRIP`, `PROBE_NO_CLUTCH_LIMIT` in the environment); `showcase` — where the skid layer is audible
  on a scripted drive around the mud showcase; `squeal` — squeal peak, output level, length and tail after the slide
  for gentle / full launches, light, moderate, tapped, full and ABS braking and handbrake / power slides on concrete,
  dirt and the flat pad (the mixer has only the two squeal sounds, as sines of the real lengths; `PROBE_TRACE=<case>`
  prints the per-wheel slip); `skid` also lists the wheel-seconds of squeal per surface under the wheel;
  `keyboard` — instant W / S (1.0 at once) against the keyboard pedal ramp (`src/input/KeyboardPedals.h`): held W
  launch, 0.15 / 0.25 / 0.4 s S taps and a held S at 50 km/h on the concrete lane and the flat pad (slip, sliding
  time, marks, 0-30 km/h, stopping time and distance); `launchmarks` — rubber laid by gentle / keyboard / hard /
  full-pedal launches in 2H and 4H and by 1 / 2 / 5 s stationary burnouts on the concrete lane (ribbon length, peak,
  mean, dark length and width; burnout stamps composited per spot);
- `surfaces` — effective friction per surface: full-throttle launches (high and low range) and 50 km/h stops on the
  flat surface lanes (with the asphalt and gravel lanes of `TrackLayout::RoadLanes`), rolling climbs and hill starts on 15° / 20° ramps of every surface (`TrackLayout::SurfaceRamps`,
  headless only); `climb [lane]` — the force balance of one ramp climb;
- `surfacecmp` — asphalt vs concrete vs gravel (`TrackLayout::RoadLanes`): 100-0 km/h stops ABS off / on (run-up
  with traction control, distance scaled to exactly 100 km/h), the maximum lateral g (1 s mean) at 35 % steer while
  the speed ramps up on the flat pad given the lane's surface, and a 5 s coast from 60 km/h with the rolling
  resistance force;
- `abs [0|1]` — 50 and 100 km/h full stops on the asphalt, concrete, rock, dirt, gravel, sand, grass and mud lanes, straight and with
  40 % steer, ABS off / on: distance, heading after 1 s, locked wheel-seconds, ABS cycle rate (steered runs are cut
  when the vehicle leaves its lane); the argument sets the friction ellipse off / on;
- `ellipse` — friction ellipse off / on: steady cornering at 30 / 60 / 100 % steer on concrete, dirt and mud
  friction, full-throttle launches on five lanes;
- `tc` — flat high-range launches and ramp hill starts with TC off / on / off-road (`TF_TC_TRACE=1` prints a trace);
  `tcmud` — the 15° mud hill start at 60 / 120 / 240 Hz from three creep points per TC mode;
- `diff` — differential configurations on `TrackLayout::SlopeWithWheelPit` (headless only) and the diagonal twister
  (`TF_DIFF_TRACE=1` prints a trace).

## Scenario mode

`./build/release/trailforge --script tools/scenarios/full_check.txt` — full window and rendering.
Input takes the same path as a live player's: the scenario pushes keyboard, mouse and wheel events
into the SDL queue (`SDL_PushEvent`), `Input` processes them, and they reach the physics on ticks.
`Input` tracks held keys from events rather than from `SDL_GetKeyboardState`,
so synthetic and real key presses are handled identically.

One command per line, `#` starts a comment:

| Command | What it does |
|---|---|
| `down/up/tap KEY` | press / release / press and release (SDL name: `W`, `Space`, `Escape`, `Left`…) |
| `wait S` | pause for S seconds of scenario time (see below) |
| `until VAR OP VAL TIMEOUT` | wait for a condition (`< <= > >= == !=`); FAIL on timeout |
| `expect VAR OP VAL` | check immediately |
| `mark VAR` | remember a value; afterwards `d_VAR` = current − remembered |
| `teleport N\|NAME` / `resize W H` / `capture file.png` | teleport to point N (or the first point whose name contains NAME, case-insensitive, e.g. `teleport mini`), `SDL_SetWindowSize`, save a frame |
| `click KEY` / `inputclick KEY` / `scrollto KEY` / `text STR` | click an ImGui widget, Cmd+click (number input), scroll the Tuning window to a widget, type text |
| `orbit DX DY` / `wheel DY` | camera orbit with the right mouse button / mouse wheel |
| `log NAME` / `reset_stats` / `quit` | telemetry line to stdout / reset min/max counters / quit |
| `quality Low\|Medium\|High\|Ultra` | apply a graphics preset |
| `set KEY V` | graphics/look setting: `render_scale`, `msaa`, `fxaa`, `cascades`, `shadow_size`, `shadow_dist`, `shadow_filter`, `far_shadows` (0/1), `env_probe` (vehicle probe face size, 0 = off), `env_probe_faces`, `reflections`, `refl_scale`, `bloom`, `grass`, `grass_dist` (m), `tree_detail` (m, tree mesh LOD distance), `tree_dist` (m, tree cull distance), `flowers` (0..1), `grass_detail` (0/1), `clouds`, `exposure`, `fog`, `auto_reverse` (0 — S only brakes), `tod` (hours), `sun_heading` (deg), `tod_speed` (h/s), `tod_run` (time-lapse 0/1), `mud`, `auto_mud`, `headlights`, `debug_lines`, `ui`, `hud`, `graphics_ui`, `vsync`, `minimap` / `compass` (0/1, session only); tire marks: `marks` (draw 0/1), `marks_gen` (generate 0/1), `marks_max` (ring capacity), `marks_fade` (s), `marks_gain` (all surfaces), `marks_clear 1`, `marks_fill 1` (synthetic fill to the cap); wheel spray: `spray` (0/1), `spray_max` (cap), `spray_dust` (0/1), `spray_clear 1`, `spray_fill 1` (synthetic fill to the cap around the vehicle); body mud: `body_mud V` (wet coat V on every region), `body_mud_clear 1`; post effects (see [Post effects](rendering.md#post-effects)): `taa`, `ao` (0/1/2), `contact_shadows` (steps, 0 = off), `motion_blur`, `shafts`, `flare`, `auto_exposure`, `dof`, `dof_driving`, `tonemap` (0 ACES, 1 AgX), `grade`, `ao_strength`, `vignette`, `grain`, `chromatic`, `sharpen`, `water_refraction`, `water_ssr`; sky: `atmosphere` (0..1), `valley_fog` |
| `post_bench PASS N` | repeat one post pass N times per frame for cost measurements (`lindepth`, `gtao`, `ao_blur`, `velocity`, `taa`, `motion_blur`, `dof`, `lum`, `shafts`, `bloom`, `tonemap`; `none` ends it) |
| `time Dawn\|Noon\|Sunset\|Night\|HOURS` | set the time of day (preset or 0–24 h); stops the time-lapse |
| `camera X Y Z TX TY TZ` / `camera follow` / `camera close` / `camera hood` | free camera at a pose looking at a target / back to the follow camera / the close follow camera / the hood camera (query `cam_mode`: 0 follow, 1 close follow, 2 hood, 3 free; `cam_behind` / `cam_height`: camera behind the rear bumper along the vehicle heading / above the body origin, m; `cam_road_ahead`: nearest ground point straight ahead of the front bumper that is in the view and not hidden by the vehicle's collision shape, m, -1 if none within 40 m) |
| `gamepad attach\|detach` / `gamepad axis leftx\|lefty\|rightx\|righty\|lt\|rt V` / `gamepad button south\|east\|west\|north\|lb\|rb\|back\|start 1\|0` | a virtual SDL gamepad that the input reads like a connected pad and that takes over from a real one (right-stick camera checks, `tools/scenarios/camera.txt`; map buttons, `tools/scenarios/map_check.txt`) |
| `tires offroad\|road\|slick` / `tires warm\|cold` / `tires ambient C` | fit a tire set through the garage parameter `tires.set` (rebuild in place, new tires at the air temperature) / put all four tires (surface and core) in the middle of the set's window or at the air temperature / set the air temperature live (`tools/scenarios/tires.txt`) |
| `wheels R W` | tire radius and width (m) on both axles, as a tire kit would set them; rebuilds the vehicle in place (`tools/scenarios/wheel_fit.txt`) |
| `camera vehicle side\|top\|front\|rear\|three_quarter\|three_quarter_rear\|roof [DIST] [FOV]` | free camera in the vehicle frame at DIST m (default 30) with FOV degrees (default: 6 m tall view); far + narrow = near-orthographic views (`tools/scenarios/car1_views.txt`) |
| `camera local X Y Z TX TY TZ [FOV]` | free camera with position and target in the vehicle frame (+X left, +Y up, +Z forward, m; FOV default 60 deg) for close-ups of body details |
| `menu intro\|main\|start\|pause\|resume\|settings\|back` | jump the menu flow: play the intro, open the start menu (its world and camera), start gameplay, open / close the pause menu, open Settings, go back one page |
| `menu select ITEM` / `click menu:ITEM` | move the menu cursor to an item (no sound) / click it with the mouse through the SDL queue. Items: `start`, `settings`, `quit`, `resume`, `restart`, `quit_to_menu`, `quit_game`, `quality`, `render_scale`, `grass_distance`, `tree_distance`, `vsync`, `master`, `music`, `effects`, `ui`, `abs`, `tc`, `diff`, `auto_reverse`, `controls`, `drive_keys`, `pad_glyphs`, `map`, `minimap`, `minimap_size`, `minimap_orientation`, `compass`, `back` |
| `controls keyboard mac\|pc\|native` / `controls pad_type none\|xbox\|ps4\|ps5\|switch\|unknown\|real` | Controls page captures only: force the key cap wording / stand in for the connected pad seen by the automatic button names (not persisted) |
| `menu garage` / `menu garage_close` | open the garage (from the start or the pause menu) / close it |
| `garage set KEY V` / `garage step KEY ±1` / `garage row KEY` | set a garage parameter (stored units, clamped) / step it / move the cursor to its row (`vehicle`, `setup`, `kit` or a parameter key) |
| `garage save` / `save_new` / `rename NAME` / `delete` / `select NAME\|factory` / `reset` / `kit ID\|none` / `vehicle ID` | setup operations of the garage model |
| `click garage:ROW[:F]` | click a garage row through the SDL queue (ROW: a parameter key, `vehicle`, `setup`, `kit`, `save`, `save_new`, `rename`, `delete`, `reset`, `back`); `:F` clicks a slider at fraction F (0..1) |
| `click modal:confirm` / `click modal:cancel` | click a button of the open modal dialog through the SDL queue |
| `transfer 2h\|4h\|4l` | select a transfer case mode (part-time 4WD) |
| `pedal throttle\|brake V` | hold an analog pedal at V (0..1) like a gamepad trigger (not ramped; max with the keys) until set to 0 |
| `perf_begin LABEL` / `perf_end` | collect frame times; prints `PERF LABEL fps frame_avg p95 p99 max cpu_avg gpu_avg …` |
| `vehicle ID` / `soc V` | choose a catalog vehicle as the vehicle page does (respawns in place on the next tick; not saved in scenario mode) / set the battery state of charge of an electric vehicle (0..1) |
| `vehicle_file PATH` | load a vehicle.json outside the catalog (e.g. `tests/data/vehicles/suv_glb_test/vehicle.json`, path relative to the working directory) and respawn it in place; the garage keeps its own vehicle, the next `vehicle ID` replaces it |
| `menu vehicles` / `click vehicle_card:N` | open the vehicle page from the start menu / click vehicle card N through the SDL queue |
| `level ID` / `place X Z [YAW]` | load a level (`exploration_peaks`, `suncoast_raceway`, `proving_ground`; not saved in scenario mode) / put the vehicle on the ground at X Z facing YAW degrees |
| `drift on\|off\|level` | force drift scoring on / off for the session, or back to the level's `"drift"` setting (same as F1 → Simulation → Drift scoring) |
| `autopilot drive\|off` / `reset` | the attract-drive autopilot drives the vehicle in gameplay (laps of a circuit) / gives it back to the input; press R (reset: on a circuit with a running lap to 4 m past the last gate passed) |
| `map on\|off\|fit\|center` / `mapclick X Z` / `mapaim X Z` / `mapdrag DX DY` / `marker X Z\|clear` | open (waits for the map build) / close the full-screen map, fit the whole map / center on the vehicle; left click at world point X Z through the SDL queue; center the view there (the crosshair point of Space / gamepad A); drag the map by DX DY points; set or clear the waypoint directly |

**Scenario clock.** While a script runs, every frame advances the game by a fixed 1/120 s (physics ticks, menu
animation, ImGui, `wait`, `until` timeouts), and frames are paced to never run faster than real time. A condition
reached in `until` is therefore acted on after the same simulated time on a 60 Hz or a 120 Hz display, with or
without VSync (before v0.14.0 the frame-counted input latency was ~67 ms at 60 Hz against ~33 ms at 120 Hz, enough
to move braking points). `perf_begin` … `perf_end` uses the real frame time, and FPS / frame statistics are always
real.

Scenarios start in gameplay (no intro, no menu) and never read or write the settings file; garage setups go to
`<tmp>/trailforge_scenario_setups`, emptied at start.

Other scenarios: `tools/scenarios/bump_check.txt` (every shipped vehicle accelerated to 30 km/h on the proving
ground's bump strip, teleport 16, coasting over a 0.35 m, 2.5 m long mound under both wheel tracks and then one under
the left wheels; no
rollover (`max_tilt` < 45°), upright on four wheels afterwards; `/tmp/trailforge_bump_*.png`),
`tools/scenarios/controls.txt` (Controls page: drive keys, gamepad button names auto / override,
no overlapping callouts, wheel scroll, values kept, then a drive with the arrow keys including the pedal ramp;
`/tmp/trailforge_controls_*.png`), `tools/scenarios/menus.txt` (intro, start menu, Settings, Controls, the blurred pause menu, reset
and quit to the start menu through keys and mouse clicks; the pause freezes ticks and position;
`/tmp/trailforge_menu_*.png`), `tools/scenarios/visual_tour.txt` (showcase screenshots `/tmp/trailforge_vis_*.png`),
`tools/scenarios/track_check.txt` (Suncoast Raceway: an autopilot lap through every checkpoint with sector times,
the stored best lap, R to the last checkpoint, the map, fixed cameras, frame times per preset;
`/tmp/trailforge_track_*.png`), `tools/scenarios/minitrack.txt` (mini handling track: teleport by name, the lap timer starting at the line, fixed
cameras on every section, the map; `/tmp/trailforge_minitrack_*.png`), `tools/scenarios/surfaces.txt` (asphalt rubber
marks after a locked-wheel stop on the mini track, gravel spray and displaced-gravel tracks in a trap;
`/tmp/trailforge_surfaces_*.png`),
`tools/scenarios/drift_check.txt` (drift
scoring on the mini track: rwd_test handbrake entry into the 32 m corner, a judged run, the `drift` switch on the
proving ground and on Suncoast Raceway; `/tmp/trailforge_drift_*.png`),
`tools/scenarios/time_of_day.txt` (Dawn/Noon/Sunset/Night and a banner-like sunset view, `/tmp/trailforge_tod_*.png`),
`tools/scenarios/perf_presets.txt` (frame times per quality preset),
`tools/scenarios/exploration_check.txt` (Exploration Peaks: spawn, a coast-road drive across a tile border with the
streaming numbers, a ford, the deep-water recovery, the play-limit warning, all 13 POI teleports, the map, the EV
on the level, overviews; `/tmp/trailforge_expl_*.png`),
`tools/scenarios/map_check.txt` (full-screen map, waypoint, compass distance while driving at it, virtual-pad map
controls, minimap options, the proving-ground map, minimap + compass cost; `/tmp/trailforge_map_*.png`),
`tools/scenarios/terrain_textures.txt` (terrain texture size per preset and Off, no reloads while driving, frame
times textured vs procedural per preset, `/tmp/trailforge_tex_*.png`),
`tools/scenarios/buildings_views.txt` (the village and every POI with buildings from fixed cameras, frame times on the
village per preset and in an Ultra stress pass, `/tmp/trailforge_bld_*.png`),
`tools/scenarios/grass_look.txt` (grass, flowers and wheat at noon and sunset, close-up and wide, flattened lanes in
the meadow and the wheat, overview per preset; `/tmp/trailforge_grass_*_after.png`),
`tools/scenarios/tracks.txt` (tire marks on concrete, dirt, mud and grass, `/tmp/trailforge_tracks_*.png`, and the
cost of the marks with the ring empty and filled to 16k / 64k segments),
`tools/scenarios/launch_marks.txt` (2H full-pedal launch, 2 s and 5 s burnouts on concrete with analog `pedal`
input, `/tmp/trailforge_launchmarks_*.png`),
`tools/scenarios/garage.txt` (garage from the pause and the start menu: keyboard, mouse slider click, rebuild in
place, RWD layout, save / typed rename / save as new / select / delete / reset, `/tmp/trailforge_garage_*.png`),
`tools/scenarios/audio_check.txt` (the situations the sound reacts to, checks the mixer parameters; run it with
`--audio-capture`, see [Audio](audio.md#audio)), `tools/scenarios/landing_audio.txt` (bumps and landings: quiet crawling,
body rattle and bump-stop knocks on hard hits, SUV and buggy; `--audio-capture`),
`tools/scenarios/suspension.txt` (stops on the diagonal twister with a wheel in the air; follow view with the HUD,
rear and side views, `/tmp/trailforge_v_susp_*.png`),
`tools/scenarios/tread_dirt.txt` (soil printed onto concrete after the mud bath and the sand, rolling and with
wheelspin, top-down `/tmp/trailforge_tread_*.png`),
`tools/scenarios/spray.txt` (wheel spray: nothing while rolling calmly, wheelspin on mud, dirt, sand and grass, a
puddle crossing, a locked-wheel slide, the cap; `/tmp/trailforge_spray_*.png`),
`tools/scenarios/spray_look.txt` (rear 3/4 view of mud wheelspin, body mud after 5 s of wheelspin, sand and grass
spray and wheel close-ups; `/tmp/trailforge_spray2_*.png`),
`tools/scenarios/skid_profile.txt` (a 65 km/h locked-wheel stop on the skid strip, top-down
`/tmp/trailforge_skid_profile*.png`),
`tools/scenarios/skid_check.txt` (tire squeal gating: silent for normal launches and turns on concrete, short for a
brake tap, audible for lock-up and handbrake slides, never while the mud layer is active; needs the mixer, i.e. a device or `--audio-capture`),
`tools/scenarios/ev_check.txt` (electric vehicle: vehicle page with keys, mouse wheel and card clicks, launch, lift-off
regen, blended braking, SoC and energy counters, drive modes, the low range message, reverse, an empty battery, the
mud bath in OFF-ROAD; `/tmp/trailforge_ev_*.png`), `tools/scenarios/ev_audio.txt` (EV launch, regen and stop for an
`--audio-capture` analysed by `tools/audio/analyze_capture.py`), `tools/scenarios/patrol_views.txt` (buggy views, full droop and full bump coil-over close-ups,
`/tmp/trailforge_patrol_*.png`),
`tools/scenarios/car2_views.txt` (EV model views
for comparison with Cars.png car #2, `/tmp/trailforge_car2_*.png`), `tools/scenarios/tires.txt` (tire sets and
temperature: cold slicks, warming on the mini track, slicks taken off warmers, the telemetry grid, the road set fitted
on a 35 C day; `/tmp/trailforge_tires_*.png`).

Variables: `speed_kmh`, `fwd_kmh`, `gear`, `rpm`, `dir`, `low`, `tilt`, `x/y/z`, `ticks`, `paused` (1 while the
simulation does not tick: pause menu or debug freeze), `frozen` (debug freeze, P), menus: `menu_screen` (0 intro,
1 start menu, 2 playing, 3 paused, 4 garage, 5 map), `menu_page` (0 start, 1 pause, 2 Settings, 3 Controls, 4 vehicle page), `menu_depth`,
`menu_cursor`, `menu_scroll` / `garage_scroll` (list offset in px), `menu_fade` (start menu fade-in 0..1), `menu_blur` (0..1), `menu_font`, `menu_logo` (assets loaded),
`setting_<key>` (the persistent settings: `quality`, `render_scale`, `grass_distance`, `tree_distance`, `vsync`, `master`, `music`, `effects`, `ui`,
`abs`, `tc`, `diff`, `auto_reverse`, `drive_keys`, `pad_glyphs`, `minimap`, `minimap_size`, `minimap_north_up`, `compass`, `vehicle` (catalog index of the remembered choice)), `settings_roundtrip` (1 if the settings file format
keeps every current value), controls: `drive_keys` (applied to the input: 0 W A S D, 1 arrows), `kb_style` (0 Mac,
1 PC), `pad_glyphs` (resolved: 0 Xbox, 1 PlayStation), `controls_callouts` / `controls_overlaps` (callouts on the
Controls page and overlapping pairs of callouts, key caps and controller outline, measured with the real font), garage: `garage_open`, `garage_cursor`, `garage_dirty`, `garage_setups`,
`garage_factory`, `garage_renaming`, `garage_modal` (0 none, 1 rename, 2 delete, 3 reset), `garage_modal_error`,
`garage_rename_len`, `garage_pending`, `garage_rate_front` / `garage_rate_rear` (wheel rate N/mm
shown next to the spring frequency), `garage:<key>` (the garage's value of a setup parameter),
`cfg:<key>` (the same parameter read from the running vehicle), `text_input` (SDL text input active), drivetrain:
`layout` (0 RWD, 1 FWD, 2 AWD, 3 part-time, 4 on-demand), `transfer` (0 2H, 1 4H, 2 4L), `front_driven`,
`rear_driven`, vehicles: `vehicle_index` (catalog index of the running vehicle, 0 = default), `vehicle_choice` (the
vehicle page's choice), `vehicle_cards` (cards drawn), `electric`, `direction` (0 drive, 1 reverse), electric drive:
`soc` (0..1), `motor_kw` (both motors, negative = regen), `battery_kw` (with the auxiliary load), `motor_nm_f` /
`motor_nm_r`, `motor_rpm_f` / `motor_rpm_r`, `regen` (0/1), `ev_mode` (0 NORMAL, 1 OFF-ROAD), `ev_front_share`,
`energy_out_kwh` / `energy_in_kwh` (battery energy out / in since the spawn),
`cam_free`, `ui_visible`, `ui_kb`, `ui_text`, `static_grip`, `clutch_limit` (Tuning toggles as applied to the vehicle), `air`, `max_air`, `max_tilt` (largest body tilt from upright since `reset_stats`, deg), `susp_spread` (longest minus shortest suspension length, m), `susp_min` / `susp_max` (shortest / longest of the four), `body_glb` (1 when the running vehicle has a body.glb), `body_parts` / `body_triangles` (drawn body model), `body_hulls` (body.glb collision pieces), `lowgrip_wheels`, `max_slip`, `load_sum`, `surface` (`SurfaceId` under most wheels in contact, −1 in the air),
`min_cam_clearance`, `front_spring`, `mass`, `depth_w/h`, `depth_creations`, `depth_matches_window`,
`auto_reverse`, `hint` (1 while the driving hint is on screen), driver aids: `abs` (0/1), `tc` (0 off, 1 on,
2 off-road), `diff_locks` (0–3), `front_diff` / `center_diff` / `rear_diff` (effective mode: 0 open, 1 LSD,
2 locked), `abs_seen` / `tc_seen` (the aid acted in a sampled frame since `reset_stats`), `max_locked` (most locked
wheels in one frame) and `lock_time` (locked wheel-seconds; locked = in contact, ground speed > 2 m/s, wheel surface
speed below 20 % of it),  `win_w/h`, `px_w/h`, `fps`, `frame_ms`, `cpu_ms`, `gpu_ms`, `quality`, `msaa`, `render_w`, `draw_calls`,
`grass_blades`, `reflection`, `mud`, `tod`, `tod_run`, `sun_el`, `sun_az` (deg), `trees`, `buildings` (placed on the
level), `model_instances` / `model_tris` / `model_draws` (level models drawn, main view), `marks` (live segments),
`marks_cap`, `marks_written`, `marks_drawn`, `marks_us` (CPU per tick), `marks_s<N>` (live segments on SurfaceId N:
0 Concrete, 1 PackedDirt, 2 Rock, 3 LowGrip, 4 Mud, 5 Grass, 6 Sand, 7 Snow, 8 Asphalt, 9 Gravel), `tread_soil_max` (most soil on any tread, 0..1),
`spray` (live spray particles), `spray_emitted`, `spray_drawn`, `spray_max`, `spray_fine` (share of fine bits),
`body_mud_max`, `body_mud_<region>` (`arch_rear_<w>`, `arch_front_<w>`, `behind_<w>` with `w` = fl/fr/rl/rr,
`rear_bumper`, `front_bumper`), `wheel_width_r`, `wheel_visual_radius_r` / `wheel_visual_width_r` (measured rear
wheel mesh), `wheel_fit_err` (largest mesh vs physics radius / width difference over both axles, m),
`msg:<substring>` (1 if the latest UI message contains the substring), tires: `tire_set` (0 offroad, 1 road,
2 slick), `ambient` (air temperature, C), `tire_surface_<w>` / `tire_core_<w>` (C), `tire_grip_<w>` (temperature
grip factor), `tire_kpa_<w>` (gauge pressure) with `w` = fl/fr/rl/rr, `tire_surface_max` / `tire_surface_min`,
`tire_hud` / `tire_temps` (1 while the HUD widget / the telemetry grid is drawn),
levels: `level` (1 on an open-world level, 0 on the proving ground), `level_choice` (index of the remembered
choice in the level list), `submersion` (m below the water surface), `recoveries`,
`water_depth` (render-only water over the ground at the vehicle), `above_grid` (body height over the level's height
grid; static scene meshes carry the vehicle above it), `wheels_contact`, `road_dist`, `terrain_chunks`, `terrain_tris`,
`tiles_loaded`, `tiles_async`, `tiles_sync`, `tiles_unloaded`, `tile_build_ms`, `tile_stall_ms`, `tile_update_ms`,
`tile_x` / `tile_z`, `grass_dist` / `tree_detail` / `tree_dist` (applied render distances, m); `set` keys `tree_dist` (m) and `terrain_detail` (LOD scale),
lap timer (proving ground mini track): `lap_running`, `lap_count`, `lap_time` (current lap, s), `lap_last`,
`lap_best`, `lap_next_gate` (0 = start / finish, 1 and 2 = sector gates),
map: `map_open`, `map_ready`, `map_build_ms`, `map_pois`, `map_scale` (points per m), `map_cx` / `map_cz` (view center),
`marker_set`, `marker_x` / `marker_z`, `marker_dist` (m, horizontal), `marker_bearing` (deg, relative to the heading, + = right),
`heading` (deg clockwise from north), attract drive: `autopilot` (0 off, 1 drive, 2 hold), `autopilot_driven` (m),
`autopilot_progress`, `autopilot_lateral` (m from the route), `autopilot_resets`,
lap timing (Suncoast Raceway, the mini handling track): `lap_active`, `lap_running`, `lap_count`, `lap_time` (running
lap, s), `lap_last`, `lap_best` (session), `lap_stored_best` (best_laps.cfg, this level and vehicle), `lap_saved`
(entries in it), `lap_best_gap` (best − last), `lap_stored_gap` (stored − session best), `lap_next` / `lap_next_gate`,
`lap_last_gate`, `lap_checkpoints`, `lap_gates`, `lap_gate_dist_N` (m from gate N), `lap_sectors`, `lap_sector_N`
(last lap, 1-based; a scenario stores its best laps in `<tmp>/trailforge_scenario_best_laps.cfg`, emptied at start),
drift scoring: `drift_active`, `drift_running`, `drift_drifting`, `drift_in_area` (inside the course area), `drift_angle` (body slip, deg), `drift_score`
(running run), `drift_max_angle`, `drift_zone` (multiplier of the zone at the vehicle, 0 outside), `drift_zone_time`,
`drift_runs`, `drift_failed`, `drift_judged` (runs + failed), `drift_last`, `drift_best` (session), `drift_end` (last
run: 0 none, 1 completed, 2 left the course, 3 spin, 4 wall hit, 5 reset), `drift_stored_best` (best_drift.cfg; a
scenario uses `<tmp>/trailforge_scenario_best_drift.cfg`, removed at start),
audio: `audio_active`, `audio_t` (captured seconds), `audio_src_rpm`, `audio_load`, `audio_skid`, `audio_gravel`,
`audio_dirt`, `audio_mud`, `audio_grass`, `audio_wind`, `audio_motor_hz` (electric motor order), `audio_motor`,
`audio_inverter`, `audio_road` (mixer levels), `audio_day`, `audio_night`, `audio_gameplay` (gain of the
gameplay sounds, 0 while paused), `audio_music` (music bus gain 0..1), `audio_music_t` (seconds of music played), `audio_voices`, `audio_oneshots`, `audio_peak_db`, `audio_clipped`, `audio_cpu`, `audio_ev<N>` (events
received by the mixer: 0 engine start, 1 gear, 2 bump-stop knock, 3 body rattle, 4 body impact, 5 splash, 6 creak,
7 UI), `audio_rattle` (body shake envelope 0..1), `audio_knocks` (synthesized knocks started), and more. Full list: `Application::scenarioValue`.
Driver aid `set` keys: `abs 0/1`, `tc 0/1/2`, `diff_locks 0..3`.
Audio `set` keys: `audio_ui` (panel 0/1), `audio_mute`, `audio_master`, `audio_pitch`, `audio_whine`, `audio_motor`,
`audio_play_ui N` (play UI sound N: 0 click, 1 hover, 2 confirm, 3 back, 4 slider tick, 5 menu open, 6 menu close).

## Diagnostics (F1)

- **HUD** (always visible, `set hud 0` hides it): top left — speed, gear, direction, rpm, suspension compression per
  wheel laid out as seen from above ("air" without contact, red on the bump stop), controls; bottom center — a
  driving read-out with the selector (D / R, N while the gearbox is in neutral), speed in km/h, gear, range
  (HIGH / LOW) and rpm, the ABS / TC lamps and the differential modes, plus a 2.5 s hint above it after a discrete
  command ("Reverse engaged", "Stop to change direction", "Transfer case range changed", "ABS on", …). Electric
  vehicles show the drive mode (NORMAL / OFF-ROAD), a SoC bar, the motor power in kW (negative and blue = regen) and
  the front / rear motor torque instead of gear, range and rpm; the Telemetry window adds per-motor rpm, torque and
  power, battery power with the auxiliary load and the energy used / recovered.
- **Telemetry:**
  - FPS and frame time; ticks per frame, physics step time (last and average), dropped time;
  - speed, rpm, gear, clutch, range, body tilt, throttle/brake/handbrake/steering bars;
  - per-wheel table: contact, surface, suspension length and compression (m and %), load, longitudinal
    and lateral forces, slip ratio, slip angle, μ, wheel angular velocity, brake torque, ABS pressure, TC brake;
  - driver aids line: ABS / TC state (and whether they act), TC throttle factor and the throttle passed to Jolt,
    effective differential modes.
- **Tuning:**
  - springs, dampers, anti-roll bars, tire curves, surface coefficients, engine, brakes
    and differentials are applied between physics steps;
  - toggles for the two tire/drivetrain fixes, also applied between steps: Tires → "Static grip" (`tireStaticGrip`)
    and Drivetrain → "Launch clutch limit" (`launchClutchLimit`) with the creep torque; Tires → "Slip speed floor"
    and "Friction ellipse (combined slip)";
  - Drivetrain: Open / Limited slip / Locked per differential, axle and center LSD ratios, the locked ratio;
  - Driver aids: ABS / TC / diff lock switches (same as B / T / X) and every ABS / TC parameter;
  - mass, inertia, center of mass, suspension travel, wheel radius, steering angle and gear ratios are applied
    with the **Rebuild vehicle** button (the vehicle is recreated);
  - physics rate, collision steps, input smoothing, debug lines, teleport.
- **Debug lines:**
  - suspension axis with min/max marks and current wheel position (red — on the bump stop);
  - contact point, normal, load bar;
  - longitudinal force (red) and lateral force (blue).

About units. Jolt reports constraint lambdas, which are **impulses in N·s over the last collision substep**.
`PhysicsSystem::Update` splits the step into `collision steps` substeps, so the UI computes forces as
`F = λ / (dt / collisionSteps)` (N). Jolt defines slip ratio as `|(ωr − v)/v|`. Below 0.5 m/s
it is numerically undefined, and the table shows "-".

Why the "Load" sum can exceed m·g while driving. Load is the suspension force along the contact normal,
and it includes bump stop hits (MaxUp). On a flat pad in a steady-state corner the average Σload = m·g
within 1 N (test `corner_load_balance`, with 1, 2 and 4 substeps). On the course (`trailforge_probe demo`)
the 1 s average reaches 113–130 % of m·g, while the vertical projection of the forces Σ(F·ŷ) exactly equals m(g + a_y).
The difference has three causes: normals are tilted on slopes, bumps produce vertical accelerations,
and bump stop peaks reach ~330 kN per substep. This is physics, not a conversion error.

## Verification results

Everything below was actually run on macOS 27 / M-series.

- **Build.** The `debug`, `release` and `headless` presets build without errors.
  The HLSL shaders are compiled to SPIR-V and metallibs at build time and embedded into the binary.
  At startup the log shows `GPU driver 'metal', shaders: metallib (precompiled)`; with `SDL_GPU_DRIVER=vulkan`
  (MoltenVK) `GPU driver 'vulkan', shaders: SPIR-V`.
- **CTest.** 75 of 75 tests pass in Debug and Release with the buildings (new: `building_kit_parts`,
  `building_kit_materials`, `level_buildings`); 69 of 69 as of v0.15.0; 56 of 56 in headless (no model tests) as of
  v0.12.0 (new in
  v0.12.0: `level_heightfield_sampling`, `level_tile_streaming`, `level_road_carving`, `level_surfaces_water`,
  `level_vehicles` in the new `trailforge_level_tests`; new in v0.11.0: `electric_definition_validation`, `ev_motor_curve`, `ev_torque_split`, `ev_regen_blend`, `ev_battery`,
  `ev_simulation`, `audio_electric`, `vehicle_page`; 30 before v0.8.0, then `vehicle_catalog_sweep`,
  `vehicle_definition_validation`, `drivetrain_layouts`, `garage_setups`, `suv_definition_matches_builtin`,
  `garage_model` and later ones) (numbers below are from the latest run where they changed):

| Test | Result |
|---|---|
| Settling on suspension | No NaN; load sum 19623 N with m·g = 19620 N |
| Throttle | 0 → 16.0 m/s in 4 s |
| Braking | 16.0 → 9.0 m/s in 1 s, then a full stop |
| Wheel over a pit | FL has no contact, load 0, the others carry the weight |
| Reset | Gear 0, 800 rpm, initial pose |
| Frame schedule independence | Different frame patterns at the same step rate: position difference 0.000000 m |
| Cornering load balance (40 km/h) | Average Σload = 19620 N with 1, 2 and 4 collision steps; outer front > 2× inner |
| Brake in low range | 11.7 m/s → stop in 1.59 s, speed never increases, final 0 m/s |
| Tire marks per surface | Rolling 30 km/h on concrete: 0 segments; full brake: 82 segments, slip 1.00; rolling on dirt/mud/grass: 480 segments each, imprint 0.85/1.00/0.90; low-range power turn: 95/66/85 segments, slip 0.35/0.95/0.76; cap 256 holds; with fade 3 s none are left after 3.5 s parked |
| Auto reverse | At 60/120/240 Hz S held from 13.8 m/s stops the vehicle in 1.86–1.90 s and keeps holding it (no switch for 4.25 s, 0.000 m/s); released and pressed again, reverse engages after 0.250–0.254 s (hold 0.25 s), −12.9…−13.1 m/s after 3 s; W the same way back to drive (+14.2 m/s); low range reverse −4.8 m/s after 3 s; reverse again right after R; W+S never switches; classic mapping holds; on the 12° hill S held from the climb holds the vehicle (0.8 mm in 6 s), pressed again it engages reverse and the vehicle backs down 6.9 m in 2 s |
| ABS prevents lock | 50 km/h full stops on dirt and grass at 60/120/240 Hz: without ABS 7.8–11.5 locked wheel-s and −9.5…−13° heading with 40 % steer; with ABS 0–0.13 wheel-s at 120/240 Hz (0.72 / 0.13 at 60 Hz) and −54…−76°; straight stop dirt 18.5 → 16.5 / 16.0 m at 120 / 240 Hz (19.9 m at 60 Hz), grass 25.4 → 22.1 / 21.6 m |
| Diff locks climb | 15° slope, FL over a pit, all diffs open: −0.22 m in 8 s, hanging wheel up to 12.5–14.5 m/s; all locked (X × 3): 30–55 m (60/120/240 Hz) |
| Traction control on mud | Flat mud launch, 3 s: max slip 9.3–9.5 m/s and 2.8 s of spin with TC off; On 2.7–6.9 m/s (the 60 Hz peak is a single tick), 1.4 s, v(3 s) 7.2–7.8 vs 8.0–8.1 m/s; Off-road 4.4–5.5 m/s |
| Driving aids logic | Synthetic wheel states, no Jolt: a locked wheel under full pedal is dumped to 0 N·m in 0.025 s while the rolling wheels keep 3000 N·m, full pressure 0.125 s after it rolls again; one spinning wheel gets 1500 N·m of TC brake and no throttle cut; all four spinning cut the throttle factor to 0.05 in 1 s; rolling back it recovers 0.05 → 0.35 in 0.5 s; TC off passes the pedal through |
| Friction ellipse | Full brake with 40 % steer from 50 km/h on concrete without ABS, after 1 s (ellipse off → on): heading −32.4° → −20.2°, sideways 2.19 → 0.64 m; straight stop 13.63 → 13.63 m; 30 % steer circle at 40 km/h 8.29 → 8.29 m/s² |
| Audio: engine layers | Equal-power band weights (Σw² = 1 within 2.4e-7), crossfade midpoints 0.7071, playback rates follow rpm × pitch / rpm_recorded |
| Audio: tire layers | Concrete lock-up skid 1.0, 12 % traction slip 0.0; rolling dirt 0.75 / mud 0.62 / grass 0.62; wheels in the air 0 |
| Audio: manifest | All 55 sounds present at 48 kHz, 12 engine loops with recorded rpm and level, licenses 54 CC0 + 1 other |
| Audio: feed and mixer | Simulated launch: 3 gear events, source rpm up to 3028; concrete skid 0.000 cruising vs 1.00 braking; mud lane mud 0.66; pause RMS 0.1486 → 0.0497; peak 0.660, 0 clipped |
| Audio: skid gating | Built-in vehicle at 60/120/240 Hz on concrete: launches and full-lock / half-lock turns 0; handbrake 0.64, full brake 1.00, full-lock power slide (front plow) 0.18–0.19, 4–5 onset chirps. Shipped SUV (2H, ABS off) on the pad: full-throttle launch 0.000 although the rears spin until 2.18 s, half-throttle launch, 0.3 brake and a 0.25 s full brake tap 0.000; 0.5 s full brake tap 0.55, silent 0.19 s after the slide; full brake 0.50, handbrake turn 0.63, both silent before the slide ends; mud lane: skid 0, mud 1.00 |
| Audio: tire profiles | Real squeal recordings, one wheel at 6 m/s, full squeal, 1.6 s window: sporty −26.0 dBFS, centroid 1473 Hz, <1 kHz 27.9 %, >3 kHz 6.7 %; trailforge_suv −26.6 dBFS, 1048 Hz, 44.3 %, 0.8 % (without scrub −28.5 dBFS, 1242 Hz); output below −60 dBFS ≤ 0.150 s after the slide |
| Audio: squeal release | Mixer with squeal sounds of their real length: 0.3 s at squeal 1.0 → peak −26.2 dBFS, one onset chirp, output below −60 dBFS 0.150 s after the slide (the 1.7 s chirp fades with it); at 0.25: −37.0 dBFS, no chirp, tail 0.092 s |
| Surface launch grip | Full-throttle high-range launch at 60/120/240 Hz: concrete v(1 s) 5.1 m/s, max slip 0.27 / 0.11 m/s at 120 / 240 Hz (3.3 m/s for 0.03 s at 60 Hz); grass and mud spin (max slip 7.2–9.5 m/s for 1.7–1.8 s), v(1 s) 3.4 / 2.4 m/s stays below μ·g; v(2 s) orders concrete > dirt > grass > mud |

- **Step cost** (`trailforge_probe bench`, Release, 20 s course drive):
  - 60 Hz: 0.038 ms per step on average, worst 0.11 ms; 2.3 ms CPU per simulated second;
  - 120 Hz: 0.035 ms, worst 0.075 ms; 4.2 ms;
  - 240 Hz: 0.031 ms, worst 0.10 ms; 7.4 ms.

  All values are finite (no NaN). Trajectories differ between rates.
- **Application launch.**
  - `--quit-after` smoke test in Debug and Release: 115–121 FPS with VSync, 120 physics ticks per second,
    0 ms dropped time.
  - Frames via `--capture`: proving ground, vehicle with wheels, suspension and force debug lines, ImGui windows.
  - In the 40 km/h demo corner the body rolls 8.5°, the outer wheels compress 77–79%,
    and the inner front wheel unloads to 0 N.
- **Scenario `tools/scenarios/full_check.txt`** (window, rendering, input via the SDL event queue → `Input` → ticks):
  206 checks, 0 failures (since the driver-aids stage; 162 before), about 150 s per run. Covered:
  - throttle and braking; D/A steering and smooth return to center;
  - auto reverse: S held from moving brakes and keeps holding at the stop; released and pressed again it engages
    reverse and drives backwards, W the same way back to drive; the same in low range, right after R, on the 28° hill
    and on the low-grip patch; the hint after Q while moving. The climb brake holds run with auto reverse on, the
    other stops with `set auto_reverse 0` (classic mapping);
  - driver aids: B / T / X cycle ABS, TC and the diff locks (effective modes checked); a 45 km/h stop on the dirt
    lane locks wheels without ABS (> 3 wheel-s) and not with it (< 0.5 wheel-s); TC acts on the low-grip patch;
  - Q and L are refused with a message while moving and work when stopped; reverse; handbrake; R reset;
  - C camera and free camera with arrow keys; F1; Esc pause (ticks stop), N — exactly one step;
  - ImGui input capture: with focus in a field, W and R do not reach the vehicle; typing `2.2` into the spring slider is applied;
  - 12/20/28/35° climbs in low range: after each, a stop where the brake holds (drift < 5 cm);
    on 35° the body tilt stays below 45°, and the camera does not go underground;
  - side slope, diagonal wheel lift (wheel in the air), 0.15/0.30/0.45 m steps, rocks;
  - low-grip section: all 4 wheels on it, noticeably weaker braking, reverse;
  - camera orbit down and zoom: camera clearance > 0.1 m;
  - resize via `SDL_SetWindowSize` to 900×560, 320×200, 1600×1000 and 1440×900:
    the depth texture is recreated each time and matches the swapchain size;
  - Rebuild vehicle with 2600 kg mass: the mass is applied, the live spring tuning is kept, Σload ≈ m·g.

  Frames: `/tmp/trailforge_01_start.png` … `/tmp/trailforge_22_rebuilt.png`.
  After the rendering stage: 128 / 0 again in Release and Debug (GPU debug layer), ctest 8/8 in both.
- **Rendering performance** (`tools/scenarios/perf_presets.txt`, Release, 1440×900 pt window = 2880×1800 px
  output, showcase follow camera and an overview camera, UI hidden, 4–5 s per sample).
  Best (least disturbed) sample per preset and view across four runs, VSync off:

| Preset | Render size | GPU ms (follow / overview) | FPS (follow / overview) | CPU ms | Grass blades drawn |
|---|---|---|---|---|---|
| Low | 1930×1206 | 3.2 / 3.1 | 264 / 314 | 0.5–0.7 | 1–2k |
| Medium | 2448×1530 | 4.0 / 3.3 | 206 / 272 | 0.6–0.8 | 4–7k |
| High | 2880×1800 | 3.9 / 3.8 | 271 / 266 | 0.8–0.9 | 16–34k |
| Ultra | 2880×1800 | 5.4 / 6.0 | 219 / 205 | 0.9–1.0 | 60–93k |

  With VSync on, High and Ultra averaged 58.4–60 FPS in these runs (frame time p95 ≈ 17.2–17.5 ms); during
  `full_check` (VSync on, UI visible) the telemetry showed 116–120 FPS, so the presentation rate itself varies
  between 60 and 120 Hz on this setup (not investigated).
  The machine was shared with other load (a VM, media analysis, WindowServer ≈ 37 % CPU), and with VSync off macOS
  still paced the background window: across four runs the same preset measured anywhere from 60 to 270 FPS,
  frame counts quantized at 60/120 Hz. Treat the FPS column as an upper bound and the GPU time as the cost;
  one contended Ultra+VSync run dropped to 51 FPS average with a 316 ms hitch.
  Showcase frames: `tools/scenarios/visual_tour.txt` → `/tmp/trailforge_vis_01_follow.png` … `07_muddy_side.png`.
- **Time of day and environment cost.** The previous commit (`4826485`) and the new build were run back-to-back
  with `perf_presets.txt` in the same session (default time 9:00, VSync off, GPU ms average, follow / overview):

| Preset | Before | After |
|---|---|---|
| Low | 4.10 / 3.92 | 4.04 / 4.21 |
| Medium | 4.24 / 3.89 | 4.29 / 4.01 |
| High | 4.56 / 4.64 | 5.13 / 4.89 |
| Ultra | 6.46 / 7.17 | 6.86 / 7.50 |

  Run-to-run noise in that session was ±0.3 ms; absolute numbers are higher than the table above because the display
  ran at 60 Hz that day (VSync runs at 58–59 FPS) and the GPU clocked lower. Draw calls 46 → 49 (follow), 10 → 13
  (overview): the pines are three instanced draws instead of being part of the scenery mesh.
- **Tire marks** (`tools/scenarios/tracks.txt`, Release): 18 checks, 0 failures — launch marks on the skid strip,
  no new concrete marks while rolling in 2nd gear, > 40 brake segments, dirt/grass marks when leaving the strip,
  mud ruts, low-range spin on grass and dirt, a handbrake drift on the start pad, 64k segments live and drawn.
  `full_check.txt` stays at 128 / 0 and `time_of_day.txt` at 8 / 0. Cost at 2880×1800, High, VSync off, mud showcase
  follow camera (one session, averages):

| Marks | GPU ms | Frame ms | CPU ms | Grass blades |
|---|---|---|---|---|
| Off | 4.09 | 3.89 | 0.93 | 29k |
| 16,384 live (default cap, synthetic fill) | 4.40 | 4.00 | 0.91 | 21k |
| 65,536 live (synthetic fill) | 5.98 | 4.81 | 1.00 | 18k |

  GPU time is the whole frame; the marks pass was not timed separately. Fewer grass blades are drawn with marks
  because torn turf kills blades, which partly offsets the marks cost.
- **Audio** (`tools/scenarios/audio_check.txt`, Release): 30 checks, 0 failures, both with the Core Audio device and
  with `--audio-capture`. `full_check.txt` 128 / 0, `time_of_day.txt` 8 / 0 and `tracks.txt` 18 / 0 both with audio
  on (device open) and with `--no-audio`. Analysis of a 53 s capture (`analyze_capture.py`):
  - sample peak −4.5 dBFS, true peak −4.3 dBTP, 0 clipped samples, limiter never engaged, integrated −23.0 LUFS
    (loudness range 9.7 LU), DC offset −0.000002 (−0.005 before the DC blocker: the exhaust-mic loops carry DC);
  - engine pitch: during the full-throttle run on the skid strip (1290 → 4800 rpm, three gears) the fundamental
    found near the expected firing frequency (played rpm / 30, 32–128 Hz) is within 5 % in 98 % of 120 ms windows,
    median error −0.3 %, correlation 0.999. An unconstrained search (15–250 Hz) lands on the crank frequency
    (rpm / 60) in 84 % of the windows: the exhaust recording has a strong half order;
  - braking on concrete raises the 1–6 kHz band by 11.7 dB over the run just before (mixer skid up to 0.84);
  - day → night (engine idling in both): the night bed is 7.1 dB louder in 1–3 kHz and 4.6 dB in 3–6 kHz (steady
    cricket lines in the spectrogram), the day bed shows short bird chirps instead;
  - audio CPU 2.3 % of real time in capture (≈ 190 µs per 400-frame tick), 20 voices at most.
  Nobody has listened to the result yet: everything above is measured, not judged by ear.
- **Tire sets and temperature** (`trailforge_probe tires`, Release, 20 C air; v0.26 era). 60-0 km/h with ABS on,
  tires at 20 C / in the middle of their window:

  | SUV | asphalt | concrete | gravel | dirt | grass | mud |
  |---|---|---|---|---|---|---|
  | OFF-ROAD | 15.69 / 15.69 m | 16.37 / 16.37 | 23.50 / 23.50 | 22.65 / 22.65 | 29.94 / 29.94 | 41.09 / 41.09 |
  | ROAD | 14.97 / 14.03 | 15.65 / 14.57 | 30.08 / 27.77 | 26.45 / 23.87 | 42.07 / 40.24 | no run-up |
  | SLICK | 18.42 / 13.32 | 19.95 / 13.54 | no run-up / 32.76 | 47.81 / 29.33 | no run-up | no run-up |

  "No run-up": the 2H SUV did not reach 60 km/h within 2.5 m of the lane line. EV slicks: asphalt 17.48 / 13.06 m,
  gravel 49.24 / 30.71 m, mud — / 108.32 m. Racing-autopilot laps from cold tires, best clean flying lap: mini track
  SUV OFF-ROAD 59.35 s, ROAD 56.80 s, SLICK 59.17 s (grip x0.67 at a 41 C tread after 5 laps); EV 58.56 / 55.32 /
  52.47 s (the EV slick lap is a single clean lap at budget 9 after a lap off the track that heated the treads to
  65 C average). Suncoast Raceway SUV 112.27 / 107.55 / 112.18 s, treads 30–35 C after 3 laps. Slicks taken off
  warmers (95 C) cool to a 52–62 C tread average within one lap. Off-road identity: `brake`, `compare`,
  `surfacecmp`, `surfaces`, `tc`, `lap` and every clean `minitrack` row print exactly what they printed before the
  tire model; `abs` (mud, dirt and grass lanes, runs cut for leaving the lane) and unclean `minitrack` rows differ
  because sustained wheelspin there heats a tread past 100 C, where the off-road compound loses grip too (with the
  grip factor pinned to 1 the outputs are identical).
- **Tire thermal calibration** (`trailforge_probe tires`, Release, 20 C air, 2026-10-27; literature coefficients, ABS / TC
  slip targets per set, see [Tire sets and temperature](vehicles.md#tire-sets-and-temperature)). Grip use of the limit driver (`tires limit`: lateral budget =
  k × g × the μ the tires offer at that moment, best clean k): 9–41 % of a clean lap with a tire at ≥ 0.8 of its peak
  slip. On the 40 m limit circle (`tires circle`, 150 s from 20 C) tread average / hottest: SUV OFF-ROAD 49 / 66 C,
  ROAD 57 / 85 C, SLICK 66 / 93 C; EV 66 / 90, 72 / 106, 79 / 124 C (before: SUV 70 / 101, 69 / 110, 65 / 91 C).
  Fixed-budget laps from cold, mini track after 5 laps: SUV OFF-ROAD 41.7 C, ROAD 45.6 C, SLICK 47.5 C (grip x0.72),
  EV SLICK 61.4 C (x0.86); best clean flying laps SUV 59.35 / 56.80 / 58.86 s, EV 58.56 / 55.03 / 55.06 s; Suncoast
  SUV 112.27 / 107.53 / 108.12 s, SLICK 47.0 C after 3 laps. Slicks off warmers: 67.8–77.9 C after one lap (before
  52–62 C); slow lap (budget 3 m/s²) from 95 C ends at 68.5 / 70.0 C (before 54.7 / 53.9 C). 60-0 warm slick SUV
  13.30 m (1.06 g): the front brakes are at full pressure with 0.016 slip, i.e. the SUV's brake torque is the limit,
  not the ABS (×1.5 brake torque gives 11.71 m, 1.21 g). Off-road: `brake`, `compare`, `surfacecmp`, `surfaces`, `tc`,
  `lap`, `minitrack` print exactly what they printed before the tire model; `abs` differs only on the dirt 100 km/h
  run-up (it does not reach 100 km/h in any version; its tread passes 100 C).
- **Found and fixed during this verification:**
  - with `collision steps` > 1 the UI forces were N times too low: the lambda is the impulse of the last substep,
    but it was divided by the full step;
  - the brake could not stop the vehicle in low range: Jolt's automatic keeps the clutch engaged in gear,
    and the idling engine pulled through the clutch harder than the brake. Now, when braking without throttle,
    the gearbox shifts to neutral near a stop and holds the current gear without shifting while moving. Braking on flat ground:
    16.1 → 8.1 m/s in 1 s (was 8.3); in low range, a stop in 1.33 s (was 1.99 s);
  - the "Steps" teleport point was on the run-out of the side slope, and the vehicle drifted off course — moved;
  - `Input` read held keys from `SDL_GetKeyboardState`, so synthetic events did not act as held keys.
    Held keys are now tracked from events;
  - the unit label in the telemetry table was corrected to "/ collision substep".
- **Observations (not bugs, but worth knowing):**
  - the 0.45 m step is higher than the wheel radius (0.4 m). In high range the vehicle stalls against it; in low range it climbs from a standstill.
    The climb is borderline: in one Debug run, where the vehicle stopped slightly closer to the step,
    it spun its wheels for about 6 s before climbing;
  - on 35°, cresting at full throttle can roll the vehicle over (roll is not limited);
  - there is no rolling resistance (except on gravel, added later), so after releasing the brake the vehicle rolls a few centimeters;
  - in low range the automatic reaches 5th gear at about 49 km/h;
  - once Rebuild did not trigger, and once a second C press was lost. Neither reproduced in isolation.
    Suspected cause: the real mouse cursor — the ImGui SDL3 backend substitutes the global cursor position when the window
    is focused but the mouse is outside it. The scenario now checks `ui_kb == 0` before the second C, with no failures since.
- **Not verified by hand.** Accessibility and Screen Recording permissions for Cursor (computer use) were disabled
  in this session, so the following were not tested with real hands:
  - keyboard and mouse in the live window (synthetic SDL events take the same code path,
    but not the system HID or macOS focus handling);
  - gamepad (not connected);
  - manually dragging the window edge (resize was verified only via `SDL_SetWindowSize`);
  - subjective handling feel;
  - how the sound actually sounds (only measured from captures and spectrograms).
