![TrailForge](Banner.png)

# TrailForge — off-road simulator prototype

A small custom engine in C++20: SDL3 (window, input, gamepad), SDL GPU (rendering via Metal), Jolt Physics
(`VehicleConstraint` + `WheeledVehicleController`), Dear ImGui (diagnostics), GLM. The focus of the prototype is
vehicle behavior: mass, suspension travel, wheelspin, wheel lift, rocks, climbs.

All geometry and textures are procedural. The only external assets are the sounds in `assets/audio/` (Freesound
recordings, see [Audio](#audio) and `assets/audio/CREDITS.md`).

![Showcase](docs/showcase.jpg)

Rendering (Metal only, see [Rendering](#rendering)): PBR, cascaded sun/moon shadows, a time of day (0–24 h, presets,
time-lapse) that drives the sun, sky, fog and ambient light, procedural sky with clouds, a large sun disc and layered
distant mountains, height fog, HDR with ACES tonemapping and bloom, MSAA or FXAA, instanced grass and pines with LOD,
planar puddle reflections, headlights that light the ground at night. A small mud & puddles showcase meadow sits north
of the proving ground.

## Platforms

Verified only on **macOS 27 / Apple Silicon (arm64), Apple clang 21, Metal**.
Shaders are written only in MSL, so other platforms (Windows/Linux, Vulkan/D3D12) are not supported.
The physics core (`trailforge_sim`) and the tests do not depend on graphics and can be built separately
(`headless` preset), but this has not been verified outside macOS either.

## Prerequisites

- Xcode Command Line Tools with the Metal toolchain (`xcrun -sdk macosx -f metal` must find the compiler).
  Without it the build still succeeds: the shader is compiled from the embedded MSL source at startup.
- CMake ≥ 3.24 and Ninja: `brew install cmake ninja`.
- Internet access for the first configure. Dependencies are downloaded via FetchContent as archives
  of exact commits with SHA256 verification:

| Dependency | Version | Commit |
|---|---|---|
| SDL | release-3.4.18 | `829a65d7` |
| Jolt Physics | v5.6.0 | `e77f1755` |
| Dear ImGui | v1.92.9b | `f1cc2ae1` |
| GLM | 1.0.3 | `8d1fd52e` |
| miniaudio | 0.11.25 | `9634bedb` |

## Build and run

```sh
cmake --preset release
cmake --build --preset release
./build/release/trailforge

# debug build (SDL GPU debug layer enabled)
cmake --preset debug && cmake --build --preset debug && ./build/debug/trailforge

# tests
ctest --preset release        # or debug

# physics + tests only, without SDL/rendering
cmake --preset headless && cmake --build --preset headless && ctest --preset headless
```

App icon (`Icon.png`, macOS): the build converts it with `sips` to a 256×256 BMP that is embedded into the binary;
at startup `SDL_SetWindowIcon` sets it (on Cocoa this is `NSApp.applicationIconImage`, i.e. the Dock icon of the
running `trailforge` process), masked to the macOS rounded-square grid. The build also produces
`build/<preset>/TrailForge.app` — a minimal bundle with `TrailForge.icns` (`sips` + `iconutil`, 16–1024 px) and a
copy of the same executable, so Finder shows the icon before launch. `Logo.png` is reserved for a future menu.

Command-line options:

- `--no-ui` — start with diagnostics hidden;
- `--demo` — scripted drive without input;
- `--quit-after N` — quit automatically after N seconds (smoke test);
- `--capture file.bmp|file.png [--capture-at N]` — save one rendered frame;
- `--script file.txt` — scenario mode (see below). Exit code 1 if any check failed;
- `--no-audio` — no audio device, no sounds loaded;
- `--audio-capture out.wav` — render the mix offline into `out.wav` (no device, see [Audio](#audio)).

`build/<preset>/trailforge_probe` — headless measurements:
- `brake`, `course`, `bench` — braking, course drive, step cost at 60/120/240 Hz;
- `corner [steer] [collisionSteps]` — steady-state cornering on a flat pad, average loads;
- `demo` — loads along the `--demo` trajectory in 1 s windows (Σload, Σ(F·ŷ), bump stop hits);
- `hillbrake` — holding on climbs with the brake.

### Scenario mode

`./build/release/trailforge --script tools/scenarios/full_check.txt` — full window and rendering.
Input takes the same path as a live player's: the scenario pushes keyboard, mouse and wheel events
into the SDL queue (`SDL_PushEvent`), `Input` processes them, and they reach the physics on ticks.
`Input` tracks held keys from events rather than from `SDL_GetKeyboardState`,
so synthetic and real key presses are handled identically.

One command per line, `#` starts a comment:

| Command | What it does |
|---|---|
| `down/up/tap KEY` | press / release / press and release (SDL name: `W`, `Space`, `Escape`, `Left`…) |
| `wait S` | pause for S seconds of real time |
| `until VAR OP VAL TIMEOUT` | wait for a condition (`< <= > >= == !=`); FAIL on timeout |
| `expect VAR OP VAL` | check immediately |
| `mark VAR` | remember a value; afterwards `d_VAR` = current − remembered |
| `teleport N` / `resize W H` / `capture file.png` | teleport to point N, `SDL_SetWindowSize`, save a frame |
| `click KEY` / `inputclick KEY` / `scrollto KEY` / `text STR` | click an ImGui widget, Cmd+click (number input), scroll the Tuning window to a widget, type text |
| `orbit DX DY` / `wheel DY` | camera orbit with the right mouse button / mouse wheel |
| `log NAME` / `reset_stats` / `quit` | telemetry line to stdout / reset min/max counters / quit |
| `quality Low\|Medium\|High\|Ultra` | apply a graphics preset |
| `set KEY V` | graphics/look setting: `render_scale`, `msaa`, `fxaa`, `cascades`, `shadow_size`, `shadow_dist`, `shadow_filter`, `reflections`, `refl_scale`, `bloom`, `grass`, `grass_dist`, `clouds`, `exposure`, `fog`, `tod` (hours), `sun_heading` (deg), `tod_speed` (h/s), `tod_run` (time-lapse 0/1), `mud`, `auto_mud`, `headlights`, `debug_lines`, `ui`, `hud`, `graphics_ui`, `vsync`; tire marks: `marks` (draw 0/1), `marks_gen` (generate 0/1), `marks_max` (ring capacity), `marks_fade` (s), `marks_gain` (all surfaces), `marks_clear 1`, `marks_fill 1` (synthetic fill to the cap) |
| `time Dawn\|Noon\|Sunset\|Night\|HOURS` | set the time of day (preset or 0–24 h); stops the time-lapse |
| `camera X Y Z TX TY TZ` / `camera follow` | free camera at a pose looking at a target / back to the follow camera |
| `perf_begin LABEL` / `perf_end` | collect frame times; prints `PERF LABEL fps frame_avg p95 p99 max cpu_avg gpu_avg …` |

Other scenarios: `tools/scenarios/visual_tour.txt` (showcase screenshots `/tmp/trailforge_vis_*.png`),
`tools/scenarios/time_of_day.txt` (Dawn/Noon/Sunset/Night and a banner-like sunset view, `/tmp/trailforge_tod_*.png`),
`tools/scenarios/perf_presets.txt` (frame times per quality preset),
`tools/scenarios/tracks.txt` (tire marks on concrete, dirt, mud and grass, `/tmp/trailforge_tracks_*.png`, and the
cost of the marks with the ring empty and filled to 16k / 64k segments),
`tools/scenarios/audio_check.txt` (the situations the sound reacts to, checks the mixer parameters; run it with
`--audio-capture`, see [Audio](#audio)).

Variables: `speed_kmh`, `fwd_kmh`, `gear`, `rpm`, `dir`, `low`, `tilt`, `x/y/z`, `ticks`, `paused`,
`cam_free`, `ui_visible`, `ui_kb`, `ui_text`, `air`, `max_air`, `lowgrip_wheels`, `max_slip`, `load_sum`,
`min_cam_clearance`, `front_spring`, `mass`, `depth_w/h`, `depth_creations`, `depth_matches_window`,
`win_w/h`, `px_w/h`, `fps`, `frame_ms`, `cpu_ms`, `gpu_ms`, `quality`, `msaa`, `render_w`, `draw_calls`,
`grass_blades`, `reflection`, `mud`, `tod`, `tod_run`, `sun_el`, `sun_az` (deg), `trees`, `marks` (live segments),
`marks_cap`, `marks_written`, `marks_drawn`, `marks_us` (CPU per tick), `marks_s<N>` (live segments on SurfaceId N:
0 Concrete, 1 PackedDirt, 2 Rock, 3 LowGrip, 4 Mud, 5 Grass), `msg:<substring>` (1 if the latest UI message contains the substring),
audio: `audio_active`, `audio_t` (captured seconds), `audio_src_rpm`, `audio_load`, `audio_skid`, `audio_gravel`,
`audio_dirt`, `audio_mud`, `audio_grass`, `audio_wind`, `audio_day`, `audio_night`, `audio_gameplay` (pause duck
gain), `audio_voices`, `audio_oneshots`, `audio_peak_db`, `audio_clipped`, `audio_cpu`, `audio_ev<N>` (events
received by the mixer: 0 engine start, 1 gear, 2 bump stop, 3 suspension thump, 4 body impact, 5 splash, 6 creak,
7 UI), and more. Full list: `Application::scenarioValue`.
Audio `set` keys: `audio_ui` (panel 0/1), `audio_mute`, `audio_master`, `audio_pitch`, `audio_whine`,
`audio_play_ui N` (play UI sound N: 0 click, 1 hover, 2 confirm, 3 back, 4 slider tick, 5 menu open, 6 menu close).

## Controls

| Action | Keyboard | Gamepad |
|---|---|---|
| Throttle / brake | W / S | RT / LT |
| Steering | A / D | left stick |
| Handbrake | Space | A (South) |
| Forward ↔ reverse (only below 1 m/s) | Q | X (West) |
| Low range (only below 1 m/s) | L | Y (North) |
| Reset to the last start point | R | Back |
| Camera: follow ↔ free | C | RB |
| Headlights | H | — |
| Diagnostics | F1 | — |
| Graphics panel (quality, look, cosmetic dirt) | F2 | — |
| Audio panel (volumes, mute, engine pitch, read-outs) | F3 | — |
| Pause | Escape | Start |
| Single physics step (while paused) | N | — |
| Camera orbit / zoom | RMB + mouse / wheel | — |
| Free camera | arrow keys, PgUp/PgDn, Shift — faster | — |

Brake and throttle are separate: S always brakes. To drive backwards, stop, press Q and hold W.
While ImGui owns the keyboard or mouse (focus in a field or hovering a window), the vehicle does not receive that input.
F1–F3 and Escape always work.

To type a number into an ImGui slider on macOS use **Cmd+click** (in ImGui's macOS mode Ctrl+click acts as a right click);
on other platforms use Ctrl+click.

## Architecture

```
src/physics/  PhysicsWorld — RAII wrapper over Jolt (allocator, factory, job system, layers)
src/track/    SurfaceProperties (materials and grip), TestTrack (procedural proving ground + MeshShape)
src/vehicle/  VehicleConfig/VehicleTuning, Vehicle (constraint, controller, telemetry), WheelContact (per-tick contact samples)
src/sim/      Simulation (physics + track + vehicle, input on ticks), FixedStepClock, TireMarks (visual mark ribbons),
              BodyImpacts (contact listener: chassis hits for the sound)
src/audio/    AudioMixer (sound model + own mixer, no device), AudioMapping (pure rpm/load/tire/ambience mappings),
              VehicleAudioFeed (per-tick frame and events), AudioManifest + Json, AudioSystem (miniaudio device / WAV capture)
src/render/   Renderer (SDL GPU passes), RenderSettings (quality presets), VehicleModel (procedural vehicle mesh),
              Scenery (backdrop terrain, pine meshes and instances), TimeOfDay (sun/moon path, sky palette), GroundMaps (grass/mud/wet masks), Mesh, Camera, DebugDraw,
              GpuTimingMetal.m (GPU time from MTLCommandBuffer)
src/input/    Input (keyboard, mouse, gamepad → DriverIntent and discrete actions)
src/ui/       DebugUI (ImGui: HUD, telemetry, tuning, teleport, graphics and audio panels)
src/app/      Application (lifecycle and main loop), Scenario (scenario mode), main
shaders/      scene.metal → metallib at build time, embedded into the binary
tests/        CTest physics and audio tests (no window, no audio device)
tools/        SimProbe — headless measurements; scenarios/ — scripts for --script; audio/ — asset rebuild and capture analysis
assets/audio/ WAV sounds (48 kHz / 16-bit), manifest.json (per-file metadata), CREDITS.md (sources and licenses)
```

The physics (`trailforge_sim`) does not depend on SDL or rendering; the sound model (`trailforge_audio`) depends only
on it, the audio device lives in the app. Resources are released via RAII;
member order defines destruction order. There is no ECS, no plugins and no graphics API abstraction.

### Coordinate system and units

- Right-handed coordinate system, **Y up**, units are meters, kilograms, seconds.
- Vehicle local axes: **+Z forward, +X left**, +Y up.
- Wheel order: 0 FL, 1 FR, 2 RL, 3 RR.
- Jolt and GLM matrices are column-major. Depth range 0..1 (`GLM_FORCE_DEPTH_ZERO_TO_ONE`).
- Steering angle in the UI: positive is left.

### Loop

Fixed physics step of 1/120 s (60 or 240 Hz selectable in the UI) with a time accumulator.
Frames longer than 0.25 s are clamped, and at most 12 steps run per frame. Dropped time is shown in the UI
("Dropped sim time"), where overload is also visible.

Rendering interpolates body and wheel poses between the last two ticks (lerp/slerp).
Input is smoothed inside the tick with a fixed dt, so it does not depend on FPS:
- steering — 2.2 1/s toward the target and 3.5 1/s back to center;
- pedals — 3 1/s on press and 6 1/s on release (analog gamepad input is not smoothed).

While paused the physics is frozen; N performs exactly one step.

## Vehicle

- Mass 2000 kg, wheelbase 2.8 m, track 1.65 m, wheels r = 0.4 m and 0.3 m wide (35 kg each).
- Center of mass 0.10 m below the frame center.
- The body collision is a compound of three boxes. The visible model (`VehicleModel`, ~7k triangles) is a
  procedural off-roader built around those boxes: beveled body, fenders, bumpers, snorkel, roof rack with lights,
  spare wheel, lamps. Visual parts stick out slightly beyond the collision boxes.
- Wheels (lugged tire, rim, hub) and suspension parts (two arms, half shaft, shock body and shaft, coil spring)
  are placed every frame from the Jolt wheel transforms and suspension length; there is no decorative animation.
- Suspension: travel 0.15–0.55 m, 1.35 Hz, damping 0.45, anti-roll bars off by default.
- Four-wheel drive:
  - engine 520 N·m, 800–4800 rpm;
  - 5-speed automatic plus reverse −4.0, final drive 3.9;
  - low range ×2.72 — a multiplier on the differential ratios, not a separate gearbox;
  - open front differential, rear and center are LSD (ratio 1.4).
- Wheels use `VehicleCollisionTesterCastCylinder`; their positions come from the simulation.
- Roll and pitch limits are disabled (`mMaxPitchRollAngle = π`): the vehicle can roll over.
- No hidden downforce, no direct setting of velocity or position.
  The only teleports are the R reset and the teleport buttons in the UI.

## Proving ground (~100×100 m)

Surrounded by an earth berm and concrete walls.

- **Start** — concrete pad (z ≈ −40).
- **Lane A** (center):
  - 0.18 m bumps every 3 m;
  - diagonal waves ±0.38 m — wheels lift diagonally on them;
  - **low-grip section**: a hard surface with μ×0.35. Not mud or ice — just a low coefficient.
- **Lane B** (right of the start, −X): hills of 12°, 20°, 28° and 35°, 1.8 m high.
- **Lane C** (left, +X):
  - 18° side slope;
  - steps of 0.15 / 0.30 / 0.45 m;
  - a field of 28 rocks and two boulders. Rocks are angular convex chunks (a subdivided icosahedron cut by random
    planes), flat-shaded; the collision mesh is the same triangles.
- **Skid strip** (east edge, x 39–45, z −48…44, teleport "Skid strip (concrete)"): a flat 6 × 92 m concrete strip
  for braking and wheelspin on concrete; it ends in a short dirt band and the meadow, so one run crosses
  concrete → dirt → grass.

- **Mud & puddles showcase** (north, z ≈ 46–86, teleport "Mud & puddles showcase"): a grass meadow with a
  winding mud path, puddle bowls in the terrain and a few rocks. The mud is the rigid `Mud` surface (μ×0.60/0.55),
  the meadow is `Grass` (μ×0.80/0.78). There is **no sinkage and no water physics**: puddles are dips in the
  hard ground, and the water surface is render-only. The ruts you see are tire marks, which are render-only too
  (see "Tire marks"). Deformable terrain is a later stage.

Rendering and collision are built from the same triangles (`TrackGeometry`). Each triangle has its own material.
The outer backdrop (hills, trees beyond the walls, distant mountains) and grass are render-only and cannot be reached.
Grip is computed per wheel: `SetCombineFriction` takes the material under the contact patch
and multiplies the tire curve by the surface coefficients — longitudinal and lateral separately.
Teleport points are in the Tuning window.

## Rendering

SDL GPU on Metal; the shaders (`shaders/scene.metal`) are MSL only. Passes per frame:

1. Sun (or moon) shadow cascades (1–4 layers of a D32 array, practical split, texel-snapped, hardware-compare PCF with
   1/4/12 taps).
2. Grass trample mask (RG8, 0.1 m texels over the ground) rendered from the tire mark segments, only when marks were
   added or removed and every 0.5 s while they fade.
3. Planar puddle reflection at reduced resolution (all puddles share one water level), skipped when no puddle is visible.
4. Main HDR pass (RGBA16F, optional MSAA 2×/4×): PBR (GGX + Lambert, sky/ground ambient), triplanar procedural
   rock/concrete/dirt, mask-blended grass/mud/wet ground, tire marks (one instanced draw), instanced grass (2.4 m
   chunks, 3 LOD levels, distance fade, wind, flattened under the wheel contacts and in the trample mask), procedural
   sky with clouds, water with Fresnel, ripples and reflection, height fog, debug lines.
5. Bloom (5-level downsample/upsample chain).
6. Tonemap (ACES fit, exposure, vignette, dither) with upscale from the render scale; optional FXAA; ImGui.

### Time of day and environment

- **Time of day** (F2 → Look, or F1 → Tuning → "Time of day"): a 0–24 h slider, presets Dawn (5:45), Noon,
  Sunset (18:12) and Night (23:30), and a time-lapse with a speed slider (0.01–4 h/s) and Run/Pause. The sun follows
  a simple solar path (latitude 45°, declination 10°: sunrise ~5:20, sunset ~18:40, noon elevation 55°); the
  "Sun path heading" rotates the path so the sunset lies north, behind the showcase meadow. Sun color and intensity,
  the sky gradient, fog color and density, sky/ground ambient, exposure and bloom are interpolated from a palette
  keyed by sun elevation (`src/render/TimeOfDay.cpp`), so time-lapse transitions are smooth.
- **Night:** a full moon (opposite the sun) lights the scene dimly and casts the shadows, with stars and a moonlit
  blue ambient. The shadow light switches from the sun to the moon while both are below about −4°, so nothing is lit
  or shadowed from below the horizon. Headlights (H) are two unshadowed spot lights on the vehicle that light the
  ground, grass and rocks (render-only, stylized falloff).
- **Sky:** large stylized sun disc (bigger near the horizon), sunset-lit cloud undersides, horizon haze that matches
  the fog. Four layers of faceted distant mountains are drawn in the sky pass at infinity (render-only, no
  collision) with aerial perspective toward a violet haze; a valley opens where the sun sets.
- **Backdrop:** flat-shaded terrain beyond the walls with a lower view corridor toward the sunset; ~900 instanced
  low-poly pines (faceted tiers, per-tree size, yaw and tint, wind sway) with three LODs by distance (55 / 140 m),
  culled per frame on the CPU, also drawn into the shadow cascades and the puddle reflection.
- The perimeter wall (5 m, collision) stays and crosses low camera views of the horizon.

Vehicle dirt is **cosmetic only**: a mud layer on the body (splash line from the bottom up) and wheels. In auto mode it
builds up while the wheels roll on `Mud` or through puddles and slowly dries off; it never affects the physics.

### Tire marks (visual only)

Every physics tick `Vehicle` fills one `WheelContactSample` per wheel (`src/vehicle/WheelContact.h`): contact patch
center and normal, suspension load, longitudinal/lateral slip speed of the patch, rolling speed, surface, tire width
and radius. `TireMarks` (`src/sim`, headless, no SDL) turns these samples into one ribbon per wheel: a segment every
0.15 m of patch travel, written into a fixed ring buffer (default 16,384 segments, the oldest is overwritten). Each
segment stores both end points, the half-width vector in the contact plane (width from the wheel), surface, birth
time, and two intensities per end:

- **imprint** — rolling imprint on soft surfaces, ∝ load (dirt 0.85, mud 1.0, grass 0.9 at static load);
- **slip** — rubber on hard surfaces, churn on soft ones, from the patch sliding speed *and* the slip ratio
  (sliding speed / max(patch speed, rolling speed)). The ratio gate keeps normal driving clean: a steady 12 % slip in
  2nd gear at full throttle (≈ 1.7 m/s) leaves nothing on concrete, while lock-up, a 1st-gear launch with real
  wheelspin (3–4.5 m/s in the sim), handbrake slides and wheelspin on loose ground do.

Per surface: concrete/rock/low-grip get dark rubber only while sliding; dirt gets a compacted tread imprint (lug
pattern) and loose churned soil on wheelspin; mud gets a dark, wet, glossy rut with lighter raised edges (the mark
is 1.3× the tire width); grass gets a flattened, lighter strip, and spinning tears the turf to soil.
Relief is in the shading normal only — the ground mesh and the collision do not change.

Rendering: the ring is mirrored in a GPU storage buffer of the same size; only new segments are uploaded (at most two
copy regions per frame, 80 bytes each). All live segments are one instanced draw of quads that follow the terrain
(both ends sit on real contact points, lifted 1.2 cm along the normal, plus a negative depth bias, so there is no
z-fighting). The fragment shader rebuilds the ground material under the mark and shades it with the regular lighting,
cascaded shadows, vehicle AO, headlights and fog, so marks match every time of day. Soft-surface segments are also
drawn into the grass trample mask; grass blades read it and lie flat, or disappear where the turf was torn.
Marks fade over the last 40 % of their lifetime (default 240 s) and are then dropped.

Graphics panel (F2) → "Tire marks (visual only)": draw / generate toggles, max segments (1k–64k), fade time,
per-surface intensity multipliers, Clear, "Fill to cap" (synthetic random ribbons for stress testing), live
count / capacity, ring size, CPU cost per tick and upload size per frame.

Quality presets (Graphics panel, F2; default High):

| Preset | Render scale | AA | Cascades × size | Shadow distance | Reflection scale | Bloom | Grass density / distance |
|---|---|---|---|---|---|---|---|
| Low | 0.67 | FXAA | 1 × 1024 | 40 m | off | off | 1 / 22 m |
| Medium | 0.85 | FXAA | 2 × 1536 | 60 m | 0.35 | on | 2 / 32 m |
| High | 1.0 | MSAA 2× | 3 × 2048 | 80 m | 0.5 | on | 3 / 45 m |
| Ultra | 1.0 | MSAA 4× | 4 × 2048 | 110 m | 0.75 | on | 4 / 60 m |

The panel also shows CPU frame time (without the swapchain wait), GPU time (from `MTLCommandBuffer`
`GPUStartTime/GPUEndTime`), render/output size, draw calls and grass blades. Every setting can be changed
individually (the preset becomes "Custom"). Physics debug lines are off by default (Tuning → Debug drawing).

## Audio

Recorded sounds (`assets/audio/`, 55 WAV files, 48 kHz / 16-bit) driven by the simulation. The mixer is our own
code on top of [miniaudio](https://miniaud.io) 0.11.25, of which only the low-level parts are used: `ma_device`
(Core Audio playback, f32 stereo 48 kHz, 10 ms period), `ma_decoder` (WAV loading) and `ma_encoder` (capture).
The `ma_engine` / node graph / resource manager are compiled out (`MA_NO_ENGINE`, `MA_NO_NODE_GRAPH`,
`MA_NO_RESOURCE_MANAGER`): the 12 engine layers must change rate and gain together, sample-accurately, from one
parameter set, and the offline capture must run the exact same code without a device or extra threads.

- **Threads.** Once per frame the game thread publishes an `AudioFrame` (listener, vehicle pose, rpm, throttle,
  gear, per-wheel contact data, time of day, pause, settings) and the events of the frame into a mailbox
  (`std::mutex`, held only for a copy). The device callback takes it with `try_lock` and never waits for the game;
  if the lock is busy it keeps the previous frame. The game never blocks on the audio thread either (stats come back
  through the same `try_lock` mailbox). Events are detected per physics tick in `VehicleAudioFeed`.
- **Engine.** 12 loops: 3 rpm bands × off-load / full-load × exhaust mic and engine-bay mic (two mics of the same
  recorded engine). Each loop plays at `rate = rpm × pitch / rpm_recorded`; bands crossfade with equal power on
  log rpm (idle → mid 1400–2000 rpm, mid → high 3200–4800 rpm of the played engine speed), throttle blends off-load
  and full-load. `pitch` (default 0.80, Audio panel) lowers the sound of the recorded small inline-4 towards the
  simulated 800–4800 rpm truck engine. The recorded level differences between layers (up to 40 dB) are compressed
  into a 12 dB spread (panel: "Level spread"). Starter recording on start and on every reset, crossfaded into the
  loops after it catches (or at once when the throttle opens); gear clunks when a gear engages; a quiet
  synthesized transmission whine at the propeller-shaft mesh frequency.
- **Tires.** From the per-wheel contact samples (surface, load, slip ratio, slip speed, rolling speed): skid on
  concrete / low grip / rock by slip × load, gravel and dirt roll and scrub on packed dirt and rock, mud slosh plus
  random squelches, grass rustle and swishes, a water splash when a wheel drops into a puddle (contact below the
  puddle's water level inside its rectangle).
- **Body.** Suspension thump on load steps, bump-stop hits, chassis creak at large articulation, metal impacts from a
  Jolt contact listener on the chassis body (impulse threshold 600 N·s, see the simplifications below).
- **Ambience.** Wind by speed (from 3 m/s, full at 30 m/s), day and night beds crossfaded by the sun elevation
  (−6° … +6°).
- **Space.** Every source has a position on the vehicle (exhaust, bay, wheels); gain and equal-power pan are
  relative to the camera (inverse distance from 5 m, silent beyond 200 m). No Doppler, no occlusion, no reverb.
- **Pause** (Escape) ramps the gameplay sounds to silence and freezes them; the ambience ducks to half. Menu
  open/close UI sounds play on pause toggle; all UI sounds are loaded and reachable through
  `AudioSystem::playUi` (there is no menu yet).
- **Output.** 10 Hz DC blocker, 2 ms look-ahead limiter at −1.5 dBFS and a hard clamp at −1 dBFS.
- **Without a device.** `--no-audio` skips everything; if no device can be opened or the assets are missing, the
  game logs it and runs silent. Tests and the headless build never open a device.

**Audio panel (F3):** master / engine / tires / impacts / ambience & wind / UI volumes, mute, pitch factor, level
spread, whine; read-outs: played rpm, load, layer weights and playback rates, tire layers, wind, day/night, active
voices, one-shots, output peak, limiter, audio-thread time per callback and in % of real time.

**Offline capture.** `trailforge --script tools/scenarios/audio_check.txt --audio-capture /tmp/trailforge_audio_check.wav`
renders the mix without a device: every physics tick produces exactly one tick of audio (paused frames render frame
time), so the sound lines up with the simulation regardless of the frame rate. The mixer parameters of every tick go
to `out.wav.csv`. `tools/audio/analyze_capture.py out.wav` (needs numpy; uses ffmpeg and sox if installed) prints
sample/true peak, clipping, integrated loudness, engine pitch tracking against the simulated rpm, the skid band level
while braking and the day/night spectrum balance, and writes spectrograms `/tmp/trailforge_audio_*.png`. The
capture is deterministic for the same tick sequence, but scenario input is timed in real time, so two runs differ
slightly.

**Assets.** `assets/audio/manifest.json` lists every file with its source, license, processing and verification;
`assets/audio/CREDITS.md` has the attributions (54 files CC0 1.0, one CC BY 4.0 — credit required, see the file).
`tools/audio/fetch_freesound.py` rebuilds the set from Freesound (needs `FREESOUND_API_KEY` in the environment,
ffmpeg and sox; the key is never written to disk). The game reads the assets from `assets/audio/` next to the
binary or in the source tree.

## Diagnostics (F1)

- **HUD** (always visible): speed, gear, direction, rpm.
- **Telemetry:**
  - FPS and frame time; ticks per frame, physics step time (last and average), dropped time;
  - speed, rpm, gear, clutch, range, body tilt, throttle/brake/handbrake/steering bars;
  - per-wheel table: contact, surface, suspension length and compression (m and %), load, longitudinal
    and lateral forces, slip ratio, slip angle, μ, wheel angular velocity.
- **Tuning:**
  - springs, dampers, anti-roll bars, tire curves, surface coefficients, engine, brakes
    and differentials are applied between physics steps;
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

## Physics simplifications (honest list)

- The suspension is virtual: in Jolt it is a raycast or cylinder cast along the spring axis, with no arms or axles.
  Solid axles and articulation are not modeled; every wheel is independent.
- The tire is two μ(slip) curves (longitudinal and lateral) plus Jolt's friction combining.
  No Pacejka model, heat, pressure or sidewall deformation.
- The ground is hard and non-deformable. Surfaces differ only in friction coefficients.
  The "mud" is a low-μ rigid surface, not a mud simulation; puddles have no water physics.
- Tire marks, ruts and flattened or torn grass are visual only: they never change friction, ground height or
  collision, and their relief exists only in the shading normal. The trample mask has 0.1 m texels and refreshes
  every 0.5 s while marks fade. The "Fill to cap" ribbons are synthetic (not driven).
- No forced differential lock: Jolt supports only open and LSD.
  The LSD here limits the wheel speed ratio; it is not a preloaded clutch pack.
- No aerodynamics or rolling resistance (linear damping = 0).
  The engine is a torque curve, with no flywheel inertia beyond Jolt's model.
- Body collision is a compound of three boxes.
- Time of day, moonlight and headlights are lighting only: they do not change grip, temperature or anything else in
  the simulation.
- Determinism is guaranteed only at the same step rate on the same platform.
  Trajectories differ at 60, 120 and 240 Hz, as the measurements below show.
- No damage.
- Sound is cosmetic: it never feeds back into the simulation. Mud slosh, puddle splashes and grass rustle are
  triggered by contact data on rigid ground, not by any fluid or vegetation simulation. Body impact strength is an
  estimate: Jolt's contact listener runs before the solver and reports no impulse, so the sound uses
  `J = v_approach / (1/m + (r×n)·I⁻¹(r×n))` (N·s) from the approach speed at the contact point. The engine sound is
  a recorded petrol inline-4 pitched down, not a synthesized model of the simulated engine.

## Verification results

Everything below was actually run on macOS 27 / M-series.

- **Build.** The `debug`, `release` and `headless` presets build without errors.
  The Metal shader is compiled to a metallib at build time and embedded into the binary.
  At startup the log shows `GPU driver 'metal', shaders: metallib (precompiled)`.
- **CTest.** 13 of 13 tests pass in Debug, Release and headless:

| Test | Result |
|---|---|
| Settling on suspension | No NaN; load sum 19623 N with m·g = 19620 N |
| Throttle | 0 → 16.1 m/s in 4 s |
| Braking | 16.1 → 8.1 m/s in 1 s, then a full stop |
| Wheel over a pit | FL has no contact, load 0, the others carry the weight |
| Reset | Gear 0, 800 rpm, initial pose |
| Frame schedule independence | Different frame patterns at the same step rate: position difference 0.000000 m |
| Cornering load balance (40 km/h) | Average Σload = 19620 N with 1, 2 and 4 collision steps; outer front > 2× inner |
| Brake in low range | 10.4 m/s → stop in 1.33 s, speed never increases, final 0 m/s |
| Tire marks per surface | Rolling 30 km/h on concrete: 0 segments; full brake: 83 segments, slip 1.00; rolling on dirt/mud/grass: 480 segments each, imprint 0.85/1.00/0.90; wheelspin: 68/76/68 segments, slip 0.56/0.48/0.54; cap 256 holds; with fade 3 s none are left after 3.5 s parked |
| Audio: engine layers | Equal-power band weights (Σw² = 1 within 2.4e-7), crossfade midpoints 0.7071, playback rates follow rpm × pitch / rpm_recorded |
| Audio: tire layers | Concrete lock-up skid 1.0, 12 % traction slip 0.0; rolling dirt 0.75 / mud 0.62 / grass 0.62; wheels in the air 0 |
| Audio: manifest | All 55 sounds present at 48 kHz, 12 engine loops with recorded rpm and level, licenses 54 CC0 + 1 other |
| Audio: feed and mixer | Simulated launch: 4 gear events, source rpm up to 3839; concrete skid 0.000 cruising vs 0.924 braking; mud lane mud 0.65; pause RMS 0.1414 → 0.0509; peak 0.672, 0 clipped |

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
  128 checks, 0 failures. Two consecutive runs in Release and one in Debug, about 110 s per run. Covered:
  - throttle and braking; D/A steering and smooth return to center;
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
  - there is no rolling resistance, so after releasing the brake the vehicle rolls a few centimeters;
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

## Next stage: tires and deformable terrain

1. **Custom tire model.** Replace Jolt's curves with `WheelSettings` + a custom `VehicleController`:
   - a simplified load-dependent brush or Pacejka model;
   - relaxation length to avoid jitter at low speed (slip is currently undefined there);
   - tire pressure as a contact patch stiffness parameter.
2. **Surface fields in `SurfaceProperties`:**
   - Bekker deformation and cohesion moduli (k_c, k_φ, n);
   - cohesion and internal friction angle (Mohr–Coulomb);
   - rolling resistance.
3. **Terrain height field.** A separate `HeightFieldShape` on top of the "hard" proving ground:
   - the wheel presses a rut according to contact patch pressure and compacts the soil;
   - heights are updated in chunks, and collision is rebuilt locally;
   - rendering uses the same height buffer.
4. **Forces.** Bulldozing resistance and wheel sinkage increase motion resistance.
   Traction is limited by soil shear strength (Janosi–Hanamoto), not only by μ.
5. **Drivetrain.** Differential locks as a separate controller mode, axles as linked wheels.
6. **Validation.** Tests for rolling resistance on soft soil, rut depth versus load
   and traction loss under wheelspin.

The tire marks already provide the input for this: `WheelContactSample` carries, per wheel and per tick, the patch
position and normal, load, slip speeds, surface, width and radius. A deformable terrain stage can consume the same
samples (patch pressure = load / (width × patch length)) to write sinkage into the height field instead of a mark
ribbon, and the mark renderer then only has to draw what the height field does not already show.
