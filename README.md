![TrailForge](Banner.png)

# TrailForge — off-road simulator prototype

A small custom engine in C++20: SDL3 (window, input, gamepad), SDL GPU (rendering via Metal), Jolt Physics
(`VehicleConstraint` + `WheeledVehicleController`), Dear ImGui (diagnostics), GLM. The focus of the prototype is
vehicle behavior: mass, suspension travel, wheelspin, wheel lift, rocks, climbs.

All geometry and textures are procedural; there are no external assets.

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
- `--script file.txt` — scenario mode (see below). Exit code 1 if any check failed.

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
| `set KEY V` | graphics/look setting: `render_scale`, `msaa`, `fxaa`, `cascades`, `shadow_size`, `shadow_dist`, `shadow_filter`, `reflections`, `refl_scale`, `bloom`, `grass`, `grass_dist`, `clouds`, `exposure`, `fog`, `tod` (hours), `sun_heading` (deg), `tod_speed` (h/s), `tod_run` (time-lapse 0/1), `mud`, `auto_mud`, `headlights`, `debug_lines`, `ui`, `hud`, `graphics_ui`, `vsync` |
| `time Dawn\|Noon\|Sunset\|Night\|HOURS` | set the time of day (preset or 0–24 h); stops the time-lapse |
| `camera X Y Z TX TY TZ` / `camera follow` | free camera at a pose looking at a target / back to the follow camera |
| `perf_begin LABEL` / `perf_end` | collect frame times; prints `PERF LABEL fps frame_avg p95 p99 max cpu_avg gpu_avg …` |

Other scenarios: `tools/scenarios/visual_tour.txt` (showcase screenshots `/tmp/trailforge_vis_*.png`),
`tools/scenarios/time_of_day.txt` (Dawn/Noon/Sunset/Night and a banner-like sunset view, `/tmp/trailforge_tod_*.png`),
`tools/scenarios/perf_presets.txt` (frame times per quality preset).

Variables: `speed_kmh`, `fwd_kmh`, `gear`, `rpm`, `dir`, `low`, `tilt`, `x/y/z`, `ticks`, `paused`,
`cam_free`, `ui_visible`, `ui_kb`, `ui_text`, `air`, `max_air`, `lowgrip_wheels`, `max_slip`, `load_sum`,
`min_cam_clearance`, `front_spring`, `mass`, `depth_w/h`, `depth_creations`, `depth_matches_window`,
`win_w/h`, `px_w/h`, `fps`, `frame_ms`, `cpu_ms`, `gpu_ms`, `quality`, `msaa`, `render_w`, `draw_calls`,
`grass_blades`, `reflection`, `mud`, `tod`, `tod_run`, `sun_el`, `sun_az` (deg), `trees`, `msg:<substring>` (1 if the latest UI message contains the substring), and more.
Full list: `Application::scenarioValue`.

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
| Pause | Escape | Start |
| Single physics step (while paused) | N | — |
| Camera orbit / zoom | RMB + mouse / wheel | — |
| Free camera | arrow keys, PgUp/PgDn, Shift — faster | — |

Brake and throttle are separate: S always brakes. To drive backwards, stop, press Q and hold W.
While ImGui owns the keyboard or mouse (focus in a field or hovering a window), the vehicle does not receive that input.
F1, F2 and Escape always work.

To type a number into an ImGui slider on macOS use **Cmd+click** (in ImGui's macOS mode Ctrl+click acts as a right click);
on other platforms use Ctrl+click.

## Architecture

```
src/physics/  PhysicsWorld — RAII wrapper over Jolt (allocator, factory, job system, layers)
src/track/    SurfaceProperties (materials and grip), TestTrack (procedural proving ground + MeshShape)
src/vehicle/  VehicleConfig/VehicleTuning, Vehicle (constraint, controller, telemetry)
src/sim/      Simulation (physics + track + vehicle, input on ticks), FixedStepClock
src/render/   Renderer (SDL GPU passes), RenderSettings (quality presets), VehicleModel (procedural vehicle mesh),
              Scenery (backdrop terrain, pine meshes and instances), TimeOfDay (sun/moon path, sky palette), GroundMaps (grass/mud/wet masks), Mesh, Camera, DebugDraw,
              GpuTimingMetal.m (GPU time from MTLCommandBuffer)
src/input/    Input (keyboard, mouse, gamepad → DriverIntent and discrete actions)
src/ui/       DebugUI (ImGui: HUD, telemetry, tuning, teleport)
src/app/      Application (lifecycle and main loop), Scenario (scenario mode), main
shaders/      scene.metal → metallib at build time, embedded into the binary
tests/        CTest physics tests (no window)
tools/        SimProbe — headless measurements; scenarios/ — scripts for --script
```

The physics (`trailforge_sim`) does not depend on SDL or rendering. Resources are released via RAII;
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

- **Mud & puddles showcase** (north, z ≈ 46–86, teleport "Mud & puddles showcase"): a grass meadow with a
  winding mud path, puddle bowls in the terrain and a few rocks. The mud is the rigid `Mud` surface (μ×0.60/0.55),
  the meadow is `Grass` (μ×0.80/0.78). There is **no sinkage, no ruts and no water physics**: puddles are dips in the
  hard ground, and the water surface is render-only. Deformable terrain is a later stage.

Rendering and collision are built from the same triangles (`TrackGeometry`). Each triangle has its own material.
The outer backdrop (hills, trees beyond the walls, distant mountains) and grass are render-only and cannot be reached.
Grip is computed per wheel: `SetCombineFriction` takes the material under the contact patch
and multiplies the tire curve by the surface coefficients — longitudinal and lateral separately.
Teleport points are in the Tuning window.

## Rendering

SDL GPU on Metal; the shaders (`shaders/scene.metal`) are MSL only. Passes per frame:

1. Sun (or moon) shadow cascades (1–4 layers of a D32 array, practical split, texel-snapped, hardware-compare PCF with
   1/4/12 taps).
2. Planar puddle reflection at reduced resolution (all puddles share one water level), skipped when no puddle is visible.
3. Main HDR pass (RGBA16F, optional MSAA 2×/4×): PBR (GGX + Lambert, sky/ground ambient), triplanar procedural
   rock/concrete/dirt, mask-blended grass/mud/wet ground, instanced grass (2.4 m chunks, 3 LOD levels, distance fade,
   wind, flattened under the wheel contacts), procedural sky with clouds, water with Fresnel, ripples and reflection,
   height fog, debug lines.
4. Bloom (5-level downsample/upsample chain).
5. Tonemap (ACES fit, exposure, vignette, dither) with upscale from the render scale; optional FXAA; ImGui.

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

## Verification results

Everything below was actually run on macOS 27 / M-series.

- **Build.** The `debug`, `release` and `headless` presets build without errors.
  The Metal shader is compiled to a metallib at build time and embedded into the binary.
  At startup the log shows `GPU driver 'metal', shaders: metallib (precompiled)`.
- **CTest.** 8 of 8 tests pass in Debug and Release:

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
  - subjective handling feel.

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
