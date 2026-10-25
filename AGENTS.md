# AGENTS.md

Instructions for coding agents working on TrailForge. Human-facing documentation is in `README.md`.

## Project overview

TrailForge is a C++20 off-road vehicle prototype on a small custom engine: SDL3 (window, input, gamepad),
SDL GPU (Metal on macOS; D3D12 / Vulkan elsewhere) with one HLSL shader source, Jolt Physics (`VehicleConstraint` + `WheeledVehicleController`),
Dear ImGui (diagnostics), GLM, miniaudio (audio device / WAV I/O only; the mixer is our own). The focus is
believable vehicle behavior: mass, suspension travel, wheelspin, wheel lift, rocks, climbs. All geometry and
textures are procedural; the only external assets are the sounds in `assets/audio/` (licenses in
`assets/audio/CREDITS.md`, rebuilt by `tools/audio/fetch_freesound.py`, which reads `FREESOUND_API_KEY` from the
environment — never write the key into any file).

Only macOS / Apple Silicon / Metal has been run (plus Vulkan through MoltenVK on the same Mac as a test of the
SPIR-V path). Windows x64 and Linux x64 build and pass the headless tests in GitHub Actions
(`.github/workflows/ci.yml`; the Linux package also starts on Mesa lavapipe under Xvfb) but have never been run on
real hardware: say so whenever they are mentioned. Dependencies are fetched by CMake FetchContent as pinned archives
with SHA256 checks (internet is needed for the first configure).

## Build and test

```sh
# configure + build (presets: debug, release, headless)
cmake --preset release
cmake --build --preset release

cmake --preset debug
cmake --build --preset debug

# headless physics and audio tests (CTest, 30 tests; no window, no audio device)
ctest --preset release
ctest --preset debug

# physics and tests only, without SDL/rendering
cmake --preset headless && cmake --build --preset headless && ctest --preset headless

# Windows (x64 Native Tools prompt: MSVC cl + Ninja) and Linux (Ninja, gcc/clang); shown only on their host
cmake --preset windows-release && cmake --build --preset windows-release && ctest --preset windows-release
cmake --preset linux-release && cmake --build --preset linux-release && ctest --preset linux-release
```

Shaders: `shaders/scene.hlsl` is the only shader source (26 entry points, each guarded by `TF_ENTRY_<name>`, SDL GPU
register/space conventions; see README "Shader pipeline"). `cmake/Shaders.cmake` compiles every entry with the host
tool `trailforge_shaderc` (`tools/shaderc`: glslang HLSL -> SPIR-V -> SPIRV-Tools legalize/validate -> SPIRV-Cross
MSL, pinned in `cmake/ShaderToolchain.cmake`), then `xcrun metal` -> metallib on macOS and the pinned prebuilt DXC ->
DXIL on Windows hosts only; `cmake/EmbedShaders.cmake` embeds the blobs. When adding a shader entry, add it to
`TRAILFORGE_SHADER_ENTRIES` and keep the resource counts in the renderer's pipeline creation in sync (the tool
prints them). Visual changes to Metal output can be measured with `tools/render/compare_captures.py BEFORE AFTER`.
Captures differ slightly between runs of the same binary (timing), so identity claims need a same-process A/B.

Build output goes to `build/<preset>/`. Binaries: `trailforge` (app), `trailforge_tests`, `trailforge_audio_tests`, `trailforge_probe`
(headless measurements: `brake`, `course`, `bench`, `corner`, `demo`, `hillbrake`, `susp`, `skid`, `launch`,
`showcase`, `surfaces`, `climb`, `abs`, `ellipse`, `tc`, `tcmud`, `diff`).

Scripted end-to-end checks run the real window and push input through the SDL event queue. Exit code is 1 if any
check failed:

```sh
./build/release/trailforge --script tools/scenarios/full_check.txt     # full regression, expect 0 failed
./build/release/trailforge --script tools/scenarios/menus.txt         # intro, start/pause/settings menus, /tmp/trailforge_menu_*.png
./build/release/trailforge --script tools/scenarios/visual_tour.txt    # showcase captures to /tmp/trailforge_vis_*.png
./build/release/trailforge --script tools/scenarios/time_of_day.txt    # time-of-day captures to /tmp/trailforge_tod_*.png
./build/release/trailforge --script tools/scenarios/perf_presets.txt   # frame times per quality preset
./build/release/trailforge --script tools/scenarios/tracks.txt         # tire marks per surface + cost, /tmp/trailforge_tracks_*.png
./build/release/trailforge --script tools/scenarios/suspension.txt     # stops with a wheel in the air, /tmp/trailforge_v_susp_*.png
./build/release/trailforge --script tools/scenarios/skid_check.txt     # tire squeal gating (needs a device or --audio-capture)
# audio: offline capture (no device) + analysis (numpy; ffmpeg/sox for loudness and spectrograms)
./build/release/trailforge --script tools/scenarios/audio_check.txt --audio-capture /tmp/trailforge_audio_check.wav
python3 tools/audio/analyze_capture.py /tmp/trailforge_audio_check.wav
```

Other app flags: `--no-ui`, `--demo`, `--quit-after N`, `--capture file.png [--capture-at N]`, `--no-audio`,
`--audio-capture file.wav`, `--paths`, `--version`. Scenario checks must not depend on an audio device: everything except
`audio_check.txt` and `skid_check.txt` (both check the mixer) passes with `--no-audio`.
The scenario command reference is in `README.md` ("Scenario mode").

Before committing a code change, build both `debug` and `release`, run `ctest` for both, and run `full_check.txt`
in Release.

### Packaged build and GitHub release (every stage)

Every stage ends with a packaged build and a GitHub release, after the stage's commits are pushed:

1. Bump the version in `project(TrailForge VERSION x.y.z)` in `CMakeLists.txt`: minor per stage (v0.1.0 was the
   first packaged release, then v0.2.0, v0.3.0, ...), patch for a fix-only re-release.
2. Write the release notes in `docs/releases/v<version>.md` (English) from the devlog: what is in the build,
   controls summary, known limitations, and how to open an ad-hoc signed app (right-click -> Open or
   `xattr -dr com.apple.quarantine TrailForge.app`). Only verified claims, as in the devlog.
3. Commit the bump and the notes, push, then run `tools/release/package_macos.sh --publish` (clean Release
   build in `build/package`, ctest, `dist/TrailForge.app`, ad-hoc signature, zip
   `dist/TrailForge-<version>-macos-arm64.zip`, smoke tests from `/` and from a fresh unzip, then
   `gh release create v<version>` with the zip and the notes). Without `--publish` it only packages and tests.
4. Record the release URL, asset name and size, and the smoke test results in the devlog.

`dist/` is git-ignored; never commit the zip or the app.

Windows / Linux packages: CI builds them with `tools/release/package_windows.ps1` and `tools/release/package_linux.sh`
and uploads them as artifacts (`trailforge-windows-x64`, `trailforge-linux-x64`). After `package_macos.sh --publish`,
download the artifacts of the green CI run for the release commit (`gh run download <run-id>`) and attach them with
`gh release upload v<version>`; the release notes must label them "untested on real hardware". Check the CI run with
`gh run watch` / `gh run view --log-failed` after every push.

## Layout

```
src/physics/  PhysicsWorld - RAII wrapper over Jolt (allocator, factory, job system, layers)
src/track/    SurfaceProperties (materials, grip), TestTrack (procedural proving ground + MeshShape)
src/vehicle/  VehicleConfig/VehicleTuning, Vehicle (constraint, controller, telemetry), WheelContact (contact samples),
              DrivingAids (ABS / traction control logic, brake torque and throttle factor only)
src/sim/      Simulation (physics + track + vehicle, input on ticks), FixedStepClock, TireMarks (visual-only ring),
              BodyImpacts (chassis contact listener for impact sounds)
src/audio/    AudioMixer (sound model + mixer, no device), AudioMapping (pure mappings), VehicleAudioFeed (per-tick
              frame + events), AudioManifest, Json; AudioSystem (app only: miniaudio device / offline WAV capture)
src/render/   Renderer (SDL GPU passes, shader format choice), RenderSettings, TimeOfDay, VehicleModel, Scenery,
              GroundMaps, Mesh, Camera, DebugDraw, ShaderBlobs.h (embedded shader table), GpuTimingMetal.m (macOS only)
src/input/    Input (keyboard, mouse, gamepad -> DriverIntent and discrete actions)
src/ui/       DebugUI (ImGui: HUD, telemetry, tuning, teleport, graphics panel, audio panel), MenuUI (intro, menus)
src/app/      Application (lifecycle, main loop), GameFlow (menu state machine), GameSettings (settings.cfg),
              Scenario (scenario mode), Paths (assets via SDL_GetBasePath, user
              dir via SDL_GetPrefPath), main
shaders/      scene.hlsl -> SPIR-V, MSL/metallib (macOS), DXIL (Windows hosts) at build time, embedded in the binary
tests/        CTest physics and audio tests (no window, no audio device)
tools/        SimProbe (headless measurements), scenarios/ (scripts for --script), audio/ (asset rebuild, capture analysis),
              release/ (package_macos.sh: .app, zip, GitHub release; package_windows.ps1, package_linux.sh: CI
              packages), shaderc/ (build-time HLSL compiler), render/compare_captures.py (pixel diff statistics)
docs/releases/ release notes per version
.github/      workflows/ci.yml (Linux / Windows build + ctest + packages, lavapipe smoke, macOS headless)
assets/audio/ WAV sounds, manifest.json, CREDITS.md; assets/music/ intro.mp3; assets/fonts/ Rajdhani (OFL);
              assets/images/ logo.png (each with CREDITS.md)
external_assets/ images/ (Icon.png app icon source, Logo.png, Banner.png, Cars.png vehicle concept sheet),
 music/ (user-provided tracks, not wired in yet)
devlog/       daily development log
```

The physics library `trailforge_sim` must not depend on SDL or rendering; `trailforge_audio` depends only on
`trailforge_sim` (no miniaudio, no device), so the sound model is testable headless. The audio thread never blocks
the frame loop: the game publishes one `AudioFrame` per frame through a mailbox the callback reads with `try_lock`. Resources are released via RAII;
member order defines destruction order. No ECS, no plugins, no graphics API abstraction.

## Coordinate system and units

- Right-handed, **Y up**; SI units: meters, kilograms, seconds (forces in N, Jolt lambdas are impulses in N·s).
- Vehicle local axes: +Z forward, +X left, +Y up. Wheel order: 0 FL, 1 FR, 2 RL, 3 RR.
- Jolt and GLM matrices are column-major; depth range 0..1 (`GLM_FORCE_DEPTH_ZERO_TO_ONE`).
- Fixed physics step 1/120 s by default (60/240 Hz selectable); rendering interpolates between ticks.

## Conventions

### Language

All repository content is English only: code, comments, identifiers, UI strings, log and console messages,
tests, commit messages, docs and the devlog. No Cyrillic characters in tracked files. This check should print
nothing:

```sh
rg -n '\p{Cyrillic}' --hidden -g '!build/**' -g '!.git/**'
```

### Devlog

Every agent doing work in the project keeps a devlog in `devlog/YYYY-MM-DD.md` (local date).

- If there is no file for today, create it from the template in `devlog/README.md`.
- If the file exists, append to it. Never rewrite or delete other agents' entries; you may only add to them
  (for example, replace an **IN PROGRESS** status with the outcome and the time).
- Sections, in this order:
  1. `# Devlog — YYYY-MM-DD (weekday)`
  2. `## Goal of the day`
  3. `## Stages` — chronological, with `HH:MM` times
  4. `## Problems and solutions` — problem, cause, solution
  5. `## Results` — only what was actually verified, with numbers
  6. `## Limitations and open questions` — unverified items, anomalies, simplifications
  7. `## Next steps`
- Keep verified and unverified items clearly separate. Never present a plan or an expectation as a result.
- Update the entry after each significant stage (build, fix, tests, new feature) and at the end of the work.
  Mark an unfinished stage as **IN PROGRESS**.
- Write briefly and factually.

Example stage entry:

```markdown
- **16:00 — MVP ready.** Debug/Release build, CTest 6/6. Physics step ~0.035 ms at 120 Hz.
```

### Physics honesty

- No hidden forces: no artificial downforce, no direct setting of velocity or position. The only teleports are
  the R reset and the teleport buttons / scenario `teleport`.
- Do not claim simulation that does not exist. Mud is a rigid low-friction surface, puddles are dips in rigid
  ground with a render-only water plane, vehicle dirt is cosmetic. Deformable terrain is a later stage.
- Wheel and suspension visuals are placed from the Jolt wheel transforms and suspension length every frame;
  no decorative animation.
- Render geometry and collision of the proving ground come from the same triangles.
- Sound is cosmetic and never feeds back into physics. It is driven only by simulation data (rpm, throttle, wheel
  contact samples, contacts); body impact strength is an estimated impulse (Jolt's listener has no solved impulse).
  Sound quality claims must come from capture analysis; nobody has judged the mix by ear unless the devlog says so.
- Keep the "Physics simplifications" list in `README.md` honest when behavior changes.
- Only claim platforms that were actually run (currently macOS / Apple Silicon / Metal).

### Commits and pushing

- Small, focused commits with a descriptive English message (imperative subject line, body with what was
  verified).
- Commit only after the build and tests above pass; mention the verified numbers (ctest, `full_check`) in the
  devlog.
- Do not commit build output, captures or local files (`build/`, `*.bmp`, `*.png` outside
  `external_assets/images/`, `imgui.ini`, `compile_commands.json`, audio captures `*.wav` / `*.wav.csv` outside `assets/audio/`).
- Never commit secrets (the Freesound API key lives only in the environment).
- Push to `origin/main` with a normal push. Rewrite history or force push only when the user explicitly asks.
