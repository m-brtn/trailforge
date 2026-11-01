# Architecture

Source layout, coordinate system and units, the main loop. All documentation: [index](README.md).

```
src/physics/  PhysicsWorld — RAII wrapper over Jolt (allocator, factory, job system, layers)
src/track/    SurfaceProperties (materials and grip), TestTrack (procedural proving ground + MeshShape),
              Level (level.json bake: heights, surfaces, water, trees, rocks, POIs, buildings), BuildingKit
              (procedural buildings: render LODs + convex collision pieces), TerrainTiles (streamed
              HeightFieldShape tiles + per-tile trunk / rock colliders)
src/core/     Json (reader, writer, merge; used by the vehicle data, the setups and the audio manifest), Ktx2 (BC7 KTX2
              reader / writer), TerrainTextureSet (terrain.json, level skip per texture size)
src/vehicle/  VehicleConfig/VehicleTuning, Vehicle (constraint, controller, layouts, telemetry), WheelContact (per-tick contact samples),
              DrivingAids (ABS / traction control: per-wheel brake torque and a throttle factor),
              ElectricDrive (motor envelope, torque split, regen blend, battery; used by the electric controller in Vehicle.cpp),
              VehicleDefinition (vehicle.json loader and validation, kits), VehicleSetup (garage parameters, setup files),
              VehicleBlueprint (traced concept body for bodyShape "traced": distance fields, wheelhouses, collision hulls)
src/sim/      Simulation (physics + track + vehicle, input on ticks), FixedStepClock, TireMarks (visual mark ribbons),
              BodyImpacts (contact listener: chassis hits for the sound)
src/audio/    AudioMixer (sound model + own mixer, no device), AudioMapping (pure rpm/load/tire/ambience mappings),
              VehicleAudioFeed (per-tick frame and events), AudioManifest + Json, AudioSystem (miniaudio device / WAV capture)
src/render/   Renderer (SDL GPU passes), RenderSettings (quality presets), StyleKit (shared vehicle parts), VehicleModel (named vehicle parts),
              Scenery (backdrop terrain, pine meshes and instances), LevelScene (level props, water, horizon ring,
              tree instances), TimeOfDay (sun/moon path, sky palette), GroundMaps (grass/mud/wet masks), Mesh, Camera, DebugDraw,
              GpuTimingMetal.m (GPU time from MTLCommandBuffer, macOS only), ShaderBlobs.h (embedded shader table)
src/input/    Input (keyboard, mouse, gamepad → DriverIntent and discrete actions), ControlLayout (bindings, key cap
              labels, gamepad glyph names; no SDL), KeyboardPedals
src/ui/       DebugUI (ImGui: HUD, telemetry, tuning, teleport, graphics and audio panels), MenuUI (intro, start,
              pause and Settings menus drawn with ImGui draw lists)
src/app/      Application (lifecycle and main loop), GameFlow (screen / menu state machine, no SDL: intro, start
              menu, playing, paused, garage; level and vehicle select pages can slot in as further pages), Garage
              (setup model, no SDL), GameSettings (settings.cfg), Scenario (scenario mode), Paths (asset / user dirs), main
shaders/      scene.hlsl → SPIR-V, MSL / metallib, DXIL at build time, embedded into the binary
tests/        CTest physics, vehicle, app and audio tests (no window, no audio device); data/vehicles/ test-only definitions
tools/        SimProbe — headless measurements; scenarios/ — scripts for --script; audio/ — asset rebuild and capture analysis;
              shaderc/ — build-time shader compiler; render/ — capture comparison; release/ — packaging scripts;
              texc/ — BC7 KTX2 encoder (standalone CMake project); textures/ — terrain texture download and rebuild;
              cars/ — blueprint tracing from the concept sheet, silhouette comparison, OBJ export
assets/vehicles/ <id>/vehicle.json — vehicle definitions (see "Vehicle data format"), <id>/blueprint.json — traced body
assets/levels/ <id>/level.json — open-world level definitions (see "Exploration Peaks")
assets/textures/ terrain/ — terrain.json, 12 BC7 KTX2 maps (Git LFS), sources.lock.json; CREDITS.md
assets/models/ <folder>/*.glb + BC7 KTX2 level models (Git LFS), models.json, sources.lock.json; CREDITS.md
assets/audio/ WAV sounds (48 kHz / 16-bit), manifest.json (per-file metadata), CREDITS.md (sources and licenses)
assets/music/ intro.mp3 (intro and start menu), assets/fonts/ Rajdhani Medium / SemiBold + OFL.txt,
              assets/images/ (CREDITS.md; logo.png is copied from external_assets/images/Logo.png into the build tree)
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
- throttle and hand brake — 3 1/s on press and 6 1/s on release; foot brake — 8 1/s on press (full line pressure
  0.125 s after a stamp, `brakeRise`; 3 1/s before v0.24.0) and 6 1/s on release. This applies to every source
  (keyboard ramp, gamepad triggers, scenario `pedal`).

The pause menu is a real pause: the accumulator is frozen (no ticks, no accumulated time, the interpolation alpha
stays put) while rendering continues; Resume continues from the same tick. The debug freeze (P) does the same
without a menu and allows single steps with N. During the intro and behind the start menu the world ticks (the
vehicle idles at the start pad); Start resets the vehicle with the aid defaults.
