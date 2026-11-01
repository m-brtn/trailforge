# TrailForge documentation

The project overview, a quick start and the controls summary are in the [top-level README](../README.md). The
detailed documentation:

| Document | Contents |
|---|---|
| [Building and running](building.md) | Platforms, prerequisites and pinned dependencies, build presets, the app bundle, packaged builds and releases, command-line options |
| [Controls](controls.md) | Full key and gamepad table, keyboard pedal ramp, cameras (follow, close follow, hood), drive keys, menus, the vehicle page, reverse, driver aid keys |
| [Architecture](architecture.md) | Source layout, coordinate system and units, the main loop |
| [Vehicles](vehicles.md) | The gasoline SUV, the electric SUV, the desert patrol buggy, vehicle rendering and the style kit, tire sets and temperature, drivetrain layouts, the garage, driver aids (ABS, traction control, differentials) |
| [Vehicle data format](vehicle-format.md) | `vehicle.json` reference, Blender bodies (`body.glb`), vehicle blueprints traced from the concept sheet |
| [Levels](levels.md) | The proving ground, Exploration Peaks, Suncoast Raceway, editing a level in Blender, drift scoring, the map, waypoint and compass, the attract drive |
| [Rendering](rendering.md) | Renderer passes, post effects, the shader pipeline, terrain textures, level models, buildings, canyon and waterfalls, grass, time of day, tire marks, wheel spray |
| [Audio](audio.md) | Sound model, mixer, sound assets, offline capture and analysis |
| [Physics simplifications](physics.md) | The honest list of what is simplified or not simulated |
| [Testing and diagnostics](testing.md) | `trailforge_probe` headless measurements, scenario mode (`--script`, command and variable reference), the F1 diagnostics, verification results |
| [Roadmap](roadmap.md) | The next stage: tires and deformable terrain |
| [Release notes](releases/) | Notes per released version |

Coding agents: see [AGENTS.md](../AGENTS.md). The daily development log is in [devlog/](../devlog/).

## Assets and licences

All geometry is procedural. External assets: the sounds in `assets/audio/` (Freesound recordings, see
[Audio](audio.md) and `assets/audio/CREDITS.md`) and the CC0 photo-scanned ground materials of the open-world level
in `assets/textures/terrain/` (Poly Haven, BC7 KTX2 in Git LFS, see [Terrain textures](rendering.md#terrain-textures)
and `assets/textures/CREDITS.md`), and the CC0 trees, ground cover, rocks, cliffs and logs of that level in
`assets/models/` (Poly Haven glTF converted to glb LODs + BC7 KTX2 in Git LFS, see
[Level models](rendering.md#level-models) and `assets/models/CREDITS.md`). Everything else (vehicles, the proving
ground, sky, grass) is procedural; the buggy's proportions were referenced from a purchased model that is not
redistributed.
