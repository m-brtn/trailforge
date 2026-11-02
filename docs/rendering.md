# Rendering

Renderer, post effects, the shader pipeline, terrain textures, level models, buildings, vegetation, time of day, tire marks and spray. All documentation: [index](README.md).

Overview (Metal only): PBR, cascaded sun/moon shadows, a time of day (0–24 h, presets,
time-lapse) that drives the sun, sky, fog and ambient light, procedural sky with clouds, a large sun disc and layered
distant mountains, height fog, HDR with ACES tonemapping and bloom, MSAA or FXAA, instanced grass (several species,
flower clusters, a wheat test patch, travelling wind gusts) and pines with LOD, planar puddle reflections, headlights
that light the ground at night. A small mud & puddles showcase meadow sits north of the proving ground.

SDL GPU (Metal on macOS; D3D12 or Vulkan elsewhere, untested on real hardware); one HLSL shader source
(`shaders/scene.hlsl`, see [Shader pipeline](#shader-pipeline)). Passes per frame:

1. Sun (or moon) shadow cascades (1–4 layers of a D32 array, practical split, texel-snapped, hardware-compare PCF with
   1/4/12 taps). On levels with a terrain grid (Medium and up, `far_shadows`), one more layer of the array holds a
   terrain-only shadow of the whole level: an ortho view of the terrain square from the shadow light (cell no coarser
   than two texels, about 1.4 m texels on Exploration Peaks at 2048²), rendered only when the light turns by more
   than about 0.26°, the terrain changes or the targets are recreated (the cascade passes then stop cycling the
   array so the layer survives). `shadowFactor` fades from the last cascade into it (4 taps, normal offset of one
   texel plus a bias of 1 m + 2 texels toward the light, which hides the coarse caster grid) instead of into "lit".
   Only the terrain casts there: trees, rocks and buildings beyond the cascades still cast nothing. Cost at
   2880×1800 on a full-screen far view (alternating A/B, minimum frame time): High +0.18 ms static, +0.29 ms with
   the light turning every frame (time-lapse); Ultra +0.13 / +0.39 ms.
   Vehicle environment probe (Medium 64², High 128², Ultra 128² with two faces per frame; `env_probe` = face size,
   0 = off, `env_probe_faces`; F2 "Vehicle reflections"): an RGBA16F cube centred 0.4 m above the vehicle origin,
   one face re-rendered per frame (all six after a jump of more than 8 m) with the vehicle left out: terrain at a
   coarse LOD, the track and scenery meshes, the coarsest mesh LOD of level models and the middle tree LOD within
   70 m, and the sky without the sun disc (the sun's GGX lobe already draws it). Mips are box-filtered
   (`SDL_GenerateMipmapsForGPUTexture`, not a GGX prefilter). Only `lit_fs` draws flagged as vehicle parts
   (`DrawItem::vehicle`) use it: specular along R from mip `sqrt(roughness) × (levels − 2)`, the clear coat from
   mip 0.5 and the diffuse ambient from mip `levels − 1.5` along N (the mean of a cube face, an approximation of the
   irradiance, not spherical harmonics), instead of the procedural sky and the sky / ground ambient palette. So
   paint reflects the ground, berms, trees and houses around it instead of sky everywhere. Without grass blades,
   water surfaces, spray or tire marks in the probe; faces rendered at different frames come from slightly
   different centres while driving. Cost at 2880×1800 (car close-up in the village, alternating A/B, minimum frame
   time): High +0.15 ms, Ultra +0.13 ms (CPU +0.3 ms for two faces).
2. Grass trample mask (RG8, 0.1 m texels over the ground) rendered from the tire mark segments, only when marks were
   added or removed and every 0.5 s while they fade.
3. Planar puddle reflection at reduced resolution (all puddles share one water level), skipped when no puddle is visible.
4. Main HDR pass (RGBA16F, optional MSAA 2×/4×): PBR (GGX + Lambert, sky/ground ambient), triplanar procedural
   rock/concrete/dirt, mask-blended grass/mud/wet ground, tire marks (one instanced draw), instanced grass (2.4 m
   chunks, 3 LOD levels, see "Grass, flowers and wheat"), scattering sky with drifting clouds, height and valley fog.
   Without MSAA the pass is split after the opaque part: linear depth, GTAO and contact shadows at half resolution
   (applied to the indirect and the sun light of the scene, see "Ambient occlusion and contact shadows"), a copy of the
   opaque scene for the water, then water, waterfalls, spray and debug lines.
5. Post effects without MSAA (see [Post effects](#post-effects)): camera + per-object velocity, TAA, motion blur,
   depth of field, sun shafts, eye adaptation.
6. Bloom (5-level downsample/upsample chain).
7. Tonemap (AgX or ACES, exposure, colour grade LUT, lens flare ghosts, vignette, chromatic aberration, film grain,
   dither, sharpening after TAA) with upscale from the render scale; optional FXAA.
8. Menu blur (only while a menu covers the world): dual-Kawase blur of the tonemapped frame, 4 levels down to 1/16
   resolution (5-tap down, 8-tap up), composited with a dim and slight desaturation; then ImGui (debug panels and
   the menus, drawn with the Rajdhani font from `assets/fonts/` at the framebuffer scale, so text stays crisp on
   HiDPI).

### Post effects

All of them need MSAA off (the depth buffer is sampled); High and Ultra use TAA instead of MSAA. Each has a control
in F2 → Graphics → "Post effects" and a scenario `set` key; the cost column is GPU time at 2880×1800 (High, forest
view, measured by repeating the pass with `post_bench`, M-series Mac).

| Effect | Low | Medium | High | Ultra | `set` key | Cost (ms) |
|---|---|---|---|---|---|---|
| TAA (Halton jitter, YCoCg variance clip, Catmull-Rom history) + sharpening | off (FXAA) | on | on | on | `taa`, `sharpen` | 0.95 |
| GTAO (half res, bilateral upsample, indirect light only) | off | 2×4 | 2×4 | 3×6 | `ao`, `ao_strength` | 0.68 + 0.46 |
| Contact shadows (half res, steps toward the sun) | off | 8 | 12 | 16 | `contact_shadows` | 0.38 (12 steps) |
| Motion blur (camera + vehicle body / wheels velocity) | off | 0.4 | 0.5 | 0.5 | `motion_blur` | 0.22 + 0.16 velocity |
| Depth of field (menus, garage; optional far blur while driving) | on | on | on | on | `dof`, `dof_driving` | 3.3 (24 taps, now 16) |
| Sun shafts (radial blur of the sky around the sun, half res) | off | 0.5 | 0.6 | 0.6 | `shafts` | 0.06 |
| Lens flare (ghosts of the bloom around the sun; lamps never ghost) | off | 0.3 | 0.4 | 0.4 | `flare` | in tonemap |
| Eye adaptation (64×64 log luminance, 1×1 adapted) | on | on | on | on | `auto_exposure` | 0.07 |
| Bloom | off | on | on | on | `bloom` | 0.41 |
| Tonemap AgX / ACES + grade LUT (32³) + vignette, grain, chromatic aberration | all | all | all | all | `tonemap`, `grade`, `vignette`, `grain`, `chromatic` | 0.64 |
| Water refraction + screen-space reflections | refraction | both | both | both | `water_refraction`, `water_ssr` | see devlog |

#### Ambient occlusion and contact shadows

The opaque shaders (`lit_fs`, `terrain_fs`, `model_fs`, `impostor_fs`, `grass_fs`) store how their radiance splits in
the alpha of the HDR target (`lightSplit`): the share of sun / moon light in 1/15 steps in the integer part, the share
of sky / bounce ambient and environment reflection in the fraction, both after the fog; the sky stores 0, tire marks
blend only their colour (the ground's split stays). `gtao_fs` writes the AO visibility and a contact shadow per
half-resolution pixel; `ao_blur_fs` gathers both bilaterally and writes `scene × (1 − indirect × (1 − AO) − sun ×
(1 − contact))` into the scene copy, which then becomes the HDR target (no extra copy). So AO no longer darkens sunlit
surfaces, and it is tinted by a multi-bounce fit (Jimenez 2016) with the pixel's hue standing in for the albedo, which
is not stored.

Contact shadows march 0.45 m (growing 2 % per metre of distance) from the surface toward the shadow light against the
linear depth (8 / 12 / 16 steps, jittered, first hit wins, fading out from 48 to 80 m), only from pixels with a sun
share. Grass blades store a negative split: they neither cast nor receive contact shadows (every blade would shadow its
neighbours into a dark speckle). The box `vehicleAO` under the vehicle stays: it is ambient occlusion of the
underbody, which the camera never sees, so neither GTAO nor the contact shadows (sun only) can replace it. Cost:
0.38 ms per pass at High (12 steps, 2880 × 1800, measured with `post_bench gtao 21` and contact shadows off / on).

The grade LUT is built on the CPU (cool shadows, warm highlights, a gentle S-curve and vibrance toward the
`Banner.png` key-art mood). Chromatic aberration is off by default.

The lens flare ghosts mirror the bloom image through the screen centre (four tinted copies, scaled -0.55, 0.42,
-1.25, 0.82), but only the bloom within about 0.2 screen heights of the projected sun feeds them, weighted like the
sun shafts (0 behind the camera, below the horizon, at night and well off screen; scenario value `post_flare`).
Without that gate every lamp ghosted at night as an enlarged copy of itself, far off the vehicle (coloured discs in
the air, red tail-lamp streaks on the ground). `tools/scenarios/lamp_flare.txt` checks it on the Blender-body test
vehicle, the procedural SUV and the EV, and that the flare is still there at sunset.

### Shader pipeline

`shaders/scene.hlsl` holds all 49 entry points (16 vertex, 33 fragment). Each entry's resources are declared inside
`#if defined(TF_ENTRY_<name>)` with the SDL GPU slot conventions: vertex stage textures / samplers / storage buffers
in `space0` (`t`/`s` registers, storage buffers after the sampled textures) and uniform buffers in `space1`;
fragment stage `space2` / `space3`. At build time (`cmake/Shaders.cmake`) every entry is compiled separately:

- `trailforge_shaderc` (`tools/shaderc`, a host tool built from source): glslang HLSL front end → SPIR-V 1.0 for
  Vulkan 1.0 → SPIRV-Tools legalization and validation → `.spv`; on macOS also SPIRV-Cross → MSL 3.0 with the
  resources remapped to SDL GPU's Metal slots (`buffer(n)` uniforms, storage buffers after them, `texture(n)` /
  `sampler(n)`, vertex buffers from `buffer(14)`). It checks that every resource is in the descriptor set of its
  stage and prints the slot counts.
- macOS: `xcrun metal` + `metallib` → one `.metallib` per entry (falls back to embedding the MSL source when the
  Metal toolchain is missing).
- Windows hosts: the pinned prebuilt DXC (`vs_6_0` / `ps_6_0`) → `.dxil`. DXC has no macOS release, so macOS and
  Linux builds carry no DXIL (SDL picks Vulkan / SPIR-V there).
- `cmake/EmbedShaders.cmake` writes every blob into `generated/shaders/shader_blobs.cpp` (table in
  `src/render/ShaderBlobs.h`). Nothing generated is committed; the toolchain is fetched as pinned archives with
  SHA256 like the other dependencies.

Why not SDL_shadercross: its HLSL input goes through DXC, which is not available on macOS arm64. glslang's HLSL
front end is officially deprecated upstream (it still works, and CMake prints a deprecation warning while
configuring); if it breaks, the replacement is DXC → SPIR-V on Windows / Linux hosts.

Metal output was compared with v0.5.0 in one process: each capture of `visual_tour`, `time_of_day`, `grass_look`
and `spray_look` (38 frames) was rendered from the same frame state with the v0.5.0 metallib and with the new
shaders. 3,100 of 197 million pixels differ (0.0016 %), 8 of them by more than 1 / 255 (grass blade edges);
rendering twice with the new shaders gives identical images. `tools/render/compare_captures.py` prints the
statistics for two capture directories (numpy + Pillow).

Backend differences (only Metal is verified on hardware): GPU frame timing is Metal only (`GPU n/a` elsewhere);
on Vulkan the shadow depth bias is not clamped (SDL does not enable the `depthBiasClamp` device feature; Metal and
D3D12 clamp at 0.01); MSAA uses the highest sample count the device supports at or below the preset's (Mesa
lavapipe has 4× but no 2×). The Linux CI job runs the packaged app on lavapipe with the Khronos validation layer
and fails on any validation error or a black frame.

### Terrain textures

The ground of Exploration Peaks uses six CC0 photo-scanned materials from [Poly Haven](https://polyhaven.com) (list,
links and authors in `assets/textures/CREDITS.md`): grass (`forrest_ground_01`), soil (`dirt_floor`), road gravel
(`rocky_trail`), rock (`rock_face_03`), sand (`coast_sand_01`), snow (`snow_02`). The proving ground keeps its
procedural materials.

- **Files:** `assets/textures/terrain/terrain.json` (layer order, tile size in m, optional tint) and per layer
  `<id>_albedo.ktx2` (BC7 sRGB 4096², height in alpha) and `<id>_nra.ktx2` (BC7 UNORM 2048²: DirectX normal x / y,
  roughness, AO), KTX 2.0 with full mip chains down to 1×1, 22.4 MB / 5.6 MB each, 167.8 MB in total, stored in Git
  LFS. `src/core/Ktx2` reads them (and skips the largest levels for smaller sizes without reading them).
- **Rebuild:** `python3 tools/textures/fetch_textures.py` downloads the source maps through the Poly Haven API
  (JPG, cached in `build/texture_cache`), checks their MD5 against `sources.lock.json`, builds `tools/texc`
  (bc7enc_rdo + stb_image, pinned with SHA256, standalone CMake project in `build/texc`) and encodes all 12 maps
  (16 s from the cache on the M-series Mac; the output is deterministic). `--only ID` rebuilds one layer, `--lock`
  records new source hashes. Mips are averaged in linear light; the normal map mips widen the roughness by the lost
  normal variance (Toksvig).
- **GPU format:** BC7 on every backend. SDL GPU also has ASTC formats, but Apple Silicon Metal samples BC7 (verified
  here) and BC7 is the desktop D3D12 / Vulkan format, so one set of files serves all backends (D3D12 / Vulkan are
  untested on real hardware). If the device does not support BC7 (`SDL_GPUTextureSupportsFormat`) or the files are
  missing / LFS pointers, the level falls back to the procedural ground and logs why.
- **Quality:** Graphics → "Level ground textures" (Off, 512, 1024, 2048, 4096; scenario `set terrain_tex N`).
  Presets: Low 512, Medium 1024, High 2048, Ultra 4096 (normal / roughness / AO maps at most 2048). The textures are
  loaded when a level is entered or the size changes (Ultra 167.8 MB in 150–175 ms, High 67.1 MB in 22–80 ms,
  Medium 16.8 MB in 27 ms, Low 4.2 MB in 19 ms) and are not streamed; driving causes no texture loads.
- **Shader (`terrain_fs`):** top-down projection per layer, triplanar for rock on steep faces; layers are blended by
  their height maps (the higher material shows through near a weight of 0.5). Beyond ~15 m a second, rotated
  sample at 0.29× frequency breaks up the repeats; from 60 to 250 m every layer fades to its mean colour (its 1×1
  mip) so the tiles do not line up into a pattern seen from far away. The mud, wet-ground, wheat and distant grass
  canopy colours of the procedural material are kept on top.
- **Cost** (2880 × 1800, Metal, vsync off, village view, same process textured vs Off; `terrain_textures.txt`): GPU
  ms per frame Low 4.78 vs 4.45, Medium 6.79 vs 5.89, High 9.73 vs 8.99, Ultra 13.78 vs 13.16; FPS Low 207.6 vs
  215.1, Medium 171.5 vs 190.7, High 128.3 vs 136.0, Ultra 97.9 vs 99.5.

### Level models

Exploration Peaks places CC0 [Poly Haven](https://polyhaven.com) models (list and authors in
`assets/models/CREDITS.md`): fir trees, fir saplings and two broadleaf trees, shrubs and ferns, boulders and mossy
rocks, two rock faces, a dead trunk, a stump and dry branches. The level's `vegetation` block names them; the bake
picks a species per tree (broadleaf near water, the village and on low ground, saplings among the smaller trees, firs
elsewhere), places model rocks and logs with collision, and scatters render-only ground cover off the roads.

- **Rebuild:** `python3 tools/models/fetch_models.py` (all) or `--only <asset>` downloads the glTF sources through the
  Poly Haven API (checked against `sources.lock.json`) and runs `trailforge_modelc` (`tools/texc`: cgltf +
  meshoptimizer + the BC7 encoder). Per model: a glb with 2–3 mesh LODs, an optional collision mesh (one of the LODs),
  an 8-view impostor atlas for trees, and the material maps as BC7 KTX2 (`<mat>_albedo`, `<mat>_nra`).
- **Tree foliage:** the source crowns are 0.4–4.2 M needle / leaf triangles. modelc splits them into spatial clusters
  (one quad per cluster on its fitted plane, tilted outward) and bakes 64 of the clusters into a per-model atlas the
  quads sample; the impostor is rendered from the full source foliage.
- **Drawing:** instanced, 64 m buckets culled against the view and the shadow cascades, LOD by distance with the same
  dithered cross-fade as the grass and pines; ranges scale with TREE DETAIL DISTANCE (trees: mesh LODs to 0.07 /
  0.15 / 0.45 of it, impostors beyond, out to the tree distance). Foliage is alpha-tested (alpha to coverage with MSAA)
  and sways in the wind (render only).
- **Cliff faces:** the two rock faces (turned by modelc so their mean face normal is +Z) clad walls of 46° and steeper
  with at least 8 m of drop and no water within 4 m below them: a 7 m jittered grid, tilted into the slope and turned downhill, kept only where every
  edge point keeps the back of the slab inside the wall (the models are open shells: a back seen from above a rim
  read as loose shards over the Falls Pool) and no part dips below water within its reach (237 faces). Collision is
  their collision mesh in the tile compound, like the other model rocks. CTest `level_poi_buildings` checks both, plus
  ground cover with its whole footprint on dry ground, the castle ruins on their POI and buildings above the water.

### Buildings

Valley Village and the POIs are built from a modular procedural kit (`src/track/BuildingKit`): 85 placed buildings
of 58 distinct specs. Village houses (stone or plaster ground floor, timber or plaster upper floors, 1–3 floors,
balconies, shutters, framed windows with glass, chimneys, roof overhangs, clay tile or slate roofs, gutters and
downpipes) in six archetypes facing the square, gardens, a well, benches, a power line along the road; the fuel
station (canopy, pump islands, shop, price pylon), the workshop, castle ruins (curtain walls with a breach, two broken
towers, a gate), the lighthouse with its keeper's house, the radio mast (lattice, ladder, antennas) with its hut, the
quarry crusher, conveyor, silo and shed, docks on the Alpine Lake and at the beach camp (placed from the first water
along a shore line), a boathouse, beach and island camps (tents, shelter, woodpile, fire pit), a forest cabin, a pass
hut, cairns and signposts. No interiors: windows are dark glass (constant roughness 0.06, the sky reflection of the
model shader).

- **Same parts for render and collision:** every solid part (wall block, roof slab, post, beam, deck, plinth) is one
  convex collision piece, wound outward, and is drawn unchanged in the far LOD. The near LODs cut window and door
  recesses (at most 0.16 m) into the same wall planes and add trims up to 0.30 m proud (frames, sills, shutters,
  flower boxes, barge boards, balusters, gutters). Render-only: power line wires, mast antennas above 6 m, thin rungs
  and rails between collision posts. CTest `building_kit_parts` checks convexity and winding of every piece and that
  every far-LOD vertex below 6 m lies within 3 cm of the collision (near LODs within 0.33 m), for every kind.
- **Placement:** structures keep 3 m from road corridors, stay out of water and off spawn points (`level_placement`,
  the same check as before); a group that has to move is moved as a whole, and only to a place where the plinths of
  its structures still reach the ground. Floors sit on the highest ground sample under a structure (its plinth reaches
  down 1–3.5 m), so no corner is buried and no plinth hangs (`level_buildings`). Trees and rocks are cleared from the
  footprints.
- **Rendering:** the kit models go through the level model renderer, instanced per material and LOD: LOD 0 to 70 m,
  LOD 1 to 220 m, LOD 2 to 1600 m at High (scaled with the tree detail distance), shadows in all cascades; small props
  (fences, benches, gardens) 30 / 90 / 320 m. Triangles with every building at one LOD: 118k / 66k / 16k; collision
  15.8k triangles.
- **Materials:** 11 CC0 Poly Haven layers in `assets/textures/buildings/` (plaster `white_stucco`, rubble stone,
  castle slates, plank siding, planks, clay roof tiles, roof slates, worn corrugated iron, metal plate, rusty metal,
  concrete; credits in `assets/textures/CREDITS.md`), BC7 KTX2 albedo 1024² and normal / roughness / AO 512², 19.2 MB
  in Git LFS, rebuilt by `fetch_textures.py --set buildings`. The albedo alpha is 1 (`"heightInAlpha": false`):
  the model shader cuts alpha below 0.5. Painted metal, glass, nylon and soil are untextured tints with a constant
  roughness. No glTF props were added (the kit covers them).
- **Map:** building footprints are drawn on the full-screen map and the minimap (roof colour per kind).
- **Views:** `tools/scenarios/buildings_views.txt` (village and every POI from fixed cameras,
  `/tmp/trailforge_bld_*.png`, frame times per preset).

### Canyon and waterfalls

- **Banks:** river banks are carved as cones around each 2 m river segment; two 3×3 smoothing passes over the carved
  cells (outside the water) remove the creases where the cones of neighbouring segments meet and round the rim.
  Banks steeper than 1.5 (the Falls River canyon, 2.2) are terraced: a ledge about every 16 m of height, with a
  position-dependent phase. Collision is the same heightfield.
- **Waterfalls** are render-only meshes generated wherever the river surface drops 2 m or more between two samples: a
  sheet on the ballistic path from the lip (launch speed from the river width), streaks that scroll with the fall time
  and tear toward the foot, a foam disc on the pool and three crossed mist cards that fade edge-on, at the ground and
  near the camera and, without MSAA, where they meet the opaque scene (soft particles) (`waterfall_fs`). Seen from
  above, the smooth upper part of the sheet is glassy dark water reflecting the sky instead of white foam. No water
  flow is simulated.
- **Water surfaces** (lakes, river, sea; render-only, the physics only knows the water height): wind swell as a sum
  of four deep-water sine waves (phase speed from the wavelength) modulated by slow gust patches and calmed in the
  shallows, plus scrolling noise ripples; detail fades with distance. Without MSAA the water samples a copy of the
  opaque scene: the bed is refracted by the wave normal and absorbed per colour channel along the view path through
  the water (Beer-Lambert, coefficients from the water body colour), and reflections are traced in screen space
  against the linear depth (falling back to the sky). Shore foam bands run up the lake and sea beaches in the last
  half metre (rivers have none). The water mesh is the 1 m sample grid: a dry corner of a shore cell sits just under
  its own ground, so the surface meets the bank inside the cell instead of ending in a wall along the cell edge, and
  the shoreline fades over the last 6 cm of depth (height map) and, with the scene copy, the last 0.25 m of view path
  to the ground behind (depth buffer). Only cells next to a waterfall are left out where the surface steps.

### Grass, flowers and wheat

Grass is render-only (no collision, no effect on grip). One instanced draw per LOD; every instance is one blade on a
regular grid per preset (cell 0.2 / 0.15 / 0.1 / 0.075 m), and coarser LODs keep a subset of the same blades. The
vertex shader decides everything per blade from hashes, the ground maps and a tiled noise texture:

- **Species zones** (noise, shared with the ground shader): short lawn grass, tall wild grass (a third of it with
  seed-head panicles), clover leaves, and dry straw-coloured blades on slopes and in sparse margins (near rocks, tufts
  in the dirt). Growth is clumped (0.55 m clumps, blades lean out of the clump centre and share a height), with
  dense patches and sparse spots.
- **Blades:** quadratic Bezier curves (the base leaves the ground at a fraction of the tip angle), wider at the base
  with a pointed tip, normals rounded across the blade, root occlusion that grows with the clump density, lighter
  tips, per-blade and per-clump hue and value, translucency toward the sun (warm at sunset) and a small tip sheen.
- **Flowers** in clusters (at most one per 3.2 m cell, plus rare lone flowers): daisies, buttercups, bellflowers,
  poppies and clover heads, a stem plus a head quad facing up and toward the camera whose petals are cut in the
  fragment shader; they sway with their stems. The far LOD draws a flower as a blade with a coloured tip.
- **Wheat** (mask channel A, the test patch): stalks on rows along x with ears (zigzag spikelets) and awns, golden to
  green-yellow by noise and per stalk, stiffer than grass; the gusts lighten the bent stalks so they roll over the
  field as bright waves.
- **Wind:** travelling gust bands (downwind, with wobbly fronts and a slower swell) bend every species by its own
  flexibility, plus a small flutter across the wind; no independent per-blade jitter.
- **Wheels** flatten every species: blades within the contact radius and in the trample mask lie down (lean up to
  83°, away from the wheel when it is near) and torn turf removes them, so driven lanes stay visible in grass and wheat.
- **Distance:** blades shrink to nothing over the last 20% of the grass distance (Settings → GRASS DISTANCE, F2
  "Grass distance", 20–250 m); the two dense LODs end at 0.3 / 0.6 of it but at most 18 / 40 m, so a longer
  distance only adds the sparse far LOD; the coarser LODs widen; the ground under grass takes
  the same zone colours as a canopy colour, fully at distance and partly near the camera (more on Low/Medium, whose
  grids are sparser), so there is no visible LOD ring. With grass off the ground shows the canopy colour everywhere.
- **Per preset:** density and distance (table below), flower density 0.5 / 0.8 / 1 / 1, fine detail (awns,
  seed-head spikelets, clover leaf marks) off on Low. The Graphics panel has "Flowers" and "Grass detail".

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
- **Sky:** single scattering in a flat exponential atmosphere (Rayleigh per colour channel with an 8 km scale height,
  Mie with 1.2 km, Kasten-Young air mass along the view and toward the sun, an ozone term against green low-sun
  skies), scaled to the palette's zenith brightness and mixed with the palette, which carries night and twilight
  (`atmosphere` 0..1). Fog takes the sky colour just above the horizon in each direction (aerial perspective), and on
  open levels a thin valley fog layer sits above the sea level (`valley_fog`, thicker in the morning). Clouds are
  two layers on planes that drift with the wind. Cumulus 2 km up (9 m/s): two value-noise octaves give the large
  shapes, two samples of the tileable noise texture add the edge detail (zero-mean, so the sky stays in step with
  the ground shadows, which use the two octaves alone). Lighting is a cheap single-layer model: optical depth from
  the core thickness plus the cover 300 m toward the sun, Beer transmittance with a powder term (thin parts scatter
  less back toward a viewer looking away from the sun), a dual-lobe Henyey-Greenstein phase whose forward lobe only
  passes thin parts (silver lining at the edges around the sun), and multiple scattering that the depth attenuates
  less. Cirrus 8 km up (14 m/s) are thin stretched streaks from three texture samples in large patches, lit by the
  sun and the sky, with no ground shadow. Cost (2880x1800, clouds on minus off, sky-filled view): about 0.3 ms on
  High and 0.6 ms on Ultra, the same as the old single-layer ALU clouds within the measurement noise, so every
  preset uses it. There is no raymarched volume: clouds are flat layers, they have no parallax, the camera never
  enters them, and lightning by the sun from below the layer is not modelled. The cumulus layer casts soft moving
  shadows on the terrain, models, grass and water. Large stylized sun disc (bigger near
  the horizon), sunset-lit cloud undersides. Four layers of faceted distant mountains are drawn in the sky pass at infinity (render-only, no
  collision) with aerial perspective toward a violet haze; a valley opens where the sun sets.
- **Backdrop:** flat-shaded terrain beyond the walls with a lower view corridor toward the sunset; ~900 instanced
  low-poly pines (faceted tiers, per-tree size, yaw and tint, wind sway) with three LODs by distance (55 / 140 m),
  culled per frame on the CPU, also drawn into the shadow cascades and the puddle reflection.
- The perimeter wall (5 m, collision) stays and crosses low camera views of the horizon.

Vehicle dirt is **cosmetic only**: a mud layer on the body (splash line from the bottom up) and wheels. In auto mode it
builds up while the wheels roll on `Mud` or through puddles and slowly dries off; it never affects the physics.
The tires additionally show the soil of their own tread (`TreadSoil`, per wheel, see "Tire marks"): the tread
face and grooves load first, then the outer sidewall, the rim only a little; the colour is the carried soil (dark wet
mud, brown dirt) and wet soil is glossy. Dry sand barely sticks: a faint muted tan dust in the grooves and at the
sidewall base, the rubber stays mostly black. Grass leaves the brown earth from under the turf with a few sparse
green specks of torn blades, never a green coat. The mask is in wheel space, so it turns with the Jolt wheel
transform.

**Body mud from the spray** (cosmetic, `BodyMud`): 14 body regions (per wheel the rear and front part of the arch
with its inner liner and the panel behind it, plus the rear and front bumper) collect what each wheel throws
(`wheelThrowRate`, the slip-driven rate of the spray model, independent of the particle cap). A wheel spinning
forward coats the rear part of its own arch first, then the sill and lower rear door (front wheels) or the rear
quarter and the rear bumper (rear wheels); reversing mirrors it, a locked wheel pushes little onto the body. The coat
builds at a rate and up to a level set by the soil's adhesion and wetness: wet mud coats a rear arch to 0.65 within
1 s of wheelspin and to 0.96 within 3 s (headless test), dirt and grass reach ~0.3 (grass: brown earth with sparse
blades), dry sand stays below 0.03 (a light dust film). On hard ground only the soil still in the tread is thrown.
The coat dries lighter over minutes, falls off very slowly, and wheels standing in a puddle at speed wash their
regions (0.96 → 0.17 in 5 s at 3 m/s). The shader places the regions around the wheel arches in body space. It has
no mass, no drag and no effect on the simulation.

### Tire marks (visual only)

Every physics tick `Vehicle` fills one `WheelContactSample` per wheel (`src/vehicle/WheelContact.h`): contact patch
center and normal, suspension load, longitudinal/lateral slip speed of the patch, rolling speed, surface, tire width
and radius. `TireMarks` (`src/sim`, headless, no SDL) turns these samples into one ribbon per wheel: a segment every
0.15 m of patch travel, written into a fixed ring buffer (default 16,384 segments, the oldest is overwritten). Each
segment stores both end points, the half-width vector in the contact plane (width from the wheel), surface, birth
time, and two intensities per end (on the textured level ground the fragment shader rebuilds the ground under a
soft mark from the same splat and terrain layers as the terrain and only darkens / flattens it, so a gravel track is
the road's own stones pressed down, not a painted band):

- **imprint** — rolling imprint on soft surfaces, ∝ load (dirt 0.85, mud 1.0, grass 0.9, sand 0.9, gravel 0.7 at
  static load; gravel is drawn as displaced stones with raised edges, asphalt marks like concrete);
  on hard surfaces it is the soil print of the tread (see below);
- **slip** — rubber on hard surfaces, churn on soft ones, from the patch sliding speed *and* the slip ratio
  (sliding speed / max(patch speed, rolling speed)). The ratio gate keeps normal driving clean: a steady 12 % slip in
  2nd gear at full throttle (≈ 1.7 m/s) leaves nothing on concrete, while lock-up, handbrake slides and wheelspin on
  loose ground do.

Rubber marks use a per-wheel gate (`rubberSlide`): wheelspin / lock-up 1..5 m/s at slip ratio 0.2..0.55, sideways
sliding 2.5..7 m/s at 0.4..0.8. Tire squeal has its own, stricter gate (`squealSlip`, see [Audio](audio.md#audio)), so a launch with
some wheelspin leaves marks but stays silent. Only concrete, asphalt, rock and the low-grip patch squeal; loose surfaces
drive their own layers. Soil in the tread suppresses both. Lock-up, braking slip and sideways sliding follow a patch
"heat" that builds up over ~0.35 s of sliding and drops within ~0.1 s when the slide ends, and scale with √load, so a
locked stop starts faint, stays even and does not darken toward the end (the front wheels lock later than the rears
and are more loaded). A locked wheel keeps marking until the patch slides slower than ~1 m/s.

Driven wheelspin (`wheelspinSlide`, the same gate with the ratio from `WheelContactSample::longSlipRatio`, which the
squeal gate also uses) lays rubber in proportion to a per-wheel tread heat H instead: dH/dt = gate × spinning speed /
15 m × (1 − H) − H / 4 s, so H grows with the sliding distance and cools within seconds (steady state ~0.5 at
3.8 m/s, ~0.73 at 10 m/s). Darkness = gate × min(1, H / 0.5) × √load, width = (0.35 + 0.65 × smoothstep(0.1, 0.45,
H)) × the tire width: a brief hard launch leaves a faint narrow streak, only seconds of fast spin (a burnout) a dark
full-width mark. In-place burnout stamps need a driven wheel spinning faster than the ground (+2 m/s), never a
braking wheel, and each stamp adds only the increase of the laid rubber, so the overlapping stamps composite to the
current darkness instead of darkening with their number. Measured (`trailforge_probe launchmarks`, SUV, concrete,
before → after): gentle 0.3, keyboard W, 0.7 pedal and 4H full pedal no rubber in both (0.7: 0.6 m at 0.03 before);
2H full pedal (2.2 s of spin, peak 4.8 m/s) 18.3 m of ribbon at max 0.76 / mean 0.43, 8.4 m darker than 0.5, 0.32 m
wide → 11.4 m at max 0.16 / mean 0.10, none darker, 0.11 m wide; stationary burnout (brake 0.6 + full throttle, rears
at 7.7–10.4 m/s) 2 s: spot cover 1.00 at 0.32 m → 0.39–0.46 at 0.16 m; 5 s: 1.00 at 0.32 m → 0.95 at 0.31 m.

**Tread soil** (cosmetic, `TreadSoil`): each wheel carries an amount, colour, wetness, adhesion and grass-blade
fraction of soil. Loose ground loads the tread toward its own level and colour by adhesion (mud: 1.0, dark, wet,
within ~0.6 m; packed dirt: 0.5, brown, dry; grass: 0.3, the brown earth under the turf with sparse blades; dry
sand: 0.15, muted tan); colours mix when the ground changes. Puddles rinse the tread to a thin film of muddy water.
On hard ground it sheds over the patch travel (rolling plus 3 × sliding distance) with an e-folding of
1 + 2 × adhesion + 3 × wetness meters: measured 1.0 m for sand, 2.0 m for dirt, 2.3 m for grass (the blades go
within ~1.5 m), 4.8 m for wet mud. While it lasts, the tread prints it onto the hard ground: lug pattern first,
the full width while heavily loaded, tinted by the carried soil, glossy when wet and matte when dry. Measured with
the synthetic boundary test: after wet mud the print starts at 0.94 and is visible for 18.5 m rolling (4–6 m with
wheelspin or lock-up); after dirt 0.21 for 4.9 m, after sand a faint 0.05 for 0.9 m. A ribbon never spans two
surfaces: at a surface change it ends in the old surface's style and a new one starts.

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

### Wheel spray (cosmetic)

`WheelSpray` (in `trailforge_sim`, no rendering dependency) throws material off the wheels: fine droplets, specks and
sand grains, rare small clods, dust puffs (or a short wet haze on mud), torn grass bits and puddle splashes. These are simple CPU particles, **not Jolt bodies**: gravity,
quadratic air drag and a collision with the ground height only (no collision with the vehicle, rocks or each other),
and they never touch the simulation. Emission comes only from the wheel contact samples. The tread surface moves
over the ground at `velocity − rollingSpeed × longitudinal`, so a spinning wheel throws a fan backward and up off the
tread, a locked or nearly locked wheel pushes material ahead of the patch, and a wheel rolling without slip throws
almost nothing (the rate grows with slip above 0.6 m/s, plus a small amount per meter rolled, most on mud).

| Surface | Thrown on wheelspin | Rolling |
|---|---|---|
| Mud | wet droplets (radius 1.5 mm–1.3 cm, skewed to the small end), rare small clods (1–2.6 cm, ~1 per 100 bits), a wet haze | a few droplets |
| Packed dirt | specks (1.2 mm–0.9 cm), rare clods, dust | rare specks, dust above 6 m/s |
| Sand | fine grains (0.8–4 mm), a dust cloud, no clods | a few grains, a little dust |
| Gravel | stones (4–12 mm, lighter tints of the ground), dust | a few stones, dust |
| Grass | thin torn blade bits, soil specks | rare specks |
| Puddles | muddy water drops (by slip and speed) | splashes by speed |
| Concrete, asphalt, rock, low grip | nothing | nothing |

The colour mixes the ground's soil with what the tread carries. Nothing piles up: a landing bit becomes a flat splat
on the ground that shrinks and fades within 0.06–0.12 s (dry grains), 0.15–0.35 s (wet droplets) or 0.3–0.5 s
(clods); water drops vanish on landing; dust grows, drifts and thins out. Rendering: all particles are one instanced
draw of camera-facing billboards from a storage buffer (48 bytes each, full upload per frame); airborne bits are
stretched along their screen-plane velocity over 1/40 s (motion streaks), have noise-perturbed soft silhouettes and
per-particle roughness, wet mud has a specular sheen, and sub-pixel droplets keep a minimum size with reduced alpha
so a fine spray stays visible. Solids are shaded per pixel with the scene lighting (sun with cascaded
shadows, sky ambient, headlights, fog), small bits also scatter some sunlight through; dust is lit once per puff in
the vertex stage with a one-tap shadow per pixel, because large overlapping puffs are the expensive part. The cap is
per quality preset; dust is limited to a sixth of the cap and at most 500 puffs, and Low has no dust.

Quality presets (Graphics panel, F2; default High):

| Preset | Render scale | AA | Cascades × size | Shadow distance | Reflection scale | Bloom | Grass density / distance | Flowers / grass detail | Wheel spray cap |
|---|---|---|---|---|---|---|---|---|---|
| Low | 0.67 | FXAA | 1 × 1024 | 40 m | off | off | 1 / 40 m | 0.5 / off | 600, no dust |
| Medium | 0.85 | FXAA | 2 × 1536 | 60 m | 0.35 | on | 2 / 70 m | 0.8 / on | 1500 |
| High | 1.0 | MSAA 2× | 3 × 2048 | 80 m | 0.5 | on | 3 / 110 m | 1 / on | 3000 |
| Ultra | 1.0 | MSAA 4× | 4 × 2048 | 110 m | 0.75 | on | 4 / 160 m | 1 / on | 6000 |

Grass distance and the tree detail distance (Exploration Peaks; see there for the trees) can also be set on their
own in Settings → Graphics and in the F2 panel; they are generic per-system distances (not tied to the current pine
mesh). Rocks and props have a single mesh each and no distance setting.

The panel also shows CPU frame time (without the swapchain wait), GPU time (from `MTLCommandBuffer`
`GPUStartTime/GPUEndTime`), render/output size, draw calls and grass blades. Every setting can be changed
individually (the preset becomes "Custom"). Physics debug lines are off by default (Tuning → Debug drawing).
