# Vehicles

The shipped vehicles, the style kit, tire sets, drivetrain layouts, the garage and the driver aids. All documentation: [index](README.md).

## Vehicle

The vehicle comes from `assets/vehicles/trailforge_suv/vehicle.json` (format below); the C++ defaults of
`VehicleConfig` are the v0.7 SUV (full-time AWD, no drag, 0.80 m tires, final drive 3.90), used by the physics tests.

- Mass 2000 kg, wheelbase 2.80 m, track 1.80 m (v0.15.0–v0.22.0 used the 2.56 / 1.75 m measured on the concept
  sheet for the traced body; the kit body is laid out for 2.80 / 1.80). The tires stick out past the flares as on
  the sheet.
- Tires (since v0.9.0): 315/70R17 mud-terrain on 17-inch rims, the 35-inch class of the concept sheet (Cars.png
  car #1 scaled by its wheelbase suggests ~0.90 m; it was 0.80 m before). Real dimensions: 17 in rim = 431.8 mm,
  sidewall 0.70 × 315 = 220.5 mm, diameter 872.8 mm, so `radius` 0.436 m, `width` 0.315 m (section width),
  `rimRatio` 0.495. Mass 43 kg per wheel (estimate: LT315/70R17 mud-terrain tires are listed at roughly 28–32 kg,
  17 × 8.5 in alloy wheels at 12–14 kg); rotational inertia 4.7 kg·m² (tread and belt 18 kg as a ring at 0.41 m,
  sidewalls 12 kg as an annulus 0.216–0.40 m, rim barrel 8 kg at 0.20 m, wheel center 5 kg as a disc) instead of
  the solid-disc default (2.8 kg·m² for the old 35 kg wheel). Jolt's wheels are virtual: the mass only enters the
  rotational inertia, not the unsprung mass. The bigger tires lift the body by the radius difference (3.6 cm; the
  suspension is unchanged).
- Final drive 4.27 (was 3.90; a real ring-and-pinion ratio, e.g. Ford Bronco), re-geared for the 9 % taller tire
  like a real 35-inch build: 4.27 / 0.436 = 9.79 vs 3.90 / 0.40 = 9.75 per meter of radius, so road speed per rpm
  stays within 0.5 % (top gear 45.3 vs 45.5 km/h per 1000 rpm, first gear 9.17 vs 9.20). Without the re-gear
  (3.90 on the new tires, probe `layouts`): 4H 0-30 km/h on concrete 1.59 s vs 1.53 s, 4L 2.33 s vs 2.40 s, the
  4L 15° mud climb 6.16 s vs 7.04 s (taller, so it carries more speed onto the ramp).
- Center of mass 0.10 m below the frame center.
- The body collision is a compound of three boxes (`vehicle/VehicleShape.h`). The visible model is the kit body, a
  procedural five-door expedition SUV (Cars.png car #1, ~17.9k triangles, 40 named parts) built around those
  boxes and fitted to the physics wheels; see "Vehicle rendering and the style kit". The body traced from the
  concept sheet (v0.15.0–v0.22.0) is still available as `"bodyShape": "traced"`; see [Vehicle blueprints](vehicle-format.md#vehicle-blueprints-bodies-traced-from-the-concept-sheet).
- Wheels (lugged tire, rim, hub) and suspension parts (two arms, half shaft, shock body and shaft, coil spring)
  are placed every frame from the Jolt wheel transforms and suspension length; there is no decorative animation.
- Suspension: travel 0.15–0.55 m, 1.35 Hz, damping 0.35 compression / 0.55 rebound with a digressive knee at
  0.25 m/s, bump stops over the last 6 cm, tire rate 250 kN/m, anti-roll bars off by default.
- Part-time 4WD (since v0.8.0; full-time AWD before): starts in 2H (rear-wheel drive), V engages 4H (front axle,
  center locked) and 4L; see "Drivetrain layouts".
  - engine 520 N·m, 800–4800 rpm;
  - 5-speed automatic plus reverse −4.0, final drive 4.27 (3.90 before the v0.9.0 tires);
  - low range ×2.72 — a multiplier on the differential ratios, not a separate gearbox;
  - open front differential, rear and center are LSD (ratio 1.4); X locks rear, rear + center or all
    (see "Driver aids");
  - aerodynamic drag Cd·A 1.3 m² (no lift or downforce).
- Wheels use `VehicleCollisionTesterCastCylinder`; their positions come from the simulation.
- Roll and pitch limits are disabled (`mMaxPitchRollAngle = π`): the vehicle can roll over.
- No hidden downforce, no direct setting of velocity or position.
  The only teleports are the R reset and the teleport buttons in the UI.

### Electric SUV (`trailforge_ev`)

`assets/vehicles/trailforge_ev/vehicle.json`: a dual-motor electric off-roader on the SUV's wheelbase and track
(2.80 / 1.80 m; 2.44 / 1.77 m from the concept sheet in v0.15.0–v0.22.0). The numbers are estimates for the current class of large electric off-road SUVs, not a copy of one vehicle.
Class reference (manufacturer figures as commonly published, not re-checked for this build): Mercedes G 580 EQ about
3.1 t with a 116 kWh pack, Rivian R1S about 2.7–3.2 t with 92–149 kWh packs, both with 0–100 km/h around 4–5 s.
TrailForge EV sits at the light end of that range.

- Mass 2650 kg (+650 kg over the SUV: pack and motors minus engine, gearbox and transfer case), front weight 0.50
  (SUV 0.518), center of mass 0.15 m below the frame center (SUV 0.10: the pack is in the floor). Inertia
  [3490, 3670, 925] kg·m² (pitch, yaw, roll), radii of gyration 1.15 / 1.18 / 0.59 m.
- Motors, one per axle, each with a fixed 12.0:1 reduction (no gearbox, no clutch): front 400 N·m / 180 kW, rear
  450 N·m / 220 kW, both 15000 rpm max. Envelope: constant torque to the base speed (maxPower / maxTorque: front
  4297 rpm, rear 4669 rpm, about 56 / 61 km/h), then constant power, fading to 0 over the last 2 % below the max
  rpm (computed rpm-limited top speed about 196 km/h, not driven). At the wheels: 4800 + 5400 N·m. Rotor inertia 0.05 kg·m²
  per motor, reflected into the wheels as 0.5 × 0.05 × 12² = 3.6 kg·m² per wheel.
- Split: NORMAL 0.40 front / 0.60 rear, OFF-ROAD 0.50 / 0.50 (V). What one axle cannot take (motor limit, traction
  control) goes to the other. Each axle has an open differential (torque split by Jolt's differential ratio).
- Regen: lift-off target 1.2 m/s² (both pedals released), then the brake pedal's friction torque request per axle up
  to the motor regen limit (300 N·m shaft per motor, same power envelope), faded out below 2 m/s, reduced on wheel
  slip or ABS action, limited to 150 kW battery charge power and off at a full battery (SoC ≥ 99.5 %). The friction
  brakes get the rest of the pedal request.
- Battery: 120 kWh usable, starts at 80 %, drive efficiency 0.90 (battery → shaft), regen 0.85 (shaft → battery),
  0.8 kW auxiliary load. Below 2 % SoC the drive torque fades to zero.
- Tires 285/70R17 all-terrain (17 in rim 431.8 mm + 2 × 0.70 × 285 mm = 830.8 mm, `radius` 0.4155 m), 40 kg,
  4.0 kg·m².
- Retuned for the mass: springs 1.45 Hz (SUV 1.35), damping 0.50 compression / 0.55 rebound (SUV 0.35 / 0.55),
  anti-roll bars 17500 / 8800 N/m at the wheel (400 / 200 in Jolt's bar units before they became suspension forces;
  converted to keep the corner roll), bump stops over the last 6 cm, travel 0.15–0.53 m (SUV 0.55); brakes 4800 N·m front / 2400 N·m
  rear per wheel (SUV 4200 / 2200; both 3800 / 3200 and 3000 / 3000 before v0.24.0); longitudinal tire peak 1.10
  (all-terrain tread on concrete; lateral 1.00, SUV mud-terrain 1.00 / 1.00).
- Driver aids as on the SUV (B, T, X); X locks the per-axle differentials (there is no center differential).

Measured against the SUV with `trailforge_probe compare` (`TRAILFORGE_VEHICLE` selects the vehicle; 120 Hz, TC and
ABS off unless stated):

| | SUV (2000 kg) | EV (2650 kg) |
|---|---|---|
| 0–30 / 0–60 / 0–100 km/h, concrete | 1.93 / 4.62 / 9.71 s | 1.42 / 2.48 / 4.31 s (peak 401 kW, 0.357 kWh) |
| 50–0 km/h ABS off / on | 14.07 / 11.96 m | 14.57 / 11.57 m (regen 3 / 4 % of the kinetic energy) |
| 100–0 km/h ABS off / on | 51.39 / 42.85 m | 50.39 / 39.92 m (regen 1 / 3 %) |
| lift-off at 80 km/h | 0.63 m/s² (drag, engine braking) | 1.36 m/s² (regen and drag) |
| corner 40 km/h, steer 0.25 / 0.45 | 6.87 / 7.91 m/s², roll 4.08 / 4.95° | 7.33 / 8.34 m/s², roll 2.23 / 2.66° |
| climb dirt 20°, full pedal | 4.33 s (4H), 5.06 s (4L) | 3.69 s (NORMAL), 3.71 s (OFF-ROAD) |
| climb mud 15°, full pedal | 6.12 s (4H), 7.03 s (4L) | 6.08 s (NORMAL and OFF-ROAD) |
| diagonal twister, travel used FL / FR / RL / RR | 0.328 / 0.301 / 0.400 / 0.375 m, 6.18 wheel-s in the air | 0.347 / 0.240 / 0.380 / 0.373 m, 5.24 wheel-s |
| steady 50 / 90 km/h energy | — | 14.2 / 28.9 kWh/100 km (drag, wheel damping, aux; no rolling resistance) |

Re-measured 2026-10-27 after the suspension force rework (per-step rates, anti-roll bars as forces; before it: SUV
0–100 9.68 s, 100–0 ABS off 51.43 m, twister 6.01 wheel-s; EV corner roll 2.16 / 2.59°, twister 4.28 wheel-s).
Measured on v0.24.0 (kit bodies, box collision, 2.80 / 1.80 m; front-biased brakes, ABS re-apply memory, 8 1/s
foot-brake rise). On v0.23.0 the stops were SUV 50–0 13.70 / 13.49 m, 100–0 47.57 / 46.42 m, EV 14.95 / 13.88 m and
52.32 / 46.37 m (regen 12 / 16 and 5 / 9 %), the EV 0–100 4.36 s; the corner values and the twister contact counts
equal the v0.14.0 ones. The SUV's ABS stop uses 0.90 of its load in braking force (`trailforge_probe brakedetail`):
at μ 0.95 the ideal stop is 41.4 m, so the rest is the mud-terrain tire curve on rigid concrete. Real large SUVs on
road tires stop from 100 km/h in about 36–40 m, on mud-terrain tires in the low-to-mid 40s. v0.15.0–v0.22.0 (traced
bodies on the concept wheelbase and track) for comparison: SUV 0–100 9.58 s, 100–0 ABS on 47.31 m, corner 7.39 /
7.78 m/s²; EV 0–100 4.42 s, 100–0 ABS on 46.43 m, corner 8.24 / 8.15 m/s². On the diagonal twister the box collision
touches the terrain in 34 (SUV) and 30 (EV) of 1200 ticks, the traced body did in 0 (`trailforge_probe susp 2`).

The full-pedal launch with TC off spins the front wheels: the front axle's 4800 N·m exceeds its grip once the weight
moves back (slip speed up to 18 m/s, about 2.5 s). Traction control (T) limits it per axle.

### Desert patrol buggy (`trailforge_patrol`)

`assets/vehicles/trailforge_patrol/vehicle.json` ("Desert Patrol Buggy"): a two-seat tube-frame buggy with a
mid-mounted engine under the cargo bed. The body is procedural StyleKit geometry like the other kit bodies (`kit::patrol` in
`src/render/VehicleModel.cpp`); its proportions (wheelbase, track, ride height, tire size, cage and roof line, seat
position, coil-over mounts) were measured from a purchased model that is not part of the repository and not
redistributed. The reference is a two-seat 4x4 utility side-by-side with the engine behind the seats, so the drive is
on-demand AWD rather than RWD.

- 750 kg, front weight 0.42, wheelbase 2.07 m, track 1.35 m, tires 0.33 m radius / 0.229 m wide (beadlock rims),
  32 cm of suspension travel (front 0.186–0.503 m, rear 0.193–0.510 m), springs 1.35 / 1.55 Hz, front anti-roll
  3500 N/m at the wheel, brakes 900 / 550 N·m per wheel, no ABS or traction control by default (B / T turn them on), drag area 1.9 m².
- Engine with the character of a 1.0 L high-revving inline-four: 112 N·m peak at 8400 rpm, about 119 kW (160 hp) at
  11000 rpm, limiter 11800 rpm (12000 rpm curve end), 0.06 kg·m² flywheel; six speeds 3.30–1.15 with an 8.0 final
  drive and 0.05 s shifts (up at 11500, down at 7000 rpm), no low range (L reports "No low range on this vehicle").
  On-demand AWD: 10 % front by default, front LSD, rear locked, center LSD.
- Body: open sand-coloured tube cage with a roof X brace and a light bar, satin black hood, nose and skid plate,
  tube bumpers with orange tow hooks, two seats, dash and steering wheel, cargo bed with two strapped cases and the
  spare lying flat. No fenders, arches or flares: wheels, lower arms and coil-overs are exposed. The coil-overs
  (orange springs) run from shock towers welded to the front and rear frame to the lower arms; their mount points are
  part of the visual (`SuspensionMounts`) and the arms and shocks are placed from the Jolt hub every frame.
  `model_suspension_mounts` checks at full droop, rest and full bump over the steering range that the shock body and
  shaft overlap (0.335–0.597 m between the eyes, attached up to 0.75 m), the top eye sits on the frame and the coil
  clears the tire (20 mm with the shipped wheels).
- Collision: five boxes of its own (floor tub, cage, nose, engine bay, cargo bed; `kPatrolBoxes` in
  `src/vehicle/VehicleShape.h`), 1.36 × 1.12 × 2.95 m.

`trailforge_probe compare` (120 Hz, TC and ABS off unless stated):

| | SUV (2000 kg) | Buggy (750 kg) |
|---|---|---|
| 0–30 / 0–60 / 0–100 km/h, concrete | 1.92 / 4.59 / 9.68 s | 1.63 / 2.90 / 5.48 s |
| 50–0 km/h ABS off / on | 14.06 / 11.97 m | 14.12 / 12.66 m |
| 100–0 km/h ABS off / on | 51.43 / 42.85 m | 46.43 / 41.75 m |
| lift-off at 80 km/h | 0.63 m/s² | 1.00 m/s² (drag, engine braking) |
| corner 40 km/h, steer 0.25 / 0.45 | 6.87 / 7.91 m/s², roll 4.08 / 4.95° | 7.12 / 6.55 m/s², roll 4.17 / 3.72° |
| climb dirt 20° / mud 15°, full pedal | 4.33 / 6.12 s (4H) | 4.03 / 5.83 s |
| diagonal twister, travel used FL / FR / RL / RR | 0.309 / 0.244 / 0.402 / 0.404 m | 0.321 / 0.295 / 0.327 / 0.325 m, 6.02 wheel-s in the air |

The engine sound uses the `sporty` profile (car recordings pitched to the engine rpm); nobody has listened to it at
11000 rpm.

### Vehicle rendering and the style kit

All vehicle geometry is procedural and visual only. The shipped bodies are kit bodies (`bodyShape` `"kit"`, the
default): StyleKit panels around the three collision boxes of `vehicle/VehicleShape.h`, fitted to the physics wheels.
v0.15.0–v0.22.0 shipped the traced bodies instead (see [Vehicle blueprints](vehicle-format.md#vehicle-blueprints-bodies-traced-from-the-concept-sheet)); v0.23.0 brought the kit bodies of
v0.14.0 back, with the later changes kept (below).

- **Style kit** (`src/render/StyleKit`): the shared building blocks for every car model.
  - Materials: clear-coat paint without metallic flakes (stylized-realistic), grained matte plastic trim
    (`MatId::Cladding`: cladding, flares, bumpers, pillars), metal, rubber, glass (opaque dark tinted glass: no
    interior is drawn behind it, the shader keeps the sun highlight and 30 % of the sky reflection), emissive LEDs
    (headlight / brake / reverse / indicator ids, scaled by the vehicle state in the shader); a `Livery` groups base paint, accent paint
    (two-tone panels, decal stripes), roof, trims.
  - Surfaces: `skin` lofts rows of points into quads with per-face orientation and normals averaged only within
    38 degrees, so chamfers and creases stay hard edges that catch the light and curved parts shade smooth;
    `roundedRect` builds rounded footprints (body sides, greenhouse) with matching stations between rings;
    `grid`, `prism`, `face`, `panelGap` (panel gaps and decal stripes).
  - Lights: LED rings (DRL outline), strips, round lamps. Accessories: low roof rails with crossbars (not a rack),
    cargo straps, light bar, winch, bow shackles, slim fender flares over any arch outline (rounded lip, chamfered
    band), profile sweeps (bumpers), jerry cans, cargo cases, rock sliders, mirrors, spare carrier. The kit still has
    a snorkel and a ladder; no model uses them (the concept has none). No mud flaps (the concept has none). No
    antennas: they were left out on purpose (the concept sheet has them).
  - Wheels: `buildWheel(radius, width, WheelStyle)` with spoke count, rim fraction, optional beadlock, mud- or
    all-terrain tread (chevron blocks, shoulder lugs wrapping the sidewall), bulging sidewalls, deep-dish rim with
    recessed spokes. The mesh fits exactly in the physics wheel: its outer radius is the physics radius (the
    beveled block corners end up to 1 mm inside it) and its axial extent is the physics width; the bulge and the
    lugs stay inside. Size never comes from the model, only from the axle.
- **Parts**: a model is a list of named parts (`VehiclePart`: name, parent, attachment point, flags `deformable`
  and `detachable`, vertex range), merged into one mesh and one draw call. The flags only mark what a later damage
  stage may bend or knock off; there is no damage logic. Spare wheels are mount points (`SpareMount`) drawn with
  the rear wheel mesh, so they follow the rear tire size.
- **trailforge_suv** (~17.9k body triangles, 40 parts): lower body swept from one profile (chamfered rocker with
  black cladding, shoulder crease, chamfered shoulder, belt deck) around a rounded footprint, arches clipped out of
  it; greenhouse lofted from the deck (8 cm inside the body side) to the roof with a raked windshield and
  tumblehome; hood with crown, panel gaps and a nose dropping to the grille. Paint by panels with straight borders:
  olive body (`style::oliveExpedition`), orange fender tops, side stripe, rear quarter panel and roof band, white
  roof, black pillars, flares, bumpers. Parts include doors, hood, tailgate, windshield, fenders, flares, bumpers
  (the rear one swept around the body corners with chamfers, a groove and lamp pockets), winch, tow hooks, lights,
  mirrors, roof rails, light bar, strapped cases and a jerry can on the crossbars, rear jerry cans, rock sliders and
  one spare mount (tailgate). Left out: the concept's roof spare (with the 315/70R17 tire it would cover two thirds
  of the rail spacing and stand 0.32 m tall), antennas, snorkel, ladder, mud flaps.
- **trailforge_ev** (~15.8k body triangles, 37 parts): shares the fitted lower body, glasshouse and hood; white /
  light grey paint, blue accents and door stripe, black cladding, a closed front panel with a full-width LED bar,
  headlights with LED rings, wrapped bumpers with blue LED lamps and blue tow hooks, a full-width tail light, a
  quarter window with a slanted blue graphic, low rails with a light bar (blue end pods, blue LED strips) and one
  flat case; under the body the floor battery pack and one motor per axle. No winch, no spare (the concept has
  none), no snorkel.
- **trailforge_patrol** (~12.4k body triangles, 28 parts): the tube-frame buggy, no fenders (see "Desert patrol
  buggy"); the traced body shape also builds the kit.
- **Suspension stays visible**: the coil-overs (orange springs on the SUV, blue on the EV, as on the sheet) are
  seen through the arch openings in front of the dark wheel-house walls and are placed from the physics suspension
  length every frame.
- **Wheels follow the physics**: the wheel meshes are built per axle from the active configuration (definition,
  kit, setup): `wheel.radius` and `wheel.width` give the tire size, the optional visual keys `wheel.rimRatio` and
  `wheel.tread` the rim and the tread. The arches, the flares and the wheel-house clearance are derived from
  the wheels too: every arch point is pushed out until it clears the tire swept over the whole suspension travel
  (`attachHeight` minus `minLength` .. `maxLength`) and 0..`maxSteerAngleDeg` with a 3 cm margin, on top of a
  circular arc of 1.2 wheel radii (vertical down to the rocker); the flares are a slim band along that arch
  (0.30 × the wheel radius wide, clamped to 10–14 cm, 7.5 cm proud of the body side), so the tires stick out past
  them as on the sheet. When a kit or setup changes any of these values the app rebuilds the body and wheel meshes.
  Tests: `model_wheel_size` (mesh radius / width equal the physics wheel for several sizes, rim ratios and treads)
  and `model_wheel_clearance` (no body triangle point inside any wheel's sweep for the default wheels and bigger /
  smaller tires, kit and traced bodies). The scenario `tools/scenarios/wheel_fit.txt` changes the tire size in the
  running app (`wheels R W`) and checks `wheel_fit_err` after each rebuild.
- Body mud and tread soil are cosmetic layers: the mud mask is computed in the shader from the body-space position
  and the per-vertex receptivity of each material; the arch and bumper zones come from the model's axle positions,
  arch height and body width (`VehicleVisual::mudArch` / `mudBody`).

### Tire sets and temperature

Three tire sets (`src/vehicle/TireModel`), chosen in the garage (`tires.set`, rebuild) or in `vehicle.json`
(`tires.set`). The vehicle's own `tires` curves describe its off-road (all-terrain) tire; a set multiplies them, so
OFF-ROAD multiplies by exactly 1 and behaves as before (the `tire_sets` test checks the tuning is bit-identical).

| Set | Surface multiplier (asphalt / concrete / gravel / dirt / grass / mud / snow) | Peak slip / slip angle / sliding μ | Window | Cold / hot loss | Crr |
|---|---|---|---|---|---|
| OFF-ROAD | 1 everywhere | ×1 / ×1 / ×1 | −10..100 C | 10 % over 30 K / 20 % over 60 K | 0.013 |
| ROAD | 1.15 / 1.15 / 0.85 / 0.90 / 0.75 / 0.55 / 0.60 | ×0.8 / ×0.75 / ×0.95 | 40..95 C | 20 % over 45 K / 15 % over 40 K | 0.010 |
| SLICK | 1.40 / 1.35 / 0.70 / 0.75 / 0.55 / 0.35 / 0.40 | ×0.7 / ×0.65 / ×0.90 | 80..110 C | 40 % over 60 K / 25 % over 40 K | 0.014 |

Sources for the shape (the numbers are hand-tuned within them): peak μ on dry tarmac ~0.9–1.0 for all-terrain, ~1.1
for performance street, 1.3–1.5 for racing slicks; stiffer tires peak at smaller slip (Milliken & Milliken, Race Car
Vehicle Dynamics ch. 2; Pacejka, Tire and Vehicle Dynamics ch. 1; Wong, Theory of Ground Vehicles ch. 1). Open tread
blocks bite into loose ground, a smooth tread floats on it (Wong ch. 2); rolling resistance 0.010–0.015 (Gillespie,
Fundamentals of Vehicle Dynamics; TRB Special Report 286). Rubber friction peaks at a temperature set by the compound's
glass transition (Grosch 1963), hence a window per compound; the window limits and losses are hand-tuned.

**Thermal model (per wheel, every physics tick, explicit Euler).** Two nodes: a 2 mm tread surface layer (S) and the
core (C: the rest of the tread, belts and carcass; 0.6 × the wheel mass × the set's construction mass: OFF-ROAD 1,
ROAD 0.6, SLICK 0.5 from published tire weights per diameter × width: BFGoodrich KM3 35x12.50 R20 LT 74.5 lb,
Michelin Pilot Sport All Season 4 305/40 R22 41 lb, Hoosier 29.5/11.5 R18 radial 29 lb). The structure and the
coefficients follow the lumped tread / carcass models fitted to measured tread temperatures (Kelly & Sharp 2012;
Tremlett & Limebeer 2016; West & Limebeer 2021; the parameter survey and FSAE TTC fit in a Politecnico di Torino thesis,
webthesis.biblio.polito.it/32743, Tables 4.2–4.6).

- Heat in: friction power in the patch P = |Fx·Vsx| + |Fy|·|V|·sin|α| (Jolt's forces and slip of the step). On paved
  ground λ = 0.6 of it heats the tread (fitted 0.49–0.68 by West & Limebeer, 0.4 by Tremlett & Limebeer, 0.40–0.55
  on TTC data), the rest the road; on loose or wet ground less (× dirt / low grip 0.6, gravel / grass 0.4, sand 0.3,
  mud / snow 0.2: the slip is soil shear, Wong ch. 2; hand-tuned). Carcass deflection and rolling hysteresis
  (Ex·|Fx| + Ey·|Fy|)·|v_ground| + Crr·N·|v_roll|, Ex = Ey = 0.03 (Kelly & Sharp; 0.01–0.05 in the other fits; the
  fits' Ez·|Fz| term is the set's Crr): 20 % into S, 80 % into C. Like the set's Crr, the deflection loss only heats
  the tire; no matching drag force is applied.
- S ↔ C: 200 W/(m² K) over the tread band (West & Limebeer fit 182–311; the TTC fit 64–80 W/K on a 0.3 m² band).
- S → air over the tread band, C → air over the sidewalls: h = 5 + Churchill–Bernstein for a cylinder of the tire
  diameter at the vehicle speed (still air).
- S → ground: patch area = load / inflation pressure, contact time = patch length / rolling speed (≥ 0.5 m/s);
  h = 2·e_eff / √(π·t_c) for two semi-infinite bodies (Carslaw & Jaeger) with the rubber and ground effusivities
  (concrete 1680, asphalt 1260, gravel 780, snow 450 W s^0.5/(m² K), from Incropera), over 30 % of the patch (real
  contact area, hand-tuned), in series with conduction through the surface layer. Ground temperature = air
  temperature (snow at most 0 C).
- C → rim → air: 5 W/K.
- Pressure: ideal gas at constant volume from the core temperature, 240 kPa gauge set at the start temperature
  (telemetry only).
- Grip: the friction callback multiplies the surface μ by the set's factor at the surface temperature: 1 inside the
  window, a smoothstep down to (1 − loss) over the cold / hot width outside it. The factor computed after a tick is
  used in the next one.

Test tire (R 0.40 m, 0.30 m wide, 35 kg wheel, OFF-ROAD): surface 3121 J/K, core 28899 J/K, S ↔ C 150.8 W/K, tread band 0.754 m²,
sidewalls 0.643 m². Air temperature: `level.json` `ambientTemperature` (−40..50 C, default 20 C), the Tuning panel
slider, or `tires ambient C`. Tires start at the air temperature on spawn, reset and every rebuild; the Tuning panel
and `tires warm` put them in the middle of the window (as if taken off tire warmers). Telemetry shows a 2×2 grid
(surface temperature, grip factor, core temperature, pressure, coloured blue cold / green in the window / red hot),
the wheel table the heat flows; the drive HUD shows a compact grid with the set name. The wheel mesh has a road tread
(shallow grooves) or no tread (slick) per set at the same physics size.

ABS and traction control slip targets follow the set's peak slip ratio (× 0.8 ROAD, × 0.7 SLICK), so they keep the
same place relative to the curve's peak as on the vehicle's own tire.

Measured with `trailforge_probe tires` (see [Verification results](testing.md#verification-results)). The racing autopilot spends only 10–40 % of a lap
with a tire at ≥ 0.8 of its peak slip, so in autopilot laps from cold the treads reach 42–61 C after 3–5 laps and cold
slicks stay below their window (grip ×0.72–0.86). Held at the limit on a 40 m circle (`tires circle`) the tread
average reaches 49–79 C after 150 s (hottest tire 66–124 C). Taken off warmers, slicks keep a 68–78 C tread after one
lap; a slow lap from 95 C ends at 69–70 C.

## Drivetrain layouts

Set by `drivetrain.layout`; the Jolt `WheeledVehicleController` gets one differential per axle and a center LSD
setting. An undriven axle has its differential disconnected (wheel indices −1): it gets no torque and does not count
for the clutch speed; it still brakes.

| Layout | Front | Rear | Center | Notes |
|---|---|---|---|---|
| RWD | free | driven | — | |
| FWD | driven | free | — | |
| AWD (full-time) | driven | driven | `centerDiff` (open / LSD / locked), static split `frontTorqueShare` | the v0.7 SUV |
| Part-time 4WD | 2H free, 4H / 4L driven | driven | 4H / 4L: locked, split 0.5 | V: 2H → 4H → 4L → 2H; L: 2H / 4H → 4L, 4L → 4H |
| On-demand AWD | driven | driven | LSD with `frontTorqueShare` (e.g. 0.1) | simplification, see below |
| Dual motor (electric) | own motor | own motor | none: electronic split `frontTorqueShare` / `offRoadFrontTorqueShare` | V: NORMAL ↔ OFF-ROAD; no low range |

Headless launch on low grip (`drivetrain_layouts`, speed after 1.5 s): RWD 2.41, FWD 2.21, AWD 4.56, on-demand
(front share 0.1) 4.54, part-time 2H 2.40, 4H 4.56, 4L 3.66 m/s; undriven wheels slip 0.00. `trailforge_probe
layouts` prints launches on concrete, mud and low grip and climbs on the 20° dirt and 15° mud ramps for every
layout with the probe vehicle; with the SUV, 2H stalls on the 15° mud ramp (13 m of 30 m) where 4H tops it in 6.1 s.
The diff-lock key X still works on top (rear, rear + center, all).

**Dual motor in Jolt.** Jolt's `WheeledVehicleController` has one engine, one clutch and one transmission shared by
all differentials, so two independent motors cannot be set up with its settings. `ElectricVehicleController`
subclasses it and overrides `PostCollide`: for each axle the motor torque (from `ElectricDrive`: envelope, split,
traction control, regen) times the reduction is split by the axle differential's torque ratio and integrated into the
wheel angular velocities, the same way Jolt applies its drivetrain torque; the tires then pass it to the ground
through Jolt's normal tire constraints. Friction brakes are applied as in Jolt. Nothing pushes the body. The motor
speed is the reduction times the average wheel speed of its axle; the battery integrates the shaft power each substep.

## Garage

Start menu → Garage, or pause menu → Garage. The simulation keeps running (also when opened from the pause menu)
so changes show on the vehicle; closing returns to the menu it came from. Layers: base definition → kit (data
structure and loader; no kit content for the SUV yet, the test vehicle has one) → fine tune within the ranges of the
definition / kit (or the defaults of `VehicleSetup.cpp`, always containing the base value):

- Suspension: spring frequency, compression and rebound damping, preload, anti-roll bar per axle; ride height.
- Tires: tire set (OFF-ROAD / ROAD / SLICK, rebuild; see "Tire sets and temperature"), grip scale (a stand-in for
  pressure: scales both peak and sliding μ), peak slip ratio and peak slip angle (stand-ins for longitudinal /
  cornering stiffness: they move the peak of the μ(slip) curves), static grip on / off. The pressure shown in
  telemetry follows the tire temperature; it is not a garage setting and does not change the curves.
- Engine and gearbox: torque scale, rev limit, final drive. Brakes: torque scale, front share.
- Drivetrain: layout, front torque share, front / rear / center differential, LSD ratio.
- Driving aids: ABS release slip, TC slip thresholds (the on / off switches stay in Settings and the B / T keys).
- Electric only (group MOTORS AND REGEN): motor torque scale, regen torque scale, lift-off regen deceleration,
  OFF-ROAD front share. Rows that do not apply to the vehicle are hidden (combustion only: engine torque scale, rev
  limit, final drive, layout, center differential); a setup file with such keys loads with the message "does not
  apply to this vehicle, ignored". Setups are per vehicle id, so each vehicle keeps its own list.

Live values change the running vehicle between physics ticks; layout, preload and ride height mark "RESPAWN" and
rebuild the vehicle in place (same position and heading, lowered onto the ground by a raycast, aid switches and the
transfer mode kept). Setups (several per vehicle: Factory is read-only; save, save as new, rename, delete, reset to
factory) are saved as JSON in `<user data>/setups/<vehicle id>.json` (`trailforge --paths`); the selected setup is
applied at the next start. Files are `"schema": 2` since v0.28.0 (anti-roll bars in N/m at the wheel). A schema 1
file (anti-roll in the units of Jolt's bar, whose effective rate depended on the damping and the step, so there is
no fixed conversion) loads with its anti-roll values dropped, so the vehicle's own bars apply; every other value is
kept. The garage logs each dropped value, shows a message, keeps the original as `<vehicle id>.json.schema1.bak`
and stores the migrated file once. Keyboard (arrows, Enter, Esc), mouse (hover, click arrows, click or drag a slider) and
gamepad (D-pad, A, B) work. Rename, Delete and Reset to factory open a modal dialog over the dimmed garage (one
reusable component): Rename has a text field with the current name preselected (typing replaces it) and a caret,
SDL text input runs only while it is open, names are checked live (not empty, not "Factory", unique, at most 32
characters of letters, digits, space and `- _ . ( ) +`) with a visible error line; Confirm / Cancel are clickable,
Enter confirms (on Delete / Reset the focus starts on Cancel), Esc or gamepad B cancels. While a modal is open it
owns the input: rows, W/A/S/D, hotkeys, F1–F3 and Escape-to-pause do not react behind it. With a gamepad only (no
keyboard) A keeps the current name. The mouse wheel / trackpad scrolls the parameter list (and every
menu list that does not fit, e.g. Settings); keyboard and gamepad moves keep the selected row in view.

**Spring and damper units.** Springs are tuned by natural frequency in Hz, f = (1/2π)·√(k/m): it says how the
body moves independently of the vehicle mass (~1–1.5 Hz soft off-road, 2–3 Hz stiff road / race). Next to each
spring the garage shows the equivalent wheel rate per corner, k = (2πf)²·m_corner, with m_corner = body mass × axle
weight fraction / 2 (SUV: 518 kg front, 482 kg rear; 1.35 Hz → 37 N/mm front, 35 N/mm rear). Damping values are
damping ratios (1 = critical; ~0.2–0.4 comfort, 0.5–0.7 sporty); next to them the equivalent damper coefficient
c = ratio × 2·√(k·m_corner) in kN·s/m. A help line under the list explains both. These are textbook estimates: Jolt
applies the frequency and the ratio to the effective mass of each wheel constraint, which includes the body's
rotational inertia. At rest the SUV front wheels carry 5.07 kN at 38 % of the 0.40 m travel, about 33 N/mm, roughly
10 % below the estimate. Values are entered in Hz only (no N/mm input).

## Driver aids

Simplified control logic on top of the tire model. Both aids act only through the per-wheel brake torque
(`WheelSettingsWV::mMaxBrakeTorque`, set before every physics step) and a throttle factor passed to Jolt's engine;
no forces or velocities are set. They read the previous tick's wheel telemetry; the ground speed of a wheel is the
true velocity of its contact point (a real ABS has to estimate it from the wheel speeds). They are not models of a
specific production unit.

- **ABS** (per wheel, brake pressed, wheel ground speed above 1.5 m/s): brake slip = (v − ωr) / v. Above 0.18 the pressure is dumped at 40 /s, below 0.08 it is re-applied, in between it is
  held. Re-apply has a pressure memory (`absReapplyFraction`, 0.85 on both vehicles, 0 in the C++ defaults): back at the
  dump rate up to 0.85 × the pressure at the last breakaway, then 8 /s (`absApplyRate`); before v0.24.0 it was 8 /s from
  0, which left the wheel under-braked for most of each cycle. Pressure goes down to 0, because a Jolt wheel at ω = 0 stays locked under any brake torque. Measured cycle
  9–15 Hz per wheel on straight stops (`trailforge_probe abs`, 120 Hz). At 60 Hz the controller also runs at 60 Hz
  and is coarser: a few short locks, and on dirt the straight stop is up to ~7 % longer than with locked wheels.
- **Traction control** (On / Off-road): drive slip = (ωr − v) / max(|v|, 2 m/s), low-pass filtered (50 ms, at least
  6 ticks). A wheel that spins more than the slowest wheel in contact by the threshold (0.12 On — the tire curves peak
  at 0.10 —, 0.35 Off-road) gets a brake torque (gain 3000 N·m per unit slip, max 1500 N·m) — that is how an open
  differential's torque gets back to the wheel with grip. The brake fades out as the slowest wheel's own slip nears the
  threshold: in a corner the unloaded inner rear spins first, and braking it sent the torque to the loaded outer tire
  that was already at its limit. The TC brake is released at once when the wheel stops spinning ahead of the ground
  (otherwise Jolt would hold it locked). An integrating controller cuts the engine throttle (down to 0.05 On, 0.10
  Off-road; cut 4 /s per unit error, at most 8 /s; recovery 0.6 /s) when even the slowest wheel spins above the
  threshold, or when the driven tires' load-weighted combined slip (√((slip / peak)² + (slip angle / peak)²), 1 = the
  peak of the friction ellipse) exceeds `tcGripThreshold` (0.85 On, 1.2 Off-road; the patrol 0.60 On, 0.80
  Off-road with a 0.05 throttle floor: at 1.2 its locked rear spun to 59° of rear slip angle on gravel): a rear tire
  at its cornering limit has no grip left for drive, so a 2H car turned at the limit stepped out under throttle before
  its wheels spun up. While the vehicle rolls back against the selected direction faster than 0.3 m/s the throttle
  recovers instead of being cut. The electric SUV's per-axle TC uses the same combined-slip signal per axle.
- **Differentials.** Jolt's differentials split torque; they have no kinematic link between the two sides. Open =
  fixed 50/50 split. LSD = Jolt's limited-slip ratio (max/min side speed, 1.4 by default; above it the torque shifts
  to the slower side). **Locked** is the same Jolt LSD with ratio 1.05: almost all torque goes to the slower side as
  soon as the speeds differ, which behaves like a locker for traction, but nothing forces the two sides to turn
  together (a wheel in the air still turns a little faster, and a locked axle still lets the wheels differ in a
  turn). The center differential uses the same mechanism between the axles.

Measured (headless, 120 Hz, Release):

- `trailforge_probe abs`, full brake on the flat surface lanes (friction ellipse on, ABS off → on). Distance =
  path length to the stop, lock = wheel-seconds with a wheel turning below 20 % of its ground speed, cycle = pressure
  releases per wheel per second of ABS activity:

  | Surface (peak μ) | 50 → 0 km/h | 100 → 0 km/h | Locked wheel-s at 50 / 100 | ABS cycle 50 / 100 |
  |---|---|---|---|---|
  | Concrete (0.95) | 14.08 → 11.99 m | 51.45 → 42.86 m | 5.90 → 0.14 / 12.63 → 0.14 | 16.0 / 11.8 Hz |
  | Rock (0.75) | 18.21 → 16.21 m | 67.30 → 58.53 m | 7.41 → 0.12 / 16.35 → 0.13 | 13.4 / 10.7 Hz |
  | Packed dirt (0.70) | 19.36 → 17.30 m | 71.97 → 61.85 m | 8.03 → 0.10 / 17.65 → 0.08 | 13.1 / 11.1 Hz |
  | Sand (0.55) | 24.40 → 21.47 m | 91.06 → 78.18 m | 10.60 → 0.02 / 22.95 → 0.00 | 14.0 / 10.9 Hz |
  | Grass (0.50) | 27.05 → 23.74 m | 99.88 → 86.13 m | 11.88 → 0.00 / 25.35 → 0.00 | 14.1 / 11.2 Hz |
  | Mud (0.35) | 39.59 → 34.38 m | 141.83 → 117.97 m | 17.67 → 0.00 / 36.68 → 0.00 | 14.6 / 11.7 Hz |

  The concrete row is from v0.24.0 (SUV, front-biased brakes, ABS re-apply memory); the other rows are older. On the
  current probe only the 50 km/h rock and dirt runs stay in the lane (ABS on 15.54 / 16.60 m); the sand, grass and
  mud runs and the 100 km/h runs off concrete leave the 5 m lane (dirt never reaches speed), the same before and after
  the v0.24.0 brake change.
  ABS stops are 3–17 % shorter because the locked tire is on the sliding part of the curve (0.8 × peak); the surfaces
  are rigid, so there is no gravel or snow wedge that makes a locked wheel stop shorter on real loose ground.
  Steering under full braking with 40 % steer, heading change after 1 s (50 km/h; ABS off → on): concrete
  −19° → −27°, rock −15° → −25°, dirt −13° → −24°, sand −11° → −21°, grass −10° → −19°, mud −6° → −14°. Without ABS
  the vehicle stays in its 12 m lane; with ABS it follows the steering out of the lane (where the run is cut) on
  every surface except concrete. At 100 km/h: concrete −23° → −29°, mud −8° → −15°.
- Friction ellipse (`trailforge_probe abs 0` / `ellipse`, ellipse off → on): straight stops are the same to
  ±0.2 m (e.g. dirt 50 km/h 19.37 / 19.36 m, mud 100 km/h 141.88 / 141.83 m); full-throttle launches are the same
  (concrete v(1 s) 5.13 / 5.13 m/s, max slip 0.27 / 0.27 m/s; dirt 2.28 / 2.29 m/s; mud 9.37 / 9.38 m/s; sand and grass
  equal to 0.01 m/s). Steady cornering at ~40 km/h (dirt and mud: the pad's friction set to the surface values, mud at
  ~26 km/h): 30 % steer unchanged (concrete 0.85 g, dirt 0.64 g, mud 0.31 g), 60 % steer −1…−3 %, full lock
  concrete 0.72 → 0.65 g, dirt 0.55 → 0.49 g, mud 0.27 → 0.24 g (front tires at ~30° slip angle, three times the
  peak, share their grip with the drive slip). Locked wheels with 40 % steer from 50 km/h, heading after 1 s:
  concrete −32° → −19°, dirt −22° → −13°, mud −13° → −6°; without the ellipse the locked vehicle turns as much as
  with ABS and leaves the lane on dirt, sand, grass and mud.
- `trailforge_probe tcmud`, 15° mud hill start (creep up the ramp, foot brake to a stop, full throttle, low range,
  15 s), nine runs per mode (60 / 120 / 240 Hz × creep to 7 / 8 / 9 m). TC off: 5 / 9 reach the plateau (14.0–15.0 s;
  the others stall 11.5–12.6 m up, spinning). TC On: 6 / 9 — every 120 Hz run (10.6–11.9 s) and 240 Hz run
  (8.8–9.2 s); at 60 Hz it slides back 3–7 m. Off-road: 6 / 9 (11.4–12.3 s at 120 / 240 Hz; at 60 Hz it stalls or
  slides back up to 3.4 m). At 60 Hz Jolt's wheel speeds alternate from tick to tick near the grip limit, and the
  TC reads that as wheelspin.
- `trailforge_probe tc` (120 Hz): 20° grass off 9.9 s, Off-road 8.7 s, but **On fails** (the low road threshold cuts
  the throttle too far and the vehicle slides back; use Off-road there). 15° dirt: 3.7 s off, 4.4 s On, 3.9 s
  Off-road. 20° mud is beyond the sliding limit (μ 0.35 < tan 20°) in every mode. Flat high-range launches: TC
  cuts the peak slip on mud 9.4 → 4.7 m/s and grass 7.3 → 3.2 m/s, but v(2 s) is lower (mud 5.2 → 4.5 m/s, grass
  7.4 → 6.3 m/s): with this tire curve (sliding μ = 0.8 × peak) the slip control costs more than it gains on the flat.
- `trailforge_probe diff`, `TrackLayout::SlopeWithWheelPit` (15° concrete slope, a bottomless pit under the
  front-left wheel, low range, full throttle, 8 s): all open — stays (the hanging wheel spins up to ~14 m/s);
  default (rear + center LSD) — out in 3.5 s; all open + X rear lock — stays (the open center sends the torque to
  the front axle with the hanging wheel); + rear + center — out in 4.5 s; all locked — out in 3.1 s; all open +
  TC On / Off-road — out in 2.7 / 2.7 s (the TC brake on the hanging wheel acts like a locker). The diagonal twister
  is climbed by every configuration (all open 6.2 s, all locked 5.0 s).
