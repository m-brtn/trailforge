# Physics simplifications

The honest list of what is simplified or not simulated. Keep it up to date when behavior changes. All documentation: [index](README.md).

## Physics simplifications (honest list)

- The suspension is virtual: in Jolt it is a raycast or cylinder cast along the spring axis, with no arms or axles.
  Solid axles and articulation are not modeled; every wheel is independent.
  The drawn arms and coil-overs (including the buggy's frame-mounted shock towers) only follow the Jolt hub; their
  geometry does not change camber, motion ratio or travel.
- The tire is two μ(slip) curves (longitudinal and lateral) plus Jolt's friction combining.
  No Pacejka model or sidewall deformation. No relaxation length: the slip angle acts on the next step.
- Load sensitivity (`tires.loadSensitivity`, default friction 0.12 and cornering stiffness 0.40): with r = load /
  reference load (default a quarter of the vehicle weight, r clamped to 0.1–4) the tire's μ is scaled by r^-0.12 and
  its lateral curve is stretched along the slip-angle axis by r^0.28, so the cornering stiffness grows as load^0.6
  and the peak force as load^0.88 (published passenger tire values: stiffness exponent ~0.5–0.8, μ falling ~10 % per
  doubled load). Weight transfer and anti-roll bars therefore move the balance, but through the force limit Jolt
  applies per step, not a tire model: measured on the SUV at 0.4 g, a +20000 N/m front bar raises the understeer
  gradient by 0.04 deg/g and a rear one lowers it by 0.05 deg/g; on the patrol the front bar lowers the limit from
  0.82 to 0.72 g (`trailforge_probe handling arb`). The direction is right; the size in the linear range is small.
- Compliance steer (`steering.compliance`, deg per kN of axle side force): stands in for steering-system, bushing and
  roll-steer compliance that is not modelled. The front wheels steer out of the turn and the rear wheels steer into it
  by the configured angle per kN of the axle's lateral force from the previous step (30 ms lag, faded in between 1
  and 3 m/s), added to Jolt's steer angle before the wheel casts. It applies no force. Values are hand-set (SUV
  0.15 / 0.12, EV 0.10 / 0.10, patrol 0.25 / 0.20 deg/kN), not measured; a published Ford Expedition breakdown has
  ~1.6 deg/g of compliance understeer. Understeer gradient at 60 km/h on asphalt (commanded road-wheel angle, steer
  ramp, `trailforge_probe handling ramp`): SUV 2.94, EV 2.63, patrol 1.69 deg/g (published: cars 2–3, SUVs 3–5).
- Grip levels (SUV on its all-terrain set; `trailforge_probe tires brake` and a 50 km/h steer ramp): 60-0 km/h with
  ABS 0.90 g on asphalt, 0.60–0.63 g on gravel and dirt, 0.47 g on grass; 0.91 g lateral on asphalt, 0.64 g on dirt,
  0.53 g on gravel and 0.46 g on grass. These match published ranges for these surfaces; most of Exploration Peaks is grass
  (54 % of the bake), sand (21 %) and snow (10 %), so most driving there is at μ 0.3–0.55 by design.
- Tire sets and temperature ("Tire sets and temperature"): a set only scales the off-road curves (peak slip, sliding
  μ, μ per surface); the per-surface and window values are hand-tuned within published ranges. Temperature is a
  2-node lumped model (tread surface layer, core) per wheel; it changes grip only through one factor on μ, at the
  surface temperature, one tick late. Not modelled: temperature across the tread width (camber, pressure), flash
  temperatures in the patch, wear, graining, blistering, rubber laid on the road, sun-heated or wet ground (the
  ground is at the air temperature), brake heat reaching the tire through the rim, cooling by water or mud. The
  pressure is computed (ideal gas) but changes neither stiffness nor the patch size used by the curves; the set's
  rolling resistance and the carcass deflection loss only heat the tire (the rolling resistance force applied to the
  vehicle is still the surface's). The heat partition, deflection loss and tread–carcass conductance are taken from
  published fits (F1 and FSAE tires, not these tires); the 2 mm layer, real contact area and convection (free stream,
  no wheel-arch model) are estimates. In autopilot laps the treads reach 40–60 C, below typical track-day readings,
  mainly because the autopilot is at the limit for only 10–40 % of a lap; cold slicks reach their window only at a
  sustained limit (EV on the 40 m circle) or off warmers.
- The ground is hard and non-deformable. Surfaces differ only in friction coefficients (table under "Proving
  ground"). The "mud" is a low-μ rigid surface, not a mud simulation; "sand" is a rigid μ 0.55 surface with no
  sinkage, flotation or bulldozing; puddles are dips under a water level (water forces below). `Gravel` (mini track traps, the level's main
  roads) is rigid too: only lower μ, a stretched slip curve with a flatter drop and a rolling resistance force — no
  sinkage, ruts or stones in the physics. Mini track asphalt and curbs are the rigid `Asphalt` surface (the curb
  colors are render-only).
- Suncoast Raceway: the curbs are rigid prisms (no rumble model beyond their shape), the gravel traps are the rigid
  `Gravel` surface (no sinkage or deceleration bed), the tyre walls and armco are rigid
  convex pieces (no energy absorption), and the track surface is a smooth fit to a 30 m DEM, not a survey of the
  real asphalt (real bumps and camber are unknown). The lap timer and the best laps only observe the body position.
- Water (rivers, lakes, sea, fords, the proving ground puddles) lies over rigid ground and pushes the vehicle with
  explicit forces on the chassis every tick ([Water forces](#water-forces)): buoyancy, pressure drag and skin friction
  on the wetted collision boxes, a bow lift on their front / rear faces, tire displacement drag. No splash forces,
  water ingestion or engine stall. Deep water (the body origin under the surface, or the vehicle afloat: buoyancy
  ≥ 80 % of its weight, for 1 s) and leaving the play area are handled by an honest teleport to the nearest road with a
  message (the only teleports besides R (on Suncoast Raceway during a lap: to the last gate passed), the F1 teleport buttons, the attract-drive reset behind the start menu and
  scenario `teleport` / `place`). Snow is rigid
  low-μ ground. The horizon ring and the open sea outside the 2 × 2 km bake are render-only scenery (no collision;
  the play limit is inside the bake). Waterfalls (sheet, foam, mist) are render-only meshes over rigid rock steps;
  the river surface is a level per sample (smoothed along the river, thinning out toward the edge of its reach where
  the banks lie lower), not a flow simulation. At fords the road's fill shoulders are kept 0.1 m under the river
  surface inside the channel; that lowered ground is the collision terrain too. Waves, shore foam, refraction and
  reflections are shading only: the water surface the physics sees stays flat, and the waves do not follow wind,
  fetch or the river current. Clouds, their ground shadows and the valley fog are render-only too (cloud shadows dim
  only the rendered sunlight).
- Drift scoring only observes: the slip angle is the body origin's velocity against the body heading (no per-tire
  slip), a "wall hit" is any chassis contact approaching at ≥ `wallSpeed` (Jolt's listener has no solved impulse,
  so it cannot tell a scrape from a crash beyond the approach speed), and the score adds nothing to the physics.
- The terrain textures are render-only: their height and normal maps shade the ground and pick which layer shows
  where layers meet; they never change the collision (the heightfield samples) or the surface friction (the baked
  per-sample material).
- Grass, flowers and the wheat patch are render-only: no rolling resistance, drag or grip change; the ground under
  the wheat is the `Grass` surface. Blades only bend visually under the wheel contacts and the tire marks.
- Buildings collide with convex pieces made from the same solid parts as their far LOD; window / door recesses
  (≤ 0.16 m) and trims (≤ 0.30 m: frames, sills, shutters, flower boxes, balusters) exist only in the near render
  LODs. Power line wires, mast antennas above 6 m and thin ladder rungs and rails are render-only. Buildings have no
  interiors and do not break.
- Level models: rocks, cliffs, logs and stumps collide with their model's collision mesh (a simplified LOD; the
  nearer render LODs differ from it by the simplification error modelc reports, a few cm to dm per model). Model trees
  collide only with a hexagonal prism along the trunk axis; branches and foliage have no collision. Bushes, ferns and
  dry branches are render-only (the vehicle drives through them). Wind sway moves only the render geometry.
- Brakes are a torque per wheel (`brakeTorque` × pedal × ABS pressure): no hydraulics, pad friction, fade or
  temperature, no load-sensing proportioning valve (the front / rear split is the fixed torque ratio). Pedal-to-pressure
  delay is the 8 1/s foot-brake rise. The concrete peak μ is 0.95 (EV all-terrain longitudinal 1.10 × 0.95), so a
  100–0 km/h ABS stop is ~40–43 m, at the long end of real large SUVs (~36–40 m on road tires).
- Static tire grip: Jolt caps the longitudinal tire force at μ(slip of the previous step) × load, and the curve is 0
  at zero slip. A tire below the peak slip ratio may therefore pass up to the peak friction (`tireStaticGrip`);
  without it a gripping wheel alternated between grip and free spin every step (wheelspin on every launch). The tire
  is stick/slip: below breakaway the measured slip is ~0 rather than a slip-stiffness curve. At 60 Hz the lighter
  front wheels still slip for one or two steps (~3 m/s) during a full-throttle concrete launch.
- Launch clutch: near the minimum rpm the clutch passes at most the engine torque at the current throttle (at least
  a 40 N·m creep torque), like a torque converter at stall. Jolt clamps the engine rpm, so an unlimited slipping
  clutch delivered ~840 N·m at any throttle.
- Shift clutch: after a gear change, until the engine runs at the clutch speed, the clutch passes at most the engine
  torque at the current (TC-limited) throttle when the engine is faster and the creep torque when it is slower, and
  while the engine is faster its own torque is cut (an automatic's upshift torque reduction). Jolt's engine revs
  freely while the clutch is open and the closing clutch dumped the flywheel's overspeed into the wheels in ~0.1 s:
  2350 N per rear tire after the SUV's 1-2 upshift in a grass corner (rear stepped out with TC on), a locked outer
  rear after a coasting 2-1 downshift. The flywheel's excess energy is lost in the clutch; there is no shift
  schedule, no torque converter and no clutch heat. Jolt still opens the clutch fully for `shiftTime` (no drive at
  all): the SUV's automatic uses 0.10 s (was 0.35 s, whose lost drive the flywheel kick had partly made up; without
  it the SUV slowed on the diagonal twister in low range and was turned off line), a clutch-to-clutch automatic
  hands over with almost no interruption.
- Once a tire spins up it is on the sliding part of the curve (μ × 0.80): full throttle from a standstill on a slope
  near the limit spins in place where a careful throttle would climb. Traction control (off by default) is
  simplified control logic, see [Driver aids](vehicles.md#driver-aids).
- Slip speed floor: Jolt's slip ratio is |ωr − v| / max(|v|, 0.001), so at a standstill any wheel rotation reads as
  full slip and the tire falls to the sliding μ before it moves. The friction curve is evaluated with
  |ωr − v| / max(|v|, 1 m/s) instead (`tireSlipSpeedFloor`, 0 = Jolt's ratio). With it a full-throttle 15° mud hill
  start without TC gets 11.5–12.6 m up the 16 m to the plateau or reaches it in 14–15 s (`trailforge_probe tcmud`)
  where it used to stay at the foot.
- Friction ellipse (combined slip): Jolt's longitudinal and lateral limits are independent (a friction rectangle), so a
  locked wheel kept steering and a driven rear tire carried full drive and full side force at once. With
  `tireFrictionEllipse` (on by default, Tuning → Tires) the max tire impulses from
  `WheeledVehicleController::SetTireMaxImpulseCallback` are reshaped with the normalized slips sx = slip ratio / peak
  and sy = slip angle / peak (s = √(sx² + sy²)): below the peak (s ≤ 1) the longitudinal limit is scaled by
  √(1 − sy²) (Jolt's lateral constraint is solved first, so the side force keeps priority); beyond it both limits
  follow the curves at the combined slip s, split along (sx, sy) / s. The two meet at s = 1 because the lateral curve
  is linear up to its peak. Blended in between 1 and 3 m/s of contact-patch speed. An approximation on top of two
  μ(slip) curves, not a combined-slip tire model.
- Tire marks, ruts and flattened or torn grass are visual only: they never change friction, ground height or
  collision, and their relief exists only in the shading normal. The trample mask has 0.1 m texels and refreshes
  every 0.5 s while marks fade. The "Fill to cap" ribbons are synthetic (not driven). Soil in the tread and its
  prints on hard ground are cosmetic too: the tread soil never changes grip. Body mud from the spray is cosmetic as
  well: no mass, no drag, no effect on the simulation; it is placed by fixed regions around the wheel arches, not by
  particles hitting the body.
- Wheel spray is cosmetic CPU particles, not Jolt bodies: they collide only with the ground height function, not with
  the vehicle, rocks or each other, remove no material from the ground and never push the vehicle.
- No rigid differential lock: Jolt supports only open and LSD torque splits. "Locked" is the LSD with ratio 1.05
  (torque to the slower side), not a shaft linking the two wheels. The LSD limits the wheel speed ratio; it is not a
  preloaded clutch pack.
- ABS and TC read the true contact-point ground speed instead of estimating it from wheel speeds, and act through
  brake torque and a throttle factor only. ABS at 60 Hz is coarser (see [Driver aids](vehicles.md#driver-aids)).
- Aerodynamics: drag only, F = ½ ρ Cd·A v² against the body velocity at the centre of mass (`dragArea`, SUV 1.3 m²),
  no lift, downforce, side force or wind. Rolling resistance only on gravel (0.015 × load at each contact patch);
  none on paved or other ground (linear damping = 0).
  The engine is a torque curve, with no flywheel inertia beyond Jolt's model.
- Suspension forces: Jolt's wheel constraint takes one stiffness and one damping coefficient per wheel. Each step,
  after collision detection, they are set from that step's contact: the stiffness is the secant rate of spring +
  bump stop + anti-roll bar + tire at the current length, the damping coefficient comes from the digressive damper
  curve (compression or rebound ratio by the direction of the suspension speed, × the slope above `kneeSpeed`).
  Within one step the force is linear in the compression.
- Bump stops: an additive progressive force (zero at `bumpStop.range` before full compression, tangent rate
  `frequencyScale` × the spring rate at full compression), scaled by 1 − `hysteresis` while the suspension extends
  to stand for the rubber's energy loss (the 0.5 default is an assumption, not a measured elastomer).
- Tire compliance: the tire acts as a vertical spring (`wheel.vertical`) only once the suspension is solid; then the
  wheel can push `maxDeflection` further up to the rim, where Jolt's hard stop is. The tire does not deflect in series
  with the spring otherwise, and the contact patch has no relaxation length. The damper has no travel left in that
  phase, and Jolt's wheel is massless, so the losses of a real bottoming (unsprung-mass impact, carcass damping, bump
  stop hysteresis) are one damping coefficient, `wheel.vertical.damping` (fraction of critical for the tire rate and
  the wheel's suspension mass, default 1.0). It was 5 % of critical (ee67b63 to v0.30): ~85 % of the bottoming energy
  went back into the body, a 1.5 m drop rebounded 15 % of its height and the SUV / EV / patrol flipped over 5 / 8 / 9
  of 10 cosine hills (0.5 × 8 to 1.5 × 20 m at 80 / 100 km/h, `trailforge_probe hill all`); with 1.0 and the 0.75
  damper slope: rebound 4 %, 0 / 1 / 1 of 10. The value is chosen to remove the rebound, not measured on a tire.
- Anti-roll bars are applied as suspension forces (N/m at the wheel on the left-right hub height difference, an
  airborne wheel counting at full droop), not with Jolt's `VehicleAntiRollBar`, whose impulse enters the
  constraint as a velocity bias, so its effective rate depends on the damping and the step and it added load noise.
- No drivetrain or brake reaction torque on the body: Jolt's controller turns the wheels but does not push back on
  the chassis, so throttle and braking do not pitch the body through the axles (only through the tire forces at the
  contact patches). For a flying vehicle this misses −I_wheel·Δω about the axle (estimated ~13°/s of pitch rate
  for the patrol buggy flooring it in the air).
- Bumps (`trailforge_probe bump`, `tools/scenarios/bump_check.txt`): no shipped vehicle rolls over a 0.35 m mound
  at 30 km/h with a constant or no throttle; a 1 m long kicker of the same height or a step acts as a ramp, and
  holding or flooring the throttle over it can still flip the light patrol buggy (see the devlog for the per-vehicle counts).
- Ackermann: the outer wheel's maximum angle is lowered before each step (cot(outer) = cot(inner) + track /
  wheelbase, blended by `ackermann`); Jolt steers each wheel to input × its maximum angle, so the geometry holds
  exactly only at full lock and is proportional in between.
- Rev limiter: a throttle cut above `revLimitRPM` with 150 rpm hysteresis, applied only when it is below `maxRPM`
  (Jolt clamps the engine at `maxRPM` anyway).
- Drivetrain layouts: "locked" (4H, 4L, locked center) is still the LSD with ratio 1.05, so there is no driveline
  wind-up or tire scrub in tight turns on high-grip ground as with a real locked part-time system. On-demand AWD is a
  center LSD with a rear-biased static split: it reacts to wheel speed differences at once, with no clutch pack
  engagement delay or heat. Undriven wheels roll freely with their own inertia; there is no hub-lock model.
- Electric drive: the motor is a torque / power envelope with one constant drive efficiency and one regen
  efficiency (no efficiency map, no temperature, no current or voltage, no inverter limits); the battery is an energy
  counter (no internal resistance, voltage sag, temperature or degradation). The motor is not a body of its own: its
  rotor inertia is added to the wheel inertia through the reduction, so the two wheels of an axle are not coupled
  through a shared shaft speed; the open differential is the torque split only. Traction control and regen blending
  are control logic that acts through the motor torque and the friction brake torque.
- Body collision of the shipped (kit) bodies is a compound of three boxes (tub, fender band, cab) with a 5 cm
  convex radius around the visible body: the arches, chamfers, bumpers and the style-kit accessories (roof load,
  winch, spare, jerry cans, mirrors) have no collision of their own. A traced body (`bodyShape` `"traced"`)
  collides as convex hulls from the blueprint (5 cm cross-sections, see [Vehicle blueprints](vehicle-format.md#vehicle-blueprints-bodies-traced-from-the-concept-sheet)), so its concave
  details are filled in. The wheels touch the ground only through Jolt's
  wheel casts.
- A Blender body (body.glb) collides only with its authored `COL_*` convex pieces (hulls with a 0.05 m convex
  radius), never with the render mesh; how closely they follow the visual body is checked only by bounds (0.12 m per
  face) and coverage (90 % of the vertices within 0.10 m). Mass, inertia and centre of mass stay those of
  vehicle.json, whatever the body shape. The validated wheel clearance holds for the definition's tires; tire sizes
  set outside it (scenario `wheels`) are not checked against a glb body (the procedural bodies refit their arches).
- Time of day, moonlight and headlights are lighting only: they do not change grip, temperature or anything else in
  the simulation.
- Determinism is guaranteed only at the same step rate on the same platform.
  Trajectories differ at 60, 120 and 240 Hz, as the measurements below show.
- No damage.
- Sound is cosmetic: it never feeds back into the simulation. Mud slosh, puddle splashes and grass rustle are
  triggered by contact data on rigid ground, not by any fluid or vegetation simulation. Body impact strength is an
  estimate: Jolt's contact listener runs before the solver and reports no impulse, so the sound uses
  `J = v_approach / (1/m + (r×n)·I⁻¹(r×n))` (N·s) from the approach speed at the contact point. The body rattle
  and the bump-stop knock are synthesized from the rigid body's felt vertical acceleration and the bump-stop hits;
  there are no simulated panels or loose parts, and Jolt's wheels have no unsprung mass, so there is no top-out
  (full rebound) event to sound. The engine sound is
  a recorded petrol inline-4 pitched down, not a synthesized model of the simulated engine. The electric drive
  sound is synthesized from the motor speed and torque (no recordings); its tone choices (pole pairs, slot count,
  gear teeth, carrier) are plausible values, not measurements of a real motor.

## Water forces

`sim/WaterForces` (no SDL) computes explicit forces on the chassis from the water surface height (`WaterSurface`:
the level's smoothed river, lake and sea levels, the proving ground puddle level) once per physics tick, from the
state after the previous step; Simulation adds the sum as one force and torque about the centre of mass before the
step. Coefficients per vehicle are vehicle.json `water` ([format](vehicle-format.md)).

What is modelled:

- **Buoyancy:** the collision boxes (a convex piece of a traced or Blender body counts as its bounding box) are split
  into ~0.2 m cells; each cell's submerged fraction, from the depth at its centre and its vertical extent in the
  current pose, displaces ρ g V × `displacement` (default 0.7: doors, floor plugs and the engine bay are not sealed;
  an estimate, no flooding over time). The tires are cylinders of the physics wheel size and displace their submerged
  segment fully. Standing in still water the SUV carries 11 % of its weight at 0.3 m, 22 % at 0.5 m, 58 % at 0.7 m,
  93 % at 0.9 m, and floats (no wheel on the ground) from ~1.0 m; the patrol buggy (`displacement` 0.35, an open
  tube frame) floats from ~1.1 m.
- **Pressure drag:** the outer surface of the boxes (faces inside another box removed) is split into ~0.2 m patches;
  a wetted patch moving into the water gets ½ ρ Cd v_n² × its wetted area against its normal (v_n: the velocity
  relative to the water along the normal; `frontalCd` 1.0 on the front / rear faces, `sideCd` 1.1, `heaveCd` 1.2 on
  the bottom / top: bluff-body values), plus skin friction ½ ρ Cf |v_t| v_t along it (`skinCf` 0.005).
- **Bow lift:** the boxes have vertical ends, a real bumper chin and the underbody behind it slope up; the pressure
  on such a slope is normal to it, so a front or rear face pushing into the water also gets tan(`frontSlopeDeg`,
  default 35°) × its pressure force upward in body axes. It grows with v² and with the wetted front area.
- **Tires:** displacement drag ½ ρ `wheelCd` v² × width × submerged height along the rolling direction (the
  contaminated-runway model for aircraft tires, ESDU 83042: Cd ~0.75) and `sideCd` on the submerged disc segment
  sideways, at the centre of the submerged part of the wheel. Jolt's wheels are casts, so the force acts on the
  chassis there; like a braking force below the centre of mass it pitches the nose down.
- **Yaw and roll** only come from where these forces act: entering at an angle the side that is in the water first
  is held back, one side deeper drags more, a current (if given) pushes downstream. No yaw torque is added.
- **Current:** `SimulationSettings::waterFlow` (horizontal water velocity per x, z) is subtracted from the velocity
  of every sample. No level provides one yet, so all water is still.

Not modelled: waves (the surface the forces see is the static level per sample), the bow wave's run-up and spray (so
the nose lifts only once the static water reaches the body: SUV / EV ~0.55 m; in shallower water only the tires drag
and the nose dips as in braking), added mass, flooding of the cabin, water in the intake or engine stall, wet brakes,
cooling of the tires, hydroplaning (the tire grip stays that of the ground at any depth and speed), and the wake of
the front tires (the rear tires see the full depth; real rear tires run in a partly cleared track, so the tire drag
is probably high at speed). The patrol buggy's cage and bed boxes count as closed faces though they are open tube
work, so its drag and bow lift once the water is above its floor are overstated (0.7 m at 40 km/h: 3.1 g peak, nose
+21° then −9°; 0.5 m at 40 km/h: 1.9 g, +3.2 / −1.9°). The water front of the probe and the test is vertical (a test setup; real fords have
sloping banks, and real water would flow out).

Measured (`trailforge_probe water`, SUV 2000 kg, coasting with throttle and brake released into still water over
the flat concrete pad; peak = the largest 50 ms average deceleration from the first wet tick, pitch relative to the
pose before the entry, + nose up; 2026-11-02, Release):

| depth | km/h | peak g | mean 1 s g | km/h after 1 s | pitch up / down deg | 20° entry: heading change after 2 s |
|---|---|---|---|---|---|---|
| dry | 60 | 0.05 | 0.05 | 62 | — | — |
| 0.3 m | 20 / 40 / 60 | 0.15 / 0.68 / 1.67 | 0.11 / 0.44 / 0.94 | 15 / 25 / 31 | 60 km/h: +0.0 / −3.2 | −0.2 / −0.8 / −2.5° |
| 0.5 m | 20 / 40 / 60 | 0.20 / 0.95 / 2.46 | 0.13 / 0.58 / 1.18 | 15 / 21 / 22 | 40 km/h: +0.2 / −0.9 | −0.3 / −1.2 / −4.8° |
| 0.7 m | 20 / 40 / 60 | 0.32 / 1.37 / 3.06 | 0.20 / 0.73 / 1.34 | 12 / 15 / 17 | 40 km/h: +3.8 / −0.3; 60: +5.5 / −4.3 | −0.5 / −2.1 / −9.4° |

At 0.5 m and 40 km/h the tires take 22 kN and the body 2 kN; at 0.7 m and 60 km/h the tires 54 kN, the body 44 kN and
the bow lift 31 kN. A 20° entry turns the vehicle toward the water front's normal (the side that enters first is held
back) at up to 6 / 12 / 14 °/s yaw rate (0.3 / 0.5 / 0.7 m at 60 km/h). The step cost of the water forces is
~0.01–0.03 ms with the vehicle in water (noisy: measured with other agents' builds running); dry, the early-out
probes ~30 points. Real-world comparison: driving into ~0.5 m of water at 50 km/h is commonly described as a hard
stop "like a wall" (no measured deceleration was found for this); 50 km/h lies between the 40 and 60 km/h rows
above (0.95–2.46 g peak, 0.58–1.18 g over the first second), falling with v² as the vehicle slows.
