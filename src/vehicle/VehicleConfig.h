#pragma once

#include "track/SurfaceProperties.h"
#include "vehicle/BodySound.h"
#include "vehicle/TireModel.h"
#include "vehicle/TireSound.h"
#include "vehicle/VehicleWater.h"

#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

// Vehicle configuration. The game reads it from assets/vehicles/<id>/vehicle.json (VehicleDefinition.h); the C++
// defaults below are the v0.7 off-roader with full-time four-wheel drive, which the headless tests use.
// Units are given for each field. Body local axes: +Z forward, +Y up, +X left. Axle index 0 = front, 1 = rear.

namespace tf {

inline constexpr int kFrontAxle = 0;
inline constexpr int kRearAxle = 1;

// Locked is Jolt's limited-slip model with a ratio close to 1 (lockedSlipRatio): a torque split that sends all drive
// torque to the slower side, not a rigid shaft (see Vehicle::applyDifferentials).
enum class DiffMode { Open, LimitedSlip, Locked };

// Traction control: Off, On (road threshold, cuts the throttle), OffRoad (higher threshold, keeps more throttle and
// mostly brakes the spinning wheel).
enum class TractionMode { Off, On, OffRoad };

// Which axles the engine drives (see docs/vehicles.md "Drivetrain layouts").
//  Rwd / Fwd: one driven axle. Awd: full-time, both axles through the center differential (frontTorqueShare, center
//  diff mode). PartTime: transfer case 2H (rear only) / 4H (center locked) / 4L (center locked + low range).
//  OnDemand: both axles through a center LSD with a rear-biased base split (a simplification of a clutch coupling).
//  DualMotor: electric, one motor per axle with a fixed reduction and no center differential; the split between the
//  axles is electronic (ElectricDrive.h), the axle differentials are the same Jolt models as above.
enum class DriveLayout { Rwd, Fwd, Awd, PartTime, OnDemand, DualMotor };

// Transfer case position of a part-time layout (runtime state, like the low range of the other layouts).
enum class TransferMode { TwoHigh, FourHigh, FourLow };

// Power unit type. Combustion: Jolt's engine, clutch and automatic gearbox. Electric: two motors (DualMotor layout).
enum class EngineType { Combustion, Electric };

// Drive mode of a dual-motor vehicle (runtime state, the V key): the front share of the drive torque at part
// throttle. Normal uses VehicleTuning::frontTorqueShare, OffRoad ElectricConfig::offRoadFrontShare.
enum class EvDriveMode { Normal, OffRoad };

// One traction motor geared to an axle differential through a fixed single-speed reduction (no gearbox, no clutch).
// Torque / speed envelope: constant torque up to the base speed (maxPower / maxTorque), then constant power, zero at
// maxRPM. Regeneration uses the same envelope with regenTorque as the torque limit.
struct MotorConfig {
    float maxTorque = 400.0f;     // N*m at the motor shaft
    float maxPower = 180000.0f;   // W at the motor shaft
    float maxRPM = 15000.0f;      // rpm, drive torque fades to 0 over the last 2 %
    float reduction = 12.0f;      // motor revolutions per axle (differential carrier) revolution
    float rotorInertia = 0.05f;   // kg*m^2, rotor + reduction gears at the motor shaft
    float regenTorque = 300.0f;   // N*m, maximum braking torque at the motor shaft

    bool operator==(const MotorConfig&) const = default;
};

struct ElectricConfig {
    std::array<MotorConfig, 2> motors{}; // front, rear
    float batteryCapacityKWh = 120.0f;   // usable energy
    float initialSoC = 0.9f;             // 0..1 at spawn / R reset
    float driveEfficiency = 0.90f;       // battery -> shaft (inverter + motor), constant
    float regenEfficiency = 0.85f;       // shaft -> battery, constant
    float auxPowerKW = 0.6f;             // constant auxiliary load (cooling, electronics)
    float regenMaxPowerKW = 150.0f;      // battery charge power limit of both motors together
    float liftOffDecel = 1.0f;           // m/s^2, regen target with both pedals released ("coast regen")
    float regenFadeSpeed = 2.0f;         // m/s, regen fades out linearly below this speed (friction takes over)
    float offRoadFrontShare = 0.5f;      // front drive torque share in the OffRoad drive mode

    bool operator==(const ElectricConfig&) const = default;
};

// Driver aids: simplified control logic that only acts on the throttle pedal and on per-wheel brake torques
// (Jolt WheelSettingsWV::mMaxBrakeTorque, set before every step). Not a model of a specific ABS/TC unit.
struct DrivingAidTuning {
    bool absEnabled = false;                 // default state of the ABS switch (B)
    TractionMode traction = TractionMode::Off; // default state of the TC switch (T)

    // ABS: per-wheel brake pressure p (0..1) multiplies the pedal torque. Braking slip = (v - w*r) / v at the wheel.
    float absReleaseSlip = 0.18f; // slip above which pressure is released
    float absApplySlip = 0.08f;   // slip below which pressure builds up again (between the two: hold)
    float absReleaseRate = 40.0f; // 1/s, pressure drop rate (fast dump valve)
    float absApplyRate = 8.0f;    // 1/s, pressure build-up rate
    // Pressure memory (the stepped re-apply of real ABS units): after a release the pressure comes back at the
    // release rate up to this fraction of the pressure at which the wheel last broke away, then at absApplyRate.
    // 0 = no memory (slow build-up from the dump level).
    float absReapplyFraction = 0.85f;
    float absMinPressure = 0.0f;  // pressure floor while releasing (Jolt keeps a stopped wheel locked under any brake torque)
    float absMinSpeed = 1.5f;     // m/s, below this the pedal acts directly (a real ABS also drops out near a stop)

    // Traction control. Drive slip = (w*r - v) / max(v, tcMinSpeed), also for a wheel in the air.
    float tcSlipThreshold = 0.12f;        // On (the tire curves peak at a slip ratio of 0.10)
    float tcOffRoadSlipThreshold = 0.35f; // OffRoad
    // Throttle cut also on the driven tires' combined slip (load-weighted DrivingAids AidWheelInput::gripUse): a tire
    // at its cornering limit has no grip left for drive force, so the rear axle of a car turned at the limit steps
    // out under throttle long before its wheels spin up. 1 = the peak of the friction ellipse. 0 = off.
    float tcGripThreshold = 0.85f;        // On
    float tcOffRoadGripThreshold = 1.2f;  // OffRoad
    float tcMinSpeed = 2.0f;              // m/s, slip denominator floor (slip is undefined at a standstill)
    float tcSlipFilterTime = 0.05f;       // s, low-pass time constant of the measured slip
    float tcBrakeGain = 3000.0f;          // N*m of brake per unit of slip above the threshold
    float tcMaxBrakeTorque = 1500.0f;     // N*m per wheel
    float tcBrakeRate = 8000.0f;          // N*m/s, rate limit of the TC brake torque (both directions)
    float tcCutRate = 4.0f;               // 1/s, throttle factor drop rate at an error of one threshold
    float tcRecoverRate = 0.6f;           // 1/s, throttle factor recovery rate at zero slip
    float tcMinThrottle = 0.05f;          // throttle factor floor, On
    float tcOffRoadMinThrottle = 0.10f;   // throttle factor floor, OffRoad
    float tcSpinSpeed = 0.3f;             // m/s, rim speed below which a wheel counts as not spinning (TC brake off)
    float tcRollbackSpeed = 0.3f;         // m/s against the selected direction: no throttle cut while rolling back

    bool operator==(const DrivingAidTuning&) const = default;
};

// Per-axle values that can change on the running vehicle.
struct AxleTuning {
    // Suspension (Jolt virtual suspension: spring-damper along the wheel suspension axis)
    float springFrequency = 1.35f; // Hz, spring natural frequency
    // Dimensionless damping ratio (1 = critical) at low suspension speed. Jolt has one damping coefficient per wheel;
    // in every step it is set from the suspension speed of that step (Vehicle::updateSuspension). Road cars run
    // about 0.2-0.4 of critical (Gillespie, Fundamentals of Vehicle Dynamics, 1992, ch. 5), with more damping in
    // rebound than in compression (Dixon, The Shock Absorber Handbook, 2nd ed., 2007).
    float compressionDamping = 0.35f;
    float reboundDamping = 0.55f;
    // Digressive damper curve (blow-off valving, Dixon): above damperKneeSpeed (m/s of suspension speed at the wheel)
    // the force grows with the high-speed slope x the low-speed slope. 1 = linear damper at every speed. 0.75 (was 0.5):
    // at 0.5 a hill crest landing at 2-4 m/s had half the damping, and the SUV / EV / patrol flipped over 0 / 1 / 3 of
    // 10 cosine hills at 80-100 km/h (`trailforge_probe hill all`); at 0.75 0 / 1 / 1, drop rebound 6 -> 4 %.
    float damperKneeSpeed = 0.25f;
    float compressionHighSpeedSlope = 0.75f;
    float reboundHighSpeedSlope = 0.75f;
    // Anti-roll bar rate at the wheel: N of extra load on the more compressed wheel (and less on the other) per m of
    // hub height difference across the axle; a wheel in the air counts at full droop (0 = none; maximum
    // articulation for an off-roader).
    float antiRoll = 0.0f;
    // Bump stop (jounce bumper): an elastic element over the last bumpStopRange m before full compression, its force
    // added to the spring. Its rate rises linearly from 0 at contact, so the wheel rate at full compression is
    // bumpStopFrequencyScale^2 x the spring rate (the local natural frequency x bumpStopFrequencyScale). Range 0 =
    // only Jolt's rigid hard point at the minimum suspension length. bumpStopHysteresis is the fraction of the
    // force lost while the bumper extends (elastomer damping: its unloading curve lies below the loading curve; 0.5
    // is an assumption, not a measured value). Road-car polyurethane jounce bumpers are a few cm to some 8 cm long
    // and strongly progressive (Reimpell, The Automotive Chassis, 2nd ed., 2001).
    float bumpStopRange = 0.06f;
    float bumpStopFrequencyScale = 4.0f;
    float bumpStopHysteresis = 0.5f;
    // Compliance steer, deg of wheel steer per kN of side force on this axle (steering rack and arm bushings, the
    // tires' aligning torque through the steering): the front wheels turn away from the corner, the rear wheels with
    // it, both toward understeer. Measured on a 2003 Ford Expedition: steering compliance 0.34 + aligning torque 0.28
    // + lateral-force compliance steer 1.02 = 1.6 deg/g (Ohio State University MSc thesis on its handling model,
    // 2003). 0 = rigid.
    float complianceSteer = 0.0f;
    // Brakes, N*m per wheel of this axle
    float brakeTorque = 3000.0f;
    float handBrakeTorque = 0.0f;

    bool operator==(const AxleTuning&) const = default;
};

// Parameters applied to the running vehicle between physics steps (without recreation).
struct VehicleTuning {
    std::array<AxleTuning, 2> axles{AxleTuning{}, AxleTuning{.handBrakeTorque = 3500.0f}};

    // Tires: longitudinal grip curve mu(slip ratio) and lateral mu(slip angle) of an all-terrain tire on dry
    // concrete; the surface multipliers scale the whole curve (SurfaceTable). This is the off-road tire set; the
    // other sets (VehicleConfig::tireSet) are applied on top of it by the vehicle (TireModel.h).
    float tirePeakLongFriction = 1.00f; // mu at the peak
    float tireSlideLongFriction = 0.80f; // mu at full slide (slip ratio >= 1)
    float tirePeakSlipRatio = 0.10f;    // dimensionless
    float tirePeakLatFriction = 1.00f;
    float tireSlideLatFriction = 0.78f; // at 30 degrees and above
    float tirePeakSlipAngleDeg = 6.0f;  // degrees
    // Per-surface multiplier of this tire on top of the surface table (e.g. a mud tread > 1 on mud). 1 = surface
    // table only.
    std::array<float, kSurfaceCount> tireSurfaceGrip{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    // A tire below the peak slip ratio may transmit up to the peak longitudinal friction before it breaks away,
    // instead of mu(slip of the previous step), which is 0 for a wheel that just gripped. false = plain Jolt curve.
    bool tireStaticGrip = true;
    // Floor of the slip ratio denominator, m/s: slip = |w*r - v| / max(|v|, floor). 0 = Jolt's 0.001 m/s, with which
    // any wheel rotation near a standstill counts as a full slide.
    float tireSlipSpeedFloor = 1.0f;
    // Tire friction ellipse (combined slip, WheeledVehicleController::SetTireMaxImpulseCallback): beyond the peak,
    // the longitudinal and lateral impulse limits share one budget in proportion to the normalized slips (slip ratio /
    // peak, slip angle / peak). A locked or spinning wheel then has little side grip. false = Jolt's independent
    // limits (a locked wheel keeps full side grip and steers).
    bool tireFrictionEllipse = true;
    // Load sensitivity: at r = load / reference load the peak friction is mu x r^-tireLoadFriction and the cornering
    // stiffness (initial slope of the side force) grows as load^(1 - tireLoadCornering), so the peak slip angle moves
    // to angle x r^(tireLoadCornering - tireLoadFriction). The reference is the tire's nominal load; 0 = the vehicle's
    // mean static wheel load (m g / 4). Both exponents 0 = no load sensitivity (force proportional to load).
    float tireLoadFriction = 0.12f;
    float tireLoadCornering = 0.40f;
    float tireReferenceLoad = 0.0f; // N

    // Steering rate toward the target (keyboard) and return-to-center rate, 1/s of full lock. The analog stick uses
    // twice these rates.
    float steerRate = 2.2f;
    float steerReturnRate = 3.5f;
    // Ackermann: 0 = both front wheels at the same angle (Jolt's default), 1 = full Ackermann geometry (the outer
    // wheel turns less, both wheels point at a common turning center on the rear axle line).
    float ackermann = 0.0f;

    // Engine and drivetrain
    float engineMaxTorque = 520.0f; // N*m
    // Rev limiter: the throttle is cut at this rpm and restored 150 rpm below. At or above engineMaxRPM only Jolt's
    // own rpm clamp acts (no cut).
    float revLimitRPM = 4800.0f;
    float finalDrive = 3.90f;       // final drive ratio
    float clutchStrength = 10.0f;   // kg*m^2/s (Jolt: Torque = S * dw)
    // Near the minimum rpm the clutch passes at most the engine torque at the current throttle (at least the
    // creep torque), like a torque converter at stall. Jolt clamps the engine at its minimum rpm, so an unlimited
    // slipping clutch would deliver S * dw (~840 N*m at launch) whatever the throttle.
    bool launchClutchLimit = true;
    float creepTorque = 40.0f;      // N*m at the engine shaft, idle creep in gear
    // Differentials (Jolt limited-slip model). These are the modes with the driver's lockers off; the locker switch
    // (X) overrides rear, rear + center, or all three with Locked.
    DiffMode frontDiff = DiffMode::Open;
    DiffMode rearDiff = DiffMode::LimitedSlip;
    DiffMode centerDiff = DiffMode::LimitedSlip;
    float limitedSlipRatio = 1.4f;       // front/rear LSD: max/min wheel speed ratio at which all torque goes to the slower wheel
    float centerLimitedSlipRatio = 1.4f; // center LSD: the same between the axles (average axle speeds)
    float lockedSlipRatio = 1.05f;       // "Locked": the same model with a ratio close to 1 (Jolt requires > 1)
    // Engine torque share of the front axle with the center differential open (Awd, OnDemand; 4H/4L of a
    // part-time layout use 0.5). Jolt's center LSD moves the split toward the slower axle from there.
    float frontTorqueShare = 0.5f;

    // Aerodynamic drag only (no downforce or lift): F = 0.5 * rho * dragArea * v^2 against the body velocity, at the
    // center of mass. dragArea = Cd * frontal area, m^2. 0 = no drag.
    float dragArea = 0.0f;
    float airDensity = 1.225f; // kg/m^3

    DrivingAidTuning aids;

    bool operator==(const VehicleTuning&) const = default;
};

// Per-axle geometry that requires recreating the vehicle.
struct AxleConfig {
    float track = 1.80f;       // m between wheel centers
    float wheelRadius = 0.40f; // m
    float wheelWidth = 0.30f;  // m
    float wheelMass = 35.0f;   // kg, only for wheel rotational inertia
    float wheelInertia = 0.0f; // kg*m^2; 0 = solid disc 0.5 * wheelMass * wheelRadius^2
    float wheelAngularDamping = 0.2f; // Jolt wheel angular damping, 1/s
    // Tire vertical compliance, acting once the suspension is solid (at suspensionMinLength): the hub stops there and
    // the tire deflects up to tireDeflection (then the rim: Jolt's rigid hard point) with tireStiffness. Below the
    // full bump the suspension spring is much softer and the tire is taken as rigid. 0 = rigid tire. Car and light
    // truck tires are some 150-300 kN/m, roughly ten times the suspension's ride rate (Gillespie, ch. 5); the
    // deflection to the rim is taken as about 60 % of the sidewall height.
    float tireStiffness = 250000.0f; // N/m
    float tireDeflection = 0.10f;    // m
    // Damping while the tire deflects (fraction of critical for tireStiffness and the wheel's suspension mass). It stands
    // for the losses of a bottoming that are not modelled: the unsprung mass's impact (the wheel is massless in Jolt),
    // the tire's carcass damping and the bump stop's hysteresis on the way out. The suspension damper is off in this
    // phase (no travel left); at 5 % of critical ~85 % of the bottoming energy went back into the body.
    float tireDamping = 1.0f;
    // Suspension travel: length from the attachment point to the wheel center
    float suspensionMinLength = 0.15f; // m, full compression (Jolt hard point)
    float suspensionMaxLength = 0.55f; // m, full droop (0.40 m travel)
    float suspensionPreload = 0.0f;    // m
    float maxSteerAngleDeg = 0.0f;     // degrees, 0 = not steered
    // Visual only (wheel mesh; no effect on physics). The tire's outer radius and width are always
    // wheelRadius / wheelWidth; these only shape the rim and the tread.
    float rimRatio = 0.58f;            // rim diameter / tire diameter (sidewall height = (1 - rimRatio) * radius)
    std::string tread = "mud_terrain"; // mud_terrain | all_terrain | road | slick

    float wheelInertiaOrDisc() const { return wheelInertia > 0.0f ? wheelInertia : 0.5f * wheelMass * wheelRadius * wheelRadius; }
    bool operator==(const AxleConfig&) const = default;
};

// Parameters that require recreating the vehicle (respawn in place).
struct VehicleConfig {
    // Identity and references (not physics)
    std::string id = "default";
    std::string name = "Default off-roader";
    std::string visualModel = "trailforge_suv"; // procedural body model (src/render/VehicleModel)
    // How the procedural body is built: "kit" (StyleKit panels; collision = the boxes of vehicle/VehicleShape.h) or
    // "traced" (meshed from the embedded blueprint; collision = its convex hulls, vehicle/VehicleBlueprint.h).
    std::string bodyShape = "kit";
    std::string audioProfile = "trailforge_suv"; // sound set (assets/audio/manifest.json) and tire sound preset
    // Blender-authored body (vehicle.json body.model: a converted body.glb, vehicle/VehicleBody.h), resolved to a
    // path; empty = the procedural body of visualModel. Its convex collision pieces (body space) are read when the
    // definition loads and replace the procedural body's collision. bodyPaint: colour overrides per paint role
    // ("paint_primary", "paint_secondary", "paint_accent"; sRGB 0..1) for that body.
    std::string bodyModel;
    std::vector<std::vector<std::array<float, 3>>> bodyHulls;
    std::map<std::string, std::array<float, 3>> bodyPaint;
    TireSoundProfile tireSound = tireSoundPreset("trailforge_suv"); // audioProfile preset + vehicle.json `tireSound`
    std::string bodySoundId = "suv"; // vehicle.json `bodySound`, default from audioProfile
    BodySoundProfile bodySound = bodySoundPreset("suv");

    // Mass and inertia
    float mass = 2000.0f;          // kg (body; Jolt wheels are virtual, their mass only counts as rotational inertia)
    float inertiaScale = 1.0f;     // multiplier on the inertia tensor computed from the body shape (uniform density)
    std::array<float, 3> inertia{0.0f, 0.0f, 0.0f}; // kg*m^2 principal Ixx, Iyy, Izz in body axes; all > 0 replaces the shape tensor
    float comHeight = -0.10f;      // m, center of mass Y in body space (body origin ~0.77 m above ground)
    float comForward = 0.05f;      // m, center of mass Z (0 = midway between the axles)
    float bodyFriction = 0.5f;     // body friction (underbody grounding, rollover)
    float bodyAngularDamping = 0.02f;

    // Geometry
    float wheelBase = 2.80f;    // m
    float attachHeight = 0.05f; // m, Y of the suspension attachment points in body space (lower = higher ride)
    std::array<AxleConfig, 2> axles{AxleConfig{.maxSteerAngleDeg = 35.0f}, AxleConfig{}};

    // Engine (Jolt VehicleEngine)
    EngineType engineType = EngineType::Combustion;
    float engineMinRPM = 800.0f;
    float engineMaxRPM = 4800.0f;
    float engineInertia = 0.6f;         // kg*m^2
    float engineAngularDamping = 0.2f;  // Jolt engine damping
    // Normalized torque curve: (rpm fraction between min and max rpm, torque fraction of engineMaxTorque).
    std::vector<std::pair<float, float>> torqueCurve{{0.0f, 0.8f}, {0.66f, 1.0f}, {1.0f, 0.8f}};

    // Transmission (automatic)
    std::vector<float> gearRatios{4.20f, 2.55f, 1.60f, 1.15f, 0.85f};
    std::vector<float> reverseRatios{-4.00f};
    float shiftUpRPM = 3800.0f;
    float shiftDownRPM = 1500.0f;
    float shiftTime = 0.10f;         // s, Jolt mSwitchTime: torque-free phase (an automatic hands over between clutches)
    float clutchReleaseTime = 0.25f; // s
    float shiftLatency = 0.5f;       // s, minimum time between shifts
    float lowRangeRatio = 2.72f;     // transfer case low range (1 = none)

    DriveLayout layout = DriveLayout::Awd;

    // Tire set on all four wheels (vehicle.json tires.set, garage "tires.set"). Fitting another set recreates the
    // vehicle: the new tires start at the ambient temperature.
    TireSet tireSet = TireSet::Offroad;

    // Electric powertrain (engineType Electric, layout DualMotor); the engine and gearbox fields above are unused.
    ElectricConfig electric;
    bool isElectric() const { return engineType == EngineType::Electric; }

    VehicleTuning tuning;

    // Hydrodynamic coefficients for driving through water (vehicle.json "water"; sim/WaterForces.h).
    VehicleWater water;

    // Hood camera mount (vehicle.json camera.hood; view only, not physics), body space. The default sits at the roof
    // line in front of the windshield of the shared SUV / EV body (VehicleModel.cpp: belt deck y 0.655, windshield
    // from z 0.92 at the deck to z 0.40 at the roof y 1.20), the hood fills the bottom ~18% of a 60 deg view.
    std::array<float, 3> hoodCamera{0.0f, 1.24f, 0.62f}; // m
    float hoodCameraPitch = -1.0f;                       // deg, negative looks down onto the hood
    float hoodCameraNear = 0.05f;                        // m, near plane in the hood view

    bool operator==(const VehicleConfig&) const = default;
};

const char* driveLayoutName(DriveLayout l);   // "RWD", "FWD", "AWD", "PART-TIME 4WD", "ON-DEMAND AWD", "DUAL-MOTOR AWD"
const char* transferModeName(TransferMode m); // "2H", "4H", "4L"
const char* evDriveModeName(EvDriveMode m);   // "NORMAL", "OFF-ROAD"

} // namespace tf
