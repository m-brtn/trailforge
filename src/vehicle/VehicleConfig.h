#pragma once

#include "track/SurfaceProperties.h"

#include <array>
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

// Which axles the engine drives (see README "Drivetrain layouts").
//  Rwd / Fwd: one driven axle. Awd: full-time, both axles through the center differential (frontTorqueShare, center
//  diff mode). PartTime: transfer case 2H (rear only) / 4H (center locked) / 4L (center locked + low range).
//  OnDemand: both axles through a center LSD with a rear-biased base split (a simplification of a clutch coupling).
// A dual-motor electric layout (one motor per axle) would be a further value with a power unit per axle.
enum class DriveLayout { Rwd, Fwd, Awd, PartTime, OnDemand };

// Transfer case position of a part-time layout (runtime state, like the low range of the other layouts).
enum class TransferMode { TwoHigh, FourHigh, FourLow };

// Power unit type. Only Combustion is simulated; Electric is reserved in the data format (torque / power curve,
// regenerative braking) and rejected by the loader until it exists.
enum class EngineType { Combustion, Electric };

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
    float absMinPressure = 0.0f;  // pressure floor while releasing (Jolt keeps a stopped wheel locked under any brake torque)
    float absMinSpeed = 1.5f;     // m/s, below this the pedal acts directly (a real ABS also drops out near a stop)

    // Traction control. Drive slip = (w*r - v) / max(v, tcMinSpeed), also for a wheel in the air.
    float tcSlipThreshold = 0.20f;        // On
    float tcOffRoadSlipThreshold = 0.35f; // OffRoad
    float tcMinSpeed = 2.0f;              // m/s, slip denominator floor (slip is undefined at a standstill)
    float tcSlipFilterTime = 0.05f;       // s, low-pass time constant of the measured slip
    float tcBrakeGain = 3000.0f;          // N*m of brake per unit of slip above the threshold
    float tcMaxBrakeTorque = 1500.0f;     // N*m per wheel
    float tcBrakeRate = 8000.0f;          // N*m/s, rate limit of the TC brake torque (both directions)
    float tcCutRate = 1.5f;               // 1/s, throttle factor drop rate at an error of one threshold
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
    // Dimensionless damping ratio (1 = critical). Jolt has one damping coefficient per wheel; before every step it is
    // set to the compression or the rebound value from the direction the suspension moved during the previous step.
    float compressionDamping = 0.45f;
    float reboundDamping = 0.45f;
    float antiRoll = 0.0f; // N/m, anti-roll bar stiffness (0 = none; maximum articulation for an off-roader)
    // Progressive bump stop: within bumpStopRange m of full compression the spring frequency rises linearly up to
    // springFrequency x bumpStopFrequencyScale (set before every step). Range 0 = only Jolt's rigid hard point at
    // the minimum suspension length.
    float bumpStopRange = 0.0f;
    float bumpStopFrequencyScale = 1.0f;
    // Brakes, N*m per wheel of this axle
    float brakeTorque = 3000.0f;
    float handBrakeTorque = 0.0f;

    bool operator==(const AxleTuning&) const = default;
};

// Parameters applied to the running vehicle between physics steps (without recreation).
struct VehicleTuning {
    std::array<AxleTuning, 2> axles{AxleTuning{}, AxleTuning{.handBrakeTorque = 3500.0f}};

    // Tires: longitudinal grip curve mu(slip ratio) and lateral mu(slip angle) of an all-terrain tire on dry
    // concrete; the surface multipliers scale the whole curve (SurfaceTable).
    float tirePeakLongFriction = 1.00f; // mu at the peak
    float tireSlideLongFriction = 0.80f; // mu at full slide (slip ratio >= 1)
    float tirePeakSlipRatio = 0.10f;    // dimensionless
    float tirePeakLatFriction = 1.00f;
    float tireSlideLatFriction = 0.78f; // at 30 degrees and above
    float tirePeakSlipAngleDeg = 6.0f;  // degrees
    // Per-surface multiplier of this tire on top of the surface table (e.g. a mud tread > 1 on mud). 1 = surface
    // table only.
    std::array<float, kSurfaceCount> tireSurfaceGrip{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
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
    float track = 1.80f;       // m between wheel centers (tires stick out ~6 cm past the flares)
    float wheelRadius = 0.40f; // m
    float wheelWidth = 0.30f;  // m
    float wheelMass = 35.0f;   // kg, only for wheel rotational inertia
    float wheelInertia = 0.0f; // kg*m^2; 0 = solid disc 0.5 * wheelMass * wheelRadius^2
    float wheelAngularDamping = 0.2f; // Jolt wheel angular damping, 1/s
    // Suspension travel: length from the attachment point to the wheel center
    float suspensionMinLength = 0.15f; // m, full compression (Jolt hard point)
    float suspensionMaxLength = 0.55f; // m, full droop (0.40 m travel)
    float suspensionPreload = 0.0f;    // m
    float maxSteerAngleDeg = 0.0f;     // degrees, 0 = not steered

    float wheelInertiaOrDisc() const { return wheelInertia > 0.0f ? wheelInertia : 0.5f * wheelMass * wheelRadius * wheelRadius; }
    bool operator==(const AxleConfig&) const = default;
};

// Parameters that require recreating the vehicle (respawn in place).
struct VehicleConfig {
    // Identity and references (not physics)
    std::string id = "default";
    std::string name = "Default off-roader";
    std::string visualModel = "trailforge_suv"; // procedural body model (src/render/VehicleModel)
    std::string audioProfile = "trailforge_suv"; // sound set (assets/audio/manifest.json)

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
    float shiftTime = 0.35f;         // s, Jolt mSwitchTime
    float clutchReleaseTime = 0.25f; // s
    float shiftLatency = 0.5f;       // s, minimum time between shifts
    float lowRangeRatio = 2.72f;     // transfer case low range (1 = none)

    DriveLayout layout = DriveLayout::Awd;

    VehicleTuning tuning;

    bool operator==(const VehicleConfig&) const = default;
};

const char* driveLayoutName(DriveLayout l);   // "RWD", "FWD", "AWD", "PART-TIME 4WD", "ON-DEMAND AWD"
const char* transferModeName(TransferMode m); // "2H", "4H", "4L"

} // namespace tf
