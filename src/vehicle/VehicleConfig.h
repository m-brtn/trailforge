#pragma once

// Off-road vehicle configuration. Starting values for tuning, not a reference real vehicle.
// Units are given for each field. Body local axes: +Z forward, +Y up, +X left.

namespace tf {

// Locked is Jolt's limited-slip model with a ratio close to 1 (lockedSlipRatio): a torque split that sends all drive
// torque to the slower side, not a rigid shaft (see Vehicle::applyDifferentials).
enum class DiffMode { Open, LimitedSlip, Locked };

// Traction control: Off, On (road threshold, cuts the throttle), OffRoad (higher threshold, keeps more throttle and
// mostly brakes the spinning wheel).
enum class TractionMode { Off, On, OffRoad };

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
};

// Parameters applied to the running vehicle between physics steps
// (without recreation): suspension (springs/dampers), tire grip, brakes,
// engine, differentials, anti-roll bars.
struct VehicleTuning {
    // Suspension (Jolt virtual suspension: spring-damper along the wheel suspension axis)
    float frontSpringFrequency = 1.35f; // Hz, spring natural frequency
    float rearSpringFrequency = 1.35f;  // Hz
    float frontDamping = 0.45f;         // dimensionless damping ratio (1 = critical)
    float rearDamping = 0.45f;
    float antiRollFront = 0.0f; // N/m, anti-roll bar stiffness (0 = none; maximum articulation for an off-roader)
    float antiRollRear = 0.0f;

    // Tires: longitudinal grip curve mu(slip ratio) and lateral mu(slip angle) of an all-terrain tire on dry
    // concrete; the surface multipliers scale the whole curve (SurfaceTable).
    float tirePeakLongFriction = 1.00f; // mu at the peak
    float tireSlideLongFriction = 0.80f; // mu at full slide (slip ratio >= 1)
    float tirePeakSlipRatio = 0.10f;    // dimensionless
    float tirePeakLatFriction = 1.00f;
    float tireSlideLatFriction = 0.78f; // at 30 degrees and above
    float tirePeakSlipAngleDeg = 6.0f;  // degrees
    // A tire below the peak slip ratio may transmit up to the peak longitudinal friction before it breaks away,
    // instead of mu(slip of the previous step), which is 0 for a wheel that just gripped. false = plain Jolt curve.
    bool tireStaticGrip = true;
    // Floor of the slip ratio denominator, m/s: slip = |w*r - v| / max(|v|, floor). 0 = Jolt's 0.001 m/s, with which
    // any wheel rotation near a standstill counts as a full slide.
    float tireSlipSpeedFloor = 1.0f;

    // Brakes
    float maxBrakeTorque = 3000.0f;     // N*m per wheel
    float maxHandBrakeTorque = 3500.0f; // N*m per rear wheel

    // Engine and drivetrain
    float engineMaxTorque = 520.0f; // N*m
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

    // Tire friction circle: beyond the peak, the longitudinal and lateral limits share one budget in proportion to
    // the normalized slips (slip ratio / peak, slip angle / peak). A locked or spinning wheel then has little side
    // grip. false = Jolt's independent limits (a locked wheel keeps full side grip).
    bool tireCombinedSlip = true;

    DrivingAidTuning aids;
};

// Parameters that require recreating the vehicle ("Rebuild" button).
struct VehicleConfig {
    // Mass and inertia
    float mass = 2000.0f;          // kg (body; Jolt wheels are virtual, their mass only counts as rotational inertia)
    float inertiaScale = 1.0f;     // multiplier on the inertia tensor computed from the body shape
    float comHeight = -0.10f;      // m, center of mass Y in body space (body origin ~0.77 m above ground)
    float comForward = 0.05f;      // m, center of mass Z

    // Geometry
    float wheelBase = 2.80f;  // m
    float trackWidth = 1.80f; // m (between wheel centers; tires stick out ~6 cm past the flares)
    float wheelRadius = 0.40f; // m
    float wheelWidth = 0.30f;  // m
    float wheelMass = 35.0f;   // kg, only for wheel rotational inertia (0.5*m*r^2)
    float attachHeight = 0.05f; // m, Y of the suspension attachment point in body space

    // Suspension travel: length from the attachment point to the wheel center
    float suspensionMinLength = 0.15f; // m, full compression (bump stop)
    float suspensionMaxLength = 0.55f; // m, full droop (0.40 m travel)
    float suspensionPreload = 0.0f;    // m

    float maxSteerAngleDeg = 35.0f; // degrees

    // Engine
    float engineMinRPM = 800.0f;
    float engineMaxRPM = 4800.0f;
    // Transmission (automatic)
    float gearRatios[5] = {4.20f, 2.55f, 1.60f, 1.15f, 0.85f};
    float reverseRatio = -4.00f;
    float shiftUpRPM = 3800.0f;
    float shiftDownRPM = 1500.0f;
    float differentialRatio = 3.90f; // final drive
    float lowRangeRatio = 2.72f;     // transfer case low range

    VehicleTuning tuning;
};

} // namespace tf
