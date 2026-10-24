#pragma once

// Off-road vehicle configuration. Starting values for tuning, not a reference real vehicle.
// Units are given for each field. Body local axes: +Z forward, +Y up, +X left.

namespace tf {

enum class DiffMode { Open, LimitedSlip };

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

    // Tires: longitudinal grip curve mu(slip ratio) and lateral mu(slip angle).
    float tirePeakLongFriction = 1.20f; // mu at the peak
    float tireSlideLongFriction = 0.95f; // mu at full slide (slip ratio >= 1)
    float tirePeakSlipRatio = 0.10f;    // dimensionless
    float tirePeakLatFriction = 1.15f;
    float tireSlideLatFriction = 0.90f; // at 30 degrees and above
    float tirePeakSlipAngleDeg = 6.0f;  // degrees
    // A tire below the peak slip ratio may transmit up to the peak longitudinal friction before it breaks away,
    // instead of mu(slip of the previous step), which is 0 for a wheel that just gripped. false = plain Jolt curve.
    bool tireStaticGrip = true;

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

    // Differentials (Jolt supports open / limited slip)
    DiffMode frontDiff = DiffMode::Open;
    DiffMode rearDiff = DiffMode::LimitedSlip;
    DiffMode centerDiff = DiffMode::LimitedSlip;
    float limitedSlipRatio = 1.4f; // ratio of max/min wheel speeds above which torque goes to the slower wheel
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
