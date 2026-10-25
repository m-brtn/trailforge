#pragma once

#include "vehicle/VehicleConfig.h"

#include <array>

// Control logic of a dual-motor electric drivetrain, without Jolt: motor torque envelope, the electronic split
// between the axles (with per-axle traction control), regenerative braking blended with the friction brakes and a
// simple battery energy balance. Vehicle applies the results through the wheels (see Vehicle.cpp,
// ElectricVehicleController); nothing here pushes the body.

namespace tf {

inline constexpr float kRpmToRadPerSec = 2.0f * 3.14159265358979f / 60.0f;

// Torque the motor can deliver at shaft speed omega (rad/s, either sign): min(maxTorque, maxPower / |omega|), fading
// to 0 over the last 2 % below maxRPM.
float motorDriveLimit(const MotorConfig& m, float omega);
// Braking torque the motor can absorb at shaft speed omega: min(regenTorque, maxPower / |omega|).
float motorRegenLimit(const MotorConfig& m, float omega);
// Base speed (end of the constant-torque range), rpm.
float motorBaseRPM(const MotorConfig& m);

struct EvAxleState {
    float motorOmega = 0.0f;   // rad/s, reduction x average wheel speed of the axle (+ = forward rotation)
    float driveSlip = 0.0f;    // low-passed drive slip of the axle's slower wheel in contact (DrivingAids definition)
    float brakeSlip = 0.0f;    // max braking slip (v - w*r) / v of the axle's wheels, 0 below absMinSpeed
    bool absModulating = false; // ABS released pressure on a wheel of this axle on the last tick
    float wheelRadius = 0.4f;  // m
    float frictionBrakeTorque = 0.0f; // N*m per wheel at full pedal (AxleTuning::brakeTorque)
};

struct EvInput {
    float throttle = 0.0f;  // 0..1, smoothed pedal
    float brake = 0.0f;     // 0..1, smoothed pedal (friction brakes before blending)
    float driveSign = 1.0f; // +1 forward selected, -1 reverse
    float speed = 0.0f;     // m/s, |forward speed|
    float mass = 2000.0f;   // kg, for the lift-off regen target
    float soc = 1.0f;       // 0..1
    float frontShare = 0.5f; // split at part throttle (drive mode)
    bool traction = false;   // traction control on (On or OffRoad)
    float tcSlipThreshold = 0.2f; // of the TC mode
    float tcMinFactor = 0.05f;    // of the TC mode
    bool rollingBack = false;     // rolling against the selected direction: the factors recover instead
    std::array<EvAxleState, 2> axles{};
    float dt = 0.0f;
};

struct EvOutput {
    // Commanded motor shaft torques as magnitudes: the controller applies drive toward the selected direction and
    // regen against the wheel rotation.
    std::array<float, 2> driveTorque{};    // N*m at the shaft, >= 0
    std::array<float, 2> regenTorque{};    // N*m at the shaft, >= 0 (applied against the rotation)
    // Fraction of the pedal friction brake torque that stays on the friction brakes after regen (0..1).
    std::array<float, 2> frictionScale{1.0f, 1.0f};
    std::array<float, 2> tcFactor{1.0f, 1.0f}; // per-axle traction control torque factor
    std::array<float, 2> regenFactor{1.0f, 1.0f}; // per-axle regen allowance (slip / ABS)
    bool tcActive = false;
    float requestedWheelTorque = 0.0f; // N*m at the wheels, total drive request before the split
};

class ElectricDrive {
public:
    EvOutput update(const ElectricConfig& c, const DrivingAidTuning& aids, const EvInput& in);
    void reset();

private:
    std::array<float, 2> m_tcFactor{1.0f, 1.0f};
    std::array<float, 2> m_regenFactor{1.0f, 1.0f};
};

// Battery bookkeeping: the controller adds the mechanical shaft power of both motors every substep.
// Energy in double: a 120 kWh pack is 4.3e8 J, and one 1/120 s step of the auxiliary load is a few J.
struct Battery {
    double capacityJ = 0.0;
    double energyJ = 0.0;
    void init(const ElectricConfig& c);
    float soc() const { return capacityJ > 0.0 ? float(energyJ / capacityJ) : 0.0f; }
    // shaftPower W (+ motoring, - regenerating, summed over the motors); returns the battery power, W (+ = discharge).
    float step(const ElectricConfig& c, float shaftPower, float dt);
};

} // namespace tf
