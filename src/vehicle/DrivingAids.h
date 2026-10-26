#pragma once

#include "vehicle/VehicleConfig.h"

#include <array>

namespace tf {

inline constexpr int kAidWheels = 4;

// Driver's aid switches (B, T, X). Separate from VehicleTuning: they are cockpit controls, not tuning.
struct AidControls {
    bool abs = false;
    TractionMode traction = TractionMode::Off;
    int diffLocks = 0; // 0 none, 1 rear, 2 rear + center, 3 all
    static constexpr int kDiffLockStages = 4;
};

// Per-wheel state of the previous physics tick, as the controller sees it.
struct AidWheelInput {
    bool contact = false;
    float groundSpeed = 0.0f;  // m/s, velocity of the ground under the wheel along the wheel's heading (body point
                               // velocity at the hub when the wheel is in the air), + = wheel forward
    float rollingSpeed = 0.0f; // m/s, w * r
};

struct AidInput {
    std::array<AidWheelInput, kAidWheels> wheels;
    float throttle = 0.0f; // 0..1, smoothed pedal
    float brake = 0.0f;    // 0..1, smoothed pedal
    std::array<float, kAidWheels> maxBrakeTorque{}; // N*m per wheel at full pedal
    std::array<bool, kAidWheels> driven{true, true, true, true}; // wheel connected to the engine (TC throttle cut)
    float driveSign = 1.0f;      // +1 forward selected, -1 reverse
    float dt = 0.0f;
};

struct AidOutput {
    float throttle = 0.0f;                         // after the TC throttle cut
    std::array<float, kAidWheels> brakeTorque{};   // N*m per wheel: pedal x ABS pressure + TC brake
    std::array<float, kAidWheels> absPressure{};   // 0..1
    std::array<float, kAidWheels> tcBrake{};       // N*m
    std::array<float, kAidWheels> brakeSlip{};     // (v - w*r) / v, 0 when not evaluated
    std::array<float, kAidWheels> driveSlip{};     // (w*r - v) / max(v, tcMinSpeed)
    bool absActive = false; // at least one wheel modulated this tick
    bool tcActive = false;  // throttle cut or a TC brake this tick
    float throttleFactor = 1.0f;
};

// ABS and traction control as simple per-tick control logic on the pedal and per-wheel brake torques.
// The ABS reference is the true ground speed under each wheel (a real unit estimates it from the wheel speeds).
class DrivingAids {
public:
    AidOutput update(const DrivingAidTuning& t, const AidControls& controls, const AidInput& in);
    void reset();

private:
    std::array<float, kAidWheels> m_pressure{1.0f, 1.0f, 1.0f, 1.0f};
    std::array<float, kAidWheels> m_breakaway{1.0f, 1.0f, 1.0f, 1.0f}; // ABS pressure at the last release
    std::array<bool, kAidWheels> m_releasing{};
    std::array<float, kAidWheels> m_tcBrake{};
    std::array<float, kAidWheels> m_slip{}; // low-passed drive slip
    float m_throttleFactor = 1.0f;
};

} // namespace tf
