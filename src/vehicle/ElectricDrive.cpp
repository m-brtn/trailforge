#include "vehicle/ElectricDrive.h"

#include <algorithm>
#include <cmath>

namespace tf {

float motorDriveLimit(const MotorConfig& m, float omega) {
    const float w = std::abs(omega);
    const float maxW = m.maxRPM * kRpmToRadPerSec;
    float t = m.maxTorque;
    if (w > 1e-3f) t = std::min(t, m.maxPower / w);
    const float fade = std::clamp((maxW - w) / (0.02f * maxW), 0.0f, 1.0f);
    return t * fade;
}

float motorRegenLimit(const MotorConfig& m, float omega) {
    const float w = std::abs(omega);
    float t = m.regenTorque;
    if (w > 1e-3f) t = std::min(t, m.maxPower / w);
    return t;
}

float motorBaseRPM(const MotorConfig& m) {
    return m.maxTorque > 0.0f ? m.maxPower / m.maxTorque / kRpmToRadPerSec : 0.0f;
}

void ElectricDrive::reset() {
    m_tcFactor = {1.0f, 1.0f};
    m_regenFactor = {1.0f, 1.0f};
}

EvOutput ElectricDrive::update(const ElectricConfig& c, const DrivingAidTuning& aids, const EvInput& in) {
    EvOutput out;
    const float dt = std::max(in.dt, 0.0f);

    // Per-axle traction control: the same integral control as the throttle factor of DrivingAids, but one factor per
    // motor on the slower wheel of its axle (a single spinning wheel is the TC brake's job).
    const bool driving = in.traction && in.throttle > 0.05f && in.brake < 0.05f;
    for (int a = 0; a < 2; ++a) {
        float& f = m_tcFactor[std::size_t(a)];
        if (!in.traction) {
            f = 1.0f;
            continue;
        }
        if (driving) {
            const float threshold = std::max(in.tcSlipThreshold, 1e-3f);
            float e = (in.axles[std::size_t(a)].driveSlip - threshold) / threshold;
            if (in.tcGripThreshold > 0.0f)
                e = std::max(e, (in.axles[std::size_t(a)].gripUse - in.tcGripThreshold) / (0.25f * in.tcGripThreshold));
            if (in.rollingBack) e = -1.0f;
            const float rate = e > 0.0f ? -aids.tcCutRate * std::min(e, 2.0f) : aids.tcRecoverRate * std::min(-e, 1.0f);
            f = std::clamp(f + rate * dt, in.tcMinFactor, 1.0f);
        } else {
            f = std::min(1.0f, f + aids.tcRecoverRate * dt);
        }
        if (driving && f < 0.999f) out.tcActive = true;
    }
    out.tcFactor = m_tcFactor;

    // Regen allowance: an axle whose wheel slips under braking or is modulated by ABS loses its regen quickly
    // (absReleaseRate) and gets it back at absApplyRate, so regen cannot lock a wheel on low grip.
    for (int a = 0; a < 2; ++a) {
        const EvAxleState& s = in.axles[std::size_t(a)];
        float& f = m_regenFactor[std::size_t(a)];
        if (s.absModulating || s.brakeSlip > aids.absReleaseSlip) f -= aids.absReleaseRate * dt;
        else f += aids.absApplyRate * dt;
        f = std::clamp(f, 0.0f, 1.0f);
    }
    out.regenFactor = m_regenFactor;

    // Drive torque. The request is a pedal fraction of what both motors can deliver at their current speeds; the
    // drive mode splits it, and what one axle cannot take (motor limit, traction control) goes to the other.
    const float socFade = std::clamp(in.soc / 0.02f, 0.0f, 1.0f);
    if (in.throttle > 0.0f && socFade > 0.0f) {
        std::array<float, 2> cap{}, lim{}, want{};
        for (int a = 0; a < 2; ++a) {
            const MotorConfig& m = c.motors[std::size_t(a)];
            cap[std::size_t(a)] = motorDriveLimit(m, in.axles[std::size_t(a)].motorOmega) * m.reduction;
            lim[std::size_t(a)] = cap[std::size_t(a)] * m_tcFactor[std::size_t(a)];
        }
        const float demand = std::clamp(in.throttle, 0.0f, 1.0f) * socFade * (cap[0] + cap[1]);
        const float s = std::clamp(in.frontShare, 0.0f, 1.0f);
        want[0] = std::min(demand * s, lim[0]);
        want[1] = std::min(demand * (1.0f - s), lim[1]);
        float rest = demand - want[0] - want[1];
        for (int a = 0; a < 2 && rest > 0.0f; ++a) {
            const float add = std::min(rest, lim[std::size_t(a)] - want[std::size_t(a)]);
            want[std::size_t(a)] += add;
            rest -= add;
        }
        out.requestedWheelTorque = demand;
        for (int a = 0; a < 2; ++a)
            out.driveTorque[std::size_t(a)] = want[std::size_t(a)] / c.motors[std::size_t(a)].reduction;
    }

    // Regenerative braking, blended: regen takes the lift-off target and then the pedal's friction torque request
    // of each axle up to its motor limit (faded out at low speed, cut on slip, battery charge power limit); the
    // friction brakes get the rest of the pedal request. The lift-off part is regen only.
    const float fade = c.regenFadeSpeed > 0.0f ? std::clamp(in.speed / c.regenFadeSpeed, 0.0f, 1.0f) : 1.0f;
    const bool full = in.soc >= 0.995f;
    if (in.throttle <= 0.01f && !full && fade > 0.0f) {
        std::array<float, 2> regenCap{}, pedal{}, liftOff{}, regen{};
        float capSum = 0.0f;
        for (int a = 0; a < 2; ++a) {
            const MotorConfig& m = c.motors[std::size_t(a)];
            const EvAxleState& s = in.axles[std::size_t(a)];
            regenCap[std::size_t(a)] =
                motorRegenLimit(m, s.motorOmega) * m.reduction * fade * m_regenFactor[std::size_t(a)];
            pedal[std::size_t(a)] = std::clamp(in.brake, 0.0f, 1.0f) * 2.0f * s.frictionBrakeTorque;
            capSum += regenCap[std::size_t(a)];
        }
        const float radius = 0.5f * (in.axles[0].wheelRadius + in.axles[1].wheelRadius);
        const float liftOffTotal = in.mass * c.liftOffDecel * radius * fade;
        for (int a = 0; a < 2; ++a) {
            liftOff[std::size_t(a)] = capSum > 0.0f ? liftOffTotal * regenCap[std::size_t(a)] / capSum : 0.0f;
            regen[std::size_t(a)] = std::min(liftOff[std::size_t(a)] + pedal[std::size_t(a)], regenCap[std::size_t(a)]);
        }
        // Battery charge power limit (shaft power of both motors).
        float power = 0.0f;
        for (int a = 0; a < 2; ++a)
            power += regen[std::size_t(a)] / c.motors[std::size_t(a)].reduction * std::abs(in.axles[std::size_t(a)].motorOmega);
        const float maxPower = c.regenMaxPowerKW * 1000.0f;
        if (power > maxPower && power > 0.0f)
            for (float& r : regen) r *= maxPower / power;
        for (int a = 0; a < 2; ++a) {
            const std::size_t i = std::size_t(a);
            out.regenTorque[i] = regen[i] / c.motors[i].reduction;
            const float pedalByRegen = std::max(0.0f, regen[i] - liftOff[i]);
            out.frictionScale[i] = pedal[i] > 0.0f ? std::clamp((pedal[i] - pedalByRegen) / pedal[i], 0.0f, 1.0f) : 1.0f;
        }
    }
    return out;
}

void Battery::init(const ElectricConfig& c) {
    capacityJ = double(c.batteryCapacityKWh) * 3.6e6;
    energyJ = capacityJ * double(std::clamp(c.initialSoC, 0.0f, 1.0f));
}

float Battery::step(const ElectricConfig& c, float shaftPower, float dt) {
    const float electrical = shaftPower >= 0.0f ? shaftPower / std::max(c.driveEfficiency, 0.05f)
                                                : shaftPower * c.regenEfficiency;
    const float battery = electrical + c.auxPowerKW * 1000.0f;
    energyJ = std::clamp(energyJ - double(battery) * double(dt), 0.0, capacityJ);
    return battery;
}

} // namespace tf
