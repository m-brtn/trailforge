#include "vehicle/DrivingAids.h"

#include <algorithm>
#include <cmath>

namespace tf {

void DrivingAids::reset() {
    m_pressure.fill(1.0f);
    m_breakaway.fill(1.0f);
    m_releasing.fill(false);
    m_tcBrake.fill(0.0f);
    m_slip.fill(0.0f);
    m_throttleFactor = 1.0f;
}

AidOutput DrivingAids::update(const DrivingAidTuning& t, const AidControls& controls, const AidInput& in) {
    AidOutput out;
    const float dt = in.dt;

    // ---- ABS: per-wheel pressure, release above absReleaseSlip, build up below absApplySlip, hold in between.
    for (int i = 0; i < kAidWheels; ++i) {
        const AidWheelInput& w = in.wheels[i];
        const float v = std::abs(w.groundSpeed);
        const bool braking = controls.abs && in.brake > 0.05f && v > t.absMinSpeed;
        float& p = m_pressure[i];
        if (!braking) {
            p = 1.0f;
            m_breakaway[i] = 1.0f;
            m_releasing[i] = false;
            continue;
        }
        // Rolling speed in the direction of travel; a wheel turning backwards reads as slip > 1.
        const float rolling = w.groundSpeed >= 0.0f ? w.rollingSpeed : -w.rollingSpeed;
        const float slip = (v - rolling) / v;
        out.brakeSlip[i] = slip;
        if (!w.contact) {
            // In the air: hold the pressure; the slip is measured again on touchdown.
        } else if (slip > t.absReleaseSlip) {
            if (!m_releasing[i]) m_breakaway[i] = p;
            m_releasing[i] = true;
            p = std::max(t.absMinPressure, p - t.absReleaseRate * dt);
        } else if (slip < t.absApplySlip) {
            m_releasing[i] = false;
            // Quickly back to just below the pressure that broke the wheel away, then slowly up to find the peak.
            const float fast = t.absReapplyFraction * m_breakaway[i];
            const float rate = p < fast ? t.absReleaseRate : t.absApplyRate;
            p = std::min(1.0f, p < fast ? std::min(fast, p + rate * dt) : p + rate * dt);
        }
        if (p < 0.999f) out.absActive = true;
    }

    // ---- Traction control: brake the wheels that spin above the threshold, cut the throttle when every wheel in
    // contact spins (an open differential sends the brake reaction torque to the other side, like a diff lock).
    const bool tcOn = controls.traction != TractionMode::Off;
    const bool offRoad = controls.traction == TractionMode::OffRoad;
    const float threshold = offRoad ? t.tcOffRoadSlipThreshold : t.tcSlipThreshold;
    const bool driving = tcOn && in.throttle > 0.05f && in.brake < 0.05f;
    // Jolt's wheel speeds alternate from tick to tick while a tire is at its grip limit; the controller works on a
    // low-passed slip.
    const float k = std::min(1.0f, dt / std::max(t.tcSlipFilterTime, 6.0f * dt));
    float minSlip = 1e9f;
    std::array<bool, kAidWheels> spinning{};
    float groundSum = 0.0f;
    int inContact = 0;
    for (int i = 0; i < kAidWheels; ++i) {
        const AidWheelInput& w = in.wheels[i];
        // Positive when the wheel turns faster than the ground in its own rolling direction (also while the vehicle
        // slides backwards with the wheels driving forwards).
        const float dv = w.rollingSpeed - w.groundSpeed;
        const float slip = (dv * w.rollingSpeed > 0.0f ? std::abs(dv) : 0.0f) / std::max(std::abs(w.groundSpeed), t.tcMinSpeed);
        spinning[i] = slip > 0.0f && std::abs(w.rollingSpeed) > t.tcSpinSpeed;
        m_slip[i] += (slip - m_slip[i]) * k;
        out.driveSlip[i] = m_slip[i];
        if (w.contact && in.driven[i]) minSlip = std::min(minSlip, m_slip[i]);
        if (w.contact) {
            groundSum += w.groundSpeed;
            ++inContact;
        }
    }
    // Rolling back against the selected direction (a hill start that is losing): cutting the throttle only makes it
    // roll back faster, so the throttle cut recovers instead.
    const bool rollingBack = inContact > 0 && in.driveSign * groundSum / float(inContact) < -t.tcRollbackSpeed;
    if (minSlip > 1e8f) minSlip = 0.0f; // all wheels in the air
    // Load-weighted combined slip of the driven tires in contact (tcGripThreshold).
    float useSum = 0.0f, loadSum = 0.0f;
    for (int i = 0; i < kAidWheels; ++i) {
        const AidWheelInput& w = in.wheels[i];
        if (!w.contact || !in.driven[i]) continue;
        useSum += w.gripUse * w.load;
        loadSum += w.load;
    }
    const float gripUse = loadSum > 0.0f ? useSum / loadSum : 0.0f;
    const float gripThreshold = offRoad ? t.tcOffRoadGripThreshold : t.tcGripThreshold;
    // Brake a wheel that spins faster than the slowest driven wheel in contact: through the open differentials the brake
    // reaction drives the other wheels. That only helps while the slowest wheel has grip to spare: in a corner the
    // unloaded inner wheel spins first, and braking it sent its torque to the loaded outer tire already at its limit
    // (a driven rear axle stepped out with TC on). The brake therefore fades out as the slowest wheel's slip approaches
    // the threshold; uniform spin of all wheels is the throttle's job.
    const float spare = std::clamp(1.0f - minSlip / std::max(threshold, 1e-3f), 0.0f, 1.0f);
    for (int i = 0; i < kAidWheels; ++i) {
        const float excess = std::min(m_slip[i] - threshold, m_slip[i] - minSlip - threshold);
        const float target = driving ? std::clamp(t.tcBrakeGain * excess * spare, 0.0f, t.tcMaxBrakeTorque) : 0.0f;
        float& b = m_tcBrake[i];
        const float step = t.tcBrakeRate * dt;
        b = driving ? b + std::clamp(target - b, -step, step) : 0.0f;
        // Released at once when the wheel no longer spins ahead of the ground: Jolt keeps a wheel at w = 0 locked
        // under any brake torque, so a slowly decaying TC brake would hold it locked instead of letting it drive.
        if (!spinning[i]) b = 0.0f;
        if (b > 1.0f) out.tcActive = true;
    }
    // Throttle: integral control of the slowest wheel's slip toward the threshold (error normalized by the threshold;
    // a faster cut than recovery).
    if (driving) {
        float e = (minSlip - threshold) / threshold;
        // Combined slip: an error of one at a quarter beyond the threshold.
        if (gripThreshold > 0.0f) e = std::max(e, (gripUse - gripThreshold) / (0.25f * gripThreshold));
        if (rollingBack) e = -1.0f;
        const float rate = e > 0.0f ? -t.tcCutRate * std::min(e, 2.0f) : t.tcRecoverRate * std::min(-e, 1.0f);
        m_throttleFactor = std::clamp(m_throttleFactor + rate * dt, offRoad ? t.tcOffRoadMinThrottle : t.tcMinThrottle, 1.0f);
    } else {
        m_throttleFactor = std::min(1.0f, m_throttleFactor + t.tcRecoverRate * dt);
    }
    if (!tcOn) m_throttleFactor = 1.0f;
    if (driving && m_throttleFactor < 0.999f) out.tcActive = true;

    out.throttle = in.throttle * m_throttleFactor;
    out.throttleFactor = m_throttleFactor;
    for (int i = 0; i < kAidWheels; ++i) {
        out.absPressure[i] = m_pressure[i];
        out.tcBrake[i] = m_tcBrake[i];
        out.brakeTorque[i] = in.brake * in.maxBrakeTorque[i] * m_pressure[i] + m_tcBrake[i];
    }
    return out;
}

} // namespace tf
