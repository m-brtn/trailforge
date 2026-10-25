#include "sim/BodyMud.h"

#include "sim/WheelSpray.h"
#include "track/TestTrack.h"

#include <algorithm>
#include <cmath>

namespace tf {

namespace {

constexpr float kBuildRate = 0.13f;   // per unit of thrown material and adhesion: wet-mud wheelspin coats an arch in ~2 s
constexpr float kDryTime = 150.0f;    // s: e-folding of the wetness (wet mud dries lighter and matte)
constexpr float kFallTime = 900.0f;   // s: e-folding of the coat itself away from water
constexpr float kWashRate = 0.35f;    // 1/s: puddle wash of the regions of a wheel in the water at >= 2 m/s

float saturate(float v) { return std::clamp(v, 0.0f, 1.0f); }

} // namespace

void BodyMud::setAll(float amount) {
    for (BodyMudCoat& c : m_coat) {
        c = {};
        c.amount = saturate(amount);
        c.wetness = 1.0f;
    }
}

float BodyMud::maxAmount() const {
    float m = 0.0f;
    for (const BodyMudCoat& c : m_coat) m = std::max(m, c.amount);
    return m;
}

void BodyMud::deposit(int region, float exposure, const SoilType& soil, float target) {
    if (exposure <= 0.0f) return;
    BodyMudCoat& c = m_coat[std::size_t(region)];
    const float x = 1.0f - std::exp(-exposure);
    if (target > c.amount) {
        const float before = c.amount;
        c.amount += (target - c.amount) * x;
        const float w = (c.amount - before) / std::max(c.amount, 1e-4f);
        for (int k = 0; k < 3; ++k) c.color[k] += (soil.color[k] - c.color[k]) * w;
        c.wetness += (soil.wetness - c.wetness) * w;
        c.flecks += (soil.flecks - c.flecks) * w;
    } else {
        // Already thicker than this soil builds: fresh material only refreshes the surface a little.
        const float w = 0.1f * x;
        for (int k = 0; k < 3; ++k) c.color[k] += (soil.color[k] - c.color[k]) * w;
        c.wetness = std::max(c.wetness, c.wetness + (soil.wetness - c.wetness) * w);
    }
}

void BodyMud::update(std::span<const WheelContactSample> wheels, std::span<const TreadSoil> soil, float dt) {
    if (dt <= 0.0f) return;
    const float dry = std::exp(-dt / kDryTime), fall = std::exp(-dt / kFallTime);
    for (BodyMudCoat& c : m_coat) {
        c.wetness *= dry;
        c.amount *= fall;
    }
    for (std::size_t i = 0; i < wheels.size() && i < 4; ++i) {
        const WheelContactSample& c = wheels[i];
        if (!c.contact) continue;
        const int w = int(i);
        const bool rear = w >= 2;
        const JPH::Vec3 pc = c.patchCenter;
        const bool inWater = underWater(m_water, pc);
        const float speed = c.velocity.Length();
        if (inWater) {
            const float wash = std::exp(-dt * kWashRate * saturate(speed / 2.0f));
            for (int r : {kMudArchRear + w, kMudArchFront + w, kMudBehind + w, rear ? int(kMudRearBumper) : int(kMudFrontBumper)}) {
                m_coat[std::size_t(r)].amount *= wash;
                m_coat[std::size_t(r)].wetness = std::max(m_coat[std::size_t(r)].wetness, saturate(speed / 2.0f));
            }
            continue;
        }

        // What is thrown: the loose ground under the tread, or on hard ground the soil the tread still carries.
        SoilType src = soilType(c.surface);
        float rate = wheelThrowRate(c);
        if (src.pickup <= 0.0f) {
            const TreadSoil* ts = i < soil.size() ? &soil[i] : nullptr;
            if (!ts || ts->amount < 0.03f) continue;
            src = {ts->amount, 1.0f, ts->wetness, {ts->color[0], ts->color[1], ts->color[2]}, ts->stick, ts->flecks};
            rate = 0.3f * ts->amount * (std::max(0.0f, c.slipSpeed() - 0.6f) + 0.08f * std::abs(c.rollingSpeed));
        }
        if (rate <= 0.0f) continue;
        // Adhesion: wet clay sticks at once and builds thick, dry dust barely clings and stays a thin film.
        const float adhesion = src.stick * (0.35f + 0.65f * src.wetness);
        const float target = saturate(0.08f + 0.92f * src.stick * (0.45f + 0.55f * src.wetness));
        const float e = kBuildRate * rate * adhesion * dt;

        const float roll = std::abs(c.rollingSpeed);
        const bool plow = roll < 0.3f * speed; // locked: material is pushed ahead low, little reaches the body
        const bool throwBack = plow ? c.velocity.Dot(c.longitudinal) < 0.0f : c.rollingSpeed >= 0.0f;
        const float k = plow ? 0.25f : 1.0f;
        if (throwBack) { // rolling forward: thrown backward
            deposit(kMudArchRear + w, e * k, src, target);
            deposit(kMudBehind + w, e * k * 0.55f, src, target);
            deposit(kMudArchFront + w, e * k * 0.12f, src, target);
            if (rear) deposit(kMudRearBumper, e * k * 0.35f, src, target);
        } else { // reversing: thrown forward
            deposit(kMudArchFront + w, e * k, src, target);
            deposit(kMudArchRear + w, e * k * 0.12f, src, target);
            if (rear) deposit(kMudBehind + (w - 2), e * k * 0.45f, src, target);
            else deposit(kMudFrontBumper, e * k * 0.35f, src, target);
        }
    }
}

} // namespace tf
