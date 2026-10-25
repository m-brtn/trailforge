#include "audio/AudioMapping.h"

#include <algorithm>
#include <cmath>

namespace tf {
namespace {

constexpr float kHalfPi = 1.57079632679f;

float logPosition(float v, float lo, float hi) {
    if (v <= lo) return 0.0f;
    if (v >= hi) return 1.0f;
    return std::log(v / lo) / std::log(hi / lo);
}

float smoothstep(float a, float b, float x) {
    const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

} // namespace

std::array<float, 2> equalPower(float x) {
    const float t = std::clamp(x, 0.0f, 1.0f) * kHalfPi;
    return {std::cos(t), std::sin(t)};
}

std::array<float, 3> engineBandWeights(float sourceRpm, const EngineMixSettings& s) {
    const float rpm = std::max(sourceRpm, 1.0f);
    const auto a = equalPower(logPosition(rpm, s.idleMidLo, s.idleMidHi));
    const auto b = equalPower(logPosition(rpm, s.midHighLo, s.midHighHi));
    // The two crossfade ranges do not overlap: below midHighLo b = {1, 0}, above idleMidHi a = {0, 1}.
    return {a[0], a[1] * b[0], a[1] * b[1]};
}

EngineLayerWeights engineLayerWeights(float engineRpm, float load, const EngineMixSettings& s) {
    const auto band = engineBandWeights(engineRpm * s.pitch, s);
    const auto l = equalPower(load);
    EngineLayerWeights w;
    for (int i = 0; i < 3; ++i) {
        w.off[std::size_t(i)] = band[std::size_t(i)] * l[0];
        w.on[std::size_t(i)] = band[std::size_t(i)] * l[1];
    }
    return w;
}

float enginePlaybackRate(float engineRpm, float pitch, float rpmMeasured) {
    return rpmMeasured > 0.0f ? std::max(engineRpm, 0.0f) * pitch / rpmMeasured : 1.0f;
}

float compressedLevelDb(float sourceDb, float minDb, float maxDb, float spreadDb) {
    if (maxDb - minDb < 1e-3f) return 0.0f;
    const float t = std::clamp((sourceDb - minDb) / (maxDb - minDb), 0.0f, 1.0f);
    return -spreadDb * (1.0f - t);
}

TireSurfaceResponse tireSurfaceResponse(SurfaceId surface) {
    using L = TireLayer;
    TireSurfaceResponse r;
    auto roll = [&](L l, float v) { r.roll[std::size_t(l)] = v; };
    auto slip = [&](L l, float v) { r.slip[std::size_t(l)] = v; };
    switch (surface) {
    case SurfaceId::Rock:
        roll(L::Gravel, 0.7f);
        slip(L::Gravel, 0.5f);
        break;
    case SurfaceId::PackedDirt:
        roll(L::Dirt, 1.0f);
        roll(L::Gravel, 0.35f);
        slip(L::Gravel, 1.0f);
        slip(L::Dirt, 0.6f);
        break;
    case SurfaceId::Mud:
        roll(L::Mud, 1.0f);
        slip(L::Mud, 1.0f);
        break;
    case SurfaceId::Grass:
        roll(L::Grass, 1.0f);
        slip(L::Grass, 0.5f);
        slip(L::Dirt, 0.5f); // spinning tears the turf
        break;
    case SurfaceId::Sand:
        roll(L::Dirt, 0.6f); // soft hiss, no stones
        slip(L::Dirt, 0.8f);
        break;
    case SurfaceId::Snow:
        roll(L::Dirt, 0.4f); // crunch approximated by the soft dirt layer (no snow recording)
        slip(L::Dirt, 0.6f);
        break;
    default:
        break;
    }
    return r;
}

float squealSurface(SurfaceId surface) {
    switch (surface) {
    case SurfaceId::Concrete: return 1.0f;
    case SurfaceId::LowGrip: return 0.55f; // hard, only a lower mu: a weaker squeal
    case SurfaceId::Rock: return 0.45f;    // small, rough contact: mostly scrape (gravel layer)
    default: return 0.0f;
    }
}

namespace {
float loadFactor(const WheelAudio& w, float referenceLoad) {
    return std::clamp(std::sqrt(std::max(w.load, 0.0f) / std::max(referenceLoad, 1.0f)), 0.0f, 1.4f);
}
} // namespace

float wheelRollIntensity(const WheelAudio& w, float referenceLoad) {
    if (!w.contact) return 0.0f;
    const float speed = std::pow(std::clamp(w.rollSpeed / 12.0f, 0.0f, 1.0f), 0.7f);
    return speed * smoothstep(0.2f, 1.0f, w.rollSpeed) * loadFactor(w, referenceLoad);
}

float wheelSlipIntensity(const WheelAudio& w, float referenceLoad) {
    if (!w.contact) return 0.0f;
    const float gate = smoothstep(0.15f, 0.45f, w.slipRatio);
    return std::clamp(w.slipSpeed / 6.0f, 0.0f, 1.0f) * gate * loadFactor(w, referenceLoad);
}

float squealSlip(const WheelAudio& w, float referenceLoad) {
    if (!w.contact) return 0.0f;
    // Elliptic combination: 1 on the onset ellipse (longitudinal 0.55, lateral 0.40), full squeal at 1.7.
    const float lo = w.longRatio / 0.55f, la = w.latRatio / 0.40f;
    const float ratio = smoothstep(1.0f, 1.7f, std::sqrt(lo * lo + la * la));
    const float sliding = smoothstep(2.5f, 9.0f, w.slipSpeed);
    const float ground = smoothstep(1.0f, 2.5f, w.rollSpeed);
    const float loaded = smoothstep(0.15f, 0.4f, w.load / std::max(referenceLoad, 1.0f));
    return ratio * sliding * ground * loaded;
}

float wheelSquealDrive(const WheelAudio& w, float referenceLoad) {
    if (!w.contact) return 0.0f;
    const float surface = squealSurface(SurfaceId(w.surface));
    if (surface <= 0.0f) return 0.0f;
    return surface * squealSlip(w, referenceLoad) * loadFactor(w, referenceLoad) *
           (1.0f - std::clamp(w.soil, 0.0f, 1.0f));
}

TireLayerMix tireLayerMix(const std::array<WheelAudio, kAudioWheels>& wheels, float referenceLoad) {
    TireLayerMix mix;
    std::array<float, kTireLayerCount> power{}, weight{};
    std::array<AudioVec3, kTireLayerCount> pos{};
    for (const WheelAudio& w : wheels) {
        if (!w.contact) continue;
        const TireSurfaceResponse r = tireSurfaceResponse(SurfaceId(w.surface));
        const float roll = wheelRollIntensity(w, referenceLoad);
        const float slip = wheelSlipIntensity(w, referenceLoad);
        for (int l = 0; l < kTireLayerCount; ++l) {
            const std::size_t i = std::size_t(l);
            const float c = TireLayer(l) == TireLayer::Skid ? std::max(w.squeal, 0.0f)
                                                            : std::max(r.roll[i] * roll, r.slip[i] * slip);
            power[i] += c * c;
            weight[i] += c;
            pos[i] = pos[i] + w.position * c;
        }
    }
    for (int l = 0; l < kTireLayerCount; ++l) {
        const std::size_t i = std::size_t(l);
        // RMS over the four wheels: four equal wheels give their own intensity, a single one half of it (-6 dB).
        mix.gain[i] = std::min(1.2f, std::sqrt(0.25f * power[i]));
        if (weight[i] > 1e-5f) mix.position[i] = pos[i] * (1.0f / weight[i]);
        else {
            AudioVec3 c;
            for (const WheelAudio& w : wheels) c = c + w.position * 0.25f;
            mix.position[i] = c;
        }
    }
    return mix;
}

std::array<float, 2> dayNightWeights(float dayAmount) {
    const auto w = equalPower(dayAmount);
    return {w[1], w[0]};
}

float windGain(float speed) {
    const float t = std::clamp((speed - 3.0f) / 27.0f, 0.0f, 1.0f);
    return t * t;
}

StereoGain spatialize(AudioVec3 source, AudioVec3 listenerPos, AudioVec3 listenerRight, AudioVec3 listenerForward,
                      float refDistance, float maxDistance) {
    const AudioVec3 d = source - listenerPos;
    const float dist = length(d);
    float g = refDistance / std::max(dist, refDistance);
    g *= 1.0f - smoothstep(0.6f * maxDistance, maxDistance, dist);
    float pan = 0.0f, front = 1.0f;
    if (dist > 1e-3f) {
        const AudioVec3 n = d * (1.0f / dist);
        // Close sources are less directional (the listener is inside the source extent).
        pan = 0.8f * dot(n, listenerRight) * smoothstep(0.5f, 3.0f, dist);
        front = dot(n, listenerForward);
    }
    g *= 0.85f + 0.15f * (0.5f + 0.5f * front);
    const auto p = equalPower(0.5f * (pan + 1.0f));
    // Equal-power pan: centered = 0.707 per channel, scaled so a centered source keeps unit gain per channel.
    constexpr float kCenter = 1.41421356f;
    return {g * p[0] * kCenter, g * p[1] * kCenter};
}

EvTones evMotorTones(const MotorAudio& m, float vehicleSpeed, const EvSoundSettings& s) {
    EvTones t;
    const float rpm = std::abs(m.rpm), rps = rpm / 60.0f;
    const float load = std::clamp(std::abs(m.torque) / std::max(m.maxTorque, 1.0f), 0.0f, 1.0f);
    const float regen = m.torque * m.rpm < 0.0f ? load : 0.0f;
    const float magnetic = 0.12f + 0.88f * std::pow(load, 0.7f);
    const float mesh = 0.1f + 0.9f * load;
    const float spin = std::clamp((rpm - 150.0f) / 750.0f, 0.0f, 1.0f);
    const float spinS = spin * spin * (3.0f - 2.0f * spin);
    const float order = 2.0f * s.polePairs;
    t.hz = {order * rps, 2.0f * order * rps, s.slots * rps, s.stage1Teeth * rps, s.stage2Teeth * rps / s.stage1Ratio,
            s.inverterHz};
    t.gain = {s.level * spinS * magnetic * (1.0f - 0.3f * regen),
              0.25f * s.level * spinS * magnetic,
              0.35f * s.level * spinS * magnetic * (1.0f + 0.9f * regen),
              0.5f * s.level * spinS * mesh,
              0.45f * s.level * spinS * mesh,
              0.0f};
    // Current flows whenever there is torque (also creeping and holding on a slope at standstill).
    const float fade = std::clamp(1.0f - (vehicleSpeed - 0.5f * s.inverterFadeSpeed) / (0.5f * s.inverterFadeSpeed), 0.0f, 1.0f);
    t.gain[5] = 0.3f * s.level * std::pow(load, 0.8f) * fade * (1.0f + 0.3f * regen);
    for (int i = 0; i < kEvTones; ++i) {
        const float hz = t.hz[std::size_t(i)];
        const float roll = 1.0f / (1.0f + (hz / 5000.0f) * (hz / 5000.0f));
        t.gain[std::size_t(i)] *= hz > 16000.0f ? 0.0f : roll;
    }
    return t;
}

float roadHum(const WheelAudio& w, float referenceLoad) {
    if (!w.contact) return 0.0f;
    const float hard = squealSurface(SurfaceId(w.surface)) > 0.0f ? 1.0f : 0.0f;
    const float v = std::clamp((w.rollSpeed - 2.0f) / 23.0f, 0.0f, 1.0f);
    const float load = std::clamp(w.load / std::max(referenceLoad, 1.0f), 0.0f, 1.6f);
    return hard * std::sqrt(v) * (0.5f + 0.5f * std::min(load, 1.4f));
}

} // namespace tf
