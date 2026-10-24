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
    case SurfaceId::Concrete:
        slip(L::Skid, 1.0f);
        break;
    case SurfaceId::LowGrip: // hard, only a lower mu: a weaker squeal
        slip(L::Skid, 0.55f);
        break;
    case SurfaceId::Rock:
        slip(L::Skid, 0.6f);
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
    default:
        break;
    }
    return r;
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
            const float c = std::max(r.roll[i] * roll, r.slip[i] * slip);
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

} // namespace tf
