#pragma once

#include "sim/TireMarks.h"
#include "vehicle/WheelContact.h"

#include <Jolt/Jolt.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

namespace tf {

// Cosmetic material thrown by the wheels: clods, crumbs, sand grains, dust, grass bits and puddle splashes. Simple
// CPU particles (gravity, quadratic drag, collision with the ground height only), not Jolt bodies; they never touch
// the simulation. Emission comes only from the contact samples: the tread surface moves over the ground at
// velocity - rollingSpeed * longitudinal, so a spinning wheel throws material backward and up, a locked wheel pushes
// it forward, and a wheel rolling without slip throws almost nothing.
enum class SprayKind : std::uint8_t { Clod = 0, Crumb = 1, Dust = 2, Grass = 3, Water = 4 };

struct SprayParticle {
    JPH::Vec3 pos = JPH::Vec3::sZero();
    JPH::Vec3 vel = JPH::Vec3::sZero();
    float age = 0.0f, life = 1.0f;
    float size = 0.03f, grow = 0.0f; // m (billboard radius), m/s
    float color[3] = {0.1f, 0.07f, 0.04f};
    float alpha = 1.0f;
    SprayKind kind = SprayKind::Clod;
    bool resting = false;
};

// GPU instance (must match SprayInstance in shaders/scene.metal).
struct SprayInstance {
    float posSize[4];  // xyz, radius (m)
    float color[4];    // linear rgb, alpha
    float velKind[4];  // xyz velocity (motion stretch), w: kind + 0.5 * resting
};
static_assert(sizeof(SprayInstance) == 48);

struct WheelSpraySettings {
    bool enabled = true;
    std::size_t maxParticles = 3000; // hard cap (per quality preset)
    bool dust = true;                // dust puffs and clouds (off on Low)
    float rate = 1.0f;               // emission multiplier
};

class WheelSpray {
public:
    using GroundHeight = std::function<float(float x, float z)>;

    WheelSpray();
    void setSettings(const WheelSpraySettings& s);
    const WheelSpraySettings& settings() const { return m_settings; }
    void setGround(GroundHeight ground) { m_ground = std::move(ground); }

    // One step: emit from the wheels (soil gives the colour of what the tread carries), then move all particles.
    void update(std::span<const WheelContactSample> wheels, std::span<const TreadSoil> soil, float dt);
    void clear() { m_particles.clear(); m_dust = 0; }
    // Synthetic fill to the cap (cost measurement): particles spread around `center`.
    void fill(JPH::Vec3Arg center);

    std::size_t size() const { return m_particles.size(); }
    const std::vector<SprayParticle>& particles() const { return m_particles; }
    std::uint64_t emitted() const { return m_emitted; }
    std::uint64_t dropped() const { return m_dropped; } // emissions refused at the cap
    // Writes the live particles as GPU instances; returns the count.
    std::size_t write(SprayInstance* dst, std::size_t capacity) const;

private:
    void emit(const SprayParticle& p);
    void countDust();
    std::size_t dustCap() const { return std::min<std::size_t>(m_settings.maxParticles / 6, 500); }
    float rnd() { m_rng = m_rng * 1664525u + 1013904223u; return float(m_rng >> 8) / float(1u << 24); }
    float range(float a, float b) { return a + (b - a) * rnd(); }

    WheelSpraySettings m_settings;
    GroundHeight m_ground;
    std::vector<SprayParticle> m_particles;
    float m_accum[4][5] = {}; // fractional emission per wheel and kind
    std::uint32_t m_rng = 0x9e3779b9u;
    std::size_t m_dust = 0; // live dust puffs (a sixth of the cap, at most 500: they cover the most pixels)
    std::uint64_t m_emitted = 0, m_dropped = 0;
};

} // namespace tf
