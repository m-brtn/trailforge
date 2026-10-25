#include "sim/WheelSpray.h"

#include "track/TestTrack.h"

#include <algorithm>
#include <cmath>

namespace tf {

namespace {

constexpr float kGravity = 9.81f;

// Emission per wheel: rate = k * (slip speed - kSlipOnset) per m/s of slip, plus `roll` per meter rolled.
constexpr float kSlipOnset = 0.6f;

// Particle counts per wheel. Fine material (crumbs: droplets, specks, grains) dominates by count; clods are rare and
// small. Fine counts are scaled with the particle cap (density()), so every preset keeps a continuous fan.
struct Recipe {
    float clod, clodRoll;   // clods [1/s per m/s of slip], [1/m]
    float crumb, crumbRoll; // fine bits
    float dust, dustRoll;   // dust [1/s per m/s of slip], [1/m above kDustRollSpeed]
    float grass;            // grass bits
    float clodMin, clodMax; // clod radius, m
    float fineMin, fineMax; // fine radius, m (skewed to fineMin)
    float mass;             // material thrown per m/s of slip (wheelThrowRate)
};

constexpr float kDustRollSpeed = 6.0f; // m/s: dry ground starts to dust behind a rolling wheel

Recipe recipe(SurfaceId s) {
    switch (s) {
    case SurfaceId::Mud: return {1.2f, 0.02f, 160.0f, 0.8f, 2.5f, 0.0f, 0.0f, 0.010f, 0.026f, 0.0015f, 0.013f, 1.0f};
    case SurfaceId::PackedDirt:
        return {0.8f, 0.004f, 70.0f, 0.08f, 4.0f, 0.5f, 0.0f, 0.009f, 0.022f, 0.0012f, 0.009f, 0.5f};
    case SurfaceId::Sand: return {0.0f, 0.0f, 130.0f, 0.12f, 6.0f, 0.8f, 0.0f, 0.0f, 0.0f, 0.0008f, 0.0040f, 0.7f};
    case SurfaceId::Snow: return {0.3f, 0.0f, 150.0f, 0.10f, 0.0f, 0.0f, 0.0f, 0.008f, 0.02f, 0.0010f, 0.0050f, 0.6f};
    case SurfaceId::Grass: return {0.6f, 0.004f, 35.0f, 0.0f, 0.0f, 0.0f, 24.0f, 0.009f, 0.022f, 0.0012f, 0.008f, 0.45f};
    default: return {};
    }
}

// Quadratic drag [1/m] of a solid bit: ~1/radius (a 2 mm sand grain falls at ~8 m/s, a 2 cm clod barely slows).
float bitDrag(SprayKind k, float size) {
    switch (k) {
    case SprayKind::Clod:
    case SprayKind::Crumb: return 0.0003f / std::max(size, 0.0006f);
    case SprayKind::Dust: return 1.6f;
    case SprayKind::Grass: return 0.8f;
    case SprayKind::Water: return 0.05f;
    }
    return 0.1f;
}

float saturate(float v) { return std::clamp(v, 0.0f, 1.0f); }

} // namespace

float packSprayCode(SprayKind kind, bool resting, bool wet, float seed) {
    return float(int(kind) * 2 + (resting ? 1 : 0)) + 0.49f * saturate(seed) + (wet ? 0.5f : 0.0f);
}

float wheelThrowRate(const WheelContactSample& c) {
    if (!c.contact) return 0.0f;
    return recipe(c.surface).mass * (std::max(0.0f, c.slipSpeed() - kSlipOnset) + 0.08f * std::abs(c.rollingSpeed));
}

WheelSpray::WheelSpray() {
    m_ground = [](float x, float z) { return TestTrack::provingGroundHeight(x, z); };
    m_particles.reserve(m_settings.maxParticles);
}

void WheelSpray::setSettings(const WheelSpraySettings& s) {
    m_settings = s;
    if (m_particles.size() > s.maxParticles) m_particles.resize(s.maxParticles);
    if (!s.dust)
        m_particles.erase(std::remove_if(m_particles.begin(), m_particles.end(),
                                         [](const SprayParticle& p) { return p.kind == SprayKind::Dust; }),
                          m_particles.end());
    m_particles.reserve(s.maxParticles);
    countDust();
}

void WheelSpray::countDust() {
    m_dust = std::size_t(std::count_if(m_particles.begin(), m_particles.end(),
                                       [](const SprayParticle& p) { return p.kind == SprayKind::Dust; }));
}

void WheelSpray::emit(SprayParticle p) {
    const std::size_t cap = m_settings.maxParticles, n = m_particles.size();
    // Above 3/4 of the cap new bits are thinned out progressively, so a full cap does not chop the fan into bursts.
    const bool thinned = n * 4 > cap * 3 && rnd() * float(cap) * 0.25f > float(cap - std::min(n, cap));
    if (n >= cap || thinned || (p.kind == SprayKind::Dust && m_dust >= dustCap())) {
        ++m_dropped;
        return;
    }
    p.seed = rnd();
    m_dust += p.kind == SprayKind::Dust ? 1 : 0;
    m_particles.push_back(p);
    ++m_emitted;
}

void WheelSpray::update(std::span<const WheelContactSample> wheels, std::span<const TreadSoil> soil, float dt) {
    if (dt <= 0.0f) return;
    if (m_settings.enabled) {
        for (std::size_t w = 0; w < wheels.size() && w < 4; ++w) {
            const WheelContactSample& c = wheels[w];
            if (!c.contact) continue;
            const JPH::Vec3 pc = c.patchCenter;
            const bool inWater = underWater(m_water, pc);
            const Recipe r = recipe(c.surface);
            const SoilType ground = soilType(c.surface);
            const float slip = c.slipSpeed(), speed = c.velocity.Length();
            const float rolled = (speed + 0.5f * std::abs(c.rollingSpeed)) * dt;
            const float excess = std::max(0.0f, slip - kSlipOnset) * dt;
            const float rate = m_settings.rate, fine = rate * density();
            float* acc = m_accum[w];
            acc[0] += rate * (r.clod * excess + r.clodRoll * rolled);
            acc[1] += fine * (r.crumb * excess + r.crumbRoll * rolled);
            if (m_settings.dust)
                acc[2] += rate * (r.dust * excess + r.dustRoll * std::max(0.0f, speed - kDustRollSpeed) * dt);
            acc[3] += fine * r.grass * excess;
            if (inWater) acc[4] += fine * (40.0f * slip + 30.0f * speed) * dt;

            // Colour: the ground's soil, darkened by what the tread already carries (wet mud on the tread).
            float col[3];
            const TreadSoil* ts = w < soil.size() ? &soil[w] : nullptr;
            for (int k = 0; k < 3; ++k) {
                col[k] = ground.pickup > 0.0f ? ground.color[k] : 0.1f;
                if (ts && ts->amount > 0.05f) col[k] = col[k] * 0.6f + ts->color[k] * 0.4f;
            }
            const bool sand = c.surface == SurfaceId::Sand;
            const bool wet = ground.wetness > 0.5f || inWater;
            // Thin flying droplets of wet mud read lighter than the mud in bulk (light scatters through the film).
            if (wet)
                for (float& k : col) k *= 1.8f;
            const float R = c.radius > 0.0f ? c.radius : 0.4f;
            const float s = c.rollingSpeed >= 0.0f ? 1.0f : -1.0f, roll = std::abs(c.rollingSpeed);
            const bool plow = roll < 0.3f * speed; // locked or nearly locked: the patch pushes material ahead
            const JPH::Vec3 lon = c.longitudinal, n = c.normal, lat = c.lateral;
            const float vs = c.velocity.Dot(lon) >= 0.0f ? 1.0f : -1.0f;

            // Thrown off the tread at angle theta behind the patch (tangential), or bulldozed ahead when sliding.
            auto thrown = [&](SprayParticle& p, float up) {
                if (plow) {
                    p.pos = pc + lon * (vs * range(0.15f, 0.3f)) + n * range(0.02f, 0.08f) + lat * range(-0.12f, 0.12f);
                    p.vel = c.velocity * range(0.5f, 0.95f) + n * (slip * range(0.08f, 0.3f) * up) +
                            lat * (slip * range(-0.1f, 0.1f));
                } else {
                    const float th = range(0.15f, 0.75f);
                    p.pos = pc - lon * (s * R * std::sin(th)) + n * (R * (1.0f - std::cos(th)) + 0.02f) +
                            lat * range(-0.14f, 0.14f);
                    const JPH::Vec3 rel = (lon * (-s * std::cos(th)) + n * std::sin(th)) * (roll * range(0.6f, 1.0f));
                    p.vel = c.velocity + rel * up + lat * (slip * range(-0.15f, 0.15f)) +
                            n * (slip * range(0.0f, 0.2f));
                }
                p.vel += lat * (c.lateralSlipSpeed * range(0.2f, 0.6f));
            };
            auto tint = [&](SprayParticle& p, float lo, float hi) {
                const float t = range(lo, hi);
                for (int k = 0; k < 3; ++k) p.color[k] = col[k] * t * range(0.95f, 1.05f);
            };

            for (; acc[0] >= 1.0f; acc[0] -= 1.0f) {
                SprayParticle p;
                p.kind = SprayKind::Clod;
                thrown(p, 0.9f);
                const float u = rnd();
                p.size = r.clodMin + (r.clodMax - r.clodMin) * u * u;
                p.life = range(1.5f, 2.5f);
                p.wet = wet;
                tint(p, 0.8f, 1.1f);
                emit(p);
            }
            for (; acc[1] >= 1.0f; acc[1] -= 1.0f) {
                SprayParticle p;
                p.kind = SprayKind::Crumb;
                // Fine droplets leave the tread faster than the clods (thrown from the lug tips, less mass).
                thrown(p, sand ? range(0.7f, 1.0f) : range(0.85f, 1.15f));
                const float u = rnd();
                p.size = r.fineMin + (r.fineMax - r.fineMin) * u * u * u;
                p.life = range(1.0f, 1.6f);
                p.wet = wet;
                tint(p, sand ? 0.95f : 0.75f, sand ? 1.3f : 1.15f);
                emit(p);
            }
            for (; acc[2] >= 1.0f; acc[2] -= 1.0f) {
                SprayParticle p;
                p.kind = SprayKind::Dust;
                p.pos = pc - lon * (s * range(0.1f, 0.4f)) + n * range(0.05f, 0.25f) + lat * range(-0.2f, 0.2f);
                p.vel = c.velocity * range(0.15f, 0.4f) + (c.velocity - lon * c.rollingSpeed) * range(0.05f, 0.15f) +
                        n * range(0.3f, 1.0f) + lat * range(-0.5f, 0.5f);
                if (wet) {
                    // Wet mud: a short-lived haze of the finest droplets around the tread, not dust.
                    p.vel = c.velocity * range(0.3f, 0.6f) + (lon * (-s) * roll + n * roll * 0.4f) * range(0.15f, 0.35f);
                    p.size = range(0.08f, 0.15f);
                    p.grow = range(0.4f, 0.7f);
                    p.life = range(0.35f, 0.6f);
                    p.alpha = range(0.15f, 0.25f);
                    for (int k = 0; k < 3; ++k) p.color[k] = col[k];
                    emit(p);
                    continue;
                }
                p.size = range(0.15f, 0.3f);
                p.grow = range(0.4f, 0.9f) * (sand ? 1.3f : 1.0f);
                p.life = range(1.8f, 3.2f) * (sand ? 1.4f : 1.0f);
                p.alpha = range(0.2f, 0.4f);
                // Airborne dust looks lighter and greyer than the ground it came from.
                for (int k = 0; k < 3; ++k) p.color[k] = std::min(0.6f, col[k] * 1.5f * 0.65f + 0.47f * 0.35f);
                emit(p);
            }
            for (; acc[3] >= 1.0f; acc[3] -= 1.0f) {
                SprayParticle p;
                p.kind = SprayKind::Grass;
                thrown(p, 0.8f);
                const float u = rnd();
                p.size = 0.007f + 0.016f * u * u;
                p.life = range(1.2f, 2.0f);
                const float g = range(0.0f, 1.0f);
                p.color[0] = 0.04f + 0.06f * g;
                p.color[1] = 0.07f + 0.05f * g;
                p.color[2] = 0.015f + 0.01f * g;
                emit(p);
            }
            for (; acc[4] >= 1.0f; acc[4] -= 1.0f) {
                SprayParticle p;
                p.kind = SprayKind::Water;
                p.pos = pc + n * range(0.02f, 0.1f) + lat * range(-0.2f, 0.2f) - lon * (s * range(0.0f, 0.3f));
                const float side = rnd() < 0.5f ? -1.0f : 1.0f;
                p.vel = c.velocity * range(0.3f, 0.8f) + lat * (side * range(0.8f, 3.5f)) +
                        n * range(1.0f, 3.0f + 0.25f * slip) - lon * (s * slip * range(0.1f, 0.4f));
                const float u = rnd();
                p.size = 0.004f + 0.022f * u * u;
                p.life = range(0.6f, 1.2f);
                p.alpha = 0.7f;
                p.wet = true;
                p.color[0] = 0.30f; // muddy water
                p.color[1] = 0.28f;
                p.color[2] = 0.24f;
                emit(p);
            }
        }
    }

    // Motion: gravity and quadratic drag (semi-implicit Euler), collision with the ground height. Landed bits stay as
    // flat splats (the ground normal is kept in vel) for a fraction of a second; nothing piles up.
    for (std::size_t i = 0; i < m_particles.size();) {
        SprayParticle& p = m_particles[i];
        p.age += dt;
        p.size += p.grow * dt;
        if (p.age >= p.life) {
            m_dust -= p.kind == SprayKind::Dust ? 1 : 0;
            p = m_particles.back();
            m_particles.pop_back();
            continue;
        }
        if (!p.resting && !p.frozen) {
            const float drag = bitDrag(p.kind, p.size);
            const float g = p.kind == SprayKind::Dust ? -0.02f : 1.0f;
            const float v = p.vel.Length();
            p.vel += (JPH::Vec3(0.0f, -kGravity * g, 0.0f) - p.vel * (drag * v)) * dt;
            p.pos += p.vel * dt;
            const float h = m_ground(p.pos.GetX(), p.pos.GetZ());
            if (p.kind == SprayKind::Dust) {
                if (p.pos.GetY() < h + 0.5f * p.size) {
                    p.pos.SetY(h + 0.5f * p.size);
                    p.vel.SetY(std::max(0.0f, p.vel.GetY()));
                }
            } else if (p.pos.GetY() < h) {
                if (p.kind == SprayKind::Water) {
                    p.life = p.age; // soaks in / rejoins the puddle
                } else {
                    const float x = p.pos.GetX(), z = p.pos.GetZ(), e = 0.2f;
                    const JPH::Vec3 nrm = JPH::Vec3(m_ground(x - e, z) - m_ground(x + e, z), 2.0f * e,
                                                    m_ground(x, z - e) - m_ground(x, z + e)).Normalized();
                    p.pos.SetY(h + 0.008f);
                    p.vel = nrm;
                    p.resting = true;
                    p.landed = p.age;
                    // Wet bits leave a short-lived dark splat; dry grains vanish into the ground almost at once.
                    const float stay = p.kind == SprayKind::Clod ? range(0.3f, 0.5f)
                                       : p.kind == SprayKind::Grass ? 0.4f
                                       : p.wet ? range(0.15f, 0.35f) : range(0.06f, 0.12f);
                    p.life = std::min(p.life, p.age + stay);
                }
            }
        }
        ++i;
    }
}

void WheelSpray::fill(JPH::Vec3Arg center) {
    // Same mix as heavy mud spray: mostly fine droplets in flight (stretched), a few clods, dust up to its cap.
    while (m_particles.size() < m_settings.maxParticles) {
        SprayParticle p;
        const float pick = rnd();
        const bool dust = pick > 0.9f && m_settings.dust && m_dust < dustCap();
        p.kind = dust ? SprayKind::Dust : pick < 0.04f ? SprayKind::Clod : SprayKind::Crumb;
        m_dust += dust ? 1 : 0;
        p.pos = center + JPH::Vec3(range(-3.0f, 3.0f), range(0.1f, 2.0f), range(-3.0f, 3.0f));
        p.vel = JPH::Vec3(range(-2.0f, 2.0f), range(-1.0f, 4.0f), range(-7.0f, 7.0f));
        const float u = rnd();
        p.size = dust ? range(0.3f, 1.0f) : p.kind == SprayKind::Clod ? 0.01f + 0.016f * u * u : 0.001f + 0.009f * u * u * u;
        p.alpha = dust ? 0.2f : 1.0f;
        p.life = 1e6f;
        p.frozen = true; // stays where it was put
        p.wet = true;
        p.seed = rnd();
        p.color[0] = 0.06f;
        p.color[1] = 0.042f;
        p.color[2] = 0.025f;
        m_particles.push_back(p);
    }
}

std::size_t WheelSpray::solids() const {
    return std::size_t(std::count_if(m_particles.begin(), m_particles.end(),
                                     [](const SprayParticle& p) { return p.kind != SprayKind::Dust; }));
}

float WheelSpray::fineShare() const {
    std::size_t fine = 0, all = 0;
    for (const SprayParticle& p : m_particles) {
        if (p.kind == SprayKind::Dust) continue;
        ++all;
        fine += p.size < 0.01f ? 1 : 0;
    }
    return all ? float(fine) / float(all) : 0.0f;
}

std::size_t WheelSpray::write(SprayInstance* dst, std::size_t capacity) const {
    const std::size_t n = std::min(capacity, m_particles.size());
    for (std::size_t i = 0; i < n; ++i) {
        const SprayParticle& p = m_particles[i];
        const float t = p.age / std::max(p.life, 1e-3f);
        float fade, size = p.size;
        if (p.kind == SprayKind::Dust) {
            // Dust swells in quickly and thins out over its life.
            fade = saturate(p.age / 0.15f) * (1.0f - t) * (1.0f - t);
        } else if (p.resting) {
            // Splat: wet bits spread a little as they flatten, dry ones shrink; both fade out.
            const float k = saturate((p.age - p.landed) / std::max(p.life - p.landed, 1e-3f));
            fade = (1.0f - k) * (p.wet ? 0.85f : 0.6f);
            size *= p.wet ? 1.3f + 0.4f * k : 1.0f - 0.5f * k;
        } else {
            fade = saturate((1.0f - t) / 0.2f);
        }
        SprayInstance& o = dst[i];
        o.posSize[0] = p.pos.GetX();
        o.posSize[1] = p.pos.GetY();
        o.posSize[2] = p.pos.GetZ();
        o.posSize[3] = size;
        o.color[0] = p.color[0];
        o.color[1] = p.color[1];
        o.color[2] = p.color[2];
        o.color[3] = p.alpha * fade;
        o.velKind[0] = p.vel.GetX();
        o.velKind[1] = p.vel.GetY();
        o.velKind[2] = p.vel.GetZ();
        o.velKind[3] = packSprayCode(p.kind, p.resting, p.wet, p.seed);
    }
    return n;
}

} // namespace tf
