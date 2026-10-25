#include "sim/WheelSpray.h"

#include "track/TestTrack.h"

#include <algorithm>
#include <cmath>

namespace tf {

namespace {

constexpr float kGravity = 9.81f;

// Emission per wheel: rate = k * (slip speed - kSlipOnset) per m/s of slip, plus `roll` per meter rolled.
constexpr float kSlipOnset = 0.6f;

struct Recipe {
    float clod, clodRoll;   // clods [1/s per m/s of slip], [1/m]
    float crumb, crumbRoll; // crumbs / sand grains
    float dust, dustRoll;   // dust [1/s per m/s of slip], [1/m above kDustRollSpeed]
    float grass;            // grass bits
    float clodMin, clodMax; // clod radius, m
};

constexpr float kDustRollSpeed = 6.0f; // m/s: dry ground starts to dust behind a rolling wheel

Recipe recipe(SurfaceId s) {
    switch (s) {
    case SurfaceId::Mud: return {30.0f, 0.25f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.03f, 0.075f};
    case SurfaceId::PackedDirt: return {10.0f, 0.03f, 25.0f, 0.05f, 4.0f, 0.5f, 0.0f, 0.02f, 0.05f};
    case SurfaceId::Sand: return {0.0f, 0.0f, 60.0f, 0.10f, 6.0f, 0.8f, 0.0f, 0.02f, 0.04f};
    case SurfaceId::Grass: return {8.0f, 0.02f, 4.0f, 0.0f, 0.0f, 0.0f, 22.0f, 0.02f, 0.05f};
    default: return {};
    }
}

struct KindParams {
    float drag;    // quadratic drag coefficient, 1/m
    float gravity; // fraction of g
};

KindParams kindParams(SprayKind k) {
    switch (k) {
    case SprayKind::Clod: return {0.03f, 1.0f};
    case SprayKind::Crumb: return {0.15f, 1.0f};
    case SprayKind::Dust: return {1.6f, -0.02f};
    case SprayKind::Grass: return {0.8f, 1.0f};
    case SprayKind::Water: return {0.05f, 1.0f};
    }
    return {0.1f, 1.0f};
}

float saturate(float v) { return std::clamp(v, 0.0f, 1.0f); }

} // namespace

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

void WheelSpray::emit(const SprayParticle& p) {
    if (m_particles.size() >= m_settings.maxParticles || (p.kind == SprayKind::Dust && m_dust >= dustCap())) {
        ++m_dropped;
        return;
    }
    m_dust += p.kind == SprayKind::Dust ? 1 : 0;
    m_particles.push_back(p);
    ++m_emitted;
}

void WheelSpray::update(std::span<const WheelContactSample> wheels, std::span<const TreadSoil> soil, float dt) {
    if (dt <= 0.0f) return;
    const WaterInfo water = TestTrack::water();
    if (m_settings.enabled) {
        for (std::size_t w = 0; w < wheels.size() && w < 4; ++w) {
            const WheelContactSample& c = wheels[w];
            if (!c.contact) continue;
            const JPH::Vec3 pc = c.patchCenter;
            const bool inWater = pc.GetY() < water.level - 0.03f && pc.GetX() > water.x0 && pc.GetX() < water.x1 &&
                                 pc.GetZ() > water.z0 && pc.GetZ() < water.z1;
            const Recipe r = recipe(c.surface);
            const SoilType ground = soilType(c.surface);
            const float slip = c.slipSpeed(), speed = c.velocity.Length();
            const float rolled = (speed + 0.5f * std::abs(c.rollingSpeed)) * dt;
            const float excess = std::max(0.0f, slip - kSlipOnset) * dt;
            const float rate = m_settings.rate;
            float* acc = m_accum[w];
            acc[0] += rate * (r.clod * excess + r.clodRoll * rolled);
            acc[1] += rate * (r.crumb * excess + r.crumbRoll * rolled);
            if (m_settings.dust)
                acc[2] += rate * (r.dust * excess + r.dustRoll * std::max(0.0f, speed - kDustRollSpeed) * dt);
            acc[3] += rate * r.grass * excess;
            if (inWater) acc[4] += rate * (40.0f * slip + 30.0f * speed) * dt;

            // Colour: the ground's soil, darkened by what the tread already carries (wet mud on the tread).
            float col[3];
            const TreadSoil* ts = w < soil.size() ? &soil[w] : nullptr;
            for (int k = 0; k < 3; ++k) {
                col[k] = ground.pickup > 0.0f ? ground.color[k] : 0.1f;
                if (ts && ts->amount > 0.05f) col[k] = col[k] * 0.6f + ts->color[k] * 0.4f;
            }
            const bool sand = c.surface == SurfaceId::Sand;
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
                            lat * range(-0.12f, 0.12f);
                    const JPH::Vec3 rel = (lon * (-s * std::cos(th)) + n * std::sin(th)) * (roll * range(0.6f, 1.0f));
                    p.vel = c.velocity + rel * up + lat * (slip * range(-0.15f, 0.15f)) +
                            n * (slip * range(0.0f, 0.2f));
                }
                p.vel += lat * (c.lateralSlipSpeed * range(0.2f, 0.6f));
            };
            auto tint = [&](SprayParticle& p, float lo, float hi) {
                const float t = range(lo, hi);
                for (int k = 0; k < 3; ++k) p.color[k] = col[k] * t;
            };

            for (; acc[0] >= 1.0f; acc[0] -= 1.0f) {
                SprayParticle p;
                p.kind = SprayKind::Clod;
                thrown(p, 1.0f);
                p.size = range(r.clodMin, r.clodMax);
                p.life = range(2.0f, 3.0f);
                tint(p, 0.8f, 1.15f);
                emit(p);
            }
            for (; acc[1] >= 1.0f; acc[1] -= 1.0f) {
                SprayParticle p;
                p.kind = SprayKind::Crumb;
                thrown(p, sand ? 0.8f : 1.0f);
                p.size = sand ? range(0.008f, 0.018f) : range(0.008f, 0.02f);
                p.life = range(1.2f, 2.0f);
                tint(p, sand ? 1.0f : 0.8f, sand ? 1.3f : 1.2f);
                emit(p);
            }
            for (; acc[2] >= 1.0f; acc[2] -= 1.0f) {
                SprayParticle p;
                p.kind = SprayKind::Dust;
                p.pos = pc - lon * (s * range(0.1f, 0.4f)) + n * range(0.05f, 0.25f) + lat * range(-0.2f, 0.2f);
                p.vel = c.velocity * range(0.15f, 0.4f) + (c.velocity - lon * c.rollingSpeed) * range(0.05f, 0.15f) +
                        n * range(0.3f, 1.0f) + lat * range(-0.5f, 0.5f);
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
                p.size = range(0.015f, 0.035f);
                p.life = range(1.5f, 2.5f);
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
                p.size = range(0.015f, 0.04f);
                p.life = range(0.6f, 1.2f);
                p.alpha = 0.7f;
                p.color[0] = 0.30f; // muddy water
                p.color[1] = 0.28f;
                p.color[2] = 0.24f;
                emit(p);
            }
        }
    }

    // Motion: gravity and quadratic drag (semi-implicit Euler), collision with the ground height.
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
        if (!p.resting) {
            const KindParams k = kindParams(p.kind);
            const float v = p.vel.Length();
            p.vel += (JPH::Vec3(0.0f, -kGravity * k.gravity, 0.0f) - p.vel * (k.drag * v)) * dt;
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
                } else if (p.kind == SprayKind::Clod && p.vel.GetY() < -2.0f) {
                    p.pos.SetY(h);
                    p.vel = JPH::Vec3(p.vel.GetX() * 0.4f, -p.vel.GetY() * 0.2f, p.vel.GetZ() * 0.4f);
                } else {
                    p.pos.SetY(h + 0.3f * p.size);
                    p.vel = JPH::Vec3::sZero();
                    p.resting = true;
                    p.life = std::min(p.life, p.age + (p.kind == SprayKind::Clod ? 1.2f : 0.5f));
                }
            }
        }
        ++i;
    }
}

void WheelSpray::fill(JPH::Vec3Arg center) {
    while (m_particles.size() < m_settings.maxParticles) {
        SprayParticle p;
        const float pick = rnd();
        p.kind = pick < 0.4f ? SprayKind::Clod : pick < 0.8f ? SprayKind::Crumb
                                                             : (m_settings.dust && m_dust < dustCap() ? SprayKind::Dust
                                                                                                       : SprayKind::Grass);
        m_dust += p.kind == SprayKind::Dust ? 1 : 0;
        p.pos = center + JPH::Vec3(range(-3.0f, 3.0f), range(0.1f, 2.0f), range(-3.0f, 3.0f));
        p.vel = JPH::Vec3(range(-1.0f, 1.0f), range(0.0f, 2.0f), range(-1.0f, 1.0f));
        p.size = p.kind == SprayKind::Dust ? range(0.3f, 1.0f) : range(0.01f, 0.06f);
        p.alpha = p.kind == SprayKind::Dust ? 0.2f : 1.0f;
        p.life = 1e6f;
        p.resting = true; // stays where it was put
        p.color[0] = 0.2f;
        p.color[1] = 0.14f;
        p.color[2] = 0.08f;
        m_particles.push_back(p);
    }
}

std::size_t WheelSpray::write(SprayInstance* dst, std::size_t capacity) const {
    const std::size_t n = std::min(capacity, m_particles.size());
    for (std::size_t i = 0; i < n; ++i) {
        const SprayParticle& p = m_particles[i];
        const float t = p.age / std::max(p.life, 1e-3f);
        // Dust swells in quickly and thins out over its life; solid bits fade only at the very end.
        const float fade = p.kind == SprayKind::Dust ? saturate(p.age / 0.15f) * (1.0f - t) * (1.0f - t)
                                                     : saturate((1.0f - t) / 0.2f);
        SprayInstance& o = dst[i];
        o.posSize[0] = p.pos.GetX();
        o.posSize[1] = p.pos.GetY();
        o.posSize[2] = p.pos.GetZ();
        o.posSize[3] = p.size;
        o.color[0] = p.color[0];
        o.color[1] = p.color[1];
        o.color[2] = p.color[2];
        o.color[3] = p.alpha * fade;
        o.velKind[0] = p.vel.GetX();
        o.velKind[1] = p.vel.GetY();
        o.velKind[2] = p.vel.GetZ();
        o.velKind[3] = float(int(p.kind)) + (p.resting ? 0.5f : 0.0f);
    }
    return n;
}

} // namespace tf
