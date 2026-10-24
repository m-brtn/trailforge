#include "track/TestTrack.h"

#include "physics/PhysicsWorld.h"

#include <Jolt/Geometry/Triangle.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace tf {

using JPH::Vec3;
using JPH::Vec3Arg;

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kHalfSize = 50.0f;   // 100 x 100 m proving ground
constexpr float kGridStep = 0.5f;    // terrain grid step, m

float deg(float d) { return d * kPi / 180.0f; }

float smoothstep(float e0, float e1, float x) {
    const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// 1 inside [a, b], smooth falloff to 0 over distance fade outside.
float window(float x, float a, float b, float fade) {
    return smoothstep(a - fade, a, x) * (1.0f - smoothstep(b, b + fade, x));
}

// Trapezoidal hill along z: climb at angle slope, plateau, descent at the same angle.
float hill(float z, float z0, float slopeDeg, float height, float plateau) {
    const float t = std::tan(deg(slopeDeg));
    const float up = height / t;
    if (z < z0) return 0.0f;
    if (z < z0 + up) return (z - z0) * t;
    if (z < z0 + up + plateau) return height;
    const float down = z - (z0 + up + plateau);
    return std::max(0.0f, height - down * t);
}

// Deterministic generator (identical on all platforms, unlike std::*_distribution).
struct Lcg {
    std::uint32_t state;
    float next() {
        state = state * 1664525u + 1013904223u;
        return float(state >> 8) / float(1u << 24);
    }
    float range(float a, float b) { return a + (b - a) * next(); }
};

// Proving ground lanes (x across, z along the driving direction from the start).
constexpr float kLaneA0 = -8.0f, kLaneA1 = 8.0f;    // center: bumps, diagonal, low grip
constexpr float kLaneB0 = -34.0f, kLaneB1 = -18.0f; // climbs
constexpr float kLaneC0 = 16.0f, kLaneC1 = 36.0f;   // side slope, steps, rocks

constexpr float kStartZ0 = -48.0f, kStartZ1 = -30.0f;
constexpr float kLowGripZ0 = 24.0f, kLowGripZ1 = 42.0f;

SurfaceId groundSurface(float x, float z) {
    if (x > kLaneA0 && x < kLaneA1 && z > kStartZ0 && z < kStartZ1) return SurfaceId::Concrete;
    if (x > kLaneA0 && x < kLaneA1 && z > kLowGripZ0 && z < kLowGripZ1) return SurfaceId::LowGrip;
    return SurfaceId::PackedDirt;
}

} // namespace

// ------------------------------------------------------------------ TrackGeometry

void TrackGeometry::addTriangle(Vec3Arg a, Vec3Arg b, Vec3Arg c, SurfaceId s) {
    positions.emplace_back(a.GetX(), a.GetY(), a.GetZ());
    positions.emplace_back(b.GetX(), b.GetY(), b.GetZ());
    positions.emplace_back(c.GetX(), c.GetY(), c.GetZ());
    surfaces.push_back(s);
}

void TrackGeometry::addQuad(Vec3Arg a, Vec3Arg b, Vec3Arg c, Vec3Arg d, SurfaceId s) {
    addTriangle(a, b, c, s);
    addTriangle(a, c, d, s);
}

void TrackGeometry::addBox(Vec3Arg center, Vec3Arg he, float yaw, SurfaceId s) {
    const JPH::Quat q = JPH::Quat::sRotation(Vec3::sAxisY(), yaw);
    auto p = [&](float sx, float sy, float sz) {
        return center + q * Vec3(sx * he.GetX(), sy * he.GetY(), sz * he.GetZ());
    };
    // Top (+Y)
    addQuad(p(-1, 1, -1), p(-1, 1, 1), p(1, 1, 1), p(1, 1, -1), s);
    // +X
    addQuad(p(1, -1, -1), p(1, 1, -1), p(1, 1, 1), p(1, -1, 1), s);
    // -X
    addQuad(p(-1, -1, 1), p(-1, 1, 1), p(-1, 1, -1), p(-1, -1, -1), s);
    // +Z
    addQuad(p(1, -1, 1), p(1, 1, 1), p(-1, 1, 1), p(-1, -1, 1), s);
    // -Z
    addQuad(p(-1, -1, -1), p(-1, 1, -1), p(1, 1, -1), p(1, -1, -1), s);
}

void TrackGeometry::addRock(Vec3Arg center, Vec3Arg radii, float yaw, std::uint32_t seed, SurfaceId s) {
    // Icosahedron subdivided once (80 triangles), with radius noise.
    const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    std::vector<Vec3> v = {
        {-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
        {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    for (auto& p : v) p = p.Normalized();
    std::vector<std::array<int, 3>> f = {
        {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
        {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8},
        {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
    std::vector<std::array<int, 3>> f2;
    auto midpoint = [&](int a, int b) {
        v.push_back((v[a] + v[b]).Normalized());
        return int(v.size() - 1);
    };
    for (auto& tri : f) {
        // Simple subdivision without an edge cache: duplicate midpoints get the same
        // offset because the noise depends only on direction (see below), so there are no cracks.
        const int a = midpoint(tri[0], tri[1]);
        const int b = midpoint(tri[1], tri[2]);
        const int c = midpoint(tri[2], tri[0]);
        f2.push_back({tri[0], a, c});
        f2.push_back({tri[1], b, a});
        f2.push_back({tri[2], c, b});
        f2.push_back({a, b, c});
    }
    const JPH::Quat q = JPH::Quat::sRotation(Vec3::sAxisY(), yaw);
    const float ph1 = float(seed % 97) * 0.37f, ph2 = float(seed % 89) * 0.53f;
    auto warp = [&](Vec3 d) {
        // Noise depends only on direction d, so it is identical for coincident vertices.
        const float n = 0.82f + 0.12f * std::sin(3.1f * d.GetX() + ph1) * std::cos(2.7f * d.GetZ() + ph2)
                      + 0.08f * std::sin(5.3f * d.GetY() + ph2 + 1.3f * d.GetX());
        return center + q * (d * radii * n);
    };
    for (auto& tri : f2)
        addTriangle(warp(v[tri[0]]), warp(v[tri[1]]), warp(v[tri[2]]), s);
}

// ------------------------------------------------------------------ TestTrack

float TestTrack::provingGroundHeight(float x, float z) {
    float h = 0.0f;

    // Lane A: a sequence of small bumps (0.18 m, 3 m apart).
    {
        const float w = window(x, kLaneA0, kLaneA1, 2.0f) * window(z, -26.0f, -8.0f, 0.5f);
        if (w > 0.0f) h += w * 0.18f * 0.5f * (1.0f - std::cos(2.0f * kPi * (z + 26.0f) / 3.0f));
    }
    // Lane A: diagonal waves of +-0.38 m; diagonal wheels hit a crest/trough,
    // which causes wheel lift (cross-axle).
    {
        const float w = window(x, kLaneA0, kLaneA1, 2.0f) * window(z, -2.0f, 18.0f, 2.0f);
        if (w > 0.0f) h += w * 0.38f * std::sin(2.0f * kPi * (z + 0.9f * x) / 7.5f);
    }
    // Lane B: climbs of 12, 20, 28, 35 degrees, 1.8 m high, with 4 m plateaus.
    {
        const float w = window(x, kLaneB0, kLaneB1, 2.5f);
        if (w > 0.0f) {
            float hb = 0.0f;
            hb = std::max(hb, hill(z, -42.0f, 12.0f, 1.8f, 4.0f));
            hb = std::max(hb, hill(z, -16.0f, 20.0f, 1.8f, 4.0f));
            hb = std::max(hb, hill(z, 2.0f, 28.0f, 1.8f, 4.0f));
            hb = std::max(hb, hill(z, 20.0f, 35.0f, 1.8f, 4.0f));
            h += w * hb;
        }
    }
    // Lane C: 18 degree side slope (the right side in the driving direction is lower: height grows toward +X, the left side).
    {
        const float w = window(z, -36.0f, -16.0f, 8.0f) * window(x, kLaneC0 - 2.0f, kLaneC1 + 2.0f, 0.0f);
        if (w > 0.0f) h += w * std::tan(deg(18.0f)) * std::clamp(x - 18.0f, 0.0f, 14.0f);
    }
    // Berm around the perimeter.
    {
        const float d = std::min(kHalfSize - std::abs(x), kHalfSize - std::abs(z));
        if (d < 5.0f) h += 0.08f * (5.0f - d) * (5.0f - d); // up to 2 m at the edge
    }
    return h;
}

TestTrack::TestTrack(PhysicsWorld& world, const SurfaceTable& surfaces, TrackLayout layout)
    : m_world(world), m_layout(layout) {
    switch (layout) {
    case TrackLayout::ProvingGround: buildProvingGround(); break;
    case TrackLayout::Flat: buildFlat(false); break;
    case TrackLayout::FlatWithWheelPit: buildFlat(true); break;
    }

    JPH::TriangleList tris;
    tris.reserve(m_geometry.triangleCount());
    for (std::size_t i = 0; i < m_geometry.triangleCount(); ++i) {
        tris.emplace_back(m_geometry.positions[3 * i], m_geometry.positions[3 * i + 1],
                          m_geometry.positions[3 * i + 2], JPH::uint32(m_geometry.surfaces[i]));
    }
    JPH::PhysicsMaterialList materials;
    for (std::size_t i = 0; i < kSurfaceCount; ++i)
        materials.push_back(surfaces.material(static_cast<SurfaceId>(i)));

    JPH::MeshShapeSettings meshSettings(tris, materials);
    JPH::ShapeSettings::ShapeResult result = meshSettings.Create();
    if (result.HasError())
        throw std::runtime_error(std::string("MeshShape: ") + result.GetError().c_str());

    JPH::BodyCreationSettings bcs(result.Get(), JPH::RVec3::sZero(), JPH::Quat::sIdentity(),
                                  JPH::EMotionType::Static, Layers::Static);
    bcs.mFriction = 0.8f; // body-ground friction (not tires; tires are handled by VehicleConstraint)
    m_body = world.bodies().CreateAndAddBody(bcs, JPH::EActivation::DontActivate);
    world.optimizeBroadPhase();
}

TestTrack::~TestTrack() {
    m_world.bodies().RemoveBody(m_body);
    m_world.bodies().DestroyBody(m_body);
}

void TestTrack::buildProvingGround() {
    auto& g = m_geometry;
    const int n = int(2.0f * kHalfSize / kGridStep);
    std::vector<float> heights((n + 1) * (n + 1));
    auto px = [&](int i) { return -kHalfSize + i * kGridStep; };
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i)
            heights[j * (n + 1) + i] = provingGroundHeight(px(i), px(j));
    auto P = [&](int i, int j) { return Vec3(px(i), heights[j * (n + 1) + i], px(j)); };

    g.positions.reserve(size_t(n) * n * 6 + 20000);
    g.surfaces.reserve(size_t(n) * n * 2 + 7000);
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const SurfaceId s = groundSurface(px(i) + 0.5f * kGridStep, px(j) + 0.5f * kGridStep);
            // CCW from above: (x0,z0) -> (x0,z1) -> (x1,z0) gives a +Y normal.
            g.addTriangle(P(i, j), P(i, j + 1), P(i + 1, j), s);
            g.addTriangle(P(i + 1, j), P(i, j + 1), P(i + 1, j + 1), s);
        }
    }

    // Concrete walls along the perimeter (behind the berm) keep the vehicle on the proving ground.
    const float wallT = 0.5f, wallTop = 5.0f, wallBottom = 0.5f;
    const float wy = 0.5f * (wallTop + wallBottom), whh = 0.5f * (wallTop - wallBottom);
    g.addBox(Vec3(kHalfSize + wallT, wy, 0.0f), Vec3(wallT, whh, kHalfSize + 2.0f * wallT), 0.0f, SurfaceId::Concrete);
    g.addBox(Vec3(-kHalfSize - wallT, wy, 0.0f), Vec3(wallT, whh, kHalfSize + 2.0f * wallT), 0.0f, SurfaceId::Concrete);
    g.addBox(Vec3(0.0f, wy, kHalfSize + wallT), Vec3(kHalfSize, whh, wallT), 0.0f, SurfaceId::Concrete);
    g.addBox(Vec3(0.0f, wy, -kHalfSize - wallT), Vec3(kHalfSize, whh, wallT), 0.0f, SurfaceId::Concrete);

    // Steps: 0.15, 0.30, 0.45 m (concrete), across lane C.
    const float stepX = 0.5f * (kLaneC0 + kLaneC1);
    const float stepH[] = {0.15f, 0.30f, 0.45f};
    const float stepZ[] = {-4.0f, 3.0f, 10.0f};
    for (int k = 0; k < 3; ++k)
        g.addBox(Vec3(stepX, 0.5f * stepH[k] - 0.05f, stepZ[k]), Vec3(4.0f, 0.5f * stepH[k] + 0.05f, 1.5f), 0.0f,
                 SurfaceId::Concrete);

    // Rocky section: rocks of different sizes, partially sunk into the ground.
    Lcg rng{12345u};
    for (int k = 0; k < 28; ++k) {
        const float r = rng.range(0.25f, 0.75f);
        const float x = rng.range(kLaneC0 + 2.0f, kLaneC1 - 2.0f);
        const float z = rng.range(18.0f, 40.0f);
        const Vec3 radii(r * rng.range(0.9f, 1.4f), r * rng.range(0.6f, 0.9f), r * rng.range(0.9f, 1.4f));
        g.addRock(Vec3(x, provingGroundHeight(x, z) - 0.25f * radii.GetY(), z), radii, rng.range(0.0f, 2.0f * kPi),
                  1000u + k, SurfaceId::Rock);
    }
    // A couple of large boulders to drive over with one wheel.
    g.addRock(Vec3(-3.0f, -0.35f, 21.5f), Vec3(0.9f, 0.7f, 1.0f), 0.4f, 7u, SurfaceId::Rock);
    g.addRock(Vec3(stepX - 2.0f, -0.3f, 15.5f), Vec3(1.1f, 0.8f, 1.0f), 1.1f, 11u, SurfaceId::Rock);

    auto feature = [&](const char* name, float x, float z) {
        m_features.push_back({name, Vec3(x, provingGroundHeight(x, z), z), 0.0f});
    };
    feature("Start pad (concrete)", 0.0f, -40.0f);
    feature("Bump sequence", 0.0f, -29.0f);
    feature("Diagonal twister (wheel lift)", 0.0f, -7.0f);
    feature("Low grip patch", 0.0f, 19.0f);
    feature("Hills 12/20/28/35 deg", -26.0f, -44.0f);
    feature("Side slope 18 deg", 25.0f, -44.0f);
    // z = -7.5: past the side slope run-out (it fades out at z = -8), otherwise the vehicle rolls off ~20 degrees.
    feature("Steps 0.15/0.30/0.45 m", stepX, -7.5f);
    feature("Rock field", stepX, 14.0f);
}

void TestTrack::buildFlat(bool withPit) {
    auto& g = m_geometry;
    const float h = 250.0f; // large pad: acceleration tests must not reach the edge
    auto rect = [&](float x0, float x1, float z0, float z1) {
        g.addQuad(Vec3(x0, 0, z0), Vec3(x0, 0, z1), Vec3(x1, 0, z1), Vec3(x1, 0, z0), SurfaceId::Concrete);
    };
    if (!withPit) {
        rect(-h, h, -h, h);
    } else {
        // Pit under the front left wheel of a vehicle standing at (0,0,0) facing +Z
        // (wheel 0: x = +0.825, z = +1.4).
        const float px0 = 0.45f, px1 = 1.65f, pz0 = 0.75f, pz1 = 2.05f;
        rect(-h, px0, -h, h);
        rect(px1, h, -h, h);
        rect(px0, px1, -h, pz0);
        rect(px0, px1, pz1, h);
    }
    m_features.push_back({"Start", Vec3(0, 0, 0), 0.0f});
}

} // namespace tf
