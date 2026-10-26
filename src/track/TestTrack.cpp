#include "track/TestTrack.h"

#include "physics/PhysicsWorld.h"
#include "track/Level.h"
#include "track/MiniTrack.h"

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
// 100 x 100 m proving ground (z < 50) plus the showcase meadow to the north.
constexpr float kXMin = -50.0f, kXMax = 50.0f, kZMin = -50.0f, kZMax = 86.0f;
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
// Concrete skid strip along the east edge (flat ground between the side slope plateau and the berm).
// It runs north up to the sand area, so the end of the strip is a sand -> concrete boundary.
constexpr float kSkidX0 = 39.0f, kSkidX1 = 45.0f, kSkidZ0 = -48.0f, kSkidZ1 = 48.0f;
// Sand area in the east meadow (rigid, low mu), flat next to the strip, dunes further north.
constexpr float kSandX0 = 28.0f, kSandX1 = 45.0f, kSandZ0 = 48.0f, kSandZ1 = 84.0f;
// West concrete strip with a flat mud bath at its north end: a direct mud -> concrete boundary at z = kMudBathZ0.
constexpr float kWestX0 = -44.5f, kWestX1 = -38.5f, kWestZ0 = 4.0f, kMudBathZ0 = 34.0f, kMudBathZ1 = 46.0f;

// ---- Showcase (z > 46): a winding mud path with puddles through a grass meadow.
constexpr float kShowZ0 = 46.0f;    // blend from the proving ground starts here
constexpr float kShowZ1 = 52.0f;    // full showcase relief from here
constexpr float kWaterLevel = 0.0f; // every puddle shares this level (one planar reflection)

float pathCenterX(float z) { return 7.0f * std::sin((z - 50.0f) * 0.11f); }

struct Puddle {
    float x, z, rx, rz, depth; // ellipse radii, m; bottom depth below the water level, m
};

const Puddle* puddles(int& count) {
    static const Puddle p[] = {
        {pathCenterX(57.0f), 57.0f, 2.3f, 3.2f, 0.16f},
        {pathCenterX(66.5f) + 0.5f, 66.5f, 2.7f, 3.8f, 0.20f},
        {pathCenterX(76.0f) - 0.4f, 76.0f, 2.1f, 2.8f, 0.14f},
        {-12.5f, 70.0f, 4.2f, 3.3f, 0.24f}, // mud wallow beside the path
        {10.5f, 59.0f, 1.5f, 1.2f, 0.10f},  // rain pool in the grass
    };
    count = int(sizeof(p) / sizeof(p[0]));
    return p;
}

// Squared normalized ellipse distance to puddle k (1 at the ellipse border).
float puddleQ2(const Puddle& p, float x, float z) {
    const float dx = (x - p.x) / p.rx, dz = (z - p.z) / p.rz;
    return dx * dx + dz * dz;
}

float showcaseRelief(float x, float z) {
    // Gentle meadow undulation, always >= ~0.12 m so that only the puddle bowls go below the water level.
    float h = 0.12f + 0.11f * (1.0f + std::sin(0.23f * x + 0.7f) * std::cos(0.19f * z - 0.4f));
    h += 0.05f * (1.0f + std::sin(0.61f * x + 0.37f * z + 1.3f));
    // Grass hummocks.
    auto mound = [&](float cx, float cz, float r, float height) {
        const float d2 = ((x - cx) * (x - cx) + (z - cz) * (z - cz)) / (r * r);
        return d2 < 1.0f ? height * (1.0f - d2) * (1.0f - d2) : 0.0f;
    };
    h += mound(15.0f, 66.0f, 7.0f, 1.3f) + mound(-17.0f, 81.0f, 6.0f, 1.0f) + mound(19.0f, 80.0f, 5.0f, 0.7f);
    // The path is a shallow channel.
    h -= 0.10f * TestTrack::showcaseMudWeight(x, z);
    // Puddle bowls: min() with a paraboloid keeps the surface continuous.
    int n = 0;
    const Puddle* p = puddles(n);
    const float rim = 0.08f;
    for (int k = 0; k < n; ++k) h = std::min(h, -p[k].depth + (p[k].depth + rim) * puddleQ2(p[k], x, z));
    return h;
}

SurfaceId groundSurface(float x, float z) { return TestTrack::groundSurfaceAt(x, z); }

} // namespace

SurfaceId TestTrack::groundSurfaceAt(float x, float z) {
    if (insideMiniTrackArea(x, z)) return miniTrackSurface(x, z);
    if (x > kLaneA0 && x < kLaneA1 && z > kStartZ0 && z < kStartZ1) return SurfaceId::Concrete;
    if (x > kLaneA0 && x < kLaneA1 && z > kLowGripZ0 && z < kLowGripZ1) return SurfaceId::LowGrip;
    if (x > kSkidX0 && x < kSkidX1 && z > kSkidZ0 && z < kSkidZ1) return SurfaceId::Concrete;
    if (x > kWestX0 && x < kWestX1 && z > kWestZ0 && z < kMudBathZ1)
        return z < kMudBathZ0 ? SurfaceId::Concrete : SurfaceId::Mud;
    if (x > kSandX0 && x < kSandX1 && z > kSandZ0 && z < kSandZ1) return SurfaceId::Sand;
    if (z > 47.0f) {
        if (showcaseMudWeight(x, z) > 0.45f) return SurfaceId::Mud;
        int n = 0;
        const Puddle* p = puddles(n);
        for (int k = 0; k < n; ++k)
            if (puddleQ2(p[k], x, z) < 1.25f) return SurfaceId::Mud;
        return SurfaceId::Grass;
    }
    return SurfaceId::PackedDirt;
}

float TestTrack::showcaseMudWeight(float x, float z) {
    if (z < kShowZ0) return 0.0f;
    const float path = 1.0f - smoothstep(2.4f, 3.8f, std::abs(x - pathCenterX(z)));
    const float wallowD = std::sqrt(((x + 12.5f) / 6.0f) * ((x + 12.5f) / 6.0f) + ((z - 70.0f) / 4.8f) * ((z - 70.0f) / 4.8f));
    const float wallow = 1.0f - smoothstep(0.7f, 1.0f, wallowD);
    return std::max(path, wallow) * window(z, 48.5f, 83.0f, 1.5f);
}

TrackBounds TestTrack::bounds() { return {kXMin, kXMax, kZMin, kZMax}; }

TrackBounds TestTrack::worldBounds() {
    const TrackBounds m = miniTrack().area;
    return {std::min(kXMin, m.xMin), std::max(kXMax, m.xMax), std::min(kZMin, m.zMin), std::max(kZMax, m.zMax)};
}

TrackBounds TestTrack::wheatPatch() { return {-46.0f, -28.0f, 56.0f, 76.0f}; }

DriveRoute TestTrack::attractRoute() {
    // A rounded rectangle on level ground, counterclockwise seen from above: east over the start pad, north between
    // lanes A and C (x = 12, clear of the bumps, the twister, the side slope and the rock field), west past the north
    // end of the low grip patch, south between lanes B and A (x = -13, clear of the climbs).
    const float x0 = -13.0f, x1 = 12.0f, z0 = -38.0f, z1 = 44.0f, r = 11.0f, step = 2.0f;
    DriveRoute route;
    route.name = "Proving ground loop";
    route.cruiseSpeed = 8.0f;
    auto line = [&](float ax, float az, float bx, float bz) {
        const int n = std::max(1, int(std::lround(std::hypot(bx - ax, bz - az) / step)));
        for (int k = 0; k < n; ++k) route.points.push_back({ax + (bx - ax) * float(k) / float(n), az + (bz - az) * float(k) / float(n)});
    };
    auto arc = [&](float cx, float cz, float a0) {
        const int n = std::max(1, int(std::lround(0.5f * kPi * r / step)));
        for (int k = 0; k < n; ++k) {
            const float a = a0 + 0.5f * kPi * float(k) / float(n);
            route.points.push_back({cx + r * std::cos(a), cz + r * std::sin(a)});
        }
    };
    line(0.0f, z0, x1 - r, z0);
    arc(x1 - r, z0 + r, -0.5f * kPi);
    line(x1, z0 + r, x1, z1 - r);
    arc(x1 - r, z1 - r, 0.0f);
    line(x1 - r, z1, x0 + r, z1);
    arc(x0 + r, z1 - r, 0.5f * kPi);
    line(x0, z1 - r, x0, z0 + r);
    arc(x0 + r, z0 + r, kPi);
    line(x0 + r, z0, 0.0f, z0);
    return route;
}

WaterInfo TestTrack::water() {
    int n = 0;
    const Puddle* p = puddles(n);
    WaterInfo w{kWaterLevel, 1e9f, 1e9f, -1e9f, -1e9f};
    for (int k = 0; k < n; ++k) {
        w.x0 = std::min(w.x0, p[k].x - p[k].rx);
        w.x1 = std::max(w.x1, p[k].x + p[k].rx);
        w.z0 = std::min(w.z0, p[k].z - p[k].rz);
        w.z1 = std::max(w.z1, p[k].z + p[k].rz);
    }
    return w;
}

// ------------------------------------------------------------------ TrackGeometry

void TrackGeometry::addTriangle(Vec3Arg a, Vec3Arg b, Vec3Arg c, SurfaceId s) {
    positions.emplace_back(a.GetX(), a.GetY(), a.GetZ());
    positions.emplace_back(b.GetX(), b.GetY(), b.GetZ());
    positions.emplace_back(c.GetX(), c.GetY(), c.GetZ());
    surfaces.push_back(s);
}

void TrackGeometry::paintFrom(std::size_t firstTriangle, Paint p) {
    paints.resize(triangleCount(), Paint::None);
    std::fill(paints.begin() + std::ptrdiff_t(std::min(firstTriangle, paints.size())), paints.end(), p);
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
    // Chunky faceted shape: the unit sphere is cut by random planes (a convex polytope), and every vertex is pulled
    // in along its direction to the nearest plane. Vertices on the same plane form one flat facet. The radius depends
    // only on the direction, so coincident vertices of the subdivision stay identical (no cracks).
    Lcg prng{seed * 2654435761u + 17u};
    struct Cut {
        Vec3 n;
        float d;
    };
    std::vector<Cut> cuts;
    cuts.push_back({Vec3(0, 1, 0), prng.range(0.62f, 0.74f)}); // flat-ish top
    for (int k = 0; k < 12; ++k) {
        // Spread the cut normals around the rock (golden angle) so no side is left round.
        const float a = float(k) * 2.39996f + prng.range(-0.4f, 0.4f), y = prng.range(-0.30f, 0.80f);
        const float rxz = std::sqrt(std::max(0.0f, 1.0f - y * y));
        const Vec3 n(rxz * std::cos(a), y, rxz * std::sin(a));
        cuts.push_back({n.Normalized(), prng.range(0.64f, 0.80f)});
    }
    auto warp = [&](Vec3 d) {
        float r = 1.0f;
        for (const Cut& c : cuts) {
            const float cosA = d.Dot(c.n);
            if (cosA > 1e-3f) r = std::min(r, c.d / cosA);
        }
        r = std::max(r, 0.55f);
        return center + q * (d * radii * r);
    };
    for (auto& tri : f2)
        addTriangle(warp(v[tri[0]]), warp(v[tri[1]]), warp(v[tri[2]]), s);
}

// ------------------------------------------------------------------ TestTrack

float TestTrack::provingGroundHeight(float x, float z) {
    if (insideMiniTrackArea(x, z)) return miniTrackGround(x, z);
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
    // Showcase meadow to the north (blends in over z = 46..52).
    if (z > kShowZ0) h += smoothstep(kShowZ0, kShowZ1, z) * showcaseRelief(x, z);
    // Dunes in the north part of the sand area: crossing ridges up to ~0.9 m, flat toward the skid strip.
    {
        const float w = window(x, kSandX0 + 2.0f, kSandX1 - 3.0f, 2.0f) * window(z, 62.0f, kSandZ1 - 5.0f, 8.0f);
        if (w > 0.0f) {
            const float ridge = 0.5f + 0.5f * std::sin(0.42f * z + 0.35f * x + 0.25f * std::sin(0.5f * x));
            h += w * (0.75f * ridge * ridge + 0.15f * (1.0f + std::sin(0.9f * x - 0.3f * z)));
        }
    }
    // Berm around the perimeter.
    {
        const float d = std::min(std::min(x - kXMin, kXMax - x), std::min(z - kZMin, kZMax - z));
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
    case TrackLayout::SurfaceLanes: buildSurfaceLanes(); break;
    case TrackLayout::SurfaceRamps: buildSurfaceRamps(); break;
    case TrackLayout::RoadLanes: buildRoadLanes(); break;
    case TrackLayout::SlopeWithWheelPit: buildSlopeWithWheelPit(); break;
    case TrackLayout::Level: throw std::runtime_error("TestTrack: the Level layout needs a Level");
    }
    createBody(surfaces);
}

TestTrack::TestTrack(PhysicsWorld& world, const SurfaceTable& surfaces, const Level& level)
    : m_world(world), m_layout(TrackLayout::Level) {
    m_geometry = level.props().geometry;
    m_features = level.features();
    createBody(surfaces);
}

void TestTrack::createBody(const SurfaceTable& surfaces) {
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
    m_body = m_world.bodies().CreateAndAddBody(bcs, JPH::EActivation::DontActivate);
    m_world.optimizeBroadPhase();
}

TestTrack::~TestTrack() {
    m_world.bodies().RemoveBody(m_body);
    m_world.bodies().DestroyBody(m_body);
}

void TestTrack::buildProvingGround() {
    auto& g = m_geometry;
    const int nx = int((kXMax - kXMin) / kGridStep);
    const int nz = int((kZMax - kZMin) / kGridStep);
    std::vector<float> heights(size_t(nx + 1) * (nz + 1));
    auto px = [&](int i) { return kXMin + i * kGridStep; };
    auto pz = [&](int j) { return kZMin + j * kGridStep; };
    for (int j = 0; j <= nz; ++j)
        for (int i = 0; i <= nx; ++i)
            heights[j * (nx + 1) + i] = provingGroundHeight(px(i), pz(j));
    auto P = [&](int i, int j) { return Vec3(px(i), heights[j * (nx + 1) + i], pz(j)); };

    g.positions.reserve(size_t(nx) * nz * 6 + 30000);
    g.surfaces.reserve(size_t(nx) * nz * 2 + 10000);
    for (int j = 0; j < nz; ++j) {
        for (int i = 0; i < nx; ++i) {
            const SurfaceId s = groundSurface(px(i) + 0.5f * kGridStep, pz(j) + 0.5f * kGridStep);
            // CCW from above: (x0,z0) -> (x0,z1) -> (x1,z0) gives a +Y normal.
            g.addTriangle(P(i, j), P(i, j + 1), P(i + 1, j), s);
            g.addTriangle(P(i + 1, j), P(i, j + 1), P(i + 1, j + 1), s);
        }
    }

    // Concrete walls along the perimeter (behind the berm) keep the vehicle on the proving ground.
    const float wallT = 0.5f, wallTop = 5.0f, wallBottom = 0.5f;
    const float wy = 0.5f * (wallTop + wallBottom), whh = 0.5f * (wallTop - wallBottom);
    const float cx = 0.5f * (kXMin + kXMax), cz = 0.5f * (kZMin + kZMax);
    const float hx = 0.5f * (kXMax - kXMin), hz = 0.5f * (kZMax - kZMin);
    g.addBox(Vec3(kXMax + wallT, wy, cz), Vec3(wallT, whh, hz + 2.0f * wallT), 0.0f, SurfaceId::Concrete);
    g.addBox(Vec3(kXMin - wallT, wy, cz), Vec3(wallT, whh, hz + 2.0f * wallT), 0.0f, SurfaceId::Concrete);
    g.addBox(Vec3(cx, wy, kZMax + wallT), Vec3(hx, whh, wallT), 0.0f, SurfaceId::Concrete);
    g.addBox(Vec3(cx, wy, kZMin - wallT), Vec3(hx, whh, wallT), 0.0f, SurfaceId::Concrete);

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
        m_rocks.push_back({x, z, std::max(radii.GetX(), radii.GetZ())});
    }
    // A couple of large boulders to drive over with one wheel.
    g.addRock(Vec3(-3.0f, -0.35f, 21.5f), Vec3(0.9f, 0.7f, 1.0f), 0.4f, 7u, SurfaceId::Rock);
    g.addRock(Vec3(stepX - 2.0f, -0.3f, 15.5f), Vec3(1.1f, 0.8f, 1.0f), 1.1f, 11u, SurfaceId::Rock);
    m_rocks.push_back({-3.0f, 21.5f, 1.0f});
    m_rocks.push_back({stepX - 2.0f, 15.5f, 1.1f});

    // Showcase rocks: in the meadow beside the mud path (a separate generator keeps the rock field unchanged).
    {
        Lcg srng{777u};
        const float spots[][2] = {{-6.5f, 54.0f}, {12.0f, 62.5f}, {-4.0f, 61.0f}, {-19.0f, 63.0f}, {-1.0f, 73.0f},
                                  {13.5f, 74.0f}, {-7.5f, 79.0f}, {6.0f, 82.0f}, {-21.0f, 74.5f}, {22.0f, 58.0f}};
        int k = 0;
        for (const auto& s : spots) {
            const float r = srng.range(0.35f, 0.9f);
            const Vec3 radii(r * srng.range(1.0f, 1.5f), r * srng.range(0.55f, 0.85f), r * srng.range(0.9f, 1.4f));
            g.addRock(Vec3(s[0], provingGroundHeight(s[0], s[1]) - 0.3f * radii.GetY(), s[1]), radii,
                      srng.range(0.0f, 2.0f * kPi), 3000u + k++, SurfaceId::Rock);
            m_rocks.push_back({s[0], s[1], std::max(radii.GetX(), radii.GetZ())});
        }
        // Two boulders that frame the path.
        const float bx = pathCenterX(70.5f) + 4.2f;
        g.addRock(Vec3(bx, provingGroundHeight(bx, 70.5f) - 0.4f, 70.5f), Vec3(1.5f, 1.1f, 1.3f), 0.7f, 41u, SurfaceId::Rock);
        m_rocks.push_back({bx, 70.5f, 1.5f});
        g.addRock(Vec3(-3.5f, provingGroundHeight(-3.5f, 67.0f) - 0.35f, 67.0f), Vec3(1.1f, 0.9f, 1.2f), 2.1f, 43u,
                  SurfaceId::Rock);
        m_rocks.push_back({-3.5f, 67.0f, 1.2f});
    }

    // Mini handling track in its own walled area south of the proving ground (teleport only).
    buildMiniTrack(g);

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
    feature("Mud & puddles showcase", 0.0f, 48.5f);
    feature("Skid strip (concrete)", 0.5f * (kSkidX0 + kSkidX1), -46.0f);
    feature("Sand area (dunes)", 36.0f, 52.0f);
    // Facing south (-Z) over the boundary onto concrete.
    m_features.push_back({"Sand to concrete", Vec3(42.0f, provingGroundHeight(42.0f, 60.0f), 60.0f), kPi});
    m_features.push_back({"Mud bath to concrete", Vec3(0.5f * (kWestX0 + kWestX1), 0.0f, kMudBathZ1 - 1.5f), kPi});
    feature("Meadow (grass)", 19.0f, 50.0f);
    feature("Puddle (splash)", pathCenterX(57.0f), 49.5f); // the first path puddle is 4-5 m ahead
    feature("Wheat patch (visual)", -37.0f, 52.5f);           // the patch starts 3.5 m ahead
    m_features.push_back(miniTrack().start);                  // grid spot 12 m behind the start/finish line
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
        // (wheel 0: x = +0.9, z = +1.4).
        const float px0 = 0.45f, px1 = 1.65f, pz0 = 0.75f, pz1 = 2.05f;
        rect(-h, px0, -h, h);
        rect(px1, h, -h, h);
        rect(px0, px1, -h, pz0);
        rect(px0, px1, pz1, h);
    }
    m_features.push_back({"Start", Vec3(0, 0, 0), 0.0f});
}

void TestTrack::buildSlopeWithWheelPit() {
    // Concrete plane rising along +Z at 15 degrees, with a bottomless 1.2 x 3.3 m hole under the front-left wheel of
    // a vehicle at the origin facing uphill: the wheel hangs for the first ~2 m of the climb.
    auto& g = m_geometry;
    const float h = 60.0f, t = std::tan(deg(15.0f));
    auto rect = [&](float x0, float x1, float z0, float z1) {
        g.addQuad(Vec3(x0, z0 * t, z0), Vec3(x0, z1 * t, z1), Vec3(x1, z1 * t, z1), Vec3(x1, z0 * t, z0),
                  SurfaceId::Concrete);
    };
    const float px0 = 0.45f, px1 = 1.65f, pz0 = 0.75f, pz1 = 4.05f;
    rect(-h, px0, -h, h);
    rect(px1, h, -h, h);
    rect(px0, px1, -h, pz0);
    rect(px0, px1, pz1, h);
    m_features.push_back({"Start", Vec3(0, 0, 0), 0.0f});
}

void TestTrack::buildSurfaceLanes() {
    // Lanes along Z, 12 m wide, centered at x = 0, 12, ..., 72; spawn points at the start of each lane. 1.1 km long
    // so that a run-up to 100 km/h on mud and the stop fit (trailforge_probe abs).
    auto& g = m_geometry;
    const SurfaceId lanes[] = {SurfaceId::Concrete, SurfaceId::PackedDirt, SurfaceId::Mud,    SurfaceId::Grass,
                               SurfaceId::Rock,     SurfaceId::LowGrip,    SurfaceId::Sand};
    const char* names[] = {"Concrete lane", "Packed dirt lane", "Mud lane", "Grass lane", "Rock lane", "Low grip lane",
                           "Sand lane"};
    const float z0 = -100.0f, z1 = 200.0f;
    for (int k = 0; k < 7; ++k) {
        const float x0 = -6.0f + 12.0f * float(k), x1 = x0 + 12.0f;
        g.addQuad(Vec3(x0, 0, z0), Vec3(x0, 0, z1), Vec3(x1, 0, z1), Vec3(x1, 0, z0), lanes[k]);
        m_features.push_back({names[k], Vec3(x0 + 6.0f, 0, -90.0f), 0.0f});
    }
    // Extension to z = 1000 as separate quads, so the contacts on the first 300 m stay those of the original lanes.
    const float z2 = 1000.0f;
    for (int k = 0; k < 7; ++k) {
        const float x0 = -6.0f + 12.0f * float(k), x1 = x0 + 12.0f;
        g.addQuad(Vec3(x0, 0, z1), Vec3(x0, 0, z2), Vec3(x1, 0, z2), Vec3(x1, 0, z1), lanes[k]);
    }
}

void TestTrack::buildRoadLanes() {
    // Same lanes as buildSurfaceLanes (12 m wide, z -100..1000), for the road surfaces: asphalt, concrete, gravel.
    // A layout of its own: more triangles in the SurfaceLanes mesh would change the contacts measured there.
    auto& g = m_geometry;
    const SurfaceId lanes[] = {SurfaceId::Asphalt, SurfaceId::Concrete, SurfaceId::Gravel};
    const char* names[] = {"Asphalt lane", "Concrete lane", "Gravel lane"};
    for (int k = 0; k < 3; ++k) {
        const float x0 = -6.0f + 12.0f * float(k), x1 = x0 + 12.0f;
        g.addQuad(Vec3(x0, 0, -100.0f), Vec3(x0, 0, 1000.0f), Vec3(x1, 0, 1000.0f), Vec3(x1, 0, -100.0f), lanes[k]);
        m_features.push_back({names[k], Vec3(x0 + 6.0f, 0, -90.0f), 0.0f});
    }
}

void TestTrack::buildSurfaceRamps() {
    // Lane 2*s + a: surface s, slope 15 (a = 0) or 20 (a = 1) degrees. 20 m flat run-up, a 22 m ramp from z = 0,
    // a 10 m plateau. Spawn 6 m before the ramp. Lanes are 10 m wide and have no side walls.
    auto& g = m_geometry;
    const SurfaceId surfaces[] = {SurfaceId::Concrete, SurfaceId::PackedDirt, SurfaceId::Rock,
                                  SurfaceId::Grass,    SurfaceId::Mud,        SurfaceId::LowGrip};
    const float slopes[] = {15.0f, 20.0f};
    const float zFlat = -20.0f, zRamp = 22.0f, zPlateau = 10.0f;
    int k = 0;
    for (SurfaceId s : surfaces) {
        for (float slope : slopes) {
            const float x0 = -5.0f + 10.0f * float(k), x1 = x0 + 10.0f;
            const float h = zRamp * std::tan(deg(slope));
            g.addQuad(Vec3(x0, 0, zFlat), Vec3(x0, 0, 0), Vec3(x1, 0, 0), Vec3(x1, 0, zFlat), s);
            g.addQuad(Vec3(x0, 0, 0), Vec3(x0, h, zRamp), Vec3(x1, h, zRamp), Vec3(x1, 0, 0), s);
            g.addQuad(Vec3(x0, h, zRamp), Vec3(x0, h, zRamp + zPlateau), Vec3(x1, h, zRamp + zPlateau),
                      Vec3(x1, h, zRamp), s);
            char name[64];
            std::snprintf(name, sizeof(name), "Ramp %d deg, surface %d", int(slope), int(s));
            m_features.push_back({name, Vec3(x0 + 5.0f, 0, -6.0f), 0.0f});
            ++k;
        }
    }
}

} // namespace tf
