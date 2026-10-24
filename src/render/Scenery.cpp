#include "render/Scenery.h"

#include "render/GroundMaps.h"
#include "track/TestTrack.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace tf {
namespace {

constexpr float kPi = 3.14159265358979f;

float smooth(float e0, float e1, float x) {
    const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

struct OuterRect {
    float x0, x1, z0, z1;
};

OuterRect wallRect() {
    const TrackBounds b = TestTrack::bounds();
    return {b.xMin - 1.0f, b.xMax + 1.0f, b.zMin - 1.0f, b.zMax + 1.0f};
}

float outsideDistance(float x, float z) {
    const OuterRect r = wallRect();
    const float dx = std::max(std::max(r.x0 - x, x - r.x1), 0.0f);
    const float dz = std::max(std::max(r.z0 - z, z - r.z1), 0.0f);
    return std::sqrt(dx * dx + dz * dz);
}

float outerHeight(float x, float z) {
    const float d = outsideDistance(x, z);
    // The perimeter wall (top at 5 m) reads as a retaining wall: the embankment behind it starts just below its top.
    float h = 4.85f + smooth(0.0f, 30.0f, d) * (-1.0f + 9.0f * fbm2(x * 0.012f, z * 0.012f, 4, 91u));
    const float ridge = 1.0f - std::abs(2.0f * fbm2(x * 0.0045f + 3.0f, z * 0.0045f, 4, 92u) - 1.0f);
    h += smooth(110.0f, 360.0f, d) * 85.0f * ridge * ridge;
    return h;
}

glm::vec3 outerNormal(float x, float z) {
    const float e = 1.5f;
    const float hx = outerHeight(x + e, z) - outerHeight(x - e, z);
    const float hz = outerHeight(x, z + e) - outerHeight(x, z - e);
    return glm::normalize(glm::vec3(-hx, 2.0f * e, -hz));
}

std::vector<float> axis(std::initializer_list<std::pair<float, float>> ranges, float step) {
    std::vector<float> v;
    for (const auto& r : ranges) {
        const int n = std::max(1, int(std::ceil((r.second - r.first) / step)));
        for (int i = 0; i < n; ++i) v.push_back(r.first + (r.second - r.first) * float(i) / float(n));
    }
    v.push_back(ranges.end()[-1].second);
    return v;
}

std::uint32_t lcg(std::uint32_t& s) {
    s = s * 1664525u + 1013904223u;
    return s;
}
float rnd(std::uint32_t& s) { return float(lcg(s) >> 8) / float(1u << 24); }

// Branch tier of a pine: a star-shaped cone (alternating long/short branch tips that droop at the ends).
void addCone(MeshData& m, const glm::vec3& base, float radius, float height, int seg, const Material& mat,
             std::uint32_t& seed) {
    const glm::vec3 apex = base + glm::vec3(0, height, 0);
    const float slope = radius / height;
    const float twist = rnd(seed) * 2.0f * kPi;
    auto tip = [&](int i) {
        const float a = twist + 2.0f * kPi * float(i) / float(seg);
        const float r = radius * ((i % 2 == 0) ? 1.0f : 0.72f);
        const float droop = (i % 2 == 0) ? -0.12f * height : 0.05f * height;
        return std::pair<glm::vec3, glm::vec3>(base + glm::vec3(std::cos(a) * r, droop, std::sin(a) * r),
                                               glm::normalize(glm::vec3(std::cos(a), slope, std::sin(a))));
    };
    const glm::vec3 under = base + glm::vec3(0, 0.3f * height, 0);
    for (int i = 0; i < seg; ++i) {
        const auto [p0, n0] = tip(i);
        const auto [p1, n1] = tip((i + 1) % seg);
        const glm::vec3 na = glm::normalize(n0 + n1 + glm::vec3(0, 0.6f, 0));
        m.addTri(p0, apex, p1, n0, na, n1, mat);
        // Underside (branches seen from below).
        m.addTri(p1, under, p0, -n1 * 0.3f + glm::vec3(0, -0.7f, 0), glm::vec3(0, -1, 0), -n0 * 0.3f + glm::vec3(0, -0.7f, 0),
                 mat);
    }
}

} // namespace

MeshData buildSceneryMesh() {
    MeshData m;
    const OuterRect r = wallRect();
    const std::vector<float> xs = axis({{-420.0f, r.x0}, {r.x0, r.x1}, {r.x1, 420.0f}}, 7.0f);
    const std::vector<float> zs = axis({{-400.0f, r.z0}, {r.z0, r.z1}, {r.z1, 520.0f}}, 7.0f);
    Material ground{MatId::Grass, {0.20f, 0.26f, 0.09f}, 0.95f, 0.0f, 0.0f};
    for (std::size_t j = 0; j + 1 < zs.size(); ++j)
        for (std::size_t i = 0; i + 1 < xs.size(); ++i) {
            const float cx = 0.5f * (xs[i] + xs[i + 1]), cz = 0.5f * (zs[j] + zs[j + 1]);
            if (cx > r.x0 && cx < r.x1 && cz > r.z0 && cz < r.z1) continue; // inside the walls
            auto P = [&](float x, float z) { return glm::vec3(x, outerHeight(x, z), z); };
            const glm::vec3 a = P(xs[i], zs[j]), b = P(xs[i], zs[j + 1]), c = P(xs[i + 1], zs[j]), d = P(xs[i + 1], zs[j + 1]);
            m.addTri(a, b, c, outerNormal(a.x, a.z), outerNormal(b.x, b.z), outerNormal(c.x, c.z), ground);
            m.addTri(c, b, d, outerNormal(c.x, c.z), outerNormal(b.x, b.z), outerNormal(d.x, d.z), ground);
        }

    // Pine tree line.
    std::uint32_t seed = 2024u;
    int placed = 0;
    for (int attempt = 0; attempt < 30000 && placed < 420; ++attempt) {
        const float x = -260.0f + 520.0f * rnd(seed), z = -230.0f + 560.0f * rnd(seed);
        const float d = outsideDistance(x, z);
        const float density = smooth(2.5f, 9.0f, d) * (1.0f - smooth(110.0f, 190.0f, d)) *
                              smooth(0.38f, 0.62f, fbm2(x * 0.02f, z * 0.02f, 3, 93u));
        if (rnd(seed) > density) continue;
        const float h = 7.0f + 9.0f * rnd(seed);
        const float y = outerHeight(x, z) - 0.3f;
        const float g = rnd(seed);
        const Material bark{MatId::Generic, {0.10f, 0.07f, 0.045f}, 0.9f, 0.0f, 0.0f};
        const Material needles{MatId::Foliage, {0.030f + 0.02f * g, 0.060f + 0.03f * g, 0.025f + 0.01f * g}, 0.9f, 0.0f, 0.0f};
        m.addCylinder({x, y, z}, {x, y + 0.35f * h, z}, 0.12f + 0.02f * h * 0.1f, 7, bark, false);
        const int layers = 6;
        for (int k = 0; k < layers; ++k) {
            const float t = float(k) / float(layers);
            const float baseY = y + h * (0.16f + 0.68f * t);
            const float rad = (0.27f - 0.20f * t) * h * (0.85f + 0.3f * rnd(seed));
            addCone(m, {x, baseY, z}, rad, h * (0.30f - 0.10f * t), 12, needles, seed);
        }
        ++placed;
    }
    return m;
}

MeshData buildWaterMesh() {
    MeshData m;
    const WaterInfo w = TestTrack::water();
    const Material water{MatId::Generic, {0.0f, 0.0f, 0.0f}, 0.05f, 0.0f, 0.0f};
    const float pad = 0.5f;
    const glm::vec3 a(w.x0 - pad, w.level, w.z0 - pad), b(w.x0 - pad, w.level, w.z1 + pad);
    const glm::vec3 c(w.x1 + pad, w.level, w.z1 + pad), d(w.x1 + pad, w.level, w.z0 - pad);
    m.addQuad(a, b, c, d, water); // CCW from above: +Y
    return m;
}

} // namespace tf
