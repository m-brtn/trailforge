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

// 1 inside the view corridor north of the showcase (toward the sunset with the default sun heading), 0 elsewhere.
// It keeps the line of sight from the meadow to the distant mountains open.
float viewCorridor(float x, float z) {
    const OuterRect r = wallRect();
    const float ang = std::abs(std::atan2(x, z - 60.0f));
    return smooth(r.z1 - 10.0f, r.z1 + 4.0f, z) * (1.0f - smooth(0.42f, 0.75f, ang));
}

float outerHeight(float x, float z) {
    const float d = outsideDistance(x, z);
    // The perimeter wall (top at 5 m) reads as a retaining wall: the embankment behind it starts just below its top.
    float h = 4.85f + smooth(0.0f, 30.0f, d) * (-1.0f + 9.0f * fbm2(x * 0.012f, z * 0.012f, 4, 91u));
    const float ridge = 1.0f - std::abs(2.0f * fbm2(x * 0.0045f + 3.0f, z * 0.0045f, 4, 92u) - 1.0f);
    const float corridor = viewCorridor(x, z);
    // Foothills: lower than before so the layered mountains in the sky pass show above them.
    h += smooth(110.0f, 360.0f, d) * 46.0f * ridge * ridge * (1.0f - 0.85f * corridor);
    // A shallow valley inside the corridor that rises again to a low rim at the far edge of the backdrop.
    h -= corridor * smooth(15.0f, 140.0f, d) * 12.0f * (1.0f - smooth(300.0f, 460.0f, d));
    h += corridor * smooth(330.0f, 460.0f, d) * 5.0f;
    return h;
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

// One branch tier of a stylized pine (unit tree height): a faceted cone skirt with alternating long/short branch tips
// that droop at the ends, plus an underside. Flat-shaded, so every face reads as a facet.
void addTier(MeshData& m, float baseY, float radius, float height, int seg, float twist, const Material& mat,
             bool underside) {
    const glm::vec3 apex(0.0f, baseY + height, 0.0f);
    const glm::vec3 under(0.0f, baseY + 0.25f * height, 0.0f);
    const glm::vec3 inside(0.0f, baseY + 0.45f * height, 0.0f);
    auto tip = [&](int i) {
        const float a = twist + 2.0f * kPi * float(i) / float(seg);
        const bool lng = (i % 2 == 0);
        const float r = radius * (lng ? 1.0f : 0.70f);
        const float droop = lng ? -0.16f * height : 0.06f * height;
        return glm::vec3(std::cos(a) * r, baseY + droop, std::sin(a) * r);
    };
    for (int i = 0; i < seg; ++i) {
        const glm::vec3 p0 = tip(i), p1 = tip((i + 1) % seg);
        m.addTriangleOutward(p0, apex, p1, inside, mat);
        if (underside) {
            Material dark = mat;
            dark.color *= 0.55f;
            m.addTriangleOutward(p0, p1, under, apex, dark);
        }
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
            // Flat-shaded: the backdrop reads as faceted low-poly hills.
            m.addTriangle(a, b, c, ground);
            m.addTriangle(c, b, d, ground);
        }
    return m;
}

std::array<MeshData, 4> buildPineMeshes() {
    const Material bark{MatId::Generic, {0.10f, 0.07f, 0.045f}, 0.9f, 0.0f, 0.0f};
    auto needles = [](float t) {
        // Lower tiers are darker (self-occlusion), upper tiers catch more light.
        return Material{MatId::Foliage, glm::vec3(0.050f, 0.090f, 0.026f) * (0.70f + 0.45f * t), 0.9f, 0.0f, 0.0f};
    };
    struct Lod {
        int tiers, seg;
        bool trunk, underside;
    };
    const Lod lods[3] = {{6, 8, true, true}, {4, 6, true, false}, {2, 5, false, false}};
    std::array<MeshData, 4> out;
    for (int l = 0; l < 3; ++l) {
        MeshData& m = out[l];
        const Lod& lod = lods[l];
        if (lod.trunk) m.addCylinder({0, -0.02f, 0}, {0, 0.32f, 0}, 0.022f, l == 0 ? 6 : 4, bark, false);
        for (int k = 0; k < lod.tiers; ++k) {
            const float t = float(k) / float(lod.tiers);
            // The same silhouette at every LOD: tiers span 0.14..0.84 of the height, the top tier reaches 1.0.
            const float baseY = 0.14f + 0.70f * t;
            const float radius = (0.30f - 0.22f * t) * (lod.tiers == 2 ? 1.05f : 1.0f);
            const bool top = k + 1 == lod.tiers;
            const float height = top ? 1.0f - baseY : (0.32f - 0.12f * t) * 4.5f / float(lod.tiers);
            addTier(m, baseY, radius, height, lod.seg, 0.9f * float(k), needles(t), lod.underside);
        }
    }
    // Cards: the cone outline 0.14..1.0 high, radius 0.30, in two vertical planes; normals lean outward and up so
    // the far canopy shades like the cones.
    MeshData& card = out[3];
    for (int k = 0; k < 2; ++k) {
        const float a = 0.5f * kPi * float(k) + 0.4f;
        const glm::vec3 side(std::cos(a), 0.0f, std::sin(a)), across(-side.z, 0.0f, side.x);
        const glm::vec3 l = 0.30f * side + glm::vec3(0, 0.14f, 0), r = -0.30f * side + glm::vec3(0, 0.14f, 0);
        const glm::vec3 apex(0.0f, 1.0f, 0.0f);
        for (float f : {1.0f, -1.0f}) {
            const glm::vec3 n = glm::normalize(f * across + glm::vec3(0, 0.8f, 0));
            const Material mat = needles(0.4f);
            if (f > 0.0f) card.addTri(l, r, apex, n, n, n, mat);
            else card.addTri(r, l, apex, n, n, n, mat);
        }
    }
    return out;
}

std::vector<TreeInstance> buildTreeInstances() {
    std::vector<TreeInstance> trees;
    std::uint32_t seed = 2024u;
    for (int attempt = 0; attempt < 60000 && trees.size() < 900; ++attempt) {
        const float x = -280.0f + 560.0f * rnd(seed), z = -250.0f + 600.0f * rnd(seed);
        const float d = outsideDistance(x, z);
        const float density = smooth(2.5f, 9.0f, d) * (1.0f - smooth(120.0f, 210.0f, d)) *
                              smooth(0.36f, 0.60f, fbm2(x * 0.02f, z * 0.02f, 3, 93u)) *
                              (1.0f - 0.92f * viewCorridor(x, z));
        if (rnd(seed) > density) continue;
        TreeInstance t;
        const float h = 7.0f + 9.5f * rnd(seed);
        t.baseHeight = glm::vec4(x, outerHeight(x, z) - 0.3f, z, h);
        t.params = glm::vec4(rnd(seed) * 2.0f * kPi, rnd(seed), 0.85f + 0.3f * rnd(seed), 0.0f);
        trees.push_back(t);
    }
    return trees;
}

MeshData buildWaterMesh() {
    MeshData m;
    const WaterInfo w = TestTrack::water();
    // water_fs: color = muddy body, roughness slot = depth (m) at which it is opaque, metallic slot = opacity.
    const Material water{MatId::Generic, {0.055f, 0.040f, 0.026f}, 0.10f, 0.88f, 0.0f};
    const float pad = 0.5f;
    const glm::vec3 a(w.x0 - pad, w.level, w.z0 - pad), b(w.x0 - pad, w.level, w.z1 + pad);
    const glm::vec3 c(w.x1 + pad, w.level, w.z1 + pad), d(w.x1 + pad, w.level, w.z0 - pad);
    m.addQuad(a, b, c, d, water); // CCW from above: +Y
    return m;
}

} // namespace tf
