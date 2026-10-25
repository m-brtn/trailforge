#include "render/LevelScene.h"

#include "track/Level.h"

#include <algorithm>
#include <cmath>

namespace tf {
namespace {

Material styleMaterial(PropStyle s) {
    switch (s) {
    case PropStyle::Wood: return {MatId::Generic, {0.30f, 0.20f, 0.12f}, 0.80f, 0.0f, 0.4f};
    case PropStyle::DarkWood: return {MatId::Generic, {0.14f, 0.09f, 0.06f}, 0.85f, 0.0f, 0.4f};
    // Masonry is plain: the procedural rock material projects along Y and smears on walls.
    case PropStyle::Stone: return {MatId::Generic, {0.40f, 0.39f, 0.37f}, 0.90f, 0.0f, 0.3f};
    case PropStyle::DarkStone: return {MatId::Generic, {0.19f, 0.18f, 0.17f}, 0.90f, 0.0f, 0.3f};
    case PropStyle::Plaster: return {MatId::Generic, {0.62f, 0.58f, 0.50f}, 0.85f, 0.0f, 0.2f};
    case PropStyle::RoofRed: return {MatId::Generic, {0.36f, 0.10f, 0.06f}, 0.70f, 0.0f, 0.0f};
    case PropStyle::RoofDark: return {MatId::Generic, {0.10f, 0.10f, 0.11f}, 0.65f, 0.0f, 0.0f};
    case PropStyle::Metal: return {MatId::Generic, {0.45f, 0.46f, 0.47f}, 0.45f, 0.8f, 0.2f};
    case PropStyle::Rust: return {MatId::Generic, {0.33f, 0.15f, 0.07f}, 0.80f, 0.3f, 0.3f};
    case PropStyle::Canvas: return {MatId::Generic, {0.40f, 0.36f, 0.26f}, 0.90f, 0.0f, 0.2f};
    case PropStyle::Marker: return {MatId::Paint, {0.80f, 0.25f, 0.03f}, 0.45f, 0.0f, 0.0f};
    case PropStyle::Beacon: return {MatId::Amber, {1.0f, 0.35f, 0.05f}, 0.5f, 0.0f, 0.0f};
    case PropStyle::Count: break;
    }
    return {};
}

glm::vec3 vec(const JPH::Float3& f) { return {f.x, f.y, f.z}; }

void addFlat(MeshData& m, const TrackGeometry& g, std::size_t first, std::size_t last, const Material& mat) {
    for (std::size_t t = first; t < last; ++t) {
        const glm::vec3 a = vec(g.positions[3 * t]), b = vec(g.positions[3 * t + 1]), c = vec(g.positions[3 * t + 2]);
        const glm::vec3 n = glm::cross(b - a, c - a);
        const float l = glm::length(n);
        if (l < 1e-9f) continue;
        m.addTri(a, b, c, n / l, n / l, n / l, mat);
    }
}

struct WaterLook {
    glm::vec3 color;
    float opaqueDepth, opacity;
};

WaterLook waterLook(WaterKind k) {
    switch (k) {
    case WaterKind::Sea: return {{0.012f, 0.040f, 0.055f}, 7.0f, 0.95f};
    case WaterKind::Lake: return {{0.015f, 0.045f, 0.050f}, 5.0f, 0.93f};
    case WaterKind::River: return {{0.035f, 0.050f, 0.045f}, 1.6f, 0.88f};
    case WaterKind::None: break;
    }
    return {{0.03f, 0.04f, 0.04f}, 3.0f, 0.9f};
}

Material waterMaterial(WaterKind k) {
    const WaterLook w = waterLook(k);
    // water_fs: color = body, roughness slot = depth (m) at which it is opaque, metallic slot = opacity.
    return {MatId::Generic, w.color, w.opaqueDepth, w.opacity, 0.0f};
}

} // namespace

MeshData buildLevelPropsMesh(const Level& level) {
    MeshData m;
    const LevelProps& props = level.props();
    const TrackGeometry& g = props.geometry;
    std::size_t t = 0;
    while (t < g.triangleCount()) {
        const PropStyle style = t < props.styles.size() ? props.styles[t] : PropStyle::Wood;
        std::size_t e = t + 1;
        while (e < g.triangleCount() && e < props.styles.size() && props.styles[e] == style) ++e;
        addFlat(m, g, t, e, styleMaterial(style));
        t = e;
    }
    TrackGeometry rocks;
    for (const LevelRock& r : level.rocks()) {
        const std::size_t first = rocks.triangleCount();
        addLevelRock(rocks, r);
        Material mat = styleMaterial(r.style);
        mat.id = MatId::Rock;
        addFlat(m, rocks, first, rocks.triangleCount(), mat);
    }
    return m;
}

MeshData buildLevelWaterMesh(const Level& level) {
    MeshData m;
    const int n = level.samples();
    const float o = level.origin(), cell = level.cell();
    auto px = [&](int i) { return o + float(i) * cell; };
    // Water surface per sample, grown 3 samples into the dry neighbors (highest neighboring surface), so quad edges
    // end under the banks and water_fs fades the shoreline by depth instead of showing the block outline.
    const std::size_t total = std::size_t(n) * std::size_t(n);
    std::vector<float> surf(level.waterSurfaces().begin(), level.waterSurfaces().end());
    std::vector<WaterKind> kinds(total, WaterKind::None);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i)
            if (surf[std::size_t(j) * n + i] > 0.5f * Level::kNoWater) kinds[std::size_t(j) * n + i] = level.waterKindAt(px(i), px(j));
    for (int pass = 0; pass < 3; ++pass) {
        std::vector<float> next = surf;
        std::vector<WaterKind> nextKind = kinds;
        for (int j = 1; j < n - 1; ++j)
            for (int i = 1; i < n - 1; ++i) {
                const std::size_t k = std::size_t(j) * n + i;
                if (surf[k] > 0.5f * Level::kNoWater) continue;
                for (const std::size_t q : {k - 1, k + 1, k - std::size_t(n), k + std::size_t(n)})
                    if (surf[q] > next[k]) {
                        next[k] = surf[q];
                        nextKind[k] = kinds[q];
                    }
            }
        surf.swap(next);
        kinds.swap(nextKind);
    }
    auto wet = [&](int i, int j) { return surf[std::size_t(j) * n + i] > 0.5f * Level::kNoWater; };
    auto waterAt = [&](int i, int j) { return surf[std::size_t(j) * n + i]; };
    const glm::vec3 up(0.0f, 1.0f, 0.0f);

    // Flat if every wet sample of the block has the same surface; returns false when the block is dry.
    auto scan = [&](int i0, int j0, int size, float& lo, float& hi, WaterKind& kind) {
        lo = 1e30f;
        hi = -1e30f;
        kind = WaterKind::None;
        for (int j = j0; j <= std::min(j0 + size, n - 1); ++j)
            for (int i = i0; i <= std::min(i0 + size, n - 1); ++i) {
                if (!wet(i, j)) continue;
                const float w = waterAt(i, j);
                lo = std::min(lo, w);
                hi = std::max(hi, w);
                if (kind == WaterKind::None) kind = kinds[std::size_t(j) * n + i];
            }
        return hi >= lo;
    };
    auto flatQuad = [&](int i0, int j0, int size, float y, WaterKind kind) {
        const int i1 = std::min(i0 + size, n - 1), j1 = std::min(j0 + size, n - 1);
        const Material mat = waterMaterial(kind);
        // CCW from above (+Y): (x0, z0) -> (x0, z1) -> (x1, z1) -> (x1, z0).
        m.addQuad({px(i0), y, px(j0)}, {px(i0), y, px(j1)}, {px(i1), y, px(j1)}, {px(i1), y, px(j0)}, mat);
    };

    constexpr int kBig = 16, kSmall = 4;
    for (int bj = 0; bj < n - 1; bj += kBig)
        for (int bi = 0; bi < n - 1; bi += kBig) {
            float lo, hi;
            WaterKind kind;
            if (!scan(bi, bj, kBig, lo, hi, kind)) continue;
            if (hi - lo < 0.02f) {
                flatQuad(bi, bj, kBig, hi, kind);
                continue;
            }
            for (int sj = bj; sj < std::min(bj + kBig, n - 1); sj += kSmall)
                for (int si = bi; si < std::min(bi + kBig, n - 1); si += kSmall) {
                    if (!scan(si, sj, kSmall, lo, hi, kind)) continue;
                    if (hi - lo < 0.02f) {
                        flatQuad(si, sj, kSmall, hi, kind);
                        continue;
                    }
                    // Sloped (river) block: corner heights from the wet corner samples, dry corners at the mean.
                    const int i1 = std::min(si + kSmall, n - 1), j1 = std::min(sj + kSmall, n - 1);
                    const int ci[4] = {si, si, i1, i1}, cj[4] = {sj, j1, j1, sj};
                    float y[4], sum = 0.0f;
                    int wetCorners = 0;
                    for (int k = 0; k < 4; ++k)
                        if (wet(ci[k], cj[k])) {
                            sum += waterAt(ci[k], cj[k]);
                            ++wetCorners;
                        }
                    const float mean = wetCorners > 0 ? sum / float(wetCorners) : 0.5f * (lo + hi);
                    for (int k = 0; k < 4; ++k) y[k] = wet(ci[k], cj[k]) ? waterAt(ci[k], cj[k]) : mean;
                    const Material mat = waterMaterial(kind);
                    const glm::vec3 a(px(ci[0]), y[0], px(cj[0])), b(px(ci[1]), y[1], px(cj[1]));
                    const glm::vec3 c(px(ci[2]), y[2], px(cj[2])), d(px(ci[3]), y[3], px(cj[3]));
                    m.addTri(a, b, c, up, up, up, mat);
                    m.addTri(a, c, d, up, up, up, mat);
                }
        }

    // Sea outside the baked square: beyond the south (+Z) edge if it is sea, and beyond the east / west edges from
    // where the coast meets them (the first sea sample scanning the edge toward +Z).
    const float e = -o, far = 6000.0f, sl = 0.0f;
    const Material sea = waterMaterial(WaterKind::Sea);
    auto edgeSea = [&](float x, float z) { return level.waterKindAt(x, z) == WaterKind::Sea; };
    auto quad = [&](float x0, float z0, float x1, float z1) {
        m.addQuad({x0, sl, z0}, {x0, sl, z1}, {x1, sl, z1}, {x1, sl, z0}, sea);
    };
    auto coastZ = [&](float x) {
        for (float z = -e; z < e; z += 4.0f)
            if (edgeSea(x, z)) return z;
        return e;
    };
    if (edgeSea(0.0f, e - 2.0f)) quad(-far, e, far, far);
    if (const float z = coastZ(e - 2.0f); z < e) quad(e, z, far, e);
    if (const float z = coastZ(-e + 2.0f); z < e) quad(-far, z, -e, e);
    return m;
}

std::vector<TreeInstance> buildLevelTreeInstances(const Level& level) {
    std::vector<TreeInstance> out;
    out.reserve(level.trees().size());
    for (const LevelTree& t : level.trees()) {
        TreeInstance i;
        i.baseHeight = glm::vec4(t.x, t.y, t.z, t.height);
        i.params = glm::vec4(t.yaw, t.hue, t.width, 0.0f);
        out.push_back(i);
    }
    return out;
}

} // namespace tf
