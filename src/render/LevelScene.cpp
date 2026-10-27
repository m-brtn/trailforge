#include "render/LevelScene.h"

#include "render/GroundMaps.h"
#include "track/BuildingKit.h"
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
    case PropStyle::CurbRed: return {MatId::Paint, {0.55f, 0.04f, 0.03f}, 0.55f, 0.0f, 0.0f};
    case PropStyle::CurbWhite: return {MatId::Paint, {0.78f, 0.78f, 0.76f}, 0.55f, 0.0f, 0.0f};
    case PropStyle::LinePaint: return {MatId::Paint, {0.80f, 0.80f, 0.78f}, 0.50f, 0.0f, 0.0f};
    case PropStyle::Hidden: case PropStyle::Count: break;
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
    case WaterKind::Sea: return {{0.012f, 0.040f, 0.055f}, 7.0f, 1.0f};
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
        if (style != PropStyle::Hidden) addFlat(m, g, t, e, styleMaterial(style));
        t = e;
    }
    TrackGeometry rocks;
    for (const LevelRock& r : level.rocks()) {
        if (r.model >= 0) continue; // drawn from the model library (Renderer::setModels)
        const std::size_t first = rocks.triangleCount();
        addLevelRock(rocks, r, level.models());
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
    // Water surface per sample, grown up to 3 samples under the banks (dry neighbors whose ground is above the
    // neighboring surface), so cell edges end inside the banks and water_fs fades the shoreline by depth.
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
                    if (surf[q] > next[k] && level.heights()[k] >= surf[q] - 0.05f) {
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

    // Block statistics: the range of the wet surfaces, the kind, and whether a dry sample lies below the surface
    // (a flat quad would put water there).
    struct Block {
        float lo = 1e30f, hi = -1e30f;
        WaterKind kind = WaterKind::None;
        bool dryLow = false;
    };
    auto scan = [&](int i0, int j0, int size) {
        Block b;
        for (int j = j0; j <= std::min(j0 + size, n - 1); ++j)
            for (int i = i0; i <= std::min(i0 + size, n - 1); ++i) {
                if (!wet(i, j)) continue;
                const float w = waterAt(i, j);
                b.lo = std::min(b.lo, w);
                b.hi = std::max(b.hi, w);
                if (b.kind == WaterKind::None) b.kind = kinds[std::size_t(j) * n + i];
            }
        if (b.hi < b.lo) return b;
        for (int j = j0; j <= std::min(j0 + size, n - 1) && !b.dryLow; ++j)
            for (int i = i0; i <= std::min(i0 + size, n - 1); ++i)
                if (!wet(i, j) && level.heightSample(i, j) < b.hi - 0.02f) {
                    b.dryLow = true;
                    break;
                }
        return b;
    };
    auto emit = [&](auto&& self, int i0, int j0, int size) -> void {
        const Block b = scan(i0, j0, size);
        if (b.hi < b.lo) return;
        const int i1 = std::min(i0 + size, n - 1), j1 = std::min(j0 + size, n - 1);
        const Material mat = waterMaterial(b.kind);
        if (b.hi - b.lo < 0.02f && !b.dryLow) {
            // CCW from above (+Y): (x0, z0) -> (x0, z1) -> (x1, z1) -> (x1, z0).
            const float y = b.hi;
            m.addQuad({px(i0), y, px(j0)}, {px(i0), y, px(j1)}, {px(i1), y, px(j1)}, {px(i1), y, px(j0)}, mat);
            return;
        }
        if (size > 1) {
            const int sub = size / 4;
            for (int sj = j0; sj < j1; sj += sub)
                for (int si = i0; si < i1; si += sub) self(self, si, sj, sub);
            return;
        }
        // One cell: corner heights from the wet corners, dry corners at their mean (water_fs fades them by depth).
        const int ci[4] = {i0, i0, i1, i1}, cj[4] = {j0, j1, j1, j0};
        float y[4], sum = 0.0f;
        int wetCorners = 0;
        for (int k = 0; k < 4; ++k)
            if (wet(ci[k], cj[k])) {
                sum += waterAt(ci[k], cj[k]);
                ++wetCorners;
            }
        if (wetCorners == 0) return;
        const float mean = sum / float(wetCorners);
        for (int k = 0; k < 4; ++k) y[k] = wet(ci[k], cj[k]) ? waterAt(ci[k], cj[k]) : mean;
        // Steps of the river surface (waterfalls) get the falling sheet of buildLevelWaterfallMesh instead.
        if (std::max({y[0], y[1], y[2], y[3]}) - std::min({y[0], y[1], y[2], y[3]}) > 1.0f) return;
        const glm::vec3 a(px(ci[0]), y[0], px(cj[0])), bb(px(ci[1]), y[1], px(cj[1]));
        const glm::vec3 c(px(ci[2]), y[2], px(cj[2])), d(px(ci[3]), y[3], px(cj[3]));
        m.addTri(a, bb, c, up, up, up, mat);
        m.addTri(a, c, d, up, up, up, mat);
    };
    constexpr int kBig = 16;
    for (int bj = 0; bj < n - 1; bj += kBig)
        for (int bi = 0; bi < n - 1; bi += kBig) emit(emit, bi, bj, kBig);

    // Open sea outside the baked square out to kHorizonSea on every side; the horizon terrain (land side) covers it.
    const float e = -o, far = kHorizonSea;
    const Material sea = waterMaterial(WaterKind::Sea);
    auto quad = [&](float x0, float z0, float x1, float z1) {
        m.addQuad({x0, 0.0f, z0}, {x0, 0.0f, z1}, {x1, 0.0f, z1}, {x1, 0.0f, z0}, sea);
    };
    quad(-far, e, far, far);
    quad(-far, -far, far, -e);
    quad(e, -e, far, e);
    quad(-far, -e, -e, e);
    return m;
}

MeshData buildLevelWaterfallMesh(const Level& level) {
    MeshData m;
    constexpr float kG = 9.81f, kPi = 3.14159265f;
    auto push = [&](const glm::vec3& p, const glm::vec3& n, const glm::vec3& c, const glm::vec4& mat) {
        m.vertices.push_back({p, n, c, mat});
    };
    auto quad = [&](const glm::vec3 p[4], const glm::vec3 n[4], const glm::vec3 c[4], const glm::vec4& mat) {
        for (int k : {0, 1, 2, 0, 2, 3}) push(p[k], n[k], c[k], mat);
    };
    int index = 0;
    for (const LevelWaterfall& w : level.waterfalls()) {
        const float drop = w.y0 - w.y1;
        glm::vec2 d(w.x1 - w.x0, w.z1 - w.z0);
        d = glm::length(d) > 1e-3f ? glm::normalize(d) : glm::vec2(0.0f, 1.0f);
        const glm::vec3 dir(d.x, 0.0f, d.y), across(-d.y, 0.0f, d.x), up(0.0f, 1.0f, 0.0f);
        const float seed = float(index++) * 0.37f;
        const glm::vec4 mat(drop, w.width, seed, 0.0f);
        // Sheet: ballistic from the lip (speed v0 along the river), widening a little as it falls.
        const float v0 = 1.6f + 0.06f * w.width, T = std::sqrt(2.0f * (drop + 0.3f) / kG);
        const glm::vec3 lip(w.x0 + d.x * 0.6f, w.y0 + 0.05f, w.z0 + d.y * 0.6f);
        const int rows = std::clamp(int(drop * 1.2f), 6, 40), cols = std::max(4, int(w.width / 1.5f));
        auto at = [&](float u, float s) {
            const float t = s * T;
            return lip + dir * (v0 * t) + across * (u * w.width * (1.0f + 0.25f * s)) - up * (0.5f * kG * t * t);
        };
        auto normal = [&](float s) {
            const glm::vec3 tangent = dir * v0 - up * (kG * s * T);
            return glm::normalize(glm::cross(tangent, across));
        };
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c) {
                const float s0 = float(r) / float(rows), s1 = float(r + 1) / float(rows);
                const float u0 = float(c) / float(cols) - 0.5f, u1 = float(c + 1) / float(cols) - 0.5f;
                const glm::vec3 p[4] = {at(u0, s0), at(u1, s0), at(u1, s1), at(u0, s1)};
                const glm::vec3 n[4] = {normal(s0), normal(s0), normal(s1), normal(s1)};
                const glm::vec3 col[4] = {{u0 + 0.5f, s0, 0.0f}, {u1 + 0.5f, s0, 0.0f}, {u1 + 0.5f, s1, 0.0f}, {u0 + 0.5f, s1, 0.0f}};
                quad(p, n, col, mat);
            }
        // Foam on the pool around the impact, and crossed mist cards rising from it.
        const glm::vec3 foot = at(0.0f, 1.0f);
        const glm::vec3 centre(foot.x, w.y1 + 0.04f, foot.z);
        const float R = 0.55f * w.width + 0.12f * drop;
        constexpr int kSeg = 24, kRings = 3;
        for (int ring = 0; ring < kRings; ++ring)
            for (int sgm = 0; sgm < kSeg; ++sgm) {
                const float r0 = R * float(ring) / kRings, r1 = R * float(ring + 1) / kRings;
                const float a0 = 2.0f * kPi * float(sgm) / kSeg, a1 = 2.0f * kPi * float(sgm + 1) / kSeg;
                const glm::vec2 q[4] = {{r0 * std::cos(a0), r0 * std::sin(a0)}, {r0 * std::cos(a1), r0 * std::sin(a1)},
                                        {r1 * std::cos(a1), r1 * std::sin(a1)}, {r1 * std::cos(a0), r1 * std::sin(a0)}};
                glm::vec3 p[4], n[4], col[4];
                for (int k = 0; k < 4; ++k) {
                    p[k] = centre + glm::vec3(q[k].x, 0.0f, q[k].y);
                    n[k] = up;
                    col[k] = glm::vec3(0.5f + 0.5f * q[k].x / R, 0.5f + 0.5f * q[k].y / R, 1.0f);
                }
                quad(p, n, col, mat);
            }
        const float mistH = 3.0f + 0.45f * drop, mistW = std::min(2.2f * R, 1.2f * w.width);
        for (int k = 0; k < 3; ++k) {
            const float a = kPi * float(k) / 3.0f + seed;
            const glm::vec3 side(std::cos(a), 0.0f, std::sin(a)), n(-side.z, 0.0f, side.x);
            const glm::vec3 base = centre - up * 0.5f;
            const glm::vec3 p[4] = {base - side * (0.5f * mistW), base + side * (0.5f * mistW),
                                    base + side * (0.5f * mistW) + up * mistH, base - side * (0.5f * mistW) + up * mistH};
            const glm::vec3 nn[4] = {n, n, n, n};
            const glm::vec3 col[4] = {{0.0f, 0.0f, 2.0f}, {1.0f, 0.0f, 2.0f}, {1.0f, 1.0f, 2.0f}, {0.0f, 1.0f, 2.0f}};
            quad(p, nn, col, mat);
        }
    }
    return m;
}

// Far-view albedo of the textured ground layers (grass: the far canopy tone of terrain_fs) and a correction for the
// ring's lighting (no ground AO, more sky light), set so the ring next to the bake edge matches the bake's mean
// pixel colour in top-down captures at 10:00.
constexpr glm::vec3 kEdgeGrass(0.090f, 0.108f, 0.030f), kEdgeSoil(0.150f, 0.118f, 0.078f), kEdgeRock(0.190f, 0.175f, 0.160f),
                    kEdgeSand(0.360f, 0.310f, 0.210f);
constexpr glm::vec3 kEdgeLight(1.46f, 1.08f, 0.88f);
// Mean of the ring's own grass / dry / forest palette (paletteMean in lit_fs MAT_HORIZON).
constexpr glm::vec3 kPaletteMean(0.098f, 0.093f, 0.045f);

MeshData buildLevelHorizonMesh(const Level& level) {
    const int n = level.samples();
    const float e = -level.origin(); // half side of the baked square
    // Columns along the bake edge (every 16 samples, shared with the terrain edge heights), counter-clockwise.
    constexpr int kStride = 16;
    const int perSide = (n - 1) / kStride;
    struct Column {
        glm::vec2 p;  // on the bake edge
        float h0;     // terrain height there
        float land;   // 0 sea .. 1 land, smoothed along the edge
    };
    std::vector<Column> cols;
    for (int side = 0; side < 4; ++side)
        for (int k = 0; k < perSide; ++k) {
            const int t = k * kStride;
            int i = 0, j = 0;
            switch (side) {
            case 0: i = t; j = n - 1; break;         // south edge, west -> east
            case 1: i = n - 1; j = n - 1 - t; break; // east edge, south -> north
            case 2: i = n - 1 - t; j = 0; break;     // north edge, east -> west
            default: i = 0; j = t; break;            // west edge, north -> south
            }
            const float x = level.origin() + float(i) * level.cell(), z = level.origin() + float(j) * level.cell();
            cols.push_back({{x, z}, level.heightSample(i, j), 0.0f});
        }
    const int nc = int(cols.size());
    // Sea where the bake's ground is below the open sea plane (y = 0) just inside the edge, smoothed along the edge.
    // The bake tags sea water only near its coast, so open sea at the edge is told by height; levels without any
    // sea water are land all round, whatever their heights.
    const bool hasSea = std::find(level.waterKinds().begin(), level.waterKinds().end(), WaterKind::Sea) != level.waterKinds().end();
    const auto seaAt = [&](glm::vec2 q) {
        return hasSea && (level.waterKindAt(q.x, q.y) == WaterKind::Sea || level.heightAt(q.x, q.y) < 0.0f);
    };
    std::vector<float> sea(static_cast<std::size_t>(nc));
    for (int c = 0; c < nc; ++c) sea[std::size_t(c)] = seaAt(cols[std::size_t(c)].p * (1.0f - 2.0f * level.cell() / e)) ? 1.0f : 0.0f;
    for (int c = 0; c < nc; ++c) {
        float sum = 0.0f;
        for (int d = -10; d <= 10; ++d) sum += sea[std::size_t((c + d + nc) % nc)];
        cols[std::size_t(c)].land = 1.0f - sum / 21.0f;
    }

    // Rings: 8 m apart at the edge, then 9 % farther apart per ring out past the outer coast.
    std::vector<float> rings{0.0f};
    while (rings.back() < kHorizonLand) rings.push_back(rings.back() + std::clamp(rings.back() * 0.15f, 8.0f, 1800.0f));
    const int nr = int(rings.size());
    // The bake edge profile (hills, cut walls) would run outward as radial ridges while it fades: it is averaged along
    // the edge over a window that grows with the distance (about +-r/2), so the ring loses it smoothly.
    std::vector<double> prefix(std::size_t(nc) + 1, 0.0);
    for (int c = 0; c < nc; ++c) prefix[std::size_t(c) + 1] = prefix[std::size_t(c)] + double(cols[std::size_t(c)].h0);
    const auto edgeHeight = [&](int c, float r) {
        const int half = std::clamp(int(r / (2.0f * float(kStride) * level.cell())), 0, nc / 8);
        double sum = 0.0;
        const int a = c - half, b = c + half; // inclusive, may wrap around the loop
        const auto range = [&](int lo, int hi) { return prefix[std::size_t(hi) + 1] - prefix[std::size_t(lo)]; };
        if (a < 0) sum = range(0, b) + range(nc + a, nc - 1);
        else if (b >= nc) sum = range(a, nc - 1) + range(0, b - nc);
        else sum = range(a, b);
        return float(sum / double(2 * half + 1));
    };
    // Land / sea along the edge around a fractional column, averaged over +-half columns (wraps around the loop).
    std::vector<double> landPrefix(std::size_t(2 * nc) + 1, 0.0);
    for (int c = 0; c < 2 * nc; ++c)
        landPrefix[std::size_t(c) + 1] = landPrefix[std::size_t(c)] + double(cols[std::size_t(c % nc)].land);
    const auto landAlong = [&](float c, int half) {
        half = std::clamp(half, 0, nc / 4);
        const int i = int(std::floor(c));
        const int a = ((i - half) % nc + nc) % nc;
        return float((landPrefix[std::size_t(a + 2 * half + 1)] - landPrefix[std::size_t(a)]) / double(2 * half + 1));
    };
    // Ring point of a column: on the square of the bake edge, rounding into a circle from ~200 m to ~4 km out, so
    // the surrounding land has no square silhouette from far above.
    const float roundR = 1.148f * e; // mean distance of the square's edge points from its centre
    const auto place = [&](const Column& col, float r) {
        const float len = glm::length(col.p);
        return col.p / len * glm::mix(len * (e + r) / e, roundR + r, glm::smoothstep(200.0f, 4000.0f, r));
    };
    // Surrounding land in world space (r: distance past the bake edge): ridged hills everywhere (they rise out of the
    // valley haze right past the edge); to the north (-Z) a mountain range from ~1 km out, lower again far away.
    const auto landHeight = [&](glm::vec2 p, float r) {
        const float north = glm::smoothstep(0.1f, 0.8f, -p.y / glm::length(p));
        const float spur = 1.0f - std::abs(2.0f * fbm2(p.x / 420.0f, p.y / 420.0f, 3, 719u) - 1.0f);
        const float hills = 60.0f + 230.0f * fbm2(p.x / 950.0f, p.y / 950.0f, 4, 711u) + 70.0f * spur * spur;
        const float ridge = 1.0f - std::abs(2.0f * fbm2(p.x / 1700.0f, p.y / 1700.0f, 4, 733u) - 1.0f);
        const float range = (260.0f + 640.0f * ridge * ridge) * glm::smoothstep(500.0f, 3200.0f, r) *
                            (1.0f - 0.55f * glm::smoothstep(7000.0f, 14000.0f, r));
        return glm::mix(hills, std::max(hills, range), north);
    };
    // The land starts at the level of the bake edge (averaged over ~320 m) and settles to its own heights by ~3.5 km.
    std::vector<float> offset(static_cast<std::size_t>(nc));
    for (int c = 0; c < nc; ++c) offset[std::size_t(c)] = edgeHeight(c, 640.0f) - landHeight(cols[std::size_t(c)].p, 0.0f);
    auto heightAt = [&](int c, glm::vec2 p, float r) {
        const Column& col = cols[std::size_t(c)];
        const glm::vec2 dir = col.p / glm::length(col.p);
        // Coast: the edge's land / sea read at a column shifted by noise growing with the distance (bays and capes
        // instead of a straight line out of the bake corner), small islands and inlets, and an outer coast 15-21 km
        // out where the land sinks into the open sea.
        const float shift = (fbm2(p.x / 3100.0f, p.y / 3100.0f, 3, 747u) - 0.5f) * 0.9f * r / float(kStride);
        float land = landAlong(float(c) + shift, int(r / (4.0f * float(kStride) * level.cell())));
        land += (fbm2(p.x / 1700.0f, p.y / 1700.0f, 4, 751u) - 0.5f) * 0.6f * glm::smoothstep(300.0f, 3500.0f, r);
        const float coastR = 15000.0f + 6000.0f * fbm2(dir.x * 2.2f + 11.0f, dir.y * 2.2f + 5.0f, 3, 757u);
        land -= glm::smoothstep(coastR - 6000.0f, coastR, r);
        // Kept well above the sea where the coast noise raises land off a sea column (its edge offset is negative).
        const float hills = landHeight(p, r);
        const float ground = std::max(hills + offset[std::size_t(c)] * (1.0f - glm::smoothstep(0.0f, 3500.0f, r)), 0.35f * hills);
        const float seabed = -18.0f - 22.0f * glm::smoothstep(0.0f, 4000.0f, r);
        // A narrow land / sea transition: shallow flats would read as pale lagoons under the sea's fog.
        const float target = glm::mix(seabed, ground, glm::smoothstep(0.42f, 0.58f, land));
        // Land stays above the open sea plane (y = 0, everywhere outside the bake) and its beach band: levels whose
        // edge is near sea level would otherwise show shallow sea around the bake.
        const float h = glm::mix(edgeHeight(c, r), target, glm::smoothstep(0.0f, 600.0f, r));
        // Only land is lifted (the seabed stays below the sea plane), easing in from the coastline without a step.
        const float lift = 8.0f * glm::smoothstep(0.5f, 1.0f, land) * glm::smoothstep(0.0f, 60.0f, r);
        return h + std::max(0.0f, lift - h) * glm::smoothstep(0.5f, 0.6f, land);
    };
    std::vector<glm::vec3> pos(std::size_t(nc + 1) * nr);
    for (int c = 0; c <= nc; ++c)
        for (int k = 0; k < nr; ++k) {
            const Column& col = cols[std::size_t(c % nc)];
            const glm::vec2 p = place(col, rings[std::size_t(k)]);
            pos[std::size_t(c) * nr + k] = glm::vec3(p.x, heightAt(c % nc, p, rings[std::size_t(k)]), p.y);
        }
    auto P = [&](int c, int k) { return pos[std::size_t(c) * nr + std::size_t(k)]; };
    auto normal = [&](int c, int k) {
        const glm::vec3 dc = P(c == nc ? 1 : c + 1, k) - P(c == 0 ? nc - 1 : c - 1, k);
        const glm::vec3 dk = P(c, std::min(k + 1, nr - 1)) - P(c, std::max(k - 1, 0));
        glm::vec3 nn = glm::cross(dk, dc);
        if (nn.y < 0.0f) nn = -nn;
        return glm::normalize(nn);
    };
    // Vertex colour: far-view ground colour of the baked terrain just inside each column (mean grass, soil, rock and
    // sand tones of the textured ground seen from hundreds of meters, from the render masks over the last 160 m),
    // smoothed along the edge over a window growing with the distance. Snow is left out: lit_fs puts it on the ring
    // by the level's snow line, so a snowy edge tone does not smear outward. The palette itself is per pixel
    // (MatId::Horizon in lit_fs): it starts from this tone and keeps its mean (as a tint) over the first few km.
    const int cn = level.cells();
    const auto cellAt = [&](float x, float z) {
        const int i = std::clamp(int((x - level.origin()) / level.cell()), 0, cn - 1);
        const int j = std::clamp(int((z - level.origin()) / level.cell()), 0, cn - 1);
        return std::size_t(j) * std::size_t(cn) + std::size_t(i);
    };
    const auto byte = [](std::uint32_t v, int ch) { return float((v >> (8 * ch)) & 0xffu) / 255.0f; };
    std::vector<glm::vec3> edgeColor(std::size_t(nc), glm::vec3(0.0f));
    for (int c = 0; c < nc; ++c) {
        const glm::vec2 p = cols[std::size_t(c)].p, in = -glm::normalize(p);
        glm::vec3 sum(0.0f);
        int count = 0;
        for (int d = -2; d <= 2; ++d)
            for (float t = 8.0f; t <= 160.0f; t += 8.0f) {
                const glm::vec2 q = p + in * t + glm::vec2(-in.y, in.x) * (float(d) * 12.0f);
                const std::size_t k = cellAt(q.x, q.y);
                if (seaAt(q)) continue; // sea floor: the ring's palette (no tint) under the sea
                const float grass = byte(level.maskTexels()[k], 0);
                const std::uint32_t sp = level.splatTexels()[k];
                const float rock = byte(sp, 1), sand = std::max(0.0f, 2.0f * byte(sp, 2) - 1.0f);
                glm::vec3 g = glm::mix(kEdgeSoil, kEdgeGrass, grass);
                g = glm::mix(g, kEdgeSand, sand);
                g = glm::mix(g, kEdgeRock, rock);
                sum += g;
                ++count;
            }
        edgeColor[std::size_t(c)] = count > 0 ? sum / float(count) : kPaletteMean / kEdgeLight;
    }
    std::vector<glm::dvec3> colorPrefix(std::size_t(2 * nc) + 1, glm::dvec3(0.0));
    for (int c = 0; c < 2 * nc; ++c)
        colorPrefix[std::size_t(c) + 1] = colorPrefix[std::size_t(c)] + glm::dvec3(edgeColor[std::size_t(c % nc)]);
    const auto edgeTone = [&](int c, float r) {
        const int half = std::clamp(4 + int(r / (2.0f * float(kStride) * level.cell())), 0, nc / 4);
        const int a = ((c - half) % nc + nc) % nc;
        const glm::dvec3 sum = colorPrefix[std::size_t(a + 2 * half + 1)] - colorPrefix[std::size_t(a)];
        return glm::vec3(sum / double(2 * half + 1)) * kEdgeLight;
    };

    MeshData m;
    m.vertices.reserve(std::size_t(nc) * std::size_t(nr - 1) * 6);
    // Material z: the level's snow line at the vertex (lit_fs puts snow above it on gentle faces, as the bake does).
    auto vert = [&](int c, int k) {
        const glm::vec3 p = P(c, k), nn = normal(c, k);
        // From ~300 m out the line climbs to at least 520 m by ~3 km, so snow stays on the range and the high hills.
        const float line = level.snowLine() + level.snowEastRise() * std::max(0.0f, p.x - level.snowEastFrom());
        const float snowLine = glm::mix(line, std::max(line, 520.0f), glm::smoothstep(300.0f, 3000.0f, rings[std::size_t(k)]));
        m.vertices.push_back({p, nn, edgeTone(c % nc, rings[std::size_t(k)]), glm::vec4(float(MatId::Horizon), 0.92f, snowLine, 0.0f)});
    };
    for (int c = 0; c < nc; ++c)
        for (int k = 0; k + 1 < nr; ++k) {
            const glm::vec3 a = P(c, k), b = P(c + 1, k), d = P(c, k + 1);
            // Winding: CCW seen from above (the columns run counter-clockwise when seen from +Y).
            const bool ccw = glm::cross(b - a, d - a).y >= 0.0f;
            const int tri[6][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 0}, {1, 1}, {0, 1}};
            for (int v = 0; v < 6; ++v) {
                const int t = ccw ? v : (v % 3 == 0 ? v : (v % 3 == 1 ? v + 1 : v - 1));
                vert(c + tri[t][0], k + tri[t][1]);
            }
        }
    return m;
}

std::vector<TreeInstance> buildLevelTreeInstances(const Level& level) {
    std::vector<TreeInstance> out;
    out.reserve(level.trees().size());
    for (const LevelTree& t : level.trees()) {
        if (t.model >= 0) continue;
        TreeInstance i;
        i.baseHeight = glm::vec4(t.x, t.y, t.z, t.height);
        i.params = glm::vec4(t.yaw, t.hue, t.width, 0.0f);
        out.push_back(i);
    }
    return out;
}

void buildLevelModelPlacements(const Level& level, std::vector<std::string>& paths, std::vector<ModelPlacement>& out) {
    paths.clear();
    out.clear();
    for (const LevelModel& m : level.models()) paths.push_back(m.path);
    if (paths.empty()) return;
    out.reserve(level.trees().size() + level.rocks().size() + level.scatter().size());
    for (const LevelTree& t : level.trees()) {
        if (t.model < 0) continue;
        const float xz = t.scale * t.width;
        out.push_back({t.model, glm::vec3(t.x, t.y, t.z), glm::vec3(xz, t.scale, xz), t.yaw});
    }
    for (const LevelRock& r : level.rocks()) {
        if (r.model < 0) continue;
        out.push_back({r.model, glm::vec3(r.x, r.y, r.z), glm::vec3(r.sx, r.sy, r.sz), r.yaw, r.pitch});
    }
    for (const LevelScatter& c : level.scatter())
        out.push_back({c.model, glm::vec3(c.x, c.y, c.z), glm::vec3(c.scale), c.yaw});
}

std::vector<ModelAsset> buildLevelBuildingModels(const Level& level) {
    std::vector<ModelAsset> out;
    out.reserve(level.buildingTypes().size());
    for (const LevelBuildingType& t : level.buildingTypes()) out.push_back(buildBuilding(t.spec, true).model);
    return out;
}

void appendLevelBuildingPlacements(const Level& level, int first, std::vector<ModelPlacement>& out) {
    for (const LevelBuilding& b : level.buildings())
        out.push_back({first + int(b.type), glm::vec3(b.x, b.y, b.z), glm::vec3(1.0f), b.yaw});
}

} // namespace tf
