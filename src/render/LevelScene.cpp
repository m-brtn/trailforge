#include "render/LevelScene.h"

#include "render/GroundMaps.h"
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
    for (int c = 0; c < nc; ++c) {
        float sum = 0.0f;
        for (int d = -8; d <= 8; ++d) sum += cols[std::size_t((c + d + nc) % nc)].h0;
        cols[std::size_t(c)].land = glm::smoothstep(-6.0f, 8.0f, sum / 17.0f);
    }

    // Rings: geometric spacing out to ~14 km; the square scales outward from the center.
    static constexpr float kRings[] = {0,    8,    24,   50,   90,   150,  220,  300,  400,  520,  660,  820,  1000, 1200,
                                       1450, 1750, 2100, 2500, 3000, 3600, 4300, 5100, 6100, 7300, 8700, 10300, 12100, 14000};
    constexpr int nr = int(std::size(kRings));
    auto heightAt = [&](const Column& col, float r) {
        const glm::vec2 p = col.p * ((e + r) / e);
        const float w = glm::smoothstep(0.0f, 700.0f, r);
        // Hills everywhere on land; to the north (-Z) a mountain range from ~2 km out, lower again far away.
        const float north = glm::smoothstep(0.1f, 0.8f, -p.y / glm::length(p));
        const float hills = 40.0f + 170.0f * fbm2(p.x / 1100.0f, p.y / 1100.0f, 4, 711u);
        const float ridge = 1.0f - std::abs(2.0f * fbm2(p.x / 1700.0f, p.y / 1700.0f, 4, 733u) - 1.0f);
        const float range = (260.0f + 640.0f * ridge * ridge) * glm::smoothstep(1200.0f, 4200.0f, r) *
                            (1.0f - 0.55f * glm::smoothstep(7000.0f, 14000.0f, r));
        const float land = glm::mix(hills, std::max(hills, range), north);
        const float seabed = -18.0f - 12.0f * glm::smoothstep(0.0f, 2500.0f, r);
        const float target = glm::mix(seabed, land, col.land);
        return glm::mix(col.h0, target, w);
    };
    std::vector<glm::vec3> pos(std::size_t(nc + 1) * nr);
    for (int c = 0; c <= nc; ++c)
        for (int k = 0; k < nr; ++k) {
            const Column& col = cols[std::size_t(c % nc)];
            const glm::vec2 p = col.p * ((e + kRings[k]) / e);
            pos[std::size_t(c) * nr + k] = glm::vec3(p.x, heightAt(col, kRings[k]), p.y);
        }
    auto P = [&](int c, int k) { return pos[std::size_t(c) * nr + std::size_t(k)]; };
    auto normal = [&](int c, int k) {
        const glm::vec3 dc = P(std::min(c + 1, nc), k) - P(std::max(c - 1, 0), k);
        const glm::vec3 dk = P(c, std::min(k + 1, nr - 1)) - P(c, std::max(k - 1, 0));
        glm::vec3 nn = glm::cross(dk, dc);
        if (nn.y < 0.0f) nn = -nn;
        return glm::normalize(nn);
    };
    // Per-vertex albedo (generic material, no procedural ground pattern that would tile visibly from far away):
    // grass and forest tones in large noise patches, rock on steep or high ground, snow only on gentle faces above
    // a noisy snow line, sand at the waterline. Colors blend smoothly across the large ring quads.
    auto albedo = [](const glm::vec3& p, const glm::vec3& nn) {
        const float slope = 1.0f - nn.y;
        const float n1 = fbm2(p.x / 700.0f, p.z / 700.0f, 3, 751u);
        const float n2 = fbm2(p.x / 260.0f, p.z / 260.0f, 3, 757u);
        glm::vec3 c = glm::mix(glm::vec3(0.060f, 0.080f, 0.026f), glm::vec3(0.115f, 0.112f, 0.045f), glm::smoothstep(0.35f, 0.75f, n1));
        c = glm::mix(c, glm::vec3(0.032f, 0.050f, 0.022f), 0.8f * glm::smoothstep(0.55f, 0.75f, n2)); // forest
        const glm::vec3 rock = glm::mix(glm::vec3(0.115f, 0.105f, 0.095f), glm::vec3(0.205f, 0.190f, 0.170f), n2);
        c = glm::mix(c, rock, glm::max(glm::smoothstep(0.18f, 0.32f, slope), glm::smoothstep(380.0f, 560.0f, p.y)));
        const float snowLine = 720.0f + 260.0f * (n1 - 0.5f);
        const float snow = glm::smoothstep(snowLine, snowLine + 120.0f, p.y) * (1.0f - glm::smoothstep(0.22f, 0.36f, slope));
        c = glm::mix(c, glm::vec3(0.72f, 0.74f, 0.78f), snow);
        c = glm::mix(c, glm::vec3(0.36f, 0.31f, 0.21f), 1.0f - glm::smoothstep(1.5f, 5.0f, p.y));
        return c;
    };
    MeshData m;
    m.vertices.reserve(std::size_t(nc) * std::size_t(nr - 1) * 6);
    auto vert = [&](int c, int k) {
        const glm::vec3 p = P(c, k), nn = normal(c, k);
        m.vertices.push_back({p, nn, albedo(p, nn), glm::vec4(float(MatId::Generic), 0.92f, 0.0f, 0.0f)});
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

} // namespace tf
