#include "track/BuildingKit.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace tf {
namespace {

constexpr float kPi = 3.14159265358979f;

struct V {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    constexpr V() = default;
    constexpr V(float a, float b, float c) : x(a), y(b), z(c) {}
};
constexpr V operator+(V a, V b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr V operator-(V a, V b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
constexpr V operator-(V a) { return {-a.x, -a.y, -a.z}; }
constexpr V operator*(V a, float s) { return {a.x * s, a.y * s, a.z * s}; }
constexpr float dot(V a, V b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr V cross(V a, V b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float len(V a) { return std::sqrt(dot(a, a)); }
V norm(V a) {
    const float l = len(a);
    return l > 1e-12f ? a * (1.0f / l) : V(0, 1, 0);
}
constexpr V kX(1, 0, 0), kY(0, 1, 0), kZ(0, 0, 1);

constexpr std::uint8_t L0 = 1, L1 = 2, L2 = 4, NEAR = L0 | L1, ALL = L0 | L1 | L2, FAR = L2, MID = L1 | L2;

// Box face bits.
constexpr int FPX = 1, FNX = 2, FPY = 4, FNY = 8, FPZ = 16, FNZ = 32, FALL = 63, FSIDES = FPX | FNX | FPZ | FNZ;

std::uint32_t hash(std::uint32_t a, std::uint32_t b) {
    std::uint32_t h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u + (a << 6) + (a >> 2));
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return h;
}
float hash01(std::uint32_t a, std::uint32_t b) { return float(hash(a, b) >> 8) / 16777216.0f; }

const KitMaterial kMaterials[kKitMatCount] = {
    {"plaster", "plaster", 2.0f, {0.92f, 0.88f, 0.80f}, -1.0f},
    {"plaster_white", "plaster", 2.0f, {1.00f, 0.98f, 0.94f}, -1.0f},
    {"plaster_red", "plaster", 2.0f, {0.72f, 0.16f, 0.12f}, -1.0f},
    {"plaster_ochre", "plaster", 2.0f, {0.98f, 0.76f, 0.48f}, -1.0f},
    {"stone", "stone", 1.5f, {1.0f, 1.0f, 1.0f}, -1.0f},
    {"castle", "castle", 2.5f, {1.0f, 1.0f, 1.0f}, -1.0f},
    {"siding", "siding", 1.6f, {1.0f, 1.0f, 1.0f}, -1.0f},
    {"siding_dark", "siding", 1.6f, {0.68f, 0.60f, 0.54f}, -1.0f},
    {"wood", "wood", 2.0f, {1.0f, 1.0f, 1.0f}, -1.0f},
    {"wood_dark", "wood", 2.0f, {0.42f, 0.33f, 0.27f}, -1.0f},
    {"paint_green", "wood", 2.0f, {0.36f, 0.62f, 0.40f}, -1.0f},
    {"paint_red", "wood", 2.0f, {0.95f, 0.30f, 0.24f}, -1.0f},
    {"paint_white", "wood", 2.0f, {1.45f, 1.42f, 1.36f}, -1.0f},
    {"roof_tiles", "roof_tiles", 2.5f, {1.0f, 1.0f, 1.0f}, -1.0f},
    {"shingles", "shingles", 3.0f, {0.86f, 0.80f, 0.76f}, -1.0f},
    {"corrugated", "corrugated", 1.8f, {1.08f, 1.00f, 0.90f}, -1.0f},
    {"corrugated_grey", "corrugated", 1.8f, {0.82f, 0.80f, 0.76f}, -1.0f},
    {"corrugated_red", "corrugated", 1.8f, {0.85f, 0.38f, 0.30f}, -1.0f},
    {"metal", nullptr, 1.0f, {0.42f, 0.43f, 0.44f}, 0.45f},
    {"metal_red", nullptr, 1.0f, {0.50f, 0.045f, 0.03f}, 0.40f},
    {"metal_white", nullptr, 1.0f, {0.72f, 0.72f, 0.70f}, 0.40f},
    {"metal_dark", "metal", 2.0f, {0.80f, 0.80f, 0.82f}, -1.0f},
    {"rust", "rust", 2.0f, {1.0f, 1.0f, 1.0f}, -1.0f},
    {"concrete", "concrete", 2.0f, {1.0f, 1.0f, 1.0f}, -1.0f},
    {"glass", nullptr, 1.0f, {0.020f, 0.026f, 0.032f}, 0.06f},
    {"shadow", nullptr, 1.0f, {0.012f, 0.012f, 0.011f}, 0.95f},
    {"soil", nullptr, 1.0f, {0.085f, 0.060f, 0.040f}, 0.95f},
    {"leaves", nullptr, 1.0f, {0.060f, 0.140f, 0.035f}, 0.80f},
    {"flowers", nullptr, 1.0f, {0.560f, 0.060f, 0.070f}, 0.70f},
    {"nylon_orange", nullptr, 1.0f, {0.700f, 0.230f, 0.040f}, 0.55f},
    {"nylon_green", nullptr, 1.0f, {0.110f, 0.260f, 0.120f}, 0.55f},
    {"lamp", nullptr, 1.0f, {0.950f, 0.820f, 0.520f}, 0.30f},
};

// Geometry of one building under construction: per LOD and material vertex lists, plus the collision pieces.
class Builder {
public:
    Builder(BuildingGeometry& out, bool render) : g(out), m_render(render) {}

    // ------------------------------------------------------------------ emission
    static void uvOf(V p, V n, float tile, float& u, float& v) {
        if (std::abs(n.y) > 0.98f) {
            u = p.x / tile;
            v = p.z / tile;
            return;
        }
        const V t = norm(cross(kY, n)), b = cross(n, t);
        u = dot(p, t) / tile;
        v = -dot(p, b) / tile;
    }
    void emit(KitMat m, std::uint8_t lods, const V p[3], const V n[3], const float uv[6]) {
        if (!m_render) return;
        for (int l = 0; l < 3; ++l) {
            if (!(lods & (1 << l))) continue;
            auto& list = m_lod[std::size_t(l)][std::size_t(m)];
            for (int k = 0; k < 3; ++k)
                list.push_back({{p[k].x, p[k].y, p[k].z}, {n[k].x, n[k].y, n[k].z}, {uv[2 * k], uv[2 * k + 1]}, {0, 0}});
        }
    }
    // Flat triangle, wound counter-clockwise around n (seen from the side n points to).
    void tri(KitMat m, std::uint8_t lods, V a, V b, V c, V n) {
        if (!m_render || lods == 0) return;
        if (dot(cross(b - a, c - a), n) < 0.0f) std::swap(b, c);
        const float tile = kMaterials[int(m)].tile;
        const V p[3] = {a, b, c}, nn[3] = {n, n, n};
        float uv[6];
        for (int k = 0; k < 3; ++k) uvOf(p[k], n, tile, uv[2 * k], uv[2 * k + 1]);
        emit(m, lods, p, nn, uv);
    }
    void quad(KitMat m, std::uint8_t lods, V a, V b, V c, V d, V n) {
        if (dot(cross(b - a, c - a), n) < 0.0f) std::swap(b, d);
        tri(m, lods, a, b, c, n);
        tri(m, lods, a, c, d, n);
    }
    void piece() { g.pieceStarts.push_back(std::uint32_t(g.collision.size() / 3)); }
    void colTri(V a, V b, V c, V n) {
        const V cr = cross(b - a, c - a);
        if (dot(cr, cr) < 1e-12f) return;
        if (dot(cr, n) < 0.0f) std::swap(b, c);
        for (const V& p : {a, b, c}) g.collision.push_back(JPH::Float3(p.x, p.y, p.z));
    }
    void colQuad(V a, V b, V c, V d, V n) {
        if (dot(cross(b - a, c - a), n) < 0.0f) std::swap(b, d);
        colTri(a, b, c, n);
        colTri(a, c, d, n);
    }

    // ------------------------------------------------------------------ parts
    // Oriented box: center c, unit axes ax / ay / az, half extents h; render faces by bit (FPX..FNZ). Solid: one
    // convex collision piece (all six faces).
    void box(KitMat m, std::uint8_t lods, bool solid, V c, V ax, V ay, V az, V h, int faces = FALL) {
        const V axis[3] = {ax, ay, az};
        const float he[3] = {h.x, h.y, h.z};
        if (solid) piece();
        for (int k = 0; k < 3; ++k)
            for (int s = 0; s < 2; ++s) {
                const float sg = s == 0 ? 1.0f : -1.0f;
                const V n = axis[k] * sg;
                const V fc = c + n * he[k];
                const V u = axis[(k + 1) % 3] * he[(k + 1) % 3], w = axis[(k + 2) % 3] * he[(k + 2) % 3];
                const V a = fc - u - w, b = fc + u - w, cc = fc + u + w, d = fc - u + w;
                if (faces & (1 << (2 * k + s))) quad(m, lods, a, b, cc, d, n);
                if (solid) colQuad(a, b, cc, d, n);
            }
    }
    void abox(KitMat m, std::uint8_t lods, bool solid, V lo, V hi, int faces = FALL) {
        box(m, lods, solid, (lo + hi) * 0.5f, kX, kY, kZ, (hi - lo) * 0.5f, faces);
    }
    // Box turned by yaw about +Y (x' = c x + s z, z' = -s x + c z).
    void ybox(KitMat m, std::uint8_t lods, bool solid, V c, V h, float yaw, int faces = FALL) {
        const float cs = std::cos(yaw), sn = std::sin(yaw);
        box(m, lods, solid, c, V(cs, 0, -sn), kY, V(sn, 0, cs), h, faces);
    }
    // Box along a -> b, 2 hw wide (horizontal across), 2 hh thick.
    void beam(KitMat m, std::uint8_t lods, bool solid, V a, V b, float hw, float hh, int faces = FALL) {
        const V d = b - a;
        const float l = len(d);
        if (l < 1e-4f) return;
        const V az = d * (1.0f / l);
        const V hint = std::abs(az.y) > 0.9f ? kX : kY;
        const V ax = norm(cross(hint, az)), ay = cross(az, ax);
        box(m, lods, solid, (a + b) * 0.5f, ax, ay, az, V(hw, hh, 0.5f * l), faces);
    }
    // Frustum from ring center a (radius ra) to b (radius rb), `sides` facets, smooth sides. caps: bit 1 at a, 2 at b.
    void cyl(KitMat m, std::uint8_t lods, bool solid, V a, V b, float ra, float rb, int sides, int caps = 3,
             float phase = 0.0f) {
        const V w = norm(b - a);
        const float l = len(b - a);
        const V e1 = norm(cross(std::abs(w.y) > 0.9f ? kX : kY, w)), e2 = cross(w, e1);
        const float tile = kMaterials[int(m)].tile, ravg = 0.5f * (ra + rb);
        auto ring = [&](float t) { return e1 * std::cos(t) + e2 * std::sin(t); };
        if (solid) piece();
        for (int k = 0; k < sides; ++k) {
            const float t0 = phase + 2.0f * kPi * float(k) / float(sides), t1 = phase + 2.0f * kPi * float(k + 1) / float(sides);
            const V r0 = ring(t0), r1 = ring(t1);
            const V p0 = a + r0 * ra, p1 = a + r1 * ra, p2 = b + r1 * rb, p3 = b + r0 * rb;
            const V faceOut = norm((r0 + r1) * l + w * (2.0f * (ra - rb)));
            if (solid) colQuad(p0, p1, p2, p3, faceOut);
            if (!m_render || lods == 0) continue;
            const V n0 = norm(r0 * l + w * (ra - rb)), n1 = norm(r1 * l + w * (ra - rb));
            const float u0 = t0 * ravg / tile, u1 = t1 * ravg / tile;
            const float va = -dot(a, w) / tile, vb = -dot(b, w) / tile;
            auto side = [&](V q0, V q1, V q2, V m0, V m1, V m2, float s0, float t0v, float s1, float t1v, float s2, float t2v) {
                const V cr = cross(q1 - q0, q2 - q0);
                if (dot(cr, cr) < 1e-12f) return;
                if (dot(cr, faceOut) >= 0.0f) {
                    const V pp[3] = {q0, q1, q2}, nn[3] = {m0, m1, m2};
                    const float uv[6] = {s0, t0v, s1, t1v, s2, t2v};
                    emit(m, lods, pp, nn, uv);
                } else {
                    const V pp[3] = {q0, q2, q1}, nn[3] = {m0, m2, m1};
                    const float uv[6] = {s0, t0v, s2, t2v, s1, t1v};
                    emit(m, lods, pp, nn, uv);
                }
            };
            side(p0, p1, p2, n0, n1, n1, u0, va, u1, va, u1, vb);
            side(p0, p2, p3, n0, n1, n0, u0, va, u1, vb, u0, vb);
        }
        for (int cap = 0; cap < 2; ++cap) {
            const float r = cap == 0 ? ra : rb;
            if (r <= 1e-4f) continue;
            const V c = cap == 0 ? a : b, n = cap == 0 ? -w : w;
            const bool draw = (caps & (1 << cap)) != 0;
            for (int k = 0; k < sides; ++k) {
                const float t0 = phase + 2.0f * kPi * float(k) / float(sides), t1 = phase + 2.0f * kPi * float(k + 1) / float(sides);
                const V q0 = c + ring(t0) * r, q1 = c + ring(t1) * r;
                if (draw) tri(m, lods, c, q0, q1, n);
                if (solid) colTri(c, q0, q1, n);
            }
        }
    }
    // Triangular prism: triangle (a, b, c) swept by `depth` along d (unit); one convex piece.
    void prism(KitMat m, std::uint8_t lods, bool solid, V a, V b, V c, V d, float depth, bool drawSides = true) {
        const V a1 = a + d * depth, b1 = b + d * depth, c1 = c + d * depth;
        const V cen = (a + b + c + a1 + b1 + c1) * (1.0f / 6.0f);
        auto out = [&](V n, V p) { return dot(n, p - cen) >= 0.0f ? n : -n; };
        const V nt = out(norm(cross(b - a, c - a)), a);
        tri(m, lods, a, b, c, nt);
        tri(m, lods, a1, b1, c1, -nt);
        if (solid) {
            piece();
            colTri(a, b, c, nt);
            colTri(a1, b1, c1, -nt);
        }
        const V e[3][2] = {{a, b}, {b, c}, {c, a}};
        for (const auto& ed : e) {
            const V p = ed[0], q = ed[1];
            const V n = out(norm(cross(q - p, d)), p);
            if (drawSides) quad(m, lods, p, q, q + d * depth, p + d * depth, n);
            if (solid) colQuad(p, q, q + d * depth, p + d * depth, n);
        }
    }

    // ------------------------------------------------------------------ result
    void finish(const BuildingSpec& spec, const char* kind) {
        if (!g.collision.empty()) {
            V lo(1e30f, 1e30f, 1e30f), hi(-1e30f, -1e30f, -1e30f);
            for (const JPH::Float3& p : g.collision) {
                lo = V(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
                hi = V(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
            }
            g.boundsMin = JPH::Float3(lo.x, lo.y, lo.z);
            g.boundsMax = JPH::Float3(hi.x, hi.y, hi.z);
        }
        if (!m_render) return;
        ModelAsset& a = g.model;
        char id[96];
        std::snprintf(id, sizeof(id), "kit_%s_%.1fx%.1fx%.1f_%d_%d", buildingKindName(spec.kind), double(spec.w),
                      double(spec.d), double(spec.h), spec.floors, spec.variant);
        a.id = id;
        a.kind = kind;
        std::array<int, kKitMatCount> slot;
        slot.fill(-1);
        for (int m = 0; m < kKitMatCount; ++m) {
            bool used = false;
            for (const auto& lod : m_lod) used = used || !lod[std::size_t(m)].empty();
            if (!used) continue;
            const KitMaterial& km = kMaterials[m];
            ModelMaterial mm;
            mm.name = km.name;
            if (km.layer) {
                mm.albedo = std::string(km.layer) + "_albedo.ktx2";
                mm.nra = std::string(km.layer) + "_nra.ktx2";
            }
            mm.tint = {km.tint[0], km.tint[1], km.tint[2]};
            mm.roughness = km.roughness;
            slot[std::size_t(m)] = int(a.materials.size());
            a.materials.push_back(mm);
        }
        V lo(1e30f, 1e30f, 1e30f), hi(-1e30f, -1e30f, -1e30f);
        for (const auto& lodLists : m_lod) {
            ModelLod lod;
            for (int m = 0; m < kKitMatCount; ++m) {
                const auto& list = lodLists[std::size_t(m)];
                if (list.empty()) continue;
                ModelPrimitive p;
                p.firstIndex = std::uint32_t(lod.indices.size());
                p.indexCount = std::uint32_t(list.size());
                p.material = slot[std::size_t(m)];
                for (const ModelVertex& v : list) {
                    lod.indices.push_back(std::uint32_t(lod.vertices.size()));
                    lod.vertices.push_back(v);
                    lo = V(std::min(lo.x, v.pos[0]), std::min(lo.y, v.pos[1]), std::min(lo.z, v.pos[2]));
                    hi = V(std::max(hi.x, v.pos[0]), std::max(hi.y, v.pos[1]), std::max(hi.z, v.pos[2]));
                }
                lod.primitives.push_back(p);
            }
            a.lods.push_back(std::move(lod));
        }
        if (lo.x > hi.x) lo = hi = V(0, 0, 0);
        a.boundsMin = {lo.x, lo.y, lo.z};
        a.boundsMax = {hi.x, hi.y, hi.z};
        a.height = hi.y;
        a.radius = std::max({std::abs(lo.x), std::abs(hi.x), std::abs(lo.z), std::abs(hi.z)});
        a.lodError = {0.0f, kKitTrim, kKitTrim};
    }

private:
    BuildingGeometry& g;
    bool m_render;
    std::array<std::array<std::vector<ModelVertex>, kKitMatCount>, 3> m_lod;
};

// ---------------------------------------------------------------------------------------------- walls and openings
// A wall plane: point o at u = 0 on the floor level (y = 0), horizontal unit u along the wall (to the right seen
// from outside), outward normal n. P(u, y, w): w > 0 in front of the plane.
struct Wall {
    V o, u, n;
    float len;
    V P(float uu, float y, float w) const { return o + u * uu + V(0, y, 0) + n * w; }
};

Wall wallOf(V o, V n, float length) { return {o, norm(cross(kY, n)), n, length}; }

// Box in wall coordinates (u0..u1 along the wall, y0..y1, w0..w1 out of the plane).
void wbox(Builder& b, KitMat m, std::uint8_t lods, bool solid, const Wall& f, float u0, float u1, float y0, float y1,
          float w0, float w1, int faces = FALL) {
    const V c = f.P(0.5f * (u0 + u1), 0.5f * (y0 + y1), 0.5f * (w0 + w1));
    b.box(m, lods, solid, c, f.u, kY, f.n, V(0.5f * (u1 - u0), 0.5f * (y1 - y0), 0.5f * (w1 - w0)), faces);
}

enum class Op : std::uint8_t { Window, Door, BalconyDoor, ShopWindow, RollerDoor, Small };
constexpr int kDecoShutters = 1, kDecoFlowers = 2, kDecoMullions = 4;

struct Opening {
    float u0, u1, v0, v1;
    Op type = Op::Window;
    int deco = 0;
};

struct FacadeStyle {
    KitMat wall = KitMat::Plaster;
    KitMat frame = KitMat::PaintWhite;
    KitMat sill = KitMat::Stone;
    KitMat shutter = KitMat::Count; // Count: none
    KitMat door = KitMat::WoodDark;
};

float recessOf(Op t) {
    switch (t) {
    case Op::ShopWindow: return 0.10f;
    case Op::Small: return 0.10f;
    default: return kKitRecess;
    }
}

// Near LODs: the wall face between y0 and y1 cut by the openings, each opening a recess with its reveals and the
// glass / door at the back, frames, sills, shutters and flower boxes. Far LOD: glass and door panels just in front
// of the plane (the plane itself is the solid wall block, drawn by the caller).
void facade(Builder& b, const Wall& f, float y0, float y1, const std::vector<Opening>& ops, const FacadeStyle& st) {
    std::vector<float> xs{0.0f, f.len}, ys{y0, y1};
    for (const Opening& o : ops) {
        xs.push_back(o.u0), xs.push_back(o.u1);
        ys.push_back(std::clamp(o.v0, y0, y1)), ys.push_back(std::clamp(o.v1, y0, y1));
    }
    std::sort(xs.begin(), xs.end());
    xs.erase(std::unique(xs.begin(), xs.end(), [](float a, float c) { return std::abs(a - c) < 1e-4f; }), xs.end());
    std::sort(ys.begin(), ys.end());
    ys.erase(std::unique(ys.begin(), ys.end(), [](float a, float c) { return std::abs(a - c) < 1e-4f; }), ys.end());
    auto open = [&](float u, float y) {
        for (const Opening& o : ops)
            if (u > o.u0 && u < o.u1 && y > o.v0 && y < o.v1) return true;
        return false;
    };
    for (std::size_t j = 0; j + 1 < ys.size(); ++j) {
        std::size_t i = 0;
        while (i + 1 < xs.size()) {
            const float yc = 0.5f * (ys[j] + ys[j + 1]);
            if (open(0.5f * (xs[i] + xs[i + 1]), yc)) {
                ++i;
                continue;
            }
            std::size_t e = i + 1;
            while (e + 1 < xs.size() && !open(0.5f * (xs[e] + xs[e + 1]), yc)) ++e;
            b.quad(st.wall, NEAR, f.P(xs[i], ys[j], 0), f.P(xs[e], ys[j], 0), f.P(xs[e], ys[j + 1], 0), f.P(xs[i], ys[j + 1], 0), f.n);
            i = e;
        }
    }
    for (const Opening& o : ops) {
        const float r = recessOf(o.type);
        const float u0 = o.u0, u1 = o.u1, v0 = o.v0, v1 = o.v1;
        // Reveals (wall material) and the back panel.
        b.quad(st.wall, NEAR, f.P(u0, v0, 0), f.P(u1, v0, 0), f.P(u1, v0, -r), f.P(u0, v0, -r), kY);
        b.quad(st.wall, NEAR, f.P(u0, v1, 0), f.P(u1, v1, 0), f.P(u1, v1, -r), f.P(u0, v1, -r), -kY);
        b.quad(st.wall, NEAR, f.P(u0, v0, 0), f.P(u0, v1, 0), f.P(u0, v1, -r), f.P(u0, v0, -r), f.u);
        b.quad(st.wall, NEAR, f.P(u1, v0, 0), f.P(u1, v1, 0), f.P(u1, v1, -r), f.P(u1, v0, -r), -f.u);
        const bool door = o.type == Op::Door || o.type == Op::RollerDoor;
        const KitMat back = o.type == Op::Door ? st.door : o.type == Op::RollerDoor ? KitMat::CorrugatedGrey : KitMat::Glass;
        b.quad(back, NEAR, f.P(u0, v0, -r), f.P(u1, v0, -r), f.P(u1, v1, -r), f.P(u0, v1, -r), f.n);
        b.quad(back, FAR, f.P(u0, v0, 0.012f), f.P(u1, v0, 0.012f), f.P(u1, v1, 0.012f), f.P(u0, v1, 0.012f), f.n);
        const float fw = o.type == Op::ShopWindow ? 0.05f : 0.07f; // frame width
        const KitMat frame = o.type == Op::ShopWindow ? KitMat::MetalDark : st.frame;
        if (o.type == Op::Window || o.type == Op::BalconyDoor || o.type == Op::ShopWindow || o.type == Op::Small) {
            // Frame ring at the back of the recess, mullions, sill.
            const float wb = -r, wf = -r + 0.08f;
            wbox(b, frame, NEAR, false, f, u0, u0 + fw, v0, v1, wb, wf, FALL & ~FNZ);
            wbox(b, frame, NEAR, false, f, u1 - fw, u1, v0, v1, wb, wf, FALL & ~FNZ);
            wbox(b, frame, NEAR, false, f, u0 + fw, u1 - fw, v1 - fw, v1, wb, wf, FALL & ~FNZ);
            wbox(b, frame, NEAR, false, f, u0 + fw, u1 - fw, v0, v0 + fw, wb, wf, FALL & ~FNZ);
            if (o.deco & kDecoMullions) {
                const float uc = 0.5f * (u0 + u1), vc = o.type == Op::BalconyDoor ? v0 + 0.62f * (v1 - v0) : v0 + 0.6f * (v1 - v0);
                wbox(b, frame, L0, false, f, uc - 0.03f, uc + 0.03f, v0 + fw, v1 - fw, wb, wf - 0.02f, FALL & ~FNZ);
                wbox(b, frame, L0, false, f, u0 + fw, u1 - fw, vc - 0.03f, vc + 0.03f, wb, wf - 0.02f, FALL & ~FNZ);
            }
            if (o.type == Op::BalconyDoor) wbox(b, st.door, NEAR, false, f, u0 + fw, u1 - fw, v0 + fw, v0 + 0.45f * (v1 - v0), wb, wb + 0.04f, FALL & ~FNZ);
            if (o.type == Op::Window || o.type == Op::Small)
                wbox(b, st.sill, NEAR, false, f, u0 - 0.06f, u1 + 0.06f, v0 - 0.06f, v0, -r, 0.07f, FALL & ~FNZ);
        }
        if (door) {
            // Proud frame around the door.
            const float t = 0.10f;
            wbox(b, o.type == Op::RollerDoor ? KitMat::MetalDark : st.frame, NEAR, false, f, u0 - t, u0, v0, v1 + t, 0.0f, 0.05f, FALL & ~FNZ);
            wbox(b, o.type == Op::RollerDoor ? KitMat::MetalDark : st.frame, NEAR, false, f, u1, u1 + t, v0, v1 + t, 0.0f, 0.05f, FALL & ~FNZ);
            wbox(b, o.type == Op::RollerDoor ? KitMat::MetalDark : st.frame, NEAR, false, f, u0, u1, v1, v1 + t, 0.0f, 0.05f, FALL & ~FNZ);
            if (o.type == Op::Door) {
                // Panels on the door leaf.
                const float uc = 0.5f * (u0 + u1);
                wbox(b, KitMat::Wood, L0, false, f, u0 + 0.12f, uc - 0.05f, v0 + 0.25f, v0 + 0.95f, -r, -r + 0.03f, FALL & ~FNZ);
                wbox(b, KitMat::Wood, L0, false, f, uc + 0.05f, u1 - 0.12f, v0 + 0.25f, v0 + 0.95f, -r, -r + 0.03f, FALL & ~FNZ);
                wbox(b, KitMat::Glass, L0, false, f, u0 + 0.18f, u1 - 0.18f, v0 + 1.25f, v1 - 0.18f, -r, -r + 0.02f, FALL & ~FNZ);
            }
        }
        if ((o.deco & kDecoShutters) && st.shutter != KitMat::Count) {
            const float sw = 0.5f * (u1 - u0);
            for (int s = 0; s < 2; ++s) {
                const float a0 = s == 0 ? u0 - sw - 0.02f : u1 + 0.02f, a1 = s == 0 ? u0 - 0.02f : u1 + sw + 0.02f;
                wbox(b, st.shutter, NEAR, false, f, a0, a1, v0, v1, 0.0f, 0.045f, FALL & ~FNZ);
                // Louvre lines: thin battens across the leaf.
                for (int k = 1; k < 6; ++k) {
                    const float y = v0 + (v1 - v0) * float(k) / 6.0f;
                    wbox(b, st.shutter, L0, false, f, a0 + 0.04f, a1 - 0.04f, y - 0.02f, y + 0.02f, 0.045f, 0.065f, FALL & ~FNZ);
                }
            }
        }
        if (o.deco & kDecoFlowers) {
            wbox(b, KitMat::WoodDark, L0, false, f, u0 - 0.02f, u1 + 0.02f, v0 - 0.30f, v0 - 0.06f, 0.07f, 0.27f, FALL & ~FNZ);
            wbox(b, KitMat::Flowers, L0, false, f, u0 + 0.04f, u1 - 0.04f, v0 - 0.06f, v0 + 0.10f, 0.09f, 0.25f, FALL & ~FNZ);
            wbox(b, KitMat::Leaves, L0, false, f, u0 + 0.02f, u1 - 0.02f, v0 - 0.10f, v0 + 0.02f, 0.08f, 0.27f, FALL & ~FNZ);
        }
    }
}

// Evenly spaced windows along a wall (at least one), sill height above the band floor.
std::vector<Opening> windowRow(float length, float floorY, float sill, float width, float height, int deco, int minCount = 1,
                               bool odd = false, float spacing = 2.6f) {
    int n = std::max(minCount, int((length - 0.8f) / spacing));
    if (odd && n % 2 == 0) n = std::max(1, n - 1);
    std::vector<Opening> out;
    for (int k = 0; k < n; ++k) {
        const float c = length * (float(k) + 0.5f) / float(n);
        out.push_back({c - 0.5f * width, c + 0.5f * width, floorY + sill, floorY + sill + height, Op::Window, deco});
    }
    return out;
}

// Gable roof over a W (x) by D (z) block whose walls end at `top`, ridge along z. Front (+z) overhang `front`,
// back overhang `back`, eaves overhang `eave`. Returns the rise.
struct RoofStyle {
    KitMat roof = KitMat::RoofTiles, ridge = KitMat::RoofTiles, trim = KitMat::WoodDark, gable = KitMat::Siding;
    float pitch = 0.42f, thickness = 0.22f, eave = 0.75f, front = 0.7f, back = 0.6f;
    bool gutters = true, purlins = true;
};

float gableRoof(Builder& b, float W, float D, float top, const RoofStyle& rs) {
    const float tp = std::tan(rs.pitch), cp = std::cos(rs.pitch), sp = std::sin(rs.pitch);
    const float R = 0.5f * W * tp, t = rs.thickness;
    const float z0 = -0.5f * D - rs.back, z1 = 0.5f * D + rs.front, zc = 0.5f * (z0 + z1), lz = z1 - z0;
    // Gable infill triangles (front and back) between the wall tops and the roof underside.
    b.prism(rs.gable, ALL, true, V(-0.5f * W, top, 0.5f * D), V(0.5f * W, top, 0.5f * D), V(0, top + R, 0.5f * D), V(0, 0, -1), D, false);
    for (int s = -1; s <= 1; s += 2) {
        const float sx = float(s);
        const V sd(sx * cp, -sp, 0.0f), nn(sx * sp, cp, 0.0f);
        const float L = (0.5f * W + rs.eave) / cp;
        const V c = V(0, top + R, zc) + sd * (0.5f * L) + nn * (0.5f * t);
        b.box(rs.roof, ALL, true, c, sd, nn, kZ, V(0.5f * L, 0.5f * t, 0.5f * lz));
        const V eaveP = V(0, top + R, 0) + sd * L; // underside at the eave (z = 0)
        // Fascia board along the eave, barge boards along the gable edges.
        b.box(rs.trim, NEAR, false, V(eaveP.x, eaveP.y, zc) + nn * (0.5f * t) + sd * 0.03f, sd, nn, kZ, V(0.03f, 0.5f * t + 0.05f, 0.5f * lz + 0.03f));
        for (float ze : {z0 - 0.03f, z1 + 0.03f})
            b.box(rs.trim, NEAR, false, V(0, top + R, ze) + sd * (0.5f * L) + nn * (0.5f * t - 0.02f), sd, nn, kZ, V(0.5f * L, 0.5f * t + 0.05f, 0.03f));
        if (rs.gutters) {
            const V gp(eaveP.x - sx * 0.08f, eaveP.y - 0.06f, 0.0f);
            b.cyl(KitMat::MetalDark, L0, false, V(gp.x, gp.y, z0), V(gp.x, gp.y, z1), 0.07f, 0.07f, 6);
            // Downpipe: from the gutter back to the wall, then down the front corner.
            const V wallTop(sx * (0.5f * W + 0.08f), gp.y - 0.35f, 0.5f * D - 0.25f);
            b.cyl(KitMat::MetalDark, L0, false, V(gp.x, gp.y, wallTop.z), wallTop, 0.045f, 0.045f, 6, 0);
            b.cyl(KitMat::MetalDark, L0, false, wallTop, V(wallTop.x, 0.1f, wallTop.z), 0.045f, 0.045f, 6, 0);
        }
        if (rs.purlins)
            for (float fx : {0.25f, 0.5f})
                for (float ze : {0.5f * D, -0.5f * D}) {
                    const float x = sx * fx * W;
                    const float y = top + (0.5f * W - std::abs(x)) * tp - 0.08f;
                    const float zs = ze > 0.0f ? z1 - 0.05f : z0 + 0.05f;
                    b.abox(rs.trim, L0, false, V(x - 0.07f, y - 0.14f, std::min(ze, zs)), V(x + 0.07f, y, std::max(ze, zs)));
                }
    }
    // Ridge cap.
    b.abox(rs.ridge, ALL, true, V(-0.2f, top + R + t / cp - 0.08f, z0 - 0.02f), V(0.2f, top + R + t / cp + 0.08f, z1 + 0.02f));
    return R;
}

// Board railing along a wall: u0..u1 at distance w out of the plane, height 1 m above y.
void railing(Builder& b, const Wall& f, float u0, float u1, float y, float w, KitMat m) {
    wbox(b, m, MID, true, f, u0, u1, y, y + 1.0f, w - 0.07f, w);
    wbox(b, KitMat::WoodDark, L0, false, f, u0, u1, y + 0.92f, y + 1.04f, w - 0.10f, w + 0.02f);
    wbox(b, KitMat::WoodDark, L0, false, f, u0, u1, y + 0.10f, y + 0.20f, w - 0.09f, w + 0.01f);
    const int n = std::max(1, int((u1 - u0) / 0.16f));
    for (int k = 0; k < n; ++k) {
        const float c = u0 + (u1 - u0) * (float(k) + 0.5f) / float(n);
        wbox(b, m, L0, false, f, c - 0.05f, c + 0.05f, y + 0.08f, y + 0.94f, w - 0.07f, w - 0.02f);
    }
}

// Balcony along a wall at floor height y: deck reaching `depth` out, u0..u1, railing on the outer edge and both ends.
void balcony(Builder& b, const Wall& f, float u0, float u1, float y, float depth) {
    wbox(b, KitMat::Wood, ALL, true, f, u0, u1, y - 0.18f, y, 0.0f, depth);
    railing(b, f, u0, u1, y, depth, KitMat::Wood);
    // End railings, in the planes across the deck ends.
    railing(b, Wall{f.P(u0, 0, 0), f.n, -f.u, depth}, 0.0f, depth - 0.07f, y, 0.07f, KitMat::Wood);
    railing(b, Wall{f.P(u1, 0, 0), f.n, f.u, depth}, 0.0f, depth - 0.07f, y, 0.07f, KitMat::Wood);
    const int brackets = std::max(2, int((u1 - u0) / 2.5f) + 1);
    for (int k = 0; k < brackets; ++k) {
        const float c = u0 + 0.15f + (u1 - u0 - 0.3f) * float(k) / float(brackets - 1);
        b.beam(KitMat::WoodDark, L0, false, f.P(c, y - 1.0f, 0.02f), f.P(c, y - 0.18f, depth - 0.15f), 0.06f, 0.06f);
    }
}

void chimney(Builder& b, float x, float z, float y0, float y1, KitMat m) {
    b.abox(m, ALL, true, V(x - 0.32f, y0, z - 0.32f), V(x + 0.32f, y1, z + 0.32f), FALL & ~FNY);
    b.abox(KitMat::Concrete, ALL, true, V(x - 0.40f, y1, z - 0.40f), V(x + 0.40f, y1 + 0.08f, z + 0.40f));
    b.abox(KitMat::MetalDark, NEAR, false, V(x - 0.10f, y1 + 0.08f, z - 0.10f), V(x + 0.10f, y1 + 0.30f, z + 0.10f));
}

// --------------------------------------------------------------------------------------------------------- houses
void house(Builder& b, const BuildingSpec& s) {
    const float W = std::max(4.0f, s.w), D = std::max(4.0f, s.d);
    const int F = std::clamp(s.floors, 1, 3);
    const int v = s.variant;
    constexpr float fh = 2.75f;
    const float top = float(F) * fh;
    const KitMat plaster = (v & kHouseOchre) ? KitMat::PlasterOchre : KitMat::PlasterWhite;
    const KitMat ground = (v & kHouseTimber) ? KitMat::Siding : (v & kHouseStoneBase) ? KitMat::Stone : plaster;
    const KitMat upper = (v & kHouseTimber) ? KitMat::Siding : (v & kHouseUpperPlaster) ? plaster : KitMat::Siding;
    const int sh = (v >> kHouseShutterShift) & 3;
    FacadeStyle st;
    st.shutter = sh == 0 ? KitMat::PaintGreen : sh == 1 ? KitMat::PaintRed : sh == 2 ? KitMat::WoodDark : KitMat::Count;
    st.frame = upper == KitMat::Siding && ground == KitMat::Siding ? KitMat::WoodDark : KitMat::PaintWhite;
    st.sill = ground == KitMat::Stone ? KitMat::Stone : KitMat::WoodDark;
    const bool frontBalcony = (v & kHouseBalcony) && F >= 2;
    const bool sideBalcony = (v & kHouseSideBalcony) && F >= 2;
    const float plinth = buildingPlinthDepth(s.kind);

    // Plinth (base course) and one solid block per floor band.
    b.abox(KitMat::Stone, ALL, true, V(-0.5f * W - 0.08f, -plinth, -0.5f * D - 0.08f), V(0.5f * W + 0.08f, 0.3f, 0.5f * D + 0.08f), FALL & ~FNY);
    const Wall walls[4] = {wallOf(V(-0.5f * W, 0, 0.5f * D), kZ, W), wallOf(V(0.5f * W, 0, 0.5f * D), kX, D),
                           wallOf(V(0.5f * W, 0, -0.5f * D), -kZ, W), wallOf(V(-0.5f * W, 0, -0.5f * D), -kX, D)};
    for (int k = 0; k < F; ++k) {
        const float y0 = k == 0 ? 0.3f : float(k) * fh, y1 = float(k + 1) * fh;
        st.wall = k == 0 ? ground : upper;
        b.abox(st.wall, FAR, true, V(-0.5f * W, y0, -0.5f * D), V(0.5f * W, y1, 0.5f * D), FSIDES);
        for (int side = 0; side < 4; ++side) {
            const Wall& f = walls[side];
            const int deco = kDecoShutters | kDecoMullions | (k > 0 && side != 2 ? kDecoFlowers : 0);
            const float floorY = float(k) * fh;
            std::vector<Opening> ops = windowRow(f.len, floorY, 0.95f, 1.0f, 1.3f, deco, 1, side == 0, side % 2 == 0 ? 2.0f : 2.6f);
            if (side == 0 && k == 0) {
                // Front door in the middle (replaces the middle window).
                const float c = 0.5f * f.len;
                ops.erase(std::remove_if(ops.begin(), ops.end(), [&](const Opening& o) { return o.u0 < c + 1.2f && o.u1 > c - 1.2f; }), ops.end());
                ops.push_back({c - 0.55f, c + 0.55f, 0.3f, 2.45f, Op::Door, 0});
            }
            if (side == 0 && k == 1 && frontBalcony) {
                const float c = 0.5f * f.len;
                ops.erase(std::remove_if(ops.begin(), ops.end(), [&](const Opening& o) { return o.u0 < c + 1.1f && o.u1 > c - 1.1f; }), ops.end());
                ops.push_back({c - 0.5f, c + 0.5f, floorY + 0.05f, floorY + 2.25f, Op::BalconyDoor, kDecoMullions});
                for (Opening& o : ops) o.deco &= ~kDecoFlowers;
            }
            if (side == 1 && k == 1 && sideBalcony) {
                const float c = 0.5f * f.len;
                ops.erase(std::remove_if(ops.begin(), ops.end(), [&](const Opening& o) { return o.u0 < c + 1.1f && o.u1 > c - 1.1f; }), ops.end());
                ops.push_back({c - 0.5f, c + 0.5f, floorY + 0.05f, floorY + 2.25f, Op::BalconyDoor, kDecoMullions});
                for (Opening& o : ops) o.deco &= ~kDecoFlowers;
            }
            facade(b, f, y0, y1, ops, st);
        }
        if (k > 0) // floor band trim
            for (const Wall& f : walls) wbox(b, KitMat::WoodDark, NEAR, false, f, -0.05f, f.len + 0.05f, y0 - 0.06f, y0 + 0.08f, 0.0f, 0.05f);
    }
    // Door step and canopy (solid: they stand out more than a trim).
    b.abox(KitMat::Stone, ALL, true, V(-0.8f, -0.6f, 0.5f * D), V(0.8f, 0.28f, 0.5f * D + 0.55f), FALL & ~FNY);
    if (!frontBalcony) {
        b.abox(KitMat::WoodDark, ALL, true, V(-0.95f, 2.75f - 0.18f, 0.5f * D), V(0.95f, 2.75f - 0.06f, 0.5f * D + 0.85f));
        b.beam(KitMat::WoodDark, L0, false, V(-0.8f, 2.0f, 0.5f * D + 0.02f), V(-0.8f, 2.6f, 0.5f * D + 0.7f), 0.05f, 0.05f);
        b.beam(KitMat::WoodDark, L0, false, V(0.8f, 2.0f, 0.5f * D + 0.02f), V(0.8f, 2.6f, 0.5f * D + 0.7f), 0.05f, 0.05f);
    }
    if (frontBalcony) balcony(b, walls[0], -0.25f, W + 0.25f, fh, 1.35f);
    if (sideBalcony) balcony(b, walls[1], 0.6f, D - 0.6f, fh, 1.2f);

    RoofStyle rs;
    rs.roof = (v & kHouseShingles) ? KitMat::Shingles : KitMat::RoofTiles;
    rs.ridge = (v & kHouseShingles) ? KitMat::WoodDark : KitMat::RoofTiles;
    rs.gable = upper;
    rs.pitch = 0.40f + 0.05f * float((v >> 1) & 1);
    rs.front = frontBalcony ? 1.55f : 0.75f;
    rs.eave = sideBalcony ? 1.4f : 0.8f;
    const float R = gableRoof(b, W, D, top, rs);
    // Attic window in both gables.
    if (R > 1.5f)
        for (int side : {0, 2}) {
            const Wall& f = walls[side];
            const float c = 0.5f * f.len, y = top + 0.35f;
            wbox(b, KitMat::Glass, ALL, false, f, c - 0.4f, c + 0.4f, y, y + 0.9f, 0.0f, 0.02f, FPZ);
            wbox(b, st.frame, NEAR, false, f, c - 0.47f, c + 0.47f, y - 0.07f, y, 0.0f, 0.07f);
            wbox(b, st.frame, NEAR, false, f, c - 0.47f, c + 0.47f, y + 0.9f, y + 0.97f, 0.0f, 0.07f);
            wbox(b, st.frame, NEAR, false, f, c - 0.47f, c - 0.4f, y, y + 0.9f, 0.0f, 0.07f);
            wbox(b, st.frame, NEAR, false, f, c + 0.4f, c + 0.47f, y, y + 0.9f, 0.0f, 0.07f);
            wbox(b, st.frame, L0, false, f, c - 0.03f, c + 0.03f, y, y + 0.9f, 0.0f, 0.05f);
        }
    const float cx = 0.22f * W, cz = -0.18f * D;
    chimney(b, cx, cz, top + (0.5f * W - cx) * std::tan(rs.pitch) - 0.3f, top + R + 0.9f, ground == KitMat::Stone ? KitMat::Stone : plaster);
}

// -------------------------------------------------------------------------------------------------- fuel station
void fuelCanopy(Builder& b, const BuildingSpec& s) {
    const float W = s.w, D = s.d, H = s.h > 0.0f ? s.h : 4.8f;
    const float plinth = buildingPlinthDepth(s.kind);
    b.abox(KitMat::Concrete, ALL, true, V(-0.5f * W - 2.0f, -plinth, -0.5f * D - 2.0f), V(0.5f * W + 2.0f, 0.12f, 0.5f * D + 2.0f), FALL & ~FNY);
    for (float sx : {-1.0f, 1.0f})
        for (float sz : {-1.0f, 1.0f})
            b.cyl(KitMat::MetalWhite, ALL, true, V(sx * (0.5f * W - 1.2f), 0.12f, sz * (0.5f * D - 1.0f)), V(sx * (0.5f * W - 1.2f), H - 0.5f, sz * (0.5f * D - 1.0f)), 0.2f, 0.2f, 10);
    b.abox(KitMat::MetalWhite, ALL, true, V(-0.5f * W, H - 0.7f, -0.5f * D), V(0.5f * W, H, 0.5f * D));
    b.abox(KitMat::MetalRed, ALL, false, V(-0.5f * W - 0.02f, H - 0.55f, -0.5f * D - 0.02f), V(0.5f * W + 0.02f, H - 0.2f, 0.5f * D + 0.02f), FSIDES);
    for (int k = 0; k < 3; ++k) {
        const float x = -0.5f * W + W * (float(k) + 0.5f) / 3.0f;
        b.quad(KitMat::Lamp, NEAR, V(x - 0.6f, H - 0.71f, -0.3f * D), V(x + 0.6f, H - 0.71f, -0.3f * D), V(x + 0.6f, H - 0.71f, 0.3f * D), V(x - 0.6f, H - 0.71f, 0.3f * D), -kY);
    }
    for (float ix : {-0.22f * W, 0.22f * W}) {
        b.abox(KitMat::Concrete, ALL, true, V(ix - 0.6f, 0.12f, -0.36f * D), V(ix + 0.6f, 0.3f, 0.36f * D), FALL & ~FNY);
        for (float bz : {-0.36f * D + 0.2f, 0.36f * D - 0.2f})
            b.cyl(KitMat::MetalRed, ALL, true, V(ix, 0.3f, bz), V(ix, 1.2f, bz), 0.1f, 0.1f, 8);
        for (float pz : {-0.15f * D, 0.15f * D}) {
            b.abox(KitMat::MetalWhite, ALL, true, V(ix - 0.35f, 0.3f, pz - 0.28f), V(ix + 0.35f, 1.95f, pz + 0.28f));
            b.abox(KitMat::MetalRed, ALL, true, V(ix - 0.38f, 1.7f, pz - 0.31f), V(ix + 0.38f, 2.05f, pz + 0.31f));
            for (float sx : {-1.0f, 1.0f}) {
                const float fx = ix + sx * 0.355f;
                b.quad(KitMat::Glass, NEAR, V(fx, 1.25f, pz - 0.18f), V(fx, 1.25f, pz + 0.18f), V(fx, 1.55f, pz + 0.18f), V(fx, 1.55f, pz - 0.18f), V(sx, 0, 0));
                b.abox(KitMat::MetalDark, L0, false, V(fx - 0.04f, 0.9f, pz + 0.05f), V(fx + 0.04f, 1.1f, pz + 0.17f));
                b.beam(KitMat::MetalDark, L0, false, V(fx, 1.0f, pz + 0.11f), V(fx + sx * 0.12f, 0.45f, pz + 0.2f), 0.025f, 0.025f);
            }
        }
    }
}

void flatRoofBlock(Builder& b, float W, float D, float H, KitMat wall, KitMat roofEdge, const std::vector<Opening> ops[4],
                   const FacadeStyle& stIn) {
    FacadeStyle st = stIn;
    st.wall = wall;
    b.abox(wall, FAR, true, V(-0.5f * W, 0.25f, -0.5f * D), V(0.5f * W, H, 0.5f * D), FSIDES);
    const Wall walls[4] = {wallOf(V(-0.5f * W, 0, 0.5f * D), kZ, W), wallOf(V(0.5f * W, 0, 0.5f * D), kX, D),
                           wallOf(V(0.5f * W, 0, -0.5f * D), -kZ, W), wallOf(V(-0.5f * W, 0, -0.5f * D), -kX, D)};
    for (int side = 0; side < 4; ++side) facade(b, walls[side], 0.25f, H, ops[side], st);
    b.abox(roofEdge, ALL, true, V(-0.5f * W - 0.25f, H, -0.5f * D - 0.25f), V(0.5f * W + 0.25f, H + 0.45f, 0.5f * D + 0.25f));
}

void fuelShop(Builder& b, const BuildingSpec& s) {
    const float W = s.w, D = s.d, H = s.h > 0.0f ? s.h : 3.4f;
    b.abox(KitMat::Concrete, ALL, true, V(-0.5f * W - 0.1f, -buildingPlinthDepth(s.kind), -0.5f * D - 0.1f), V(0.5f * W + 0.1f, 0.25f, 0.5f * D + 0.1f), FALL & ~FNY);
    std::vector<Opening> ops[4];
    const float c = 0.5f * W;
    ops[0].push_back({0.5f, c - 0.75f, 0.7f, 2.6f, Op::ShopWindow, 0});
    ops[0].push_back({c + 0.75f, W - 0.5f, 0.7f, 2.6f, Op::ShopWindow, 0});
    ops[0].push_back({c - 0.55f, c + 0.55f, 0.25f, 2.6f, Op::ShopWindow, 0});
    ops[1] = windowRow(D, 0.0f, 1.1f, 1.2f, 1.1f, 0, 1);
    ops[2].push_back({0.5f * W - 0.5f, 0.5f * W + 0.5f, 0.25f, 2.3f, Op::Door, 0});
    ops[3] = windowRow(D, 0.0f, 1.1f, 1.2f, 1.1f, 0, 1);
    FacadeStyle st;
    st.frame = KitMat::MetalDark;
    st.door = KitMat::MetalDark;
    flatRoofBlock(b, W, D, H, KitMat::PlasterWhite, KitMat::MetalRed, ops, st);
    // Sign on the roof edge, awning over the glass front, roof unit.
    b.abox(KitMat::MetalWhite, ALL, true, V(-0.32f * W, H + 0.45f, 0.5f * D - 0.1f), V(0.32f * W, H + 1.25f, 0.5f * D + 0.15f));
    b.abox(KitMat::MetalRed, ALL, false, V(-0.30f * W, H + 0.62f, 0.5f * D + 0.15f), V(0.30f * W, H + 0.80f, 0.5f * D + 0.17f), FPZ);
    b.abox(KitMat::MetalRed, ALL, false, V(-0.30f * W, H + 0.92f, 0.5f * D + 0.15f), V(0.30f * W, H + 1.08f, 0.5f * D + 0.17f), FPZ);
    b.abox(KitMat::MetalRed, ALL, true, V(-0.5f * W, 2.75f, 0.5f * D), V(0.5f * W, 2.9f, 0.5f * D + 1.1f));
    b.abox(KitMat::MetalWhite, ALL, true, V(0.1f * W, H + 0.45f, -0.4f * D), V(0.1f * W + 1.1f, H + 1.15f, -0.4f * D + 0.8f));
}

void pylon(Builder& b, const BuildingSpec& s) {
    const float H = s.h > 0.0f ? s.h : 6.5f;
    for (float x : {-0.8f, 0.8f}) b.cyl(KitMat::MetalDark, ALL, true, V(x, -buildingPlinthDepth(s.kind), 0), V(x, H - 0.3f, 0), 0.1f, 0.1f, 8);
    b.abox(KitMat::MetalRed, ALL, true, V(-1.2f, H - 2.6f, -0.2f), V(1.2f, H, 0.2f));
    for (int k = 0; k < 3; ++k) {
        const float y = H - 2.4f + 0.75f * float(k);
        for (float sz : {-1.0f, 1.0f})
            b.quad(KitMat::MetalWhite, ALL, V(-1.0f, y, sz * 0.21f), V(1.0f, y, sz * 0.21f), V(1.0f, y + 0.55f, sz * 0.21f), V(-1.0f, y + 0.55f, sz * 0.21f), V(0, 0, sz));
    }
}

// ------------------------------------------------------------------------------------------------- garage / sheds
void garage(Builder& b, const BuildingSpec& s) {
    const float W = s.w, D = s.d, H = s.h > 0.0f ? s.h : 4.4f;
    const bool rusty = s.variant == 1;
    const KitMat wall = rusty ? KitMat::CorrugatedRed : KitMat::Corrugated;
    b.abox(KitMat::Concrete, ALL, true, V(-0.5f * W - 0.1f, -buildingPlinthDepth(s.kind), -0.5f * D - 0.1f), V(0.5f * W + 0.1f, 0.3f, 0.5f * D + 0.1f), FALL & ~FNY);
    b.abox(wall, FAR, true, V(-0.5f * W, 0.3f, -0.5f * D), V(0.5f * W, H, 0.5f * D), FSIDES);
    const Wall walls[4] = {wallOf(V(-0.5f * W, 0, 0.5f * D), kZ, W), wallOf(V(0.5f * W, 0, 0.5f * D), kX, D),
                           wallOf(V(0.5f * W, 0, -0.5f * D), -kZ, W), wallOf(V(-0.5f * W, 0, -0.5f * D), -kX, D)};
    FacadeStyle st;
    st.wall = wall;
    st.frame = KitMat::MetalDark;
    st.sill = KitMat::MetalDark;
    st.door = KitMat::MetalDark;
    const float dw = std::min(4.4f, W - 3.0f), c = 0.42f * W;
    std::vector<Opening> ops[4];
    ops[0].push_back({c - 0.5f * dw, c + 0.5f * dw, 0.3f, 3.8f, Op::RollerDoor, 0});
    ops[0].push_back({W - 1.6f, W - 0.65f, 0.3f, 2.4f, Op::Door, 0});
    ops[1] = windowRow(D, 0.0f, 2.1f, 1.6f, 0.9f, 0, 1, false, 3.0f);
    ops[3] = windowRow(D, 0.0f, 2.1f, 1.6f, 0.9f, 0, 1, false, 3.0f);
    for (Opening& o : ops[1]) o.type = Op::Small;
    for (Opening& o : ops[3]) o.type = Op::Small;
    for (int side = 0; side < 4; ++side) facade(b, walls[side], 0.3f, H, ops[side], st);
    // Corner flashings.
    for (float sx : {-1.0f, 1.0f})
        for (float sz : {-1.0f, 1.0f})
            b.abox(KitMat::MetalDark, NEAR, false, V(sx * 0.5f * W - 0.06f, 0.3f, sz * 0.5f * D - 0.06f), V(sx * 0.5f * W + 0.06f, H, sz * 0.5f * D + 0.06f));
    RoofStyle rs;
    rs.roof = rusty ? KitMat::Rust : KitMat::CorrugatedGrey;
    rs.ridge = KitMat::MetalDark;
    rs.trim = KitMat::MetalDark;
    rs.gable = wall;
    rs.pitch = 0.2f;
    rs.thickness = 0.14f;
    rs.eave = 0.4f;
    rs.front = 0.35f;
    rs.back = 0.35f;
    rs.purlins = false;
    gableRoof(b, W, D, H, rs);
    if (!rusty) {
        // Sign board over the roller door.
        wbox(b, KitMat::MetalWhite, ALL, true, walls[0], c - 2.4f, c + 2.4f, H - 0.15f, H + 0.75f, 0.0f, 0.12f);
        wbox(b, KitMat::MetalRed, ALL, false, walls[0], c - 2.3f, c + 2.3f, H + 0.1f, H + 0.45f, 0.12f, 0.14f, FPZ);
    }
}

// ------------------------------------------------------------------------------------------------------ lighthouse
void lighthouse(Builder& b, const BuildingSpec& s) {
    const float H = s.h > 0.0f ? s.h : 18.0f;
    const float r0 = 2.6f, r1 = 1.9f;
    auto radius = [&](float y) { return r0 + (r1 - r0) * std::clamp(y / H, 0.0f, 1.0f); };
    b.cyl(KitMat::Stone, ALL, true, V(0, -buildingPlinthDepth(s.kind), 0), V(0, 0.6f, 0), 3.4f, 3.4f, 8, 2, kPi / 8.0f);
    constexpr int kBands = 5;
    for (int k = 0; k < kBands; ++k) {
        const float y0 = 0.6f + (H - 0.6f) * float(k) / kBands, y1 = 0.6f + (H - 0.6f) * float(k + 1) / kBands;
        b.cyl(k % 2 ? KitMat::PlasterRed : KitMat::PlasterWhite, ALL, true, V(0, y0, 0), V(0, y1, 0), radius(y0), radius(y1), 20, 2);
    }
    // Door and small windows on the tower face (proud panels with frames).
    const float rd = radius(1.5f);
    b.abox(KitMat::WoodDark, ALL, true, V(-0.55f, 0.6f, rd - 0.15f), V(0.55f, 2.8f, rd + 0.04f), FPZ | FPX | FNX | FPY);
    b.abox(KitMat::PaintWhite, NEAR, false, V(-0.68f, 0.6f, rd - 0.15f), V(0.68f, 2.95f, rd + 0.02f), FPZ | FPX | FNX | FPY);
    for (int k = 0; k < 3; ++k) {
        const float y = 5.5f + 4.2f * float(k), a = 0.9f * float(k) + 0.3f, r = radius(y);
        const V n(std::sin(a), 0, std::cos(a));
        const Wall f = wallOf(V(0, 0, 0) + n * (r - 0.05f) + norm(cross(kY, n)) * -0.35f, n, 0.7f);
        wbox(b, KitMat::Glass, ALL, true, f, 0.0f, 0.7f, y, y + 1.0f, 0.0f, 0.09f, FALL & ~FNZ);
        wbox(b, KitMat::PaintWhite, NEAR, false, f, -0.08f, 0.78f, y - 0.08f, y, 0.0f, 0.13f);
        wbox(b, KitMat::PaintWhite, NEAR, false, f, -0.08f, 0.78f, y + 1.0f, y + 1.08f, 0.0f, 0.13f);
    }
    // Gallery, railing, lantern room, roof.
    b.cyl(KitMat::MetalDark, ALL, true, V(0, H, 0), V(0, H + 0.25f, 0), 2.75f, 2.75f, 20);
    constexpr int kPosts = 20;
    for (int k = 0; k < kPosts; ++k) {
        const float a0 = 2.0f * kPi * float(k) / kPosts, a1 = 2.0f * kPi * float(k + 1) / kPosts;
        const V p0(2.6f * std::cos(a0), 0, 2.6f * std::sin(a0)), p1(2.6f * std::cos(a1), 0, 2.6f * std::sin(a1));
        b.beam(KitMat::MetalDark, L0, false, p0 + V(0, H + 0.25f, 0), p0 + V(0, H + 1.3f, 0), 0.03f, 0.03f);
        b.beam(KitMat::MetalDark, NEAR, false, p0 + V(0, H + 1.3f, 0), p1 + V(0, H + 1.3f, 0), 0.035f, 0.035f);
        b.beam(KitMat::MetalDark, L0, false, p0 + V(0, H + 0.75f, 0), p1 + V(0, H + 0.75f, 0), 0.02f, 0.02f);
    }
    b.cyl(KitMat::MetalDark, ALL, true, V(0, H + 0.25f, 0), V(0, H + 0.85f, 0), 1.45f, 1.45f, 16);
    b.cyl(KitMat::Glass, ALL, true, V(0, H + 0.85f, 0), V(0, H + 2.6f, 0), 1.35f, 1.35f, 16, 0);
    b.cyl(KitMat::Lamp, NEAR, false, V(0, H + 1.2f, 0), V(0, H + 2.1f, 0), 0.45f, 0.45f, 12);
    for (int k = 0; k < 8; ++k) {
        const float a = 2.0f * kPi * float(k) / 8.0f;
        const V p(1.38f * std::cos(a), 0, 1.38f * std::sin(a));
        b.beam(KitMat::MetalDark, NEAR, false, p + V(0, H + 0.85f, 0), p + V(0, H + 2.6f, 0), 0.04f, 0.04f);
    }
    b.cyl(KitMat::MetalRed, ALL, true, V(0, H + 2.6f, 0), V(0, H + 3.7f, 0), 1.65f, 0.15f, 16, 1);
    b.cyl(KitMat::MetalDark, NEAR, false, V(0, H + 3.7f, 0), V(0, H + 4.3f, 0), 0.06f, 0.06f, 6);
}

// ------------------------------------------------------------------------------------------------------- radio
void radioMast(Builder& b, const BuildingSpec& s) {
    const float H = s.h > 0.0f ? s.h : 34.0f;
    constexpr int kSections = 8;
    const float h0 = 2.0f, h1 = 0.45f;
    auto corner = [&](int c, float y) {
        const float hw = h0 + (h1 - h0) * y / H;
        const float sx = (c == 0 || c == 3) ? -1.0f : 1.0f, sz = (c < 2) ? -1.0f : 1.0f;
        return V(sx * hw, y, sz * hw);
    };
    for (int c = 0; c < 4; ++c) {
        const V p = corner(c, 0.0f);
        b.abox(KitMat::Concrete, ALL, true, V(p.x - 0.45f, -buildingPlinthDepth(s.kind), p.z - 0.45f), V(p.x + 0.45f, 0.35f, p.z + 0.45f), FALL & ~FNY);
    }
    for (int k = 0; k < kSections; ++k) {
        const float y0 = H * float(k) / kSections, y1 = H * float(k + 1) / kSections;
        const KitMat m = k % 2 ? KitMat::MetalRed : KitMat::MetalWhite;
        for (int c = 0; c < 4; ++c) {
            b.beam(m, ALL, true, corner(c, y0), corner(c, y1), 0.09f, 0.09f);
            const int n = (c + 1) % 4;
            b.beam(m, ALL, true, corner(c, y1), corner(n, y1), 0.05f, 0.05f);
            b.beam(m, MID, true, corner(c, y0), corner(n, y1), 0.035f, 0.035f);
            b.beam(m, MID, true, corner(n, y0), corner(c, y1), 0.035f, 0.035f);
        }
    }
    // Antennas (out of reach: no collision): pole, dishes, panels, beacon.
    b.cyl(KitMat::MetalWhite, ALL, false, V(0, H, 0), V(0, H + 4.5f, 0), 0.09f, 0.06f, 8);
    for (float sx : {-1.0f, 1.0f}) {
        b.cyl(KitMat::MetalWhite, ALL, false, V(sx * 0.55f, H - 3.0f, 0), V(sx * 0.85f, H - 3.0f, 0), 0.75f, 0.55f, 14);
        b.abox(KitMat::MetalWhite, NEAR, false, V(-0.15f, H - 6.0f, sx * 0.62f - 0.08f), V(0.15f, H - 4.2f, sx * 0.62f + 0.08f));
    }
    b.cyl(KitMat::MetalRed, ALL, false, V(0, H + 4.5f, 0), V(0, H + 4.9f, 0), 0.16f, 0.16f, 8);
    // Ladder up one face: solid stiles, rungs between them.
    const V l0 = (corner(0, 0.4f) + corner(1, 0.4f)) * 0.5f, l1 = (corner(0, H) + corner(1, H)) * 0.5f;
    for (float sx : {-0.2f, 0.2f})
        b.beam(KitMat::MetalDark, NEAR, true, l0 + V(sx, 0, -0.06f), l1 + V(sx, 0, -0.06f), 0.02f, 0.02f);
    for (int k = 0; k < int(H / 0.6f); ++k) {
        const float y = 0.4f + 0.6f * float(k);
        const V p = corner(0, y), q = corner(1, y);
        const V m = (p + q) * 0.5f;
        b.beam(KitMat::MetalDark, L0, false, m + V(-0.2f, 0, -0.06f), m + V(0.2f, 0, -0.06f), 0.015f, 0.015f);
    }
}

void radioHut(Builder& b, const BuildingSpec& s) {
    const float W = s.w, D = s.d, H = s.h > 0.0f ? s.h : 2.8f;
    b.abox(KitMat::Concrete, ALL, true, V(-0.5f * W - 0.1f, -buildingPlinthDepth(s.kind), -0.5f * D - 0.1f), V(0.5f * W + 0.1f, 0.25f, 0.5f * D + 0.1f), FALL & ~FNY);
    std::vector<Opening> ops[4];
    ops[0].push_back({0.5f * W - 0.5f, 0.5f * W + 0.5f, 0.25f, 2.3f, Op::Door, 0});
    ops[1].push_back({0.5f * D - 0.4f, 0.5f * D + 0.4f, 1.5f, 2.1f, Op::Small, 0});
    FacadeStyle st;
    st.frame = KitMat::MetalDark;
    st.sill = KitMat::Concrete;
    st.door = KitMat::MetalDark;
    flatRoofBlock(b, W, D, H, KitMat::Concrete, KitMat::Concrete, ops, st);
    b.abox(KitMat::MetalWhite, ALL, true, V(-0.5f * W - 0.55f, 0.6f, -0.4f), V(-0.5f * W, 1.4f, 0.5f));
    b.abox(KitMat::MetalDark, NEAR, false, V(-0.5f * W - 0.57f, 0.75f, -0.25f), V(-0.5f * W - 0.55f, 1.25f, 0.35f), FNX);
}

// ------------------------------------------------------------------------------------------------------ ruins
void ruinWall(Builder& b, const BuildingSpec& s) {
    const float L = s.w, H = s.h > 0.0f ? s.h : 6.0f, T = 1.4f;
    const std::uint32_t seed = std::uint32_t(s.variant) * 7919u + 13u;
    const int n = std::max(2, int(L / 1.6f));
    const float plinth = buildingPlinthDepth(s.kind);
    std::vector<float> hs(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) hs[std::size_t(k)] = H * (0.55f + 0.45f * hash01(seed, std::uint32_t(k)));
    // A breach: two neighboring segments broken down.
    const int breach = int(hash01(seed, 99u) * float(n - 2)) + 1;
    if (n > 4) hs[std::size_t(breach)] = H * 0.18f, hs[std::size_t(breach + 1)] = H * 0.28f;
    for (int k = 0; k < n; ++k) {
        const float x0 = -0.5f * L + L * float(k) / float(n), x1 = -0.5f * L + L * float(k + 1) / float(n);
        const float h = hs[std::size_t(k)];
        b.abox(KitMat::Castle, ALL, true, V(x0, -plinth, -0.5f * T), V(x1, h, 0.5f * T), FALL & ~FNY);
        if (h > 0.85f * H) // merlons
            for (int m = 0; m < 2; ++m) {
                const float a = x0 + (x1 - x0) * (0.1f + 0.5f * float(m)), e = a + 0.35f * (x1 - x0);
                b.abox(KitMat::Castle, ALL, true, V(a, h, -0.5f * T + 0.1f), V(e, h + 0.7f, -0.5f * T + 0.65f), FALL & ~FNY);
            }
        if (h > 3.5f && k % 2 == 1) // arrow slit on both faces
            for (float sz : {-1.0f, 1.0f}) {
                const float xc = 0.5f * (x0 + x1), z = sz * (0.5f * T + 0.012f);
                b.quad(KitMat::Shadow, ALL, V(xc - 0.12f, 0.55f * h, z), V(xc + 0.12f, 0.55f * h, z), V(xc + 0.12f, 0.55f * h + 1.1f, z), V(xc - 0.12f, 0.55f * h + 1.1f, z), V(0, 0, sz));
            }
    }
    // Fallen blocks at the foot.
    const int blocks = 3 + int(hash01(seed, 7u) * 4.0f);
    for (int k = 0; k < blocks; ++k) {
        const float x = -0.5f * L + L * hash01(seed, 200u + std::uint32_t(k));
        const float side = hash01(seed, 300u + std::uint32_t(k)) < 0.5f ? -1.0f : 1.0f;
        const float sz = 0.25f + 0.3f * hash01(seed, 400u + std::uint32_t(k));
        const float z = side * (0.5f * T + 0.4f + 1.2f * hash01(seed, 500u + std::uint32_t(k)));
        b.ybox(KitMat::Castle, ALL, true, V(x, sz - 0.25f, z), V(sz * 1.3f, sz, sz), 3.0f * hash01(seed, 600u + std::uint32_t(k)), FALL & ~FNY);
    }
}

void ruinTower(Builder& b, const BuildingSpec& s) {
    const float R = 0.5f * s.w, H = s.h > 0.0f ? s.h : 10.0f;
    const std::uint32_t seed = std::uint32_t(s.variant) * 104729u + 7u;
    const float plinth = buildingPlinthDepth(s.kind);
    const float hl = 0.55f * H;
    b.cyl(KitMat::Castle, ALL, true, V(0, -plinth, 0), V(0, hl, 0), R + 0.15f, R, 14, 2);
    constexpr int kSeg = 14;
    const int gap = int(hash01(seed, 1u) * kSeg);
    for (int k = 0; k < kSeg; ++k) {
        if (k == gap || k == (gap + 1) % kSeg) continue;
        const float a = 2.0f * kPi * (float(k) + 0.5f) / kSeg;
        const float h = hl + (H - hl) * (0.35f + 0.65f * hash01(seed, 10u + std::uint32_t(k)));
        const float rm = R - 0.5f;
        const V c(rm * std::cos(a), 0.5f * (hl + h), rm * std::sin(a));
        const float tang = 2.0f * kPi * R / kSeg * 0.52f;
        // Segment turned so its local z points out of the tower.
        b.ybox(KitMat::Castle, ALL, true, c, V(tang, 0.5f * (h - hl), 0.5f), std::atan2(std::cos(a), std::sin(a)) + 0.5f * kPi, FALL & ~FNY);
    }
    // Slits and the doorway as decals on facet centers of the base (facet k spans angles 2 pi k / 14 from +Z).
    const float apothem = std::cos(kPi / kSeg);
    auto facet = [&](int k, float y0, float y1, float halfW) {
        const float t = 2.0f * kPi * (float(k) + 0.5f) / kSeg;
        const V n(std::sin(t), 0, std::cos(t)), tu(std::cos(t), 0, -std::sin(t));
        auto r = [&](float y) { return (R + 0.15f - 0.15f * (y + plinth) / (hl + plinth)) * apothem + 0.012f; };
        b.quad(KitMat::Shadow, ALL, n * r(y0) - tu * halfW + V(0, y0, 0), n * r(y0) + tu * halfW + V(0, y0, 0),
               n * r(y1) + tu * halfW + V(0, y1, 0), n * r(y1) - tu * halfW + V(0, y1, 0), n);
    };
    for (int k : {3, 6, 10, 13}) facet(k, 0.4f * H, 0.4f * H + 1.3f, 0.15f);
    facet(0, 0.0f, 2.3f, 0.7f);
}

void ruinGate(Builder& b, const BuildingSpec& s) {
    const float W = s.w, H = s.h > 0.0f ? s.h : 6.0f;
    const float plinth = buildingPlinthDepth(s.kind);
    for (float sx : {-1.0f, 1.0f})
        b.abox(KitMat::Castle, ALL, true, V(sx > 0 ? 0.5f * W : -0.5f * W - 1.9f, -plinth, -1.0f), V(sx > 0 ? 0.5f * W + 1.9f : -0.5f * W, H, 1.0f), FALL & ~FNY);
    b.abox(KitMat::Castle, ALL, true, V(-0.5f * W - 1.9f, H - 1.3f, -1.0f), V(0.5f * W + 1.9f, H, 1.0f));
    for (int k = 0; k < 3; ++k) { // stepped arch under the lintel
        const float d = 0.45f * float(3 - k), y = H - 1.3f - 0.32f * float(k + 1);
        for (float sx : {-1.0f, 1.0f})
            b.abox(KitMat::Castle, ALL, true, V(sx > 0 ? 0.5f * W - d : -0.5f * W, y, -0.95f), V(sx > 0 ? 0.5f * W : -0.5f * W + d, y + 0.32f, 0.95f));
    }
    for (int k = 0; k < 4; ++k) {
        const float x = -0.5f * W - 1.7f + (W + 3.4f) * float(k) / 3.0f;
        if (k == 2) continue;
        b.abox(KitMat::Castle, ALL, true, V(x - 0.35f, H, -0.9f), V(x + 0.35f, H + 0.7f, -0.3f), FALL & ~FNY);
    }
}

// ------------------------------------------------------------------------------------------------------ quarry
void crusher(Builder& b, const BuildingSpec& s) {
    const float H = s.h > 0.0f ? s.h : 9.0f;
    const float plinth = buildingPlinthDepth(s.kind);
    for (float sx : {-1.0f, 1.0f})
        for (float sz : {-1.0f, 1.0f}) {
            b.abox(KitMat::Concrete, ALL, true, V(sx * 2.2f - 0.4f, -plinth, sz * 1.8f - 0.4f), V(sx * 2.2f + 0.4f, 0.4f, sz * 1.8f + 0.4f), FALL & ~FNY);
            b.abox(KitMat::Rust, ALL, true, V(sx * 2.2f - 0.18f, 0.4f, sz * 1.8f - 0.18f), V(sx * 2.2f + 0.18f, H - 5.0f, sz * 1.8f + 0.18f));
        }
    for (float sz : {-1.0f, 1.0f}) b.beam(KitMat::Rust, ALL, true, V(-2.2f, 0.6f, sz * 1.8f), V(2.2f, H - 5.2f, sz * 1.8f), 0.08f, 0.08f);
    for (float sx : {-1.0f, 1.0f}) b.beam(KitMat::Rust, ALL, true, V(sx * 2.2f, 0.6f, -1.8f), V(sx * 2.2f, H - 5.2f, 1.8f), 0.08f, 0.08f);
    b.abox(KitMat::Rust, ALL, true, V(-2.4f, H - 5.0f, -2.0f), V(2.4f, H - 2.6f, 2.0f));
    b.cyl(KitMat::Rust, ALL, true, V(0, H - 2.6f, 0), V(0, H, 0), 2.0f, 3.6f, 4, 3, 0.25f * kPi);
    b.abox(KitMat::MetalDark, ALL, true, V(2.4f, H - 4.6f, -0.8f), V(3.6f, H - 3.4f, 0.6f));
    b.abox(KitMat::Metal, NEAR, false, V(2.45f, H - 4.3f, 0.6f), V(3.3f, H - 3.7f, 0.82f));
    b.beam(KitMat::Rust, ALL, true, V(0, H - 5.0f, -1.2f), V(0, 1.6f, -5.2f), 0.7f, 0.12f);
    // Walkway rail around the crusher top (above reach).
    for (float sz : {-1.0f, 1.0f}) b.beam(KitMat::MetalDark, NEAR, false, V(-2.6f, H - 1.6f, sz * 2.2f), V(2.6f, H - 1.6f, sz * 2.2f), 0.03f, 0.03f);
}

void conveyor(Builder& b, const BuildingSpec& s) {
    const float L = s.w, Hr = s.h;
    const V a(0, 1.0f, 0), e(0, 1.0f + Hr, L);
    b.beam(KitMat::MetalDark, ALL, true, a, e, 0.6f, 0.14f);
    const V dir = norm(e - a);
    const V up = cross(dir, kX) * -1.0f;
    b.beam(KitMat::Soil, NEAR, false, a + up * 0.15f, e + up * 0.15f, 0.5f, 0.02f);
    for (float sx : {-1.0f, 1.0f}) b.beam(KitMat::Rust, NEAR, false, a + V(sx * 0.68f, 0.25f, 0), e + V(sx * 0.68f, 0.25f, 0), 0.04f, 0.18f);
    const int legs = std::max(1, int(L / 6.0f));
    for (int k = 1; k <= legs; ++k) {
        const float t = float(k) / float(legs + 1);
        const V p = a + (e - a) * t;
        for (float sx : {-1.0f, 1.0f}) b.beam(KitMat::Rust, ALL, true, V(sx * 0.55f, -buildingPlinthDepth(s.kind), p.z), V(sx * 0.55f, p.y - 0.12f, p.z), 0.1f, 0.1f);
        b.beam(KitMat::Rust, ALL, true, V(-0.55f, p.y * 0.5f, p.z), V(0.55f, p.y * 0.5f, p.z), 0.06f, 0.06f);
    }
}

void silo(Builder& b, const BuildingSpec& s) {
    const float R = 0.5f * s.w, H = s.h > 0.0f ? s.h : 12.0f;
    const float yl = 0.32f * H, yc = 0.48f * H;
    for (int k = 0; k < 4; ++k) {
        const float a = 0.25f * kPi + 0.5f * kPi * float(k);
        const V p(0.75f * R * std::cos(a), 0, 0.75f * R * std::sin(a));
        b.abox(KitMat::Concrete, ALL, true, V(p.x - 0.35f, -buildingPlinthDepth(s.kind), p.z - 0.35f), V(p.x + 0.35f, 0.3f, p.z + 0.35f), FALL & ~FNY);
        b.abox(KitMat::Metal, ALL, true, V(p.x - 0.15f, 0.3f, p.z - 0.15f), V(p.x + 0.15f, yc, p.z + 0.15f));
    }
    b.cyl(KitMat::Metal, ALL, true, V(0, yl, 0), V(0, yc, 0), 0.35f, R, 16, 1);
    b.cyl(KitMat::Metal, ALL, true, V(0, yc, 0), V(0, H, 0), R, R, 16, 0);
    b.cyl(KitMat::Metal, ALL, true, V(0, H, 0), V(0, H + 0.4f * R, 0), R + 0.08f, 0.25f, 16, 1);
    for (int k = 1; k < 4; ++k) {
        const float y = yc + (H - yc) * float(k) / 4.0f;
        b.cyl(KitMat::MetalDark, NEAR, false, V(0, y - 0.06f, 0), V(0, y + 0.06f, 0), R + 0.03f, R + 0.03f, 16, 0);
    }
    for (float x : {-0.22f, 0.22f}) b.beam(KitMat::MetalDark, L0, false, V(x, yc, R + 0.12f), V(x, H, R + 0.12f), 0.02f, 0.02f);
}

// -------------------------------------------------------------------------------------------- docks, camps, small
void dock(Builder& b, const BuildingSpec& s) {
    const float L = s.w, Wd = s.d, depth = s.h > 0.0f ? s.h : 4.0f;
    b.abox(KitMat::Wood, ALL, true, V(-0.5f * Wd, -0.25f, 0.0f), V(0.5f * Wd, 0.0f, L));
    const int posts = std::max(2, int(L / 3.0f) + 1);
    for (int k = 0; k < posts; ++k) {
        const float z = 0.25f + (L - 0.5f) * float(k) / float(posts - 1);
        for (float sx : {-1.0f, 1.0f}) b.cyl(KitMat::WoodDark, ALL, true, V(sx * (0.5f * Wd - 0.12f), -depth, z), V(sx * (0.5f * Wd - 0.12f), 0.55f, z), 0.14f, 0.14f, 8);
    }
    for (float sx : {-1.0f, 1.0f}) b.abox(KitMat::WoodDark, NEAR, false, V(sx * 0.5f * Wd - 0.08f, -0.5f, 0.0f), V(sx * 0.5f * Wd + 0.04f, -0.25f, L));
    for (int k = 0; k < int(L / 2.0f); ++k) {
        const float z = 1.0f + 2.0f * float(k);
        b.abox(KitMat::WoodDark, L0, false, V(-0.5f * Wd, -0.004f, z - 0.01f), V(0.5f * Wd, 0.002f, z + 0.01f), FPY);
    }
    // Ladder at the end.
    for (float sx : {-0.25f, 0.25f}) b.beam(KitMat::MetalDark, NEAR, true, V(sx, 0.6f, L + 0.05f), V(sx, -2.0f, L + 0.05f), 0.025f, 0.025f);
    for (int k = 0; k < 5; ++k) b.beam(KitMat::MetalDark, L0, false, V(-0.25f, 0.2f - 0.45f * float(k), L + 0.05f), V(0.25f, 0.2f - 0.45f * float(k), L + 0.05f), 0.015f, 0.015f);
}

void tent(Builder& b, const BuildingSpec& s) {
    if (s.variant == 1) { // dome
        const float R = 1.5f, Hd = 1.35f;
        constexpr int kRings = 4;
        for (int k = 0; k < kRings; ++k) {
            const float a0 = 0.5f * kPi * float(k) / kRings, a1 = 0.5f * kPi * float(k + 1) / kRings;
            b.cyl(KitMat::NylonGreen, ALL, true, V(0, Hd * std::sin(a0), 0), V(0, Hd * std::sin(a1), 0), R * std::cos(a0),
                  std::max(R * std::cos(a1), 0.0f), 10, 0, kPi / 10.0f);
        }
        // Door decal on the +Z facets of the two lowest rings (ring k spans y = Hd sin(a_k) .. Hd sin(a_k+1)).
        const float ap = std::cos(kPi / 10.0f) * R;
        const float y1 = Hd * std::sin(0.5f * kPi / kRings), z1 = ap * std::cos(0.5f * kPi / kRings) + 0.012f;
        const float y2 = 0.95f, a2 = 0.5f * kPi / kRings, a3 = kPi / kRings;
        const float r2 = R * (std::cos(a2) + (std::cos(a3) - std::cos(a2)) * (y2 - y1) / (Hd * std::sin(a3) - y1));
        const float z0 = ap + 0.012f, z2 = r2 * std::cos(kPi / 10.0f) + 0.012f;
        b.quad(KitMat::Shadow, ALL, V(-0.4f, 0.02f, z0), V(0.4f, 0.02f, z0), V(0.36f, y1, z1), V(-0.36f, y1, z1), norm(V(0, 0.15f, 1)));
        b.quad(KitMat::Shadow, ALL, V(-0.36f, y1, z1), V(0.36f, y1, z1), V(0.3f, y2, z2), V(-0.3f, y2, z2), norm(V(0, 0.6f, 1)));
        return;
    }
    const float hw = 1.15f, hd = 1.45f, Ht = 1.45f;
    b.prism(KitMat::NylonOrange, ALL, true, V(-hw, 0, hd), V(hw, 0, hd), V(0, Ht, hd), V(0, 0, -1), 2.0f * hd);
    b.tri(KitMat::Shadow, ALL, V(-0.45f, 0.02f, hd + 0.012f), V(0.45f, 0.02f, hd + 0.012f), V(0, 1.1f, hd + 0.012f), kZ);
    for (float z : {-hd - 0.02f, hd + 0.02f}) b.beam(KitMat::MetalDark, L0, false, V(0, 0, z), V(0, Ht + 0.12f, z), 0.015f, 0.015f);
}

// One fence run from (0, 0, 0) to (len, 0, 0) along local x.
void fenceRun(Builder& b, float len, int style, V origin, V dir) {
    const V side = norm(cross(kY, dir));
    auto P = [&](float u, float y, float w) { return origin + dir * u + V(0, y, 0) + side * w; };
    const float spacing = style == 1 ? 2.5f : 2.0f, H = style == 0 ? 1.0f : 1.1f;
    const int posts = std::max(1, int(std::ceil(len / spacing)));
    const KitMat postMat = style == 2 ? KitMat::MetalDark : KitMat::WoodDark;
    for (int k = 0; k <= posts; ++k) {
        const float u = len * float(k) / float(posts);
        if (style == 2) b.cyl(postMat, ALL, true, P(u, -0.5f, 0), P(u, H, 0), 0.04f, 0.04f, 6);
        else b.box(postMat, ALL, true, P(u, 0.5f * (H + 0.1f) - 0.5f, 0), dir, kY, side, V(0.06f, 0.5f * (H + 0.6f), 0.06f), FALL & ~FNY);
    }
    const KitMat railMat = style == 0 ? KitMat::PaintWhite : style == 1 ? KitMat::Wood : KitMat::MetalDark;
    const float rails[2] = {style == 0 ? 0.3f : 0.4f, style == 0 ? 0.75f : 0.95f};
    for (float y : rails) b.beam(railMat, ALL, true, P(0, y, 0.07f), P(len, y, 0.07f), 0.025f, 0.05f);
    if (style == 0) {
        const int n = std::max(1, int(len / 0.15f));
        for (int k = 0; k < n; ++k) {
            const float u = len * (float(k) + 0.5f) / float(n);
            b.box(railMat, NEAR, false, P(u, 0.55f, 0.11f), dir, kY, side, V(0.04f, 0.45f, 0.012f), FALL & ~FNY);
        }
    }
}

void fence(Builder& b, const BuildingSpec& s) { fenceRun(b, s.w, s.variant, V(-0.5f * s.w, 0, 0), kX); }

void garden(Builder& b, const BuildingSpec& s) {
    const float W = s.w, D = s.d, g = 1.2f;
    fenceRun(b, W, 0, V(-0.5f * W, 0, -0.5f * D), kX);
    fenceRun(b, D, 0, V(0.5f * W, 0, -0.5f * D), kZ);
    fenceRun(b, D, 0, V(-0.5f * W, 0, 0.5f * D), -kZ);
    fenceRun(b, 0.5f * (W - g), 0, V(0.5f * W, 0, 0.5f * D), -kX);
    fenceRun(b, 0.5f * (W - g), 0, V(-0.5f * g, 0, 0.5f * D), -kX);
    const int beds = std::max(1, int((D - 1.5f) / 1.6f));
    for (int k = 0; k < beds; ++k) {
        const float z0 = -0.5f * D + 0.8f + (D - 1.6f) * float(k) / float(beds), z1 = z0 + 1.0f;
        b.abox(KitMat::Wood, ALL, true, V(-0.5f * W + 0.8f, -0.3f, z0), V(0.5f * W - 0.8f, 0.3f, z1), FALL & ~FNY);
        b.quad(KitMat::Soil, ALL, V(-0.5f * W + 0.85f, 0.305f, z0 + 0.05f), V(0.5f * W - 0.85f, 0.305f, z0 + 0.05f), V(0.5f * W - 0.85f, 0.305f, z1 - 0.05f), V(-0.5f * W + 0.85f, 0.305f, z1 - 0.05f), kY);
        const int plants = int((W - 2.0f) / 0.45f);
        for (int p = 0; p < plants; ++p)
            for (int r = 0; r < 2; ++r) {
                const float x = -0.5f * W + 1.1f + 0.45f * float(p), z = z0 + 0.3f + 0.4f * float(r);
                const float h = 0.12f + 0.12f * hash01(std::uint32_t(k * 31 + p), std::uint32_t(r + s.variant * 7));
                const bool flower = hash01(std::uint32_t(p * 17 + k), 5u + std::uint32_t(s.variant)) > 0.8f;
                b.ybox(flower ? KitMat::Flowers : KitMat::Leaves, NEAR, false, V(x, 0.3f + 0.5f * h, z), V(0.14f, 0.5f * h, 0.14f), 0.7f * float(p + r), FALL & ~FNY);
            }
    }
}

void powerPole(Builder& b, const BuildingSpec& s) {
    const float H = s.h > 0.0f ? s.h : 8.0f;
    b.cyl(KitMat::WoodDark, ALL, true, V(0, -1.5f, 0), V(0, H, 0), 0.15f, 0.12f, 8);
    b.abox(KitMat::Wood, ALL, true, V(-1.15f, H - 0.62f, -0.07f), V(1.15f, H - 0.46f, 0.07f));
    for (float x : {-0.95f, 0.0f, 0.95f}) {
        const float y0 = x == 0.0f ? H : H - 0.46f;
        b.cyl(KitMat::PaintWhite, NEAR, false, V(x, y0, 0), V(x, y0 + 0.22f, 0), 0.05f, 0.035f, 6);
    }
    b.beam(KitMat::Wood, L0, false, V(-0.6f, H - 0.54f, 0.08f), V(0.0f, H - 1.3f, 0.12f), 0.035f, 0.035f);
    b.beam(KitMat::Wood, L0, false, V(0.6f, H - 0.54f, 0.08f), V(0.0f, H - 1.3f, 0.12f), 0.035f, 0.035f);
    if (s.variant == 1) b.cyl(KitMat::MetalDark, ALL, true, V(0, H - 3.2f, 0.42f), V(0, H - 2.2f, 0.42f), 0.32f, 0.32f, 10);
}

// Three conductors from x = -0.95 / 0 / 0.95 at the insulator tops (origin, middle wire 0.46 m higher) to the next
// pole w m along +z and h m lower; sagging 1.2 % of the span. No collision (6 m and more above the ground).
void wire(Builder& b, const BuildingSpec& s) {
    const float L = s.w, drop = s.h;
    constexpr int kSeg = 10;
    for (float x : {-0.95f, 0.0f, 0.95f}) {
        const float y0 = x == 0.0f ? 0.46f : 0.0f;
        for (int k = 0; k < kSeg; ++k) {
            auto at = [&](int i) {
                const float t = float(i) / kSeg;
                return V(x, y0 - drop * t - 4.0f * 0.012f * L * t * (1.0f - t), L * t);
            };
            b.beam(KitMat::MetalDark, ALL, false, at(k), at(k + 1), 0.012f, 0.012f, FPX | FNX | FPY | FNY);
        }
    }
}

void woodpile(Builder& b, const BuildingSpec& s) {
    const float L = s.w, H = 1.3f, Dp = 0.7f;
    b.abox(KitMat::Wood, FAR, true, V(-0.5f * L, -0.2f, -0.5f * Dp), V(0.5f * L, H, 0.5f * Dp), FALL & ~FNY);
    const int rows = 5;
    for (int r = 0; r < rows; ++r) {
        const float rr = 0.5f * H / float(rows);
        const int cols = int(Dp / (2.0f * rr));
        for (int c = 0; c < cols; ++c) {
            const float y = rr + 2.0f * rr * float(r) - 0.02f, z = -0.5f * Dp + rr + 2.0f * rr * float(c);
            b.cyl(KitMat::Wood, NEAR, false, V(-0.5f * L + 0.02f, y, z), V(0.5f * L - 0.02f, y, z), rr * 0.95f, rr * 0.95f, 7, 3, 0.4f * float(r + c));
        }
    }
    b.cyl(KitMat::WoodDark, ALL, true, V(-0.5f * L - 0.1f, -0.5f, -0.45f), V(-0.5f * L - 0.1f, H + 0.45f, -0.45f), 0.07f, 0.07f, 6);
    b.cyl(KitMat::WoodDark, ALL, true, V(0.5f * L + 0.1f, -0.5f, -0.45f), V(0.5f * L + 0.1f, H + 0.45f, -0.45f), 0.07f, 0.07f, 6);
    const V a(-0.5f * L - 0.3f, H + 0.55f, -0.6f), e(0.5f * L + 0.3f, H + 0.25f, 0.6f);
    b.box(KitMat::Shingles, ALL, true, V(0, 0.5f * (a.y + e.y), 0), kX, norm(V(0, 1.0f, 0.25f)), norm(V(0, -0.25f, 1.0f)), V(0.5f * (e.x - a.x), 0.05f, 0.62f));
}

void cairn(Builder& b, const BuildingSpec& s) {
    const std::uint32_t seed = std::uint32_t(s.variant) + 3u;
    float y = -0.4f;
    for (int k = 0; k < 6; ++k) {
        const float r = 0.75f - 0.1f * float(k);
        const float h = 0.28f + 0.06f * hash01(seed, std::uint32_t(k));
        b.ybox(KitMat::Stone, ALL, true, V(0.05f * hash01(seed, 20u + std::uint32_t(k)), y + 0.5f * h, 0), V(r, 0.5f * h, r * 0.85f), 1.3f * float(k) + hash01(seed, 9u), FALL & ~FNY);
        y += h * 0.92f;
    }
    b.abox(KitMat::WoodDark, ALL, true, V(-0.07f, y - 0.3f, -0.07f), V(0.07f, y + 2.6f, 0.07f));
    b.abox(KitMat::WoodDark, ALL, true, V(-0.6f, y + 1.7f, -0.06f), V(0.6f, y + 1.86f, 0.06f));
    b.abox(KitMat::MetalDark, L0, false, V(-0.12f, y + 1.0f, 0.07f), V(0.12f, y + 1.3f, 0.2f));
}

void signpost(Builder& b, const BuildingSpec& s) {
    b.abox(KitMat::WoodDark, ALL, true, V(-0.06f, -0.6f, -0.06f), V(0.06f, 2.5f, 0.06f));
    b.ybox(KitMat::Wood, ALL, true, V(0.45f, 2.15f, 0.0f), V(0.6f, 0.13f, 0.025f), 0.0f);
    b.ybox(KitMat::Wood, ALL, true, V(-0.25f, 1.8f, 0.3f), V(0.6f, 0.13f, 0.025f), 0.9f + 0.3f * float(s.variant));
    b.prism(KitMat::Wood, NEAR, false, V(1.05f, 2.02f, -0.025f), V(1.05f, 2.28f, -0.025f), V(1.25f, 2.15f, -0.025f), kZ, 0.05f);
}

void well(Builder& b, const BuildingSpec& s) {
    (void)s;
    b.cyl(KitMat::Stone, ALL, true, V(0, -1.0f, 0), V(0, 0.8f, 0), 1.0f, 1.0f, 12, 2);
    b.cyl(KitMat::Shadow, ALL, false, V(0, 0.8f, 0), V(0, 0.81f, 0), 0.72f, 0.72f, 12, 2);
    for (float sx : {-1.0f, 1.0f}) b.abox(KitMat::WoodDark, ALL, true, V(sx * 0.95f - 0.08f, 0.8f, -0.08f), V(sx * 0.95f + 0.08f, 2.5f, 0.08f));
    b.cyl(KitMat::Wood, ALL, true, V(-0.87f, 1.75f, 0), V(0.87f, 1.75f, 0), 0.1f, 0.1f, 8);
    for (int sz = -1; sz <= 1; sz += 2) {
        const float p = 0.6f;
        const V sd(0, -std::sin(p), float(sz) * std::cos(p)), nn(0, std::cos(p), float(sz) * std::sin(p));
        b.box(KitMat::Shingles, ALL, true, V(0, 2.55f, 0) + sd * 0.65f + nn * 0.05f, kX, nn, sd, V(1.25f, 0.05f, 0.65f));
    }
    b.cyl(KitMat::WoodDark, L0, false, V(0, 1.1f, 0), V(0, 1.4f, 0), 0.16f, 0.18f, 8);
}

void shelter(Builder& b, const BuildingSpec& s) {
    const float W = s.w, D = s.d, Hf = 2.6f, Hb = 2.1f;
    for (float sx : {-1.0f, 1.0f}) {
        b.abox(KitMat::WoodDark, ALL, true, V(sx * (0.5f * W - 0.1f) - 0.08f, -0.5f, 0.5f * D - 0.18f), V(sx * (0.5f * W - 0.1f) + 0.08f, Hf, 0.5f * D - 0.02f));
        b.abox(KitMat::WoodDark, ALL, true, V(sx * (0.5f * W - 0.1f) - 0.08f, -0.5f, -0.5f * D + 0.02f), V(sx * (0.5f * W - 0.1f) + 0.08f, Hb, -0.5f * D + 0.18f));
    }
    b.abox(KitMat::Siding, ALL, true, V(-0.5f * W, -0.2f, -0.5f * D), V(0.5f * W, Hb - 0.1f, -0.5f * D + 0.06f));
    const float p = std::atan2(Hf - Hb, D);
    const V sd(0, -std::sin(p), -std::cos(p)), nn(0, std::cos(p), -std::sin(p));
    const float l = (D + 0.8f) / std::cos(p);
    b.box(KitMat::Shingles, ALL, true, V(0, Hf, 0.5f * D + 0.4f) + sd * (0.5f * l) + nn * 0.06f, kX, nn, sd, V(0.5f * W + 0.3f, 0.06f, 0.5f * l));
    b.abox(KitMat::Wood, ALL, true, V(-0.5f * W + 0.4f, 0.35f, -0.5f * D + 0.15f), V(0.5f * W - 0.4f, 0.45f, -0.5f * D + 0.55f));
}

void bench(Builder& b, const BuildingSpec& s) {
    const float L = s.w > 0.0f ? s.w : 1.8f;
    for (float x : {-0.5f * L + 0.2f, 0.5f * L - 0.2f}) {
        b.abox(KitMat::MetalDark, ALL, true, V(x - 0.04f, -0.3f, -0.22f), V(x + 0.04f, 0.42f, 0.2f));
        b.abox(KitMat::MetalDark, ALL, true, V(x - 0.04f, 0.42f, -0.24f), V(x + 0.04f, 0.9f, -0.17f));
    }
    for (int k = 0; k < 3; ++k) {
        const float z0 = -0.2f + 0.14f * float(k);
        b.abox(KitMat::Wood, ALL, true, V(-0.5f * L, 0.42f, z0), V(0.5f * L, 0.47f, z0 + 0.12f));
    }
    for (int k = 0; k < 2; ++k) {
        const float y0 = 0.58f + 0.16f * float(k);
        b.abox(KitMat::Wood, ALL, true, V(-0.5f * L, y0, -0.17f), V(0.5f * L, y0 + 0.12f, -0.12f));
    }
}


void barrier(Builder& b, const BuildingSpec& s) {
    const float L = s.w > 0.0f ? s.w : 4.0f;
    if (s.variant == 1) {
        const int posts = std::max(2, int(std::round(L / 2.0f)) + 1);
        for (int k = 0; k < posts; ++k) {
            const float x = -0.5f * L + L * float(k) / float(posts - 1);
            b.abox(KitMat::MetalDark, ALL, true, V(x - 0.05f, -0.5f, -0.12f), V(x + 0.05f, 0.75f, -0.02f));
        }
        b.abox(KitMat::Metal, ALL, true, V(-0.5f * L, 0.42f, -0.02f), V(0.5f * L, 0.74f, 0.06f));
        return;
    }
    b.abox(KitMat::Concrete, ALL, true, V(-0.5f * L, -0.3f, -0.3f), V(0.5f * L, 0.28f, 0.3f), FALL & ~FNY);
    b.abox(KitMat::Concrete, ALL, true, V(-0.5f * L, 0.28f, -0.12f), V(0.5f * L, 0.82f, 0.12f), FALL & ~FNY);
    const int stripes = std::max(1, int(L / 1.0f));
    for (int k = 0; k < stripes; k += 2) {
        const float x0 = -0.5f * L + L * float(k) / float(stripes), x1 = -0.5f * L + L * float(k + 1) / float(stripes);
        for (float sz : {-1.0f, 1.0f})
            b.abox(KitMat::PaintRed, NEAR, false, V(x0, 0.3f, sz > 0 ? 0.12f : -0.132f), V(x1, 0.8f, sz > 0 ? 0.132f : -0.12f),
                   sz > 0 ? FPZ : FNZ);
    }
}

// Columns of three tyres; the collision is one box around the stack (the tyres touch its faces).
void tyreWall(Builder& b, const BuildingSpec& s) {
    const float L = s.w > 0.0f ? s.w : 6.0f, R = 0.33f, T = 0.26f;
    b.abox(KitMat::MetalDark, FAR, true, V(-0.5f * L, -0.2f, -R), V(0.5f * L, 3.0f * T, R), FALL & ~FNY);
    const int cols = std::max(1, int(L / (2.0f * R)));
    for (int c = 0; c < cols; ++c) {
        const float x = -0.5f * L + R + (L - 2.0f * R) * (cols > 1 ? float(c) / float(cols - 1) : 0.5f);
        for (int r = 0; r < 3; ++r) {
            const KitMat m = s.variant == 1 ? ((c + r) % 2 ? KitMat::PaintWhite : KitMat::PaintRed) : KitMat::MetalDark;
            b.cyl(m, NEAR, false, V(x, T * float(r), 0), V(x, T * float(r + 1) - 0.01f, 0), R, R, 12, 2);
        }
        b.cyl(KitMat::Shadow, NEAR, false, V(x, 3.0f * T - 0.008f, 0), V(x, 3.0f * T - 0.002f, 0), 0.6f * R, 0.6f * R, 12, 2);
    }
}

void flag(Builder& b, const BuildingSpec& s) {
    const float H = s.h > 0.0f ? s.h : 5.0f;
    static constexpr KitMat cloth[] = {KitMat::PaintRed, KitMat::PaintWhite, KitMat::NylonOrange, KitMat::NylonGreen};
    b.abox(KitMat::Concrete, ALL, true, V(-0.25f, -0.4f, -0.25f), V(0.25f, 0.2f, 0.25f), FALL & ~FNY);
    b.cyl(KitMat::MetalWhite, ALL, true, V(0, 0.2f, 0), V(0, H, 0), 0.05f, 0.04f, 8);
    b.abox(cloth[std::size_t(s.variant) % 4], ALL, true, V(0.05f, H - 0.9f, -0.01f), V(1.35f, H - 0.05f, 0.01f));
}

void grandstand(Builder& b, const BuildingSpec& s) {
    const float W = s.w, D = s.d;
    constexpr int kRows = 5;
    const float rowD = (D - 1.2f) / float(kRows), rise = 0.45f;
    for (int k = 0; k < kRows; ++k) {
        const float zf = 0.5f * D - 1.2f - rowD * float(k), y = rise * float(k + 1);
        b.abox(KitMat::Concrete, ALL, true, V(-0.5f * W, -1.0f, -0.5f * D), V(0.5f * W, y, zf), FALL & ~FNY);
        b.abox(k % 2 ? KitMat::PaintWhite : KitMat::PaintRed, NEAR, false, V(-0.5f * W + 0.3f, y, zf - 0.75f),
               V(0.5f * W - 0.3f, y + 0.12f, zf - 0.35f), FALL & ~FNY);
    }
    const float top = rise * float(kRows);
    b.abox(KitMat::Concrete, ALL, true, V(-0.5f * W, -1.0f, 0.5f * D - 1.2f), V(0.5f * W, 0.15f, 0.5f * D), FALL & ~FNY);
    b.abox(KitMat::Siding, ALL, true, V(-0.5f * W, top, -0.5f * D), V(0.5f * W, top + 2.8f, -0.5f * D + 0.15f));
    for (float sx : {-1.0f, 1.0f}) b.abox(KitMat::Siding, ALL, true, V(sx > 0 ? 0.5f * W - 0.15f : -0.5f * W, 0.15f, -0.5f * D),
                                          V(sx > 0 ? 0.5f * W : -0.5f * W + 0.15f, top + 2.8f, -0.5f * D + 0.4f * D));
    const int posts = std::max(2, int(W / 6.0f) + 1);
    for (int k = 0; k < posts; ++k) {
        const float x = -0.5f * W + 0.3f + (W - 0.6f) * float(k) / float(posts - 1);
        b.abox(KitMat::MetalWhite, ALL, true, V(x - 0.1f, top, -0.5f * D + 0.15f), V(x + 0.1f, top + 2.8f, -0.5f * D + 0.35f));
        b.beam(KitMat::MetalWhite, ALL, true, V(x, top + 2.8f, -0.5f * D + 0.2f), V(x, top + 3.1f, 0.5f * D - 0.4f), 0.08f, 0.1f);
    }
    b.box(KitMat::CorrugatedRed, ALL, true, V(0, top + 3.05f, 0.05f), kX, norm(V(0, 1.0f, -0.03f)), norm(V(0, 0.03f, 1.0f)),
          V(0.5f * W + 0.3f, 0.06f, 0.5f * D + 0.1f));
    for (int k = 0; k <= int(W / 2.0f); ++k) {
        const float x = -0.5f * W + 0.2f + (W - 0.4f) * float(k) / float(int(W / 2.0f));
        b.abox(KitMat::MetalDark, ALL, true, V(x - 0.025f, 0.15f, 0.5f * D - 0.15f), V(x + 0.025f, 1.15f, 0.5f * D - 0.1f));
    }
    b.abox(KitMat::MetalDark, ALL, true, V(-0.5f * W, 1.05f, 0.5f * D - 0.16f), V(0.5f * W, 1.15f, 0.5f * D - 0.08f));
}

void startGantry(Builder& b, const BuildingSpec& s) {
    const float W = s.w, H = s.h > 0.0f ? s.h : 6.0f, B = 1.4f;
    for (float sx : {-1.0f, 1.0f}) {
        const float x = sx * (0.5f * W + 0.25f);
        b.abox(KitMat::Concrete, ALL, true, V(x - 0.45f, -0.6f, -0.45f), V(x + 0.45f, 0.3f, 0.45f), FALL & ~FNY);
        b.abox(KitMat::MetalWhite, ALL, true, V(x - 0.25f, 0.3f, -0.25f), V(x + 0.25f, H + B + 0.4f, 0.25f));
    }
    b.abox(KitMat::MetalDark, ALL, true, V(-0.5f * W, H, -0.2f), V(0.5f * W, H + B, 0.2f));
    const int cols = std::max(4, int(W / (0.5f * B)));
    const float cw = W / float(cols), ch = 0.5f * B;
    for (int c = 0; c < cols; ++c)
        for (int r = 0; r < 2; ++r) {
            if ((c + r) % 2) continue;
            const float x0 = -0.5f * W + cw * float(c), y0 = H + ch * float(r);
            b.abox(KitMat::PaintWhite, ALL, false, V(x0, y0, 0.2f), V(x0 + cw, y0 + ch, 0.212f), FPZ);
            b.abox(KitMat::PaintWhite, ALL, false, V(x0, y0, -0.212f), V(x0 + cw, y0 + ch, -0.2f), FNZ);
        }
    b.abox(KitMat::MetalRed, ALL, true, V(-0.5f * W - 0.5f, H + B, -0.25f), V(0.5f * W + 0.5f, H + B + 0.4f, 0.25f));
    // Start lights: a dark box under the banner, five red lamps towards the grid (-Z).
    b.abox(KitMat::MetalDark, ALL, true, V(-1.6f, H - 0.75f, -0.18f), V(1.6f, H, 0.18f));
    for (int k = 0; k < 5; ++k) {
        const float x = -1.28f + 0.64f * float(k);
        b.abox(KitMat::MetalRed, ALL, false, V(x - 0.2f, H - 0.58f, -0.192f), V(x + 0.2f, H - 0.18f, -0.18f), FNZ);
    }
}

void marshalPost(Builder& b, const BuildingSpec&) {
    b.abox(KitMat::Concrete, ALL, true, V(-1.1f, -0.4f, -1.1f), V(1.1f, 0.15f, 1.1f), FALL & ~FNY);
    b.abox(KitMat::PlasterWhite, ALL, true, V(-0.9f, 0.15f, -0.9f), V(0.9f, 2.3f, -0.78f));
    for (float sx : {-1.0f, 1.0f})
        b.abox(KitMat::PlasterWhite, ALL, true, V(sx > 0 ? 0.78f : -0.9f, 0.15f, -0.78f), V(sx > 0 ? 0.9f : -0.78f, 2.3f, 0.9f));
    b.abox(KitMat::PlasterWhite, ALL, true, V(-0.78f, 0.15f, 0.78f), V(0.78f, 1.05f, 0.9f));
    b.abox(KitMat::CorrugatedRed, ALL, true, V(-1.05f, 2.3f, -1.05f), V(1.05f, 2.42f, 1.05f));
    b.abox(KitMat::NylonOrange, ALL, false, V(0.9f, 1.25f, -0.5f), V(0.912f, 1.75f, 0.5f), FPX);
}

void raceControl(Builder& b, const BuildingSpec& s) {
    const float W = s.w, D = s.d, H1 = 3.2f, H2 = 6.2f;
    b.abox(KitMat::Concrete, ALL, true, V(-0.5f * W, -3.0f, -0.5f * D), V(0.5f * W, H1, 0.5f * D), FALL & ~FNY);
    b.abox(KitMat::PlasterWhite, ALL, true, V(-0.5f * W, H1, -0.5f * D), V(0.5f * W, H2, 0.5f * D - 1.6f));
    b.abox(KitMat::Glass, ALL, false, V(-0.5f * W + 0.3f, H1 + 0.9f, 0.5f * D - 1.6f), V(0.5f * W - 0.3f, H2 - 0.4f, 0.5f * D - 1.588f), FPZ);
    b.abox(KitMat::Concrete, ALL, true, V(-0.5f * W - 0.3f, H2, -0.5f * D - 0.3f), V(0.5f * W + 0.3f, H2 + 0.35f, 0.5f * D + 0.3f));
    b.abox(KitMat::MetalDark, ALL, true, V(-0.5f * W, H1 + 1.0f, 0.5f * D - 0.06f), V(0.5f * W, H1 + 1.1f, 0.5f * D));
    for (float sx : {-1.0f, 1.0f})
        b.abox(KitMat::MetalDark, ALL, true, V(sx * 0.5f * W - 0.04f, H1, 0.5f * D - 0.08f), V(sx * 0.5f * W + 0.04f, H1 + 1.1f, 0.5f * D));
    b.abox(KitMat::Glass, ALL, false, V(-1.2f, 0.0f, 0.5f * D), V(1.2f, 2.4f, 0.5f * D + 0.012f), FPZ);
    b.abox(KitMat::Shadow, ALL, false, V(-0.5f * W + 1.0f, 0.4f, 0.5f * D), V(-0.5f * W + 3.0f, 2.2f, 0.5f * D + 0.012f), FPZ);
    b.abox(KitMat::MetalRed, ALL, true, V(-0.3f * W, H2 + 0.35f, 0.5f * D - 0.1f), V(0.3f * W, H2 + 1.35f, 0.5f * D + 0.1f));
    b.abox(KitMat::PaintWhite, ALL, false, V(-0.3f * W + 0.2f, H2 + 0.55f, 0.5f * D + 0.1f), V(0.3f * W - 0.2f, H2 + 1.15f, 0.5f * D + 0.112f), FPZ);
}
} // namespace

const KitMaterial& kitMaterial(KitMat m) { return kMaterials[std::min(int(m), kKitMatCount - 1)]; }

bool buildingIsStructure(BuildingKind k) {
    switch (k) {
    case BuildingKind::House: case BuildingKind::FuelShop: case BuildingKind::Garage: case BuildingKind::Lighthouse:
    case BuildingKind::RadioMast: case BuildingKind::RadioHut: case BuildingKind::RuinWall: case BuildingKind::RuinTower:
    case BuildingKind::RuinGate: case BuildingKind::Crusher: case BuildingKind::Silo: case BuildingKind::FuelCanopy:
    case BuildingKind::Grandstand: case BuildingKind::RaceControl:
        return true;
    default: return false;
    }
}

float buildingPlinthDepth(BuildingKind k) {
    switch (k) {
    case BuildingKind::House: case BuildingKind::Garage: case BuildingKind::FuelShop: case BuildingKind::RadioHut:
    case BuildingKind::Lighthouse: case BuildingKind::RaceControl: return 3.0f;
    case BuildingKind::RuinWall: case BuildingKind::RuinTower: case BuildingKind::RuinGate: return 3.5f;
    case BuildingKind::FuelCanopy: return 1.5f;
    default: return 1.0f;
    }
}

const char* buildingKindName(BuildingKind k) {
    static const char* const names[] = {"house", "fuel_canopy", "fuel_shop", "pylon", "garage", "lighthouse", "radio_mast",
                                        "radio_hut", "ruin_wall", "ruin_tower", "ruin_gate", "crusher", "conveyor", "silo",
                                        "dock", "tent", "fence", "garden", "power_pole", "wire", "woodpile", "cairn",
                                        "signpost", "well", "shelter", "bench", "barrier", "tyre_wall", "flag", "grandstand",
                                        "start_gantry", "race_control", "marshal_post"};
    static_assert(std::size(names) == std::size_t(BuildingKind::Count));
    return names[std::min(std::size_t(k), std::size(names) - 1)];
}

BuildingGeometry buildBuilding(const BuildingSpec& spec, bool render) {
    BuildingGeometry g;
    Builder b(g, render);
    float fx = 0.5f * spec.w, fz = 0.5f * spec.d;
    switch (spec.kind) {
    case BuildingKind::House: house(b, spec); break;
    case BuildingKind::FuelCanopy: fuelCanopy(b, spec); break;
    case BuildingKind::FuelShop: fuelShop(b, spec); break;
    case BuildingKind::Pylon: pylon(b, spec), fx = 1.2f, fz = 0.2f; break;
    case BuildingKind::Garage: garage(b, spec); break;
    case BuildingKind::Lighthouse: lighthouse(b, spec), fx = fz = 3.4f; break;
    case BuildingKind::RadioMast: radioMast(b, spec), fx = fz = 2.4f; break;
    case BuildingKind::RadioHut: radioHut(b, spec); break;
    case BuildingKind::RuinWall: ruinWall(b, spec), fz = 0.7f; break;
    case BuildingKind::RuinTower: ruinTower(b, spec), fz = fx; break;
    case BuildingKind::RuinGate: ruinGate(b, spec), fx = 0.5f * spec.w + 1.9f, fz = 1.0f; break;
    case BuildingKind::Crusher: crusher(b, spec), fx = 2.6f, fz = 2.2f; break;
    case BuildingKind::Conveyor: conveyor(b, spec), fx = 0.7f, fz = 0.5f * spec.w; break;
    case BuildingKind::Silo: silo(b, spec), fz = fx; break;
    case BuildingKind::Dock: dock(b, spec), fx = 0.5f * spec.d, fz = 0.5f * spec.w; break;
    case BuildingKind::Tent: tent(b, spec), fx = 1.3f, fz = 1.5f; break;
    case BuildingKind::Fence: fence(b, spec), fz = 0.1f; break;
    case BuildingKind::Garden: garden(b, spec); break;
    case BuildingKind::PowerPole: powerPole(b, spec), fx = fz = 0.2f; break;
    case BuildingKind::Wire: wire(b, spec), fx = fz = 0.0f; break;
    case BuildingKind::Woodpile: woodpile(b, spec), fz = 0.6f; break;
    case BuildingKind::Cairn: cairn(b, spec), fx = fz = 0.8f; break;
    case BuildingKind::Signpost: signpost(b, spec), fx = fz = 0.3f; break;
    case BuildingKind::Well: well(b, spec), fx = fz = 1.2f; break;
    case BuildingKind::Shelter: shelter(b, spec); break;
    case BuildingKind::Bench: bench(b, spec), fx = 0.5f * spec.w, fz = 0.25f; break;
    case BuildingKind::Barrier: barrier(b, spec), fx = 0.5f * (spec.w > 0.0f ? spec.w : 4.0f), fz = 0.3f; break;
    case BuildingKind::TyreWall: tyreWall(b, spec), fx = 0.5f * (spec.w > 0.0f ? spec.w : 6.0f), fz = 0.33f; break;
    case BuildingKind::Flag: flag(b, spec), fx = fz = 0.25f; break;
    case BuildingKind::Grandstand: grandstand(b, spec); break;
    case BuildingKind::StartGantry: startGantry(b, spec), fx = 0.5f * spec.w + 0.7f, fz = 0.45f; break;
    case BuildingKind::RaceControl: raceControl(b, spec); break;
    case BuildingKind::MarshalPost: marshalPost(b, spec), fx = fz = 1.1f; break;
    case BuildingKind::Count: break;
    }
    g.footprintX = fx;
    g.footprintZ = fz;
    b.finish(spec, buildingIsStructure(spec.kind) ? "building" : "prop");
    return g;
}

} // namespace tf
