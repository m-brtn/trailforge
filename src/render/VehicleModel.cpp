#include "render/VehicleModel.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <tuple>
#include <vector>

namespace tf {
namespace {

constexpr float kPi = 3.14159265358979f;

// Quad with per-vertex normals; the winding follows the normals.
void quadN(MeshData& m, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d,
           const glm::vec3& na, const glm::vec3& nb, const glm::vec3& nc, const glm::vec3& nd, const Material& mat) {
    if (glm::dot(glm::cross(b - a, c - a), na + nb + nc + nd) >= 0.0f) {
        m.addTri(a, b, c, na, nb, nc, mat);
        m.addTri(a, c, d, na, nc, nd, mat);
    } else {
        m.addTri(a, c, b, na, nc, nb, mat);
        m.addTri(a, d, c, na, nd, nc, mat);
    }
}

// Parts under construction; each has its own mesh so a part can be added to from several places.
class Parts {
public:
    MeshData& add(const std::string& name, const std::string& parent, const glm::vec3& attach, bool deformable,
                  bool detachable) {
        m_parts.push_back({name, parent, attach, deformable, detachable});
        return m_meshes.emplace_back();
    }
    MeshData& operator[](const std::string& name) {
        for (std::size_t i = 0; i < m_parts.size(); ++i)
            if (m_parts[i].name == name) return m_meshes[i];
        return add(name, "", glm::vec3(0.0f), false, false);
    }
    void finish(VehicleVisual& v) {
        for (std::size_t i = 0; i < m_parts.size(); ++i) {
            m_parts[i].firstVertex = v.mesh.vertices.size();
            v.mesh.append(m_meshes[i]);
            m_parts[i].vertexCount = m_meshes[i].vertices.size();
        }
        v.parts = std::move(m_parts);
    }

private:
    std::vector<VehiclePart> m_parts;
    std::deque<MeshData> m_meshes;
};

// ================================================================ trailforge_suv
// Gasoline expedition SUV (Cars.png car #1): boxy five-door body, olive paint with orange accents, white roof,
// black flares and steel bumpers. Physics wheel positions: x = +-0.90 (track 1.80), z = +-1.40 (wheelbase
// 2.80), radius 0.40, hub at y = -0.35 at rest and -0.10 at full bump (attach 0.05, travel 0.15..0.55 m).
namespace suv {

constexpr float kSkinX = 0.88f;   // outer side skin (footprint half width)
constexpr float kFaceZ = 2.20f;   // front / rear faces
constexpr float kCornerR = 0.14f; // footprint corner radius
constexpr float kSill = -0.28f;   // rocker bottom = collision tub bottom (-0.30) + 2 cm
constexpr float kDeck = 0.655f;   // belt deck: hood edge and glasshouse base
constexpr float kRoofY = 1.20f;   // glasshouse top
constexpr float kRoofTop = 1.245f;
constexpr float kShieldZ = 0.92f, kShieldTopZ = 0.40f; // windshield base / top
constexpr float kFlareProud = 0.075f; // flare height above the body side
constexpr float kSeamFront = 0.76f, kSeamB = -0.30f, kSeamRear = -1.13f; // door seams on the lower skin

// Wheel-dependent body geometry, derived from the physics wheels so a different tire size or travel does not
// clip: axle positions, arch outline, flare band.
struct WheelFit {
    float axleZ[2] = {1.40f, -1.40f}; // front, rear
    std::vector<glm::vec2> arch;      // (dz, y) from the rear end over the top to the front end
    float flareBand = 0.12f; // flare width along the arch (scales with the tire)
    float innerX = 0.52f; // body points outboard of this are lifted out of the arches (any footprint segment)
};

// Arch styling: a circular arc of 1.2 wheel radii centred just below the full-bump hub, from the rocker over the
// top. Every point is pushed out until it clears the swept tire by kArchMargin.
constexpr float kArchMargin = 0.03f;
constexpr int kArchHalfPoints = 10;

WheelFit fitWheels(const VehicleConfig& c) {
    WheelFit f;
    f.axleZ[0] = 0.5f * c.wheelBase;
    f.axleZ[1] = -0.5f * c.wheelBase;
    float r = 0.0f, hubBump = -1e3f;
    for (const AxleConfig& a : c.axles) {
        r = std::max(r, a.wheelRadius);
        hubBump = std::max(hubBump, c.attachHeight - a.suspensionMinLength);
    }
    // Horizontal half reach of the swept tire (whole travel, 0..max steer) at height y, plus the margin.
    auto need = [&](float y) {
        float n = 0.0f;
        for (const AxleConfig& a : c.axles) {
            const float hi = c.attachHeight - a.suspensionMinLength, lo = c.attachHeight - a.suspensionMaxLength;
            const float dy = y > hi ? y - hi : (y < lo ? lo - y : 0.0f);
            const float rr = a.wheelRadius + kArchMargin;
            if (dy >= rr) continue;
            const float reach = std::sqrt(rr * rr - dy * dy), hw = 0.5f * a.wheelWidth + kArchMargin;
            const float steer = a.maxSteerAngleDeg * kPi / 180.0f;
            for (int k = 0; k <= 8; ++k) {
                const float s = steer * float(k) / 8.0f;
                n = std::max(n, reach * std::cos(s) + hw * std::sin(s));
            }
        }
        return n;
    };
    const float yc = hubBump - 0.05f * r;
    const float R = std::max(1.2f * r, hubBump + r + 1.5f * kArchMargin - yc);
    // Vertical from the rocker up to the arc centre, then the arc over the top.
    const float a0 = std::max(0.0f, std::asin(std::clamp((kSill - yc) / R, -1.0f, 1.0f)));
    std::vector<glm::vec2> half;
    if (a0 == 0.0f) half.push_back({-std::max(R, need(kSill)), kSill});
    for (int k = 0; k <= kArchHalfPoints; ++k) {
        const float a = a0 + (0.5f * kPi - a0) * float(k) / float(kArchHalfPoints);
        const float y = yc + R * std::sin(a);
        const float dz = k == kArchHalfPoints ? 0.0f : std::max(R * std::cos(a), need(y));
        half.push_back({-std::max(dz, k > 0 ? 0.0f : -half.front().x), y});
    }
    f.arch = half;
    for (int k = int(half.size()) - 2; k >= 0; --k) f.arch.push_back({-half[std::size_t(k)].x, half[std::size_t(k)].y});
    for (const AxleConfig& a : c.axles) {
        const float s = a.maxSteerAngleDeg * kPi / 180.0f;
        f.innerX = std::min(f.innerX, 0.5f * a.track - 0.5f * a.wheelWidth * std::cos(s) - a.wheelRadius * std::sin(s) -
                                          kArchMargin);
    }
    f.flareBand = std::clamp(0.30f * r, 0.10f, 0.14f);
    return f;
}

std::vector<glm::vec2> archAt(const WheelFit& w, float zc) {
    std::vector<glm::vec2> a;
    for (const glm::vec2& p : w.arch) a.push_back({zc + p.x, p.y});
    return a;
}

float archBottom(const WheelFit& w, float z) {
    float b = -1e3f;
    for (float zc : w.axleZ) {
        const float dz = z - zc;
        for (std::size_t k = 0; k + 1 < w.arch.size(); ++k)
            if (dz >= w.arch[k].x && dz <= w.arch[k + 1].x && w.arch[k + 1].x > w.arch[k].x) {
                const float t = (dz - w.arch[k].x) / (w.arch[k + 1].x - w.arch[k].x);
                b = std::max(b, w.arch[k].y + (w.arch[k + 1].y - w.arch[k].y) * t);
            }
    }
    return b;
}

const char* sideName(float sx) { return sx > 0 ? "left" : "right"; }
std::string sided(const char* base, float sx) { return std::string(base) + "_" + sideName(sx); }

// Lower body: one profile (inset d from the footprint, height y) swept around the rounded-rectangle
// footprint: chamfered rocker with black cladding, flat sides with a shoulder crease, chamfered shoulder,
// belt deck. The arches clip the profile on the sides; the last profile point closes the wheel-house wall.
void lowerBody(Parts& P, const style::Livery& L, const WheelFit& W) {
    const std::vector<glm::vec2> prof{{-0.38f, kSill}, {-0.07f, kSill},  {-0.01f, kSill + 0.03f},
                                      {0.010f, -0.20f}, {0.010f, -0.01f}, {0.0f, 0.0f},
                                      {0.0f, 0.26f},    {0.006f, 0.44f},  {-0.006f, 0.456f},
                                      {-0.006f, 0.57f}, {-0.035f, 0.635f}, {-0.08f, kDeck},
                                      {-0.38f, kDeck},  {-0.38f, kSill}};
    const int kClad0 = 1, kClad1 = 4, kPaint1 = 10; // segments [1,4] cladding, [5,10] paint, 11 deck
    std::vector<float> zs;
    const float zEnd = kFaceZ - kCornerR;
    auto addZ = [&](float z) {
        if (z < zEnd - 0.005f && z > -zEnd + 0.005f) zs.push_back(z);
    };
    for (float z = zEnd - 0.04f; z > -zEnd; z -= 0.04f) addZ(z);
    for (float zc : W.axleZ)
        for (const glm::vec2& p : W.arch) addZ(zc + p.x);
    for (float z : {kSeamFront, kSeamB, kSeamRear, 0.95f, 1.75f}) addZ(z);
    std::sort(zs.begin(), zs.end(), std::greater<float>());
    zs.erase(std::unique(zs.begin(), zs.end(), [](float a, float b) { return std::abs(a - b) < 0.004f; }), zs.end());
    const float tg = (0.70f) / (kSkinX - kCornerR);
    const style::Footprint f =
        style::roundedRect(kSkinX, -kFaceZ, kFaceZ, kCornerR, 0.0f, 5, zs, {-1.0f, -tg, -0.5f, 0.0f, 0.5f, tg, 1.0f});
    const int N = int(f.p.size()), R = int(prof.size());
    std::vector<std::vector<glm::vec3>> rows(static_cast<std::size_t>(R), std::vector<glm::vec3>(static_cast<std::size_t>(N)));
    std::vector<std::vector<char>> clamped(static_cast<std::size_t>(R), std::vector<char>(static_cast<std::size_t>(N), 0));
    for (int k = 0; k < N; ++k) {
        const bool side = f.seg[std::size_t(k)] == style::Footprint::Left || f.seg[std::size_t(k)] == style::Footprint::Right;
        for (int i = 0; i < R; ++i) {
            glm::vec3 p = f.p[std::size_t(k)] + f.n[std::size_t(k)] * prof[std::size_t(i)].x;
            p.y = prof[std::size_t(i)].y;
            // Side stations and the inset (floor, wheel-house wall) points of the corners clear the arches.
            const float b = side || std::abs(p.x) > W.innerX ? archBottom(W, p.z) : -1e3f;
            if (p.y < b) {
                p.y = b;
                clamped[std::size_t(i)][std::size_t(k)] = 1;
            }
            rows[std::size_t(i)][std::size_t(k)] = p;
        }
    }
    auto partAt = [&](int k) -> MeshData& {
        const glm::vec3& a = f.p[std::size_t(k)];
        const glm::vec3& b = f.p[std::size_t((k + 1) % N)];
        const glm::vec3 c = 0.5f * (a + b);
        const style::Footprint::Seg s = f.seg[std::size_t(k)];
        const float sx = c.x > 0 ? 1.0f : -1.0f;
        if (s == style::Footprint::Left || s == style::Footprint::Right) {
            if (c.z > kSeamFront) return P[sided("fender_front", sx)];
            if (c.z > kSeamB) return P[sided("door_front", sx)];
            if (c.z > kSeamRear) return P[sided("door_rear", sx)];
        }
        if (s == style::Footprint::Rear && f.seg[std::size_t((k + 1) % N)] == style::Footprint::Rear &&
            std::abs(c.x) < 0.70f)
            return P["tailgate"];
        return P["body_shell"];
    };
    style::skin(
        rows, true,
        [&](int i, int k) -> style::SkinCell {
            const int k1 = (k + 1) % N;
            const bool ceiling = clamped[std::size_t(i)][std::size_t(k)] && clamped[std::size_t(i)][std::size_t(k1)] &&
                                 clamped[std::size_t(i + 1)][std::size_t(k)] && clamped[std::size_t(i + 1)][std::size_t(k1)];
            const Material* m = &L.body;
            if (ceiling || i == R - 2) m = &L.liner;
            else if (i == 0) m = &L.underbody;
            else if (i >= kClad0 && i <= kClad1) m = &L.trim;
            else if (i > kPaint1) m = &L.body;
            return {&partAt(k), m};
        },
        [&](int i, int k, const glm::vec3&) {
            const int k1 = (k + 1) % N;
            if (clamped[std::size_t(i)][std::size_t(k)] && clamped[std::size_t(i)][std::size_t(k1)] &&
                clamped[std::size_t(i + 1)][std::size_t(k)] && clamped[std::size_t(i + 1)][std::size_t(k1)])
                return glm::vec3(0, -1, 0);
            const glm::vec2 t = prof[std::size_t(i + 1)] - prof[std::size_t(i)];
            return f.n[std::size_t(k)] * t.y + glm::vec3(0, -t.x, 0);
        });

    // Panel gaps: door seams follow the profile; tailgate seams on the rear face.
    MeshData& shell = P["body_shell"];
    auto seamAt = [&](int k, const glm::vec3& along) {
        for (int i = kClad1 + 1; i < kPaint1; ++i) {
            if (clamped[std::size_t(i)][std::size_t(k)]) continue;
            const glm::vec2 t = prof[std::size_t(i + 1)] - prof[std::size_t(i)];
            const glm::vec3 n = glm::normalize(f.n[std::size_t(k)] * t.y + glm::vec3(0, -t.x, 0));
            const glm::vec3 a = rows[std::size_t(i)][std::size_t(k)] + n * 0.0015f;
            const glm::vec3 b = rows[std::size_t(i + 1)][std::size_t(k)] + n * 0.0015f;
            const glm::vec3 w = along * 0.0035f;
            style::face(shell, a - w, a + w, b + w, b - w, n, L.liner);
        }
    };
    for (int k = 0; k < N; ++k) {
        const style::Footprint::Seg s = f.seg[std::size_t(k)];
        const glm::vec3& p = f.p[std::size_t(k)];
        if ((s == style::Footprint::Left || s == style::Footprint::Right) &&
            (std::abs(p.z - kSeamFront) < 0.003f || std::abs(p.z - kSeamB) < 0.003f || std::abs(p.z - kSeamRear) < 0.003f))
            seamAt(k, {0, 0, 1});
        if (s == style::Footprint::Rear && std::abs(std::abs(p.x) - 0.70f) < 0.003f) seamAt(k, {1, 0, 0});
    }
}

// Glasshouse: lofted from the belt deck (inset 8 cm from the body side) to the roof with a raked windshield and
// tumblehome; black A/B/C pillars and window frames, orange rear quarter panel, chamfered white roof.
void glasshouse(Parts& P, const style::Livery& L) {
    const std::vector<float> endX{-1.0f, -0.94f, -0.80f, -0.74f, 0.74f, 0.80f, 0.94f, 1.0f};
    const style::Footprint base =
        style::roundedRect(0.80f, -2.13f, kShieldZ, 0.06f, kDeck, 4, {0.84f, -0.26f, -0.34f, -1.09f, -1.16f, -2.02f}, endX);
    const style::Footprint top =
        style::roundedRect(0.72f, -2.09f, kShieldTopZ, 0.05f, kRoofY, 4, {0.33f, -0.26f, -0.34f, -1.09f, -1.16f, -1.99f}, endX);
    const int N = int(base.p.size());
    std::vector<std::vector<glm::vec3>> rows;
    for (float t : {0.0f, 0.08f, 0.92f, 1.0f}) {
        std::vector<glm::vec3> r(static_cast<std::size_t>(N));
        for (int k = 0; k < N; ++k) r[std::size_t(k)] = glm::mix(base.p[std::size_t(k)], top.p[std::size_t(k)], t);
        rows.push_back(r);
    }
    for (const glm::vec2 inset : {glm::vec2(0.035f, kRoofY + 0.032f), glm::vec2(0.065f, kRoofTop)}) {
        std::vector<glm::vec3> r(static_cast<std::size_t>(N));
        for (int k = 0; k < N; ++k) {
            r[std::size_t(k)] = top.p[std::size_t(k)] - top.n[std::size_t(k)] * inset.x;
            r[std::size_t(k)].y = inset.y;
        }
        rows.push_back(r);
    }
    enum Kind { Paint, Glass, Trim, Accent };
    auto column = [&](int k, Kind& kind) -> MeshData& {
        const int k1 = (k + 1) % N;
        const glm::vec3 c = 0.5f * (base.p[std::size_t(k)] + base.p[std::size_t(k1)]);
        const auto s0 = base.seg[std::size_t(k)], s1 = base.seg[std::size_t(k1)];
        const float sx = c.x > 0 ? 1.0f : -1.0f;
        using F = style::Footprint;
        if (s0 == F::Front && s1 == F::Front) {
            kind = std::abs(c.x) < 0.94f * 0.74f ? Glass : Trim;
            return P["windshield"];
        }
        if (s0 == F::Rear && s1 == F::Rear) {
            const float ax = std::abs(c.x) / 0.74f;
            kind = ax > 0.80f ? Paint : ax > 0.74f ? Trim : Glass;
            return ax > 0.80f ? P["body_shell"] : P["tailgate"];
        }
        const bool side = (s0 == F::Left || s0 == F::Right) && (s1 == F::Left || s1 == F::Right);
        if (!side) {
            kind = c.z > 0.0f ? Trim : Paint; // A pillar wrap / D pillar corner
            return P["body_shell"];
        }
        if (c.z > 0.84f) kind = Trim;
        else if (c.z > -0.26f) { kind = Glass; return P[sided("door_front", sx)]; }
        else if (c.z > -0.34f) kind = Trim;
        else if (c.z > -1.09f) { kind = Glass; return P[sided("door_rear", sx)]; }
        else if (c.z > -1.16f) kind = Trim;
        else if (c.z > -2.02f) kind = Accent;
        else kind = Paint;
        return P["body_shell"];
    };
    const glm::vec3 inside(0.0f, 0.90f, -0.85f);
    style::skin(
        rows, true,
        [&](int i, int k) -> style::SkinCell {
            Kind kind = Paint;
            MeshData& m = column(k, kind);
            if (i >= 3) return {&P["body_shell"], &L.roof};
            const Material* mat = kind == Glass ? (i == 1 ? &L.glass : &L.trim)
                                  : kind == Trim ? &L.trim
                                  : kind == Accent ? &L.accent
                                                   : &L.body;
            return {&m, mat};
        },
        [&](int, int, const glm::vec3& c) { return c - inside; });
    // Roof cap and the orange decal band behind the windshield.
    MeshData& shell = P["body_shell"];
    const std::vector<glm::vec3>& cap = rows.back();
    glm::vec3 center(0.0f);
    for (const glm::vec3& p : cap) center += p;
    center /= float(cap.size());
    for (int k = 0; k < N; ++k) {
        const glm::vec3 a = cap[std::size_t(k)], b = cap[std::size_t((k + 1) % N)];
        const glm::vec3 up(0, 1, 0);
        if (glm::dot(glm::cross(a - center, b - center), up) >= 0.0f) shell.addTri(center, a, b, up, up, up, L.roof);
        else shell.addTri(center, b, a, up, up, up, L.roof);
    }
    style::face(shell, {-0.64f, kRoofTop + 0.001f, 0.33f}, {0.64f, kRoofTop + 0.001f, 0.33f},
                {0.64f, kRoofTop + 0.001f, 0.16f}, {-0.64f, kRoofTop + 0.001f, 0.16f}, {0, 1, 0}, L.accent);
    for (float sx : {1.0f, -1.0f})
        shell.addBar({sx * 0.725f, kRoofY + 0.004f, -2.03f}, {sx * 0.725f, kRoofY + 0.004f, 0.34f}, 0.010f, 0.008f,
                     {0, 1, 0}, L.trim); // drip rails
}

// Hood: cross-section (chamfered edges, panel gaps to the fender tops, raised center) lofted along z with a
// crown that drops to a rounded nose over the grille. The fender tops carry the orange accent.
void hood(Parts& P, const style::Livery& L) {
    const std::vector<float> xs{-0.79f, -0.772f, -0.52f, -0.512f, -0.504f, -0.36f, -0.34f,
                                0.34f,  0.36f,   0.504f, 0.512f,  0.52f,   0.772f, 0.79f};
    const std::vector<float> dy{-0.035f, 0.0f, 0.0f, -0.004f, 0.0f, 0.0f, 0.02f,
                                0.02f,   0.0f, 0.0f, -0.004f, 0.0f, 0.0f, -0.035f};
    struct Station { float z, y; };
    const Station st[] = {{0.925f, 0.705f}, {0.95f, 0.704f}, {1.10f, 0.699f}, {1.30f, 0.692f}, {1.50f, 0.685f},
                          {1.75f, 0.676f},  {1.90f, 0.670f}, {2.00f, 0.667f}, {2.06f, 0.664f}, {2.11f, 0.656f},
                          {2.15f, 0.644f},  {2.18f, 0.626f}, {2.20f, 0.602f}, {2.212f, 0.572f}, {2.195f, 0.556f}};
    std::vector<std::vector<glm::vec3>> rows;
    for (const Station& s : st) {
        const float bulge = std::clamp((2.10f - s.z) / 0.15f, 0.0f, 1.0f);
        std::vector<glm::vec3> r;
        for (std::size_t j = 0; j < xs.size(); ++j) {
            const bool center = j == 6 || j == 7;
            r.push_back({xs[j], s.y + (center ? dy[j] * bulge : dy[j]), s.z});
        }
        rows.push_back(r);
    }
    MeshData& h = P["hood"];
    style::skin(
        rows, false,
        [&](int i, int k) -> style::SkinCell {
            const float z = 0.5f * (st[i].z + st[i + 1].z);
            const bool outer = k <= 1 || k >= 12;
            if (k == 2 || k == 3 || k == 9 || k == 10) return {&h, &L.liner};
            return {&h, outer && z > 0.95f && z < 1.75f ? &L.accent : &L.body};
        },
        [](int, int, const glm::vec3& c) { return c - glm::vec3(0.0f, 0.3f, 1.5f); });
    for (float sx : {1.0f, -1.0f}) {
        h.addBox({sx * 0.43f, 0.6995f, 1.12f}, {0.06f, 0.002f, 0.075f}, L.liner); // vents
        for (int k = 0; k < 4; ++k) h.addBox({sx * 0.43f, 0.7015f, 1.06f + 0.04f * k}, {0.055f, 0.0025f, 0.006f}, L.trim);
    }
}

void buildSuv(VehicleVisual& v, const VehicleConfig& cfg) {
    const WheelFit W = fitWheels(cfg);
    const style::Livery L = style::oliveExpedition();
    const Material chrome = style::metal(glm::vec3(0.92f), 0.05f, 0.0f);
    const Material head = style::led(MatId::Headlight, {1.0f, 0.95f, 0.88f});
    const Material barLed = style::led(MatId::Headlight, {0.85f, 0.92f, 1.0f});
    const Material brake = style::led(MatId::BrakeLight, {1.0f, 0.04f, 0.02f});
    const Material reverse = style::led(MatId::ReverseLight, {1.0f, 1.0f, 1.0f});
    const Material amber = style::led(MatId::Amber, {1.0f, 0.42f, 0.04f});
    const Material red = style::paint(style::srgb(0.80f, 0.12f, 0.08f), 0.40f, 0.6f);
    const Material olive = style::plastic(style::srgb(0.33f, 0.36f, 0.22f), 0.65f, 0.6f);
    const Material iron{MatId::Generic, {0.06f, 0.06f, 0.06f}, 0.50f, 0.5f, 1.0f};
    const glm::vec3 up(0, 1, 0), fwd(0, 0, 1);

    Parts P;
    MeshData& chassis = P.add("chassis", "", {0, 0, 0}, false, false);
    P.add("body_shell", "chassis", {0, 0.30f, 0}, true, false);
    for (float sx : {1.0f, -1.0f}) {
        P.add(sided("fender_front", sx), "body_shell", {sx * kSkinX, 0.3f, 1.45f}, true, true);
        P.add(sided("door_front", sx), "body_shell", {sx * kSkinX, 0.3f, kSeamFront}, true, true); // hinge at the front
        P.add(sided("door_rear", sx), "body_shell", {sx * kSkinX, 0.3f, kSeamB}, true, true);
    }
    P.add("hood", "body_shell", {0, kDeck, kShieldZ}, true, true);                       // hinge at the cowl
    MeshData& tailgate = P.add("tailgate", "body_shell", {-0.70f, 0.4f, -kFaceZ}, true, true); // side hinge (right)
    P.add("windshield", "body_shell", {0, 0.93f, 0.66f}, false, true);
    MeshData& grille = P.add("grille", "body_shell", {0, 0.41f, kFaceZ}, true, true);
    MeshData& shell = P["body_shell"];

    // ---- Chassis: ladder frame, tub, differentials, exhaust
    for (float sx : {1.0f, -1.0f})
        chassis.addBeveledBox({sx * 0.40f, -0.14f, 0.0f}, {0.045f, 0.07f, 2.12f}, 0.01f, L.steel);
    for (float z : {-2.0f, -0.7f, 0.5f, 2.0f}) chassis.addBox({0.0f, -0.14f, z}, {0.40f, 0.04f, 0.04f}, L.steel);
    chassis.addBox({0.0f, 0.0f, 0.0f}, {0.50f, 0.27f, 2.10f}, L.underbody);
    for (float zc : W.axleZ) {
        chassis.addCylinder({0.0f, -0.24f, zc - 0.10f}, {0.0f, -0.24f, zc + 0.07f}, 0.12f, 18, iron);
        chassis.addCylinder({0.0f, -0.24f, zc + 0.07f}, {0.0f, -0.24f, zc + 0.10f}, 0.085f, 18, iron);
        chassis.addBox({0.0f, -0.22f, zc}, {0.16f, 0.05f, 0.05f}, iron);
    }
    chassis.addCylinder({-0.45f, -0.29f, -1.55f}, {-0.45f, -0.29f, -2.30f}, 0.032f, 12, L.alu);
    P.add("skid_plate", "chassis", {0, -0.26f, 2.20f}, true, true)
        .addBar({0.0f, -0.24f, 2.26f}, {0.0f, -0.31f, 1.66f}, 0.42f, 0.008f, up, L.alu, 0.004f);

    // ---- Body shell, glasshouse, hood, flares
    lowerBody(P, L, W);
    // Wheel-house walls inboard of the tires: close the view through the arches above the tub.
    for (float sx : {1.0f, -1.0f})
        for (float zc : W.axleZ) {
            const float top = W.arch[W.arch.size() / 2].y + 0.03f, x = std::min(0.50f, W.innerX) - 0.006f;
            shell.addBox({sx * x, 0.5f * (0.20f + top), zc}, {0.004f, 0.5f * (top - 0.20f), -W.arch.front().x},
                         L.liner);
        }
    glasshouse(P, L);
    hood(P, L);
    shell.addBar({-0.80f, 0.708f, kShieldZ + 0.004f}, {0.80f, 0.708f, kShieldZ + 0.004f}, 0.018f, 0.012f, up, L.trim);
    for (float sx : {1.0f, -1.0f}) {
        for (float zc : W.axleZ) {
            const std::string name = std::string("flare_") + (zc > 0 ? "front_" : "rear_") + sideName(sx);
            style::fenderFlare(P.add(name, zc > 0 ? sided("fender_front", sx) : "body_shell", {sx * kSkinX, 0.10f, zc},
                                     true, true),
                               sx, archAt(W, zc), W.flareBand, kSkinX, kFlareProud, L.trim, L.liner);
        }
        P[sided("fender_front", sx)].addBox({sx * (kSkinX + 0.004f), 0.30f, 2.03f}, {0.004f, 0.02f, 0.04f}, amber);
        shell.addBox({sx * (kSkinX + 0.004f), 0.30f, -2.04f}, {0.004f, 0.02f, 0.03f}, amber);
        for (float z : {0.28f, -0.64f})
            P[z > kSeamB ? sided("door_front", sx) : sided("door_rear", sx)].addBeveledBox(
                {sx * 0.886f, 0.52f, z}, {0.010f, 0.016f, 0.07f}, 0.006f, L.trim);
        // Orange side stripe on the doors (decal).
        for (const auto& [z0, z1, part] : {std::tuple{kSeamFront - 0.02f, kSeamB + 0.01f, "door_front"},
                                           std::tuple{kSeamB - 0.01f, kSeamRear + 0.02f, "door_rear"}})
            style::panelGap(P[sided(part, sx)], {sx * 0.8847f, 0.40f, z0}, {sx * 0.8847f, 0.40f, z1}, {sx, 0, 0}, 0.018f,
                            L.accent);
        style::rockSlider(P.add(sided("rock_slider", sx), "chassis", {sx * 0.80f, kSill, 0}, true, true), sx, 0.95f,
                          kSill - 0.02f, -0.70f, 0.60f, L.alu, L.accent, L.steel);
        style::mirror(P.add(sided("mirror", sx), sided("door_front", sx), {sx * 0.82f, 0.70f, 0.74f}, false, true), sx,
                      {sx * 0.82f, 0.70f, 0.74f}, L.trim, chrome);
    }

    // ---- Front: grille, headlights, bumper with winch, shackles and uprights
    grille.addBeveledBox({0.0f, 0.41f, kFaceZ + 0.006f}, {0.44f, 0.14f, 0.012f}, 0.006f, L.liner);
    for (int k = 0; k < 4; ++k)
        grille.addBeveledBox({0.0f, 0.315f + 0.065f * k, kFaceZ + 0.018f}, {0.42f, 0.010f, 0.008f}, 0.004f, L.gunmetal);
    for (float x : {-0.43f, -0.15f, 0.15f, 0.43f})
        grille.addBox({x, 0.41f, kFaceZ + 0.018f}, {0.010f, 0.13f, 0.010f}, L.gunmetal);
    grille.addBox({0.0f, 0.558f, kFaceZ + 0.010f}, {0.45f, 0.008f, 0.010f}, L.stripe); // white bar over the grille
    grille.addBeveledBox({0.0f, 0.505f, kFaceZ + 0.029f}, {0.05f, 0.015f, 0.004f}, 0.003f, L.alu); // badge
    for (float sx : {1.0f, -1.0f}) {
        MeshData& hl = P.add(sided("headlight", sx), "body_shell", {sx * 0.64f, 0.45f, kFaceZ}, false, true);
        const glm::vec3 c(sx * 0.64f, 0.45f, kFaceZ + 0.008f);
        hl.addBeveledBox(c, {0.15f, 0.075f, 0.014f}, 0.007f, L.trim);
        style::ledRing(hl, c + fwd * 0.014f, fwd, {1, 0, 0}, 0.13f, 0.058f, 0.028f, 0.011f, head);
        style::roundLamp(hl, c + fwd * 0.018f, fwd, 0.042f, 0.01f, L.gunmetal, head);
    }
    {
        MeshData& b = P.add("bumper_front", "chassis", {0, 0.08f, kFaceZ}, true, true);
        b.addBeveledBox({0.0f, 0.08f, 2.255f}, {0.28f, 0.14f, 0.055f}, 0.015f, L.steel);
        b.addBeveledBox({0.0f, -0.07f, 2.34f}, {0.30f, 0.015f, 0.08f}, 0.006f, L.steel); // winch tray
        for (float sx : {1.0f, -1.0f}) {
            b.addBeveledBox({sx * 0.54f, 0.08f, 2.31f}, {0.26f, 0.14f, 0.11f}, 0.025f, L.steel);
            b.addBar({sx * 0.78f, 0.08f, 2.31f}, {sx * 0.98f, 0.11f, 2.12f}, 0.06f, 0.12f, up, L.steel, 0.02f); // wings
            b.addBox({sx * 0.62f, 0.15f, 2.421f}, {0.065f, 0.020f, 0.003f}, amber);
            b.addBox({sx * 0.40f, 0.15f, 2.421f}, {0.040f, 0.018f, 0.003f}, head);
            b.addCylinder({sx * 0.84f, 0.21f, 2.31f}, {sx * 0.82f, 0.50f, 2.27f}, 0.022f, 12, L.steel); // uprights
            style::shackle(P.add(sided("tow_hook_front", sx), "bumper_front", {sx * 0.40f, -0.02f, 2.42f}, false, true),
                           {sx * 0.40f, -0.02f, 2.42f}, fwd, L.hooks);
        }
        style::winch(P.add("winch", "bumper_front", {0, 0.06f, 2.31f}, false, true), {0.0f, 0.06f, 2.37f}, L.gunmetal,
                     L.hooks, L.trim);
    }

    // ---- Rear: tail lights, bumper, spare carrier, jerry cans, ladder
    tailgate.addBeveledBox({0.0f, 0.18f, -kFaceZ - 0.006f}, {0.16f, 0.012f, 0.006f}, 0.003f, L.trim); // handle
    for (float sx : {1.0f, -1.0f}) {
        MeshData& tl = P.add(sided("taillight", sx), "body_shell", {sx * 0.77f, 0.44f, -kFaceZ}, false, true);
        tl.addBeveledBox({sx * 0.765f, 0.44f, -kFaceZ - 0.006f}, {0.058f, 0.15f, 0.012f}, 0.006f, L.trim);
        tl.addBox({sx * 0.765f, 0.49f, -kFaceZ - 0.019f}, {0.044f, 0.09f, 0.002f}, brake);
        tl.addBox({sx * 0.765f, 0.335f, -kFaceZ - 0.019f}, {0.044f, 0.03f, 0.002f}, reverse);
    }
    {
        MeshData& b = P.add("bumper_rear", "chassis", {0, 0.02f, -kFaceZ}, true, true);
        // Swept along the rear face and around the body corners: top deck, chamfer, upright face with a groove,
        // lower chamfer sloping back under the body.
        std::vector<glm::vec3> path, normals;
        for (float sx : {-1.0f, 1.0f})
            for (int k = 0; k <= 6; ++k) {
                const float a = 0.5f * kPi * float(sx < 0 ? 6 - k : k) / 6.0f; // 0 = side, pi/2 = rear
                const glm::vec3 nrm(sx * std::cos(a), 0.0f, -std::sin(a));
                path.push_back(glm::vec3(sx * (kSkinX - kCornerR), 0.0f, -kFaceZ + kCornerR) + nrm * kCornerR);
                normals.push_back(nrm);
            }
        for (float sx : {-1.0f, 1.0f}) {
            const std::size_t end = sx < 0 ? 0 : path.size() - 1;
            path.insert(sx < 0 ? path.begin() : path.end(), path[end] + glm::vec3(0, 0, 0.04f));
            normals.insert(sx < 0 ? normals.begin() : normals.end(), normals[end]);
        }
        style::sweepProfile(b, path, normals,
                            {{-0.01f, 0.15f}, {0.075f, 0.15f}, {0.105f, 0.12f}, {0.105f, 0.045f}, {0.096f, 0.035f},
                             {0.096f, 0.020f}, {0.105f, 0.010f}, {0.105f, -0.065f}, {0.085f, -0.11f}, {0.04f, -0.15f},
                             {-0.01f, -0.15f}},
                            L.steel);
        b.addBeveledBox({0.0f, -0.12f, -2.27f}, {0.30f, 0.012f, 0.06f}, 0.004f, L.steel); // centre step plate
        b.addBox({0.0f, -0.08f, -2.36f}, {0.04f, 0.04f, 0.05f}, L.steel);               // hitch receiver
        b.addBox({0.0f, -0.08f, -2.411f}, {0.028f, 0.028f, 0.002f}, L.liner);
        for (float sx : {1.0f, -1.0f}) {
            // Lamp pockets in the bumper face: reverse lamp with a red reflector strip below.
            b.addBox({sx * 0.62f, 0.085f, -kFaceZ - 0.1055f}, {0.080f, 0.030f, 0.002f}, L.liner);
            b.addBox({sx * 0.62f, 0.092f, -kFaceZ - 0.1075f}, {0.064f, 0.016f, 0.001f}, reverse);
            b.addBox({sx * 0.62f, 0.066f, -kFaceZ - 0.1075f}, {0.064f, 0.005f, 0.001f}, brake);
            style::shackle(P.add(sided("tow_hook_rear", sx), "bumper_rear", {sx * 0.40f, -0.03f, -2.31f}, false, true),
                           {sx * 0.40f, -0.03f, -2.31f}, -fwd, L.hooks);
        }
        style::spareCarrier(P.add("spare_carrier_rear", "tailgate", {0, 0.62f, -kFaceZ}, false, true),
                            {0.0f, 0.62f, -kFaceZ}, -fwd, 0.30f, L.steel);
        style::jerryCan(P.add("jerry_can_left", "tailgate", {0.54f, 0.46f, -kFaceZ}, false, true), {0.54f, 0.46f, -2.27f},
                        -fwd, L.accent, L.trim);
        style::jerryCan(P.add("jerry_can_right", "tailgate", {-0.54f, 0.46f, -kFaceZ}, false, true),
                        {-0.54f, 0.46f, -2.27f}, -fwd, red, L.trim);
        style::ladder(P.add("ladder", "body_shell", {-0.70f, 1.0f, -kFaceZ}, false, true), {-0.70f, 0.72f, -2.23f},
                      {-0.70f, 1.30f, -2.17f}, 0.11f, {1, 0, 0}, L.steel);
    }
    v.spares.push_back({"spare_rear", "spare_carrier_rear", {0.0f, 0.62f, -kFaceZ - 0.026f}, {0, 0, -1}, true});

    // ---- Snorkel (right side)
    style::snorkel(P.add("snorkel", "fender_front_right", {-0.93f, 0.32f, 0.86f}, true, true), -1.0f,
                   {-0.93f, 0.32f, 0.86f}, {-0.93f, 0.66f, 0.86f}, {-0.80f, 1.22f, 0.42f}, L.trim, L.liner);

    // ---- Roof rack with light bar, pods, cases (the spare wheel is drawn separately)
    {
        const float rackY = kRoofTop + 0.08f;
        MeshData& rack = P.add("roof_rack", "body_shell", {0, kRoofTop, -0.92f}, true, true);
        style::roofRack(rack, 0.66f, -2.00f, 0.16f, kRoofTop, 0.08f, L.steel);
        for (float sx : {1.0f, -1.0f})
            for (float z : {0.16f, -2.00f})
                rack.addBeveledBox({sx * 0.66f, rackY, z}, {0.028f, 0.028f, 0.028f}, 0.008f, L.accent); // rack end caps
        MeshData& lb = P.add("light_bar", "roof_rack", {0, rackY, 0.22f}, false, true);
        style::lightBar(lb, {0.0f, rackY + 0.06f, 0.22f}, 0.48f, L.trim, barLed);
        for (float sx : {1.0f, -1.0f})
            style::roundLamp(lb, {sx * 0.62f, rackY + 0.06f, 0.26f}, fwd, 0.05f, 0.06f, L.trim, barLed);
        style::cargoCase(P.add("case_front", "roof_rack", {-0.36f, rackY, -0.12f}, false, true),
                         {-0.36f, rackY + 0.07f, -0.12f}, {0.22f, 0.07f, 0.15f}, L.accent, L.trim);
        style::cargoCase(P.add("case_rear", "roof_rack", {0.34f, rackY, -1.62f}, false, true),
                         {0.34f, rackY + 0.08f, -1.62f}, {0.24f, 0.08f, 0.22f}, olive, L.trim);
        v.spares.insert(v.spares.begin(), {"spare_roof", "roof_rack", {0.0f, rackY + 0.004f, -0.86f}, {0, 1, 0}, true});
    }

    P.finish(v);
    v.wheel.spokes = 8;
    v.wheel.beadlock = false;
    v.wheel.rim = {MatId::Generic, glm::vec3(0.040f), 0.40f, 0.9f, 0.6f};
    v.wheel.ring = {MatId::Generic, glm::vec3(0.22f), 0.35f, 1.0f, 0.5f};
}

} // namespace suv
} // namespace

const VehiclePart* VehicleVisual::find(const std::string& name) const {
    for (const VehiclePart& p : parts)
        if (p.name == name) return &p;
    return nullptr;
}

VehicleVisual buildVehicleVisual(const VehicleConfig& config) {
    VehicleVisual v;
    // "trailforge_suv" is the only model; the loader rejects unknown ids
    suv::buildSuv(v, config);
    return v;
}

style::WheelStyle axleWheelStyle(const VehicleVisual& visual, const AxleConfig& axle) {
    style::WheelStyle w = visual.wheel;
    w.rimFraction = axle.rimRatio;
    w.tread = axle.tread == "all_terrain" ? style::Tread::AllTerrain : style::Tread::MudTerrain;
    return w;
}

MeshData buildAxleWheelMesh(const VehicleVisual& visual, const AxleConfig& axle) {
    return style::buildWheel(axle.wheelRadius, axle.wheelWidth, axleWheelStyle(visual, axle));
}

glm::vec2 wheelMeshSize(const MeshData& wheel) {
    float r = 0.0f, x0 = 0.0f, x1 = 0.0f;
    for (const auto& v : wheel.vertices) {
        const glm::vec3& p = v.position;
        r = std::max(r, std::sqrt(p.y * p.y + p.z * p.z));
        x0 = std::min(x0, p.x);
        x1 = std::max(x1, p.x);
    }
    return {r, x1 - x0};
}

bool sameWheelFit(const VehicleConfig& a, const VehicleConfig& b) {
    if (a.visualModel != b.visualModel || a.wheelBase != b.wheelBase || a.attachHeight != b.attachHeight) return false;
    for (std::size_t i = 0; i < a.axles.size(); ++i) {
        const AxleConfig &x = a.axles[i], &y = b.axles[i];
        if (x.track != y.track || x.wheelRadius != y.wheelRadius || x.wheelWidth != y.wheelWidth ||
            x.suspensionMinLength != y.suspensionMinLength || x.suspensionMaxLength != y.suspensionMaxLength ||
            x.maxSteerAngleDeg != y.maxSteerAngleDeg || x.rimRatio != y.rimRatio || x.tread != y.tread)
            return false;
    }
    return true;
}

glm::mat4 spareWheelMatrix(const SpareMount& mount, float radius, float width) {
    (void)radius;
    const glm::vec3 x = glm::normalize(mount.outward);
    const glm::vec3 ref = std::abs(x.y) > 0.9f ? glm::vec3(0, 0, 1) : glm::vec3(0, 1, 0);
    const glm::vec3 z = glm::normalize(glm::cross(x, ref));
    const glm::vec3 y = glm::cross(z, x);
    glm::mat4 m(1.0f);
    m[0] = glm::vec4(x, 0);
    m[1] = glm::vec4(y, 0);
    m[2] = glm::vec4(z, 0);
    m[3] = glm::vec4(mount.seat + x * (0.5f * width), 1);
    return m;
}

MeshData buildUnitSpringMesh() {
    MeshData m;
    const Material coil{MatId::Generic, {0.16f, 0.025f, 0.012f}, 0.40f, 0.1f, 0.8f};
    const float rc = 0.052f, rw = 0.0095f, turns = 7.5f, lref = 0.62f;
    constexpr int kPerTurn = 14, kSides = 6;
    const int n = int(turns * kPerTurn);
    auto center = [&](float t) {
        const float a = 2.0f * kPi * turns * t;
        return glm::vec3(rc * std::cos(a), rc * std::sin(a), t);
    };
    for (int i = 0; i < n; ++i) {
        const float t0 = float(i) / float(n), t1 = float(i + 1) / float(n);
        auto ringAt = [&](float t, int k) {
            const float a = 2.0f * kPi * turns * t;
            const glm::vec3 radial(std::cos(a), std::sin(a), 0.0f);
            const float b = 2.0f * kPi * float(k) / float(kSides);
            // Wire cross-section in the radial/axial plane; axial extent pre-divided by the typical stretch.
            const glm::vec3 off = radial * (rw * std::cos(b)) + glm::vec3(0, 0, rw * std::sin(b) / lref);
            const glm::vec3 nrm = glm::normalize(radial * std::cos(b) + glm::vec3(0, 0, std::sin(b)));
            return std::pair<glm::vec3, glm::vec3>(center(t) + off, nrm);
        };
        for (int k = 0; k < kSides; ++k) {
            const auto a0 = ringAt(t0, k), a1 = ringAt(t0, k + 1), b0 = ringAt(t1, k), b1 = ringAt(t1, k + 1);
            quadN(m, a0.first, b0.first, b1.first, a1.first, a0.second, b0.second, b1.second, a1.second, coil);
        }
    }
    return m;
}

MeshData buildUnitTubeMesh(float radius) {
    MeshData m;
    const Material steel{MatId::Generic, {0.035f, 0.035f, 0.037f}, 0.45f, 0.4f, 1.0f};
    m.addCylinder({0, 0, 0}, {0, 0, 1}, radius, 10, steel);
    return m;
}

MeshData buildShockBodyMesh() {
    MeshData m;
    const Material gold{MatId::Generic, {0.30f, 0.22f, 0.08f}, 0.35f, 1.0f, 0.8f};
    const Material black{MatId::Generic, {0.03f, 0.03f, 0.03f}, 0.5f, 0.3f, 0.8f};
    m.addCylinder({0, 0, 0}, {0, 0, kShockBodyLength}, 0.034f, 14, gold);
    m.addCylinder({0, 0, kSpringPerch - 0.02f}, {0, 0, kSpringPerch}, 0.066f, 18, black); // lower spring perch
    m.addCylinder({0.0f, 0.045f, 0.03f}, {0.0f, 0.045f, 0.20f}, 0.022f, 10, gold); // reservoir
    return m;
}

MeshData buildShockShaftMesh() {
    MeshData m;
    const Material chrome{MatId::Generic, {0.92f, 0.92f, 0.92f}, 0.10f, 1.0f, 0.3f};
    const Material black{MatId::Generic, {0.03f, 0.03f, 0.03f}, 0.5f, 0.3f, 0.5f};
    m.addCylinder({0, 0, 0}, {0, 0, kShockShaftLength}, 0.014f, 10, chrome);
    m.addCylinder({0, 0, 0}, {0, 0, 0.025f}, 0.064f, 18, black); // top hat / upper perch
    return m;
}

SuspensionPose suspensionPose(const glm::vec3& hub, float attachZ) {
    SuspensionPose s;
    const float sx = hub.x >= 0.0f ? 1.0f : -1.0f;
    const float zc = attachZ;
    s.shockTop = {sx * 0.585f, 0.29f, zc};
    s.armPivotA = {sx * 0.22f, -0.27f, zc + 0.24f};
    s.armPivotB = {sx * 0.22f, -0.27f, zc - 0.24f};
    s.armOuter = {hub.x - sx * 0.11f, hub.y - 0.11f, hub.z};
    const glm::vec3 pivotMid(sx * 0.22f, -0.27f, zc);
    const float span = std::abs(s.armOuter.x) - 0.22f;
    const float t = span > 1e-3f ? std::clamp((0.585f - 0.22f) / span, 0.0f, 1.0f) : 1.0f;
    s.shockLow = glm::mix(pivotMid, s.armOuter, t) + glm::vec3(0.0f, 0.03f, 0.0f);
    s.shaftInner = {sx * 0.13f, -0.24f, zc};
    s.shaftOuter = {hub.x - sx * 0.09f, hub.y, hub.z};
    return s;
}

glm::mat4 segmentMatrix(const glm::vec3& a, const glm::vec3& b) {
    const glm::vec3 d = b - a;
    const float len = glm::length(d);
    const glm::vec3 z = len > 1e-6f ? d / len : glm::vec3(0, 0, 1);
    glm::vec3 x = std::abs(z.y) < 0.9f ? glm::cross(glm::vec3(0, 1, 0), z) : glm::cross(glm::vec3(1, 0, 0), z);
    x = glm::normalize(x);
    const glm::vec3 y = glm::cross(z, x);
    glm::mat4 m(1.0f);
    m[0] = glm::vec4(x, 0);
    m[1] = glm::vec4(y, 0);
    m[2] = glm::vec4(d, 0);
    m[3] = glm::vec4(a, 1);
    return m;
}

glm::mat4 aimMatrix(const glm::vec3& a, const glm::vec3& b) {
    glm::mat4 m = segmentMatrix(a, b);
    const float len = glm::length(b - a);
    m[2] = len > 1e-6f ? m[2] / len : glm::vec4(0, 0, 1, 0);
    return m;
}


} // namespace tf
