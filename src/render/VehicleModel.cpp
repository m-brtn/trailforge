#include "render/VehicleModel.h"

#include "render/FieldMesh.h"
#include "vehicle/VehicleBlueprint.h"
#include "vehicle/VehicleBody.h"
#include "vehicle/WheelSweep.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <tuple>
#include <unordered_map>
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
    void forEachMesh(const std::function<void(MeshData&)>& fn) {
        for (MeshData& m : m_meshes) fn(m);
    }

private:
    std::vector<VehiclePart> m_parts;
    std::deque<MeshData> m_meshes;
};

const char* sideName(float sx) { return sx > 0 ? "left" : "right"; }
std::string sided(const char* base, float sx) { return std::string(base) + "_" + sideName(sx); }

// ================================================================ kit bodies (bodyShape "kit", the default)
// Lofted StyleKit panels around the physics wheels (README "Vehicle rendering"); the collision is the three boxes of
// vehicle/VehicleShape.h. Laid out for track 1.80 and wheelbase 2.80 (wheels at x = +-0.90, z = +-1.40); the
// arches, flares and wheel-house walls follow the configured wheels, travel and steering.
namespace kit {

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
// tumblehome; black A/B/C pillars and window frames, accent rear quarter panel (or a quarter window with a
// slanted accent graphic when `quarterWindow`), chamfered roof.
void glasshouse(Parts& P, const style::Livery& L, bool quarterWindow = false) {
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
        else if (c.z > -2.02f) kind = quarterWindow ? Glass : Accent;
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
    if (!quarterWindow) return;
    // Slanted accent graphic over the rear part of the quarter window (decal on the glass plane).
    for (float sx : {1.0f, -1.0f}) {
        const glm::vec3 n = glm::normalize(glm::vec3(sx * (kRoofY - kDeck), 0.08f, 0.0f));
        auto at = [&](float t, float z) {
            return glm::vec3(sx * (0.80f - 0.08f * t), kDeck + t * (kRoofY - kDeck), z) + n * 0.003f;
        };
        style::face(shell, at(0.10f, -1.86f), at(0.10f, -2.00f), at(0.90f, -1.97f), at(0.90f, -1.50f), n, L.accent);
    }
}

// Hood styling: the fender tops carry the accent paint between accentZ0 and accentZ1; `ventPanel` replaces the
// two small louvred vents by one closed dark panel in the center (electric: no radiator air to exhaust).
struct HoodStyle {
    float accentZ0 = 0.95f, accentZ1 = 1.75f;
    bool ventPanel = false;
};

// Hood: cross-section (chamfered edges, panel gaps to the fender tops, raised center) lofted along z with a
// crown that drops to a rounded nose over the grille.
void hood(Parts& P, const style::Livery& L, const HoodStyle& hs = {}) {
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
            const bool outer = k <= 1 || k >= 11;
            if (k == 2 || k == 3 || k == 9 || k == 10) return {&h, &L.liner};
            return {&h, outer && z > hs.accentZ0 && z < hs.accentZ1 ? &L.accent : &L.body};
        },
        [](int, int, const glm::vec3& c) { return c - glm::vec3(0.0f, 0.3f, 1.5f); });
    if (hs.ventPanel) {
        // Hood crown at the center columns: y(z) from the stations plus the 2 cm bulge.
        auto crown = [&](float z) {
            for (std::size_t i = 0; i + 1 < std::size(st); ++i)
                if (z >= st[i].z && z <= st[i + 1].z)
                    return st[i].y + (st[i + 1].y - st[i].y) * (z - st[i].z) / (st[i + 1].z - st[i].z) + 0.02f;
            return st[0].y + 0.02f;
        };
        const float z0 = 1.30f, z1 = 1.78f;
        const glm::vec3 a(-0.20f, crown(z0) + 0.002f, z0), b(0.20f, crown(z0) + 0.002f, z0);
        const glm::vec3 c(0.20f, crown(z1) + 0.002f, z1), d(-0.20f, crown(z1) + 0.002f, z1);
        const glm::vec3 n = glm::normalize(glm::cross(c - b, a - b));
        style::face(h, a, b, c, d, n.y > 0 ? n : -n, L.liner);
        for (int k = 1; k < 6; ++k) {
            const float z = z0 + (z1 - z0) * float(k) / 6.0f;
            h.addBox({0.0f, crown(z) + 0.004f, z}, {0.185f, 0.0025f, 0.005f}, L.trim);
        }
        return;
    }
    for (float sx : {1.0f, -1.0f}) {
        h.addBox({sx * 0.43f, 0.6995f, 1.12f}, {0.06f, 0.002f, 0.075f}, L.liner); // vents
        for (int k = 0; k < 4; ++k) h.addBox({sx * 0.43f, 0.7015f, 1.06f + 0.04f * k}, {0.055f, 0.0025f, 0.006f}, L.trim);
    }
}

// Bumper swept along the front (zSign +1) or rear (-1) face and around the body corners, ending 4 cm along the
// sides. Profile: (d outward from the footprint skin, y above `y`).
void wrapBumper(MeshData& m, float zSign, float y, const std::vector<glm::vec2>& profile, const Material& mat) {
    std::vector<glm::vec3> path, normals;
    for (float sx : {-1.0f, 1.0f})
        for (int k = 0; k <= 6; ++k) {
            // Right corner from the side to the end face, left corner back to the side.
            const float a = 0.5f * kPi * float(sx < 0 ? k : 6 - k) / 6.0f; // 0 = side, pi/2 = end face
            const glm::vec3 nrm(sx * std::cos(a), 0.0f, zSign * std::sin(a));
            path.push_back(glm::vec3(sx * (kSkinX - kCornerR), y, zSign * (kFaceZ - kCornerR)) + nrm * kCornerR);
            normals.push_back(nrm);
        }
    for (float sx : {-1.0f, 1.0f}) {
        const std::size_t end = sx < 0 ? 0 : path.size() - 1;
        path.insert(sx < 0 ? path.begin() : path.end(), path[end] - glm::vec3(0, 0, zSign * 0.04f));
        normals.insert(sx < 0 ? normals.begin() : normals.end(), normals[end]);
    }
    style::sweepProfile(m, path, normals, profile, mat);
}

// Gasoline expedition SUV (Cars.png car #1): boxy five-door body, olive paint with orange accents, white roof,
// black flares and steel bumpers; winch, shackles, skid plate, rock sliders, spare wheel and jerry cans on the
// tailgate, low roof rails with a light bar and strapped cargo. No snorkel or ladder (the concept has none).
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
        wrapBumper(b, -1.0f, 0.0f,
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
    }
    v.spares.push_back({"spare_rear", "spare_carrier_rear", {0.0f, 0.62f, -kFaceZ - 0.026f}, {0, 0, -1}, true});

    // ---- Roof rails with crossbars, light bar, pods and strapped cargo. No roof spare: a 0.87 m tire lying flat
    // would cover two thirds of the rail spacing and stand 0.32 m tall; the spare rides on the tailgate.
    {
        constexpr float kRailX = 0.64f, kRail0 = -2.00f, kRail1 = 0.16f;
        MeshData& rails = P.add("roof_rails", "body_shell", {0, kRoofTop, -0.92f}, true, true);
        const float top = style::roofRails(rails, kRailX, kRail0, kRail1, kRoofTop, {0.04f, -0.85f, -1.70f}, L.trim,
                                           L.liner);
        const Material strap{MatId::Generic, {0.03f, 0.03f, 0.035f}, 0.7f, 0.0f, 0.8f};
        MeshData& lb = P.add("light_bar", "roof_rails", {0, top, 0.04f}, false, true);
        style::lightBar(lb, {0.0f, top + 0.07f, 0.09f}, 0.48f, L.trim, barLed);
        for (float sx : {1.0f, -1.0f})
            style::roundLamp(lb, {sx * 0.60f, top + 0.05f, 0.10f}, fwd, 0.045f, 0.05f, L.trim, barLed);
        auto strapOver = [&](MeshData& m, float x0, float x1, float y, float z) {
            style::cargoStrap(m, {x0 - 0.012f, top, z}, {x0 - 0.012f, y + 0.004f, z}, {x1 + 0.012f, y + 0.004f, z},
                              {x1 + 0.012f, top, z}, strap);
        };
        MeshData& cf = P.add("case_front", "roof_rails", {-0.28f, top, -0.85f}, false, true);
        style::cargoCase(cf, {-0.28f, top + 0.07f, -0.85f}, {0.24f, 0.07f, 0.20f}, L.accent, L.trim);
        strapOver(cf, -0.52f, -0.04f, top + 0.14f, -0.85f);
        MeshData& cr = P.add("case_rear", "roof_rails", {0.0f, top, -1.70f}, false, true);
        style::cargoCase(cr, {0.0f, top + 0.08f, -1.70f}, {0.34f, 0.08f, 0.22f}, olive, L.trim);
        strapOver(cr, -0.34f, 0.34f, top + 0.16f, -1.70f);
        // Jerry can lying flat along the vehicle, strapped to the middle crossbar.
        MeshData& jc = P.add("jerry_can_roof", "roof_rails", {0.30f, top, -0.85f}, false, true);
        const glm::mat4 saved = jc.xf;
        jc.xf = glm::rotate(glm::translate(glm::mat4(1.0f), {0.30f, top + 0.062f, -0.85f}), 0.5f * kPi, {1, 0, 0});
        style::jerryCan(jc, glm::vec3(0.0f), {0, 0, -1}, red, L.trim);
        jc.xf = saved;
        strapOver(jc, 0.205f, 0.395f, top + 0.124f, -0.85f);
    }

    P.finish(v);
    v.mudArch.x = W.axleZ[0];
    v.mudArch.y = W.axleZ[1];
    v.headlightLens = {0.67f, 0.40f, 2.24f};
}

// ================================================================ kit trailforge_ev
// Dual-motor electric SUV (Cars.png car #2) on the same wheel positions as trailforge_suv (track 1.80, wheelbase
// 2.80), so it shares the fitted lower body, glasshouse and hood. Own livery (white / light grey, blue accents,
// black cladding), closed front panel with a full-width LED bar, blue LED lamps, full-width tail light, low roof
// rails with a light bar; no spare (the concept has none). Chassis: frame rails, floor battery pack, one motor per
// axle.
void buildEv(VehicleVisual& v, const VehicleConfig& cfg) {
    const WheelFit W = fitWheels(cfg);
    const style::Livery L = style::arcticElectric();
    const Material chrome = style::metal(glm::vec3(0.92f), 0.05f, 0.0f);
    const Material head = style::led(MatId::Headlight, {0.90f, 0.95f, 1.0f});
    const Material blueLed = style::led(MatId::Headlight, {0.10f, 0.40f, 1.0f});
    const Material brake = style::led(MatId::BrakeLight, {1.0f, 0.04f, 0.02f});
    const Material reverse = style::led(MatId::ReverseLight, {1.0f, 1.0f, 1.0f});
    const Material amber = style::led(MatId::Amber, {1.0f, 0.42f, 0.04f});
    const Material gloss{MatId::Generic, glm::vec3(0.012f), 0.12f, 0.0f, 0.7f}; // piano-black front panel
    const Material pack{MatId::Generic, glm::vec3(0.05f), 0.55f, 0.6f, 1.0f};
    const glm::vec3 up(0, 1, 0), fwd(0, 0, 1);

    Parts P;
    MeshData& chassis = P.add("chassis", "", {0, 0, 0}, false, false);
    P.add("body_shell", "chassis", {0, 0.30f, 0}, true, false);
    for (float sx : {1.0f, -1.0f}) {
        P.add(sided("fender_front", sx), "body_shell", {sx * kSkinX, 0.3f, 1.45f}, true, true);
        P.add(sided("door_front", sx), "body_shell", {sx * kSkinX, 0.3f, kSeamFront}, true, true);
        P.add(sided("door_rear", sx), "body_shell", {sx * kSkinX, 0.3f, kSeamB}, true, true);
    }
    P.add("hood", "body_shell", {0, kDeck, kShieldZ}, true, true);
    MeshData& tailgate = P.add("tailgate", "body_shell", {-0.70f, 0.4f, -kFaceZ}, true, true);
    P.add("windshield", "body_shell", {0, 0.93f, 0.66f}, false, true);
    MeshData& panel = P.add("front_panel", "body_shell", {0, 0.41f, kFaceZ}, true, true);
    MeshData& shell = P["body_shell"];

    // ---- Chassis: frame rails, battery pack between the axles, motor + reduction housing on each axle
    for (float sx : {1.0f, -1.0f})
        chassis.addBeveledBox({sx * 0.40f, -0.14f, 0.0f}, {0.045f, 0.07f, 2.12f}, 0.01f, L.steel);
    for (float z : {-2.0f, 2.0f}) chassis.addBox({0.0f, -0.14f, z}, {0.40f, 0.04f, 0.04f}, L.steel);
    chassis.addBox({0.0f, 0.0f, 0.0f}, {0.50f, 0.27f, 2.10f}, L.underbody);
    MeshData& battery = P.add("battery_pack", "chassis", {0, -0.24f, 0}, true, false);
    battery.addBeveledBox({0.0f, -0.235f, 0.0f}, {0.50f, 0.055f, 0.88f}, 0.02f, pack);
    for (float z : {-0.6f, -0.2f, 0.2f, 0.6f}) battery.addBox({0.0f, -0.291f, z}, {0.46f, 0.002f, 0.012f}, L.steel);
    for (float zc : W.axleZ) {
        MeshData& motor = P.add(zc > 0 ? "motor_front" : "motor_rear", "chassis", {0, -0.22f, zc}, false, true);
        motor.addCylinder({-0.17f, -0.22f, zc + 0.02f}, {0.12f, -0.22f, zc + 0.02f}, 0.115f, 20, L.alu);
        motor.addCylinder({0.12f, -0.22f, zc + 0.02f}, {0.15f, -0.22f, zc + 0.02f}, 0.085f, 20, L.gunmetal);
        motor.addBeveledBox({-0.02f, -0.09f, zc + 0.02f}, {0.14f, 0.035f, 0.09f}, 0.012f, L.gunmetal); // inverter
        motor.addCylinder({-0.08f, -0.22f, zc - 0.05f}, {-0.08f, -0.22f, zc - 0.16f}, 0.07f, 16, L.alu); // gearset
    }
    P.add("skid_plate", "chassis", {0, -0.26f, 2.20f}, true, true)
        .addBar({0.0f, -0.24f, 2.24f}, {0.0f, -0.31f, 1.66f}, 0.40f, 0.008f, up, L.alu, 0.004f);

    // ---- Body shell, glasshouse, hood, flares
    lowerBody(P, L, W);
    for (float sx : {1.0f, -1.0f})
        for (float zc : W.axleZ) {
            const float top = W.arch[W.arch.size() / 2].y + 0.03f, x = std::min(0.50f, W.innerX) - 0.006f;
            shell.addBox({sx * x, 0.5f * (0.20f + top), zc}, {0.004f, 0.5f * (top - 0.20f), -W.arch.front().x},
                         L.liner);
        }
    glasshouse(P, L, true);
    HoodStyle hs;
    hs.accentZ0 = 1.62f;
    hs.accentZ1 = 2.13f;
    hs.ventPanel = true;
    hood(P, L, hs);
    shell.addBar({-0.80f, 0.708f, kShieldZ + 0.004f}, {0.80f, 0.708f, kShieldZ + 0.004f}, 0.018f, 0.012f, up, L.trim);
    const float stripeZ0 = W.axleZ[0] + W.arch.front().x - 0.03f, stripeZ1 = W.axleZ[1] + W.arch.back().x + 0.03f;
    for (float sx : {1.0f, -1.0f}) {
        for (float zc : W.axleZ) {
            const std::string name = std::string("flare_") + (zc > 0 ? "front_" : "rear_") + sideName(sx);
            style::fenderFlare(P.add(name, zc > 0 ? sided("fender_front", sx) : "body_shell", {sx * kSkinX, 0.10f, zc},
                                     true, true),
                               sx, archAt(W, zc), W.flareBand, kSkinX, kFlareProud, L.trim, L.liner);
        }
        P[sided("fender_front", sx)].addBox({sx * (kSkinX + 0.004f), 0.30f, 2.03f}, {0.004f, 0.02f, 0.04f}, amber);
        shell.addBox({sx * (kSkinX + 0.004f), 0.30f, -2.04f}, {0.004f, 0.02f, 0.03f}, amber);
        // Flush door handles.
        for (float z : {0.28f, -0.64f})
            P[z > kSeamB ? sided("door_front", sx) : sided("door_rear", sx)].addBeveledBox(
                {sx * 0.884f, 0.52f, z}, {0.006f, 0.012f, 0.075f}, 0.004f, L.trim);
        // Blue stripe low on the doors, just above the cladding, from arch to arch (decal).
        for (const auto& [z0, z1, part] : {std::tuple{std::min(stripeZ0, kSeamFront - 0.02f), kSeamB + 0.01f, "door_front"},
                                           std::tuple{kSeamB - 0.01f, stripeZ1, "door_rear"}})
            style::panelGap(P[sided(part, sx)], {sx * 0.8847f, 0.035f, z0}, {sx * 0.8847f, 0.035f, z1}, {sx, 0, 0},
                            0.022f, L.accent);
        style::rockSlider(P.add(sided("rock_slider", sx), "chassis", {sx * 0.80f, kSill, 0}, true, true), sx, 0.95f,
                          kSill - 0.02f, -0.70f, 0.60f, L.trim, L.accent, L.steel);
        style::mirror(P.add(sided("mirror", sx), sided("door_front", sx), {sx * 0.82f, 0.70f, 0.74f}, false, true), sx,
                      {sx * 0.82f, 0.70f, 0.74f}, L.trim, chrome);
    }

    // ---- Front: closed panel, full-width LED bar, headlights, wrapped bumper with blue lamps and tow points
    panel.addBeveledBox({0.0f, 0.41f, kFaceZ + 0.006f}, {0.46f, 0.13f, 0.012f}, 0.006f, gloss);
    for (int k = 0; k < 3; ++k)
        panel.addBox({0.0f, 0.34f + 0.05f * k, kFaceZ + 0.0185f}, {0.40f, 0.0035f, 0.0015f}, L.trim);
    panel.addBeveledBox({0.0f, 0.47f, kFaceZ + 0.020f}, {0.045f, 0.012f, 0.003f}, 0.002f, L.alu); // badge
    panel.addBox({0.0f, 0.543f, kFaceZ + 0.014f}, {0.76f, 0.0055f, 0.004f}, head);             // light bar
    for (float sx : {1.0f, -1.0f}) {
        MeshData& hl = P.add(sided("headlight", sx), "body_shell", {sx * 0.64f, 0.45f, kFaceZ}, false, true);
        const glm::vec3 c(sx * 0.64f, 0.45f, kFaceZ + 0.008f);
        hl.addBeveledBox(c, {0.15f, 0.07f, 0.014f}, 0.007f, L.trim);
        style::ledRing(hl, c + fwd * 0.014f, fwd, {1, 0, 0}, 0.075f, 0.050f, 0.022f, 0.010f, head);
        hl.addBox(c + glm::vec3(0.0f, 0.0f, 0.016f), {0.032f, 0.016f, 0.002f}, head);
        hl.addBox(c + glm::vec3(-sx * 0.115f, 0.0f, 0.016f), {0.022f, 0.045f, 0.002f}, L.gunmetal);
    }
    {
        MeshData& b = P.add("bumper_front", "chassis", {0, 0.07f, kFaceZ}, true, true);
        wrapBumper(b, 1.0f, 0.07f,
                   {{-0.01f, 0.15f}, {0.055f, 0.15f}, {0.085f, 0.125f}, {0.10f, 0.05f}, {0.11f, 0.0f},
                    {0.11f, -0.07f}, {0.09f, -0.11f}, {0.04f, -0.14f}, {-0.01f, -0.14f}},
                   L.trim);
        b.addBox({0.0f, 0.05f, kFaceZ + 0.111f}, {0.36f, 0.035f, 0.002f}, L.liner); // lower intake (closed)
        for (float sx : {1.0f, -1.0f}) {
            b.addBox({sx * 0.60f, 0.10f, kFaceZ + 0.111f}, {0.085f, 0.022f, 0.002f}, L.liner);
            b.addBox({sx * 0.60f, 0.10f, kFaceZ + 0.1125f}, {0.075f, 0.014f, 0.001f}, blueLed);
            b.addBox({sx * 0.40f, -0.02f, kFaceZ + 0.112f}, {0.035f, 0.025f, 0.002f}, L.accent); // recovery point
            style::shackle(P.add(sided("tow_hook_front", sx), "bumper_front", {sx * 0.40f, -0.04f, 2.32f}, false, true),
                           {sx * 0.40f, -0.04f, 2.32f}, fwd, L.hooks);
        }
    }

    // ---- Rear: full-width tail light, wrapped bumper with blue tow hooks, spare on a tailgate carrier
    tailgate.addBeveledBox({0.0f, 0.24f, -kFaceZ - 0.006f}, {0.16f, 0.012f, 0.006f}, 0.003f, L.trim); // handle
    tailgate.addBox({0.0f, 0.505f, -kFaceZ - 0.008f}, {0.56f, 0.004f, 0.003f}, brake);              // light line
    for (float sx : {1.0f, -1.0f}) {
        MeshData& tl = P.add(sided("taillight", sx), "body_shell", {sx * 0.72f, 0.50f, -kFaceZ}, false, true);
        tl.addBeveledBox({sx * 0.71f, 0.50f, -kFaceZ - 0.006f}, {0.15f, 0.035f, 0.012f}, 0.006f, L.trim);
        tl.addBox({sx * 0.71f, 0.51f, -kFaceZ - 0.019f}, {0.135f, 0.012f, 0.002f}, brake);
        tl.addBox({sx * 0.76f, 0.484f, -kFaceZ - 0.019f}, {0.06f, 0.006f, 0.002f}, reverse);
    }
    {
        MeshData& b = P.add("bumper_rear", "chassis", {0, 0.02f, -kFaceZ}, true, true);
        wrapBumper(b, -1.0f, 0.0f,
                   {{-0.01f, 0.15f}, {0.075f, 0.15f}, {0.105f, 0.12f}, {0.105f, 0.045f}, {0.096f, 0.035f},
                    {0.096f, 0.020f}, {0.105f, 0.010f}, {0.105f, -0.065f}, {0.085f, -0.11f}, {0.04f, -0.15f},
                    {-0.01f, -0.15f}},
                   L.trim);
        b.addBox({0.0f, -0.08f, -2.36f}, {0.04f, 0.04f, 0.05f}, L.steel); // hitch receiver
        b.addBox({0.0f, -0.08f, -2.411f}, {0.028f, 0.028f, 0.002f}, L.liner);
        for (float sx : {1.0f, -1.0f}) {
            b.addBox({sx * 0.62f, 0.075f, -kFaceZ - 0.1055f}, {0.070f, 0.026f, 0.002f}, L.accent); // blue blocks
            b.addBox({sx * 0.62f, 0.075f, -kFaceZ - 0.1075f}, {0.050f, 0.005f, 0.001f}, brake);    // reflector
            style::shackle(P.add(sided("tow_hook_rear", sx), "bumper_rear", {sx * 0.40f, -0.03f, -2.31f}, false, true),
                           {sx * 0.40f, -0.03f, -2.31f}, -fwd, L.hooks);
        }
    }

    // ---- Low roof rails with crossbars, front light bar with blue end pods, blue LED strips on the rails, one
    // flat case. No antenna, no snorkel (no engine air intake).
    {
        constexpr float kRailX = 0.64f, kRail0 = -2.00f, kRail1 = 0.16f;
        MeshData& rails = P.add("roof_rails", "body_shell", {0, kRoofTop, -0.92f}, true, true);
        const float top = style::roofRails(rails, kRailX, kRail0, kRail1, kRoofTop, {0.04f, -0.85f, -1.70f}, L.trim,
                                           L.liner);
        for (float sx : {1.0f, -1.0f})
            rails.addBox({sx * (kRailX + 0.0095f), kRoofTop + 0.050f, -0.92f}, {0.0015f, 0.004f, 0.80f}, blueLed);
        MeshData& lb = P.add("light_bar", "roof_rails", {0, top, 0.04f}, false, true);
        style::lightBar(lb, {0.0f, top + 0.06f, 0.09f}, 0.46f, L.trim, head);
        for (float sx : {1.0f, -1.0f}) {
            lb.addBeveledBox({sx * 0.57f, top + 0.05f, 0.08f}, {0.05f, 0.03f, 0.03f}, 0.01f, L.trim);
            lb.addBox({sx * 0.57f, top + 0.05f, 0.111f}, {0.040f, 0.020f, 0.002f}, blueLed);
        }
        MeshData& cs = P.add("case_roof", "roof_rails", {0.0f, top, -1.27f}, false, true);
        style::cargoCase(cs, {0.0f, top + 0.06f, -1.27f}, {0.42f, 0.06f, 0.36f}, L.trim, L.accent);
    }

    P.finish(v);
    v.mudArch.x = W.axleZ[0];
    v.mudArch.y = W.axleZ[1];
    v.headlightLens = {0.64f, 0.45f, 2.23f};
}

// ================================================================ kit trailforge_patrol
// Tube-frame patrol buggy. Its proportions (wheelbase, track, tires, cage and roof line, seat, hood and bed heights,
// coil-over towers) were measured from a purchased reference model; every triangle is built here. Sand
// powder-coated cage and frame, satin black panels, orange coils; mid engine under the cargo bed. Collision:
// kPatrolBoxes (vehicle/VehicleShape.h). Laid out for the shipped wheels (track 1.35, wheelbase 2.068, r 0.33);
// fitToWheels pushes whatever a bigger tire would sweep through out of its way.
namespace patrol {

constexpr float kGround = -0.613f; // ground below the body origin at rest (vehicle.json trailforge_patrol)
constexpr float kHubRest = kGround + 0.33f;
constexpr float kFitGap = 0.03f;   // body clearance to the wheel sweeps after the fit
float H(float h) { return kGround + h; } // height above the ground -> body y

// Moves p out of the wheel sweeps: up above the hub centre, toward the centre line below it.
glm::vec3 pushOut(const std::array<WheelSweep, 4>& S, glm::vec3 p, float gap) {
    const glm::vec3 dir = p.y > kHubRest ? glm::vec3(0, 1, 0) : glm::vec3(p.x >= 0.0f ? -1.0f : 1.0f, 0, 0);
    float moved = 0.0f;
    for (float c = wheelSweepClearance(S, p.x, p.y, p.z); c < gap && moved < 0.6f; c = wheelSweepClearance(S, p.x, p.y, p.z)) {
        const float step = std::max(0.004f, gap - c);
        p += dir * step;
        moved += step;
    }
    return p;
}

// Shock tower position clear of the sweeps: moved inboard, so the coil-over keeps its length on larger tires.
glm::vec3 pushInboard(const std::array<WheelSweep, 4>& S, glm::vec3 p, float gap) {
    const float dx = p.x >= 0.0f ? -0.004f : 0.004f;
    for (int k = 0; k < 60 && wheelSweepClearance(S, p.x, p.y, p.z) < gap; ++k) p.x += dx;
    return p;
}

// Splits the triangles that come closer than `gap` to a wheel sweep (vertices, edge midpoints, centroid) down to
// 3 cm and pushes their vertices out; the moved triangles get flat normals.
bool fitPass(MeshData& m, const std::array<WheelSweep, 4>& S, float gap) {
    auto bad = [&](const glm::vec3& p) { return wheelSweepClearance(S, p.x, p.y, p.z) < gap; };
    auto nearSweep = [&](const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
        const glm::vec3 lo = glm::min(a, glm::min(b, c)) - gap, hi = glm::max(a, glm::max(b, c)) + gap;
        for (const WheelSweep& w : S) {
            const float reach = std::sqrt(w.radius * w.radius + w.halfWidth * w.halfWidth);
            if (hi.x > w.x - reach && lo.x < w.x + reach && hi.z > w.z - reach && lo.z < w.z + reach &&
                hi.y > w.hubLow - w.radius && lo.y < w.hubHigh + w.radius)
                return true;
        }
        return false;
    };
    auto mid = [](const LitVertex& a, const LitVertex& b) {
        LitVertex r = a;
        r.position = 0.5f * (a.position + b.position);
        const glm::vec3 n = a.normal + b.normal;
        r.normal = glm::length(n) > 1e-6f ? glm::normalize(n) : a.normal;
        r.color = 0.5f * (a.color + b.color);
        return r;
    };
    std::vector<LitVertex> out;
    out.reserve(m.vertices.size());
    bool changed = false;
    std::function<void(const LitVertex&, const LitVertex&, const LitVertex&, int)> emit =
        [&](const LitVertex& a, const LitVertex& b, const LitVertex& c, int depth) {
            const glm::vec3 &pa = a.position, &pb = b.position, &pc = c.position;
            const bool hit = nearSweep(pa, pb, pc) && (bad(pa) || bad(pb) || bad(pc) || bad(0.5f * (pa + pb)) || bad(0.5f * (pb + pc)) ||
                             bad(0.5f * (pc + pa)) || bad((pa + pb + pc) / 3.0f));
            const float longest = std::max({glm::length(pb - pa), glm::length(pc - pb), glm::length(pa - pc)});
            changed = changed || hit;
            if (!hit || depth >= 7 || longest < 0.03f) {
                out.push_back(a);
                out.push_back(b);
                out.push_back(c);
                return;
            }
            const LitVertex ab = mid(a, b), bc = mid(b, c), ca = mid(c, a);
            emit(a, ab, ca, depth + 1);
            emit(ab, b, bc, depth + 1);
            emit(ca, bc, c, depth + 1);
            emit(ab, bc, ca, depth + 1);
        };
    for (std::size_t i = 0; i + 2 < m.vertices.size(); i += 3) emit(m.vertices[i], m.vertices[i + 1], m.vertices[i + 2], 0);
    if (!changed) return false;
    for (std::size_t i = 0; i + 2 < out.size(); i += 3) {
        bool moved = false;
        for (std::size_t k = 0; k < 3; ++k) {
            glm::vec3& p = out[i + k].position;
            if (!bad(p)) continue;
            p = pushOut(S, p, kFitGap + 0.005f);
            moved = true;
        }
        if (!moved) continue;
        const glm::vec3 n = glm::cross(out[i + 1].position - out[i].position, out[i + 2].position - out[i].position);
        if (glm::length(n) < 1e-9f) continue;
        for (std::size_t k = 0; k < 3; ++k) out[i + k].normal = glm::normalize(n);
    }
    m.vertices = std::move(out);
    return true;
}

// Body geometry is kept out of the wheel sweeps (larger tires than the shipped ones). Vertices move independently,
// so a pushed triangle can still cross a sweep (frame tubes the tire would need moved): those are dropped.
void fitToWheels(MeshData& m, const std::array<WheelSweep, 4>& S) {
    fitPass(m, S, kFitGap);
    auto inside = [&](const glm::vec3& p) { return wheelSweepClearance(S, p.x, p.y, p.z) < kBodyClearance; };
    std::vector<LitVertex> kept;
    kept.reserve(m.vertices.size());
    for (std::size_t i = 0; i + 2 < m.vertices.size(); i += 3) {
        const glm::vec3 &a = m.vertices[i].position, &b = m.vertices[i + 1].position, &c = m.vertices[i + 2].position;
        if (inside(a) || inside(b) || inside(c) || inside(0.5f * (a + b)) || inside(0.5f * (b + c)) ||
            inside(0.5f * (c + a)) || inside((a + b + c) / 3.0f))
            continue;
        kept.insert(kept.end(), m.vertices.begin() + std::ptrdiff_t(i), m.vertices.begin() + std::ptrdiff_t(i + 3));
    }
    m.vertices = std::move(kept);
}

glm::vec3 side(const glm::vec3& p, float sx) { return {sx * p.x, p.y, p.z}; }

// Steering wheel: rim of `segs` tube pieces around `axis` (pointing at the driver), three spokes, hub.
void steeringWheel(MeshData& m, const glm::vec3& c, const glm::vec3& axis, float r, const Material& rim,
                   const Material& hub) {
    const glm::vec3 a = glm::normalize(axis);
    const glm::vec3 u = glm::normalize(glm::cross(a, glm::vec3(1, 0, 0)));
    const glm::vec3 w = glm::cross(a, u);
    constexpr int segs = 18;
    for (int k = 0; k < segs; ++k) {
        const float t0 = 2.0f * kPi * float(k) / segs, t1 = 2.0f * kPi * float(k + 1) / segs;
        m.addCylinder(c + r * (std::cos(t0) * u + std::sin(t0) * w), c + r * (std::cos(t1) * u + std::sin(t1) * w),
                      0.016f, 8, rim);
    }
    for (float t : {0.5f * kPi, 0.5f * kPi + 2.0944f, 0.5f * kPi + 4.1888f})
        m.addCylinder(c, c + 0.92f * r * (std::cos(t) * u + std::sin(t) * w), 0.011f, 6, hub, false);
    m.addCylinder(c - 0.02f * a, c + 0.03f * a, 0.045f, 14, hub);
}

void buildPatrol(VehicleVisual& v, const VehicleConfig& cfg) {
    const style::Livery L = style::desertPatrol();
    const WheelFit W = fitWheels(cfg);
    const auto S = wheelSweeps(cfg);
    const float zF = W.axleZ[0], zR = W.axleZ[1];
    const Material& tubeMat = L.roof;
    const Material head = style::led(MatId::Headlight, {1.0f, 0.95f, 0.88f});
    const Material barLed = style::led(MatId::Headlight, {0.85f, 0.92f, 1.0f});
    const Material brake = style::led(MatId::BrakeLight, {1.0f, 0.04f, 0.02f});
    const Material reverse = style::led(MatId::ReverseLight, {1.0f, 1.0f, 1.0f});
    const Material amber = style::led(MatId::Amber, {1.0f, 0.42f, 0.04f});
    const Material seat = style::plastic(style::srgb(0.09f, 0.09f, 0.10f), 0.80f, 0.5f);
    const Material canvas = style::plastic(style::srgb(0.60f, 0.52f, 0.38f), 0.88f, 0.9f);
    const Material iron{MatId::Generic, {0.06f, 0.06f, 0.06f}, 0.50f, 0.5f, 1.0f};
    const glm::vec3 up(0, 1, 0), fwd(0, 0, 1);
    constexpr float rMain = 0.024f, rBrace = 0.018f;

    Parts P;
    MeshData& chassis = P.add("chassis", "", {0, 0, 0}, false, false);
    MeshData& cage = P.add("cage", "chassis", {0, H(0.92f), -0.10f}, true, false);
    MeshData& front = P.add("front_frame", "chassis", {0, H(0.60f), 1.0f}, true, false);
    MeshData& rear = P.add("rear_frame", "chassis", {0, H(0.60f), -1.0f}, true, false);
    MeshData& hood = P.add("hood", "front_frame", {0, H(0.96f), 0.60f}, true, true); // hinge at the cowl
    auto tube = [&](MeshData& m, const glm::vec3& a, const glm::vec3& b, float r) { m.addCylinder(a, b, r, 12, tubeMat); };

    // ---- Cage: B hoop, roof, A pillars, sills, door bars, cross tubes
    const glm::vec3 S0(0.60f, H(0.38f), -0.48f), S1(0.60f, H(0.38f), 0.62f);
    const glm::vec3 B1(0.64f, H(0.92f), -0.47f), B2(0.50f, H(1.45f), -0.47f);
    const glm::vec3 A2(0.50f, H(1.40f), 0.27f), A1(0.60f, H(0.93f), 0.60f), D1(0.60f, H(0.57f), 0.60f);
    const glm::vec3 harness = glm::mix(B1, B2, (1.05f - 0.92f) / (1.45f - 0.92f));
    for (float sx : {1.0f, -1.0f}) {
        tube(cage, side(S0, sx), side(B1, sx), rMain);
        tube(cage, side(B1, sx), side(B2, sx), rMain);
        tube(cage, side(B2, sx), side(A2, sx), rMain);
        tube(cage, side(A2, sx), side(A1, sx), rMain);
        tube(cage, side(A1, sx), side(S1, sx), rMain);
        tube(cage, side(S0, sx), side(S1, sx), 0.026f);
        tube(cage, side(B1, sx), side(D1, sx), rBrace);
    }
    for (const glm::vec3& n : {S0, S1, B2, A2, A1, harness}) tube(cage, side(n, 1.0f), side(n, -1.0f), n == S0 || n == S1 ? 0.022f : rMain);
    tube(cage, glm::mix(B2, A2, 0.5f), side(glm::mix(B2, A2, 0.5f), -1.0f), rBrace);
    for (float sx : {1.0f, -1.0f}) tube(cage, side(B2, sx), side(glm::mix(B2, A2, 0.5f), -sx), rBrace); // roof X

    // ---- Front frame: upper and lower rails, nose hoop, shock towers
    const glm::vec3 FU(0.36f, H(0.80f), 1.32f), FL0(0.24f, H(0.30f), 0.80f), FL1(0.24f, H(0.30f), 1.40f);
    const glm::vec3 TF = pushInboard(S, {0.38f, H(0.795f), zF + 0.016f}, 0.09f);
    for (float sx : {1.0f, -1.0f}) {
        tube(front, side(A1, sx), side(FU, sx), rMain);
        tube(front, side(S1, sx), side(FL0, sx), rMain);
        tube(front, side(FL0, sx), side(FL1, sx), rMain);
        tube(front, side(FL1, sx), side(FU, sx), rMain);
        const float t = (TF.z - A1.z) / (FU.z - A1.z);
        tube(front, side({FL0.x, FL0.y, TF.z}, sx), side(TF, sx), rBrace); // tower post on the lower rail
        tube(front, side(TF, sx), side(glm::mix(A1, FU, t), sx), rBrace);    // brace to the upper rail
        front.addBeveledBox(side(TF + glm::vec3(0.0f, 0.016f, 0.0f), sx), {0.055f, 0.010f, 0.05f}, 0.003f, L.steel);
    }
    for (const glm::vec3& n : {FU, FL1, TF}) tube(front, side(n, 1.0f), side(n, -1.0f), n == TF ? rBrace : rMain);
    front.addBox({0.0f, H(0.62f), 1.30f}, {0.22f, 0.15f, 0.025f}, L.gunmetal); // radiator

    // ---- Rear frame: bed rails, braces to the roof, lower rails, uprights, shock towers
    const glm::vec3 BR1(0.64f, H(0.92f), -1.40f), RD(0.62f, H(0.92f), -1.20f);
    const glm::vec3 RL0(0.24f, H(0.30f), -0.70f), RL1(0.24f, H(0.30f), -1.42f), RLow(0.50f, H(0.45f), -1.44f);
    const glm::vec3 TR = pushInboard(S, {0.383f, H(0.792f), zR + 0.014f}, 0.09f);
    for (float sx : {1.0f, -1.0f}) {
        tube(rear, side(B1, sx), side(BR1, sx), rMain);
        tube(rear, side(B2, sx), side(RD, sx), rMain);
        tube(rear, side(S0, sx), side(RL0, sx), rMain);
        tube(rear, side(RL0, sx), side(RL1, sx), rMain);
        tube(rear, side(BR1, sx), side(RLow, sx), rMain);
        tube(rear, side(RLow, sx), side(RL1, sx), rBrace);
        tube(rear, side({RL0.x, RL0.y, TR.z}, sx), side(TR, sx), rBrace);      // tower post on the lower rail
        tube(rear, side(TR, sx), side({TR.x, H(0.90f), TR.z}, sx), rBrace);      // up to the bed floor
        rear.addBeveledBox(side(TR + glm::vec3(0.0f, 0.016f, 0.0f), sx), {0.055f, 0.010f, 0.05f}, 0.003f, L.steel);
    }
    for (const glm::vec3& n : {BR1, RL1, TR}) tube(rear, side(n, 1.0f), side(n, -1.0f), n == TR ? rBrace : rMain);

    // ---- Chassis: floor pan, tunnel, sill skirts, engine, transaxle, exhaust
    chassis.addBox({0.0f, H(0.345f), 0.07f}, {0.58f, 0.012f, 0.55f}, L.underbody);
    chassis.addBox({0.0f, H(0.42f), 0.07f}, {0.10f, 0.065f, 0.52f}, L.trim);
    for (float sx : {1.0f, -1.0f})
        chassis.addBox({sx * 0.605f, H(0.43f), 0.07f}, {0.008f, 0.05f, 0.53f}, L.body);
    {
        MeshData& e = P.add("engine", "chassis", {0, H(0.55f), -0.86f}, false, false);
        e.addBeveledBox({0.0f, H(0.55f), -0.86f}, {0.24f, 0.15f, 0.22f}, 0.02f, L.gunmetal);
        e.addBeveledBox({0.0f, H(0.73f), -0.86f}, {0.20f, 0.03f, 0.16f}, 0.01f, L.accent); // cam cover
        e.addBeveledBox({0.0f, H(0.43f), -1.12f}, {0.16f, 0.08f, 0.10f}, 0.01f, iron);     // sequential transaxle
        for (int k = 0; k < 4; ++k) {
            const float z = -0.98f + 0.08f * float(k);
            e.addCylinder({-0.24f, H(0.62f), z}, {-0.30f, H(0.50f), -1.10f}, 0.016f, 8, L.alu);
        }
        e.addCylinder({-0.30f, H(0.50f), -1.10f}, {-0.30f, H(0.58f), -1.28f}, 0.024f, 10, L.alu);
        e.addCylinder({-0.30f, H(0.60f), -1.26f}, {-0.30f, H(0.60f), -1.46f}, 0.06f, 16, L.alu); // silencer
    }

    // ---- Hood with nose face, grille and head lamps (no fenders: wheels and suspension stay exposed)
    {
        const float zs[] = {0.60f, 0.80f, 1.05f, 1.30f, 1.40f};
        const float hw[] = {0.52f, 0.50f, 0.45f, 0.37f, 0.33f};
        const float ys[] = {H(0.96f), H(0.955f), H(0.935f), H(0.875f), H(0.835f)};
        const float cross[] = {-1.0f, -0.4f, 0.4f, 1.0f};
        const float crown[] = {0.0f, 0.022f, 0.022f, 0.0f};
        for (int i = 0; i + 1 < 5; ++i) {
            for (int k = 0; k + 1 < 4; ++k) {
                const glm::vec3 a(hw[i] * cross[k], ys[i] + crown[k], zs[i]), b(hw[i] * cross[k + 1], ys[i] + crown[k + 1], zs[i]);
                const glm::vec3 c(hw[i + 1] * cross[k + 1], ys[i + 1] + crown[k + 1], zs[i + 1]),
                    d(hw[i + 1] * cross[k], ys[i + 1] + crown[k], zs[i + 1]);
                glm::vec3 n = glm::normalize(glm::cross(b - a, d - a));
                if (n.y < 0.0f) n = -n;
                style::face(hood, a, b, c, d, n, L.body);
            }
            for (float sx : {1.0f, -1.0f}) { // side skirts
                const glm::vec3 a(sx * hw[i], ys[i], zs[i]), b(sx * hw[i + 1], ys[i + 1], zs[i + 1]);
                style::face(hood, a, b, b - glm::vec3(0, 0.06f, 0), a - glm::vec3(0, 0.06f, 0), {sx, 0, 0}, L.body);
            }
        }
        // Hood scoop and nose face.
        hood.addBeveledBox({0.0f, H(0.975f), 0.90f}, {0.13f, 0.018f, 0.16f}, 0.008f, L.trim);
        const glm::vec3 n0(-0.33f, H(0.835f), 1.40f), n1(0.33f, H(0.835f), 1.40f), n2(0.30f, H(0.62f), 1.43f),
            n3(-0.30f, H(0.62f), 1.43f);
        style::face(hood, n0, n1, n2, n3, glm::normalize(glm::vec3(0.0f, 0.14f, 1.0f)), L.body);
        hood.addBox({0.0f, H(0.70f), 1.425f}, {0.12f, 0.06f, 0.008f}, L.liner); // grille opening
        for (int k = 0; k < 3; ++k)
            hood.addBox({0.0f, H(0.665f) + 0.035f * float(k), 1.434f}, {0.115f, 0.006f, 0.004f}, L.gunmetal);
        for (float sx : {1.0f, -1.0f}) {
            MeshData& hl = P.add(sided("headlight", sx), "hood", {sx * 0.235f, H(0.74f), 1.42f}, false, true);
            hl.addBeveledBox({sx * 0.235f, H(0.74f), 1.418f}, {0.085f, 0.065f, 0.012f}, 0.006f, L.trim);
            style::roundLamp(hl, {sx * 0.235f, H(0.74f), 1.445f}, fwd, 0.052f, 0.02f, L.gunmetal, head);
            hood.addBox({sx * 0.33f, H(0.80f), 1.402f}, {0.012f, 0.012f, 0.003f}, amber); // marker
        }
    }

    // ---- Front guard, skid plate, tow hooks
    {
        MeshData& g = P.add("bumper_front", "front_frame", {0, H(0.55f), 1.47f}, true, true);
        auto gt = [&](const glm::vec3& a, const glm::vec3& b) { g.addCylinder(a, b, 0.022f, 12, L.steel); };
        for (float sx : {1.0f, -1.0f}) {
            gt(side({0.30f, H(0.36f), 1.49f}, sx), side({0.30f, H(0.80f), 1.49f}, sx));
            gt(side({0.30f, H(0.80f), 1.49f}, sx), side({0.46f, H(0.84f), 1.42f}, sx));
            gt(side({0.46f, H(0.84f), 1.42f}, sx), side({0.48f, H(0.86f), 1.28f}, sx));
            gt(side({0.30f, H(0.36f), 1.49f}, sx), side(FL1, sx));
            gt(side({0.30f, H(0.58f), 1.49f}, sx), side({0.24f, H(0.58f), 1.41f}, sx));
            g.addBox(side({0.40f, H(0.86f), 1.40f}, sx), {0.02f, 0.012f, 0.012f}, amber);
            style::shackle(P.add(sided("tow_hook_front", sx), "bumper_front", {sx * 0.22f, H(0.33f), 1.50f}, false, true),
                           {sx * 0.22f, H(0.33f), 1.50f}, fwd, L.hooks);
        }
        gt({-0.30f, H(0.36f), 1.49f}, {0.30f, H(0.36f), 1.49f});
        gt({-0.30f, H(0.80f), 1.49f}, {0.30f, H(0.80f), 1.49f});
        gt({-0.30f, H(0.58f), 1.49f}, {0.30f, H(0.58f), 1.49f});
        P.add("skid_plate", "front_frame", {0, H(0.30f), 1.40f}, true, true)
            .addBar({0.0f, H(0.31f), 1.48f}, {0.0f, H(0.28f), 0.80f}, 0.27f, 0.006f, up, L.alu, 0.003f);
    }

    // ---- Cockpit: floor-mounted bucket seats with harnesses, dash, steering wheel, mirrors
    for (float sx : {1.0f, -1.0f}) {
        MeshData& s = P.add(sided("seat", sx), "chassis", {sx * 0.32f, H(0.50f), -0.10f}, false, true);
        s.addBeveledBox({sx * 0.32f, H(0.55f), -0.10f}, {0.21f, 0.05f, 0.20f}, 0.025f, seat);
        s.addBar({sx * 0.32f, H(0.58f), -0.33f}, {sx * 0.32f, H(1.24f), -0.38f}, 0.21f, 0.045f, fwd, seat, 0.02f);
        for (float bx : {0.19f, -0.19f}) // side bolsters
            s.addBar({sx * 0.32f + bx, H(0.62f), -0.29f}, {sx * 0.32f + bx, H(1.05f), -0.33f}, 0.025f, 0.06f, fwd, seat, 0.01f);
        for (float bx : {0.07f, -0.07f}) // harness straps
            s.addBar({sx * 0.32f + bx, H(0.62f), -0.282f}, {sx * 0.32f + bx, H(1.16f), -0.326f}, 0.018f, 0.003f, fwd,
                     L.accent);
        s.addBeveledBox({sx * 0.32f, H(0.40f), -0.10f}, {0.16f, 0.05f, 0.16f}, 0.01f, L.steel); // seat base
        style::mirror(P.add(sided("mirror", sx), "cage", glm::mix(A1, A2, 0.36f) * glm::vec3(sx, 1, 1), false, true), sx,
                      glm::mix(A1, A2, 0.36f) * glm::vec3(sx, 1, 1) + glm::vec3(sx * 0.03f, 0, 0), L.trim, L.alu);
    }
    {
        MeshData& d = P.add("dash", "cage", {0, H(0.86f), 0.56f}, false, false);
        d.addBeveledBox({0.0f, H(0.86f), 0.55f}, {0.55f, 0.06f, 0.06f}, 0.015f, L.trim);
        d.addBeveledBox({0.32f, H(0.905f), 0.50f}, {0.09f, 0.035f, 0.012f}, 0.006f, L.liner); // instrument pod
        MeshData& w = P.add("steering_wheel", "dash", {0.32f, H(0.95f), 0.30f}, false, true);
        w.addCylinder({0.32f, H(0.95f), 0.30f}, {0.32f, H(0.84f), 0.52f}, 0.02f, 10, L.gunmetal);
        steeringWheel(w, {0.32f, H(0.96f), 0.29f}, {0.0f, 0.45f, -1.0f}, 0.165f, L.trim, L.gunmetal);
    }

    // ---- Cargo bed with strapped cases and the spare lying flat
    {
        MeshData& bed = P.add("cargo_bed", "rear_frame", {0, H(0.905f), -0.95f}, true, true);
        bed.addBox({0.0f, H(0.905f), -0.95f}, {0.62f, 0.008f, 0.46f}, L.steel);
        for (float z : {-0.60f, -0.95f, -1.30f}) bed.addBox({0.0f, H(0.915f), z}, {0.60f, 0.003f, 0.012f}, L.gunmetal);
        for (float sx : {1.0f, -1.0f}) {
            MeshData& c = P.add(sided("case", sx), "cargo_bed", {sx * 0.47f, H(0.92f), -0.95f}, false, true);
            style::cargoCase(c, {sx * 0.47f, H(0.913f) + 0.11f, -0.95f}, {0.13f, 0.11f, 0.36f}, canvas, L.trim);
        }
        style::spareCarrier(P.add("spare_carrier", "cargo_bed", {0, H(0.913f), -0.98f}, false, true),
                            {0.0f, H(0.913f), -0.98f}, up, 0.22f, L.steel);
        v.spares.push_back({"spare_bed", "spare_carrier", {0.0f, H(0.913f) + 0.004f, -0.98f}, up, true});
    }

    // ---- Rear: bumper, rack bars, tail lamps, tow hooks
    {
        MeshData& b = P.add("bumper_rear", "rear_frame", {0, H(0.45f), -1.50f}, true, true);
        auto bt = [&](const glm::vec3& a, const glm::vec3& c) { b.addCylinder(a, c, 0.024f, 12, L.steel); };
        bt({-0.55f, H(0.45f), -1.50f}, {0.55f, H(0.45f), -1.50f});
        for (float sx : {1.0f, -1.0f}) {
            bt(side({0.55f, H(0.45f), -1.50f}, sx), side(RLow, sx));
            bt(side({0.30f, H(0.45f), -1.50f}, sx), side({0.30f, H(0.91f), -1.46f}, sx)); // rack bars
            MeshData& tl = P.add(sided("taillight", sx), "bumper_rear", {sx * 0.44f, H(0.62f), -1.50f}, false, true);
            tl.addBeveledBox({sx * 0.44f, H(0.62f), -1.505f}, {0.07f, 0.045f, 0.016f}, 0.006f, L.trim);
            tl.addBox({sx * 0.44f, H(0.635f), -1.522f}, {0.055f, 0.022f, 0.002f}, brake);
            tl.addBox({sx * 0.44f, H(0.598f), -1.522f}, {0.055f, 0.008f, 0.002f}, reverse);
            tl.addBox({sx * 0.513f, H(0.62f), -1.50f}, {0.003f, 0.02f, 0.012f}, amber);
            style::shackle(P.add(sided("tow_hook_rear", sx), "bumper_rear", {sx * 0.22f, H(0.40f), -1.52f}, false, true),
                           {sx * 0.22f, H(0.40f), -1.52f}, -fwd, L.hooks);
        }
        bt({-0.30f, H(0.91f), -1.46f}, {0.30f, H(0.91f), -1.46f});
    }

    // ---- LED light bar on the front roof tube
    {
        MeshData& lb = P.add("light_bar", "cage", {0, A2.y, A2.z}, false, true);
        style::lightBar(lb, {0.0f, A2.y + 0.062f, A2.z + 0.03f}, 0.42f, L.trim, barLed);
        for (float sx : {1.0f, -1.0f}) lb.addBox({sx * 0.30f, A2.y + 0.03f, A2.z + 0.01f}, {0.012f, 0.03f, 0.02f}, L.steel);
    }

    P.forEachMesh([&](MeshData& m) { fitToWheels(m, S); });
    P.finish(v);

    // Coil-overs from the tower eyes to the lower arms; the arms pivot on the lower frame rails.
    for (int a = 0; a < 2; ++a) {
        const AxleConfig& ax = cfg.axles[std::size_t(a)];
        const glm::vec3& T = a == 0 ? TF : TR;
        SuspensionMounts& m = v.mounts[a];
        m.set = true;
        m.shockTopX = T.x;
        m.shockTopY = T.y;
        m.shockTopDZ = T.z - W.axleZ[a];
        m.shockLowX = 0.5f * ax.track - 0.5f * ax.wheelWidth - (a == 0 ? 0.11f : 0.07f);
        m.armPivotX = FL0.x;
        m.armPivotY = FL0.y;
        m.armPivotHalfZ = 0.20f;
        m.armOuterInset = 0.14f;
        m.armOuterDrop = a == 0 ? 0.06f : 0.05f;
    }
    v.headlightLens = {0.235f, H(0.74f), 1.45f};
    v.mudArch = {zF, zR, kHubRest, 0.42f};
    v.mudBody = {0.64f, 1.30f, -1.35f, 0.0f};
}

} // namespace patrol

} // namespace kit

// ================================================================ traced bodies (bodyShape "traced")
// The body is the traced blueprint (vehicle/VehicleBlueprint.h): its field is polygonized, every triangle takes the
// colour class of the concept view it faces, and a part by where it lies (door shut lines, arches, lamp boxes). The
// accessories sit at the blueprint's anchor boxes, on the body surface found by ray casts.
namespace bpm {

constexpr float kCell = 0.02f; // polygonizer lattice (m)

// Concept-frame landmarks used to split the body into parts.
struct Landmarks {
    float R = 0.4f;           // concept tire radius
    float bumperTopF = 0.3f;  // bumper tops, just under the lamps
    float bumperTopR = 0.3f;
    float hoodRearZ = 0.8f;   // windshield base
    float headInnerX = 0.4f;  // |x| of the inner edge of the head lamps (grille / front panel between them)
    float tailInnerX = 0.6f;  // |x| of the inner edge of the tail lamps (tailgate between them)
    float roofFrontZ = 0.0f;  // roof: within 4 cm of the top
    float roofRearZ = 0.0f;
};

Landmarks landmarks(const BlueprintBody& B) {
    const VehicleBlueprint& bp = B.blueprint();
    Landmarks k;
    k.R = 0.5f * bp.tireDiameter;
    float fTop = 1e9f, rTop = 1e9f, hx = 1e9f, tx = 1e9f;
    for (const BlueprintLight& l : bp.lights) {
        const float lo = std::min(l.box[1], l.box[3]);
        const float inner = std::min(std::abs(l.box[0]), std::abs(l.box[2]));
        if (l.view == "front") {
            fTop = std::min(fTop, lo);
            if (l.kind == "head") hx = std::min(hx, inner);
        } else if (l.view == "rear" && l.kind == "tail") {
            rTop = std::min(rTop, lo);
            tx = std::min(tx, inner);
        }
    }
    if (fTop < 1e8f) k.bumperTopF = fTop - 0.02f;
    if (rTop < 1e8f) k.bumperTopR = rTop - 0.02f;
    if (hx < 1e8f) k.headInnerX = hx;
    if (tx < 1e8f) k.tailInnerX = tx;
    // Windshield base: going back from the front axle, where the top line first rises 4 cm above the hood (median
    // top around the axle).
    std::vector<float> hood;
    for (float zc = bp.archZ[0] - 0.35f; zc < bp.archZ[0] + 0.1f; zc += 0.01f)
        if (const float top = B.sideTop(zc); !std::isnan(top)) hood.push_back(top);
    if (!hood.empty()) {
        std::nth_element(hood.begin(), hood.begin() + std::ptrdiff_t(hood.size() / 2), hood.end());
        const float hoodTop = hood[hood.size() / 2];
        for (float zc = bp.archZ[0]; zc > 0.0f; zc -= 0.01f) {
            const float top = B.sideTop(zc);
            if (!std::isnan(top) && top > hoodTop + 0.04f) {
                k.hoodRearZ = zc;
                break;
            }
        }
    }
    const float yTop = B.conceptY(B.bodyTop());
    k.roofFrontZ = -1e9f;
    k.roofRearZ = 1e9f;
    for (float zc = -3.0f; zc < 3.0f; zc += 0.01f) {
        const float top = B.sideTop(zc);
        if (!std::isnan(top) && top > yTop - 0.04f) {
            k.roofFrontZ = std::max(k.roofFrontZ, zc);
            k.roofRearZ = std::min(k.roofRearZ, zc);
        }
    }
    return k;
}

struct BodyMaterials {
    style::Livery L;
    Material head, bar, brake;
    std::string centerPart; // between the head lamps: "grille" or "front_panel"
};

const BlueprintLight* lightAt(const VehicleBlueprint& bp, const char* view, float u, float v) {
    for (const BlueprintLight& l : bp.lights)
        if (l.view == view && u >= std::min(l.box[0], l.box[2]) && u <= std::max(l.box[0], l.box[2]) &&
            v >= std::min(l.box[1], l.box[3]) && v <= std::max(l.box[1], l.box[3]))
            return &l;
    return nullptr;
}

// Outermost body surface seen by each concept view, from the body mesh: top (max y over x, z), side (max |x| over z,
// y), front (max z over x, y), rear (max -z over x, y). A view only paints the surface it actually sees, so the
// plan view's shoulder colour does not land on the greenhouse below the roof edge, and so on.
struct ViewDepth {
    float u0 = 0.0f, v0 = 0.0f, cell = 0.02f;
    int w = 0, h = 0;
    std::vector<float> d;
    void init(float uLo, float uHi, float vLo, float vHi, float c) {
        u0 = uLo, v0 = vLo, cell = c;
        w = int((uHi - uLo) / c) + 2, h = int((vHi - vLo) / c) + 2;
        d.assign(std::size_t(w) * std::size_t(h), -1e9f);
    }
    int index(float u, float v) const {
        const int j = int(std::lround((u - u0) / cell)), i = int(std::lround((v - v0) / cell));
        return i < 0 || j < 0 || i >= h || j >= w ? -1 : i * w + j;
    }
    float depth(float u, float v) const {
        const int k = index(u, v);
        return k < 0 ? -1e9f : d[std::size_t(k)];
    }
    void add(float u, float v, float depth) {
        if (const int k = index(u, v); k >= 0) d[std::size_t(k)] = std::max(d[std::size_t(k)], depth);
    }
    void fillGaps() { // cells the mesh vertices skipped take their neighbours' surface
        for (int pass = 0; pass < 2; ++pass) {
            std::vector<float> n = d;
            for (int i = 0; i < h; ++i)
                for (int j = 0; j < w; ++j) {
                    if (d[std::size_t(i * w + j)] > -1e8f) continue;
                    for (int di = -1; di <= 1; ++di)
                        for (int dj = -1; dj <= 1; ++dj) {
                            const int y = i + di, x = j + dj;
                            if (y >= 0 && x >= 0 && y < h && x < w)
                                n[std::size_t(i * w + j)] = std::max(n[std::size_t(i * w + j)], d[std::size_t(y * w + x)]);
                        }
                }
            d.swap(n);
        }
    }
};
struct ViewDepths {
    ViewDepth side, top, front, rear;
    static constexpr float kTolerance = 0.04f; // m below the outermost surface that still counts as seen
    std::array<bool, 4> seen(const glm::vec3& c) const {
        return {std::abs(c.x) >= side.depth(c.z, c.y) - kTolerance, c.y >= top.depth(c.x, c.z) - kTolerance,
                c.z >= front.depth(c.x, c.y) - kTolerance, -c.z >= rear.depth(c.x, c.y) - kTolerance};
    }
};

// Colour class of a body point seen along its normal: the view the surface faces most among the views that see the
// point, then the next ones where the first has no body (silhouette edges). The side view is preferred over the
// plan (biased weights), so the greenhouse and the shoulders take the side colours; the end views come last.
char colourClass(const BlueprintBody& B, const ViewDepths& D, const glm::vec3& c, const glm::vec3& n) {
    const VehicleBlueprint& bp = B.blueprint();
    const float zc = B.conceptZ(c.z), yc = B.conceptY(c.y);
    const std::array<bool, 4> seen = D.seen(c);
    std::array<std::pair<float, int>, 4> w{{{2.0f * std::abs(n.x), 0}, {1.0f * std::max(n.y, 0.0f), 1},
                                            {std::max(n.z, 0.0f), 2}, {std::max(-n.z, 0.0f), 3}}};
    for (auto& [weight, view] : w)
        if (!seen[std::size_t(view)]) weight *= 0.1f;
    std::sort(w.begin(), w.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    for (const auto& [weight, view] : w) {
        if (weight < 0.005f) break;
        const char cls = view == 0   ? bp.side.smoothClassAt(zc, yc)
                         : view == 1 ? bp.top.smoothClassAt(zc, c.x)
                         : view == 2 ? bp.front.smoothClassAt(c.x, yc)
                                     : bp.rear.smoothClassAt(c.x, yc);
        if (cls != '.') return cls;
    }
    return '.';
}

std::string doorPart(const VehicleBlueprint& bp, const Landmarks& K, float zc, float sx) {
    const std::vector<float>& cuts = bp.doorCuts; // front to rear
    if (cuts.empty()) return "body_shell";
    if (zc > cuts[0]) return zc < bp.archZ[0] + 1.2f * K.R ? sided("fender_front", sx) : "body_shell";
    if (cuts.size() > 1 && zc > cuts[1]) return sided("door_front", sx);
    if (cuts.size() > 2 && zc > cuts[2]) return sided("door_rear", sx);
    return "body_shell";
}

std::pair<const Material*, std::string> classify(const BlueprintBody& B, const ViewDepths& D, const Landmarks& K,
                                                 const BodyMaterials& M, const glm::vec3& c, const glm::vec3& n,
                                                 const glm::vec3& viewN) {
    const VehicleBlueprint& bp = B.blueprint();
    const style::Livery& L = M.L;
    const float sx = c.x >= 0.0f ? 1.0f : -1.0f;
    const float zc = B.conceptZ(c.z), yc = B.conceptY(c.y);
    const float hull = B.hull(c.x, c.y, c.z), wh = B.wheelhouse(c.x, c.y, c.z), fen = B.fender(c.x, c.y, c.z);
    const std::string flare = std::string("flare_") + (c.z > 0.0f ? "front_" : "rear_") + sideName(sx);
    const bool onFender = fen < std::max(hull, -wh);
    if (onFender && std::abs(wh) < 0.5f * kFenderShell) return {&L.liner, flare}; // under the fender
    if (!onFender && -wh > hull) return {&L.liner, "body_shell"};                 // wheelhouse wall
    if (onFender) return {&L.trim, flare}; // the fender lips are unpainted plastic on both sheets

    std::string part = "body_shell";
    if (onFender) part = flare;
    else if (zc > bp.archZ[0] + 1.2f * K.R && yc < K.bumperTopF) part = "bumper_front";
    else if (zc < bp.archZ[1] - 1.2f * K.R && yc < K.bumperTopR) part = "bumper_rear";
    else if (n.z > 0.5f && c.z > 0.0f && yc < B.beltY(zc))
        part = std::abs(c.x) < K.headInnerX ? M.centerPart : "body_shell";
    else if (n.y > 0.5f && zc > K.hoodRearZ) part = "hood";
    else if (std::abs(n.x) > 0.45f && yc < B.glassTopY(zc) + 0.02f) part = doorPart(bp, K, zc, sx);
    else if (n.z < -0.5f && c.z < 0.0f && std::abs(c.x) < K.tailInnerX && yc > K.bumperTopR) part = "tailgate";

    if (n.z > 0.35f && c.z > 0.0f)
        if (const BlueprintLight* l = lightAt(bp, "front", c.x, yc))
            return l->kind == "bar" ? std::pair{&M.bar, M.centerPart} : std::pair{&M.head, sided("headlight", sx)};
    if (n.z < -0.35f && c.z < 0.0f)
        if (const BlueprintLight* l = lightAt(bp, "rear", c.x, yc))
            return {&M.brake, l->kind == "brake" ? std::string("tailgate") : sided("taillight", sx)};
    if (n.y < -0.6f) return {&L.underbody, part};

    switch (colourClass(B, D, c, viewN)) {
    case 'p': return {&L.body, part};
    case 'a': return {&L.accent, part};
    case 'w': return {&L.roof, part};
    case 'r': return {&L.hooks, part};
    case 'd':
        if (!onFender && yc > B.beltY(zc) - 0.01f && zc < K.hoodRearZ + 0.03f) {
            if (n.z > 0.3f && c.z > 0.0f) part = "windshield";
            return {&L.glass, part};
        }
        return {&L.trim, part};
    default: return {&L.trim, part};
    }
}

// Ray cast onto the body; `hit` is false when the ray misses within maxT.
struct Hit {
    glm::vec3 p{0.0f};
    bool hit = false;
};
Hit cast(const BlueprintBody& B, const glm::vec3& o, const glm::vec3& d, float maxT = 3.0f) {
    const float t = B.raySurface({o.x, o.y, o.z}, {d.x, d.y, d.z}, maxT);
    return {o + d * t, t < maxT};
}
// |x| of the body side at (y, z) (left side; the body is symmetric).
float sideX(const BlueprintBody& B, float y, float z) {
    const Hit h = cast(B, {1.6f, y, z}, {-1, 0, 0});
    return h.hit ? h.p.x : 0.0f;
}
float topY(const BlueprintBody& B, float x, float z) {
    const Hit h = cast(B, {x, B.bodyTop() + 0.3f, z}, {0, -1, 0});
    return h.hit ? h.p.y : B.bodyTop();
}
float endZ(const BlueprintBody& B, float x, float y, float zs) {
    const float z0 = zs > 0.0f ? B.bodyFront() + 0.3f : B.bodyRear() - 0.3f;
    const Hit h = cast(B, {x, y, z0}, {0, 0, -zs});
    return h.hit ? h.p.z : z0 - 0.3f * zs;
}

// Centre of an anchor box (u, v) in its view; body-space conversions are done by the caller.
glm::vec2 mid(const BlueprintAnchor& a) { return {0.5f * (a.box[0] + a.box[2]), 0.5f * (a.box[1] + a.box[3])}; }
glm::vec2 half(const BlueprintAnchor& a) {
    return {0.5f * std::abs(a.box[2] - a.box[0]), 0.5f * std::abs(a.box[3] - a.box[1])};
}

// Body mesh: the polygonized blueprint, split into parts and coloured from the concept views. Creates the
// body parts (shell, panels, lamps, flares) with their parents and hinge points.
void body(Parts& P, const BlueprintBody& B, const Landmarks& K, const BodyMaterials& M) {
    const VehicleBlueprint& bp = B.blueprint();
    const float yMid = B.bodyY(0.5f * (K.bumperTopF + B.beltY(0.0f)));
    const float skin = sideX(B, yMid, 0.0f);
    P.add("body_shell", "chassis", {0, yMid, 0}, true, false);
    const std::vector<float>& cuts = bp.doorCuts;
    for (float sx : {1.0f, -1.0f}) {
        P.add(sided("fender_front", sx), "body_shell", {sx * skin, yMid, B.axleZ(0)}, true, true);
        P.add(sided("door_front", sx), "body_shell", {sx * skin, yMid, B.bodyZ(cuts.empty() ? 0.7f : cuts[0])}, true,
              true); // hinge at the front
        P.add(sided("door_rear", sx), "body_shell", {sx * skin, yMid, B.bodyZ(cuts.size() > 1 ? cuts[1] : -0.25f)},
              true, true);
        for (int axle = 0; axle < 2; ++axle)
            P.add(std::string("flare_") + (axle == 0 ? "front_" : "rear_") + sideName(sx),
                  axle == 0 ? sided("fender_front", sx) : "body_shell",
                  {sx * B.fenderOuterX(axle), B.bodyY(K.R), B.axleZ(axle)}, true, true);
    }
    P.add("hood", "body_shell", {0, B.bodyY(B.beltY(K.hoodRearZ)), B.bodyZ(K.hoodRearZ)}, true, true); // cowl hinge
    P.add("tailgate", "body_shell", {-K.tailInnerX, yMid, B.bodyRear()}, true, true);             // side hinge (right)
    P.add("windshield", "body_shell", {0, B.bodyY(B.beltY(K.hoodRearZ) + 0.2f), B.bodyZ(K.hoodRearZ - 0.15f)}, false,
          true);
    P.add(M.centerPart, "body_shell", {0, B.bodyY(K.bumperTopF + 0.15f), B.bodyFront()}, true, true);
    P.add("bumper_front", "chassis", {0, B.bodyY(K.bumperTopF - 0.12f), B.bodyFront()}, true, true);
    P.add("bumper_rear", "chassis", {0, B.bodyY(K.bumperTopR - 0.12f), B.bodyRear()}, true, true);
    for (const BlueprintLight& l : bp.lights) {
        const float u = 0.5f * (l.box[0] + l.box[2]), sx = u >= 0.0f ? 1.0f : -1.0f;
        const glm::vec3 at(u, B.bodyY(0.5f * (l.box[1] + l.box[3])), l.view == "front" ? B.bodyFront() : B.bodyRear());
        if (l.kind == "head") P.add(sided("headlight", sx), "body_shell", at, false, true);
        if (l.kind == "tail") P.add(sided("taillight", sx), "body_shell", at, false, true);
    }

    float xMax = 0.0f;
    for (int axle = 0; axle < 2; ++axle) xMax = std::max(xMax, B.fenderOuterX(axle));
    xMax = std::max(xMax, 1.2f) + 2.0f * kCell;
    const glm::vec3 lo(-xMax, B.bodyBottom() - 2.0f * kCell, B.bodyRear() - 2.0f * kCell);
    const glm::vec3 hi(xMax, B.bodyTop() + 2.0f * kCell, B.bodyFront() + 2.0f * kCell);
    const FieldMesh fm =
        polygonizeField([&](const glm::vec3& p) { return B.field(p.x, p.y, p.z); }, lo, hi, kCell);
    const float crease = std::cos(glm::radians(40.0f));

    // Paint: every mesh vertex gets its region (material and part) from the view its normal faces; a triangle whose
    // corners disagree is split where the region changes along its edges (bisection on the edge, the same point for
    // both triangles sharing it), so region borders are straight cuts through the triangles rather than the
    // 2 cm staircase of the mesh lattice.
    using Region = std::pair<const Material*, std::string>;
    ViewDepths D;
    D.side.init(lo.z, hi.z, lo.y, hi.y, kCell);
    D.top.init(lo.x, hi.x, lo.z, hi.z, kCell);
    D.front.init(lo.x, hi.x, lo.y, hi.y, kCell);
    D.rear.init(lo.x, hi.x, lo.y, hi.y, kCell);
    for (const glm::vec3& p : fm.positions) {
        // The fender lips stand out of the body side: they must not hide the panel around the arch from the side
        // view (they are plain trim anyway).
        if (B.fender(p.x, p.y, p.z) < std::max(B.hull(p.x, p.y, p.z), -B.wheelhouse(p.x, p.y, p.z))) continue;
        D.side.add(p.z, p.y, std::abs(p.x));
        D.top.add(p.x, p.z, p.y);
        D.front.add(p.x, p.y, p.z);
        D.rear.add(p.x, p.y, -p.z);
    }
    for (ViewDepth* v : {&D.side, &D.top, &D.front, &D.rear}) v->fillGaps();
    // Regions are chosen with the body's normal at panel scale (field gradient over 8 cm), not the mesh normal:
    // the surface-net normals ripple at the lattice size. Which concept view paints the point is decided with the
    // normal over 30 cm, so the borders between views (roof edges, shoulders) run along the panels instead of
    // following every ripple of the traced outline.
    auto gradient = [&](const glm::vec3& p, const glm::vec3& n, float h) {
        const glm::vec3 g(B.field(p.x + h, p.y, p.z) - B.field(p.x - h, p.y, p.z),
                          B.field(p.x, p.y + h, p.z) - B.field(p.x, p.y - h, p.z),
                          B.field(p.x, p.y, p.z + h) - B.field(p.x, p.y, p.z - h));
        const float len = glm::length(g);
        return len > 1e-6f && glm::dot(g, n) > 0.0f ? g / len : n;
    };
    auto regionAt = [&](const glm::vec3& p, const glm::vec3& n) {
        const glm::vec3 np = gradient(p, n, 0.08f);
        return classify(B, D, K, M, p, np, gradient(p, np, 0.30f));
    };
    std::vector<Region> vreg(fm.positions.size());
    std::vector<char> vdone(fm.positions.size(), 0);
    auto vertexRegion = [&](std::uint32_t v) -> const Region& {
        if (!vdone[v]) {
            vreg[v] = regionAt(fm.positions[v], fm.normals[v]);
            vdone[v] = 1;
        }
        return vreg[v];
    };
    struct Split {
        glm::vec3 p, n;
    };
    std::unordered_map<std::uint64_t, Split> splits;
    auto edgeSplit = [&](std::uint32_t a, std::uint32_t b) -> Split {
        if (a > b) std::swap(a, b);
        const std::uint64_t key = (std::uint64_t(a) << 32) | b;
        if (auto it = splits.find(key); it != splits.end()) return it->second;
        const glm::vec3 pa = fm.positions[a], pb = fm.positions[b], na = fm.normals[a], nb = fm.normals[b];
        const Region& ra = vertexRegion(a);
        float lo = 0.0f, hi = 1.0f;
        for (int it = 0; it < 7; ++it) {
            const float t = 0.5f * (lo + hi);
            const glm::vec3 n = na + (nb - na) * t;
            (regionAt(pa + (pb - pa) * t, glm::dot(n, n) > 1e-8f ? glm::normalize(n) : na) == ra ? lo : hi) = t;
        }
        const float t = 0.5f * (lo + hi);
        glm::vec3 n = na + (nb - na) * t;
        n = glm::dot(n, n) > 1e-8f ? glm::normalize(n) : na;
        return splits[key] = Split{pa + (pb - pa) * t, n};
    };

    for (const auto& q : fm.quads)
        for (const auto& tri : {std::array<std::uint32_t, 3>{q[0], q[1], q[2]}, std::array<std::uint32_t, 3>{q[0], q[2], q[3]}}) {
            const glm::vec3 a = fm.positions[tri[0]], b = fm.positions[tri[1]], c = fm.positions[tri[2]];
            const glm::vec3 cr = glm::cross(b - a, c - a);
            const float len = glm::length(cr);
            if (len < 1e-10f) continue;
            // Surface-net faces zig-zag on oblique surfaces, so the face orientation comes from the field normals.
            glm::vec3 fn = fm.normals[tri[0]] + fm.normals[tri[1]] + fm.normals[tri[2]];
            fn = glm::dot(fn, fn) > 1e-8f ? glm::normalize(fn) : cr / len;
            if (glm::dot(fn, cr) < 0.0f) fn = cr / len;
            auto emit = [&](const Split& p0, const Split& p1, const Split& p2, const Region& r) {
                glm::vec3 nv[3] = {p0.n, p1.n, p2.n};
                for (glm::vec3& n : nv)
                    if (glm::dot(n, fn) < crease) n = fn; // a vertex normal far from the triangle's mean: crease
                P[r.second].addTri(p0.p, p1.p, p2.p, nv[0], nv[1], nv[2], *r.first);
            };
            Split s[3];
            const Region* r[3];
            for (int k = 0; k < 3; ++k) {
                s[k] = {fm.positions[tri[k]], fm.normals[tri[k]]};
                r[k] = &vertexRegion(tri[k]);
            }
            if (*r[0] == *r[1] && *r[1] == *r[2]) {
                emit(s[0], s[1], s[2], *r[0]);
                continue;
            }
            // The corner whose region differs from the other two (rotated first, keeping the winding).
            int lone = -1;
            for (int k = 0; k < 3; ++k)
                if (*r[(k + 1) % 3] == *r[(k + 2) % 3]) lone = k;
            if (lone >= 0) {
                const int k0 = lone, k1 = (lone + 1) % 3, k2 = (lone + 2) % 3;
                const Split e01 = edgeSplit(tri[k0], tri[k1]), e02 = edgeSplit(tri[k0], tri[k2]);
                emit(s[k0], e01, e02, *r[k0]);
                emit(e01, s[k1], s[k2], *r[k1]);
                emit(e01, s[k2], e02, *r[k1]);
                continue;
            }
            // Three regions meet: split every edge and join the splits at the centroid.
            const Split e[3] = {edgeSplit(tri[0], tri[1]), edgeSplit(tri[1], tri[2]), edgeSplit(tri[2], tri[0])};
            glm::vec3 gn = s[0].n + s[1].n + s[2].n;
            const Split g{(a + b + c) / 3.0f, glm::dot(gn, gn) > 1e-8f ? glm::normalize(gn) : fn};
            for (int k = 0; k < 3; ++k) {
                emit(s[k], e[k], g, *r[k]);
                emit(s[k], g, e[(k + 2) % 3], *r[k]);
            }
        }

    // Door shut lines: thin dark strips on the side skin at the traced cuts, from the sill up to the belt.
    for (float zc : cuts)
        for (float sx : {1.0f, -1.0f}) {
            const float z = B.bodyZ(zc);
            const std::string part = doorPart(bp, K, zc - 0.02f, sx);
            glm::vec3 prev(0.0f);
            bool have = false;
            for (float yc = K.bumperTopF - 0.25f; yc <= B.beltY(zc) - 0.01f; yc += 0.02f) {
                const Hit h = cast(B, {sx * 1.6f, B.bodyY(yc), z}, {-sx, 0, 0});
                const bool ok = h.hit && std::abs(h.p.x) > B.innerX() + 0.12f && B.wheelhouse(h.p.x, h.p.y, h.p.z) > 0.03f;
                if (ok && have) style::panelGap(P[part], prev, h.p, {sx, 0, 0}, 0.006f, M.L.liner, 0.002f);
                prev = h.p;
                have = ok;
            }
        }
}

// Chassis seen through the wheel openings and from below: frame rails, a dark tub wall just inboard of the
// wheelhouses (backdrop for the coil-overs) and the skid plate.
void chassis(Parts& P, const BlueprintBody& B, const style::Livery& L) {
    MeshData& ch = P.add("chassis", "", {0, 0, 0}, false, false);
    const float zF = B.bodyFront() - 0.25f, zR = B.bodyRear() + 0.25f;
    const float floorY = B.bodyBottom() + 0.06f;
    for (float sx : {1.0f, -1.0f})
        ch.addBeveledBox({sx * 0.40f, floorY + 0.02f, 0.5f * (zF + zR)}, {0.045f, 0.07f, 0.5f * (zF - zR)}, 0.01f,
                         L.steel);
    for (float z : {zR + 0.15f, 0.5f * (B.axleZ(1) + 0.0f), 0.5f * B.axleZ(0), zF - 0.15f})
        ch.addBox({0.0f, floorY + 0.02f, z}, {0.40f, 0.04f, 0.04f}, L.steel);
    const float x = B.innerX() - 0.008f, top = B.bodyY(0.30f);
    ch.addBox({0.0f, 0.5f * (floorY + top), 0.5f * (zF + zR)}, {x, 0.5f * (top - floorY), 0.5f * (zF - zR)}, L.underbody);
}

// Skid plate guards at the front / rear anchor boxes (end view: width and height), just outboard of the lower arms
// and within the tires' length, so the side view shows them only behind the tires, as on the concept.
void skidPlates(Parts& P, const BlueprintBody& B, const style::Livery& L) {
    for (int axle = 0; axle < 2; ++axle) {
        const auto a = B.blueprint().find("skid", axle == 0 ? "front" : "rear");
        if (a.empty()) continue;
        const glm::vec2 c = mid(*a[0]), h = half(*a[0]);
        const float zs = axle == 0 ? 1.0f : -1.0f;
        const float z0 = B.axleZ(axle) + zs * 0.30f, z1 = B.axleZ(axle) + zs * 0.40f;
        MeshData& m = P.add(axle == 0 ? "skid_plate" : "skid_plate_rear", "chassis", {c.x, B.bodyY(c.y), z1}, true, true);
        m.addBeveledBox({c.x, B.bodyY(c.y), 0.5f * (z0 + z1)}, {h.x, h.y, 0.05f}, 0.02f, L.steel);
        m.addBeveledBox({c.x, B.bodyY(c.y - h.y) + 0.006f, 0.5f * (z0 + z1)}, {h.x - 0.03f, 0.006f, 0.04f}, 0.004f,
                        L.alu);
    }
}

// Mirror housing centre (x, concept y) from a front-view anchor; when the rear view has mirror anchors too, the
// height is the mean of both views (the sheet draws them at different heights).
glm::vec2 mirrorCentre(const VehicleBlueprint& bp, const BlueprintAnchor& front) {
    glm::vec2 c = mid(front);
    const auto rear = bp.find("mirror", "rear");
    if (!rear.empty()) {
        float y = 0.0f;
        for (const BlueprintAnchor* r : rear) y += mid(*r).y;
        c.y = 0.5f * (c.y + y / float(rear.size()));
    }
    return c;
}

// Rock sliders: tubes along the sill on a belly plate between them (the collision box of BlueprintBody::sliderBox).
void sliders(Parts& P, const BlueprintBody& B, const Material& tube, const Material& caps, const Material& plate) {
    const BlueprintBody::Box b = B.sliderBox();
    if (b.halfX <= 0.0f) return;
    const float r = 0.038f, y = b.y1 - r;
    for (float sx : {1.0f, -1.0f}) {
        MeshData& m = P.add(sided("rock_slider", sx), "chassis", {sx * b.halfX, y, 0}, true, true);
        style::rockSlider(m, sx, b.halfX - r, y, b.z0 + 0.08f, b.z1 - 0.08f, tube, caps, plate);
        m.addBeveledBox({sx * 0.5f * (b.halfX - r), 0.5f * (b.y0 + y), 0.5f * (b.z0 + b.z1)},
                        {0.5f * (b.halfX - r), 0.5f * (y - b.y0), 0.5f * (b.z1 - b.z0)}, 0.015f, plate);
    }
}

// Raised fender tops along the hood edges: z and height from the side-view anchor, x from the plan anchors (one per
// side). The tracer masks them out of the side silhouette, which would extrude them across the whole hood.
void fenderTops(Parts& P, const BlueprintBody& B, const Material& mat) {
    const VehicleBlueprint& bp = B.blueprint();
    const auto side = bp.find("fender_top", "side");
    if (side.empty()) return;
    const auto& s = side[0]->box;
    const float z0 = B.bodyZ(std::min(s[0], s[2])), z1 = B.bodyZ(std::max(s[0], s[2]));
    const float top = B.bodyY(std::max(s[1], s[3]));
    for (const BlueprintAnchor* a : bp.find("fender_top", "top")) {
        const float x0 = std::min(a->box[1], a->box[3]), x1 = std::max(a->box[1], a->box[3]);
        const float xc = 0.5f * (x0 + x1);
        const glm::vec3 down(0, -1, 0);
        const float y0 = cast(B, {xc, top + 0.3f, z0}, down).p.y, y1 = cast(B, {xc, top + 0.3f, z1}, down).p.y;
        const std::vector<glm::vec2> outline{{z1, y1 - 0.02f}, {z1 - 0.07f, top}, {z0 + 0.02f, top}, {z0, y0 - 0.02f}};
        MeshData& m = P.add(sided("fender_top", xc >= 0.0f ? 1.0f : -1.0f), "body_shell", {xc, top, 0.5f * (z0 + z1)},
                            true, false);
        style::prism(m, outline, glm::vec3(0.0f), {0, 0, 1}, {0, 1, 0}, {1, 0, 0}, x0, x1, mat, mat, mat);
    }
}

} // namespace bpm

// ================================================================ traced trailforge_suv
// Gasoline expedition SUV (Cars.png car #1): traced boxy five-door body, olive paint with orange accents, white
// roof, black flares and steel bumpers; winch, tow shackles, skid plate, roof rails with a light bar and strapped
// cargo, spare wheel and jerry cans on the tailgate. The concept has no snorkel and no ladder.
void buildTracedSuv(VehicleVisual& v, const VehicleConfig& cfg, const VehicleBlueprint& bp) {
    const BlueprintBody B(bp, cfg);
    const bpm::Landmarks K = bpm::landmarks(B);
    bpm::BodyMaterials M{style::oliveExpedition(), style::led(MatId::Headlight, {1.0f, 0.95f, 0.88f}),
                         style::led(MatId::Headlight, {0.85f, 0.92f, 1.0f}),
                         style::led(MatId::BrakeLight, {1.0f, 0.04f, 0.02f}), "grille"};
    const style::Livery& L = M.L;
    const Material chrome = style::metal(glm::vec3(0.92f), 0.05f, 0.0f);
    const Material barLed = M.bar;
    const Material red = style::paint(style::srgb(0.80f, 0.12f, 0.08f), 0.40f, 0.6f);
    const Material olive = style::plastic(style::srgb(0.33f, 0.36f, 0.22f), 0.65f, 0.6f);
    const Material iron{MatId::Generic, {0.06f, 0.06f, 0.06f}, 0.50f, 0.5f, 1.0f};
    const glm::vec3 fwd(0, 0, 1);

    Parts P;
    bpm::chassis(P, B, L);
    MeshData& chassis = P["chassis"];
    const float diffY = bp.restHubY + 0.11f;
    for (int axle = 0; axle < 2; ++axle) {
        const float zc = B.axleZ(axle);
        chassis.addCylinder({0.0f, diffY, zc - 0.10f}, {0.0f, diffY, zc + 0.07f}, 0.12f, 18, iron);
        chassis.addCylinder({0.0f, diffY, zc + 0.07f}, {0.0f, diffY, zc + 0.10f}, 0.085f, 18, iron);
    }
    chassis.addCylinder({-0.45f, B.bodyBottom() + 0.02f, B.axleZ(1) - 0.15f},
                        {-0.45f, B.bodyBottom() + 0.02f, B.bodyRear() - 0.05f}, 0.032f, 12, L.alu);
    bpm::body(P, B, K, M);
    bpm::fenderTops(P, B, L.accent); // orange strips on the raised fender tops, as on the plan

    // ---- Front: winch and shackles at their anchors on the bumper face, skid plate under it
    const float yWinch = B.bodyY(bpm::mid(*bp.find("winch", "front").at(0)).y);
    const float zBumper = bpm::endZ(B, 0.0f, yWinch, 1.0f);
    bpm::skidPlates(P, B, L);
    style::winch(P.add("winch", "bumper_front", {0, yWinch, zBumper}, false, true), {0.0f, yWinch, zBumper + 0.04f},
                 L.gunmetal, L.hooks, L.trim);
    for (const BlueprintAnchor* a : bp.find("tow_hook", "front")) {
        const glm::vec2 m = bpm::mid(*a);
        const float sx = m.x >= 0.0f ? 1.0f : -1.0f;
        const glm::vec3 mount(m.x, B.bodyY(m.y + 0.5f * bpm::half(*a).y), bpm::endZ(B, m.x, B.bodyY(m.y), 1.0f));
        style::shackle(P.add(sided("tow_hook_front", sx), "bumper_front", mount, false, true), mount, fwd, L.hooks);
    }
    for (float sx : {1.0f, -1.0f}) {
        const glm::vec3 mount(sx * 0.40f, B.bodyY(K.bumperTopR - 0.22f), 0.0f);
        const glm::vec3 m2(mount.x, mount.y, bpm::endZ(B, mount.x, mount.y, -1.0f));
        style::shackle(P.add(sided("tow_hook_rear", sx), "bumper_rear", m2, false, true), m2, -fwd, L.hooks);
    }

    // ---- Sides: mirrors, rock sliders
    const BlueprintAnchor& mirrorSide = *bp.find("mirror", "side").at(0);
    for (const BlueprintAnchor* a : bp.find("mirror", "front")) {
        const float sx = bpm::mid(*a).x >= 0.0f ? 1.0f : -1.0f;
        // style::mirror puts the housing centre at base + (0.155 sx, 0.095, -0.03).
        const glm::vec2 c = bpm::mirrorCentre(bp, *a);
        const float z = B.bodyZ(bpm::mid(mirrorSide).x) + 0.03f;
        const glm::vec3 base(c.x - sx * 0.155f, B.bodyY(c.y) - 0.095f, z);
        style::mirror(P.add(sided("mirror", sx), sided("door_front", sx), base, false, true), sx, base, L.trim, chrome);
    }
    {
        bpm::sliders(P, B, L.alu, L.accent, L.steel);
    }

    // ---- Rear: spare on a tailgate carrier, jerry cans
    {
        const BlueprintAnchor& s = *bp.find("spare", "rear").at(0);
        const glm::vec2 c = bpm::mid(s);
        const float y = B.bodyY(c.y), z = bpm::endZ(B, c.x, y, -1.0f);
        style::spareCarrier(P.add("spare_carrier_rear", "tailgate", {c.x, y, z}, false, true), {c.x, y, z}, -fwd,
                            0.30f, L.steel);
        v.spares.push_back({"spare_rear", "spare_carrier_rear", {c.x, y, z - 0.026f}, {0, 0, -1}, true});
    }
    for (const BlueprintAnchor* a : bp.find("jerry_can", "rear")) {
        const glm::vec2 c = bpm::mid(*a);
        const float sx = c.x >= 0.0f ? 1.0f : -1.0f;
        const float y = B.bodyY(c.y), z = bpm::endZ(B, c.x, y, -1.0f) - 0.07f;
        style::jerryCan(P.add(sided("jerry_can", sx), "tailgate", {c.x, y, z}, false, true), {c.x, y, z}, -fwd,
                        sx > 0.0f ? L.accent : red, L.trim);
    }
    // ---- Roof rails with crossbars, light bar, pods and strapped cargo (the concept's rack load; no roof spare)
    {
        const BlueprintAnchor& r = *bp.find("rack", "side").at(0);
        const float z0 = B.bodyZ(r.box[0]) + 0.05f, z1 = B.bodyZ(r.box[2]) - 0.05f;
        const float roof = bpm::topY(B, 0.0f, 0.5f * (z0 + z1));
        const float railX = bpm::sideX(B, roof - 0.04f, 0.5f * (z0 + z1)) - 0.08f;
        const float cz[3] = {z1 - 0.12f, 0.5f * (z0 + z1), z0 + 0.20f};
        MeshData& rails = P.add("roof_rails", "body_shell", {0, roof, 0.5f * (z0 + z1)}, true, true);
        const float top = style::roofRails(rails, railX, z0, z1, roof, {cz[0], cz[1], cz[2]}, L.trim, L.liner);
        const Material strap{MatId::Generic, {0.03f, 0.03f, 0.035f}, 0.7f, 0.0f, 0.8f};
        MeshData& lb = P.add("light_bar", "roof_rails", {0, top, cz[0]}, false, true);
        style::lightBar(lb, {0.0f, top + 0.07f, cz[0] + 0.05f}, railX - 0.16f, L.trim, barLed);
        for (float sx : {1.0f, -1.0f})
            style::roundLamp(lb, {sx * (railX - 0.04f), top + 0.05f, cz[0] + 0.06f}, fwd, 0.045f, 0.05f, L.trim,
                             barLed);
        auto strapOver = [&](MeshData& m, float x0, float x1, float y, float z) {
            style::cargoStrap(m, {x0 - 0.012f, top, z}, {x0 - 0.012f, y + 0.004f, z}, {x1 + 0.012f, y + 0.004f, z},
                              {x1 + 0.012f, top, z}, strap);
        };
        // The two cases of the side view (the plan draws other cargo there); the roof spare between them is left out.
        std::vector<const BlueprintAnchor*> cases = bp.find("case", "side");
        std::sort(cases.begin(), cases.end(), [](auto* a, auto* b) { return a->box[0] + a->box[2] > b->box[0] + b->box[2]; });
        for (std::size_t i = 0; i < cases.size(); ++i) {
            const auto& b = cases[i]->box;
            const float c0 = B.bodyZ(std::min(b[0], b[2])), c1 = B.bodyZ(std::max(b[0], b[2]));
            const float hy = 0.5f * std::abs(b[3] - b[1]), z = 0.5f * (c0 + c1), hz = 0.5f * (c1 - c0);
            const bool front = i == 0;
            MeshData& m = P.add(front ? "case_front" : "case_rear", "roof_rails", {0.0f, top, z}, false, true);
            const float hx = front ? 0.30f : 0.34f;
            style::cargoCase(m, {0.0f, top + hy, z}, {hx, hy, hz}, front ? L.trim : olive, L.trim);
            if (front)
                for (float sx : {1.0f, -1.0f})
                    m.addBox({sx * (hx - 0.08f), top + hy, z + hz + 0.002f}, {0.05f, 0.5f * hy, 0.003f}, barLed);
            strapOver(m, -hx, hx, top + 2.0f * hy, z);
        }
    }

    P.finish(v);
}

// ================================================================ traced trailforge_ev
// Dual-motor electric SUV (Cars.png car #2): traced body, white / light grey paint, blue accents, black cladding,
// closed front panel with a full-width LED bar, full-width tail light, low roof rails with a light bar and one case,
// blue tow hooks. No spare (the concept has none), no snorkel. Chassis: frame rails, floor battery pack, one motor
// per axle.
void buildTracedEv(VehicleVisual& v, const VehicleConfig& cfg, const VehicleBlueprint& bp) {
    const BlueprintBody B(bp, cfg);
    const bpm::Landmarks K = bpm::landmarks(B);
    bpm::BodyMaterials M{style::arcticElectric(), style::led(MatId::Headlight, {0.90f, 0.95f, 1.0f}),
                         style::led(MatId::Headlight, {0.90f, 0.95f, 1.0f}),
                         style::led(MatId::BrakeLight, {1.0f, 0.04f, 0.02f}), "front_panel"};
    const style::Livery& L = M.L;
    const Material chrome = style::metal(glm::vec3(0.92f), 0.05f, 0.0f);
    const Material blueLed = style::led(MatId::Headlight, {0.10f, 0.40f, 1.0f});
    const Material pack{MatId::Generic, glm::vec3(0.05f), 0.55f, 0.6f, 1.0f};
    const glm::vec3 fwd(0, 0, 1);

    Parts P;
    bpm::chassis(P, B, L);
    const float floorY = B.bodyBottom() + 0.06f;
    MeshData& battery = P.add("battery_pack", "chassis", {0, floorY, 0}, true, false);
    const float packHalf = 0.5f * (B.axleZ(0) - B.axleZ(1)) - 0.52f;
    battery.addBeveledBox({0.0f, floorY - 0.005f, 0.0f}, {B.innerX() - 0.02f, 0.055f, packHalf}, 0.02f, pack);
    for (float z : {-0.6f, -0.2f, 0.2f, 0.6f})
        if (std::abs(z) < packHalf - 0.05f)
            battery.addBox({0.0f, floorY - 0.061f, z}, {B.innerX() - 0.06f, 0.002f, 0.012f}, L.steel);
    const float motorY = bp.restHubY + 0.13f;
    for (int axle = 0; axle < 2; ++axle) {
        const float zc = B.axleZ(axle);
        MeshData& motor = P.add(axle == 0 ? "motor_front" : "motor_rear", "chassis", {0, motorY, zc}, false, true);
        motor.addCylinder({-0.17f, motorY, zc + 0.02f}, {0.12f, motorY, zc + 0.02f}, 0.115f, 20, L.alu);
        motor.addCylinder({0.12f, motorY, zc + 0.02f}, {0.15f, motorY, zc + 0.02f}, 0.085f, 20, L.gunmetal);
        motor.addBeveledBox({-0.02f, motorY + 0.13f, zc + 0.02f}, {0.14f, 0.035f, 0.09f}, 0.012f, L.gunmetal);
        motor.addCylinder({-0.08f, motorY, zc - 0.05f}, {-0.08f, motorY, zc - 0.16f}, 0.07f, 16, L.alu);
    }
    bpm::body(P, B, K, M);
    bpm::fenderTops(P, B, L.body);

    // ---- Front: skid plate, tow shackles under the bumper (in line with the rear ones)
    bpm::skidPlates(P, B, L);
    for (const BlueprintAnchor* a : bp.find("tow_hook", "rear")) {
        const glm::vec2 m = bpm::mid(*a);
        const float sx = m.x >= 0.0f ? 1.0f : -1.0f;
        const float y = B.bodyY(m.y + 0.5f * bpm::half(*a).y);
        const glm::vec3 rear(m.x, y, bpm::endZ(B, m.x, B.bodyY(m.y), -1.0f));
        style::shackle(P.add(sided("tow_hook_rear", sx), "bumper_rear", rear, false, true), rear, -fwd, L.hooks);
        const glm::vec3 front(m.x, y, bpm::endZ(B, m.x, B.bodyY(m.y), 1.0f));
        style::shackle(P.add(sided("tow_hook_front", sx), "bumper_front", front, false, true), front, fwd, L.hooks);
    }

    // ---- Sides: mirrors, step sliders
    const BlueprintAnchor& mirrorSide = *bp.find("mirror", "side").at(0);
    for (const BlueprintAnchor* a : bp.find("mirror", "front")) {
        const float sx = bpm::mid(*a).x >= 0.0f ? 1.0f : -1.0f;
        // style::mirror puts the housing centre at base + (0.155 sx, 0.095, -0.03).
        const glm::vec2 c = bpm::mirrorCentre(bp, *a);
        const float z = B.bodyZ(bpm::mid(mirrorSide).x) + 0.03f;
        const glm::vec3 base(c.x - sx * 0.155f, B.bodyY(c.y) - 0.095f, z);
        style::mirror(P.add(sided("mirror", sx), sided("door_front", sx), base, false, true), sx, base, L.trim, chrome);
    }
    {
        bpm::sliders(P, B, L.trim, L.accent, L.steel);
    }

    // ---- Low roof rails with crossbars, front light bar with blue end pods, blue LED strips on the rails, one
    // flat case. No antenna, no snorkel (no engine air intake).
    {
        const BlueprintAnchor& r = *bp.find("rack", "side").at(0);
        const float z0 = B.bodyZ(r.box[0]) + 0.05f, z1 = B.bodyZ(r.box[2]) - 0.05f;
        const float roof = bpm::topY(B, 0.0f, 0.5f * (z0 + z1));
        const float railX = bpm::sideX(B, roof - 0.04f, 0.5f * (z0 + z1)) - 0.08f;
        const float cz[3] = {z1 - 0.12f, 0.5f * (z0 + z1), z0 + 0.20f};
        MeshData& rails = P.add("roof_rails", "body_shell", {0, roof, 0.5f * (z0 + z1)}, true, true);
        const float top = style::roofRails(rails, railX, z0, z1, roof, {cz[0], cz[1], cz[2]}, L.trim, L.liner);
        for (float sx : {1.0f, -1.0f})
            rails.addBox({sx * (railX + 0.0095f), roof + 0.050f, 0.5f * (z0 + z1)},
                         {0.0015f, 0.004f, 0.5f * (z1 - z0) - 0.12f}, blueLed);
        MeshData& lb = P.add("light_bar", "roof_rails", {0, top, cz[0]}, false, true);
        style::lightBar(lb, {0.0f, top + 0.06f, cz[0] + 0.05f}, railX - 0.18f, L.trim, M.head);
        for (float sx : {1.0f, -1.0f}) {
            lb.addBeveledBox({sx * (railX - 0.07f), top + 0.05f, cz[0] + 0.04f}, {0.05f, 0.03f, 0.03f}, 0.01f, L.trim);
            lb.addBox({sx * (railX - 0.07f), top + 0.05f, cz[0] + 0.071f}, {0.040f, 0.020f, 0.002f}, blueLed);
        }
        const float cmid = 0.5f * (cz[1] + cz[2]);
        MeshData& cs = P.add("case_roof", "roof_rails", {0.0f, top, cmid}, false, true);
        style::cargoCase(cs, {0.0f, top + 0.06f, cmid}, {railX - 0.22f, 0.06f, 0.36f}, L.trim, L.accent);
    }

    P.finish(v);
}

// Wheels and coil springs of a model (also used under a Blender body: wheels and suspension stay procedural).
void wheelStyle(VehicleVisual& v, bool ev) {
    v.wheel.beadlock = false;
    if (ev) {
        v.wheel.spokes = 6;
        v.wheel.rim = {MatId::Generic, glm::vec3(0.030f), 0.38f, 0.9f, 0.6f};
        v.wheel.ring = {MatId::Generic, glm::vec3(0.16f), 0.35f, 1.0f, 0.5f};
        v.coil = {MatId::Generic, style::srgb(0.10f, 0.42f, 0.92f), 0.40f, 0.2f, 0.8f}; // blue coils (EV accent)
    } else {
        v.wheel.spokes = 8;
        v.wheel.rim = {MatId::Generic, glm::vec3(0.040f), 0.40f, 0.9f, 0.6f};
        v.wheel.ring = {MatId::Generic, glm::vec3(0.22f), 0.35f, 1.0f, 0.5f};
        v.coil = {MatId::Generic, style::srgb(0.96f, 0.50f, 0.13f), 0.40f, 0.2f, 0.8f}; // orange coils (concept)
    }
}

// ================================================================ Blender bodies (body.glb)
// LOD0 of every part, materials by role (README "Vehicle data format"): the light roles get the shader's light ids
// so the lamps follow the headlight switch, brake pedal, reverse gear and indicators; paint roles can be recoloured
// by vehicle.json body.paint.
Material roleMaterial(const BodyMaterial& m, const VehicleConfig& cfg) {
    const glm::vec3 base(m.color[0], m.color[1], m.color[2]);
    const glm::vec3 emissive(m.emissive[0], m.emissive[1], m.emissive[2]);
    const float peak = std::max({emissive.x, emissive.y, emissive.z});
    const glm::vec3 light = peak > 0.0f ? emissive / peak : base;
    Material out{MatId::Generic, base, m.roughness, m.metallic, m.mud};
    switch (m.role) {
    case BodyRole::PaintPrimary:
    case BodyRole::PaintSecondary:
    case BodyRole::PaintAccent: {
        out.id = MatId::Paint;
        const auto it = cfg.bodyPaint.find(bodyRoleName(m.role));
        if (it != cfg.bodyPaint.end()) out.color = style::srgb(it->second[0], it->second[1], it->second[2]);
        break;
    }
    case BodyRole::Cladding: out.id = MatId::Cladding; break;
    case BodyRole::Glass: out.id = MatId::Glass; break;
    case BodyRole::Rubber: out.id = MatId::Rubber; break;
    case BodyRole::LightHead: out = style::led(MatId::Headlight, light); break;
    case BodyRole::LightTail:
    case BodyRole::LightBrake: out = style::led(MatId::BrakeLight, light); break;
    case BodyRole::LightReverse: out = style::led(MatId::ReverseLight, light); break;
    case BodyRole::LightIndicator: out = style::led(MatId::Amber, light); break;
    default: break;
    }
    if (m.role >= BodyRole::LightHead && m.role <= BodyRole::LightIndicator) out.mud = m.mud;
    return out;
}

bool buildFromBody(VehicleVisual& v, const VehicleConfig& cfg, std::string& error) {
    BodyModel body;
    if (!loadBodyModel(cfg.bodyModel, body, true, &error)) return false;
    std::vector<Material> mats;
    for (const BodyMaterial& m : body.materials) mats.push_back(roleMaterial(m, cfg));
    glm::vec3 lens(0.0f);
    int lensCount = 0;
    const float r = 0.5f * (cfg.axles[0].wheelRadius + cfg.axles[1].wheelRadius);
    const auto sweeps = wheelSweeps(cfg);
    // Static sag of a spring with natural frequency f is g / (2 pi f)^2, whatever the load.
    float restHub = 0.0f;
    for (int a = 0; a < 2; ++a) {
        const AxleConfig& ax = cfg.axles[std::size_t(a)];
        const float w = 2.0f * kPi * cfg.tuning.axles[std::size_t(a)].springFrequency;
        const float len = std::clamp(ax.suspensionMaxLength - 9.81f / (w * w), ax.suspensionMinLength, ax.suspensionMaxLength);
        restHub += 0.5f * (cfg.attachHeight - len);
    }
    // The body has no shock towers: the procedural coil-over and lower arm layout (made for the SUV frame) moves with
    // each axle's hub so the drawn shock is 0.06 m longer than its body at full bump (the top eye as low as it can
    // be, under a low bonnet), and the coil (0.064 m radius) sits inboard of the steered tire's inner edge.
    for (int a = 0; a < 2; ++a) {
        const WheelSweep& S = sweeps[std::size_t(2 * a)];
        SuspensionMounts& m = v.mounts[a];
        const float inner = std::abs(S.x) - S.halfWidth * std::cos(S.steer) - 0.5f * S.radius * std::sin(S.steer);
        m.shockTopX = m.shockLowX = std::min(m.shockTopX, inner - 0.064f - 0.02f);
        m.armPivotY = restHub - m.armOuterDrop; // lower arm level at rest
        for (int it = 0; it < 3; ++it) {
            const SuspensionPose p = suspensionPose({S.x, S.hubHigh, S.z}, S.z, m);
            m.shockTopY += kShockBodyLength + 0.06f - glm::length(p.shockTop - p.shockLow);
        }
    }
    float sideX = 0.0f;
    for (const BodyPart& part : body.parts) {
        if (part.lods.empty()) continue;
        const BodyMesh& m = part.lods[0];
        VehiclePart vp{part.name, part.parent, {part.attach[0], part.attach[1], part.attach[2]}, part.deformable,
                       part.detachable, v.mesh.vertices.size(), 0};
        auto pos = [&](std::uint32_t i) { return glm::vec3(m.positions[3 * i], m.positions[3 * i + 1], m.positions[3 * i + 2]); };
        auto nrm = [&](std::uint32_t i) { return glm::vec3(m.normals[3 * i], m.normals[3 * i + 1], m.normals[3 * i + 2]); };
        for (const BodyMesh::Primitive& prim : m.primitives) {
            const Material& base = mats.at(std::size_t(prim.material));
            const bool head = body.materials[std::size_t(prim.material)].role == BodyRole::LightHead;
            for (std::uint32_t t = prim.firstIndex; t + 2 < prim.firstIndex + prim.indexCount + 0u; t += 3) {
                const std::uint32_t i[3] = {m.indices[t], m.indices[t + 1], m.indices[t + 2]};
                if (m.colors.empty()) {
                    v.mesh.addTri(pos(i[0]), pos(i[1]), pos(i[2]), nrm(i[0]), nrm(i[1]), nrm(i[2]), base);
                } else {
                    // Vertex colours tint the material: one material per corner, as addTri takes one per triangle.
                    const std::size_t first = v.mesh.vertices.size();
                    v.mesh.addTri(pos(i[0]), pos(i[1]), pos(i[2]), nrm(i[0]), nrm(i[1]), nrm(i[2]), base);
                    for (int k = 0; k < 3; ++k)
                        v.mesh.vertices[first + std::size_t(k)].color *=
                            glm::vec3(m.colors[3 * i[k]], m.colors[3 * i[k] + 1], m.colors[3 * i[k] + 2]);
                }
                for (int k = 0; k < 3; ++k) {
                    const glm::vec3 p = pos(i[k]);
                    if (head && p.x > 0.0f) lens += p, ++lensCount;
                    if (part.collides && std::abs(p.y - (restHub + r + 0.10f)) < 0.05f) sideX = std::max(sideX, std::abs(p.x));
                }
            }
        }
        vp.vertexCount = v.mesh.vertices.size() - vp.firstVertex;
        v.parts.push_back(vp);
    }
    for (const BodySpare& s : body.spares)
        v.spares.push_back({s.name, s.parent, {s.seat[0], s.seat[1], s.seat[2]},
                            {s.outward[0], s.outward[1], s.outward[2]}, true});
    if (lensCount > 0) v.headlightLens = lens / float(lensCount) + glm::vec3(0.0f, 0.0f, 0.01f);
    else v.headlightLens = {0.67f, restHub + 0.75f, body.boundsMax[2]};
    v.mudArch = {sweeps[0].z, sweeps[2].z, restHub + 0.15f, r + 0.11f};
    v.mudBody = {sideX > 0.0f ? sideX : body.boundsMax[0], body.boundsMax[2] - 0.15f, body.boundsMin[2] + 0.15f, 0.0f};
    return true;
}

// Head lamp lens (left) from the blueprint: centre of the left head lamp box on the front face.
glm::vec3 lensFromBlueprint(const VehicleBlueprint& bp, const VehicleConfig& cfg) {
    const BlueprintBody B(bp, cfg);
    for (const BlueprintLight& l : bp.lights)
        if (l.view == "front" && l.kind == "head" && l.box[0] + l.box[2] > 0.0f) {
            const float x = 0.5f * (l.box[0] + l.box[2]), y = B.bodyY(0.5f * (l.box[1] + l.box[3]));
            return {x, y, bpm::endZ(B, x, y, 1.0f) + 0.01f};
        }
    return {0.67f, 0.40f, B.bodyFront()};
}

// BodyMud shading regions from the body: arches around the rest hubs (tire radius + 0.11, centred 0.15 m above the
// hub like the arch openings), skin at the sill-height body side, bumper zones the last 0.15 m of each end.
void mudFromBlueprint(VehicleVisual& v, const VehicleBlueprint& bp, const VehicleConfig& cfg) {
    const BlueprintBody B(bp, cfg);
    const float r = 0.5f * (cfg.axles[0].wheelRadius + cfg.axles[1].wheelRadius);
    v.mudArch = {B.axleZ(0), B.axleZ(1), bp.restHubY + 0.15f, r + 0.11f};
    v.mudBody = {bpm::sideX(B, bp.restHubY + r + 0.10f, 0.0f), B.bodyFront() - 0.15f, B.bodyRear() + 0.15f, 0.0f};
}
} // namespace

const VehiclePart* VehicleVisual::find(const std::string& name) const {
    for (const VehiclePart& p : parts)
        if (p.name == name) return &p;
    return nullptr;
}

VehicleVisual buildVehicleVisual(const VehicleConfig& config) {
    VehicleVisual v;
    // The loader rejects unknown ids; an empty id falls back to trailforge_suv.
    const bool ev = config.visualModel == "trailforge_ev";
    wheelStyle(v, ev);
    if (!config.bodyModel.empty()) {
        std::string error;
        if (buildFromBody(v, config, error)) return v;
        std::fprintf(stderr, "TrailForge: %s; drawing the procedural body\n", error.c_str());
        v = VehicleVisual{};
        wheelStyle(v, ev);
    }
    if (config.visualModel == "trailforge_patrol") {
        // No traced blueprint: the kit body for both body shapes.
        v.wheel.beadlock = true;
        v.wheel.spokes = 10;
        kit::patrol::buildPatrol(v, config);
        return v;
    }
    if (config.bodyShape == "traced") {
        // The blueprints are embedded at build time.
        if (const VehicleBlueprint* bp = findVehicleBlueprint(ev ? "trailforge_ev" : "trailforge_suv")) {
            if (ev) buildTracedEv(v, config, *bp);
            else buildTracedSuv(v, config, *bp);
            v.headlightLens = lensFromBlueprint(*bp, config);
            mudFromBlueprint(v, *bp, config);
            return v;
        }
    }
    if (ev) kit::buildEv(v, config);
    else kit::buildSuv(v, config);
    return v;
}

style::WheelStyle axleWheelStyle(const VehicleVisual& visual, const AxleConfig& axle, TireSet set) {
    style::WheelStyle w = visual.wheel;
    w.rimFraction = axle.rimRatio;
    w.tread = set == TireSet::Road            ? style::Tread::Road
              : set == TireSet::Slick         ? style::Tread::Slick
              : axle.tread == "all_terrain" ? style::Tread::AllTerrain
                                            : style::Tread::MudTerrain;
    return w;
}

MeshData buildAxleWheelMesh(const VehicleVisual& visual, const AxleConfig& axle, TireSet set) {
    return style::buildWheel(axle.wheelRadius, axle.wheelWidth, axleWheelStyle(visual, axle, set));
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
    if (a.visualModel != b.visualModel || a.bodyShape != b.bodyShape || a.bodyModel != b.bodyModel || a.bodyPaint != b.bodyPaint || a.wheelBase != b.wheelBase || a.attachHeight != b.attachHeight || a.tireSet != b.tireSet) return false;
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

MeshData buildUnitSpringMesh(const Material& coil) {
    MeshData m;
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

SuspensionPose suspensionPose(const glm::vec3& hub, float attachZ, const SuspensionMounts& m) {
    SuspensionPose s;
    const float sx = hub.x >= 0.0f ? 1.0f : -1.0f;
    const float zc = attachZ;
    s.shockTop = {sx * m.shockTopX, m.shockTopY, zc + m.shockTopDZ};
    s.armPivotA = {sx * m.armPivotX, m.armPivotY, zc + m.armPivotHalfZ};
    s.armPivotB = {sx * m.armPivotX, m.armPivotY, zc - m.armPivotHalfZ};
    s.armOuter = {hub.x - sx * m.armOuterInset, hub.y - m.armOuterDrop, hub.z};
    const glm::vec3 pivotMid(sx * m.armPivotX, m.armPivotY, zc);
    const float span = std::abs(s.armOuter.x) - m.armPivotX;
    const float t = span > 1e-3f ? std::clamp((m.shockLowX - m.armPivotX) / span, 0.0f, 1.0f) : 1.0f;
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
