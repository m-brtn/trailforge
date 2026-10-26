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

private:
    std::vector<VehiclePart> m_parts;
    std::deque<MeshData> m_meshes;
};

const char* sideName(float sx) { return sx > 0 ? "left" : "right"; }
std::string sided(const char* base, float sx) { return std::string(base) + "_" + sideName(sx); }

// ================================================================ blueprint bodies
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

// ================================================================ trailforge_suv
// Gasoline expedition SUV (Cars.png car #1): traced boxy five-door body, olive paint with orange accents, white
// roof, black flares and steel bumpers; winch, tow shackles, skid plate, roof rails with a light bar and strapped
// cargo, spare wheel and jerry cans on the tailgate. The concept has no snorkel and no ladder.
void buildSuv(VehicleVisual& v, const VehicleConfig& cfg, const VehicleBlueprint& bp) {
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

// ================================================================ trailforge_ev
// Dual-motor electric SUV (Cars.png car #2): traced body, white / light grey paint, blue accents, black cladding,
// closed front panel with a full-width LED bar, full-width tail light, low roof rails with a light bar and one case,
// blue tow hooks. No spare (the concept has none), no snorkel. Chassis: frame rails, floor battery pack, one motor
// per axle.
void buildEv(VehicleVisual& v, const VehicleConfig& cfg, const VehicleBlueprint& bp) {
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
    v.mudArch = {sweeps[0].z, -sweeps[2].z, restHub + 0.15f, r + 0.11f};
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
    v.mudArch = {B.axleZ(0), -B.axleZ(1), bp.restHubY + 0.15f, r + 0.11f};
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
    const VehicleBlueprint* bp = findVehicleBlueprint(ev ? "trailforge_ev" : "trailforge_suv");
    if (!bp) return v; // the blueprints are embedded at build time
    if (ev) buildEv(v, config, *bp);
    else buildSuv(v, config, *bp);
    v.headlightLens = lensFromBlueprint(*bp, config);
    mudFromBlueprint(v, *bp, config);
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
    if (a.visualModel != b.visualModel || a.bodyModel != b.bodyModel || a.bodyPaint != b.bodyPaint || a.wheelBase != b.wheelBase || a.attachHeight != b.attachHeight) return false;
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
