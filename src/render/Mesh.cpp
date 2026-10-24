#include "render/Mesh.h"

#include "track/SurfaceProperties.h"
#include "track/TestTrack.h"

#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace tf {
namespace {

constexpr float kPi = 3.14159265358979f;

glm::vec4 packMaterial(const Material& m) {
    return {float(int(m.id)), m.roughness, m.metallic, m.mud};
}

glm::vec3 safeNormalize(const glm::vec3& v, const glm::vec3& fallback = {0, 1, 0}) {
    const float l = glm::length(v);
    return l > 1e-12f ? v / l : fallback;
}

// Any vector perpendicular to d.
glm::vec3 perpendicular(const glm::vec3& d) {
    return std::abs(d.y) < 0.9f ? safeNormalize(glm::cross(d, glm::vec3(0, 1, 0)))
                                : safeNormalize(glm::cross(d, glm::vec3(1, 0, 0)));
}

} // namespace

void MeshData::addTri(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& na,
                      const glm::vec3& nb, const glm::vec3& nc, const Material& m) {
    const glm::mat3 nm = glm::transpose(glm::inverse(glm::mat3(xf)));
    const glm::vec4 mat = packMaterial(m);
    auto tp = [&](const glm::vec3& p) { return glm::vec3(xf * glm::vec4(p, 1.0f)); };
    vertices.push_back({tp(a), safeNormalize(nm * na), m.color, mat});
    vertices.push_back({tp(b), safeNormalize(nm * nb), m.color, mat});
    vertices.push_back({tp(c), safeNormalize(nm * nc), m.color, mat});
}

void MeshData::addTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const Material& m) {
    const glm::vec3 n = safeNormalize(glm::cross(b - a, c - a));
    addTri(a, b, c, n, n, n, m);
}

void MeshData::addTriangleOutward(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& inside,
                                  const Material& m) {
    const glm::vec3 n = glm::cross(b - a, c - a);
    if (glm::dot(n, (a + b + c) / 3.0f - inside) >= 0.0f) addTriangle(a, b, c, m);
    else addTriangle(a, c, b, m);
}

void MeshData::addQuad(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d,
                       const Material& m) {
    addTriangle(a, b, c, m);
    addTriangle(a, c, d, m);
}

void MeshData::addQuadOutward(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d,
                              const glm::vec3& inside, const Material& m) {
    addTriangleOutward(a, b, c, inside, m);
    addTriangleOutward(a, c, d, inside, m);
}

void MeshData::addBox(const glm::vec3& c, const glm::vec3& h, const Material& m) {
    auto p = [&](float x, float y, float z) { return c + glm::vec3(x * h.x, y * h.y, z * h.z); };
    addQuad(p(-1, 1, -1), p(-1, 1, 1), p(1, 1, 1), p(1, 1, -1), m);      // +Y
    addQuad(p(-1, -1, -1), p(1, -1, -1), p(1, -1, 1), p(-1, -1, 1), m);  // -Y
    addQuad(p(1, -1, -1), p(1, 1, -1), p(1, 1, 1), p(1, -1, 1), m);      // +X
    addQuad(p(-1, -1, 1), p(-1, 1, 1), p(-1, 1, -1), p(-1, -1, -1), m);  // -X
    addQuad(p(1, -1, 1), p(1, 1, 1), p(-1, 1, 1), p(-1, -1, 1), m);      // +Z
    addQuad(p(-1, -1, -1), p(-1, 1, -1), p(1, 1, -1), p(1, -1, -1), m);  // -Z
}

void MeshData::addBeveledBox(const glm::vec3& c, const glm::vec3& h, float bevel, const Material& m) {
    const float b = std::min(bevel, 0.45f * std::min(h.x, std::min(h.y, h.z)));
    if (b <= 1e-5f) {
        addBox(c, h, m);
        return;
    }
    const glm::vec3 in = h - glm::vec3(b);
    // Point on the face with normal axis `ax` (sign s), at in-plane coordinates u, v (in units of the inner box).
    auto P = [&](int ax, float s, float u, float v) {
        glm::vec3 p;
        const int a1 = (ax + 1) % 3, a2 = (ax + 2) % 3;
        p[ax] = s * h[ax];
        p[a1] = u * in[a1];
        p[a2] = v * in[a2];
        return c + p;
    };
    // 6 main faces.
    for (int ax = 0; ax < 3; ++ax)
        for (float s : {-1.0f, 1.0f})
            addQuadOutward(P(ax, s, -1, -1), P(ax, s, 1, -1), P(ax, s, 1, 1), P(ax, s, -1, 1), c, m);
    // 12 edge chamfers: between face (ax, s) and face (ax2, s2).
    for (int ax = 0; ax < 3; ++ax) {
        const int a1 = (ax + 1) % 3, a2 = (ax + 2) % 3;
        for (float s : {-1.0f, 1.0f})
            for (float s1 : {-1.0f, 1.0f}) {
                // Edge along a2 between faces (ax, s) and (a1, s1).
                auto pa = [&](float t) {
                    glm::vec3 p;
                    p[ax] = s * h[ax];
                    p[a1] = s1 * in[a1];
                    p[a2] = t * in[a2];
                    return c + p;
                };
                auto pb = [&](float t) {
                    glm::vec3 p;
                    p[ax] = s * in[ax];
                    p[a1] = s1 * h[a1];
                    p[a2] = t * in[a2];
                    return c + p;
                };
                addQuadOutward(pa(-1), pa(1), pb(1), pb(-1), c, m);
            }
    }
    // 8 corner triangles.
    for (float sx : {-1.0f, 1.0f})
        for (float sy : {-1.0f, 1.0f})
            for (float sz : {-1.0f, 1.0f}) {
                const glm::vec3 a = c + glm::vec3(sx * h.x, sy * in.y, sz * in.z);
                const glm::vec3 bb = c + glm::vec3(sx * in.x, sy * h.y, sz * in.z);
                const glm::vec3 cc = c + glm::vec3(sx * in.x, sy * in.y, sz * h.z);
                addTriangleOutward(a, bb, cc, c, m);
            }
}

void MeshData::addBar(const glm::vec3& p0, const glm::vec3& p1, float halfW, float halfH, const glm::vec3& up,
                      const Material& m, float bevel) {
    const glm::vec3 d = p1 - p0;
    const float len = glm::length(d);
    if (len < 1e-6f) return;
    const glm::vec3 z = d / len;
    glm::vec3 x = glm::cross(up, z);
    if (glm::length(x) < 1e-4f) x = perpendicular(z);
    x = glm::normalize(x);
    const glm::vec3 y = glm::cross(z, x);
    glm::mat4 local(1.0f);
    local[0] = glm::vec4(x, 0);
    local[1] = glm::vec4(y, 0);
    local[2] = glm::vec4(z, 0);
    local[3] = glm::vec4(0.5f * (p0 + p1), 1);
    const glm::mat4 saved = xf;
    xf = saved * local;
    addBeveledBox(glm::vec3(0), glm::vec3(halfW, halfH, 0.5f * len), bevel, m);
    xf = saved;
}

void MeshData::addCylinder(const glm::vec3& p0, const glm::vec3& p1, float radius, int segments, const Material& m,
                           bool caps) {
    const glm::vec3 d = p1 - p0;
    const float len = glm::length(d);
    if (len < 1e-6f) return;
    const glm::vec3 axis = d / len;
    const glm::vec3 u = perpendicular(axis);
    const glm::vec3 v = glm::cross(axis, u);
    for (int i = 0; i < segments; ++i) {
        const float a0 = 2.0f * kPi * float(i) / float(segments);
        const float a1 = 2.0f * kPi * float(i + 1) / float(segments);
        const glm::vec3 n0 = u * std::cos(a0) + v * std::sin(a0);
        const glm::vec3 n1 = u * std::cos(a1) + v * std::sin(a1);
        const glm::vec3 b0 = p0 + n0 * radius, b1 = p0 + n1 * radius, t0 = p1 + n0 * radius, t1 = p1 + n1 * radius;
        addTri(b0, b1, t1, n0, n1, n1, m);
        addTri(b0, t1, t0, n0, n1, n0, m);
        if (caps) {
            addTri(p1, t0, t1, axis, axis, axis, m);
            addTri(p0, b1, b0, -axis, -axis, -axis, m);
        }
    }
}

void MeshData::addFrustum(float hx0, float y0, float z0a, float z0b, float hx1, float y1, float z1a, float z1b,
                          const Material& sides, const Material& front, const Material& back, const Material& top) {
    const glm::vec3 inside(0.0f, 0.5f * (y0 + y1), 0.25f * (z0a + z0b + z1a + z1b));
    const glm::vec3 b00(-hx0, y0, z0a), b10(hx0, y0, z0a), b11(hx0, y0, z0b), b01(-hx0, y0, z0b);
    const glm::vec3 t00(-hx1, y1, z1a), t10(hx1, y1, z1a), t11(hx1, y1, z1b), t01(-hx1, y1, z1b);
    addQuadOutward(b10, b11, t11, t10, inside, sides);  // +X
    addQuadOutward(b00, b01, t01, t00, inside, sides);  // -X
    addQuadOutward(b01, b11, t11, t01, inside, front);  // +Z (z0b/z1b side)
    addQuadOutward(b00, b10, t10, t00, inside, back);   // -Z
    addQuadOutward(t00, t10, t11, t01, inside, top);
}

void MeshData::addLathe(const std::vector<glm::vec2>& profile, int segments, const Material& m) {
    // Profile normals in the (x, r) plane from neighboring points (smooth).
    const std::size_t n = profile.size();
    if (n < 2) return;
    std::vector<glm::vec2> pn(n);
    for (std::size_t i = 0; i < n; ++i) {
        const glm::vec2 a = profile[i > 0 ? i - 1 : i];
        const glm::vec2 b = profile[i + 1 < n ? i + 1 : i];
        const glm::vec2 t = glm::normalize(b - a);
        pn[i] = glm::vec2(-t.y, t.x); // rotate tangent: profile goes around the outside CCW
    }
    for (int s = 0; s < segments; ++s) {
        const float a0 = 2.0f * kPi * float(s) / float(segments);
        const float a1 = 2.0f * kPi * float(s + 1) / float(segments);
        const float c0 = std::cos(a0), s0 = std::sin(a0), c1 = std::cos(a1), s1 = std::sin(a1);
        for (std::size_t i = 0; i + 1 < n; ++i) {
            auto P = [&](std::size_t k, float c, float sn) {
                return glm::vec3(profile[k].x, profile[k].y * c, profile[k].y * sn);
            };
            auto N = [&](std::size_t k, float c, float sn) { return glm::vec3(pn[k].x, pn[k].y * c, pn[k].y * sn); };
            const glm::vec3 p00 = P(i, c0, s0), p01 = P(i, c1, s1), p10 = P(i + 1, c0, s0), p11 = P(i + 1, c1, s1);
            const glm::vec3 n00 = N(i, c0, s0), n01 = N(i, c1, s1), n10 = N(i + 1, c0, s0), n11 = N(i + 1, c1, s1);
            // Orientation: make the triangle face along the averaged profile normal.
            const glm::vec3 fn = glm::cross(p10 - p00, p11 - p00);
            if (glm::dot(fn, n00 + n11) >= 0.0f) {
                addTri(p00, p10, p11, n00, n10, n11, m);
                addTri(p00, p11, p01, n00, n11, n01, m);
            } else {
                addTri(p00, p11, p10, n00, n11, n10, m);
                addTri(p00, p01, p11, n00, n01, n11, m);
            }
        }
    }
}

void MeshData::append(const MeshData& other) {
    vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());
}

MeshData buildTrackMesh(const TrackGeometry& g, const SurfaceTable& surfaces) {
    // Smooth normals: average face normals at shared positions, but only across faces within
    // ~50 degrees of each other (box edges of steps and walls stay sharp, terrain and rocks are smooth).
    const std::size_t triCount = g.triangleCount();
    std::vector<glm::vec3> faceN(triCount);
    auto pos = [&](std::size_t t, int k) {
        const JPH::Float3& f = g.positions[3 * t + k];
        return glm::vec3(f.x, f.y, f.z);
    };
    for (std::size_t t = 0; t < triCount; ++t)
        faceN[t] = glm::cross(pos(t, 1) - pos(t, 0), pos(t, 2) - pos(t, 0)); // area-weighted

    auto key = [](const glm::vec3& p) {
        const auto q = [](float v) { return std::uint64_t(std::int64_t(std::lround(v * 1000.0f)) & 0x1fffff); };
        return q(p.x) | (q(p.y) << 21) | (q(p.z) << 42);
    };
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> shared;
    shared.reserve(triCount * 2);
    for (std::size_t t = 0; t < triCount; ++t)
        for (int k = 0; k < 3; ++k) shared[key(pos(t, k))].push_back(std::uint32_t(t));

    MeshData m;
    m.vertices.reserve(triCount * 3);
    const float cosCrease = std::cos(50.0f * kPi / 180.0f);
    for (std::size_t t = 0; t < triCount; ++t) {
        const glm::vec3 nf = safeNormalize(faceN[t]);
        glm::vec3 vn[3];
        for (int k = 0; k < 3; ++k) {
            glm::vec3 sum(0.0f);
            for (std::uint32_t o : shared[key(pos(t, k))]) {
                const glm::vec3 no = safeNormalize(faceN[o]);
                if (glm::dot(no, nf) >= cosCrease) sum += faceN[o];
            }
            vn[k] = safeNormalize(sum, nf);
        }
        const SurfaceId sid = g.surfaces[t];
        const auto& s = surfaces.get(sid);
        Material mat;
        mat.color = glm::vec3(s.color[0], s.color[1], s.color[2]);
        mat.roughness = 0.9f;
        switch (sid) {
        case SurfaceId::Concrete: mat.id = MatId::Concrete; break;
        case SurfaceId::PackedDirt: mat.id = MatId::Dirt; break;
        case SurfaceId::Rock: mat.id = MatId::Rock; break;
        case SurfaceId::LowGrip: mat.id = MatId::LowGrip; break;
        case SurfaceId::Mud: mat.id = MatId::Mud; break;
        case SurfaceId::Grass: mat.id = MatId::Grass; break;
        default: mat.id = MatId::Dirt; break;
        }
        m.addTri(pos(t, 0), pos(t, 1), pos(t, 2), vn[0], vn[1], vn[2], mat);
    }
    return m;
}

} // namespace tf
