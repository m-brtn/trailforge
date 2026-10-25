#include "render/StyleKit.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>

namespace tf::style {
namespace {

constexpr float kPi = 3.14159265358979f;

void tri(MeshData& m, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& n,
         const Material& mat) {
    const glm::vec3 nn = glm::normalize(n);
    if (glm::dot(glm::cross(b - a, c - a), nn) >= 0.0f)
        m.addTri(a, b, c, nn, nn, nn, mat);
    else
        m.addTri(a, c, b, nn, nn, nn, mat);
}

glm::vec3 lin(float c) { return glm::vec3(std::pow(c, 2.2f)); }

} // namespace

// ---------------------------------------------------------------- materials

glm::vec3 srgb(float r, float g, float b) { return {std::pow(r, 2.2f), std::pow(g, 2.2f), std::pow(b, 2.2f)}; }

Material paint(const glm::vec3& c, float roughness, float mud) { return {MatId::Paint, c, roughness, 0.0f, mud}; }
Material plastic(const glm::vec3& c, float roughness, float mud) { return {MatId::Generic, c, roughness, 0.0f, mud}; }
Material metal(const glm::vec3& c, float roughness, float mud) { return {MatId::Generic, c, roughness, 1.0f, mud}; }
Material rubber(float mud) { return {MatId::Rubber, {0.032f, 0.032f, 0.034f}, 0.85f, 0.0f, mud}; }
Material glass() { return {MatId::Glass, {0.012f, 0.016f, 0.020f}, 0.04f, 0.0f, 0.15f}; }
Material led(MatId id, const glm::vec3& c) { return {id, c, 0.2f, 0.0f, 0.1f}; }

Livery oliveExpedition() {
    Livery l;
    l.body = paint(srgb(0.45f, 0.50f, 0.32f));
    l.accent = paint(srgb(0.96f, 0.50f, 0.13f), 0.34f);
    l.roof = paint(srgb(0.90f, 0.90f, 0.87f), 0.36f, 0.35f);
    l.stripe = paint(srgb(0.93f, 0.93f, 0.90f), 0.38f, 0.8f);
    l.trim = plastic(lin(0.17f), 0.78f);
    l.steel = {MatId::Generic, lin(0.16f), 0.48f, 0.35f, 1.0f};
    l.liner = plastic(lin(0.12f), 0.92f);
    l.underbody = plastic(lin(0.20f), 0.88f);
    l.gunmetal = {MatId::Generic, lin(0.30f), 0.38f, 0.8f, 0.8f};
    l.alu = metal(lin(0.80f), 0.32f);
    l.hooks = {MatId::Generic, srgb(0.88f, 0.18f, 0.10f), 0.42f, 0.0f, 0.5f};
    l.lens = {MatId::Generic, lin(0.10f), 0.12f, 0.6f, 0.2f};
    l.glass = glass();
    return l;
}

// ---------------------------------------------------------------- panels

void face(MeshData& m, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d,
          const glm::vec3& n, const Material& mat) {
    const glm::vec3 nn = glm::normalize(n);
    if (glm::dot(glm::cross(b - a, c - a) + glm::cross(c - a, d - a), nn) >= 0.0f) {
        m.addTri(a, b, c, nn, nn, nn, mat);
        m.addTri(a, c, d, nn, nn, nn, mat);
    } else {
        m.addTri(a, c, b, nn, nn, nn, mat);
        m.addTri(a, d, c, nn, nn, nn, mat);
    }
}

void panelGap(MeshData& m, const glm::vec3& a, const glm::vec3& b, const glm::vec3& n, float width,
              const Material& mat, float lift) {
    const glm::vec3 nn = glm::normalize(n);
    const glm::vec3 side = glm::normalize(glm::cross(nn, b - a)) * (0.5f * width);
    const glm::vec3 o = nn * lift;
    face(m, a - side + o, b - side + o, b + side + o, a + side + o, nn, mat);
}

void prism(MeshData& m, const std::vector<glm::vec2>& outline, const glm::vec3& origin, const glm::vec3& u,
           const glm::vec3& v, const glm::vec3& w, float w0, float w1, const Material& sides, const Material& cap0,
           const Material& cap1) {
    const std::size_t n = outline.size();
    if (n < 3) return;
    glm::vec2 c(0.0f);
    for (const glm::vec2& p : outline) c += p;
    c /= float(n);
    auto P = [&](const glm::vec2& p, float ww) { return origin + u * p.x + v * p.y + w * ww; };
    const glm::vec3 cMid = P(c, 0.5f * (w0 + w1));
    for (std::size_t k = 0; k < n; ++k) {
        const glm::vec2 a = outline[k], b = outline[(k + 1) % n];
        const glm::vec3 a0 = P(a, w0), b0 = P(b, w0), b1 = P(b, w1), a1 = P(a, w1);
        glm::vec3 sn = glm::cross(b0 - a0, a1 - a0);
        if (glm::dot(sn, 0.5f * (a0 + b1) - cMid) < 0.0f) sn = -sn;
        face(m, a0, b0, b1, a1, sn, sides);
        const glm::vec3 wn = glm::normalize(w) * (w1 > w0 ? 1.0f : -1.0f);
        tri(m, P(c, w0), a0, b0, -wn, cap0);
        tri(m, P(c, w1), a1, b1, wn, cap1);
    }
}

// ---------------------------------------------------------------- lofted surfaces

void skin(const std::vector<std::vector<glm::vec3>>& rows, bool closedRows, const std::function<SkinCell(int, int)>& cell,
          const std::function<glm::vec3(int, int, const glm::vec3&)>& hint, float smoothDeg) {
    const int R = int(rows.size());
    if (R < 2) return;
    const int N = int(rows[0].size());
    const int C = closedRows ? N : N - 1;
    if (C < 1) return;
    auto P = [&](int i, int k) { return rows[std::size_t(i)][std::size_t((k + N) % N)]; };
    std::vector<glm::vec3> qn(std::size_t((R - 1) * C), glm::vec3(0.0f));
    std::vector<char> valid(qn.size(), 0);
    for (int i = 0; i + 1 < R; ++i)
        for (int k = 0; k < C; ++k) {
            const glm::vec3 a = P(i, k), b = P(i, k + 1), c = P(i + 1, k + 1), d = P(i + 1, k);
            glm::vec3 n = glm::cross(c - a, d - b);
            const float len = glm::length(n);
            if (len < 1e-9f) continue;
            n /= len;
            if (glm::dot(n, hint(i, k, 0.25f * (a + b + c + d))) < 0.0f) n = -n;
            qn[std::size_t(i * C + k)] = n;
            valid[std::size_t(i * C + k)] = 1;
        }
    const float cosT = std::cos(glm::radians(smoothDeg));
    auto vnormal = [&](int vi, int vk, const glm::vec3& nq) {
        glm::vec3 sum(0.0f);
        for (int di = -1; di <= 0; ++di)
            for (int dk = -1; dk <= 0; ++dk) {
                const int i = vi + di;
                int k = vk + dk;
                if (i < 0 || i + 1 >= R) continue;
                if (closedRows) k = (k + C) % C;
                else if (k < 0 || k >= C) continue;
                const std::size_t q = std::size_t(i * C + k);
                if (valid[q] && glm::dot(qn[q], nq) >= cosT) sum += qn[q];
            }
        return glm::length(sum) > 1e-6f ? glm::normalize(sum) : nq;
    };
    for (int i = 0; i + 1 < R; ++i)
        for (int k = 0; k < C; ++k) {
            const std::size_t q = std::size_t(i * C + k);
            if (!valid[q]) continue;
            const SkinCell sc = cell(i, k);
            if (!sc.mesh || !sc.mat) continue;
            const glm::vec3& nq = qn[q];
            const glm::vec3 v[4] = {P(i, k), P(i, k + 1), P(i + 1, k + 1), P(i + 1, k)};
            const glm::vec3 vn[4] = {vnormal(i, k, nq), vnormal(i, k + 1, nq), vnormal(i + 1, k + 1, nq),
                                     vnormal(i + 1, k, nq)};
            static constexpr int kTris[2][3] = {{0, 1, 2}, {0, 2, 3}};
            for (const auto& t : kTris) {
                const glm::vec3 &a = v[t[0]], &b = v[t[1]], &c = v[t[2]];
                if (glm::length(glm::cross(b - a, c - a)) < 1e-10f) continue;
                if (glm::dot(glm::cross(b - a, c - a), nq) >= 0.0f)
                    sc.mesh->addTri(a, b, c, vn[t[0]], vn[t[1]], vn[t[2]], *sc.mat);
                else
                    sc.mesh->addTri(a, c, b, vn[t[0]], vn[t[2]], vn[t[1]], *sc.mat);
            }
        }
}

Footprint roundedRect(float hx, float zRear, float zFront, float r, float y, int cornerSegs,
                      const std::vector<float>& sideZ, const std::vector<float>& endX) {
    Footprint f;
    const float cx = hx - r, zf = zFront - r, zr = zRear + r;
    auto push = [&](float x, float z, const glm::vec3& n, Footprint::Seg s) {
        f.p.push_back({x, y, z});
        f.n.push_back(n);
        f.seg.push_back(s);
    };
    auto corner = [&](float ccx, float ccz, float a0, float a1, Footprint::Seg s) {
        for (int j = 1; j < cornerSegs; ++j) {
            const float a = a0 + (a1 - a0) * float(j) / float(cornerSegs);
            const glm::vec3 n(std::sin(a), 0.0f, std::cos(a));
            push(ccx + r * n.x, ccz + r * n.z, n, s);
        }
    };
    const float h = 0.5f * kPi;
    for (float e : endX) push(e * cx, zFront, {0, 0, 1}, Footprint::Front);
    corner(cx, zf, 0.0f, h, Footprint::CornerFL);
    push(hx, zf, {1, 0, 0}, Footprint::Left);
    for (float z : sideZ) push(hx, z, {1, 0, 0}, Footprint::Left);
    push(hx, zr, {1, 0, 0}, Footprint::Left);
    corner(cx, zr, h, 2 * h, Footprint::CornerRL);
    for (auto it = endX.rbegin(); it != endX.rend(); ++it) push(*it * cx, zRear, {0, 0, -1}, Footprint::Rear);
    corner(-cx, zr, 2 * h, 3 * h, Footprint::CornerRR);
    push(-hx, zr, {-1, 0, 0}, Footprint::Right);
    for (auto it = sideZ.rbegin(); it != sideZ.rend(); ++it) push(-hx, *it, {-1, 0, 0}, Footprint::Right);
    push(-hx, zf, {-1, 0, 0}, Footprint::Right);
    corner(-cx, zf, 3 * h, 4 * h, Footprint::CornerFR);
    return f;
}

// ---------------------------------------------------------------- lights

void ledRing(MeshData& m, const glm::vec3& center, const glm::vec3& n, const glm::vec3& right, float halfW, float halfH,
             float radius, float stripW, const Material& mat) {
    const glm::vec3 nn = glm::normalize(n);
    const glm::vec3 r = glm::normalize(right - nn * glm::dot(right, nn));
    const glm::vec3 up = glm::cross(nn, r);
    const glm::vec3 c = center + nn * 0.002f;
    constexpr int kArc = 5;
    std::vector<glm::vec2> path;
    const glm::vec2 corners[4] = {{halfW - radius, halfH - radius}, {-(halfW - radius), halfH - radius},
                                  {-(halfW - radius), -(halfH - radius)}, {halfW - radius, -(halfH - radius)}};
    for (int q = 0; q < 4; ++q)
        for (int i = 0; i <= kArc; ++i) {
            const float a = 0.5f * kPi * (float(q) + float(i) / kArc);
            path.push_back(corners[q] + glm::vec2(std::cos(a), std::sin(a)));
        }
    // path holds the corner centers plus unit directions; expand to inner/outer radius.
    std::vector<glm::vec3> inner, outer;
    for (int q = 0; q < 4; ++q)
        for (int i = 0; i <= kArc; ++i) {
            const glm::vec2 cc = corners[q];
            const glm::vec2 d = path[std::size_t(q * (kArc + 1) + i)] - cc;
            const glm::vec2 pi = cc + d * std::max(radius - 0.5f * stripW, 0.0f), po = cc + d * (radius + 0.5f * stripW);
            inner.push_back(c + r * pi.x + up * pi.y);
            outer.push_back(c + r * po.x + up * po.y);
        }
    for (std::size_t k = 0; k < inner.size(); ++k) {
        const std::size_t j = (k + 1) % inner.size();
        face(m, inner[k], outer[k], outer[j], inner[j], nn, mat);
    }
}

void ledStrip(MeshData& m, const glm::vec3& a, const glm::vec3& b, const glm::vec3& n, float width, const Material& mat,
              float lift) {
    panelGap(m, a, b, n, width, mat, lift);
}

void roundLamp(MeshData& m, const glm::vec3& center, const glm::vec3& axis, float radius, float depth,
               const Material& bezel, const Material& lens) {
    const glm::vec3 a = glm::normalize(axis);
    m.addCylinder(center - a * depth, center, radius, 20, bezel);
    m.addCylinder(center, center + a * 0.004f, radius * 0.80f, 20, lens);
}

// ---------------------------------------------------------------- accessories

void roofRack(MeshData& m, float halfW, float z0, float z1, float yBase, float height, const Material& steel) {
    const float y = yBase + height, r = 0.018f;
    for (float sx : {1.0f, -1.0f}) {
        const float x = sx * halfW;
        m.addCylinder({x, y, z0}, {x, y, z1}, r, 10, steel);
        // Gallery rail on posts.
        m.addCylinder({x, y + 0.07f, z0 + 0.10f}, {x, y + 0.07f, z1 - 0.10f}, 0.014f, 8, steel);
        for (float t : {0.0f, 0.33f, 0.66f, 1.0f}) {
            const float z = z0 + 0.10f + (z1 - z0 - 0.20f) * t;
            m.addCylinder({x, y, z}, {x, y + 0.07f, z}, 0.011f, 6, steel);
        }
        // Feet on the roof gutters.
        for (float t : {0.04f, 0.5f, 0.96f}) {
            const float z = z0 + (z1 - z0) * t;
            m.addBar({x, yBase, z}, {x, y, z}, 0.025f, 0.035f, {0, 0, 1}, steel, 0.006f);
        }
    }
    for (float z : {z0, z1}) m.addCylinder({-halfW, y, z}, {halfW, y, z}, r, 10, steel);
    for (float z = z0 + 0.12f; z < z1 - 0.06f; z += 0.12f)
        m.addBox({0.0f, y - 0.004f, z}, {halfW - 0.01f, 0.007f, 0.028f}, steel);
}

void lightBar(MeshData& m, const glm::vec3& c, float halfLen, const Material& housing, const Material& ledMat) {
    m.addBeveledBox(c, {halfLen, 0.034f, 0.032f}, 0.012f, housing);
    const int cells = std::max(2, int(std::round(halfLen / 0.075f)));
    const float step = 2.0f * (halfLen - 0.03f) / float(cells);
    for (int k = 0; k < cells; ++k) {
        const float x = -halfLen + 0.03f + step * (float(k) + 0.5f);
        m.addBox(c + glm::vec3(x, 0.0f, 0.033f), {0.5f * step - 0.008f, 0.020f, 0.003f}, ledMat);
    }
    for (float sx : {1.0f, -1.0f})
        m.addBar(c + glm::vec3(sx * (halfLen + 0.02f), 0.0f, 0.0f), c + glm::vec3(sx * (halfLen + 0.02f), -0.07f, -0.02f),
                 0.012f, 0.02f, {0, 0, 1}, housing);
}

void winch(MeshData& m, const glm::vec3& c, const Material& drum, const Material& fairlead, const Material& dark) {
    m.addCylinder(c - glm::vec3(0.15f, 0, 0), c + glm::vec3(0.15f, 0, 0), 0.055f, 16, drum);
    for (float s : {1.0f, -1.0f}) {
        m.addCylinder(c + glm::vec3(s * 0.15f, 0, 0), c + glm::vec3(s * 0.21f, 0, 0), 0.068f, 16, dark);
        m.addCylinder(c + glm::vec3(s * 0.21f, 0, 0), c + glm::vec3(s * 0.30f, 0, 0), 0.050f, 14, dark);
    }
    // Cable wraps (dark bands on the drum).
    for (int k = -3; k <= 3; ++k)
        m.addCylinder(c + glm::vec3(0.04f * k - 0.012f, 0, 0), c + glm::vec3(0.04f * k + 0.012f, 0, 0), 0.058f, 16, dark,
                      false);
    m.addBeveledBox(c + glm::vec3(0.0f, 0.0f, 0.075f), {0.105f, 0.045f, 0.012f}, 0.006f, fairlead);
    m.addBox(c + glm::vec3(0.0f, 0.0f, 0.088f), {0.060f, 0.010f, 0.002f}, dark);
    m.addBeveledBox(c + glm::vec3(0.0f, -0.035f, 0.11f), {0.012f, 0.025f, 0.010f}, 0.004f, fairlead); // hook
}

void shackle(MeshData& m, const glm::vec3& mount, const glm::vec3& forward, const Material& mat) {
    const glm::vec3 f = glm::normalize(forward);
    const glm::vec3 o = mount + f * 0.03f;
    const float r = 0.034f, t = 0.011f, drop = 0.045f;
    std::vector<glm::vec3> pts;
    pts.push_back(o + glm::vec3(-r, 0, 0));
    for (int i = 0; i <= 8; ++i) {
        const float a = kPi + kPi * float(i) / 8.0f;
        pts.push_back(o + glm::vec3(r * std::cos(a), -drop + r * std::sin(a), 0));
    }
    pts.push_back(o + glm::vec3(r, 0, 0));
    for (std::size_t k = 0; k + 1 < pts.size(); ++k) m.addCylinder(pts[k], pts[k + 1], t, 8, mat);
    m.addCylinder(o + glm::vec3(-r - 0.02f, 0, 0), o + glm::vec3(r + 0.02f, 0, 0), 0.008f, 8, mat);
}

void snorkel(MeshData& m, float sx, const glm::vec3& base, const glm::vec3& bend, const glm::vec3& top,
             const Material& body, const Material& dark) {
    (void)sx;
    m.addCylinder(base, bend, 0.045f, 14, body);
    m.addCylinder(bend, top, 0.045f, 14, body);
    m.addBeveledBox(top + glm::vec3(0.0f, 0.05f, 0.01f), {0.06f, 0.065f, 0.095f}, 0.02f, body);
    m.addBox(top + glm::vec3(0.0f, 0.05f, 0.106f), {0.045f, 0.045f, 0.002f}, dark);
    for (const glm::vec3& p : {glm::mix(base, bend, 0.2f), glm::mix(base, bend, 0.8f)})
        m.addCylinder(p - glm::vec3(0, 0.012f, 0), p + glm::vec3(0, 0.012f, 0), 0.049f, 14, dark);
}

void sweepProfile(MeshData& m, const std::vector<glm::vec3>& path, const std::vector<glm::vec3>& normals,
                  const std::vector<glm::vec2>& prof, const Material& mat) {
    const std::size_t R = prof.size(), n = path.size();
    if (R < 2 || n < 2) return;
    std::vector<std::vector<glm::vec3>> rows(R, std::vector<glm::vec3>(n));
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t k = 0; k < n; ++k) rows[i][k] = path[k] + normals[k] * prof[i].x + glm::vec3(0, prof[i].y, 0);
    skin(
        rows, false, [&](int, int) { return SkinCell{&m, &mat}; },
        [&](int i, int k, const glm::vec3&) {
            const glm::vec2 t = prof[std::size_t(i + 1)] - prof[std::size_t(i)];
            return normals[std::size_t(k)] * -t.y + glm::vec3(0, t.x, 0);
        });
    for (std::size_t k : {std::size_t(0), n - 1}) {
        const glm::vec3 nn = glm::normalize(k == 0 ? path[0] - path[1] : path[n - 1] - path[n - 2]);
        glm::vec3 c(0.0f);
        for (std::size_t i = 0; i < R; ++i) c += rows[i][k];
        c /= float(R);
        for (std::size_t i = 0; i + 1 < R; ++i) {
            const glm::vec3 &a = rows[i][k], &b = rows[i + 1][k];
            if (glm::dot(glm::cross(b - a, c - a), nn) >= 0.0f) m.addTri(a, b, c, nn, nn, nn, mat);
            else m.addTri(a, c, b, nn, nn, nn, mat);
        }
    }
}

void fenderFlare(MeshData& m, float sx, const std::vector<glm::vec2>& arch, float band, float xBody, float proud,
                 const Material& mat, const Material& liner) {
    const std::size_t n = arch.size();
    if (n < 2) return;
    // Outward normal (away from the wheel) of segment k in (z, y).
    auto segN = [&](std::size_t k) {
        const glm::vec2 d = glm::normalize(arch[k + 1] - arch[k]);
        return glm::vec2(-d.y, d.x);
    };
    // Offset direction per arch point (mitred); the end points move horizontally so the flare ends flat on the
    // rocker line.
    std::vector<glm::vec2> dir(n);
    for (std::size_t k = 0; k < n; ++k) {
        if (k == 0 || k + 1 == n) {
            dir[k] = glm::vec2(k == 0 ? -1.0f : 1.0f, 0.0f) / std::max(std::abs(segN(k == 0 ? 0 : n - 2).x), 0.3f);
        } else {
            const glm::vec2 n0 = segN(k - 1), nn = glm::normalize(n0 + segN(k));
            dir[k] = nn / std::max(glm::dot(nn, n0), 0.3f);
        }
    }
    // Cross-section (distance from the arch edge, height above the body side): liner return inside the opening,
    // rounded lip, slightly crowned top, chamfer down onto the body.
    const std::vector<glm::vec2> prof{{0.0f, -0.02f},
                                      {0.0f, proud - 0.022f},
                                      {0.010f, proud - 0.006f},
                                      {0.028f, proud},
                                      {0.55f * band, proud - 0.010f},
                                      {band - 0.030f, 0.55f * proud},
                                      {band - 0.006f, 0.12f * proud},
                                      {band, 0.0f}};
    const std::size_t R = prof.size();
    std::vector<std::vector<glm::vec3>> rows(R, std::vector<glm::vec3>(n));
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t k = 0; k < n; ++k) {
            const glm::vec2 zy = arch[k] + dir[k] * prof[i].x;
            rows[i][k] = {sx * (xBody + prof[i].y), zy.y, zy.x};
        }
    skin(
        rows, false, [&](int i, int) { return SkinCell{&m, i == 0 ? &liner : &mat}; },
        [&](int i, int k, const glm::vec3&) {
            const glm::vec2 t = prof[std::size_t(i + 1)] - prof[std::size_t(i)];
            const glm::vec2 o = dir[std::size_t(k)];
            // Perpendicular of the profile edge pointing out of the flare (toward the wheel on the lip).
            return glm::vec3(sx * t.x, 0.0f, 0.0f) - glm::vec3(0.0f, o.y, o.x) * t.y;
        });
    // End caps on the rocker line.
    for (std::size_t k : {std::size_t(0), n - 1}) {
        glm::vec3 c(0.0f);
        for (std::size_t i = 0; i < R; ++i) c += rows[i][k];
        c /= float(R);
        for (std::size_t i = 0; i + 1 < R; ++i) {
            const glm::vec3 &a = rows[i][k], &b = rows[i + 1][k];
            const glm::vec3 nn(0, -1, 0);
            if (glm::dot(glm::cross(b - a, c - a), nn) >= 0.0f) m.addTri(a, b, c, nn, nn, nn, mat);
            else m.addTri(a, c, b, nn, nn, nn, mat);
        }
    }
}


void jerryCan(MeshData& m, const glm::vec3& c, const glm::vec3& outward, const Material& body, const Material& dark) {
    const bool alongZ = std::abs(outward.z) > 0.5f;
    const glm::vec3 half = alongZ ? glm::vec3(0.095f, 0.19f, 0.062f) : glm::vec3(0.062f, 0.19f, 0.095f);
    m.addBeveledBox(c, half, 0.018f, body);
    const glm::vec3 o = glm::normalize(outward);
    const glm::vec3 across = alongZ ? glm::vec3(1, 0, 0) : glm::vec3(0, 0, 1);
    const glm::vec3 fc = c + o * (alongZ ? half.z : half.x);
    const float w = alongZ ? half.x : half.z;
    // Embossed X on the outer face.
    for (float s : {1.0f, -1.0f})
        m.addBar(fc - across * (w * 0.70f) + glm::vec3(0, -s * half.y * 0.70f, 0),
                 fc + across * (w * 0.70f) + glm::vec3(0, s * half.y * 0.70f, 0), 0.006f, 0.004f, o, body);
    // Three handles and the spout.
    for (int k = -1; k <= 1; ++k)
        m.addBar(c + across * (0.035f * k) + glm::vec3(0, half.y, 0), c + across * (0.035f * k) + glm::vec3(0, half.y + 0.03f, 0),
                 0.007f, 0.016f, across, body);
    m.addCylinder(c + across * (w * 0.55f) + glm::vec3(0, half.y - 0.01f, 0),
                  c + across * (w * 0.75f) + glm::vec3(0, half.y + 0.04f, 0), 0.016f, 10, dark);
    // Strap of the holder.
    m.addBox(c + glm::vec3(0, -0.02f, 0), {half.x + 0.006f, 0.012f, half.z + 0.006f}, dark);
}

void cargoCase(MeshData& m, const glm::vec3& c, const glm::vec3& half, const Material& body, const Material& latch) {
    m.addBeveledBox(c, half, 0.016f, body);
    m.addBox(c + glm::vec3(0, half.y * 0.25f, 0), {half.x + 0.003f, 0.004f, half.z + 0.003f}, latch);
    for (float sz : {1.0f, -1.0f})
        for (float sx : {0.55f, -0.55f})
            m.addBeveledBox(c + glm::vec3(sx * half.x, half.y * 0.25f, sz * (half.z + 0.006f)), {0.022f, 0.026f, 0.006f},
                            0.003f, latch);
}

void ladder(MeshData& m, const glm::vec3& bottom, const glm::vec3& top, float halfW, const glm::vec3& side,
            const Material& mat) {
    const glm::vec3 s = glm::normalize(side) * halfW;
    const glm::vec3 toBody = glm::normalize(glm::cross(glm::normalize(side), glm::normalize(top - bottom)));
    for (float k : {1.0f, -1.0f}) {
        m.addCylinder(bottom + s * k, top + s * k, 0.013f, 8, mat);
        for (const glm::vec3& p : {bottom, top}) m.addCylinder(p + s * k, p + s * k + toBody * 0.05f, 0.010f, 6, mat);
    }
    const float len = glm::length(top - bottom);
    const int rungs = std::max(2, int(len / 0.10f));
    for (int i = 1; i < rungs; ++i) {
        const glm::vec3 p = glm::mix(bottom, top, float(i) / float(rungs));
        m.addCylinder(p - s, p + s, 0.009f, 6, mat);
    }
}

void rockSlider(MeshData& m, float sx, float x, float y, float z0, float z1, const Material& tube, const Material& caps,
                const Material& brackets) {
    m.addCylinder({sx * x, y, z0}, {sx * x, y, z1}, 0.038f, 14, tube);
    m.addCylinder({sx * x, y, z1}, {sx * (x - 0.05f), y + 0.07f, z1 + 0.07f}, 0.036f, 12, caps);
    m.addCylinder({sx * x, y, z0}, {sx * (x - 0.05f), y + 0.07f, z0 - 0.07f}, 0.036f, 12, caps);
    for (float t : {0.15f, 0.5f, 0.85f}) {
        const float z = z0 + (z1 - z0) * t;
        m.addBar({sx * (x - 0.16f), y, z}, {sx * x, y, z}, 0.02f, 0.03f, {0, 1, 0}, brackets);
    }
}

void mirror(MeshData& m, float sx, const glm::vec3& base, const Material& housing, const Material& glassMat) {
    const glm::vec3 arm = base + glm::vec3(sx * 0.10f, 0.03f, -0.03f);
    m.addBar(base, arm, 0.014f, 0.014f, {0, 1, 0}, housing);
    const glm::vec3 h = arm + glm::vec3(sx * 0.055f, 0.065f, 0.0f);
    m.addBeveledBox(h, {0.045f, 0.09f, 0.045f}, 0.016f, housing);
    m.addBox(h + glm::vec3(0, 0, -0.046f), {0.036f, 0.078f, 0.002f}, glassMat);
}

void spareCarrier(MeshData& m, const glm::vec3& c, const glm::vec3& outward, float radius, const Material& steel) {
    const glm::vec3 o = glm::normalize(outward);
    m.addCylinder(c, c + o * 0.025f, 0.11f, 18, steel);
    const glm::vec3 a = std::abs(o.y) > 0.5f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
    const glm::vec3 b = glm::cross(o, a);
    for (const glm::vec3& d : {a, b})
        m.addBar(c - d * (radius * 0.75f), c + d * (radius * 0.75f), 0.022f, 0.012f, o, steel, 0.004f);
}

// ---------------------------------------------------------------- wheels

MeshData buildWheel(float radius, float width, const WheelStyle& st) {
    MeshData m;
    const float hw = 0.5f * width;
    const float rr = st.rimFraction * radius; // rim radius
    const bool mud = st.tread == Tread::MudTerrain;
    const float lug = mud ? 0.026f : 0.018f;  // tread depth
    const float rt = radius - lug;            // tread base (lugs reach `radius`)
    const Material& tire = st.tire;
    const Material dark{MatId::Generic, {0.05f, 0.05f, 0.05f}, 0.6f, 0.6f, 0.6f};
    const Material disc{MatId::Generic, {0.35f, 0.34f, 0.33f}, 0.45f, 1.0f, 0.3f};
    constexpr int kSeg = 56;

    // ---- Carcass with bulging sidewalls (clockwise profile in (x, r): left bead -> over the tread -> right bead).
    // Everything stays within |x| <= hw and r <= radius: the visual tire is exactly the physics wheel.
    const float rm = 0.5f * (rr + rt);
    m.addLathe({{-hw * 0.77f, rr}, {-hw * 0.89f, rr + 0.03f}, {-hw * 0.98f, rm}, {-hw * 0.96f, rt - 0.05f},
                {-hw * 0.89f, rt - 0.016f}, {-hw * 0.77f, rt}, {hw * 0.77f, rt}, {hw * 0.89f, rt - 0.016f},
                {hw * 0.96f, rt - 0.05f}, {hw * 0.98f, rm}, {hw * 0.89f, rr + 0.03f}, {hw * 0.77f, rr}},
               kSeg, tire);
    for (float s : {-1.0f, 1.0f})
        m.addLathe(s > 0 ? std::vector<glm::vec2>{{hw * 0.975f, rm + 0.012f}, {hw, rm + 0.006f},
                                                  {hw, rm - 0.006f}, {hw * 0.975f, rm - 0.012f}}
                         : std::vector<glm::vec2>{{-hw * 0.975f, rm - 0.012f}, {-hw, rm - 0.006f},
                                                  {-hw, rm + 0.006f}, {-hw * 0.975f, rm + 0.012f}},
                   kSeg, tire);

    // ---- Tread: staggered chevron blocks; mud terrain adds shoulder lugs that wrap onto the sidewall.
    const int pitches = mud ? 24 : 34;
    const float pitchLen = 2.0f * kPi * radius / float(pitches);
    const float yaw = mud ? 0.24f : 0.12f, bx = hw * (mud ? 0.36f : 0.27f), bz = pitchLen * (mud ? 0.31f : 0.34f);
    // Flat block tops: the corners (tangential reach tz) touch `radius`.
    const float tz = bx * std::sin(yaw) + bz * std::cos(yaw);
    const float top = std::sqrt(radius * radius - tz * tz);
    const glm::mat4 base = m.xf;
    for (int p = 0; p < pitches; ++p) {
        for (int side = 0; side < 2; ++side) {
            const float s = side == 0 ? -1.0f : 1.0f;
            const float th = 2.0f * kPi * (float(p) + 0.5f * float(side)) / float(pitches);
            const glm::mat4 r = glm::rotate(glm::mat4(1.0f), th, glm::vec3(1, 0, 0)); // around the axle
            // Local frame after rotation: +Y radial, +Z tangential, +X axial.
            glm::mat4 blk = glm::translate(r, glm::vec3(s * hw * (mud ? 0.41f : 0.45f), 0.5f * (top + rt), 0.0f));
            blk = glm::rotate(blk, s * yaw, glm::vec3(0, 1, 0));
            m.xf = base * blk;
            m.addBeveledBox(glm::vec3(0), {bx, 0.5f * (top - rt), bz}, 0.005f, tire);
            if (mud && (p + side) % 2 == 0) {
                const glm::mat4 sh = glm::translate(r, glm::vec3(s * hw * 0.90f, rt - 0.03f, 0.0f));
                m.xf = base * sh;
                m.addBeveledBox(glm::vec3(0), {hw * 0.08f, 0.04f, pitchLen * 0.27f}, 0.006f, tire);
            }
        }
    }
    m.xf = base;

    // ---- Deep-dish rim: barrel, outer lip (or beadlock ring with bolts), spokes recessed toward the hub
    m.addLathe({{hw * 0.75f, rr - 0.03f}, {-hw * 0.76f, rr - 0.03f}}, 40, dark); // barrel (faces the axle)
    const float lipOut = st.beadlock ? hw * 0.92f : hw * 0.85f;
    const Material& lipMat = st.beadlock ? st.ring : st.rim;
    m.addLathe({{hw * 0.75f, rr - 0.03f}, {hw * 0.75f, rr + 0.012f}, {hw * 0.75f, rr + 0.012f}, {lipOut, rr + 0.012f},
                {lipOut, rr + 0.012f}, {lipOut, rr - 0.03f}, {lipOut, rr - 0.03f}, {hw * 0.75f, rr - 0.03f}},
               48, lipMat);
    if (st.beadlock) {
        constexpr int kBolts = 18;
        for (int i = 0; i < kBolts; ++i) {
            const float a = 2.0f * kPi * float(i) / float(kBolts);
            const glm::vec3 p(0.0f, (rr - 0.009f) * std::cos(a), (rr - 0.009f) * std::sin(a));
            m.addCylinder(p + glm::vec3(hw * 0.92f, 0, 0), p + glm::vec3(hw * 0.95f, 0, 0), 0.0055f, 6, dark);
        }
    }
    const int spokes = std::max(3, st.spokes);
    const float spokeW = std::clamp(0.15f / float(spokes), 0.012f, 0.026f);
    for (int i = 0; i < spokes; ++i) {
        const float a = 2.0f * kPi * float(i) / float(spokes);
        const glm::vec3 dir(0.0f, std::cos(a), std::sin(a));
        const glm::vec3 inner = glm::vec3(hw * 0.38f, 0, 0) + dir * 0.075f, outer = glm::vec3(hw * 0.75f, 0, 0) + dir * (rr - 0.03f);
        m.addBar(inner, outer, spokeW, 0.016f, glm::vec3(1, 0, 0), st.rim, 0.006f);
        // Window ribs: thin raised edge on each spoke catches the light.
        m.addBar(inner + glm::vec3(hw * 0.09f, 0, 0), outer + glm::vec3(hw * 0.05f, 0, 0), spokeW * 0.35f,
                 0.006f, glm::vec3(1, 0, 0), st.ring, 0.002f);
    }
    m.addCylinder({hw * 0.25f, 0, 0}, {hw * 0.43f, 0, 0}, 0.090f, 28, st.rim);
    for (int i = 0; i < 6; ++i) {
        const float a = 2.0f * kPi * (float(i) + 0.5f) / 6.0f;
        const glm::vec3 p(0.0f, 0.062f * std::cos(a), 0.062f * std::sin(a));
        m.addCylinder(p + glm::vec3(hw * 0.43f, 0, 0), p + glm::vec3(hw * 0.51f, 0, 0), 0.011f, 6, st.ring);
    }
    m.addCylinder({hw * 0.43f, 0, 0}, {hw * 0.55f, 0, 0}, 0.040f, 20, dark);
    // Brake disc (spins with the wheel).
    m.addCylinder({-hw * 0.30f, 0, 0}, {-hw * 0.12f, 0, 0}, 0.165f, 28, disc);
    return m;
}

} // namespace tf::style
