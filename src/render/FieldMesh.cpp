#include "render/FieldMesh.h"

#include <algorithm>
#include <cmath>

namespace tf {

FieldMesh polygonizeField(const std::function<float(const glm::vec3&)>& field, const glm::vec3& lo, const glm::vec3& hi,
                          float h) {
    const int nx = int(std::ceil((hi.x - lo.x) / h)) + 1, ny = int(std::ceil((hi.y - lo.y) / h)) + 1,
              nz = int(std::ceil((hi.z - lo.z) / h)) + 1;
    auto node = [&](int i, int j, int k) { return (std::size_t(k) * std::size_t(ny) + std::size_t(j)) * std::size_t(nx) + std::size_t(i); };
    auto at = [&](int i, int j, int k) { return lo + h * glm::vec3(float(i), float(j), float(k)); };

    // Field on the lattice. A coarse pass (4 cells per block) skips the blocks the surface cannot reach: the field
    // changes by at most about its distance, so a block whose centre value exceeds its half diagonal is uniform.
    std::vector<float> v(std::size_t(nx) * std::size_t(ny) * std::size_t(nz), 1.0f);
    constexpr int B = 4;
    const float reach = 1.5f * std::sqrt(3.0f) * 0.5f * float(B) * h;
    for (int bk = 0; bk < nz; bk += B)
        for (int bj = 0; bj < ny; bj += B)
            for (int bi = 0; bi < nx; bi += B) {
                const int i1 = std::min(bi + B, nx - 1), j1 = std::min(bj + B, ny - 1), k1 = std::min(bk + B, nz - 1);
                const float c = field(0.5f * (at(bi, bj, bk) + at(i1, j1, k1)));
                if (std::abs(c) > reach) {
                    for (int k = bk; k <= k1; ++k)
                        for (int j = bj; j <= j1; ++j)
                            for (int i = bi; i <= i1; ++i) v[node(i, j, k)] = c;
                    continue;
                }
                for (int k = bk; k <= k1; ++k)
                    for (int j = bj; j <= j1; ++j)
                        for (int i = bi; i <= i1; ++i) v[node(i, j, k)] = field(at(i, j, k));
            }

    FieldMesh m;
    const int cx = nx - 1, cy = ny - 1, cz = nz - 1;
    std::vector<std::int32_t> cellVertex(std::size_t(cx) * std::size_t(cy) * std::size_t(cz), -1);
    auto cellId = [&](int i, int j, int k) { return (std::size_t(k) * std::size_t(cy) + std::size_t(j)) * std::size_t(cx) + std::size_t(i); };
    static const int kEdges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                                      {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (int k = 0; k < cz; ++k)
        for (int j = 0; j < cy; ++j)
            for (int i = 0; i < cx; ++i) {
                float c[8];
                int inside = 0;
                for (int n = 0; n < 8; ++n) {
                    c[n] = v[node(i + (n & 1), j + ((n >> 1) & 1), k + ((n >> 2) & 1))];
                    inside += c[n] < 0.0f;
                }
                if (inside == 0 || inside == 8) continue;
                glm::vec3 sum(0.0f);
                int count = 0;
                for (const auto& e : kEdges) {
                    const float a = c[e[0]], b = c[e[1]];
                    if ((a < 0.0f) == (b < 0.0f)) continue;
                    const float t = a / (a - b);
                    const glm::vec3 pa(float(e[0] & 1), float((e[0] >> 1) & 1), float((e[0] >> 2) & 1));
                    const glm::vec3 pb(float(e[1] & 1), float((e[1] >> 1) & 1), float((e[1] >> 2) & 1));
                    sum += pa + (pb - pa) * t;
                    ++count;
                }
                cellVertex[cellId(i, j, k)] = std::int32_t(m.positions.size());
                m.positions.push_back(at(i, j, k) + h * sum / float(count));
            }

    auto quad = [&](std::size_t a, std::size_t b, std::size_t c, std::size_t d, bool flip) {
        const std::int32_t q[4] = {cellVertex[a], cellVertex[b], cellVertex[c], cellVertex[d]};
        if (q[0] < 0 || q[1] < 0 || q[2] < 0 || q[3] < 0) return;
        if (flip) m.quads.push_back({std::uint32_t(q[0]), std::uint32_t(q[3]), std::uint32_t(q[2]), std::uint32_t(q[1])});
        else m.quads.push_back({std::uint32_t(q[0]), std::uint32_t(q[1]), std::uint32_t(q[2]), std::uint32_t(q[3])});
    };
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const bool in = v[node(i, j, k)] < 0.0f;
                // Edge along +x: the four cells around it in the (y, z) plane.
                if (i < cx && j > 0 && j < cy && k > 0 && k < cz && in != (v[node(i + 1, j, k)] < 0.0f))
                    quad(cellId(i, j - 1, k - 1), cellId(i, j, k - 1), cellId(i, j, k), cellId(i, j - 1, k), !in);
                // Edge along +y: cells around it in the (z, x) plane.
                if (j < cy && i > 0 && i < cx && k > 0 && k < cz && in != (v[node(i, j + 1, k)] < 0.0f))
                    quad(cellId(i - 1, j, k - 1), cellId(i - 1, j, k), cellId(i, j, k), cellId(i, j, k - 1), !in);
                // Edge along +z: cells around it in the (x, y) plane.
                if (k < cz && i > 0 && i < cx && j > 0 && j < cy && in != (v[node(i, j, k + 1)] < 0.0f))
                    quad(cellId(i - 1, j - 1, k), cellId(i, j - 1, k), cellId(i, j, k), cellId(i - 1, j, k), !in);
            }

    // Project onto the surface (two Newton steps along the gradient, at most half a cell) and take the normals from
    // a gradient over most of a cell (smooths detail finer than the lattice can show).
    auto gradAt = [&](const glm::vec3& p, float e) {
        return glm::vec3(field(p + glm::vec3(e, 0, 0)) - field(p - glm::vec3(e, 0, 0)),
                         field(p + glm::vec3(0, e, 0)) - field(p - glm::vec3(0, e, 0)),
                         field(p + glm::vec3(0, 0, e)) - field(p - glm::vec3(0, 0, e))) /
               (2.0f * e);
    };
    auto grad = [&](const glm::vec3& p) { return gradAt(p, 0.25f * h); };
    m.normals.resize(m.positions.size());
    for (std::size_t n = 0; n < m.positions.size(); ++n) {
        glm::vec3 p = m.positions[n];
        const glm::vec3 p0 = p;
        glm::vec3 g = grad(p);
        for (int it = 0; it < 2; ++it) {
            const float g2 = glm::dot(g, g);
            if (g2 < 1e-8f) break;
            glm::vec3 step = g * (field(p) / g2);
            p -= step;
            const glm::vec3 moved = p - p0;
            const float len = glm::length(moved);
            if (len > 0.5f * h) p = p0 + moved * (0.5f * h / len);
            g = grad(p);
        }
        m.positions[n] = p;
        g = gradAt(p, 0.75f * h);
        const float gl = glm::length(g);
        m.normals[n] = gl > 1e-6f ? g / gl : glm::vec3(0, 1, 0);
    }
    return m;
}

} // namespace tf
