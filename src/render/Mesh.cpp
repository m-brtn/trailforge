#include "render/Mesh.h"

#include "track/SurfaceProperties.h"
#include "track/TestTrack.h"
#include "vehicle/VehicleShape.h"

#include <cmath>

namespace tf {

void MeshData::addTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& color) {
    glm::vec3 n = glm::cross(b - a, c - a);
    const float len = glm::length(n);
    n = len > 1e-12f ? n / len : glm::vec3(0, 1, 0);
    vertices.push_back({a, n, color});
    vertices.push_back({b, n, color});
    vertices.push_back({c, n, color});
}

void MeshData::addQuad(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d,
                       const glm::vec3& color) {
    addTriangle(a, b, c, color);
    addTriangle(a, c, d, color);
}

void MeshData::addBox(const glm::vec3& c, const glm::vec3& h, const glm::vec3& color) {
    auto p = [&](float x, float y, float z) { return c + glm::vec3(x * h.x, y * h.y, z * h.z); };
    addQuad(p(-1, 1, -1), p(-1, 1, 1), p(1, 1, 1), p(1, 1, -1), color);      // +Y
    addQuad(p(-1, -1, -1), p(1, -1, -1), p(1, -1, 1), p(-1, -1, 1), color);  // -Y
    addQuad(p(1, -1, -1), p(1, 1, -1), p(1, 1, 1), p(1, -1, 1), color);      // +X
    addQuad(p(-1, -1, 1), p(-1, 1, 1), p(-1, 1, -1), p(-1, -1, -1), color);  // -X
    addQuad(p(1, -1, 1), p(1, 1, 1), p(-1, 1, 1), p(-1, -1, 1), color);      // +Z
    addQuad(p(-1, -1, -1), p(-1, 1, -1), p(1, 1, -1), p(1, -1, -1), color);  // -Z
}

MeshData buildTrackMesh(const TrackGeometry& g, const SurfaceTable& surfaces) {
    MeshData m;
    m.vertices.reserve(g.positions.size());
    for (std::size_t i = 0; i < g.triangleCount(); ++i) {
        const auto& s = surfaces.get(g.surfaces[i]);
        auto v = [&](std::size_t k) {
            const JPH::Float3& f = g.positions[3 * i + k];
            return glm::vec3(f.x, f.y, f.z);
        };
        // Small deterministic per-triangle brightness variation makes terrain easier to read.
        const std::uint32_t hsh = std::uint32_t(i) * 2654435761u;
        const float var = 0.93f + 0.07f * float((hsh >> 16) & 0xff) / 255.0f;
        m.addTriangle(v(0), v(1), v(2), glm::vec3(s.color[0], s.color[1], s.color[2]) * var);
    }
    return m;
}

MeshData buildVehicleBodyMesh() {
    MeshData m;
    for (const BodyBox& b : kBodyBoxes)
        m.addBox({b.center[0], b.center[1], b.center[2]}, {b.halfExtents[0], b.halfExtents[1], b.halfExtents[2]},
                 {b.color[0], b.color[1], b.color[2]});
    // Windows (thin dark panels on the cab surface): visual only, not part of the collision.
    const BodyBox& cab = kBodyBoxes[2];
    const glm::vec3 cc(cab.center[0], cab.center[1], cab.center[2]);
    const glm::vec3 ch(cab.halfExtents[0], cab.halfExtents[1], cab.halfExtents[2]);
    const glm::vec3 glass(0.10f, 0.13f, 0.17f);
    m.addBox(cc + glm::vec3(0, 0.08f, ch.z + 0.005f), glm::vec3(ch.x - 0.08f, ch.y - 0.15f, 0.006f), glass); // windshield
    m.addBox(cc + glm::vec3(0, 0.08f, -ch.z - 0.005f), glm::vec3(ch.x - 0.12f, ch.y - 0.17f, 0.006f), glass);
    m.addBox(cc + glm::vec3(ch.x + 0.005f, 0.08f, 0), glm::vec3(0.006f, ch.y - 0.15f, ch.z - 0.12f), glass);
    m.addBox(cc + glm::vec3(-ch.x - 0.005f, 0.08f, 0), glm::vec3(0.006f, ch.y - 0.15f, ch.z - 0.12f), glass);
    // Headlights (+Z is the vehicle front).
    const glm::vec3 lamp(0.95f, 0.92f, 0.75f);
    m.addBox({0.45f, 0.45f, 2.305f}, {0.12f, 0.06f, 0.006f}, lamp);
    m.addBox({-0.45f, 0.45f, 2.305f}, {0.12f, 0.06f, 0.006f}, lamp);
    const glm::vec3 tail(0.7f, 0.05f, 0.05f);
    m.addBox({0.75f, 0.45f, -2.305f}, {0.1f, 0.06f, 0.006f}, tail);
    m.addBox({-0.75f, 0.45f, -2.305f}, {0.1f, 0.06f, 0.006f}, tail);
    return m;
}

MeshData buildWheelMesh(float radius, float width) {
    MeshData m;
    constexpr int kSegments = 28;
    const float hw = 0.5f * width;
    const glm::vec3 tire(0.08f, 0.08f, 0.085f), tread(0.16f, 0.16f, 0.17f), rim(0.62f, 0.62f, 0.60f),
        marker(0.85f, 0.15f, 0.1f);
    const float rimR = radius * 0.6f;
    for (int i = 0; i < kSegments; ++i) {
        const float a0 = 2.0f * 3.14159265f * float(i) / kSegments;
        const float a1 = 2.0f * 3.14159265f * float(i + 1) / kSegments;
        // Wheel axis is X; the circle lies in the YZ plane.
        auto ring = [&](float r, float a, float x) { return glm::vec3(x, r * std::cos(a), r * std::sin(a)); };
        const glm::vec3 tc = (i % 2 == 0) ? tire : tread;
        // Tread (outward)
        m.addQuad(ring(radius, a0, hw), ring(radius, a0, -hw), ring(radius, a1, -hw), ring(radius, a1, hw), tc);
        // Sidewalls: tire (ring) and rim
        const glm::vec3 discColor = (i == 0) ? marker : rim;
        m.addQuad(ring(rimR, a0, hw), ring(radius, a0, hw), ring(radius, a1, hw), ring(rimR, a1, hw), tire);
        m.addTriangle(glm::vec3(hw + 0.01f, 0, 0), ring(rimR, a0, hw + 0.01f), ring(rimR, a1, hw + 0.01f), discColor);
        m.addQuad(ring(radius, a0, -hw), ring(rimR, a0, -hw), ring(rimR, a1, -hw), ring(radius, a1, -hw), tire);
        m.addTriangle(glm::vec3(-hw - 0.01f, 0, 0), ring(rimR, a1, -hw - 0.01f), ring(rimR, a0, -hw - 0.01f), discColor);
    }
    return m;
}

} // namespace tf
