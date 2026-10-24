#pragma once

#include <glm/glm.hpp>

#include <vector>

namespace tf {

struct TrackGeometry;
class SurfaceTable;
struct VehicleConfig;

// Lit mesh vertex. Layout matches LitIn in shaders/scene.metal.
struct LitVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color;
};

struct LineVertex {
    glm::vec3 position;
    glm::vec4 color;
};

// CPU mesh: a non-indexed triangle list (flat shading).
struct MeshData {
    std::vector<LitVertex> vertices;

    void addTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& color);
    void addQuad(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d, const glm::vec3& color);
    void addBox(const glm::vec3& center, const glm::vec3& halfExtents, const glm::vec3& color);
};

// Proving ground mesh from the same geometry as the collision.
MeshData buildTrackMesh(const TrackGeometry& geometry, const SurfaceTable& surfaces);
// Body: the same boxes as the collision shape in Vehicle.cpp (body space).
MeshData buildVehicleBodyMesh();
// Wheel: a cylinder along the model X axis with tread stripes (rotation is visible).
MeshData buildWheelMesh(float radius, float width);

} // namespace tf
