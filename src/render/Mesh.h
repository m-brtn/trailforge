#pragma once

#include <glm/glm.hpp>

#include <vector>

namespace tf {

struct TrackGeometry;
class SurfaceTable;

// Shader material ids (LitVertex::material.x). Must match the MAT_* constants in shaders/scene.hlsl.
enum class MatId : int {
    Generic = 0,      // PBR from the vertex: color, roughness, metallic
    Dirt = 1,         // procedural ground materials (world-space noise)
    Concrete = 2,
    Rock = 3,
    LowGrip = 4,
    Mud = 5,
    Grass = 6,
    Glass = 7,
    Headlight = 8,    // emissive, scaled by the headlight switch
    BrakeLight = 9,   // emissive, scaled by the brake pedal
    ReverseLight = 10,
    Rubber = 11,
    Amber = 12,       // emissive indicator / marker
    Foliage = 13,
    Paint = 14,       // metallic paint with a clear coat
    Sand = 15,
    Snow = 16,
};

struct Material {
    MatId id = MatId::Generic;
    glm::vec3 color{0.5f};  // linear albedo (emissive materials: emission color)
    float roughness = 0.6f;
    float metallic = 0.0f;
    float mud = 0.0f;       // 0..1, how much cosmetic mud this surface collects
};

// Lit mesh vertex. Layout matches LitIn in shaders/scene.hlsl.
struct LitVertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color;
    glm::vec4 material; // x: MatId, y: roughness, z: metallic, w: mud receptivity
};

// Instance of a unit-height tree mesh. Layout matches the instance buffer read by tree_vs (two float4).
struct TreeInstance {
    glm::vec4 baseHeight; // xyz: trunk base, w: height (m)
    glm::vec4 params;     // x: yaw (rad), y: color variation 0..1, z: width scale, w: unused
};

struct LineVertex {
    glm::vec3 position;
    glm::vec4 color;
};

// CPU mesh: a non-indexed triangle list. Every add* call transforms by `xf` (positions) and its
// inverse transpose (normals), so parts can be built in a local frame.
struct MeshData {
    std::vector<LitVertex> vertices;
    glm::mat4 xf{1.0f};

    void addTri(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& na, const glm::vec3& nb,
                const glm::vec3& nc, const Material& m);
    // Flat-shaded triangle, CCW = front.
    void addTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const Material& m);
    // Flat triangle whose winding is fixed so that it faces away from `inside` (convex parts).
    void addTriangleOutward(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& inside,
                            const Material& m);
    void addQuad(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d, const Material& m);
    void addQuadOutward(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d,
                        const glm::vec3& inside, const Material& m);
    void addBox(const glm::vec3& center, const glm::vec3& halfExtents, const Material& m);
    // Box with chamfered edges and corners (bevel in meters).
    void addBeveledBox(const glm::vec3& center, const glm::vec3& halfExtents, float bevel, const Material& m);
    // Box spanning the segment p0..p1 (cross-section 2*halfW x 2*halfH; `up` orients the halfH axis).
    void addBar(const glm::vec3& p0, const glm::vec3& p1, float halfW, float halfH, const glm::vec3& up,
                const Material& m, float bevel = 0.0f);
    // Smooth cylinder from p0 to p1.
    void addCylinder(const glm::vec3& p0, const glm::vec3& p1, float radius, int segments, const Material& m,
                     bool caps = true);
    // Convex hull of two parallel rectangles (bottom/top, centered on x = 0) given in this frame:
    // bottom (hx0, y0, z0a..z0b), top (hx1, y1, z1a..z1b). Each of the 4 sides can have its own material.
    void addFrustum(float hx0, float y0, float z0a, float z0b, float hx1, float y1, float z1a, float z1b,
                    const Material& sides, const Material& front, const Material& back, const Material& top);
    // Surface of revolution around the local X axis: profile points (x, r), smooth normals.
    void addLathe(const std::vector<glm::vec2>& profile, int segments, const Material& m);
    void append(const MeshData& other);
};

// Proving ground mesh from the same triangles as the collision (smooth normals, ground materials).
MeshData buildTrackMesh(const TrackGeometry& geometry, const SurfaceTable& surfaces);

} // namespace tf
