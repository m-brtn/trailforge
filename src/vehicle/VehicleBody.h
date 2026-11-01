#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

// Blender-authored vehicle bodies (docs/vehicle-format.md, body.glb): the converted model the game reads,
// written by trailforge_vehiclec (tools/cars) from a Blender glTF export. Body space (+X left, +Y up, +Z forward, m,
// origin = the vehicle body origin the physics wheels are placed from). Named parts with LOD meshes, materials
// mapped to the game's material roles, convex collision pieces (COL_*) and spare wheel seats. Wheels, tires and
// suspension are never part of it: they stay procedural and are placed from the Jolt wheels. No SDL, no rendering.

namespace tf {

struct VehicleConfig;
struct VehicleDefinition;

// Game material a body material maps to (by material name or the glTF extras key "tf_role").
enum class BodyRole {
    PaintPrimary,
    PaintSecondary,
    PaintAccent,
    Cladding,       // grained unpainted plastic
    Plastic,        // smooth trim
    Chrome,
    Metal,          // steel, aluminium (colour, roughness and metallic from the material)
    Glass,
    Rubber,
    Lens,           // clear lamp housing (not emissive)
    LightHead,      // emissive, scaled by the headlight switch
    LightTail,      // emissive, scaled by the brake pedal (dim running level without it)
    LightBrake,
    LightReverse,   // emissive only in reverse
    LightIndicator, // amber marker
    Liner,          // wheelhouse liner, recesses
    Underbody,
    Count
};
const char* bodyRoleName(BodyRole r); // "paint_primary", ...
bool bodyRoleFromName(const std::string& name, BodyRole& out);
std::string bodyRoleList(); // comma-separated names
float bodyRoleDefaultMud(BodyRole r);

struct BodyMaterial {
    std::string name;
    BodyRole role = BodyRole::Plastic;
    std::array<float, 3> color{0.5f, 0.5f, 0.5f}; // linear base colour
    float roughness = 0.5f, metallic = 0.0f;
    float mud = 1.0f;                             // cosmetic mud receptivity 0..1
    std::array<float, 3> emissive{0.0f, 0.0f, 0.0f};
    float emissiveStrength = 1.0f;
    float clearcoat = 0.0f, clearcoatRoughness = 0.0f;
    float transmission = 0.0f, ior = 1.5f, alpha = 1.0f;
};

// Indexed triangle mesh; primitives are index ranges of one material each.
struct BodyMesh {
    struct Primitive {
        std::uint32_t firstIndex = 0, indexCount = 0;
        int material = 0;
    };
    std::vector<float> positions, normals; // xyz per vertex
    std::vector<float> colors;             // rgb per vertex (linear, multiplies the material colour); may be empty
    std::vector<std::uint32_t> indices;
    std::vector<Primitive> primitives;
    std::size_t vertexCount() const { return positions.size() / 3; }
    std::size_t triangles() const { return indices.size() / 3; }
};

struct BodyPart {
    std::string name, parent;              // parent part ("" = the body root)
    std::array<float, 3> attach{};         // hinge / bolt point (default: centre of the LOD0 bounds)
    bool deformable = false, detachable = false;
    bool collides = true;                  // false: left out of the visual vs collision check (mirrors, antennas)
    std::vector<BodyMesh> lods;            // lods[0] is drawn; lods[1], lods[2] are simplified versions
};

// Convex collision piece: the authored triangles (validated convex); the physics uses the hull of the points.
struct BodyHull {
    std::string name;
    std::vector<std::array<float, 3>> points;
    std::vector<std::uint32_t> indices;
};

// Spare wheel seat: the procedural rear wheel is drawn with its inner face at `seat`, facing `outward`.
struct BodySpare {
    std::string name, parent;
    std::array<float, 3> seat{}, outward{0.0f, 0.0f, -1.0f};
};

struct BodyModel {
    std::vector<BodyMaterial> materials;
    std::vector<BodyPart> parts;
    std::vector<BodyHull> hulls;
    std::vector<BodySpare> spares;
    std::array<float, 3> boundsMin{}, boundsMax{}; // LOD0 of all parts
    std::vector<float> lodError;                   // m per LOD: largest simplification error
    std::string source;                            // source file name (informative)

    const BodyPart* find(const std::string& name) const;
    void computeBounds();
};

inline constexpr int kBodyLodCount = 3;

// Reads a converted body.glb. withMeshes = false skips the render meshes (materials, collision and spares only).
bool loadBodyModel(const std::string& path, BodyModel& out, bool withMeshes, std::string* error = nullptr);
// Writes the converted format (identity node transforms, glTF material extensions for clear coat, transmission,
// IOR and emissive strength, roles and mud in the material extras).
bool writeBodyModel(const std::string& path, const BodyModel& model, std::string* error = nullptr);

// Concavity of a collision piece: the largest depth (m) of one of its points below the surface of the convex hull of
// all its points (0 for a convex piece; a dent shows as its depth). Returns a large value for a degenerate piece
// (fewer than 4 points, under 10 mm in some direction).
float hullConvexityError(const BodyHull& hull);

struct BodyValidation {
    std::vector<std::string> errors, warnings;
    float minClearance = 1e9f;   // m, smallest wheel-sweep clearance of the render LOD0 (vertices, edge midpoints, centroids)
    std::string clearanceWhere;  // part and point of the smallest clearance
    float hullClearance = 1e9f;  // same for the collision pieces
    float boundsDeviation = 0.0f; // m, largest difference between the visual and the collision bounds (any face)
    float hullCoverage = 0.0f;    // share of LOD0 vertices within 0.10 m of a collision piece
};

inline constexpr float kBodyConvexTolerance = 0.002f; // m
inline constexpr float kBodyClearance = 0.01f;        // m, required gap between body and wheel sweep
inline constexpr float kBodyBoundsTolerance = 0.12f;  // m, visual vs collision bounds per face

// Checks a body against every configuration it can be driven with: names (no wheel / tire / suspension parts),
// materials, convex collision pieces, LOD0 (and the collision) outside every wheel sweep by `clearance`, visual vs
// collision bounds within kBodyBoundsTolerance and coverage.
BodyValidation validateBodyModel(const BodyModel& model, const std::vector<VehicleConfig>& configs,
                                 float clearance = kBodyClearance);

// The configurations a body must clear: the base, every kit, and each of them at the lowest ride height the garage
// allows (wheels highest in the body). Tire sizes changed outside the definition (scenario `wheels`) are not covered.
std::vector<VehicleConfig> bodyCheckConfigs(const VehicleDefinition& def);

// True when a part name names something the game draws procedurally (wheel, tire, rim, suspension, spare tire).
bool bodyNameIsReserved(const std::string& name);

} // namespace tf
