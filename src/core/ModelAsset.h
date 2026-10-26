#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace tf {

// A converted model of assets/models (written by tools/texc trailforge_modelc from a Poly Haven glTF): mesh LODs
// "lod0".. with one primitive per material, an optional "collision" mesh (positions only), BC7 KTX2 material maps
// referenced by file name, an optional impostor atlas, and the measurements in the root extras. No rendering
// dependency: the level bake reads trunk radii and collision meshes from here, the renderer the meshes.
struct ModelVertex {
    float pos[3];
    float normal[3];
    float uv[2];
    float wind[2]; // x: trunk / branch bend weight, y: leaf flutter weight (render-only wind sway)
};

struct ModelPrimitive {
    std::uint32_t firstIndex = 0, indexCount = 0;
    int material = 0;
};

struct ModelLod {
    std::vector<ModelVertex> vertices;
    std::vector<std::uint32_t> indices; // absolute into vertices
    std::vector<ModelPrimitive> primitives;
    std::size_t triangles() const { return indices.size() / 3; }
};

struct ModelMaterial {
    std::string name;
    std::string albedo, nra; // file names next to the .glb
    bool alphaTest = false;  // alpha-tested, double-sided (foliage cards)
    std::array<float, 3> tint{1.0f, 1.0f, 1.0f}; // linear albedo multiplier (building kit materials)
    float roughness = -1.0f; // >= 0: constant roughness without an nra map (building kit glass, nylon)
};

struct ModelAsset {
    std::string id, kind; // kind: tree, bush, rock, cliff, log, debris
    float height = 0.0f, radius = 0.0f;
    std::array<float, 3> boundsMin{}, boundsMax{};
    float trunkRadius = 0.0f;           // trees: trunk radius at 1.0 .. 1.6 m (unit scale)
    std::array<float, 3> trunkTop{};    // trees: trunk axis point at 0.3 x height (the base is the origin)
    std::vector<float> lodError;        // m per LOD: largest simplification error of the opaque parts
    std::vector<ModelLod> lods;         // empty unless loaded with meshes
    std::vector<ModelMaterial> materials;
    int collisionLod = -1;                      // LOD the collision mesh was made from (-1: none)
    std::vector<float> collisionPositions;      // xyz, unit scale (empty: no collision mesh)
    std::vector<std::uint32_t> collisionIndices;
    std::string impostorAlbedo, impostorNra; // file names (empty: no impostor)
    int impostorViews = 0;                   // atlas tiles in one row: view k from azimuth 2 pi k / views

    // Reads a .glb. withMeshes = false skips the render LODs (measurements, materials and collision only).
    // Returns false with a message on any error.
    bool load(const std::string& path, bool withMeshes, std::string* error = nullptr);
};

} // namespace tf
