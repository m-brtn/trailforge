#pragma once

#include "core/Ktx2.h"

#include <cstdint>
#include <string>
#include <vector>

namespace tf {

// The terrain material textures of assets/textures/terrain/terrain.json: one albedo (+ height) and one normal /
// roughness / AO map per layer, in the shader's layer order. Render-only data; no SDL.
struct TerrainTextureSet {
    static constexpr int kLayers = 6; // grass, soil, road, rock, sand, snow (shaders/scene.hlsl LAYER_*)
    struct Layer {
        std::string id;
        float tile = 2.0f; // m per texture repeat
        float tint[3] = {1.0f, 1.0f, 1.0f}; // linear albedo multiplier (optional "tint" in terrain.json)
        std::string albedoPath, nraPath;
    };
    std::uint32_t albedoSize = 0, nraSize = 0; // stored full size
    std::vector<Layer> layers;
};

// Reads terrain.json from `dir`. Returns false with a message if it is missing or does not list the kLayers layers
// in order.
bool loadTerrainTextureSet(const std::string& dir, TerrainTextureSet& out, std::string& error);

// Largest levels to skip so that a map stored at `stored` texels is used at no more than `wanted` texels.
std::uint32_t terrainTextureSkip(std::uint32_t stored, std::uint32_t wanted);

// Loads one map (both layers' maps must then have the same size and format). `srgb`: expected BC7 variant.
bool loadTerrainTextureMap(const std::string& path, bool srgb, std::uint32_t skip, ktx2::Image& out,
                           std::string& error);

} // namespace tf
