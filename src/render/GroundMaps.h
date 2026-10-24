#pragma once

#include <cstdint>
#include <vector>

namespace tf {

class TestTrack;

// Render-side maps baked once from the procedural track (no collision role):
// - height: terrain height (m) on a regular grid, used by grass and water;
// - mask (RGBA8): R grass density, G mud weight, B wetness, A unused;
// - noise (RGBA8, tileable): R/G fbm, B cellular, A fine noise for procedural materials.
struct GroundMaps {
    float x0 = 0, z0 = 0;   // world position of texel (0, 0)
    float texel = 0.25f;    // m
    int width = 0, height = 0;
    std::vector<std::uint16_t> heights; // half floats
    std::vector<std::uint8_t> mask;     // 4 bytes per texel

    int noiseSize = 256;
    std::vector<std::uint8_t> noise;    // 4 bytes per texel

    float sizeX() const { return float(width - 1) * texel; }
    float sizeZ() const { return float(height - 1) * texel; }
    float grassAt(float x, float z) const;  // bilinear-free lookup (nearest), 0..1
    float heightAt(float x, float z) const; // nearest, m
};

GroundMaps buildGroundMaps(const TestTrack& track);

// Deterministic value-noise fbm in [0, 1] (CPU side, also used for scenery).
float fbm2(float x, float z, int octaves, std::uint32_t seed);

} // namespace tf
