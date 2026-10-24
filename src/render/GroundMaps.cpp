#include "render/GroundMaps.h"

#include "track/TestTrack.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace tf {
namespace {

std::uint32_t hash3(std::int32_t x, std::int32_t y, std::uint32_t seed) {
    std::uint32_t h = std::uint32_t(x) * 0x8da6b343u ^ std::uint32_t(y) * 0xd8163841u ^ seed * 0xcb1ab31fu;
    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return h;
}

float hashUnit(std::int32_t x, std::int32_t y, std::uint32_t seed) { return float(hash3(x, y, seed) & 0xffffff) / 16777215.0f; }

float fade(float t) { return t * t * (3.0f - 2.0f * t); }

// Value noise; period > 0 wraps the lattice (tileable).
float valueNoise(float x, float y, std::uint32_t seed, int period = 0) {
    const float fx = std::floor(x), fy = std::floor(y);
    std::int32_t ix = std::int32_t(fx), iy = std::int32_t(fy);
    const float tx = fade(x - fx), ty = fade(y - fy);
    auto w = [&](std::int32_t v) { return period > 0 ? ((v % period) + period) % period : v; };
    const float a = hashUnit(w(ix), w(iy), seed), b = hashUnit(w(ix + 1), w(iy), seed);
    const float c = hashUnit(w(ix), w(iy + 1), seed), d = hashUnit(w(ix + 1), w(iy + 1), seed);
    return (a + (b - a) * tx) + ((c + (d - c) * tx) - (a + (b - a) * tx)) * ty;
}

float tileFbm(float u, float v, int baseCells, int octaves, std::uint32_t seed) {
    float sum = 0.0f, amp = 0.5f, norm = 0.0f;
    int cells = baseCells;
    for (int o = 0; o < octaves; ++o) {
        sum += amp * valueNoise(u * float(cells), v * float(cells), seed + std::uint32_t(o) * 101u, cells);
        norm += amp;
        amp *= 0.5f;
        cells *= 2;
    }
    return sum / norm;
}

// Tileable cellular noise: distance to the nearest feature point, normalized to ~[0, 1].
float tileCells(float u, float v, int cells, std::uint32_t seed) {
    const float x = u * float(cells), y = v * float(cells);
    const std::int32_t ix = std::int32_t(std::floor(x)), iy = std::int32_t(std::floor(y));
    float best = 10.0f;
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
            const std::int32_t cx = ix + dx, cy = iy + dy;
            const std::int32_t wx = ((cx % cells) + cells) % cells, wy = ((cy % cells) + cells) % cells;
            const float px = float(cx) + hashUnit(wx, wy, seed), py = float(cy) + hashUnit(wx, wy, seed + 7u);
            best = std::min(best, std::hypot(px - x, py - y));
        }
    return std::clamp(best / 1.1f, 0.0f, 1.0f);
}

std::uint16_t toHalf(float f) {
    std::uint32_t x;
    std::memcpy(&x, &f, 4);
    const std::uint32_t sign = (x >> 16) & 0x8000u;
    std::int32_t exp = std::int32_t((x >> 23) & 0xff) - 127 + 15;
    std::uint32_t mant = x & 0x7fffffu;
    if (exp <= 0) return std::uint16_t(sign); // flush tiny values to zero
    if (exp >= 31) return std::uint16_t(sign | 0x7c00u);
    return std::uint16_t(sign | (std::uint32_t(exp) << 10) | (mant >> 13));
}

float smooth(float e0, float e1, float x) {
    const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// 1 away from the proving-ground lanes (they are driven on), ramping to 0 inside them.
float laneFree(float x, float z) {
    if (z > 46.0f) return 1.0f;
    const float lanes[][2] = {{-8.0f, 8.0f}, {-34.0f, -18.0f}, {16.0f, 36.0f}};
    float f = 1.0f;
    for (const auto& l : lanes) {
        const float d = std::max(l[0] - x, x - l[1]); // > 0 outside the lane
        f = std::min(f, smooth(0.0f, 1.8f, d));
    }
    return f;
}

} // namespace

float fbm2(float x, float z, int octaves, std::uint32_t seed) {
    float sum = 0.0f, amp = 0.5f, norm = 0.0f, f = 1.0f;
    for (int o = 0; o < octaves; ++o) {
        sum += amp * valueNoise(x * f, z * f, seed + std::uint32_t(o) * 131u);
        norm += amp;
        amp *= 0.5f;
        f *= 2.03f;
    }
    return sum / norm;
}

float GroundMaps::grassAt(float x, float z) const {
    const int i = int(std::lround((x - x0) / texel)), j = int(std::lround((z - z0) / texel));
    if (i < 0 || j < 0 || i >= width || j >= height) return 0.0f;
    return float(mask[(std::size_t(j) * width + i) * 4]) / 255.0f;
}

float GroundMaps::heightAt(float x, float z) const {
    const float cx = std::clamp(x, x0, x0 + sizeX()), cz = std::clamp(z, z0, z0 + sizeZ());
    return TestTrack::provingGroundHeight(cx, cz);
}

GroundMaps buildGroundMaps(const TestTrack& track) {
    GroundMaps g;
    const TrackBounds b = TestTrack::bounds();
    const WaterInfo water = TestTrack::water();
    g.x0 = b.xMin;
    g.z0 = b.zMin;
    g.texel = 0.25f;
    g.width = int(std::lround((b.xMax - b.xMin) / g.texel)) + 1;
    g.height = int(std::lround((b.zMax - b.zMin) / g.texel)) + 1;
    g.heights.resize(std::size_t(g.width) * g.height);
    g.mask.resize(std::size_t(g.width) * g.height * 4);

    // Mud surface indicator of the physics, box-blurred over +-0.75 m so the visual edge is soft but in place.
    const std::size_t count = std::size_t(g.width) * g.height;
    std::vector<float> mudInd(count, 0.0f), tmp(count, 0.0f);
    for (int j = 0; j < g.height; ++j)
        for (int i = 0; i < g.width; ++i)
            mudInd[std::size_t(j) * g.width + i] =
                TestTrack::groundSurfaceAt(g.x0 + float(i) * g.texel, g.z0 + float(j) * g.texel) == SurfaceId::Mud ? 1.0f : 0.0f;
    constexpr int kR = 3;
    for (int pass = 0; pass < 2; ++pass) {
        const std::vector<float>& src = pass == 0 ? mudInd : tmp;
        std::vector<float>& dst = pass == 0 ? tmp : mudInd;
        for (int j = 0; j < g.height; ++j)
            for (int i = 0; i < g.width; ++i) {
                float sum = 0.0f;
                for (int k = -kR; k <= kR; ++k) {
                    const int ii = pass == 0 ? std::clamp(i + k, 0, g.width - 1) : i;
                    const int jj = pass == 1 ? std::clamp(j + k, 0, g.height - 1) : j;
                    sum += src[std::size_t(jj) * g.width + ii];
                }
                dst[std::size_t(j) * g.width + i] = sum / float(2 * kR + 1);
            }
    }

    const auto& rocks = track.rocks();
    for (int j = 0; j < g.height; ++j) {
        for (int i = 0; i < g.width; ++i) {
            const float x = g.x0 + float(i) * g.texel, z = g.z0 + float(j) * g.texel;
            const float h = TestTrack::provingGroundHeight(x, z);
            g.heights[std::size_t(j) * g.width + i] = toHalf(h);

            const SurfaceId s = TestTrack::groundSurfaceAt(x, z);
            // Visual mud: the smooth path weight plus the blurred Mud surface (puddle rims) of the physics.
            const float blurMud = mudInd[std::size_t(j) * g.width + i];
            const float mudW = std::max(TestTrack::showcaseMudWeight(x, z), blurMud);
            const float n = fbm2(x * 0.09f, z * 0.09f, 4, 17u);
            float grass = 0.0f;
            if (s == SurfaceId::Grass) {
                grass = (0.8f + 0.2f * n) * (1.0f - smooth(0.15f, 0.45f, mudW));
            } else if (s == SurfaceId::PackedDirt) {
                const float d = std::min(std::min(x - b.xMin, b.xMax - x), std::min(z - b.zMin, b.zMax - z));
                const float berm = 1.0f - smooth(4.0f, 6.0f, d);
                const float tufts = smooth(0.42f, 0.66f, n);
                grass = std::max(tufts * 0.85f, berm * 0.8f) * laneFree(x, z);
                if (z > 42.0f) grass = std::max(grass, smooth(42.0f, 47.0f, z) * 0.8f); // toward the meadow
            }
            for (const RockFootprint& r : rocks) {
                const float dr = std::hypot(x - r.x, z - r.z);
                if (dr < r.radius * 1.4f) grass *= smooth(r.radius * 0.85f, r.radius * 1.4f, dr);
            }
            float wet = 0.0f;
            if (z > 46.0f)
                wet = std::clamp(blurMud * (1.0f - smooth(water.level, water.level + 0.15f, h)) + 0.35f * mudW, 0.0f, 1.0f);
            else
                wet = 0.45f * blurMud; // the flat mud bath on the proving ground: wet, no standing water
            std::uint8_t* px = &g.mask[(std::size_t(j) * g.width + i) * 4];
            px[0] = std::uint8_t(std::lround(std::clamp(grass, 0.0f, 1.0f) * 255.0f));
            px[1] = std::uint8_t(std::lround(std::clamp(mudW, 0.0f, 1.0f) * 255.0f));
            px[2] = std::uint8_t(std::lround(wet * 255.0f));
            px[3] = 255;
        }
    }

    g.noise.resize(std::size_t(g.noiseSize) * g.noiseSize * 4);
    for (int j = 0; j < g.noiseSize; ++j)
        for (int i = 0; i < g.noiseSize; ++i) {
            const float u = (float(i) + 0.5f) / float(g.noiseSize), v = (float(j) + 0.5f) / float(g.noiseSize);
            std::uint8_t* px = &g.noise[(std::size_t(j) * g.noiseSize + i) * 4];
            px[0] = std::uint8_t(std::lround(tileFbm(u, v, 4, 5, 1u) * 255.0f));
            px[1] = std::uint8_t(std::lround(tileFbm(u, v, 8, 4, 2u) * 255.0f));
            px[2] = std::uint8_t(std::lround(tileCells(u, v, 16, 3u) * 255.0f));
            px[3] = std::uint8_t(std::lround(tileFbm(u, v, 32, 2, 4u) * 255.0f));
        }
    return g;
}

} // namespace tf
