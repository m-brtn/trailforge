#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace tf {

// Binary PGM (P5) images of the baked level files (tools/levels/blender export): the 16-bit heightmap (big-endian
// samples, maxval 65535) and the 8-bit per-cell surface map. No SDL.
struct PgmImage {
    int width = 0, height = 0;
    int maxValue = 0;                 // 255: 8-bit samples, 65535: 16-bit
    std::vector<std::uint16_t> data;  // row-major, width x height
};

bool readPgm(const std::string& path, PgmImage& out, std::string& error);
bool writePgm(const std::string& path, const PgmImage& image, std::string& error);

// Surface classes of the baked surface map (one byte per cell). The physics surface of each class is
// levelSurfaceClassSurface(); gravel and dirt share the packed dirt surface and differ only in the render look.
enum class LevelSurfaceClass : std::uint8_t { Asphalt = 0, Gravel, Dirt, Mud, Grass, Sand, Rock, Snow, Count };
const char* levelSurfaceClassName(LevelSurfaceClass c);

} // namespace tf
