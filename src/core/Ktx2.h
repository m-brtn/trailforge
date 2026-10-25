#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace tf {

// Minimal KTX 2.0 container for block-compressed 2D textures (one layer, one face, no supercompression).
// Used for the terrain material textures: written by tools/texc, read by the renderer. Only the BC7 formats are
// produced; the reader accepts any vkFormat and leaves the format check to the caller.
namespace ktx2 {

constexpr std::uint32_t kVkFormatBc7Unorm = 145; // VK_FORMAT_BC7_UNORM_BLOCK
constexpr std::uint32_t kVkFormatBc7Srgb = 146;  // VK_FORMAT_BC7_SRGB_BLOCK

struct Level {
    std::uint32_t width = 0, height = 0;
    std::vector<std::uint8_t> data; // block data, rows of 4x4 blocks (16 bytes each for BC7)
};

struct Image {
    std::uint32_t vkFormat = 0;
    std::uint32_t width = 0, height = 0;
    std::vector<Level> levels; // level 0 = full size
};

// Bytes of one level of a 16-byte-per-block format (BC7) at w x h texels.
std::size_t bc7LevelBytes(std::uint32_t w, std::uint32_t h);

// Serializes the image (levels stored smallest first, as the format requires). Throws on inconsistent sizes.
std::vector<std::uint8_t> write(const Image& image, const std::string& writer);

// Parses a file image. skipLevels drops that many of the largest levels (lower texture quality presets) without
// copying them. Returns false with a message on any structural error.
bool read(const std::vector<std::uint8_t>& file, Image& out, std::string& error, std::uint32_t skipLevels = 0);
bool readFile(const std::string& path, Image& out, std::string& error, std::uint32_t skipLevels = 0);

// True when the file is a Git LFS pointer instead of the real data (repository cloned without `git lfs pull`).
bool isLfsPointer(const std::vector<std::uint8_t>& file);

} // namespace ktx2
} // namespace tf
