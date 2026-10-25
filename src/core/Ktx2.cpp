#include "core/Ktx2.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace tf::ktx2 {
namespace {

constexpr std::uint8_t kIdentifier[12] = {0xAB, 'K', 'T', 'X', ' ', '2', '0', 0xBB, '\r', '\n', 0x1A, '\n'};
constexpr std::size_t kHeaderBytes = 80; // identifier + 9 u32 + index (4 u32 + 2 u64)
constexpr std::size_t kLevelEntryBytes = 24;
constexpr std::uint32_t kDfdModelBc7 = 136;
constexpr std::uint32_t kMaxLevels = 16;

void put32(std::vector<std::uint8_t>& b, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) b.push_back(std::uint8_t(v >> (8 * i)));
}
void put64(std::vector<std::uint8_t>& b, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) b.push_back(std::uint8_t(v >> (8 * i)));
}
void set64(std::vector<std::uint8_t>& b, std::size_t at, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) b[at + std::size_t(i)] = std::uint8_t(v >> (8 * i));
}
std::uint32_t get32(const std::uint8_t* p) {
    return std::uint32_t(p[0]) | std::uint32_t(p[1]) << 8 | std::uint32_t(p[2]) << 16 | std::uint32_t(p[3]) << 24;
}
std::uint64_t get64(const std::uint8_t* p) { return std::uint64_t(get32(p)) | std::uint64_t(get32(p + 4)) << 32; }
void pad(std::vector<std::uint8_t>& b, std::size_t align) {
    while (b.size() % align != 0) b.push_back(0);
}

struct Parsed {
    std::uint32_t vkFormat = 0, width = 0, height = 0, levelCount = 0;
    std::uint64_t offset[kMaxLevels]{}, length[kMaxLevels]{};
};

// Header and level index only; `n` must cover kHeaderBytes + levels * kLevelEntryBytes.
bool parseHeader(const std::uint8_t* p, std::size_t n, std::uint64_t fileSize, Parsed& out, std::string& error) {
    if (n < kHeaderBytes || std::memcmp(p, kIdentifier, sizeof(kIdentifier)) != 0) {
        error = "not a KTX2 file";
        return false;
    }
    out.vkFormat = get32(p + 12);
    const std::uint32_t typeSize = get32(p + 16);
    out.width = get32(p + 20);
    out.height = get32(p + 24);
    const std::uint32_t depth = get32(p + 28), layers = get32(p + 32), faces = get32(p + 36);
    out.levelCount = std::max(get32(p + 40), 1u);
    const std::uint32_t scheme = get32(p + 44);
    if (typeSize != 1 || depth != 0 || layers > 1 || faces != 1 || scheme != 0 || out.width == 0 || out.height == 0) {
        error = "unsupported KTX2 layout (needs a 2D, single layer, single face, uncompressed-container file)";
        return false;
    }
    if (out.levelCount > kMaxLevels || n < kHeaderBytes + out.levelCount * kLevelEntryBytes) {
        error = "bad KTX2 level count";
        return false;
    }
    for (std::uint32_t i = 0; i < out.levelCount; ++i) {
        const std::uint8_t* e = p + kHeaderBytes + i * kLevelEntryBytes;
        out.offset[i] = get64(e);
        out.length[i] = get64(e + 8);
        const std::uint32_t w = std::max(out.width >> i, 1u), h = std::max(out.height >> i, 1u);
        if (out.offset[i] + out.length[i] > fileSize || out.length[i] != bc7LevelBytes(w, h)) {
            error = "KTX2 level " + std::to_string(i) + " out of range or of unexpected size";
            return false;
        }
    }
    return true;
}

} // namespace

std::size_t bc7LevelBytes(std::uint32_t w, std::uint32_t h) {
    return std::size_t((w + 3) / 4) * std::size_t((h + 3) / 4) * 16;
}

std::vector<std::uint8_t> write(const Image& image, const std::string& writer) {
    if (image.levels.empty() || image.levels.size() > kMaxLevels) throw std::runtime_error("ktx2: bad level count");
    for (std::size_t i = 0; i < image.levels.size(); ++i) {
        const Level& l = image.levels[i];
        const std::uint32_t w = std::max(image.width >> i, 1u), h = std::max(image.height >> i, 1u);
        if (l.width != w || l.height != h || l.data.size() != bc7LevelBytes(w, h))
            throw std::runtime_error("ktx2: level " + std::to_string(i) + " has an unexpected size");
    }
    const std::uint32_t levels = std::uint32_t(image.levels.size());
    std::vector<std::uint8_t> b(kIdentifier, kIdentifier + sizeof(kIdentifier));
    put32(b, image.vkFormat);
    put32(b, 1); // typeSize
    put32(b, image.width);
    put32(b, image.height);
    put32(b, 0); // pixelDepth
    put32(b, 0); // layerCount (not an array)
    put32(b, 1); // faceCount
    put32(b, levels);
    put32(b, 0); // supercompressionScheme
    const std::size_t indexAt = b.size();
    for (int i = 0; i < 4; ++i) put32(b, 0); // dfd offset / length, kvd offset / length
    put64(b, 0);                              // sgd offset
    put64(b, 0);                              // sgd length
    const std::size_t levelIndexAt = b.size();
    b.resize(b.size() + levels * kLevelEntryBytes, 0);

    // Data format descriptor: one basic block, one BC7 sample covering the 128-bit block.
    const std::uint32_t dfdAt = std::uint32_t(b.size());
    const bool srgb = image.vkFormat == kVkFormatBc7Srgb;
    put32(b, 44);                                   // dfdTotalSize
    put32(b, 0);                                    // vendorId 0 (Khronos), descriptorType 0 (basic)
    put32(b, 2u | (40u << 16));                     // versionNumber 2, descriptorBlockSize 24 + 16
    put32(b, kDfdModelBc7 | (1u << 8) | ((srgb ? 2u : 1u) << 16)); // model BC7, primaries BT709, transfer, flags 0
    put32(b, 3u | (3u << 8));                       // texel block 4 x 4 (dimension - 1)
    put32(b, 16);                                   // bytesPlane0
    put32(b, 0);                                    // bytesPlane4..7
    put32(b, 0u | (127u << 16));                    // bitOffset 0, bitLength 128 - 1, channel BC7 color
    put32(b, 0);                                    // sample position
    put32(b, 0);                                    // sampleLower
    put32(b, 0xFFFFFFFFu);                          // sampleUpper

    // Key / value data: the writer name.
    const std::uint32_t kvdAt = std::uint32_t(b.size());
    const std::string key = "KTXwriter";
    const std::uint32_t kvLen = std::uint32_t(key.size() + 1 + writer.size() + 1);
    put32(b, kvLen);
    b.insert(b.end(), key.begin(), key.end());
    b.push_back(0);
    b.insert(b.end(), writer.begin(), writer.end());
    b.push_back(0);
    pad(b, 4);
    const std::uint32_t kvdLen = std::uint32_t(b.size()) - kvdAt;

    for (int i = 0; i < 4; ++i) {
        const std::uint32_t v = i == 0 ? dfdAt : i == 1 ? 44u : i == 2 ? kvdAt : kvdLen;
        for (int k = 0; k < 4; ++k) b[indexAt + std::size_t(i) * 4 + std::size_t(k)] = std::uint8_t(v >> (8 * k));
    }
    // Level data, smallest level first, each aligned to lcm(16, 4) = 16 bytes.
    for (std::uint32_t i = levels; i-- > 0;) {
        pad(b, 16);
        const std::size_t at = b.size();
        const Level& l = image.levels[i];
        b.insert(b.end(), l.data.begin(), l.data.end());
        set64(b, levelIndexAt + i * kLevelEntryBytes, at);
        set64(b, levelIndexAt + i * kLevelEntryBytes + 8, l.data.size());
        set64(b, levelIndexAt + i * kLevelEntryBytes + 16, l.data.size());
    }
    return b;
}

bool read(const std::vector<std::uint8_t>& file, Image& out, std::string& error, std::uint32_t skipLevels) {
    Parsed p;
    if (!parseHeader(file.data(), file.size(), file.size(), p, error)) return false;
    const std::uint32_t skip = std::min(skipLevels, p.levelCount - 1);
    out = Image{};
    out.vkFormat = p.vkFormat;
    out.width = std::max(p.width >> skip, 1u);
    out.height = std::max(p.height >> skip, 1u);
    for (std::uint32_t i = skip; i < p.levelCount; ++i) {
        Level l;
        l.width = std::max(p.width >> i, 1u);
        l.height = std::max(p.height >> i, 1u);
        l.data.assign(file.begin() + std::ptrdiff_t(p.offset[i]),
                      file.begin() + std::ptrdiff_t(p.offset[i] + p.length[i]));
        out.levels.push_back(std::move(l));
    }
    return true;
}

bool readFile(const std::string& path, Image& out, std::string& error, std::uint32_t skipLevels) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    const std::uint64_t size = std::uint64_t(in.tellg());
    std::vector<std::uint8_t> head(std::size_t(std::min<std::uint64_t>(size, kHeaderBytes + kMaxLevels * kLevelEntryBytes)));
    in.seekg(0);
    in.read(reinterpret_cast<char*>(head.data()), std::streamsize(head.size()));
    if (isLfsPointer(head)) {
        error = path + " is a Git LFS pointer (run `git lfs pull`)";
        return false;
    }
    Parsed p;
    if (!parseHeader(head.data(), head.size(), size, p, error)) {
        error = path + ": " + error;
        return false;
    }
    // Only the kept levels are read (the skipped full-size levels are most of the file).
    const std::uint32_t skip = std::min(skipLevels, p.levelCount - 1);
    out = Image{};
    out.vkFormat = p.vkFormat;
    out.width = std::max(p.width >> skip, 1u);
    out.height = std::max(p.height >> skip, 1u);
    for (std::uint32_t i = skip; i < p.levelCount; ++i) {
        Level l;
        l.width = std::max(p.width >> i, 1u);
        l.height = std::max(p.height >> i, 1u);
        l.data.resize(std::size_t(p.length[i]));
        in.seekg(std::streamoff(p.offset[i]));
        in.read(reinterpret_cast<char*>(l.data.data()), std::streamsize(l.data.size()));
        if (!in) {
            error = path + ": truncated level " + std::to_string(i);
            return false;
        }
        out.levels.push_back(std::move(l));
    }
    return true;
}

bool isLfsPointer(const std::vector<std::uint8_t>& file) {
    static const char kPrefix[] = "version https://git-lfs.github.com/spec/";
    return file.size() >= sizeof(kPrefix) - 1 && std::memcmp(file.data(), kPrefix, sizeof(kPrefix) - 1) == 0;
}

} // namespace tf::ktx2
