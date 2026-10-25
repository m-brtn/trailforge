// trailforge_texc: encodes one terrain material map into a BC7 KTX2 file with a full mip chain (down to 1 x 1).
//
//   trailforge_texc albedo --color diff.jpg --height disp.jpg --size 4096 --out grass_albedo.ktx2
//       RGB: albedo (sRGB), A: height (stretched to 0..1 between the 1st and 99th percentile). BC7 sRGB.
//   trailforge_texc nra --normal nor_dx.jpg --arm arm.jpg --size 2048 --out grass_nra.ktx2
//       R, G: tangent-space normal x, y (DirectX convention: +G points down the image), B: roughness, A: ambient
//       occlusion (from the glTF ARM map: R = AO, G = roughness). BC7 linear.
//
// Mips: albedo averaged in linear light; normals averaged as vectors and renormalized, the shortening of the
// average folded into the roughness (Toksvig), so distant ground does not turn glossy. Deterministic output: the
// blocks are split across threads but each is encoded independently.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#include <stb_image.h>

#include "bc7decomp.h"
#include "bc7enc.h"
#include "core/Ktx2.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Plane { // float RGBA, row-major
    int w = 0, h = 0;
    std::vector<float> px;
    float* at(int x, int y) { return &px[(std::size_t(y) * std::size_t(w) + std::size_t(x)) * 4]; }
    const float* at(int x, int y) const { return &px[(std::size_t(y) * std::size_t(w) + std::size_t(x)) * 4]; }
};

[[noreturn]] void die(const std::string& msg) {
    std::fprintf(stderr, "trailforge_texc: %s\n", msg.c_str());
    std::exit(1);
}

float srgbToLinear(float c) { return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f); }
float linearToSrgb(float c) {
    c = std::clamp(c, 0.0f, 1.0f);
    return c <= 0.0031308f ? c * 12.92f : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
}

// Loads an image as float RGBA 0..1 (channels replicated for grayscale), box-downsampled to size x size.
Plane load(const std::string& path, int size) {
    int w = 0, h = 0, n = 0;
    const bool sixteen = stbi_is_16_bit(path.c_str());
    Plane src;
    if (sixteen) {
        stbi_us* d = stbi_load_16(path.c_str(), &w, &h, &n, 4);
        if (!d) die("cannot decode " + path + ": " + stbi_failure_reason());
        src.w = w;
        src.h = h;
        src.px.resize(std::size_t(w) * std::size_t(h) * 4);
        for (std::size_t i = 0; i < src.px.size(); ++i) src.px[i] = float(d[i]) / 65535.0f;
        stbi_image_free(d);
    } else {
        stbi_uc* d = stbi_load(path.c_str(), &w, &h, &n, 4);
        if (!d) die("cannot decode " + path + ": " + stbi_failure_reason());
        src.w = w;
        src.h = h;
        src.px.resize(std::size_t(w) * std::size_t(h) * 4);
        for (std::size_t i = 0; i < src.px.size(); ++i) src.px[i] = float(d[i]) / 255.0f;
        stbi_image_free(d);
    }
    if (w != h || w < size || w % size != 0) die(path + ": needs a square image whose side is a multiple of " + std::to_string(size));
    const int f = w / size;
    if (f == 1) return src;
    Plane out;
    out.w = out.h = size;
    out.px.assign(std::size_t(size) * std::size_t(size) * 4, 0.0f);
    const float inv = 1.0f / float(f * f);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x) {
            float* o = out.at(x, y);
            for (int j = 0; j < f; ++j)
                for (int i = 0; i < f; ++i) {
                    const float* s = src.at(x * f + i, y * f + j);
                    for (int c = 0; c < 4; ++c) o[c] += s[c] * inv;
                }
        }
    return out;
}

// 2 x 2 average of a linear plane (channels as given).
Plane half(const Plane& p) {
    Plane o;
    o.w = std::max(p.w / 2, 1);
    o.h = std::max(p.h / 2, 1);
    o.px.resize(std::size_t(o.w) * std::size_t(o.h) * 4);
    for (int y = 0; y < o.h; ++y)
        for (int x = 0; x < o.w; ++x) {
            const float *a = p.at(2 * x, 2 * y), *b = p.at(2 * x + 1, 2 * y), *c = p.at(2 * x, 2 * y + 1),
                        *d = p.at(2 * x + 1, 2 * y + 1);
            float* r = o.at(x, y);
            for (int k = 0; k < 4; ++k) r[k] = 0.25f * (a[k] + b[k] + c[k] + d[k]);
        }
    return o;
}

struct Encoded {
    tf::ktx2::Level level;
    double sqErr = 0.0;
    std::size_t samples = 0;
};

// Encodes an 8-bit RGBA plane (already in its storage encoding) to BC7 and measures the RGBA error.
Encoded encode(const std::vector<std::uint8_t>& rgba, int w, int h, bool perceptual) {
    Encoded e;
    e.level.width = std::uint32_t(w);
    e.level.height = std::uint32_t(h);
    const int bw = (w + 3) / 4, bh = (h + 3) / 4;
    e.level.data.resize(std::size_t(bw) * std::size_t(bh) * 16);
    bc7enc_compress_block_params params;
    bc7enc_compress_block_params_init(&params);
    if (!perceptual) bc7enc_compress_block_params_init_linear_weights(&params);
    params.m_uber_level = 1;
    params.m_max_partitions = BC7ENC_MAX_PARTITIONS;

    const unsigned threads = std::max(1u, std::thread::hardware_concurrency());
    std::atomic<int> nextRow{0};
    std::vector<double> err(threads, 0.0);
    auto work = [&](unsigned t) {
        for (int by; (by = nextRow.fetch_add(1)) < bh;) {
            for (int bx = 0; bx < bw; ++bx) {
                std::uint8_t block[64];
                for (int j = 0; j < 4; ++j)
                    for (int i = 0; i < 4; ++i) {
                        const int x = std::min(bx * 4 + i, w - 1), y = std::min(by * 4 + j, h - 1);
                        std::memcpy(block + (j * 4 + i) * 4, &rgba[(std::size_t(y) * std::size_t(w) + std::size_t(x)) * 4], 4);
                    }
                std::uint8_t* out = &e.level.data[(std::size_t(by) * std::size_t(bw) + std::size_t(bx)) * 16];
                bc7enc_compress_block(out, block, &params);
                bc7decomp::color_rgba dec[16];
                bc7decomp::unpack_bc7(out, dec);
                for (int k = 0; k < 16; ++k)
                    for (int c = 0; c < 4; ++c) {
                        const double d = double(dec[k].m_comps[c]) - double(block[k * 4 + c]);
                        err[t] += d * d;
                    }
            }
        }
    };
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < threads; ++t) pool.emplace_back(work, t);
    for (auto& th : pool) th.join();
    for (double v : err) e.sqErr += v;
    e.samples = std::size_t(bw) * std::size_t(bh) * 64;
    return e;
}

std::uint8_t q8(float v) { return std::uint8_t(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f)); }

std::map<std::string, std::string> parseArgs(int argc, char** argv) {
    std::map<std::string, std::string> a;
    for (int i = 2; i + 1 < argc; i += 2) {
        if (std::strncmp(argv[i], "--", 2) != 0) die(std::string("unexpected argument ") + argv[i]);
        a[argv[i] + 2] = argv[i + 1];
    }
    return a;
}

std::string need(const std::map<std::string, std::string>& a, const char* key) {
    const auto it = a.find(key);
    if (it == a.end()) die(std::string("missing --") + key);
    return it->second;
}

void writeFile(const std::string& path, const std::vector<std::uint8_t>& bytes) {
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.data()), std::streamsize(bytes.size()));
    if (!out) die("cannot write " + path);
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2 || (std::strcmp(argv[1], "albedo") != 0 && std::strcmp(argv[1], "nra") != 0)) {
        std::fprintf(stderr, "usage: trailforge_texc albedo --color C --height H --size N --out F\n"
                             "       trailforge_texc nra --normal N --arm A --size N --out F\n");
        return 2;
    }
    const bool albedo = std::strcmp(argv[1], "albedo") == 0;
    const auto args = parseArgs(argc, argv);
    const int size = std::stoi(need(args, "size"));
    if (size < 4 || (size & (size - 1)) != 0) die("--size must be a power of two >= 4");
    bc7enc_compress_block_init();

    // Level 0 in linear working space: albedo RGB linear + height; or normal xyz (-1..1) + roughness, AO packed as
    // (nx, ny, nz) in px[0..2] and the roughness / AO in px[3] / a second plane.
    Plane base, extra; // extra: nra only (R roughness, G AO)
    if (albedo) {
        base = load(need(args, "color"), size);
        const Plane height = load(need(args, "height"), size);
        std::vector<float> hs;
        hs.reserve(height.px.size() / 4);
        for (std::size_t i = 0; i < height.px.size(); i += 4) hs.push_back(height.px[i]);
        std::vector<float> sorted = hs;
        std::sort(sorted.begin(), sorted.end());
        const float lo = sorted[sorted.size() / 100], hi = sorted[sorted.size() - 1 - sorted.size() / 100];
        const float scale = hi > lo ? 1.0f / (hi - lo) : 1.0f;
        for (std::size_t i = 0, k = 0; i < base.px.size(); i += 4, ++k) {
            for (int c = 0; c < 3; ++c) base.px[i + std::size_t(c)] = srgbToLinear(base.px[i + std::size_t(c)]);
            base.px[i + 3] = std::clamp((hs[k] - lo) * scale, 0.0f, 1.0f);
        }
        std::printf("height range %.3f..%.3f (1st..99th percentile) stretched to 0..1\n", lo, hi);
    } else {
        base = load(need(args, "normal"), size);
        const Plane arm = load(need(args, "arm"), size);
        extra.w = base.w;
        extra.h = base.h;
        extra.px.resize(base.px.size());
        for (std::size_t i = 0; i < base.px.size(); i += 4) {
            float n[3] = {base.px[i] * 2 - 1, base.px[i + 1] * 2 - 1, base.px[i + 2] * 2 - 1};
            const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            for (int c = 0; c < 3; ++c) base.px[i + std::size_t(c)] = len > 0 ? n[c] / len : (c == 2 ? 1.0f : 0.0f);
            base.px[i + 3] = 0.0f;
            extra.px[i] = arm.px[i + 1]; // roughness
            extra.px[i + 1] = arm.px[i]; // AO
            extra.px[i + 2] = 0.0f;
            extra.px[i + 3] = 0.0f;
        }
    }

    tf::ktx2::Image image;
    image.vkFormat = albedo ? tf::ktx2::kVkFormatBc7Srgb : tf::ktx2::kVkFormatBc7Unorm;
    image.width = image.height = std::uint32_t(size);
    double sqErr0 = 0.0;
    std::size_t samples0 = 0;
    // Down to 1 x 1: distant terrain must reach the average colour, a 4 x 4 last level would alias into dots.
    for (int level = 0; (size >> level) >= 1; ++level) {
        if (level > 0) {
            base = half(base);
            if (!albedo) extra = half(extra);
        }
        std::vector<std::uint8_t> rgba(std::size_t(base.w) * std::size_t(base.h) * 4);
        for (std::size_t i = 0; i < rgba.size(); i += 4) {
            const float* p = &base.px[i];
            if (albedo) {
                rgba[i] = q8(linearToSrgb(p[0]));
                rgba[i + 1] = q8(linearToSrgb(p[1]));
                rgba[i + 2] = q8(linearToSrgb(p[2]));
                rgba[i + 3] = q8(p[3]);
            } else {
                // Averaged normals are shorter where they disagree: renormalize and widen the roughness.
                const float len = std::max(std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]), 1e-4f);
                const float variance = std::clamp((1.0f - len) / len, 0.0f, 1.0f);
                const float rough = std::sqrt(std::min(1.0f, extra.px[i] * extra.px[i] + variance));
                rgba[i] = q8(p[0] / len * 0.5f + 0.5f);
                rgba[i + 1] = q8(p[1] / len * 0.5f + 0.5f);
                rgba[i + 2] = q8(rough);
                rgba[i + 3] = q8(extra.px[i + 1]);
            }
        }
        Encoded e = encode(rgba, base.w, base.h, albedo);
        if (level == 0) {
            sqErr0 = e.sqErr;
            samples0 = e.samples;
        }
        image.levels.push_back(std::move(e.level));
    }
    const std::vector<std::uint8_t> bytes = tf::ktx2::write(image, "trailforge_texc (bc7enc_rdo b9438627)");
    const std::string out = need(args, "out");
    writeFile(out, bytes);
    const double mse = sqErr0 / double(std::max<std::size_t>(samples0, 1));
    const double psnr = mse > 0 ? 10.0 * std::log10(255.0 * 255.0 / mse) : 99.0;
    std::printf("%s: %dx%d, %zu levels, %zu bytes, level 0 RGBA PSNR %.2f dB\n", out.c_str(), size, size,
                image.levels.size(), bytes.size(), psnr);
    return 0;
}
