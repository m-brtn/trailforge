// Shared helpers of the offline texture and model encoders (trailforge_texc, trailforge_modelc): image loading,
// box filters, BC7 encoding with error measurement, argument parsing. Header-only; the including translation unit
// defines STB_IMAGE_IMPLEMENTATION and kToolName (used in error messages) before including it.
#pragma once

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

namespace texcore {

struct Plane { // float RGBA, row-major
    int w = 0, h = 0;
    std::vector<float> px;
    float* at(int x, int y) { return &px[(std::size_t(y) * std::size_t(w) + std::size_t(x)) * 4]; }
    const float* at(int x, int y) const { return &px[(std::size_t(y) * std::size_t(w) + std::size_t(x)) * 4]; }
};

[[noreturn]] inline void die(const std::string& msg) {
    std::fprintf(stderr, "%s: %s\n", kToolName, msg.c_str());
    std::exit(1);
}

inline float srgbToLinear(float c) { return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f); }
inline float linearToSrgb(float c) {
    c = std::clamp(c, 0.0f, 1.0f);
    return c <= 0.0031308f ? c * 12.92f : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
}

// Loads an image as float RGBA 0..1 (channels replicated for grayscale), box-downsampled to size x size.
inline Plane load(const std::string& path, int size) {
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
inline Plane half(const Plane& p) {
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
inline Encoded encode(const std::vector<std::uint8_t>& rgba, int w, int h, bool perceptual) {
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

inline std::uint8_t q8(float v) { return std::uint8_t(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f)); }

inline std::map<std::string, std::string> parseArgs(int argc, char** argv) {
    std::map<std::string, std::string> a;
    for (int i = 2; i + 1 < argc; i += 2) {
        if (std::strncmp(argv[i], "--", 2) != 0) die(std::string("unexpected argument ") + argv[i]);
        a[argv[i] + 2] = argv[i + 1];
    }
    return a;
}

inline std::string need(const std::map<std::string, std::string>& a, const char* key) {
    const auto it = a.find(key);
    if (it == a.end()) die(std::string("missing --") + key);
    return it->second;
}

inline void writeFile(const std::string& path, const std::vector<std::uint8_t>& bytes) {
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(bytes.data()), std::streamsize(bytes.size()));
    if (!out) die("cannot write " + path);
}

} // namespace texcore
