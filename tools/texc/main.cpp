// trailforge_texc: encodes one terrain material map into a BC7 KTX2 file with a full mip chain (down to 1 x 1).
//
//   trailforge_texc albedo --color diff.jpg --height disp.jpg --size 4096 --out grass_albedo.ktx2
//       RGB: albedo (sRGB), A: height (stretched to 0..1 between the 1st and 99th percentile). BC7 sRGB.
//       Without --height, A is 1: level model materials (the model shader cuts alpha below 0.5).
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
#undef STB_IMAGE_IMPLEMENTATION

static const char* const kToolName = "trailforge_texc";
#include "TexCore.h"

using namespace texcore;

int main(int argc, char** argv) {
    if (argc < 2 || (std::strcmp(argv[1], "albedo") != 0 && std::strcmp(argv[1], "nra") != 0)) {
        std::fprintf(stderr, "usage: trailforge_texc albedo --color C [--height H] --size N --out F\n"
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
    if (albedo && !args.count("height")) {
        base = load(need(args, "color"), size);
        for (std::size_t i = 0; i < base.px.size(); i += 4) {
            for (int c = 0; c < 3; ++c) base.px[i + std::size_t(c)] = srgbToLinear(base.px[i + std::size_t(c)]);
            base.px[i + 3] = 1.0f;
        }
    } else if (albedo) {
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
