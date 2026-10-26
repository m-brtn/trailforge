#include "render/Renderer.h"

#include "core/TerrainTextureSet.h"
#include "render/GroundMaps.h"
#include "render/ShaderBlobs.h"
#include "render/TimeOfDay.h"
#include "sim/TireMarks.h"
#include "sim/BodyMud.h"
#include "sim/WheelSpray.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_log.h>
#include <SDL3/SDL_surface.h>
#include <imgui.h>
#include <imgui_impl_sdlgpu3.h>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

#if defined(__APPLE__)
extern "C" double tf_metal_fence_gpu_ms(const void* fence);
#endif

namespace tf {
namespace {

[[noreturn]] void fail(const std::string& what) {
    throw std::runtime_error(what + ": " + SDL_GetError());
}

enum VertexLayout { kLayoutLit = 0, kLayoutLine = 1, kLayoutNone = 2 };
enum BlendMode { kBlendNone = 0, kBlendAlpha = 1, kBlendPremultiplied = 2, kBlendAdd = 3, kBlendMax = 4 };
// Shadow casters: positive slope bias, no depth clip. Decals: pulled toward the camera so they win against the
// surface they lie on. The scene uses reversed depth (near 1, far 0, compare GREATER_OR_EQUAL) for precision over
// kilometres; the shadow maps keep the standard 0..1 depth.
enum DepthBias { kBiasNone = 0, kBiasShadow = 1, kBiasDecal = 2 };

constexpr float kMarkLift = 0.012f; // m: tire marks are drawn this far above the contact plane

constexpr float kGrassChunk = 2.4f;
constexpr int kGrassBucket = 32;           // chunks per bucket side (76.8 m)
constexpr float kTrampleWindow = 204.8f;   // m: side of the trample window on large ground maps
constexpr int kTerrainPatch = 64;          // cells per terrain chunk side
constexpr int kTerrainLevels = 4;          // steps 8, 4, 2, 1
constexpr float kGrassCells[4] = {0.2f, 0.15f, 0.1f, 0.075f};
constexpr int kGrassSegs[3] = {4, 2, 1};
// Outer edges of the two dense grass LODs: a longer grass distance only adds sparse far-LOD rings.
constexpr float kGrassLod0Max = 18.0f;
constexpr float kGrassLod1Max = 40.0f;
constexpr float kGrassNearCanopy[4] = {0.6f, 0.5f, 0.35f, 0.3f};
// Full quads per segment, so cut shapes (ears, seed heads, flower heads) get a wide top; the far LOD is one triangle.
constexpr int grassTriangles(int lod) { return lod >= 2 ? 1 : 2 * kGrassSegs[lod]; }
constexpr int kBloomLevels = 5;

SDL_GPUSampleCount sampleCount(int n) {
    return n >= 4 ? SDL_GPU_SAMPLECOUNT_4 : n == 2 ? SDL_GPU_SAMPLECOUNT_2 : SDL_GPU_SAMPLECOUNT_1;
}

struct Frustum {
    glm::vec4 planes[6];
    explicit Frustum(const glm::mat4& m) {
        auto row = [&](int i) { return glm::vec4(m[0][i], m[1][i], m[2][i], m[3][i]); };
        const glm::vec4 r0 = row(0), r1 = row(1), r2 = row(2), r3 = row(3);
        planes[0] = r3 + r0;
        planes[1] = r3 - r0;
        planes[2] = r3 + r1;
        planes[3] = r3 - r1;
        planes[4] = r2; // depth 0..1
        planes[5] = r3 - r2;
        for (glm::vec4& p : planes) p /= glm::length(glm::vec3(p));
    }
    bool sphere(const glm::vec3& c, float r) const {
        for (const glm::vec4& p : planes)
            if (glm::dot(glm::vec3(p), c) + p.w < -r) return false;
        return true;
    }
    bool box(const glm::vec3& bmin, const glm::vec3& bmax) const {
        for (const glm::vec4& p : planes) {
            const glm::vec3 v(p.x > 0 ? bmax.x : bmin.x, p.y > 0 ? bmax.y : bmin.y, p.z > 0 ? bmax.z : bmin.z);
            if (glm::dot(glm::vec3(p), v) + p.w < 0.0f) return false;
        }
        return true;
    }
};

// World-space bounding sphere of a draw (mesh AABB under the model transform).
void drawSphere(const DrawItem& d, glm::vec3& center, float& radius) {
    const glm::vec3 c = 0.5f * (d.mesh->boundsMin() + d.mesh->boundsMax());
    const glm::vec3 e = 0.5f * (d.mesh->boundsMax() - d.mesh->boundsMin());
    center = glm::vec3(d.model * glm::vec4(c, 1.0f));
    const float s = std::max({glm::length(glm::vec3(d.model[0])), glm::length(glm::vec3(d.model[1])),
                              glm::length(glm::vec3(d.model[2]))});
    radius = glm::length(e) * s;
}

} // namespace

GpuMesh& GpuMesh::operator=(GpuMesh&& o) noexcept {
    if (this != &o) {
        release();
        m_device = o.m_device;
        m_buffer = o.m_buffer;
        m_count = o.m_count;
        m_min = o.m_min;
        m_max = o.m_max;
        o.m_device = nullptr;
        o.m_buffer = nullptr;
        o.m_count = 0;
    }
    return *this;
}

void GpuMesh::release() {
    if (m_device && m_buffer) SDL_ReleaseGPUBuffer(m_device, m_buffer);
    m_buffer = nullptr;
    m_count = 0;
}

// ---------------------------------------------------------------- setup

Renderer::Renderer(SDL_Window* window, bool debugDevice) : m_window(window) {
    // Offer every format that was compiled into this build (shaders/scene.hlsl -> SPIR-V everywhere, metallib or MSL
    // on macOS, DXIL on Windows). SDL tries Metal, then D3D12, then Vulkan; SDL_GPU_DRIVER overrides the choice.
    const ShaderBlob* probe = findShaderBlob("lit_vs");
    if (!probe) throw std::runtime_error("no embedded shaders");
    SDL_GPUShaderFormat formats = 0;
    if (probe->spirv.data) formats |= SDL_GPU_SHADERFORMAT_SPIRV;
    if (probe->msl.data) formats |= SDL_GPU_SHADERFORMAT_MSL;
    if (probe->metallib.data) formats |= SDL_GPU_SHADERFORMAT_METALLIB;
    if (probe->dxil.data) formats |= SDL_GPU_SHADERFORMAT_DXIL;
    m_device = SDL_CreateGPUDevice(formats, debugDevice, nullptr);
    if (!m_device) fail("SDL_CreateGPUDevice");
    m_driver = SDL_GetGPUDeviceDriver(m_device);

    if (!SDL_ClaimWindowForGPUDevice(m_device, m_window)) {
        SDL_DestroyGPUDevice(m_device);
        fail("SDL_ClaimWindowForGPUDevice");
    }
    SDL_SetGPUSwapchainParameters(m_device, m_window, SDL_GPU_SWAPCHAINCOMPOSITION_SDR, SDL_GPU_PRESENTMODE_VSYNC);
    m_colorFormat = swapchainFormat();

    const SDL_GPUShaderFormat supported = SDL_GetGPUShaderFormats(m_device) & formats;
    if (supported & SDL_GPU_SHADERFORMAT_METALLIB) {
        m_shaderFormat = SDL_GPU_SHADERFORMAT_METALLIB;
        m_shaderFormatName = "metallib (precompiled)";
    } else if (supported & SDL_GPU_SHADERFORMAT_MSL) {
        m_shaderFormat = SDL_GPU_SHADERFORMAT_MSL;
        m_shaderFormatName = "MSL source (runtime compile)";
    } else if (supported & SDL_GPU_SHADERFORMAT_DXIL) {
        m_shaderFormat = SDL_GPU_SHADERFORMAT_DXIL;
        m_shaderFormatName = "DXIL";
    } else if (supported & SDL_GPU_SHADERFORMAT_SPIRV) {
        m_shaderFormat = SDL_GPU_SHADERFORMAT_SPIRV;
        m_shaderFormatName = "SPIR-V";
    } else {
        fail("No supported shader format");
    }
#if defined(__APPLE__)
    m_gpuTiming = std::strcmp(m_driver, "metal") == 0;
#endif

    if (!SDL_GPUTextureSupportsFormat(m_device, kDepthFormat, SDL_GPU_TEXTURETYPE_2D_ARRAY,
                                      SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER))
        fail("D32 depth textures are required");
    // Sample counts are not a ladder: Mesa lavapipe, for example, supports 1x and 4x but not 2x.
    m_maxMsaa = 1;
    for (int n : {2, 4}) {
        const bool ok = SDL_GPUTextureSupportsSampleCount(m_device, kHdrFormat, sampleCount(n)) &&
                        SDL_GPUTextureSupportsSampleCount(m_device, kDepthFormat, sampleCount(n));
        m_msaa2 = m_msaa2 || (ok && n == 2);
        if (ok) m_maxMsaa = n;
    }
    // SDL's Vulkan backend does not enable the depthBiasClamp device feature, so the clamp must stay 0 there.
    m_shadowBiasClamp = std::strcmp(m_driver, "vulkan") == 0 ? 0.0f : 0.01f;

    // Samplers.
    {
        SDL_GPUSamplerCreateInfo si{};
        si.min_filter = SDL_GPU_FILTER_LINEAR;
        si.mag_filter = SDL_GPU_FILTER_LINEAR;
        si.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST;
        si.address_mode_u = si.address_mode_v = si.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
        si.enable_compare = true;
        si.compare_op = SDL_GPU_COMPAREOP_LESS_OR_EQUAL;
        m_cmpSampler = SDL_CreateGPUSampler(m_device, &si);

        si.enable_compare = false;
        m_clampSampler = SDL_CreateGPUSampler(m_device, &si);

        si.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR;
        si.address_mode_u = si.address_mode_v = si.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
        si.enable_anisotropy = true;
        si.max_anisotropy = 8.0f;
        si.max_lod = 16.0f;
        m_repeatSampler = SDL_CreateGPUSampler(m_device, &si);
        m_terrainSampler = SDL_CreateGPUSampler(m_device, &si);
        if (!m_cmpSampler || !m_clampSampler || !m_repeatSampler || !m_terrainSampler) fail("samplers");
    }

    // Grass chunk storage buffer.
    {
        SDL_GPUBufferCreateInfo bi{};
        bi.usage = SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ;
        bi.size = kMaxGrassChunks * sizeof(glm::vec4);
        m_grassBuffer = SDL_CreateGPUBuffer(m_device, &bi);
        SDL_GPUTransferBufferCreateInfo ti{};
        ti.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
        ti.size = bi.size;
        m_grassTransfer = SDL_CreateGPUTransferBuffer(m_device, &ti);
        if (!m_grassBuffer || !m_grassTransfer) fail("grass buffers");
    }

    // 1x1 placeholders until setGround() is called.
    {
        const std::uint8_t px[4] = {128, 128, 128, 255};
        const float h = 0.0f;
        m_noise = makeTexture(1, 1, SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM, SDL_GPU_TEXTUREUSAGE_SAMPLER);
        m_mask = makeTexture(1, 1, SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM, SDL_GPU_TEXTUREUSAGE_SAMPLER);
        m_height = makeTexture(1, 1, SDL_GPU_TEXTUREFORMAT_R32_FLOAT, SDL_GPU_TEXTUREUSAGE_SAMPLER);
        uploadTexture(m_noise.tex, px, 1, 1, 4);
        const std::uint8_t noWheat[4] = {128, 128, 128, 0};
        uploadTexture(m_mask.tex, noWheat, 1, 1, 4);
        uploadTexture(m_height.tex, &h, 1, 1, 4);
        m_trample = makeTexture(1, 1, kTrampleFormat, SDL_GPU_TEXTUREUSAGE_SAMPLER | SDL_GPU_TEXTUREUSAGE_COLOR_TARGET);
        clearTrample();
        m_terrainAlbedo = makeTexture(1, 1, SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM, SDL_GPU_TEXTUREUSAGE_SAMPLER,
                                      SDL_GPU_SAMPLECOUNT_1, 1, 1, SDL_GPU_TEXTURETYPE_2D_ARRAY);
        m_terrainNra = makeTexture(1, 1, SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM, SDL_GPU_TEXTUREUSAGE_SAMPLER,
                                   SDL_GPU_SAMPLECOUNT_1, 1, 1, SDL_GPU_TEXTURETYPE_2D_ARRAY);
        uploadTexture(m_terrainAlbedo.tex, px, 1, 1, 4);
        uploadTexture(m_terrainNra.tex, px, 1, 1, 4);
    }

    m_settings = presetSettings(QualityPreset::High);
    m_settings.msaa = supportedMsaa(m_settings.msaa);
    createStaticPipelines();
    createScenePipelines();
}

Renderer::~Renderer() {
    if (!m_device) return;
    SDL_WaitForGPUIdle(m_device);
    for (PendingFence& f : m_fences) SDL_ReleaseGPUFence(m_device, f.fence);
    m_fences.clear();
    releaseTargets();
    releaseScenePipelines();
    for (SDL_GPUGraphicsPipeline* p : {m_shadowPipe, m_reflLitPipe, m_reflSkyPipe, m_bloomDownPipe, m_bloomUpPipe,
                                       m_tonemapPipe, m_fxaaPipe, m_treeShadowPipe, m_reflTreePipe, m_tramplePipe,
                                       m_blurDownPipe, m_blurUpPipe, m_blurCompositePipe, m_terrainShadowPipe})
        if (p) SDL_ReleaseGPUGraphicsPipeline(m_device, p);
    for (GpuMesh& m : m_treeMeshes) m.release();
    m_terrainPatch.release();
    releaseTexture(m_terrainHeight);
    releaseTexture(m_terrainSplat);
    releaseTexture(m_terrainAlbedo);
    releaseTexture(m_terrainNra);
    if (m_markBuffer) SDL_ReleaseGPUBuffer(m_device, m_markBuffer);
    if (m_markTransfer) SDL_ReleaseGPUTransferBuffer(m_device, m_markTransfer);
    if (m_sprayBuffer) SDL_ReleaseGPUBuffer(m_device, m_sprayBuffer);
    if (m_sprayTransfer) SDL_ReleaseGPUTransferBuffer(m_device, m_sprayTransfer);
    releaseTexture(m_trample);
    if (m_treeBuffer) SDL_ReleaseGPUBuffer(m_device, m_treeBuffer);
    if (m_treeTransfer) SDL_ReleaseGPUTransferBuffer(m_device, m_treeTransfer);
    releaseTexture(m_noise);
    releaseTexture(m_mask);
    releaseTexture(m_height);
    for (SDL_GPUSampler* s : {m_cmpSampler, m_repeatSampler, m_clampSampler, m_terrainSampler})
        if (s) SDL_ReleaseGPUSampler(m_device, s);
    if (m_grassBuffer) SDL_ReleaseGPUBuffer(m_device, m_grassBuffer);
    if (m_grassTransfer) SDL_ReleaseGPUTransferBuffer(m_device, m_grassTransfer);
    if (m_lineBuffer) SDL_ReleaseGPUBuffer(m_device, m_lineBuffer);
    if (m_lineTransfer) SDL_ReleaseGPUTransferBuffer(m_device, m_lineTransfer);
    SDL_ReleaseWindowFromGPUDevice(m_device, m_window);
    SDL_DestroyGPUDevice(m_device);
}

SDL_GPUTextureFormat Renderer::swapchainFormat() const {
    return SDL_GetGPUSwapchainTextureFormat(m_device, m_window);
}

void Renderer::setVSync(bool on) {
    m_vsync = on;
    SDL_SetGPUSwapchainParameters(m_device, m_window, SDL_GPU_SWAPCHAINCOMPOSITION_SDR,
                                  on ? SDL_GPU_PRESENTMODE_VSYNC
                                     : (SDL_WindowSupportsGPUPresentMode(m_device, m_window, SDL_GPU_PRESENTMODE_IMMEDIATE)
                                            ? SDL_GPU_PRESENTMODE_IMMEDIATE
                                            : SDL_GPU_PRESENTMODE_VSYNC));
}

int Renderer::supportedMsaa(int n) const {
    if (n >= 4 && m_maxMsaa >= 4) return 4;
    if (n >= 2 && m_msaa2) return 2;
    return 1;
}

void Renderer::setSettings(const RenderSettings& in) {
    RenderSettings s = in;
    s.renderScale = std::clamp(s.renderScale, 0.4f, 1.0f);
    s.msaa = supportedMsaa(s.msaa >= 4 ? 4 : s.msaa >= 2 ? 2 : 1);
    s.shadowCascades = std::clamp(s.shadowCascades, 0, 4);
    s.shadowMapSize = std::clamp(s.shadowMapSize, 512, 4096);
    s.shadowDistance = std::clamp(s.shadowDistance, 10.0f, 300.0f);
    s.shadowFilter = std::clamp(s.shadowFilter, 0, 2);
    s.reflectionScale = std::clamp(s.reflectionScale, 0.2f, 1.0f);
    s.grassLevel = std::clamp(s.grassLevel, 0, 4);
    s.grassDistance = std::clamp(s.grassDistance, 5.0f, 250.0f);
    s.treeDetailDistance = std::clamp(s.treeDetailDistance, 100.0f, 2000.0f);
    s.treeDistance = std::clamp(s.treeDistance, s.treeDetailDistance, 6000.0f);
    s.flowerDensity = std::clamp(s.flowerDensity, 0.0f, 1.0f);
    s.exposure = std::clamp(s.exposure, 0.1f, 8.0f);
    s.timeOfDay = std::fmod(std::fmod(s.timeOfDay, 24.0f) + 24.0f, 24.0f);
    s.sunHeadingDeg = std::fmod(std::fmod(s.sunHeadingDeg, 360.0f) + 360.0f, 360.0f);
    const bool targets = s.renderScale != m_settings.renderScale || s.msaa != m_settings.msaa ||
                         s.shadowCascades != m_settings.shadowCascades || s.shadowMapSize != m_settings.shadowMapSize ||
                         s.reflectionScale != m_settings.reflectionScale || s.fxaa != m_settings.fxaa ||
                         s.reflections != m_settings.reflections || s.bloom != m_settings.bloom;
    m_settings = s;
    if (targets) m_targetsDirty = true;
}

SDL_GPUShader* Renderer::createShader(const char* entry, SDL_GPUShaderStage stage, std::uint32_t uniformBuffers,
                                      std::uint32_t samplers, std::uint32_t storageBuffers) {
    SDL_GPUShaderCreateInfo info{};
    info.entrypoint = entry;
    info.format = m_shaderFormat;
    info.stage = stage;
    info.num_uniform_buffers = uniformBuffers;
    info.num_samplers = samplers;
    info.num_storage_buffers = storageBuffers;
    const ShaderBlob* blob = findShaderBlob(entry);
    if (!blob) throw std::runtime_error(std::string("shader entry point not embedded: ") + entry);
    const ShaderCode code = m_shaderFormat == SDL_GPU_SHADERFORMAT_METALLIB ? blob->metallib
                            : m_shaderFormat == SDL_GPU_SHADERFORMAT_MSL    ? blob->msl
                            : m_shaderFormat == SDL_GPU_SHADERFORMAT_DXIL   ? blob->dxil
                                                                            : blob->spirv;
    info.code = code.data;
    info.code_size = code.size;
    SDL_GPUShader* s = SDL_CreateGPUShader(m_device, &info);
    if (!s) fail(std::string("SDL_CreateGPUShader ") + entry);
    return s;
}

SDL_GPUGraphicsPipeline* Renderer::createPipeline(const char* vsName, const char* fsName, std::uint32_t vsUniforms,
                                                  std::uint32_t fsUniforms, std::uint32_t vsSamplers,
                                                  std::uint32_t fsSamplers, std::uint32_t vsStorage, int vertexLayout,
                                                  SDL_GPUTextureFormat color, SDL_GPUSampleCount samples,
                                                  bool depthTest, bool depthWrite, SDL_GPUCullMode cull,
                                                  SDL_GPUFrontFace front, int blend, SDL_GPUPrimitiveType prim,
                                                  int depthBias) {
    SDL_GPUShader* vs = createShader(vsName, SDL_GPU_SHADERSTAGE_VERTEX, vsUniforms, vsSamplers, vsStorage);
    SDL_GPUShader* fs = createShader(fsName, SDL_GPU_SHADERSTAGE_FRAGMENT, fsUniforms, fsSamplers);

    SDL_GPUVertexBufferDescription vb{};
    vb.slot = 0;
    vb.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    SDL_GPUVertexAttribute attrs[4]{};
    std::uint32_t numAttrs = 0;
    if (vertexLayout == kLayoutLit) {
        vb.pitch = sizeof(LitVertex);
        attrs[0] = {0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, std::uint32_t(offsetof(LitVertex, position))};
        attrs[1] = {1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, std::uint32_t(offsetof(LitVertex, normal))};
        attrs[2] = {2, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, std::uint32_t(offsetof(LitVertex, color))};
        attrs[3] = {3, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, std::uint32_t(offsetof(LitVertex, material))};
        numAttrs = 4;
    } else if (vertexLayout == kLayoutLine) {
        vb.pitch = sizeof(LineVertex);
        attrs[0] = {0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, std::uint32_t(offsetof(LineVertex, position))};
        attrs[1] = {1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, std::uint32_t(offsetof(LineVertex, color))};
        numAttrs = 2;
    }

    SDL_GPUColorTargetDescription ct{};
    ct.format = color;
    SDL_GPUColorTargetBlendState& b = ct.blend_state;
    if (blend != kBlendNone) {
        b.enable_blend = true;
        b.color_blend_op = SDL_GPU_BLENDOP_ADD;
        b.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
        if (blend == kBlendAlpha) {
            b.src_color_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA;
            b.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
            b.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
            b.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
        } else if (blend == kBlendPremultiplied) {
            b.src_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
            b.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
            b.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
            b.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
        } else {
            b.src_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
            b.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
            b.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
            b.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
            if (blend == kBlendMax) b.color_blend_op = b.alpha_blend_op = SDL_GPU_BLENDOP_MAX;
        }
    }

    SDL_GPUGraphicsPipelineCreateInfo pi{};
    pi.vertex_shader = vs;
    pi.fragment_shader = fs;
    if (numAttrs > 0) {
        pi.vertex_input_state.vertex_buffer_descriptions = &vb;
        pi.vertex_input_state.num_vertex_buffers = 1;
        pi.vertex_input_state.vertex_attributes = attrs;
        pi.vertex_input_state.num_vertex_attributes = numAttrs;
    }
    pi.primitive_type = prim;
    pi.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;
    pi.rasterizer_state.cull_mode = cull;
    pi.rasterizer_state.front_face = front;
    // Shadow casters behind the light near plane are clamped.
    pi.rasterizer_state.enable_depth_clip = depthBias != kBiasShadow;
    if (depthBias == kBiasShadow) {
        pi.rasterizer_state.enable_depth_bias = true;
        pi.rasterizer_state.depth_bias_constant_factor = 1.0f;
        pi.rasterizer_state.depth_bias_slope_factor = 1.75f;
        pi.rasterizer_state.depth_bias_clamp = m_shadowBiasClamp;
    } else if (depthBias == kBiasDecal) {
        pi.rasterizer_state.enable_depth_bias = true;
        pi.rasterizer_state.depth_bias_constant_factor = 2.0f; // toward the camera in reversed depth
        pi.rasterizer_state.depth_bias_slope_factor = 2.0f;
        pi.rasterizer_state.depth_bias_clamp = 0.0f;
    }
    pi.multisample_state.sample_count = samples;
    pi.depth_stencil_state.enable_depth_test = depthTest;
    pi.depth_stencil_state.enable_depth_write = depthWrite;
    pi.depth_stencil_state.compare_op =
        depthBias == kBiasShadow ? SDL_GPU_COMPAREOP_LESS_OR_EQUAL : SDL_GPU_COMPAREOP_GREATER_OR_EQUAL;
    pi.target_info.color_target_descriptions = color != SDL_GPU_TEXTUREFORMAT_INVALID ? &ct : nullptr;
    pi.target_info.num_color_targets = color != SDL_GPU_TEXTUREFORMAT_INVALID ? 1 : 0;
    const bool hasDepth = depthTest || depthWrite;
    pi.target_info.depth_stencil_format = hasDepth ? kDepthFormat : SDL_GPU_TEXTUREFORMAT_INVALID;
    pi.target_info.has_depth_stencil_target = hasDepth;
    SDL_GPUGraphicsPipeline* p = SDL_CreateGPUGraphicsPipeline(m_device, &pi);
    SDL_ReleaseGPUShader(m_device, vs);
    SDL_ReleaseGPUShader(m_device, fs);
    if (!p) fail(std::string("pipeline ") + vsName + "/" + fsName);
    return p;
}

void Renderer::createStaticPipelines() {
    const auto none = SDL_GPU_TEXTUREFORMAT_INVALID;
    const auto s1 = SDL_GPU_SAMPLECOUNT_1;
    const auto ccw = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    m_shadowPipe = createPipeline("shadow_vs", "shadow_fs", 2, 0, 0, 0, 0, kLayoutLit, none, s1, true, true,
                                  SDL_GPU_CULLMODE_NONE, ccw, kBlendNone, SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, kBiasShadow);
    // Mirrored view: the winding flips, so front faces are clockwise.
    m_reflLitPipe = createPipeline("lit_vs", "lit_fs", 2, 1, 0, 3, 0, kLayoutLit, kHdrFormat, s1, true, true,
                                   SDL_GPU_CULLMODE_BACK, SDL_GPU_FRONTFACE_CLOCKWISE, kBlendNone);
    m_reflSkyPipe = createPipeline("fullscreen_vs", "sky_fs", 0, 1, 0, 1, 0, kLayoutNone, kHdrFormat, s1, true, false,
                                   SDL_GPU_CULLMODE_NONE, ccw, kBlendNone);
    m_bloomDownPipe = createPipeline("fullscreen_vs", "bloom_down_fs", 0, 1, 0, 1, 0, kLayoutNone, kHdrFormat, s1,
                                     false, false, SDL_GPU_CULLMODE_NONE, ccw, kBlendNone);
    m_bloomUpPipe = createPipeline("fullscreen_vs", "bloom_up_fs", 0, 1, 0, 1, 0, kLayoutNone, kHdrFormat, s1, false,
                                   false, SDL_GPU_CULLMODE_NONE, ccw, kBlendAdd);
    m_tonemapPipe = createPipeline("fullscreen_vs", "tonemap_fs", 0, 1, 0, 2, 0, kLayoutNone, m_colorFormat, s1, false,
                                   false, SDL_GPU_CULLMODE_NONE, ccw, kBlendNone);
    m_fxaaPipe = createPipeline("fullscreen_vs", "fxaa_fs", 0, 1, 0, 1, 0, kLayoutNone, m_colorFormat, s1, false, false,
                                SDL_GPU_CULLMODE_NONE, ccw, kBlendNone);
    m_treeShadowPipe = createPipeline("tree_shadow_vs", "shadow_fs", 2, 0, 0, 0, 1, kLayoutLit, none, s1, true, true,
                                      SDL_GPU_CULLMODE_NONE, ccw, kBlendNone, SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, kBiasShadow);
    m_reflTreePipe = createPipeline("tree_vs", "lit_fs", 2, 1, 0, 3, 1, kLayoutLit, kHdrFormat, s1, true, true,
                                    SDL_GPU_CULLMODE_BACK, SDL_GPU_FRONTFACE_CLOCKWISE, kBlendNone);
    m_tramplePipe = createPipeline("mark_mask_vs", "mark_mask_fs", 2, 0, 0, 0, 1, kLayoutNone, kTrampleFormat, s1,
                                   false, false, SDL_GPU_CULLMODE_NONE, ccw, kBlendMax);
    m_blurDownPipe = createPipeline("fullscreen_vs", "blur_down_fs", 0, 1, 0, 1, 0, kLayoutNone, m_colorFormat, s1,
                                    false, false, SDL_GPU_CULLMODE_NONE, ccw, kBlendNone);
    m_blurUpPipe = createPipeline("fullscreen_vs", "blur_up_fs", 0, 1, 0, 1, 0, kLayoutNone, m_colorFormat, s1, false,
                                  false, SDL_GPU_CULLMODE_NONE, ccw, kBlendNone);
    m_blurCompositePipe = createPipeline("fullscreen_vs", "blur_composite_fs", 0, 1, 0, 2, 0, kLayoutNone,
                                         m_colorFormat, s1, false, false, SDL_GPU_CULLMODE_NONE, ccw, kBlendNone);
    m_terrainShadowPipe = createPipeline("terrain_shadow_vs", "shadow_fs", 2, 0, 1, 0, 0, kLayoutLit, none, s1, true,
                                         true, SDL_GPU_CULLMODE_NONE, ccw, kBlendNone,
                                         SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, kBiasShadow);
}

void Renderer::createScenePipelines() {
    const SDL_GPUSampleCount sc = sampleCount(m_settings.msaa);
    const auto ccw = SDL_GPU_FRONTFACE_COUNTER_CLOCKWISE;
    m_litPipe = createPipeline("lit_vs", "lit_fs", 2, 1, 0, 3, 0, kLayoutLit, kHdrFormat, sc, true, true,
                               SDL_GPU_CULLMODE_BACK, ccw, kBlendNone);
    m_skyPipe = createPipeline("fullscreen_vs", "sky_fs", 0, 1, 0, 1, 0, kLayoutNone, kHdrFormat, sc, true, false,
                               SDL_GPU_CULLMODE_NONE, ccw, kBlendNone);
    m_grassPipe = createPipeline("grass_vs", "grass_fs", 2, 1, 4, 1, 1, kLayoutNone, kHdrFormat, sc, true, true,
                                 SDL_GPU_CULLMODE_NONE, ccw, kBlendNone);
    m_markPipe = createPipeline("mark_vs", "mark_fs", 2, 2, 0, 3, 1, kLayoutNone, kHdrFormat, sc, true, false,
                                SDL_GPU_CULLMODE_NONE, ccw, kBlendAlpha, SDL_GPU_PRIMITIVETYPE_TRIANGLELIST, kBiasDecal);
    m_sprayPipe = createPipeline("spray_vs", "spray_fs", 2, 1, 0, 3, 1, kLayoutNone, kHdrFormat, sc, true, false,
                                 SDL_GPU_CULLMODE_NONE, ccw, kBlendAlpha);
    m_waterPipe = createPipeline("lit_vs", "water_fs", 2, 1, 0, 4, 0, kLayoutLit, kHdrFormat, sc, true, false,
                                 SDL_GPU_CULLMODE_NONE, ccw, kBlendPremultiplied);
    m_linePipe = createPipeline("line_vs", "line_fs", 1, 1, 0, 0, 0, kLayoutLine, kHdrFormat, sc, true, false,
                                SDL_GPU_CULLMODE_NONE, ccw, kBlendAlpha, SDL_GPU_PRIMITIVETYPE_LINELIST);
    m_lineOnTopPipe = createPipeline("line_vs", "line_fs", 1, 1, 0, 0, 0, kLayoutLine, kHdrFormat, sc, false, false,
                                     SDL_GPU_CULLMODE_NONE, ccw, kBlendAlpha, SDL_GPU_PRIMITIVETYPE_LINELIST);
    m_treePipe = createPipeline("tree_vs", "lit_fs", 2, 1, 0, 3, 1, kLayoutLit, kHdrFormat, sc, true, true,
                                SDL_GPU_CULLMODE_BACK, ccw, kBlendNone);
    // Skirts hang down from both sides of a chunk edge, so the terrain is drawn without culling.
    m_terrainPipe = createPipeline("terrain_vs", "terrain_fs", 2, 2, 2, 5, 0, kLayoutLit, kHdrFormat, sc, true, true,
                                   SDL_GPU_CULLMODE_NONE, ccw, kBlendNone);
    m_sceneMsaa = m_settings.msaa;
}

void Renderer::releaseScenePipelines() {
    for (SDL_GPUGraphicsPipeline** p :
         {&m_litPipe, &m_skyPipe, &m_grassPipe, &m_waterPipe, &m_linePipe, &m_lineOnTopPipe, &m_treePipe, &m_markPipe,
          &m_sprayPipe, &m_terrainPipe}) {
        if (*p) SDL_ReleaseGPUGraphicsPipeline(m_device, *p);
        *p = nullptr;
    }
    m_sceneMsaa = 0;
}

Renderer::Texture Renderer::makeTexture(std::uint32_t w, std::uint32_t h, SDL_GPUTextureFormat fmt,
                                        SDL_GPUTextureUsageFlags usage, SDL_GPUSampleCount samples,
                                        std::uint32_t layers, std::uint32_t levels, SDL_GPUTextureType type) {
    SDL_GPUTextureCreateInfo ti{};
    ti.type = type;
    ti.format = fmt;
    ti.usage = usage;
    ti.width = std::max<std::uint32_t>(w, 1);
    ti.height = std::max<std::uint32_t>(h, 1);
    ti.layer_count_or_depth = layers;
    ti.num_levels = levels;
    ti.sample_count = samples;
    Texture t;
    t.tex = SDL_CreateGPUTexture(m_device, &ti);
    if (!t.tex) fail("texture");
    t.w = ti.width;
    t.h = ti.height;
    return t;
}

void Renderer::releaseTexture(Texture& t) {
    if (t.tex) SDL_ReleaseGPUTexture(m_device, t.tex);
    t = {};
}

void Renderer::releaseTargets() {
    releaseTexture(m_hdrMsaa);
    releaseTexture(m_hdr);
    releaseTexture(m_depth);
    releaseTexture(m_reflColor);
    releaseTexture(m_reflDepth);
    releaseTexture(m_shadow);
    for (Texture& t : m_bloom) releaseTexture(t);
    m_bloom.clear();
    releaseTexture(m_ldr);
    releaseTexture(m_blurSrc);
    for (Texture& t : m_blur) releaseTexture(t);
    m_blur.clear();
}

void Renderer::ensureTargets(std::uint32_t outW, std::uint32_t outH) {
    if (!m_targetsDirty && outW == m_outW && outH == m_outH && m_sceneMsaa == m_settings.msaa) return;
    SDL_WaitForGPUIdle(m_device);
    if (m_sceneMsaa != m_settings.msaa) {
        releaseScenePipelines();
        createScenePipelines();
    }
    releaseTargets();
    const RenderSettings& s = m_settings;
    const std::uint32_t rw = std::max<std::uint32_t>(16, std::uint32_t(std::lround(outW * s.renderScale)));
    const std::uint32_t rh = std::max<std::uint32_t>(16, std::uint32_t(std::lround(outH * s.renderScale)));
    const SDL_GPUSampleCount sc = sampleCount(s.msaa);
    const auto colorRT = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET;
    const auto sampled = SDL_GPU_TEXTUREUSAGE_SAMPLER;
    const auto depthRT = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;
    if (s.msaa > 1) m_hdrMsaa = makeTexture(rw, rh, kHdrFormat, colorRT, sc);
    m_hdr = makeTexture(rw, rh, kHdrFormat, colorRT | sampled);
    m_depth = makeTexture(rw, rh, kDepthFormat, depthRT, sc);
    if (s.reflections) {
        const std::uint32_t fw = std::uint32_t(std::lround(rw * s.reflectionScale));
        const std::uint32_t fh = std::uint32_t(std::lround(rh * s.reflectionScale));
        m_reflColor = makeTexture(fw, fh, kHdrFormat, colorRT | sampled);
        m_reflDepth = makeTexture(fw, fh, kDepthFormat, depthRT);
    }
    m_shadow = makeTexture(std::uint32_t(s.shadowMapSize), std::uint32_t(s.shadowMapSize), kDepthFormat,
                           depthRT | sampled, SDL_GPU_SAMPLECOUNT_1, std::uint32_t(std::max(1, s.shadowCascades)), 1,
                           SDL_GPU_TEXTURETYPE_2D_ARRAY);
    if (s.bloom) {
        std::uint32_t bw = rw, bh = rh;
        for (int i = 0; i < kBloomLevels; ++i) {
            bw = std::max<std::uint32_t>(1, bw / 2);
            bh = std::max<std::uint32_t>(1, bh / 2);
            m_bloom.push_back(makeTexture(bw, bh, kHdrFormat, colorRT | sampled));
        }
    }
    if (s.fxaa) m_ldr = makeTexture(outW, outH, m_colorFormat, colorRT | sampled);
    // Menu blur: the finished frame, then a 1/2 .. 1/16 chain.
    m_blurSrc = makeTexture(outW, outH, m_colorFormat, colorRT | sampled);
    for (std::uint32_t i = 1, bw = outW, bh = outH; i <= kBlurLevels; ++i) {
        bw = std::max<std::uint32_t>(1, bw / 2);
        bh = std::max<std::uint32_t>(1, bh / 2);
        m_blur.push_back(makeTexture(bw, bh, m_colorFormat, colorRT | sampled));
    }
    m_outW = outW;
    m_outH = outH;
    m_stats.renderW = rw;
    m_stats.renderH = rh;
    m_stats.outputW = outW;
    m_stats.outputH = outH;
    ++m_targetCreations;
    m_targetsDirty = false;
}

void Renderer::uploadTexture(SDL_GPUTexture* tex, const void* data, std::uint32_t w, std::uint32_t h,
                             std::uint32_t bytesPerPixel) {
    const std::uint32_t bytes = w * h * bytesPerPixel;
    SDL_GPUTransferBufferCreateInfo ti{};
    ti.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    ti.size = bytes;
    SDL_GPUTransferBuffer* tb = SDL_CreateGPUTransferBuffer(m_device, &ti);
    if (!tb) fail("texture transfer buffer");
    std::memcpy(SDL_MapGPUTransferBuffer(m_device, tb, false), data, bytes);
    SDL_UnmapGPUTransferBuffer(m_device, tb);
    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_device);
    SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);
    SDL_GPUTextureTransferInfo src{};
    src.transfer_buffer = tb;
    src.pixels_per_row = w;
    src.rows_per_layer = h;
    SDL_GPUTextureRegion dst{};
    dst.texture = tex;
    dst.w = w;
    dst.h = h;
    dst.d = 1;
    SDL_UploadToGPUTexture(copy, &src, &dst, false);
    SDL_EndGPUCopyPass(copy);
    SDL_SubmitGPUCommandBuffer(cmd);
    SDL_ReleaseGPUTransferBuffer(m_device, tb);
}

SDL_GPUTexture* Renderer::createImage(const void* rgba, std::uint32_t w, std::uint32_t h) {
    std::uint32_t levels = 1;
    while ((std::max(w, h) >> levels) > 0) ++levels;
    // Mip generation needs the texture to be a color target as well.
    Texture t = makeTexture(w, h, SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
                            SDL_GPU_TEXTUREUSAGE_SAMPLER | SDL_GPU_TEXTUREUSAGE_COLOR_TARGET, SDL_GPU_SAMPLECOUNT_1, 1,
                            levels);
    uploadTexture(t.tex, rgba, w, h, 4);
    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_device);
    SDL_GenerateMipmapsForGPUTexture(cmd, t.tex);
    SDL_SubmitGPUCommandBuffer(cmd);
    return t.tex;
}

void Renderer::releaseImage(SDL_GPUTexture* tex) {
    if (tex) SDL_ReleaseGPUTexture(m_device, tex);
}

void Renderer::setGround(const GroundMaps& maps, float waterLevel) {
    SDL_WaitForGPUIdle(m_device);
    releaseTexture(m_noise);
    releaseTexture(m_mask);
    releaseTexture(m_height);
    const std::uint32_t ns = std::uint32_t(maps.noiseSize);
    std::uint32_t levels = 1;
    while ((ns >> levels) > 0) ++levels;
    m_noise = makeTexture(ns, ns, SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
                          SDL_GPU_TEXTUREUSAGE_SAMPLER | SDL_GPU_TEXTUREUSAGE_COLOR_TARGET, SDL_GPU_SAMPLECOUNT_1, 1,
                          levels);
    uploadTexture(m_noise.tex, maps.noise.data(), ns, ns, 4);
    {
        SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_device);
        SDL_GenerateMipmapsForGPUTexture(cmd, m_noise.tex);
        SDL_SubmitGPUCommandBuffer(cmd);
    }
    const std::uint32_t w = std::uint32_t(maps.width), h = std::uint32_t(maps.height);
    m_mask = makeTexture(w, h, SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM, SDL_GPU_TEXTUREUSAGE_SAMPLER);
    uploadTexture(m_mask.tex, maps.mask.data(), w, h, 4);
    m_height = makeTexture(w, h, SDL_GPU_TEXTUREFORMAT_R32_FLOAT, SDL_GPU_TEXTUREUSAGE_SAMPLER);
    uploadTexture(m_height.tex, maps.heights.data(), w, h, 4);
    // Texel centers sit on the grid points: uv = (xz - x0 + texel/2) / (n * texel).
    const float half = maps.texel * 0.5f;
    m_groundRect = glm::vec4(maps.x0 - half, maps.z0 - half, 1.0f / (float(w) * maps.texel),
                             1.0f / (float(h) * maps.texel));
    m_waterLevel = waterLevel;
    // The trample mask covers the same rect at a finer texel; on large maps it is a window around the camera that
    // is re-rendered from the mark ring when it moves.
    releaseTexture(m_trample);
    const float sizeX = float(w) * maps.texel, sizeZ = float(h) * maps.texel;
    m_trampleWindowed = std::max(sizeX, sizeZ) / kTrampleTexel > 4096.0f;
    const float tw = m_trampleWindowed ? kTrampleWindow : sizeX, th = m_trampleWindowed ? kTrampleWindow : sizeZ;
    m_trample = makeTexture(std::uint32_t(std::ceil(tw / kTrampleTexel)), std::uint32_t(std::ceil(th / kTrampleTexel)),
                            kTrampleFormat, SDL_GPU_TEXTUREUSAGE_SAMPLER | SDL_GPU_TEXTUREUSAGE_COLOR_TARGET);
    m_trampleRect = m_trampleWindowed ? glm::vec4(-1e6f, -1e6f, 1.0f / tw, 1.0f / th) : m_groundRect;
    m_trampleCenter = glm::vec2(1e9f);
    clearTrample();

    // Grass chunks: aligned to multiples of the chunk size so blade ids are stable on every density grid, sorted
    // into buckets for culling.
    m_grassChunks.clear();
    m_grassBuckets.clear();
    const float xMax = maps.x0 + maps.sizeX(), zMax = maps.z0 + maps.sizeZ();
    const int i0 = int(std::ceil(maps.x0 / kGrassChunk)), i1 = int(std::floor(xMax / kGrassChunk)) - 1;
    const int j0 = int(std::ceil(maps.z0 / kGrassChunk)), j1 = int(std::floor(zMax / kGrassChunk)) - 1;
    const int kS = (i1 - i0) * (j1 - j0) > 200000 ? 4 : 8; // samples per chunk side for the bounds and cover
    for (int bj = j0; bj <= j1; bj += kGrassBucket) {
        for (int bi = i0; bi <= i1; bi += kGrassBucket) {
            GrassBucket bucket{1e9f, 1e9f, -1e9f, -1e9f, std::uint32_t(m_grassChunks.size()), 0};
            for (int j = bj; j <= std::min(j1, bj + kGrassBucket - 1); ++j) {
                for (int i = bi; i <= std::min(i1, bi + kGrassBucket - 1); ++i) {
                    const float x0 = float(i) * kGrassChunk, z0 = float(j) * kGrassChunk;
                    float maxGrass = 0.0f, minY = 1e9f, maxY = -1e9f;
                    for (int b = 0; b <= kS; ++b) {
                        for (int a = 0; a <= kS; ++a) {
                            const float x = x0 + kGrassChunk * float(a) / float(kS), z = z0 + kGrassChunk * float(b) / float(kS);
                            maxGrass = std::max(maxGrass, maps.grassAt(x, z));
                            const float y = maps.heightAt(x, z);
                            minY = std::min(minY, y);
                            maxY = std::max(maxY, y);
                        }
                    }
                    if (maxGrass <= 0.03f) continue;
                    m_grassChunks.push_back({x0, z0, minY - 0.1f, maxY + 1.3f});
                    bucket.x0 = std::min(bucket.x0, x0);
                    bucket.z0 = std::min(bucket.z0, z0);
                    bucket.x1 = std::max(bucket.x1, x0 + kGrassChunk);
                    bucket.z1 = std::max(bucket.z1, z0 + kGrassChunk);
                }
            }
            bucket.count = std::uint32_t(m_grassChunks.size()) - bucket.first;
            if (bucket.count > 0) m_grassBuckets.push_back(bucket);
        }
    }
    SDL_Log("TrailForge: ground maps %dx%d, %zu grass chunks in %zu buckets", maps.width, maps.height,
            m_grassChunks.size(), m_grassBuckets.size());
}

void Renderer::setTerrain(const TerrainSource* src) {
    SDL_WaitForGPUIdle(m_device);
    releaseTexture(m_terrainHeight);
    releaseTexture(m_terrainSplat);
    m_terrainPatch.release();
    for (auto& level : m_terrainNodes) level.clear();
    m_terrainSamples = 0;
    m_terrainRoots = 0;
    if (!src || src->samples < 2) return;
    const int n = src->samples, cells = n - 1;
    const int rootCells = kTerrainPatch << (kTerrainLevels - 1);
    if (cells % rootCells != 0) throw std::runtime_error("terrain: samples - 1 must be a multiple of 512");
    m_terrainSamples = n;
    m_terrainOrigin = src->origin;
    m_terrainCell = src->cell;
    m_terrainRoots = cells / rootCells;
    m_terrainHeight = makeTexture(std::uint32_t(n), std::uint32_t(n), SDL_GPU_TEXTUREFORMAT_R32_FLOAT,
                                  SDL_GPU_TEXTUREUSAGE_SAMPLER);
    uploadTexture(m_terrainHeight.tex, src->heights, std::uint32_t(n), std::uint32_t(n), 4);
    m_terrainSplat = makeTexture(std::uint32_t(cells), std::uint32_t(cells), SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
                                 SDL_GPU_TEXTUREUSAGE_SAMPLER);
    uploadTexture(m_terrainSplat.tex, src->splat, std::uint32_t(cells), std::uint32_t(cells), 4);

    // Height bounds per node: finest level (step 1, 64 cells) from the samples, coarser levels from their children.
    for (int level = kTerrainLevels - 1; level >= 0; --level) {
        const int side = m_terrainRoots << level;
        std::vector<TerrainNode>& nodes = m_terrainNodes[std::size_t(level)];
        nodes.assign(std::size_t(side) * std::size_t(side), {1e9f, -1e9f});
        for (int nj = 0; nj < side; ++nj)
            for (int ni = 0; ni < side; ++ni) {
                TerrainNode& node = nodes[std::size_t(nj) * std::size_t(side) + std::size_t(ni)];
                if (level == kTerrainLevels - 1) {
                    for (int j = nj * kTerrainPatch; j <= (nj + 1) * kTerrainPatch; ++j)
                        for (int i = ni * kTerrainPatch; i <= (ni + 1) * kTerrainPatch; ++i) {
                            const float y = src->heights[std::size_t(j) * std::size_t(n) + std::size_t(i)];
                            node.minY = std::min(node.minY, y);
                            node.maxY = std::max(node.maxY, y);
                        }
                } else {
                    const auto& fine = m_terrainNodes[std::size_t(level + 1)];
                    for (int c = 0; c < 4; ++c) {
                        const TerrainNode& ch = fine[std::size_t(nj * 2 + c / 2) * std::size_t(side * 2) + std::size_t(ni * 2 + c % 2)];
                        node.minY = std::min(node.minY, ch.minY);
                        node.maxY = std::max(node.maxY, ch.maxY);
                    }
                }
            }
    }

    // Geometric error per node: the step grid interpolated over its cells (same diagonal as the patch) against every
    // sample inside, then the maximum with the children, so a coarse node is never more accurate than its subtree.
    const float* hs = src->heights;
    auto H = [&](int i, int j) { return hs[std::size_t(j) * std::size_t(n) + std::size_t(i)]; };
    for (int level = kTerrainLevels - 2; level >= 0; --level) {
        const int side = m_terrainRoots << level;
        const int step = 1 << (kTerrainLevels - 1 - level);
        std::vector<TerrainNode>& nodes = m_terrainNodes[std::size_t(level)];
        const auto& fine = m_terrainNodes[std::size_t(level + 1)];
        for (int nj = 0; nj < side; ++nj)
            for (int ni = 0; ni < side; ++ni) {
                TerrainNode& node = nodes[std::size_t(nj) * std::size_t(side) + std::size_t(ni)];
                float err = 0.0f;
                const int ci0 = ni * kTerrainPatch * step, cj0 = nj * kTerrainPatch * step;
                for (int cj = cj0; cj < cj0 + kTerrainPatch * step; cj += step)
                    for (int ci = ci0; ci < ci0 + kTerrainPatch * step; ci += step) {
                        const float h00 = H(ci, cj), h10 = H(ci + step, cj), h01 = H(ci, cj + step), h11 = H(ci + step, cj + step);
                        for (int j = 0; j <= step; ++j)
                            for (int i = 0; i <= step; ++i) {
                                const float u = float(i) / float(step), w = float(j) / float(step);
                                // Triangles {00, 01, 11} (w >= u) and {00, 11, 10}.
                                const float y = w >= u ? h00 + u * (h11 - h01) + w * (h01 - h00)
                                                       : h00 + u * (h10 - h00) + w * (h11 - h10);
                                err = std::max(err, std::abs(y - H(ci + i, cj + j)));
                            }
                    }
                for (int c = 0; c < 4; ++c)
                    err = std::max(err, fine[std::size_t(nj * 2 + c / 2) * std::size_t(side * 2) + std::size_t(ni * 2 + c % 2)].error);
                node.error = err;
            }
    }

    // Shared patch: kTerrainPatch^2 cells as two triangles each, plus a skirt quad along every edge segment.
    MeshData patch;
    const Material m{};
    auto v = [](int i, int j, float skirt) { return glm::vec3(float(i), skirt, float(j)); };
    for (int j = 0; j < kTerrainPatch; ++j)
        for (int i = 0; i < kTerrainPatch; ++i) {
            const glm::vec3 a = v(i, j, 0), b = v(i, j + 1, 0), c = v(i + 1, j + 1, 0), d = v(i + 1, j, 0);
            patch.addTriangle(a, b, c, m); // {00, 01, 11}
            patch.addTriangle(a, c, d, m); // {00, 11, 10}
        }
    for (int k = 0; k < kTerrainPatch; ++k) {
        const int edges[4][4] = {{k, 0, k + 1, 0},
                                 {kTerrainPatch, k, kTerrainPatch, k + 1},
                                 {k + 1, kTerrainPatch, k, kTerrainPatch},
                                 {0, k + 1, 0, k}};
        for (const auto& e : edges) {
            const glm::vec3 a = v(e[0], e[1], 0), b = v(e[2], e[3], 0), c = v(e[2], e[3], 1), d = v(e[0], e[1], 1);
            patch.addTriangle(a, b, c, m);
            patch.addTriangle(a, c, d, m);
        }
    }
    m_terrainPatch = upload(patch);
    SDL_Log("TrailForge: terrain %dx%d samples, %d root chunks, patch %u triangles", n, n,
            m_terrainRoots * m_terrainRoots, m_terrainPatch.vertexCount() / 3);
}

void Renderer::selectTerrain(const glm::vec3& cam, const glm::mat4* viewProj, float maxDistance, float lodScale,
                             std::vector<TerrainChunk>& out) const {
    out.clear();
    if (m_terrainSamples == 0) return;
    const std::optional<Frustum> fr = viewProj ? std::optional<Frustum>(Frustum(*viewProj)) : std::nullopt;
    auto visit = [&](auto&& self, int level, int ni, int nj) -> void {
        const int side = m_terrainRoots << level;
        const TerrainNode& node = m_terrainNodes[std::size_t(level)][std::size_t(nj) * std::size_t(side) + std::size_t(ni)];
        const int step = 1 << (kTerrainLevels - 1 - level);
        const int i0 = ni * kTerrainPatch * step, j0 = nj * kTerrainPatch * step;
        const float size = float(kTerrainPatch * step) * m_terrainCell;
        const glm::vec3 bmin(m_terrainOrigin + float(i0) * m_terrainCell, node.minY, m_terrainOrigin + float(j0) * m_terrainCell);
        const glm::vec3 bmax(bmin.x + size, node.maxY, bmin.z + size);
        const glm::vec3 nearest = glm::clamp(cam, bmin, bmax);
        const float d = glm::length(nearest - cam);
        if (d > maxDistance) return;
        if (fr && !fr->box(bmin - glm::vec3(0.0f, 2.0f * float(step), 0.0f), bmax)) return;
        // Split near the camera, or where the coarse grid would deviate by more than ~0.2 % of the distance (banks,
        // shorelines, road cuts): about one pixel at 1080p and 60 deg for lodScale 2.
        const float tolerance = 0.004f * 2.0f / std::max(lodScale, 0.5f);
        if (step > 1 && (d < lodScale * size || node.error > tolerance * std::max(d, 1.0f))) {
            for (int c = 0; c < 4; ++c) self(self, level + 1, ni * 2 + c % 2, nj * 2 + c / 2);
            return;
        }
        out.push_back({i0, j0, step});
    };
    for (int nj = 0; nj < m_terrainRoots; ++nj)
        for (int ni = 0; ni < m_terrainRoots; ++ni) visit(visit, 0, ni, nj);
}

void Renderer::setTrees(const std::array<MeshData, 4>& lods, std::span<const TreeInstance> instances) {
    SDL_WaitForGPUIdle(m_device);
    for (int i = 0; i < 4; ++i) {
        m_treeMeshes[i].release();
        m_treeMeshes[i] = upload(lods[i]);
    }
    m_trees.assign(instances.begin(), instances.end());
    if (m_treeBuffer) SDL_ReleaseGPUBuffer(m_device, m_treeBuffer);
    if (m_treeTransfer) SDL_ReleaseGPUTransferBuffer(m_device, m_treeTransfer);
    m_treeBuffer = nullptr;
    m_treeTransfer = nullptr;
    // Main view (twice: LOD cross-fade copies), reflection and shadow lists can each hold every tree.
    m_treeCapacity = std::max<std::uint32_t>(1, std::uint32_t(m_trees.size()) * 4);
    SDL_GPUBufferCreateInfo bi{};
    bi.usage = SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ;
    bi.size = m_treeCapacity * sizeof(TreeInstance);
    m_treeBuffer = SDL_CreateGPUBuffer(m_device, &bi);
    SDL_GPUTransferBufferCreateInfo ti{};
    ti.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    ti.size = bi.size;
    m_treeTransfer = SDL_CreateGPUTransferBuffer(m_device, &ti);
    if (!m_treeBuffer || !m_treeTransfer) fail("tree buffers");
    SDL_Log("TrailForge: %zu trees, LOD triangles %u / %u / %u / %u", m_trees.size(), m_treeMeshes[0].vertexCount() / 3,
            m_treeMeshes[1].vertexCount() / 3, m_treeMeshes[2].vertexCount() / 3, m_treeMeshes[3].vertexCount() / 3);
}

void Renderer::clearTrample() {
    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_device);
    SDL_GPUColorTargetInfo ct{};
    ct.texture = m_trample.tex;
    ct.load_op = SDL_GPU_LOADOP_CLEAR;
    ct.store_op = SDL_GPU_STOREOP_STORE;
    SDL_EndGPURenderPass(SDL_BeginGPURenderPass(cmd, &ct, 1, nullptr));
    SDL_SubmitGPUCommandBuffer(cmd);
    m_trampleEmpty = true;
    m_trampleGeneration = ~0ull;
}

void Renderer::ensureMarkCapacity(std::uint32_t segments) {
    if (segments == m_markCapacity && m_markBuffer) return;
    if (m_markBuffer) SDL_ReleaseGPUBuffer(m_device, m_markBuffer);
    if (m_markTransfer) SDL_ReleaseGPUTransferBuffer(m_device, m_markTransfer);
    SDL_GPUBufferCreateInfo bi{};
    bi.usage = SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ;
    bi.size = segments * std::uint32_t(sizeof(TireMarkSegment));
    m_markBuffer = SDL_CreateGPUBuffer(m_device, &bi);
    SDL_GPUTransferBufferCreateInfo ti{};
    ti.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    ti.size = bi.size;
    m_markTransfer = SDL_CreateGPUTransferBuffer(m_device, &ti);
    if (!m_markBuffer || !m_markTransfer) fail("tire mark buffers");
    m_markCapacity = segments;
    m_markGeneration = ~0ull; // forces a full upload
}

std::uint32_t Renderer::uploadMarks(SDL_GPUCopyPass* copy, const TireMarks& marks) {
    const std::uint32_t cap = std::uint32_t(marks.capacity());
    const bool full = marks.generation() != m_markGeneration || marks.written() - m_markWritten >= cap;
    const std::uint32_t fresh = full ? cap : std::uint32_t(marks.written() - m_markWritten);
    m_markGeneration = marks.generation();
    m_markWritten = marks.written();
    if (fresh == 0) return 0;
    // The newest `fresh` segments end just before the head; at most two contiguous runs in the ring.
    const std::uint32_t head = std::uint32_t(marks.headIndex());
    const std::uint32_t start = full ? 0 : (head + cap - fresh) % cap;
    const std::uint32_t run0 = std::min(fresh, cap - start), run1 = fresh - run0;
    constexpr std::uint32_t kSeg = sizeof(TireMarkSegment);
    auto* dst = static_cast<std::uint8_t*>(SDL_MapGPUTransferBuffer(m_device, m_markTransfer, true));
    std::memcpy(dst, marks.data() + start, std::size_t(run0) * kSeg);
    if (run1 > 0) std::memcpy(dst + std::size_t(run0) * kSeg, marks.data(), std::size_t(run1) * kSeg);
    SDL_UnmapGPUTransferBuffer(m_device, m_markTransfer);
    SDL_GPUTransferBufferLocation src0{m_markTransfer, 0};
    SDL_GPUBufferRegion region0{m_markBuffer, start * kSeg, run0 * kSeg};
    SDL_UploadToGPUBuffer(copy, &src0, &region0, false);
    if (run1 > 0) {
        SDL_GPUTransferBufferLocation src1{m_markTransfer, run0 * kSeg};
        SDL_GPUBufferRegion region1{m_markBuffer, 0, run1 * kSeg};
        SDL_UploadToGPUBuffer(copy, &src1, &region1, false);
    }
    return fresh * kSeg;
}

void Renderer::ensureLineCapacity(std::uint32_t bytes) {
    if (bytes <= m_lineCapacity) return;
    std::uint32_t cap = m_lineCapacity ? m_lineCapacity : 64 * 1024;
    while (cap < bytes) cap *= 2;
    if (m_lineBuffer) SDL_ReleaseGPUBuffer(m_device, m_lineBuffer);
    if (m_lineTransfer) SDL_ReleaseGPUTransferBuffer(m_device, m_lineTransfer);
    SDL_GPUBufferCreateInfo bi{};
    bi.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
    bi.size = cap;
    m_lineBuffer = SDL_CreateGPUBuffer(m_device, &bi);
    SDL_GPUTransferBufferCreateInfo ti{};
    ti.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    ti.size = cap;
    m_lineTransfer = SDL_CreateGPUTransferBuffer(m_device, &ti);
    if (!m_lineBuffer || !m_lineTransfer) fail("line buffers");
    m_lineCapacity = cap;
}

GpuMesh Renderer::upload(const MeshData& mesh) {
    const std::uint32_t bytes = std::uint32_t(mesh.vertices.size() * sizeof(LitVertex));
    if (bytes == 0) return {};
    glm::vec3 bmin(1e30f), bmax(-1e30f);
    for (const LitVertex& v : mesh.vertices) {
        bmin = glm::min(bmin, v.position);
        bmax = glm::max(bmax, v.position);
    }
    SDL_GPUBufferCreateInfo bi{};
    bi.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
    bi.size = bytes;
    SDL_GPUBuffer* buffer = SDL_CreateGPUBuffer(m_device, &bi);
    if (!buffer) fail("vertex buffer");

    SDL_GPUTransferBufferCreateInfo ti{};
    ti.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    ti.size = bytes;
    SDL_GPUTransferBuffer* transfer = SDL_CreateGPUTransferBuffer(m_device, &ti);
    if (!transfer) fail("transfer buffer");
    void* dst = SDL_MapGPUTransferBuffer(m_device, transfer, false);
    std::memcpy(dst, mesh.vertices.data(), bytes);
    SDL_UnmapGPUTransferBuffer(m_device, transfer);

    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_device);
    SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);
    SDL_GPUTransferBufferLocation src{transfer, 0};
    SDL_GPUBufferRegion region{buffer, 0, bytes};
    SDL_UploadToGPUBuffer(copy, &src, &region, false);
    SDL_EndGPUCopyPass(copy);
    SDL_SubmitGPUCommandBuffer(cmd);
    SDL_ReleaseGPUTransferBuffer(m_device, transfer); // SDL defers the release until the commands complete
    return GpuMesh(m_device, buffer, std::uint32_t(mesh.vertices.size()), bmin, bmax);
}

// GPU time of a completed command buffer. SDL GPU has no timestamp queries; only the Metal backend exposes the
// command buffer times (GpuTimingMetal.m). Elsewhere the GPU time is reported as unavailable.
double Renderer::gpuFenceMs(SDL_GPUFence* fence) const {
#if defined(__APPLE__)
    if (m_gpuTiming) return tf_metal_fence_gpu_ms(fence);
#endif
    (void)fence;
    return -1.0;
}

void Renderer::pollGpuTiming() {
    while (!m_fences.empty()) {
        SDL_GPUFence* f = m_fences.front().fence;
        if (!SDL_QueryGPUFence(m_device, f)) break;
        const double ms = gpuFenceMs(f);
        if (ms >= 0.0) {
            m_stats.gpuMs = ms;
            m_stats.gpuMsAvg = m_stats.gpuMsAvg <= 0.0 ? ms : m_stats.gpuMsAvg * 0.9 + ms * 0.1;
        }
        SDL_ReleaseGPUFence(m_device, f);
        m_fences.pop_front();
    }
}

// ---------------------------------------------------------------- per-frame setup

void Renderer::fillLighting(const FrameDesc& frame, FrameUniforms& fu, float& exposure, float& bloomMul) const {
    const RenderSettings& s = m_settings;
    const SkyState sky = computeSky(s.timeOfDay, s.sunHeadingDeg);
    fu.sunDir = glm::vec4(sky.lightDir, 0.0f);
    fu.sunColor = glm::vec4(sky.lightColor, 0.0f);
    fu.skyZenith = glm::vec4(sky.zenith, 0.0f);
    fu.skyHorizon = glm::vec4(sky.horizon, 0.0f);
    fu.groundAmbient = glm::vec4(sky.groundAmbient, 0.0f);
    fu.fogParams = glm::vec4(frame.fogDensity * s.fogDensity * sky.fogMul, frame.fogFalloff, 1.0f, s.clouds ? 0.55f : 0.0f);
    fu.skySun = glm::vec4(sky.sunDir, sky.sunDiscCos);
    fu.skySunGlow = glm::vec4(sky.sunGlow, sky.stars);
    fu.sunDisc = glm::vec4(sky.sunDisc, sky.moonDisc);
    fu.moonDir = glm::vec4(sky.moonDir, sky.night);
    // The mountain valley lies where the sun sets (compass azimuth ~279 deg at the sunset preset).
    fu.fogColor = glm::vec4(sky.fogColor, glm::radians(std::fmod(279.0f + s.sunHeadingDeg, 360.0f)));
    fu.skyAmbient = glm::vec4(sky.skyAmbient, 0.0f);
    // Low-beam spot lights (render-only, unshadowed). Intensity in HDR irradiance units at 1 m.
    const float lamp = frame.headlights > 0.0f ? 110.0f : 0.0f;
    fu.headlightL = glm::vec4(frame.headlightPos[0], lamp);
    fu.headlightR = glm::vec4(frame.headlightPos[1], 0.0f);
    fu.headlightDir = glm::vec4(glm::normalize(frame.headlightDir), std::cos(glm::radians(32.0f)));
    exposure = s.exposure * sky.exposure;
    bloomMul = sky.bloomMul;
}

std::uint32_t Renderer::cullTrees(const FrameDesc& frame, const glm::mat4& viewProj, const glm::mat4* reflViewProj,
                                  TreeLists& lists) {
    lists = {};
    m_treeScratch.clear();
    if (m_trees.empty()) return 0;
    const Frustum fr(viewProj);
    const glm::vec3 cam = frame.cameraPos;
    // LOD boundaries scale with the detail distance (where the last mesh LOD hands over to the far cards); in a band
    // before each boundary a tree is drawn in both LODs with complementary dither (no popping).
    const float detail = m_settings.treeDetailDistance;
    const float bounds[3] = {0.145f * detail, 0.37f * detail, detail};
    const float maxD = m_settings.treeDistance;
    for (auto& l : m_treeLod) l.clear();
    for (const TreeInstance& t : m_trees) {
        const glm::vec3 base(t.baseHeight);
        const float h = t.baseHeight.w;
        const glm::vec3 c = base + glm::vec3(0, 0.5f * h, 0);
        const glm::vec3 dc = c - cam;
        const float d2 = glm::dot(dc, dc);
        if (d2 > maxD * maxD || !fr.sphere(c, 0.6f * h)) continue;
        const float d = std::sqrt(d2);
        int l = 0;
        while (l < 3 && d >= bounds[l]) ++l;
        if (l < 3 && d > bounds[l] * (1.0f - kTreeFadeBand)) {
            // Nearer LOD l keeps share k, the next LOD the rest.
            const float k = std::clamp((bounds[l] - d) / (bounds[l] * kTreeFadeBand), 0.02f, 1.0f);
            TreeInstance a = t, b = t;
            a.params.w = k;
            b.params.w = 2.0f + k;
            m_treeLod[std::size_t(l)].push_back(a);
            m_treeLod[std::size_t(l + 1)].push_back(b);
        } else {
            TreeInstance a = t;
            a.params.w = 0.0f;
            m_treeLod[std::size_t(l)].push_back(a);
        }
    }
    const auto& lod = m_treeLod;
    for (int l = 0; l < 4; ++l) {
        lists.first[l] = std::uint32_t(m_treeScratch.size());
        lists.count[l] = std::uint32_t(lod[l].size());
        m_treeScratch.insert(m_treeScratch.end(), lod[l].begin(), lod[l].end());
    }
    if (reflViewProj) {
        const Frustum rf(*reflViewProj);
        lists.reflFirst = std::uint32_t(m_treeScratch.size());
        for (const TreeInstance& t : m_trees) {
            const glm::vec3 c = glm::vec3(t.baseHeight) + glm::vec3(0, 0.5f * t.baseHeight.w, 0);
            if (rf.sphere(c, 0.6f * t.baseHeight.w) && glm::length(c - cam) < 160.0f) m_treeScratch.push_back(t);
        }
        lists.reflCount = std::uint32_t(m_treeScratch.size()) - lists.reflFirst;
    }
    if (m_settings.shadowCascades > 0) {
        // Casters near the camera: the last cascade ends at the shadow distance; tall trees just beyond it still
        // throw shadows into it.
        const float r = m_settings.shadowDistance * frame.shadowScale + 25.0f;
        lists.shadowFirst = std::uint32_t(m_treeScratch.size());
        for (const TreeInstance& t : m_trees) {
            const glm::vec2 dxz(t.baseHeight.x - cam.x, t.baseHeight.z - cam.z);
            if (glm::dot(dxz, dxz) < r * r) m_treeScratch.push_back(t);
        }
        lists.shadowCount = std::uint32_t(m_treeScratch.size()) - lists.shadowFirst;
    }
    if (m_treeScratch.size() > m_treeCapacity) m_treeScratch.resize(m_treeCapacity);
    return std::uint32_t(m_treeScratch.size());
}

void Renderer::computeCascades(const FrameDesc& frame, float aspect, FrameUniforms& fu, glm::mat4* lightViewProj) {
    const RenderSettings& s = m_settings;
    const int n = s.shadowCascades;
    fu.shadowParams = glm::vec4(float(n), 1.0f / float(s.shadowMapSize), float(s.shadowFilter), 1.6f);
    fu.cascadeSplits = glm::vec4(1e9f);
    fu.cascadeTexel = glm::vec4(0.0f);
    if (n <= 0) return;
    const float nearD = 0.5f, farD = s.shadowDistance * frame.shadowScale;
    const glm::mat4 invView = glm::inverse(frame.view);
    const float tanY = std::tan(frame.fovYRadians * 0.5f), tanX = tanY * aspect;
    const glm::vec3 sun(fu.sunDir);
    const glm::vec3 up = std::abs(sun.y) > 0.98f ? glm::vec3(0, 0, 1) : glm::vec3(0, 1, 0);
    float prev = 0.1f;
    for (int c = 0; c < n; ++c) {
        const float t = float(c + 1) / float(n);
        const float logD = nearD * std::pow(farD / nearD, t), linD = nearD + (farD - nearD) * t;
        const float split = glm::mix(linD, logD, 0.72f);
        fu.cascadeSplits[c] = split;
        // Bounding sphere of the frustum slice (radius depends only on the slice, so it is rotation-stable).
        glm::vec3 corners[8];
        int k = 0;
        for (float d : {prev, split})
            for (float sy : {-1.0f, 1.0f})
                for (float sx : {-1.0f, 1.0f})
                    corners[k++] = glm::vec3(invView * glm::vec4(sx * tanX * d, sy * tanY * d, -d, 1.0f));
        glm::vec3 center(0.0f);
        for (const glm::vec3& p : corners) center += p;
        center /= 8.0f;
        float radius = 0.0f;
        for (const glm::vec3& p : corners) radius = std::max(radius, glm::length(p - center));
        radius = std::ceil(radius * 4.0f) / 4.0f;
        const float back = 80.0f; // casters behind the slice (toward the sun)
        const glm::mat4 lv = glm::lookAt(center + sun * (radius + back), center, up);
        glm::mat4 lp = glm::ortho(-radius, radius, -radius, radius, 0.0f, 2.0f * radius + back);
        // Snap the projection to whole shadow texels to avoid shimmering when the camera moves.
        const glm::mat4 vp = lp * lv;
        const float halfSize = float(s.shadowMapSize) * 0.5f;
        glm::vec4 origin = vp * glm::vec4(0, 0, 0, 1);
        origin *= halfSize;
        const glm::vec2 off = (glm::round(glm::vec2(origin)) - glm::vec2(origin)) / halfSize;
        lp[3][0] += off.x;
        lp[3][1] += off.y;
        lightViewProj[c] = lp * lv;
        fu.shadowViewProj[c] = lightViewProj[c];
        fu.cascadeTexel[c] = 2.0f * radius / float(s.shadowMapSize);
        prev = split;
    }
}

std::uint32_t Renderer::cullGrass(const FrameDesc& frame, const glm::mat4& viewProj,
                                  std::array<std::uint32_t, 3>& counts) {
    counts = {0, 0, 0};
    const RenderSettings& s = m_settings;
    if (s.grassLevel <= 0 || m_grassChunks.empty()) return 0;
    const Frustum fr(viewProj);
    const float D = s.grassDistance;
    const float radii[3] = {std::min(D * 0.3f, kGrassLod0Max), std::min(D * 0.6f, kGrassLod1Max), D};
    std::vector<glm::vec4> lists[3];
    const glm::vec2 cam(frame.cameraPos.x, frame.cameraPos.z);
    for (const GrassBucket& b : m_grassBuckets) {
        const glm::vec2 nb(std::clamp(cam.x, b.x0, b.x1), std::clamp(cam.y, b.z0, b.z1));
        if (glm::length(nb - cam) > D) continue;
        for (std::uint32_t k = b.first; k < b.first + b.count; ++k) {
            const GrassChunk& ch = m_grassChunks[k];
            const glm::vec2 nearest(std::clamp(cam.x, ch.x0, ch.x0 + kGrassChunk), std::clamp(cam.y, ch.z0, ch.z0 + kGrassChunk));
            const float d = glm::length(nearest - cam);
            if (d > D) continue;
            if (!fr.box(glm::vec3(ch.x0, ch.minY, ch.z0), glm::vec3(ch.x0 + kGrassChunk, ch.maxY, ch.z0 + kGrassChunk)))
                continue;
            const int lod = d < radii[0] ? 0 : d < radii[1] ? 1 : 2;
            lists[lod].push_back(glm::vec4(ch.x0, ch.z0, 0, 0));
        }
    }
    m_grassScratch.clear();
    for (int l = 0; l < 3; ++l) {
        counts[l] = std::uint32_t(lists[l].size());
        m_grassScratch.insert(m_grassScratch.end(), lists[l].begin(), lists[l].end());
    }
    if (m_grassScratch.size() > kMaxGrassChunks) m_grassScratch.resize(kMaxGrassChunks);
    return std::uint32_t(m_grassScratch.size());
}

// ---------------------------------------------------------------- frame

void Renderer::updateTerrainTextures() {
    // Wanted size: 0 without a terrain or with the textures off (placeholders, procedural ground).
    const int wanted = m_terrainSamples > 0 && m_settings.terrainTextureSize > 0
                           ? std::clamp(m_settings.terrainTextureSize, 512, 4096)
                           : 0;
    if (wanted == m_terrainTexWanted) return;
    m_terrainTexWanted = wanted;
    const Uint64 start = SDL_GetPerformanceCounter();
    const auto placeholders = [&] {
        releaseTexture(m_terrainAlbedo);
        releaseTexture(m_terrainNra);
        const std::uint8_t px[4] = {128, 128, 128, 255};
        for (Texture* t : {&m_terrainAlbedo, &m_terrainNra}) {
            *t = makeTexture(1, 1, SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM, SDL_GPU_TEXTUREUSAGE_SAMPLER,
                             SDL_GPU_SAMPLECOUNT_1, 1, 1, SDL_GPU_TEXTURETYPE_2D_ARRAY);
            uploadTexture(t->tex, px, 1, 1, 4);
        }
        m_terrainTexUniforms.params.x = 0.0f;
        m_stats.terrainTexAlbedo = m_stats.terrainTexNra = 0;
        m_stats.terrainTexBytes = 0;
    };
    if (wanted == 0) {
        if (m_stats.terrainTexAlbedo != 0) placeholders();
        return;
    }

    constexpr auto kAlbedoFmt = SDL_GPU_TEXTUREFORMAT_BC7_RGBA_UNORM_SRGB;
    constexpr auto kNraFmt = SDL_GPU_TEXTUREFORMAT_BC7_RGBA_UNORM;
    constexpr int kLayers = TerrainTextureSet::kLayers;
    TerrainTextureSet set;
    std::string error = "no texture folder";
    std::array<ktx2::Image, kLayers> albedo, nra;
    bool ok = !m_terrainTexDir.empty() && loadTerrainTextureSet(m_terrainTexDir, set, error);
    if (ok && !(SDL_GPUTextureSupportsFormat(m_device, kAlbedoFmt, SDL_GPU_TEXTURETYPE_2D_ARRAY,
                                             SDL_GPU_TEXTUREUSAGE_SAMPLER) &&
                SDL_GPUTextureSupportsFormat(m_device, kNraFmt, SDL_GPU_TEXTURETYPE_2D_ARRAY,
                                             SDL_GPU_TEXTUREUSAGE_SAMPLER))) {
        ok = false;
        error = "BC7 textures are not supported by this GPU";
    }
    const std::uint32_t skipA = ok ? terrainTextureSkip(set.albedoSize, std::uint32_t(wanted)) : 0;
    const std::uint32_t skipN = ok ? terrainTextureSkip(set.nraSize, std::uint32_t(std::min(wanted, 2048))) : 0;
    for (int i = 0; ok && i < kLayers; ++i) {
        const TerrainTextureSet::Layer& l = set.layers[std::size_t(i)];
        ok = loadTerrainTextureMap(l.albedoPath, true, skipA, albedo[std::size_t(i)], error) &&
             loadTerrainTextureMap(l.nraPath, false, skipN, nra[std::size_t(i)], error);
        if (ok && (albedo[std::size_t(i)].width != albedo[0].width || nra[std::size_t(i)].width != nra[0].width ||
                   albedo[std::size_t(i)].levels.size() != albedo[0].levels.size() ||
                   nra[std::size_t(i)].levels.size() != nra[0].levels.size())) {
            ok = false;
            error = "layer '" + l.id + "' differs in size from the first layer";
        }
    }
    if (!ok) {
        SDL_Log("TrailForge: terrain textures unavailable (%s); procedural ground", error.c_str());
        placeholders();
        return;
    }

    // One upload per layer: all mip levels of that layer in one transfer buffer.
    std::uint64_t bytes = 0;
    const auto build = [&](const std::array<ktx2::Image, kLayers>& images, SDL_GPUTextureFormat fmt, Texture& dst) {
        releaseTexture(dst);
        const ktx2::Image& first = images[0];
        dst = makeTexture(first.width, first.height, fmt, SDL_GPU_TEXTUREUSAGE_SAMPLER, SDL_GPU_SAMPLECOUNT_1,
                          kLayers, std::uint32_t(first.levels.size()), SDL_GPU_TEXTURETYPE_2D_ARRAY);
        for (int layer = 0; layer < kLayers; ++layer) {
            const ktx2::Image& img = images[std::size_t(layer)];
            std::size_t total = 0;
            for (const ktx2::Level& lv : img.levels) total += lv.data.size();
            SDL_GPUTransferBufferCreateInfo ti{};
            ti.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
            ti.size = std::uint32_t(total);
            SDL_GPUTransferBuffer* tb = SDL_CreateGPUTransferBuffer(m_device, &ti);
            if (!tb) fail("terrain texture transfer buffer");
            auto* mapped = static_cast<std::uint8_t*>(SDL_MapGPUTransferBuffer(m_device, tb, false));
            std::size_t offset = 0;
            for (const ktx2::Level& lv : img.levels) {
                std::memcpy(mapped + offset, lv.data.data(), lv.data.size());
                offset += lv.data.size();
            }
            SDL_UnmapGPUTransferBuffer(m_device, tb);
            SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_device);
            SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);
            offset = 0;
            for (std::size_t level = 0; level < img.levels.size(); ++level) {
                const ktx2::Level& lv = img.levels[level];
                SDL_GPUTextureTransferInfo src{};
                src.transfer_buffer = tb;
                src.offset = std::uint32_t(offset);
                SDL_GPUTextureRegion region{};
                region.texture = dst.tex;
                region.mip_level = std::uint32_t(level);
                region.layer = std::uint32_t(layer);
                region.w = lv.width;
                region.h = lv.height;
                region.d = 1;
                SDL_UploadToGPUTexture(copy, &src, &region, false);
                offset += lv.data.size();
            }
            SDL_EndGPUCopyPass(copy);
            SDL_SubmitGPUCommandBuffer(cmd);
            SDL_ReleaseGPUTransferBuffer(m_device, tb);
            bytes += total;
        }
    };
    build(albedo, kAlbedoFmt, m_terrainAlbedo);
    build(nra, kNraFmt, m_terrainNra);

    for (int i = 0; i < kLayers; ++i) {
        const TerrainTextureSet::Layer& l = set.layers[std::size_t(i)];
        m_terrainTexUniforms.invTile[i / 4][i % 4] = 1.0f / l.tile;
        m_terrainTexUniforms.tint[i] = glm::vec4(l.tint[0], l.tint[1], l.tint[2], 0.0f);
    }
    m_terrainTexUniforms.params = glm::vec4(1.0f, 1.0f, 0.0f, 0.0f);
    m_stats.terrainTexAlbedo = int(albedo[0].width);
    m_stats.terrainTexNra = int(nra[0].width);
    m_stats.terrainTexBytes = bytes;
    m_stats.terrainTexLoadMs = 1000.0 * double(SDL_GetPerformanceCounter() - start) / double(SDL_GetPerformanceFrequency());
    ++m_stats.terrainTexLoads;
    SDL_Log("TrailForge: terrain textures %d layers, albedo %u / normal %u texels (BC7, %zu / %zu levels), %.1f MB, "
            "%.0f ms",
            kLayers, albedo[0].width, nra[0].width, albedo[0].levels.size(), nra[0].levels.size(), double(bytes) / 1e6,
            m_stats.terrainTexLoadMs);
}

bool Renderer::drawFrame(const FrameDesc& frame) {
    pollGpuTiming();
    updateTerrainTextures();
    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_device);
    if (!cmd) fail("SDL_AcquireGPUCommandBuffer");

    SDL_GPUTexture* swapchain = nullptr;
    std::uint32_t w = 0, h = 0;
    const Uint64 acquireStart = SDL_GetPerformanceCounter();
    const bool acquired = SDL_WaitAndAcquireGPUSwapchainTexture(cmd, m_window, &swapchain, &w, &h);
    m_stats.acquireWaitMs =
        1000.0 * double(SDL_GetPerformanceCounter() - acquireStart) / double(SDL_GetPerformanceFrequency());
    if (!acquired || !swapchain || w == 0 || h == 0) {
        SDL_SubmitGPUCommandBuffer(cmd);
        return false;
    }
    ensureTargets(w, h);
    const RenderSettings& s = m_settings;
    const std::uint32_t rw = m_hdr.w, rh = m_hdr.h;
    const float aspect = float(rw) / float(rh);
    m_stats.drawCalls = 0;
    m_stats.shadowDraws = 0;

    // The swapchain cannot be read back, so a capture frame is rendered into an offscreen texture.
    SDL_GPUTexture* target = swapchain;
    SDL_GPUTexture* captureTex = nullptr;
    if (frame.capturePath) {
        SDL_GPUTextureCreateInfo ci{};
        ci.type = SDL_GPU_TEXTURETYPE_2D;
        ci.format = m_colorFormat;
        ci.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
        ci.width = w;
        ci.height = h;
        ci.layer_count_or_depth = 1;
        ci.num_levels = 1;
        captureTex = SDL_CreateGPUTexture(m_device, &ci);
        if (captureTex) target = captureTex;
    }

    // ---- Frame uniforms
    FrameUniforms fu{};
    // Reversed depth: z' = w - z maps the 0..1 range of the projection to 1 (near) .. 0 (far).
    glm::mat4 reverse(1.0f);
    reverse[2][2] = -1.0f;
    reverse[3][2] = 1.0f;
    const glm::mat4 proj = reverse * frame.proj;
    const glm::mat4 viewProj = proj * frame.view;
    fu.viewProj = viewProj;
    fu.invViewProj = glm::inverse(viewProj);
    fu.vehicleInv = glm::inverse(frame.vehicleModel);
    fu.cameraPos = glm::vec4(frame.cameraPos, frame.time);
    fu.cameraFwd = glm::vec4(-glm::vec3(glm::inverse(frame.view)[2]), 0.0f);
    float exposure = s.exposure, bloomMul = 1.0f;
    fillLighting(frame, fu, exposure, bloomMul);
    glm::mat4 lightViewProj[4];
    computeCascades(frame, aspect, fu, lightViewProj);
    fu.groundRect = m_groundRect;
    if (m_trampleWindowed) {
        // Recentre the trample window (snapped to 10 m) when the camera has moved 40 m from its centre.
        const glm::vec2 cam(frame.cameraPos.x, frame.cameraPos.z);
        if (glm::length(cam - m_trampleCenter) > 40.0f) {
            m_trampleCenter = glm::round(cam / 10.0f) * 10.0f;
            const float half = 0.5f * kTrampleWindow;
            m_trampleRect = glm::vec4(m_trampleCenter.x - half, m_trampleCenter.y - half, 1.0f / kTrampleWindow,
                                      1.0f / kTrampleWindow);
            m_trampleGeneration = ~0ull - 1; // forces a re-render
        }
    }
    fu.trampleRect = m_trampleRect;
    fu.worldParams = glm::vec4(frame.skyMountains ? 1.0f : 0.0f, frame.deepOutsideGround ? 1.0f : 0.0f, 0.0f, 0.0f);
    fu.vehicleParams = glm::vec4(frame.vehicleMud, frame.brakeLight, frame.reverseLight, frame.headlights);
    if (frame.bodyMud) {
        for (int r = 0; r < kBodyMudRegions; ++r) {
            const BodyMudCoat& c = frame.bodyMud->coat(r);
            fu.bodyMudAmount[r / 4][r % 4] = c.amount;
            fu.bodyMudFlecks[r / 4][r % 4] = c.flecks;
            fu.bodyMudColor[r] = glm::vec4(c.color[0], c.color[1], c.color[2], c.wetness);
        }
    }
    fu.clipPlane = glm::vec4(0, 0, 0, 1);
    fu.options = glm::vec4(s.groundGrid ? 0.35f : 0.0f, exposure, 1.0f, s.clouds ? 1.0f : 0.0f);
    // Sparser blade grids show more ground between the blades: give it more of the canopy colour.
    fu.grassParams = glm::vec4(s.grassLevel > 0 ? s.grassDistance : 0.0f,
                               s.grassLevel > 0 ? kGrassNearCanopy[s.grassLevel - 1] : 1.0f, 0.0f, 0.0f);

    // Puddle reflection only when the camera is above the water and near enough to see it.
    const bool wantRefl = s.reflections && frame.waterReflection && m_reflColor.tex && frame.water &&
                          frame.water->vertexCount() > 0 &&
                          frame.cameraPos.y > m_waterLevel + 0.05f;
    bool reflVisible = false;
    if (wantRefl) {
        const Frustum fr(viewProj);
        const glm::vec3 bmin = frame.water->boundsMin(), bmax = frame.water->boundsMax();
        const glm::vec2 cam(frame.cameraPos.x, frame.cameraPos.z);
        const glm::vec2 nearest(std::clamp(cam.x, bmin.x, bmax.x), std::clamp(cam.y, bmin.z, bmax.z));
        reflVisible = fr.box(bmin - glm::vec3(0, 0.1f, 0), bmax + glm::vec3(0, 0.1f, 0)) &&
                      glm::length(nearest - cam) < 120.0f;
    }
    fu.waterParams = glm::vec4(m_waterLevel, reflVisible ? 1.0f : 0.0f, 1.0f / float(rw), 1.0f / float(rh));
    m_stats.reflectionRendered = reflVisible;
    glm::mat4 mirror(1.0f);
    mirror[1][1] = -1.0f;
    mirror[3][1] = 2.0f * m_waterLevel;
    const glm::mat4 reflView = frame.view * mirror;
    const glm::mat4 reflViewProj = proj * reflView;

    // ---- Uploads (lines, grass chunks, tree instances, ImGui) before any render pass
    const std::uint32_t lineBytes = std::uint32_t(frame.lines.size_bytes());
    std::array<std::uint32_t, 3> grassCounts{};
    const std::uint32_t grassTotal = cullGrass(frame, viewProj, grassCounts);
    TreeLists trees;
    const std::uint32_t treeTotal = cullTrees(frame, viewProj, reflVisible ? &reflViewProj : nullptr, trees);
    for (int l = 0; l < 4; ++l) m_stats.trees[l] = int(trees.count[l]);
    m_stats.shadowTrees = int(trees.shadowCount);
    const TireMarks* marks = frame.tireMarks;
    if (marks) ensureMarkCapacity(std::uint32_t(marks->capacity()));
    const bool markSync = marks && (marks->written() != m_markWritten || marks->generation() != m_markGeneration);
    const std::uint32_t markCount = marks ? std::uint32_t(marks->size()) : 0;
    m_stats.markSegments = markCount;
    m_stats.markUploadBytes = 0;
    const std::uint32_t sprayCount =
        frame.spray ? std::uint32_t(std::min<std::size_t>(frame.spray->size(), kMaxSpray)) : 0;
    m_stats.sprayParticles = sprayCount;
    if (sprayCount > 0 && !m_sprayBuffer) {
        SDL_GPUBufferCreateInfo bi{};
        bi.usage = SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ;
        bi.size = kMaxSpray * std::uint32_t(sizeof(SprayInstance));
        m_sprayBuffer = SDL_CreateGPUBuffer(m_device, &bi);
        SDL_GPUTransferBufferCreateInfo ti{};
        ti.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
        ti.size = bi.size;
        m_sprayTransfer = SDL_CreateGPUTransferBuffer(m_device, &ti);
        if (!m_sprayBuffer || !m_sprayTransfer) fail("wheel spray buffers");
    }
    if (lineBytes > 0 || grassTotal > 0 || treeTotal > 0 || markSync || sprayCount > 0) {
        SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);
        if (markSync) m_stats.markUploadBytes = uploadMarks(copy, *marks);
        if (sprayCount > 0) {
            auto* dst = static_cast<SprayInstance*>(SDL_MapGPUTransferBuffer(m_device, m_sprayTransfer, true));
            frame.spray->write(dst, sprayCount);
            SDL_UnmapGPUTransferBuffer(m_device, m_sprayTransfer);
            SDL_GPUTransferBufferLocation src{m_sprayTransfer, 0};
            SDL_GPUBufferRegion region{m_sprayBuffer, 0, sprayCount * std::uint32_t(sizeof(SprayInstance))};
            SDL_UploadToGPUBuffer(copy, &src, &region, true);
        }
        if (lineBytes > 0) {
            ensureLineCapacity(lineBytes);
            void* dst = SDL_MapGPUTransferBuffer(m_device, m_lineTransfer, true);
            std::memcpy(dst, frame.lines.data(), lineBytes);
            SDL_UnmapGPUTransferBuffer(m_device, m_lineTransfer);
            SDL_GPUTransferBufferLocation src{m_lineTransfer, 0};
            SDL_GPUBufferRegion region{m_lineBuffer, 0, lineBytes};
            SDL_UploadToGPUBuffer(copy, &src, &region, true);
        }
        if (grassTotal > 0) {
            const std::uint32_t bytes = grassTotal * sizeof(glm::vec4);
            void* dst = SDL_MapGPUTransferBuffer(m_device, m_grassTransfer, true);
            std::memcpy(dst, m_grassScratch.data(), bytes);
            SDL_UnmapGPUTransferBuffer(m_device, m_grassTransfer);
            SDL_GPUTransferBufferLocation src{m_grassTransfer, 0};
            SDL_GPUBufferRegion region{m_grassBuffer, 0, bytes};
            SDL_UploadToGPUBuffer(copy, &src, &region, true);
        }
        if (treeTotal > 0) {
            const std::uint32_t bytes = treeTotal * sizeof(TreeInstance);
            void* dst = SDL_MapGPUTransferBuffer(m_device, m_treeTransfer, true);
            std::memcpy(dst, m_treeScratch.data(), bytes);
            SDL_UnmapGPUTransferBuffer(m_device, m_treeTransfer);
            SDL_GPUTransferBufferLocation src{m_treeTransfer, 0};
            SDL_GPUBufferRegion region{m_treeBuffer, 0, bytes};
            SDL_UploadToGPUBuffer(copy, &src, &region, true);
        }
        SDL_EndGPUCopyPass(copy);
    }
    if (frame.ui) ImGui_ImplSDLGPU3_PrepareDrawData(frame.ui, cmd);

    auto drawLit = [&](SDL_GPURenderPass* pass, const DrawItem& d) {
        DrawUniforms du{};
        du.model = d.model;
        du.normalMatrix = glm::transpose(glm::inverse(d.model));
        du.tint = d.tint;
        du.params = glm::vec4(d.mud, d.wheelMud ? 1.0f : 0.0f, d.soilWetness, d.wheelRadius);
        du.soil = d.soil;
        du.soilExtra = glm::vec4(d.soilFlecks, d.soilStick, 0.0f, 0.0f);
        SDL_PushGPUVertexUniformData(cmd, 1, &du, sizeof(du));
        SDL_GPUBufferBinding binding{d.mesh->buffer(), 0};
        SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
        SDL_DrawGPUPrimitives(pass, d.mesh->vertexCount(), 1, 0, 0);
    };
    // Instanced trees; the pipeline (and its vertex uniform slot 0) must already be bound.
    auto drawTrees = [&](SDL_GPURenderPass* pass, const GpuMesh& mesh, std::uint32_t first, std::uint32_t count) {
        if (count == 0 || mesh.vertexCount() == 0) return;
        TreeUniforms tu{glm::vec4(float(first), frame.time, 0, 0)};
        SDL_PushGPUVertexUniformData(cmd, 1, &tu, sizeof(tu));
        SDL_BindGPUVertexStorageBuffers(pass, 0, &m_treeBuffer, 1);
        SDL_GPUBufferBinding binding{mesh.buffer(), 0};
        SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
        SDL_DrawGPUPrimitives(pass, mesh.vertexCount(), count, 0, 0);
    };

    // Level terrain chunks: main view (frustum, LOD by distance) and shadow casters (coarser, around the camera).
    selectTerrain(frame.cameraPos, &viewProj, 1e9f, s.terrainDetail, m_terrainScratch);
    if (s.shadowCascades > 0)
        selectTerrain(frame.cameraPos, nullptr, s.shadowDistance * frame.shadowScale + 60.0f, 1.0f, m_terrainShadowScratch);
    else
        m_terrainShadowScratch.clear();
    m_stats.terrainChunks = int(m_terrainScratch.size());
    m_stats.terrainLod = {};
    for (const TerrainChunk& c : m_terrainScratch) ++m_stats.terrainLod[std::size_t(std::countr_zero(unsigned(c.step)))];
    m_stats.terrainTriangles = std::uint64_t(m_terrainScratch.size()) * (m_terrainPatch.vertexCount() / 3);
    // samplers: 2 for the main view (heights, splat), 1 for the shadow pass (heights).
    auto drawTerrain = [&](SDL_GPURenderPass* pass, const std::vector<TerrainChunk>& chunks, std::uint32_t samplers,
                           const glm::mat4* lightViewProj) {
        if (chunks.empty() || m_terrainPatch.vertexCount() == 0) return 0;
        SDL_GPUTextureSamplerBinding vt[2] = {{m_terrainHeight.tex, m_clampSampler}, {m_terrainSplat.tex, m_clampSampler}};
        SDL_BindGPUVertexSamplers(pass, 0, vt, samplers);
        SDL_GPUBufferBinding binding{m_terrainPatch.buffer(), 0};
        SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
        int drawn = 0;
        for (const TerrainChunk& c : chunks) {
            const float size = float(kTerrainPatch * c.step) * m_terrainCell;
            if (lightViewProj) {
                // Ortho light: skip chunks whose bounding sphere is outside the cascade's clip square.
                const glm::vec3 center(m_terrainOrigin + (float(c.i0) + 0.5f * float(kTerrainPatch * c.step)) * m_terrainCell,
                                       frame.cameraPos.y,
                                       m_terrainOrigin + (float(c.j0) + 0.5f * float(kTerrainPatch * c.step)) * m_terrainCell);
                const glm::mat4& lv = *lightViewProj;
                const glm::vec4 p = lv * glm::vec4(center, 1.0f);
                const float r = size * 0.75f + 300.0f; // height range of the map
                const float sx = glm::length(glm::vec3(lv[0][0], lv[1][0], lv[2][0]));
                const float sy = glm::length(glm::vec3(lv[0][1], lv[1][1], lv[2][1]));
                if (std::abs(p.x) > 1.0f + r * sx || std::abs(p.y) > 1.0f + r * sy) continue;
            }
            DrawUniforms du{};
            du.params = glm::vec4(float(c.i0), float(c.j0), float(c.step), 1.0f + 1.5f * float(c.step) * m_terrainCell);
            du.tint = glm::vec4(float(m_terrainSamples), m_terrainOrigin, m_terrainCell, 0.0f);
            SDL_PushGPUVertexUniformData(cmd, 1, &du, sizeof(du));
            SDL_DrawGPUPrimitives(pass, m_terrainPatch.vertexCount(), 1, 0, 0);
            ++drawn;
        }
        return drawn;
    };

    // ---- 1. Sun shadow cascades
    for (int c = 0; c < s.shadowCascades; ++c) {
        SDL_GPUDepthStencilTargetInfo dt{};
        dt.texture = m_shadow.tex;
        dt.clear_depth = 1.0f;
        dt.load_op = SDL_GPU_LOADOP_CLEAR;
        dt.store_op = SDL_GPU_STOREOP_STORE;
        dt.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
        dt.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
        dt.layer = std::uint8_t(c);
        dt.cycle = c == 0;
        SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmd, nullptr, 0, &dt);
        SDL_BindGPUGraphicsPipeline(pass, m_shadowPipe);
        ShadowUniforms su{lightViewProj[c]};
        SDL_PushGPUVertexUniformData(cmd, 0, &su, sizeof(su));
        const glm::mat4 lv = lightViewProj[c];
        for (const DrawItem& d : frame.draws) {
            if (!d.mesh || d.mesh->vertexCount() == 0 || !d.castsShadow) continue;
            glm::vec3 center;
            float radius;
            drawSphere(d, center, radius);
            const glm::vec4 p = lv * glm::vec4(center, 1.0f);
            // Ortho: clip-space xy in [-1, 1]; the radius in clip units is radius * scale of the projection.
            const float sx = glm::length(glm::vec3(lv[0][0], lv[1][0], lv[2][0]));
            const float sy = glm::length(glm::vec3(lv[0][1], lv[1][1], lv[2][1]));
            if (std::abs(p.x) > 1.0f + radius * sx || std::abs(p.y) > 1.0f + radius * sy) continue;
            drawLit(pass, d);
            ++m_stats.shadowDraws;
        }
        if (!m_terrainShadowScratch.empty()) {
            SDL_BindGPUGraphicsPipeline(pass, m_terrainShadowPipe);
            SDL_PushGPUVertexUniformData(cmd, 0, &su, sizeof(su));
            m_stats.shadowDraws += drawTerrain(pass, m_terrainShadowScratch, 1, &lightViewProj[c]);
        }
        if (trees.shadowCount > 0) {
            SDL_BindGPUGraphicsPipeline(pass, m_treeShadowPipe);
            SDL_PushGPUVertexUniformData(cmd, 0, &su, sizeof(su));
            drawTrees(pass, m_treeMeshes[1], trees.shadowFirst, trees.shadowCount);
            ++m_stats.shadowDraws;
        }
        SDL_EndGPURenderPass(pass);
    }

    // Shared fragment resources of the lit pipelines.
    SDL_GPUTextureSamplerBinding litTex[3] = {
        {m_shadow.tex, m_cmpSampler}, {m_noise.tex, m_repeatSampler}, {m_mask.tex, m_clampSampler}};

    MarkUniforms mu{};
    if (marks) {
        mu.info = glm::vec4(float(marks->firstIndex()), float(marks->capacity()), float(frame.simTime),
                            marks->settings().fadeTime);
        mu.params = glm::vec4(kMarkLift, 0, 0, 0);
        mu.gainA = glm::vec4(frame.markGain[0], frame.markGain[1], frame.markGain[2], frame.markGain[3]);
        mu.gainB = glm::vec4(frame.markGain[4], frame.markGain[5], frame.markGain[6], frame.markGain[7]);
        float width[8] = {1, 1, 1, 1, 1, 1, 1, 1};
        for (std::size_t i = 0; i < kSurfaceCount; ++i) width[i] = surfaceMarkResponse(SurfaceId(i)).widthScale;
        mu.widthA = glm::vec4(width[0], width[1], width[2], width[3]);
        mu.widthB = glm::vec4(width[4], width[5], width[6], width[7]);
    }

    // ---- Grass trample mask: re-rendered when segments were added or removed, and every half second while marks
    // exist (fading). Flattened / torn grass follows the marks.
    m_stats.trampleUpdated = false;
    {
        const bool changed = markCount != m_trampleCount || (marks && (marks->written() != m_trampleWritten ||
                                                                      marks->generation() != m_trampleGeneration));
        const bool due = markCount > 0 && (changed || frame.simTime - m_trampleTime > 0.5);
        if (due || (markCount == 0 && !m_trampleEmpty)) {
            SDL_GPUColorTargetInfo ct{};
            ct.texture = m_trample.tex;
            ct.load_op = SDL_GPU_LOADOP_CLEAR;
            ct.store_op = SDL_GPU_STOREOP_STORE;
            SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmd, &ct, 1, nullptr);
            if (markCount > 0) {
                SDL_BindGPUGraphicsPipeline(pass, m_tramplePipe);
                SDL_PushGPUVertexUniformData(cmd, 0, &fu, sizeof(fu));
                SDL_PushGPUVertexUniformData(cmd, 1, &mu, sizeof(mu));
                SDL_BindGPUVertexStorageBuffers(pass, 0, &m_markBuffer, 1);
                SDL_DrawGPUPrimitives(pass, 6, markCount, 0, 0);
            }
            SDL_EndGPURenderPass(pass);
            m_trampleEmpty = markCount == 0;
            m_trampleCount = markCount;
            m_trampleWritten = marks ? marks->written() : 0;
            m_trampleGeneration = marks ? marks->generation() : ~0ull;
            m_trampleTime = frame.simTime;
            m_stats.trampleUpdated = true;
        }
    }

    // ---- 2. Puddle reflection (mirrored camera, clip plane at the water level)
    if (reflVisible) {
        FrameUniforms rf = fu;
        rf.viewProj = reflViewProj;
        rf.invViewProj = glm::inverse(rf.viewProj);
        const glm::vec3 rcam(frame.cameraPos.x, 2.0f * m_waterLevel - frame.cameraPos.y, frame.cameraPos.z);
        rf.cameraPos = glm::vec4(rcam, frame.time);
        rf.cameraFwd = glm::vec4(fu.cameraFwd.x, -fu.cameraFwd.y, fu.cameraFwd.z, 0.0f);
        rf.clipPlane = glm::vec4(0, 1, 0, -(m_waterLevel - 0.03f));
        rf.waterParams.y = 0.0f;
        rf.options.x = 0.0f;

        SDL_GPUColorTargetInfo ct{};
        ct.texture = m_reflColor.tex;
        ct.load_op = SDL_GPU_LOADOP_DONT_CARE;
        ct.store_op = SDL_GPU_STOREOP_STORE;
        ct.cycle = true;
        SDL_GPUDepthStencilTargetInfo dt{};
        dt.texture = m_reflDepth.tex;
        dt.clear_depth = 0.0f;
        dt.load_op = SDL_GPU_LOADOP_CLEAR;
        dt.store_op = SDL_GPU_STOREOP_DONT_CARE;
        dt.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
        dt.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
        dt.cycle = true;
        SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmd, &ct, 1, &dt);
        SDL_BindGPUGraphicsPipeline(pass, m_reflLitPipe);
        SDL_PushGPUVertexUniformData(cmd, 0, &rf, sizeof(rf));
        SDL_PushGPUFragmentUniformData(cmd, 0, &rf, sizeof(rf));
        SDL_BindGPUFragmentSamplers(pass, 0, litTex, 3);
        const Frustum fr(rf.viewProj);
        for (const DrawItem& d : frame.draws) {
            if (!d.mesh || d.mesh->vertexCount() == 0 || !d.inReflection) continue;
            glm::vec3 center;
            float radius;
            drawSphere(d, center, radius);
            if (center.y + radius < m_waterLevel - 0.05f || !fr.sphere(center, radius)) continue;
            drawLit(pass, d);
            ++m_stats.drawCalls;
        }
        if (trees.reflCount > 0) {
            SDL_BindGPUGraphicsPipeline(pass, m_reflTreePipe);
            SDL_PushGPUVertexUniformData(cmd, 0, &rf, sizeof(rf));
            SDL_PushGPUFragmentUniformData(cmd, 0, &rf, sizeof(rf));
            SDL_BindGPUFragmentSamplers(pass, 0, litTex, 3);
            drawTrees(pass, m_treeMeshes[1], trees.reflFirst, trees.reflCount);
            ++m_stats.drawCalls;
        }
        SDL_BindGPUGraphicsPipeline(pass, m_reflSkyPipe);
        SDL_PushGPUFragmentUniformData(cmd, 0, &rf, sizeof(rf));
        SDL_GPUTextureSamplerBinding skyTex{m_noise.tex, m_repeatSampler};
        SDL_BindGPUFragmentSamplers(pass, 0, &skyTex, 1);
        SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
        ++m_stats.drawCalls;
        SDL_EndGPURenderPass(pass);
    }

    // ---- 3. Main HDR scene
    {
        SDL_GPUColorTargetInfo ct{};
        if (s.msaa > 1) {
            ct.texture = m_hdrMsaa.tex;
            ct.resolve_texture = m_hdr.tex;
            ct.store_op = SDL_GPU_STOREOP_RESOLVE;
            ct.cycle_resolve_texture = true;
        } else {
            ct.texture = m_hdr.tex;
            ct.store_op = SDL_GPU_STOREOP_STORE;
        }
        ct.load_op = SDL_GPU_LOADOP_DONT_CARE;
        ct.cycle = true;
        SDL_GPUDepthStencilTargetInfo dt{};
        dt.texture = m_depth.tex;
        dt.clear_depth = 0.0f;
        dt.load_op = SDL_GPU_LOADOP_CLEAR;
        dt.store_op = SDL_GPU_STOREOP_DONT_CARE;
        dt.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
        dt.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
        dt.cycle = true;
        SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmd, &ct, 1, &dt);

        // Level terrain first (the largest occluder).
        if (!m_terrainScratch.empty()) {
            SDL_BindGPUGraphicsPipeline(pass, m_terrainPipe);
            SDL_PushGPUVertexUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_PushGPUFragmentUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_PushGPUFragmentUniformData(cmd, 1, &m_terrainTexUniforms, sizeof(m_terrainTexUniforms));
            const SDL_GPUTextureSamplerBinding terrainTex[5] = {litTex[0], litTex[1], litTex[2],
                                                                {m_terrainAlbedo.tex, m_terrainSampler},
                                                                {m_terrainNra.tex, m_terrainSampler}};
            SDL_BindGPUFragmentSamplers(pass, 0, terrainTex, 5);
            m_stats.drawCalls += drawTerrain(pass, m_terrainScratch, 2, nullptr);
        }

        // Opaque meshes.
        SDL_BindGPUGraphicsPipeline(pass, m_litPipe);
        SDL_PushGPUVertexUniformData(cmd, 0, &fu, sizeof(fu));
        SDL_PushGPUFragmentUniformData(cmd, 0, &fu, sizeof(fu));
        SDL_BindGPUFragmentSamplers(pass, 0, litTex, 3);
        const Frustum fr(viewProj);
        for (const DrawItem& d : frame.draws) {
            if (!d.mesh || d.mesh->vertexCount() == 0) continue;
            glm::vec3 center;
            float radius;
            drawSphere(d, center, radius);
            if (!fr.sphere(center, radius)) continue;
            drawLit(pass, d);
            ++m_stats.drawCalls;
        }

        // Trees (one instanced draw per LOD).
        if (trees.count[0] + trees.count[1] + trees.count[2] + trees.count[3] > 0) {
            SDL_BindGPUGraphicsPipeline(pass, m_treePipe);
            SDL_PushGPUVertexUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_PushGPUFragmentUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_BindGPUFragmentSamplers(pass, 0, litTex, 3);
            for (int l = 0; l < 4; ++l) {
                if (trees.count[l] == 0) continue;
                drawTrees(pass, m_treeMeshes[l], trees.first[l], trees.count[l]);
                ++m_stats.drawCalls;
            }
        }

        // Tire marks: one instanced draw over the live part of the ring (6 vertices per segment), blended over the
        // ground with the same lighting, shadows and fog.
        if (markCount > 0) {
            SDL_BindGPUGraphicsPipeline(pass, m_markPipe);
            SDL_PushGPUVertexUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_PushGPUVertexUniformData(cmd, 1, &mu, sizeof(mu));
            SDL_PushGPUFragmentUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_PushGPUFragmentUniformData(cmd, 1, &mu, sizeof(mu));
            SDL_BindGPUFragmentSamplers(pass, 0, litTex, 3);
            SDL_BindGPUVertexStorageBuffers(pass, 0, &m_markBuffer, 1);
            SDL_DrawGPUPrimitives(pass, 6, markCount, 0, 0);
            ++m_stats.drawCalls;
        }

        // Grass blades (one instanced draw per LOD).
        m_stats.grassChunks = int(grassTotal);
        m_stats.grassBlades = 0;
        if (grassTotal > 0) {
            SDL_BindGPUGraphicsPipeline(pass, m_grassPipe);
            SDL_PushGPUVertexUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_PushGPUFragmentUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_GPUTextureSamplerBinding vtex[4] = {{m_height.tex, m_clampSampler},
                                                    {m_mask.tex, m_clampSampler},
                                                    {m_trample.tex, m_clampSampler},
                                                    {m_noise.tex, m_repeatSampler}};
            SDL_BindGPUVertexSamplers(pass, 0, vtex, 4);
            SDL_BindGPUVertexStorageBuffers(pass, 0, &m_grassBuffer, 1);
            SDL_GPUTextureSamplerBinding ftex{m_shadow.tex, m_cmpSampler};
            SDL_BindGPUFragmentSamplers(pass, 0, &ftex, 1);
            const float fine = kGrassCells[s.grassLevel - 1];
            const int perSide0 = int(std::lround(kGrassChunk / fine));
            const float D = s.grassDistance;
            const float radii[3] = {std::min(D * 0.3f, kGrassLod0Max), std::min(D * 0.6f, kGrassLod1Max), D};
            const float grassPixel = 2.0f * std::tan(0.5f * frame.fovYRadians) / float(std::max(m_stats.renderH, 1u));
            std::uint32_t first = 0;
            for (int l = 0; l < 3; ++l) {
                const std::uint32_t count = grassCounts[l];
                if (count > 0) {
                    const int perSide = perSide0 >> l;
                    GrassUniforms gu{};
                    gu.lod = glm::vec4(fine, float(l), float(perSide), float(first));
                    gu.fade = glm::vec4(radii[l], radii[l] * 0.18f, D, float(kGrassSegs[l]));
                    gu.blade = glm::vec4(1.0f, 0.6f * fine, 0.22f, kGrassChunk);
                    gu.extra = glm::vec4(grassPixel, s.flowerDensity, s.grassDetail ? 1.0f : 0.0f, 0.0f);
                    gu.wind = glm::vec4(0.85f, 0.53f, 3.2f, 1.0f);
                    for (int i = 0; i < 4; ++i) gu.wheels[i] = glm::vec4(glm::vec3(frame.wheelContacts[i]),
                                                                         frame.wheelContacts[i].w > 0 ? 0.32f : 0.0f);
                    SDL_PushGPUVertexUniformData(cmd, 1, &gu, sizeof(gu));
                    const std::uint32_t instances = count * std::uint32_t(perSide * perSide);
                    SDL_DrawGPUPrimitives(pass, std::uint32_t(grassTriangles(l) * 3), instances, 0, 0);
                    m_stats.grassBlades += instances;
                    ++m_stats.drawCalls;
                }
                first += count;
            }
        }

        // Sky behind everything (depth test against the cleared far plane).
        SDL_BindGPUGraphicsPipeline(pass, m_skyPipe);
        SDL_PushGPUFragmentUniformData(cmd, 0, &fu, sizeof(fu));
        SDL_GPUTextureSamplerBinding skyTex{m_noise.tex, m_repeatSampler};
        SDL_BindGPUFragmentSamplers(pass, 0, &skyTex, 1);
        SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
        ++m_stats.drawCalls;

        // Puddle surfaces.
        if (frame.water && frame.water->vertexCount() > 0) {
            SDL_BindGPUGraphicsPipeline(pass, m_waterPipe);
            SDL_PushGPUVertexUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_PushGPUFragmentUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_GPUTextureSamplerBinding wtex[4] = {{m_shadow.tex, m_cmpSampler},
                                                    {m_noise.tex, m_repeatSampler},
                                                    {m_height.tex, m_clampSampler},
                                                    {reflVisible ? m_reflColor.tex : m_noise.tex, m_clampSampler}};
            SDL_BindGPUFragmentSamplers(pass, 0, wtex, 4);
            DrawItem wd;
            wd.mesh = frame.water;
            drawLit(pass, wd);
            ++m_stats.drawCalls;
        }

        // Wheel spray: one instanced draw of camera-facing particles, lit like the scene (sun shadows, ambient,
        // headlights, fog), blended after the water.
        if (sprayCount > 0) {
            SDL_BindGPUGraphicsPipeline(pass, m_sprayPipe);
            // y: angular size of one render pixel (sub-pixel bits are drawn at >= ~0.7 px with less alpha instead).
            const float pixelAngle = 2.0f * std::tan(0.5f * frame.fovYRadians) / float(std::max(m_stats.renderH, 1u));
            const glm::vec4 sprayInfo(float(sprayCount), pixelAngle, 0, 0);
            SDL_PushGPUVertexUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_PushGPUVertexUniformData(cmd, 1, &sprayInfo, sizeof(sprayInfo));
            SDL_PushGPUFragmentUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_BindGPUFragmentSamplers(pass, 0, litTex, 3);
            SDL_BindGPUVertexStorageBuffers(pass, 0, &m_sprayBuffer, 1);
            SDL_DrawGPUPrimitives(pass, 6, sprayCount, 0, 0);
            ++m_stats.drawCalls;
        }

        // Debug lines.
        if (lineBytes > 0) {
            SDL_BindGPUGraphicsPipeline(pass, frame.linesOnTop ? m_lineOnTopPipe : m_linePipe);
            SDL_PushGPUVertexUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_PushGPUFragmentUniformData(cmd, 0, &fu, sizeof(fu));
            SDL_GPUBufferBinding binding{m_lineBuffer, 0};
            SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
            SDL_DrawGPUPrimitives(pass, std::uint32_t(frame.lines.size()), 1, 0, 0);
            ++m_stats.drawCalls;
        }
        SDL_EndGPURenderPass(pass);
    }

    auto fullscreen = [&](SDL_GPUTexture* dst, SDL_GPULoadOp load, SDL_GPUGraphicsPipeline* pipe,
                          const PostUniforms& pu, const SDL_GPUTextureSamplerBinding* tex, std::uint32_t numTex) {
        SDL_GPUColorTargetInfo ct{};
        ct.texture = dst;
        ct.load_op = load;
        ct.store_op = SDL_GPU_STOREOP_STORE;
        ct.cycle = load != SDL_GPU_LOADOP_LOAD && dst != target;
        SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(cmd, &ct, 1, nullptr);
        SDL_BindGPUGraphicsPipeline(pass, pipe);
        SDL_PushGPUFragmentUniformData(cmd, 0, &pu, sizeof(pu));
        SDL_BindGPUFragmentSamplers(pass, 0, tex, numTex);
        SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
        SDL_EndGPURenderPass(pass);
    };

    // ---- 4. Bloom (13-tap downsample chain, tent upsample with additive blend)
    const bool bloom = s.bloom && !m_bloom.empty();
    if (bloom) {
        for (int i = 0; i < kBloomLevels; ++i) {
            const Texture& src = i == 0 ? m_hdr : m_bloom[i - 1];
            PostUniforms pu{};
            pu.texel = glm::vec4(1.0f / float(src.w), 1.0f / float(src.h), 0, 0);
            pu.params = glm::vec4(1.6f, 0, 0, i == 0 ? 1.0f : 0.0f);
            pu.extra = glm::vec4(0, 0, 1, 1);
            SDL_GPUTextureSamplerBinding b{src.tex, m_clampSampler};
            fullscreen(m_bloom[i].tex, SDL_GPU_LOADOP_DONT_CARE, m_bloomDownPipe, pu, &b, 1);
        }
        for (int i = kBloomLevels - 1; i > 0; --i) {
            PostUniforms pu{};
            pu.texel = glm::vec4(1.0f / float(m_bloom[i].w), 1.0f / float(m_bloom[i].h), 0, 0);
            pu.extra = glm::vec4(0, 0, 1, 1);
            SDL_GPUTextureSamplerBinding b{m_bloom[i].tex, m_clampSampler};
            fullscreen(m_bloom[i - 1].tex, SDL_GPU_LOADOP_LOAD, m_bloomUpPipe, pu, &b, 1);
        }
    }

    // ---- 5. Tonemap (+ upscale from the render resolution), 6. FXAA; into the blur source while a menu blurs
    const bool blur = frame.blur > 0.0f && !m_blur.empty();
    SDL_GPUTexture* const post = blur ? m_blurSrc.tex : target;
    {
        PostUniforms pu{};
        pu.texel = glm::vec4(1.0f / float(rw), 1.0f / float(rh), 1.0f / float(w), 1.0f / float(h));
        pu.params = glm::vec4(0.0f, bloom ? s.bloomStrength * bloomMul : 0.0f, exposure, 0.0f);
        pu.extra = glm::vec4(0.28f, s.fxaa ? 1.0f : 0.0f, 1.0f, 1.0f);
        SDL_GPUTextureSamplerBinding tex[2] = {{m_hdr.tex, m_clampSampler},
                                               {bloom ? m_bloom[0].tex : m_hdr.tex, m_clampSampler}};
        if (s.fxaa && m_ldr.tex) {
            fullscreen(m_ldr.tex, SDL_GPU_LOADOP_DONT_CARE, m_tonemapPipe, pu, tex, 2);
            PostUniforms fx{};
            fx.texel = glm::vec4(1.0f / float(w), 1.0f / float(h), 0, 0);
            SDL_GPUTextureSamplerBinding b{m_ldr.tex, m_clampSampler};
            fullscreen(post, SDL_GPU_LOADOP_DONT_CARE, m_fxaaPipe, fx, &b, 1);
        } else {
            fullscreen(post, SDL_GPU_LOADOP_DONT_CARE, m_tonemapPipe, pu, tex, 2);
        }
    }

    // ---- 6b. Menu blur: dual Kawase down to 1/16 and back up to 1/2, composited over the sharp frame
    if (blur) {
        PostUniforms pu{};
        pu.params.x = 1.0f + 0.5f * frame.blur; // tap offset grows with the blur amount
        for (int i = 0; i < kBlurLevels; ++i) {
            const Texture& src = i == 0 ? m_blurSrc : m_blur[std::size_t(i - 1)];
            pu.texel = glm::vec4(1.0f / float(src.w), 1.0f / float(src.h), 0, 0);
            SDL_GPUTextureSamplerBinding b{src.tex, m_clampSampler};
            fullscreen(m_blur[std::size_t(i)].tex, SDL_GPU_LOADOP_DONT_CARE, m_blurDownPipe, pu, &b, 1);
        }
        for (int i = kBlurLevels - 1; i > 0; --i) {
            const Texture& src = m_blur[std::size_t(i)];
            pu.texel = glm::vec4(1.0f / float(src.w), 1.0f / float(src.h), 0, 0);
            SDL_GPUTextureSamplerBinding b{src.tex, m_clampSampler};
            fullscreen(m_blur[std::size_t(i - 1)].tex, SDL_GPU_LOADOP_DONT_CARE, m_blurUpPipe, pu, &b, 1);
        }
        PostUniforms cu{};
        cu.params = glm::vec4(frame.blur, frame.blurDim, frame.blurDesaturate, 0.0f);
        SDL_GPUTextureSamplerBinding tex[2] = {{m_blurSrc.tex, m_clampSampler}, {m_blur[0].tex, m_clampSampler}};
        fullscreen(target, SDL_GPU_LOADOP_DONT_CARE, m_blurCompositePipe, cu, tex, 2);
    }

    // ---- 7. UI on top (the ImGui pipeline is created without a depth target)
    if (frame.ui && frame.ui->DisplaySize.x > 0 && frame.ui->DisplaySize.y > 0) {
        SDL_GPUColorTargetInfo uiColor{};
        uiColor.texture = target;
        uiColor.load_op = SDL_GPU_LOADOP_LOAD;
        uiColor.store_op = SDL_GPU_STOREOP_STORE;
        SDL_GPURenderPass* uiPass = SDL_BeginGPURenderPass(cmd, &uiColor, 1, nullptr);
        ImGui_ImplSDLGPU3_RenderDrawData(frame.ui, cmd, uiPass);
        SDL_EndGPURenderPass(uiPass);
    }

    if (captureTex) {
        SDL_GPUBlitInfo blit{};
        blit.source.texture = captureTex;
        blit.source.w = w;
        blit.source.h = h;
        blit.destination.texture = swapchain;
        blit.destination.w = w;
        blit.destination.h = h;
        blit.load_op = SDL_GPU_LOADOP_DONT_CARE;
        blit.filter = SDL_GPU_FILTER_NEAREST;
        SDL_BlitGPUTexture(cmd, &blit);
        saveCapture(cmd, captureTex, w, h, frame.capturePath); // submits cmd
        SDL_ReleaseGPUTexture(m_device, captureTex);
        return true;
    }

    if (SDL_GPUFence* fence = SDL_SubmitGPUCommandBufferAndAcquireFence(cmd)) {
        m_fences.push_back({fence});
        while (m_fences.size() > 8) {
            SDL_ReleaseGPUFence(m_device, m_fences.front().fence);
            m_fences.pop_front();
        }
    }
    return true;
}

bool Renderer::saveCapture(SDL_GPUCommandBuffer* cmd, SDL_GPUTexture* src, std::uint32_t w, std::uint32_t h,
                           const char* path) {
    const std::uint32_t bytes = w * h * 4;
    SDL_GPUTransferBufferCreateInfo ti{};
    ti.usage = SDL_GPU_TRANSFERBUFFERUSAGE_DOWNLOAD;
    ti.size = bytes;
    SDL_GPUTransferBuffer* tb = SDL_CreateGPUTransferBuffer(m_device, &ti);
    if (!tb) {
        SDL_SubmitGPUCommandBuffer(cmd);
        return false;
    }
    SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(cmd);
    SDL_GPUTextureRegion region{};
    region.texture = src;
    region.w = w;
    region.h = h;
    region.d = 1;
    SDL_GPUTextureTransferInfo dst{};
    dst.transfer_buffer = tb;
    SDL_DownloadFromGPUTexture(copy, &region, &dst);
    SDL_EndGPUCopyPass(copy);
    SDL_GPUFence* fence = SDL_SubmitGPUCommandBufferAndAcquireFence(cmd);
    bool ok = false;
    if (fence) {
        SDL_WaitForGPUFences(m_device, true, &fence, 1);
        const double ms = gpuFenceMs(fence);
        if (ms >= 0.0) m_stats.gpuMs = ms;
        SDL_ReleaseGPUFence(m_device, fence);
        const SDL_GPUTextureFormat fmt = m_colorFormat;
        const SDL_PixelFormat pf = (fmt == SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM || fmt == SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM_SRGB)
                                       ? SDL_PIXELFORMAT_ARGB8888
                                       : SDL_PIXELFORMAT_ABGR8888;
        void* pixels = SDL_MapGPUTransferBuffer(m_device, tb, false);
        if (SDL_Surface* surf = SDL_CreateSurfaceFrom(int(w), int(h), pf, pixels, int(w * 4))) {
            const std::size_t len = std::strlen(path);
            const bool png = len > 4 && SDL_strcasecmp(path + len - 4, ".png") == 0;
            ok = png ? SDL_SavePNG(surf, path) : SDL_SaveBMP(surf, path);
            SDL_DestroySurface(surf);
        }
        SDL_UnmapGPUTransferBuffer(m_device, tb);
    }
    SDL_ReleaseGPUTransferBuffer(m_device, tb);
    SDL_Log("TrailForge: capture %s -> %s", path, ok ? "ok" : SDL_GetError());
    return ok;
}

} // namespace tf
