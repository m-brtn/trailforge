#pragma once

#include "render/Mesh.h"
#include "render/RenderSettings.h"

#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_video.h>
#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <deque>
#include <span>
#include <vector>

struct ImDrawData;

namespace tf {

struct GroundMaps;

// Uniform block layouts; must match the structs of the same name in shaders/scene.metal.
struct FrameUniforms {
    glm::mat4 viewProj;
    glm::mat4 invViewProj;
    glm::mat4 shadowViewProj[4];
    glm::mat4 vehicleInv;
    glm::vec4 cascadeSplits;
    glm::vec4 cascadeTexel;
    glm::vec4 cameraPos;
    glm::vec4 cameraFwd;
    glm::vec4 sunDir;
    glm::vec4 sunColor;
    glm::vec4 skyZenith;
    glm::vec4 skyHorizon;
    glm::vec4 groundAmbient;
    glm::vec4 fogParams;
    glm::vec4 shadowParams;
    glm::vec4 waterParams;
    glm::vec4 groundRect;
    glm::vec4 vehicleParams;
    glm::vec4 clipPlane;
    glm::vec4 options;
};

struct DrawUniforms {
    glm::mat4 model;
    glm::mat4 normalMatrix;
    glm::vec4 tint;
    glm::vec4 params;
};

struct GrassUniforms {
    glm::vec4 lod;
    glm::vec4 fade;
    glm::vec4 blade;
    glm::vec4 wheels[4];
};

struct ShadowUniforms {
    glm::mat4 lightViewProj;
};

struct PostUniforms {
    glm::vec4 texel;
    glm::vec4 params;
    glm::vec4 extra;
};

// GPU vertex buffer with explicit ownership (released in the destructor).
class GpuMesh {
public:
    GpuMesh() = default;
    GpuMesh(SDL_GPUDevice* device, SDL_GPUBuffer* buffer, std::uint32_t vertexCount, const glm::vec3& bmin,
            const glm::vec3& bmax)
        : m_device(device), m_buffer(buffer), m_count(vertexCount), m_min(bmin), m_max(bmax) {}
    ~GpuMesh() { release(); }
    GpuMesh(GpuMesh&& o) noexcept { *this = std::move(o); }
    GpuMesh& operator=(GpuMesh&& o) noexcept;
    GpuMesh(const GpuMesh&) = delete;
    GpuMesh& operator=(const GpuMesh&) = delete;

    SDL_GPUBuffer* buffer() const { return m_buffer; }
    std::uint32_t vertexCount() const { return m_count; }
    const glm::vec3& boundsMin() const { return m_min; }
    const glm::vec3& boundsMax() const { return m_max; }
    void release();

private:
    SDL_GPUDevice* m_device = nullptr;
    SDL_GPUBuffer* m_buffer = nullptr;
    std::uint32_t m_count = 0;
    glm::vec3 m_min{0.0f}, m_max{0.0f};
};

struct DrawItem {
    const GpuMesh* mesh = nullptr;
    glm::mat4 model{1.0f};
    glm::vec4 tint{1.0f};
    float mud = 1.0f;          // multiplier for the cosmetic mud receptivity baked in the vertices
    bool wheelMud = false;     // map the mud radially (wheel) instead of by height (body)
    bool castsShadow = true;
    bool inReflection = true;  // drawn into the puddle reflection
};

struct FrameDesc {
    glm::mat4 view{1.0f};
    glm::mat4 proj{1.0f};      // perspective, depth 0..1
    glm::vec3 cameraPos{0.0f};
    float fovYRadians = 1.0f;
    float nearPlane = 0.1f;
    float time = 0.0f;         // s, drives wind, clouds and ripples
    std::span<const DrawItem> draws;
    const GpuMesh* water = nullptr; // puddle surface (identity transform)
    std::span<const LineVertex> lines;
    bool linesOnTop = false;   // true: lines without depth test (visible through the body)
    ImDrawData* ui = nullptr;
    const char* capturePath = nullptr; // non-null: save this frame as BMP or PNG (by extension)

    // Vehicle state for shading only (cosmetic): body transform, mud amount and lamps.
    glm::mat4 vehicleModel{1.0f};
    float vehicleMud = 0.0f;
    float brakeLight = 0.0f;
    float reverseLight = 0.0f;
    float headlights = 1.0f;
    std::array<glm::vec4, 4> wheelContacts{}; // xyz contact point, w > 0 when the wheel touches the ground
};

struct RenderStats {
    std::uint32_t renderW = 0, renderH = 0;   // 3D resolution
    std::uint32_t outputW = 0, outputH = 0;   // swapchain resolution
    int drawCalls = 0;
    int shadowDraws = 0;
    int grassChunks = 0;
    std::uint64_t grassBlades = 0;            // blade instances submitted (before the mask test)
    bool reflectionRendered = false;
    double gpuMs = 0.0;                       // last completed frame (command buffer GPU time)
    double gpuMsAvg = 0.0;                    // exponential average
    double acquireWaitMs = 0.0;               // CPU time blocked in SDL_WaitAndAcquireGPUSwapchainTexture
};

// SDL GPU renderer (Metal). Passes: sun shadow cascades -> puddle reflection -> HDR scene
// (lit, grass, sky, water, lines) -> bloom -> tonemap (+ render-scale upscale) -> FXAA -> ImGui.
class Renderer {
public:
    explicit Renderer(SDL_Window* window, bool debugDevice);
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    SDL_GPUDevice* device() const { return m_device; }
    SDL_GPUTextureFormat swapchainFormat() const;
    const char* driverName() const { return m_driver; }
    const char* shaderFormatName() const { return m_shaderFormatName; }

    GpuMesh upload(const MeshData& mesh);
    // Ground maps (height, mask, noise) and grass chunk list; call once after loading the track.
    void setGround(const GroundMaps& maps, float waterLevel);

    // Draws a frame. Returns false if the swapchain texture is unavailable (window minimized).
    bool drawFrame(const FrameDesc& frame);

    void setVSync(bool on);
    bool vsync() const { return m_vsync; }

    const RenderSettings& settings() const { return m_settings; }
    void setSettings(const RenderSettings& s); // validated; pipelines/targets are rebuilt lazily
    int maxMsaa() const { return m_maxMsaa; }
    const RenderStats& stats() const { return m_stats; }

    // Output (swapchain) size the frame targets were last built for, and how many times they were created.
    std::uint32_t depthWidth() const { return m_outW; }
    std::uint32_t depthHeight() const { return m_outH; }
    int depthCreations() const { return m_targetCreations; }

private:
    struct Texture {
        SDL_GPUTexture* tex = nullptr;
        std::uint32_t w = 0, h = 0;
    };
    struct GrassChunk {
        float x0, z0, minY, maxY;
    };
    struct PendingFence {
        SDL_GPUFence* fence;
    };

    SDL_GPUShader* createShader(const char* entry, SDL_GPUShaderStage stage, std::uint32_t uniformBuffers,
                                std::uint32_t samplers = 0, std::uint32_t storageBuffers = 0);
    SDL_GPUGraphicsPipeline* createPipeline(const char* vs, const char* fs, std::uint32_t vsUniforms,
                                            std::uint32_t fsUniforms, std::uint32_t vsSamplers,
                                            std::uint32_t fsSamplers, std::uint32_t vsStorage, int vertexLayout,
                                            SDL_GPUTextureFormat color, SDL_GPUSampleCount samples, bool depthTest,
                                            bool depthWrite, SDL_GPUCullMode cull, SDL_GPUFrontFace front, int blend,
                                            SDL_GPUPrimitiveType prim = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
                                            bool depthBias = false);
    void createStaticPipelines();
    void createScenePipelines();
    void releaseScenePipelines();
    void ensureTargets(std::uint32_t outW, std::uint32_t outH);
    void releaseTargets();
    Texture makeTexture(std::uint32_t w, std::uint32_t h, SDL_GPUTextureFormat fmt, SDL_GPUTextureUsageFlags usage,
                        SDL_GPUSampleCount samples = SDL_GPU_SAMPLECOUNT_1, std::uint32_t layers = 1,
                        std::uint32_t levels = 1, SDL_GPUTextureType type = SDL_GPU_TEXTURETYPE_2D);
    void releaseTexture(Texture& t);
    void uploadTexture(SDL_GPUTexture* tex, const void* data, std::uint32_t w, std::uint32_t h,
                       std::uint32_t bytesPerPixel);
    void ensureLineCapacity(std::uint32_t bytes);
    void pollGpuTiming();
    bool saveCapture(SDL_GPUCommandBuffer* cmd, SDL_GPUTexture* src, std::uint32_t w, std::uint32_t h, const char* path);

    void computeCascades(const FrameDesc& frame, float aspect, FrameUniforms& fu, glm::mat4* lightViewProj);
    void fillLighting(FrameUniforms& fu) const;
    std::uint32_t cullGrass(const FrameDesc& frame, const glm::mat4& viewProj, std::array<std::uint32_t, 3>& counts);

    SDL_Window* m_window = nullptr;
    SDL_GPUDevice* m_device = nullptr;
    const char* m_driver = "";
    const char* m_shaderFormatName = "";
    SDL_GPUShaderFormat m_shaderFormat = SDL_GPU_SHADERFORMAT_INVALID;
    SDL_GPUTextureFormat m_colorFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    static constexpr SDL_GPUTextureFormat kHdrFormat = SDL_GPU_TEXTUREFORMAT_R16G16B16A16_FLOAT;
    static constexpr SDL_GPUTextureFormat kDepthFormat = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    bool m_vsync = true;
    int m_maxMsaa = 1;

    RenderSettings m_settings;
    bool m_targetsDirty = true;

    // Static pipelines (single-sample targets).
    SDL_GPUGraphicsPipeline* m_shadowPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_reflLitPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_reflSkyPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_bloomDownPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_bloomUpPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_tonemapPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_fxaaPipe = nullptr;
    // Scene pipelines (depend on the MSAA sample count).
    int m_sceneMsaa = 0;
    SDL_GPUGraphicsPipeline* m_litPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_skyPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_grassPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_waterPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_linePipe = nullptr;
    SDL_GPUGraphicsPipeline* m_lineOnTopPipe = nullptr;

    // Frame targets.
    std::uint32_t m_outW = 0, m_outH = 0;
    int m_targetCreations = 0;
    Texture m_hdrMsaa, m_hdr, m_depth;
    Texture m_reflColor, m_reflDepth;
    Texture m_shadow;
    std::vector<Texture> m_bloom;
    Texture m_ldr;

    // Static textures and samplers.
    Texture m_noise, m_mask, m_height;
    SDL_GPUSampler* m_cmpSampler = nullptr;
    SDL_GPUSampler* m_repeatSampler = nullptr;
    SDL_GPUSampler* m_clampSampler = nullptr;
    glm::vec4 m_groundRect{0.0f};
    float m_waterLevel = 0.0f;

    // Grass.
    std::vector<GrassChunk> m_grassChunks;
    std::vector<glm::vec4> m_grassScratch;
    SDL_GPUBuffer* m_grassBuffer = nullptr;
    SDL_GPUTransferBuffer* m_grassTransfer = nullptr;
    static constexpr std::uint32_t kMaxGrassChunks = 8192;

    // Debug lines.
    SDL_GPUBuffer* m_lineBuffer = nullptr;
    SDL_GPUTransferBuffer* m_lineTransfer = nullptr;
    std::uint32_t m_lineCapacity = 0;

    std::deque<PendingFence> m_fences;
    RenderStats m_stats;
};

} // namespace tf
