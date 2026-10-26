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
#include <string>
#include <vector>

struct ImDrawData;

namespace tf {

struct ModelAsset;

struct GroundMaps;
class TireMarks;
class WheelSpray;
class BodyMud;

// Uniform block layouts; must match the structs of the same name in shaders/scene.hlsl.
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
    glm::vec4 skySun;       // xyz: true sun direction (sky, glow), w: cos of the sun disc radius
    glm::vec4 skySunGlow;   // rgb: glow / in-scatter color around the sun, w: star visibility
    glm::vec4 sunDisc;      // rgb: sun disc radiance, w: moon disc brightness
    glm::vec4 moonDir;      // xyz, w: night factor
    glm::vec4 fogColor;     // rgb, w: world azimuth of the mountain valley (rad)
    glm::vec4 skyAmbient;   // rgb: sky diffuse irradiance, w: scattering sky amount
    glm::vec4 headlightL;   // xyz: lamp position, w: intensity (0 = off)
    glm::vec4 headlightR;   // xyz: lamp position
    glm::vec4 headlightDir; // xyz: beam axis, w: cos of the outer cone angle
    glm::vec4 bodyMudAmount[4]; // BodyMud regions 0..13 (BodyMudRegion order), cosmetic
    glm::vec4 bodyMudColor[16]; // rgb linear, w: wetness
    glm::vec4 bodyMudFlecks[4]; // grass blades in the coat per region
    glm::vec4 grassParams;      // x: grass distance (m, 0 = grass off), y: canopy colour on the ground near the camera
    glm::vec4 trampleRect;      // xMin, zMin, 1/sizeX, 1/sizeZ of the grass trample mask
    glm::vec4 worldParams;      // x: sky-pass mountains (0/1), y: water outside the ground maps is deep (0/1),
                                // z: valley fog density at its base, w: valley fog base height
    glm::vec4 bodyMudArch;      // vehicle body: axle z front / rear, arch centre y, arch radius (BodyMud regions)
    glm::vec4 bodyMudBody;      // vehicle body: side |x|, bumper zones from z front / rear
    glm::vec4 screenFx;         // x: opaque scene copy + linear depth bound (0/1), y: water SSR (0/1)
};

struct DrawUniforms {
    glm::mat4 model;
    glm::mat4 normalMatrix;
    glm::vec4 tint;
    glm::vec4 params;
    glm::vec4 soil;
    glm::vec4 soilExtra; // x: grass flecks in the tread, y: soil adhesion
};

struct GrassUniforms {
    glm::vec4 lod;
    glm::vec4 fade;
    glm::vec4 blade;
    glm::vec4 extra; // x: radians per pixel, y: flower density, z: fine detail (0/1)
    glm::vec4 wind;  // xy: wind direction, z: gust speed (m/s), w: gust strength
    glm::vec4 wheels[4];
};

struct ShadowUniforms {
    glm::mat4 lightViewProj;
};

struct TerrainTextureUniforms {
    glm::vec4 invTile[2]; // 1 / repeat size (m) per layer: grass, soil, road, rock | sand, snow
    glm::vec4 tint[6];    // rgb albedo multiplier per layer
    glm::vec4 params;     // x: textures loaded (0 = procedural fallback), y: normal map strength, z: snow weight is asphalt
};

struct TreeUniforms {
    glm::vec4 info; // x: first instance in the per-frame instance buffer, y: time (s)
};

struct ModelUniforms {
    glm::vec4 info;   // x: first instance, y: time (s), z: wind strength, w: model height (m, unit scale)
    glm::vec4 params; // x: sway amplitude (share of the height), y: flutter amplitude (m), z: impostor views, w: radius
};

struct ModelMaterialUniforms {
    glm::vec4 params; // x: alpha test, y: foliage, z: alpha to coverage, w: normal map strength (impostors: views)
    glm::vec4 tint;   // rgb: albedo multiplier, w: per-instance hue variation
};

struct ModelInstance {
    glm::vec4 posFade;  // xyz: model origin, w: LOD cross-fade code (as TreeInstance::params.w)
    glm::vec4 scaleYaw; // xyz: scale per model axis, w: yaw (rad)
    glm::vec4 tilt;     // x: pitch (rad) about the model x axis, after the scale and before the yaw
};

struct PostUniforms {
    glm::vec4 texel;
    glm::vec4 params;
    glm::vec4 extra;
    glm::vec4 look{0.0f};
    glm::vec4 look2{0.0f};
    glm::vec4 look3{0.0f};
};

struct PostFxUniforms {
    glm::mat4 viewProj;     // without the TAA jitter
    glm::mat4 prevViewProj; // previous frame, without jitter
    glm::mat4 invView;
    glm::vec4 proj;         // P00, P11, P20, P21 of the jittered projection
    glm::vec4 texel;
    glm::vec4 params;
    glm::vec4 params2;
    glm::vec4 camPos;
    glm::vec4 sun;          // xy: sun uv, z: shaft weight
    glm::vec4 sunColor;
    glm::vec4 depthParams;  // near * far, near, far - near, far
};

struct VelocityUniforms {
    glm::mat4 viewProjJ;
    glm::mat4 viewProj;
    glm::mat4 prevViewProj;
    glm::mat4 model;
    glm::mat4 prevModel;
};

struct MarkUniforms {
    glm::vec4 info;   // x: first live segment (ring index), y: ring capacity, z: simulation time (s), w: fade time (s)
    glm::vec4 params; // x: lift along the contact normal (m)
    glm::vec4 gainA;  // per-surface intensity multipliers: Concrete, PackedDirt, Rock, LowGrip
    glm::vec4 gainB;  // Mud, Grass, Sand, Snow
    glm::vec4 widthA; // mark width / tire width per surface (SurfaceMarkResponse::widthScale), same order
    glm::vec4 widthB;
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
    glm::vec4 soil{0.0f};      // wheels: soil in the tread (rgb linear colour, a amount 0..1), cosmetic
    float soilWetness = 0.0f;
    float soilFlecks = 0.0f;   // grass blades stuck in the tread
    float soilStick = 1.0f;    // adhesion of the tread soil (dry sand dusts, mud cakes)
    float wheelRadius = 0.0f;
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
    const GpuMesh* water = nullptr; // water surfaces (identity transform): puddles, or a level's lakes, rivers and sea
    const GpuMesh* waterfalls = nullptr; // level waterfalls (identity transform, waterfall_fs)
    bool waterReflection = true;    // planar reflection at the ground maps' water level (puddles only)
    bool skyMountains = true;       // stylized mountain silhouettes in the sky (off where real distant terrain exists)
    bool deepOutsideGround = false; // water beyond the ground maps is open sea (no floor height there)
    float fogDensity = 0.0016f;     // height fog at y = 0 (1/m), times the fog setting
    float fogFalloff = 0.05f;       // 1/m
    float valleyFog = 0.0f;         // valley layer density at its base (1/m), times the valley fog setting
    float valleyFogBase = 0.0f;     // valley layer base height (m)
    float shadowScale = 1.0f;       // multiplies the shadow distance setting (open levels see farther)
    std::span<const LineVertex> lines;
    bool linesOnTop = false;   // true: lines without depth test (visible through the body)
    ImDrawData* ui = nullptr;
    const char* capturePath = nullptr; // non-null: save this frame as BMP or PNG (by extension)
    float blur = 0.0f;           // 0..1: menu blur of the finished 3D frame (under the UI)
    float blurDim = 0.55f;       // brightness of the blurred frame at full blur
    float blurDesaturate = 0.25f;
    // Depth of field (RenderSettings::dof / dofDriving): focus distance (m) and amount 0..1; 0 = off.
    float dofFocus = 0.0f;
    float dofAmount = 0.0f;
    bool cameraCut = false;       // teleport / camera switch: drop the TAA history, motion vectors and adaptation

    // Vehicle state for shading only (cosmetic): body transform, mud amount and lamps.
    glm::mat4 vehicleModel{1.0f};
    float vehicleMud = 0.0f;
    const BodyMud* bodyMud = nullptr;            // per-region spray mud on the body (cosmetic)
    glm::vec4 bodyMudArch{1.40f, -1.40f, -0.20f, 0.545f}; // where its regions are (VehicleVisual::mudArch / mudBody)
    glm::vec4 bodyMudBody{0.88f, 2.15f, -2.10f, 0.0f};
    float brakeLight = 0.0f;
    float reverseLight = 0.0f;
    float headlights = 1.0f;
    std::array<glm::vec3, 2> headlightPos{};  // world positions of the two lamps (spot lights, no shadows)
    glm::vec3 headlightDir{0, 0, 1};          // beam axis
    std::array<glm::vec4, 4> wheelContacts{}; // xyz contact point, w > 0 when the wheel touches the ground

    // Persistent tire marks (render-only). The renderer keeps a GPU copy of the segment ring and uploads only the
    // segments written since the previous frame.
    const TireMarks* tireMarks = nullptr;
    const WheelSpray* spray = nullptr;           // wheel spray particles (one instanced draw)
    double simTime = 0.0;                        // simulation time the segment birth times refer to (s)
    std::array<float, 8> markGain{1, 1, 1, 1, 1, 1, 1, 1}; // per-surface intensity multipliers (SurfaceId order)
};

struct RenderStats {
    std::uint32_t renderW = 0, renderH = 0;   // 3D resolution
    std::uint32_t outputW = 0, outputH = 0;   // swapchain resolution
    int drawCalls = 0;
    int shadowDraws = 0;
    int grassChunks = 0;
    std::uint64_t grassBlades = 0;            // blade instances submitted (before the mask test)
    std::array<int, 4> trees{};               // tree instances drawn per LOD (main view; 3 = crossed cards)
    int shadowTrees = 0;                      // tree instances per shadow cascade
    bool reflectionRendered = false;
    std::uint32_t markSegments = 0;           // tire mark segments drawn (one instanced draw)
    std::uint32_t markUploadBytes = 0;        // tire mark bytes uploaded this frame
    std::uint32_t sprayParticles = 0;         // wheel spray particles drawn
    bool trampleUpdated = false;              // grass trample mask re-rendered this frame
    double gpuMs = 0.0;                       // last completed frame (command buffer GPU time)
    double gpuMsAvg = 0.0;                    // exponential average
    double acquireWaitMs = 0.0;               // CPU time blocked in SDL_WaitAndAcquireGPUSwapchainTexture
    int terrainChunks = 0;                    // level terrain chunks drawn (main view)
    std::uint64_t terrainTriangles = 0;
    std::array<int, 4> terrainLod{};          // chunks per LOD (step 1, 2, 4, 8 samples)
    int terrainTexAlbedo = 0, terrainTexNra = 0; // loaded terrain texture sizes (0 = procedural ground)
    std::uint64_t terrainTexBytes = 0;          // GPU bytes of the two texture arrays
    double terrainTexLoadMs = 0.0;              // last load (read + upload)
    int terrainTexLoads = 0;
    int models = 0;                             // level model files loaded (meshes on the GPU)
    std::uint64_t modelGpuBytes = 0;            // their vertex, index and texture bytes
    double modelLoadMs = 0.0;
    int modelInstances = 0;                     // mesh instances drawn (main view, cross-fade copies included)
    int modelImpostors = 0;                     // impostor cards drawn
    std::uint64_t modelTriangles = 0;           // main view
    int modelShadowInstances = 0;               // summed over the cascades
    std::uint64_t modelShadowTriangles = 0;
    int modelDraws = 0;                         // draw calls (main view)
    bool depthEffects = false;                  // single-sample scene: TAA, AO, motion blur, shafts, DOF available
    bool taa = false, ao = false, motionBlur = false, shafts = false, dof = false, autoExposure = false;
    int velocityObjects = 0;                    // moving meshes drawn into the velocity buffer
    float flareSun = 0.0f;                      // sun weight feeding the lens flare ghosts (0: no ghosts)
    float sceneLogLum = 0.0f;                   // adapted log2 mean scene luminance (after the base exposure)
    float autoExposureEv = 0.0f;                // correction applied on top of the time-of-day exposure
};

// Level terrain for the renderer: the physics height grid and the per-cell splat weights (render-only copy).
struct TerrainSource {
    int samples = 0;                          // per side; samples - 1 must be a multiple of 512
    float origin = 0.0f, cell = 1.0f;         // sample (0, 0) at (origin, origin)
    const float* heights = nullptr;           // samples^2, row-major by z
    const std::uint32_t* splat = nullptr;     // (samples - 1)^2 RGBA8: road paint, rock, sand, snow
    bool asphalt = false;                     // the snow weight draws asphalt (level.json asphaltLayer)
};

// SDL GPU renderer (Metal). Passes: sun/moon shadow cascades -> grass trample mask -> puddle reflection -> HDR scene
// (lit, instanced trees, tire marks, grass, sky with mountains, water, lines) -> bloom -> tonemap (+ render-scale upscale)
// -> FXAA -> menu blur (dual Kawase, only with FrameDesc::blur) -> ImGui.
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
    // GPU frame time is measured only on the Metal backend; elsewhere it is reported as unavailable.
    bool gpuTimingAvailable() const { return m_gpuTiming; }

    GpuMesh upload(const MeshData& mesh);
    // Ground maps (height, mask, noise) and grass chunk list; call once after loading the track.
    void setGround(const GroundMaps& maps, float waterLevel);
    // Level terrain (quadtree of 64 x 64 cell chunks, step 1..8 samples, skirts); nullptr removes it.
    void setTerrain(const TerrainSource* terrain);
    // Folder with terrain.json and the material KTX2 files (assets/textures/terrain). Loaded while a terrain is set,
    // at RenderSettings::terrainTextureSize; missing or invalid files fall back to the procedural ground.
    void setTerrainTextureDir(const std::string& dir) { m_terrainTexDir = dir; m_terrainTexWanted = -1; }
    // Instanced trees: one unit-height mesh per LOD (near, mid, far, crossed cards) and the static instance list.
    void setTrees(const std::array<MeshData, 4>& lods, std::span<const TreeInstance> instances);
    // Instanced level models (core/ModelAsset .glb: mesh LODs, BC7 materials, impostor atlas) and their static
    // placements. Files that fail to load are skipped with a log line (their placements are not drawn).
    // In-memory models (the building kit) follow the files: placement model paths.size() + k is assets[k], its
    // material textures are relative to assetTextureDir. Returns the number of models loaded.
    int setModels(std::span<const std::string> paths, std::span<const ModelPlacement> placements,
                  std::span<const ModelAsset* const> assets = {}, const std::string& assetTextureDir = {});

    // Draws a frame. Returns false if the swapchain texture is unavailable (window minimized).
    bool drawFrame(const FrameDesc& frame);

    // RGBA8 image (sRGB-encoded, straight alpha) with a full mip chain, for ImGui (ImTextureID = SDL_GPUTexture*).
    // The caller releases it with releaseImage before the renderer goes away.
    SDL_GPUTexture* createImage(const void* rgba, std::uint32_t w, std::uint32_t h);
    void releaseImage(SDL_GPUTexture* tex);

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

    // GPU cost measurement: the named post pass is recorded `repeat` times per frame (same inputs and output, so
    // the image is unchanged); (frame GPU time at N - at 1) / (N - 1) is its cost. Unknown name or repeat 1: off.
    static const char* const kPostPassNames[];
    void setPostBench(const std::string& pass, int repeat);

private:
    int supportedMsaa(int n) const; // highest supported sample count <= n
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
                                            int depthBias = 0);
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
    // Copies the tire mark segments written since the last call into the GPU ring (inside an open copy pass).
    std::uint32_t uploadMarks(SDL_GPUCopyPass* copy, const TireMarks& marks);
    void ensureMarkCapacity(std::uint32_t segments);
    void clearTrample();
    void pollGpuTiming();
    double gpuFenceMs(SDL_GPUFence* fence) const;
    bool saveCapture(SDL_GPUCommandBuffer* cmd, SDL_GPUTexture* src, std::uint32_t w, std::uint32_t h, const char* path);

    void computeCascades(const FrameDesc& frame, float aspect, FrameUniforms& fu, glm::mat4* lightViewProj);
    void fillLighting(const FrameDesc& frame, FrameUniforms& fu, float& exposure, float& bloomMul) const;
    std::uint32_t cullGrass(const FrameDesc& frame, const glm::mat4& viewProj, std::array<std::uint32_t, 3>& counts);
    // Fills m_treeScratch: main view LOD 0/1/2, puddle reflection, shadow casters. Returns the total instance count.
    struct TreeLists {
        std::array<std::uint32_t, 4> first{}, count{};
        std::uint32_t reflFirst = 0, reflCount = 0, shadowFirst = 0, shadowCount = 0;
    };
    std::uint32_t cullTrees(const FrameDesc& frame, const glm::mat4& viewProj, const glm::mat4* reflViewProj,
                            TreeLists& lists);
    struct TerrainChunk {
        int i0, j0, step;
    };
    // Quadtree selection: frustum (optional) and maximum distance from the camera.
    void selectTerrain(const glm::vec3& cam, const glm::mat4* viewProj, float maxDistance, float lodScale,
                       std::vector<TerrainChunk>& out) const;

    SDL_Window* m_window = nullptr;
    SDL_GPUDevice* m_device = nullptr;
    const char* m_driver = "";
    const char* m_shaderFormatName = "";
    bool m_gpuTiming = false;
    SDL_GPUShaderFormat m_shaderFormat = SDL_GPU_SHADERFORMAT_INVALID;
    SDL_GPUTextureFormat m_colorFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    static constexpr SDL_GPUTextureFormat kHdrFormat = SDL_GPU_TEXTUREFORMAT_R16G16B16A16_FLOAT;
    static constexpr SDL_GPUTextureFormat kDepthFormat = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    bool m_vsync = true;
    int m_maxMsaa = 1;
    bool m_msaa2 = false;
    float m_shadowBiasClamp = 0.01f;

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
    SDL_GPUGraphicsPipeline* m_treeShadowPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_reflTreePipe = nullptr;
    SDL_GPUGraphicsPipeline* m_tramplePipe = nullptr;
    SDL_GPUGraphicsPipeline* m_blurDownPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_blurUpPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_blurCompositePipe = nullptr;
    SDL_GPUGraphicsPipeline* m_terrainShadowPipe = nullptr;
    // Post effects on the single-sample scene (static).
    SDL_GPUGraphicsPipeline* m_linDepthPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_gtaoPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_aoBlurPipe = nullptr;      // multiplies onto the HDR scene
    SDL_GPUGraphicsPipeline* m_velocityPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_velocityObjPipe = nullptr; // depth-tested against the scene depth
    SDL_GPUGraphicsPipeline* m_taaPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_motionBlurPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_dofBlurPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_dofPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_lumPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_adaptPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_shaftsPipe = nullptr;
    // Scene pipelines (depend on the MSAA sample count).
    int m_sceneMsaa = 0;
    SDL_GPUGraphicsPipeline* m_litPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_skyPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_grassPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_waterPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_waterfallPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_linePipe = nullptr;
    SDL_GPUGraphicsPipeline* m_lineOnTopPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_treePipe = nullptr;
    SDL_GPUGraphicsPipeline* m_markPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_sprayPipe = nullptr;
    SDL_GPUGraphicsPipeline* m_terrainPipe = nullptr;

    // Frame targets.
    std::uint32_t m_outW = 0, m_outH = 0;
    int m_targetCreations = 0;
    Texture m_hdrMsaa, m_hdr, m_depth;
    Texture m_reflColor, m_reflDepth;
    Texture m_shadow;
    std::vector<Texture> m_bloom;
    Texture m_ldr;
    static constexpr int kBlurLevels = 4;
    Texture m_blurSrc;
    std::vector<Texture> m_blur;
    // Post effects (single-sample scene only).
    static constexpr SDL_GPUTextureFormat kLinDepthFormat = SDL_GPU_TEXTUREFORMAT_R32_FLOAT;
    static constexpr SDL_GPUTextureFormat kVelocityFormat = SDL_GPU_TEXTUREFORMAT_R16G16_FLOAT;
    static constexpr SDL_GPUTextureFormat kAoFormat = SDL_GPU_TEXTUREFORMAT_R16G16_FLOAT;
    Texture m_linDepth, m_velocity, m_ao, m_sceneCopy;
    std::array<Texture, 2> m_taa; // history ping-pong
    std::array<Texture, 2> m_fx;  // motion blur / DOF outputs
    Texture m_dofHalf, m_shafts;
    Texture m_lum;                // 64 x 64 log luminance
    std::array<Texture, 2> m_adapt; // 1 x 1 adapted log luminance, ping-pong
    Texture m_lut;                // 32^3 colour grade
    Texture m_farDepth;           // 1 x 1 linear depth "far" for passes without a depth buffer
    SDL_GPUSampler* m_pointSampler = nullptr;
    int m_taaIndex = 0, m_adaptIndex = 0;
    bool m_historyValid = false, m_adaptValid = false;
    std::uint32_t m_frameIndex = 0;
    glm::mat4 m_prevViewProj{1.0f};
    glm::vec3 m_prevCamPos{0.0f}, m_prevCamFwd{0.0f, 0.0f, -1.0f};
    bool m_prevValid = false;
    float m_prevTime = 0.0f;
    std::vector<std::pair<const GpuMesh*, glm::mat4>> m_prevModels; // per draw of the previous frame
    std::vector<std::pair<const GpuMesh*, glm::mat4>> m_curModels;
    int m_benchPass = -1, m_benchRepeat = 1;
    SDL_GPUTransferBuffer* m_adaptReadback = nullptr;
    void buildGradeLut();

    // Static textures and samplers.
    Texture m_noise, m_mask, m_height;
    SDL_GPUSampler* m_cmpSampler = nullptr;
    SDL_GPUSampler* m_repeatSampler = nullptr;
    SDL_GPUSampler* m_clampSampler = nullptr;
    glm::vec4 m_groundRect{0.0f};
    float m_waterLevel = 0.0f;

    // Level terrain.
    struct TerrainNode {
        float minY, maxY;
        float error = 0.0f; // m: largest height deviation of this node's step grid (and its subtree) from the samples
    };
    Texture m_terrainHeight, m_terrainSplat;
    GpuMesh m_terrainPatch;
    int m_terrainSamples = 0;
    float m_terrainOrigin = 0.0f, m_terrainCell = 1.0f;
    bool m_terrainAsphalt = false;
    int m_terrainRoots = 0;                         // root nodes per side (step 8)
    std::array<std::vector<TerrainNode>, 4> m_terrainNodes; // per level (step 8, 4, 2, 1), row-major
    std::vector<TerrainChunk> m_terrainScratch, m_terrainShadowScratch;
    // Terrain material textures (BC7 arrays, one layer per material) or 1 x 1 placeholders.
    void updateTerrainTextures();
    Texture m_terrainAlbedo, m_terrainNra;
    SDL_GPUSampler* m_terrainSampler = nullptr;
    std::string m_terrainTexDir;
    int m_terrainTexWanted = -1; // size the arrays were loaded for (-1: not tried)
    TerrainTextureUniforms m_terrainTexUniforms{};

    // Grass: chunks sorted into square buckets of kGrassBucket x kGrassBucket chunks for culling.
    struct GrassBucket {
        float x0, z0, x1, z1;
        std::uint32_t first, count;
    };
    std::vector<GrassBucket> m_grassBuckets;
    std::vector<GrassChunk> m_grassChunks;
    std::vector<glm::vec4> m_grassScratch;
    SDL_GPUBuffer* m_grassBuffer = nullptr;
    SDL_GPUTransferBuffer* m_grassTransfer = nullptr;
    static constexpr std::uint32_t kMaxGrassChunks = 8192;

    // Trees (static instance list; the visible subsets are uploaded every frame).
    std::array<GpuMesh, 4> m_treeMeshes;
    std::vector<TreeInstance> m_trees;
    std::vector<TreeInstance> m_treeScratch;
    std::array<std::vector<TreeInstance>, 4> m_treeLod;
    static constexpr float kTreeFadeBand = 0.12f; // share of a LOD boundary distance over which trees cross-fade
    SDL_GPUBuffer* m_treeBuffer = nullptr;
    SDL_GPUTransferBuffer* m_treeTransfer = nullptr;
    std::uint32_t m_treeCapacity = 0; // instances

    // Level models. Instances are sorted into square buckets for culling; every frame the visible ones are sorted by
    // (model, LOD) into m_modelScratch (main view, then the shadow cascades) and drawn per material.
    struct GpuModelLod {
        SDL_GPUBuffer* vertices = nullptr;
        SDL_GPUBuffer* indices = nullptr;
        struct Primitive {
            std::uint32_t firstIndex, indexCount;
            int material;
        };
        std::vector<Primitive> primitives;
        std::uint32_t triangles = 0;
    };
    struct GpuModelMaterial {
        int albedo = -1, nra = -1; // m_modelTextures
        bool alphaTest = false;
        glm::vec3 tint{1.0f};
    };
    struct GpuModel {
        std::string kind;
        float height = 0.0f, radius = 0.0f;
        glm::vec3 center{0.0f}; // bounds center, unit scale
        float boundRadius = 0.0f;
        std::vector<GpuModelLod> lods;
        std::vector<GpuModelMaterial> materials;
        int impostorAlbedo = -1, impostorNra = -1, impostorViews = 0;
        bool foliage = false; // any alpha-tested material (trees, bushes)
    };
    struct ModelObject {
        glm::vec3 position, scale;
        float yaw, pitch;
        glm::vec3 center; // world bounding sphere
        float radius;
        int model;
    };
    struct ModelBucket {
        glm::vec3 bmin, bmax; // of the bounding spheres
        std::uint32_t first, count;
    };
    struct ModelDraw {
        int model, lod; // lod -1: impostor cards
        std::uint32_t first, count;
    };
    // Per kind and settings: end distance of each mesh LOD (m), the farthest distance drawn, impostors beyond the last
    // mesh LOD, cascades the model casts into (cascade 0 from LOD 1, the others from the last LOD).
    struct ModelRange {
        std::array<float, 4> bounds{};
        float maxDistance = 0.0f;
        bool impostor = false;
        int shadowCascades = 0;
    };
    ModelRange modelRange(const GpuModel& m) const;
    int loadModelTexture(const std::string& path, bool srgb, std::uint64_t& bytes);
    void releaseModels();
    // Fills m_modelScratch and the draw lists. Returns the total instance count.
    std::uint32_t cullModels(const FrameDesc& frame, const glm::mat4& viewProj, const glm::mat4* lightViewProj);
    std::vector<GpuModel> m_models;
    std::vector<Texture> m_modelTextures;
    std::vector<ModelObject> m_modelObjects;
    std::vector<ModelBucket> m_modelBuckets;
    std::vector<ModelInstance> m_modelScratch;
    std::vector<std::vector<ModelInstance>> m_modelBins; // per (model, LOD slot) while culling
    std::vector<ModelDraw> m_modelDraws;
    std::array<std::vector<ModelDraw>, 4> m_modelShadowDraws;
    SDL_GPUBuffer* m_modelBuffer = nullptr;
    SDL_GPUTransferBuffer* m_modelTransfer = nullptr;
    std::uint32_t m_modelCapacity = 0; // instances
    SDL_GPUGraphicsPipeline* m_modelShadowPipe = nullptr; // static
    SDL_GPUGraphicsPipeline* m_modelPipe = nullptr;       // scene: opaque, back faces culled
    SDL_GPUGraphicsPipeline* m_modelFoliagePipe = nullptr; // scene: alpha tested, two-sided, alpha to coverage
    SDL_GPUGraphicsPipeline* m_impostorPipe = nullptr;    // scene
    bool m_modelA2C = false;

    // Wheel spray: all live particles are uploaded every frame (at most kMaxSpray).
    static constexpr std::uint32_t kMaxSpray = 8192;
    SDL_GPUBuffer* m_sprayBuffer = nullptr;
    SDL_GPUTransferBuffer* m_sprayTransfer = nullptr;

    // Tire marks: GPU copy of the simulation's segment ring (same capacity and indices).
    SDL_GPUBuffer* m_markBuffer = nullptr;
    SDL_GPUTransferBuffer* m_markTransfer = nullptr;
    std::uint32_t m_markCapacity = 0; // segments
    std::uint64_t m_markWritten = 0;
    std::uint64_t m_markGeneration = ~0ull;
    // Grass trample mask over the ground rect (R: flattened, G: torn), rendered from the mark segments.
    static constexpr float kTrampleTexel = 0.1f; // m
    static constexpr SDL_GPUTextureFormat kTrampleFormat = SDL_GPU_TEXTUREFORMAT_R8G8_UNORM;
    Texture m_trample;
    glm::vec4 m_trampleRect{0.0f};
    bool m_trampleWindowed = false; // the mask is a window that follows the camera (large ground maps)
    glm::vec2 m_trampleCenter{0.0f};
    std::uint64_t m_trampleWritten = 0, m_trampleGeneration = ~0ull;
    std::size_t m_trampleCount = 0;
    double m_trampleTime = -1e9;
    bool m_trampleEmpty = true;

    // Debug lines.
    SDL_GPUBuffer* m_lineBuffer = nullptr;
    SDL_GPUTransferBuffer* m_lineTransfer = nullptr;
    std::uint32_t m_lineCapacity = 0;

    std::deque<PendingFence> m_fences;
    RenderStats m_stats;
};

} // namespace tf
