#pragma once

#include "render/Mesh.h"

#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_video.h>
#include <glm/glm.hpp>

#include <cstdint>
#include <span>

struct ImDrawData;

namespace tf {

// Uniform block layouts; match FrameUniforms/DrawUniforms in shaders/scene.metal.
struct FrameUniforms {
    glm::mat4 viewProj;
    glm::vec4 lightDir;
    glm::vec4 lightColor;
    glm::vec4 skyColor;
    glm::vec4 groundColor;
    glm::vec4 cameraPos;
    glm::vec4 fogParams;
};

struct DrawUniforms {
    glm::mat4 model;
    glm::vec4 tint;
    glm::vec4 params;
};

// GPU vertex buffer with explicit ownership (released in the destructor).
class GpuMesh {
public:
    GpuMesh() = default;
    GpuMesh(SDL_GPUDevice* device, SDL_GPUBuffer* buffer, std::uint32_t vertexCount)
        : m_device(device), m_buffer(buffer), m_count(vertexCount) {}
    ~GpuMesh() { release(); }
    GpuMesh(GpuMesh&& o) noexcept { *this = std::move(o); }
    GpuMesh& operator=(GpuMesh&& o) noexcept;
    GpuMesh(const GpuMesh&) = delete;
    GpuMesh& operator=(const GpuMesh&) = delete;

    SDL_GPUBuffer* buffer() const { return m_buffer; }
    std::uint32_t vertexCount() const { return m_count; }
    void release();

private:
    SDL_GPUDevice* m_device = nullptr;
    SDL_GPUBuffer* m_buffer = nullptr;
    std::uint32_t m_count = 0;
};

struct DrawItem {
    const GpuMesh* mesh = nullptr;
    glm::mat4 model{1.0f};
    glm::vec4 tint{1.0f};
    float gridStrength = 0.0f;
};

struct FrameDesc {
    glm::mat4 view{1.0f};
    glm::mat4 proj{1.0f};
    glm::vec3 cameraPos{0.0f};
    glm::vec3 clearColor{0.55f, 0.68f, 0.82f};
    std::span<const DrawItem> draws;
    std::span<const LineVertex> lines;
    bool linesOnTop = false; // true: lines without depth test (visible through the body)
    ImDrawData* ui = nullptr;
    const char* capturePath = nullptr; // non-null: save this frame as BMP or PNG (by extension)
};

// SDL GPU renderer (main platform: Metal). Owns the device, pipelines,
// depth buffer and dynamic line buffer. The window is not owned by the renderer.
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

    // Draws a frame. Returns false if the swapchain texture is unavailable (window minimized).
    bool drawFrame(const FrameDesc& frame);

    void setVSync(bool on);

    // Size of the current depth texture (= swapchain size of the last frame) and how many times it was created.
    std::uint32_t depthWidth() const { return m_depthW; }
    std::uint32_t depthHeight() const { return m_depthH; }
    int depthCreations() const { return m_depthCreations; }

private:
    SDL_GPUShader* createShader(const char* entry, SDL_GPUShaderStage stage, std::uint32_t uniformBuffers);
    void createPipelines();
    void ensureDepthTexture(std::uint32_t w, std::uint32_t h);
    void ensureLineCapacity(std::uint32_t bytes);
    bool saveCapture(SDL_GPUCommandBuffer* cmd, SDL_GPUTexture* src, std::uint32_t w, std::uint32_t h, const char* path);

    SDL_Window* m_window = nullptr;
    SDL_GPUDevice* m_device = nullptr;
    const char* m_driver = "";
    const char* m_shaderFormatName = "";
    SDL_GPUShaderFormat m_shaderFormat = SDL_GPU_SHADERFORMAT_INVALID;
    SDL_GPUGraphicsPipeline* m_litPipeline = nullptr;
    SDL_GPUGraphicsPipeline* m_linePipeline = nullptr;
    SDL_GPUGraphicsPipeline* m_lineOnTopPipeline = nullptr;
    SDL_GPUTextureFormat m_depthFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    SDL_GPUTexture* m_depth = nullptr;
    std::uint32_t m_depthW = 0, m_depthH = 0;
    int m_depthCreations = 0;
    SDL_GPUBuffer* m_lineBuffer = nullptr;
    SDL_GPUTransferBuffer* m_lineTransfer = nullptr;
    std::uint32_t m_lineCapacity = 0;
};

} // namespace tf
