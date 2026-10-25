#pragma once

// Compiled shaders embedded at build time from shaders/scene.hlsl (cmake/Shaders.cmake, cmake/EmbedShaders.cmake).
// One row per entry point; a format that was not built for this platform has data == nullptr.

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace tf {

struct ShaderCode {
    const std::uint8_t* data;
    std::size_t size;
};

struct ShaderBlob {
    const char* entry;
    ShaderCode spirv;    // Vulkan
    ShaderCode msl;      // Metal source (only without the Metal toolchain at build time)
    ShaderCode metallib; // Metal, precompiled
    ShaderCode dxil;     // D3D12 (Windows builds)
};

extern const ShaderBlob g_shaderBlobs[];
extern const std::size_t g_shaderBlobCount;

inline const ShaderBlob* findShaderBlob(const char* entry) {
    for (std::size_t i = 0; i < g_shaderBlobCount; ++i)
        if (std::strcmp(g_shaderBlobs[i].entry, entry) == 0) return &g_shaderBlobs[i];
    return nullptr;
}

} // namespace tf
