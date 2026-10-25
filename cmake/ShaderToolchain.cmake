# Build-time shader toolchain (pinned archives + SHA256, like cmake/Dependencies.cmake).
#
# glslang (HLSL front end), SPIRV-Tools (legalization + validation), SPIRV-Headers and SPIRV-Cross (SPIR-V -> MSL)
# are built from source into the host tool `trailforge_shaderc` (tools/shaderc). All four are taken from the same
# Vulkan SDK release tag, so they agree on the SPIR-V grammar. On Windows the prebuilt DXC release (dxc.exe +
# dxil.dll for signing) compiles the same HLSL to DXIL; there is no DXC build for macOS, so DXIL exists only in
# Windows builds.
include(FetchContent)

# ---------------------------------------------------------------- vulkan-sdk-1.4.357.0
FetchContent_Declare(spirv_headers
    URL https://github.com/KhronosGroup/SPIRV-Headers/archive/29981f65241605e08b0ede4cfeb999fe3b723c6a.tar.gz
    URL_HASH SHA256=232899f1ad4104fb5bc377b94596c7621575eee62ad9a9e8f929b63a7dd8a7ad
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    EXCLUDE_FROM_ALL)
FetchContent_Declare(spirv_tools
    URL https://github.com/KhronosGroup/SPIRV-Tools/archive/9a49b0883b9b635689a85b5647dbfcb223268151.tar.gz
    URL_HASH SHA256=5cd1a935296c324e7bfa8f0e4c223742ac490d45d6af0577fa3c2cf7e1a7dd22
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    EXCLUDE_FROM_ALL)
FetchContent_Declare(glslang
    URL https://github.com/KhronosGroup/glslang/archive/168d452a4f460d24b588fed08477a81c44ee27a1.tar.gz
    URL_HASH SHA256=c167b2474af06cbab93abae8249efaf88fb79a09f30488e2a24d399e4258ce7a
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    EXCLUDE_FROM_ALL)
FetchContent_Declare(spirv_cross
    URL https://github.com/KhronosGroup/SPIRV-Cross/archive/6c09849fe88c48eaed08413aa022aaa136a3a057.tar.gz
    URL_HASH SHA256=b81b9956289950570953738e666a031ca32ff64e4fc925eba89f227c42109518
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    EXCLUDE_FROM_ALL)

set(SPIRV_HEADERS_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(SPIRV_HEADERS_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(SPIRV_SKIP_TESTS ON CACHE BOOL "" FORCE)
set(SPIRV_SKIP_EXECUTABLES ON CACHE BOOL "" FORCE)
set(SPIRV_WERROR OFF CACHE BOOL "" FORCE)
set(SPIRV_TOOLS_BUILD_STATIC ON CACHE BOOL "" FORCE)
set(SKIP_SPIRV_TOOLS_INSTALL ON CACHE BOOL "" FORCE)
set(ENABLE_OPT OFF CACHE BOOL "" FORCE)                 # glslang: legalization is run by trailforge_shaderc
set(ENABLE_HLSL ON CACHE BOOL "" FORCE)
set(ENABLE_GLSLANG_BINARIES OFF CACHE BOOL "" FORCE)
set(ENABLE_SPVREMAPPER OFF CACHE BOOL "" FORCE)
set(ENABLE_PCH OFF CACHE BOOL "" FORCE)
set(GLSLANG_TESTS OFF CACHE BOOL "" FORCE)
set(GLSLANG_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(BUILD_EXTERNAL OFF CACHE BOOL "" FORCE)
set(SPIRV_CROSS_CLI OFF CACHE BOOL "" FORCE)
set(SPIRV_CROSS_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(SPIRV_CROSS_ENABLE_HLSL OFF CACHE BOOL "" FORCE)
set(SPIRV_CROSS_ENABLE_CPP OFF CACHE BOOL "" FORCE)
set(SPIRV_CROSS_ENABLE_REFLECT OFF CACHE BOOL "" FORCE)
set(SPIRV_CROSS_ENABLE_C_API OFF CACHE BOOL "" FORCE)
set(SPIRV_CROSS_ENABLE_UTIL OFF CACHE BOOL "" FORCE)
set(SPIRV_CROSS_SKIP_INSTALL ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(spirv_headers spirv_tools glslang spirv_cross)

add_executable(trailforge_shaderc ${CMAKE_SOURCE_DIR}/tools/shaderc/ShaderCompiler.cpp)
target_link_libraries(trailforge_shaderc PRIVATE glslang glslang-default-resource-limits SPIRV-Tools-opt
                                                 SPIRV-Tools-static spirv-cross-msl)
target_compile_features(trailforge_shaderc PRIVATE cxx_std_20)
# glslang's public header hides the HLSL options unless the consumer defines ENABLE_HLSL as well.
target_compile_definitions(trailforge_shaderc PRIVATE ENABLE_HLSL=1)

# ---------------------------------------------------------------- DXC (Windows only), v1.9.2609
set(TRAILFORGE_DXC "")
if(CMAKE_HOST_WIN32)
    FetchContent_Declare(dxc
        URL https://github.com/microsoft/DirectXShaderCompiler/releases/download/v1.9.2609/dxc_2026_09_29.zip
        URL_HASH SHA256=ad31b1fc8443175d204f77a611fdb3ef2ec42759bdc2f1167368de24a4a7e7f1
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(dxc)
    if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "ARM64|arm64|aarch64")
        set(TRAILFORGE_DXC ${dxc_SOURCE_DIR}/bin/arm64/dxc.exe)
    else()
        set(TRAILFORGE_DXC ${dxc_SOURCE_DIR}/bin/x64/dxc.exe)
    endif()
    message(STATUS "TrailForge: DXC: ${TRAILFORGE_DXC}")
endif()
