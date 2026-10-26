# All dependencies are pinned to an archive of a specific commit + SHA256.
# Floating branches (main/master) are not used.
include(FetchContent)

# GitHub serves commit archives deterministically; if the hash ever
# changes on GitHub's side, the build fails with an explicit URL_HASH error.

# ---------------------------------------------------------------- SDL 3
# release-3.4.18
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
set(SDL_TESTS OFF CACHE BOOL "" FORCE)
set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SDL_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(SDL3
    URL https://github.com/libsdl-org/SDL/archive/829a65d769d935c4852f8159e964312c0957260a.tar.gz
    URL_HASH SHA256=48e15cf1a6928453d77afa955d1fdfbbca9169e2bc3d76d111acb9b8bb076d66
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    EXCLUDE_FROM_ALL)

# ---------------------------------------------------------------- Jolt Physics
# v5.6.0
set(DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)
set(GENERATE_DEBUG_SYMBOLS ON CACHE BOOL "" FORCE)
set(OVERRIDE_CXX_FLAGS OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
set(FLOATING_POINT_EXCEPTIONS_ENABLED OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
set(CPP_RTTI_ENABLED ON CACHE BOOL "" FORCE)
set(CPP_EXCEPTIONS_ENABLED ON CACHE BOOL "" FORCE)
set(DEBUG_RENDERER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "" FORCE)
set(PROFILER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "" FORCE)
set(ENABLE_OBJECT_STREAM OFF CACHE BOOL "" FORCE)
set(JPH_USE_DX12 OFF CACHE BOOL "" FORCE)
set(JPH_USE_VK OFF CACHE BOOL "" FORCE)
set(JPH_USE_MTL OFF CACHE BOOL "" FORCE)
set(JPH_USE_CPU_COMPUTE OFF CACHE BOOL "" FORCE)
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
set(USE_ASSERTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(JoltPhysics
    URL https://github.com/jrouwe/JoltPhysics/archive/e77f175595e64cb44218cc9d9d56fc365ad0e36a.tar.gz
    URL_HASH SHA256=1f32328fb763135de10a244568d6ccb2ed9b1e6593fafe6dc6db5b2719d330bd
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR Build
    EXCLUDE_FROM_ALL)

# ---------------------------------------------------------------- Dear ImGui
# v1.92.9b (no CMake of its own; the target is built below)
FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/f1cc2ae15e53a861a874c3034aae6798fde194ab.tar.gz
    URL_HASH SHA256=f30be7f3e2136615b1848a9afb72f6dfd8a60820ddf1c1a85997bf3b722e7cfc
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)

# ---------------------------------------------------------------- GLM
# 1.0.3 (used header-only)
FetchContent_Declare(glm
    URL https://github.com/g-truc/glm/archive/8d1fd52e5ab5590e2c81768ace50c72bae28f2ed.tar.gz
    URL_HASH SHA256=36eefbae41503e6822e8d9e1b4cc85c6a6621f17676f6258aaf0a64c2de0a09c
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR header-only-no-cmake)

# ---------------------------------------------------------------- miniaudio
# 0.11.25 (single-file library; playback device, WAV decoding and encoding). Its own CMakeLists.txt is not used:
# SOURCE_SUBDIR points to a nonexistent folder and the library is built below from miniaudio.c.
FetchContent_Declare(miniaudio
    URL https://github.com/mackron/miniaudio/archive/9634bedb5b5a2ca38c1ee7108a9358a4e233f14d.tar.gz
    URL_HASH SHA256=1a3a79b80fc6f0b0cc155e28b954a598e0ddfa2db64e2afa8466be88c476fa55
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no-cmake)

# ---------------------------------------------------------------- cgltf
# v1.15 (single-header glTF 2.0 parser; reads the converted models in assets/models, including in headless builds:
# the level bake takes tree trunks and rock collision from them). Its CMakeLists.txt is not used.
FetchContent_Declare(cgltf
    URL https://github.com/jkuhlmann/cgltf/archive/360db1a95480fe102ae9c69b27c5d101167ff5ba.tar.gz
    URL_HASH SHA256=445d135cf793232ae6a585ca1404e4ff28d4f4dbca070689034fe780370ac84a
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no-cmake)

# Physics and the glTF parser are always needed (including for headless tests).
FetchContent_MakeAvailable(JoltPhysics cgltf)

if(TRAILFORGE_BUILD_APP)
    # imgui has no CMakeLists.txt, and glm's SOURCE_SUBDIR points to
    # a nonexistent folder, so MakeAvailable only downloads the sources.
    FetchContent_MakeAvailable(SDL3 imgui glm miniaudio)

    # Only the low-level API is used (device, decoder, encoder); the engine, node graph, resource manager and
    # the FLAC decoder are compiled out. MP3 stays in for the music (assets/music/intro.mp3). The defines are PUBLIC so the header sees the same configuration.
    add_library(miniaudio_lib STATIC ${miniaudio_SOURCE_DIR}/miniaudio.c)
    target_include_directories(miniaudio_lib SYSTEM PUBLIC ${miniaudio_SOURCE_DIR})
    target_compile_definitions(miniaudio_lib PUBLIC
        MA_NO_ENGINE MA_NO_NODE_GRAPH MA_NO_RESOURCE_MANAGER MA_NO_FLAC MA_NO_GENERATION)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(miniaudio_lib PRIVATE -w)
    endif()
    if(UNIX AND NOT APPLE)
        target_link_libraries(miniaudio_lib PUBLIC ${CMAKE_DL_LIBS} pthread m)
    endif()

    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlgpu3.cpp)
    target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
    target_link_libraries(imgui PUBLIC SDL3::SDL3-static)
    target_compile_features(imgui PUBLIC cxx_std_20)

    add_library(glm_headers INTERFACE)
    target_include_directories(glm_headers SYSTEM INTERFACE ${glm_SOURCE_DIR})
    target_compile_definitions(glm_headers INTERFACE
        GLM_FORCE_DEPTH_ZERO_TO_ONE   # SDL GPU (Metal/D3D12/Vulkan): NDC z in [0, 1]
        GLM_FORCE_RADIANS)
endif()
