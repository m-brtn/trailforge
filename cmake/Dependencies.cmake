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

# Physics is always needed (including for headless tests).
FetchContent_MakeAvailable(JoltPhysics)

if(TRAILFORGE_BUILD_APP)
    # imgui has no CMakeLists.txt, and glm's SOURCE_SUBDIR points to
    # a nonexistent folder, so MakeAvailable only downloads the sources.
    FetchContent_MakeAvailable(SDL3 imgui glm)

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
