# Shader delivery.
#
# Source of truth: shaders/scene.metal (MSL). On macOS it is
#   1) compiled at build time: xcrun metal -> .air -> xcrun metallib -> .metallib;
#   2) the metallib is embedded into the binary as a byte array (cmake/EmbedFile.cmake).
# If the Metal toolchain is unavailable (xcrun metal not found), the MSL source is embedded instead,
# and SDL GPU compiles it at startup (SDL_GPU_SHADERFORMAT_MSL).
# No shader files are loaded from disk at runtime.
#
# Windows/DX12 and Linux/Vulkan: not implemented and not tested. They need an
# HLSL/SPIR-V version of the same shaders (e.g. via SDL_shadercross), see README.

set(TRAILFORGE_SHADER_SOURCES "")
set(TRAILFORGE_SHADER_DEFINES "")

set(_msl_src ${CMAKE_SOURCE_DIR}/shaders/scene.metal)
set(_gen_dir ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${_gen_dir})

if(NOT APPLE)
    message(WARNING "TrailForge: shaders exist only for Metal; on this platform the application will not be able to create pipelines.")
endif()

# The MSL source is always embedded (fallback and for debugging).
add_custom_command(
    OUTPUT ${_gen_dir}/scene_msl.cpp
    COMMAND ${CMAKE_COMMAND} -DINPUT=${_msl_src} -DOUTPUT=${_gen_dir}/scene_msl.cpp
            -DSYMBOL=g_scene_msl -P ${CMAKE_SOURCE_DIR}/cmake/EmbedFile.cmake
    DEPENDS ${_msl_src} ${CMAKE_SOURCE_DIR}/cmake/EmbedFile.cmake
    COMMENT "Embedding scene.metal source")
list(APPEND TRAILFORGE_SHADER_SOURCES ${_gen_dir}/scene_msl.cpp)

if(APPLE)
    execute_process(COMMAND xcrun -sdk macosx -f metal
        OUTPUT_VARIABLE _metal_path OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _metal_res ERROR_QUIET)
    if(_metal_res EQUAL 0 AND EXISTS "${_metal_path}")
        message(STATUS "TrailForge: Metal compiler: ${_metal_path} - shaders will be precompiled into a metallib")
        add_custom_command(
            OUTPUT ${_gen_dir}/scene.metallib
            COMMAND xcrun -sdk macosx metal -c ${_msl_src} -o ${_gen_dir}/scene.air -mmacosx-version-min=14.0
            COMMAND xcrun -sdk macosx metallib ${_gen_dir}/scene.air -o ${_gen_dir}/scene.metallib
            DEPENDS ${_msl_src}
            COMMENT "Compiling scene.metal -> scene.metallib")
        add_custom_command(
            OUTPUT ${_gen_dir}/scene_metallib.cpp
            COMMAND ${CMAKE_COMMAND} -DINPUT=${_gen_dir}/scene.metallib -DOUTPUT=${_gen_dir}/scene_metallib.cpp
                    -DSYMBOL=g_scene_metallib -P ${CMAKE_SOURCE_DIR}/cmake/EmbedFile.cmake
            DEPENDS ${_gen_dir}/scene.metallib ${CMAKE_SOURCE_DIR}/cmake/EmbedFile.cmake
            COMMENT "Embedding scene.metallib")
        list(APPEND TRAILFORGE_SHADER_SOURCES ${_gen_dir}/scene_metallib.cpp)
        list(APPEND TRAILFORGE_SHADER_DEFINES TRAILFORGE_HAS_METALLIB=1)
    else()
        message(WARNING "TrailForge: xcrun metal not found - the embedded MSL source will be used (compiled at startup). "
                        "To install the toolchain: xcodebuild -downloadComponent MetalToolchain")
    endif()
endif()
