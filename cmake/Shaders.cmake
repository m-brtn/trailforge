# Shader pipeline.
#
# Source of truth: shaders/scene.hlsl (HLSL, one entry point per shader). At build time every entry point is
# compiled by the host tool `trailforge_shaderc` (tools/shaderc, built from the pinned glslang, SPIRV-Tools and
# SPIRV-Cross sources in cmake/ShaderToolchain.cmake):
#   - SPIR-V (Vulkan), on every platform;
#   - macOS: SPIR-V -> MSL (SPIRV-Cross, SDL GPU resource slots) -> xcrun metal -> one .metallib per entry point.
#     Without the Metal toolchain the generated MSL source is embedded instead and SDL compiles it at startup;
#   - Windows: DXIL (D3D12) with the pinned DXC release (dxc.exe compiles the same HLSL directly).
# All blobs are embedded into the binary (cmake/EmbedShaders.cmake -> render/ShaderBlobs.h); no shader files are
# read at runtime. The renderer picks the format the SDL GPU device reports (SDL_GetGPUShaderFormats).

include(${CMAKE_CURRENT_LIST_DIR}/ShaderToolchain.cmake)

set(TRAILFORGE_SHADER_SOURCES "")
set(TRAILFORGE_SHADER_DEFINES "")

set(_hlsl ${CMAKE_SOURCE_DIR}/shaders/scene.hlsl)
set(_gen_dir ${CMAKE_BINARY_DIR}/generated/shaders)
file(MAKE_DIRECTORY ${_gen_dir})

# entry:stage. Resource counts are given where the pipelines are created (src/render/Renderer.cpp).
set(TRAILFORGE_SHADER_ENTRIES
    lit_vs:vertex lit_fs:fragment shadow_vs:vertex shadow_fs:fragment tree_vs:vertex tree_shadow_vs:vertex
    fullscreen_vs:vertex sky_fs:fragment water_fs:fragment grass_vs:vertex grass_fs:fragment mark_vs:vertex
    mark_fs:fragment mark_mask_vs:vertex mark_mask_fs:fragment spray_vs:vertex spray_fs:fragment line_vs:vertex
    line_fs:fragment bloom_down_fs:fragment bloom_up_fs:fragment tonemap_fs:fragment fxaa_fs:fragment)

set(_metal_ok OFF)
if(APPLE)
    execute_process(COMMAND xcrun -sdk macosx -f metal
        OUTPUT_VARIABLE _metal_path OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _metal_res ERROR_QUIET)
    if(_metal_res EQUAL 0 AND EXISTS "${_metal_path}")
        set(_metal_ok ON)
        message(STATUS "TrailForge: Metal compiler: ${_metal_path} - MSL is precompiled into metallibs")
    else()
        message(WARNING "TrailForge: xcrun metal not found - the generated MSL source is embedded (compiled at startup). "
                        "To install the toolchain: xcodebuild -downloadComponent MetalToolchain")
    endif()
endif()

set(_formats spirv)
if(APPLE)
    if(_metal_ok)
        set(_formats spirv metallib)
        list(APPEND TRAILFORGE_SHADER_DEFINES TRAILFORGE_HAS_METALLIB=1)
    else()
        set(_formats spirv msl)
    endif()
endif()
if(TRAILFORGE_DXC)
    list(APPEND _formats dxil)
endif()

set(_blobs "")
set(_entries "")
foreach(item IN LISTS TRAILFORGE_SHADER_ENTRIES)
    string(REPLACE ":" ";" pair ${item})
    list(GET pair 0 entry)
    list(GET pair 1 stage)
    list(APPEND _entries ${entry})

    set(_outs ${_gen_dir}/${entry}.spv)
    set(_msl_args "")
    if(APPLE)
        list(APPEND _outs ${_gen_dir}/${entry}.msl)
        set(_msl_args --msl ${_gen_dir}/${entry}.msl)
    endif()
    add_custom_command(
        OUTPUT ${_outs}
        COMMAND trailforge_shaderc --input ${_hlsl} --entry ${entry} --stage ${stage} --spirv ${_gen_dir}/${entry}.spv
                ${_msl_args}
        DEPENDS ${_hlsl} trailforge_shaderc
        COMMENT "Compiling scene.hlsl:${entry} (${stage})"
        VERBATIM)
    list(APPEND _blobs ${_outs})

    if(APPLE AND _metal_ok)
        add_custom_command(
            OUTPUT ${_gen_dir}/${entry}.metallib
            COMMAND xcrun -sdk macosx metal -x metal -c ${_gen_dir}/${entry}.msl -o ${_gen_dir}/${entry}.air
                    -mmacosx-version-min=14.0
            COMMAND xcrun -sdk macosx metallib ${_gen_dir}/${entry}.air -o ${_gen_dir}/${entry}.metallib
            DEPENDS ${_gen_dir}/${entry}.msl
            COMMENT "Compiling ${entry}.msl -> ${entry}.metallib"
            VERBATIM)
        list(APPEND _blobs ${_gen_dir}/${entry}.metallib)
    endif()

    if(TRAILFORGE_DXC)
        if(stage STREQUAL "vertex")
            set(_profile vs_6_0)
            set(_stage_define TF_STAGE_VERTEX=1)
        else()
            set(_profile ps_6_0)
            set(_stage_define TF_STAGE_FRAGMENT=1)
        endif()
        add_custom_command(
            OUTPUT ${_gen_dir}/${entry}.dxil
            COMMAND ${TRAILFORGE_DXC} -nologo -T ${_profile} -E ${entry} -D TF_ENTRY_${entry}=1 -D ${_stage_define}
                    -HV 2021 -O3 -Fo ${_gen_dir}/${entry}.dxil ${_hlsl}
            DEPENDS ${_hlsl}
            COMMENT "Compiling scene.hlsl:${entry} -> DXIL (${_profile})"
            VERBATIM)
        list(APPEND _blobs ${_gen_dir}/${entry}.dxil)
    endif()
endforeach()

string(REPLACE ";" "," _entries_csv "${_entries}")
string(REPLACE ";" "," _formats_csv "${_formats}")
add_custom_command(
    OUTPUT ${_gen_dir}/shader_blobs.cpp
    COMMAND ${CMAKE_COMMAND} -DENTRIES=${_entries_csv} -DFORMATS=${_formats_csv} -DGEN_DIR=${_gen_dir}
            -DOUTPUT=${_gen_dir}/shader_blobs.cpp -P ${CMAKE_SOURCE_DIR}/cmake/EmbedShaders.cmake
    DEPENDS ${_blobs} ${CMAKE_SOURCE_DIR}/cmake/EmbedShaders.cmake
    COMMENT "Embedding shader blobs (${_formats_csv})"
    VERBATIM)
list(APPEND TRAILFORGE_SHADER_SOURCES ${_gen_dir}/shader_blobs.cpp)
