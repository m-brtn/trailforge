# `cmake -P` patch for the pinned Dear ImGui source (FetchContent PATCH_COMMAND, cmake/Dependencies.cmake); run in
# the ImGui source directory. imgui_impl_sdlgpu3_shaders.h tests TARGET_OS_MAC before TARGET_OS_IPHONE, and
# TARGET_OS_MAC is 1 on every Apple OS, so iOS builds embed the macOS metallibs, which SDL GPU rejects on iOS (the
# pipeline is null and the first ImGui draw fails). On iOS the backend takes its MSL source instead (SDL compiles it
# when the device is created; SDL GPU's Metal backend always accepts MSL). macOS is unchanged. Idempotent.
set(_file backends/imgui_impl_sdlgpu3.cpp)
file(READ ${_file} _src)
set(_marker "TrailForge patch: iOS")
string(FIND "${_src}" "${_marker}" _found)
if(NOT _found EQUAL -1)
    return()
endif()
set(_old "        if (supported_formats & SDL_GPU_SHADERFORMAT_METALLIB)\n")
string(FIND "${_src}" "${_old}" _at)
if(_at EQUAL -1)
    message(FATAL_ERROR "ImguiIos.cmake: the metallib format check was not found in ${_file}")
endif()
string(REPLACE "${_old}"
       "        if ((supported_formats & SDL_GPU_SHADERFORMAT_METALLIB) && !TARGET_OS_IPHONE) // ${_marker}: MSL on iOS\n"
       _src "${_src}")
file(WRITE ${_file} "${_src}")
