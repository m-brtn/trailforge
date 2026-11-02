# `cmake -P` patch for the pinned SDL source (FetchContent PATCH_COMMAND, cmake/Dependencies.cmake); run in the SDL
# source directory. The iOS Simulator's Metal device reports only GPU family Apple2, and SDL GPU's Metal backend
#   1) requires MTLGPUFamilyApple3 on iOS, so SDL_CreateGPUDevice fails: the check is skipped and the families the
#      device reports are logged;
#   2) calls setDepthClipMode on every pipeline bind, which the Simulator does not support (Metal validation asserts,
#      without validation the GPU stalls): the call is skipped, so depth is always clipped (no depth clamp).
# Both changes are inside `#if TARGET_OS_SIMULATOR`; macOS and real iOS devices compile the original code. Idempotent.
set(_file src/gpu/metal/SDL_gpu_metal.m)
file(READ ${_file} _src)
set(_marker "TrailForge patch: iOS Simulator")
string(FIND "${_src}" "${_marker}" _found)
if(NOT _found EQUAL -1)
    return()
endif()
function(_tf_require text what)
    string(FIND "${_src}" "${text}" _at)
    if(_at EQUAL -1)
        message(FATAL_ERROR "SdlIosSimulator.cmake: ${what} was not found in ${_file}")
    endif()
endfunction()
set(_old "            hasHardwareSupport = [device supportsFamily:MTLGPUFamilyApple3];\n        }\n")
_tf_require("${_old}" "the Apple3 family check")
set(_old_clip "#ifndef SDL_PLATFORM_VISIONOS\n        [metalCommandBuffer->renderEncoder setDepthClipMode:")
_tf_require("${_old_clip}" "the setDepthClipMode call")
string(REPLACE "${_old_clip}"
       "#if !defined(SDL_PLATFORM_VISIONOS) && !TARGET_OS_SIMULATOR // ${_marker}: no depth clip mode\n        [metalCommandBuffer->renderEncoder setDepthClipMode:"
       _src "${_src}")
set(_new "            hasHardwareSupport = [device supportsFamily:MTLGPUFamilyApple3];
#if TARGET_OS_SIMULATOR
            // ${_marker} (cmake/patches/SdlIosSimulator.cmake): the Simulator device does not report Apple3.
            SDL_Log(\"SDL_GPU Metal (Simulator): Apple2 %d Apple3 %d Apple7 %d Mac2 %d Common3 %d, BC %d\",
                    (int)[device supportsFamily:MTLGPUFamilyApple2], (int)[device supportsFamily:MTLGPUFamilyApple3],
                    (int)[device supportsFamily:MTLGPUFamilyApple7], (int)[device supportsFamily:MTLGPUFamilyMac2],
                    (int)[device supportsFamily:MTLGPUFamilyCommon3], (int)device.supportsBCTextureCompression);
            hasHardwareSupport = true;
#endif
        }
")
string(REPLACE "${_old}" "${_new}" _src "${_src}")
file(WRITE ${_file} "${_src}")
