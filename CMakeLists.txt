cmake_minimum_required(VERSION 3.24)
project(TrailForge VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(TRAILFORGE_BUILD_APP "Build the windowed application (SDL3/SDL GPU/ImGui)" ON)
option(TRAILFORGE_BUILD_TESTS "Build the headless physics tests" ON)
# Packaged builds (tools/release/package_macos.sh) read assets only from the bundle / next to the executable,
# never from the source tree path compiled into development builds.
option(TRAILFORGE_PACKAGED "Build for distribution: no source-tree asset fallback" OFF)

if(APPLE)
    enable_language(OBJC)
endif()

include(cmake/Dependencies.cmake)

# ------------------------------------------------------------------ simulation
# Physics, vehicle, proving ground. Does not depend on SDL/rendering, runs headless.
add_library(trailforge_sim STATIC
    src/physics/PhysicsWorld.cpp
    src/track/SurfaceProperties.cpp
    src/track/TestTrack.cpp
    src/vehicle/Vehicle.cpp
    src/sim/Simulation.cpp
    src/sim/BodyImpacts.cpp
    src/sim/TireMarks.cpp)
target_include_directories(trailforge_sim PUBLIC src)
target_link_libraries(trailforge_sim PUBLIC Jolt)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(trailforge_sim PRIVATE -Wall -Wextra)
endif()

# ------------------------------------------------------------------ audio model
# Manifest, parameter mapping, mixer and the per-tick vehicle feed. No device and no miniaudio: runs headless.
add_library(trailforge_audio STATIC
    src/audio/Json.cpp
    src/audio/AudioManifest.cpp
    src/audio/AudioMapping.cpp
    src/audio/AudioMixer.cpp
    src/audio/VehicleAudioFeed.cpp)
target_include_directories(trailforge_audio PUBLIC src)
target_link_libraries(trailforge_audio PUBLIC trailforge_sim)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(trailforge_audio PRIVATE -Wall -Wextra)
endif()

add_executable(trailforge_probe tools/SimProbe.cpp)
target_link_libraries(trailforge_probe PRIVATE trailforge_audio)

# ------------------------------------------------------------------ application
if(TRAILFORGE_BUILD_APP)
    include(cmake/Shaders.cmake)
    include(cmake/AppIcon.cmake)

    add_executable(trailforge
        src/app/main.cpp
        src/app/Application.cpp
        src/app/Scenario.cpp
        src/app/AppIcon.cpp
        src/audio/AudioSystem.cpp
        src/input/Input.cpp
        src/render/Renderer.cpp
        src/render/Mesh.cpp
        src/render/GroundMaps.cpp
        src/render/VehicleModel.cpp
        src/render/Scenery.cpp
        src/render/TimeOfDay.cpp
        src/render/Camera.cpp
        src/render/DebugDraw.cpp
        src/ui/DebugUI.cpp
        ${TRAILFORGE_SHADER_SOURCES}
        ${TRAILFORGE_ICON_SOURCES})
    # trailforge_audio brings trailforge_sim.
    target_link_libraries(trailforge PRIVATE trailforge_audio miniaudio_lib SDL3::SDL3-static imgui glm_headers)
    if(APPLE)
        target_sources(trailforge PRIVATE src/render/GpuTimingMetal.m)
        target_link_libraries(trailforge PRIVATE "-framework Metal")
    endif()
    target_compile_definitions(trailforge PRIVATE ${TRAILFORGE_SHADER_DEFINES} ${TRAILFORGE_ICON_DEFINES}
                                                  TRAILFORGE_VERSION="${PROJECT_VERSION}")
    if(TRAILFORGE_PACKAGED)
        target_compile_definitions(trailforge PRIVATE TRAILFORGE_PACKAGED=1)
    else()
        target_compile_definitions(trailforge PRIVATE TRAILFORGE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    endif()
    trailforge_add_macos_bundle(trailforge)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(trailforge PRIVATE -Wall -Wextra)
    endif()
endif()

# ------------------------------------------------------------------ tests
if(TRAILFORGE_BUILD_TESTS)
    enable_testing()
    add_executable(trailforge_tests tests/SimTests.cpp)
    target_link_libraries(trailforge_tests PRIVATE trailforge_sim)
    foreach(test_name
            settles_on_flat_ground
            throttle_moves_vehicle
            brake_reduces_speed
            lifted_wheel_loses_load
            reset_restores_state
            frame_schedule_independence
            corner_load_balance
            brake_holds_in_low_range
            tire_marks_surfaces
            auto_reverse)
        add_test(NAME ${test_name} COMMAND trailforge_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
    endforeach()

    # Audio parameter mapping and the offline mixer (no audio device).
    add_executable(trailforge_audio_tests tests/AudioTests.cpp)
    target_link_libraries(trailforge_audio_tests PRIVATE trailforge_audio)
    target_compile_definitions(trailforge_audio_tests PRIVATE TRAILFORGE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    foreach(test_name
            audio_engine_layers
            audio_tire_layers
            audio_manifest
            audio_feed_and_mixer
            audio_skid_gating)
        add_test(NAME ${test_name} COMMAND trailforge_audio_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
    endforeach()
endif()
