cmake_minimum_required(VERSION 3.24)
project(TrailForge VERSION 0.13.0 LANGUAGES C CXX)

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

# Jolt builds with the static MSVC runtime (USE_STATIC_MSVC_RUNTIME_LIBRARY); everything else must match. It also
# means the Windows package needs no Visual C++ redistributable.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

include(cmake/Dependencies.cmake)

# ------------------------------------------------------------------ simulation
# Physics, vehicle, proving ground. Does not depend on SDL/rendering, runs headless.
add_library(trailforge_sim STATIC
    src/core/Json.cpp
    src/core/Ktx2.cpp
    src/core/TerrainTextureSet.cpp
    src/physics/PhysicsWorld.cpp
    src/track/SurfaceProperties.cpp
    src/track/TestTrack.cpp
    src/track/Level.cpp
    src/track/TerrainTiles.cpp
    src/vehicle/Vehicle.cpp
    src/vehicle/DrivingAids.cpp
    src/vehicle/ElectricDrive.cpp
    src/vehicle/VehicleDefinition.cpp
    src/vehicle/VehicleSetup.cpp
    src/sim/Simulation.cpp
    src/sim/BodyImpacts.cpp
    src/sim/TireMarks.cpp
    src/sim/WheelSpray.cpp
    src/sim/BodyMud.cpp)
target_include_directories(trailforge_sim PUBLIC src)
target_link_libraries(trailforge_sim PUBLIC Jolt)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(trailforge_sim PRIVATE -Wall -Wextra)
endif()

# ------------------------------------------------------------------ audio model
# Manifest, parameter mapping, mixer and the per-tick vehicle feed. No device and no miniaudio: runs headless.
add_library(trailforge_audio STATIC
    src/audio/AudioManifest.cpp
    src/audio/AudioMapping.cpp
    src/audio/AudioMixer.cpp
    src/audio/VehicleAudioFeed.cpp)
target_include_directories(trailforge_audio PUBLIC src)
target_link_libraries(trailforge_audio PUBLIC trailforge_sim)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(trailforge_audio PRIVATE -Wall -Wextra)
endif()

# ------------------------------------------------------------------ game flow
# Menu / screen state machine and the settings file. Plain C++ (no SDL), tested headless.
add_library(trailforge_flow STATIC
    src/app/GameFlow.cpp
    src/app/GameSettings.cpp
    src/app/Garage.cpp
    src/input/ControlLayout.cpp)
target_include_directories(trailforge_flow PUBLIC src)
target_link_libraries(trailforge_flow PUBLIC trailforge_sim)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(trailforge_flow PRIVATE -Wall -Wextra)
endif()

add_executable(trailforge_probe tools/SimProbe.cpp)
target_link_libraries(trailforge_probe PRIVATE trailforge_audio)
target_compile_definitions(trailforge_probe PRIVATE TRAILFORGE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# ------------------------------------------------------------------ application
if(TRAILFORGE_BUILD_APP)
    include(cmake/Shaders.cmake)
    include(cmake/AppIcon.cmake)

    add_executable(trailforge
        src/app/main.cpp
        src/app/Application.cpp
        src/app/Scenario.cpp
        src/app/AppIcon.cpp
        src/app/Paths.cpp
        src/audio/AudioSystem.cpp
        src/input/Input.cpp
        src/render/Renderer.cpp
        src/render/Mesh.cpp
        src/render/GroundMaps.cpp
        src/render/VehicleModel.cpp
        src/render/StyleKit.cpp
        src/render/Scenery.cpp
        src/render/LevelScene.cpp
        src/render/TimeOfDay.cpp
        src/render/Camera.cpp
        src/render/DebugDraw.cpp
        src/ui/DebugUI.cpp
        src/ui/MenuUI.cpp
        ${TRAILFORGE_SHADER_SOURCES}
        ${TRAILFORGE_ICON_SOURCES})
    # trailforge_audio brings trailforge_sim.
    target_link_libraries(trailforge PRIVATE trailforge_audio trailforge_flow miniaudio_lib SDL3::SDL3-static imgui glm_headers)
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
    # Images the menus load at runtime (assets/images/ next to the executable). The PNG sources stay in
    # external_assets/images (PNGs elsewhere are git-ignored), so they are copied into the build tree here.
    configure_file(${CMAKE_SOURCE_DIR}/external_assets/images/Logo.png ${CMAKE_BINARY_DIR}/assets/images/logo.png
                   COPYONLY)
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
    target_compile_definitions(trailforge_tests PRIVATE TRAILFORGE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
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
            auto_reverse
            surface_launch_grip
            skid_mark_profile suv_marks_keyboard launch_marks_proportional tire_marks_boundary wheel_spray
            abs_prevents_lock
            diff_locks_climb
            traction_control_mud driving_aids_logic friction_ellipse
            tread_soil_surfaces body_mud)
        add_test(NAME ${test_name} COMMAND trailforge_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
    endforeach()

    # Vehicle definitions (every assets/vehicles/*/vehicle.json and tests/data/vehicles), layouts, garage setups.
    add_executable(trailforge_vehicle_tests tests/VehicleTests.cpp)
    target_link_libraries(trailforge_vehicle_tests PRIVATE trailforge_sim)
    target_compile_definitions(trailforge_vehicle_tests PRIVATE TRAILFORGE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    foreach(test_name
            vehicle_catalog_sweep
            vehicle_definition_validation
            drivetrain_layouts
            garage_setups
            suv_definition_matches_builtin
            electric_definition_validation
            ev_motor_curve
            ev_torque_split
            ev_regen_blend
            ev_battery
            ev_simulation)
        add_test(NAME ${test_name} COMMAND trailforge_vehicle_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 300)
    endforeach()

    # Open-world level: bake, physics tiles and streaming, roads, water, both vehicles on Exploration Peaks; terrain
    # texture files (KTX2 reader, shipped BC7 maps).
    add_executable(trailforge_level_tests tests/LevelTests.cpp)
    target_link_libraries(trailforge_level_tests PRIVATE trailforge_sim)
    target_compile_definitions(trailforge_level_tests PRIVATE TRAILFORGE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    foreach(test_name
            level_heightfield_sampling
            level_tile_streaming
            level_road_carving
            level_surfaces_water
            level_vehicles
            level_prop_winding
            level_bridges
            level_placement
            level_grass_clear
            terrain_textures_ktx2
            terrain_textures_assets)
        add_test(NAME ${test_name} COMMAND trailforge_level_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 300)
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
            audio_skid_gating
            audio_squeal_release
            audio_tire_profiles
            audio_electric
            audio_music)
        add_test(NAME ${test_name} COMMAND trailforge_audio_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
    endforeach()

    # Game flow: settings file, menu navigation, pause.
    add_executable(trailforge_app_tests tests/AppTests.cpp)
    target_link_libraries(trailforge_app_tests PRIVATE trailforge_flow trailforge_sim)
    target_compile_definitions(trailforge_app_tests PRIVATE TRAILFORGE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    foreach(test_name settings_roundtrip menu_navigation pause_freezes_ticks garage_model vehicle_page controls_layout pad_glyph_detect
                         callout_spread)
        add_test(NAME ${test_name} COMMAND trailforge_app_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
    endforeach()

    # Procedural vehicle model: parts and wheel clearance (CPU mesh only; GLM comes with the app build).
    if(TRAILFORGE_BUILD_APP)
        add_executable(trailforge_model_tests tests/ModelTests.cpp src/render/Mesh.cpp src/render/StyleKit.cpp
                                              src/render/VehicleModel.cpp)
        target_include_directories(trailforge_model_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
        target_link_libraries(trailforge_model_tests PRIVATE trailforge_sim glm_headers)
        target_compile_definitions(trailforge_model_tests PRIVATE TRAILFORGE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
        foreach(test_name model_parts model_wheel_size model_wheel_clearance)
            add_test(NAME ${test_name} COMMAND trailforge_model_tests ${test_name})
            set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
        endforeach()

        # Camera math: hood mount, right-stick orbit and look-around (no window, no gamepad).
        add_executable(trailforge_camera_tests tests/CameraTests.cpp src/render/Camera.cpp)
        target_include_directories(trailforge_camera_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
        target_link_libraries(trailforge_camera_tests PRIVATE trailforge_flow glm_headers)
        foreach(test_name camera_hood_mount camera_stick_dead_zone camera_stick_orbit camera_hood_look)
            add_test(NAME ${test_name} COMMAND trailforge_camera_tests ${test_name})
            set_tests_properties(${test_name} PROPERTIES TIMEOUT 60)
        endforeach()
    endif()
endif()
