cmake_minimum_required(VERSION 3.24)
project(TrailForge VERSION 0.29.0 LANGUAGES C CXX)

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
    src/core/ModelAsset.cpp
    src/core/TerrainTextureSet.cpp
    src/physics/PhysicsWorld.cpp
    src/track/SurfaceProperties.cpp
    src/track/TestTrack.cpp
    src/track/MiniTrack.cpp
    src/track/Level.cpp
    src/track/LevelFiles.cpp
    src/track/BuildingKit.cpp
    src/track/TerrainTiles.cpp
    src/vehicle/Vehicle.cpp
    src/vehicle/DrivingAids.cpp
    src/vehicle/ElectricDrive.cpp
    src/vehicle/VehicleDefinition.cpp
    src/vehicle/VehicleSetup.cpp
    src/vehicle/TireModel.cpp
    src/sim/Simulation.cpp
    src/sim/Autopilot.cpp
    src/sim/LapTimer.cpp
    src/sim/DriftScore.cpp
    src/sim/BodyImpacts.cpp
    src/sim/TireMarks.cpp
    src/sim/WheelSpray.cpp
    src/sim/BodyMud.cpp
    src/vehicle/VehicleBlueprint.cpp
    src/vehicle/VehicleBody.cpp
    src/vehicle/WheelSweep.cpp)
# Traced body blueprints, embedded so the default configuration and the headless tests need no asset path.
set(TRAILFORGE_BLUEPRINT_IDS trailforge_suv trailforge_ev)
set(_bp_files "")
foreach(id IN LISTS TRAILFORGE_BLUEPRINT_IDS)
    list(APPEND _bp_files ${CMAKE_SOURCE_DIR}/assets/vehicles/${id}/blueprint.json)
endforeach()
string(REPLACE ";" "," _bp_ids "${TRAILFORGE_BLUEPRINT_IDS}")
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/generated/vehicle_blueprints.cpp
    COMMAND ${CMAKE_COMMAND} -DIDS=${_bp_ids} -DSRC_DIR=${CMAKE_SOURCE_DIR}
            -DOUTPUT=${CMAKE_BINARY_DIR}/generated/vehicle_blueprints.cpp -P ${CMAKE_SOURCE_DIR}/cmake/EmbedBlueprints.cmake
    DEPENDS ${_bp_files} ${CMAKE_SOURCE_DIR}/cmake/EmbedBlueprints.cmake
    COMMENT "Embedding vehicle blueprints"
    VERBATIM)
target_sources(trailforge_sim PRIVATE ${CMAKE_BINARY_DIR}/generated/vehicle_blueprints.cpp)
target_include_directories(trailforge_sim PUBLIC src)
target_link_libraries(trailforge_sim PUBLIC Jolt)
target_include_directories(trailforge_sim SYSTEM PRIVATE ${cgltf_SOURCE_DIR})
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
    src/app/BestLaps.cpp
    src/app/Garage.cpp
    src/app/MapData.cpp
    src/app/MapView.cpp
    src/input/ControlLayout.cpp)
target_include_directories(trailforge_flow PUBLIC src)
target_link_libraries(trailforge_flow PUBLIC trailforge_sim)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(trailforge_flow PRIVATE -Wall -Wextra)
endif()

# Vehicle body converter (Blender glTF export -> body.glb, validation against the vehicle definition): the source
# reader and LOD builder as a library for the tests, and the command-line tool.
file(GLOB TF_MESHOPT_SOURCES ${meshoptimizer_SOURCE_DIR}/src/*.cpp)
add_library(trailforge_bodyc STATIC tools/cars/BodySource.cpp ${TF_MESHOPT_SOURCES})
target_include_directories(trailforge_bodyc PUBLIC tools/cars)
target_include_directories(trailforge_bodyc SYSTEM PRIVATE ${cgltf_SOURCE_DIR} ${meshoptimizer_SOURCE_DIR}/src)
target_link_libraries(trailforge_bodyc PUBLIC trailforge_sim)
add_executable(trailforge_vehiclec tools/cars/vehiclec.cpp)
target_link_libraries(trailforge_vehiclec PRIVATE trailforge_bodyc)

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
        src/render/FieldMesh.cpp
        src/render/StyleKit.cpp
        src/render/Scenery.cpp
        src/render/LevelScene.cpp
        src/render/TimeOfDay.cpp
        src/render/Camera.cpp
        src/render/DebugDraw.cpp
        src/ui/DebugUI.cpp
        src/ui/MenuUI.cpp
        src/ui/MapUI.cpp
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

    # Vehicle model at rest as OBJ (CPU mesh only) for the silhouette comparison in tools/cars.
    add_executable(trailforge_model_export tools/cars/ModelExport.cpp src/render/Mesh.cpp src/render/StyleKit.cpp
                                           src/render/VehicleModel.cpp src/render/FieldMesh.cpp)
    target_link_libraries(trailforge_model_export PRIVATE trailforge_sim glm_headers)
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
            tread_soil_surfaces body_mud lap_timer_logic mini_track_lap asphalt_gravel_surfaces
            drift_score_logic drift_config_parse mini_track_drift)
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
            ev_simulation
            tire_thermal_model
            tire_sets
            suspension_force_model
            bump_no_rollover)
        add_test(NAME ${test_name} COMMAND trailforge_vehicle_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 300)
    endforeach()

    # Blender vehicle bodies (body.glb): converter round trip, validation rules, vehicle.json reference, collision.
    add_executable(trailforge_body_tests tests/BodyTests.cpp)
    target_link_libraries(trailforge_body_tests PRIVATE trailforge_bodyc)
    target_compile_definitions(trailforge_body_tests PRIVATE TRAILFORGE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    foreach(test_name body_hull_convexity body_convert_source body_validation_rules body_definition body_physics)
        add_test(NAME ${test_name} COMMAND trailforge_body_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
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
            level_slopes
            autopilot_loops
            terrain_textures_ktx2
            terrain_textures_assets
            building_kit_parts
            building_kit_materials
            level_buildings
            level_poi_buildings
            level_baked_layout
            lap_timer_gates
            level_baked_track_smooth
            level_baked_lap
            level_baked_lap_ev)
        add_test(NAME ${test_name} COMMAND trailforge_level_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 300)
    endforeach()
    # Blender level pipeline: import -> export without edits gives the same files and the same bake (only where
    # Blender is installed; README "Editing a level in Blender").
    find_program(TRAILFORGE_BLENDER NAMES blender Blender PATHS /Applications/Blender.app/Contents/MacOS NO_CACHE)
    if(TRAILFORGE_BLENDER)
        add_test(NAME level_blender_roundtrip
                 COMMAND ${CMAKE_COMMAND} -D BLENDER=${TRAILFORGE_BLENDER} -D SOURCE_DIR=${CMAKE_SOURCE_DIR}
                         -D LEVEL=suncoast_raceway -D WORK=${CMAKE_BINARY_DIR}/level_roundtrip
                         -D TESTS=$<TARGET_FILE:trailforge_level_tests> -P ${CMAKE_SOURCE_DIR}/cmake/LevelRoundtrip.cmake)
        set_tests_properties(level_blender_roundtrip PROPERTIES TIMEOUT 300)
    endif()

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
            audio_landing
            audio_music)
        add_test(NAME ${test_name} COMMAND trailforge_audio_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
    endforeach()

    # Game flow: settings file, menu navigation, pause.
    add_executable(trailforge_app_tests tests/AppTests.cpp)
    target_link_libraries(trailforge_app_tests PRIVATE trailforge_flow trailforge_sim)
    target_compile_definitions(trailforge_app_tests PRIVATE TRAILFORGE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    foreach(test_name settings_roundtrip menu_navigation pause_freezes_ticks garage_model vehicle_page controls_layout pad_glyph_detect
                         callout_spread map_view_math map_level_raster best_laps_file setup_migration)
        add_test(NAME ${test_name} COMMAND trailforge_app_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
    endforeach()

    # Procedural vehicle model: parts and wheel clearance (CPU mesh only; GLM comes with the app build).
    if(TRAILFORGE_BUILD_APP)
        add_executable(trailforge_model_tests tests/ModelTests.cpp src/render/Mesh.cpp src/render/StyleKit.cpp
                                              src/render/VehicleModel.cpp src/render/FieldMesh.cpp)
        target_include_directories(trailforge_model_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
        target_link_libraries(trailforge_model_tests PRIVATE trailforge_sim glm_headers)
        target_compile_definitions(trailforge_model_tests PRIVATE TRAILFORGE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
        foreach(test_name model_parts model_wheel_size model_wheel_clearance model_suspension_mounts model_body_glb)
            add_test(NAME ${test_name} COMMAND trailforge_model_tests ${test_name})
            set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
        endforeach()

        # Camera math: hood mount, close follow sizing, right-stick orbit and look-around (no window, no gamepad).
        add_executable(trailforge_camera_tests tests/CameraTests.cpp src/render/Camera.cpp src/render/TimeOfDay.cpp)
        target_include_directories(trailforge_camera_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
        target_link_libraries(trailforge_camera_tests PRIVATE trailforge_flow glm_headers)
        foreach(test_name camera_hood_mount camera_stick_dead_zone camera_stick_orbit camera_hood_look camera_close_follow sun_flare_source)
            add_test(NAME ${test_name} COMMAND trailforge_camera_tests ${test_name})
            set_tests_properties(${test_name} PROPERTIES TIMEOUT 60)
        endforeach()
    endif()
endif()
