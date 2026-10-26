# trailforge_texc: offline texture encoder for the terrain materials (JPEG / PNG -> BC7 KTX2 with mips).
# Standalone project, built only by tools/textures/fetch_textures.py; the game build does not need it because the
# encoded textures are stored in the repository (Git LFS, assets/textures/).
cmake_minimum_required(VERSION 3.24)
project(trailforge_texc LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)
# stb (public domain): stb_image for JPEG / PNG decoding.
FetchContent_Declare(stb
    URL https://github.com/nothings/stb/archive/003501f6d7a8668ce8b6d875e34dd97ad0d5d5b5.tar.gz
    URL_HASH SHA256=a6c31950b3bdf05eb92258768ba2a11bd44449490574b1aac67028452288c40d
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no-cmake)
# bc7enc_rdo (MIT or public domain, Richard Geldreich): only the bc7enc block encoder is used.
FetchContent_Declare(bc7enc
    URL https://github.com/richgel999/bc7enc_rdo/archive/b9438627eef73a1157e84201b6fa6eb2ffd6d9f0.tar.gz
    URL_HASH SHA256=bbb33d1dbb6178a3a2c044956c3cc0b8ea9fc2f2903824379dd86f87d53b64c6
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no-cmake)
# cgltf 1.15 (MIT): glTF parsing for trailforge_modelc.
FetchContent_Declare(cgltf
    URL https://github.com/jkuhlmann/cgltf/archive/360db1a95480fe102ae9c69b27c5d101167ff5ba.tar.gz
    URL_HASH SHA256=445d135cf793232ae6a585ca1404e4ff28d4f4dbca070689034fe780370ac84a
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no-cmake)
# meshoptimizer 1.3 (MIT): simplification and vertex cache optimization for trailforge_modelc.
FetchContent_Declare(meshoptimizer
    URL https://github.com/zeux/meshoptimizer/archive/9e1f07b159d3cb777f1c67ed31fc11fd117986f4.tar.gz
    URL_HASH SHA256=e753b45fbc410031bd69cb278180ae31fa0d5430d0c3e662e3a3b100467ce0b3
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR no-cmake)
FetchContent_MakeAvailable(stb bc7enc cgltf meshoptimizer)

add_executable(trailforge_texc
    main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/core/Ktx2.cpp
    ${bc7enc_SOURCE_DIR}/bc7enc.cpp
    ${bc7enc_SOURCE_DIR}/bc7decomp.cpp)
target_include_directories(trailforge_texc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_include_directories(trailforge_texc SYSTEM PRIVATE ${stb_SOURCE_DIR} ${bc7enc_SOURCE_DIR})
find_package(Threads REQUIRED)
target_link_libraries(trailforge_texc PRIVATE Threads::Threads)

# trailforge_modelc: offline glTF model converter (LODs, collision mesh, impostor atlas, BC7 KTX2 material maps),
# built only by tools/models/fetch_models.py.
file(GLOB TF_MESHOPT_SOURCES ${meshoptimizer_SOURCE_DIR}/src/*.cpp)
add_executable(trailforge_modelc
    modelc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/core/Ktx2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/core/Json.cpp
    ${TF_MESHOPT_SOURCES}
    ${bc7enc_SOURCE_DIR}/bc7enc.cpp
    ${bc7enc_SOURCE_DIR}/bc7decomp.cpp)
target_include_directories(trailforge_modelc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_include_directories(trailforge_modelc SYSTEM PRIVATE ${stb_SOURCE_DIR} ${bc7enc_SOURCE_DIR} ${cgltf_SOURCE_DIR}
    ${meshoptimizer_SOURCE_DIR}/src)
target_link_libraries(trailforge_modelc PRIVATE Threads::Threads)
