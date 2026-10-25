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
FetchContent_MakeAvailable(stb bc7enc)

add_executable(trailforge_texc
    main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/core/Ktx2.cpp
    ${bc7enc_SOURCE_DIR}/bc7enc.cpp
    ${bc7enc_SOURCE_DIR}/bc7decomp.cpp)
target_include_directories(trailforge_texc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
target_include_directories(trailforge_texc SYSTEM PRIVATE ${stb_SOURCE_DIR} ${bc7enc_SOURCE_DIR})
find_package(Threads REQUIRED)
target_link_libraries(trailforge_texc PRIVATE Threads::Threads)
