# Application icon (source of truth: external_assets/images/Icon.png).
#
# macOS only (needs `sips` and `iconutil`, both part of the base system):
#   1) Icon.png -> 256x256 BMP -> embedded into the binary (cmake/EmbedFile.cmake);
#      at startup SDL_SetWindowIcon sets it, which on Cocoa is the Dock icon of the running process;
#   2) Icon.png -> TrailForge.icns (iconset 16..1024 px) -> build/<preset>/TrailForge.app, a self-contained bundle:
#      a copy of the `trailforge` executable, Info.plist (version from project()), the icns and the runtime assets
#      (assets/ -> Contents/Resources/assets: audio, music, fonts, images, plus the logo from the build tree; found at runtime through SDL_GetBasePath()).
# Elsewhere no icon is embedded and the window keeps the default icon.

set(TRAILFORGE_ICON_SOURCES "")
set(TRAILFORGE_ICON_DEFINES "")

set(_icon_png ${CMAKE_SOURCE_DIR}/external_assets/images/Icon.png)
set(_icon_gen ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${_icon_gen})

find_program(TRAILFORGE_SIPS sips)
find_program(TRAILFORGE_ICONUTIL iconutil)

if(APPLE AND TRAILFORGE_SIPS AND EXISTS ${_icon_png})
    add_custom_command(
        OUTPUT ${_icon_gen}/icon.bmp
        COMMAND ${TRAILFORGE_SIPS} -s format bmp -z 256 256 ${_icon_png} --out ${_icon_gen}/icon.bmp
        DEPENDS ${_icon_png}
        COMMENT "Converting Icon.png -> icon.bmp (256x256)")
    add_custom_command(
        OUTPUT ${_icon_gen}/icon_bmp.cpp
        COMMAND ${CMAKE_COMMAND} -DINPUT=${_icon_gen}/icon.bmp -DOUTPUT=${_icon_gen}/icon_bmp.cpp
                -DSYMBOL=g_app_icon_bmp -P ${CMAKE_SOURCE_DIR}/cmake/EmbedFile.cmake
        DEPENDS ${_icon_gen}/icon.bmp ${CMAKE_SOURCE_DIR}/cmake/EmbedFile.cmake
        COMMENT "Embedding icon.bmp")
    list(APPEND TRAILFORGE_ICON_SOURCES ${_icon_gen}/icon_bmp.cpp)
    list(APPEND TRAILFORGE_ICON_DEFINES TRAILFORGE_HAS_ICON=1)
endif()

# Bundle with the .icns. Call after the `trailforge` target exists.
function(trailforge_add_macos_bundle target)
    if(NOT APPLE OR NOT TRAILFORGE_SIPS OR NOT TRAILFORGE_ICONUTIL OR NOT EXISTS ${_icon_png})
        return()
    endif()
    set(_iconset ${_icon_gen}/TrailForge.iconset)
    set(_icns ${_icon_gen}/TrailForge.icns)
    set(_sips_cmds "")
    foreach(size 16 32 128 256 512)
        math(EXPR size2 "${size} * 2")
        list(APPEND _sips_cmds
            COMMAND ${TRAILFORGE_SIPS} -s format png -z ${size} ${size} ${_icon_png}
                    --out ${_iconset}/icon_${size}x${size}.png
            COMMAND ${TRAILFORGE_SIPS} -s format png -z ${size2} ${size2} ${_icon_png}
                    --out ${_iconset}/icon_${size}x${size}@2x.png)
    endforeach()
    add_custom_command(
        OUTPUT ${_icns}
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${_iconset}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${_iconset}
        ${_sips_cmds}
        COMMAND ${TRAILFORGE_ICONUTIL} -c icns ${_iconset} -o ${_icns}
        DEPENDS ${_icon_png}
        COMMENT "Building TrailForge.icns")

    set(_app ${CMAKE_BINARY_DIR}/TrailForge.app)
    # LSMinimumSystemVersion follows the deployment target the binary is actually built for.
    if(CMAKE_OSX_DEPLOYMENT_TARGET)
        set(TRAILFORGE_MIN_MACOS ${CMAKE_OSX_DEPLOYMENT_TARGET})
    else()
        execute_process(COMMAND xcrun --sdk macosx --show-sdk-version OUTPUT_VARIABLE TRAILFORGE_MIN_MACOS
                        OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()
    configure_file(${CMAKE_SOURCE_DIR}/cmake/Info.plist.in ${_icon_gen}/Info.plist @ONLY)
    add_custom_command(
        OUTPUT ${_app}/Contents/MacOS/trailforge
        COMMAND ${CMAKE_COMMAND} -E make_directory ${_app}/Contents/MacOS ${_app}/Contents/Resources
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_icon_gen}/Info.plist ${_app}/Contents/Info.plist
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_icns} ${_app}/Contents/Resources/TrailForge.icns
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${target}> ${_app}/Contents/MacOS/trailforge
        DEPENDS ${target} ${_icns} ${_icon_gen}/Info.plist
        COMMENT "Assembling TrailForge.app")

    # Runtime assets: re-copied only when a file under assets/ (or the logo copied into the build tree) changes.
    set(_assets ${CMAKE_SOURCE_DIR}/assets)
    file(GLOB_RECURSE _asset_files CONFIGURE_DEPENDS ${_assets}/*)
    set(_assets_stamp ${_icon_gen}/bundle_assets.stamp)
    add_custom_command(
        OUTPUT ${_assets_stamp}
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${_app}/Contents/Resources/assets
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${_assets} ${_app}/Contents/Resources/assets
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/assets/images/logo.png
                ${_app}/Contents/Resources/assets/images/logo.png
        COMMAND ${CMAKE_COMMAND} -E touch ${_assets_stamp}
        DEPENDS ${_asset_files} ${CMAKE_BINARY_DIR}/assets/images/logo.png
        COMMENT "Copying assets/ into TrailForge.app")
    add_custom_target(trailforge_bundle ALL DEPENDS ${_app}/Contents/MacOS/trailforge ${_assets_stamp})
endfunction()
