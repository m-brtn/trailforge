# iOS app bundle (CMAKE_SYSTEM_NAME iOS, presets ios-sim-* / ios-device-*): build/<preset>/TrailForge.app with the
# executable, Info.plist (cmake/Info.ios.plist.in), the app icon compiled by actool from
# external_assets/images/Icon.png, and the runtime assets copied to TrailForge.app/assets (SDL_GetBasePath() is the
# bundle root on iOS). The bundle is signed with TRAILFORGE_IOS_SIGN_IDENTITY ("-" = ad-hoc, enough for the
# Simulator; a real device needs a development identity and a provisioning profile, docs/building.md "iOS").

set(TRAILFORGE_IOS_BUNDLE_ID "com.trailforge.app" CACHE STRING "iOS bundle identifier")
set(TRAILFORGE_IOS_SIGN_IDENTITY "-" CACHE STRING "codesign identity for the iOS bundle (- = ad-hoc)")
set(TRAILFORGE_IOS_PROVISIONING_PROFILE "" CACHE FILEPATH
    "Provisioning profile copied into the bundle as embedded.mobileprovision (device builds)")
set(TRAILFORGE_IOS_ENTITLEMENTS "" CACHE FILEPATH "Entitlements plist passed to codesign (device builds)")

function(trailforge_add_ios_bundle target)
    if(CMAKE_OSX_SYSROOT MATCHES "[Ss]imulator")
        set(TRAILFORGE_IOS_PLATFORM iPhoneSimulator)
        set(TRAILFORGE_IOS_SDK iphonesimulator)
    else()
        set(TRAILFORGE_IOS_PLATFORM iPhoneOS)
        set(TRAILFORGE_IOS_SDK iphoneos)
    endif()
    set(_gen ${CMAKE_BINARY_DIR}/generated/ios)
    file(MAKE_DIRECTORY ${_gen})
    configure_file(${CMAKE_SOURCE_DIR}/cmake/Info.ios.plist.in ${_gen}/Info.plist @ONLY)

    set_target_properties(${target} PROPERTIES
        MACOSX_BUNDLE ON
        OUTPUT_NAME TrailForge
        MACOSX_BUNDLE_INFO_PLIST ${_gen}/Info.plist
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    set(_app ${CMAKE_BINARY_DIR}/TrailForge.app)

    # App icon: one 1024 px image in an asset catalog; actool writes Assets.car and the AppIcon*.png files.
    set(_icon_png ${CMAKE_SOURCE_DIR}/external_assets/images/Icon.png)
    set(_xcassets ${_gen}/Assets.xcassets)
    set(_icon_out ${_gen}/actool)
    file(WRITE ${_xcassets}/Contents.json "{\"info\":{\"author\":\"xcode\",\"version\":1}}\n")
    file(WRITE ${_xcassets}/AppIcon.appiconset/Contents.json
         "{\"images\":[{\"filename\":\"icon_1024.png\",\"idiom\":\"universal\",\"platform\":\"ios\","
         "\"size\":\"1024x1024\"}],\"info\":{\"author\":\"xcode\",\"version\":1}}\n")
    add_custom_command(
        OUTPUT ${_icon_out}/Assets.car
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${_icon_out}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${_icon_out}
        COMMAND sips -s format png -z 1024 1024 ${_icon_png} --out ${_xcassets}/AppIcon.appiconset/icon_1024.png
        COMMAND xcrun actool ${_xcassets} --compile ${_icon_out} --platform ${TRAILFORGE_IOS_SDK}
                --minimum-deployment-target ${CMAKE_OSX_DEPLOYMENT_TARGET} --app-icon AppIcon
                --target-device iphone --target-device ipad --output-partial-info-plist ${_gen}/actool_partial.plist
        DEPENDS ${_icon_png}
        COMMENT "Compiling the iOS app icon (actool)"
        VERBATIM)

    set(_assets ${CMAKE_SOURCE_DIR}/assets)
    file(GLOB_RECURSE _asset_files CONFIGURE_DEPENDS ${_assets}/*)
    set(_stamp ${_gen}/bundle_assets.stamp)
    add_custom_command(
        OUTPUT ${_stamp}
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${_app}/assets
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${_assets} ${_app}/assets
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/assets/images/logo.png ${_app}/assets/images/logo.png
        COMMAND ${CMAKE_COMMAND} -E copy ${_icon_out}/Assets.car ${_icon_out}/AppIcon60x60@2x.png
                ${_icon_out}/AppIcon76x76@2x~ipad.png ${_app}/
        COMMAND ${CMAKE_COMMAND} -E touch ${_stamp}
        DEPENDS ${_asset_files} ${CMAKE_BINARY_DIR}/assets/images/logo.png ${_icon_out}/Assets.car
        COMMENT "Copying assets/ and the icon into TrailForge.app"
        VERBATIM)
    add_custom_target(${target}_ios_assets ALL DEPENDS ${_stamp})
    add_dependencies(${target}_ios_assets ${target})

    set(_sign_args --force --sign ${TRAILFORGE_IOS_SIGN_IDENTITY} --timestamp=none)
    if(TRAILFORGE_IOS_ENTITLEMENTS)
        list(APPEND _sign_args --entitlements ${TRAILFORGE_IOS_ENTITLEMENTS})
    endif()
    set(_profile_cmd "")
    if(TRAILFORGE_IOS_PROVISIONING_PROFILE)
        set(_profile_cmd COMMAND ${CMAKE_COMMAND} -E copy ${TRAILFORGE_IOS_PROVISIONING_PROFILE}
                         ${_app}/embedded.mobileprovision)
    endif()
    # Runs on every build: the signature covers the executable and every resource.
    add_custom_target(${target}_ios_sign ALL
        ${_profile_cmd}
        COMMAND codesign ${_sign_args} ${_app}
        COMMENT "Signing TrailForge.app (${TRAILFORGE_IOS_SIGN_IDENTITY})"
        VERBATIM)
    add_dependencies(${target}_ios_sign ${target}_ios_assets)
endfunction()
