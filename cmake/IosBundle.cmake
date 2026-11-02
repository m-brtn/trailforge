# iOS app bundle (CMAKE_SYSTEM_NAME iOS, presets ios-sim-* / ios-device-*): build/<preset>/TrailForge.app with the
# executable, Info.plist (cmake/Info.ios.plist.in), the app icon compiled by actool from
# external_assets/images/Icon.png, and the runtime assets copied to TrailForge.app/assets (SDL_GetBasePath() is the
# bundle root on iOS). The bundle is signed with TRAILFORGE_IOS_SIGN_IDENTITY ("-" = ad-hoc, enough for the
# Simulator; a real device needs a development identity and a provisioning profile, docs/building.md "iOS").
#
# With the Xcode generator (preset ios-xcode, device only) Xcode does the bundling and signing instead: Automatic
# signing with TRAILFORGE_IOS_TEAM (empty = pick the team in Xcode), the icon as an asset catalog, assets/ as a folder
# reference in the Resources phase, and a shared `trailforge` scheme that launches the Release configuration.

if(CMAKE_GENERATOR STREQUAL "Xcode")
    # Bundle ids are global per Apple developer account: a free Personal Team cannot reuse one registered elsewhere.
    string(REGEX REPLACE "[^A-Za-z0-9-]" "-" _user_id "$ENV{USER}")
    set(_default_bundle_id "com.trailforge.app.${_user_id}")
else()
    set(_default_bundle_id "com.trailforge.app")
endif()
set(TRAILFORGE_IOS_BUNDLE_ID "${_default_bundle_id}" CACHE STRING "iOS bundle identifier")
set(TRAILFORGE_IOS_TEAM "" CACHE STRING "Apple development team id for Xcode Automatic signing (empty = set in Xcode)")
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
    if(CMAKE_GENERATOR STREQUAL "Xcode")
        trailforge_add_ios_xcode_bundle(${target})
        return()
    endif()
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

# Xcode generator path of trailforge_add_ios_bundle (called from it; _gen and the platform variables come from there).
function(trailforge_add_ios_xcode_bundle target)
    if(CMAKE_OSX_SYSROOT MATCHES "[Ss]imulator")
        message(FATAL_ERROR "TrailForge: the Xcode project is device only (the metallibs are built for one SDK); "
                            "use the ios-sim-release preset for the Simulator")
    endif()
    set(_bundle_id ${TRAILFORGE_IOS_BUNDLE_ID})
    set(TRAILFORGE_IOS_BUNDLE_ID "$(PRODUCT_BUNDLE_IDENTIFIER)")
    configure_file(${CMAKE_SOURCE_DIR}/cmake/Info.ios.plist.in ${_gen}/Info.plist @ONLY)

    # Xcode compiles the asset catalog (actool) in the Resources phase; the 1024 px icon is made at configure time.
    set(_icon_png ${CMAKE_SOURCE_DIR}/external_assets/images/Icon.png)
    set(_xcassets ${_gen}/Assets.xcassets)
    file(WRITE ${_xcassets}/Contents.json "{\"info\":{\"author\":\"xcode\",\"version\":1}}\n")
    file(WRITE ${_xcassets}/AppIcon.appiconset/Contents.json
         "{\"images\":[{\"filename\":\"icon_1024.png\",\"idiom\":\"universal\",\"platform\":\"ios\","
         "\"size\":\"1024x1024\"}],\"info\":{\"author\":\"xcode\",\"version\":1}}\n")
    set(_icon_out ${_xcassets}/AppIcon.appiconset/icon_1024.png)
    if(NOT EXISTS ${_icon_out} OR ${_icon_png} IS_NEWER_THAN ${_icon_out})
        execute_process(COMMAND sips -s format png -z 1024 1024 ${_icon_png} --out ${_icon_out}
                        OUTPUT_QUIET RESULT_VARIABLE _sips_res)
        if(NOT _sips_res EQUAL 0)
            message(FATAL_ERROR "TrailForge: sips could not convert ${_icon_png}")
        endif()
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_icon_png})

    # A directory source becomes a folder reference: Xcode copies assets/ recursively to the bundle root
    # (SDL_GetBasePath() on iOS). The logo lives in the build tree and gets its own copy phase into assets/images.
    set(_assets ${CMAKE_SOURCE_DIR}/assets)
    set(_logo ${CMAKE_BINARY_DIR}/assets/images/logo.png)
    target_sources(${target} PRIVATE ${_assets} ${_xcassets} ${_logo})
    set_source_files_properties(${_assets} ${_xcassets} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    set_source_files_properties(${_logo} PROPERTIES MACOSX_PACKAGE_LOCATION assets/images)
    source_group("Resources" FILES ${_assets} ${_xcassets} ${_logo})

    set_target_properties(${target} PROPERTIES
        MACOSX_BUNDLE ON
        OUTPUT_NAME TrailForge
        MACOSX_BUNDLE_INFO_PLIST ${_gen}/Info.plist
        RESOURCE "${_assets};${_xcassets}"
        XCODE_GENERATE_SCHEME ON
        XCODE_SCHEME_LAUNCH_CONFIGURATION Release
        XCODE_ATTRIBUTE_PRODUCT_NAME TrailForge
        XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER "${_bundle_id}"
        XCODE_ATTRIBUTE_CODE_SIGN_STYLE Automatic
        XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "Apple Development"
        XCODE_ATTRIBUTE_DEVELOPMENT_TEAM "${TRAILFORGE_IOS_TEAM}"
        XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET ${CMAKE_OSX_DEPLOYMENT_TARGET}
        XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2"
        XCODE_ATTRIBUTE_SUPPORTED_PLATFORMS iphoneos
        XCODE_ATTRIBUTE_SUPPORTS_MACCATALYST NO
        XCODE_ATTRIBUTE_SUPPORTS_MAC_DESIGNED_FOR_IPHONE_IPAD NO
        XCODE_ATTRIBUTE_ASSETCATALOG_COMPILER_APPICON_NAME AppIcon
        XCODE_ATTRIBUTE_ENABLE_BITCODE NO
        XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT "dwarf-with-dsym")
endfunction()
