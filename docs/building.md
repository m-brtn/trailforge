# Building and running

Platforms, prerequisites, build presets, the app bundle, packaging and command-line options. All documentation: [index](README.md).

## Platforms

| Platform | Status |
|---|---|
| macOS 27 / Apple Silicon (arm64), Apple clang 21, Metal | **Run and verified** (everything in [Verification results](testing.md#verification-results)) |
| macOS, Vulkan through MoltenVK (`SDL_GPU_DRIVER=vulkan`) | Run on the same Mac: `full_check` 210 / 0, captures look like Metal. Not a supported configuration, only a test of the SPIR-V path |
| Windows x64 (MSVC, D3D12 / Vulkan) | Builds and the headless tests pass in GitHub Actions; the package is built there. **Not run on real hardware** |
| Linux x64 (gcc, Vulkan) | Builds and the headless tests pass in GitHub Actions; the packaged app starts and renders on Mesa lavapipe (software Vulkan) under Xvfb. **Not run on real hardware** |
| iOS / iPadOS 26+ (arm64, Metal), game controller only | Run in the **iOS Simulator only** (Xcode 27.2, iOS 27.0 runtime, iPhone 18 Pro): start, menus, both levels, driving with SDL's virtual gamepad (`ios_check` 14 / 0). The device build compiles and links but is **not run on a real iPhone or iPad**; a real controller is untested. See [iOS](#ios) |

The shaders have one HLSL source compiled at build time to SPIR-V, MSL / metallib and (on Windows) DXIL, see
[Shader pipeline](rendering.md#shader-pipeline). The renderer takes the first format the SDL GPU device accepts, in the order
metallib, MSL, DXIL, SPIR-V. GPU frame timing exists only on Metal; elsewhere the UI shows `GPU n/a`.

Trying Windows or Linux (please report what happens):

1. Download the package from the GitHub release (`TrailForge-<version>-windows-x64.zip` or `...-linux-x64.tar.gz`),
   extract it and keep `assets/` next to the executable. Windows: SmartScreen warns about an unsigned executable
   ("More info" → "Run anyway"). Linux: a Vulkan driver is needed (Mesa RADV / ANV / NVK or the NVIDIA driver).
2. `trailforge --version` and `trailforge --paths` (asset directory and user data directory) in a terminal.
3. Start `trailforge`; the console prints `GPU driver '<d3d12|vulkan>', shaders: <DXIL|SPIR-V>`. Check that the
   picture looks like the macOS screenshots (sky, shadows, grass, puddle reflections, no upside-down image).
4. Drive (W/S, A/D, Space, R), press F1, change Graphics → quality presets, and listen for the engine and tire
   sounds.
5. From a source checkout, `trailforge --script tools/scenarios/full_check.txt --no-audio` must end with
   `0 failed`; the console log and the exit code are the most useful report. Windows: try
   `SDL_GPU_DRIVER=vulkan` as well.

## Prerequisites

- macOS: Xcode Command Line Tools with the Metal toolchain (`xcrun -sdk macosx -f metal` must find the compiler).
  Without it the build still succeeds: the shaders are compiled from the embedded MSL source at startup.
  CMake ≥ 3.24 and Ninja: `brew install cmake ninja`.
- Windows: Visual Studio 2022 or newer with the C++ x64 tools (MSVC), CMake ≥ 3.24 and Ninja (both ship with
  Visual Studio); configure from an "x64 Native Tools" prompt.
- Linux: gcc or clang with C++20, CMake ≥ 3.24, Ninja and the SDL3 build dependencies (on Ubuntu:
  `libx11-dev libxext-dev libxrandr-dev libxcursor-dev libxfixes-dev libxi-dev libxss-dev libxtst-dev
  libxkbcommon-dev libwayland-dev wayland-protocols libegl1-mesa-dev libdrm-dev libgbm-dev libasound2-dev
  libpulse-dev libudev-dev libdbus-1-dev`, the list used in `.github/workflows/ci.yml`).
- Git LFS (`brew install git-lfs`, then `git lfs install` once) before cloning, or `git lfs pull` after: the terrain
  textures (~168 MB) and the level models (~84 MB) are LFS objects. Without them the level falls back to the
  procedural ground and pines and the `terrain_textures_assets` test fails.
- Internet access for the first configure. Dependencies are downloaded via FetchContent as archives
  of exact commits with SHA256 verification:

| Dependency | Version | Commit |
|---|---|---|
| SDL | release-3.4.18 | `829a65d7` |
| Jolt Physics | v5.6.0 | `e77f1755` |
| Dear ImGui | v1.92.9b | `f1cc2ae1` |
| GLM | 1.0.3 | `8d1fd52e` |
| miniaudio | 0.11.25 | `9634bedb` |
| SPIRV-Headers, SPIRV-Tools, glslang, SPIRV-Cross (shader compiler, host tool only) | Vulkan SDK tag vulkan-sdk-1.4.357.0 | `29981f65`, `9a49b088`, `168d452a`, `6c09849f` |
| DXC (prebuilt, Windows hosts only) | v1.9.2609 | release zip `dxc_2026_09_29.zip` |
| bc7enc_rdo, stb_image (texture encoder `tools/texc`, only for `fetch_textures.py`) | master snapshots | `b9438627`, `003501f6` |

## Build and run

```sh
cmake --preset release
cmake --build --preset release
./build/release/trailforge

# debug build (SDL GPU debug layer enabled)
cmake --preset debug && cmake --build --preset debug && ./build/debug/trailforge

# tests
ctest --preset release        # or debug

# physics + tests only, without SDL/rendering
cmake --preset headless && cmake --build --preset headless && ctest --preset headless

# Windows (x64 Native Tools prompt) and Linux; presets only show up on their host system
cmake --preset windows-release && cmake --build --preset windows-release && ctest --preset windows-release
cmake --preset linux-release && cmake --build --preset linux-release && ctest --preset linux-release
# (windows-debug / linux-debug likewise)
```

`trailforge --paths` prints the asset lookup directories and the user data directory (`SDL_GetPrefPath`, e.g.
`~/Library/Application Support/TrailForge/`, `%APPDATA%\TrailForge\`, `~/.local/share/TrailForge/`; reserved for
future settings, nothing is written there yet except the empty directory that SDL creates).

App icon (`external_assets/images/Icon.png`, macOS): the build converts it with `sips` to a 256×256 BMP that is embedded into the binary;
at startup `SDL_SetWindowIcon` sets it (on Cocoa this is `NSApp.applicationIconImage`, i.e. the Dock icon of the
running `trailforge` process), masked to the macOS rounded-square grid. `Logo.png` is reserved for a future menu
and is not loaded at runtime.

The build also produces `build/<preset>/TrailForge.app`, a self-contained bundle: a copy of the executable,
`Info.plist` (version from `project(TrailForge VERSION …)` in `CMakeLists.txt`, `LSMinimumSystemVersion` = the
deployment target `CMAKE_OSX_DEPLOYMENT_TARGET`, 14.0 by default in `CMakeLists.txt`, the same as the binary's
minos and the metallib target; `package_macos.sh` fails if it is above the binary minos or the host macOS), `TrailForge.icns`
(`sips` + `iconutil`, 16–1024 px) and the runtime assets in `Contents/Resources/assets/audio`. Shaders and the
window icon are embedded in the binary, so the sounds are the only files read at runtime. They are looked up via
`SDL_GetBasePath()` (SDL 3.4.18 on macOS: `Contents/Resources/` inside a bundle, the executable's directory
otherwise): `<base>/assets/audio`, then, in development builds only, `<base>/../../assets/audio` and the source
tree. The version is shown in the window title, the HUD and the Telemetry window, and by `trailforge --version`.

### Packaged build and release

```sh
tools/release/package_macos.sh            # -> dist/TrailForge.app and dist/TrailForge-<version>-macos-arm64.zip
tools/release/package_macos.sh --publish  # the same, then gh release create v<version> with docs/releases/v<version>.md
```

The script does a clean Release configure in `build/package` with `-DTRAILFORGE_PACKAGED=ON` (no source-tree
asset fallback), builds, runs CTest, copies the bundle to `dist/`, checks that `otool -L` lists only system
libraries and frameworks, signs ad hoc (`codesign --force --deep -s -`) and verifies the signature, zips with
`ditto -c -k --keepParent`, then runs the bundle from `/` and from a fresh unzip and checks that all 55 sounds load
from inside the bundle. `dist/` is git-ignored. The app is not notarized: after downloading, open it with
right-click → Open, or `xattr -dr com.apple.quarantine TrailForge.app`. Releases are on GitHub
(`gh release list`); release notes live in `docs/releases/`.

Windows and Linux packages are built by CI (`.github/workflows/ci.yml`) and can be built locally the same way:

```sh
tools/release/package_linux.sh [--smoke-window]               # -> dist/TrailForge-<version>-linux-<arch>.tar.gz
pwsh tools/release/package_windows.ps1 [-SmokeWindow]         # -> dist/TrailForge-<version>-windows-x64.zip
```

Both do a clean packaged Release build in `build/package`, run CTest, put the executable, `assets/` (audio, music, fonts, images) and a
`README.txt` into `dist/TrailForge-<version>-<os>-<arch>/`, archive it, extract the archive into a temporary
directory and check from another working directory that `--version` matches, that `--paths` reports exactly one
sound asset directory, the one inside the package, and that its music directory has `intro.mp3`. The optional window smoke test needs a GPU (CI runs the Linux package
on lavapipe in a separate step instead).

Command-line options:

- `--no-intro` — skip the intro and go straight to the start menu (scripts and `--demo` start in gameplay);
- `--no-ui` — start with diagnostics hidden;
- `--demo` — scripted drive without input;
- `--quit-after N` — quit automatically after N seconds (smoke test);
- `--capture file.bmp|file.png [--capture-at N]` — save one rendered frame;
- `--script file.txt` — scenario mode (see below). Exit code 1 if any check failed;
- `--vehicle ID` — start with catalog vehicle ID (`assets/vehicles/ID`, e.g. `trailforge_ev`), remembered like a
  choice on the vehicle page (not in scenario mode);
- `--level ID` — start on level ID (`exploration_peaks`, `suncoast_raceway`, `proving_ground`); scenarios and `--demo` start on the
  proving ground unless it is given;
- `--no-audio` — no audio device, no sounds loaded;
- `--audio-capture out.wav` — render the mix offline into `out.wav` (no device, see [Audio](audio.md#audio));
- `--paths` — print the asset lookup directories (`assets:` sounds, `music:`, `fonts:`, `images:`) and the user data
  directory, then exit;
- `--version` — print the version and exit.

## iOS

First port (v0.29.0 tree): **iOS Simulator only, game controller only, untested on a real iPhone or iPad.** Touch
works in the menus only (tap a row or a value arrow, drag a list to scroll it, a flick coasts); there are no touch
driving controls, so driving needs a controller. Minimum iOS 26.0 (iPhone and iPad,
landscape only, arm64). For a real iPhone / iPad use the Xcode project ("iOS device: Xcode project" below).

Prerequisites: Xcode with the iOS SDK and an iOS Simulator runtime (checked with Xcode 27.2, iOS SDK 27.2 and the
iOS 27.0 runtime; there was no iOS 26 runtime on the test Mac, so iOS 26 itself was not run), the Metal toolchain for
the iOS SDKs (`xcrun -sdk iphonesimulator -f metal`), CMake and Ninja.

```sh
# 1) host tools: trailforge_shaderc for macOS (cross builds cannot run a compiler built for iOS)
cmake --preset host-tools && cmake --build --preset host-tools
# 2) the app for the Simulator (arm64) -> build/ios-sim-release/TrailForge.app (ad-hoc signed)
cmake --preset ios-sim-release && cmake --build --preset ios-sim-release
# 3) boot a device, install, launch with the console; arguments after -- go to the app
tools/ios/run_simulator.sh --no-build
tools/ios/run_simulator.sh --no-build --device "iPad Air 11-inch (M4)" -- --demo --quit-after 30
tools/ios/run_simulator.sh --no-build -- --script $PWD/tools/scenarios/ios_check.txt --no-audio
# device build (compiles and links; installing it needs signing, below)
cmake --preset ios-device-release && cmake --build --preset ios-device-release
```

`tools/ios/run_simulator.sh` boots the device headless (`--gui` opens the Simulator window), installs the bundle and
runs `simctl launch --console-pty`. It calls CoreSimulator's own `simctl`: on the test Mac the `xcrun simctl` wrapper
hangs in `xcodebuild -runFirstLaunch` (it waits for an administrator password because the installed CoreSimulator
1174.9.2 is older than the 1174.21 Xcode 27.2 expects). Paths given to the app (`--script`, `--capture`) are host
paths: Simulator apps read and write the Mac's file system, e.g. `--capture /tmp/trailforge_ios.png`. Settings,
setups and best laps go to the app container (`simctl get_app_container <udid> com.trailforge.app data`, then
`Library/Application Support/TrailForge/`).

How the build works:

- Presets `host-tools` (macOS, builds only `trailforge_shaderc`), `ios-sim-release` and `ios-device-release`
  (`CMAKE_SYSTEM_NAME=iOS`, `CMAKE_OSX_DEPLOYMENT_TARGET=26.0`, arm64, Ninja). Only the app is built for iOS: the
  tests, the probe, `trailforge_vehiclec` and the model exporter stay on macOS, and `TRAILFORGE_PACKAGED` is forced on
  (assets only from the bundle). `texc` / `modelc` are not needed: the assets in the repository are already converted.
- Shaders: the host `trailforge_shaderc` (`TRAILFORGE_HOST_SHADERC`) translates SPIR-V to MSL with SPIRV-Cross's iOS
  options (`--msl-platform ios`), and `xcrun -sdk iphonesimulator|iphoneos metal` compiles the metallibs for the
  deployment target.
- `cmake/IosBundle.cmake`: `TrailForge.app` with `Info.plist` from `cmake/Info.ios.plist.in` (bundle id
  `com.trailforge.app`, UIScene manifest, landscape only, full screen, status bar hidden, `GCSupportedGameControllers`
  extended gamepad, `GCSupportsControllerUserInteraction`), the app icon compiled by `actool` from
  `external_assets/images/Icon.png`, and `assets/` copied to the bundle root (`SDL_GetBasePath()` on iOS). It is
  signed with `TRAILFORGE_IOS_SIGN_IDENTITY` (default `-`, ad hoc).
- Two patches of the pinned dependencies run as FetchContent `PATCH_COMMAND`s:
  `cmake/patches/SdlIosSimulator.cmake` (inside `#if TARGET_OS_SIMULATOR` only: SDL GPU accepts the Simulator's Metal
  device, which reports only GPU family Apple2, and skips `setDepthClipMode`, which that device rejects; Metal
  validation asserts on it and without validation the GPU stalls) and `cmake/patches/ImguiIos.cmake` (ImGui's SDL GPU
  backend tests `TARGET_OS_MAC` first, which is also true on iOS, and so picks its macOS metallibs; on iOS it now uses
  its MSL source). macOS compiles the original code paths.
- Code: Low is the default quality preset on iOS (`kDefaultQuality`, until the port is measured on a device). The app
  lifecycle events pause the game when it loses focus (pause menu), and in the background nothing is rendered, the
  simulation does not tick and the audio device is stopped; it restarts on return.

Measured in the Simulator (iPhone 18 Pro, iOS 27.0, 2622 × 1206 output, Low preset 1757 × 808 render): about 2 s
from launch to the loaded menu world (log timestamps), 112–185 FPS in the menu, on the proving ground and on
Exploration Peaks (`perf_begin` averages 5.4–8.9 ms; the Simulator does not hold VSync), process memory 520–600 MB.
The Simulator's GPU is the Mac's GPU behind a translation layer, so these numbers say nothing about an iPhone.

Known limitations:

- The Simulator's Metal device has no BC texture compression: the level ground falls back to the procedural ground and
  the level models (trees, ground cover, rocks, cliffs, logs) are skipped with a log line, as on any GPU without BC7:
  Exploration Peaks shows bare hills. Whether a given iPhone / iPad reports `supportsBCTextureCompression` was not checked; an
  ASTC copy of the textures would be the portable fix.
- No depth clamp in the Simulator (see the SDL patch): shadow casters behind the light's near plane are clipped.
- Controller input was verified only with SDL's virtual gamepad (`gamepad attach` in `tools/scenarios/ios_check.txt`),
  which goes through the game's gamepad code but not through Apple's GameController framework. The Simulator does not
  pass a physical controller through here; MFi / Xbox / DualSense pads are untested.
- Touch in the menus only (SDL's touch -> mouse events: a tap acts when the finger lifts, a drag past 10 points
  scrolls the list and activates nothing, garage sliders are set by tapping the bar); no touch driving controls, no
  on-screen keyboard; the HUD still shows the keyboard help line. UI layout ignores the safe
  area (the Dynamic Island can cover the left edge of the HUD in landscape).
- Background and return: the lifecycle events arrive and the app stops rendering and audio (log lines `App: entered
  the background`, `Audio: device stopped`). Returning crashed once because the Simulator's own GPU host process
  (`SimMetalHost`) had crashed; it also crashed when the Simulator window opened and when other apps launched with
  TrailForge not running, so the resume path is unverified.
- Scenario checks other than `ios_check.txt` were not run on iOS.

### iOS device: Xcode project

Preset `ios-xcode` generates `build/ios-xcode/TrailForge.xcodeproj` (Xcode generator, iOS device only, arm64,
iOS 26.0+) with Xcode's **Automatic** signing. The project has the app target `trailforge` (product `TrailForge.app`),
its libraries (`trailforge_sim`, `trailforge_audio`, `trailforge_flow`, Jolt, SDL3, ImGui, miniaudio) and one shared
scheme, `trailforge`, whose Run action uses the Release configuration (Debug Jolt is too slow to drive). Not run on a
real iPhone or iPad yet.

```sh
cmake --preset host-tools && cmake --build --preset host-tools   # once: macOS trailforge_shaderc
cmake --preset ios-xcode -DTRAILFORGE_IOS_TEAM=<team id>         # team optional, see below
open build/ios-xcode/TrailForge.xcodeproj
```

The first configure downloads and configures the dependencies (about 5.5 min here); later ones take seconds.
In Xcode:

1. Xcode → Settings → Accounts: an Apple ID must be signed in (a free Personal Team is enough; its builds expire
   after 7 days).
2. Connect the iPhone / iPad by cable, unlock it and tap "Trust". On the device enable Settings → Privacy & Security
   → Developer Mode (the device restarts) if Xcode asks for it.
3. Scheme `trailforge` (top bar); destination: the device (not "Any iOS Device"; Simulators are not offered).
4. Without `TRAILFORGE_IOS_TEAM`: select the project in the navigator → target `trailforge` → Signing & Capabilities
   → Team. Leave "Automatically manage signing" on. Xcode registers the device and creates the profile.
5. Run (⌘R). The first launch of an app from a free team is blocked on the device: Settings → General → VPN & Device
   Management → the developer app → Trust, then Run again.

Details:

- `TRAILFORGE_IOS_TEAM` (cache variable, default empty) is written to `DEVELOPMENT_TEAM`. A team picked in the Xcode
  UI is stored in the generated project only and is lost when CMake regenerates it (any configure, or Xcode's
  `ZERO_CHECK` after a CMake file changed), so set the cache variable once. Team ids: Xcode → Settings → Accounts, or
  `defaults read com.apple.dt.Xcode IDEProvisioningTeamByIdentifier`.
- `TRAILFORGE_IOS_BUNDLE_ID` defaults to `com.trailforge.app.<user>` (login name) for the Xcode project: bundle ids
  are unique per Apple developer account, and a free team cannot register one that another team owns. The Ninja
  presets keep `com.trailforge.app`. `Info.plist` comes from `cmake/Info.ios.plist.in` with
  `$(PRODUCT_BUNDLE_IDENTIFIER)`.
- Resources: `assets/` is a folder reference in the Resources phase (copied recursively to `TrailForge.app/assets`),
  the logo from the build tree has its own copy phase into `assets/images`, and the app icon is an asset catalog
  generated at configure time (`build/ios-xcode/generated/ios/Assets.xcassets`) that Xcode compiles.
- Shaders: the metallibs are built with `xcrun -sdk iphoneos metal`, so the project is device only
  (`SUPPORTED_PLATFORMS = iphoneos`); the Simulator stays on `ios-sim-release` and `tools/ios/run_simulator.sh`.
- Command line: `cmake --build --preset ios-xcode` (Release, `-allowProvisioningUpdates`) builds a signed app when a
  team is set; without signing, to check compile and link:
  `xcodebuild -project build/ios-xcode/TrailForge.xcodeproj -scheme trailforge -configuration Release
  -destination generic/platform=iOS CODE_SIGNING_ALLOWED=NO build`. The app is
  `build/ios-xcode/Release-iphoneos/TrailForge.app`. Install and launch without Xcode:
  `devicectl device install app --device <udid> build/ios-xcode/Release-iphoneos/TrailForge.app`, then
  `devicectl device process launch --device <udid> com.trailforge.app.<user>`. On the test Mac `xcrun devicectl`
  hangs in `xcodebuild -runFirstLaunch` like `xcrun simctl`; CoreDevice's own binary works:
  `/Library/Developer/PrivateFrameworks/CoreDevice.framework/Versions/A/Resources/bin/devicectl`.
- The Ninja preset `ios-device-release` still builds an ad-hoc bundle; signing it by hand needs
  `TRAILFORGE_IOS_SIGN_IDENTITY`, `TRAILFORGE_IOS_PROVISIONING_PROFILE` and `TRAILFORGE_IOS_ENTITLEMENTS`
  (`cmake/IosBundle.cmake`). The Xcode project is the simpler path.

On the device, still to check: BC7 support, a real controller, and the Low / Medium presets.
