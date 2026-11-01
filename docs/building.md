# Building and running

Platforms, prerequisites, build presets, the app bundle, packaging and command-line options. All documentation: [index](README.md).

## Platforms

| Platform | Status |
|---|---|
| macOS 27 / Apple Silicon (arm64), Apple clang 21, Metal | **Run and verified** (everything in [Verification results](testing.md#verification-results)) |
| macOS, Vulkan through MoltenVK (`SDL_GPU_DRIVER=vulkan`) | Run on the same Mac: `full_check` 210 / 0, captures look like Metal. Not a supported configuration, only a test of the SPIR-V path |
| Windows x64 (MSVC, D3D12 / Vulkan) | Builds and the headless tests pass in GitHub Actions; the package is built there. **Not run on real hardware** |
| Linux x64 (gcc, Vulkan) | Builds and the headless tests pass in GitHub Actions; the packaged app starts and renders on Mesa lavapipe (software Vulkan) under Xvfb. **Not run on real hardware** |

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
deployment target, i.e. the SDK version 27.0 unless `CMAKE_OSX_DEPLOYMENT_TARGET` is set), `TrailForge.icns`
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
