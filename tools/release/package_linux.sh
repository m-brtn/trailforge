#!/usr/bin/env bash
# Builds, tests and packages TrailForge for Linux x86_64 / arm64 as a tar.gz (executable + assets).
#
#   tools/release/package_linux.sh                 # build + test + package into dist/
#   tools/release/package_linux.sh --smoke-window  # additionally start the window for a few seconds (needs a GPU
#                                                  # with Vulkan, or Mesa lavapipe under xvfb in CI)
#
# Steps: clean Release configure in build/package with TRAILFORGE_PACKAGED=ON (assets are read only from next to
# the executable), build, ctest, assemble dist/TrailForge-<version>-linux-<arch>/ (trailforge, assets/ (all of it),
# README.txt), check that the binary has no source-tree references and only system shared libraries, tar.gz it,
# then smoke test a fresh extraction from another working directory (--version, --paths: the asset directory must
# be the one inside the package). The version comes from project(TrailForge VERSION x.y.z) in CMakeLists.txt.
# Requires the SDL3 build dependencies (X11/Wayland/ALSA/PulseAudio headers, see .github/workflows/ci.yml).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BUILD="$ROOT/build/package"
DIST="$ROOT/dist"
SMOKE_WINDOW=0
for arg in "$@"; do
    case "$arg" in
    --smoke-window) SMOKE_WINDOW=1 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

VERSION="$(sed -n 's/^project(TrailForge VERSION \([0-9][0-9.]*\).*/\1/p' "$ROOT/CMakeLists.txt")"
[[ -n "$VERSION" ]] || { echo "cannot read the version from CMakeLists.txt" >&2; exit 1; }
case "$(uname -m)" in
x86_64) ARCH=x64 ;;
aarch64 | arm64) ARCH=arm64 ;;
*) ARCH="$(uname -m)" ;;
esac
NAME="TrailForge-$VERSION-linux-$ARCH"
PKG="$DIST/$NAME"
TGZ="$DIST/$NAME.tar.gz"
step() { printf '\n=== %s\n' "$*"; }
fail() { echo "FAIL: $*" >&2; exit 1; }

step "TrailForge $VERSION ($ARCH): clean Release configure"
rm -rf "$BUILD"
cmake -S "$ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release -DTRAILFORGE_PACKAGED=ON \
    -DTRAILFORGE_BUILD_APP=ON -DTRAILFORGE_BUILD_TESTS=ON >/dev/null

step "build"
cmake --build "$BUILD"

step "ctest"
ctest --test-dir "$BUILD" --output-on-failure

step "assemble dist/$NAME"
[[ "$("$BUILD/trailforge" --version)" == "TrailForge $VERSION" ]] || fail "binary version mismatch"
mkdir -p "$DIST"
rm -rf "$PKG" "$TGZ"
mkdir -p "$PKG/assets"
cp "$BUILD/trailforge" "$PKG/"
strip "$PKG/trailforge"
cp -R "$ROOT/assets/." "$PKG/assets/"
cp "$BUILD/assets/images/logo.png" "$PKG/assets/images/" # copied from external_assets/images at configure time
cat >"$PKG/README.txt" <<EOF
TrailForge $VERSION for Linux ($ARCH)

UNTESTED ON REAL HARDWARE: this build was compiled and its headless tests passed in CI,
but it has not been run on a real Linux machine with a GPU.

Run:  ./trailforge          (needs a Vulkan driver: Mesa RADV/ANV/NVK, NVIDIA, or lavapipe)
      ./trailforge --help   (options), ./trailforge --paths (asset and user data directories)
Keep the assets/ directory next to the executable.
EOF
for f in audio/manifest.json music/intro.mp3 fonts/Rajdhani-SemiBold.ttf fonts/OFL.txt images/logo.png \
         vehicles/trailforge_suv/vehicle.json levels/exploration_peaks/level.json; do
    [[ -f "$PKG/assets/$f" ]] || fail "assets/$f missing in the package"
done

step "shared library dependencies"
ldd "$PKG/trailforge" || true
if strings "$PKG/trailforge" | grep -qF "$ROOT/assets"; then
    fail "the packaged binary still references the source tree assets"
fi

step "tar.gz"
tar -C "$DIST" -czf "$TGZ" "$NAME"
ls -l "$TGZ"
sha256sum "$TGZ"

step "smoke test: fresh extraction, run from /"
EXTRACT="$(mktemp -d)"
tar -C "$EXTRACT" -xzf "$TGZ"
BIN="$EXTRACT/$NAME/trailforge"
(cd / && "$BIN" --version)
PATHS="$(cd / && "$BIN" --paths)"
echo "$PATHS"
FIRST_ASSETS="$(echo "$PATHS" | sed -n 's/^assets: //p' | head -n 1)"
[[ "$(echo "$PATHS" | grep -c '^assets: ')" == "1" ]] || fail "packaged build must have exactly one asset directory"
[[ -f "$FIRST_ASSETS/manifest.json" ]] || fail "asset directory $FIRST_ASSETS does not contain manifest.json"
MUSIC_DIR="$(echo "$PATHS" | sed -n 's/^music: //p' | head -n 1)"
[[ -f "$MUSIC_DIR/intro.mp3" ]] || fail "music directory $MUSIC_DIR does not contain intro.mp3"

if [[ $SMOKE_WINDOW -eq 1 ]]; then
    step "smoke test: window (Vulkan)"
    LOG="$(mktemp)"
    (cd / && "$BIN" --no-ui --quit-after 4 --audio-capture "$LOG.wav") >"$LOG" 2>&1 || { cat "$LOG"; fail "window smoke run failed"; }
    grep -E "GPU driver|sounds loaded|auto-quit after" "$LOG"
    grep -F "sounds loaded from $FIRST_ASSETS" "$LOG" | grep -F "0 missing" || { cat "$LOG"; fail "assets did not load"; }
    grep -F "Audio: music $MUSIC_DIR/intro.mp3" "$LOG" || { cat "$LOG"; fail "music did not load"; }
    grep -E "^Menu: (font|logo) " "$LOG" || { cat "$LOG"; fail "menu font / logo did not load"; }
    grep -E "^Levels: [1-9]" "$LOG" || { cat "$LOG"; fail "levels did not load"; }
    rm -f "$LOG" "$LOG.wav" "$LOG.wav.csv"
fi
rm -rf "$EXTRACT"

step "done: $TGZ"
