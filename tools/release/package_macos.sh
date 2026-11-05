#!/usr/bin/env bash
# Builds, tests, signs (ad hoc) and zips a self-contained TrailForge.app for macOS / Apple Silicon.
#
#   tools/release/package_macos.sh            # build + test + package into dist/
#   tools/release/package_macos.sh --publish  # the same, then create the GitHub release v<version>
#
# Steps: clean Release configure in build/package with TRAILFORGE_PACKAGED=ON (assets are read only from the
# bundle), build, ctest, copy the bundle to dist/TrailForge.app, check its dynamic dependencies (system only),
# codesign --force --deep -s - and verify, zip with ditto to dist/TrailForge-<version>-macos-arm64.zip, then smoke
# test the bundle from another working directory and from a fresh unzip of the archive (assets must resolve from
# inside the bundle). The version comes from project(TrailForge VERSION x.y.z) in CMakeLists.txt.
# --publish needs a clean, pushed main, the notes in docs/releases/v<version>.md and an authenticated gh.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BUILD="$ROOT/build/package"
DIST="$ROOT/dist"
APP="$DIST/TrailForge.app"
PUBLISH=0
for arg in "$@"; do
    case "$arg" in
    --publish) PUBLISH=1 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

VERSION="$(sed -n 's/^project(TrailForge VERSION \([0-9][0-9.]*\).*/\1/p' "$ROOT/CMakeLists.txt")"
[[ -n "$VERSION" ]] || { echo "cannot read the version from CMakeLists.txt" >&2; exit 1; }
ARCH="$(uname -m)"
ZIP="$DIST/TrailForge-$VERSION-macos-$ARCH.zip"
NOTES="$ROOT/docs/releases/v$VERSION.md"
step() { printf '\n=== %s\n' "$*"; }
fail() { echo "FAIL: $*" >&2; exit 1; }

if [[ $PUBLISH -eq 1 ]]; then
    [[ -f "$NOTES" ]] || fail "release notes $NOTES are missing"
    [[ -z "$(git -C "$ROOT" status --porcelain)" ]] || fail "working tree is not clean"
    git -C "$ROOT" fetch -q origin
    [[ "$(git -C "$ROOT" rev-parse HEAD)" == "$(git -C "$ROOT" rev-parse origin/main)" ]] || fail "HEAD is not origin/main"
    gh release view "v$VERSION" >/dev/null 2>&1 && fail "release v$VERSION already exists (bump the version)"
fi

step "TrailForge $VERSION ($ARCH): clean Release configure"
rm -rf "$BUILD"
cmake -S "$ROOT" -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release -DTRAILFORGE_PACKAGED=ON \
    -DTRAILFORGE_BUILD_APP=ON -DTRAILFORGE_BUILD_TESTS=ON >/dev/null

step "build"
cmake --build "$BUILD"

step "ctest"
ctest --test-dir "$BUILD" --output-on-failure

step "assemble dist/TrailForge.app"
[[ "$("$BUILD/trailforge" --version)" == "TrailForge $VERSION" ]] || fail "binary version mismatch"
mkdir -p "$DIST"
rm -rf "$APP" "$ZIP"
ditto "$BUILD/TrailForge.app" "$APP"
for f in audio/manifest.json music/intro.mp3 fonts/Rajdhani-SemiBold.ttf fonts/OFL.txt images/logo.png \
         vehicles/trailforge_suv/vehicle.json levels/exploration_peaks/level.json textures/terrain/terrain.json \
         textures/terrain/grass_albedo.ktx2; do
    [[ -f "$APP/Contents/Resources/assets/$f" ]] || fail "assets/$f missing in the bundle"
done
# A Git LFS pointer instead of the texture (clone without `git lfs pull`) is ~130 bytes.
[[ "$(stat -f %z "$APP/Contents/Resources/assets/textures/terrain/grass_albedo.ktx2")" -gt 1000000 ]] ||
    fail "terrain textures are Git LFS pointers (run git lfs pull)"
[[ -f "$APP/Contents/Resources/TrailForge.icns" ]] || fail "icns missing in the bundle"
[[ "$(/usr/libexec/PlistBuddy -c 'Print :CFBundleShortVersionString' "$APP/Contents/Info.plist")" == "$VERSION" ]] ||
    fail "Info.plist version mismatch"
# LaunchServices refuses to open the app ("You can't use this version of the application") when
# LSMinimumSystemVersion is above the running macOS, even if the binary's minos would allow it.
ver_le() { [[ "$(printf '%s\n%s\n' "$1" "$2" | sort -V | head -n 1)" == "$1" ]]; }
MIN_PLIST="$(/usr/libexec/PlistBuddy -c 'Print :LSMinimumSystemVersion' "$APP/Contents/Info.plist")"
MIN_BINARY="$(vtool -show-build "$APP/Contents/MacOS/trailforge" | awk '$1 == "minos" { print $2; exit }')"
HOST_MACOS="$(sw_vers -productVersion)"
echo "LSMinimumSystemVersion $MIN_PLIST, binary minos $MIN_BINARY, host macOS $HOST_MACOS"
[[ -n "$MIN_PLIST" && -n "$MIN_BINARY" ]] || fail "cannot read LSMinimumSystemVersion or the binary minos"
ver_le "$MIN_PLIST" "$MIN_BINARY" || fail "LSMinimumSystemVersion $MIN_PLIST is above the binary minos $MIN_BINARY"
ver_le "$MIN_PLIST" "$HOST_MACOS" || fail "LSMinimumSystemVersion $MIN_PLIST is above this Mac's macOS $HOST_MACOS"
cmp -s "$BUILD/trailforge" "$APP/Contents/MacOS/trailforge" || fail "bundle executable differs from the build"

step "dynamic dependencies (only system libraries expected)"
otool -L "$APP/Contents/MacOS/trailforge"
if otool -L "$APP/Contents/MacOS/trailforge" | tail -n +2 | awk '{print $1}' | grep -vE '^(/System/Library/|/usr/lib/)'; then
    fail "non-system dynamic dependency"
fi
if strings "$APP/Contents/MacOS/trailforge" | grep -qF "$ROOT/assets"; then
    fail "the packaged binary still references the source tree assets"
fi

step "ad-hoc codesign"
codesign --force --deep -s - "$APP"
codesign --verify --deep --strict --verbose=2 "$APP"

step "zip"
ditto -c -k --keepParent "$APP" "$ZIP"
ls -l "$ZIP"
shasum -a 256 "$ZIP"

# Runs the bundle executable from an unrelated working directory with the offline audio capture (no device needed):
# the sounds must load from the bundle's Contents/Resources.
smoke() {
    local app log
    app="$(cd "$1" && pwd -P)" # SDL reports the resolved path (/var -> /private/var)
    log="$(mktemp -t trailforge_smoke)"
    (cd / && "$app/Contents/MacOS/trailforge" --no-ui --quit-after 4 --audio-capture "$log.wav") >"$log" 2>&1 ||
        { cat "$log"; fail "smoke run of $app failed"; }
    grep -F "sounds loaded from $app/Contents/Resources/assets/audio" "$log" | grep -F "0 missing" ||
        { cat "$log"; fail "assets did not resolve from $app"; }
    grep -F "Audio: music $app/Contents/Resources/assets/music/intro.mp3" "$log" || { cat "$log"; fail "music missing"; }
    grep -F "Menu: font $app/Contents/Resources/assets/fonts/" "$log" || { cat "$log"; fail "menu font missing"; }
    grep -F "Menu: logo $app/Contents/Resources/assets/images/logo.png" "$log" || { cat "$log"; fail "logo missing"; }
    grep -F "definition(s) from $app/Contents/Resources/assets/vehicles" "$log" ||
        { cat "$log"; fail "vehicle definitions missing"; }
    grep -F "level(s) from $app/Contents/Resources/assets/levels" "$log" || { cat "$log"; fail "levels missing"; }
    grep -F "Textures: terrain materials from $app/Contents/Resources/assets/textures" "$log" ||
        { cat "$log"; fail "terrain textures missing"; }
    grep -F "app: quit after" "$log" || { cat "$log"; fail "no quit-after line"; }
    rm -f "$log" "$log.wav" "$log.wav.csv"
}

step "smoke test: dist/TrailForge.app from /"
smoke "$APP"

step "smoke test: fresh unzip"
UNZIP="$(mktemp -d -t trailforge_unzip)"
ditto -x -k "$ZIP" "$UNZIP"
codesign --verify --deep --strict "$UNZIP/TrailForge.app"
smoke "$UNZIP/TrailForge.app"
rm -rf "$UNZIP"

if [[ $PUBLISH -eq 1 ]]; then
    step "GitHub release v$VERSION"
    gh release create "v$VERSION" "$ZIP" --target "$(git -C "$ROOT" rev-parse HEAD)" \
        --title "TrailForge v$VERSION" --notes-file "$NOTES"
fi

step "done: $APP and $ZIP"
