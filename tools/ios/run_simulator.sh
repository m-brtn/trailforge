#!/bin/bash
# Builds the iOS Simulator app (host-tools + ios-sim-release presets), boots a Simulator device, installs
# build/ios-sim-release/TrailForge.app and launches it with the console attached. Arguments after `--` go to the app.
#
#   tools/ios/run_simulator.sh                                   # build, install, launch on "iPhone 18 Pro"
#   tools/ios/run_simulator.sh --device "iPad Air 11-inch (M4)" -- --demo --quit-after 30
#   tools/ios/run_simulator.sh --no-build -- --capture /tmp/trailforge_ios.png --capture-at 20 --quit-after 25
#
# The device runs headless unless --gui opens the Simulator window (screenshots work either way:
# `simctl io <udid> screenshot file.png`). Here the Simulator's GPU host process (SimMetalHost) crashed several times
# when the window opened or another app launched, and the app then loses its Metal device (docs/building.md "iOS").
#
# SIMCTL overrides the simctl binary. By default CoreSimulator's own simctl is used when it exists: the `xcrun simctl`
# wrapper of Xcode runs `xcodebuild -runFirstLaunch` (which waits for an administrator password) whenever the
# installed CoreSimulator is older than the one Xcode expects.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
DEVICE="iPhone 18 Pro"
BUILD=1
GUI=0
APP_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --device) DEVICE="$2"; shift 2 ;;
        --no-build) BUILD=0; shift ;;
        --gui) GUI=1; shift ;;
        --) shift; APP_ARGS=("$@"); break ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done

CORE_SIMCTL=/Library/Developer/PrivateFrameworks/CoreSimulator.framework/Versions/A/Resources/bin/simctl
if [[ -z "${SIMCTL:-}" ]]; then
    if [[ -x "$CORE_SIMCTL" ]]; then SIMCTL="$CORE_SIMCTL"; else SIMCTL="xcrun simctl"; fi
fi
BUNDLE_ID=com.trailforge.app
APP="$ROOT/build/ios-sim-release/TrailForge.app"

if [[ $BUILD -eq 1 ]]; then
    cd "$ROOT"
    if [[ ! -x build/host-tools/_deps/quench-build/quench_shaderc ]]; then
        cmake --preset host-tools
        cmake --build --preset host-tools
    fi
    cmake --preset ios-sim-release
    cmake --build --preset ios-sim-release
fi

UDID="$($SIMCTL list devices available | grep -F "    $DEVICE (" | head -1 | sed -E 's/.*\(([0-9A-F-]{36})\).*/\1/')"
if [[ -z "$UDID" ]]; then
    echo "no available Simulator device named '$DEVICE' ($SIMCTL list devices available)" >&2
    exit 1
fi
$SIMCTL boot "$UDID" 2>/dev/null || true
$SIMCTL bootstatus "$UDID" -b >/dev/null
if [[ $GUI -eq 1 ]]; then
    open -a Simulator --args -CurrentDeviceUDID "$UDID" || true
    sleep 3
fi
$SIMCTL install "$UDID" "$APP"
echo "Installed $APP on $DEVICE ($UDID); user data: $($SIMCTL get_app_container "$UDID" "$BUNDLE_ID" data)"
exec $SIMCTL launch --console-pty --terminate-running-process "$UDID" "$BUNDLE_ID" "${APP_ARGS[@]+"${APP_ARGS[@]}"}"
