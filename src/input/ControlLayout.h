#pragma once

#include <algorithm>
#include <string>
#include <vector>

namespace tf {

// Bindings, key cap labels and gamepad glyph names in plain C++ (no SDL), shared by Input (which maps them to SDL
// keys and buttons) and the Controls page (which draws them), and tested headless.

// Discrete user actions (triggered on press). Keys and buttons: keyActions() / padActions().
enum class Action {
    ToggleDirection,
    ToggleLowRange,
    Reset,
    ToggleCamera,
    ToggleDebugUI,    // F1-F3 work even while ImGui owns the keyboard
    ToggleGraphicsUI,
    ToggleAudioUI,
    ToggleHeadlights,
    ToggleAbs,
    CycleTraction,
    CycleDiffLock,
    CycleTransfer,    // part-time transfer case 2H -> 4H -> 4L
    TogglePause,      // debug freeze without the menu (Escape / gamepad Start open the pause menu, see MenuKeys)
    StepOnce,         // while frozen: a single physics step
    ToggleMap,        // full-screen map (M / pad LB): waypoint marker, no teleports
    Quit,
    Count
};

// ------------------------------------------------------------------ keyboard

// Which cluster drives (throttle, brake / reverse, steering). The other cluster moves the free camera.
enum class DriveKeys { Wasd, Arrows };

// Key cap wording: macOS keyboards say Return, Delete, Option, Command; PC keyboards Enter, Backspace, Alt, Ctrl.
enum class KeyboardStyle { Mac, Pc };
// The style of the platform the game runs on (chosen at compile time: macOS -> Mac, everything else -> Pc).
KeyboardStyle nativeKeyboardStyle();

// Physical keys the game binds (and the modifiers, for labels). Positions are those of a US ANSI keyboard.
enum class Key {
    Escape, F1, F2, F3,
    Backspace, PageUp,
    Q, W, R, T, P, PageDown,
    A, S, D, H, L, Return,
    LeftShift, X, C, V, B, N, M, Up,
    Space, Left, Down, Right,
    LeftCtrl, LeftAlt, LeftGui,
    Count
};

// Key cap text in the given style; empty for the arrow keys (drawn as arrows). '\n' splits two-line caps.
const char* keyLabel(Key key, KeyboardStyle style);
// Short name for hints and text ("ENTER", "RETURN", "W", "UP").
const char* keyName(Key key, KeyboardStyle style);

// Key cap rectangle in key units (1 = one letter key pitch) on an ANSI keyboard: x from the left edge of Esc, row 0
// the function row, 1 the number row ... 5 the space bar row.
struct KeyRect {
    float x = 0.0f;
    int row = 0;
    float w = 1.0f;
};
KeyRect keyRect(Key key);

struct DriveKeyMap {
    Key forward, back, left, right;
};
// Throttle (forward pedal), brake / reverse (backward pedal), steer left, steer right.
DriveKeyMap driveKeyMap(DriveKeys layout);
// Free camera move keys: the cluster that does not drive.
DriveKeyMap freeCameraKeyMap(DriveKeys layout);

struct KeyAction {
    Key key;
    Action action;
};
// Discrete actions on key press (letters and F1-F3; Escape opens the pause menu through MenuKeys).
const std::vector<KeyAction>& keyActions();

enum class BindingGroup { Drive, Camera, Vehicle, System };
struct KeyBinding {
    Key key;
    std::string label; // callout text (uppercase)
    BindingGroup group;
};
// Every bound gameplay key with its callout, for the given drive layout. Each key appears once. Menu navigation
// (Return / Backspace / arrows) is left out: the menu footers name it.
std::vector<KeyBinding> keyboardBindings(DriveKeys layout);

const char* actionLabel(Action a);
const char* driveKeysName(DriveKeys layout); // "W A S D" / "ARROWS"

// HUD line for the driving keys, e.g. "W gas / S brake (stopped: release, press again for reverse)  A/D steer".
std::string driveHint(DriveKeys layout, KeyboardStyle style);

// ------------------------------------------------------------------ gamepad

enum class PadGlyphs { Xbox, PlayStation };
// Settings value: 0 follows the connected pad, 1 Xbox, 2 PlayStation.
enum class PadGlyphSetting { Auto = 0, Xbox = 1, PlayStation = 2 };

// The same order and values as SDL_GamepadType (Input.cpp checks this at compile time), plus None for no pad.
enum class PadType {
    Unknown = 0, Standard, Xbox360, XboxOne, PS3, PS4, PS5, SwitchPro, JoyConLeft, JoyConRight, JoyConPair, GameCube,
    Steam, None = 100
};
PadGlyphs glyphsForPadType(PadType type); // PlayStation pads -> PlayStation, everything else -> Xbox
PadGlyphs resolvePadGlyphs(PadGlyphSetting setting, PadType connected);
const char* padGlyphsName(PadGlyphs g); // "XBOX" / "PLAYSTATION"
const char* padTypeName(PadType type);

// Buttons and axes by position (SDL names them south / east / west / north).
enum class PadControl {
    South, East, West, North, LeftShoulder, RightShoulder, LeftTrigger, RightTrigger, LeftStick, RightStick,
    DpadUp, DpadDown, DpadLeft, DpadRight, Back, Start, Guide, Count
};
// "A" / "CROSS", "LT" / "L2", "VIEW" / "CREATE", "MENU" / "OPTIONS", "LEFT STICK", "D-PAD UP".
const char* padControlName(PadControl c, PadGlyphs glyphs);

struct PadAction {
    PadControl control;
    Action action;
};
// Discrete gameplay actions on button press (South / East / Start drive the menus through MenuKeys).
const std::vector<PadAction>& padActions();

struct PadBinding {
    PadControl control;
    std::string label;
    BindingGroup group;
};
// Every bound gameplay control with its callout (unbound controls and menu-only buttons are absent). Each control
// appears once.
std::vector<PadBinding> padBindings();

// Right stick (camera): a radial dead zone rescaled to 0..1, then a squared response for fine aim near the centre.
struct StickValue {
    float x = 0.0f, y = 0.0f;
};
inline constexpr float kRightStickDeadZone = 0.15f;
StickValue rightStickValue(float x, float y);

// ------------------------------------------------------------------ callout placement

// Label centres along one axis (least squares from `wanted`, which must be sorted): neighbours do not overlap (label i
// covers centre +- sizes[i] / 2, plus `pad` between labels) and all labels stay inside [lo, hi] when they fit. Used
// for the key callouts of a keyboard row and the gamepad callout columns.
std::vector<float> spreadLabels(const std::vector<float>& wanted, const std::vector<float>& sizes, float pad, float lo,
                                float hi);

// Splits a callout into at most two lines so that each fits `maxWidth` by the given width function (word breaks
// only, balanced); returns one line when it fits or cannot be split.
template <typename WidthFn>
std::vector<std::string> wrapLabel(const std::string& s, float maxWidth, WidthFn width) {
    if (width(s) <= maxWidth) return {s};
    std::vector<std::string> best{s};
    float bestWidest = width(s);
    for (std::size_t i = s.find(' '); i != std::string::npos; i = s.find(' ', i + 1)) {
        std::string a = s.substr(0, i), b = s.substr(i + 1);
        if (!a.empty() && a.back() == '/') continue; // never end a line on a slash
        if (!b.empty() && b.front() == '/') {        // keep "A / B" split as "A /" + "B"
            a += " /";
            b = b.substr(b.find_first_not_of("/ "));
        }
        const float widest = std::max(width(a), width(b));
        if (widest < bestWidest) {
            bestWidest = widest;
            best = {a, b};
        }
    }
    return best;
}

} // namespace tf
