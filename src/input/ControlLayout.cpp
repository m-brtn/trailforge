#include "input/ControlLayout.h"

#include <algorithm>
#include <cmath>

namespace tf {

KeyboardStyle nativeKeyboardStyle() {
#if defined(__APPLE__)
    return KeyboardStyle::Mac;
#else
    return KeyboardStyle::Pc;
#endif
}

const char* keyLabel(Key key, KeyboardStyle style) {
    const bool mac = style == KeyboardStyle::Mac;
    switch (key) {
    case Key::Escape: return "ESC";
    case Key::F1: return "F1";
    case Key::F2: return "F2";
    case Key::F3: return "F3";
    case Key::Backspace: return mac ? "DELETE" : "BACKSPACE";
    case Key::PageUp: return mac ? "PAGE\nUP" : "PG UP";
    case Key::PageDown: return mac ? "PAGE\nDOWN" : "PG DN";
    case Key::Return: return mac ? "RETURN" : "ENTER";
    case Key::LeftShift: return "SHIFT";
    case Key::Space: return "SPACE";
    case Key::LeftCtrl: return mac ? "CONTROL" : "CTRL";
    case Key::LeftAlt: return mac ? "OPTION" : "ALT";
    case Key::LeftGui: return mac ? "COMMAND" : "WIN";
    case Key::Up: case Key::Down: case Key::Left: case Key::Right: return "";
    case Key::Q: return "Q";
    case Key::W: return "W";
    case Key::R: return "R";
    case Key::T: return "T";
    case Key::P: return "P";
    case Key::A: return "A";
    case Key::S: return "S";
    case Key::D: return "D";
    case Key::H: return "H";
    case Key::L: return "L";
    case Key::X: return "X";
    case Key::C: return "C";
    case Key::V: return "V";
    case Key::B: return "B";
    case Key::N: return "N";
    case Key::M: return "M";
    case Key::Count: break;
    }
    return "";
}

const char* keyName(Key key, KeyboardStyle style) {
    const bool mac = style == KeyboardStyle::Mac;
    switch (key) {
    case Key::Up: return "UP";
    case Key::Down: return "DOWN";
    case Key::Left: return "LEFT";
    case Key::Right: return "RIGHT";
    case Key::PageUp: return mac ? "PAGE UP" : "PG UP";
    case Key::PageDown: return mac ? "PAGE DOWN" : "PG DN";
    default: return keyLabel(key, style);
    }
}

KeyRect keyRect(Key key) {
    switch (key) {
    case Key::Escape: return {0.0f, 0};
    case Key::F1: return {2.0f, 0};
    case Key::F2: return {3.0f, 0};
    case Key::F3: return {4.0f, 0};
    case Key::Backspace: return {13.0f, 1, 2.0f};
    case Key::PageUp: return {17.25f, 1};
    case Key::Q: return {1.5f, 2};
    case Key::W: return {2.5f, 2};
    case Key::R: return {4.5f, 2};
    case Key::T: return {5.5f, 2};
    case Key::P: return {10.5f, 2};
    case Key::PageDown: return {17.25f, 2};
    case Key::A: return {1.75f, 3};
    case Key::S: return {2.75f, 3};
    case Key::D: return {3.75f, 3};
    case Key::H: return {6.75f, 3};
    case Key::L: return {9.75f, 3};
    case Key::Return: return {12.75f, 3, 2.25f};
    case Key::LeftShift: return {0.0f, 4, 2.25f};
    case Key::X: return {3.25f, 4};
    case Key::C: return {4.25f, 4};
    case Key::V: return {5.25f, 4};
    case Key::B: return {6.25f, 4};
    case Key::N: return {7.25f, 4};
    case Key::M: return {8.25f, 4};
    case Key::Up: return {16.25f, 4};
    case Key::Space: return {3.75f, 5, 6.25f};
    case Key::Left: return {15.25f, 5};
    case Key::Down: return {16.25f, 5};
    case Key::Right: return {17.25f, 5};
    case Key::LeftCtrl: return {0.0f, 5, 1.25f};
    case Key::LeftGui: return {1.25f, 5, 1.25f};
    case Key::LeftAlt: return {2.5f, 5, 1.25f};
    case Key::Count: break;
    }
    return {};
}

DriveKeyMap driveKeyMap(DriveKeys layout) {
    if (layout == DriveKeys::Arrows) return {Key::Up, Key::Down, Key::Left, Key::Right};
    return {Key::W, Key::S, Key::A, Key::D};
}

DriveKeyMap freeCameraKeyMap(DriveKeys layout) {
    return driveKeyMap(layout == DriveKeys::Arrows ? DriveKeys::Wasd : DriveKeys::Arrows);
}

const std::vector<KeyAction>& keyActions() {
    static const std::vector<KeyAction> k = {
        {Key::Q, Action::ToggleDirection}, {Key::L, Action::ToggleLowRange}, {Key::R, Action::Reset},
        {Key::C, Action::ToggleCamera},    {Key::N, Action::StepOnce},       {Key::P, Action::TogglePause},
        {Key::H, Action::ToggleHeadlights}, {Key::B, Action::ToggleAbs},     {Key::T, Action::CycleTraction},
        {Key::X, Action::CycleDiffLock},   {Key::V, Action::CycleTransfer},  {Key::F1, Action::ToggleDebugUI},
        {Key::F2, Action::ToggleGraphicsUI}, {Key::F3, Action::ToggleAudioUI}, {Key::M, Action::ToggleMap},
    };
    return k;
}

const char* actionLabel(Action a) {
    switch (a) {
    case Action::ToggleDirection: return "FWD / REV";
    case Action::ToggleLowRange: return "LOW RANGE";
    case Action::Reset: return "RESET";
    case Action::ToggleCamera: return "CAMERA: FOLLOW / CLOSE / HOOD / FREE";
    case Action::ToggleDebugUI: return "DIAGNOSTICS";
    case Action::ToggleGraphicsUI: return "GRAPHICS";
    case Action::ToggleAudioUI: return "AUDIO";
    case Action::ToggleHeadlights: return "HEADLIGHTS";
    case Action::ToggleAbs: return "ABS";
    case Action::CycleTraction: return "TRACTION CONTROL";
    case Action::CycleDiffLock: return "DIFF LOCKS";
    case Action::CycleTransfer: return "2H / 4H / 4L";
    case Action::TogglePause: return "DEBUG FREEZE";
    case Action::StepOnce: return "SINGLE STEP";
    case Action::ToggleMap: return "MAP";
    case Action::Quit: return "QUIT";
    case Action::Count: break;
    }
    return "";
}

namespace {

BindingGroup actionGroup(Action a) {
    switch (a) {
    case Action::ToggleCamera: return BindingGroup::Camera;
    case Action::ToggleDebugUI: case Action::ToggleGraphicsUI: case Action::ToggleAudioUI: case Action::TogglePause:
    case Action::StepOnce: case Action::Quit: case Action::ToggleMap: return BindingGroup::System;
    default: return BindingGroup::Vehicle;
    }
}

} // namespace

std::vector<KeyBinding> keyboardBindings(DriveKeys layout) {
    const DriveKeyMap d = driveKeyMap(layout), c = freeCameraKeyMap(layout);
    std::vector<KeyBinding> b = {
        {d.forward, "THROTTLE", BindingGroup::Drive},
        {d.back, "BRAKE / REVERSE", BindingGroup::Drive},
        {d.left, "STEER LEFT", BindingGroup::Drive},
        {d.right, "STEER RIGHT", BindingGroup::Drive},
        {Key::Space, "HANDBRAKE", BindingGroup::Drive},
        {c.forward, "CAMERA FORWARD", BindingGroup::Camera},
        {c.back, "CAMERA BACK", BindingGroup::Camera},
        {c.left, "CAMERA LEFT", BindingGroup::Camera},
        {c.right, "CAMERA RIGHT", BindingGroup::Camera},
        {Key::PageUp, "CAMERA UP", BindingGroup::Camera},
        {Key::PageDown, "CAMERA DOWN", BindingGroup::Camera},
        {Key::LeftShift, "FAST CAMERA", BindingGroup::Camera},
        {Key::Escape, "PAUSE MENU", BindingGroup::System},
    };
    for (const KeyAction& a : keyActions()) b.push_back({a.key, actionLabel(a.action), actionGroup(a.action)});
    return b;
}

const char* driveKeysName(DriveKeys layout) { return layout == DriveKeys::Arrows ? "ARROWS" : "W A S D"; }

std::string driveHint(DriveKeys layout, KeyboardStyle style) {
    const DriveKeyMap d = driveKeyMap(layout);
    return std::string(keyName(d.forward, style)) + " gas / " + keyName(d.back, style) +
           " brake (stopped: release, press again for reverse)  " + keyName(d.left, style) + "/" +
           keyName(d.right, style) + " steer  " + keyName(Key::Space, style) + " handbrake";
}

PadGlyphs glyphsForPadType(PadType type) {
    switch (type) {
    case PadType::PS3: case PadType::PS4: case PadType::PS5: return PadGlyphs::PlayStation;
    default: return PadGlyphs::Xbox;
    }
}

PadGlyphs resolvePadGlyphs(PadGlyphSetting setting, PadType connected) {
    switch (setting) {
    case PadGlyphSetting::Xbox: return PadGlyphs::Xbox;
    case PadGlyphSetting::PlayStation: return PadGlyphs::PlayStation;
    case PadGlyphSetting::Auto: break;
    }
    return glyphsForPadType(connected);
}

const char* padGlyphsName(PadGlyphs g) { return g == PadGlyphs::PlayStation ? "PLAYSTATION" : "XBOX"; }

const char* padTypeName(PadType type) {
    switch (type) {
    case PadType::Unknown: return "unknown";
    case PadType::Standard: return "standard";
    case PadType::Xbox360: return "Xbox 360";
    case PadType::XboxOne: return "Xbox One";
    case PadType::PS3: return "PS3";
    case PadType::PS4: return "PS4";
    case PadType::PS5: return "PS5";
    case PadType::SwitchPro: return "Switch Pro";
    case PadType::JoyConLeft: return "Joy-Con (L)";
    case PadType::JoyConRight: return "Joy-Con (R)";
    case PadType::JoyConPair: return "Joy-Con pair";
    case PadType::GameCube: return "GameCube";
    case PadType::Steam: return "Steam";
    case PadType::None: return "none";
    }
    return "unknown";
}

const char* padControlName(PadControl c, PadGlyphs glyphs) {
    const bool ps = glyphs == PadGlyphs::PlayStation;
    switch (c) {
    case PadControl::South: return ps ? "CROSS" : "A";
    case PadControl::East: return ps ? "CIRCLE" : "B";
    case PadControl::West: return ps ? "SQUARE" : "X";
    case PadControl::North: return ps ? "TRIANGLE" : "Y";
    case PadControl::LeftShoulder: return ps ? "L1" : "LB";
    case PadControl::RightShoulder: return ps ? "R1" : "RB";
    case PadControl::LeftTrigger: return ps ? "L2" : "LT";
    case PadControl::RightTrigger: return ps ? "R2" : "RT";
    case PadControl::LeftStick: return "LEFT STICK";
    case PadControl::RightStick: return "RIGHT STICK";
    case PadControl::DpadUp: return "D-PAD UP";
    case PadControl::DpadDown: return "D-PAD DOWN";
    case PadControl::DpadLeft: return "D-PAD LEFT";
    case PadControl::DpadRight: return "D-PAD RIGHT";
    case PadControl::Back: return ps ? "CREATE" : "VIEW";
    case PadControl::Start: return ps ? "OPTIONS" : "MENU";
    case PadControl::Guide: return ps ? "PS" : "GUIDE";
    case PadControl::Count: break;
    }
    return "";
}

const std::vector<PadAction>& padActions() {
    static const std::vector<PadAction> p = {
        {PadControl::West, Action::ToggleDirection},  {PadControl::North, Action::ToggleLowRange},
        {PadControl::Back, Action::Reset},            {PadControl::RightShoulder, Action::ToggleCamera},
        {PadControl::DpadUp, Action::ToggleAbs},      {PadControl::DpadRight, Action::CycleTraction},
        {PadControl::DpadDown, Action::CycleDiffLock}, {PadControl::DpadLeft, Action::CycleTransfer},
        {PadControl::LeftShoulder, Action::ToggleMap},
    };
    return p;
}

std::vector<PadBinding> padBindings() {
    std::vector<PadBinding> b = {
        {PadControl::RightTrigger, "THROTTLE", BindingGroup::Drive},
        {PadControl::LeftTrigger, "BRAKE / REVERSE", BindingGroup::Drive},
        {PadControl::LeftStick, "STEER", BindingGroup::Drive},
        {PadControl::South, "HANDBRAKE", BindingGroup::Drive},
        {PadControl::RightStick, "CAMERA ORBIT / LOOK", BindingGroup::Camera},
        {PadControl::Start, "PAUSE MENU", BindingGroup::System},
    };
    for (const PadAction& a : padActions()) b.push_back({a.control, actionLabel(a.action), actionGroup(a.action)});
    return b;
}

StickValue rightStickValue(float x, float y) {
    const float m = std::hypot(x, y);
    if (m <= kRightStickDeadZone) return {};
    const float t = std::min(1.0f, (m - kRightStickDeadZone) / (1.0f - kRightStickDeadZone));
    const float scale = t * t / m;
    return {x * scale, y * scale};
}

std::vector<float> spreadLabels(const std::vector<float>& wanted, const std::vector<float>& sizes, float pad, float lo,
                                float hi) {
    // With q_i = p_i - o_i (o_i: the smallest distance from label 0 to label i) the spacing constraint becomes q
    // non-decreasing: least-squares isotonic regression (pool adjacent violators) of wanted_i - o_i, then clamped to
    // the box, which stays optimal because the bounds are the same for every q_i.
    const std::size_t n = wanted.size();
    if (n == 0) return {};
    std::vector<float> offset(n, 0.0f);
    for (std::size_t i = 1; i < n; ++i) offset[i] = offset[i - 1] + 0.5f * (sizes[i - 1] + sizes[i]) + pad;
    struct Block {
        float sum;
        int count;
    };
    std::vector<Block> blocks;
    for (std::size_t i = 0; i < n; ++i) {
        blocks.push_back({wanted[i] - offset[i], 1});
        while (blocks.size() > 1) {
            const Block& b = blocks.back();
            const Block& a = blocks[blocks.size() - 2];
            if (a.sum / float(a.count) <= b.sum / float(b.count)) break;
            const Block merged{a.sum + b.sum, a.count + b.count};
            blocks.pop_back();
            blocks.back() = merged;
        }
    }
    const float qLo = lo + 0.5f * sizes[0];
    const float qHi = std::max(qLo, hi - 0.5f * sizes[n - 1] - offset[n - 1]);
    std::vector<float> out;
    out.reserve(n);
    for (const Block& b : blocks)
        for (int k = 0; k < b.count; ++k) out.push_back(std::clamp(b.sum / float(b.count), qLo, qHi));
    for (std::size_t i = 0; i < n; ++i) out[i] += offset[i];
    return out;
}

} // namespace tf
