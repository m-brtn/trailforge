#include "input/TouchControls.h"

#include <algorithm>
#include <cmath>

namespace tf {

const char* touchButtonName(TouchButton b) {
    switch (b) {
    case TouchButton::Throttle: return "throttle";
    case TouchButton::Brake: return "brake";
    case TouchButton::Handbrake: return "handbrake";
    case TouchButton::Pause: return "pause";
    case TouchButton::Map: return "map";
    case TouchButton::Camera: return "camera";
    case TouchButton::Reset: return "reset";
    case TouchButton::Lights: return "lights";
    case TouchButton::Count: break;
    }
    return "";
}

const char* touchButtonLabel(TouchButton b) {
    switch (b) {
    case TouchButton::Throttle: return "GAS";
    case TouchButton::Brake: return "BRAKE";
    case TouchButton::Handbrake: return "HAND";
    case TouchButton::Pause: return "II";
    case TouchButton::Map: return "MAP";
    case TouchButton::Camera: return "CAM";
    case TouchButton::Reset: return "R";
    case TouchButton::Lights: return "LIGHT";
    case TouchButton::Count: break;
    }
    return "";
}

TouchLayout touchLayout(float width, float height, float insetLeft, float insetTop, float insetRight, float insetBottom) {
    TouchLayout L;
    L.width = width;
    L.height = height;
    // One unit scales every control: ~80 points on a landscape iPhone (402 points tall), 120 on a large window.
    const float u = std::clamp(0.21f * height, 64.0f, 120.0f);
    const float m = 0.18f * u;
    const float right = width - insetRight - m, bottom = height - insetBottom - m;
    const float left = insetLeft + m, top = insetTop + m;
    auto& B = L.buttons;
    B[int(TouchButton::Throttle)] = {right - u, bottom - 1.9f * u, right, bottom};
    const float brakeX1 = right - 1.22f * u;
    B[int(TouchButton::Brake)] = {brakeX1 - u, bottom - 1.35f * u, brakeX1, bottom};
    const float handY1 = bottom - 1.55f * u;
    B[int(TouchButton::Handbrake)] = {brakeX1 - u, handY1 - 0.7f * u, brakeX1, handY1};
    // Top row, right to left.
    const float s = 0.56f * u, gap = 0.14f * u;
    float x1 = right;
    for (TouchButton b : {TouchButton::Pause, TouchButton::Map, TouchButton::Camera, TouchButton::Reset, TouchButton::Lights}) {
        B[int(b)] = {x1 - s, top, x1, top + s};
        x1 -= s + gap;
    }
    L.topRowLeft = x1 + gap;
    L.topRowBottom = top + s;
    L.steerZone = {0.0f, 0.28f * height, std::min(0.5f * width, B[int(TouchButton::Handbrake)].x0 - 0.3f * u), height};
    L.stickRadius = 0.75f * u;
    L.restX = left + 1.3f * u;
    L.restY = bottom - 1.0f * u;
    return L;
}

TouchControls::Finger* TouchControls::find(std::uint64_t id) {
    for (Finger& f : m_fingers)
        if (f.down && f.id == id) return &f;
    return nullptr;
}

const TouchControls::Finger* TouchControls::stickFinger() const {
    for (const Finger& f : m_fingers)
        if (f.down && f.control == kStick) return &f;
    return nullptr;
}

TouchButton TouchControls::fingerDown(std::uint64_t id, float x, float y) {
    Finger* f = find(id);
    if (!f) {
        for (Finger& g : m_fingers)
            if (!g.down) {
                f = &g;
                break;
            }
    }
    if (!f) return TouchButton::Count; // more fingers than slots: ignored
    *f = Finger{true, id, kNone, x, y, x, y};
    for (int b = 0; b < kTouchButtonCount; ++b)
        if (m_layout.buttons[std::size_t(b)].contains(x, y)) {
            f->control = b;
            return TouchButton(b);
        }
    // One stick: a second finger in the zone does nothing.
    if (m_layout.steerZone.contains(x, y) && !stickFinger()) f->control = kStick;
    return TouchButton::Count;
}

void TouchControls::fingerMove(std::uint64_t id, float x, float y) {
    Finger* f = find(id);
    if (!f) return;
    f->x = x;
    f->y = y;
    if (f->control == kStick) {
        // A floating centre: dragged along past full lock, so turning back answers at once.
        const float r = m_layout.stickRadius;
        if (x - f->x0 > r) f->x0 = x - r;
        if (f->x0 - x > r) f->x0 = x + r;
        return;
    }
    // A finger slides between the two pedals like a foot.
    const int gas = int(TouchButton::Throttle), brake = int(TouchButton::Brake);
    if (f->control == gas && m_layout.buttons[std::size_t(brake)].contains(x, y)) f->control = brake;
    else if (f->control == brake && m_layout.buttons[std::size_t(gas)].contains(x, y)) f->control = gas;
}

void TouchControls::fingerUp(std::uint64_t id) {
    if (Finger* f = find(id)) *f = Finger{};
}

void TouchControls::releaseAll() { m_fingers.fill(Finger{}); }

bool TouchControls::held(TouchButton b) const {
    for (const Finger& f : m_fingers)
        if (f.down && f.control == int(b)) return true;
    return false;
}

bool TouchControls::steering() const { return stickFinger() != nullptr; }

float TouchControls::steer() const {
    const Finger* f = stickFinger();
    if (!f) return 0.0f;
    const float d = std::clamp((f->x - f->x0) / std::max(m_layout.stickRadius, 1.0f), -1.0f, 1.0f);
    constexpr float kDead = 0.06f;
    if (std::abs(d) < kDead) return 0.0f;
    return std::copysign((std::abs(d) - kDead) / (1.0f - kDead), d);
}

float TouchControls::stickX() const {
    const Finger* f = stickFinger();
    return f ? f->x0 : m_layout.restX;
}

float TouchControls::stickY() const {
    const Finger* f = stickFinger();
    return f ? f->y0 : m_layout.restY;
}

float TouchControls::fingerX() const {
    const Finger* f = stickFinger();
    return f ? f->x : m_layout.restX;
}

float TouchControls::fingerY() const {
    const Finger* f = stickFinger();
    return f ? f->y : m_layout.restY;
}

int TouchControls::fingers() const {
    int n = 0;
    for (const Finger& f : m_fingers) n += f.down;
    return n;
}

} // namespace tf
