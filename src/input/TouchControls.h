#pragma once

#include <array>
#include <cstdint>

namespace tf {

// On-screen driving controls for touch screens (iOS), in plain C++ (no SDL): the layout in window points and the
// fingers on it. A finger belongs to the control it first touched until it lifts; any number of fingers at once.
// Steering is a floating stick: a finger anywhere in the left part of the screen sets its centre where it lands and
// steers by its horizontal travel from there. The pedals are held like keys (Input ramps them like the keyboard
// pedals); the other buttons act on the touch.

enum class TouchButton { Throttle, Brake, Handbrake, Pause, Map, Camera, Reset, Lights, Count };
constexpr int kTouchButtonCount = static_cast<int>(TouchButton::Count);

const char* touchButtonName(TouchButton b); // "throttle", "brake", ... (scenario `finger down ID button:NAME`)
const char* touchButtonLabel(TouchButton b); // "GAS", "BRAKE", ... (drawn on the button)

struct TouchRect {
    float x0 = 0.0f, y0 = 0.0f, x1 = 0.0f, y1 = 0.0f;
    bool contains(float x, float y) const { return x >= x0 && x <= x1 && y >= y0 && y <= y1; }
    float cx() const { return 0.5f * (x0 + x1); }
    float cy() const { return 0.5f * (y0 + y1); }
};

struct TouchLayout {
    float width = 0.0f, height = 0.0f; // window, points
    std::array<TouchRect, kTouchButtonCount> buttons{};
    TouchRect steerZone;               // where a finger starts the steering stick (outside every button)
    float stickRadius = 60.0f;         // horizontal travel for full lock, points
    float restX = 0.0f, restY = 0.0f;  // where the stick is drawn while no finger steers
    float topRowBottom = 0.0f;         // lower edge of the top button row (the minimap goes below it)
    float topRowLeft = 0.0f;           // left edge of the top button row (the compass stays left of it)
};

// Insets: the window's safe area (notch / Dynamic Island, home indicator) in points.
TouchLayout touchLayout(float width, float height, float insetLeft = 0.0f, float insetTop = 0.0f, float insetRight = 0.0f,
                        float insetBottom = 0.0f);

class TouchControls {
public:
    void setLayout(const TouchLayout& layout) { m_layout = layout; }
    const TouchLayout& layout() const { return m_layout; }

    // Points in window coordinates. fingerDown returns the button the finger pressed (Count: none or the stick).
    TouchButton fingerDown(std::uint64_t id, float x, float y);
    void fingerMove(std::uint64_t id, float x, float y);
    void fingerUp(std::uint64_t id);
    void releaseAll();

    bool held(TouchButton b) const;
    bool steering() const;   // a finger holds the stick
    float steer() const;     // -1 (left) .. 1 (right), 0 without a steering finger
    // Stick centre and finger position for drawing (valid while steering()).
    float stickX() const;
    float stickY() const;
    float fingerX() const;
    float fingerY() const;
    int fingers() const;

private:
    static constexpr int kStick = -2, kNone = -1;
    struct Finger {
        bool down = false;
        std::uint64_t id = 0;
        int control = kNone; // a TouchButton, kStick or kNone (landed on nothing)
        float x0 = 0.0f, y0 = 0.0f, x = 0.0f, y = 0.0f;
    };
    Finger* find(std::uint64_t id);
    const Finger* stickFinger() const;

    TouchLayout m_layout;
    std::array<Finger, 10> m_fingers{};
};

} // namespace tf
