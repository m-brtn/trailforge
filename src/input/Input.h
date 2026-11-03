#pragma once

#include <string>

#include "input/ControlLayout.h"
#include "input/KeyboardPedals.h"
#include "input/TouchControls.h"
#include "sim/Simulation.h"

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_gamepad.h>
#include <glm/glm.hpp>

#include <array>

namespace tf {

// Menu navigation of one frame, from every device. Gameplay ignores it except `pause`.
struct MenuKeys {
    bool up = false, down = false, left = false, right = false; // with key / D-pad / stick auto-repeat
    bool accept = false;  // Enter, Space, gamepad A
    bool back = false;    // Escape, Backspace, gamepad B
    bool pause = false;   // Escape, gamepad Start
    bool any = false;     // any key, mouse button or gamepad button
    bool backspace = false; // Backspace alone (also sets back; text fields use it to delete)
    std::string text;       // SDL text input of this frame (only while text input is started)
    bool mouseDown = false; // left button held (slider drags)
    bool mouseMoved = false;
    bool click = false;   // left mouse button pressed (not over an ImGui window)
    bool release = false; // left mouse button released after a press that reached the game
    float wheel = 0.0f;   // mouse wheel / trackpad scroll this frame (not over an ImGui window), + = up
    float mouseX = 0.0f, mouseY = 0.0f; // window coordinates (the same as ImGui's)
    // Finger (SDL's mouse events synthesized from touch): click and release still report the press, but the menus
    // act on `tap` (lifted without moving past kTouchSlop) so that a drag never activates the row it started on.
    bool touch = false;    // the press of this frame (held or just released) is a finger
    bool tap = false;      // a finger lifted this frame without dragging
    bool dragging = false; // a finger is held and has moved past kTouchSlop
    float dragY = 0.0f;    // finger movement of this frame while dragging (points, + = down)
};

constexpr float kTouchSlop = 10.0f; // points a finger may move and still tap

// Keyboard, mouse and gamepad state. When ImGui captures the keyboard/mouse,
// the corresponding commands are not passed to the vehicle and the camera.
class Input {
public:
    Input() = default;
    ~Input();
    Input(const Input&) = delete;
    Input& operator=(const Input&) = delete;

    // A modal dialog owns the keyboard and gamepad: only Enter, Escape (back, never pause), Backspace and the arrow
    // keys / D-pad reach the menus; no hotkeys, no F1-F3, no WASD navigation. textEntry: Space types (SDL text
    // input) instead of accepting.
    void setModalCapture(bool on, bool textEntry) {
        m_modal = on;
        m_modalText = on && textEntry;
    }
    void beginFrame();
    void handleEvent(const SDL_Event& e, bool uiWantsKeyboard, bool uiWantsMouse);
    // Call after processing the frame's events. dt drives the menu auto-repeat of held directions.
    void endEvents(bool uiWantsKeyboard, bool uiWantsMouse, float dt = 0.0f);

    bool pressed(Action a) const { return m_pressed[static_cast<int>(a)]; }
    const DriverIntent& driverIntent() const { return m_intent; }
    const MenuKeys& menuKeys() const { return m_menu; }

    glm::vec2 mouseDelta() const { return m_mouseDelta; }
    float wheel() const { return m_wheel; }
    glm::vec3 freeCameraMove() const { return m_freeMove; }
    // Gamepad right stick after the dead zone (camera orbit / hood look-around), -1..1, x right, y down.
    glm::vec2 rightStick() const { return m_rightStick; }
    // Left stick with the same dead zone and response (map panning), and the triggers (x LT, y RT) 0..1.
    glm::vec2 leftStick() const { return m_leftStick; }
    glm::vec2 triggers() const { return m_triggers; }
    // A gamepad button pressed this frame (the map's own buttons; padActions still fire as usual).
    bool padPressed(PadControl c) const { return m_padPressed[static_cast<std::size_t>(c)]; }
    bool scancodeHeld(SDL_Scancode s) const { return m_held[s]; }
    bool fastModifier() const { return m_fast; }

    bool gamepadConnected() const { return m_gamepad != nullptr; }
    SDL_JoystickID gamepadId() const; // 0 without a pad
    // Scenario `pedal throttle|brake V`: an analog pedal held like a gamepad trigger (max with keyboard / gamepad).
    void setScriptPedals(float throttle, float brake) {
        m_scriptThrottle = throttle;
        m_scriptBrake = brake;
    }
    float scriptThrottle() const { return m_scriptThrottle; }
    float scriptBrake() const { return m_scriptBrake; }
    const char* gamepadName() const;
    PadType gamepadType() const; // PadType::None without a pad

    // On-screen touch driving (TouchControls, layout in window points). Active: SDL finger events drive and pause, and
    // SDL's touch-synthesized mouse events are ignored; inactive: fingers reach the menus only as that mouse.
    void setTouchControls(bool active, const TouchLayout& layout);
    bool touchControlsActive() const { return m_touchActive; }
    const TouchControls& touchControls() const { return m_touch; }
    float touchThrottle() const { return m_touchPedals.throttle(); }
    float touchBrake() const { return m_touchPedals.brake(); }

    // Driving cluster (W A S D or the arrows); the other cluster moves the free camera. Both ramp (KeyboardPedals).
    void setDriveKeys(DriveKeys layout) { m_driveKeys = layout; }
    DriveKeys driveKeys() const { return m_driveKeys; }

private:
    void press(Action a) { m_pressed[static_cast<int>(a)] = true; }

    std::array<bool, static_cast<int>(Action::Count)> m_pressed{};
    // Held keys come from KEY_DOWN/KEY_UP events (not SDL_GetKeyboardState) so that
    // scenario events (SDL_PushEvent) take the same path. On focus loss SDL sends KEY_UP itself.
    std::array<bool, SDL_SCANCODE_COUNT> m_held{};
    KeyboardPedals m_keyPedals; // forward / backward drive keys: progressive pedals (KeyboardPedals.h)
    DriveKeys m_driveKeys = DriveKeys::Wasd;
    float m_scriptThrottle = 0.0f, m_scriptBrake = 0.0f;
    DriverIntent m_intent;
    glm::vec2 m_mouseDelta{0.0f};
    float m_wheel = 0.0f;
    glm::vec3 m_freeMove{0.0f};
    glm::vec2 m_rightStick{0.0f};
    glm::vec2 m_leftStick{0.0f}, m_triggers{0.0f};
    std::array<bool, static_cast<std::size_t>(PadControl::Count)> m_padPressed{};
    bool m_fast = false;
    bool m_rmb = false;
    bool m_modal = false, m_modalText = false;
    bool m_mouseDown = false;
    bool m_touchDown = false, m_touchDrag = false;
    float m_touchX0 = 0.0f, m_touchY0 = 0.0f;
    TouchControls m_touch;
    KeyboardPedals m_touchPedals; // the on-screen pedals ramp like the keyboard's
    bool m_touchActive = false;
    SDL_Gamepad* m_gamepad = nullptr;
    MenuKeys m_menu;
    float m_mouseX = 0.0f, m_mouseY = 0.0f;
    // Held menu directions from the gamepad (D-pad or stick): 0 up, 1 down, 2 left, 3 right, with repeat timers.
    std::array<bool, 4> m_padHeld{};
    std::array<float, 4> m_padRepeat{};
};

} // namespace tf
