#pragma once

#include <string>

#include "sim/Simulation.h"

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_gamepad.h>
#include <glm/glm.hpp>

#include <array>

namespace tf {

// Discrete user actions (triggered on press).
enum class Action {
    ToggleDirection, // Q / gamepad X
    ToggleLowRange,  // L / gamepad Y
    Reset,           // R / gamepad Back
    ToggleCamera,    // C / gamepad RB
    ToggleDebugUI,   // F1
    ToggleGraphicsUI, // F2
    ToggleAudioUI,    // F3
    ToggleHeadlights, // H
    ToggleAbs,        // B / gamepad D-pad up
    CycleTraction,    // T / gamepad D-pad right
    CycleDiffLock,    // X / gamepad D-pad down
    CycleTransfer,    // V / gamepad D-pad left: part-time transfer case 2H -> 4H -> 4L
    TogglePause,     // P: debug freeze without the menu (Escape / gamepad Start open the pause menu, see MenuKeys)
    StepOnce,        // N (while frozen: a single physics step)
    Quit,
    Count
};

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
    float mouseX = 0.0f, mouseY = 0.0f; // window coordinates (the same as ImGui's)
};

// Keyboard, mouse and gamepad state. When ImGui captures the keyboard/mouse,
// the corresponding commands are not passed to the vehicle and the camera.
class Input {
public:
    Input() = default;
    ~Input();
    Input(const Input&) = delete;
    Input& operator=(const Input&) = delete;

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
    bool fastModifier() const { return m_fast; }

    bool gamepadConnected() const { return m_gamepad != nullptr; }
    const char* gamepadName() const;

private:
    void press(Action a) { m_pressed[static_cast<int>(a)] = true; }

    std::array<bool, static_cast<int>(Action::Count)> m_pressed{};
    // Held keys come from KEY_DOWN/KEY_UP events (not SDL_GetKeyboardState) so that
    // scenario events (SDL_PushEvent) take the same path. On focus loss SDL sends KEY_UP itself.
    std::array<bool, SDL_SCANCODE_COUNT> m_held{};
    DriverIntent m_intent;
    glm::vec2 m_mouseDelta{0.0f};
    float m_wheel = 0.0f;
    glm::vec3 m_freeMove{0.0f};
    bool m_fast = false;
    bool m_rmb = false;
    bool m_mouseDown = false;
    SDL_Gamepad* m_gamepad = nullptr;
    MenuKeys m_menu;
    float m_mouseX = 0.0f, m_mouseY = 0.0f;
    // Held menu directions from the gamepad (D-pad or stick): 0 up, 1 down, 2 left, 3 right, with repeat timers.
    std::array<bool, 4> m_padHeld{};
    std::array<float, 4> m_padRepeat{};
};

} // namespace tf
