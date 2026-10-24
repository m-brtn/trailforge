#pragma once

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
    TogglePause,     // Escape / gamepad Start
    StepOnce,        // N (while paused: a single physics step)
    Quit,
    Count
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
    // Call after processing the frame's events.
    void endEvents(bool uiWantsKeyboard, bool uiWantsMouse);

    bool pressed(Action a) const { return m_pressed[static_cast<int>(a)]; }
    const DriverIntent& driverIntent() const { return m_intent; }

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
    SDL_Gamepad* m_gamepad = nullptr;
};

} // namespace tf
