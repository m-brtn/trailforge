#include "input/Input.h"

#include <SDL3/SDL_keyboard.h>
#include <SDL3/SDL_mouse.h>

#include <algorithm>
#include <cmath>

namespace tf {
namespace {

float axisValue(SDL_Gamepad* pad, SDL_GamepadAxis axis, float deadzone) {
    const float v = float(SDL_GetGamepadAxis(pad, axis)) / 32767.0f;
    const float a = std::abs(v);
    if (a < deadzone) return 0.0f;
    return std::copysign(std::min(1.0f, (a - deadzone) / (1.0f - deadzone)), v);
}

} // namespace

Input::~Input() {
    if (m_gamepad) SDL_CloseGamepad(m_gamepad);
}

const char* Input::gamepadName() const {
    return m_gamepad ? SDL_GetGamepadName(m_gamepad) : "";
}

void Input::beginFrame() {
    m_pressed.fill(false);
    m_mouseDelta = glm::vec2(0.0f);
    m_wheel = 0.0f;
}

void Input::handleEvent(const SDL_Event& e, bool uiWantsKeyboard, bool uiWantsMouse) {
    switch (e.type) {
    case SDL_EVENT_QUIT: press(Action::Quit); break;
    case SDL_EVENT_KEY_UP:
        if (e.key.scancode < SDL_SCANCODE_COUNT) m_held[e.key.scancode] = false;
        break;
    case SDL_EVENT_KEY_DOWN:
        // Held state is always tracked (even when the UI captures input) so a release is not lost;
        // it is applied in endEvents only when the UI does not own the keyboard.
        if (e.key.scancode < SDL_SCANCODE_COUNT) m_held[e.key.scancode] = true;
        if (e.key.repeat) break;
        // F1-F3 and Escape always work; everything else only if the UI does not own the keyboard.
        if (e.key.key == SDLK_F1) { press(Action::ToggleDebugUI); break; }
        if (e.key.key == SDLK_F2) { press(Action::ToggleGraphicsUI); break; }
        if (e.key.key == SDLK_F3) { press(Action::ToggleAudioUI); break; }
        if (e.key.key == SDLK_ESCAPE) { press(Action::TogglePause); break; }
        if (uiWantsKeyboard) break;
        switch (e.key.key) {
        case SDLK_Q: press(Action::ToggleDirection); break;
        case SDLK_L: press(Action::ToggleLowRange); break;
        case SDLK_R: press(Action::Reset); break;
        case SDLK_C: press(Action::ToggleCamera); break;
        case SDLK_N: press(Action::StepOnce); break;
        case SDLK_H: press(Action::ToggleHeadlights); break;
        default: break;
        }
        break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        if (!uiWantsMouse && e.button.button == SDL_BUTTON_RIGHT) m_rmb = true;
        break;
    case SDL_EVENT_MOUSE_BUTTON_UP:
        if (e.button.button == SDL_BUTTON_RIGHT) m_rmb = false;
        break;
    case SDL_EVENT_MOUSE_MOTION:
        if (m_rmb && !uiWantsMouse) m_mouseDelta += glm::vec2(e.motion.xrel, e.motion.yrel);
        break;
    case SDL_EVENT_MOUSE_WHEEL:
        if (!uiWantsMouse) m_wheel += e.wheel.y;
        break;
    case SDL_EVENT_GAMEPAD_ADDED:
        if (!m_gamepad) m_gamepad = SDL_OpenGamepad(e.gdevice.which);
        break;
    case SDL_EVENT_GAMEPAD_REMOVED:
        if (m_gamepad && SDL_GetGamepadID(m_gamepad) == e.gdevice.which) {
            SDL_CloseGamepad(m_gamepad);
            m_gamepad = nullptr;
        }
        break;
    case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
        if (!m_gamepad || e.gbutton.which != SDL_GetGamepadID(m_gamepad)) break;
        switch (e.gbutton.button) {
        case SDL_GAMEPAD_BUTTON_WEST: press(Action::ToggleDirection); break;
        case SDL_GAMEPAD_BUTTON_NORTH: press(Action::ToggleLowRange); break;
        case SDL_GAMEPAD_BUTTON_BACK: press(Action::Reset); break;
        case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER: press(Action::ToggleCamera); break;
        case SDL_GAMEPAD_BUTTON_START: press(Action::TogglePause); break;
        default: break;
        }
        break;
    default: break;
    }
}

void Input::endEvents(bool uiWantsKeyboard, bool /*uiWantsMouse*/) {
    DriverIntent in;
    m_freeMove = glm::vec3(0.0f);
    m_fast = false;
    if (!uiWantsKeyboard) {
        const auto& k = m_held;
        in.throttle = k[SDL_SCANCODE_W] ? 1.0f : 0.0f;
        in.brake = k[SDL_SCANCODE_S] ? 1.0f : 0.0f;
        in.steer = (k[SDL_SCANCODE_D] ? 1.0f : 0.0f) - (k[SDL_SCANCODE_A] ? 1.0f : 0.0f);
        in.handBrake = k[SDL_SCANCODE_SPACE] ? 1.0f : 0.0f;
        // Free camera: arrow keys, PageUp/PageDown, Shift for faster.
        m_freeMove.z = (k[SDL_SCANCODE_UP] ? 1.0f : 0.0f) - (k[SDL_SCANCODE_DOWN] ? 1.0f : 0.0f);
        m_freeMove.x = (k[SDL_SCANCODE_RIGHT] ? 1.0f : 0.0f) - (k[SDL_SCANCODE_LEFT] ? 1.0f : 0.0f);
        m_freeMove.y = (k[SDL_SCANCODE_PAGEUP] ? 1.0f : 0.0f) - (k[SDL_SCANCODE_PAGEDOWN] ? 1.0f : 0.0f);
        m_fast = k[SDL_SCANCODE_LSHIFT] || k[SDL_SCANCODE_RSHIFT];
    }
    if (m_gamepad) {
        // Gamepad: RT throttle, LT brake, left stick steering, A handbrake. Take the maximum with the keyboard.
        const float rt = std::max(0.0f, axisValue(m_gamepad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER, 0.05f));
        const float lt = std::max(0.0f, axisValue(m_gamepad, SDL_GAMEPAD_AXIS_LEFT_TRIGGER, 0.05f));
        const float sx = axisValue(m_gamepad, SDL_GAMEPAD_AXIS_LEFTX, 0.12f);
        in.throttle = std::max(in.throttle, rt);
        in.brake = std::max(in.brake, lt);
        if (std::abs(sx) > std::abs(in.steer)) {
            in.steer = sx;
            in.analog = true;
        }
        if (SDL_GetGamepadButton(m_gamepad, SDL_GAMEPAD_BUTTON_SOUTH)) in.handBrake = 1.0f;
    }
    m_intent = in;
}

} // namespace tf
