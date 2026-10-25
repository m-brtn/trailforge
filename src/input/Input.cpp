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
    m_menu = MenuKeys{};
    m_menu.mouseDown = m_mouseDown;
    m_menu.mouseX = m_mouseX;
    m_menu.mouseY = m_mouseY;
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
        if (e.key.key == SDLK_ESCAPE) {
            // Escape always works: pause menu in gameplay, back in the menus.
            if (!e.key.repeat) m_menu.back = m_menu.pause = m_menu.any = true;
            break;
        }
        if (!uiWantsKeyboard) {
            // Menu navigation (held arrows repeat with the system key repeat).
            switch (e.key.key) {
            case SDLK_UP: case SDLK_W: m_menu.up = true; break;
            case SDLK_DOWN: case SDLK_S: m_menu.down = true; break;
            case SDLK_LEFT: case SDLK_A: m_menu.left = true; break;
            case SDLK_RIGHT: case SDLK_D: m_menu.right = true; break;
            case SDLK_RETURN: case SDLK_KP_ENTER: case SDLK_SPACE: if (!e.key.repeat) m_menu.accept = true; break;
            case SDLK_BACKSPACE:
                m_menu.backspace = true;
                if (!e.key.repeat) m_menu.back = true;
                break;
            default: break;
            }
            if (!e.key.repeat) m_menu.any = true;
        }
        if (e.key.repeat) break;
        // F1-F3 always work; everything else only if the UI does not own the keyboard.
        if (e.key.key == SDLK_F1) { press(Action::ToggleDebugUI); break; }
        if (e.key.key == SDLK_F2) { press(Action::ToggleGraphicsUI); break; }
        if (e.key.key == SDLK_F3) { press(Action::ToggleAudioUI); break; }
        if (uiWantsKeyboard) break;
        switch (e.key.key) {
        case SDLK_Q: press(Action::ToggleDirection); break;
        case SDLK_L: press(Action::ToggleLowRange); break;
        case SDLK_R: press(Action::Reset); break;
        case SDLK_C: press(Action::ToggleCamera); break;
        case SDLK_N: press(Action::StepOnce); break;
        case SDLK_P: press(Action::TogglePause); break;
        case SDLK_H: press(Action::ToggleHeadlights); break;
        case SDLK_B: press(Action::ToggleAbs); break;
        case SDLK_T: press(Action::CycleTraction); break;
        case SDLK_X: press(Action::CycleDiffLock); break;
        case SDLK_V: press(Action::CycleTransfer); break;
        default: break;
        }
        break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        if (!uiWantsMouse && e.button.button == SDL_BUTTON_RIGHT) m_rmb = true;
        if (!uiWantsMouse) {
            m_menu.any = true;
            if (e.button.button == SDL_BUTTON_LEFT) {
                m_menu.click = true;
                m_menu.mouseDown = m_mouseDown = true;
                m_menu.mouseX = m_mouseX = e.button.x;
                m_menu.mouseY = m_mouseY = e.button.y;
            }
        }
        break;
    case SDL_EVENT_MOUSE_BUTTON_UP:
        if (e.button.button == SDL_BUTTON_RIGHT) m_rmb = false;
        if (e.button.button == SDL_BUTTON_LEFT) m_menu.mouseDown = m_mouseDown = false;
        break;
    case SDL_EVENT_MOUSE_MOTION:
        if (m_rmb && !uiWantsMouse) m_mouseDelta += glm::vec2(e.motion.xrel, e.motion.yrel);
        m_menu.mouseX = m_mouseX = e.motion.x;
        m_menu.mouseY = m_mouseY = e.motion.y;
        m_menu.mouseMoved = !uiWantsMouse;
        break;
    case SDL_EVENT_TEXT_INPUT:
        if (e.text.text) m_menu.text += e.text.text;
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
        m_menu.any = true;
        switch (e.gbutton.button) {
        case SDL_GAMEPAD_BUTTON_SOUTH: m_menu.accept = true; break;
        case SDL_GAMEPAD_BUTTON_EAST: m_menu.back = true; break;
        case SDL_GAMEPAD_BUTTON_START: m_menu.pause = true; break;
        default: break;
        }
        switch (e.gbutton.button) {
        case SDL_GAMEPAD_BUTTON_WEST: press(Action::ToggleDirection); break;
        case SDL_GAMEPAD_BUTTON_NORTH: press(Action::ToggleLowRange); break;
        case SDL_GAMEPAD_BUTTON_BACK: press(Action::Reset); break;
        case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER: press(Action::ToggleCamera); break;
        case SDL_GAMEPAD_BUTTON_DPAD_UP: press(Action::ToggleAbs); break;
        case SDL_GAMEPAD_BUTTON_DPAD_RIGHT: press(Action::CycleTraction); break;
        case SDL_GAMEPAD_BUTTON_DPAD_DOWN: press(Action::CycleDiffLock); break;
        case SDL_GAMEPAD_BUTTON_DPAD_LEFT: press(Action::CycleTransfer); break;
        default: break;
        }
        break;
    default: break;
    }
}

void Input::endEvents(bool uiWantsKeyboard, bool /*uiWantsMouse*/, float dt) {
    // Gamepad menu directions: D-pad or left stick, first step on the press, then repeat after a delay.
    std::array<bool, 4> held{};
    if (m_gamepad) {
        const float sx = axisValue(m_gamepad, SDL_GAMEPAD_AXIS_LEFTX, 0.5f);
        const float sy = axisValue(m_gamepad, SDL_GAMEPAD_AXIS_LEFTY, 0.5f);
        held[0] = SDL_GetGamepadButton(m_gamepad, SDL_GAMEPAD_BUTTON_DPAD_UP) || sy < 0.0f;
        held[1] = SDL_GetGamepadButton(m_gamepad, SDL_GAMEPAD_BUTTON_DPAD_DOWN) || sy > 0.0f;
        held[2] = SDL_GetGamepadButton(m_gamepad, SDL_GAMEPAD_BUTTON_DPAD_LEFT) || sx < 0.0f;
        held[3] = SDL_GetGamepadButton(m_gamepad, SDL_GAMEPAD_BUTTON_DPAD_RIGHT) || sx > 0.0f;
    }
    bool* dirs[4] = {&m_menu.up, &m_menu.down, &m_menu.left, &m_menu.right};
    for (std::size_t i = 0; i < 4; ++i) {
        if (held[i] && !m_padHeld[i]) {
            *dirs[i] = true;
            m_padRepeat[i] = 0.4f;
        } else if (held[i]) {
            m_padRepeat[i] -= dt;
            if (m_padRepeat[i] <= 0.0f) {
                *dirs[i] = true;
                m_padRepeat[i] = 0.12f;
            }
        }
        m_padHeld[i] = held[i];
    }
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
