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

// Held keys are positional (scancodes); discrete actions follow the key cap (keycodes), as before the tables.
SDL_Scancode scancode(Key k) {
    switch (k) {
    case Key::Escape: return SDL_SCANCODE_ESCAPE;
    case Key::F1: return SDL_SCANCODE_F1;
    case Key::F2: return SDL_SCANCODE_F2;
    case Key::F3: return SDL_SCANCODE_F3;
    case Key::Backspace: return SDL_SCANCODE_BACKSPACE;
    case Key::PageUp: return SDL_SCANCODE_PAGEUP;
    case Key::PageDown: return SDL_SCANCODE_PAGEDOWN;
    case Key::Q: return SDL_SCANCODE_Q;
    case Key::W: return SDL_SCANCODE_W;
    case Key::R: return SDL_SCANCODE_R;
    case Key::T: return SDL_SCANCODE_T;
    case Key::P: return SDL_SCANCODE_P;
    case Key::A: return SDL_SCANCODE_A;
    case Key::S: return SDL_SCANCODE_S;
    case Key::D: return SDL_SCANCODE_D;
    case Key::H: return SDL_SCANCODE_H;
    case Key::L: return SDL_SCANCODE_L;
    case Key::Return: return SDL_SCANCODE_RETURN;
    case Key::LeftShift: return SDL_SCANCODE_LSHIFT;
    case Key::X: return SDL_SCANCODE_X;
    case Key::C: return SDL_SCANCODE_C;
    case Key::V: return SDL_SCANCODE_V;
    case Key::B: return SDL_SCANCODE_B;
    case Key::N: return SDL_SCANCODE_N;
    case Key::M: return SDL_SCANCODE_M;
    case Key::Up: return SDL_SCANCODE_UP;
    case Key::Space: return SDL_SCANCODE_SPACE;
    case Key::Left: return SDL_SCANCODE_LEFT;
    case Key::Down: return SDL_SCANCODE_DOWN;
    case Key::Right: return SDL_SCANCODE_RIGHT;
    case Key::LeftCtrl: return SDL_SCANCODE_LCTRL;
    case Key::LeftAlt: return SDL_SCANCODE_LALT;
    case Key::LeftGui: return SDL_SCANCODE_LGUI;
    case Key::Count: break;
    }
    return SDL_SCANCODE_UNKNOWN;
}

SDL_Keycode keycode(Key k) {
    switch (k) {
    case Key::Q: return SDLK_Q;
    case Key::W: return SDLK_W;
    case Key::R: return SDLK_R;
    case Key::T: return SDLK_T;
    case Key::P: return SDLK_P;
    case Key::A: return SDLK_A;
    case Key::S: return SDLK_S;
    case Key::D: return SDLK_D;
    case Key::H: return SDLK_H;
    case Key::L: return SDLK_L;
    case Key::X: return SDLK_X;
    case Key::C: return SDLK_C;
    case Key::V: return SDLK_V;
    case Key::B: return SDLK_B;
    case Key::N: return SDLK_N;
    case Key::M: return SDLK_M;
    case Key::F1: return SDLK_F1;
    case Key::F2: return SDLK_F2;
    case Key::F3: return SDLK_F3;
    default: return SDL_SCANCODE_TO_KEYCODE(scancode(k));
    }
}

SDL_GamepadButton padButton(PadControl c) {
    switch (c) {
    case PadControl::South: return SDL_GAMEPAD_BUTTON_SOUTH;
    case PadControl::East: return SDL_GAMEPAD_BUTTON_EAST;
    case PadControl::West: return SDL_GAMEPAD_BUTTON_WEST;
    case PadControl::North: return SDL_GAMEPAD_BUTTON_NORTH;
    case PadControl::LeftShoulder: return SDL_GAMEPAD_BUTTON_LEFT_SHOULDER;
    case PadControl::RightShoulder: return SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER;
    case PadControl::LeftStick: return SDL_GAMEPAD_BUTTON_LEFT_STICK;
    case PadControl::RightStick: return SDL_GAMEPAD_BUTTON_RIGHT_STICK;
    case PadControl::DpadUp: return SDL_GAMEPAD_BUTTON_DPAD_UP;
    case PadControl::DpadDown: return SDL_GAMEPAD_BUTTON_DPAD_DOWN;
    case PadControl::DpadLeft: return SDL_GAMEPAD_BUTTON_DPAD_LEFT;
    case PadControl::DpadRight: return SDL_GAMEPAD_BUTTON_DPAD_RIGHT;
    case PadControl::Back: return SDL_GAMEPAD_BUTTON_BACK;
    case PadControl::Start: return SDL_GAMEPAD_BUTTON_START;
    case PadControl::Guide: return SDL_GAMEPAD_BUTTON_GUIDE;
    default: return SDL_GAMEPAD_BUTTON_INVALID; // triggers are axes
    }
}

static_assert(int(PadType::Unknown) == SDL_GAMEPAD_TYPE_UNKNOWN && int(PadType::XboxOne) == SDL_GAMEPAD_TYPE_XBOXONE &&
                  int(PadType::PS3) == SDL_GAMEPAD_TYPE_PS3 && int(PadType::PS5) == SDL_GAMEPAD_TYPE_PS5 &&
                  int(PadType::Steam) == SDL_GAMEPAD_TYPE_STEAM && int(PadType::Steam) + 1 == SDL_GAMEPAD_TYPE_COUNT,
              "PadType must mirror SDL_GamepadType");

} // namespace

Input::~Input() {
    if (m_gamepad) SDL_CloseGamepad(m_gamepad);
}

const char* Input::gamepadName() const {
    return m_gamepad ? SDL_GetGamepadName(m_gamepad) : "";
}

SDL_JoystickID Input::gamepadId() const { return m_gamepad ? SDL_GetGamepadID(m_gamepad) : 0; }

PadType Input::gamepadType() const { return m_gamepad ? PadType(SDL_GetGamepadType(m_gamepad)) : PadType::None; }

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
        if (m_modal) {
            switch (e.key.key) {
            case SDLK_ESCAPE: if (!e.key.repeat) m_menu.back = true; break;
            case SDLK_RETURN: case SDLK_KP_ENTER: if (!e.key.repeat) m_menu.accept = true; break;
            case SDLK_SPACE: if (!e.key.repeat && !m_modalText) m_menu.accept = true; break;
            case SDLK_BACKSPACE: m_menu.backspace = true; break;
            case SDLK_UP: m_menu.up = true; break;
            case SDLK_DOWN: m_menu.down = true; break;
            case SDLK_LEFT: m_menu.left = true; break;
            case SDLK_RIGHT: m_menu.right = true; break;
            default: break;
            }
            break;
        }
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
        for (const KeyAction& a : keyActions()) {
            if (e.key.key != keycode(a.key)) continue;
            const bool panel = a.action == Action::ToggleDebugUI || a.action == Action::ToggleGraphicsUI ||
                               a.action == Action::ToggleAudioUI;
            if (panel || !uiWantsKeyboard) press(a.action);
            break;
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
        if (!uiWantsMouse) {
            m_wheel += e.wheel.y;
            m_menu.wheel += e.wheel.y; // already in the user's scroll direction (natural scrolling included)
            m_menu.mouseX = m_mouseX = e.wheel.mouse_x;
            m_menu.mouseY = m_mouseY = e.wheel.mouse_y;
        }
        break;
    case SDL_EVENT_GAMEPAD_ADDED:
        // The most recently connected pad drives (picking up a second pad just works).
        if (SDL_Gamepad* pad = SDL_OpenGamepad(e.gdevice.which)) {
            if (m_gamepad && m_gamepad != pad) SDL_CloseGamepad(m_gamepad);
            m_gamepad = pad;
        }
        break;
    case SDL_EVENT_GAMEPAD_REMOVED:
        if (m_gamepad && SDL_GetGamepadID(m_gamepad) == e.gdevice.which) {
            SDL_CloseGamepad(m_gamepad);
            m_gamepad = nullptr;
            // Fall back to another connected pad.
            int n = 0;
            if (SDL_JoystickID* ids = SDL_GetGamepads(&n)) {
                for (int i = n - 1; i >= 0 && !m_gamepad; --i)
                    if (ids[i] != e.gdevice.which) m_gamepad = SDL_OpenGamepad(ids[i]);
                SDL_free(ids);
            }
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
        if (m_modal) {
            m_menu.pause = false;
            break;
        }
        for (const PadAction& a : padActions()) {
            if (e.gbutton.button != padButton(a.control)) continue;
            press(a.action);
            break;
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
    m_rightStick = glm::vec2(0.0f);
    m_fast = false;
    const auto keyHeld = [this](Key k) { return m_held[scancode(k)] ? 1.0f : 0.0f; };
    const DriveKeyMap drive = driveKeyMap(m_driveKeys), cam = freeCameraKeyMap(m_driveKeys);
    m_keyPedals.update(!uiWantsKeyboard && keyHeld(drive.forward) > 0.0f, !uiWantsKeyboard && keyHeld(drive.back) > 0.0f, dt);
    if (!uiWantsKeyboard) {
        in.throttle = m_keyPedals.throttle();
        in.brake = m_keyPedals.brake();
        in.steer = keyHeld(drive.right) - keyHeld(drive.left);
        in.handBrake = keyHeld(Key::Space);
        // Free camera: the cluster that does not drive, PageUp/PageDown, Shift for faster.
        m_freeMove.z = keyHeld(cam.forward) - keyHeld(cam.back);
        m_freeMove.x = keyHeld(cam.right) - keyHeld(cam.left);
        m_freeMove.y = keyHeld(Key::PageUp) - keyHeld(Key::PageDown);
        m_fast = m_held[SDL_SCANCODE_LSHIFT] || m_held[SDL_SCANCODE_RSHIFT];
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
        const StickValue rs = rightStickValue(float(SDL_GetGamepadAxis(m_gamepad, SDL_GAMEPAD_AXIS_RIGHTX)) / 32767.0f,
                                              float(SDL_GetGamepadAxis(m_gamepad, SDL_GAMEPAD_AXIS_RIGHTY)) / 32767.0f);
        m_rightStick = glm::vec2(rs.x, rs.y);
    }
    in.throttle = std::max(in.throttle, m_scriptThrottle);
    in.brake = std::max(in.brake, m_scriptBrake);
    m_intent = in;
}

} // namespace tf
