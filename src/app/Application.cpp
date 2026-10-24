#include "app/Application.h"

#include "render/MathConvert.h"

#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlgpu3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

namespace tf {

// ------------------------------------------------------------------ RAII helpers

SdlContext::SdlContext() {
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD))
        throw std::runtime_error(std::string("SDL_Init: ") + SDL_GetError());
}

SdlContext::~SdlContext() { SDL_Quit(); }

Window::Window(const char* title, int w, int h) {
    const float scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
    const float s = scale > 0.0f ? scale : 1.0f;
    m_window = SDL_CreateWindow(title, int(w * s), int(h * s), SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (!m_window) throw std::runtime_error(std::string("SDL_CreateWindow: ") + SDL_GetError());
    SDL_SetWindowPosition(m_window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
}

Window::~Window() { SDL_DestroyWindow(m_window); }

ImGuiContextGuard::ImGuiContextGuard(SDL_Window* window, const Renderer& renderer) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr; // do not write imgui.ini to the working directory
    ImGui::StyleColorsDark();
    const float scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
    if (scale > 0.0f) {
        ImGui::GetStyle().ScaleAllSizes(scale);
        ImGui::GetStyle().FontScaleDpi = scale;
    }
    ImGui_ImplSDL3_InitForSDLGPU(window);
    ImGui_ImplSDLGPU3_InitInfo info{};
    info.Device = renderer.device();
    info.ColorTargetFormat = renderer.swapchainFormat();
    info.MSAASamples = SDL_GPU_SAMPLECOUNT_1;
    ImGui_ImplSDLGPU3_Init(&info);
}

ImGuiContextGuard::~ImGuiContextGuard() {
    ImGui_ImplSDL3_Shutdown();
    ImGui_ImplSDLGPU3_Shutdown();
    ImGui::DestroyContext();
}

// ------------------------------------------------------------------ Application

Application::Application(const AppOptions& options)
    : m_options(options),
      m_window("TrailForge", 1440, 900),
      m_sim(SimulationSettings{}),
      m_ui(m_sim.vehicleConfig()),
      m_clock(m_sim.fixedStep()) {
#if defined(NDEBUG)
    const bool debugDevice = false;
#else
    const bool debugDevice = true;
#endif
    m_renderer = std::make_unique<Renderer>(m_window.get(), debugDevice);
    m_imgui = std::make_unique<ImGuiContextGuard>(m_window.get(), *m_renderer);
    m_ui.visible = options.startWithUi;

    m_trackMesh = m_renderer->upload(buildTrackMesh(m_sim.track().geometry(), m_sim.surfaces()));
    m_bodyMesh = m_renderer->upload(buildVehicleBodyMesh());
    m_wheelMesh = m_renderer->upload(buildWheelMesh(m_sim.vehicleConfig().wheelRadius, m_sim.vehicleConfig().wheelWidth));
    std::printf("TrailForge: GPU driver '%s', shaders: %s, track triangles: %zu\n", m_renderer->driverName(),
                m_renderer->shaderFormatName(), m_sim.track().geometry().triangleCount());
    if (!options.scriptPath.empty()) {
        m_scenario = Scenario::load(options.scriptPath);
        if (!m_scenario) throw std::runtime_error("cannot load scenario " + options.scriptPath);
    }
}

Application::~Application() {
    // Mesh GPU resources are released before the device is destroyed.
    if (m_renderer) SDL_WaitForGPUIdle(m_renderer->device());
    m_trackMesh.release();
    m_bodyMesh.release();
    m_wheelMesh.release();
    m_imgui.reset();
    m_renderer.reset();
}

void Application::processEvents() {
    m_input.beginFrame();
    if (m_scenario) updateScenario();
    const ImGuiIO& io = ImGui::GetIO();
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        ImGui_ImplSDL3_ProcessEvent(&e);
        if (e.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && e.window.windowID == SDL_GetWindowID(m_window.get()))
            m_running = false;
        m_input.handleEvent(e, io.WantCaptureKeyboard, io.WantCaptureMouse);
    }
    m_input.endEvents(io.WantCaptureKeyboard, io.WantCaptureMouse);

    if (m_input.pressed(Action::Quit)) m_running = false;
    if (m_input.pressed(Action::ToggleDebugUI)) m_ui.visible = !m_ui.visible;
    if (m_input.pressed(Action::TogglePause)) m_paused = !m_paused;
    if (m_input.pressed(Action::StepOnce) && m_paused) m_stepOnce = true;
    if (m_input.pressed(Action::ToggleCamera)) m_camera.toggleMode();
    if (m_input.pressed(Action::ToggleDirection)) m_sim.requestToggleDirection();
    if (m_input.pressed(Action::ToggleLowRange)) m_sim.requestToggleLowRange();
    if (m_input.pressed(Action::Reset)) m_sim.requestReset();

    DriverIntent intent = m_input.driverIntent();
    if (m_options.demoDrive) {
        // Slow circular drive through the start pad and the lane A bumps.
        const double phase = std::fmod(m_time, 24.0);
        intent.throttle = phase < 20.0 ? 0.35f : 0.0f;
        intent.brake = phase >= 20.0 ? 0.5f : 0.0f;
        intent.steer = m_time < 3.0 ? 0.0f : -0.45f;
    }
    m_sim.setIntent(intent);
}

void Application::stepSimulation(double frameTime) {
    if (m_clock.step() != m_sim.fixedStep()) m_clock.setStep(m_sim.fixedStep());
    m_stepsThisFrame = 0;
    if (m_paused) {
        m_clock.reset();
        if (m_stepOnce) {
            m_sim.tick();
            m_stepsThisFrame = 1;
            m_stepOnce = false;
        }
        return;
    }
    // Input is already passed to Simulation; each tick applies it with smoothing over the fixed dt.
    const int steps = m_clock.advance(frameTime);
    for (int i = 0; i < steps; ++i) m_sim.tick();
    m_stepsThisFrame = steps;
}

void Application::renderFrame(float frameTime) {
    // ---- Pose interpolation between the last two physics ticks
    const float alpha = m_paused ? 1.0f : float(m_clock.alpha());
    const RenderSnapshot& a = m_sim.previousSnapshot();
    const RenderSnapshot& b = m_sim.currentSnapshot();
    auto interp = [&](const Pose& p0, const Pose& p1) {
        const glm::vec3 pos = glm::mix(toGlm(p0.position), toGlm(p1.position), alpha);
        const glm::quat rot = glm::slerp(toGlm(p0.rotation), toGlm(p1.rotation), alpha);
        return poseMatrix(pos, rot);
    };
    const glm::mat4 bodyM = interp(a.body, b.body);

    // ---- Camera (read-only access to the world)
    CameraInput ci;
    ci.mouseDelta = m_input.mouseDelta();
    ci.wheel = m_input.wheel();
    ci.move = m_input.freeCameraMove();
    ci.fast = m_input.fastModifier();
    m_camera.update(frameTime, ci, glm::vec3(bodyM[3]), glm::vec3(bodyM[2]),
                    [this](const glm::vec3& from, const glm::vec3& to) {
                        return m_sim.castRayIgnoringVehicle(JPH::Vec3(from.x, from.y, from.z), JPH::Vec3(to.x, to.y, to.z));
                    });
    if (m_scenario) {
        // Camera height above the surface below it (downward ray; negative means the camera is underground).
        const glm::vec3 cp = m_camera.position();
        const JPH::Vec3 above(cp.x, cp.y + 30.0f, cp.z), below(cp.x, cp.y - 5.0f, cp.z);
        if (auto hit = m_sim.castRayIgnoringVehicle(above, below)) {
            m_camClearance = cp.y - (above.GetY() + (below.GetY() - above.GetY()) * *hit);
            if (m_camera.mode() == Camera::Mode::Follow) m_minCamClearance = std::min(m_minCamClearance, m_camClearance);
        }
        int air = 0;
        for (const auto& w : m_sim.vehicle().telemetry().wheels) air += w.contact ? 0 : 1;
        m_maxAirWheels = std::max(m_maxAirWheels, air);
    }

    // ---- UI
    ImGui_ImplSDLGPU3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
    FrameStats stats;
    stats.fps = m_fps;
    stats.frameMs = m_frameMs;
    stats.stepsThisFrame = m_stepsThisFrame;
    stats.droppedLastMs = m_clock.lastDropped() * 1000.0;
    stats.droppedTotalMs = m_clock.totalDropped() * 1000.0;
    stats.overloaded = m_clock.overloaded();
    stats.paused = m_paused;
    stats.followCamera = m_camera.mode() == Camera::Mode::Follow;
    stats.gpuDriver = m_renderer->driverName();
    stats.shaderFormat = m_renderer->shaderFormatName();
    stats.gamepad = m_input.gamepadConnected() ? m_input.gamepadName() : "";
    const UiRequests req = m_ui.draw(m_sim, stats);
    ImGui::Render();
    if (req.togglePause) m_paused = !m_paused;
    if (req.stepOnce && m_paused) m_stepOnce = true;
    if (req.setVSync) m_renderer->setVSync(req.vsync);

    // ---- Scene
    DrawItem draws[2 + kWheelCount];
    draws[0] = {&m_trackMesh, glm::mat4(1.0f), glm::vec4(1.0f), 0.18f};
    draws[1] = {&m_bodyMesh, bodyM, glm::vec4(1.0f), 0.0f};
    for (int i = 0; i < kWheelCount; ++i)
        draws[2 + i] = {&m_wheelMesh, interp(a.wheels[i], b.wheels[i]), glm::vec4(1.0f), 0.0f};

    m_debugDraw.clear();
    if (m_ui.drawPhysicsDebug) {
        const VehicleConfig& vc = m_sim.vehicleConfig();
        drawVehicleDebug(m_debugDraw, m_sim.vehicle().telemetry(), vc.suspensionMinLength, vc.suspensionMaxLength,
                         m_ui.debug);
        if (m_ui.debug.bodyAxes) m_debugDraw.axes(bodyM, 1.5f);
    }

    int pw = 1, ph = 1;
    SDL_GetWindowSizeInPixels(m_window.get(), &pw, &ph);
    FrameDesc frame;
    frame.view = m_camera.view();
    frame.proj = m_camera.projection(float(pw) / float(ph > 0 ? ph : 1));
    frame.cameraPos = m_camera.position();
    frame.draws = draws;
    frame.lines = m_debugDraw.lines();
    frame.linesOnTop = m_ui.linesOnTop;
    frame.ui = ImGui::GetDrawData();
    if (!m_options.capturePath.empty() && !m_captured && m_time >= m_options.captureAtSeconds) {
        frame.capturePath = m_options.capturePath.c_str();
        m_captured = true;
    }
    if (!m_captureRequest.empty()) frame.capturePath = m_captureRequest.c_str();
    m_renderer->drawFrame(frame);
    m_captureRequest.clear();
}

// ------------------------------------------------------------------ verification scenario

void Application::queueEvent(const SDL_Event& e, int framesFromNow) {
    m_queuedEvents.push_back({framesFromNow, e});
}

void Application::queueClick(float x, float y, bool textInput) {
    const SDL_WindowID win = SDL_GetWindowID(m_window.get());
    SDL_Event motion{};
    motion.type = SDL_EVENT_MOUSE_MOTION;
    motion.motion.windowID = win;
    motion.motion.x = x;
    motion.motion.y = y;
    SDL_Event button{};
    button.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
    button.button.windowID = win;
    button.button.button = SDL_BUTTON_LEFT;
    button.button.down = true;
    button.button.clicks = 1;
    button.button.x = x;
    button.button.y = y;
    SDL_Event up = button;
    up.type = SDL_EVENT_MOUSE_BUTTON_UP;
    up.button.down = false;
    SDL_Event modDown{};
    modDown.type = SDL_EVENT_KEY_DOWN;
    modDown.key.windowID = win;
#if defined(__APPLE__)
    modDown.key.scancode = SDL_SCANCODE_LGUI;
    modDown.key.key = SDLK_LGUI;
    modDown.key.mod = SDL_KMOD_LGUI;
#else
    modDown.key.scancode = SDL_SCANCODE_LCTRL;
    modDown.key.key = SDLK_LCTRL;
    modDown.key.mod = SDL_KMOD_LCTRL;
#endif
    modDown.key.down = true;
    SDL_Event modUp = modDown;
    modUp.type = SDL_EVENT_KEY_UP;
    modUp.key.mod = SDL_KMOD_NONE;
    modUp.key.down = false;
    // Typing a number into an ImGui slider: Ctrl+click, on macOS Cmd+click (Ctrl+click there emulates the right button).
    // Mouse position goes in a separate frame before the press, like a real mouse.
    if (textInput) queueEvent(modDown, 0);
    queueEvent(motion, 0);
    queueEvent(motion, 1);
    queueEvent(button, 1);
    queueEvent(up, 3);
    if (textInput) queueEvent(modUp, 4);
}

void Application::updateScenario() {
    m_pushedThisFrame = false;
    for (auto it = m_queuedEvents.begin(); it != m_queuedEvents.end();) {
        if (it->frames-- <= 0) {
            SDL_Event e = it->event;
            if (e.type == SDL_EVENT_KEY_DOWN || e.type == SDL_EVENT_KEY_UP) e.key.timestamp = SDL_GetTicksNS();
            else e.common.timestamp = SDL_GetTicksNS();
            SDL_PushEvent(&e);
            m_pushedThisFrame = true;
            it = m_queuedEvents.erase(it);
        } else {
            ++it;
        }
    }
    const bool running = m_scenario->update(
        m_time, [this] { return m_pushedThisFrame || !m_queuedEvents.empty(); },
        [this](const std::string& n) { return scenarioValue(n); },
        [this](const ScenarioCommand& c) { return execScenario(c); });
    if (!running) {
        std::printf("SCENARIO RESULT: %d passed, %d failed (%s)\n", m_scenario->passed(), m_scenario->failed(),
                    m_options.scriptPath.c_str());
        m_exitCode = m_scenario->failed() > 0 ? 1 : 0;
        m_scenario.reset();
        m_running = false;
    }
}

Scenario::Exec Application::execScenario(const ScenarioCommand& c) {
    using Exec = Scenario::Exec;
    const SDL_WindowID win = SDL_GetWindowID(m_window.get());
    const std::string arg = c.args.empty() ? std::string() : c.args[0];
    if (c.op == "down" || c.op == "up" || c.op == "tap") {
        const SDL_Scancode sc = SDL_GetScancodeFromName(arg.c_str());
        if (sc == SDL_SCANCODE_UNKNOWN) return Exec::Failed;
        SDL_Event e{};
        e.type = c.op == "up" ? SDL_EVENT_KEY_UP : SDL_EVENT_KEY_DOWN;
        e.key.windowID = win;
        e.key.scancode = sc;
        e.key.key = SDL_GetKeyFromScancode(sc, SDL_KMOD_NONE, true);
        e.key.down = c.op != "up";
        queueEvent(e, 0);
        if (c.op == "tap") {
            e.type = SDL_EVENT_KEY_UP;
            e.key.down = false;
            queueEvent(e, 1);
        }
        return Exec::Done;
    }
    if (c.op == "text") {
        m_textStorage.push_back(arg);
        SDL_Event e{};
        e.type = SDL_EVENT_TEXT_INPUT;
        e.text.windowID = win;
        e.text.text = m_textStorage.back().c_str();
        queueEvent(e, 0);
        return Exec::Done;
    }
    if (c.op == "click" || c.op == "inputclick") {
        const UiItemRect* r = m_ui.item(arg);
        if (!r || !r->visible) {
            std::printf("SCENARIO: UI item '%s' is not visible\n", arg.c_str());
            return Exec::Failed;
        }
        std::printf("SCENARIO %s %s at (%.0f, %.0f), ImGui mouse (%.0f, %.0f), window %dx%d\n", c.op.c_str(), arg.c_str(),
                    0.5f * (r->x0 + r->x1), 0.5f * (r->y0 + r->y1), ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y,
                    int(ImGui::GetIO().DisplaySize.x), int(ImGui::GetIO().DisplaySize.y));
        queueClick(0.5f * (r->x0 + r->x1), 0.5f * (r->y0 + r->y1), c.op == "inputclick");
        return Exec::Done;
    }
    if (c.op == "scrollto") {
        const UiItemRect* r = m_ui.item(arg);
        const UiItemRect* w = m_ui.item("tuning");
        if (!w) return Exec::Failed;
        if (++m_scrollFrames > 240) {
            m_scrollFrames = 0;
            m_scrollStableFrames = 0;
            return Exec::Failed;
        }
        if (r && r->visible && r->y1 <= w->y1 - 4.0f && r->y0 >= w->y0) {
            // ImGui may apply a queued wheel step a frame or two later; click only once the item stops moving.
            m_scrollStableFrames = (m_scrollStableFrames > 0 && r->y0 == m_scrollLastY) ? m_scrollStableFrames + 1 : 1;
            m_scrollLastY = r->y0;
            if (m_scrollStableFrames < 4) return Exec::Busy;
            m_scrollFrames = 0;
            m_scrollStableFrames = 0;
            return Exec::Done;
        }
        m_scrollStableFrames = 0;
        const float x = 0.5f * (w->x0 + w->x1), y = 0.5f * (w->y0 + w->y1);
        SDL_Event motion{};
        motion.type = SDL_EVENT_MOUSE_MOTION;
        motion.motion.windowID = win;
        motion.motion.x = x;
        motion.motion.y = y;
        SDL_Event wheel{};
        wheel.type = SDL_EVENT_MOUSE_WHEEL;
        wheel.wheel.windowID = win;
        wheel.wheel.y = (r && r->y0 < w->y0) ? 1.0f : -1.0f;
        wheel.wheel.integer_y = int(wheel.wheel.y);
        wheel.wheel.mouse_x = x;
        wheel.wheel.mouse_y = y;
        queueEvent(motion, 0);
        queueEvent(wheel, 0);
        return Exec::Busy;
    }
    if (c.op == "orbit" || c.op == "wheel") {
        int ww = 0, wh = 0;
        SDL_GetWindowSize(m_window.get(), &ww, &wh);
        const float x = 0.5f * ww, y = 0.5f * wh;
        SDL_Event motion{};
        motion.type = SDL_EVENT_MOUSE_MOTION;
        motion.motion.windowID = win;
        motion.motion.x = x;
        motion.motion.y = y;
        queueEvent(motion, 0);
        if (c.op == "wheel") {
            SDL_Event wheel{};
            wheel.type = SDL_EVENT_MOUSE_WHEEL;
            wheel.wheel.windowID = win;
            wheel.wheel.y = float(std::atof(arg.c_str()));
            wheel.wheel.mouse_x = x;
            wheel.wheel.mouse_y = y;
            queueEvent(wheel, 1);
            return Exec::Done;
        }
        if (c.args.size() < 2) return Exec::Failed;
        SDL_Event down{};
        down.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
        down.button.windowID = win;
        down.button.button = SDL_BUTTON_RIGHT;
        down.button.down = true;
        down.button.x = x;
        down.button.y = y;
        SDL_Event drag = motion;
        drag.motion.xrel = float(std::atof(c.args[0].c_str()));
        drag.motion.yrel = float(std::atof(c.args[1].c_str()));
        drag.motion.x = x + drag.motion.xrel;
        drag.motion.y = y + drag.motion.yrel;
        SDL_Event up = down;
        up.type = SDL_EVENT_MOUSE_BUTTON_UP;
        up.button.down = false;
        queueEvent(down, 1);
        queueEvent(drag, 2);
        queueEvent(up, 3);
        return Exec::Done;
    }
    if (c.op == "teleport") {
        m_sim.requestTeleport(std::atoi(arg.c_str()));
        return Exec::Done;
    }
    if (c.op == "resize") {
        if (c.args.size() < 2) return Exec::Failed;
        return SDL_SetWindowSize(m_window.get(), std::atoi(c.args[0].c_str()), std::atoi(c.args[1].c_str()))
                   ? Exec::Done
                   : Exec::Failed;
    }
    if (c.op == "capture") {
        m_captureRequest = arg;
        return Exec::Done;
    }
    if (c.op == "reset_stats") {
        m_minCamClearance = 1e9f;
        m_maxAirWheels = 0;
        return Exec::Done;
    }
    if (c.op == "log") {
        const auto& t = m_sim.vehicle().telemetry();
        const auto p = m_sim.currentSnapshot().body.position;
        std::printf("SCENARIO log t=%.2f [%s] pos=(%.1f %.2f %.1f) v=%.1f km/h fwd=%+.1f gear=%d %s%s thr=%.2f brk=%.2f "
                    "hb=%.2f steer=%+.2f tilt=%.1f cam=%.2f | ",
                    m_time, arg.c_str(), p.GetX(), p.GetY(), p.GetZ(), t.speed * 3.6f, t.forwardSpeed * 3.6f, t.gear,
                    t.direction == DriveDirection::Forward ? "D" : "R", t.lowRange ? " LOW" : "", t.command.throttle,
                    t.command.brake, t.command.handBrake, t.command.steer,
                    std::acos(std::fmin(1.0f, t.upDot)) * 57.2958f, m_camClearance);
        for (int i = 0; i < kWheelCount; ++i) {
            const auto& w = t.wheels[i];
            std::printf("%s %s %5.0fN %3.0f%%%s mu%.2f  ", kWheelNames[i],
                        w.contact ? m_sim.surfaces().get(w.surface).name : "AIR", w.load, 100.0f * w.compressionRatio,
                        w.bumpStop ? "!" : "", w.combinedLongFriction);
        }
        std::printf("msg='%s'\n", m_sim.lastMessage());
        return Exec::Done;
    }
    return Exec::Failed;
}

std::optional<double> Application::scenarioValue(const std::string& n) const {
    const VehicleTelemetry& t = m_sim.vehicle().telemetry();
    const auto p = m_sim.currentSnapshot().body.position;
    if (n.rfind("msg:", 0) == 0) return std::strstr(m_sim.lastMessage(), n.c_str() + 4) ? 1.0 : 0.0;
    if (n == "time") return m_time;
    if (n == "speed_kmh") return t.speed * 3.6;
    if (n == "fwd_kmh") return t.forwardSpeed * 3.6;
    if (n == "throttle") return t.command.throttle;
    if (n == "brake") return t.command.brake;
    if (n == "handbrake") return t.command.handBrake;
    if (n == "steer") return t.command.steer;
    if (n == "steer_deg") return t.steerAngleDeg;
    if (n == "gear") return t.gear;
    if (n == "rpm") return t.engineRPM;
    if (n == "dir") return t.direction == DriveDirection::Forward ? 1.0 : -1.0;
    if (n == "low") return t.lowRange ? 1.0 : 0.0;
    if (n == "tilt") return std::acos(std::fmin(1.0f, t.upDot)) * 57.2958;
    if (n == "x") return p.GetX();
    if (n == "y") return p.GetY();
    if (n == "z") return p.GetZ();
    if (n == "ticks") return double(m_sim.timing().ticks);
    if (n == "paused") return m_paused ? 1.0 : 0.0;
    if (n == "cam_free") return m_camera.mode() == Camera::Mode::Free ? 1.0 : 0.0;
    if (n == "ui_visible") return m_ui.visible ? 1.0 : 0.0;
    if (n == "ui_kb") return ImGui::GetIO().WantCaptureKeyboard ? 1.0 : 0.0;
    if (n == "ui_text") return ImGui::GetIO().WantTextInput ? 1.0 : 0.0;
    if (n == "cam_clearance") return m_camClearance;
    if (n == "min_cam_clearance") return m_minCamClearance;
    if (n == "max_air") return m_maxAirWheels;
    if (n == "front_spring") return m_sim.vehicleConfig().tuning.frontSpringFrequency;
    if (n == "mass") return m_sim.vehicleConfig().mass;
    if (n == "depth_w") return m_renderer->depthWidth();
    if (n == "depth_h") return m_renderer->depthHeight();
    if (n == "depth_creations") return m_renderer->depthCreations();
    if (n == "depth_matches_window") {
        int w = 0, h = 0;
        SDL_GetWindowSizeInPixels(m_window.get(), &w, &h);
        return m_renderer->depthWidth() == std::uint32_t(w) && m_renderer->depthHeight() == std::uint32_t(h) ? 1.0 : 0.0;
    }
    if (n == "win_w" || n == "win_h") {
        int w = 0, h = 0;
        SDL_GetWindowSize(m_window.get(), &w, &h);
        return n == "win_w" ? w : h;
    }
    if (n == "px_w" || n == "px_h") {
        int w = 0, h = 0;
        SDL_GetWindowSizeInPixels(m_window.get(), &w, &h);
        return n == "px_w" ? w : h;
    }
    if (n == "air" || n == "lowgrip_wheels" || n == "max_slip" || n == "load_sum") {
        double air = 0, low = 0, slip = 0, load = 0;
        for (const auto& w : t.wheels) {
            air += w.contact ? 0 : 1;
            low += w.contact && w.surface == SurfaceId::LowGrip ? 1 : 0;
            if (w.contact && w.slipValid) slip = std::max(slip, double(w.longitudinalSlip));
            load += w.load;
        }
        return n == "air" ? air : n == "lowgrip_wheels" ? low : n == "max_slip" ? slip : load;
    }
    return std::nullopt;
}

int Application::run() {
    const double freq = double(SDL_GetPerformanceFrequency());
    Uint64 last = SDL_GetPerformanceCounter();
    while (m_running) {
        const Uint64 now = SDL_GetPerformanceCounter();
        const double frameTime = double(now - last) / freq;
        last = now;
        m_time += frameTime;

        m_fpsAccum += float(frameTime);
        ++m_fpsFrames;
        if (m_fpsAccum >= 0.5f) {
            m_fps = float(m_fpsFrames) / m_fpsAccum;
            m_frameMs = 1000.0f * m_fpsAccum / float(m_fpsFrames);
            m_fpsAccum = 0.0f;
            m_fpsFrames = 0;
        }

        processEvents();
        if (SDL_GetWindowFlags(m_window.get()) & SDL_WINDOW_MINIMIZED) {
            m_clock.reset();
            SDL_Delay(16);
            continue;
        }
        stepSimulation(frameTime);
        renderFrame(float(std::min(frameTime, 0.1)));

        if (m_options.quitAfterSeconds > 0.0 && m_time >= m_options.quitAfterSeconds) {
            const auto p = m_sim.currentSnapshot().body.position;
            std::printf("TrailForge: auto-quit after %.1f s, %llu physics ticks, body at (%.2f %.2f %.2f), "
                        "%.0f FPS, dropped %.1f ms\n",
                        m_time, m_sim.timing().ticks, p.GetX(), p.GetY(), p.GetZ(), m_fps,
                        m_clock.totalDropped() * 1000.0);
            m_running = false;
        }
    }
    return m_exitCode;
}

} // namespace tf
