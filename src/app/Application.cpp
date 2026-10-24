#include "app/Application.h"

#include "app/AppIcon.h"

#include "render/GroundMaps.h"
#include "render/MathConvert.h"
#include "render/Scenery.h"
#include "render/TimeOfDay.h"
#include "render/VehicleModel.h"

#include <SDL3/SDL.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
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
    if (setAppIcon(m_window)) SDL_Log("App icon set");
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
      m_window("TrailForge " TRAILFORGE_VERSION, 1440, 900),
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

    const Uint64 t0 = SDL_GetPerformanceCounter();
    const MeshData trackMesh = buildTrackMesh(m_sim.track().geometry(), m_sim.surfaces());
    const MeshData sceneryMesh = buildSceneryMesh();
    const MeshData bodyMesh = buildVehicleBodyMesh();
    m_trackMesh = m_renderer->upload(trackMesh);
    m_sceneryMesh = m_renderer->upload(sceneryMesh);
    m_waterMesh = m_renderer->upload(buildWaterMesh());
    m_bodyMesh = m_renderer->upload(bodyMesh);
    m_springMesh = m_renderer->upload(buildUnitSpringMesh());
    m_armMesh = m_renderer->upload(buildUnitTubeMesh(0.021f));
    m_halfShaftMesh = m_renderer->upload(buildUnitTubeMesh(0.028f));
    m_shockBodyMesh = m_renderer->upload(buildShockBodyMesh());
    m_shockShaftMesh = m_renderer->upload(buildShockShaftMesh());
    m_renderer->setGround(buildGroundMaps(m_sim.track()), TestTrack::water().level);
    const std::vector<TreeInstance> trees = buildTreeInstances();
    m_renderer->setTrees(buildPineMeshes(), trees);
    m_gfx.settings = m_renderer->settings();
    std::printf("TrailForge: GPU driver '%s', shaders: %s, track triangles: %zu, render triangles: track %zu, "
                "scenery %zu, vehicle body %zu, trees %zu (instanced), scene build %.0f ms\n",
                m_renderer->driverName(), m_renderer->shaderFormatName(), m_sim.track().geometry().triangleCount(),
                trackMesh.vertices.size() / 3, sceneryMesh.vertices.size() / 3, bodyMesh.vertices.size() / 3,
                trees.size(), 1000.0 * double(SDL_GetPerformanceCounter() - t0) / double(SDL_GetPerformanceFrequency()));
    if (!options.scriptPath.empty()) {
        m_scenario = Scenario::load(options.scriptPath);
        if (!m_scenario) throw std::runtime_error("cannot load scenario " + options.scriptPath);
    }

    AudioOptions audio;
    audio.device = options.audio;
    audio.capturePath = options.audioCapturePath;
    // SDL_GetBasePath(): Contents/Resources/ inside TrailForge.app, the executable's directory otherwise.
    if (const char* base = SDL_GetBasePath()) {
        audio.assetDirs.push_back(std::string(base) + "assets/audio");
#if !defined(TRAILFORGE_PACKAGED)
        audio.assetDirs.push_back(std::string(base) + "../../assets/audio"); // build/<preset>/trailforge
#endif
    }
#if !defined(TRAILFORGE_PACKAGED)
    audio.assetDirs.push_back(std::string(TRAILFORGE_SOURCE_DIR) + "/assets/audio");
#endif
    m_audio = std::make_unique<AudioSystem>(audio);
}

Application::~Application() {
    m_audio.reset(); // stop the audio thread first
    // Mesh GPU resources are released before the device is destroyed.
    if (m_renderer) SDL_WaitForGPUIdle(m_renderer->device());
    for (GpuMesh* m : {&m_trackMesh, &m_sceneryMesh, &m_waterMesh, &m_bodyMesh, &m_wheelMesh, &m_springMesh,
                       &m_armMesh, &m_halfShaftMesh, &m_shockBodyMesh, &m_shockShaftMesh})
        m->release();
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
        if (m_scenario && e.type == SDL_EVENT_KEY_DOWN && !e.key.repeat && io.WantCaptureKeyboard)
            std::printf("SCENARIO: key %s arrived while ImGui owns the keyboard (only F1-F3 and Escape act)\n",
                        SDL_GetScancodeName(e.key.scancode));
        m_input.handleEvent(e, io.WantCaptureKeyboard, io.WantCaptureMouse);
    }
    m_input.endEvents(io.WantCaptureKeyboard, io.WantCaptureMouse);

    if (m_input.pressed(Action::Quit)) m_running = false;
    if (m_input.pressed(Action::ToggleDebugUI)) m_ui.visible = !m_ui.visible;
    if (m_input.pressed(Action::ToggleGraphicsUI)) m_gfx.visible = !m_gfx.visible;
    if (m_input.pressed(Action::ToggleAudioUI)) m_audioPanel.visible = !m_audioPanel.visible;
    if (m_input.pressed(Action::ToggleHeadlights)) m_gfx.headlights = !m_gfx.headlights;
    if (m_input.pressed(Action::TogglePause)) setPaused(!m_paused);
    if (m_input.pressed(Action::StepOnce) && m_paused) m_stepOnce = true;
    if (m_input.pressed(Action::ToggleCamera)) m_camera.toggleMode();
    if (m_input.pressed(Action::ToggleDirection)) m_sim.requestToggleDirection();
    if (m_input.pressed(Action::ToggleLowRange)) m_sim.requestToggleLowRange();
    if (m_input.pressed(Action::Reset)) m_sim.requestReset();
    if (m_input.pressed(Action::ToggleAbs)) m_sim.requestToggleAbs();
    if (m_input.pressed(Action::CycleTraction)) m_sim.requestCycleTraction();
    if (m_input.pressed(Action::CycleDiffLock)) m_sim.requestCycleDiffLock();

    DriverIntent intent = m_input.driverIntent();
    intent.autoReverse = m_ui.autoReverse;
    if (m_options.demoDrive) {
        intent.autoReverse = false;
        // Slow circular drive through the start pad and the lane A bumps.
        const double phase = std::fmod(m_time, 24.0);
        intent.throttle = phase < 20.0 ? 0.35f : 0.0f;
        intent.brake = phase >= 20.0 ? 0.5f : 0.0f;
        intent.steer = m_time < 3.0 ? 0.0f : -0.45f;
    }
    m_sim.setIntent(intent);
}

void Application::setPaused(bool paused) {
    if (paused == m_paused) return;
    m_paused = paused;
    // Gameplay sounds fade out and freeze while paused; the menu open/close sounds mark the toggle.
    m_audio->playUi(paused ? UiSound::MenuOpen : UiSound::MenuClose);
}

void Application::publishAudio() {
    AudioFrame f = m_audioFeed.frame();
    const glm::mat4 v = m_camera.view();
    const glm::vec3 pos = m_camera.position();
    f.listenerPos = {pos.x, pos.y, pos.z};
    f.listenerRight = {v[0][0], v[1][0], v[2][0]};
    f.listenerForward = {-v[0][2], -v[1][2], -v[2][2]};
    const SkyState sky = computeSky(m_gfx.settings.timeOfDay, m_gfx.settings.sunHeadingDeg);
    f.dayAmount = std::clamp((sky.sunElevationDeg + 6.0f) / 12.0f, 0.0f, 1.0f);
    f.paused = m_paused;
    f.settings = m_audioPanel.settings;
    m_audio->publish(f, m_audioFeed.events());
    m_audioFeed.clearEvents();
}

void Application::stepSimulation(double frameTime) {
    if (m_clock.step() != m_sim.fixedStep()) m_clock.setStep(m_sim.fixedStep());
    m_stepsThisFrame = 0;
    const bool capture = m_audio->capturing();
    auto tick = [&] {
        m_sim.tick();
        m_audioFeed.onTick(m_sim);
        if (capture) {
            // Offline audio follows the physics clock: one tick of sound per physics tick.
            publishAudio();
            m_audio->captureAdvance(m_sim.fixedStep());
        }
    };
    if (m_paused) {
        m_clock.reset();
        if (m_stepOnce) {
            tick();
            m_stepsThisFrame = 1;
            m_stepOnce = false;
        } else if (capture) {
            publishAudio();
            m_audio->captureAdvance(frameTime);
        }
        return;
    }
    // Input is already passed to Simulation; each tick applies it with smoothing over the fixed dt.
    const int steps = m_clock.advance(frameTime);
    for (int i = 0; i < steps; ++i) tick();
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
        const VehicleTelemetry& t = m_sim.vehicle().telemetry();
        int locked = 0;
        for (const auto& w : t.wheels)
            locked += w.contact && std::abs(w.groundSpeed) > 2.0f && std::abs(w.rollingSpeed) < 0.2f * std::abs(w.groundSpeed);
        m_maxLockedWheels = std::max(m_maxLockedWheels, locked);
        m_lockedWheelTime += locked * frameTime;
        m_absSeen |= t.absActive;
        m_tcSeen |= t.tcActive;
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
    const RenderStats& rs = m_renderer->stats();
    stats.cpuFrameMs = m_cpuMs;
    stats.gpuFrameMs = rs.gpuMs;
    stats.renderW = rs.renderW;
    stats.renderH = rs.renderH;
    stats.outputW = rs.outputW;
    stats.outputH = rs.outputH;
    stats.drawCalls = rs.drawCalls;
    stats.grassBlades = rs.grassBlades;
    stats.reflection = rs.reflectionRendered;
    stats.maxMsaa = m_renderer->maxMsaa();
    stats.markUploadBytes = rs.markUploadBytes;
    m_gfx.sprayDrawn = rs.sprayParticles;
    const UiRequests req = m_ui.draw(m_sim, stats, m_gfx);
    if (m_gfx.fillMarks) {
        fillTireMarks();
        m_gfx.fillMarks = false;
    }
    m_audioPanel.stats = m_audio->stats();
    m_audioPanel.status = m_audio->status().c_str();
    m_audioPanel.active = m_audio->active();
    m_ui.drawAudio(m_audioPanel);
    ImGui::Render();
    if (req.togglePause) setPaused(!m_paused);
    if (!m_audio->capturing()) publishAudio();
    if (req.stepOnce && m_paused) m_stepOnce = true;
    if (req.setVSync) m_renderer->setVSync(req.vsync);
    if (m_gfx.timeLapse) {
        // Cosmetic clock: runs in real time (also while the physics is paused), stopped with the UI pause button.
        m_gfx.settings.timeOfDay = std::fmod(m_gfx.settings.timeOfDay + m_gfx.timeLapseSpeed * frameTime, 24.0f);
        m_gfx.changed = true;
    }
    if (m_gfx.changed) {
        m_renderer->setSettings(m_gfx.settings);
        m_gfx.settings = m_renderer->settings();
        m_gfx.changed = false;
    }

    // ---- Scene
    std::array<glm::mat4, kWheelCount> wheelM;
    for (int i = 0; i < kWheelCount; ++i) wheelM[i] = interp(a.wheels[i], b.wheels[i]);
    updateCosmetics(frameTime);
    {
        WheelSpraySettings ss = m_spray.settings();
        const std::size_t cap = std::size_t(std::clamp(m_gfx.settings.sprayMax, 0, 8192));
        if (ss.maxParticles != cap || ss.dust != m_gfx.settings.sprayDust || ss.enabled != m_gfx.spray) {
            ss.maxParticles = cap;
            ss.dust = m_gfx.settings.sprayDust;
            ss.enabled = m_gfx.spray;
            m_spray.setSettings(ss);
        }
        if (!m_paused) {
            std::array<TreadSoil, kWheelCount> soil;
            for (std::size_t i = 0; i < kWheelCount; ++i) soil[i] = m_sim.tireMarks().soil(i);
            m_spray.update(m_sim.vehicle().contacts(), soil, std::min(frameTime, 0.05f));
        }
        m_gfx.sprayLive = m_spray.size();
    }
    buildDraws(bodyM, wheelM);

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
    frame.fovYRadians = glm::radians(m_camera.fovYDeg);
    frame.nearPlane = m_camera.nearPlane;
    frame.time = float(m_time);
    frame.draws = m_draws;
    frame.water = &m_waterMesh;
    frame.vehicleModel = bodyM;
    frame.vehicleMud = m_gfx.mud;
    {
        const VehicleTelemetry& t = m_sim.vehicle().telemetry();
        frame.brakeLight = std::max(t.command.brake, t.command.handBrake) > 0.05f ? 1.0f : 0.0f;
        frame.reverseLight = t.direction == DriveDirection::Reverse ? 1.0f : 0.0f;
        frame.headlights = m_gfx.headlights ? 1.0f : 0.0f;
        // Lamp lens centers in body space (see VehicleModel.cpp), beams pitched ~3 deg down.
        const glm::vec3 lens(0.67f, 0.40f, 2.24f);
        frame.headlightPos[0] = glm::vec3(bodyM * glm::vec4(lens.x, lens.y, lens.z, 1.0f));
        frame.headlightPos[1] = glm::vec3(bodyM * glm::vec4(-lens.x, lens.y, lens.z, 1.0f));
        frame.headlightDir = glm::normalize(glm::vec3(bodyM * glm::vec4(0.0f, -0.05f, 1.0f, 0.0f)));
        for (int i = 0; i < kWheelCount; ++i) {
            const auto& w = t.wheels[i];
            frame.wheelContacts[i] = glm::vec4(toGlm(w.contactPosition), w.contact ? 1.0f : 0.0f);
        }
    }
    frame.tireMarks = m_gfx.tireMarks ? &m_sim.tireMarks() : nullptr;
    frame.spray = m_gfx.spray ? &m_spray : nullptr;
    frame.simTime = m_sim.simTime();
    frame.markGain = m_gfx.markGain;
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

// Performance test: fills the whole ring with synthetic ribbons (random curves on the proving ground terrain,
// surface from the ground map). Not driven marks; they only load the renderer at the configured cap.
void Application::fillTireMarks() {
    TireMarks& marks = m_sim.tireMarks();
    const float t = float(m_sim.simTime());
    std::uint32_t seed = 12345u;
    auto rnd = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return float(seed >> 8) / float(1u << 24);
    };
    auto ground = [](float x, float z) { return glm::vec3(x, TestTrack::provingGroundHeight(x, z), z); };
    auto normalAt = [&](float x, float z) {
        const float e = 0.25f;
        const glm::vec3 dx = ground(x + e, z) - ground(x - e, z), dz = ground(x, z + e) - ground(x, z - e);
        return glm::normalize(glm::cross(dz, dx));
    };
    const float seg = marks.settings().segmentLength, halfWidth = 0.16f;
    std::size_t left = marks.capacity();
    while (left > 0) {
        glm::vec2 p(-44.0f + 88.0f * rnd(), -44.0f + 124.0f * rnd());
        float heading = 6.2832f * rnd(), u = 0.0f;
        glm::vec3 lastPos = ground(p.x, p.y), lastSide(0.0f);
        for (int i = 0; i < 400 && left > 0; ++i) {
            heading += (rnd() - 0.5f) * 0.12f;
            if (std::abs(p.x) > 45.0f || p.y < -45.0f || p.y > 81.0f)
                heading = std::atan2(-p.x, 18.0f - p.y); // back toward the middle at the walls
            p += glm::vec2(std::sin(heading), std::cos(heading)) * seg;
            const glm::vec3 pos = ground(p.x, p.y), n = normalAt(p.x, p.y);
            const glm::vec3 side = glm::normalize(glm::cross(n, pos - lastPos)) * halfWidth;
            if (i > 0) {
                const SurfaceId s = TestTrack::groundSurfaceAt(p.x, p.y);
                const bool soft = surfaceMarkResponse(s).imprint > 0.0f;
                TireMarkSegment m{};
                std::memcpy(m.p0, &lastPos, sizeof(m.p0));
                std::memcpy(m.p1, &pos, sizeof(m.p1));
                std::memcpy(m.side0, &lastSide, sizeof(m.side0));
                std::memcpy(m.side1, &side, sizeof(m.side1));
                m.u0 = u;
                m.u1 = u + seg;
                m.birth = t;
                m.surface = float(s);
                m.imprint0 = m.imprint1 = soft ? 0.7f : 0.0f;
                m.slip0 = m.slip1 = soft ? 0.1f : 0.6f;
                marks.push(m);
                --left;
                u += seg;
            }
            lastPos = pos;
            lastSide = side;
        }
    }
}

void Application::buildDraws(const glm::mat4& bodyM, const std::array<glm::mat4, kWheelCount>& wheelM) {
    const VehicleConfig& vc = m_sim.vehicleConfig();
    if (vc.wheelRadius != m_wheelMeshRadius || vc.wheelWidth != m_wheelMeshWidth) {
        SDL_WaitForGPUIdle(m_renderer->device());
        m_wheelMesh = m_renderer->upload(buildWheelMesh(vc.wheelRadius, vc.wheelWidth));
        m_wheelMeshRadius = vc.wheelRadius;
        m_wheelMeshWidth = vc.wheelWidth;
    }

    m_draws.clear();
    auto add = [&](const GpuMesh& mesh, const glm::mat4& model, bool castsShadow = true, bool inReflection = true) {
        DrawItem d;
        d.mesh = &mesh;
        d.model = model;
        d.castsShadow = castsShadow;
        d.inReflection = inReflection;
        m_draws.push_back(d);
    };
    add(m_trackMesh, glm::mat4(1.0f));
    add(m_sceneryMesh, glm::mat4(1.0f));
    add(m_bodyMesh, bodyM);
    add(m_wheelMesh, bodyM * spareWheelMatrix(vc.wheelRadius, vc.wheelWidth));
    m_draws.back().mud = 0.25f; // the spare on the roof stays mostly clean
    m_draws.back().wheelMud = true;

    const glm::mat4 invBody = glm::inverse(bodyM);
    for (int i = 0; i < kWheelCount; ++i) {
        // Wheel and suspension follow the Jolt wheel transform (interpolated between ticks).
        const glm::mat4 wheelBody = invBody * wheelM[i];
        const glm::vec3 hub(wheelBody[3]);
        const float side = hub.x >= 0.0f ? 1.0f : -1.0f;
        // The wheel mesh has its outer face at +X; Jolt uses one wheel-right axis for both sides.
        const bool flip = glm::dot(glm::vec3(wheelBody[0]), glm::vec3(side, 0, 0)) < 0.0f;
        add(m_wheelMesh, flip ? wheelM[i] * glm::rotate(glm::mat4(1.0f), glm::pi<float>(), glm::vec3(0, 1, 0))
                              : wheelM[i]);
        m_draws.back().wheelMud = true;
        const TreadSoil& soil = m_sim.tireMarks().soil(std::size_t(i));
        m_draws.back().soil = glm::vec4(soil.color[0], soil.color[1], soil.color[2], soil.amount);
        m_draws.back().soilWetness = soil.wetness;
        m_draws.back().wheelRadius = vc.wheelRadius;

        const SuspensionPose p = suspensionPose(hub, hub.z);
        const glm::vec3 dir = glm::normalize(p.shockTop - p.shockLow);
        add(m_armMesh, bodyM * segmentMatrix(p.armPivotA, p.armOuter), true, false);
        add(m_armMesh, bodyM * segmentMatrix(p.armPivotB, p.armOuter), true, false);
        add(m_halfShaftMesh, bodyM * segmentMatrix(p.shaftInner, p.shaftOuter), true, false);
        add(m_shockBodyMesh, bodyM * aimMatrix(p.shockLow, p.shockTop), true, false);
        add(m_shockShaftMesh, bodyM * aimMatrix(p.shockTop, p.shockLow), false, false);
        add(m_springMesh, bodyM * segmentMatrix(p.shockTop, p.shockLow + dir * kSpringPerch), true, false);
    }
}

void Application::updateCosmetics(float dt) {
    if (!m_gfx.autoMud || m_paused) return;
    // Cosmetic only: mud builds up while the wheels roll on the Mud surface or through the puddles
    // and slowly dries/falls off elsewhere. It never feeds back into the simulation.
    const VehicleTelemetry& t = m_sim.vehicle().telemetry();
    const WaterInfo water = TestTrack::water();
    float gain = 0.0f;
    for (const auto& w : t.wheels) {
        if (!w.contact) continue;
        const float roll = std::min(std::abs(w.angularVelocity) * m_sim.vehicleConfig().wheelRadius, 8.0f);
        const bool wet = w.contactPosition.GetY() < water.level - 0.02f && w.contactPosition.GetX() > water.x0 &&
                         w.contactPosition.GetX() < water.x1 && w.contactPosition.GetZ() > water.z0 &&
                         w.contactPosition.GetZ() < water.z1;
        if (w.surface == SurfaceId::Mud || wet) gain += roll * (wet ? 0.010f : 0.006f);
    }
    m_gfx.mud = std::clamp(m_gfx.mud + (gain - 0.002f) * dt, 0.0f, 1.0f);
}

void Application::endPerf() {
    if (!m_perfActive) return;
    m_perfActive = false;
    std::vector<double> v = m_perfFrameMs;
    if (v.empty()) return;
    std::sort(v.begin(), v.end());
    double sum = 0.0;
    for (double x : v) sum += x;
    const double avg = sum / double(v.size());
    const double p95 = v[std::min(v.size() - 1, std::size_t(double(v.size()) * 0.95))];
    const double p99 = v[std::min(v.size() - 1, std::size_t(double(v.size()) * 0.99))];
    const RenderStats& rs = m_renderer->stats();
    std::printf("PERF %-12s frames=%zu fps=%.1f frame_avg=%.2f ms p95=%.2f p99=%.2f max=%.2f cpu_avg=%.2f ms "
                "gpu_avg=%.2f ms render=%ux%u output=%ux%u vsync=%d draws=%d grass=%.0fk\n",
                m_perfLabel.c_str(), v.size(), 1000.0 / avg, avg, p95, p99, v.back(),
                m_perfCpuSum / double(v.size()), m_perfGpuCount ? m_perfGpuSum / m_perfGpuCount : -1.0, rs.renderW,
                rs.renderH, rs.outputW, rs.outputH, m_renderer->vsync() ? 1 : 0, rs.drawCalls,
                double(rs.grassBlades) / 1000.0);
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
    if (c.op == "quality") {
        for (int p = 0; p < int(QualityPreset::Custom); ++p) {
            if (arg == presetName(QualityPreset(p))) {
                const RenderSettings look = m_gfx.settings;
                m_gfx.settings = presetSettings(QualityPreset(p));
                copyLook(m_gfx.settings, look);
                m_gfx.preset = QualityPreset(p);
                m_gfx.changed = true;
                return Exec::Done;
            }
        }
        return Exec::Failed;
    }
    if (c.op == "set") {
        if (c.args.size() < 2) return Exec::Failed;
        const float v = float(std::atof(c.args[1].c_str()));
        RenderSettings& s = m_gfx.settings;
        const std::string& k = c.args[0];
        bool render = true;
        if (k == "render_scale") s.renderScale = v;
        else if (k == "msaa") s.msaa = int(v);
        else if (k == "fxaa") s.fxaa = v != 0.0f;
        else if (k == "cascades") s.shadowCascades = int(v);
        else if (k == "shadow_size") s.shadowMapSize = int(v);
        else if (k == "shadow_dist") s.shadowDistance = v;
        else if (k == "shadow_filter") s.shadowFilter = int(v);
        else if (k == "reflections") s.reflections = v != 0.0f;
        else if (k == "refl_scale") s.reflectionScale = v;
        else if (k == "bloom") s.bloom = v != 0.0f;
        else if (k == "grass") s.grassLevel = int(v);
        else if (k == "grass_dist") s.grassDistance = v;
        else if (k == "clouds") s.clouds = v != 0.0f;
        else if (k == "spray_max") s.sprayMax = int(v);
        else if (k == "spray_dust") s.sprayDust = v != 0.0f;
        else if (k == "exposure") s.exposure = v;
        else if (k == "fog") s.fogDensity = v;
        else {
            render = false;
            // Look-only settings: they do not turn the quality preset into Custom.
            if (k == "tod") { s.timeOfDay = v; m_gfx.changed = true; }
            else if (k == "sun_heading") { s.sunHeadingDeg = v; m_gfx.changed = true; }
            else if (k == "tod_speed") m_gfx.timeLapseSpeed = v;
            else if (k == "tod_run") m_gfx.timeLapse = v != 0.0f;
            else if (k == "mud") { m_gfx.mud = std::clamp(v, 0.0f, 1.0f); m_gfx.autoMud = false; }
            else if (k == "auto_mud") m_gfx.autoMud = v != 0.0f;
            else if (k == "headlights") m_gfx.headlights = v != 0.0f;
            else if (k == "debug_lines") m_ui.drawPhysicsDebug = v != 0.0f;
            else if (k == "ui") m_ui.visible = v != 0.0f;
            else if (k == "hud") m_ui.hud = v != 0.0f;
            else if (k == "auto_reverse") m_ui.autoReverse = v != 0.0f;
            else if (k == "abs" || k == "tc" || k == "diff_locks") {
                AidControls a = m_sim.aidControls();
                if (k == "abs") a.abs = v != 0.0f;
                else if (k == "tc") a.traction = TractionMode(std::clamp(int(v), 0, 2));
                else a.diffLocks = std::clamp(int(v), 0, AidControls::kDiffLockStages - 1);
                m_sim.requestAidControls(a);
            }
            else if (k == "graphics_ui") m_gfx.visible = v != 0.0f;
            else if (k == "vsync") m_renderer->setVSync(v != 0.0f);
            else if (k == "marks") m_gfx.tireMarks = v != 0.0f;
            else if (k == "audio_ui") m_audioPanel.visible = v != 0.0f;
            else if (k == "audio_mute") m_audioPanel.settings.mute = v != 0.0f;
            else if (k == "audio_master") m_audioPanel.settings.master = v;
            else if (k == "audio_pitch") m_audioPanel.settings.enginePitch = v;
            else if (k == "audio_whine") m_audioPanel.settings.whine = v;
            else if (k == "audio_play_ui") m_audio->playUi(UiSound(std::clamp(int(v), 0, int(UiSound::MenuClose))));
            else if (k == "marks_clear") m_sim.tireMarks().clear();
            else if (k == "spray") m_gfx.spray = v != 0.0f;
            else if (k == "spray_clear") m_spray.clear();
            else if (k == "spray_fill") m_spray.fill(m_sim.currentSnapshot().body.position);
            else if (k == "marks_fill") fillTireMarks();
            else if (k == "marks_gain") m_gfx.markGain.fill(v);
            else if (k == "marks_gen" || k == "marks_max" || k == "marks_fade") {
                TireMarkSettings ms = m_sim.tireMarks().settings();
                if (k == "marks_gen") ms.enabled = v != 0.0f;
                else if (k == "marks_max") ms.maxSegments = int(v);
                else ms.fadeTime = v;
                m_sim.tireMarks().setSettings(ms);
            }
            else return Exec::Failed;
        }
        if (render) {
            m_gfx.preset = QualityPreset::Custom;
            m_gfx.changed = true;
        }
        return Exec::Done;
    }
    if (c.op == "time") {
        // time Dawn|Noon|Sunset|Night or time HOURS (0..24); stops the time-lapse so captures are repeatable.
        float hours = -1.0f;
        for (const TimePreset& p : kTimePresets)
            if (arg == p.name) hours = p.hours;
        if (hours < 0.0f) {
            char* end = nullptr;
            hours = std::strtof(arg.c_str(), &end);
            if (arg.empty() || *end != '\0' || hours < 0.0f || hours > 24.0f) return Exec::Failed;
        }
        m_gfx.settings.timeOfDay = hours;
        m_gfx.timeLapse = false;
        m_gfx.changed = true;
        return Exec::Done;
    }
    if (c.op == "camera") {
        if (arg == "follow") {
            if (m_camera.mode() == Camera::Mode::Free) m_camera.toggleMode();
            return Exec::Done;
        }
        if (c.args.size() < 6) return Exec::Failed;
        float v[6];
        for (int i = 0; i < 6; ++i) v[i] = float(std::atof(c.args[std::size_t(i)].c_str()));
        m_camera.setFreePose({v[0], v[1], v[2]}, {v[3], v[4], v[5]});
        return Exec::Done;
    }
    if (c.op == "perf_begin") {
        m_perfActive = true;
        m_perfLabel = arg.empty() ? "perf" : arg;
        m_perfFrameMs.clear();
        m_perfCpuSum = m_perfGpuSum = 0.0;
        m_perfGpuCount = 0;
        return Exec::Done;
    }
    if (c.op == "perf_end") {
        endPerf();
        return Exec::Done;
    }
    if (c.op == "reset_stats") {
        m_minCamClearance = 1e9f;
        m_maxAirWheels = 0;
        m_maxLockedWheels = 0;
        m_lockedWheelTime = 0.0;
        m_absSeen = m_tcSeen = false;
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
        if (m_audio->capturing()) std::printf("audio_t=%.3f ", m_audio->capturedSeconds());
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
    if (n == "auto_reverse") return m_ui.autoReverse ? 1.0 : 0.0;
    if (n == "hint") return m_ui.hintVisible(m_sim) ? 1.0 : 0.0;
    if (n == "abs") return t.controls.abs ? 1.0 : 0.0;
    if (n == "tc") return double(int(t.controls.traction));
    if (n == "diff_locks") return t.controls.diffLocks;
    if (n == "front_diff") return double(int(t.frontDiff));
    if (n == "rear_diff") return double(int(t.rearDiff));
    if (n == "center_diff") return double(int(t.centerDiff));
    if (n == "abs_seen") return m_absSeen ? 1.0 : 0.0;
    if (n == "tc_seen") return m_tcSeen ? 1.0 : 0.0;
    if (n == "max_locked") return m_maxLockedWheels;
    if (n == "lock_time") return m_lockedWheelTime;
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
    if (n == "static_grip") return m_sim.vehicleConfig().tuning.tireStaticGrip ? 1.0 : 0.0;
    if (n == "clutch_limit") return m_sim.vehicleConfig().tuning.launchClutchLimit ? 1.0 : 0.0;
    if (n == "fps") return m_fps;
    if (n == "frame_ms") return m_frameMs;
    if (n == "cpu_ms") return m_cpuMs;
    if (n == "gpu_ms") return m_renderer->stats().gpuMs;
    if (n == "mud") return m_gfx.mud;
    if (n == "tod") return m_gfx.settings.timeOfDay;
    if (n == "tod_run") return m_gfx.timeLapse ? 1.0 : 0.0;
    if (n == "sun_el" || n == "sun_az") {
        const SkyState sky = computeSky(m_gfx.settings.timeOfDay, m_gfx.settings.sunHeadingDeg);
        return n == "sun_el" ? sky.sunElevationDeg : sky.sunAzimuthDeg;
    }
    if (n == "trees") {
        const auto& t = m_renderer->stats().trees;
        return t[0] + t[1] + t[2];
    }
    if (n == "quality") return double(int(m_gfx.preset));
    if (n == "msaa") return m_renderer->settings().msaa;
    if (n == "grass_blades") return double(m_renderer->stats().grassBlades);
    if (n == "marks") return double(m_sim.tireMarks().size());
    if (n == "spray") return double(m_spray.size());
    if (n == "spray_emitted") return double(m_spray.emitted());
    if (n == "spray_drawn") return double(m_renderer->stats().sprayParticles);
    if (n == "spray_max") return double(m_spray.settings().maxParticles);
    if (n == "marks_cap") return double(m_sim.tireMarks().capacity());
    if (n == "marks_written") return double(m_sim.tireMarks().written());
    if (n == "marks_drawn") return double(m_renderer->stats().markSegments);
    if (n == "marks_us") return m_sim.timing().marksUs;
    if (n == "tread_soil_max") {
        float v = 0.0f;
        for (std::size_t i = 0; i < kWheelCount; ++i) v = std::max(v, m_sim.tireMarks().soil(i).amount);
        return double(v);
    }
    if (n.size() == 8 && n.rfind("marks_s", 0) == 0) {
        // marks_s<SurfaceId>: live segments on that surface (0 Concrete .. 6 Sand).
        const TireMarks& tm = m_sim.tireMarks();
        const int id = n[7] - '0';
        double count = 0;
        for (std::size_t i = 0; i < tm.size(); ++i) count += int(markSurface(tm.oldest(i))) == id ? 1.0 : 0.0;
        return count;
    }
    if (n.rfind("audio_", 0) == 0) {
        const AudioStats a = m_audio->stats();
        if (n == "audio_active") return m_audio->active() ? 1.0 : 0.0;
        if (n == "audio_t") return m_audio->capturedSeconds();
        if (n == "audio_frames") return double(a.framesRendered);
        if (n == "audio_src_rpm") return a.sourceRpm;
        if (n == "audio_load") return a.load;
        if (n == "audio_skid") return a.skid;
        if (n == "audio_gravel") return a.gravel;
        if (n == "audio_dirt") return a.dirt;
        if (n == "audio_mud") return a.mud;
        if (n == "audio_grass") return a.grass;
        if (n == "audio_wind") return a.wind;
        if (n == "audio_day") return a.day;
        if (n == "audio_night") return a.night;
        if (n == "audio_gameplay") return a.gameplayGain;
        if (n == "audio_voices") return a.activeVoices;
        if (n == "audio_oneshots") return a.oneShotsPlayed;
        if (n == "audio_peak_db") return a.peakDb;
        if (n == "audio_clipped") return double(a.clippedSamples);
        if (n == "audio_cpu") return a.cpuLoad;
        // audio_ev<N>: events of AudioEventType N received by the mixer (0 start, 1 gear, 2 bump stop, 3 thump,
        // 4 body impact, 5 splash, 6 creak, 7 UI).
        if (n.size() == 9 && n.rfind("audio_ev", 0) == 0 && n[8] >= '0' && n[8] <= '7')
            return a.eventsByType[std::size_t(n[8] - '0')];
        return std::nullopt;
    }
    if (n == "draw_calls") return m_renderer->stats().drawCalls;
    if (n == "render_w") return m_renderer->stats().renderW;
    if (n == "reflection") return m_renderer->stats().reflectionRendered ? 1.0 : 0.0;
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
        // CPU frame cost: everything since the frame start except blocking on the swapchain (VSync/back-pressure).
        const double workMs = 1000.0 * double(SDL_GetPerformanceCounter() - now) / freq;
        m_cpuMs = std::max(0.0, workMs - m_renderer->stats().acquireWaitMs);
        if (m_perfActive) {
            m_perfFrameMs.push_back(frameTime * 1000.0);
            m_perfCpuSum += m_cpuMs;
            if (m_renderer->stats().gpuMs > 0.0) {
                m_perfGpuSum += m_renderer->stats().gpuMs;
                ++m_perfGpuCount;
            }
        }

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
