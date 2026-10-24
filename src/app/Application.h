#pragma once

#include "app/Scenario.h"
#include "audio/AudioSystem.h"
#include "audio/VehicleAudioFeed.h"
#include "input/Input.h"
#include "render/Camera.h"
#include "render/DebugDraw.h"
#include "render/Renderer.h"
#include "sim/FixedStepClock.h"
#include "sim/Simulation.h"
#include "sim/WheelSpray.h"
#include "ui/DebugUI.h"

#include <SDL3/SDL_video.h>

#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace tf {

struct AppOptions {
    double quitAfterSeconds = 0.0; // >0: quit automatically (startup smoke test)
    bool demoDrive = false;        // scripted throttle/steering for a demo without input
    bool startWithUi = true;
    std::string capturePath;   // non-empty: save a frame (BMP/PNG) at captureAtSeconds
    double captureAtSeconds = 3.0;
    std::string scriptPath;    // non-empty: run a verification scenario (see Scenario.h)
    bool audio = true;         // false: --no-audio (no device, no sound assets loaded)
    std::string audioCapturePath; // non-empty: offline audio render to this WAV instead of the device
};

// RAII wrapper for SDL initialization.
class SdlContext {
public:
    SdlContext();
    ~SdlContext();
    SdlContext(const SdlContext&) = delete;
    SdlContext& operator=(const SdlContext&) = delete;
};

class Window {
public:
    Window(const char* title, int w, int h);
    ~Window();
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
    SDL_Window* get() const { return m_window; }

private:
    SDL_Window* m_window = nullptr;
};

// RAII Dear ImGui context with SDL3 + SDL GPU backends.
class ImGuiContextGuard {
public:
    ImGuiContextGuard(SDL_Window* window, const Renderer& renderer);
    ~ImGuiContextGuard();
    ImGuiContextGuard(const ImGuiContextGuard&) = delete;
    ImGuiContextGuard& operator=(const ImGuiContextGuard&) = delete;
};

// Lifecycle and main loop: events -> fixed physics steps -> interpolation -> render.
class Application {
public:
    explicit Application(const AppOptions& options);
    ~Application();
    int run();

private:
    void processEvents();
    void setPaused(bool paused);
    void publishAudio();
    void stepSimulation(double frameTime);
    void renderFrame(float frameTime);

    // ---- Verification scenario (--script)
    void updateScenario();
    Scenario::Exec execScenario(const ScenarioCommand& c);
    std::optional<double> scenarioValue(const std::string& name) const;
    void queueEvent(const SDL_Event& e, int framesFromNow);
    void queueClick(float x, float y, bool textInput);

    AppOptions m_options;
    // Member order = construction order; destruction is the reverse.
    SdlContext m_sdl;
    Window m_window;
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<ImGuiContextGuard> m_imgui;
    Simulation m_sim;
    WheelSpray m_spray; // cosmetic particles thrown by the wheels (render-only)
    Input m_input;
    Camera m_camera;
    DebugDraw m_debugDraw;
    DebugUI m_ui;
    FixedStepClock m_clock;
    // Audio: the feed turns every physics tick into sound parameters and events; the system owns the device
    // (or the offline capture) and the mixer that runs on the audio thread.
    VehicleAudioFeed m_audioFeed;
    AudioPanelState m_audioPanel;
    std::unique_ptr<AudioSystem> m_audio;

    void buildDraws(const glm::mat4& bodyM, const std::array<glm::mat4, kWheelCount>& wheelM);
    void updateCosmetics(float dt);
    void endPerf();
    void fillTireMarks();

    GpuMesh m_trackMesh, m_sceneryMesh, m_waterMesh, m_bodyMesh, m_wheelMesh;
    GpuMesh m_springMesh, m_armMesh, m_halfShaftMesh, m_shockBodyMesh, m_shockShaftMesh;
    float m_wheelMeshRadius = 0.0f, m_wheelMeshWidth = 0.0f;
    std::vector<DrawItem> m_draws;
    GraphicsState m_gfx;
    double m_cpuMs = 0.0;            // last frame: CPU work without the swapchain wait

    // perf_begin / perf_end (scenario frame-time measurement)
    bool m_perfActive = false;
    std::string m_perfLabel;
    std::vector<double> m_perfFrameMs;
    double m_perfCpuSum = 0.0, m_perfGpuSum = 0.0;
    int m_perfGpuCount = 0;

    bool m_running = true;
    bool m_paused = false;
    bool m_stepOnce = false;
    bool m_captured = false;
    int m_stepsThisFrame = 0;
    double m_time = 0.0;
    float m_fpsAccum = 0.0f;
    int m_fpsFrames = 0;
    float m_fps = 0.0f;
    float m_frameMs = 0.0f;
    int m_exitCode = 0;

    std::optional<Scenario> m_scenario;
    struct QueuedEvent { int frames; SDL_Event event; };
    std::vector<QueuedEvent> m_queuedEvents;
    std::deque<std::string> m_textStorage; // SDL_EVENT_TEXT_INPUT stores a pointer to the string
    bool m_pushedThisFrame = false;
    int m_scrollFrames = 0;
    int m_scrollStableFrames = 0;
    float m_scrollLastY = 0.0f;
    std::string m_captureRequest;
    float m_camClearance = 0.0f;     // m: camera height above the surface below it
    float m_minCamClearance = 1e9f;  // minimum since the last reset_stats (follow camera only)
    int m_maxAirWheels = 0;          // maximum number of wheels without contact since the last reset_stats
    int m_maxLockedWheels = 0;       // maximum number of locked wheels (in contact, > 2 m/s) since the last reset_stats
    double m_lockedWheelTime = 0.0; // wheel-seconds of locked wheels since the last reset_stats (sampled per frame)
    bool m_absSeen = false, m_tcSeen = false; // ABS / TC acted in a sampled frame since the last reset_stats
};

} // namespace tf
