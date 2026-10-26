#pragma once

#include "audio/AudioTypes.h"
#include "render/DebugDraw.h"
#include "render/RenderSettings.h"
#include "vehicle/VehicleConfig.h"

#include <array>
#include <map>
#include <string>

namespace tf {

// Widget screen rectangle (SDL window coordinates, points), for scenarios that click
// the UI through regular mouse events.
struct UiItemRect {
    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    bool visible = false; // not clipped by the window (visible on screen)
    int frame = -1;       // ImGui frame in which the widget was drawn
};

class Simulation;

struct FrameStats {
    float fps = 0.0f;
    float frameMs = 0.0f;
    int stepsThisFrame = 0;
    double droppedLastMs = 0.0;
    double droppedTotalMs = 0.0;
    bool overloaded = false;
    bool paused = false;
    int cameraMode = 0; // Camera::Mode: 0 follow, 1 close follow, 2 hood, 3 free
    const char* gpuDriver = "";
    const char* shaderFormat = "";
    const char* gamepad = ""; // empty: not connected
    double cpuFrameMs = 0.0;  // CPU time of the last frame without waiting for the swapchain
    double gpuFrameMs = 0.0;  // GPU time of the last completed command buffer, < 0: not available (non-Metal)
    unsigned renderW = 0, renderH = 0, outputW = 0, outputH = 0;
    int drawCalls = 0;
    unsigned long long grassBlades = 0;
    bool reflection = false;
    int maxMsaa = 1;
    unsigned markUploadBytes = 0; // tire mark bytes uploaded in the last frame
    float autoExposureEv = 0.0f;  // auto exposure correction on top of the time-of-day exposure
};

// Graphics panel state (owned by the application, edited by the UI).
struct GraphicsState {
    RenderSettings settings;
    QualityPreset preset = QualityPreset::High;
    bool changed = false;        // set by the UI when settings were edited
    float mud = 0.0f;            // cosmetic mud on the vehicle, 0..1 (no effect on physics)
    bool autoMud = true;         // accumulate while driving on the mud surface, wash off in water
    bool headlights = true;
    bool visible = false;        // F2
    bool timeLapse = false;      // advance settings.timeOfDay in real time
    float timeLapseSpeed = 0.25f; // game hours per real second
    // Tire marks (render-only): drawing toggle and per-surface intensity multipliers (SurfaceId order).
    bool tireMarks = true;
    std::array<float, 8> markGain{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    bool fillMarks = false;      // request: fill the ring with synthetic segments (performance test at the cap)
    bool spray = true;           // wheel spray particles (render-only)
    std::size_t sprayLive = 0, sprayDrawn = 0;
};

// Audio panel state (owned by the application; settings go to the audio thread with the next frame).
struct AudioPanelState {
    AudioSettings settings;
    AudioStats stats;          // latest read-out from the audio thread
    const char* status = "";   // device / capture / disabled
    bool active = false;
    bool visible = false;      // F3
};

// UI requests to the application (executed outside the ImGui frame).
struct UiRequests {
    bool togglePause = false;
    bool stepOnce = false;
    bool setVSync = false;
    bool vsync = true;
};

// Dear ImGui windows: telemetry, suspension/grip tuning, diagnostics.
// All physics changes go to Simulation as requests and are applied between steps.
class DebugUI {
public:
    explicit DebugUI(const VehicleConfig& initial);

    UiRequests draw(Simulation& sim, const FrameStats& stats, GraphicsState& gfx);
    void drawAudio(AudioPanelState& audio);

    bool visible = true;
    bool hud = true;               // the small always-on hint/perf window and the driving read-out
    bool menuOpen = false;         // a menu is on screen: no HUD and no diagnostics windows (F1-F3 keep their state)
    bool autoReverse = true;       // S/W at a standstill switch direction (DriverIntent::autoReverse)
    std::string driveHint = "W gas / S brake (stopped: release, press again for reverse)  A/D steer  SPACE handbrake";
    bool drawPhysicsDebug = false; // off by default so the scene reads cleanly; Tuning > Debug drawing
    bool linesOnTop = true;
    VehicleDebugOptions debug;
    double storedBestLap = 0.0; // s, best lap of this level and vehicle from best_laps.cfg (0: none)
    float lapHudX = -1.0f, lapHudTop = 12.0f; // lap panel top-center (below the compass bar; x < 0: screen center)

    // The short driving hint (latest Simulation message) is on screen.
    bool hintVisible(const Simulation& sim) const;
    static constexpr double kHintSeconds = 2.5; // s of simulated time

    // Widget drawn in the last frame, or nullptr.
    const UiItemRect* item(const std::string& key) const;

private:
    void remember(const char* key, bool window = false);
    std::map<std::string, UiItemRect> m_items;

    void drawDriveHud(const Simulation& sim);
    void drawLapHud(const Simulation& sim);
    void drawAidIndicators(const Simulation& sim);
    void drawTelemetry(Simulation& sim, const FrameStats& stats, UiRequests& req);
    void drawTuning(Simulation& sim, UiRequests& req, GraphicsState& gfx);
    void drawGraphics(Simulation& sim, const FrameStats& stats, GraphicsState& gfx);
    void drawTireMarks(Simulation& sim, const FrameStats& stats, GraphicsState& gfx);
    void drawTimeOfDay(GraphicsState& gfx);

    VehicleTuning m_tuning;
    VehicleConfig m_config; // editable copy; applied with the "Rebuild vehicle" button
    VehicleConfig m_seen;   // the simulation's configuration when the copies were last taken
    bool m_vsync = true;
    int m_rateIndex = 1;
    double m_absLastActive = -1.0e9, m_tcLastActive = -1.0e9; // simulated time the aid last acted (indicator blink)
};

} // namespace tf
