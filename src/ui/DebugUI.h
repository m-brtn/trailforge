#pragma once

#include "render/DebugDraw.h"
#include "vehicle/VehicleConfig.h"

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
    bool followCamera = true;
    const char* gpuDriver = "";
    const char* shaderFormat = "";
    const char* gamepad = ""; // empty: not connected
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

    UiRequests draw(Simulation& sim, const FrameStats& stats);

    bool visible = true;
    bool drawPhysicsDebug = true;
    bool linesOnTop = true;
    VehicleDebugOptions debug;

    // Widget drawn in the last frame, or nullptr.
    const UiItemRect* item(const std::string& key) const;

private:
    void remember(const char* key, bool window = false);
    std::map<std::string, UiItemRect> m_items;

    void drawTelemetry(Simulation& sim, const FrameStats& stats, UiRequests& req);
    void drawTuning(Simulation& sim, UiRequests& req);

    VehicleTuning m_tuning;
    VehicleConfig m_config; // editable copy; applied with the "Rebuild vehicle" button
    bool m_vsync = true;
    int m_rateIndex = 1;
};

} // namespace tf
