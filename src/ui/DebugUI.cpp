#include "ui/DebugUI.h"

#include "render/TimeOfDay.h"
#include "sim/Simulation.h"

#include <imgui.h>

#include <algorithm>

#include <cmath>
#include <cstdio>

namespace tf {
namespace {

const char* gearName(int gear, char* buf, std::size_t n) {
    if (gear < 0) std::snprintf(buf, n, "R%d", -gear);
    else if (gear == 0) std::snprintf(buf, n, "N");
    else std::snprintf(buf, n, "%d", gear);
    return buf;
}

void bar(const char* label, float v, float lo, float hi, const char* fmt) {
    char text[64];
    std::snprintf(text, sizeof(text), fmt, v);
    ImGui::ProgressBar((v - lo) / (hi - lo), ImVec2(-80.0f, 0.0f), text);
    ImGui::SameLine();
    ImGui::TextUnformatted(label);
}

} // namespace

DebugUI::DebugUI(const VehicleConfig& initial) : m_tuning(initial.tuning), m_config(initial) {}

void DebugUI::remember(const char* key, bool window) {
    UiItemRect& r = m_items[key];
    const ImVec2 a = window ? ImGui::GetWindowPos() : ImGui::GetItemRectMin();
    const ImVec2 b = window ? ImVec2(a.x + ImGui::GetWindowWidth(), a.y + ImGui::GetWindowHeight()) : ImGui::GetItemRectMax();
    r = {a.x, a.y, b.x, b.y, window || ImGui::IsItemVisible(), ImGui::GetFrameCount()};
}

const UiItemRect* DebugUI::item(const std::string& key) const {
    const auto it = m_items.find(key);
    return it != m_items.end() && it->second.frame == ImGui::GetFrameCount() ? &it->second : nullptr;
}

UiRequests DebugUI::draw(Simulation& sim, const FrameStats& stats, GraphicsState& gfx) {
    UiRequests req;
    req.vsync = m_vsync;

    // A short hint is always visible (even with the UI hidden), unless turned off for clean screenshots.
    if (hud) {
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.45f);
        ImGui::Begin("##hud", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoInputs);
        const auto& t = sim.vehicle().telemetry();
        char g[8];
        ImGui::Text("%5.1f km/h  gear %s  %s%s  %4.0f rpm", std::abs(t.forwardSpeed) * 3.6f, gearName(t.gear, g, 8),
                    t.direction == DriveDirection::Forward ? "D" : "R", t.lowRange ? " LOW" : "", t.engineRPM);
        if (stats.paused) ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1), "PAUSED (Esc to resume, N = single step)");
        if (sim.lastMessage()[0]) ImGui::TextDisabled("%s", sim.lastMessage());
        ImGui::TextDisabled("W/S throttle/brake  A/D steer  Space handbrake  Q fwd/rev  L low range  R reset  C camera  "
                            "H lights  F1 UI  F2 graphics");
        ImGui::TextDisabled("%.0f FPS  frame %.2f ms  CPU %.2f ms  GPU %.2f ms  %s %ux%u", stats.fps, stats.frameMs,
                            stats.cpuFrameMs, stats.gpuFrameMs, presetName(gfx.preset), stats.renderW, stats.renderH);
        ImGui::End();
    }

    if (gfx.visible) drawGraphics(stats, gfx);
    if (!visible) return req;
    drawTelemetry(sim, stats, req);
    drawTuning(sim, req, gfx);
    return req;
}

void DebugUI::drawGraphics(const FrameStats& stats, GraphicsState& gfx) {
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float w = std::min(380.0f, display.x * 0.3f);
    ImGui::SetNextWindowPos(ImVec2(0.5f * (display.x - w), 60), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(w, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("Graphics (F2)", &gfx.visible);
    remember("graphics", true);
    RenderSettings& s = gfx.settings;
    bool edited = false;

    ImGui::Text("CPU %.2f ms  GPU %.2f ms  frame %.2f ms (%.0f FPS)", stats.cpuFrameMs, stats.gpuFrameMs, stats.frameMs,
                stats.fps);
    ImGui::Text("3D %ux%u -> %ux%u, %d draws, %.0fk grass blades%s", stats.renderW, stats.renderH, stats.outputW,
                stats.outputH, stats.drawCalls, double(stats.grassBlades) / 1000.0,
                stats.reflection ? ", reflection" : "");

    int preset = int(gfx.preset);
    const char* presets[] = {"Low", "Medium", "High", "Ultra", "Custom"};
    if (ImGui::Combo("Quality", &preset, presets, 5) && preset != int(QualityPreset::Custom)) {
        const RenderSettings look = s;
        s = presetSettings(QualityPreset(preset));
        copyLook(s, look);
        gfx.preset = QualityPreset(preset);
        gfx.changed = true;
    }
    remember("quality");

    if (ImGui::CollapsingHeader("Performance", ImGuiTreeNodeFlags_DefaultOpen)) {
        edited |= ImGui::SliderFloat("Render scale", &s.renderScale, 0.5f, 1.0f, "%.2f");
        int msaa = s.msaa >= 4 ? 2 : s.msaa >= 2 ? 1 : 0;
        const char* msaaNames[] = {"Off", "2x", "4x"};
        if (ImGui::Combo("MSAA", &msaa, msaaNames, stats.maxMsaa >= 4 ? 3 : stats.maxMsaa >= 2 ? 2 : 1)) {
            s.msaa = msaa == 2 ? 4 : msaa == 1 ? 2 : 1;
            edited = true;
        }
        edited |= ImGui::Checkbox("FXAA", &s.fxaa);
        edited |= ImGui::SliderInt("Shadow cascades", &s.shadowCascades, 0, 4);
        int sizeIdx = s.shadowMapSize >= 2048 ? 2 : s.shadowMapSize >= 1536 ? 1 : 0;
        const char* sizes[] = {"1024", "1536", "2048"};
        if (ImGui::Combo("Shadow map", &sizeIdx, sizes, 3)) {
            s.shadowMapSize = sizeIdx == 2 ? 2048 : sizeIdx == 1 ? 1536 : 1024;
            edited = true;
        }
        edited |= ImGui::SliderFloat("Shadow distance [m]", &s.shadowDistance, 20.0f, 150.0f, "%.0f");
        const char* filters[] = {"Hard (1 tap)", "Soft (4 taps)", "Softer (12 taps)"};
        edited |= ImGui::Combo("Shadow filter", &s.shadowFilter, filters, 3);
        edited |= ImGui::Checkbox("Puddle reflections", &s.reflections);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100.0f);
        edited |= ImGui::SliderFloat("##reflscale", &s.reflectionScale, 0.25f, 1.0f, "%.2fx res");
        const char* grass[] = {"Off", "Sparse", "Medium", "Dense", "Very dense"};
        edited |= ImGui::Combo("Grass", &s.grassLevel, grass, 5);
        edited |= ImGui::SliderFloat("Grass distance [m]", &s.grassDistance, 10.0f, 90.0f, "%.0f");
        edited |= ImGui::Checkbox("Bloom", &s.bloom);
        ImGui::SameLine();
        edited |= ImGui::Checkbox("Clouds", &s.clouds);
    }
    if (ImGui::CollapsingHeader("Look", ImGuiTreeNodeFlags_DefaultOpen)) {
        drawTimeOfDay(gfx);
        gfx.changed |= ImGui::SliderFloat("Exposure", &s.exposure, 0.3f, 3.0f, "%.2f");
        gfx.changed |= ImGui::SliderFloat("Sun path heading [deg]", &s.sunHeadingDeg, 0.0f, 360.0f, "%.0f");
        gfx.changed |= ImGui::SliderFloat("Fog density", &s.fogDensity, 0.0f, 4.0f, "%.2f");
        gfx.changed |= ImGui::SliderFloat("Bloom strength", &s.bloomStrength, 0.0f, 0.2f, "%.3f");
        gfx.changed |= ImGui::Checkbox("1 m ground grid", &s.groundGrid);
        ImGui::Checkbox("Headlights (H)", &gfx.headlights);
    }
    if (ImGui::CollapsingHeader("Vehicle dirt (cosmetic)", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Auto (builds up on mud and in puddles, slowly dries off)", &gfx.autoMud);
        ImGui::BeginDisabled(gfx.autoMud);
        ImGui::SliderFloat("Mud amount", &gfx.mud, 0.0f, 1.0f, "%.2f");
        ImGui::EndDisabled();
        ImGui::TextDisabled("Visual only: the ground is rigid; the Mud surface only has a lower friction.");
    }
    if (edited) {
        gfx.preset = QualityPreset::Custom;
        gfx.changed = true;
    }
    ImGui::End();
}

void DebugUI::drawTimeOfDay(GraphicsState& gfx) {
    RenderSettings& s = gfx.settings;
    const int minutes = int(s.timeOfDay * 60.0f + 0.5f) % (24 * 60);
    char label[32];
    std::snprintf(label, sizeof(label), "%02d:%02d", minutes / 60, minutes % 60);
    gfx.changed |= ImGui::SliderFloat("Time of day [h]", &s.timeOfDay, 0.0f, 24.0f, label);
    for (std::size_t i = 0; i < std::size(kTimePresets); ++i) {
        if (i > 0) ImGui::SameLine();
        if (ImGui::Button(kTimePresets[i].name)) {
            s.timeOfDay = kTimePresets[i].hours;
            gfx.changed = true;
        }
    }
    ImGui::Checkbox("Time-lapse", &gfx.timeLapse);
    ImGui::SameLine();
    if (ImGui::Button(gfx.timeLapse ? "Pause##tod" : "Run##tod")) gfx.timeLapse = !gfx.timeLapse;
    ImGui::SliderFloat("Speed [h/s]", &gfx.timeLapseSpeed, 0.01f, 4.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
    const SkyState sky = computeSky(s.timeOfDay, s.sunHeadingDeg);
    ImGui::TextDisabled("Sun elevation %.1f deg, azimuth %.0f deg; shadows from the %s", sky.sunElevationDeg,
                        sky.sunAzimuthDeg, sky.sunElevationDeg > -4.0f ? "sun" : "moon");
}

void DebugUI::drawTelemetry(Simulation& sim, const FrameStats& stats, UiRequests& req) {
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos(ImVec2(10, 60), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(std::min(560.0f, display.x * 0.38f), display.y - 70.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Telemetry (F1)");

    const StepTiming& tm = sim.timing();
    ImGui::Text("Render: %.0f FPS, %.2f ms/frame   GPU: %s, shaders: %s", stats.fps, stats.frameMs, stats.gpuDriver,
                stats.shaderFormat);
    ImGui::Text("Physics: %.0f Hz fixed step, %d step(s) this frame, step CPU last %.3f / avg %.3f / max %.3f ms",
                1.0 / sim.fixedStep(), stats.stepsThisFrame, tm.lastStepMs, tm.avgStepMs, tm.maxStepMs);
    if (stats.overloaded)
        ImGui::TextColored(ImVec4(1, 0.3f, 0.2f, 1), "PHYSICS OVERLOAD: dropped %.1f ms this frame", stats.droppedLastMs);
    else
        ImGui::TextDisabled("Dropped sim time: total %.1f ms (frame clamp / step cap)", stats.droppedTotalMs);
    if (sim.hadPhysicsError()) ImGui::TextColored(ImVec4(1, 0.2f, 0.2f, 1), "Jolt reported an update error (buffers full)");
    ImGui::Text("Gamepad: %s   Camera: %s", stats.gamepad[0] ? stats.gamepad : "not connected",
                stats.followCamera ? "follow" : "free (arrows, PgUp/PgDn, RMB look)");
    if (ImGui::Button(stats.paused ? "Resume (Esc)" : "Pause (Esc)")) req.togglePause = true;
    ImGui::SameLine();
    ImGui::BeginDisabled(!stats.paused);
    if (ImGui::Button("Step one physics tick (N)")) req.stepOnce = true;
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Checkbox("VSync", &m_vsync)) {
        req.setVSync = true;
        req.vsync = m_vsync;
    }

    ImGui::SeparatorText("Vehicle");
    const VehicleTelemetry& t = sim.vehicle().telemetry();
    char g[8];
    ImGui::Text("Speed %.2f m/s (%.1f km/h), along body Z %.2f m/s", t.speed, t.speed * 3.6f, t.forwardSpeed);
    ImGui::Text("Engine %.0f rpm   Gear %s%s   Clutch %.2f   Direction %s   Range %s", t.engineRPM,
                gearName(t.gear, g, 8), t.shifting ? " (shifting)" : "", t.clutchFriction,
                t.direction == DriveDirection::Forward ? "forward" : "reverse", t.lowRange ? "LOW" : "high");
    ImGui::Text("Body tilt from vertical: %.1f deg", std::acos(std::fmin(1.0f, std::fmax(-1.0f, t.upDot))) * 57.2958f);
    bar("throttle [0..1]", t.command.throttle, 0, 1, "%.2f");
    bar("brake [0..1]", t.command.brake, 0, 1, "%.2f");
    bar("handbrake [0..1]", t.command.handBrake, 0, 1, "%.2f");
    bar("steer input [-1..1]", t.command.steer, -1, 1, "%+.2f");
    ImGui::Text("Front wheel steer angle: %+.1f deg (+ = left)", t.steerAngleDeg);

    ImGui::SeparatorText("Wheels");
    if (ImGui::BeginTable("wheels", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("Quantity", ImGuiTableColumnFlags_WidthStretch);
        for (const char* n : kWheelNames) ImGui::TableSetupColumn(n, ImGuiTableColumnFlags_WidthFixed, 92.0f);
        ImGui::TableHeadersRow();
        auto row = [&](const char* name, auto&& cell) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(name);
            for (int i = 0; i < kWheelCount; ++i) {
                ImGui::TableNextColumn();
                cell(t.wheels[i]);
            }
        };
        row("Contact", [](const WheelTelemetry& w) {
            if (w.contact) ImGui::TextColored(ImVec4(0.4f, 1, 0.4f, 1), "yes");
            else ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), "AIR");
        });
        auto& surfaces = sim.surfaces();
        row("Surface", [&](const WheelTelemetry& w) { ImGui::TextUnformatted(w.contact ? surfaces.get(w.surface).name : "-"); });
        row("Susp. length [m]", [](const WheelTelemetry& w) { ImGui::Text("%.3f", w.suspensionLength); });
        row("Compression [m]", [](const WheelTelemetry& w) { ImGui::Text("%.3f", w.compression); });
        row("Compression [%]", [](const WheelTelemetry& w) {
            ImGui::ProgressBar(w.compressionRatio, ImVec2(-1, 0), w.bumpStop ? "BUMP STOP" : nullptr);
        });
        row("Load [N]", [](const WheelTelemetry& w) { ImGui::Text("%.0f", w.load); });
        row("Long. force [N]", [](const WheelTelemetry& w) { ImGui::Text("%+.0f", w.longitudinalForce); });
        row("Lat. force [N]", [](const WheelTelemetry& w) { ImGui::Text("%+.0f", w.lateralForce); });
        row("Slip ratio [-]", [](const WheelTelemetry& w) {
            if (w.contact && w.slipValid) ImGui::Text("%.3f", w.longitudinalSlip); else ImGui::TextDisabled("-");
        });
        row("Slip angle [deg]", [](const WheelTelemetry& w) {
            if (w.contact && w.slipValid) ImGui::Text("%.1f", w.lateralSlipDeg); else ImGui::TextDisabled("-");
        });
        row("Friction mu [-]", [](const WheelTelemetry& w) { ImGui::Text("%.2f", w.combinedLongFriction); });
        row("Wheel speed [rad/s]", [](const WheelTelemetry& w) { ImGui::Text("%+.1f", w.angularVelocity); });
        ImGui::EndTable();
    }
    ImGui::TextDisabled("Load/forces = Jolt constraint impulse [N*s] / collision substep [s] (fixed step / collision steps). Slip ratio: |(w*r - v)/v| (Jolt), '-' below 0.5 m/s.");
    ImGui::End();
}

void DebugUI::drawTuning(Simulation& sim, UiRequests&, GraphicsState& gfx) {
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float tw = std::min(400.0f, display.x * 0.28f);
    ImGui::SetNextWindowPos(ImVec2(display.x - tw - 10.0f, 60), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(tw, display.y - 70.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Tuning");
    remember("tuning", true);

    bool changed = false;
    if (ImGui::CollapsingHeader("Suspension (live)", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::SliderFloat("Front spring [Hz]", &m_tuning.frontSpringFrequency, 0.5f, 3.0f, "%.2f");
        remember("front_spring");
        changed |= ImGui::SliderFloat("Rear spring [Hz]", &m_tuning.rearSpringFrequency, 0.5f, 3.0f, "%.2f");
        changed |= ImGui::SliderFloat("Front damping [-]", &m_tuning.frontDamping, 0.05f, 1.5f, "%.2f");
        changed |= ImGui::SliderFloat("Rear damping [-]", &m_tuning.rearDamping, 0.05f, 1.5f, "%.2f");
        changed |= ImGui::SliderFloat("Anti-roll front [N/m]", &m_tuning.antiRollFront, 0.0f, 20000.0f, "%.0f");
        changed |= ImGui::SliderFloat("Anti-roll rear [N/m]", &m_tuning.antiRollRear, 0.0f, 20000.0f, "%.0f");
    }
    if (ImGui::CollapsingHeader("Tires (live)", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::SliderFloat("Long. peak mu", &m_tuning.tirePeakLongFriction, 0.2f, 2.0f, "%.2f");
        changed |= ImGui::SliderFloat("Long. sliding mu", &m_tuning.tireSlideLongFriction, 0.1f, 2.0f, "%.2f");
        changed |= ImGui::SliderFloat("Peak slip ratio", &m_tuning.tirePeakSlipRatio, 0.02f, 0.5f, "%.2f");
        changed |= ImGui::SliderFloat("Lat. peak mu", &m_tuning.tirePeakLatFriction, 0.2f, 2.0f, "%.2f");
        changed |= ImGui::SliderFloat("Lat. sliding mu", &m_tuning.tireSlideLatFriction, 0.1f, 2.0f, "%.2f");
        changed |= ImGui::SliderFloat("Peak slip angle [deg]", &m_tuning.tirePeakSlipAngleDeg, 1.0f, 20.0f, "%.1f");
    }
    if (ImGui::CollapsingHeader("Surfaces (live)", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextDisabled("Multiplier on tire curve. Ground is rigid in this MVP.");
        for (std::size_t i = 0; i < kSurfaceCount; ++i) {
            SurfaceProperties& s = sim.surfaces().get(static_cast<SurfaceId>(i));
            ImGui::PushID(int(i));
            // The table is only read inside the physics step; the UI runs between steps on the same thread.
            ImGui::SliderFloat2(s.name, &s.longitudinalFriction, 0.05f, 1.5f, "%.2f");
            ImGui::PopID();
        }
        ImGui::TextDisabled("(longitudinal, lateral)");
    }
    if (ImGui::CollapsingHeader("Drivetrain (live)", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::SliderFloat("Engine torque [N*m]", &m_tuning.engineMaxTorque, 100.0f, 1200.0f, "%.0f");
        changed |= ImGui::SliderFloat("Clutch strength", &m_tuning.clutchStrength, 1.0f, 40.0f, "%.1f");
        changed |= ImGui::SliderFloat("Brake torque [N*m/wheel]", &m_tuning.maxBrakeTorque, 500.0f, 8000.0f, "%.0f");
        changed |= ImGui::SliderFloat("Handbrake torque [N*m]", &m_tuning.maxHandBrakeTorque, 0.0f, 8000.0f, "%.0f");
        const char* modes[] = {"Open", "Limited slip"};
        int f = int(m_tuning.frontDiff), r = int(m_tuning.rearDiff), c = int(m_tuning.centerDiff);
        if (ImGui::Combo("Front diff", &f, modes, 2)) { m_tuning.frontDiff = DiffMode(f); changed = true; }
        if (ImGui::Combo("Rear diff", &r, modes, 2)) { m_tuning.rearDiff = DiffMode(r); changed = true; }
        if (ImGui::Combo("Center diff", &c, modes, 2)) { m_tuning.centerDiff = DiffMode(c); changed = true; }
        changed |= ImGui::SliderFloat("LSD ratio (max/min)", &m_tuning.limitedSlipRatio, 1.05f, 4.0f, "%.2f");
        ImGui::TextDisabled("No hard locker: Jolt only offers open/LSD torque split.");
    }
    if (changed) sim.requestTuning(m_tuning);

    const bool vehicleOpen = ImGui::CollapsingHeader("Vehicle (requires rebuild)");
    remember("vehicle_header");
    if (vehicleOpen) {
        ImGui::SliderFloat("Mass [kg]", &m_config.mass, 800.0f, 4000.0f, "%.0f");
        remember("mass");
        ImGui::SliderFloat("Inertia scale", &m_config.inertiaScale, 0.3f, 3.0f, "%.2f");
        ImGui::SliderFloat("COM height [m, body]", &m_config.comHeight, -0.5f, 0.8f, "%.2f");
        ImGui::SliderFloat("COM forward [m]", &m_config.comForward, -0.8f, 0.8f, "%.2f");
        ImGui::SliderFloat("Susp. min length [m]", &m_config.suspensionMinLength, 0.05f, 0.5f, "%.2f");
        ImGui::SliderFloat("Susp. max length [m]", &m_config.suspensionMaxLength, 0.2f, 0.9f, "%.2f");
        ImGui::SliderFloat("Wheel radius [m]", &m_config.wheelRadius, 0.3f, 0.6f, "%.2f");
        ImGui::SliderFloat("Max steer [deg]", &m_config.maxSteerAngleDeg, 15.0f, 45.0f, "%.0f");
        ImGui::SliderFloat("Final drive", &m_config.differentialRatio, 2.5f, 6.0f, "%.2f");
        ImGui::SliderFloat("Low range ratio", &m_config.lowRangeRatio, 1.5f, 4.5f, "%.2f");
        if (m_config.suspensionMaxLength <= m_config.suspensionMinLength + 0.02f)
            m_config.suspensionMaxLength = m_config.suspensionMinLength + 0.02f;
        const bool rebuild = ImGui::Button("Rebuild vehicle (at current spawn)");
        remember("rebuild");
        if (rebuild) {
            VehicleConfig c = m_config;
            c.tuning = m_tuning;
            sim.requestRebuild(c);
        }
        ImGui::SameLine();
        if (ImGui::Button("Defaults")) {
            m_config = VehicleConfig{};
            m_tuning = m_config.tuning;
            sim.requestTuning(m_tuning);
        }
    }
    if (ImGui::CollapsingHeader("Simulation")) {
        const char* rates[] = {"60 Hz", "120 Hz", "240 Hz"};
        const double values[] = {60.0, 120.0, 240.0};
        if (ImGui::Combo("Fixed step", &m_rateIndex, rates, 3)) sim.setFixedStep(1.0 / values[m_rateIndex]);
        int cs = sim.collisionSteps();
        if (ImGui::SliderInt("Collision steps", &cs, 1, 4)) sim.setCollisionSteps(cs);
        InputSmoothing& sm = sim.smoothing();
        ImGui::SliderFloat("Steer rate [1/s]", &sm.steerRate, 0.5f, 10.0f, "%.1f");
        ImGui::SliderFloat("Steer return [1/s]", &sm.steerReturnRate, 0.5f, 10.0f, "%.1f");
        ImGui::SliderFloat("Pedal rise [1/s]", &sm.pedalRise, 0.5f, 20.0f, "%.1f");
        ImGui::SliderFloat("Pedal fall [1/s]", &sm.pedalFall, 0.5f, 20.0f, "%.1f");
    }
    if (ImGui::CollapsingHeader("Debug drawing", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Physics debug lines", &drawPhysicsDebug);
        ImGui::SameLine();
        ImGui::Checkbox("Draw on top", &linesOnTop);
        ImGui::Checkbox("Contacts + normals + load", &debug.contacts);
        ImGui::Checkbox("Suspension axes / travel", &debug.suspension);
        ImGui::Checkbox("Tire forces (red long., blue lat.)", &debug.tireForces);
        ImGui::Checkbox("Body axes", &debug.bodyAxes);
        ImGui::SliderFloat("Force scale [m/N]", &debug.forceScale, 0.00002f, 0.001f, "%.5f", ImGuiSliderFlags_Logarithmic);
    }
    if (ImGui::CollapsingHeader("Teleport", ImGuiTreeNodeFlags_DefaultOpen)) {
        const auto& f = sim.track().features();
        for (std::size_t i = 0; i < f.size(); ++i)
            if (ImGui::Button(f[i].name.c_str(), ImVec2(-1, 0))) sim.requestTeleport(int(i));
    }
    if (ImGui::CollapsingHeader("Time of day (render only)", ImGuiTreeNodeFlags_DefaultOpen)) drawTimeOfDay(gfx);
    ImGui::End();
}

} // namespace tf
