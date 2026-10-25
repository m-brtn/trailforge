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
        // Suspension compression per wheel, laid out as seen from above (front row first, left wheel on the left).
        for (int row = 0; row < 2; ++row) {
            for (int col = 0; col < 2; ++col) {
                const int i = row * 2 + col;
                const WheelTelemetry& w = t.wheels[i];
                char overlay[16];
                std::snprintf(overlay, sizeof(overlay), w.contact ? "%s %3.0f%%" : "%s air", kWheelNames[i],
                              100.0f * w.compressionRatio);
                if (col > 0) ImGui::SameLine();
                const ImVec4 color = w.bumpStop ? ImVec4(0.85f, 0.2f, 0.15f, 1) : !w.contact ? ImVec4(0.45f, 0.45f, 0.5f, 1)
                                                                                             : ImVec4(0.3f, 0.6f, 0.3f, 1);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, color);
                ImGui::ProgressBar(std::clamp(w.compressionRatio, 0.0f, 1.0f), ImVec2(110, 0), overlay);
                ImGui::PopStyleColor();
            }
        }
        if (stats.paused) ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1), "PAUSED (Esc to resume, N = single step)");
        if (sim.lastMessage()[0]) ImGui::TextDisabled("%s", sim.lastMessage());
        ImGui::TextDisabled("W gas / S brake (stopped: release, press again for reverse)  A/D steer  Space handbrake");
        ImGui::TextDisabled("Q fwd/rev  L low range  B ABS  T traction control  X diff locks  R reset");
        ImGui::TextDisabled("C camera  H lights  F1 UI  F2 graphics  F3 audio");
        ImGui::TextDisabled("%.0f FPS  frame %.2f ms  CPU %.2f ms  GPU %.2f ms  %s %ux%u  v%s", stats.fps, stats.frameMs,
                            stats.cpuFrameMs, stats.gpuFrameMs, presetName(gfx.preset), stats.renderW, stats.renderH,
                            TRAILFORGE_VERSION);
        ImGui::End();
        drawDriveHud(sim);
    }

    if (gfx.visible) drawGraphics(sim, stats, gfx);
    if (!visible) return req;
    drawTelemetry(sim, stats, req);
    drawTuning(sim, req, gfx);
    return req;
}

// ABS / TC lamps (dim = off, green = armed, blinking amber = acting) and the effective differential modes.
void DebugUI::drawAidIndicators(const Simulation& sim) {
    const auto& t = sim.vehicle().telemetry();
    const double now = sim.simTime();
    if (t.absActive) m_absLastActive = now;
    if (t.tcActive) m_tcLastActive = now;
    // Lamps stay lit 0.3 s after the last intervention and blink at 4 Hz while acting.
    const bool blinkOn = std::fmod(now, 0.25) < 0.15;
    auto lamp = [&](const char* text, bool armed, double lastActive) {
        if (!armed) ImGui::TextDisabled("%s", text);
        else if (now - lastActive < 0.3) ImGui::TextColored(blinkOn ? ImVec4(1.0f, 0.65f, 0.1f, 1) : ImVec4(0.45f, 0.3f, 0.1f, 1), "%s", text);
        else ImGui::TextColored(ImVec4(0.45f, 0.9f, 0.45f, 1), "%s", text);
    };
    ImGui::BeginGroup();
    lamp("ABS", t.controls.abs, m_absLastActive);
    ImGui::SameLine();
    lamp(t.controls.traction == TractionMode::OffRoad ? "TC OFF-ROAD" : "TC", t.controls.traction != TractionMode::Off,
         m_tcLastActive);
    auto diff = [](const char* axle, DiffMode m) {
        const char* name = m == DiffMode::Open ? "open" : m == DiffMode::LimitedSlip ? "LSD" : "LOCK";
        if (m == DiffMode::Locked) ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1), "%s %s", axle, name);
        else ImGui::TextDisabled("%s %s", axle, name);
    };
    diff("F", t.frontDiff);
    ImGui::SameLine();
    diff("C", t.centerDiff);
    ImGui::SameLine();
    diff("R", t.rearDiff);
    ImGui::EndGroup();
}

bool DebugUI::hintVisible(const Simulation& sim) const {
    return sim.lastMessage()[0] && sim.simTime() - sim.lastMessageTime() < kHintSeconds;
}

// Bottom-center driving read-out: selector, gear, range and speed, plus a short hint after a discrete command.
void DebugUI::drawDriveHud(const Simulation& sim) {
    const auto& t = sim.vehicle().telemetry();
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                   ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoInputs;
    const bool reverse = t.direction == DriveDirection::Reverse;
    const char* selector = t.gear == 0 ? "N" : reverse ? "R" : "D";
    char gear[8];
    ImGui::SetNextWindowPos(ImVec2(0.5f * display.x, display.y - 16.0f), ImGuiCond_Always, ImVec2(0.5f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::Begin("##drivehud", nullptr, flags);
    remember("drive_hud", true);
    ImGui::PushFont(nullptr, 2.4f * ImGui::GetStyle().FontSizeBase);
    const ImVec4 selColor = reverse ? ImVec4(1.0f, 0.55f, 0.25f, 1) : t.gear == 0 ? ImVec4(0.75f, 0.75f, 0.75f, 1)
                                                                                  : ImVec4(0.45f, 0.9f, 0.45f, 1);
    ImGui::TextColored(selColor, "%s", selector);
    ImGui::SameLine();
    ImGui::Text("%3.0f", std::abs(t.forwardSpeed) * 3.6f);
    ImGui::PopFont();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::TextUnformatted("km/h");
    ImGui::Text("gear %s  %s", gearName(t.gear, gear, sizeof(gear)), reverse ? "(reverse)" : "(forward)");
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    if (t.lowRange) ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1), "LOW");
    else ImGui::TextDisabled("HIGH");
    ImGui::TextDisabled("%4.0f rpm", t.engineRPM);
    ImGui::EndGroup();
    ImGui::SameLine();
    drawAidIndicators(sim);
    ImGui::End();

    if (!hintVisible(sim)) return;
    ImGui::SetNextWindowPos(ImVec2(0.5f * display.x, display.y - 110.0f), ImGuiCond_Always, ImVec2(0.5f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.6f);
    ImGui::Begin("##drivehint", nullptr, flags);
    ImGui::PushFont(nullptr, 1.5f * ImGui::GetStyle().FontSizeBase);
    ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.5f, 1), "%s", sim.lastMessage());
    ImGui::PopFont();
    ImGui::End();
}

void DebugUI::drawGraphics(Simulation& sim, const FrameStats& stats, GraphicsState& gfx) {
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
        edited |= ImGui::SliderFloat("Flowers", &s.flowerDensity, 0.0f, 1.0f, "%.2f");
        edited |= ImGui::Checkbox("Grass detail (awns, seed heads)", &s.grassDetail);
        edited |= ImGui::Checkbox("Bloom", &s.bloom);
        ImGui::SameLine();
        edited |= ImGui::Checkbox("Clouds", &s.clouds);
        edited |= ImGui::SliderInt("Spray particles", &s.sprayMax, 0, 8192);
        ImGui::SameLine();
        edited |= ImGui::Checkbox("Dust", &s.sprayDust);
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
    if (ImGui::CollapsingHeader("Tire marks (visual only)", ImGuiTreeNodeFlags_DefaultOpen))
        drawTireMarks(sim, stats, gfx);
    if (edited) {
        gfx.preset = QualityPreset::Custom;
        gfx.changed = true;
    }
    ImGui::End();
}

void DebugUI::drawTireMarks(Simulation& sim, const FrameStats& stats, GraphicsState& gfx) {
    TireMarks& marks = sim.tireMarks();
    TireMarkSettings ms = marks.settings();
    bool apply = false;
    ImGui::Checkbox("Draw", &gfx.tireMarks);
    ImGui::SameLine();
    apply |= ImGui::Checkbox("Generate", &ms.enabled);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) marks.clear();
    ImGui::SameLine();
    if (ImGui::Button("Fill to cap")) gfx.fillMarks = true;
    // Changing the capacity clears the ring, so it is applied when the slider is released.
    ImGui::SliderInt("Max segments", &ms.maxSegments, 1024, 65536, "%d", ImGuiSliderFlags_Logarithmic);
    apply |= ImGui::IsItemDeactivatedAfterEdit();
    ImGui::SliderFloat("Fade time [s]", &ms.fadeTime, 10.0f, 1200.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
    apply |= ImGui::IsItemDeactivatedAfterEdit();
    if (apply) marks.setSettings(ms);
    static const char* kNames[7] = {"Concrete", "Packed dirt", "Rock", "Low grip", "Mud", "Grass", "Sand"};
    ImGui::TextUnformatted("Intensity per surface");
    for (int i = 0; i < 7; ++i) {
        ImGui::PushID(i);
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
        ImGui::SliderFloat(kNames[i], &gfx.markGain[std::size_t(i)], 0.0f, 2.0f, "%.2f");
        ImGui::PopID();
    }
    const std::size_t bytes = marks.size() * sizeof(TireMarkSegment);
    ImGui::Text("%zu / %zu segments (%.1f MB ring), %llu written", marks.size(), marks.capacity(),
                double(marks.capacity() * sizeof(TireMarkSegment)) / (1024.0 * 1024.0),
                static_cast<unsigned long long>(marks.written()));
    ImGui::Text("CPU update %.1f us/tick, upload %.1f KB/frame, live %.0f KB, 1 draw", sim.timing().marksUs,
                double(stats.markUploadBytes) / 1024.0, double(bytes) / 1024.0);
    ImGui::TextDisabled("No effect on friction or ground height; the ground stays rigid.");
    ImGui::Separator();
    ImGui::Checkbox("Wheel spray", &gfx.spray);
    ImGui::SameLine();
    ImGui::Text("%zu live, %zu drawn, cap %d%s", gfx.sprayLive, gfx.sprayDrawn, gfx.settings.sprayMax,
                gfx.settings.sprayDust ? "" : ", no dust");
    ImGui::TextDisabled("Cosmetic particles, not physics bodies.");
}

void DebugUI::drawAudio(AudioPanelState& a) {
    if (!a.visible) return;
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float w = std::min(400.0f, display.x * 0.3f);
    ImGui::SetNextWindowPos(ImVec2(0.5f * (display.x - w), 80), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(w, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("Audio (F3)", &a.visible);
    remember("audio", true);
    AudioSettings& s = a.settings;
    const AudioStats& st = a.stats;
    ImGui::TextWrapped("%s", a.status);
    if (!a.active) ImGui::TextDisabled("No audio output (--no-audio, no device or no assets).");

    ImGui::Checkbox("Mute", &s.mute);
    ImGui::SliderFloat("Master", &s.master, 0.0f, 1.5f, "%.2f");
    ImGui::SliderFloat("Engine", &s.engine, 0.0f, 2.0f, "%.2f");
    ImGui::SliderFloat("Tires", &s.tires, 0.0f, 2.0f, "%.2f");
    ImGui::SliderFloat("Impacts", &s.impacts, 0.0f, 2.0f, "%.2f");
    ImGui::SliderFloat("Ambience & wind", &s.ambience, 0.0f, 2.0f, "%.2f");
    ImGui::SliderFloat("UI", &s.ui, 0.0f, 2.0f, "%.2f");
    if (ImGui::CollapsingHeader("Engine model", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Pitch factor", &s.enginePitch, 0.6f, 1.1f, "%.2f");
        ImGui::SliderFloat("Level spread [dB]", &s.levelSpreadDb, 0.0f, 40.0f, "%.1f");
        ImGui::SliderFloat("Transmission whine", &s.whine, 0.0f, 3.0f, "%.2f");
        ImGui::Text("Source rpm %.0f (engine x %.2f), load %.2f", st.sourceRpm, s.enginePitch, st.load);
        ImGui::Text("Off-load  idle %.2f  mid %.2f  high %.2f", st.offWeights[0], st.offWeights[1], st.offWeights[2]);
        ImGui::Text("Full load low  %.2f  mid %.2f  high %.2f", st.onWeights[0], st.onWeights[1], st.onWeights[2]);
        ImGui::Text("Exhaust rates  %.2f / %.2f / %.2f, whine %.0f Hz", st.exhaustRates[0], st.exhaustRates[1],
                    st.exhaustRates[2], st.whineHz);
    }
    if (ImGui::CollapsingHeader("Layers", ImGuiTreeNodeFlags_DefaultOpen)) {
        bar("skid", st.skid, 0, 1.2f, "%.2f");
        bar("gravel", st.gravel, 0, 1.2f, "%.2f");
        bar("dirt", st.dirt, 0, 1.2f, "%.2f");
        bar("mud", st.mud, 0, 1.2f, "%.2f");
        bar("grass", st.grass, 0, 1.2f, "%.2f");
        bar("wind", st.wind, 0, 1, "%.2f");
        ImGui::Text("Ambience day %.2f / night %.2f, gameplay %.2f", st.day, st.night, st.gameplayGain);
    }
    if (ImGui::CollapsingHeader("Output", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("%d voices, %d one-shots played, %d events dropped", st.activeVoices, st.oneShotsPlayed,
                    st.eventsDropped);
        ImGui::Text("Peak %.1f dBFS, limiter %.1f dB, %lld clamped samples", st.peakDb, st.limiterDb,
                    st.clippedSamples);
        ImGui::Text("Audio thread: %.0f us/callback (max %.0f), %.2f %% of real time", st.callbackUs,
                    st.callbackMaxUs, 100.0 * st.cpuLoad);
    }
    ImGui::TextDisabled("Pause-menu UI sounds are loaded; the menu itself is not built yet.");
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
    // Below the always-on HUD (top left).
    ImGui::SetNextWindowPos(ImVec2(10, 180), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(std::min(560.0f, display.x * 0.38f), display.y - 190.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Telemetry (F1)");

    ImGui::TextDisabled("TrailForge %s", TRAILFORGE_VERSION);
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
    auto diffName = [](DiffMode m) { return m == DiffMode::Open ? "open" : m == DiffMode::LimitedSlip ? "LSD" : "locked"; };
    const char* tcModes[] = {"off", "on", "off-road"};
    ImGui::Text("ABS %s%s   TC %s%s, throttle x%.2f -> %.2f   Diffs F %s / C %s / R %s", t.controls.abs ? "on" : "off",
                t.absActive ? " (acting)" : "", tcModes[int(t.controls.traction)], t.tcActive ? " (acting)" : "",
                t.throttleFactor, t.engineThrottle, diffName(t.frontDiff), diffName(t.centerDiff), diffName(t.rearDiff));

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
        row("Brake [N*m]", [](const WheelTelemetry& w) { ImGui::Text("%.0f", w.brakeTorque); });
        row("ABS pressure [-]", [](const WheelTelemetry& w) { ImGui::Text("%.2f", w.absPressure); });
        row("TC brake [N*m]", [](const WheelTelemetry& w) { ImGui::Text("%.0f", w.tcBrake); });
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
        changed |= ImGui::Checkbox("Static grip (peak mu below breakaway)", &m_tuning.tireStaticGrip);
        remember("static_grip");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Off: plain Jolt curve, mu(0) = 0; gripping wheels alternate grip / free spin.");
        changed |= ImGui::SliderFloat("Slip speed floor [m/s]", &m_tuning.tireSlipSpeedFloor, 0.0f, 3.0f, "%.2f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Slip ratio for the friction curve = |w*r - v| / max(|v|, floor).\n"
                              "0: Jolt's own ratio, where any wheel turn at a standstill reads as full slip.");
        changed |= ImGui::Checkbox("Friction ellipse (combined slip)", &m_tuning.tireFrictionEllipse);
        remember("friction_ellipse");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Beyond the peak the longitudinal and lateral limits share one budget (ellipse).\n"
                              "Off: Jolt's independent limits, a locked wheel still steers.");
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
        changed |= ImGui::Checkbox("Launch clutch limit", &m_tuning.launchClutchLimit);
        remember("clutch_limit");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Near min rpm the clutch passes at most the engine torque at this throttle.\n"
                              "Off: the slipping clutch delivers ~840 N*m at any throttle.");
        changed |= ImGui::SliderFloat("Creep torque [N*m]", &m_tuning.creepTorque, 0.0f, 150.0f, "%.0f");
        changed |= ImGui::SliderFloat("Brake torque [N*m/wheel]", &m_tuning.maxBrakeTorque, 500.0f, 8000.0f, "%.0f");
        changed |= ImGui::SliderFloat("Handbrake torque [N*m]", &m_tuning.maxHandBrakeTorque, 0.0f, 8000.0f, "%.0f");
        const char* modes[] = {"Open", "Limited slip", "Locked"};
        int f = int(m_tuning.frontDiff), r = int(m_tuning.rearDiff), c = int(m_tuning.centerDiff);
        if (ImGui::Combo("Front diff", &f, modes, 3)) { m_tuning.frontDiff = DiffMode(f); changed = true; }
        if (ImGui::Combo("Rear diff", &r, modes, 3)) { m_tuning.rearDiff = DiffMode(r); changed = true; }
        if (ImGui::Combo("Center diff", &c, modes, 3)) { m_tuning.centerDiff = DiffMode(c); changed = true; }
        changed |= ImGui::SliderFloat("Axle LSD ratio (max/min)", &m_tuning.limitedSlipRatio, 1.05f, 4.0f, "%.2f");
        changed |= ImGui::SliderFloat("Center LSD ratio", &m_tuning.centerLimitedSlipRatio, 1.05f, 4.0f, "%.2f");
        changed |= ImGui::SliderFloat("Locked ratio", &m_tuning.lockedSlipRatio, 1.001f, 1.5f, "%.3f");
        ImGui::TextDisabled("Jolt diffs split torque; LSD/Locked cap the faster/slower side speed ratio.\n"
                            "Locked = ratio near 1: torque goes to the slower side, no rigid shaft link.\n"
                            "X cycles lockers on top: rear, rear + center, all.");
    }
    if (ImGui::CollapsingHeader("Driver aids (live)", ImGuiTreeNodeFlags_DefaultOpen)) {
        AidControls a = sim.aidControls();
        bool sw = ImGui::Checkbox("ABS (B)", &a.abs);
        remember("abs_switch");
        int tc = int(a.traction);
        const char* tcModes[] = {"Off", "On", "Off-road"};
        if (ImGui::Combo("Traction control (T)", &tc, tcModes, 3)) { a.traction = TractionMode(tc); sw = true; }
        const char* lockNames[] = {"none", "rear", "rear + center", "all"};
        sw |= ImGui::SliderInt("Diff locks (X)", &a.diffLocks, 0, AidControls::kDiffLockStages - 1,
                               lockNames[std::clamp(a.diffLocks, 0, AidControls::kDiffLockStages - 1)]);
        if (sw) sim.requestAidControls(a);
        DrivingAidTuning& d = m_tuning.aids;
        ImGui::SeparatorText("ABS");
        changed |= ImGui::SliderFloat("Release slip", &d.absReleaseSlip, 0.05f, 0.5f, "%.2f");
        changed |= ImGui::SliderFloat("Re-apply slip", &d.absApplySlip, 0.0f, 0.4f, "%.2f");
        changed |= ImGui::SliderFloat("Release rate [1/s]", &d.absReleaseRate, 5.0f, 100.0f, "%.0f");
        changed |= ImGui::SliderFloat("Apply rate [1/s]", &d.absApplyRate, 1.0f, 40.0f, "%.1f");
        changed |= ImGui::SliderFloat("ABS min speed [m/s]", &d.absMinSpeed, 0.0f, 5.0f, "%.1f");
        ImGui::SeparatorText("Traction control");
        changed |= ImGui::SliderFloat("Slip threshold (On)", &d.tcSlipThreshold, 0.05f, 1.0f, "%.2f");
        changed |= ImGui::SliderFloat("Slip threshold (Off-road)", &d.tcOffRoadSlipThreshold, 0.05f, 1.5f, "%.2f");
        changed |= ImGui::SliderFloat("Throttle floor (On)", &d.tcMinThrottle, 0.0f, 1.0f, "%.2f");
        changed |= ImGui::SliderFloat("Throttle floor (Off-road)", &d.tcOffRoadMinThrottle, 0.0f, 1.0f, "%.2f");
        changed |= ImGui::SliderFloat("Throttle cut rate [1/s]", &d.tcCutRate, 0.1f, 10.0f, "%.1f");
        changed |= ImGui::SliderFloat("Throttle recover rate [1/s]", &d.tcRecoverRate, 0.1f, 10.0f, "%.1f");
        changed |= ImGui::SliderFloat("Brake gain [N*m]", &d.tcBrakeGain, 0.0f, 10000.0f, "%.0f");
        changed |= ImGui::SliderFloat("Max brake [N*m/wheel]", &d.tcMaxBrakeTorque, 0.0f, 4000.0f, "%.0f");
        ImGui::TextDisabled("Simplified control logic on top of the tire model: brake torque per\n"
                            "wheel and an engine throttle factor; no hidden forces.");
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
        ImGui::Checkbox("Auto reverse (S/W at a standstill)", &autoReverse);
        AutoReverseSettings& ar = sim.autoReverse();
        ImGui::SliderFloat("Auto reverse speed [m/s]", &ar.stopSpeed, 0.1f, 1.0f, "%.2f");
        ImGui::SliderFloat("Auto reverse hold [s]", &ar.holdTime, 0.0f, 1.0f, "%.2f");
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
