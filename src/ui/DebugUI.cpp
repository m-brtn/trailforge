#include "ui/DebugUI.h"

#include "render/TimeOfDay.h"
#include "sim/Simulation.h"

#include <imgui.h>

#include <algorithm>

#include <cmath>
#include <cstdio>
#include <string>

namespace tf {
namespace {

const char* gearName(int gear, char* buf, std::size_t n) {
    if (gear < 0) std::snprintf(buf, n, "R%d", -gear);
    else if (gear == 0) std::snprintf(buf, n, "N");
    else std::snprintf(buf, n, "%d", gear);
    return buf;
}

std::string gpuText(double ms) {
    if (ms < 0.0) return "n/a";
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f ms", ms);
    return buf;
}

// Tire temperature colour: blue below the compound's window, green inside, red above.
ImVec4 tireColor(float state) {
    const ImVec4 cold(0.20f, 0.45f, 0.95f, 1.0f), ok(0.25f, 0.70f, 0.30f, 1.0f), hot(0.95f, 0.30f, 0.15f, 1.0f);
    const ImVec4& end = state < 0.0f ? cold : hot;
    const float k = std::clamp(std::abs(state), 0.0f, 1.0f);
    return ImVec4(ok.x + (end.x - ok.x) * k, ok.y + (end.y - ok.y) * k, ok.z + (end.z - ok.z) * k, 1.0f);
}

// Four tires as seen from above (front row first): tread surface temperature on a cold / optimal / hot colour, with
// the core temperature and the grip multiplier when not compact.
void tireTemperatureGrid(const VehicleTelemetry& t, bool compact) {
    const TireCompound& c = tireCompound(t.tireSet);
    const ImVec2 cell = compact ? ImVec2(44.0f, 18.0f) : ImVec2(118.0f, 38.0f);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImGui::BeginGroup();
    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 2; ++col) {
            const WheelTelemetry& w = t.wheels[std::size_t(row * 2 + col)];
            if (col > 0) ImGui::SameLine(0.0f, 3.0f);
            const ImVec2 a = ImGui::GetCursorScreenPos();
            ImGui::Dummy(cell);
            const ImVec2 b(a.x + cell.x, a.y + cell.y);
            const ImVec4 color = tireColor(compoundTemperatureState(c, w.tireSurfaceC));
            dl->AddRectFilled(a, b, ImGui::GetColorU32(ImVec4(color.x, color.y, color.z, 0.85f)), 3.0f);
            char text[64];
            if (compact) std::snprintf(text, sizeof(text), "%.0f", w.tireSurfaceC);
            else
                std::snprintf(text, sizeof(text), "%s %.0f C  x%.2f\ncore %.0f C %.0f kPa", kWheelNames[row * 2 + col],
                              w.tireSurfaceC, w.tireGrip, w.tireCoreC, w.tirePressureKPa);
            const ImVec2 size = ImGui::CalcTextSize(text);
            dl->AddText(ImVec2(a.x + 0.5f * (cell.x - size.x), a.y + 0.5f * (cell.y - size.y)), IM_COL32(255, 255, 255, 255), text);
        }
    }
    ImGui::EndGroup();
}

void bar(const char* label, float v, float lo, float hi, const char* fmt) {
    char text[64];
    std::snprintf(text, sizeof(text), fmt, v);
    ImGui::ProgressBar((v - lo) / (hi - lo), ImVec2(-80.0f, 0.0f), text);
    ImGui::SameLine();
    ImGui::TextUnformatted(label);
}

} // namespace

DebugUI::DebugUI(const VehicleConfig& initial) : m_tuning(initial.tuning), m_config(initial), m_seen(initial) {}

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

    if (menuOpen) return req;
    // The read-out with the key hints is visible with the F1 windows hidden (not with the debug windows setting off);
    // the player HUD is turned off only for clean screenshots.
    if (hud && debugWindows) {
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.45f);
        ImGui::Begin("##hud", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoInputs);
        const auto& t = sim.vehicle().telemetry();
        char g[8];
        if (t.ev.electric)
            ImGui::Text("%5.1f km/h  %s  %s  SoC %.1f %%  %+4.0f kW  motors %+4.0f / %+4.0f N*m", std::abs(t.forwardSpeed) * 3.6f,
                        t.direction == DriveDirection::Forward ? "D" : "R", evDriveModeName(t.ev.mode), 100.0f * t.ev.soc,
                        t.ev.motorPowerKW, t.ev.motorTorque[0], t.ev.motorTorque[1]);
        else
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
        if (stats.paused) ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1), "FROZEN (P to resume, N = single step)");
        if (sim.lastMessage()[0]) ImGui::TextDisabled("%s", sim.lastMessage());
        ImGui::TextDisabled("%s", driveHint.c_str());
        if (sim.vehicleConfig().isElectric())
            ImGui::TextDisabled("Q fwd/rev  V drive mode NORMAL/OFF-ROAD  B ABS  T traction control  X diff locks  R reset");
        else
            ImGui::TextDisabled("Q fwd/rev  L low range  V 2H/4H/4L  B ABS  T traction control  X diff locks  R reset");
        ImGui::TextDisabled("M map  C camera (follow / close / hood / free)  H lights  Esc menu  P freeze  F1 UI  F2 graphics  F3 audio");
        ImGui::TextDisabled("%.0f FPS  frame %.2f ms  CPU %.2f ms  GPU %s  %s %ux%u  v%s", stats.fps, stats.frameMs,
                            stats.cpuFrameMs, gpuText(stats.gpuFrameMs).c_str(), presetName(gfx.preset), stats.renderW,
                            stats.renderH, TRAILFORGE_VERSION);
        remember("hud", true);
        ImGui::End();
    }
    if (hud) {
        drawDriveHud(sim);
        drawLapHud(sim);
        drawDriftHud(sim);
    }

    if (!debugWindows) return req;
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
    const char* selector = t.gear == 0 && !t.ev.electric ? "N" : reverse ? "R" : "D";
    char gear[8];
    ImGui::SetNextWindowPos(ImVec2(0.5f * display.x, display.y - 16.0f), ImGuiCond_Always, ImVec2(0.5f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::Begin("##drivehud", nullptr, flags);
    remember("drive_hud", true);
    ImGui::PushFont(nullptr, 2.4f * ImGui::GetStyle().FontSizeBase);
    const ImVec4 selColor = reverse ? ImVec4(1.0f, 0.55f, 0.25f, 1) : t.gear == 0 && !t.ev.electric ? ImVec4(0.75f, 0.75f, 0.75f, 1)
                                                                                  : ImVec4(0.45f, 0.9f, 0.45f, 1);
    ImGui::TextColored(selColor, "%s", selector);
    ImGui::SameLine();
    ImGui::Text("%3.0f", std::abs(t.forwardSpeed) * 3.6f);
    ImGui::PopFont();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::TextUnformatted("km/h");
    if (t.ev.electric) ImGui::Text("single speed  %s", reverse ? "(reverse)" : "(forward)");
    else ImGui::Text("gear %s  %s", gearName(t.gear, gear, sizeof(gear)), reverse ? "(reverse)" : "(forward)");
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    if (t.ev.electric) {
        // Drive mode, battery state of charge and motor power (negative = regeneration, drawn in blue).
        if (t.ev.mode == EvDriveMode::OffRoad) ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1), "%s", evDriveModeName(t.ev.mode));
        else ImGui::TextDisabled("%s", evDriveModeName(t.ev.mode));
        ImGui::SameLine();
        char soc[24];
        std::snprintf(soc, sizeof(soc), "SoC %.1f %%", 100.0f * t.ev.soc);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, t.ev.soc < 0.1f ? ImVec4(0.85f, 0.3f, 0.2f, 1) : ImVec4(0.3f, 0.6f, 0.9f, 1));
        ImGui::ProgressBar(std::clamp(t.ev.soc, 0.0f, 1.0f), ImVec2(110, 0), soc);
        ImGui::PopStyleColor();
        if (t.ev.motorPowerKW < -0.5f) ImGui::TextColored(ImVec4(0.45f, 0.75f, 1.0f, 1), "%+4.0f kW regen", t.ev.motorPowerKW);
        else ImGui::TextDisabled("%+4.0f kW", t.ev.motorPowerKW);
        ImGui::SameLine();
        ImGui::TextDisabled("F %+4.0f  R %+4.0f N*m", t.ev.motorTorque[0], t.ev.motorTorque[1]);
    } else if (t.layout == DriveLayout::PartTime) {
        // Transfer case: 2H dim, 4H green, 4L amber.
        const char* mode = transferModeName(t.transfer);
        if (t.transfer == TransferMode::FourLow) ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1), "%s", mode);
        else if (t.transfer == TransferMode::FourHigh) ImGui::TextColored(ImVec4(0.45f, 0.9f, 0.45f, 1), "%s", mode);
        else ImGui::TextDisabled("%s", mode);
    } else {
        ImGui::TextDisabled("%s", driveLayoutName(t.layout));
        ImGui::SameLine();
        if (t.lowRange) ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1), "LOW");
        else ImGui::TextDisabled("HIGH");
    }
    if (!t.ev.electric) ImGui::TextDisabled("%4.0f rpm", t.engineRPM);
    ImGui::EndGroup();
    ImGui::SameLine();
    drawAidIndicators(sim);
    ImGui::SameLine(0.0f, 14.0f);
    ImGui::BeginGroup();
    ImGui::TextDisabled("%s tires", tireSetName(t.tireSet));
    tireTemperatureGrid(t, true);
    ImGui::EndGroup();
    remember("tire_hud");
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

namespace {

std::string lapClock(double t) {
    if (t <= 0.0) return "-:--.---";
    char buf[24];
    const int m = int(t / 60.0);
    std::snprintf(buf, sizeof(buf), "%d:%06.3f", m, t - 60.0 * m);
    return buf;
}

} // namespace

// Top-center lap timing on a level with a circuit: the running lap, the checkpoints passed, the last lap and the
// best (this session or best_laps.cfg, whichever is faster).
void DebugUI::drawLapHud(const Simulation& sim) {
    const LapTimer& lt = sim.lapTimer();
    m_lapHudBottom = lapHudTop - 8.0f;
    // On the proving ground (the mini handling track) only once a lap was started there.
    if (!lt.active() || (!sim.level() && !lt.running() && lt.laps() == 0)) return;
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                   ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoInputs;
    ImGui::SetNextWindowPos(ImVec2(lapHudX >= 0.0f ? lapHudX : 0.5f * display.x, lapHudTop), ImGuiCond_Always,
                            ImVec2(0.5f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::Begin("##laphud", nullptr, flags);
    remember("lap_hud", true);
    const int checkpoints = int(lt.gates().size()) - 1;
    ImGui::BeginGroup();
    if (lt.running()) ImGui::Text("LAP %d", lt.laps() + 1);
    else ImGui::TextDisabled("LAP");
    ImGui::TextDisabled("CP %d/%d", lt.checkpointsPassed(), checkpoints);
    ImGui::EndGroup();
    ImGui::SameLine(0.0f, 18.0f);
    ImGui::PushFont(nullptr, 2.0f * ImGui::GetStyle().FontSizeBase);
    if (lt.running()) ImGui::Text("%s", lapClock(lt.lapTime(sim.simTime())).c_str());
    else ImGui::TextDisabled("cross the start line");
    ImGui::PopFont();
    ImGui::SameLine(0.0f, 18.0f);
    ImGui::BeginGroup();
    ImGui::TextDisabled("LAST %s", lapClock(lt.lastLap()).c_str());
    double best = lt.bestLap();
    if (storedBestLap > 0.0 && (best <= 0.0 || storedBestLap < best)) best = storedBestLap;
    ImGui::TextColored(ImVec4(0.45f, 0.9f, 0.45f, 1), "BEST %s", lapClock(best).c_str());
    ImGui::EndGroup();
    // Sectors: this lap's finished sectors (white; purple when a session best), the last lap's otherwise (dim).
    if (lt.sectorCount() > 1) {
        for (int k = 0; k < lt.sectorCount(); ++k) {
            ImGui::SameLine(0.0f, 14.0f);
            ImGui::BeginGroup();
            const double now = lt.sectorTime(k), last = lt.lastSectorTime(k);
            ImGui::TextDisabled("S%d", k + 1);
            if (now > 0.0) {
                const bool bestSector = lt.bestSectorTime(k) <= 0.0 || now <= lt.bestSectorTime(k) + 1e-6;
                ImGui::TextColored(bestSector ? ImVec4(0.8f, 0.5f, 1.0f, 1) : ImVec4(1, 1, 1, 1), "%.3f", now);
            } else if (last > 0.0) {
                ImGui::TextDisabled("%.3f", last);
            } else {
                ImGui::TextDisabled("--.---");
            }
            ImGui::EndGroup();
        }
    }
    m_lapHudBottom = ImGui::GetWindowPos().y + ImGui::GetWindowSize().y;
    ImGui::End();
}

// Drift scoring panel below the lap panel (or the compass): body slip angle, speed, the running run's score and zone
// multiplier, the last run (or why it was lost) and the best (this session or best_drift.cfg, whichever is higher).
void DebugUI::drawDriftHud(const Simulation& sim) {
    const DriftScorer& d = sim.drift();
    if (!d.active() || !(d.inArea() || d.running())) return;
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                   ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoInputs;
    ImGui::SetNextWindowPos(ImVec2(lapHudX >= 0.0f ? lapHudX : 0.5f * display.x, m_lapHudBottom + 8.0f), ImGuiCond_Always,
                            ImVec2(0.5f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::Begin("##drifthud", nullptr, flags);
    remember("drift_hud", true);
    const ImVec4 hot(1.0f, 0.62f, 0.2f, 1.0f);
    ImGui::BeginGroup();
    if (d.drifting()) ImGui::TextColored(hot, "DRIFT");
    else ImGui::TextDisabled("DRIFT");
    ImGui::Text("%3.0f deg", double(std::abs(d.slipAngleDeg())));
    ImGui::EndGroup();
    ImGui::SameLine(0.0f, 14.0f);
    ImGui::BeginGroup();
    ImGui::TextDisabled("SPEED");
    ImGui::Text("%3.0f km/h", double(d.speedKmh()));
    ImGui::EndGroup();
    ImGui::SameLine(0.0f, 18.0f);
    ImGui::PushFont(nullptr, 2.0f * ImGui::GetStyle().FontSizeBase);
    if (d.running()) ImGui::TextColored(d.drifting() ? hot : ImVec4(1, 1, 1, 1), "%6.0f", d.score());
    else ImGui::TextDisabled("%6s", "-");
    ImGui::PopFont();
    if (d.running() && d.zone() >= 0) {
        ImGui::SameLine(0.0f, 10.0f);
        ImGui::BeginGroup();
        ImGui::TextColored(hot, "x%.1f", double(d.multiplier()));
        ImGui::TextDisabled("%s", d.config().zones[std::size_t(d.zone())].name.c_str());
        ImGui::EndGroup();
    }
    ImGui::SameLine(0.0f, 18.0f);
    ImGui::BeginGroup();
    const DriftScorer::End end = d.lastEnd();
    if (end == DriftScorer::End::Spin || end == DriftScorer::End::Wall)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.35f, 1), "LOST %s", end == DriftScorer::End::Spin ? "SPIN" : "WALL");
    else
        ImGui::TextDisabled("LAST %.0f", d.lastScore());
    const double best = std::max(d.bestScore(), storedBestDrift);
    ImGui::TextColored(ImVec4(0.45f, 0.9f, 0.45f, 1), "BEST %.0f", best);
    ImGui::EndGroup();
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

    ImGui::Text("CPU %.2f ms  GPU %s  frame %.2f ms (%.0f FPS)", stats.cpuFrameMs, gpuText(stats.gpuFrameMs).c_str(),
                stats.frameMs, stats.fps);
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
        edited |= ImGui::Checkbox("Far terrain shadows", &s.farShadows);
        edited |= ImGui::Checkbox("Puddle reflections", &s.reflections);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100.0f);
        edited |= ImGui::SliderFloat("##reflscale", &s.reflectionScale, 0.25f, 1.0f, "%.2fx res");
        {
            const char* probe[] = {"Sky only", "64 px", "128 px", "256 px"};
            int p = s.envProbe <= 0 ? 0 : s.envProbe <= 64 ? 1 : s.envProbe <= 128 ? 2 : 3;
            if (ImGui::Combo("Vehicle reflections", &p, probe, 4)) {
                s.envProbe = p == 0 ? 0 : 32 << p;
                edited = true;
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(100.0f);
            edited |= ImGui::SliderInt("##probefaces", &s.envProbeFaces, 1, 6, "%d faces/frame");
        }
        edited |= ImGui::SliderInt("Ground parallax steps", &s.parallaxSteps, 0, 32, s.parallaxSteps > 0 ? "%d" : "off");
        const char* tiling[] = {"Plain repeats", "Macro tone", "Hex tiling + macro tone"};
        edited |= ImGui::Combo("Ground texture tiling", &s.groundTiling, tiling, 3);
        edited |= ImGui::Checkbox("Ground detail normals", &s.groundDetail);
        edited |= ImGui::Checkbox("Wet ground near water", &s.groundWet);
        const char* grass[] = {"Off", "Sparse", "Medium", "Dense", "Very dense"};
        edited |= ImGui::Combo("Grass", &s.grassLevel, grass, 5);
        edited |= ImGui::SliderFloat("Grass distance [m]", &s.grassDistance, 10.0f, 250.0f, "%.0f");
        edited |= ImGui::SliderFloat("Tree detail distance [m]", &s.treeDetailDistance, 100.0f, 2000.0f, "%.0f");
        edited |= ImGui::SliderFloat("Tree distance [m]", &s.treeDistance, 500.0f, 6000.0f, "%.0f");
        edited |= ImGui::SliderFloat("Flowers", &s.flowerDensity, 0.0f, 1.0f, "%.2f");
        edited |= ImGui::Checkbox("Grass detail (awns, seed heads)", &s.grassDetail);
        edited |= ImGui::Checkbox("Bloom", &s.bloom);
        ImGui::SameLine();
        edited |= ImGui::Checkbox("Clouds", &s.clouds);
        edited |= ImGui::SliderInt("Spray particles", &s.sprayMax, 0, 8192);
        ImGui::SameLine();
        edited |= ImGui::Checkbox("Dust", &s.sprayDust);
        const int ts = s.terrainTextureSize;
        int texIdx = ts >= 4096 ? 4 : ts >= 2048 ? 3 : ts >= 1024 ? 2 : ts > 0 ? 1 : 0;
        const char* texSizes[] = {"Off (procedural)", "512", "1024", "2048", "4096"};
        if (ImGui::Combo("Level ground textures", &texIdx, texSizes, 5)) {
            s.terrainTextureSize = texIdx == 0 ? 0 : 256 << texIdx;
            edited = true;
        }
    }
    if (ImGui::CollapsingHeader("Post effects", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (s.msaa > 1)
            ImGui::TextDisabled("TAA, AO, motion blur, sun shafts and depth of field need MSAA off.");
        edited |= ImGui::Checkbox("TAA", &s.taa);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110.0f);
        gfx.changed |= ImGui::SliderFloat("Sharpen", &s.sharpen, 0.0f, 1.0f, "%.2f");
        edited |= ImGui::Checkbox("Water refraction", &s.waterRefraction);
        ImGui::SameLine();
        edited |= ImGui::Checkbox("Water SSR", &s.waterSsr);
        const char* aoNames[] = {"Off", "GTAO 2x4 (half res)", "GTAO 3x6 (half res)"};
        edited |= ImGui::Combo("Ambient occlusion", &s.ao, aoNames, 3);
        gfx.changed |= ImGui::SliderFloat("AO strength", &s.aoStrength, 0.0f, 3.0f, "%.2f");
        edited |= ImGui::SliderInt("Contact shadow steps", &s.contactShadows, 0, 16);
        edited |= ImGui::SliderFloat("Motion blur (shutter)", &s.motionBlur, 0.0f, 1.0f, "%.2f");
        edited |= ImGui::SliderFloat("Sun shafts", &s.sunShafts, 0.0f, 2.0f, "%.2f");
        edited |= ImGui::SliderFloat("Lens flare", &s.lensFlare, 0.0f, 2.0f, "%.2f");
        edited |= ImGui::Checkbox("Auto exposure", &s.autoExposure);
        ImGui::SameLine();
        ImGui::TextDisabled("%+.2f EV", stats.autoExposureEv);
        edited |= ImGui::Checkbox("Depth of field (menus, garage)", &s.dof);
        edited |= ImGui::SliderFloat("Depth of field while driving", &s.dofDriving, 0.0f, 1.0f, "%.2f");
        const char* curves[] = {"ACES (fitted)", "AgX (punchy)"};
        gfx.changed |= ImGui::Combo("Tonemap", &s.tonemapper, curves, 2);
        gfx.changed |= ImGui::SliderFloat("Colour grade", &s.grade, 0.0f, 1.0f, "%.2f");
        gfx.changed |= ImGui::SliderFloat("Vignette", &s.vignette, 0.0f, 1.0f, "%.2f");
        gfx.changed |= ImGui::SliderFloat("Film grain", &s.grain, 0.0f, 0.1f, "%.3f");
        gfx.changed |= ImGui::SliderFloat("Chromatic aberration", &s.chromaticAberration, 0.0f, 1.0f, "%.2f");
    }
    if (ImGui::CollapsingHeader("Look", ImGuiTreeNodeFlags_DefaultOpen)) {
        drawTimeOfDay(gfx);
        gfx.changed |= ImGui::SliderFloat("Exposure", &s.exposure, 0.3f, 3.0f, "%.2f");
        gfx.changed |= ImGui::SliderFloat("Sun path heading [deg]", &s.sunHeadingDeg, 0.0f, 360.0f, "%.0f");
        gfx.changed |= ImGui::SliderFloat("Fog density", &s.fogDensity, 0.0f, 4.0f, "%.2f");
        gfx.changed |= ImGui::SliderFloat("Valley fog", &s.valleyFog, 0.0f, 3.0f, "%.2f");
        gfx.changed |= ImGui::SliderFloat("Scattering sky", &s.atmosphere, 0.0f, 1.0f, "%.2f");
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
    static const char* kNames[kSurfaceCount] = {"Concrete", "Packed dirt", "Rock", "Low grip", "Mud",
                                                "Grass",    "Sand",        "Snow", "Asphalt",  "Gravel"};
    ImGui::TextUnformatted("Intensity per surface");
    for (int i = 0; i < int(kSurfaceCount); ++i) {
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
    if (!a.visible || !debugWindows) return;
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
    ImGui::SliderFloat("Music", &s.music, 0.0f, 1.0f, "%.2f");
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
    if (ImGui::CollapsingHeader("Electric drive (synthesized)", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Motor, gear and inverter", &s.motor, 0.0f, 3.0f, "%.2f");
        ImGui::Text("Motor order %.0f Hz, tone level %.4f, inverter %.4f, road hum %.2f", st.motorHz, st.motorLevel,
                    st.inverterLevel, st.road);
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
    ImGui::Text("Music %.0f %% (%.0f s played)", 100.0 * st.musicGain, st.musicSeconds);
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
                stats.cameraMode == 0 ? "follow (RMB / right stick orbit)"
                : stats.cameraMode == 1 ? "close follow (RMB / right stick orbit)"
                : stats.cameraMode == 2 ? "hood (RMB / right stick look)"
                                        : "free (arrows, PgUp/PgDn, RMB look)");
    if (ImGui::Button(stats.paused ? "Resume (P)" : "Freeze (P)")) req.togglePause = true;
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
    if (t.ev.electric) {
        const EvTelemetry& e = t.ev;
        ImGui::Text("Motors F %.0f rpm %+.0f N*m %+.0f kW   R %.0f rpm %+.0f N*m %+.0f kW   Direction %s", e.motorRPM[0],
                    e.motorTorque[0], e.motorPowerAxleKW[0], e.motorRPM[1], e.motorTorque[1], e.motorPowerAxleKW[1],
                    t.direction == DriveDirection::Forward ? "forward" : "reverse");
        ImGui::Text("Battery SoC %.2f %%  %+.1f kW (incl. aux)   used %.3f kWh, recovered %.3f kWh   mode %s, front share %.2f",
                    100.0f * e.soc, e.batteryPowerKW, e.energyOutKWh, e.energyInKWh, evDriveModeName(e.mode), e.frontShare);
        ImGui::Text("TC factor F %.2f R %.2f   regen allowance F %.2f R %.2f   friction brake F %.2f R %.2f%s", e.tcFactor[0],
                    e.tcFactor[1], e.regenFactor[0], e.regenFactor[1], e.frictionScale[0], e.frictionScale[1],
                    e.regenActive ? "   REGEN" : "");
    } else {
        ImGui::Text("Engine %.0f rpm   Gear %s%s   Clutch %.2f   Direction %s   Range %s", t.engineRPM,
                    gearName(t.gear, g, 8), t.shifting ? " (shifting)" : "", t.clutchFriction,
                    t.direction == DriveDirection::Forward ? "forward" : "reverse", t.lowRange ? "LOW" : "high");
    }
    ImGui::Text("Drivetrain %s%s%s   driven: %s%s   front share %.2f   drag %.0f N%s", driveLayoutName(t.layout),
                t.layout == DriveLayout::PartTime ? " " : "",
                t.layout == DriveLayout::PartTime ? transferModeName(t.transfer) : "", t.frontDriven ? "front " : "",
                t.rearDriven ? "rear" : "", t.frontTorqueShare, t.dragForce, t.revLimited ? "   REV LIMIT" : "");
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

    ImGui::SeparatorText("Tires");
    {
        const TireCompound& c = tireCompound(t.tireSet);
        ImGui::Text("Set %s   window %.0f..%.0f C   air %.0f C", tireSetName(t.tireSet), c.optimumLowC, c.optimumHighC,
                    t.ambientC);
        tireTemperatureGrid(t, false);
        remember("tire_temps");
        ImGui::TextDisabled("Surface = 2 mm tread layer, core = carcass (2-node model, docs/vehicles.md "
                            "\"Tire sets and temperature\").");
    }

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
            ImGui::ProgressBar(w.compressionRatio, ImVec2(-1, 0), w.rimHit ? "RIM" : w.bumpStop ? "SOLID" : nullptr);
        });
        row("Tire defl. [mm]", [](const WheelTelemetry& w) { ImGui::Text("%.0f", w.tireDeflection * 1000.0f); });
        row("Susp. speed [m/s]", [](const WheelTelemetry& w) { ImGui::Text("%+.2f", w.suspensionSpeed); });
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
        row("Tire surface / core [C]", [](const WheelTelemetry& w) { ImGui::Text("%.1f / %.1f", w.tireSurfaceC, w.tireCoreC); });
        row("Tire grip [x]", [](const WheelTelemetry& w) { ImGui::Text("%.3f", w.tireGrip); });
        row("Tire heat in / out [W]", [](const WheelTelemetry& w) { ImGui::Text("%.0f / %.0f", w.tireHeatW, w.tireCoolW); });
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

    // The garage (or a respawn with another configuration) changed the vehicle: edit its values from now on.
    if (!(sim.vehicleConfig() == m_seen)) {
        m_seen = sim.vehicleConfig();
        m_config = m_seen;
        m_tuning = m_seen.tuning;
    }
    bool changed = false;
    AxleTuning& fa = m_tuning.axles[kFrontAxle];
    AxleTuning& ra = m_tuning.axles[kRearAxle];
    if (ImGui::CollapsingHeader("Suspension (live)", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::SliderFloat("Front spring [Hz]", &fa.springFrequency, 0.5f, 3.0f, "%.2f");
        remember("front_spring");
        changed |= ImGui::SliderFloat("Rear spring [Hz]", &ra.springFrequency, 0.5f, 3.0f, "%.2f");
        changed |= ImGui::SliderFloat("Front compression [-]", &fa.compressionDamping, 0.05f, 1.5f, "%.2f");
        changed |= ImGui::SliderFloat("Front rebound [-]", &fa.reboundDamping, 0.05f, 1.5f, "%.2f");
        changed |= ImGui::SliderFloat("Rear compression [-]", &ra.compressionDamping, 0.05f, 1.5f, "%.2f");
        changed |= ImGui::SliderFloat("Rear rebound [-]", &ra.reboundDamping, 0.05f, 1.5f, "%.2f");
        changed |= ImGui::SliderFloat("Anti-roll front [N/m]", &fa.antiRoll, 0.0f, 40000.0f, "%.0f");
        changed |= ImGui::SliderFloat("Anti-roll rear [N/m]", &ra.antiRoll, 0.0f, 40000.0f, "%.0f");
        changed |= ImGui::SliderFloat("Damper knee [m/s]", &fa.damperKneeSpeed, 0.0f, 1.0f, "%.2f");
        ra.damperKneeSpeed = fa.damperKneeSpeed;
        ImGui::TextDisabled("Damping ratio per direction (and above the knee the\n"
                            "high-speed slope): Jolt has one coefficient, set inside\n"
                            "each step from that step's suspension speed.");
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
        float ambient = sim.ambientTemperature();
        if (ImGui::SliderFloat("Air temperature [C]", &ambient, -20.0f, 45.0f, "%.0f")) sim.setAmbientTemperature(ambient);
        if (ImGui::Button("Tires to air temperature")) sim.setTireTemperatures(ambient, ambient);
        ImGui::SameLine();
        if (ImGui::Button("Tires warm (window)")) {
            const TireCompound& c = tireCompound(sim.vehicleConfig().tireSet);
            const float warm = 0.5f * (std::max(c.optimumLowC, ambient) + c.optimumHighC);
            sim.setTireTemperatures(warm, warm);
        }
        ImGui::TextDisabled("Tire set: garage (rebuild). %s fitted.", tireSetName(sim.vehicleConfig().tireSet));
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
        const bool electric = sim.vehicleConfig().isElectric();
        if (electric) ImGui::TextDisabled("Electric: motor torque, regen and the OFF-ROAD split are set in the garage.");
        if (!electric) {
            changed |= ImGui::SliderFloat("Engine torque [N*m]", &m_tuning.engineMaxTorque, 100.0f, 1200.0f, "%.0f");
            changed |= ImGui::SliderFloat("Clutch strength", &m_tuning.clutchStrength, 1.0f, 40.0f, "%.1f");
            changed |= ImGui::Checkbox("Launch clutch limit", &m_tuning.launchClutchLimit);
            remember("clutch_limit");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Near min rpm the clutch passes at most the engine torque at this throttle.\n"
                                  "Off: the slipping clutch delivers ~840 N*m at any throttle.");
            changed |= ImGui::SliderFloat("Creep torque [N*m]", &m_tuning.creepTorque, 0.0f, 150.0f, "%.0f");
            changed |= ImGui::SliderFloat("Final drive", &m_tuning.finalDrive, 2.0f, 7.0f, "%.2f");
            changed |= ImGui::SliderFloat("Rev limit [rpm]", &m_tuning.revLimitRPM, 1000.0f, 12000.0f, "%.0f");
        }
        changed |= ImGui::SliderFloat("Front brake [N*m/wheel]", &fa.brakeTorque, 0.0f, 8000.0f, "%.0f");
        changed |= ImGui::SliderFloat("Rear brake [N*m/wheel]", &ra.brakeTorque, 0.0f, 8000.0f, "%.0f");
        changed |= ImGui::SliderFloat("Handbrake torque [N*m]", &ra.handBrakeTorque, 0.0f, 8000.0f, "%.0f");
        changed |= ImGui::SliderFloat("Front torque share", &m_tuning.frontTorqueShare, 0.05f, 0.95f, "%.2f");
        changed |= ImGui::SliderFloat("Drag area Cd*A [m^2]", &m_tuning.dragArea, 0.0f, 2.0f, "%.2f");
        changed |= ImGui::SliderFloat("Ackermann", &m_tuning.ackermann, 0.0f, 1.0f, "%.2f");
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
        changed |= ImGui::SliderFloat("Re-apply memory", &d.absReapplyFraction, 0.0f, 1.0f, "%.2f");
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
        for (AxleConfig& a : m_config.axles) {
            ImGui::PushID(&a);
            ImGui::SeparatorText(&a == &m_config.axles[0] ? "Front axle" : "Rear axle");
            ImGui::SliderFloat("Susp. min length [m]", &a.suspensionMinLength, 0.05f, 0.5f, "%.2f");
            ImGui::SliderFloat("Susp. max length [m]", &a.suspensionMaxLength, 0.2f, 0.9f, "%.2f");
            ImGui::SliderFloat("Wheel radius [m]", &a.wheelRadius, 0.3f, 0.6f, "%.2f");
            if (a.suspensionMaxLength <= a.suspensionMinLength + 0.02f)
                a.suspensionMaxLength = a.suspensionMinLength + 0.02f;
            ImGui::PopID();
        }
        ImGui::SliderFloat("Max steer [deg]", &m_config.axles[0].maxSteerAngleDeg, 15.0f, 45.0f, "%.0f");
        ImGui::SliderFloat("Low range ratio", &m_config.lowRangeRatio, 1.0f, 4.5f, "%.2f");
        int layout = int(m_config.layout);
        const char* layouts[] = {"RWD", "FWD", "AWD", "Part-time 4WD", "On-demand AWD"};
        ImGui::Combo("Layout", &layout, layouts, 5);
        m_config.layout = DriveLayout(layout);
        const bool rebuild = ImGui::Button("Rebuild vehicle (at current spawn)");
        remember("rebuild");
        if (rebuild) {
            VehicleConfig c = m_config;
            c.tuning = m_tuning;
            sim.requestRebuild(c);
        }
        ImGui::SameLine();
        if (ImGui::Button("Undo edits")) {
            m_config = sim.vehicleConfig();
            m_tuning = m_config.tuning;
        }
    }
    if (ImGui::CollapsingHeader("Simulation")) {
        const char* rates[] = {"60 Hz", "120 Hz", "240 Hz"};
        const double values[] = {60.0, 120.0, 240.0};
        if (ImGui::Combo("Fixed step", &m_rateIndex, rates, 3)) sim.setFixedStep(1.0 / values[m_rateIndex]);
        int cs = sim.collisionSteps();
        if (ImGui::SliderInt("Collision steps", &cs, 1, 4)) sim.setCollisionSteps(cs);
        InputSmoothing& sm = sim.smoothing();
        if (ImGui::SliderFloat("Steer rate [1/s]", &m_tuning.steerRate, 0.5f, 10.0f, "%.1f")) sim.requestTuning(m_tuning);
        if (ImGui::SliderFloat("Steer return [1/s]", &m_tuning.steerReturnRate, 0.5f, 10.0f, "%.1f"))
            sim.requestTuning(m_tuning);
        ImGui::SliderFloat("Pedal rise [1/s]", &sm.pedalRise, 0.5f, 20.0f, "%.1f");
        ImGui::SliderFloat("Pedal fall [1/s]", &sm.pedalFall, 0.5f, 20.0f, "%.1f");
        ImGui::SliderFloat("Brake rise [1/s]", &sm.brakeRise, 0.5f, 20.0f, "%.1f");
        ImGui::Checkbox("Auto reverse (S/W at a standstill)", &autoReverse);
        AutoReverseSettings& ar = sim.autoReverse();
        ImGui::SliderFloat("Auto reverse speed [m/s]", &ar.stopSpeed, 0.1f, 1.0f, "%.2f");
        ImGui::SliderFloat("Auto reverse hold [s]", &ar.holdTime, 0.0f, 1.0f, "%.2f");
        // Drift scoring: the world's setting (level.json "drift", the mini handling track) or forced on / off.
        int drift = !sim.driftOverride() ? 0 : *sim.driftOverride() ? 1 : 2;
        const char* driftModes[] = {"World setting", "On", "Off"};
        if (ImGui::Combo("Drift scoring", &drift, driftModes, 3))
            sim.setDriftOverride(drift == 0 ? std::nullopt : std::optional<bool>(drift == 1));
        remember("drift_scoring");
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
