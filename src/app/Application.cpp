#include "app/Application.h"

#include "app/AppIcon.h"
#include "app/Paths.h"

#include "render/GroundMaps.h"
#include "render/LevelScene.h"
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
#include <cctype>
#include <cstdlib>
#include <filesystem>
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

namespace {

// Scenarios and the demo drive start in gameplay so existing scripts keep working; `menu ...` commands reach the menus.
Screen initialScreen(const AppOptions& o) {
    if (!o.scriptPath.empty() || o.demoDrive) return Screen::Playing;
    return o.intro ? Screen::Intro : Screen::MainMenu;
}

UiSound uiSound(MenuSound s) {
    switch (s) {
    case MenuSound::Move: return UiSound::Hover;
    case MenuSound::Confirm: return UiSound::Confirm;
    case MenuSound::Back: return UiSound::Back;
    case MenuSound::Adjust: return UiSound::SliderTick;
    case MenuSound::Open: return UiSound::MenuOpen;
    case MenuSound::Close: return UiSound::MenuClose;
    default: return UiSound::Click;
    }
}

constexpr float kMenuTimeOfDay = 18.25f; // h: low warm sun for the intro and the start menu

struct NamedItem {
    const char* name;
    MenuItem item;
};
constexpr NamedItem kItemNames[] = {
    {"start", MenuItem::Start},         {"settings", MenuItem::Settings},     {"quit", MenuItem::Quit},
    {"resume", MenuItem::Resume},       {"restart", MenuItem::Restart},       {"quit_to_menu", MenuItem::QuitToMenu},
    {"garage", MenuItem::Garage},       {"quit_game", MenuItem::QuitGame},  {"quality", MenuItem::Quality},       {"render_scale", MenuItem::RenderScale},
    {"grass_distance", MenuItem::GrassDistance}, {"tree_distance", MenuItem::TreeDistance},
    {"vsync", MenuItem::VSync}, {"tree_style", MenuItem::TreeStyle},         {"master", MenuItem::MasterVolume},   {"music", MenuItem::MusicVolume},
    {"effects", MenuItem::EffectsVolume}, {"ui", MenuItem::UiVolume},         {"abs", MenuItem::Abs},
    {"tc", MenuItem::Traction},         {"diff", MenuItem::DiffLocks},        {"auto_reverse", MenuItem::AutoReverse},
    {"controls", MenuItem::Controls},   {"back", MenuItem::Back},             {"drive_keys", MenuItem::DriveKeys},
    {"pad_glyphs", MenuItem::PadGlyphs}, {"map", MenuItem::Map},        {"minimap", MenuItem::Minimap},
    {"minimap_size", MenuItem::MinimapSize}, {"minimap_orientation", MenuItem::MinimapNorthUp},
    {"compass", MenuItem::Compass},     {"time_of_day", MenuItem::TimeOfDay}, {"time_preset", MenuItem::TimePreset},
    {"time_flow", MenuItem::TimeFlow},  {"debug_windows", MenuItem::DebugWindows},
};

std::optional<MenuItem> itemByName(const std::string& n) {
    for (const NamedItem& i : kItemNames)
        if (n == i.name) return i.item;
    return std::nullopt;
}

// The vehicle catalog from the first asset directory that has one; the game falls back to the built-in
// configuration (identical to trailforge_suv) when no definition loads.
std::vector<VehicleDefinition> loadVehicles() {
    for (const std::string& dir : assetDirs("vehicles")) {
        std::error_code ec;
        if (!std::filesystem::is_directory(dir, ec)) continue;
        std::vector<std::string> errors;
        std::vector<VehicleDefinition> defs = loadVehicleCatalog(dir, errors);
        for (const std::string& e : errors) std::printf("Vehicles: %s\n", e.c_str());
        std::printf("Vehicles: %zu definition(s) from %s\n", defs.size(), dir.c_str());
        if (!defs.empty()) return defs;
    }
    std::printf("Vehicles: no definition loaded, using the built-in vehicle\n");
    return {};
}

SimulationSettings simulationSettings(const std::vector<VehicleDefinition>& vehicles) {
    SimulationSettings s;
    if (!vehicles.empty()) s.vehicle = vehicles.front().config;
    return s;
}

} // namespace

Application::Application(const AppOptions& options)
    : m_options(options),
      m_window("TrailForge " TRAILFORGE_VERSION, 1440, 900),
      m_vehicles(loadVehicles()),
      m_sim(simulationSettings(m_vehicles)),
      m_ui(m_sim.vehicleConfig()),
      m_flow(initialScreen(options)),
      m_clock(m_sim.fixedStep()) {
#if defined(NDEBUG)
    const bool debugDevice = false;
#else
    const bool debugDevice = true;
#endif
    m_renderer = std::make_unique<Renderer>(m_window.get(), debugDevice);
    m_imgui = std::make_unique<ImGuiContextGuard>(m_window.get(), *m_renderer);
    m_menu.load(*m_renderer, assetDirs("fonts"), assetDirs("images"));
    m_mapUI.setFonts(m_menu.bodyFont(), m_menu.headingFont());
    for (const std::string& dir : assetDirs("textures")) {
        std::error_code ec;
        if (!std::filesystem::exists(dir + "/terrain/terrain.json", ec)) continue;
        m_renderer->setTerrainTextureDir(dir + "/terrain");
        std::printf("Textures: terrain materials from %s/terrain\n", dir.c_str());
        break;
    }
    for (const std::string& dir : assetDirs("textures")) {
        std::error_code ec;
        if (!std::filesystem::exists(dir + "/buildings/buildings.json", ec)) continue;
        m_buildingTextureDir = dir + "/buildings";
        break;
    }
    m_ui.visible = options.startWithUi;

    const Uint64 t0 = SDL_GetPerformanceCounter();
    rebuildVehicleVisual();
    const MeshData& bodyMesh = m_vehicleVisual.mesh;
    buildWorldContent();
    m_armMesh = m_renderer->upload(buildUnitTubeMesh(0.021f));
    m_halfShaftMesh = m_renderer->upload(buildUnitTubeMesh(0.028f));
    m_shockBodyMesh = m_renderer->upload(buildShockBodyMesh());
    m_shockShaftMesh = m_renderer->upload(buildShockShaftMesh());
    m_gfx.settings = m_renderer->settings();
    std::printf("TrailForge: GPU driver '%s', shaders: %s, track triangles: %zu, render triangles: track %u, "
                "scenery %u, vehicle body %zu (%zu parts), scene build %.0f ms\n",
                m_renderer->driverName(), m_renderer->shaderFormatName(), m_sim.track().geometry().triangleCount(),
                m_trackMesh.vertexCount() / 3, m_sceneryMesh.vertexCount() / 3, bodyMesh.vertices.size() / 3,
                m_vehicleVisual.parts.size(), 1000.0 * double(SDL_GetPerformanceCounter() - t0) / double(SDL_GetPerformanceFrequency()));
    if (!options.scriptPath.empty()) {
        m_scenario = Scenario::load(options.scriptPath);
        if (!m_scenario) throw std::runtime_error("cannot load scenario " + options.scriptPath);
    }

    // Settings: loaded from (and saved to) the user data directory, except in scenario mode where every run starts
    // from the defaults and nothing is written.
    if (!m_scenario) {
        const std::string dir = userDataDir();
        if (!dir.empty()) m_settingsPath = dir + "settings.cfg";
        int skipped = 0;
        if (!m_settingsPath.empty() && loadSettingsFile(m_settingsPath, m_settings, &skipped))
            std::printf("Settings: loaded %s (%d lines skipped)\n", m_settingsPath.c_str(), skipped);
        else
            std::printf("Settings: defaults (%s)\n", m_settingsPath.empty() ? "no user data directory" : "no settings file yet");
    } else {
        m_settings.debugWindows = true; // scenario checks read and click the F1-F3 windows
    }

    if (m_scenario) {
        m_bestLapsPath = (std::filesystem::temp_directory_path() / "trailforge_scenario_best_laps.cfg").string();
        std::error_code ec;
        std::filesystem::remove(m_bestLapsPath, ec);
    } else if (const std::string dir = userDataDir(); !dir.empty()) {
        m_bestLapsPath = dir + "best_laps.cfg";
    }
    if (!m_bestLapsPath.empty() && m_bestLaps.load(m_bestLapsPath))
        std::printf("Best laps: %zu loaded from %s\n", m_bestLaps.size(), m_bestLapsPath.c_str());
    if (m_scenario) {
        m_bestDriftPath = (std::filesystem::temp_directory_path() / "trailforge_scenario_best_drift.cfg").string();
        std::error_code ec;
        std::filesystem::remove(m_bestDriftPath, ec);
    } else if (const std::string dir = userDataDir(); !dir.empty()) {
        m_bestDriftPath = dir + "best_drift.cfg";
    }
    if (!m_bestDriftPath.empty() && m_bestDrift.load(m_bestDriftPath))
        std::printf("Best drift scores: %zu loaded from %s\n", m_bestDrift.size(), m_bestDriftPath.c_str());

    // Garage setups: <user data>/setups/<vehicle id>.json; scenario runs use a scratch directory that starts empty.
    if (!m_vehicles.empty()) {
        std::string setupDir;
        if (m_scenario) {
            setupDir = (std::filesystem::temp_directory_path() / "trailforge_scenario_setups").string();
            std::error_code ec;
            std::filesystem::remove_all(setupDir, ec);
        } else if (const std::string dir = userDataDir(); !dir.empty()) {
            setupDir = dir + "setups";
        }
        m_garage = std::make_unique<Garage>(m_vehicles, setupDir);
        std::vector<std::string> ids;
        for (const VehicleDefinition& d : m_vehicles) ids.push_back(d.id);
        m_flow.setVehicles(ids);
        // --vehicle beats the remembered choice; an unknown id keeps the default vehicle.
        if (!options.vehicleId.empty()) m_settings.vehicle = options.vehicleId;
        if (!m_settings.vehicle.empty() && !m_garage->selectVehicle(m_settings.vehicle)) {
            std::printf("Vehicles: no vehicle '%s', using '%s'\n", m_settings.vehicle.c_str(), m_garage->vehicle().id.c_str());
            m_settings.vehicle.clear();
        }
        std::printf("Vehicles: active '%s'\n", m_garage->vehicle().id.c_str());
        for (const VehicleDefinition& d : m_vehicles)
            m_vehicleCards.push_back({d.id, d.name, d.description, vehicleSpecs(d.config), d.config.isElectric()});
        std::printf("Garage: setups in %s, active '%s'\n", setupDir.empty() ? "(memory only)" : setupDir.c_str(),
                    m_garage->factoryActive() ? "factory" : m_garage->activeName().c_str());
        if (!(m_garage->config() == m_sim.vehicleConfig())) m_sim.requestRebuild(m_garage->config());
        m_garage->markApplied();
    }

    // Levels: the assets/levels catalog, then the proving ground. Scenarios and the demo drive start on the proving
    // ground (their scripts are written for it) unless --level is given.
    for (const std::string& dir : assetDirs("levels")) {
        std::error_code ec;
        if (!std::filesystem::is_directory(dir, ec)) continue;
        std::vector<std::string> ids;
        for (const auto& e : std::filesystem::directory_iterator(dir, ec))
            if (std::filesystem::exists(e.path() / "level.json", ec)) ids.push_back(e.path().filename().string());
        std::sort(ids.begin(), ids.end());
        if (ids.empty()) continue;
        m_levelIds = ids;
        std::printf("Levels: %zu level(s) from %s\n", ids.size(), dir.c_str());
        break;
    }
    if (m_levelIds.empty()) std::printf("Levels: none found (proving ground only)\n");
    m_levelIds.push_back("proving_ground");
    m_flow.setLevels(m_levelIds);
    if (!options.levelId.empty()) m_settings.level = options.levelId;
    const std::string startLevel = !options.levelId.empty() ? options.levelId
                                   : (m_scenario || options.demoDrive) ? std::string("proving_ground")
                                                                       : m_settings.level;
    if (!selectLevel(startLevel)) {
        std::printf("Levels: no level '%s', using the proving ground\n", startLevel.c_str());
        m_settings.level.clear();
        selectLevel("proving_ground");
    }

    AudioOptions audio;
    audio.device = options.audio;
    audio.capturePath = options.audioCapturePath;
    audio.assetDirs = assetDirs("audio");
    for (const std::string& d : assetDirs("music")) audio.musicFiles.push_back(d + "/intro.mp3");
    m_audio = std::make_unique<AudioSystem>(audio);

    applySettings(true);
    m_gfx.settings.timeOfDay = m_settings.timeOfDay;
    m_gfx.changed = true;
    if (m_flow.menuActive()) enterMenuWorld(false);
    SDL_AddEventWatch(&Application::onAppLifecycle, this);
}

Application::~Application() {
    SDL_RemoveEventWatch(&Application::onAppLifecycle, this);
    m_audio.reset(); // stop the audio thread first
    if (m_virtualPad) SDL_CloseJoystick(m_virtualPad);
    if (m_virtualPadId) SDL_DetachVirtualJoystick(m_virtualPadId);
    // Mesh GPU resources are released before the device is destroyed.
    if (m_renderer) SDL_WaitForGPUIdle(m_renderer->device());
    m_menu.release();
    if (m_mapJob.valid()) m_mapJob.wait();
    m_mapUI.release();
    for (GpuMesh* m : {&m_trackMesh, &m_sceneryMesh, &m_waterMesh, &m_waterfallMesh, &m_bodyMesh, &m_wheelMesh, &m_rearWheelMesh, &m_springMesh,
                       &m_armMesh, &m_halfShaftMesh, &m_shockBodyMesh, &m_shockShaftMesh})
        m->release();
    m_imgui.reset();
    m_renderer.reset();
}

void Application::processEvents() {
    m_input.beginFrame();
    if (showsGameplayTime()) {
        // The TIME row and settings.cfg follow the live clock (time flow, the F2 time-lapse, scenario `time`).
        m_settings.timeOfDay = std::fmod(std::fmod(m_gfx.settings.timeOfDay, 24.0f) + 24.0f, 24.0f);
        m_applied.timeOfDay = m_settings.timeOfDay;
    }
    if (m_scenario) updateScenario();
    updateTouchControls();
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
    m_input.endEvents(io.WantCaptureKeyboard, io.WantCaptureMouse, float(std::min(m_frameDt, 0.1)));

    if (m_input.pressed(Action::Quit)) m_running = false;
    if (m_settings.debugWindows) {
        if (m_input.pressed(Action::ToggleDebugUI)) m_ui.visible = !m_ui.visible;
        if (m_input.pressed(Action::ToggleGraphicsUI)) m_gfx.visible = !m_gfx.visible;
        if (m_input.pressed(Action::ToggleAudioUI)) m_audioPanel.visible = !m_audioPanel.visible;
    }
    const bool mapHandled = m_flow.screen() == Screen::Map;
    if (m_flow.screen() == Screen::Garage) handleGarage();
    else if (mapHandled) handleMap();
    else handleMenu();
    // Behind the intro and the start menu the vehicle drives the attract route on its own pedals and steering; the
    // garage and the vehicle page hold it still for the preview.
    const Screen screen = m_flow.screen();
    AutopilotMode autopilot = AutopilotMode::Off;
    if (screen == Screen::Intro || screen == Screen::MainMenu) autopilot = AutopilotMode::Drive;
    if (screen == Screen::Garage || (screen == Screen::MainMenu && m_flow.page() == MenuPage::Vehicle))
        autopilot = AutopilotMode::Hold;
    if (m_scenarioAutopilot && screen == Screen::Playing) autopilot = AutopilotMode::Drive;
    m_sim.setAutopilot(autopilot);
    if (autopilot == AutopilotMode::Drive && screen != Screen::Playing) updateAttractDrive();
    if (m_flow.menuActive() || mapHandled) {
        // Menus and the map own the input (also in the frame that closes the map): no gameplay actions, no pedals.
        m_sim.setIntent(DriverIntent{});
        return;
    }
    if (m_input.pressed(Action::ToggleHeadlights)) m_gfx.headlights = !m_gfx.headlights;
    if (m_input.pressed(Action::TogglePause)) setPaused(!m_paused);
    if (m_input.pressed(Action::StepOnce) && m_paused) m_stepOnce = true;
    if (m_input.pressed(Action::ToggleCamera)) m_camera.cycleMode();
    if (m_input.pressed(Action::ToggleMap) && m_mapUI.hasMap()) {
        m_flow.openMap();
        openMap();
        m_audio->playUi(UiSound::MenuOpen);
        m_sim.setIntent(DriverIntent{});
        return;
    }
    if (m_input.pressed(Action::ToggleDirection)) m_sim.requestToggleDirection();
    if (m_input.pressed(Action::ToggleLowRange)) m_sim.requestToggleLowRange();
    if (m_input.pressed(Action::Reset)) m_sim.requestReset();
    if (m_input.pressed(Action::ToggleAbs)) m_sim.requestToggleAbs();
    if (m_input.pressed(Action::CycleTraction)) m_sim.requestCycleTraction();
    if (m_input.pressed(Action::CycleDiffLock)) m_sim.requestCycleDiffLock();
    if (m_input.pressed(Action::CycleTransfer)) m_sim.requestCycleTransfer();

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

// App lifecycle (iOS): an event watch sees the events while the OS waits in its callback, before the next frame.
// Losing focus opens the pause menu; in the background nothing is rendered (iOS ends apps that use the GPU there),
// the simulation does not tick and the audio device is stopped.
bool SDLCALL Application::onAppLifecycle(void* app, SDL_Event* event) {
    Application& a = *static_cast<Application*>(app);
    switch (event->type) {
    case SDL_EVENT_WILL_ENTER_BACKGROUND:
        if (!a.m_scenario) a.m_flow.pause();
        break;
    case SDL_EVENT_DID_ENTER_BACKGROUND:
        if (a.m_inBackground) break;
        a.m_inBackground = true;
        if (a.m_audio) a.m_audio->setSuspended(true);
        std::printf("App: entered the background (rendering, simulation and audio stopped)\n");
        break;
    case SDL_EVENT_WILL_ENTER_FOREGROUND:
        if (!a.m_inBackground) break;
        a.m_inBackground = false;
        a.m_clock.reset();
        if (a.m_audio) a.m_audio->setSuspended(false);
        std::printf("App: back in the foreground\n");
        break;
    default: break;
    }
    return true;
}

void Application::setPaused(bool paused) {
    if (paused == m_paused) return;
    m_paused = paused;
}

void Application::handleMenu() {
    const MenuKeys& k = m_input.menuKeys();
    MenuInput in;
    in.up = k.up;
    in.down = k.down;
    in.left = k.left;
    in.right = k.right;
    in.accept = k.accept;
    in.back = k.back;
    in.pause = k.pause;
    in.any = k.any;
    if (m_flow.menuActive() && m_flow.screen() != Screen::Intro) {
        int dir = 0;
        const bool click = k.touch ? k.tap : k.click;
        if (k.mouseMoved) in.hover = m_menu.hitTest(k.mouseX, k.mouseY, &dir);
        if (click) {
            in.click = m_menu.hitTest(k.mouseX, k.mouseY, &dir);
            in.clickDir = dir;
        }
        m_menu.drag(k.dragY, k.dragging);
        if (m_flow.page() == MenuPage::Vehicle) {
            if (click) in.pick = m_menu.vehicleCardHit(k.mouseX, k.mouseY);
            if (in.pick >= 0) in.click = -1;
            m_vehicleWheel += k.wheel;
            if (std::abs(m_vehicleWheel) >= 1.0f) {
                in.wheel = m_vehicleWheel > 0.0f ? 1 : -1; // one vehicle per notch (trackpads send fractions)
                m_vehicleWheel = 0.0f;
            }
        } else {
            m_vehicleWheel = 0.0f;
        }
        if (in.hover >= 0 || in.click >= 0) m_menu.notePointerCursor();
        if (k.wheel != 0.0f && m_flow.page() != MenuPage::Vehicle) m_menu.scroll(k.wheel);
    }
    const Screen before = m_flow.screen();
    const MenuResult r = m_flow.handle(in, m_settings);
    if (r.sound != MenuSound::None) m_audio->playUi(uiSound(r.sound));
    if (before == Screen::Intro && m_flow.screen() == Screen::MainMenu) m_audio->playUi(UiSound::Confirm);
    onMenuCommand(r.command);
}

void Application::onMenuCommand(MenuCommand c) {
    switch (c) {
    case MenuCommand::StartGame: startGameplay(); break;
    case MenuCommand::Restart: m_sim.requestReset(); break;
    case MenuCommand::QuitToMenu: enterMenuWorld(true); break;
    case MenuCommand::QuitGame:
        saveSettings();
        m_running = false;
        break;
    case MenuCommand::SettingsChanged: applySettings(false); break;
    case MenuCommand::SettingsClosed: saveSettings(); break;
    case MenuCommand::OpenGarage: openGarage(); break;
    case MenuCommand::OpenMap: openMap(); break;
    case MenuCommand::VehicleChanged:
        selectVehicle(m_settings.vehicle);
        saveSettings();
        break;
    case MenuCommand::LevelChanged:
        if (!selectLevel(m_settings.level)) std::printf("Levels: cannot load '%s'\n", m_settings.level.c_str());
        saveSettings();
        break;
    default: break;
    }
}

void Application::openGarage() {
    if (m_flow.screen() != Screen::Garage) m_flow.openGarage();
    if (m_flow.screen() != Screen::Garage) return;
    if (!m_garage) {
        m_flow.closeGarage();
        return;
    }
    m_garageCameraMode = m_camera.mode();
    m_garageCamTime = 0.0;
}

void Application::closeGarage() {
    SDL_StopTextInput(m_window.get());
    m_input.setModalCapture(false, false);
    m_flow.closeGarage();
    if (m_garageCameraMode != Camera::Mode::Free) m_camera.setMode(m_garageCameraMode);
    if (m_flow.screen() == Screen::MainMenu) m_menuCamTime = 0.0;
}

std::string Application::currentLevelId() const {
    return m_sim.level() ? m_sim.level()->id() : std::string("proving_ground");
}

std::string Application::currentVehicleId() const {
    return m_garage ? m_garage->vehicle().id : std::string("builtin");
}

// A lap completed with the player driving (or the scenario autopilot) goes into best_laps.cfg if it is the best.
void Application::updateBestLaps() {
    const LapTimer& lt = m_sim.lapTimer();
    if (lt.laps() > m_lapsSeen) {
        const bool counts = m_sim.autopilotMode() == AutopilotMode::Off || (m_scenario && m_scenarioAutopilot);
        if (counts && m_bestLaps.submit(currentLevelId(), currentVehicleId(), lt.lastLap())) {
            std::string error;
            if (!m_bestLapsPath.empty() && m_bestLaps.save(m_bestLapsPath, &error))
                std::printf("Best laps: %s / %s %.3f s saved to %s\n", currentLevelId().c_str(), currentVehicleId().c_str(),
                            lt.lastLap(), m_bestLapsPath.c_str());
            else if (!m_bestLapsPath.empty())
                std::printf("Best laps: %s\n", error.c_str());
        }
    }
    m_lapsSeen = lt.laps();
}

// A completed drift run with the player driving goes into best_drift.cfg if it is the best of this level and vehicle.
void Application::updateBestDrift() {
    const DriftScorer& d = m_sim.drift();
    if (d.runs() < m_driftRunsSeen) m_driftRunsSeen = 0; // the session was cleared (another world or vehicle)
    if (d.runs() > m_driftRunsSeen && m_sim.autopilotMode() == AutopilotMode::Off &&
        m_bestDrift.submit(currentLevelId(), currentVehicleId(), d.lastScore())) {
        std::string error;
        if (!m_bestDriftPath.empty() && m_bestDrift.save(m_bestDriftPath, &error))
            std::printf("Best drift: %s / %s %.0f pts saved to %s\n", currentLevelId().c_str(), currentVehicleId().c_str(),
                        d.lastScore(), m_bestDriftPath.c_str());
        else if (!m_bestDriftPath.empty())
            std::printf("Best drift: %s\n", error.c_str());
    }
    m_driftRunsSeen = d.runs();
}

bool Application::selectVehicle(const std::string& id) {
    if (!m_garage || !m_garage->selectVehicle(id)) return false;
    m_settings.vehicle = m_garage->vehicle().id;
    applyGarageConfig();
    return true;
}

// Live values go into the running vehicle between ticks; geometry, wheels and layout rebuild it in place
// (Simulation::requestVehicleConfig decides).
void Application::applyGarageConfig() {
    if (!m_garage || !m_garage->needsApply()) return;
    m_sim.requestVehicleConfig(m_garage->config());
    m_garage->markApplied();
}

void Application::handleGarage() {
    const MenuKeys& k = m_input.menuKeys();
    GarageInput in;
    in.up = k.up;
    in.down = k.down;
    in.left = k.left;
    in.right = k.right;
    in.accept = k.accept;
    in.back = k.back;
    in.text = k.text;
    in.backspace = k.backspace;
    if (m_garage->modal() != GarageModal::None) {
        // The modal owns the pointer too: only its buttons react.
        if (k.mouseMoved) in.modalHover = m_menu.modalHitTest(k.mouseX, k.mouseY);
        if (k.click) in.modalClick = m_menu.modalHitTest(k.mouseX, k.mouseY);
    } else {
        int dir = 0;
        float fraction = -1.0f;
        // A finger taps rows and scrolls the list (no slider drags); the mouse also drags sliders.
        const bool click = k.touch ? k.tap : k.click;
        if (k.mouseMoved && !k.mouseDown) in.hover = m_menu.garageHitTest(k.mouseX, k.mouseY, &dir, &fraction);
        if (click || (k.mouseDown && k.mouseMoved)) {
            const int row = m_menu.garageHitTest(k.mouseX, k.mouseY, &dir, &fraction);
            // Dragging only moves sliders; a plain move with the button held does not activate rows.
            if (click || (fraction >= 0.0f && row == m_garage->cursor())) {
                in.click = row;
                in.clickDir = dir;
                in.clickFraction = fraction;
            }
        }
        if (in.hover >= 0 || in.click >= 0) m_menu.notePointerCursor();
        if (k.wheel != 0.0f) m_menu.scroll(k.wheel);
        m_menu.drag(k.dragY, k.dragging);
    }
    const bool wasRenaming = m_garage->renaming();
    const GarageResult r = m_garage->handle(in);
    if (m_garage->renaming() != wasRenaming) {
        if (m_garage->renaming()) SDL_StartTextInput(m_window.get());
        else SDL_StopTextInput(m_window.get());
    }
    m_input.setModalCapture(m_garage->modal() != GarageModal::None, m_garage->renaming());
    switch (r.sound) {
    case GarageSound::Move: m_audio->playUi(UiSound::Hover); break;
    case GarageSound::Confirm: m_audio->playUi(UiSound::Confirm); break;
    case GarageSound::Back: m_audio->playUi(UiSound::Back); break;
    case GarageSound::Adjust: m_audio->playUi(UiSound::SliderTick); break;
    case GarageSound::Error: m_audio->playUi(UiSound::Back); break;
    case GarageSound::None: break;
    }
    applyGarageConfig();
    if (m_garage->vehicle().id != m_flow.vehicles()[std::size_t(m_flow.vehicleIndex(m_settings))]) {
        m_settings.vehicle = m_garage->vehicle().id; // the vehicle row of the garage also sets the remembered choice
        saveSettings();
    }
    if (r.close) closeGarage();
}

// Start: a fresh run at the start of the current proving ground with the current vehicle (level and vehicle
// selection will come before this), the driving aid defaults from the settings, the follow camera, and the gameplay
// time of day. The music fades out.
void Application::startGameplay() {
    m_sim.requestTeleport(0); // the spawn, wherever the attract drive left the vehicle
    AidControls aids;
    aids.abs = m_settings.abs;
    aids.traction = TractionMode(std::max(m_settings.traction, 0));
    aids.diffLocks = m_settings.diffLocks;
    m_sim.requestAidControls(aids);
    if (m_settings.traction < 0) m_sim.requestDefaultTraction();
    m_ui.autoReverse = m_settings.autoReverse;
    if (m_camera.mode() == Camera::Mode::Free) m_camera.setMode(Camera::Mode::Follow);
    m_gfx.settings.timeOfDay = m_settings.timeOfDay;
    m_gfx.changed = true;
    m_musicTarget = 0.0f;
    m_musicFade = 3.0f;
}

// Main menu (or intro) world: the vehicle back at the start, a sunset, the cinematic camera, the music from the top.
void Application::enterMenuWorld(bool restartMusic) {
    if (restartMusic) {
        ++m_musicRestart;
        m_sim.requestTeleport(0);
        m_attractStartPending = true;
        m_attractStartLevel = m_sim.level();
    }
    m_gfx.settings.timeOfDay = kMenuTimeOfDay;
    m_gfx.timeLapse = false;
    m_gfx.changed = true;
    m_musicTarget = 1.0f;
    m_musicFade = restartMusic ? 1.0f : 0.3f;
    m_menuCamTime = 0.0;
}

void Application::applySettings(bool force) {
    const GameSettings& s = m_settings;
    if (force || s.quality != m_applied.quality || s.renderScale != m_applied.renderScale ||
        s.grassDistance != m_applied.grassDistance || s.treeDistance != m_applied.treeDistance) {
        const QualityPreset preset = QualityPreset(s.quality);
        const RenderSettings p = presetSettings(preset);
        RenderSettings r = p;
        copyLook(r, m_gfx.settings);
        if (s.renderScale > 0.0f) r.renderScale = s.renderScale;
        if (s.grassDistance > 0.0f) r.grassDistance = s.grassDistance;
        if (s.treeDistance > 0.0f) r.treeDetailDistance = s.treeDistance;
        const bool own = r.renderScale == p.renderScale && r.grassDistance == p.grassDistance &&
                         r.treeDetailDistance == p.treeDetailDistance;
        m_gfx.preset = own ? preset : QualityPreset::Custom;
        if (!force || !own || m_gfx.preset != QualityPreset::High) {
            m_gfx.settings = r;
            m_gfx.changed = true;
        }
    }
    if (force || s.vsync != m_applied.vsync) m_renderer->setVSync(s.vsync);
    if (!force && s.stylizedTrees != m_applied.stylizedTrees && m_sim.level()) {
        // The tree style is part of the bake: the current level is baked again (as a level change; the vehicle
        // goes back to the spawn).
        const std::string id = currentLevelId();
        m_levelCache.clear(); // every cached level was baked with the other style
        if (!selectLevel(id)) std::printf("Levels: cannot load '%s'\n", id.c_str());
    }
    AudioSettings& a = m_audioPanel.settings;
    a.master = s.masterVolume;
    a.music = s.musicVolume;
    a.effects = s.effectsVolume;
    a.ui = s.uiVolume;
    if (force || s.abs != m_applied.abs || s.traction != m_applied.traction || s.diffLocks != m_applied.diffLocks) {
        AidControls aids = m_sim.aidControls();
        aids.abs = s.abs;
        if (s.traction >= 0) aids.traction = TractionMode(s.traction);
        aids.diffLocks = s.diffLocks;
        m_sim.requestAidControls(aids);
        if (s.traction < 0 && (force || s.traction != m_applied.traction)) m_sim.requestDefaultTraction();
    }
    if (force || s.autoReverse != m_applied.autoReverse) m_ui.autoReverse = s.autoReverse;
    if (!force && s.timeOfDay != m_applied.timeOfDay && showsGameplayTime()) {
        m_gfx.settings.timeOfDay = s.timeOfDay;
        m_gfx.changed = true;
    }
    m_ui.debugWindows = s.debugWindows;
    m_input.setDriveKeys(DriveKeys(s.driveKeys));
    m_ui.driveHint = driveHint(DriveKeys(s.driveKeys), keyboardStyle());
    m_applied = s;
}

bool Application::showsGameplayTime() const {
    switch (m_flow.screen()) {
    case Screen::Playing:
    case Screen::Paused:
    case Screen::Map: return true;
    case Screen::Garage: return m_flow.garageReturn() == Screen::Paused;
    default: return false;
    }
}

void Application::saveSettings() {
    if (m_settingsPath.empty()) return;
    std::string error;
    if (saveSettingsFile(m_settingsPath, m_settings, &error)) std::printf("Settings: saved %s\n", m_settingsPath.c_str());
    else std::printf("Settings: %s\n", error.c_str());
}

// Slow cinematic camera for the intro and the start menu while the vehicle drives the attract route: a lazy chase
// that swings between a rear and a front three-quarter view (the vehicle right of the menu column). The heading
// it follows and its position are smoothed over seconds, so curves and bumps turn into slow drifts. Read-only like
// the follow camera; a ray keeps it above the ground.
void Application::updateMenuCamera(float dt, const glm::mat4& body) {
    const bool fresh = m_menuCamTime == 0.0;
    m_menuCamTime += dt;
    const double t = m_menuCamTime;
    const glm::vec3 car(body[3]);
    const glm::vec3 fwd3(body[2]);
    if (std::hypot(fwd3.x, fwd3.z) > 0.2f) {
        const float heading = std::atan2(fwd3.x, fwd3.z);
        if (fresh) m_menuCamHeading = heading;
        float d = heading - m_menuCamHeading;
        d = std::atan2(std::sin(d), std::cos(d));
        m_menuCamHeading += d * (1.0f - std::exp(-dt / 3.5f));
    }
    // Behind the vehicle, swinging out to its right side and back over about 80 s.
    const float swing = 0.5f + 0.9f * float(0.5 - 0.5 * std::cos(t * 6.2831853 / 80.0));
    const float yaw = m_menuCamHeading + 3.14159265f - swing;
    const float radius = 9.0f + 1.5f * float(std::sin(t * 6.2831853 / 47.0 + 1.0));
    const float height = 1.8f + 0.5f * float(std::sin(t * 6.2831853 / 39.0));
    glm::vec3 pos = car + glm::vec3(std::sin(yaw) * radius, height, std::cos(yaw) * radius);
    const JPH::Vec3 above(pos.x, pos.y + 20.0f, pos.z), below(pos.x, pos.y - 20.0f, pos.z);
    if (auto hit = m_sim.castRayIgnoringVehicle(above, below)) {
        const float ground = above.GetY() + (below.GetY() - above.GetY()) * *hit;
        pos.y = std::max(pos.y, ground + 1.0f);
    }
    // The offset from the vehicle is smoothed, not the world position: the camera keeps up at any speed.
    const glm::vec3 offset = pos - car;
    m_menuCamOffset = fresh ? offset : glm::mix(m_menuCamOffset, offset, 1.0f - std::exp(-dt / 0.6f));
    pos = car + m_menuCamOffset;
    const glm::vec3 fwd = glm::normalize(glm::vec3(car.x - pos.x, 0.0f, car.z - pos.z));
    const glm::vec3 right = glm::normalize(glm::cross(fwd, glm::vec3(0, 1, 0)));
    const glm::vec3 target = car + glm::vec3(0.0f, 0.75f, 0.0f) - right * 2.6f;
    m_camera.setFreePose(pos, target);
}

// Attract drive behind the intro and the start menu. On a level the vehicle starts at the route start facing along
// it (the spawn is where the player starts, not where the route begins), and it drives in its most sure-footed road
// mode: 4H on a part-time transfer case, ABS and traction control on, no lockers. Start puts the vehicle on the
// spawn (a new vehicle: its default transfer mode) and applies the aids of the settings again.
void Application::updateAttractDrive() {
    if (m_attractStartPending && m_sim.level() == m_attractStartLevel) {
        m_attractStartPending = false;
        if (m_sim.level() && m_sim.autopilot().hasRoute()) {
            const RoutePoint s = m_sim.autopilot().startPoint();
            TrackFeature at;
            at.name = "route start";
            at.position = JPH::Vec3(s.x, m_sim.level()->heightAt(s.x, s.z), s.z);
            at.yaw = m_sim.autopilot().startYaw();
            m_sim.requestPlace(at);
        }
    }
    const VehicleTelemetry& t = m_sim.vehicle().telemetry();
    if (t.layout == DriveLayout::PartTime && t.transfer == TransferMode::TwoHigh) m_sim.requestTransfer(TransferMode::FourHigh);
    const AidControls aids = m_sim.aidControls();
    if (!aids.abs || aids.traction != TractionMode::On || aids.diffLocks != 0) {
        AidControls attract;
        attract.abs = true;
        attract.traction = TractionMode::On;
        m_sim.requestAidControls(attract);
    }
}

// Garage camera: a slow orbit around the vehicle, which sits right of the panel.
void Application::updateGarageCamera(float dt) {
    m_garageCamTime += dt;
    const glm::vec3 car = toGlm(m_sim.currentSnapshot().body.position);
    const glm::quat rot = toGlm(m_sim.currentSnapshot().body.rotation);
    const glm::vec3 fwd3 = rot * glm::vec3(0, 0, 1);
    const float heading = std::atan2(fwd3.x, fwd3.z);
    // Front three-quarter view to start, then a slow full turn.
    const float yaw = heading + 0.75f + float(m_garageCamTime) * 6.2831853f / 90.0f;
    const float radius = 6.2f;
    glm::vec3 pos = car + glm::vec3(std::sin(yaw) * radius, 1.5f, std::cos(yaw) * radius);
    const JPH::Vec3 above(pos.x, pos.y + 20.0f, pos.z), below(pos.x, pos.y - 20.0f, pos.z);
    if (auto hit = m_sim.castRayIgnoringVehicle(above, below)) {
        const float ground = above.GetY() + (below.GetY() - above.GetY()) * *hit;
        pos.y = std::max(pos.y, ground + 0.8f);
    }
    const glm::vec3 look = glm::normalize(glm::vec3(car.x - pos.x, 0.0f, car.z - pos.z));
    const glm::vec3 right = glm::normalize(glm::cross(look, glm::vec3(0, 1, 0)));
    m_camera.setFreePose(pos, car + glm::vec3(0.0f, 0.45f, 0.0f) - right * 2.2f);
}

std::shared_ptr<const Level> Application::loadLevel(const std::string& id) {
    if (const auto it = m_levelCache.find(id); it != m_levelCache.end()) return it->second;
    for (const std::string& dir : assetDirs("levels")) {
        const std::string path = dir + "/" + id + "/level.json";
        std::error_code ec;
        if (!std::filesystem::exists(path, ec)) continue;
        std::string error;
        std::shared_ptr<const Level> level = Level::load(path, &error, 0, !m_settings.stylizedTrees);
        if (!level) {
            std::printf("Levels: %s\n", error.c_str());
            return nullptr;
        }
        const LevelBakeStats& st = level->bakeStats();
        std::printf("Levels: baked '%s' in %.0f ms (%d^2 samples, heights %.1f..%.1f m, %zu trees, %zu rocks, "
                    "%zu prop triangles)\n", id.c_str(), st.totalMs, st.samples, double(st.minHeight),
                    double(st.maxHeight), st.trees, st.rocks, st.propTriangles);
        m_levelCache[id] = level;
        return level;
    }
    return nullptr;
}

bool Application::selectLevel(const std::string& requested) {
    const std::string id = requested.empty() ? m_levelIds.front() : requested;
    if (id == "proving_ground") {
        m_sim.requestLevel(nullptr);
        m_attractStartPending = false;
        return true;
    }
    if (std::find(m_levelIds.begin(), m_levelIds.end(), id) == m_levelIds.end()) return false;
    std::shared_ptr<const Level> level = loadLevel(id);
    if (!level) return false;
    m_attractStartPending = true;
    m_attractStartLevel = level.get();
    m_sim.requestLevel(std::move(level));
    return true;
}

void Application::buildWorldContent() {
    const Uint64 t0 = SDL_GetPerformanceCounter();
    SDL_WaitForGPUIdle(m_renderer->device());
    for (GpuMesh* m : {&m_trackMesh, &m_sceneryMesh, &m_waterMesh, &m_waterfallMesh}) m->release();
    const Level* level = m_sim.level();
    m_contentLevel = level;
    m_contentBuilt = true;
    startMapBuild();
    std::size_t trees = 0;
    if (level) {
        m_trackMesh = m_renderer->upload(buildLevelPropsMesh(*level));
        m_waterMesh = m_renderer->upload(buildLevelWaterMesh(*level));
        if (const MeshData falls = buildLevelWaterfallMesh(*level); !falls.vertices.empty())
            m_waterfallMesh = m_renderer->upload(falls);
        m_sceneryMesh = m_renderer->upload(buildLevelHorizonMesh(*level));
        m_renderer->setGround(buildLevelGroundMaps(*level), 0.0f);
        TerrainSource src;
        src.samples = level->samples();
        src.origin = level->origin();
        src.cell = level->cell();
        src.heights = level->heights().data();
        src.splat = level->splatTexels().data();
        src.asphalt = level->asphaltLayer();
        m_renderer->setTerrain(&src);
        std::vector<std::string> modelPaths;
        std::vector<ModelPlacement> placements;
        buildLevelModelPlacements(*level, modelPaths, placements);
        const std::vector<ModelAsset> kit = buildLevelBuildingModels(*level);
        std::vector<const ModelAsset*> kitPtrs;
        for (const ModelAsset& a : kit) kitPtrs.push_back(&a);
        appendLevelBuildingPlacements(*level, int(modelPaths.size()), placements);
        const int models = m_renderer->setModels(modelPaths, placements, kitPtrs, m_buildingTextureDir);
        if (models < int(modelPaths.size()))
            std::printf("Levels: %d of %zu level models loaded (see the log)\n", models, modelPaths.size());
        if (!level->modelError().empty())
            std::printf("Levels: model library unavailable (%s); procedural trees and rocks\n",
                        level->modelError().c_str());
        const std::vector<TreeInstance> inst = buildLevelTreeInstances(*level);
        trees = inst.size() + placements.size();
        m_renderer->setTrees(buildPineMeshes(), inst);
        m_camera.farPlane = 1.1f * kHorizonSea; // the sea reaches the horizon
    } else {
        m_trackMesh = m_renderer->upload(buildTrackMesh(m_sim.track().geometry(), m_sim.surfaces()));
        m_sceneryMesh = m_renderer->upload(buildSceneryMesh());
        m_waterMesh = m_renderer->upload(buildWaterMesh());
        m_renderer->setTerrain(nullptr);
        m_renderer->setGround(buildGroundMaps(m_sim.track()), TestTrack::water().level);
        m_renderer->setModels({}, {});
        const std::vector<TreeInstance> inst = buildTreeInstances();
        trees = inst.size();
        m_renderer->setTrees(buildPineMeshes(), inst);
        m_camera.farPlane = Camera{}.farPlane;
    }
    std::printf("World: %s, render triangles: props/track %u, scenery/horizon %u, water %u, trees + models %zu, build %.0f ms\n",
                level ? level->name().c_str() : "Proving Ground", m_trackMesh.vertexCount() / 3,
                m_sceneryMesh.vertexCount() / 3, m_waterMesh.vertexCount() / 3, trees,
                1000.0 * double(SDL_GetPerformanceCounter() - t0) / double(SDL_GetPerformanceFrequency()));
}

// Map of the current world, made on a worker thread from the level (or the proving ground triangles). The level
// is kept alive by the cache entry the lambda holds.
void Application::startMapBuild() {
    if (m_mapJob.valid()) m_mapJob.wait();
    m_mapJob = {};
    m_marker = {};
    m_mapViewSet = false;
    if (const Level* level = m_sim.level()) {
        std::shared_ptr<const Level> keep;
        for (const auto& [id, l] : m_levelCache)
            if (l.get() == level) keep = l;
        if (!keep) return;
        m_mapJob = std::async(std::launch::async, [keep] { return buildLevelMap(*keep); });
        return;
    }
    if (m_sim.track().layout() != TrackLayout::ProvingGround) return;
    ProvingGroundMapInput in;
    const TrackBounds b = TestTrack::worldBounds();
    in.x0 = b.xMin - 4.0f;
    in.x1 = b.xMax + 4.0f;
    in.z0 = b.zMin - 4.0f;
    in.z1 = b.zMax + 4.0f;
    const WaterInfo w = TestTrack::water();
    in.waterLevel = w.level;
    in.waterX0 = w.x0;
    in.waterZ0 = w.z0;
    in.waterX1 = w.x1;
    in.waterZ1 = w.z1;
    const std::vector<TrackFeature>& features = m_sim.track().features();
    for (std::size_t k = 0; k < features.size(); ++k)
        in.pois.push_back({int(k) + 1, features[k].name, {}, features[k].position.GetX(), features[k].position.GetZ()});
    m_mapJob = std::async(std::launch::async, [in, geometry = m_sim.track().geometry()]() mutable {
        in.geometry = &geometry;
        return buildProvingGroundMap(in);
    });
}

void Application::pollMapBuild() {
    if (!m_mapJob.valid() || m_mapJob.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    MapData d = m_mapJob.get();
    m_mapBuildMs = d.buildMs;
    std::printf("Map: %s raster %d^2 (%.2f m/px), %zu lines, %zu points of interest, built in %.0f ms on a worker thread\n",
                d.title.c_str(), d.raster.size, double(d.raster.metersPerPixel), d.lines.size(), d.pois.size(), d.buildMs);
    m_mapUI.setMap(*m_renderer, std::move(d));
}

MapPlayer Application::mapPlayer() const {
    const RenderSnapshot& s = m_sim.currentSnapshot();
    const glm::vec3 p = toGlm(s.body.position);
    const glm::vec3 f = toGlm(s.body.rotation) * glm::vec3(0, 0, 1);
    return {p.x, p.z, bearingDeg(f.x, f.z)};
}

MapHints Application::mapHints() const {
    MapHints h;
    h.gamepad = m_input.gamepadConnected();
    if (h.gamepad) {
        const PadGlyphs g = resolvePadGlyphs(PadGlyphSetting(m_settings.padGlyphs), padType());
        h.zoom = std::string(padControlName(PadControl::LeftTrigger, g)) + " / " + padControlName(PadControl::RightTrigger, g);
        h.pan = padControlName(PadControl::LeftStick, g);
        h.marker = padControlName(PadControl::South, g);
        h.center = padControlName(PadControl::North, g);
        h.close = std::string(padControlName(PadControl::East, g)) + " / " + padControlName(PadControl::LeftShoulder, g);
    } else {
        h.zoom = "WHEEL  Q / E";
        h.pan = "DRAG  W A S D";
        h.marker = "CLICK  SPACE";
        h.center = "C";
        h.close = "M  ESC";
    }
    return h;
}

void Application::openMap() {
    if (m_flow.screen() != Screen::Map) return;
    const ImVec2 d = ImGui::GetIO().DisplaySize;
    const MapUI::Layout l = MapUI::layout(d.x, d.y);
    if (!m_mapViewSet) {
        m_mapView = m_mapUI.fitView(l);
        m_mapViewSet = true;
    }
    m_mapDragging = false;
    m_mapCrosshair = m_input.gamepadConnected();
    m_mapHoverPoi = -1;
}

void Application::closeMap() {
    m_flow.closeMap();
    m_mapDragging = false;
    m_audio->playUi(UiSound::MenuClose);
}

// Full-screen map input: zoom (wheel around the pointer, Q / E, triggers), pan (drag, W A S D / arrows, left stick),
// waypoint (click, Space / Enter / pad A at the crosshair; on a point of interest it snaps to it; again on the
// marker clears it), C / pad Y centers on the vehicle, M / Esc / Backspace / pad B / LB close. No teleports.
void Application::handleMap() {
    const MenuKeys& k = m_input.menuKeys();
    if (k.back || k.pause || m_input.pressed(Action::ToggleMap)) {
        closeMap();
        return;
    }
    const ImVec2 D = ImGui::GetIO().DisplaySize;
    const MapUI::Layout l = MapUI::layout(D.x, D.y);
    // The view keeps its world center in the middle of the panel when the window changes size.
    m_mapView.sx = 0.5f * (l.mapX0 + l.mapX1);
    m_mapView.sy = 0.5f * (l.mapY0 + l.mapY1);
    const float dt = float(std::min(m_frameDt, 0.1));
    const float minS = m_mapUI.minScale(l), maxS = m_mapUI.maxScale();
    auto inPanel = [&](float x, float y) { return x >= l.mapX0 && x <= l.mapX1 && y >= l.mapY0 && y <= l.mapY1; };
    auto held = [&](SDL_Scancode a, SDL_Scancode b) { return m_input.scancodeHeld(a) || m_input.scancodeHeld(b) ? 1.0f : 0.0f; };

    if (k.wheel != 0.0f && inPanel(k.mouseX, k.mouseY))
        m_mapView.zoomAround(std::pow(1.2f, k.wheel), k.mouseX, k.mouseY, minS, maxS);
    const glm::vec2 trig = m_input.triggers();
    const float zoomKeys = held(SDL_SCANCODE_E, SDL_SCANCODE_PAGEUP) - held(SDL_SCANCODE_Q, SDL_SCANCODE_PAGEDOWN) + trig.y - trig.x;
    if (zoomKeys != 0.0f) m_mapView.zoomAround(std::exp(1.8f * zoomKeys * dt), m_mapView.sx, m_mapView.sy, minS, maxS);

    glm::vec2 pan(held(SDL_SCANCODE_D, SDL_SCANCODE_RIGHT) - held(SDL_SCANCODE_A, SDL_SCANCODE_LEFT),
                  held(SDL_SCANCODE_S, SDL_SCANCODE_DOWN) - held(SDL_SCANCODE_W, SDL_SCANCODE_UP));
    pan += m_input.leftStick();
    if (glm::length(pan) > 0.0f) {
        const float speed = 0.7f * (l.mapY1 - l.mapY0); // points per second
        m_mapView.pan(-pan.x * speed * dt, -pan.y * speed * dt);
        m_mapCrosshair = true;
    }
    if (k.mouseMoved && !m_mapDragging) m_mapCrosshair = m_input.gamepadConnected() && m_mapCrosshair;

    // Mouse: press on the panel starts a drag; a release that moved less than 5 points is a click.
    if (k.click && inPanel(k.mouseX, k.mouseY)) {
        m_mapDragging = true;
        m_mapDragMoved = 0.0f;
        m_mapLastX = k.mouseX;
        m_mapLastY = k.mouseY;
        m_mapCrosshair = false;
    } else if (k.click) {
        const int row = m_mapUI.listRowAt(k.mouseX, k.mouseY);
        if (row >= 0) {
            const MapPoi& p = m_mapUI.data().pois[std::size_t(row)];
            m_mapView.cx = p.x;
            m_mapView.cz = p.z;
            m_audio->playUi(UiSound::Click);
        }
    }
    if (m_mapDragging && k.mouseDown) {
        const float dx = k.mouseX - m_mapLastX, dy = k.mouseY - m_mapLastY;
        m_mapDragMoved += std::hypot(dx, dy);
        if (m_mapDragMoved >= 5.0f && (dx != 0.0f || dy != 0.0f)) m_mapView.pan(dx, dy);
        m_mapLastX = k.mouseX;
        m_mapLastY = k.mouseY;
    }
    auto toggleAt = [&](float sx, float sy) {
        const int poi = m_mapUI.poiAt(m_mapView, sx, sy, 16.0f);
        MapPoint w = m_mapView.toWorld(sx, sy);
        if (poi >= 0) w = {m_mapUI.data().pois[std::size_t(poi)].x, m_mapUI.data().pois[std::size_t(poi)].z};
        const bool set = toggleMarker(m_marker, w.x, w.y, 16.0f / m_mapView.scale);
        m_audio->playUi(set ? UiSound::Confirm : UiSound::Back);
    };
    if (m_mapDragging && k.release) {
        if (m_mapDragMoved < 5.0f && inPanel(k.mouseX, k.mouseY)) toggleAt(k.mouseX, k.mouseY);
        m_mapDragging = false;
    }
    if (k.accept) toggleAt(m_mapView.sx, m_mapView.sy);
    if (m_input.pressed(Action::ToggleCamera) || m_input.padPressed(PadControl::North)) {
        const MapPlayer p = mapPlayer();
        m_mapView.cx = p.x;
        m_mapView.cz = p.z;
    }
    // Keep the view over the map.
    const MapData& d = m_mapUI.data();
    m_mapView.cx = std::clamp(m_mapView.cx, d.playX0 - 0.1f * (d.playX1 - d.playX0), d.playX1 + 0.1f * (d.playX1 - d.playX0));
    m_mapView.cz = std::clamp(m_mapView.cz, d.playZ0 - 0.1f * (d.playZ1 - d.playZ0), d.playZ1 + 0.1f * (d.playZ1 - d.playZ0));
    m_mapHoverPoi = -1;
    if (!m_mapCrosshair) {
        m_mapHoverPoi = inPanel(k.mouseX, k.mouseY) ? m_mapUI.poiAt(m_mapView, k.mouseX, k.mouseY, 16.0f)
                                                    : m_mapUI.listRowAt(k.mouseX, k.mouseY);
    } else {
        m_mapHoverPoi = m_mapUI.poiAt(m_mapView, m_mapView.sx, m_mapView.sy, 16.0f);
    }
}

// Gameplay HUD: compass bar on top (clear of the HUD box), minimap in the bottom-right corner.
void Application::drawMapHud() {
    if (!m_mapUI.hasMap() || !m_ui.hud) return;
    const ImVec2 D = ImGui::GetIO().DisplaySize;
    const MapPlayer p = mapPlayer();
    const bool open = m_sim.level() != nullptr;
    const bool touch = m_input.touchControlsActive();
    const TouchLayout& tl = m_input.touchControls().layout();
    if (m_settings.compass) {
        const float w = std::clamp(0.36f * D.x, touch ? 260.0f : 360.0f, 640.0f);
        float cx = 0.5f * D.x;
        if (const UiItemRect* hud = m_ui.item("hud")) cx = std::max(cx, hud->x1 + 14.0f + 0.5f * w);
        cx = std::min(cx, (touch ? tl.topRowLeft : D.x) - 10.0f - 0.5f * w);
        m_mapUI.drawCompass(cx, 10.0f, w, open ? 900.0f : 120.0f, m_marker, p);
        m_ui.lapHudX = cx;
        m_ui.lapHudTop = 10.0f + std::clamp(0.075f * w, 30.0f, 42.0f) + 8.0f;
    } else {
        m_ui.lapHudX = -1.0f;
        m_ui.lapHudTop = 12.0f;
    }
    if (m_settings.minimap) {
        static constexpr float kSize[] = {0.085f, 0.11f, 0.145f};
        const float r = std::clamp(kSize[std::clamp(m_settings.minimapSize, 0, 2)] * D.y, 60.0f, 220.0f);
        const float margin = 22.0f;
        // With the touch controls the pedals take the bottom right corner: the minimap moves under the button row.
        const float cy = touch ? tl.topRowBottom + 14.0f + r : D.y - margin - r - 22.0f;
        m_mapUI.drawMinimap(D.x - margin - r, cy, r, open ? 180.0f : 45.0f, m_settings.minimapNorthUp, m_marker, p);
    }
}

// On-screen driving controls: on iOS while no gamepad is connected (a pad takes over at once and hides them), only in
// gameplay; the scenario `touch_controls on` shows them on any platform for the checks.
void Application::updateTouchControls() {
#if defined(SDL_PLATFORM_IOS)
    constexpr bool kTouchPlatform = true;
#else
    constexpr bool kTouchPlatform = false;
#endif
    const bool wanted = m_touchMode == TouchMode::On ||
                        (m_touchMode == TouchMode::Auto && kTouchPlatform && !m_input.gamepadConnected());
    const bool active = wanted && m_flow.screen() == Screen::Playing && !m_scenarioAutopilot && !m_options.demoDrive;
    int w = 0, h = 0;
    SDL_GetWindowSize(m_window.get(), &w, &h);
    SDL_Rect safe{0, 0, w, h};
    if (!SDL_GetWindowSafeArea(m_window.get(), &safe) || safe.w <= 0 || safe.h <= 0) safe = SDL_Rect{0, 0, w, h};
    m_input.setTouchControls(active, touchLayout(float(w), float(h), float(safe.x), float(safe.y),
                                                 float(w - safe.x - safe.w), float(h - safe.y - safe.h)));
    m_ui.keyHints = !active;
}

void Application::drawTouchControls() {
    if (!m_input.touchControlsActive() || !m_ui.hud) return;
    const TouchControls& tc = m_input.touchControls();
    const TouchLayout& L = tc.layout();
    ImDrawList* dl = ImGui::GetBackgroundDrawList(); // over the scene, under the ImGui windows
    ImFont* font = ImGui::GetFont();
    const float u = L.stickRadius / 0.75f;
    const ImU32 fill = IM_COL32(18, 22, 26, 95), lit = IM_COL32(236, 150, 40, 150), edge = IM_COL32(255, 255, 255, 120),
                ink = IM_COL32(255, 255, 255, 215);
    const auto label = [&](const char* text, float size, ImVec2 centre) {
        const ImVec2 ts = font->CalcTextSizeA(size, 1e9f, 0.0f, text);
        dl->AddText(font, size, ImVec2(centre.x - 0.5f * ts.x, centre.y - 0.5f * ts.y), ink, text);
    };
    for (int i = 0; i < kTouchButtonCount; ++i) {
        const TouchButton b = TouchButton(i);
        const TouchRect& r = L.buttons[std::size_t(i)];
        const float round = 0.22f * std::min(r.x1 - r.x0, r.y1 - r.y0);
        const bool on = tc.held(b) || (b == TouchButton::Lights && m_gfx.headlights);
        dl->AddRectFilled(ImVec2(r.x0, r.y0), ImVec2(r.x1, r.y1), on && b != TouchButton::Throttle && b != TouchButton::Brake ? lit : fill, round);
        // The pedals fill from the bottom with their ramped travel.
        const float travel = b == TouchButton::Throttle ? m_input.touchThrottle() : b == TouchButton::Brake ? m_input.touchBrake() : 0.0f;
        if (travel > 0.0f)
            dl->AddRectFilled(ImVec2(r.x0, r.y1 - travel * (r.y1 - r.y0)), ImVec2(r.x1, r.y1), lit, round,
                              ImDrawFlags_RoundCornersBottom);
        dl->AddRect(ImVec2(r.x0, r.y0), ImVec2(r.x1, r.y1), edge, round, 0, 2.0f);
        const bool big = b == TouchButton::Throttle || b == TouchButton::Brake || b == TouchButton::Handbrake;
        label(touchButtonLabel(b), (big ? 0.24f : 0.18f) * u, ImVec2(r.cx(), r.cy()));
    }
    // Steering stick: drawn at rest in the corner, under the finger while steering; the knob moves sideways only.
    const bool steering = tc.steering();
    const float R = L.stickRadius;
    const ImVec2 c(tc.stickX(), tc.stickY());
    dl->AddCircleFilled(c, R, IM_COL32(18, 22, 26, steering ? 95 : 55), 48);
    dl->AddCircle(c, R, IM_COL32(255, 255, 255, steering ? 120 : 70), 48, 2.0f);
    const float knob = c.x + std::clamp(tc.fingerX() - c.x, -R, R);
    dl->AddCircleFilled(ImVec2(knob, c.y), 0.42f * R, IM_COL32(255, 255, 255, steering ? 175 : 85), 32);
    if (!steering) {
        label("<", 0.3f * u, ImVec2(c.x - 0.72f * R, c.y));
        label(">", 0.3f * u, ImVec2(c.x + 0.72f * R, c.y));
        label("STEER", 0.18f * u, ImVec2(c.x, c.y + R + 0.18f * u));
    }
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
    // Gameplay sounds stop with the simulation (pause menu, debug freeze) and stay silent behind the start menu.
    f.paused = !simulationRuns() || m_flow.screen() == Screen::Intro || m_flow.screen() == Screen::MainMenu;
    f.music = m_musicTarget;
    f.musicFadeSeconds = m_musicFade;
    f.musicRestart = m_musicRestart;
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
    if (!simulationRuns()) {
        // Frozen: no ticks and the fixed-step accumulator keeps its value (resume continues where it stopped).
        if (m_stepOnce && m_paused && m_flow.simulationRuns()) {
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
    const int steps = m_flow.stepsForFrame(m_clock, frameTime);
    for (int i = 0; i < steps; ++i) tick();
    m_stepsThisFrame = steps;
}

void Application::renderFrame(float frameTime) {
    if (m_sim.level() != m_contentLevel) buildWorldContent();
    // ---- Pose interpolation between the last two physics ticks
    const float alpha = float(m_clock.alpha()); // constant while frozen: the accumulator does not change
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
    if (!m_flow.menuActive()) {
        ci.mouseDelta = m_input.mouseDelta();
        ci.wheel = m_input.wheel();
        ci.move = m_input.freeCameraMove();
        ci.fast = m_input.fastModifier();
        ci.stick = m_input.rightStick();
    }
    ci.vehicleSpeed = m_sim.vehicle().telemetry().speed;
    {
        const VehicleConfig& vc = m_sim.vehicle().config();
        HoodMount hm;
        hm.position = glm::vec3(vc.hoodCamera[0], vc.hoodCamera[1], vc.hoodCamera[2]);
        hm.pitchDeg = vc.hoodCameraPitch;
        hm.nearPlane = vc.hoodCameraNear;
        m_camera.setHoodMount(hm);
        const JPH::AABox& bb = m_sim.vehicle().bodyBounds();
        m_camera.setVehicleBounds(toGlm(bb.mMin), toGlm(bb.mMax));
    }
    const bool vehiclePage = m_flow.screen() == Screen::MainMenu && m_flow.page() == MenuPage::Vehicle;
    if (vehiclePage && !m_vehiclePageCamera) m_garageCamTime = 0.0;
    m_vehiclePageCamera = vehiclePage;
    if ((m_flow.screen() == Screen::Intro || m_flow.screen() == Screen::MainMenu) && !vehiclePage) updateMenuCamera(frameTime, bodyM);
    if (m_flow.screen() == Screen::Garage || vehiclePage) updateGarageCamera(frameTime); // the preview: the live vehicle
    m_camera.update(frameTime, ci, bodyM,
                    [this](const glm::vec3& from, const glm::vec3& to) {
                        return m_sim.castRayIgnoringVehicle(JPH::Vec3(from.x, from.y, from.z), JPH::Vec3(to.x, to.y, to.z));
                    });
    if (m_scenario) {
        // Camera height above the surface below it (downward ray; negative means the camera is underground).
        const glm::vec3 cp = m_camera.position();
        const JPH::Vec3 above(cp.x, cp.y + 30.0f, cp.z), below(cp.x, cp.y - 5.0f, cp.z);
        if (auto hit = m_sim.castRayIgnoringVehicle(above, below)) {
            m_camClearance = cp.y - (above.GetY() + (below.GetY() - above.GetY()) * *hit);
            if (m_camera.chaseMode()) m_minCamClearance = std::min(m_minCamClearance, m_camClearance);
        }
        int air = 0;
        for (const auto& w : m_sim.vehicle().telemetry().wheels) air += w.contact ? 0 : 1;
        m_maxAirWheels = std::max(m_maxAirWheels, air);
        const VehicleTelemetry& t = m_sim.vehicle().telemetry();
        m_maxTilt = std::max(m_maxTilt, std::acos(std::fmin(1.0f, t.upDot)) * 57.2958f);
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
    if (m_scenario && !m_perfActive) ImGui::GetIO().DeltaTime = float(m_frameDt); // menu animations on the scenario clock
    ImGui::NewFrame();
    FrameStats stats;
    stats.fps = m_fps;
    stats.frameMs = m_frameMs;
    stats.stepsThisFrame = m_stepsThisFrame;
    stats.droppedLastMs = m_clock.lastDropped() * 1000.0;
    stats.droppedTotalMs = m_clock.totalDropped() * 1000.0;
    stats.overloaded = m_clock.overloaded();
    stats.paused = m_paused;
    m_ui.menuOpen = m_flow.menuActive();
    stats.cameraMode = int(m_camera.mode());
    stats.gpuDriver = m_renderer->driverName();
    stats.shaderFormat = m_renderer->shaderFormatName();
    stats.gamepad = m_input.gamepadConnected() ? m_input.gamepadName() : "";
    const RenderStats& rs = m_renderer->stats();
    stats.cpuFrameMs = m_cpuMs;
    stats.gpuFrameMs = m_renderer->gpuTimingAvailable() ? rs.gpuMs : -1.0;
    stats.renderW = rs.renderW;
    stats.renderH = rs.renderH;
    stats.outputW = rs.outputW;
    stats.outputH = rs.outputH;
    stats.drawCalls = rs.drawCalls;
    stats.grassBlades = rs.grassBlades;
    stats.reflection = rs.reflectionRendered;
    stats.maxMsaa = m_renderer->maxMsaa();
    stats.autoExposureEv = rs.autoExposureEv;
    stats.markUploadBytes = rs.markUploadBytes;
    m_gfx.sprayDrawn = rs.sprayParticles;
    updateBestLaps();
    m_ui.storedBestLap = m_bestLaps.best(currentLevelId(), currentVehicleId());
    updateBestDrift();
    m_ui.storedBestDrift = m_bestDrift.best(currentLevelId(), currentVehicleId());
    const UiRequests req = m_ui.draw(m_sim, stats, m_gfx);
    if (m_gfx.fillMarks) {
        fillTireMarks();
        m_gfx.fillMarks = false;
    }
    m_audioPanel.stats = m_audio->stats();
    m_audioPanel.status = m_audio->status().c_str();
    m_audioPanel.active = m_audio->active();
    if (!m_flow.menuActive()) m_ui.drawAudio(m_audioPanel);
    pollMapBuild();
    if (m_flow.screen() == Screen::Playing) drawMapHud();
    drawTouchControls();
    if (m_flow.screen() == Screen::Map) m_mapUI.drawFullMap(m_mapView, m_marker, mapPlayer(), mapHints(), m_mapHoverPoi, m_mapCrosshair);
    MenuView view;
    view.gamepad = m_input.gamepadConnected();
    view.version = TRAILFORGE_VERSION;
    view.driveKeys = DriveKeys(m_settings.driveKeys);
    view.keyboard = keyboardStyle();
    view.padType = padType();
    view.padGlyphs = resolvePadGlyphs(PadGlyphSetting(m_settings.padGlyphs), view.padType);
    view.vehicles = &m_vehicleCards;
    m_menu.draw(m_flow, m_settings, view);
    if (m_flow.screen() == Screen::Garage && m_garage) m_menu.drawGarage(*m_garage, m_sim.vehicle().telemetry(), view);
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
    if (m_settings.timeFlow > 0 && m_flow.screen() == Screen::Playing) {
        // Settings > TIME FLOW: the gameplay clock runs while driving (frozen in the menus and the map).
        const float rate = kTimeFlowRates[std::clamp(m_settings.timeFlow, 0, 2)];
        m_gfx.settings.timeOfDay = std::fmod(m_gfx.settings.timeOfDay + rate * frameTime, 24.0f);
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
    if (m_ui.drawPhysicsDebug && m_ui.debugWindows) {
        const VehicleConfig& vc = m_sim.vehicleConfig();
        drawVehicleDebug(m_debugDraw, m_sim.vehicle().telemetry(),
                         {vc.axles[0].suspensionMinLength, vc.axles[1].suspensionMinLength},
                         {vc.axles[0].suspensionMaxLength, vc.axles[1].suspensionMaxLength}, m_ui.debug);
        if (m_ui.debug.bodyAxes) m_debugDraw.axes(bodyM, 1.5f);
    }

    int pw = 1, ph = 1;
    SDL_GetWindowSizeInPixels(m_window.get(), &pw, &ph);
    FrameDesc frame;
    frame.view = m_camera.view();
    frame.proj = m_camera.projection(float(pw) / float(ph > 0 ? ph : 1));
    frame.cameraPos = m_camera.position();
    frame.fovYRadians = glm::radians(m_camera.fovYDeg);
    frame.nearPlane = m_camera.activeNearPlane();
    frame.time = float(m_time);
    frame.draws = m_draws;
    frame.water = &m_waterMesh;
    frame.waterfalls = &m_waterfallMesh;
    if (m_sim.level()) {
        // Open world: thinner fog over kilometers, longer shadow cascades, no planar puddle reflection.
        frame.fogDensity = 0.00045f;
        frame.fogFalloff = 0.004f;
        frame.valleyFog = 0.001f;
        frame.valleyFogBase = 15.0f;
        frame.shadowScale = 2.0f;
        frame.waterReflection = false;
        frame.skyMountains = false;
        frame.deepOutsideGround = true;
    }
    frame.vehicleModel = bodyM;
    frame.vehicleMud = m_gfx.mud;
    frame.bodyMud = &m_bodyMud;
    frame.bodyMudArch = m_vehicleVisual.mudArch;
    frame.bodyMudBody = m_vehicleVisual.mudBody;
    {
        const VehicleTelemetry& t = m_sim.vehicle().telemetry();
        frame.brakeLight = std::max(t.command.brake, t.command.handBrake) > 0.05f ? 1.0f : 0.0f;
        frame.reverseLight = t.direction == DriveDirection::Reverse ? 1.0f : 0.0f;
        frame.headlights = m_gfx.headlights ? 1.0f : 0.0f;
        // Lamp lens centers in body space (per model), beams pitched ~3 deg down.
        const glm::vec3 lens = m_vehicleVisual.headlightLens;
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
    {
        // Blur under the pause menu and under Settings / Controls opened from the start menu.
        const float target = m_flow.screen() == Screen::Paused ? 1.0f
                             : m_flow.screen() == Screen::MainMenu && m_flow.depth() > 1 &&
                                       m_flow.page() != MenuPage::Vehicle ? 0.75f
                                                                                         : 0.0f;
        const float step = frameTime / 0.25f;
        m_menuBlur = m_menuBlur < target ? std::min(target, m_menuBlur + step) : std::max(target, m_menuBlur - step);
        frame.blur = m_menuBlur;
        // Depth of field on the vehicle: the start menu backdrop and the garage; optional (subtle) while driving.
        const Screen sc = m_flow.screen();
        const bool showcase = sc == Screen::Garage || (sc == Screen::MainMenu && target == 0.0f);
        const float dofTarget = showcase && m_gfx.settings.dof ? 0.5f
                                : sc == Screen::Playing ? 0.25f * m_gfx.settings.dofDriving
                                                        : 0.0f;
        const float dofStep = frameTime / 0.4f;
        m_dofAmount = m_dofAmount < dofTarget ? std::min(dofTarget, m_dofAmount + dofStep)
                                              : std::max(dofTarget, m_dofAmount - dofStep);
        frame.dofAmount = m_dofAmount;
        frame.dofFocus = m_dofAmount > 0.0f ? glm::length(glm::vec3(bodyM[3]) - m_camera.position()) : 0.0f;
    }
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

void Application::rebuildVehicleVisual() {
    const VehicleConfig& vc = m_sim.vehicleConfig();
    m_vehicleVisual = buildVehicleVisual(vc);
    m_bodyMesh = m_renderer->upload(m_vehicleVisual.mesh);
    m_springMesh = m_renderer->upload(buildUnitSpringMesh(m_vehicleVisual.coil));
    for (int a = 0; a < 2; ++a) {
        const MeshData wheel = buildAxleWheelMesh(m_vehicleVisual, vc.axles[std::size_t(a)], vc.tireSet);
        m_wheelVisualSize[std::size_t(a)] = wheelMeshSize(wheel);
        (a == 0 ? m_wheelMesh : m_rearWheelMesh) = m_renderer->upload(wheel);
    }
    m_visualFit = vc;
}

void Application::buildDraws(const glm::mat4& bodyM, const std::array<glm::mat4, kWheelCount>& wheelM) {
    const VehicleConfig& vc = m_sim.vehicleConfig();
    if (!m_visualFit || !sameWheelFit(*m_visualFit, vc)) {
        SDL_WaitForGPUIdle(m_renderer->device());
        rebuildVehicleVisual();
    }

    m_draws.clear();
    bool vehiclePart = false;
    auto add = [&](const GpuMesh& mesh, const glm::mat4& model, bool castsShadow = true, bool inReflection = true) {
        DrawItem d;
        d.mesh = &mesh;
        d.model = model;
        d.castsShadow = castsShadow;
        d.inReflection = inReflection;
        d.vehicle = vehiclePart;
        m_draws.push_back(d);
    };
    add(m_trackMesh, glm::mat4(1.0f));
    add(m_sceneryMesh, glm::mat4(1.0f), !m_sim.level(), !m_sim.level()); // the level horizon casts no shadows
    vehiclePart = true;
    add(m_bodyMesh, bodyM);
    for (const SpareMount& spare : m_vehicleVisual.spares) {
        add(m_rearWheelMesh,
            bodyM * spareWheelMatrix(spare, vc.axles[kRearAxle].wheelRadius, vc.axles[kRearAxle].wheelWidth));
        m_draws.back().mud = spare.outward.y > 0.5f ? 0.25f : 0.6f; // the roof spare stays mostly clean
        m_draws.back().wheelMud = true;
    }

    const glm::mat4 invBody = glm::inverse(bodyM);
    for (int i = 0; i < kWheelCount; ++i) {
        // Wheel and suspension follow the Jolt wheel transform (interpolated between ticks).
        const glm::mat4 wheelBody = invBody * wheelM[i];
        const glm::vec3 hub(wheelBody[3]);
        const float side = hub.x >= 0.0f ? 1.0f : -1.0f;
        // The wheel mesh has its outer face at +X; Jolt uses one wheel-right axis for both sides.
        const bool flip = glm::dot(glm::vec3(wheelBody[0]), glm::vec3(side, 0, 0)) < 0.0f;
        const std::size_t axle = std::size_t(i / 2);
        add(axle == 0 ? m_wheelMesh : m_rearWheelMesh, flip ? wheelM[i] * glm::rotate(glm::mat4(1.0f), glm::pi<float>(), glm::vec3(0, 1, 0))
                              : wheelM[i]);
        m_draws.back().wheelMud = true;
        const TreadSoil& soil = m_sim.tireMarks().soil(std::size_t(i));
        m_draws.back().soil = glm::vec4(soil.color[0], soil.color[1], soil.color[2], soil.amount);
        m_draws.back().soilWetness = soil.wetness;
        m_draws.back().soilFlecks = soil.flecks;
        m_draws.back().soilStick = soil.stick;
        m_draws.back().wheelRadius = vc.axles[axle].wheelRadius;

        const SuspensionPose p = suspensionPose(hub, hub.z, m_vehicleVisual.mounts[axle]);
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
    // Cosmetic only, never fed back into the simulation. Spray mud per body region from what each wheel throws, plus
    // a slow even film that builds up while the wheels roll on the Mud surface or through the puddles.
    {
        std::array<TreadSoil, kWheelCount> soil;
        for (std::size_t i = 0; i < kWheelCount; ++i) soil[i] = m_sim.tireMarks().soil(i);
        m_bodyMud.update(m_sim.vehicle().contacts(), soil, std::min(dt, 0.05f));
    }
    const VehicleTelemetry& t = m_sim.vehicle().telemetry();
    const WaterInfo water = TestTrack::water();
    float gain = 0.0f;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
        const WheelTelemetry& w = t.wheels[i];
        if (!w.contact) continue;
        const float roll = std::min(std::abs(w.angularVelocity) * m_sim.vehicleConfig().axles[i / 2].wheelRadius, 8.0f);
        const bool wet = w.contactPosition.GetY() < water.level - 0.02f && w.contactPosition.GetX() > water.x0 &&
                         w.contactPosition.GetX() < water.x1 && w.contactPosition.GetZ() > water.z0 &&
                         w.contactPosition.GetZ() < water.z1;
        if (w.surface == SurfaceId::Mud || wet) gain += roll * (wet ? 0.006f : 0.0025f);
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
    // Low percentiles: with other GPU work on the machine the fastest frames are the least disturbed ones.
    const double p10 = v[std::size_t(double(v.size()) * 0.10)];
    std::vector<double> g = m_perfGpuMs;
    std::sort(g.begin(), g.end());
    const double gpuMin = g.empty() ? -1.0 : g.front();
    const double gpuP10 = g.empty() ? -1.0 : g[std::size_t(double(g.size()) * 0.10)];
    const RenderStats& rs = m_renderer->stats();
    std::printf("PERF %-12s frames=%zu fps=%.1f frame_avg=%.2f ms p95=%.2f p99=%.2f max=%.2f cpu_avg=%.2f ms "
                "gpu_avg=%.2f ms render=%ux%u output=%ux%u vsync=%d draws=%d grass=%.0fk frame_p10=%.2f gpu_min=%.2f "
                "gpu_p10=%.2f\n",
                m_perfLabel.c_str(), v.size(), 1000.0 / avg, avg, p95, p99, v.back(),
                m_perfCpuSum / double(v.size()), m_perfGpuCount ? m_perfGpuSum / m_perfGpuCount : -1.0, rs.renderW,
                rs.renderH, rs.outputW, rs.outputH, m_renderer->vsync() ? 1 : 0, rs.drawCalls,
                double(rs.grassBlades) / 1000.0, p10, gpuMin, gpuP10);
}

// ------------------------------------------------------------------ verification scenario

void Application::queueEvent(const SDL_Event& e, int framesFromNow) {
    m_queuedEvents.push_back({framesFromNow, e});
}

void Application::queueTouch(float x, float y, float dx, float dy, int steps) {
    const SDL_WindowID win = SDL_GetWindowID(m_window.get());
    SDL_Event motion{};
    motion.type = SDL_EVENT_MOUSE_MOTION;
    motion.motion.windowID = win;
    motion.motion.which = SDL_TOUCH_MOUSEID;
    motion.motion.x = x;
    motion.motion.y = y;
    queueEvent(motion, 0);
    SDL_Event down{};
    down.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
    down.button.windowID = win;
    down.button.which = SDL_TOUCH_MOUSEID;
    down.button.button = SDL_BUTTON_LEFT;
    down.button.down = true;
    down.button.clicks = 1;
    down.button.x = x;
    down.button.y = y;
    queueEvent(down, 0);
    for (int i = 1; i <= steps; ++i) {
        motion.motion.x = x + dx * float(i) / float(steps);
        motion.motion.y = y + dy * float(i) / float(steps);
        motion.motion.xrel = dx / float(steps);
        motion.motion.yrel = dy / float(steps);
        queueEvent(motion, i);
    }
    SDL_Event up = down;
    up.type = SDL_EVENT_MOUSE_BUTTON_UP;
    up.button.down = false;
    up.button.x = x + dx;
    up.button.y = y + dy;
    queueEvent(up, steps + 1);
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
    // Commands act on the world that is loaded, not on one still waiting for its first tick (the start level too).
    if (m_sim.levelPending() && simulationRuns()) return;
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
    if (c.op == "menu") {
        // menu intro|main|start|pause|resume|settings|back|select ITEM: drive the game flow directly (key taps and
        // `click menu:ITEM` go through the real input path instead).
        const Screen before = m_flow.screen();
        if (arg == "intro") {
            std::vector<std::string> ids = m_flow.vehicles();
            m_flow = GameFlow(Screen::Intro);
            m_flow.setVehicles(std::move(ids));
            m_flow.setLevels(m_levelIds);
            enterMenuWorld(true);
        } else if (arg == "main") {
            m_flow.toMainMenu();
            if (before != Screen::MainMenu) enterMenuWorld(true);
        } else if (arg == "start") {
            if (before != Screen::MainMenu) return Exec::Failed;
            m_flow.startGame();
            startGameplay();
        } else if (arg == "pause") {
            if (before != Screen::Playing) return Exec::Failed;
            m_flow.pause();
            m_audio->playUi(UiSound::MenuOpen);
        } else if (arg == "resume") {
            if (before != Screen::Paused) return Exec::Failed;
            m_flow.resume();
            m_audio->playUi(UiSound::MenuClose);
        } else if (arg == "garage") {
            if (before != Screen::MainMenu && before != Screen::Paused) return Exec::Failed;
            openGarage();
            if (m_flow.screen() != Screen::Garage) return Exec::Failed;
        } else if (arg == "garage_close") {
            if (before != Screen::Garage) return Exec::Failed;
            closeGarage();
        } else if (arg == "vehicles") {
            m_flow.openVehiclePage();
            if (m_flow.page() != MenuPage::Vehicle) return Exec::Failed;
        } else if (arg == "settings") {
            m_flow.openSettings();
            if (m_flow.page() != MenuPage::Settings) return Exec::Failed;
        } else if (arg == "back") {
            if (m_flow.page() == MenuPage::Settings) saveSettings();
            if (!m_flow.back()) return Exec::Failed;
        } else if (arg == "select") {
            const auto item = itemByName(c.args.size() > 1 ? c.args[1] : "");
            if (!item || !m_flow.select(*item)) {
                std::printf("SCENARIO: menu item '%s' is not on this page\n", c.args.size() > 1 ? c.args[1].c_str() : "");
                return Exec::Failed;
            }
        } else {
            return Exec::Failed;
        }
        return Exec::Done;
    }
    if (c.op == "controls") {
        // controls keyboard mac|pc|native | pad_type none|xbox|ps4|ps5|switch|unknown|real: Controls page overrides for
        // the captures (key cap wording, the connected pad seen by the automatic glyph choice). Not persisted.
        const std::string v = c.args.size() > 1 ? c.args[1] : std::string();
        if (arg == "keyboard") {
            if (v == "mac") m_keyboardOverride = KeyboardStyle::Mac;
            else if (v == "pc") m_keyboardOverride = KeyboardStyle::Pc;
            else if (v == "native") m_keyboardOverride.reset();
            else return Exec::Failed;
            m_ui.driveHint = driveHint(DriveKeys(m_settings.driveKeys), keyboardStyle());
        } else if (arg == "pad_type") {
            if (v == "none") m_padTypeOverride = PadType::None;
            else if (v == "xbox") m_padTypeOverride = PadType::XboxOne;
            else if (v == "ps4") m_padTypeOverride = PadType::PS4;
            else if (v == "ps5") m_padTypeOverride = PadType::PS5;
            else if (v == "switch") m_padTypeOverride = PadType::SwitchPro;
            else if (v == "unknown") m_padTypeOverride = PadType::Unknown;
            else if (v == "real") m_padTypeOverride.reset();
            else return Exec::Failed;
        } else {
            return Exec::Failed;
        }
        return Exec::Done;
    }
    if (c.op == "gamepad") {
        // gamepad attach | detach | axis leftx|lefty|rightx|righty|lt|rt V (-1..1, triggers 0..1): a virtual SDL gamepad,
        // read by the input through SDL_GetGamepadAxis like a connected pad (stands in for a real pad in the checks).
        if (arg == "attach") {
            if (m_virtualPadId) return Exec::Done;
            // Scenario runs often have no keyboard focus; SDL drops pad motion of unfocused windows by default.
            SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
            SDL_VirtualJoystickDesc d;
            SDL_INIT_INTERFACE(&d);
            d.type = SDL_JOYSTICK_TYPE_GAMEPAD;
            d.naxes = SDL_GAMEPAD_AXIS_COUNT;
            d.nbuttons = SDL_GAMEPAD_BUTTON_COUNT;
            d.name = "TrailForge scenario pad";
            m_virtualPadId = SDL_AttachVirtualJoystick(&d);
            m_virtualPad = m_virtualPadId ? SDL_OpenJoystick(m_virtualPadId) : nullptr;
            if (!m_virtualPad) {
                std::fprintf(stderr, "scenario: virtual gamepad: %s\n", SDL_GetError());
                return Exec::Failed;
            }
            // Report the centred sticks first: SDL takes an axis' first report as its rest value.
            for (int a = SDL_GAMEPAD_AXIS_LEFTX; a <= SDL_GAMEPAD_AXIS_RIGHTY; ++a) SDL_SetJoystickVirtualAxis(m_virtualPad, a, 0);
            return Exec::Done;
        }
        if (arg == "detach") {
            if (m_virtualPad) SDL_CloseJoystick(m_virtualPad);
            if (m_virtualPadId) SDL_DetachVirtualJoystick(m_virtualPadId);
            m_virtualPad = nullptr;
            m_virtualPadId = 0;
            return Exec::Done;
        }
        if (arg == "axis" && c.args.size() > 2 && m_virtualPad) {
            const std::string& name = c.args[1];
            const float v = std::clamp(float(std::atof(c.args[2].c_str())), -1.0f, 1.0f);
            SDL_GamepadAxis axis = SDL_GAMEPAD_AXIS_INVALID;
            if (name == "leftx") axis = SDL_GAMEPAD_AXIS_LEFTX;
            else if (name == "lefty") axis = SDL_GAMEPAD_AXIS_LEFTY;
            else if (name == "rightx") axis = SDL_GAMEPAD_AXIS_RIGHTX;
            else if (name == "righty") axis = SDL_GAMEPAD_AXIS_RIGHTY;
            else if (name == "lt") axis = SDL_GAMEPAD_AXIS_LEFT_TRIGGER;
            else if (name == "rt") axis = SDL_GAMEPAD_AXIS_RIGHT_TRIGGER;
            if (axis == SDL_GAMEPAD_AXIS_INVALID) return Exec::Failed;
            const bool trigger = axis == SDL_GAMEPAD_AXIS_LEFT_TRIGGER || axis == SDL_GAMEPAD_AXIS_RIGHT_TRIGGER;
            // Triggers span the full raw range: -32768 released, 32767 fully pressed.
            const float raw = trigger ? -32768.0f + 65535.0f * std::max(v, 0.0f) : 32767.0f * v;
            return SDL_SetJoystickVirtualAxis(m_virtualPad, axis, Sint16(std::lround(raw))) ? Exec::Done : Exec::Failed;
        }
        if (arg == "button" && c.args.size() > 2 && m_virtualPad) {
            // gamepad button south|east|west|north|lb|rb|back|start 1|0
            static const std::pair<const char*, SDL_GamepadButton> kButtons[] = {
                {"south", SDL_GAMEPAD_BUTTON_SOUTH}, {"east", SDL_GAMEPAD_BUTTON_EAST},
                {"west", SDL_GAMEPAD_BUTTON_WEST},   {"north", SDL_GAMEPAD_BUTTON_NORTH},
                {"lb", SDL_GAMEPAD_BUTTON_LEFT_SHOULDER}, {"rb", SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER},
                {"back", SDL_GAMEPAD_BUTTON_BACK},   {"start", SDL_GAMEPAD_BUTTON_START}};
            for (const auto& [name, button] : kButtons)
                if (c.args[1] == name)
                    return SDL_SetJoystickVirtualButton(m_virtualPad, button, std::atoi(c.args[2].c_str()) != 0) ? Exec::Done
                                                                                                                 : Exec::Failed;
        }
        return Exec::Failed;
    }
    if (c.op == "garage") {
        // garage set KEY V | step KEY DIR | row KEY | save | save_new | rename NAME | delete | select NAME|factory |
        // reset | kit ID|none | vehicle ID: the garage model directly (keys and `click garage:...` use the input path).
        if (!m_garage) return Exec::Failed;
        const std::string a1 = c.args.size() > 1 ? c.args[1] : std::string();
        const std::string a2 = c.args.size() > 2 ? c.args[2] : std::string();
        bool ok = true;
        const int serial = m_garage->messageSerial();
        if (arg == "set") {
            ok = findSetupParam(a1) != nullptr && !a2.empty();
            if (ok) m_garage->setValue(a1, std::stof(a2));
        }
        else if (arg == "step") ok = m_garage->stepValue(a1, a2.empty() ? 1 : std::stoi(a2));
        else if (arg == "row") ok = a1 == "vehicle" ? m_garage->selectRow(GarageRowKind::Vehicle)
                                    : a1 == "setup" ? m_garage->selectRow(GarageRowKind::Setup)
                                    : a1 == "kit"   ? m_garage->selectRow(GarageRowKind::Kit)
                                                    : m_garage->selectRow(GarageRowKind::Param, a1);
        else if (arg == "save") ok = m_garage->save();
        else if (arg == "save_new") ok = m_garage->saveAsNew();
        else if (arg == "rename") {
            std::string name;
            for (std::size_t i = 1; i < c.args.size(); ++i) name += (i > 1 ? " " : "") + c.args[i];
            ok = m_garage->rename(name);
        } else if (arg == "delete") ok = m_garage->deleteActive();
        else if (arg == "select") {
            std::string name;
            for (std::size_t i = 1; i < c.args.size(); ++i) name += (i > 1 ? " " : "") + c.args[i];
            ok = m_garage->selectSetup(name == "factory" ? std::string() : name);
        }
        else if (arg == "reset") m_garage->resetToFactory();
        else if (arg == "kit") ok = m_garage->setKit(a1 == "none" ? std::string() : a1);
        else if (arg == "vehicle") ok = m_garage->selectVehicle(a1);
        else return Exec::Failed;
        if (m_garage->messageSerial() != serial) std::printf("SCENARIO garage: %s\n", m_garage->message().c_str());
        applyGarageConfig();
        return ok ? Exec::Done : Exec::Failed;
    }
    if (c.op == "tires") {
        // tires offroad|road|slick: fit a tire set through the garage (the vehicle is recreated in place, new tires at
        // the air temperature). tires ambient C: air temperature. tires warm|cold: all tires to the middle of the set's
        // window (taken off warmers) / to the air temperature.
        if (arg == "ambient" && c.args.size() > 1) {
            m_sim.setAmbientTemperature(std::strtof(c.args[1].c_str(), nullptr));
            return Exec::Done;
        }
        if (arg == "warm" || arg == "cold") {
            const TireCompound& tc = tireCompound(m_sim.vehicleConfig().tireSet);
            const float air = m_sim.ambientTemperature();
            const float warm = 0.5f * (std::max(tc.optimumLowC, air) + tc.optimumHighC);
            m_sim.setTireTemperatures(arg == "warm" ? warm : air, arg == "warm" ? warm : air);
            return Exec::Done;
        }
        const TireSet set = tireSetFromKey(arg);
        if (set == TireSet::Count || !m_garage) return Exec::Failed;
        m_garage->setValue("tires.set", float(int(set)));
        applyGarageConfig();
        return Exec::Done;
    }
    if (c.op == "transfer") {
        if (arg == "2h") m_sim.requestTransfer(TransferMode::TwoHigh);
        else if (arg == "4h") m_sim.requestTransfer(TransferMode::FourHigh);
        else if (arg == "4l") m_sim.requestTransfer(TransferMode::FourLow);
        else return Exec::Failed;
        return Exec::Done;
    }
    if (c.op == "pedal") {
        // pedal throttle|brake V: hold an analog pedal at V (0..1) like a gamepad trigger, until set to 0.
        if (c.args.size() != 2) return Exec::Failed;
        const float v = std::clamp(std::strtof(c.args[1].c_str(), nullptr), 0.0f, 1.0f);
        if (arg == "throttle") m_input.setScriptPedals(v, m_input.scriptBrake());
        else if (arg == "brake") m_input.setScriptPedals(m_input.scriptThrottle(), v);
        else return Exec::Failed;
        return Exec::Done;
    }
    if (c.op == "click" && arg.rfind("garage:", 0) == 0) {
        // click garage:ROW[:FRACTION] - ROW: a parameter key, vehicle, setup, kit or an action (save, save_new,
        // rename, delete, reset, back); FRACTION 0..1 clicks a slider bar at that position.
        if (!m_garage || m_flow.screen() != Screen::Garage) return Exec::Failed;
        std::string rowName = arg.substr(7);
        float fraction = -1.0f;
        if (const auto colon = rowName.find(':'); colon != std::string::npos) {
            fraction = std::stof(rowName.substr(colon + 1));
            rowName = rowName.substr(0, colon);
        }
        static const std::pair<const char*, GarageAction> kActions[] = {
            {"save", GarageAction::Save},   {"save_new", GarageAction::SaveAsNew}, {"rename", GarageAction::Rename},
            {"delete", GarageAction::Delete}, {"reset", GarageAction::ResetFactory}, {"back", GarageAction::Back}};
        int row = -1;
        const auto& rows = m_garage->rows();
        for (std::size_t i = 0; i < rows.size() && row < 0; ++i) {
            const GarageRow& r = rows[i];
            if ((r.kind == GarageRowKind::Vehicle && rowName == "vehicle") || (r.kind == GarageRowKind::Setup && rowName == "setup") ||
                (r.kind == GarageRowKind::Kit && rowName == "kit") || (r.kind == GarageRowKind::Param && rowName == r.param->key))
                row = int(i);
            for (const auto& [name, action] : kActions)
                if (r.kind == GarageRowKind::Action && r.action == action && rowName == name) row = int(i);
        }
        float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        if (row < 0) {
            std::printf("SCENARIO: garage row '%s' does not exist\n", rowName.c_str());
            return Exec::Failed;
        }
        if (!m_menu.garageRowRect(row, x0, y0, x1, y1)) {
            // Off screen: move the cursor there first (the list scrolls), then retry next frame.
            m_garage->setCursor(row);
            if (++m_scrollFrames > 120) {
                m_scrollFrames = 0;
                return Exec::Failed;
            }
            return Exec::Busy;
        }
        m_scrollFrames = 0;
        float x = x0 + 0.12f * (x1 - x0);
        if (fraction >= 0.0f) {
            // Find the bar by probing along the row (the hit test reports the fraction under the point).
            float best = -1.0f, bestX = x;
            for (float px = x0; px < x1; px += 1.0f) {
                int dir = 0;
                float f = -1.0f;
                if (m_menu.garageHitTest(px, 0.5f * (y0 + y1), &dir, &f) == row && f >= 0.0f &&
                    (best < 0.0f || std::abs(f - fraction) < std::abs(best - fraction))) {
                    best = f;
                    bestX = px;
                }
            }
            if (best < 0.0f) return Exec::Failed;
            x = bestX;
        }
        queueClick(x, 0.5f * (y0 + y1), false);
        return Exec::Done;
    }
    if (c.op == "click" && arg.rfind("modal:", 0) == 0) {
        // click modal:confirm|cancel - a button of the open modal dialog, through the real mouse path.
        const int button = arg == "modal:confirm" ? 0 : arg == "modal:cancel" ? 1 : -1;
        float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        if (!m_menu.modalButtonRect(button, x0, y0, x1, y1)) {
            std::printf("SCENARIO: modal button '%s' is not on screen\n", arg.c_str());
            return Exec::Failed;
        }
        queueClick(0.5f * (x0 + x1), 0.5f * (y0 + y1), false);
        return Exec::Done;
    }
    if (c.op == "soc") {
        // soc V: battery state of charge of an electric vehicle (0..1).
        if (!m_sim.vehicleConfig().isElectric() || arg.empty()) return Exec::Failed;
        m_sim.requestStateOfCharge(std::clamp(std::strtof(arg.c_str(), nullptr), 0.0f, 1.0f));
        return Exec::Done;
    }
    if (c.op == "vehicle") {
        // vehicle ID: the vehicle-page choice directly (also remembered); the vehicle respawns in place.
        if (!selectVehicle(arg)) {
            std::printf("SCENARIO: no vehicle '%s'\n", arg.c_str());
            return Exec::Failed;
        }
        m_sim.requestVehicleConfig(m_garage->config()); // also replaces a vehicle_file vehicle
        return Exec::Done;
    }
    if (c.op == "vehicle_file") {
        // vehicle_file PATH: a vehicle.json outside the catalog (test definitions) replaces the running vehicle in
        // place until the next `vehicle`; the garage keeps its own vehicle.
        VehicleDefinition def;
        std::vector<std::string> errors;
        if (!loadVehicleDefinition(arg, def, errors)) {
            for (const std::string& e : errors) std::printf("SCENARIO: %s\n", e.c_str());
            return Exec::Failed;
        }
        std::printf("SCENARIO: vehicle '%s' from %s%s\n", def.id.c_str(), arg.c_str(),
                    def.config.bodyModel.empty() ? "" : " (body.glb)");
        m_sim.requestVehicleConfig(def.config);
        return Exec::Done;
    }
    if (c.op == "click" && arg.rfind("vehicle_card:", 0) == 0) {
        float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        if (!m_menu.vehicleCardRect(std::atoi(arg.c_str() + 13), x0, y0, x1, y1)) {
            std::printf("SCENARIO: vehicle card '%s' is not on screen\n", arg.c_str());
            return Exec::Failed;
        }
        queueClick(0.5f * (x0 + x1), 0.5f * (y0 + y1), false);
        return Exec::Done;
    }
    if (c.op == "touch" && arg.rfind("menu:", 0) == 0) {
        // touch menu:ROW: a finger tap on a menu row (SDL's touch mouse), in the middle of its label.
        const auto item = itemByName(arg.substr(5));
        float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        if (!item || !m_menu.itemRect(*item, x0, y0, x1, y1)) {
            std::printf("SCENARIO: menu row '%s' is not on screen\n", arg.c_str());
            return Exec::Failed;
        }
        queueTouch(x0 + 0.15f * (x1 - x0), 0.5f * (y0 + y1), 0.0f, 0.0f, 0);
        return Exec::Done;
    }
    if (c.op == "touch_controls") {
        // touch_controls on | off | auto: the on-screen driving controls (auto: iOS without a gamepad).
        if (arg == "on") m_touchMode = TouchMode::On;
        else if (arg == "off") m_touchMode = TouchMode::Off;
        else if (arg == "auto") m_touchMode = TouchMode::Auto;
        else return Exec::Failed;
        updateTouchControls();
        return Exec::Done;
    }
    if (c.op == "finger") {
        // finger down|move|up ID [X Y | button:NAME | stick] [DX DY]: an SDL finger event (window points; stick = the
        // stick's rest position, DX DY an offset from the target), through the same path as a real touch screen.
        if (c.args.size() < 2) return Exec::Failed;
        const std::string& what = c.args[0];
        const std::uint64_t id = std::strtoull(c.args[1].c_str(), nullptr, 10);
        const TouchLayout& L = m_input.touchControls().layout();
        if (L.width <= 0.0f || L.height <= 0.0f) return Exec::Failed;
        float x = 0.0f, y = 0.0f;
        std::size_t next = 3;
        if (what != "up") {
            if (c.args.size() < 3) return Exec::Failed;
            const std::string& target = c.args[2];
            if (target == "stick") {
                x = L.restX;
                y = L.restY;
            } else if (target.rfind("button:", 0) == 0) {
                int found = -1;
                for (int b = 0; b < kTouchButtonCount; ++b)
                    if (target.substr(7) == touchButtonName(TouchButton(b))) found = b;
                if (found < 0) return Exec::Failed;
                x = L.buttons[std::size_t(found)].cx();
                y = L.buttons[std::size_t(found)].cy();
            } else {
                if (c.args.size() < 4) return Exec::Failed;
                x = float(std::atof(target.c_str()));
                y = float(std::atof(c.args[3].c_str()));
                next = 4;
            }
            if (c.args.size() >= next + 2) {
                x += float(std::atof(c.args[next].c_str()));
                y += float(std::atof(c.args[next + 1].c_str()));
            }
        }
        SDL_Event ev{};
        if (what == "down") ev.type = SDL_EVENT_FINGER_DOWN;
        else if (what == "move") ev.type = SDL_EVENT_FINGER_MOTION;
        else if (what == "up") ev.type = SDL_EVENT_FINGER_UP;
        else return Exec::Failed;
        ev.tfinger.touchID = 1;
        ev.tfinger.fingerID = id;
        ev.tfinger.x = x / L.width;
        ev.tfinger.y = y / L.height;
        ev.tfinger.pressure = what == "up" ? 0.0f : 1.0f;
        ev.tfinger.windowID = SDL_GetWindowID(m_window.get());
        return SDL_PushEvent(&ev) ? Exec::Done : Exec::Failed;
    }
    if (c.op == "touchdrag") {
        // touchdrag [menu:ROW] DX DY [STEPS]: a finger drag by (DX, DY) points in STEPS frames (default 8), starting on
        // the label of a menu row, else over the menu lists (a quarter across, half way down the window).
        std::vector<std::string> a = c.args;
        int ww = 0, wh = 0;
        SDL_GetWindowSize(m_window.get(), &ww, &wh);
        float x = 0.25f * float(ww), y = 0.5f * float(wh);
        if (!a.empty() && a[0].rfind("menu:", 0) == 0) {
            const auto item = itemByName(a[0].substr(5));
            float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
            if (!item || !m_menu.itemRect(*item, x0, y0, x1, y1)) {
                std::printf("SCENARIO: menu row '%s' is not on screen\n", a[0].c_str());
                return Exec::Failed;
            }
            x = x0 + 0.15f * (x1 - x0);
            y = 0.5f * (y0 + y1);
            a.erase(a.begin());
        }
        if (a.size() < 2) return Exec::Failed;
        const int steps = a.size() >= 3 ? std::max(1, std::atoi(a[2].c_str())) : 8;
        queueTouch(x, y, float(std::atof(a[0].c_str())), float(std::atof(a[1].c_str())), steps);
        return Exec::Done;
    }
    if (c.op == "click" && arg.rfind("menu:", 0) == 0) {
        const auto item = itemByName(arg.substr(5));
        float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        if (!item || !m_menu.itemRect(*item, x0, y0, x1, y1)) {
            std::printf("SCENARIO: menu row '%s' is not on screen\n", arg.c_str());
            return Exec::Failed;
        }
        // The middle of the label (left part of the row), away from the value arrows.
        queueClick(x0 + 0.15f * (x1 - x0), 0.5f * (y0 + y1), false);
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
        // teleport N (feature index) or teleport WORD (first feature whose name contains it, case-insensitive).
        if (!arg.empty() && (std::isdigit(static_cast<unsigned char>(arg[0])) || arg[0] == '-')) {
            m_sim.requestTeleport(std::atoi(arg.c_str()));
            return Exec::Done;
        }
        auto lower = [](std::string v) {
            for (char& ch : v) ch = char(std::tolower(static_cast<unsigned char>(ch)));
            return v;
        };
        const auto& features = m_sim.track().features();
        for (std::size_t k = 0; k < features.size(); ++k)
            if (lower(features[k].name).find(lower(arg)) != std::string::npos) {
                m_sim.requestTeleport(int(k));
                return Exec::Done;
            }
        std::printf("SCENARIO: no teleport matching '%s'\n", arg.c_str());
        return Exec::Failed;
    }
    if (c.op == "autopilot") {
        // autopilot drive|off: the autopilot drives the world's attract route while driving (laps count as
        // best laps in scenario mode only).
        if (arg != "drive" && arg != "off") return Exec::Failed;
        m_scenarioAutopilot = arg == "drive";
        return Exec::Done;
    }
    if (c.op == "drift") {
        // drift on|off|level: force drift scoring on (with the world's zones, if any) or off, or back to the
        // world's setting (level.json "drift", the mini handling track on the proving ground).
        if (arg != "on" && arg != "off" && arg != "level") return Exec::Failed;
        m_sim.setDriftOverride(arg == "level" ? std::nullopt : std::optional<bool>(arg == "on"));
        return Exec::Done;
    }
    if (c.op == "reset") {
        m_sim.requestReset();
        return Exec::Done;
    }
    if (c.op == "level") {
        // level ID: switch the world like the start menu row (proving_ground or an assets/levels id).
        if (!selectLevel(arg)) {
            std::printf("SCENARIO: no level '%s'\n", arg.c_str());
            return Exec::Failed;
        }
        m_settings.level = arg;
        return Exec::Done;
    }
    if (c.op == "place") {
        // place X Z [YAW_DEG]: vehicle on the ground at (X, Z) of the current world.
        if (c.args.size() < 2) return Exec::Failed;
        const float x = float(std::atof(c.args[0].c_str())), z = float(std::atof(c.args[1].c_str()));
        TrackFeature at;
        at.name = "place";
        float y = 0.0f;
        if (const Level* level = m_sim.level()) {
            y = level->heightAt(x, z);
        } else if (auto hit = m_sim.castRayIgnoringVehicle(JPH::Vec3(x, 200.0f, z), JPH::Vec3(x, -50.0f, z))) {
            y = 200.0f - 250.0f * *hit;
        }
        at.position = JPH::Vec3(x, y, z);
        at.yaw = c.args.size() > 2 ? float(std::atof(c.args[2].c_str())) * 0.0174532925f : 0.0f;
        m_sim.requestPlace(at);
        return Exec::Done;
    }
    if (c.op == "map") {
        // map on | off: open the full-screen map from gameplay or the pause menu / close it (M, LB and the pause
        // menu row take the real input path). map fit | center: whole map / view on the vehicle.
        if (arg == "on" || arg == "1") {
            if (!m_mapUI.hasMap()) return Exec::Busy; // still building
            m_flow.openMap();
            openMap();
            return m_flow.screen() == Screen::Map ? Exec::Done : Exec::Failed;
        }
        if (arg == "off" || arg == "0") {
            if (m_flow.screen() == Screen::Map) closeMap();
            return Exec::Done;
        }
        if (arg == "fit" || arg == "center") {
            const ImVec2 d = ImGui::GetIO().DisplaySize;
            if (arg == "fit") {
                m_mapView = m_mapUI.fitView(MapUI::layout(d.x, d.y));
            } else {
                const MapPlayer p = mapPlayer();
                m_mapView.cx = p.x;
                m_mapView.cz = p.z;
            }
            return Exec::Done;
        }
        return Exec::Failed;
    }
    if (c.op == "mapclick" || c.op == "mapaim") {
        // mapclick X Z: a left click (real mouse path) on the world point (X, Z) of the open full-screen map.
        // mapaim X Z: centers the view on (X, Z) (the crosshair point of Space / pad A).
        if (m_flow.screen() != Screen::Map || c.args.size() < 2) return Exec::Failed;
        const float x = float(std::atof(c.args[0].c_str())), z = float(std::atof(c.args[1].c_str()));
        if (c.op == "mapaim") {
            m_mapView.cx = x;
            m_mapView.cz = z;
            return Exec::Done;
        }
        const MapPoint s = m_mapView.toScreen(x, z);
        queueClick(s.x, s.y, false);
        return Exec::Done;
    }
    if (c.op == "mapdrag") {
        // mapdrag DX DY: left-button drag over the map panel by (DX, DY) points, in four motion steps.
        if (m_flow.screen() != Screen::Map || c.args.size() < 2) return Exec::Failed;
        const float dx = float(std::atof(c.args[0].c_str())), dy = float(std::atof(c.args[1].c_str()));
        const float x = m_mapView.sx, y = m_mapView.sy;
        SDL_Event e{};
        e.type = SDL_EVENT_MOUSE_MOTION;
        e.motion.windowID = win;
        e.motion.x = x;
        e.motion.y = y;
        queueEvent(e, 0);
        SDL_Event down{};
        down.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
        down.button.windowID = win;
        down.button.button = SDL_BUTTON_LEFT;
        down.button.down = true;
        down.button.clicks = 1;
        down.button.x = x;
        down.button.y = y;
        queueEvent(down, 1);
        for (int i = 1; i <= 4; ++i) {
            e.motion.x = x + dx * float(i) / 4.0f;
            e.motion.y = y + dy * float(i) / 4.0f;
            e.motion.xrel = dx / 4.0f;
            e.motion.yrel = dy / 4.0f;
            queueEvent(e, 1 + i);
        }
        SDL_Event up = down;
        up.type = SDL_EVENT_MOUSE_BUTTON_UP;
        up.button.down = false;
        up.button.x = x + dx;
        up.button.y = y + dy;
        queueEvent(up, 6);
        return Exec::Done;
    }
    if (c.op == "marker") {
        // marker X Z | clear: sets the waypoint directly (the map's click path is mapclick).
        if (arg == "clear") {
            m_marker = {};
            return Exec::Done;
        }
        if (c.args.size() < 2) return Exec::Failed;
        m_marker = {true, float(std::atof(c.args[0].c_str())), float(std::atof(c.args[1].c_str()))};
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
    if (c.op == "post_bench") {
        // post_bench PASS N: record that post pass N times per frame (GPU cost measurement), N = 1 ends it.
        if (c.args.size() < 2) return Exec::Failed;
        m_renderer->setPostBench(c.args[0], std::atoi(c.args[1].c_str()));
        return Exec::Done;
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
        else if (k == "far_shadows") s.farShadows = v != 0.0f;
        else if (k == "env_probe") s.envProbe = int(v);
        else if (k == "env_probe_faces") s.envProbeFaces = int(v);
        else if (k == "parallax") s.parallaxSteps = int(v);
        else if (k == "ground_tiling") s.groundTiling = int(v);
        else if (k == "ground_detail") s.groundDetail = v != 0.0;
        else if (k == "ground_wet") s.groundWet = v != 0.0;
        else if (k == "ground_spec") s.groundSpecular = v != 0.0;
        else if (k == "micro_shadows") s.microShadowSteps = int(v);
        else if (k == "reflections") s.reflections = v != 0.0f;
        else if (k == "refl_scale") s.reflectionScale = v;
        else if (k == "bloom") s.bloom = v != 0.0f;
        else if (k == "grass") s.grassLevel = int(v);
        else if (k == "grass_dist") s.grassDistance = v;
        else if (k == "flowers") s.flowerDensity = v;
        else if (k == "grass_detail") s.grassDetail = v != 0.0f;
        else if (k == "clouds") s.clouds = v != 0.0f;
        else if (k == "spray_max") s.sprayMax = int(v);
        else if (k == "spray_dust") s.sprayDust = v != 0.0f;
        else if (k == "exposure") s.exposure = v;
        else if (k == "fog") s.fogDensity = v;
        else if (k == "water_refraction") s.waterRefraction = v != 0.0f;
        else if (k == "water_ssr") s.waterSsr = v != 0.0f;
        else if (k == "valley_fog") s.valleyFog = v;
        else if (k == "atmosphere") s.atmosphere = v;
        else if (k == "tree_dist") s.treeDistance = v;
        else if (k == "tree_detail") s.treeDetailDistance = v;
        else if (k == "terrain_detail") s.terrainDetail = v;
        else if (k == "terrain_tex") s.terrainTextureSize = int(v);
        else if (k == "taa") s.taa = v != 0.0f;
        else if (k == "ao") s.ao = int(v);
        else if (k == "contact_shadows") s.contactShadows = int(v);
        else if (k == "motion_blur") s.motionBlur = v;
        else if (k == "shafts") s.sunShafts = v;
        else if (k == "flare") s.lensFlare = v;
        else if (k == "auto_exposure") s.autoExposure = v != 0.0f;
        else if (k == "dof") s.dof = v != 0.0f;
        else if (k == "dof_driving") s.dofDriving = v;
        else {
            render = false;
            // Look-only settings: they do not turn the quality preset into Custom.
            if (k == "tod") { s.timeOfDay = v; m_gfx.changed = true; }
            else if (k == "tonemap") { s.tonemapper = int(v); m_gfx.changed = true; }
            else if (k == "grade") { s.grade = v; m_gfx.changed = true; }
            else if (k == "ao_strength") { s.aoStrength = v; m_gfx.changed = true; }
            else if (k == "vignette") { s.vignette = v; m_gfx.changed = true; }
            else if (k == "grain") { s.grain = v; m_gfx.changed = true; }
            else if (k == "chromatic") { s.chromaticAberration = v; m_gfx.changed = true; }
            else if (k == "sharpen") { s.sharpen = v; m_gfx.changed = true; }
            else if (k == "sun_heading") { s.sunHeadingDeg = v; m_gfx.changed = true; }
            else if (k == "tod_speed") m_gfx.timeLapseSpeed = v;
            else if (k == "tod_run") m_gfx.timeLapse = v != 0.0f;
            else if (k == "mud") { m_gfx.mud = std::clamp(v, 0.0f, 1.0f); m_gfx.autoMud = false; }
            else if (k == "auto_mud") m_gfx.autoMud = v != 0.0f;
            else if (k == "body_mud") m_bodyMud.setAll(v);
            else if (k == "body_mud_clear") m_bodyMud.clear();
            else if (k == "headlights") m_gfx.headlights = v != 0.0f;
            else if (k == "debug_lines") m_ui.drawPhysicsDebug = v != 0.0f;
            else if (k == "ui") m_ui.visible = v != 0.0f;
            else if (k == "hud") m_ui.hud = v != 0.0f;
            else if (k == "debug_windows") m_ui.debugWindows = m_applied.debugWindows = m_settings.debugWindows = v != 0.0f;
            else if (k == "time_flow") m_applied.timeFlow = m_settings.timeFlow = std::clamp(int(v), 0, 2);
            else if (k == "minimap") m_settings.minimap = v != 0.0f;
            else if (k == "compass") m_settings.compass = v != 0.0f;
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
            else if (k == "audio_motor") m_audioPanel.settings.motor = v;
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
    if (c.op == "wheels") {
        // wheels RADIUS WIDTH: a tire size change on both axles (what a tire kit would do); rebuilds the vehicle.
        if (c.args.size() < 2) return Exec::Failed;
        VehicleConfig vc = m_sim.vehicleConfig();
        const float r = float(std::atof(c.args[0].c_str())), w = float(std::atof(c.args[1].c_str()));
        if (!(r >= 0.2f && r <= 0.8f && w >= 0.1f && w <= 0.6f)) return Exec::Failed;
        for (AxleConfig& ax : vc.axles) {
            ax.wheelRadius = r;
            ax.wheelWidth = w;
        }
        m_sim.requestRebuild(vc);
        return Exec::Done;
    }
    if (c.op == "camera") {
        if (arg == "follow" || arg == "close" || arg == "hood") {
            m_camera.setMode(arg == "hood" ? Camera::Mode::Hood : arg == "close" ? Camera::Mode::CloseFollow : Camera::Mode::Follow);
            m_camera.fovYDeg = 60.0f;
            return Exec::Done;
        }
        if (arg == "vehicle") {
            // Near-orthographic views of the parked vehicle in its own frame: a far camera with a narrow
            // field of view (the top view is tilted 4 deg, the free camera's pitch limit).
            if (c.args.size() < 2) return Exec::Failed;
            const std::string& view = c.args[1];
            glm::vec3 dir;
            if (view == "side") dir = {1.0f, 0.0f, 0.0f};
            else if (view == "top") dir = {0.07f, 1.0f, 0.0f};
            else if (view == "front") dir = {0.0f, 0.0f, 1.0f};
            else if (view == "rear") dir = {0.0f, 0.05f, -1.0f};
            else if (view == "three_quarter") dir = {0.72f, 0.30f, 0.62f};
            else if (view == "three_quarter_rear") dir = {-0.72f, 0.30f, -0.62f};
            else if (view == "roof") dir = {-0.20f, 0.80f, -0.56f}; // high behind, onto the roof
            else return Exec::Failed;
            const float dist = c.args.size() > 2 ? float(std::atof(c.args[2].c_str())) : 30.0f;
            const float fov = c.args.size() > 3 ? float(std::atof(c.args[3].c_str()))
                                                : glm::degrees(2.0f * std::atan(3.0f / std::max(dist, 1.0f)));
            if (dist < 1.0f || fov < 1.0f || fov > 120.0f) return Exec::Failed;
            const glm::vec3 car = toGlm(m_sim.currentSnapshot().body.position);
            const glm::quat rot = toGlm(m_sim.currentSnapshot().body.rotation);
            const glm::vec3 aim = car + rot * glm::vec3(0.0f, 0.30f, 0.0f);
            m_camera.setFreePose(aim + rot * (glm::normalize(dir) * dist), aim);
            m_camera.fovYDeg = fov;
            return Exec::Done;
        }
        if (arg == "local") {
            // Free camera with position and target in the vehicle frame (close-ups of body details).
            if (c.args.size() < 7) return Exec::Failed;
            float v[6];
            for (int i = 0; i < 6; ++i) v[i] = float(std::atof(c.args[std::size_t(i + 1)].c_str()));
            const float fov = c.args.size() > 7 ? float(std::atof(c.args[7].c_str())) : 60.0f;
            if (fov < 1.0f || fov > 120.0f) return Exec::Failed;
            const glm::vec3 car = toGlm(m_sim.currentSnapshot().body.position);
            const glm::quat rot = toGlm(m_sim.currentSnapshot().body.rotation);
            m_camera.setFreePose(car + rot * glm::vec3(v[0], v[1], v[2]), car + rot * glm::vec3(v[3], v[4], v[5]));
            m_camera.fovYDeg = fov;
            return Exec::Done;
        }
        if (c.args.size() < 6) return Exec::Failed;
        float v[6];
        for (int i = 0; i < 6; ++i) v[i] = float(std::atof(c.args[std::size_t(i)].c_str()));
        m_camera.setFreePose({v[0], v[1], v[2]}, {v[3], v[4], v[5]});
        m_camera.fovYDeg = 60.0f;
        return Exec::Done;
    }
    if (c.op == "perf_begin") {
        m_perfActive = true;
        m_perfLabel = arg.empty() ? "perf" : arg;
        m_perfFrameMs.clear();
        m_perfGpuMs.clear();
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
        m_maxTilt = 0.0f;
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
    if (n == "touch_active") return m_input.touchControlsActive() ? 1.0 : 0.0;
    if (n == "touch_fingers") return m_input.touchControls().fingers();
    if (n == "touch_steer") return m_input.touchControls().steer();
    if (n == "touch_throttle") return m_input.touchThrottle();
    if (n == "touch_brake") return m_input.touchBrake();
    if (n == "headlights") return m_gfx.headlights ? 1.0 : 0.0;
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
    if (n == "wheels_contact") {
        int c = 0;
        for (const auto& w : t.wheels) c += w.contact;
        return double(c);
    }
    if (n == "x") return p.GetX();
    if (n == "y") return p.GetY();
    if (n == "z") return p.GetZ();
    if (n == "ticks") return double(m_sim.timing().ticks);
    if (n == "paused") return simulationRuns() ? 0.0 : 1.0;
    if (n == "frozen") return m_paused ? 1.0 : 0.0;
    if (n == "menu_screen") return double(int(m_flow.screen())); // 0 intro, 1 main menu, 2 playing, 3 paused
    if (n == "menu_page") return double(int(m_flow.page()));     // 0 main, 1 pause, 2 settings, 3 controls, 4 vehicle
    if (n == "autopilot") return double(int(m_sim.autopilotMode())); // 0 off, 1 drive, 2 hold
    if (n == "drift_active") return m_sim.drift().active() ? 1.0 : 0.0;
    if (n == "drift_running") return m_sim.drift().running() ? 1.0 : 0.0;
    if (n == "drift_drifting") return m_sim.drift().drifting() ? 1.0 : 0.0;
    if (n == "drift_in_area") return m_sim.drift().inArea() ? 1.0 : 0.0;
    if (n == "drift_angle") return std::abs(m_sim.drift().slipAngleDeg()); // deg, |body slip|
    if (n == "drift_score") return m_sim.drift().score();                  // running run
    if (n == "drift_max_angle") return m_sim.drift().maxAngleDeg();        // of the running / last run
    if (n == "drift_zone") return m_sim.drift().zone();                    // -1: none
    if (n == "drift_zone_time") return m_sim.drift().zoneTime();
    if (n == "drift_runs") return m_sim.drift().runs();
    if (n == "drift_failed") return m_sim.drift().failedRuns();
    if (n == "drift_judged") return m_sim.drift().runs() + m_sim.drift().failedRuns();
    if (n == "drift_last") return m_sim.drift().lastScore();
    if (n == "drift_best") return m_sim.drift().bestScore();
    if (n == "drift_end") return double(int(m_sim.drift().lastEnd())); // 0 none, 1 completed, 2 left area, 3 spin, 4 wall, 5 reset
    if (n == "drift_stored_best") return m_bestDrift.best(currentLevelId(), currentVehicleId());
    if (n == "lap_active") return m_sim.lapTimer().active() ? 1.0 : 0.0;
    if (n == "lap_running") return m_sim.lapTimer().running() ? 1.0 : 0.0;
    if (n == "lap_count") return m_sim.lapTimer().laps();
    if (n == "lap_time") return m_sim.lapTimer().lapTime(m_sim.simTime());
    if (n == "lap_last") return m_sim.lapTimer().lastLap();
    if (n == "lap_best") return m_sim.lapTimer().bestLap();
    if (n == "lap_stored_best") return m_bestLaps.best(currentLevelId(), currentVehicleId());
    if (n == "lap_saved") return double(m_bestLaps.size());
    if (n == "lap_best_gap") return m_sim.lapTimer().bestLap() - m_sim.lapTimer().lastLap();
    if (n == "lap_stored_gap") return m_bestLaps.best(currentLevelId(), currentVehicleId()) - m_sim.lapTimer().bestLap();
    if (n == "lap_next") return m_sim.lapTimer().nextGate();
    if (n == "lap_last_gate") return m_sim.lapTimer().lastGate();
    if (n == "lap_checkpoints") return m_sim.lapTimer().checkpointsPassed();
    if (n == "lap_gates") return double(m_sim.lapTimer().gates().size());
    if (n == "lap_sectors") return m_sim.lapTimer().sectorCount();
    if (n.rfind("lap_sector_", 0) == 0) return m_sim.lapTimer().lastSectorTime(std::atoi(n.c_str() + 11) - 1); // last lap, 1-based
    if (n.rfind("lap_gate_dist_", 0) == 0) { // distance from the body to gate N, m
        const int g = std::atoi(n.c_str() + 14);
        const auto& gates = m_sim.lapTimer().gates();
        if (g < 0 || g >= int(gates.size())) return std::nullopt;
        return std::hypot(p.GetX() - gates[std::size_t(g)].x, p.GetZ() - gates[std::size_t(g)].z);
    }
    if (n == "autopilot_driven") return m_sim.autopilot().distanceDriven(); // m along the route since it took over
    if (n == "autopilot_progress") return m_sim.autopilot().progress();     // m from the route start
    if (n == "autopilot_lateral") return m_sim.autopilot().lateralError();
    if (n == "autopilot_resets") return m_sim.autopilotResets();
    if (n == "lap_next_gate") return m_sim.lapTimer().nextGate(); // = lap_next
    if (n == "vehicle_index") return m_garage ? m_garage->vehicleIndex() : 0.0; // catalog order, 0 = default vehicle
    if (n == "vehicle_choice") return m_flow.vehicleIndex(m_settings);         // the vehicle page's choice
    if (n == "vehicle_cards") return double(m_menu.vehicleCardCount());
    if (n == "electric") return m_sim.vehicleConfig().isElectric() ? 1.0 : 0.0;
    if (n == "direction") return t.direction == DriveDirection::Reverse ? 1.0 : 0.0; // 0 drive, 1 reverse
    if (n == "mass") return m_sim.vehicleConfig().mass;
    if (n == "soc") return t.ev.soc;                       // 0..1 (electric)
    if (n == "motor_kw") return t.ev.motorPowerKW;         // both motors, + drive, - regen
    if (n == "battery_kw") return t.ev.batteryPowerKW;     // incl. the auxiliary load
    if (n == "motor_nm_f") return t.ev.motorTorque[0];
    if (n == "motor_nm_r") return t.ev.motorTorque[1];
    if (n == "motor_rpm_f") return t.ev.motorRPM[0];
    if (n == "motor_rpm_r") return t.ev.motorRPM[1];
    if (n == "regen") return t.ev.regenActive ? 1.0 : 0.0;
    if (n == "ev_mode") return double(int(t.ev.mode));    // 0 normal, 1 off-road
    if (n == "ev_front_share") return t.ev.frontShare;
    if (n == "energy_out_kwh") return t.ev.energyOutKWh;
    if (n == "energy_in_kwh") return t.ev.energyInKWh;
    if (n == "menu_depth") return m_flow.depth();
    if (n == "menu_cursor") return m_flow.cursor();
    if (n == "menu_scroll") return m_menu.menuScroll();
    if (n == "garage_scroll") return m_menu.garageScroll();
    if (n == "menu_fade") return m_flow.menuFade();
    if (n == "menu_blur") return m_menuBlur;
    if (n == "dof_amount") return m_dofAmount;
    if (n == "menu_font") return m_menu.fontsLoaded() ? 1.0 : 0.0;
    if (n == "menu_logo") return m_menu.logoLoaded() ? 1.0 : 0.0;
    if (n.rfind("setting_", 0) == 0) {
        const GameSettings& g = m_settings;
        const std::string k = n.substr(8);
        if (k == "quality") return g.quality;
        if (k == "render_scale") return g.renderScale;
        if (k == "grass_distance") return g.grassDistance;
        if (k == "tree_distance") return g.treeDistance;
        if (k == "vsync") return g.vsync ? 1.0 : 0.0;
        if (k == "tree_style") return g.stylizedTrees ? 0.0 : 1.0; // 0 stylized, 1 detailed
        if (k == "master") return g.masterVolume;
        if (k == "music") return g.musicVolume;
        if (k == "effects") return g.effectsVolume;
        if (k == "ui") return g.uiVolume;
        if (k == "abs") return g.abs ? 1.0 : 0.0;
        if (k == "tc") return g.traction;
        if (k == "diff") return g.diffLocks;
        if (k == "auto_reverse") return g.autoReverse ? 1.0 : 0.0;
        if (k == "drive_keys") return g.driveKeys;
        if (k == "pad_glyphs") return g.padGlyphs;
        if (k == "minimap") return g.minimap ? 1.0 : 0.0;
        if (k == "minimap_size") return g.minimapSize;
        if (k == "minimap_north_up") return g.minimapNorthUp ? 1.0 : 0.0;
        if (k == "compass") return g.compass ? 1.0 : 0.0;
        if (k == "time_of_day") return g.timeOfDay;
        if (k == "time_flow") return g.timeFlow;
        if (k == "debug_windows") return g.debugWindows ? 1.0 : 0.0;
        if (k == "vehicle") return m_flow.vehicleIndex(g);
        return std::nullopt;
    }
    // The settings file format keeps every value of the current settings (parse(serialize(s)) == s).
    if (n == "settings_roundtrip") return parseSettings(serializeSettings(m_settings)) == m_settings ? 1.0 : 0.0;
    if (n == "body_glb") return m_sim.vehicleConfig().bodyModel.empty() ? 0.0 : 1.0; // running vehicle has a body.glb
    if (n == "body_parts") return double(m_vehicleVisual.parts.size());
    if (n == "body_triangles") return double(m_vehicleVisual.mesh.vertices.size() / 3);
    if (n == "body_hulls") return double(m_sim.vehicleConfig().bodyHulls.size());
    if (n == "drive_keys") return double(int(m_input.driveKeys()));
    if (n == "kb_style") return double(int(keyboardStyle())); // 0 Mac, 1 PC
    if (n == "pad_glyphs") return double(int(resolvePadGlyphs(PadGlyphSetting(m_settings.padGlyphs), padType())));
    if (n == "controls_callouts") return m_menu.controlsCallouts();
    if (n == "controls_overlaps") return m_menu.controlsOverlaps();
    if (n == "cam_free") return m_camera.mode() == Camera::Mode::Free ? 1.0 : 0.0;
    if (n == "cam_mode") return double(int(m_camera.mode())); // 0 follow, 1 close follow, 2 hood, 3 free
    if (n == "cam_road_ahead") {
        // Nearest ground point straight ahead of the front bumper (0.25 m steps, up to 40 m) that is in the view and
        // not hidden by the vehicle's collision shape, m ahead of the bumper; -1 if none.
        const glm::vec3 car = toGlm(m_sim.currentSnapshot().body.position);
        const glm::vec3 fwd = toGlm(m_sim.currentSnapshot().body.rotation) * glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3 flat = glm::normalize(glm::vec3(fwd.x, 0.0f, fwd.z));
        int pw = 0, ph = 0;
        SDL_GetWindowSizeInPixels(m_window.get(), &pw, &ph);
        const glm::mat4 vp = m_camera.projection(float(pw) / float(std::max(ph, 1))) * m_camera.view();
        const glm::vec3 cam = m_camera.position();
        const float front = m_sim.vehicle().bodyBounds().mMax.GetZ();
        for (float a = 0.0f; a <= 40.0f; a += 0.25f) {
            const glm::vec3 xz = car + flat * (front + a);
            const JPH::Vec3 above(xz.x, xz.y + 20.0f, xz.z), below(xz.x, xz.y - 20.0f, xz.z);
            const auto g = m_sim.castRayIgnoringVehicle(above, below);
            if (!g) continue;
            const glm::vec3 p(xz.x, above.GetY() + (below.GetY() - above.GetY()) * *g + 0.02f, xz.z);
            const glm::vec4 clip = vp * glm::vec4(p, 1.0f);
            if (clip.w <= 0.0f || std::abs(clip.x / clip.w) > 1.0f || std::abs(clip.y / clip.w) > 1.0f) continue;
            if (m_sim.castRayAtVehicle(JPH::Vec3(cam.x, cam.y, cam.z), JPH::Vec3(p.x, p.y, p.z))) continue;
            return a;
        }
        return -1.0;
    }
    if (n == "cam_behind" || n == "cam_height") {
        // Camera behind the rear bumper (along the vehicle's level heading) / above the body origin, m.
        const glm::vec3 car = toGlm(m_sim.currentSnapshot().body.position);
        const glm::vec3 fwd = toGlm(m_sim.currentSnapshot().body.rotation) * glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3 d = m_camera.position() - car;
        if (n == "cam_height") return d.y;
        const glm::vec2 flat = glm::normalize(glm::vec2(fwd.x, fwd.z));
        return -glm::dot(glm::vec2(d.x, d.z), flat) + m_sim.vehicle().bodyBounds().mMin.GetZ();
    }
    if (n == "cam_orbit_yaw") return glm::degrees(m_camera.orbitYaw());
    if (n == "cam_orbit_pitch") return glm::degrees(m_camera.orbitPitch());
    if (n == "cam_look_yaw") return glm::degrees(m_camera.hoodLookYaw());
    if (n == "cam_look_pitch") return glm::degrees(m_camera.hoodLookPitch());
    if (n == "grass_dist") return m_gfx.settings.grassDistance;
    if (n == "tree_detail") return m_gfx.settings.treeDetailDistance;
    if (n == "tree_dist") return m_gfx.settings.treeDistance;
    if (n == "pad_connected") return m_input.gamepadConnected() ? 1.0 : 0.0;
    if (n == "pad_virtual") return m_virtualPadId && m_input.gamepadId() == m_virtualPadId ? 1.0 : 0.0;
    if (n == "pad_right_x") return m_input.rightStick().x;
    if (n == "pad_right_y") return m_input.rightStick().y;
    if (n == "ui_visible") return m_ui.visible ? 1.0 : 0.0;
    if (n == "ui_kb") return ImGui::GetIO().WantCaptureKeyboard ? 1.0 : 0.0;
    if (n == "ui_text") return ImGui::GetIO().WantTextInput ? 1.0 : 0.0;
    if (n == "cam_clearance") return m_camClearance;
    if (n == "min_cam_clearance") return m_minCamClearance;
    if (n == "max_air") return m_maxAirWheels;
    if (n == "max_tilt") return m_maxTilt;
    if (n == "garage_open") return m_flow.screen() == Screen::Garage ? 1.0 : 0.0;
    if (n == "map_open") return m_flow.screen() == Screen::Map ? 1.0 : 0.0;
    if (n == "map_ready") return m_mapUI.hasMap() ? 1.0 : 0.0;
    if (n == "map_build_ms") return m_mapBuildMs;
    if (n == "map_pois") return double(m_mapUI.data().pois.size());
    if (n == "map_scale") return m_mapView.scale; // points per meter
    if (n == "map_cx") return m_mapView.cx;
    if (n == "map_cz") return m_mapView.cz;
    if (n == "marker_set") return m_marker.set ? 1.0 : 0.0;
    if (n == "marker_x") return m_marker.x;
    if (n == "marker_z") return m_marker.z;
    if (n == "marker_dist" || n == "marker_bearing") {
        // Horizontal distance (m) and bearing relative to the vehicle heading (deg, + = right) as the compass shows.
        if (!m_marker.set) return -1.0;
        const MapPlayer p = mapPlayer();
        if (n == "marker_dist") return std::hypot(m_marker.x - p.x, m_marker.z - p.z);
        return relativeBearingDeg(bearingDeg(p.x, p.z, m_marker.x, m_marker.z), p.headingDeg);
    }
    if (n == "heading") return mapPlayer().headingDeg;
    if (m_garage) {
        if (n == "garage_cursor") return m_garage->cursor();
        if (n == "garage_modal") return double(int(m_garage->modal())); // 0 none, 1 rename, 2 delete, 3 reset
        if (n == "garage_modal_error") return m_garage->modalError().empty() ? 0.0 : 1.0;
        if (n == "garage_rename_len") return double(m_garage->renameText().size());
        if (n == "garage_rate_front" || n == "garage_rate_rear") {
            // Wheel rate per corner (N/mm) shown next to the spring frequency.
            const int a = n == "garage_rate_front" ? kFrontAxle : kRearAxle;
            const VehicleConfig& gc = m_garage->config();
            return wheelRateFromFrequency(gc.tuning.axles[std::size_t(a)].springFrequency, sprungCornerMass(gc, a)) / 1000.0;
        }
        if (n == "garage_dirty") return m_garage->dirty() ? 1.0 : 0.0;
        if (n == "garage_setups") return double(m_garage->setups().setups.size());
        if (n == "garage_factory") return m_garage->factoryActive() ? 1.0 : 0.0;
        if (n == "garage_renaming") return m_garage->renaming() ? 1.0 : 0.0;
        if (n == "garage_pending") return m_garage->needsApply() ? 1.0 : 0.0;
        // garage:KEY = the garage's value, cfg:KEY = the same parameter read from the running vehicle (stored units).
        if (n.rfind("garage:", 0) == 0 || n.rfind("cfg:", 0) == 0) {
            const bool garage = n[0] == 'g';
            const SetupParamInfo* sp = findSetupParam(n.substr(garage ? 7 : 4));
            if (!sp) return std::nullopt;
            return garage ? m_garage->value(*sp) : setupValue(*sp, m_sim.vehicleConfig(), m_garage->kitBase());
        }
    }
    if (n == "text_input") return SDL_TextInputActive(m_window.get()) ? 1.0 : 0.0;
    if (n == "layout") return double(int(t.layout));
    if (n == "transfer") return double(int(t.transfer));
    if (n == "front_driven") return t.frontDriven ? 1.0 : 0.0;
    if (n == "rear_driven") return t.rearDriven ? 1.0 : 0.0;
    if (n == "wheel_radius_r") return m_sim.vehicleConfig().axles[kRearAxle].wheelRadius;
    if (n == "wheel_width_r") return m_sim.vehicleConfig().axles[kRearAxle].wheelWidth;
    if (n == "wheel_visual_radius_r") return m_wheelVisualSize[kRearAxle].x;
    if (n == "wheel_visual_width_r") return m_wheelVisualSize[kRearAxle].y;
    if (n == "wheel_fit_err") {
        // Largest difference between a wheel mesh as drawn and its physics wheel (radius, width), m.
        double e = m_visualFit && sameWheelFit(*m_visualFit, m_sim.vehicleConfig()) ? 0.0 : 1.0;
        for (std::size_t a = 0; a < 2; ++a) {
            const AxleConfig& ax = m_sim.vehicleConfig().axles[a];
            e = std::max({e, double(std::abs(m_wheelVisualSize[a].x - ax.wheelRadius)),
                          double(std::abs(m_wheelVisualSize[a].y - ax.wheelWidth))});
        }
        return e;
    }
    if (n == "front_spring") return m_sim.vehicleConfig().tuning.axles[kFrontAxle].springFrequency;
    if (n == "mass") return m_sim.vehicleConfig().mass;
    if (n == "soc") return t.ev.soc;                       // 0..1 (electric)
    if (n == "motor_kw") return t.ev.motorPowerKW;         // both motors, + drive, - regen
    if (n == "battery_kw") return t.ev.batteryPowerKW;     // incl. the auxiliary load
    if (n == "motor_nm_f") return t.ev.motorTorque[0];
    if (n == "motor_nm_r") return t.ev.motorTorque[1];
    if (n == "motor_rpm_f") return t.ev.motorRPM[0];
    if (n == "motor_rpm_r") return t.ev.motorRPM[1];
    if (n == "regen") return t.ev.regenActive ? 1.0 : 0.0;
    if (n == "ev_mode") return double(int(t.ev.mode));    // 0 normal, 1 off-road
    if (n == "ev_front_share") return t.ev.frontShare;
    if (n == "energy_out_kwh") return t.ev.energyOutKWh;
    if (n == "energy_in_kwh") return t.ev.energyInKWh;
    if (n == "tire_set") return double(int(m_sim.vehicleConfig().tireSet)); // 0 offroad, 1 road, 2 slick
    if (n == "ambient") return m_sim.ambientTemperature();
    if (n == "debug_hud") return m_ui.item("hud") ? 1.0 : 0.0; // the top-left read-out drawn this frame
    if (n == "tire_hud" || n == "tire_temps") {
        // 1 while the compact HUD widget / the telemetry grid is drawn this frame.
        const UiItemRect* r = m_ui.item(n);
        return r && r->visible ? 1.0 : 0.0;
    }
    if (n.rfind("tire_", 0) == 0) {
        // tire_surface_fl, tire_core_rr, tire_grip_fr, tire_kpa_rl: per-wheel tire thermal state; tire_surface_max /
        // tire_surface_min over the four.
        static const char* kWheelKeys[] = {"fl", "fr", "rl", "rr"};
        const std::string rest = n.substr(5);
        if (rest == "surface_max" || rest == "surface_min") {
            double v = rest == "surface_max" ? -1e9 : 1e9;
            for (const WheelTelemetry& w : t.wheels) v = rest == "surface_max" ? std::max(v, double(w.tireSurfaceC)) : std::min(v, double(w.tireSurfaceC));
            return v;
        }
        for (int i = 0; i < kWheelCount; ++i) {
            const WheelTelemetry& w = t.wheels[std::size_t(i)];
            const std::string k = kWheelKeys[i];
            if (rest == "surface_" + k) return w.tireSurfaceC;
            if (rest == "core_" + k) return w.tireCoreC;
            if (rest == "grip_" + k) return w.tireGrip;
            if (rest == "kpa_" + k) return w.tirePressureKPa;
        }
    }
    if (n == "static_grip") return m_sim.vehicleConfig().tuning.tireStaticGrip ? 1.0 : 0.0;
    if (n == "friction_ellipse") return m_sim.vehicleConfig().tuning.tireFrictionEllipse ? 1.0 : 0.0;
    if (n == "clutch_limit") return m_sim.vehicleConfig().tuning.launchClutchLimit ? 1.0 : 0.0;
    if (n == "fps") return m_fps;
    if (n == "frame_ms") return m_frameMs;
    if (n == "cpu_ms") return m_cpuMs;
    if (n == "gpu_ms") return m_renderer->stats().gpuMs;
    if (n == "mud") return m_gfx.mud;
    if (n == "body_mud_max") return m_bodyMud.maxAmount();
    if (n.rfind("body_mud_", 0) == 0) {
        // body_mud_<region>: arch_rear_<w>, arch_front_<w>, behind_<w> (w: fl fr rl rr), rear_bumper, front_bumper.
        static const char* kW[4] = {"fl", "fr", "rl", "rr"};
        const std::string r = n.substr(9);
        if (r == "rear_bumper") return m_bodyMud.coat(kMudRearBumper).amount;
        if (r == "front_bumper") return m_bodyMud.coat(kMudFrontBumper).amount;
        for (int w = 0; w < 4; ++w) {
            if (r == std::string("arch_rear_") + kW[w]) return m_bodyMud.coat(kMudArchRear + w).amount;
            if (r == std::string("arch_front_") + kW[w]) return m_bodyMud.coat(kMudArchFront + w).amount;
            if (r == std::string("behind_") + kW[w]) return m_bodyMud.coat(kMudBehind + w).amount;
        }
    }
    if (n == "spray_fine") return m_spray.fineShare();
    if (n == "tod") return m_gfx.settings.timeOfDay;
    if (n == "tod_run") return m_gfx.timeLapse ? 1.0 : 0.0;
    if (n == "sun_el" || n == "sun_az") {
        const SkyState sky = computeSky(m_gfx.settings.timeOfDay, m_gfx.settings.sunHeadingDeg);
        return n == "sun_el" ? sky.sunElevationDeg : sky.sunAzimuthDeg;
    }
    if (n == "trees") {
        const auto& t = m_renderer->stats().trees;
        return t[0] + t[1] + t[2] + t[3];
    }
    if (n == "quality") return double(int(m_gfx.preset));
    if (n == "level") return m_sim.level() ? 1.0 : 0.0; // 1 on an open-world level, 0 on the proving ground
    if (n == "level_choice") return m_flow.levelIndex(m_settings);
    if (n == "submersion") return m_sim.vehicleSubmersion();
    if (const Level* level = m_sim.level()) {
        // Water over the ground at the body position (m, 0 when dry) and the distance to the nearest road center.
        if (n == "above_grid") return double(p.GetY() - level->heightAt(p.GetX(), p.GetZ())); // body over the height grid
        if (n == "water_depth") {
            const std::optional<float> w = level->waterAt(p.GetX(), p.GetZ());
            return w ? std::max(0.0, double(*w - level->heightAt(p.GetX(), p.GetZ()))) : 0.0;
        }
        if (n == "road_dist") {
            const TrackFeature r = level->nearestRoadPoint(p.GetX(), p.GetZ());
            return std::hypot(double(r.position.GetX() - p.GetX()), double(r.position.GetZ() - p.GetZ()));
        }
    }
    if (n == "recoveries") return m_sim.recoveries();
    if (n == "terrain_chunks") return m_renderer->stats().terrainChunks;
    if (n == "terrain_tris") return double(m_renderer->stats().terrainTriangles);
    if (n == "terrain_tex") return m_renderer->stats().terrainTexAlbedo;
    if (n == "terrain_tex_nra") return m_renderer->stats().terrainTexNra;
    if (n == "terrain_tex_mb") return double(m_renderer->stats().terrainTexBytes) / 1e6;
    if (n == "terrain_tex_ms") return m_renderer->stats().terrainTexLoadMs;
    if (n == "terrain_tex_loads") return m_renderer->stats().terrainTexLoads;
    if (n == "buildings") return m_sim.level() ? double(m_sim.level()->buildings().size()) : 0.0;
    if (n == "model_instances") return m_renderer->stats().modelInstances;
    if (n == "model_tris") return double(m_renderer->stats().modelTriangles);
    if (n == "model_draws") return m_renderer->stats().modelDraws;
    if (const TerrainTiles* tt = m_sim.terrain()) {
        const TerrainStreamingStats& st = tt->stats();
        if (n == "tiles_loaded") return st.loaded;
        if (n == "tiles_async") return st.builtAsync;
        if (n == "tiles_sync") return st.builtSync;
        if (n == "tiles_unloaded") return st.unloaded;
        if (n == "tile_build_ms") return st.maxBuildMs;
        if (n == "tile_stall_ms") return st.maxSyncStallMs;
        if (n == "tile_update_ms") return st.maxUpdateMs;
        if (n == "tile_x") return tt->tileIndexOf(float(p.GetX()));
        if (n == "tile_z") return tt->tileIndexOf(float(p.GetZ()));
    }
    if (n == "msaa") return m_renderer->settings().msaa;
    if (n == "post_taa") return m_renderer->stats().taa ? 1.0 : 0.0;
    if (n == "post_ao") return m_renderer->stats().ao ? 1.0 : 0.0;
    if (n == "post_contact") return m_renderer->stats().contactShadows ? 1.0 : 0.0;
    if (n == "far_shadows") return m_renderer->stats().farShadows ? 1.0 : 0.0;
    if (n == "far_shadow_updates") return m_renderer->stats().farShadowUpdates;
    if (n == "env_probe") return m_renderer->stats().envProbe ? 1.0 : 0.0;
    if (n == "env_probe_faces") return m_renderer->stats().envProbeFaces;
    if (n == "post_motion_blur") return m_renderer->stats().motionBlur ? 1.0 : 0.0;
    if (n == "post_dof") return m_renderer->stats().dof ? 1.0 : 0.0;
    if (n == "post_shafts") return m_renderer->stats().shafts ? 1.0 : 0.0;
    if (n == "post_flare") return m_renderer->stats().flareSun;
    if (n == "post_velocity_objects") return m_renderer->stats().velocityObjects;
    if (n == "auto_exposure_ev") return m_renderer->stats().autoExposureEv;
    if (n == "scene_log_lum") return m_renderer->stats().sceneLogLum;
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
        // marks_s<SurfaceId>: live segments on that surface (0 Concrete .. 7 Snow, 8 Asphalt, 9 Gravel).
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
        if (n == "audio_music") return a.musicGain;
        if (n == "audio_music_t") return a.musicSeconds;
        if (n == "audio_voices") return a.activeVoices;
        if (n == "audio_oneshots") return a.oneShotsPlayed;
        if (n == "audio_peak_db") return a.peakDb;
        if (n == "audio_clipped") return double(a.clippedSamples);
        if (n == "audio_cpu") return a.cpuLoad;
        if (n == "audio_motor_hz") return a.motorHz;
        if (n == "audio_motor") return a.motorLevel;
        if (n == "audio_inverter") return a.inverterLevel;
        if (n == "audio_road") return a.road;
        if (n == "audio_src_rpm_raw") return a.rpm;
        if (n == "audio_rattle") return a.rattle;
        if (n == "audio_knocks") return a.knocks;
        // audio_ev<N>: events of AudioEventType N received by the mixer (0 start, 1 gear, 2 bump stop, 3 body rattle,
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
    if (n == "susp_spread" || n == "susp_min" || n == "susp_max") { // suspension lengths of the four wheels, m
        float lo = 1e9f, hi = -1e9f;
        for (const auto& w : t.wheels) lo = std::min(lo, w.suspensionLength), hi = std::max(hi, w.suspensionLength);
        return double(n == "susp_min" ? lo : n == "susp_max" ? hi : hi - lo);
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
    if (n == "surface") { // SurfaceId under most of the wheels in contact, -1 in the air
        int count[kSurfaceCount] = {}, best = -1;
        for (const auto& w : t.wheels)
            if (w.contact) ++count[int(w.surface)];
        for (int i = 0; i < int(kSurfaceCount); ++i)
            if (count[i] > 0 && (best < 0 || count[i] > count[best])) best = i;
        return double(best);
    }
    return std::nullopt;
}

int Application::run() {
    const double freq = double(SDL_GetPerformanceFrequency());
    Uint64 last = SDL_GetPerformanceCounter();
    while (m_running) {
        const Uint64 now = SDL_GetPerformanceCounter();
        const double realFrameTime = double(now - last) / freq;
        last = now;
        const bool fixedStep = m_scenario && !m_perfActive;
        const double frameTime = fixedStep ? kScenarioFrameStep : realFrameTime;
        m_frameDt = frameTime;
        m_time += frameTime;

        m_fpsAccum += float(realFrameTime);
        ++m_fpsFrames;
        if (m_fpsAccum >= 0.5f) {
            m_fps = float(m_fpsFrames) / m_fpsAccum;
            m_frameMs = 1000.0f * m_fpsAccum / float(m_fpsFrames);
            m_fpsAccum = 0.0f;
            m_fpsFrames = 0;
        }

        m_flow.update(std::min(frameTime, 0.1));
        processEvents();
        if (m_inBackground || (SDL_GetWindowFlags(m_window.get()) & SDL_WINDOW_MINIMIZED)) {
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
            m_perfFrameMs.push_back(realFrameTime * 1000.0);
            m_perfCpuSum += m_cpuMs;
            if (m_renderer->stats().gpuMs > 0.0) {
                m_perfGpuSum += m_renderer->stats().gpuMs;
                ++m_perfGpuCount;
                m_perfGpuMs.push_back(m_renderer->stats().gpuMs);
            }
        }

        if (fixedStep) {
            // Never faster than real time (vsync off): background work such as tile streaming keeps its real pace.
            const double spent = double(SDL_GetPerformanceCounter() - now) / freq;
            if (spent < kScenarioFrameStep) SDL_DelayPrecise(Uint64((kScenarioFrameStep - spent) * 1e9));
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
