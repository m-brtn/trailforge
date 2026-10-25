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
    {"vsync", MenuItem::VSync},         {"master", MenuItem::MasterVolume},   {"music", MenuItem::MusicVolume},
    {"effects", MenuItem::EffectsVolume}, {"ui", MenuItem::UiVolume},         {"abs", MenuItem::Abs},
    {"tc", MenuItem::Traction},         {"diff", MenuItem::DiffLocks},        {"auto_reverse", MenuItem::AutoReverse},
    {"controls", MenuItem::Controls},   {"back", MenuItem::Back},             {"drive_keys", MenuItem::DriveKeys},
    {"pad_glyphs", MenuItem::PadGlyphs},
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
    m_ui.visible = options.startWithUi;

    const Uint64 t0 = SDL_GetPerformanceCounter();
    rebuildVehicleVisual();
    const MeshData& bodyMesh = m_vehicleVisual.mesh;
    buildWorldContent();
    m_springMesh = m_renderer->upload(buildUnitSpringMesh());
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
    }

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
        break;
    }
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
    m_gameplayTimeOfDay = m_gfx.settings.timeOfDay;
    if (m_flow.menuActive()) enterMenuWorld(false);
}

Application::~Application() {
    m_audio.reset(); // stop the audio thread first
    // Mesh GPU resources are released before the device is destroyed.
    if (m_renderer) SDL_WaitForGPUIdle(m_renderer->device());
    m_menu.release();
    for (GpuMesh* m : {&m_trackMesh, &m_sceneryMesh, &m_waterMesh, &m_bodyMesh, &m_wheelMesh, &m_rearWheelMesh, &m_springMesh,
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
    m_input.endEvents(io.WantCaptureKeyboard, io.WantCaptureMouse, float(std::min(io.DeltaTime, 0.1f)));

    if (m_input.pressed(Action::Quit)) m_running = false;
    if (m_input.pressed(Action::ToggleDebugUI)) m_ui.visible = !m_ui.visible;
    if (m_input.pressed(Action::ToggleGraphicsUI)) m_gfx.visible = !m_gfx.visible;
    if (m_input.pressed(Action::ToggleAudioUI)) m_audioPanel.visible = !m_audioPanel.visible;
    if (m_flow.screen() == Screen::Garage) handleGarage();
    else handleMenu();
    if (m_flow.menuActive()) {
        // Menus own the input: no gameplay actions, no pedals.
        m_sim.setIntent(DriverIntent{});
        return;
    }
    if (m_input.pressed(Action::ToggleHeadlights)) m_gfx.headlights = !m_gfx.headlights;
    if (m_input.pressed(Action::TogglePause)) setPaused(!m_paused);
    if (m_input.pressed(Action::StepOnce) && m_paused) m_stepOnce = true;
    if (m_input.pressed(Action::ToggleCamera)) m_camera.toggleMode();
    if (m_input.pressed(Action::ToggleMap)) m_mapOpen = !m_mapOpen && m_sim.level();
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
        if (k.mouseMoved) in.hover = m_menu.hitTest(k.mouseX, k.mouseY, &dir);
        if (k.click) {
            in.click = m_menu.hitTest(k.mouseX, k.mouseY, &dir);
            in.clickDir = dir;
        }
        if (m_flow.page() == MenuPage::Vehicle) {
            if (k.click) in.pick = m_menu.vehicleCardHit(k.mouseX, k.mouseY);
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
    case MenuCommand::VehicleChanged:
        selectVehicle(m_settings.vehicle);
        saveSettings();
        break;
    case MenuCommand::LevelChanged:
        if (!selectLevel(m_settings.level)) std::printf("Levels: cannot load '%s'\n", m_settings.level.c_str());
        m_mapOpen = false;
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
    m_garageFollowCamera = m_camera.mode() == Camera::Mode::Follow;
    m_garageCamTime = 0.0;
}

void Application::closeGarage() {
    SDL_StopTextInput(m_window.get());
    m_input.setModalCapture(false, false);
    m_flow.closeGarage();
    if (m_garageFollowCamera && m_camera.mode() == Camera::Mode::Free) m_camera.toggleMode();
    if (m_flow.screen() == Screen::MainMenu) m_menuCamTime = 0.0;
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
        if (k.mouseMoved && !k.mouseDown) in.hover = m_menu.garageHitTest(k.mouseX, k.mouseY, &dir, &fraction);
        if (k.click || (k.mouseDown && k.mouseMoved)) {
            const int row = m_menu.garageHitTest(k.mouseX, k.mouseY, &dir, &fraction);
            // Dragging only moves sliders; a plain move with the button held does not activate rows.
            if (k.click || (fraction >= 0.0f && row == m_garage->cursor())) {
                in.click = row;
                in.clickDir = dir;
                in.clickFraction = fraction;
            }
        }
        if (in.hover >= 0 || in.click >= 0) m_menu.notePointerCursor();
        if (k.wheel != 0.0f) m_menu.scroll(k.wheel);
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
    m_sim.requestReset();
    AidControls aids;
    aids.abs = m_settings.abs;
    aids.traction = TractionMode(std::max(m_settings.traction, 0));
    aids.diffLocks = m_settings.diffLocks;
    m_sim.requestAidControls(aids);
    if (m_settings.traction < 0) m_sim.requestDefaultTraction();
    m_ui.autoReverse = m_settings.autoReverse;
    if (m_camera.mode() == Camera::Mode::Free) m_camera.toggleMode();
    m_gfx.settings.timeOfDay = m_gameplayTimeOfDay;
    m_gfx.changed = true;
    m_musicTarget = 0.0f;
    m_musicFade = 3.0f;
}

// Main menu (or intro) world: the vehicle back at the start, a sunset, the cinematic camera, the music from the top.
void Application::enterMenuWorld(bool restartMusic) {
    if (restartMusic) {
        ++m_musicRestart;
        m_gameplayTimeOfDay = m_gfx.settings.timeOfDay;
        m_sim.requestReset();
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
    if (force || s.quality != m_applied.quality || s.renderScale != m_applied.renderScale) {
        const QualityPreset preset = QualityPreset(s.quality);
        RenderSettings r = presetSettings(preset);
        copyLook(r, m_gfx.settings);
        if (s.renderScale > 0.0f) r.renderScale = s.renderScale;
        m_gfx.preset = r.renderScale == presetSettings(preset).renderScale ? preset : QualityPreset::Custom;
        if (!force || r.renderScale != m_gfx.settings.renderScale || m_gfx.preset != QualityPreset::High) {
            m_gfx.settings = r;
            m_gfx.changed = true;
        }
    }
    if (force || s.vsync != m_applied.vsync) m_renderer->setVSync(s.vsync);
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
    m_input.setDriveKeys(DriveKeys(s.driveKeys));
    m_ui.driveHint = driveHint(DriveKeys(s.driveKeys), keyboardStyle());
    m_applied = s;
}

void Application::saveSettings() {
    if (m_settingsPath.empty()) return;
    std::string error;
    if (saveSettingsFile(m_settingsPath, m_settings, &error)) std::printf("Settings: saved %s\n", m_settingsPath.c_str());
    else std::printf("Settings: %s\n", error.c_str());
}

// Slow cinematic camera for the intro and the start menu: a slow arc behind the vehicle into the low sun (the vehicle
// and the sun right of the menu column). Read-only like the follow camera; a ray keeps it above the ground.
void Application::updateMenuCamera(float dt) {
    m_menuCamTime += dt;
    const double t = m_menuCamTime;
    const glm::vec3 car = toGlm(m_sim.currentSnapshot().body.position);
    const float yaw = -0.3f + 0.35f * float(std::sin(t * 6.2831853 / 70.0));
    const float radius = 8.0f + 1.2f * float(std::sin(t * 6.2831853 / 47.0 + 1.0));
    const float height = 1.25f + 0.35f * float(std::sin(t * 6.2831853 / 39.0));
    glm::vec3 pos = car + glm::vec3(std::sin(yaw) * radius, height, -std::cos(yaw) * radius);
    const JPH::Vec3 above(pos.x, pos.y + 20.0f, pos.z), below(pos.x, pos.y - 20.0f, pos.z);
    if (auto hit = m_sim.castRayIgnoringVehicle(above, below)) {
        const float ground = above.GetY() + (below.GetY() - above.GetY()) * *hit;
        pos.y = std::max(pos.y, ground + 0.9f);
    }
    const glm::vec3 fwd = glm::normalize(glm::vec3(car.x - pos.x, 0.0f, car.z - pos.z));
    const glm::vec3 right = glm::normalize(glm::cross(fwd, glm::vec3(0, 1, 0)));
    const glm::vec3 target = car + glm::vec3(0.0f, 0.75f, 0.0f) - right * 2.6f;
    m_camera.setFreePose(pos, target);
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
        std::shared_ptr<const Level> level = Level::load(path, &error);
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
        return true;
    }
    if (std::find(m_levelIds.begin(), m_levelIds.end(), id) == m_levelIds.end()) return false;
    std::shared_ptr<const Level> level = loadLevel(id);
    if (!level) return false;
    m_sim.requestLevel(std::move(level));
    return true;
}

void Application::buildWorldContent() {
    const Uint64 t0 = SDL_GetPerformanceCounter();
    SDL_WaitForGPUIdle(m_renderer->device());
    for (GpuMesh* m : {&m_trackMesh, &m_sceneryMesh, &m_waterMesh}) m->release();
    const Level* level = m_sim.level();
    m_contentLevel = level;
    m_contentBuilt = true;
    m_mapColors.clear();
    std::size_t trees = 0;
    if (level) {
        m_trackMesh = m_renderer->upload(buildLevelPropsMesh(*level));
        m_waterMesh = m_renderer->upload(buildLevelWaterMesh(*level));
        m_renderer->setGround(buildLevelGroundMaps(*level), 0.0f);
        TerrainSource src;
        src.samples = level->samples();
        src.origin = level->origin();
        src.cell = level->cell();
        src.heights = level->heights().data();
        src.splat = level->splatTexels().data();
        m_renderer->setTerrain(&src);
        const std::vector<TreeInstance> inst = buildLevelTreeInstances(*level);
        trees = inst.size();
        m_renderer->setTrees(buildPineMeshes(), inst);
        m_camera.farPlane = 4000.0f;
    } else {
        m_trackMesh = m_renderer->upload(buildTrackMesh(m_sim.track().geometry(), m_sim.surfaces()));
        m_sceneryMesh = m_renderer->upload(buildSceneryMesh());
        m_waterMesh = m_renderer->upload(buildWaterMesh());
        m_renderer->setTerrain(nullptr);
        m_renderer->setGround(buildGroundMaps(m_sim.track()), TestTrack::water().level);
        const std::vector<TreeInstance> inst = buildTreeInstances();
        trees = inst.size();
        m_renderer->setTrees(buildPineMeshes(), inst);
        m_camera.farPlane = Camera{}.farPlane;
    }
    std::printf("World: %s, render triangles: props/track %u, scenery %u, water %u, trees %zu, build %.0f ms\n",
                level ? level->name().c_str() : "Proving Ground", m_trackMesh.vertexCount() / 3,
                m_sceneryMesh.vertexCount() / 3, m_waterMesh.vertexCount() / 3, trees,
                1000.0 * double(SDL_GetPerformanceCounter() - t0) / double(SDL_GetPerformanceFrequency()));
}

// Level map (M): hill-shaded surfaces and water of the play area, roads, rivers, the vehicle and the points of
// interest; clicking a marker or a list entry teleports there (the same teleports as the F1 panel).
void Application::drawMap() {
    const Level& L = *m_sim.level();
    constexpr int kMapRes = 160;
    const float half = L.playHalfExtent();
    const float cellM = 2.0f * half / float(kMapRes);
    if (m_mapColors.size() != std::size_t(kMapRes * kMapRes)) {
        m_mapColors.assign(std::size_t(kMapRes * kMapRes), 0u);
        for (int j = 0; j < kMapRes; ++j)
            for (int i = 0; i < kMapRes; ++i) {
                const float x = -half + (float(i) + 0.5f) * cellM, z = -half + (float(j) + 0.5f) * cellM;
                glm::vec3 c;
                switch (L.waterKindAt(x, z)) {
                case WaterKind::Sea: c = {0.13f, 0.29f, 0.40f}; break;
                case WaterKind::Lake: c = {0.17f, 0.37f, 0.46f}; break;
                case WaterKind::River: c = {0.22f, 0.42f, 0.52f}; break;
                case WaterKind::None:
                    switch (L.surfaceAt(x, z)) {
                    case SurfaceId::Snow: c = {0.88f, 0.90f, 0.92f}; break;
                    case SurfaceId::Rock: c = {0.47f, 0.45f, 0.42f}; break;
                    case SurfaceId::Sand: c = {0.75f, 0.69f, 0.52f}; break;
                    case SurfaceId::PackedDirt: case SurfaceId::Mud: c = {0.52f, 0.42f, 0.30f}; break;
                    default: c = glm::mix(glm::vec3(0.27f, 0.41f, 0.21f), glm::vec3(0.40f, 0.44f, 0.30f),
                                          std::clamp(L.heightAt(x, z) / 220.0f, 0.0f, 1.0f));
                    }
                }
                // Hill shade: light from the north-west.
                const float d = 6.0f;
                const float gx = (L.heightAt(x + d, z) - L.heightAt(x - d, z)) / (2.0f * d);
                const float gz = (L.heightAt(x, z + d) - L.heightAt(x, z - d)) / (2.0f * d);
                const float shade = std::clamp(1.0f - 0.9f * (gx + gz), 0.55f, 1.25f);
                c = glm::clamp(c * shade, 0.0f, 1.0f);
                m_mapColors[std::size_t(j * kMapRes + i)] =
                    IM_COL32(int(c.r * 255.0f), int(c.g * 255.0f), int(c.b * 255.0f), 255);
            }
    }
    const ImGuiIO& io = ImGui::GetIO();
    const float side = std::floor(std::min(io.DisplaySize.x * 0.62f, io.DisplaySize.y * 0.80f));
    const float listW = 0.30f * side;
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(side + listW + 3.0f * ImGui::GetStyle().WindowPadding.x, side + ImGui::GetFrameHeightWithSpacing() * 2.0f));
    if (!ImGui::Begin("Map - Exploration (M closes)", &m_mapOpen,
                      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::End();
        return;
    }
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 o = ImGui::GetCursorScreenPos();
    const float px = side / float(kMapRes);
    auto toMap = [&](float x, float z) { return ImVec2(o.x + (x + half) / (2.0f * half) * side, o.y + (z + half) / (2.0f * half) * side); };
    for (int j = 0; j < kMapRes; ++j)
        for (int i = 0; i < kMapRes; ++i)
            dl->AddRectFilled(ImVec2(o.x + float(i) * px, o.y + float(j) * px),
                              ImVec2(o.x + float(i + 1) * px + 0.5f, o.y + float(j + 1) * px + 0.5f),
                              m_mapColors[std::size_t(j * kMapRes + i)]);
    std::vector<ImVec2> line;
    for (const LevelRiver& r : L.rivers()) {
        line.clear();
        for (std::size_t k = 0; k < r.samples.size(); k += 3) line.push_back(toMap(r.samples[k].x, r.samples[k].z));
        dl->AddPolyline(line.data(), int(line.size()), IM_COL32(70, 130, 170, 255), 0, 2.0f);
    }
    for (const LevelRoad& r : L.roads()) {
        line.clear();
        for (std::size_t k = 0; k < r.samples.size(); k += 3) line.push_back(toMap(r.samples[k].x, r.samples[k].z));
        if (!r.samples.empty()) line.push_back(toMap(r.samples.back().x, r.samples.back().z));
        dl->AddPolyline(line.data(), int(line.size()), r.trail ? IM_COL32(150, 112, 70, 255) : IM_COL32(232, 205, 150, 255),
                        0, r.trail ? 1.5f : 2.5f);
    }
    // Points of interest: features 1.. are the POIs in map order (feature 0 is the spawn).
    const auto& pois = L.pois();
    int clicked = -1;
    const ImVec2 mouse = io.MousePos;
    for (std::size_t k = 0; k < pois.size(); ++k) {
        const ImVec2 c = toMap(pois[k].x, pois[k].z);
        const bool hover = std::hypot(mouse.x - c.x, mouse.y - c.y) < 10.0f && ImGui::IsWindowHovered();
        dl->AddCircleFilled(c, hover ? 9.0f : 7.0f, hover ? IM_COL32(255, 160, 40, 255) : IM_COL32(240, 110, 20, 255));
        dl->AddCircle(c, hover ? 9.0f : 7.0f, IM_COL32(20, 20, 20, 255), 0, 1.5f);
        const std::string id = std::to_string(pois[k].id);
        const ImVec2 ts = ImGui::CalcTextSize(id.c_str());
        dl->AddText(ImVec2(c.x - 0.5f * ts.x, c.y - 0.5f * ts.y), IM_COL32(255, 255, 255, 255), id.c_str());
        dl->AddText(ImVec2(c.x + 11.0f, c.y - 0.5f * ts.y), IM_COL32(15, 15, 15, 255), pois[k].name.c_str());
        if (hover && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) clicked = int(k) + 1;
    }
    {
        const JPH::RMat44 xf = m_sim.vehicle().bodyTransform();
        const JPH::Vec3 p(xf.GetTranslation()), f = xf.GetAxisZ();
        glm::vec2 fw(f.GetX(), f.GetZ());
        fw = glm::length(fw) > 1e-3f ? glm::normalize(fw) : glm::vec2(0, 1);
        const ImVec2 c = toMap(p.GetX(), p.GetZ());
        const glm::vec2 s(-fw.y, fw.x);
        auto pt = [&](float a, float b) { return ImVec2(c.x + fw.x * a + s.x * b, c.y + fw.y * a + s.y * b); };
        dl->AddTriangleFilled(pt(11, 0), pt(-7, 6), pt(-7, -6), IM_COL32(255, 255, 255, 255));
        dl->AddTriangle(pt(11, 0), pt(-7, 6), pt(-7, -6), IM_COL32(0, 0, 0, 255), 1.5f);
    }
    ImGui::Dummy(ImVec2(side, side));
    ImGui::SameLine();
    ImGui::BeginChild("teleports", ImVec2(listW, side));
    ImGui::TextUnformatted("Teleport");
    ImGui::Separator();
    const std::vector<TrackFeature>& features = m_sim.track().features();
    for (std::size_t k = 0; k < features.size(); ++k)
        if (ImGui::Selectable(features[k].name.c_str())) clicked = int(k);
    ImGui::EndChild();
    ImGui::End();
    if (clicked >= 0) {
        m_sim.requestTeleport(clicked);
        m_mapOpen = false;
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
    }
    const bool vehiclePage = m_flow.screen() == Screen::MainMenu && m_flow.page() == MenuPage::Vehicle;
    if (vehiclePage && !m_vehiclePageCamera) m_garageCamTime = 0.0;
    m_vehiclePageCamera = vehiclePage;
    if ((m_flow.screen() == Screen::Intro || m_flow.screen() == Screen::MainMenu) && !vehiclePage) updateMenuCamera(frameTime);
    if (m_flow.screen() == Screen::Garage || vehiclePage) updateGarageCamera(frameTime); // the preview: the live vehicle
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
    m_ui.menuOpen = m_flow.menuActive();
    stats.followCamera = m_camera.mode() == Camera::Mode::Follow;
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
    if (!m_flow.menuActive()) m_ui.drawAudio(m_audioPanel);
    if (m_mapOpen && !m_flow.menuActive() && m_sim.level()) drawMap();
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
    frame.nearPlane = m_camera.nearPlane;
    frame.time = float(m_time);
    frame.draws = m_draws;
    frame.water = &m_waterMesh;
    if (m_sim.level()) {
        // Open world: thinner fog over kilometers, longer shadow cascades, no planar puddle reflection.
        frame.fogDensity = 0.00045f;
        frame.fogFalloff = 0.004f;
        frame.shadowScale = 2.0f;
        frame.waterReflection = false;
    }
    frame.vehicleModel = bodyM;
    frame.vehicleMud = m_gfx.mud;
    frame.bodyMud = &m_bodyMud;
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
    for (int a = 0; a < 2; ++a) {
        const MeshData wheel = buildAxleWheelMesh(m_vehicleVisual, vc.axles[std::size_t(a)]);
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
    if (c.op == "menu") {
        // menu intro|main|start|pause|resume|settings|back|select ITEM: drive the game flow directly (key taps and
        // `click menu:ITEM` go through the real input path instead).
        const Screen before = m_flow.screen();
        if (arg == "intro") {
            std::vector<std::string> ids = m_flow.vehicles();
            m_flow = GameFlow(Screen::Intro);
            m_flow.setVehicles(std::move(ids));
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
        m_sim.requestTeleport(std::atoi(arg.c_str()));
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
        m_mapOpen = (arg == "on" || arg == "1") && m_sim.level();
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
        else if (k == "flowers") s.flowerDensity = v;
        else if (k == "grass_detail") s.grassDetail = v != 0.0f;
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
            else if (k == "body_mud") m_bodyMud.setAll(v);
            else if (k == "body_mud_clear") m_bodyMud.clear();
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
        if (arg == "follow") {
            if (m_camera.mode() == Camera::Mode::Free) m_camera.toggleMode();
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
    if (n == "paused") return simulationRuns() ? 0.0 : 1.0;
    if (n == "frozen") return m_paused ? 1.0 : 0.0;
    if (n == "menu_screen") return double(int(m_flow.screen())); // 0 intro, 1 main menu, 2 playing, 3 paused
    if (n == "menu_page") return double(int(m_flow.page()));     // 0 main, 1 pause, 2 settings, 3 controls, 4 vehicle
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
    if (n == "menu_font") return m_menu.fontsLoaded() ? 1.0 : 0.0;
    if (n == "menu_logo") return m_menu.logoLoaded() ? 1.0 : 0.0;
    if (n.rfind("setting_", 0) == 0) {
        const GameSettings& g = m_settings;
        const std::string k = n.substr(8);
        if (k == "quality") return g.quality;
        if (k == "render_scale") return g.renderScale;
        if (k == "vsync") return g.vsync ? 1.0 : 0.0;
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
        if (k == "vehicle") return m_flow.vehicleIndex(g);
        return std::nullopt;
    }
    // The settings file format keeps every value of the current settings (parse(serialize(s)) == s).
    if (n == "settings_roundtrip") return parseSettings(serializeSettings(m_settings)) == m_settings ? 1.0 : 0.0;
    if (n == "drive_keys") return double(int(m_input.driveKeys()));
    if (n == "kb_style") return double(int(keyboardStyle())); // 0 Mac, 1 PC
    if (n == "pad_glyphs") return double(int(resolvePadGlyphs(PadGlyphSetting(m_settings.padGlyphs), padType())));
    if (n == "controls_callouts") return m_menu.controlsCallouts();
    if (n == "controls_overlaps") return m_menu.controlsOverlaps();
    if (n == "cam_free") return m_camera.mode() == Camera::Mode::Free ? 1.0 : 0.0;
    if (n == "ui_visible") return m_ui.visible ? 1.0 : 0.0;
    if (n == "ui_kb") return ImGui::GetIO().WantCaptureKeyboard ? 1.0 : 0.0;
    if (n == "ui_text") return ImGui::GetIO().WantTextInput ? 1.0 : 0.0;
    if (n == "cam_clearance") return m_camClearance;
    if (n == "min_cam_clearance") return m_minCamClearance;
    if (n == "max_air") return m_maxAirWheels;
    if (n == "garage_open") return m_flow.screen() == Screen::Garage ? 1.0 : 0.0;
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

        m_flow.update(std::min(frameTime, 0.1));
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
