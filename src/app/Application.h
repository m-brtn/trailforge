#pragma once

#include "app/BestLaps.h"
#include "app/GameFlow.h"
#include "app/Garage.h"
#include "app/GameSettings.h"
#include "app/Scenario.h"
#include "audio/AudioSystem.h"
#include "audio/VehicleAudioFeed.h"
#include "input/Input.h"
#include "render/Camera.h"
#include "render/DebugDraw.h"
#include "render/Renderer.h"
#include "render/VehicleModel.h"
#include "sim/FixedStepClock.h"
#include "sim/Simulation.h"
#include "sim/BodyMud.h"
#include "sim/WheelSpray.h"
#include "ui/DebugUI.h"
#include "ui/MapUI.h"
#include "ui/MenuUI.h"
#include "vehicle/VehicleDefinition.h"

#include <SDL3/SDL_video.h>

#include <array>
#include <deque>
#include <future>
#include <map>
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
    std::string vehicleId;     // --vehicle ID: start with this catalog vehicle (remembered like a menu choice)
    std::string levelId;       // --level ID: start on this level (proving_ground or an assets/levels id; remembered)
    bool intro = true;         // false: --no-intro (start at the main menu); scenarios and --demo start driving
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
    void handleMenu();
    void onMenuCommand(MenuCommand c);
    void startGameplay();
    void enterMenuWorld(bool restartMusic);
    void applySettings(bool force);
    void saveSettings();
    void updateMenuCamera(float dt, const glm::mat4& body);
    void updateGarageCamera(float dt);
    void handleGarage();
    void openGarage();
    void closeGarage();
    void applyGarageConfig();
    // Garage, simulation and the remembered choice to a catalog vehicle; false if the id is unknown.
    bool selectVehicle(const std::string& id);
    // Levels: "proving_ground" or an assets/levels/<id>/level.json (baked once, then cached). Empty = the first
    // (default) level. Requests the switch from the simulation; false if the id is unknown or fails to load.
    bool selectLevel(const std::string& id);
    std::shared_ptr<const Level> loadLevel(const std::string& id);
    // Render content of the simulation's current world (track or level props, water, ground maps, terrain, trees).
    void buildWorldContent();
    // Map: built on a worker thread per world (buildWorldContent starts it, renderFrame uploads the result).
    void startMapBuild();
    void pollMapBuild();
    void openMap();
    void closeMap();
    void handleMap();
    MapPlayer mapPlayer() const;
    MapHints mapHints() const;
    void drawMapHud(); // minimap and compass bar in gameplay
    bool simulationRuns() const { return m_flow.simulationRuns() && !m_paused; }
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
    std::vector<VehicleDefinition> m_vehicles; // assets/vehicles/*/vehicle.json (before m_sim: it spawns the first)
    Simulation m_sim;
    WheelSpray m_spray; // cosmetic particles thrown by the wheels (render-only)
    BodyMud m_bodyMud;  // cosmetic spray mud on the body per region (render-only)
    Input m_input;
    Camera m_camera;
    DebugDraw m_debugDraw;
    DebugUI m_ui;
    MenuUI m_menu;        // releases its logo texture explicitly before the renderer (see ~Application)
    GameFlow m_flow;
    std::unique_ptr<Garage> m_garage; // vehicle setups; null when no vehicle definition loaded
    Camera::Mode m_garageCameraMode = Camera::Mode::Follow; // camera mode to restore when the garage closes
    double m_garageCamTime = 0.0;
    std::vector<VehicleCard> m_vehicleCards; // vehicle page (catalog order)
    float m_vehicleWheel = 0.0f;             // vehicle page: wheel steps not yet turned into a vehicle change
    bool m_vehiclePageCamera = false;        // last frame showed the vehicle page (its camera restarts on entry)
    std::vector<std::string> m_levelIds;     // start menu order: assets/levels ids, then proving_ground
    std::map<std::string, std::shared_ptr<const Level>> m_levelCache;
    const Level* m_contentLevel = nullptr;   // world the render content was built for
    std::string m_buildingTextureDir;        // assets/textures/buildings (building kit materials)
    bool m_contentBuilt = false;
    MapUI m_mapUI;                           // releases its texture explicitly before the renderer
    std::future<MapData> m_mapJob;
    MapView m_mapView;                       // full-screen map view (center, zoom); reset per world
    bool m_mapViewSet = false;
    MapMarker m_marker;                      // waypoint, cleared when the world changes
    int m_mapHoverPoi = -1;
    bool m_mapDragging = false;              // left button went down on the map panel
    float m_mapDragMoved = 0.0f;             // points moved since the press (a click below 5)
    float m_mapLastX = 0.0f, m_mapLastY = 0.0f;
    bool m_mapCrosshair = false;             // keyboard / gamepad aiming: marker at the panel middle
    double m_mapBuildMs = 0.0;               // worker time of the current world's map
    GameSettings m_settings;
    GameSettings m_applied;      // what applySettings last pushed to the renderer / simulation / mixer
    std::string m_settingsPath;  // empty: settings are not persisted (scenario mode)
    // Best lap per level and vehicle (best_laps.cfg in the user data directory; scenario runs: a scratch file that
    // starts empty). Laps count when the player drove them (or the scenario `autopilot drive`, not the menu backdrop).
    BestLaps m_bestLaps;
    std::string m_bestLapsPath;
    int m_lapsSeen = 0;
    bool m_scenarioAutopilot = false;
    void updateBestLaps();
    std::string currentLevelId() const;
    std::string currentVehicleId() const;
    // Scenario-only overrides for the Controls page captures: key cap wording and the connected pad type.
    std::optional<KeyboardStyle> m_keyboardOverride;
    std::optional<PadType> m_padTypeOverride;
    // Scenario `gamepad attach`: an SDL virtual gamepad that the input opens like a real one (axes set by the script).
    SDL_JoystickID m_virtualPadId = 0;
    SDL_Joystick* m_virtualPad = nullptr;
    KeyboardStyle keyboardStyle() const { return m_keyboardOverride.value_or(nativeKeyboardStyle()); }
    PadType padType() const { return m_padTypeOverride.value_or(m_input.gamepadType()); }
    float m_musicTarget = 0.0f;
    float m_musicFade = 0.3f;
    int m_musicRestart = 0;
    double m_menuCamTime = 0.0;
    float m_menuCamHeading = 0.0f;   // smoothed vehicle heading the menu camera follows
    glm::vec3 m_menuCamOffset{0.0f}; // smoothed camera offset from the vehicle
    float m_gameplayTimeOfDay = 9.0f; // restored when gameplay starts (the menus use a sunset)
    float m_menuBlur = 0.0f;          // eased blur of the 3D frame under the pause / settings menus
    float m_dofAmount = 0.0f;         // eased depth of field amount (menus, garage, optional while driving)
    FixedStepClock m_clock;
    // Audio: the feed turns every physics tick into sound parameters and events; the system owns the device
    // (or the offline capture) and the mixer that runs on the audio thread.
    VehicleAudioFeed m_audioFeed;
    AudioPanelState m_audioPanel;
    std::unique_ptr<AudioSystem> m_audio;

    void rebuildVehicleVisual(); // body (fitted to the wheels) and wheel meshes from the current config
    void buildDraws(const glm::mat4& bodyM, const std::array<glm::mat4, kWheelCount>& wheelM);
    void updateCosmetics(float dt);
    void endPerf();
    void fillTireMarks();

    // m_sceneryMesh: the proving ground backdrop, or the render-only horizon ring of a level.
    GpuMesh m_trackMesh, m_sceneryMesh, m_waterMesh, m_waterfallMesh, m_bodyMesh, m_wheelMesh, m_rearWheelMesh; // wheels: front, rear axle
    GpuMesh m_springMesh, m_armMesh, m_halfShaftMesh, m_shockBodyMesh, m_shockShaftMesh;
    std::optional<VehicleConfig> m_visualFit;  // config the body and wheel meshes were built for
    std::array<glm::vec2, 2> m_wheelVisualSize{}; // measured wheel mesh (outer radius, width) per axle
    VehicleVisual m_vehicleVisual; // parts, spare mounts and wheel style of the body model
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
    bool m_paused = false;   // debug freeze (P / Pause button in F1), independent of the pause menu
    bool m_stepOnce = false;
    bool m_captured = false;
    int m_stepsThisFrame = 0;
    double m_time = 0.0;     // application time: real, or the fixed scenario frame steps
    // Scenario runs advance by a fixed step per frame (paced to at most real time), so input, waits and conditions
    // land on the same physics ticks at any display refresh rate. perf_begin..perf_end measures in real time.
    static constexpr double kScenarioFrameStep = 1.0 / 120.0;
    double m_frameDt = 0.0;  // the step of the current frame
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
