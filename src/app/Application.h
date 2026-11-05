#pragma once

#include "app/BestLaps.h"
#include "app/GameFlow.h"
#include "app/Garage.h"
#include "app/GameSettings.h"
#include "audio/AudioSystem.h"
#include "audio/VehicleAudioFeed.h"
#include "input/Input.h"
#include "render/Camera.h"
#include "render/DebugDraw.h"
#include "render/Renderer.h"
#include "render/VehicleModel.h"
#include "sim/Simulation.h"
#include "sim/BodyMud.h"
#include "sim/WheelSpray.h"
#include "ui/DebugUI.h"
#include "ui/MapUI.h"
#include "ui/MenuUI.h"
#include "vehicle/VehicleDefinition.h"

#include <SDL3/SDL_events.h>
#include <quench/app/App.h>
#include <quench/app/Scenario.h>
#include <quench/core/FixedStepClock.h>

#include <array>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace tf {

// TrailForge's own command line flags (the engine's, such as --script and --capture, are in quench::app).
struct AppOptions {
    bool demoDrive = false;        // scripted throttle/steering for a demo without input
    bool startWithUi = true;
    bool audio = true;         // false: --no-audio (no device, no sound assets loaded)
    std::string audioCapturePath; // non-empty: offline audio render to this WAV instead of the device
    std::string vehicleId;     // --vehicle ID: start with this catalog vehicle (remembered like a menu choice)
    std::string levelId;       // --level ID: start on this level (proving_ground or an assets/levels id; remembered)
    bool intro = true;         // false: --no-intro (start at the main menu); scenarios and --demo start driving
};

// The game on the engine's app shell (quench::app, see TrailForgeGame.cpp): the engine owns the window, the GPU
// device, the frame loop, ImGui, the scenario runner, captures and perf regions. Per frame: event() for every SDL
// event -> scenario commands -> update() (input, game flow, physics ticks, camera) -> ui() (ImGui windows, menus and
// what they change) -> render() (the 3D frame; the engine draws ImGui on top). Valid between Game::init and
// Game::shutdown.
class Application {
public:
    Application(quench::app::App& app, const AppOptions& options);
    ~Application();
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    void event(const SDL_Event& e);
    void update(double frameDt);
    void ui();
    void render(quench::app::Frame& frame);
    // Scenario support (--script): game values and the commands the engine does not know.
    std::optional<double> scenarioValue(const std::string& name) const;
    quench::app::ScenarioExec scenarioCommand(const quench::app::ScenarioCommand& c);
    // Appended to the engine's perf line: render and output size, VSync, draw calls, grass blades.
    std::string perfDetails() const;
    // One line at exit: run time, physics ticks, body position, FPS, dropped time.
    void printExitSummary() const;

private:
    // Start of a frame's input, before its events: per-frame input state, the TIME row from the live clock, touch
    // controls for the window.
    void beginInputFrame();
    // The frame's input: debug toggles, menus, garage and map, gameplay actions, the driver intent.
    void handleInput();
    void handleMenu();
    void onMenuCommand(MenuCommand c);
    void startGameplay();
    void enterMenuWorld(bool restartMusic);
    void applySettings(bool force);
    // The sky shows the gameplay clock (GameSettings::timeOfDay), not the sunset of the start menu world.
    bool showsGameplayTime() const;
    void saveSettings();
    void updateMenuCamera(float dt, const glm::mat4& body);
    void updateAttractDrive();
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
    // Map: built on a worker thread per world (buildWorldContent starts it, the UI uploads the result).
    void startMapBuild();
    void pollMapBuild();
    void openMap();
    void closeMap();
    void handleMap();
    MapPlayer mapPlayer() const;
    MapHints mapHints() const;
    void drawMapHud(); // minimap and compass bar in gameplay
    void updateTouchControls(); // on-screen driving controls on or off for this frame, layout from the window
    void drawTouchControls();
    bool simulationRuns() const { return m_flow.simulationRuns() && !m_paused; }
    void setPaused(bool paused);
    void setVSync(bool on);
    void publishAudio();
    void stepSimulation(double frameTime);
    // Pose interpolation between the last two physics ticks, camera, scenario statistics.
    void updateView(float frameTime);

    // ---- Verification scenario (--script)
    quench::app::ScenarioExec execScenario(const quench::app::ScenarioCommand& c);
    // Multi-frame input of the game's commands (clicks on named UI items, touches, drags): SDL events pushed
    // framesFromNow frames later, from scenarioCommand, so each lands in the frame it would have in a live run.
    void queueEvent(const SDL_Event& e, int framesFromNow);
    void pushQueuedEvents();
    // Events still queued, or the last one not yet handled by a whole frame (update and ui).
    bool inputInFlight() const;
    void queueClick(float x, float y, bool textInput);
    // A finger as SDL reports it (mouse events from SDL_TOUCH_MOUSEID): down at (x, y), `steps` moves over one frame
    // each to (x + dx, y + dy), up. steps 0: a tap.
    void queueTouch(float x, float y, float dx, float dy, int steps);

    quench::app::App& m_app;
    AppOptions m_options;
    // Member order = construction order; destruction is the reverse.
    std::unique_ptr<Renderer> m_renderer;
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
    // Best drift score per level and vehicle (best_drift.cfg, same rules: player runs or scenario runs).
    BestLaps m_bestDrift{BestLaps::Order::Higher};
    std::string m_bestDriftPath;
    int m_driftRunsSeen = 0;
    void updateBestDrift();
    std::string currentLevelId() const;
    std::string currentVehicleId() const;
    // Scenario-only overrides for the Controls page captures: key cap wording and the connected pad type.
    std::optional<KeyboardStyle> m_keyboardOverride;
    std::optional<PadType> m_padTypeOverride;
    // On-screen touch driving: auto (iOS without a gamepad), or forced on / off by the scenario `touch_controls`.
    enum class TouchMode { Auto, On, Off } m_touchMode = TouchMode::Auto;
    KeyboardStyle keyboardStyle() const { return m_keyboardOverride.value_or(nativeKeyboardStyle()); }
    PadType padType() const { return m_padTypeOverride.value_or(m_input.gamepadType()); }
    float m_musicTarget = 0.0f;
    float m_musicFade = 0.3f;
    int m_musicRestart = 0;
    double m_menuCamTime = 0.0;
    float m_menuCamHeading = 0.0f;   // smoothed vehicle heading the menu camera follows
    glm::vec3 m_menuCamOffset{0.0f}; // smoothed camera offset from the vehicle
    // The attract drive puts the vehicle on its route start once this world is the simulation's (a level switch or
    // the spawn teleport is processed on the next tick).
    bool m_attractStartPending = false;
    const Level* m_attractStartLevel = nullptr;
    float m_menuBlur = 0.0f;          // eased blur of the 3D frame under the pause / settings menus
    float m_dofAmount = 0.0f;         // eased depth of field amount (menus, garage, optional while driving)
    quench::FixedStepClock m_clock;
    // Audio: the feed turns every physics tick into sound parameters and events; the system owns the device
    // (or the offline capture) and the mixer that runs on the audio thread.
    VehicleAudioFeed m_audioFeed;
    AudioPanelState m_audioPanel;
    std::unique_ptr<AudioSystem> m_audio;

    void rebuildVehicleVisual(); // body (fitted to the wheels) and wheel meshes from the current config
    void ensureVehicleVisual();  // rebuilds them when the wheel fit of the running vehicle changed
    void buildDraws(const glm::mat4& bodyM, const std::array<glm::mat4, kWheelCount>& wheelM);
    void updateCosmetics(float dt);
    void fillTireMarks();

    // m_sceneryMesh: the proving ground backdrop, or the render-only horizon ring of a level.
    GpuMesh m_trackMesh, m_sceneryMesh, m_waterMesh, m_waterfallMesh, m_bodyMesh, m_wheelMesh, m_rearWheelMesh; // wheels: front, rear axle
    GpuMesh m_springMesh, m_armMesh, m_halfShaftMesh, m_shockBodyMesh, m_shockShaftMesh;
    std::optional<VehicleConfig> m_visualFit;  // config the body and wheel meshes were built for
    std::array<glm::vec2, 2> m_wheelVisualSize{}; // measured wheel mesh (outer radius, width) per axle
    VehicleVisual m_vehicleVisual; // parts, spare mounts and wheel style of the body model
    std::vector<DrawItem> m_draws;
    GraphicsState m_gfx;
    bool m_vsync = true;     // the present mode TrailForge asked the engine for (settings, F2, scenario `set vsync`)

    bool m_paused = false;   // debug freeze (P / Pause button in F1), independent of the pause menu
    bool m_inBackground = false; // iOS: between DID_ENTER_BACKGROUND and WILL_ENTER_FOREGROUND (no GPU, no audio)
    bool m_stepOnce = false;
    int m_stepsThisFrame = 0;
    // The engine's frame step: real time, or one fixed step per frame in scenario and capture runs (paced to at most
    // real time), so input, waits and conditions land on the same physics ticks at any display refresh rate.
    double m_frameDt = 0.0;
    float m_frameTime = 0.0f; // m_frameDt clamped to 0.1 s (camera, cosmetics, easing)
    // Interpolated poses of this frame (update) for the UI and the scene (render).
    glm::mat4 m_bodyM{1.0f};
    std::array<glm::mat4, kWheelCount> m_wheelM{};

    struct QueuedEvent { std::uint64_t frame; SDL_Event event; }; // pushed in this engine frame
    std::vector<QueuedEvent> m_queuedEvents;
    std::optional<std::uint64_t> m_lastPushFrame; // engine frame of the last pushed queued event
    int m_inputCommandLine = -1; // the command whose queued input is in flight (Busy until it is handled)
    int m_scrollFrames = 0;
    int m_scrollStableFrames = 0;
    float m_scrollLastY = 0.0f;
    float m_camClearance = 0.0f;     // m: camera height above the surface below it
    float m_minCamClearance = 1e9f;  // minimum since the last reset_stats (follow camera only)
    int m_maxAirWheels = 0;          // maximum number of wheels without contact since the last reset_stats
    float m_maxTilt = 0.0f;          // deg: maximum body tilt from upright since the last reset_stats
    int m_maxLockedWheels = 0;       // maximum number of locked wheels (in contact, > 2 m/s) since the last reset_stats
    double m_lockedWheelTime = 0.0; // wheel-seconds of locked wheels since the last reset_stats (sampled per frame)
    bool m_absSeen = false, m_tcSeen = false; // ABS / TC acted in a sampled frame since the last reset_stats
};

} // namespace tf
