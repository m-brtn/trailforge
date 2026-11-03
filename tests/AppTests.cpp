// Headless checks of the game flow: settings file, menu navigation, the pause freezing the simulation, and the
// garage model (setups, ranges, persistence, applying to a running simulation).
// Usage: trailforge_app_tests <test_name>

#include "app/BestLaps.h"
#include "app/GameFlow.h"
#include "app/Garage.h"
#include "app/GameSettings.h"
#include "app/MapData.h"
#include "app/MapView.h"
#include "input/ControlLayout.h"
#include "input/TouchControls.h"
#include "track/Level.h"
#include "sim/FixedStepClock.h"
#include "sim/Simulation.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace tf;

namespace {

int g_failures = 0;

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("  FAIL %s:%d: %s — ", __FILE__, __LINE__, #cond);      \
            std::printf(__VA_ARGS__);                                           \
            std::printf("\n");                                                  \
            ++g_failures;                                                       \
        }                                                                       \
    } while (0)

// Best laps per level and vehicle: only faster times replace a stored one, the file survives a round trip,
// malformed lines are skipped.
void bestLapsFile() {
    BestLaps b;
    CHECK(b.best("suncoast_raceway", "trailforge_suv") == 0.0, "empty table");
    CHECK(b.submit("suncoast_raceway", "trailforge_suv", 171.25), "first lap stored");
    CHECK(!b.submit("suncoast_raceway", "trailforge_suv", 180.0), "a slower lap is not stored");
    CHECK(b.submit("suncoast_raceway", "trailforge_suv", 165.5) && b.best("suncoast_raceway", "trailforge_suv") == 165.5, "faster lap");
    CHECK(b.submit("suncoast_raceway", "trailforge_ev", 150.125), "per vehicle");
    CHECK(!b.submit("Suncoast Raceway", "trailforge_ev", 10.0) && !b.submit("suncoast_raceway", "x", -1.0), "invalid ids / times");
    BestLaps c;
    c.parse(b.serialize());
    CHECK(c.size() == 2 && c.best("suncoast_raceway", "trailforge_ev") == 150.125 && c.best("suncoast_raceway", "trailforge_suv") == 165.5,
          "round trip: %zu entries", c.size());
    int skipped = 0;
    c.parse("# comment\nsuncoast_raceway trailforge_suv 99.5\nbroken line\nsuncoast_raceway a b c\nlevel veh -3\n\n", &skipped);
    CHECK(c.size() == 1 && skipped == 3, "%zu entries, %d skipped", c.size(), skipped);
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "trailforge_app_tests";
    std::filesystem::create_directories(dir);
    const std::string path = (dir / "best_laps.cfg").string();
    std::string error;
    CHECK(b.save(path, &error), "save failed: %s", error.c_str());
    BestLaps d;
    CHECK(d.load(path) && d.size() == 2 && d.best("suncoast_raceway", "trailforge_suv") == 165.5, "load: %zu entries", d.size());
    std::filesystem::remove(path);
    CHECK(!d.load(path) && d.size() == 0, "a missing file clears the table");

    // Drift scores (best_drift.cfg): the higher value is kept.
    BestLaps s(BestLaps::Order::Higher);
    CHECK(s.submit("proving_ground", "rwd_test", 812.4) && !s.submit("proving_ground", "rwd_test", 500.0), "lower score kept out");
    CHECK(s.submit("proving_ground", "rwd_test", 1200.0) && s.best("proving_ground", "rwd_test") == 1200.0, "higher score");
    BestLaps t(BestLaps::Order::Higher);
    t.parse(s.serialize() + "proving_ground rwd_test 900\n");
    CHECK(t.size() == 1 && t.best("proving_ground", "rwd_test") == 1200.0 && s.serialize().find("drift") != std::string::npos,
          "drift round trip");
}

void settingsRoundtrip() {
    const GameSettings defaults;
    CHECK(parseSettings(serializeSettings(defaults)) == defaults, "defaults do not survive a roundtrip");

    GameSettings s;
    s.quality = 0;
    s.renderScale = 0.75f;
    s.grassDistance = 135.0f;
    s.treeDistance = 1100.0f;
    s.vsync = false;
    s.masterVolume = 0.35f;
    s.musicVolume = 0.0f;
    s.effectsVolume = 0.6f;
    s.uiVolume = 0.25f;
    s.abs = true;
    s.traction = 2;
    s.diffLocks = 3;
    s.autoReverse = false;
    s.driveKeys = 1;
    s.padGlyphs = 2;
    s.minimap = false;
    s.minimapSize = 2;
    s.minimapNorthUp = true;
    s.compass = false;
    s.timeOfDay = 17.123456f; // a clock that ran: the file keeps every bit
    s.timeFlow = 2;
    s.debugWindows = true;
    s.stylizedTrees = false;
    s.vehicle = "trailforge_ev";
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "trailforge_app_tests";
    std::filesystem::create_directories(dir);
    const std::string path = (dir / "settings.cfg").string();
    std::string error;
    CHECK(saveSettingsFile(path, s, &error), "save failed: %s", error.c_str());
    CHECK(!std::filesystem::exists(path + ".tmp"), "temporary file left behind");
    GameSettings loaded;
    int skipped = -1;
    CHECK(loadSettingsFile(path, loaded, &skipped), "load failed");
    CHECK(loaded == s, "file roundtrip changed the settings");
    CHECK(skipped == 0, "skipped lines in our own file: %d", skipped);
    // Overwriting an existing file works (rename over an existing file).
    s.quality = 3;
    CHECK(saveSettingsFile(path, s, &error) && loadSettingsFile(path, loaded) && loaded.quality == 3,
          "overwrite failed: %s", error.c_str());

    // Hand-edited / damaged file: unknown keys and garbage are skipped, values are clamped, the rest kept.
    const GameSettings odd = parseSettings("quality = 9\n"
                                           "volume_music = 1.7  # too loud\n"
                                           "volume_ui = -2\n"
                                           "render_scale = 0.2\n"
                                           "traction = 5\n"
                                           "diff_locks = 2\n"
                                           "drive_keys = 7\n"
                                           "pad_glyphs = -3\n"
                                           "vsync = banana\n"
                                           "no equals sign\n"
                                           "future_option = 1\n"
                                           "vehicle = ../../etc\n",
                                           &skipped);
    std::printf("    damaged file: quality %d music %.2f ui %.2f scale %.2f tc %d diff %d vsync %d, %d lines skipped\n",
                odd.quality, double(odd.musicVolume), double(odd.uiVolume), double(odd.renderScale), odd.traction,
                odd.diffLocks, odd.vsync, skipped);
    CHECK(odd.quality == 3 && odd.musicVolume == 1.0f && odd.uiVolume == 0.0f && odd.renderScale == 0.0f &&
              odd.traction == 2 && odd.diffLocks == 2 && odd.vsync == defaults.vsync && odd.driveKeys == 1 &&
              odd.padGlyphs == 0 && odd.vehicle.empty(),
          "damaged values not clamped as expected");
    CHECK(skipped == 4, "skipped lines %d (expected 4)", skipped);
    CHECK(!defaults.debugWindows && defaults.timeFlow == 0 && defaults.timeOfDay == 9.0f,
          "defaults: debug windows off, time 09:00, clock stopped");
    const GameSettings clock = parseSettings("time_of_day = -1\ntime_flow = 7\n");
    CHECK(clock.timeOfDay == 23.0f && clock.timeFlow == 2, "time wrapped %.2f, flow clamped %d", double(clock.timeOfDay),
          clock.timeFlow);
    CHECK(parseSettings("time_of_day = 24\n").timeOfDay == 0.0f, "24:00 is midnight");
    CHECK(defaults.stylizedTrees && serializeSettings(defaults).find("tree_style = stylized") != std::string::npos,
          "trees default to the stylized firs");
    int badTrees = 0;
    CHECK(!parseSettings("tree_style = models\n").stylizedTrees &&
              parseSettings("tree_style = fancy\n", &badTrees).stylizedTrees && badTrees == 1,
          "tree_style models / unknown value skipped");
    const GameSettings far = parseSettings("grass_distance = 900\ntree_distance = 5\n");
    CHECK(far.grassDistance == 250.0f && far.treeDistance == 0.0f, "distances clamped: grass %.0f tree %.0f",
          double(far.grassDistance), double(far.treeDistance));
    CHECK(defaults.minimap && defaults.minimapSize == 1 && !defaults.minimapNorthUp && defaults.compass,
          "map defaults: minimap on, medium, turning, compass on");
    CHECK(parseSettings("minimap_size = 9\n").minimapSize == 2 && parseSettings("minimap_size = -1\n").minimapSize == 0,
          "minimap size clamped");
    CHECK(defaults.traction == -1, "traction control defaults to the vehicle's own mode");
    CHECK(parseSettings("traction = 0\n").traction == -1 && parseSettings("traction = 1\n").traction == 1 &&
              parseSettings("traction_mode = 0\n").traction == 0 && parseSettings("traction_mode = -4\n").traction == -1,
          "legacy traction key / traction_mode mapping");
    CHECK(serializeSettings(defaults).find("vehicle") == std::string::npos, "no vehicle line without a choice");
    CHECK(serializeSettings(defaults).find("level") == std::string::npos, "no level line without a choice");
    GameSettings withLevel = defaults;
    withLevel.level = "proving_ground";
    CHECK(parseSettings(serializeSettings(withLevel)).level == "proving_ground", "level choice roundtrip");
    int badLevel = 0;
    CHECK(parseSettings("level = ../x\n", &badLevel).level.empty() && badLevel == 1, "invalid level id rejected");
    GameSettings untouched = s;
    CHECK(!loadSettingsFile((dir / "missing.ini").string(), untouched) && untouched == s,
          "a missing file must leave the settings alone");
    std::filesystem::remove_all(dir);
}

MenuInput key(bool MenuInput::*field) {
    MenuInput in;
    in.*field = true;
    in.any = true;
    return in;
}

void menuNavigation() {
    GameSettings settings;
    GameFlow flow;
    CHECK(flow.screen() == Screen::Intro && flow.menuActive() && flow.simulationRuns(), "intro state");
    for (int i = 0; i < 100; ++i) flow.update(0.05); // 5 s, longer than the intro
    CHECK(flow.screen() == Screen::MainMenu, "the intro did not end in the main menu");
    CHECK(flow.menuFade() > 0.0f && flow.menuFade() < 1.0f, "the start menu should be fading in: %.2f", flow.menuFade());
    flow.update(GameFlow::kMenuFadeIn);
    CHECK(flow.menuFade() == 1.0f, "start menu fade %.2f", flow.menuFade());

    GameFlow skipped;
    skipped.update(0.5);
    skipped.handle(key(&MenuInput::any), settings);
    CHECK(skipped.screen() == Screen::MainMenu && skipped.menuFade() == 0.0f, "any key skips the intro");

    // Main menu: Start, Level, Garage, Settings, Quit; Up from the top wraps to Quit.
    CHECK(flow.current() == MenuItem::Start, "cursor starts on Start");
    MenuResult r = flow.handle(key(&MenuInput::up), settings);
    CHECK(flow.current() == MenuItem::Quit && r.sound == MenuSound::Move, "up wraps to Quit");
    flow.handle(key(&MenuInput::down), settings);
    flow.handle(key(&MenuInput::down), settings);
    CHECK(flow.current() == MenuItem::LevelChoice, "down moves to Level");
    // Level row: cycles the level ids (empty setting = the first), LevelChanged on every change.
    r = flow.handle(key(&MenuInput::right), settings);
    CHECK(r.command == MenuCommand::None && settings.level.empty(), "no levels set: the row does nothing");
    flow.setLevels({"exploration_peaks", "proving_ground"});
    CHECK(flow.levelIndex(settings) == 0 && menuValue(MenuItem::LevelChoice, settings, "exploration_peaks") ==
              "EXPLORATION PEAKS", "default level shown");
    r = flow.handle(key(&MenuInput::right), settings);
    CHECK(r.command == MenuCommand::LevelChanged && settings.level == "proving_ground" &&
              menuValue(MenuItem::LevelChoice, settings) == "PROVING GROUND", "right picks the proving ground");
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::LevelChanged && settings.level == "exploration_peaks", "accept cycles (wraps)");
    CHECK(flow.screen() == Screen::MainMenu, "the level row stays on the start menu");
    flow.handle(key(&MenuInput::down), settings);
    CHECK(flow.current() == MenuItem::Garage, "down moves to Garage");
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::OpenGarage && flow.screen() == Screen::Garage && flow.menuActive() &&
              flow.simulationRuns(),
          "accept opens the garage (menu active, simulation running)");
    r = flow.handle(key(&MenuInput::back), settings);
    CHECK(flow.screen() == Screen::Garage && r.command == MenuCommand::None, "GameFlow leaves garage input alone");
    flow.closeGarage();
    CHECK(flow.screen() == Screen::MainMenu && flow.current() == MenuItem::Garage, "garage closes to the start menu");
    flow.handle(key(&MenuInput::down), settings);
    CHECK(flow.current() == MenuItem::Settings, "down moves to Settings");
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(flow.page() == MenuPage::Settings && r.sound == MenuSound::Confirm, "accept opens Settings");

    // Settings: left / right change values, Back (Esc) returns with SettingsClosed.
    CHECK(flow.current() == MenuItem::Quality, "settings start at the quality preset");
    r = flow.handle(key(&MenuInput::right), settings);
    CHECK(settings.quality == 3 && r.command == MenuCommand::SettingsChanged, "quality High -> Ultra (%d)",
          settings.quality);
    r = flow.handle(key(&MenuInput::right), settings);
    CHECK(settings.quality == 0, "quality wraps Ultra -> Low (%d)", settings.quality);
    flow.select(MenuItem::MusicVolume);
    flow.handle(key(&MenuInput::left), settings);
    CHECK(std::abs(settings.musicVolume - 0.75f) < 1e-6f, "music 0.80 -> 0.75 (%.3f)", double(settings.musicVolume));
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::None && std::abs(settings.musicVolume - 0.75f) < 1e-6f,
          "accept on a slider changes nothing");
    flow.select(MenuItem::Traction);
    flow.handle(key(&MenuInput::accept), settings);
    CHECK(settings.traction == 0, "accept cycles traction control (%d)", settings.traction);
    CHECK(menuValue(MenuItem::Traction, settings) == "OFF", "value text '%s'",
          menuValue(MenuItem::Traction, settings).c_str());
    flow.handle(key(&MenuInput::left), settings);
    flow.handle(key(&MenuInput::left), settings);
    CHECK(settings.traction == 2 && menuValue(MenuItem::Traction, settings) == "OFF-ROAD",
          "left wraps vehicle default -> off-road (%d)", settings.traction);
    flow.handle(key(&MenuInput::right), settings);
    CHECK(settings.traction == -1 && menuValue(MenuItem::Traction, settings) == "VEHICLE DEFAULT", "right wraps back");
    // Mouse: hover moves the cursor, a click on the right arrow steps the value.
    MenuInput mouse;
    mouse.hover = 4;
    flow.handle(mouse, settings);
    CHECK(flow.current() == MenuItem::VSync, "hover moves the cursor");
    mouse = {};
    mouse.click = 4;
    mouse.clickDir = 1;
    flow.handle(mouse, settings);
    CHECK(!settings.vsync, "click toggles vsync");
    flow.select(MenuItem::GrassDistance);
    CHECK(menuValue(MenuItem::GrassDistance, settings) == "PRESET (40 M)", "grass row '%s'",
          menuValue(MenuItem::GrassDistance, settings).c_str());
    flow.handle(key(&MenuInput::right), settings);
    CHECK(settings.grassDistance == 30.0f && menuValue(MenuItem::GrassDistance, settings) == "30 M",
          "grass distance steps off the preset (%.0f)", double(settings.grassDistance));
    flow.handle(key(&MenuInput::left), settings);
    CHECK(settings.grassDistance == 0.0f, "left returns to the preset");
    flow.select(MenuItem::TreeDistance);
    flow.handle(key(&MenuInput::left), settings);
    CHECK(settings.treeDistance == 0.0f, "tree distance stays at the preset on the left end");
    // Time of day: half-hour steps that wrap at midnight, presets after the current time, the time flow, and the
    // debug windows switch (off by default).
    CHECK(flow.select(MenuItem::TimeOfDay) && menuValue(MenuItem::TimeOfDay, settings) == "09:00" &&
              menuValue(MenuItem::TimePreset, settings) == "MORNING",
          "time row '%s'", menuValue(MenuItem::TimeOfDay, settings).c_str());
    r = flow.handle(key(&MenuInput::right), settings);
    CHECK(r.command == MenuCommand::SettingsChanged && settings.timeOfDay == 9.5f &&
              menuValue(MenuItem::TimePreset, settings) == "CUSTOM",
          "time 09:00 -> 09:30 (%.2f)", double(settings.timeOfDay));
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::None && settings.timeOfDay == 9.5f, "accept on the time row changes nothing");
    settings.timeOfDay = 9.37f; // a clock that ran
    flow.handle(key(&MenuInput::left), settings);
    CHECK(settings.timeOfDay == 9.0f, "left from 09:22 stops on 09:00 (%.2f)", double(settings.timeOfDay));
    settings.timeOfDay = 23.5f;
    flow.handle(key(&MenuInput::right), settings);
    CHECK(settings.timeOfDay == 0.0f && menuValue(MenuItem::TimeOfDay, settings) == "00:00", "time wraps at midnight");
    flow.handle(key(&MenuInput::left), settings);
    CHECK(settings.timeOfDay == 23.5f, "and back (%.2f)", double(settings.timeOfDay));
    flow.handle(key(&MenuInput::down), settings);
    CHECK(flow.current() == MenuItem::TimePreset, "down moves to the preset row");
    flow.handle(key(&MenuInput::right), settings);
    CHECK(settings.timeOfDay == 5.75f && menuValue(MenuItem::TimePreset, settings) == "DAWN", "night -> dawn (wraps)");
    flow.handle(key(&MenuInput::accept), settings);
    CHECK(settings.timeOfDay == 9.0f, "accept: the next preset (%.2f)", double(settings.timeOfDay));
    settings.timeOfDay = 14.0f;
    flow.handle(key(&MenuInput::right), settings);
    CHECK(menuValue(MenuItem::TimePreset, settings) == "GOLDEN HOUR", "14:00 -> golden hour");
    settings.timeOfDay = 14.0f;
    flow.handle(key(&MenuInput::left), settings);
    CHECK(menuValue(MenuItem::TimePreset, settings) == "NOON", "14:00 <- noon");
    flow.handle(key(&MenuInput::down), settings);
    CHECK(flow.current() == MenuItem::TimeFlow && menuValue(MenuItem::TimeFlow, settings) == "OFF", "time flow row");
    flow.handle(key(&MenuInput::left), settings);
    CHECK(settings.timeFlow == 2 && menuValue(MenuItem::TimeFlow, settings) == "FAST (1 H PER MIN)", "time flow wraps");
    flow.handle(key(&MenuInput::accept), settings);
    CHECK(settings.timeFlow == 0, "accept cycles the time flow");
    flow.handle(key(&MenuInput::down), settings);
    CHECK(flow.current() == MenuItem::DebugWindows && !settings.debugWindows &&
              menuValue(MenuItem::DebugWindows, settings) == "OFF",
          "debug windows row, off by default");
    flow.handle(key(&MenuInput::accept), settings);
    CHECK(settings.debugWindows && menuValue(MenuItem::DebugWindows, settings) == "ON", "accept toggles debug windows");
    flow.handle(key(&MenuInput::left), settings);
    CHECK(!settings.debugWindows, "left toggles back");
    settings.timeOfDay = 9.0f;
    flow.select(MenuItem::TreeStyle);
    CHECK(menuValue(MenuItem::TreeStyle, settings) == "STYLIZED", "trees row '%s'",
          menuValue(MenuItem::TreeStyle, settings).c_str());
    r = flow.handle(key(&MenuInput::right), settings);
    CHECK(r.command == MenuCommand::SettingsChanged && !settings.stylizedTrees &&
              menuValue(MenuItem::TreeStyle, settings) == "DETAILED",
          "right switches to the detailed trees");
    flow.handle(key(&MenuInput::accept), settings);
    CHECK(settings.stylizedTrees, "accept switches back");
    flow.select(MenuItem::Controls);
    flow.handle(key(&MenuInput::accept), settings);
    CHECK(flow.page() == MenuPage::Controls && flow.depth() == 3, "controls page");
    // Controls: drive keys W A S D <-> arrows, gamepad buttons auto -> Xbox -> PlayStation.
    CHECK(flow.current() == MenuItem::DriveKeys && menuValue(MenuItem::DriveKeys, settings) == "W A S D",
          "controls start on DRIVE KEYS = W A S D");
    r = flow.handle(key(&MenuInput::right), settings);
    CHECK(settings.driveKeys == 1 && r.command == MenuCommand::SettingsChanged &&
              menuValue(MenuItem::DriveKeys, settings) == "ARROWS",
          "right switches to the arrows (%d)", settings.driveKeys);
    flow.handle(key(&MenuInput::accept), settings);
    CHECK(settings.driveKeys == 0, "accept switches back to W A S D");
    flow.handle(key(&MenuInput::down), settings);
    CHECK(flow.current() == MenuItem::PadGlyphs && menuValue(MenuItem::PadGlyphs, settings) == "AUTO", "pad glyphs row");
    flow.handle(key(&MenuInput::right), settings);
    flow.handle(key(&MenuInput::right), settings);
    CHECK(settings.padGlyphs == 2 && menuValue(MenuItem::PadGlyphs, settings) == "PLAYSTATION", "pad glyphs %d",
          settings.padGlyphs);
    flow.handle(key(&MenuInput::right), settings);
    CHECK(settings.padGlyphs == 0, "pad glyphs wrap to auto");
    flow.handle(key(&MenuInput::down), settings);
    CHECK(flow.current() == MenuItem::Back, "back row last");
    r = flow.handle(key(&MenuInput::back), settings);
    CHECK(flow.page() == MenuPage::Settings && r.command == MenuCommand::None && r.sound == MenuSound::Back,
          "back from controls");
    r = flow.handle(key(&MenuInput::back), settings);
    CHECK(flow.page() == MenuPage::Main && r.command == MenuCommand::SettingsClosed, "back from settings");
    r = flow.handle(key(&MenuInput::back), settings);
    CHECK(flow.screen() == Screen::MainMenu && r.command == MenuCommand::None, "back at the main menu stays");

    // Start -> gameplay; Esc pauses; Esc at the pause root resumes.
    flow.select(MenuItem::Start);
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::StartGame && flow.screen() == Screen::Playing && !flow.menuActive(),
          "start game");
    MenuInput esc = key(&MenuInput::pause);
    esc.back = true;
    r = flow.handle(esc, settings);
    CHECK(flow.screen() == Screen::Paused && !flow.simulationRuns() && r.sound == MenuSound::Open, "Esc pauses");
    CHECK(flow.current() == MenuItem::Resume, "pause menu starts at Resume");
    r = flow.handle(esc, settings);
    CHECK(flow.screen() == Screen::Playing && r.command == MenuCommand::Resume, "Esc resumes");
    flow.pause();
    flow.select(MenuItem::Restart);
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::Restart && flow.screen() == Screen::Playing, "reset vehicle resumes");
    flow.pause();
    flow.openSettings();
    r = flow.handle(esc, settings);
    CHECK(flow.screen() == Screen::Paused && flow.page() == MenuPage::Pause, "Esc in the pause settings goes back");
    flow.select(MenuItem::Garage);
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::OpenGarage && flow.screen() == Screen::Garage && flow.simulationRuns(),
          "garage from the pause menu runs the simulation");
    flow.closeGarage();
    CHECK(flow.screen() == Screen::Paused && flow.current() == MenuItem::Garage && !flow.simulationRuns(),
          "garage closes back to the (frozen) pause menu");
    // Map: a pause menu row; the map owns its input and freezes the simulation; it closes back to where it opened.
    CHECK(GameFlow::items(MenuPage::Pause)[1] == MenuItem::Map, "MAP is the second pause row");
    flow.select(MenuItem::Map);
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::OpenMap && flow.screen() == Screen::Map && flow.menuActive() && !flow.simulationRuns(),
          "MAP opens the frozen full-screen map");
    r = flow.handle(esc, settings);
    CHECK(flow.screen() == Screen::Map && r.command == MenuCommand::None, "GameFlow leaves map input alone");
    flow.closeMap();
    CHECK(flow.screen() == Screen::Paused && flow.current() == MenuItem::Map, "map closes to the pause menu on MAP");
    flow.resume();
    flow.openMap();
    CHECK(flow.screen() == Screen::Map && flow.mapReturn() == Screen::Playing, "map from gameplay");
    flow.closeMap();
    CHECK(flow.screen() == Screen::Playing && !flow.menuActive(), "map closes to gameplay");
    flow.pause();
    flow.openSettings();
    flow.select(MenuItem::Minimap);
    CHECK(std::strcmp(menuSection(MenuItem::Minimap), "MAP") == 0 && menuValue(MenuItem::Minimap, settings) == "ON",
          "MAP section, minimap on");
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(!settings.minimap && r.command == MenuCommand::SettingsChanged, "minimap off");
    flow.select(MenuItem::MinimapSize);
    flow.handle(key(&MenuInput::right), settings);
    CHECK(settings.minimapSize == 2 && menuValue(MenuItem::MinimapSize, settings) == "LARGE", "minimap size %d",
          settings.minimapSize);
    flow.select(MenuItem::MinimapNorthUp);
    flow.handle(key(&MenuInput::right), settings);
    CHECK(settings.minimapNorthUp && menuValue(MenuItem::MinimapNorthUp, settings) == "NORTH UP", "north up");
    flow.select(MenuItem::Compass);
    flow.handle(key(&MenuInput::left), settings);
    CHECK(!settings.compass, "compass off");
    flow.handle(key(&MenuInput::back), settings);
    CHECK(flow.page() == MenuPage::Pause, "back to the pause menu");
    flow.select(MenuItem::QuitToMenu);
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::QuitToMenu && flow.screen() == Screen::MainMenu && flow.menuFade() == 1.0f,
          "quit to main menu");
    flow.select(MenuItem::Quit);
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::QuitGame, "quit");
}

// Vehicle page: Start opens it when there is more than one vehicle; left / right, the wheel and card clicks change
// GameSettings::vehicle (VehicleChanged), accept / DRIVE / a click on the selected card start the game.
void vehiclePage() {
    GameSettings settings;
    GameFlow flow(Screen::MainMenu);
    flow.setVehicles({"trailforge_suv", "trailforge_ev"});
    CHECK(flow.vehicleIndex(settings) == 0, "empty choice = the default (first) vehicle");
    MenuResult r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(flow.page() == MenuPage::Vehicle && flow.current() == MenuItem::VehicleChoice && r.command == MenuCommand::None &&
              flow.screen() == Screen::MainMenu,
          "Start opens the vehicle page");
    r = flow.handle(key(&MenuInput::right), settings);
    CHECK(r.command == MenuCommand::VehicleChanged && settings.vehicle == "trailforge_ev" && flow.vehicleIndex(settings) == 1,
          "right selects the EV");
    r = flow.handle(key(&MenuInput::right), settings);
    CHECK(r.command == MenuCommand::VehicleChanged && settings.vehicle == "trailforge_suv", "right wraps to the SUV");
    MenuInput wheel;
    wheel.wheel = -1;
    r = flow.handle(wheel, settings);
    CHECK(r.command == MenuCommand::VehicleChanged && settings.vehicle == "trailforge_ev", "wheel down: next vehicle");
    MenuInput pick;
    pick.pick = 0;
    r = flow.handle(pick, settings);
    CHECK(r.command == MenuCommand::VehicleChanged && settings.vehicle == "trailforge_suv" && flow.page() == MenuPage::Vehicle,
          "a click on another card selects it");
    r = flow.handle(key(&MenuInput::back), settings);
    CHECK(flow.page() == MenuPage::Main && flow.current() == MenuItem::Start && settings.vehicle == "trailforge_suv",
          "back keeps the choice");
    flow.handle(key(&MenuInput::accept), settings);
    r = flow.handle(pick, settings);
    CHECK(r.command == MenuCommand::StartGame && flow.screen() == Screen::Playing, "a click on the selected card drives");
    flow.toMainMenu();
    flow.openVehiclePage();
    flow.select(MenuItem::Drive);
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::StartGame && flow.screen() == Screen::Playing, "DRIVE starts the game");
    flow.toMainMenu();
    flow.openVehiclePage();
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::StartGame, "accept on the vehicle row starts the game");
    GameFlow single(Screen::MainMenu);
    single.setVehicles({"trailforge_suv"});
    r = single.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::StartGame, "one vehicle: Start drives directly");
    settings.vehicle = "gone_vehicle";
    CHECK(flow.vehicleIndex(settings) == 0, "an unknown id falls back to the default vehicle");

    // Card specs from the definitions.
    std::vector<std::string> errors;
    const std::vector<VehicleDefinition> catalog = loadVehicleCatalog(TRAILFORGE_SOURCE_DIR "/assets/vehicles", errors);
    CHECK(catalog.size() >= 2 && catalog.front().id == kDefaultVehicleId, "catalog starts with the default vehicle");
    for (const VehicleDefinition& d : catalog) {
        std::printf("    %s:", d.id.c_str());
        for (const VehicleSpec& sp : vehicleSpecs(d.config)) std::printf(" %s %s;", sp.label.c_str(), sp.value.c_str());
        std::printf("\n");
        if (d.config.isElectric()) {
            CHECK(std::abs(peakPowerKW(d.config) - 0.001f * (d.config.electric.motors[0].maxPower +
                                                             d.config.electric.motors[1].maxPower)) < 0.01f,
                  "EV power = motor ratings");
            CHECK(vehicleSpecs(d.config)[4].label == "BATTERY", "EV card shows the battery");
        } else {
            // Plausibility of the card value.
            CHECK(peakPowerKW(d.config) > 100.0f && peakPowerKW(d.config) < 500.0f, "engine power %.0f kW",
                  double(peakPowerKW(d.config)));
        }
    }
}

// The pause menu freezes the simulation: no ticks, the same body position and accumulator; resume continues.
void pauseFreezesTicks() {
    Simulation sim{SimulationSettings{}};
    FixedStepClock clock(sim.fixedStep());
    GameFlow flow(Screen::Playing);
    const double frame = 1.0 / 60.0;
    auto runFrames = [&](int n, const DriverIntent& in) {
        for (int i = 0; i < n; ++i) {
            sim.setIntent(in);
            const int steps = flow.stepsForFrame(clock, frame * (i % 3 == 0 ? 1.37 : 0.9)); // uneven frames
            for (int k = 0; k < steps; ++k) sim.tick();
        }
    };
    runFrames(120, {});
    runFrames(60, DriverIntent{.throttle = 1.0f});
    const auto ticks0 = sim.timing().ticks;
    const JPH::Vec3 p0 = sim.currentSnapshot().body.position;
    const double alpha0 = clock.alpha();
    flow.pause();
    runFrames(240, DriverIntent{.throttle = 1.0f}); // throttle held while paused
    const JPH::Vec3 p1 = sim.currentSnapshot().body.position;
    std::printf("    paused 240 frames: ticks %llu -> %llu, body z %.4f -> %.4f, alpha %.3f -> %.3f\n",
                (unsigned long long)ticks0, (unsigned long long)sim.timing().ticks, double(p0.GetZ()),
                double(p1.GetZ()), alpha0, clock.alpha());
    CHECK(sim.timing().ticks == ticks0, "ticks advanced while paused");
    CHECK(p0 == p1, "the body moved while paused");
    CHECK(clock.alpha() == alpha0, "the fixed-step accumulator changed while paused");
    flow.resume();
    runFrames(60, DriverIntent{.throttle = 1.0f});
    const JPH::Vec3 p2 = sim.currentSnapshot().body.position;
    std::printf("    resumed 60 frames: ticks +%llu, moved %.2f m\n",
                (unsigned long long)(sim.timing().ticks - ticks0), double((p2 - p1).Length()));
    CHECK(sim.timing().ticks > ticks0 + 100, "resume did not continue the simulation");
    CHECK((p2 - p1).Length() > 0.5f, "the vehicle did not move after resume");
}

GarageInput gkey(bool GarageInput::*field) {
    GarageInput in;
    in.*field = true;
    return in;
}

// Garage: factory -> change values -> save -> rename -> reload from disk -> delete; ranges and clamping; kits;
// live values and in-place rebuilds reach a running simulation.
void garageModel() {
    std::vector<std::string> errors;
    std::vector<VehicleDefinition> catalog = loadVehicleCatalog(TRAILFORGE_SOURCE_DIR "/assets/vehicles", errors);
    VehicleDefinition rwd;
    CHECK(loadVehicleDefinition(TRAILFORGE_SOURCE_DIR "/tests/data/vehicles/rwd_test/vehicle.json", rwd, errors),
          "rwd_test did not load");
    catalog.push_back(rwd);
    for (const std::string& e : errors) std::printf("    %s\n", e.c_str());
    CHECK(catalog.size() >= 2 && errors.empty(), "catalog: %zu definitions, %zu errors", catalog.size(), errors.size());
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "trailforge_garage_tests";
    std::filesystem::remove_all(dir);

    Garage g(catalog, dir.string());
    CHECK(g.vehicle().id == "trailforge_suv" && g.factoryActive() && !g.dirty(), "starts on the SUV factory setup");
    CHECK(g.config() == catalog[0].config, "factory config == definition");

    // Spring / damper equivalents: SUV front corner 2000 kg x 0.518 / 2 = 518 kg, 1.35 Hz -> 37.3 N/mm.
    {
        const VehicleConfig& c = g.config();
        const float m = sprungCornerMass(c, kFrontAxle), mr = sprungCornerMass(c, kRearAxle);
        const float k = wheelRateFromFrequency(c.tuning.axles[kFrontAxle].springFrequency, m);
        std::printf("    front corner %.0f kg, rear %.0f kg, front wheel rate %.1f N/mm\n", double(m), double(mr),
                    double(k / 1000.0f));
        CHECK(std::abs(m + mr - 0.5f * c.mass) < 1e-3f && std::abs(m - 517.86f) < 0.5f, "corner masses");
        CHECK(std::abs(k / 1000.0f - 37.26f) < 0.1f, "wheel rate %.2f N/mm", double(k / 1000.0f));
        CHECK(std::abs(frequencyFromWheelRate(k, m) - 1.35f) < 1e-4f, "N/m -> Hz round trip");
        CHECK(std::abs(criticalDamping(1.35f, m) - 2.0f * std::sqrt(k * m)) < 1.0f, "critical damping 2 sqrt(k m)");
        CHECK(g.selectRow(GarageRowKind::Param, "suspension.front.frequency") &&
                  g.valueNote(g.rows()[std::size_t(g.cursor())]) == "37 N/mm",
              "spring row note '%s'", g.valueNote(g.rows()[std::size_t(g.cursor())]).c_str());
        CHECK(g.selectRow(GarageRowKind::Param, "suspension.front.compression") &&
                  !g.helpText(g.rows()[std::size_t(g.cursor())]).empty(),
              "damping row help");
        CHECK(g.selectRow(GarageRowKind::Vehicle) && g.valueNote(g.rows()[0]).empty(), "no note on other rows");
    }

    // Keyboard: cursor to the front spring, right steps it; the range clamps.
    CHECK(g.selectRow(GarageRowKind::Param, "suspension.front.frequency"), "front spring row");
    const SetupParamInfo& spring = *findSetupParam("suspension.front.frequency");
    const float base = g.value(spring);
    GarageResult r = g.handle(gkey(&GarageInput::right));
    std::printf("    front spring %.2f -> %.2f Hz (range %s)\n", double(base), double(g.value(spring)),
                g.rangeText(spring).c_str());
    CHECK(std::abs(g.value(spring) - (base + spring.step)) < 1e-4f && r.configChanged && g.dirty(), "right steps up");
    for (int i = 0; i < 200; ++i) g.handle(gkey(&GarageInput::right));
    CHECK(std::abs(g.value(spring) - g.range(spring).max) < 1e-4f, "clamped at the range max %.3f", double(g.value(spring)));
    g.setValue("suspension.front.frequency", 1.6f);
    CHECK(std::abs(g.config().tuning.axles[kFrontAxle].springFrequency - 1.6f) < 1e-6f, "config follows the value");

    // Mouse: a click on the slider bar at 0 sets the minimum.
    GarageInput click;
    click.click = g.cursor();
    click.clickFraction = 0.0f;
    g.handle(click);
    CHECK(std::abs(g.value(spring) - g.range(spring).min) < 1e-4f, "bar click at 0 -> min (%.3f)", double(g.value(spring)));
    g.setValue("suspension.front.frequency", 1.6f);

    // Save from factory creates "Setup 1"; rename; a second setup; reload from disk.
    CHECK(g.save() && g.activeName() == "Setup 1" && !g.dirty(), "save from factory -> Setup 1");
    CHECK(std::filesystem::exists(dir / "trailforge_suv.json"), "setup file written");
    CHECK(!g.rename("Factory") && !g.rename("  "), "reserved / empty names refused");
    CHECK(g.rename("Rock crawl") && g.activeName() == "Rock crawl", "rename");
    g.setValue("drivetrain.layout", float(int(DriveLayout::Rwd)));
    CHECK(g.config().layout == DriveLayout::Rwd && g.dirty(), "layout choice");
    CHECK(g.saveAsNew() && g.activeName() == "Setup 1" && g.setups().setups.size() == 2, "save as new");
    CHECK(!g.rename("Rock crawl"), "duplicate name refused");
    {
        Garage reloaded(catalog, dir.string());
        CHECK(reloaded.activeName() == "Setup 1" && reloaded.config().layout == DriveLayout::Rwd &&
                  std::abs(reloaded.config().tuning.axles[kFrontAxle].springFrequency - 1.6f) < 1e-6f,
              "reloaded from disk: active '%s'", reloaded.activeName().c_str());
        CHECK(reloaded.selectSetup("Rock crawl") && reloaded.config().layout == catalog[0].config.layout,
              "the other setup keeps the base layout");
    }
    // Rename typed through the input path.
    g.selectRow(GarageRowKind::Action, "RENAME");
    g.handle(gkey(&GarageInput::accept));
    CHECK(g.renaming() && g.modal() == GarageModal::Rename && g.renameSelected(), "rename modal, name preselected");
    {
        // Modal validation: unsafe characters refused, a taken name blocks Confirm, rows do not react.
        const int cursorBefore = g.cursor();
        g.handle(gkey(&GarageInput::down));
        CHECK(g.cursor() == cursorBefore, "rows ignore input under the modal");
        GarageInput bad;
        bad.text = "a/b";
        g.handle(bad);
        CHECK(g.renameText() == "ab" && !g.modalError().empty(), "'/' refused: '%s' (%s)", g.renameText().c_str(),
              g.modalError().c_str());
        g.handle(gkey(&GarageInput::backspace));
        g.handle(gkey(&GarageInput::backspace));
        GarageInput taken;
        taken.text = "Rock crawl";
        g.handle(taken);
        g.handle(gkey(&GarageInput::accept));
        CHECK(g.renaming() && g.modalError().find("ALREADY") != std::string::npos, "taken name keeps the modal: %s",
              g.modalError().c_str());
        CHECK(g.nameProblem("Mud").empty() && !g.nameProblem("factory").empty() &&
                  !g.nameProblem(std::string(40, 'x')).empty() && !g.nameProblem("  ").empty(),
              "name rules");
    }
    for (int i = 0; i < 12; ++i) g.handle(gkey(&GarageInput::backspace));
    GarageInput typed;
    typed.text = "Mud";
    g.handle(typed);
    g.handle(gkey(&GarageInput::accept));
    CHECK(!g.renaming() && g.activeName() == "Mud", "typed rename -> '%s'", g.activeName().c_str());
    // Delete asks first: Cancel has the focus, Esc cancels, Confirm deletes.
    g.selectRow(GarageRowKind::Action, "DELETE");
    g.handle(gkey(&GarageInput::accept));
    CHECK(g.modal() == GarageModal::ConfirmDelete && g.modalFocus() == 1, "delete confirmation, focus on Cancel");
    g.handle(gkey(&GarageInput::accept));
    CHECK(g.modal() == GarageModal::None && g.activeName() == "Mud", "Enter on Cancel keeps the setup");
    g.handle(gkey(&GarageInput::accept));
    GarageInput confirm;
    confirm.modalClick = 0;
    g.handle(confirm);
    CHECK(g.modal() == GarageModal::None && g.factoryActive() && g.setups().setups.size() == 1, "delete -> factory");
    CHECK(g.config() == catalog[0].config, "factory config after delete");
    // Selection on the Setup row: right cycles Factory -> Rock crawl.
    g.selectRow(GarageRowKind::Setup);
    g.handle(gkey(&GarageInput::right));
    CHECK(g.activeName() == "Rock crawl", "setup row cycles to '%s'", g.activeName().c_str());
    g.resetToFactory();
    CHECK(g.config() == catalog[0].config && g.dirty(), "reset to factory (unsaved)");

    // Kit on the test vehicle: overrides apply, its ranges replace the base ones, values clamp into them.
    CHECK(g.selectVehicle("rwd_test") && g.factoryActive(), "test vehicle");
    const float massBase = g.config().mass;
    CHECK(g.setKit("test_lift") && g.config().mass > massBase, "kit adds mass %.0f -> %.0f", double(massBase),
          double(g.config().mass));
    const ParamRange ride = g.range(*findSetupParam("suspension.rideHeight"));
    CHECK(std::abs(ride.min + 0.02f) < 1e-6f && std::abs(ride.max - 0.12f) < 1e-6f, "kit ride height range %.3f..%.3f",
          double(ride.min), double(ride.max));
    g.setValue("suspension.rideHeight", 0.5f);
    CHECK(std::abs(g.value(*findSetupParam("suspension.rideHeight")) - 0.12f) < 1e-6f, "ride height clamped to the kit");
    CHECK(!g.setKit("no_such_kit"), "unknown kit refused");

    // Electric vehicle: no engine, gearbox, layout or center differential rows; motor and regen rows instead. A value
    // of a parameter that does not apply (from a hand-edited file) is ignored with a warning.
    {
        const auto hasRow = [&](const char* key) {
            for (const GarageRow& r : g.rows())
                if (r.kind == GarageRowKind::Param && std::strcmp(r.param->key, key) == 0) return true;
            return false;
        };
        CHECK(hasRow("engine.torqueScale") && !hasRow("motors.torqueScale"), "combustion rows on the test vehicle");
        CHECK(g.selectRow(GarageRowKind::Param, "brakes.frontBias"), "cursor on a shared row");
        CHECK(g.selectVehicle("trailforge_ev") && g.factoryActive(), "EV selected");
        CHECK(g.rows()[std::size_t(g.cursor())].kind == GarageRowKind::Param &&
                  std::strcmp(g.rows()[std::size_t(g.cursor())].param->key, "brakes.frontBias") == 0,
              "cursor kept on the same row across vehicles");
        for (const char* key : {"engine.torqueScale", "engine.revLimit", "gearbox.finalDrive", "drivetrain.layout",
                                "drivetrain.centerDiff"})
            CHECK(!hasRow(key), "EV shows %s", key);
        for (const char* key : {"motors.torqueScale", "motors.regenScale", "regen.liftOffDecel",
                                "drivetrain.offRoadFrontShare", "drivetrain.frontShare"})
            CHECK(hasRow(key), "EV lacks %s", key);
        const float rear = g.config().electric.motors[kRearAxle].maxTorque;
        CHECK(g.setValue("motors.torqueScale", 0.8f) &&
                  std::abs(g.config().electric.motors[kRearAxle].maxTorque - 0.8f * rear) < 1e-3f,
              "motor torque scale");
        CHECK(g.setValue("regen.liftOffDecel", 0.0f) && g.config().electric.liftOffDecel == 0.0f, "lift-off regen off");
        CHECK(g.config().layout == DriveLayout::DualMotor, "layout stays dual motor");
        VehicleSetup bad;
        bad.name = "Hand edited";
        bad.values["gearbox.finalDrive"] = 4.0f;
        VehicleConfig out;
        std::vector<std::string> warnings;
        CHECK(buildSetupConfig(g.vehicle(), bad, out, warnings) && out == g.vehicle().config && warnings.size() == 1,
              "inapplicable value ignored (%zu warnings)", warnings.size());
        g.resetToFactory();
    }

    // A running simulation: live values change the vehicle between ticks, rebuild values respawn it in place.
    g.selectVehicle("trailforge_suv");
    Simulation sim{[&] {
        SimulationSettings s;
        s.vehicle = g.config();
        return s;
    }()};
    for (int i = 0; i < 240; ++i) sim.tick();
    g.setValue("suspension.front.frequency", 1.7f);
    sim.requestVehicleConfig(g.config());
    sim.tick();
    CHECK(std::abs(sim.vehicleConfig().tuning.axles[kFrontAxle].springFrequency - 1.7f) < 1e-6f, "live value applied");
    const JPH::Vec3 before = sim.currentSnapshot().body.position;
    g.setValue("suspension.rideHeight", 0.05f);
    sim.requestVehicleConfig(g.config());
    sim.tick();
    for (int i = 0; i < 240; ++i) sim.tick();
    const JPH::Vec3 after = sim.currentSnapshot().body.position;
    std::printf("    ride height +50 mm: body y %.3f -> %.3f, horizontal move %.3f m\n", double(before.GetY()),
                double(after.GetY()), double(std::hypot(after.GetX() - before.GetX(), after.GetZ() - before.GetZ())));
    CHECK(after.GetY() > before.GetY() + 0.03f, "body higher after +50 mm ride height");
    CHECK(std::hypot(after.GetX() - before.GetX(), after.GetZ() - before.GetZ()) < 0.05f, "rebuilt in place");
    std::filesystem::remove_all(dir);
}

} // namespace

// Bindings: every key / control once, the drive and free-camera clusters swap, labels per platform style.
void controlsLayout() {
    for (const DriveKeys layout : {DriveKeys::Wasd, DriveKeys::Arrows}) {
        const std::vector<KeyBinding> b = keyboardBindings(layout);
        for (std::size_t i = 0; i < b.size(); ++i)
            for (std::size_t j = i + 1; j < b.size(); ++j)
                CHECK(b[i].key != b[j].key, "key %s bound twice (%s / %s)", keyName(b[i].key, KeyboardStyle::Pc),
                      b[i].label.c_str(), b[j].label.c_str());
        const DriveKeyMap d = driveKeyMap(layout), c = freeCameraKeyMap(layout);
        for (const Key k : {d.forward, d.back, d.left, d.right}) {
            for (const Key m : {c.forward, c.back, c.left, c.right}) CHECK(k != m, "drive key also moves the camera");
            for (const KeyAction& a : keyActions()) CHECK(k != a.key, "drive key %s also triggers an action", keyName(k, KeyboardStyle::Pc));
        }
        int drive = 0;
        for (const KeyBinding& k : b) {
            CHECK(!k.label.empty(), "key without a callout");
            drive += k.group == BindingGroup::Drive;
        }
        CHECK(drive == 5, "drive keys + Space: %d", drive);
        // The page shows gameplay keys only: menu navigation is in the footer hints.
        for (const KeyBinding& k : b) {
            CHECK(k.label.find("MENU SELECT") == std::string::npos && k.label.find("MENU BACK") == std::string::npos,
                  "menu navigation callout '%s'", k.label.c_str());
            CHECK(k.key != Key::Return && k.key != Key::Backspace, "menu-only key %s drawn", keyName(k.key, KeyboardStyle::Pc));
        }
        std::printf("    %s: %zu keys bound, drive %s / %s / %s / %s\n", driveKeysName(layout), b.size(),
                    keyName(d.forward, KeyboardStyle::Pc), keyName(d.back, KeyboardStyle::Pc),
                    keyName(d.left, KeyboardStyle::Pc), keyName(d.right, KeyboardStyle::Pc));
    }
    CHECK(driveKeyMap(DriveKeys::Wasd).forward == Key::W && driveKeyMap(DriveKeys::Arrows).forward == Key::Up &&
              freeCameraKeyMap(DriveKeys::Wasd).forward == Key::Up && freeCameraKeyMap(DriveKeys::Arrows).left == Key::A,
          "cluster mapping");
    CHECK(driveHint(DriveKeys::Arrows, KeyboardStyle::Mac).rfind("UP gas / DOWN brake", 0) == 0, "arrows hint '%s'",
          driveHint(DriveKeys::Arrows, KeyboardStyle::Mac).c_str());
    CHECK(driveHint(DriveKeys::Wasd, KeyboardStyle::Pc).rfind("W gas / S brake", 0) == 0, "wasd hint");

    // Key cap wording: Mac vs PC where keyboards differ, the same elsewhere.
    struct Expect {
        Key key;
        const char* mac;
        const char* pc;
    };
    for (const Expect& e : {Expect{Key::Return, "RETURN", "ENTER"}, Expect{Key::Backspace, "DELETE", "BACKSPACE"},
                            Expect{Key::LeftAlt, "OPTION", "ALT"}, Expect{Key::LeftGui, "COMMAND", "WIN"},
                            Expect{Key::LeftCtrl, "CONTROL", "CTRL"}, Expect{Key::PageUp, "PAGE UP", "PG UP"},
                            Expect{Key::Escape, "ESC", "ESC"}, Expect{Key::Space, "SPACE", "SPACE"}, Expect{Key::W, "W", "W"}}) {
        CHECK(std::strcmp(keyName(e.key, KeyboardStyle::Mac), e.mac) == 0, "mac name '%s'", keyName(e.key, KeyboardStyle::Mac));
        CHECK(std::strcmp(keyName(e.key, KeyboardStyle::Pc), e.pc) == 0, "pc name '%s'", keyName(e.key, KeyboardStyle::Pc));
    }
    CHECK(keyLabel(Key::Up, KeyboardStyle::Mac)[0] == '\0', "arrow caps are drawn, not lettered");
#if defined(__APPLE__)
    CHECK(nativeKeyboardStyle() == KeyboardStyle::Mac, "macOS uses Mac key caps");
#else
    CHECK(nativeKeyboardStyle() == KeyboardStyle::Pc, "Windows / Linux use PC key caps");
#endif
    // Key positions keep the ANSI order inside a row and the inverted T of the arrows.
    CHECK(keyRect(Key::Q).x < keyRect(Key::W).x && keyRect(Key::A).row == keyRect(Key::S).row &&
              keyRect(Key::Up).x == keyRect(Key::Down).x && keyRect(Key::Up).row + 1 == keyRect(Key::Down).row &&
              keyRect(Key::Left).x + 1.0f == keyRect(Key::Down).x,
          "key geometry");

    // Gamepad bindings: each control once; names per glyph style.
    const std::vector<PadBinding> pad = padBindings();
    for (std::size_t i = 0; i < pad.size(); ++i)
        for (std::size_t j = i + 1; j < pad.size(); ++j)
            CHECK(pad[i].control != pad[j].control, "pad control bound twice: %s", padControlName(pad[i].control, PadGlyphs::Xbox));
    CHECK(std::strcmp(padControlName(PadControl::South, PadGlyphs::Xbox), "A") == 0 &&
              std::strcmp(padControlName(PadControl::South, PadGlyphs::PlayStation), "CROSS") == 0 &&
              std::strcmp(padControlName(PadControl::North, PadGlyphs::PlayStation), "TRIANGLE") == 0 &&
              std::strcmp(padControlName(PadControl::RightTrigger, PadGlyphs::PlayStation), "R2") == 0 &&
              std::strcmp(padControlName(PadControl::Back, PadGlyphs::Xbox), "VIEW") == 0 &&
              std::strcmp(padControlName(PadControl::Start, PadGlyphs::PlayStation), "OPTIONS") == 0,
          "pad control names");
    bool rightStick = false;
    for (const PadBinding& p : pad) {
        CHECK(p.control != PadControl::East, "B / Circle (menu back only) has a callout");
        CHECK(std::string(p.label).find("MENU BACK") == std::string::npos, "menu navigation callout '%s'", std::string(p.label).c_str());
        rightStick |= p.control == PadControl::RightStick && p.group == BindingGroup::Camera;
    }
    CHECK(rightStick, "right stick camera callout");
    std::printf("    gamepad: %zu controls bound\n", pad.size());
}

// Automatic glyphs from the connected pad type (the values mirror SDL_GamepadType; Input.cpp static_asserts that).
void padGlyphDetect() {
    struct Case {
        PadType type;
        PadGlyphs glyphs;
    };
    for (const Case& c : {Case{PadType::None, PadGlyphs::Xbox}, Case{PadType::Unknown, PadGlyphs::Xbox},
                          Case{PadType::Standard, PadGlyphs::Xbox}, Case{PadType::Xbox360, PadGlyphs::Xbox},
                          Case{PadType::XboxOne, PadGlyphs::Xbox}, Case{PadType::PS3, PadGlyphs::PlayStation},
                          Case{PadType::PS4, PadGlyphs::PlayStation}, Case{PadType::PS5, PadGlyphs::PlayStation},
                          Case{PadType::SwitchPro, PadGlyphs::Xbox}, Case{PadType::JoyConPair, PadGlyphs::Xbox},
                          Case{PadType::GameCube, PadGlyphs::Xbox}, Case{PadType::Steam, PadGlyphs::Xbox}}) {
        CHECK(glyphsForPadType(c.type) == c.glyphs, "%s -> %s", padTypeName(c.type), padGlyphsName(glyphsForPadType(c.type)));
        CHECK(resolvePadGlyphs(PadGlyphSetting::Auto, c.type) == c.glyphs, "auto %s", padTypeName(c.type));
        CHECK(resolvePadGlyphs(PadGlyphSetting::Xbox, c.type) == PadGlyphs::Xbox, "override Xbox for %s", padTypeName(c.type));
        CHECK(resolvePadGlyphs(PadGlyphSetting::PlayStation, c.type) == PadGlyphs::PlayStation, "override PS for %s",
              padTypeName(c.type));
    }
    CHECK(int(PadType::Unknown) == 0 && int(PadType::PS5) == 6 && int(PadType::Steam) == 12, "SDL_GamepadType order");
}

// Callout placement: no overlaps, least displacement, inside the bounds; label wrapping at word breaks.
void calloutSpread() {
    const std::vector<float> want = {10.0f, 12.0f, 13.0f, 40.0f, 41.0f};
    const std::vector<float> size = {6.0f, 8.0f, 4.0f, 10.0f, 10.0f};
    const std::vector<float> at = spreadLabels(want, size, 1.0f, 0.0f, 100.0f);
    for (std::size_t i = 1; i < at.size(); ++i)
        CHECK(at[i] - at[i - 1] >= 0.5f * (size[i] + size[i - 1]) + 1.0f - 1e-4f, "labels %zu / %zu overlap", i - 1, i);
    CHECK(std::abs((at[0] + at[1] + at[2]) / 3.0f - (10.0f + 12.0f + 13.0f) / 3.0f) < 1e-3f, "group stays centred");
    const std::vector<float> clamped = spreadLabels({1.0f, 2.0f}, {4.0f, 4.0f}, 0.0f, 0.0f, 100.0f);
    CHECK(clamped[0] - 2.0f >= -1e-4f && clamped[1] - clamped[0] >= 4.0f - 1e-4f, "left bound %.2f %.2f", double(clamped[0]),
          double(clamped[1]));
    const std::vector<float> right = spreadLabels({99.0f}, {10.0f}, 0.0f, 0.0f, 100.0f);
    CHECK(std::abs(right[0] - 95.0f) < 1e-4f, "right bound %.2f", double(right[0]));
    CHECK(spreadLabels({}, {}, 1.0f, 0.0f, 1.0f).empty(), "empty");
    std::printf("    spread: %.2f %.2f %.2f %.2f %.2f\n", double(at[0]), double(at[1]), double(at[2]), double(at[3]), double(at[4]));

    const auto chars = [](const std::string& t) { return float(t.size()); };
    CHECK(wrapLabel("ABS", 5.0f, chars) == std::vector<std::string>{"ABS"}, "short label stays");
    CHECK((wrapLabel("BRAKE / REVERSE", 9.0f, chars) == std::vector<std::string>{"BRAKE /", "REVERSE"}), "slash split");
    CHECK((wrapLabel("TRACTION CONTROL", 9.0f, chars) == std::vector<std::string>{"TRACTION", "CONTROL"}), "word split");
    CHECK(wrapLabel("DIAGNOSTICS", 5.0f, chars) == std::vector<std::string>{"DIAGNOSTICS"}, "one word cannot split");
}

// Map math: world <-> screen with and without rotation, bearings, the compass tape, distances, scale bar, zoom
// around the pointer, panning, the marker toggle, and the compass distance shrinking while driving to the marker.
void mapViewMath() {
    auto near = [](float a, float b, float eps = 1e-3f) { return std::abs(a - b) <= eps; };
    MapView v;
    v.cx = 100.0f;
    v.cz = -50.0f;
    v.sx = 400.0f;
    v.sy = 300.0f;
    v.scale = 2.0f;
    MapPoint p = v.toScreen(100.0f, -150.0f); // 100 m north
    CHECK(near(p.x, 400.0f) && near(p.y, 100.0f), "north is up: (%.2f, %.2f)", double(p.x), double(p.y));
    p = v.toScreen(150.0f, -50.0f); // 50 m east
    CHECK(near(p.x, 500.0f) && near(p.y, 300.0f), "east is right: (%.2f, %.2f)", double(p.x), double(p.y));
    v.rotationDeg = 90.0f; // heading east: east points up, north points left
    p = v.toScreen(150.0f, -50.0f);
    CHECK(near(p.x, 400.0f) && near(p.y, 200.0f), "heading east: east is up (%.2f, %.2f)", double(p.x), double(p.y));
    p = v.toScreen(100.0f, -100.0f);
    CHECK(near(p.x, 300.0f) && near(p.y, 300.0f), "heading east: north is left (%.2f, %.2f)", double(p.x), double(p.y));
    for (const float rot : {0.0f, 37.0f, 90.0f, 211.0f, -75.0f}) {
        v.rotationDeg = rot;
        for (const auto& w : {MapPoint{-300.0f, 20.0f}, MapPoint{100.0f, -50.0f}, MapPoint{512.5f, 731.25f}}) {
            const MapPoint s = v.toScreen(w.x, w.y), back = v.toWorld(s.x, s.y);
            CHECK(near(back.x, w.x, 1e-2f) && near(back.y, w.y, 1e-2f), "roundtrip at %.0f deg: (%.3f, %.3f)", double(rot),
                  double(back.x), double(back.y));
        }
    }
    v.rotationDeg = 0.0f;
    const MapPoint under = v.toWorld(620.0f, 140.0f);
    v.zoomAround(2.5f, 620.0f, 140.0f, 0.1f, 100.0f);
    const MapPoint after = v.toWorld(620.0f, 140.0f);
    CHECK(near(v.scale, 5.0f) && near(after.x, under.x, 1e-2f) && near(after.y, under.y, 1e-2f), "zoom keeps the pointed point");
    v.zoomAround(1000.0f, 400.0f, 300.0f, 0.1f, 8.0f);
    CHECK(near(v.scale, 8.0f), "zoom clamped at the maximum (%.2f)", double(v.scale));
    const MapPoint c0 = v.toWorld(400.0f, 300.0f);
    v.pan(80.0f, -40.0f); // content dragged right and up: the view looks further west and south
    const MapPoint c1 = v.toWorld(400.0f, 300.0f);
    CHECK(near(c1.x, c0.x - 10.0f) && near(c1.y, c0.y + 5.0f), "pan (%.2f, %.2f)", double(c1.x - c0.x), double(c1.y - c0.y));

    CHECK(near(bearingDeg(0.0f, -1.0f), 0.0f) && near(bearingDeg(1.0f, 0.0f), 90.0f) && near(bearingDeg(0.0f, 1.0f), 180.0f) &&
              near(bearingDeg(-1.0f, 0.0f), 270.0f) && near(bearingDeg(1.0f, -1.0f), 45.0f),
          "bearings N/E/S/W/NE");
    CHECK(near(relativeBearingDeg(10.0f, 350.0f), 20.0f) && near(relativeBearingDeg(350.0f, 10.0f), -20.0f) &&
              near(relativeBearingDeg(180.0f, 0.0f), 180.0f) && near(relativeBearingDeg(90.0f, 270.0f), 180.0f),
          "relative bearings wrap");
    const auto off = compassOffset(45.0f, 0.0f, 90.0f);
    CHECK(off && near(*off, 0.5f), "NE on a north tape at +0.5");
    CHECK(!compassOffset(135.0f, 0.0f, 90.0f) && compassOffset(300.0f, 350.0f, 90.0f) &&
              near(*compassOffset(300.0f, 350.0f, 90.0f), -50.0f / 90.0f),
          "outside the tape / wrapped left");
    CHECK(std::strcmp(compassPointName(0.0f), "N") == 0 && std::strcmp(compassPointName(44.0f), "NE") == 0 &&
              std::strcmp(compassPointName(359.0f), "N") == 0 && std::strcmp(compassPointName(225.0f), "SW") == 0,
          "compass point names");
    CHECK(formatDistance(8.4f) == "8 m" && formatDistance(354.0f) == "350 m" && formatDistance(996.0f) == "1.0 km" &&
              formatDistance(1234.0f) == "1.2 km" && formatDistance(12600.0f) == "13 km",
          "distances: %s %s %s %s %s", formatDistance(8.4f).c_str(), formatDistance(354.0f).c_str(),
          formatDistance(996.0f).c_str(), formatDistance(1234.0f).c_str(), formatDistance(12600.0f).c_str());
    CHECK(near(scaleBarMeters(1.0f, 160.0f), 100.0f) && near(scaleBarMeters(0.37f, 160.0f), 200.0f) &&
              near(scaleBarMeters(6.0f, 160.0f), 20.0f) && scaleBarMeters(0.0f, 100.0f) == 0.0f,
          "scale bar lengths %.0f %.0f %.0f", double(scaleBarMeters(1.0f, 160.0f)), double(scaleBarMeters(0.37f, 160.0f)),
          double(scaleBarMeters(6.0f, 160.0f)));

    MapMarker m;
    CHECK(toggleMarker(m, 10.0f, 20.0f, 5.0f) && m.set && near(m.x, 10.0f), "first click sets");
    CHECK(toggleMarker(m, 40.0f, 20.0f, 5.0f) && near(m.x, 40.0f), "a click elsewhere moves it");
    CHECK(!toggleMarker(m, 42.0f, 21.0f, 5.0f) && !m.set, "a click on it clears it");

    // Driving towards the marker: the compass distance shrinks and the marker stays near the tape middle.
    m = {true, 300.0f, -400.0f};
    float x = 0.0f, z = 0.0f, last = 1e9f;
    bool shrinking = true;
    for (int i = 0; i < 20; ++i) {
        const float heading = bearingDeg(x, z, m.x, m.z);
        const float d = std::hypot(m.x - x, m.z - z);
        shrinking = shrinking && d < last;
        last = d;
        const auto o = compassOffset(bearingDeg(x, z, m.x, m.z), heading, 90.0f);
        CHECK(o && std::abs(*o) < 1e-3f, "marker centered while heading to it");
        x += 20.0f * std::sin(heading * 0.0174532925f);
        z -= 20.0f * std::cos(heading * 0.0174532925f);
    }
    CHECK(shrinking && last < 121.0f, "distance shrinks to %.1f m", double(last)); // 500 m, 19 steps of 20 m
}

// Map data of Exploration Peaks (worker-thread build, timed) and of the proving ground (its triangles).
void mapLevelRaster() {
    std::string error;
    const auto level = Level::load(TRAILFORGE_SOURCE_DIR "/assets/levels/exploration_peaks/level.json", &error);
    CHECK(level != nullptr, "level: %s", error.c_str());
    if (!level) return;
    const MapData d = buildLevelMap(*level);
    const MapRaster& r = d.raster;
    std::printf("    %s: raster %d^2 at %.2f m/px over %.0f m, %zu lines, %zu POIs, built in %.0f ms\n", d.title.c_str(),
                r.size, double(r.metersPerPixel), double(r.size) * double(r.metersPerPixel), d.lines.size(), d.pois.size(),
                d.buildMs);
    CHECK(r.size == 2048 && r.rgba.size() == std::size_t(r.size) * std::size_t(r.size), "raster size");
    CHECK(r.x0 < d.playX0 && r.x0 + float(r.size) * r.metersPerPixel > d.playX1, "raster covers the play area");
    CHECK(d.pois.size() == level->pois().size() && d.pois.size() == 13, "13 points of interest (%zu)", d.pois.size());
    int roads = 0, trails = 0, rivers = 0, bridges = 0;
    for (const MapLine& l : d.lines) {
        roads += l.kind == MapLineKind::Road;
        trails += l.kind == MapLineKind::Trail;
        rivers += l.kind == MapLineKind::River;
        bridges += l.kind == MapLineKind::Bridge;
        CHECK(l.points.size() >= 2, "line '%s' with %zu points", l.name.c_str(), l.points.size());
    }
    CHECK(roads > 0 && trails > 0 && rivers > 0 && bridges == int(level->bridges().size()), "roads %d trails %d rivers %d bridges %d",
          roads, trails, rivers, bridges);
    // Colors where the level says what is there: water is blue, dry ground is not, every pixel opaque.
    auto pixel = [&](float x, float z) {
        const int i = std::clamp(int((x - r.x0) / r.metersPerPixel), 0, r.size - 1);
        const int j = std::clamp(int((z - r.z0) / r.metersPerPixel), 0, r.size - 1);
        return r.rgba[std::size_t(j) * r.size + i];
    };
    int wet = 0, wetBlue = 0, dry = 0, dryBlue = 0;
    for (float z = -900.0f; z <= 900.0f; z += 37.0f)
        for (float x = -900.0f; x <= 900.0f; x += 37.0f) {
            const std::uint32_t c = pixel(x, z);
            CHECK((c >> 24) == 0xFF, "transparent pixel at %.0f, %.0f", double(x), double(z));
            const int R = int(c & 0xFF), B = int((c >> 16) & 0xFF);
            if (level->waterAt(x, z)) {
                ++wet;
                wetBlue += B > R + 20;
            } else {
                ++dry;
                dryBlue += B > R + 20;
            }
        }
    std::printf("    samples: %d wet (%d blue), %d dry (%d blue)\n", wet, wetBlue, dry, dryBlue);
    CHECK(wet > 50 && wetBlue >= wet * 9 / 10, "water drawn blue: %d of %d", wetBlue, wet);
    CHECK(dryBlue <= dry / 50, "dry ground drawn blue: %d of %d", dryBlue, dry);

    SimulationSettings s;
    Simulation sim(s);
    ProvingGroundMapInput in;
    in.geometry = &sim.track().geometry();
    const TrackBounds b = TestTrack::bounds();
    in.x0 = b.xMin;
    in.x1 = b.xMax;
    in.z0 = b.zMin;
    in.z1 = b.zMax;
    const WaterInfo w = TestTrack::water();
    in.waterLevel = w.level;
    in.waterX0 = w.x0;
    in.waterZ0 = w.z0;
    in.waterX1 = w.x1;
    in.waterZ1 = w.z1;
    const MapData pg = buildProvingGroundMap(in);
    int pgWet = 0;
    for (const std::uint32_t c : pg.raster.rgba) pgWet += int((c >> 16) & 0xFF) > int(c & 0xFF) + 40;
    std::printf("    proving ground: raster %d^2 at %.3f m/px, %d water pixels, built in %.0f ms\n", pg.raster.size,
                double(pg.raster.metersPerPixel), pgWet, pg.buildMs);
    CHECK(pg.raster.size == 1024 && pgWet > 200, "proving ground puddles on the map (%d px)", pgWet);
    // The start pad (concrete) is grey, not grass green.
    const MapRaster& q = pg.raster;
    const int i = int((0.0f - q.x0) / q.metersPerPixel), j = int((-40.0f - q.z0) / q.metersPerPixel);
    const std::uint32_t c = q.rgba[std::size_t(j) * q.size + i];
    const int R = int(c & 0xFF), G = int((c >> 8) & 0xFF), B = int((c >> 16) & 0xFF);
    CHECK(std::abs(R - G) < 25 && std::abs(G - B) < 25, "start pad color %d %d %d", R, G, B);
}

// Setup files of schema 1 (anti-roll in Jolt bar units) load with their anti-roll values dropped, everything else
// kept; the garage keeps a backup and stores the file as schema 2 once.
void setupMigration() {
    // A real schema 1 file as the v0.27.0 garage wrote it (the owner's SUV setup).
    const std::string v1 = R"({
  "schema": 1,
  "selected": "Setup 1",
  "setups": [
    {
      "kit": "",
      "name": "Setup 1",
      "values": {
        "suspension.front.antiRoll": 11000,
        "suspension.front.compression": 0.5399999618530273,
        "suspension.front.preload": 0.03999999910593033,
        "suspension.front.rebound": 0.6800000071525574,
        "suspension.rear.antiRoll": 15000,
        "suspension.rear.compression": 0.5799999833106995,
        "suspension.rear.preload": 0.03999999910593033,
        "suspension.rear.rebound": 0.7400000095367432,
        "suspension.rideHeight": -0.04999999701976776,
        "tires.gripScale": 1.0399999618530273
      }
    },
    {"kit": "", "name": "No bars", "values": {"suspension.front.frequency": 1.5}}
  ],
  "vehicle": "trailforge_suv"
})";
    SetupFile f;
    std::string error;
    CHECK(parseSetupFile(v1, f, &error), "schema 1 rejected: %s", error.c_str());
    for (const std::string& n : f.migrationNotes) std::printf("    %s\n", n.c_str());
    CHECK(f.loadedSchema == 1 && f.migrationNotes.size() == 2, "schema %d, %zu notes", f.loadedSchema,
          f.migrationNotes.size());
    const VehicleSetup* s = f.find("Setup 1");
    CHECK(s && s->values.size() == 8 && !s->values.count("suspension.front.antiRoll") &&
              !s->values.count("suspension.rear.antiRoll") &&
              std::abs(s->values.at("suspension.front.rebound") - 0.68f) < 1e-6f,
          "Setup 1 keeps its other 8 values");
    CHECK(f.find("No bars") && f.find("No bars")->values.size() == 1 && f.selected == "Setup 1", "second setup kept");

    // Schema 2 keeps anti-roll values (N/m) as they are; unknown schemas fail.
    SetupFile back;
    const std::string v2 = writeSetupFile(f);
    CHECK(v2.find("\"schema\": 2") != std::string::npos, "written as schema 2");
    SetupFile withBars = f;
    withBars.setups[0].values["suspension.front.antiRoll"] = 12000.0f;
    CHECK(parseSetupFile(writeSetupFile(withBars), back, &error) && back.loadedSchema == 2 &&
              back.migrationNotes.empty() && back.find("Setup 1")->values.at("suspension.front.antiRoll") == 12000.0f,
          "schema 2 anti-roll kept");
    std::string bad = v1;
    bad.replace(bad.find("\"schema\": 1"), 11, "\"schema\": 3");
    CHECK(!parseSetupFile(bad, back, &error), "schema 3 accepted");

    // Garage: migrates the file on disk once, with a backup of the original.
    std::vector<std::string> errors;
    std::vector<VehicleDefinition> catalog = loadVehicleCatalog(TRAILFORGE_SOURCE_DIR "/assets/vehicles", errors);
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "trailforge_setup_migration";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const std::filesystem::path path = dir / "trailforge_suv.json";
    { std::ofstream(path, std::ios::binary) << v1; }
    Garage g(catalog, dir.string());
    std::printf("    garage message: %s\n", g.message().c_str());
    CHECK(g.vehicle().id == "trailforge_suv", "garage starts on the SUV");
    const std::filesystem::path backup = dir / "trailforge_suv.json.schema1.bak";
    std::string kept;
    {
        std::ifstream in(backup, std::ios::binary);
        std::stringstream ss;
        ss << in.rdbuf();
        kept = ss.str();
    }
    CHECK(kept == v1, "backup holds the original schema 1 text");
    SetupFile onDisk;
    CHECK(loadSetupFile(path.string(), onDisk, &error) && onDisk.loadedSchema == 2 && onDisk.migrationNotes.empty() &&
              onDisk.find("Setup 1") && onDisk.find("Setup 1")->values.size() == 8,
          "file stored as schema 2 without the anti-roll values");
    CHECK(g.message().find("anti-roll") != std::string::npos, "garage message '%s'", g.message().c_str());
    const VehicleConfig& c = g.config();
    CHECK(c.tuning.axles[kFrontAxle].antiRoll == catalog[0].config.tuning.axles[kFrontAxle].antiRoll &&
              c.tuning.axles[kRearAxle].antiRoll == catalog[0].config.tuning.axles[kRearAxle].antiRoll &&
              std::abs(c.tuning.axles[kFrontAxle].reboundDamping - 0.68f) < 1e-6f,
          "active setup: vehicle bars, the other values applied");
    std::filesystem::remove_all(dir);
}

// On-screen touch controls: the layout fits a landscape iPhone with its safe area and a desktop window (no overlaps,
// inside the safe area), fingers map to buttons, the stick steers both ways with a dead zone and a centre that follows
// past full lock, a pedal finger slides between GAS and BRAKE, and only one finger steers.
void touchControlsLogic() {
    struct Screen {
        float w, h, l, t, r, b;
    };
    for (const Screen& sc : {Screen{874, 402, 62, 0, 62, 21}, Screen{1280, 720, 0, 0, 0, 0}, Screen{667, 375, 0, 0, 0, 0}}) {
        const TouchLayout L = touchLayout(sc.w, sc.h, sc.l, sc.t, sc.r, sc.b);
        for (int i = 0; i < kTouchButtonCount; ++i) {
            const TouchRect& a = L.buttons[std::size_t(i)];
            CHECK(a.x0 >= sc.l && a.x1 <= sc.w - sc.r && a.y0 >= sc.t && a.y1 <= sc.h - sc.b && a.x1 > a.x0 + 30.0f &&
                      a.y1 > a.y0 + 30.0f,
                  "%gx%g: %s inside the safe area, at least 30 points", sc.w, sc.h, touchButtonName(TouchButton(i)));
            for (int j = i + 1; j < kTouchButtonCount; ++j) {
                const TouchRect& c = L.buttons[std::size_t(j)];
                CHECK(a.x1 <= c.x0 || c.x1 <= a.x0 || a.y1 <= c.y0 || c.y1 <= a.y0, "%gx%g: %s overlaps %s", sc.w, sc.h,
                      touchButtonName(TouchButton(i)), touchButtonName(TouchButton(j)));
            }
            CHECK(!L.steerZone.contains(a.cx(), a.cy()) || i >= int(TouchButton::Pause),
                  "%gx%g: pedal %s in the steering zone", sc.w, sc.h, touchButtonName(TouchButton(i)));
        }
        CHECK(L.steerZone.contains(L.restX, L.restY) && L.restX - L.stickRadius >= sc.l,
              "%gx%g: stick rest inside the zone and the safe area", sc.w, sc.h);
    }

    TouchControls tc;
    const TouchLayout L = touchLayout(874, 402, 62, 0, 62, 21);
    tc.setLayout(L);
    const TouchRect& gas = L.buttons[std::size_t(TouchButton::Throttle)];
    const TouchRect& brake = L.buttons[std::size_t(TouchButton::Brake)];
    CHECK(tc.fingerDown(1, gas.cx(), gas.cy()) == TouchButton::Throttle && tc.held(TouchButton::Throttle), "GAS");
    tc.fingerMove(1, brake.cx(), brake.cy());
    CHECK(!tc.held(TouchButton::Throttle) && tc.held(TouchButton::Brake), "GAS finger slid onto BRAKE");
    tc.fingerMove(1, 10.0f, 10.0f);
    CHECK(tc.held(TouchButton::Brake), "a pedal stays held when the finger leaves it");

    const float r = L.stickRadius;
    CHECK(tc.fingerDown(2, L.restX, L.restY) == TouchButton::Count && tc.steering() && tc.steer() == 0.0f, "stick down");
    CHECK(tc.stickX() == L.restX && tc.stickY() == L.restY, "stick centre under the finger");
    tc.fingerMove(2, L.restX + 0.03f * r, L.restY);
    CHECK(tc.steer() == 0.0f, "dead zone: %g", tc.steer());
    tc.fingerMove(2, L.restX + 0.5f * r, L.restY - 40.0f);
    CHECK(tc.steer() > 0.4f && tc.steer() < 0.5f, "half right, vertical motion ignored: %g", tc.steer());
    tc.fingerMove(2, L.restX - 3.0f * r, L.restY);
    CHECK(tc.steer() == -1.0f && std::abs(tc.stickX() - (L.restX - 2.0f * r)) < 1e-3f, "full left, centre follows: %g %g",
          tc.steer(), tc.stickX());
    tc.fingerMove(2, L.restX - 2.0f * r, L.restY);
    CHECK(tc.steer() == 0.0f, "back to the dragged centre: %g", tc.steer());
    CHECK(tc.fingerDown(3, L.restX + 20.0f, L.restY) == TouchButton::Count && tc.fingers() == 3, "second finger in the zone");
    tc.fingerMove(3, L.restX + 3.0f * r, L.restY);
    CHECK(tc.steer() == 0.0f, "only the first finger steers: %g", tc.steer());
    tc.fingerUp(2);
    CHECK(!tc.steering() && tc.steer() == 0.0f && tc.stickX() == L.restX, "stick released back to rest");
    tc.fingerUp(1);
    CHECK(!tc.held(TouchButton::Brake) && tc.fingers() == 1, "BRAKE released");
    tc.releaseAll();
    CHECK(tc.fingers() == 0, "release all");
    const TouchRect& pause = L.buttons[std::size_t(TouchButton::Pause)];
    CHECK(tc.fingerDown(4, pause.cx(), pause.cy()) == TouchButton::Pause, "pause button");
}

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"setup_migration", setupMigration},
        {"map_view_math", mapViewMath},
        {"map_level_raster", mapLevelRaster},
        {"settings_roundtrip", settingsRoundtrip},
        {"best_laps_file", bestLapsFile},
        {"menu_navigation", menuNavigation},
        {"pause_freezes_ticks", pauseFreezesTicks},
        {"garage_model", garageModel},
        {"vehicle_page", vehiclePage},
        {"controls_layout", controlsLayout},
        {"pad_glyph_detect", padGlyphDetect},
        {"callout_spread", calloutSpread},
        {"touch_controls_logic", touchControlsLogic},
    };
    int ran = 0;
    for (const auto& [name, fn] : tests) {
        if (argc > 1 && std::strcmp(argv[1], name) != 0) continue;
        std::printf("[ RUN ] %s\n", name);
        const int before = g_failures;
        fn();
        std::printf("[%s] %s\n", g_failures == before ? "  OK " : "FAIL ", name);
        ++ran;
    }
    if (ran == 0) {
        std::printf("unknown test: %s\n", argc > 1 ? argv[1] : "");
        return 2;
    }
    return g_failures == 0 ? 0 : 1;
}
