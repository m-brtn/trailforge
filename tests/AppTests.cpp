// Headless checks of the game flow: settings file, menu navigation, the pause freezing the simulation, and the
// garage model (setups, ranges, persistence, applying to a running simulation).
// Usage: trailforge_app_tests <test_name>

#include "app/GameFlow.h"
#include "app/Garage.h"
#include "app/GameSettings.h"
#include "sim/FixedStepClock.h"
#include "sim/Simulation.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
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

void settingsRoundtrip() {
    const GameSettings defaults;
    CHECK(parseSettings(serializeSettings(defaults)) == defaults, "defaults do not survive a roundtrip");

    GameSettings s;
    s.quality = 0;
    s.renderScale = 0.75f;
    s.vsync = false;
    s.masterVolume = 0.35f;
    s.musicVolume = 0.0f;
    s.effectsVolume = 0.6f;
    s.uiVolume = 0.25f;
    s.abs = true;
    s.traction = 2;
    s.diffLocks = 3;
    s.autoReverse = false;
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
                                           "vsync = banana\n"
                                           "no equals sign\n"
                                           "future_option = 1\n",
                                           &skipped);
    std::printf("    damaged file: quality %d music %.2f ui %.2f scale %.2f tc %d diff %d vsync %d, %d lines skipped\n",
                odd.quality, double(odd.musicVolume), double(odd.uiVolume), double(odd.renderScale), odd.traction,
                odd.diffLocks, odd.vsync, skipped);
    CHECK(odd.quality == 3 && odd.musicVolume == 1.0f && odd.uiVolume == 0.0f && odd.renderScale == 0.0f &&
              odd.traction == 2 && odd.diffLocks == 2 && odd.vsync == defaults.vsync,
          "damaged values not clamped as expected");
    CHECK(skipped == 3, "skipped lines %d (expected 3)", skipped);
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

    // Main menu: Start, Garage, Settings, Quit; Up from the top wraps to Quit.
    CHECK(flow.current() == MenuItem::Start, "cursor starts on Start");
    MenuResult r = flow.handle(key(&MenuInput::up), settings);
    CHECK(flow.current() == MenuItem::Quit && r.sound == MenuSound::Move, "up wraps to Quit");
    flow.handle(key(&MenuInput::down), settings);
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
    CHECK(settings.traction == 1, "accept cycles traction control (%d)", settings.traction);
    CHECK(menuValue(MenuItem::Traction, settings) == "ON", "value text '%s'",
          menuValue(MenuItem::Traction, settings).c_str());
    // Mouse: hover moves the cursor, a click on the right arrow steps the value.
    MenuInput mouse;
    mouse.hover = 2;
    flow.handle(mouse, settings);
    CHECK(flow.current() == MenuItem::VSync, "hover moves the cursor");
    mouse = {};
    mouse.click = 2;
    mouse.clickDir = 1;
    flow.handle(mouse, settings);
    CHECK(!settings.vsync, "click toggles vsync");
    flow.select(MenuItem::Controls);
    flow.handle(key(&MenuInput::accept), settings);
    CHECK(flow.page() == MenuPage::Controls && flow.depth() == 3, "controls page");
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
    flow.select(MenuItem::QuitToMenu);
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::QuitToMenu && flow.screen() == Screen::MainMenu && flow.menuFade() == 1.0f,
          "quit to main menu");
    flow.select(MenuItem::Quit);
    r = flow.handle(key(&MenuInput::accept), settings);
    CHECK(r.command == MenuCommand::QuitGame, "quit");
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
    CHECK(g.renaming(), "rename mode");
    for (int i = 0; i < 8; ++i) g.handle(gkey(&GarageInput::backspace));
    GarageInput typed;
    typed.text = "Mud";
    g.handle(typed);
    g.handle(gkey(&GarageInput::accept));
    CHECK(!g.renaming() && g.activeName() == "Mud", "typed rename -> '%s'", g.activeName().c_str());
    CHECK(g.deleteActive() && g.factoryActive() && g.setups().setups.size() == 1, "delete -> factory");
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

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"settings_roundtrip", settingsRoundtrip},
        {"menu_navigation", menuNavigation},
        {"pause_freezes_ticks", pauseFreezesTicks},
        {"garage_model", garageModel},
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
