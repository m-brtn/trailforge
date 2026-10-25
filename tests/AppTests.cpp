// Headless checks of the game flow: settings file, menu navigation, and the pause freezing the simulation.
// Usage: trailforge_app_tests <test_name>

#include "app/GameFlow.h"
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

    // Main menu: Start, Settings, Quit; Up from the top wraps to Quit.
    CHECK(flow.current() == MenuItem::Start, "cursor starts on Start");
    MenuResult r = flow.handle(key(&MenuInput::up), settings);
    CHECK(flow.current() == MenuItem::Quit && r.sound == MenuSound::Move, "up wraps to Quit");
    flow.handle(key(&MenuInput::down), settings);
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

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"settings_roundtrip", settingsRoundtrip},
        {"menu_navigation", menuNavigation},
        {"pause_freezes_ticks", pauseFreezesTicks},
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
