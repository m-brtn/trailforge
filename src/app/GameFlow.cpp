#include "app/GameFlow.h"

#include "input/ControlLayout.h"
#include "render/RenderSettings.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <iterator>

namespace tf {
namespace {

constexpr float kScaleSteps[] = {0.0f, 0.5f, 0.6f, 0.67f, 0.75f, 0.85f, 1.0f};
// 0 = the preset's own distance.
constexpr float kGrassSteps[] = {0.0f, 30.0f, 40.0f, 55.0f, 70.0f, 90.0f, 110.0f, 135.0f, 160.0f, 200.0f, 250.0f};
constexpr float kTreeSteps[] = {0.0f, 250.0f, 380.0f, 500.0f, 650.0f, 800.0f, 900.0f, 1100.0f, 1400.0f, 2000.0f};

float step(const float* steps, int n, float v, int dir) {
    int best = 0;
    for (int i = 0; i < n; ++i)
        if (std::abs(steps[i] - v) < std::abs(steps[best] - v)) best = i;
    return steps[std::clamp(best + dir, 0, n - 1)];
}

int wrap(int v, int n) { return ((v % n) + n) % n; }

constexpr float kSameTime = 1.0f / 120.0f; // h: a clock within 30 s of a preset shows that preset

bool stepVolume(float& v, int dir) {
    const float old = v;
    v = std::clamp(std::round((v + 0.05f * float(dir)) * 20.0f) / 20.0f, 0.0f, 1.0f);
    return v != old;
}

std::string percent(float v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%d %%", int(std::lround(v * 100.0f)));
    return buf;
}

} // namespace

GameFlow::GameFlow(Screen start) : m_screen(start) {
    m_stack.push_back({start == Screen::Paused ? MenuPage::Pause : MenuPage::Main, 0});
}

const std::vector<MenuItem>& GameFlow::items(MenuPage p) {
    using M = MenuItem;
    static const std::vector<MenuItem> kMain = {M::Start, M::LevelChoice, M::Garage, M::Settings, M::Quit};
    static const std::vector<MenuItem> kPause = {M::Resume, M::Map, M::Restart, M::Garage, M::Settings, M::QuitToMenu,
                                                 M::QuitGame};
    static const std::vector<MenuItem> kSettings = {M::Quality,       M::RenderScale, M::GrassDistance, M::TreeDistance,
                                                    M::VSync,         M::MasterVolume,
                                                    M::MusicVolume,   M::EffectsVolume, M::UiVolume, M::Abs,
                                                    M::Traction,      M::DiffLocks,   M::AutoReverse, M::Minimap,
                                                    M::MinimapSize,   M::MinimapNorthUp, M::Compass, M::TimeOfDay,
                                                    M::TimePreset,    M::TimeFlow,    M::DebugWindows, M::Controls,
                                                    M::Back};
    static const std::vector<MenuItem> kControls = {M::DriveKeys, M::PadGlyphs, M::Back};
    static const std::vector<MenuItem> kVehicle = {M::VehicleChoice, M::Drive, M::Back};
    switch (p) {
    case MenuPage::Main: return kMain;
    case MenuPage::Pause: return kPause;
    case MenuPage::Settings: return kSettings;
    case MenuPage::Controls: return kControls;
    case MenuPage::Vehicle: return kVehicle;
    }
    return kMain;
}

float GameFlow::menuFade() const {
    if (m_screen == Screen::Intro) return 0.0f;
    if (m_screen != Screen::MainMenu || m_menuFadeLength <= 0.0) return 1.0f;
    return float(std::clamp(m_screenTime / m_menuFadeLength, 0.0, 1.0));
}

void GameFlow::setScreen(Screen s, MenuPage root) {
    m_screen = s;
    m_stack.assign(1, PageState{root, 0});
    m_screenTime = 0.0;
    m_menuFadeLength = 0.0;
}

void GameFlow::update(double dt) {
    m_screenTime += dt;
    if (m_screen == Screen::Intro) {
        m_introTime += dt;
        if (m_introTime >= kIntroLength) {
            setScreen(Screen::MainMenu, MenuPage::Main);
            m_menuFadeLength = kMenuFadeIn;
        }
    }
}

void GameFlow::skipIntro() {
    if (m_screen != Screen::Intro) return;
    setScreen(Screen::MainMenu, MenuPage::Main);
    m_menuFadeLength = kSkipFade;
}

void GameFlow::startGame() { setScreen(Screen::Playing, MenuPage::Main); }

void GameFlow::pause() {
    if (m_screen == Screen::Playing) setScreen(Screen::Paused, MenuPage::Pause);
}

void GameFlow::resume() {
    if (m_screen == Screen::Paused) setScreen(Screen::Playing, MenuPage::Main);
}

void GameFlow::toMainMenu() { setScreen(Screen::MainMenu, MenuPage::Main); }

void GameFlow::openSettings() {
    if (!menuActive() || m_screen == Screen::Intro || page() == MenuPage::Settings) return;
    m_stack.push_back({MenuPage::Settings, 0});
}

void GameFlow::openGarage() {
    if (m_screen != Screen::MainMenu && m_screen != Screen::Paused) return;
    m_garageReturn = m_screen;
    setScreen(Screen::Garage, MenuPage::Main);
}

void GameFlow::closeGarage() {
    if (m_screen != Screen::Garage) return;
    setScreen(m_garageReturn, m_garageReturn == Screen::Paused ? MenuPage::Pause : MenuPage::Main);
    select(MenuItem::Garage);
}

void GameFlow::openMap() {
    if (m_screen != Screen::Playing && m_screen != Screen::Paused) return;
    m_mapReturn = m_screen;
    setScreen(Screen::Map, MenuPage::Main);
}

void GameFlow::closeMap() {
    if (m_screen != Screen::Map) return;
    setScreen(m_mapReturn, m_mapReturn == Screen::Paused ? MenuPage::Pause : MenuPage::Main);
    if (m_mapReturn == Screen::Paused) select(MenuItem::Map);
}

int GameFlow::vehicleIndex(const GameSettings& settings) const {
    const auto it = std::find(m_vehicleIds.begin(), m_vehicleIds.end(), settings.vehicle);
    return it == m_vehicleIds.end() ? 0 : int(it - m_vehicleIds.begin());
}

void GameFlow::openVehiclePage() {
    if (m_screen != Screen::MainMenu || page() != MenuPage::Main) return;
    m_stack.push_back({MenuPage::Vehicle, 0});
}

int GameFlow::levelIndex(const GameSettings& settings) const {
    const auto it = std::find(m_levelIds.begin(), m_levelIds.end(), settings.level);
    return it == m_levelIds.end() ? 0 : int(it - m_levelIds.begin());
}

MenuResult GameFlow::pickLevel(int index, GameSettings& settings) {
    if (m_levelIds.size() < 2) return {};
    const int old = levelIndex(settings);
    const int i = wrap(index, int(m_levelIds.size()));
    settings.level = m_levelIds[std::size_t(i)];
    return i == old ? MenuResult{} : MenuResult{MenuCommand::LevelChanged, MenuSound::Adjust};
}

MenuResult GameFlow::pickVehicle(int index, GameSettings& settings) {
    if (m_vehicleIds.empty()) return {};
    const int i = wrap(index, int(m_vehicleIds.size()));
    const bool same = i == vehicleIndex(settings);
    settings.vehicle = m_vehicleIds[std::size_t(i)];
    return same ? MenuResult{} : MenuResult{MenuCommand::VehicleChanged, MenuSound::Adjust};
}

bool GameFlow::back() {
    if (m_stack.size() <= 1) return false;
    m_stack.pop_back();
    return true;
}

bool GameFlow::select(MenuItem item) {
    const auto& list = items();
    const auto it = std::find(list.begin(), list.end(), item);
    if (it == list.end()) return false;
    m_stack.back().cursor = int(it - list.begin());
    return true;
}

MenuResult GameFlow::goBack() {
    const MenuPage leaving = page();
    if (back()) {
        return {leaving == MenuPage::Settings ? MenuCommand::SettingsClosed : MenuCommand::None, MenuSound::Back};
    }
    if (m_screen == Screen::Paused) {
        resume();
        return {MenuCommand::Resume, MenuSound::Close};
    }
    return {};
}

MenuResult GameFlow::activate(MenuItem item, GameSettings& settings, int dir) {
    using M = MenuItem;
    switch (item) {
    case M::Start:
        if (m_vehicleIds.size() > 1) {
            m_stack.push_back({MenuPage::Vehicle, 0});
            return {MenuCommand::None, MenuSound::Confirm};
        }
        startGame();
        return {MenuCommand::StartGame, MenuSound::Confirm};
    case M::LevelChoice: return pickLevel(levelIndex(settings) + (dir == 0 ? 1 : dir), settings);
    case M::VehicleChoice:
        if (dir != 0) return pickVehicle(vehicleIndex(settings) + dir, settings);
        [[fallthrough]];
    case M::Drive:
        startGame();
        return {MenuCommand::StartGame, MenuSound::Confirm};
    case M::Settings:
        m_stack.push_back({MenuPage::Settings, 0});
        return {MenuCommand::None, MenuSound::Confirm};
    case M::Garage:
        openGarage();
        return {MenuCommand::OpenGarage, MenuSound::Confirm};
    case M::Controls:
        m_stack.push_back({MenuPage::Controls, 0});
        return {MenuCommand::None, MenuSound::Confirm};
    case M::Quit:
    case M::QuitGame: return {MenuCommand::QuitGame, MenuSound::Confirm};
    case M::Resume:
        resume();
        return {MenuCommand::Resume, MenuSound::Close};
    case M::Map:
        openMap();
        return {MenuCommand::OpenMap, MenuSound::Open};
    case M::Restart:
        resume();
        return {MenuCommand::Restart, MenuSound::Confirm};
    case M::QuitToMenu:
        toMainMenu();
        return {MenuCommand::QuitToMenu, MenuSound::Confirm};
    case M::Back: return goBack();
    default: break;
    }
    // Volumes change only with left / right; choices and switches also cycle on accept.
    const bool slider = item == M::MasterVolume || item == M::MusicVolume || item == M::EffectsVolume ||
                        item == M::UiVolume || item == M::RenderScale || item == M::GrassDistance ||
                        item == M::TreeDistance || item == M::TimeOfDay;
    if (dir == 0 && slider) return {};
    if (!adjustSetting(item, settings, dir == 0 ? 1 : dir)) return {};
    return {MenuCommand::SettingsChanged, MenuSound::Adjust};
}

MenuResult GameFlow::handle(const MenuInput& in, GameSettings& settings) {
    if (m_screen == Screen::Intro) {
        if (in.any) skipIntro();
        return {};
    }
    if (m_screen == Screen::Garage || m_screen == Screen::Map) return {}; // the garage model / the map own the input
    if (m_screen == Screen::Playing) {
        if (in.pause) {
            pause();
            return {MenuCommand::None, MenuSound::Open};
        }
        return {};
    }
    const int n = int(items().size());
    int& cur = m_stack.back().cursor;
    MenuResult r;
    if (in.hover >= 0 && in.hover < n && in.hover != cur) {
        cur = in.hover;
        r.sound = MenuSound::Move;
    }
    if (in.up || in.down) {
        cur = wrap(cur + (in.down ? 1 : -1), n);
        r.sound = MenuSound::Move;
    }
    if (page() == MenuPage::Vehicle) {
        if (in.pick >= 0 && in.pick < int(m_vehicleIds.size())) {
            cur = 0;
            // A click on the selected card drives it; on another card it selects that vehicle.
            if (in.pick == vehicleIndex(settings)) return activate(MenuItem::Drive, settings, 0);
            return pickVehicle(in.pick, settings);
        }
        if (in.wheel != 0) {
            const MenuResult p = pickVehicle(vehicleIndex(settings) - in.wheel, settings);
            if (p.command != MenuCommand::None) return p;
        }
    }
    if (in.click >= 0 && in.click < n) {
        cur = in.click;
        const MenuItem item = items()[std::size_t(cur)];
        if (in.clickDir != 0 && menuAdjustable(item)) return activate(item, settings, in.clickDir);
        return activate(item, settings, 0);
    }
    const MenuItem item = items()[std::size_t(cur)];
    if ((in.left || in.right) && menuAdjustable(item)) {
        const MenuResult a = activate(item, settings, in.right ? 1 : -1);
        return a.command == MenuCommand::None ? r : a;
    }
    if (in.accept) return activate(item, settings, 0);
    if (in.back || in.pause) return goBack();
    return r;
}

const char* menuLabel(MenuItem item) {
    using M = MenuItem;
    switch (item) {
    case M::Start: return "START";
    case M::LevelChoice: return "LEVEL";
    case M::Garage: return "GARAGE";
    case M::Settings: return "SETTINGS";
    case M::Quit: return "QUIT";
    case M::Resume: return "RESUME";
    case M::Map: return "MAP";
    case M::Restart: return "RESET VEHICLE";
    case M::QuitToMenu: return "QUIT TO MAIN MENU";
    case M::QuitGame: return "QUIT GAME";
    case M::Quality: return "QUALITY PRESET";
    case M::RenderScale: return "RESOLUTION SCALE";
    case M::GrassDistance: return "GRASS DISTANCE";
    case M::TreeDistance: return "TREE DETAIL DISTANCE";
    case M::VSync: return "VSYNC";
    case M::MasterVolume: return "MASTER VOLUME";
    case M::MusicVolume: return "MUSIC";
    case M::EffectsVolume: return "EFFECTS";
    case M::UiVolume: return "INTERFACE";
    case M::Abs: return "ABS";
    case M::Traction: return "TRACTION CONTROL";
    case M::DiffLocks: return "DIFFERENTIAL LOCKS";
    case M::AutoReverse: return "AUTO REVERSE";
    case M::Minimap: return "MINIMAP";
    case M::MinimapSize: return "MINIMAP SIZE";
    case M::MinimapNorthUp: return "MINIMAP ORIENTATION";
    case M::Compass: return "COMPASS BAR";
    case M::TimeOfDay: return "TIME";
    case M::TimePreset: return "PRESET";
    case M::TimeFlow: return "TIME FLOW";
    case M::DebugWindows: return "DEBUG WINDOWS (F1-F3)";
    case M::DriveKeys: return "DRIVE KEYS";
    case M::PadGlyphs: return "GAMEPAD BUTTONS";
    case M::Controls: return "CONTROLS";
    case M::Back: return "BACK";
    case M::VehicleChoice: return "VEHICLE";
    case M::Drive: return "DRIVE";
    }
    return "";
}

const char* menuPageTitle(MenuPage page) {
    switch (page) {
    case MenuPage::Main: return "";
    case MenuPage::Pause: return "PAUSED";
    case MenuPage::Settings: return "SETTINGS";
    case MenuPage::Controls: return "CONTROLS";
    case MenuPage::Vehicle: return "SELECT VEHICLE";
    }
    return "";
}

const char* menuSection(MenuItem item) {
    switch (item) {
    case MenuItem::Quality: return "GRAPHICS";
    case MenuItem::MasterVolume: return "AUDIO";
    case MenuItem::Abs: return "DRIVING AIDS (DEFAULTS)";
    case MenuItem::Minimap: return "MAP";
    case MenuItem::TimeOfDay: return "TIME OF DAY";
    case MenuItem::DebugWindows: return "DEVELOPER";
    default: return nullptr;
    }
}

bool menuAdjustable(MenuItem item) {
    return (int(item) >= int(MenuItem::Quality) && int(item) <= int(MenuItem::PadGlyphs)) || item == MenuItem::VehicleChoice ||
           item == MenuItem::LevelChoice;
}

std::string levelDisplayName(const std::string& id) {
    std::string n = id;
    for (char& c : n) c = c == '_' ? ' ' : char(std::toupper(static_cast<unsigned char>(c)));
    return n;
}

std::string menuValue(MenuItem item, const GameSettings& s, const std::string& defaultLevel) {
    using M = MenuItem;
    switch (item) {
    case M::LevelChoice: return levelDisplayName(s.level.empty() ? defaultLevel : s.level);
    case M::Quality: {
        std::string n = presetName(QualityPreset(std::clamp(s.quality, 0, 3)));
        for (char& c : n) c = char(std::toupper(static_cast<unsigned char>(c)));
        return n;
    }
    case M::RenderScale:
        if (s.renderScale <= 0.0f)
            return "PRESET (" + percent(presetSettings(QualityPreset(std::clamp(s.quality, 0, 3))).renderScale) + ")";
        return percent(s.renderScale);
    case M::GrassDistance: {
        const float v = s.grassDistance > 0.0f ? s.grassDistance : presetSettings(QualityPreset(std::clamp(s.quality, 0, 3))).grassDistance;
        return s.grassDistance > 0.0f ? std::to_string(int(v)) + " M" : "PRESET (" + std::to_string(int(v)) + " M)";
    }
    case M::TreeDistance: {
        const float v = s.treeDistance > 0.0f ? s.treeDistance : presetSettings(QualityPreset(std::clamp(s.quality, 0, 3))).treeDetailDistance;
        return s.treeDistance > 0.0f ? std::to_string(int(v)) + " M" : "PRESET (" + std::to_string(int(v)) + " M)";
    }
    case M::VSync: return s.vsync ? "ON" : "OFF";
    case M::MasterVolume: return percent(s.masterVolume);
    case M::MusicVolume: return percent(s.musicVolume);
    case M::EffectsVolume: return percent(s.effectsVolume);
    case M::UiVolume: return percent(s.uiVolume);
    case M::Abs: return s.abs ? "ON" : "OFF";
    case M::Traction: {
        static const char* kNames[] = {"VEHICLE DEFAULT", "OFF", "ON", "OFF-ROAD"};
        return kNames[std::clamp(s.traction, -1, 2) + 1];
    }
    case M::DiffLocks: {
        static const char* kNames[] = {"OPEN", "REAR", "REAR + CENTER", "ALL"};
        return kNames[std::clamp(s.diffLocks, 0, 3)];
    }
    case M::AutoReverse: return s.autoReverse ? "ON" : "OFF";
    case M::Minimap: return s.minimap ? "ON" : "OFF";
    case M::MinimapSize: {
        static const char* kNames[] = {"SMALL", "MEDIUM", "LARGE"};
        return kNames[std::clamp(s.minimapSize, 0, 2)];
    }
    case M::MinimapNorthUp: return s.minimapNorthUp ? "NORTH UP" : "TURNS WITH VEHICLE";
    case M::Compass: return s.compass ? "ON" : "OFF";
    case M::TimeOfDay: {
        const int minutes = int(std::lround(s.timeOfDay * 60.0f)) % (24 * 60);
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%02d:%02d", minutes / 60, minutes % 60);
        return buf;
    }
    case M::TimePreset: {
        for (const MenuTimePreset& p : kMenuTimePresets)
            if (std::abs(p.hours - s.timeOfDay) < kSameTime) return p.name;
        return "CUSTOM";
    }
    case M::TimeFlow: {
        static const char* kNames[] = {"OFF", "REAL TIME", "FAST (1 H PER MIN)"};
        return kNames[std::clamp(s.timeFlow, 0, 2)];
    }
    case M::DebugWindows: return s.debugWindows ? "ON" : "OFF";
    case M::DriveKeys: return driveKeysName(DriveKeys(std::clamp(s.driveKeys, 0, 1)));
    case M::PadGlyphs: {
        static const char* kNames[] = {"AUTO", "XBOX", "PLAYSTATION"};
        return kNames[std::clamp(s.padGlyphs, 0, 2)];
    }
    default: return {};
    }
}

bool adjustSetting(MenuItem item, GameSettings& s, int dir) {
    using M = MenuItem;
    const GameSettings before = s;
    switch (item) {
    case M::Quality: s.quality = wrap(s.quality + dir, 4); break;
    case M::RenderScale: {
        int best = 0;
        for (int i = 0; i < int(std::size(kScaleSteps)); ++i)
            if (std::abs(kScaleSteps[i] - s.renderScale) < std::abs(kScaleSteps[best] - s.renderScale)) best = i;
        s.renderScale = kScaleSteps[std::clamp(best + dir, 0, int(std::size(kScaleSteps)) - 1)];
        break;
    }
    case M::GrassDistance: s.grassDistance = step(kGrassSteps, int(std::size(kGrassSteps)), s.grassDistance, dir); break;
    case M::TreeDistance: s.treeDistance = step(kTreeSteps, int(std::size(kTreeSteps)), s.treeDistance, dir); break;
    case M::VSync: s.vsync = !s.vsync; break;
    case M::MasterVolume: stepVolume(s.masterVolume, dir); break;
    case M::MusicVolume: stepVolume(s.musicVolume, dir); break;
    case M::EffectsVolume: stepVolume(s.effectsVolume, dir); break;
    case M::UiVolume: stepVolume(s.uiVolume, dir); break;
    case M::Abs: s.abs = !s.abs; break;
    case M::Traction: s.traction = wrap(s.traction + 1 + dir, 4) - 1; break;
    case M::DiffLocks: s.diffLocks = wrap(s.diffLocks + dir, 4); break;
    case M::AutoReverse: s.autoReverse = !s.autoReverse; break;
    case M::Minimap: s.minimap = !s.minimap; break;
    case M::MinimapSize: s.minimapSize = wrap(s.minimapSize + dir, 3); break;
    case M::MinimapNorthUp: s.minimapNorthUp = !s.minimapNorthUp; break;
    case M::Compass: s.compass = !s.compass; break;
    case M::TimeOfDay: {
        // The next half hour in the direction (a clock that ran stops on the grid first).
        const float half = dir > 0 ? std::floor(s.timeOfDay * 2.0f + 0.01f) + 1.0f : std::ceil(s.timeOfDay * 2.0f - 0.01f) - 1.0f;
        s.timeOfDay = float(wrap(int(half), 48)) * 0.5f;
        break;
    }
    case M::TimePreset: {
        constexpr int n = int(std::size(kMenuTimePresets));
        int next = dir > 0 ? 0 : n - 1;
        if (dir > 0) {
            for (int i = n - 1; i >= 0; --i)
                if (kMenuTimePresets[i].hours > s.timeOfDay + kSameTime) next = i;
        } else {
            for (int i = 0; i < n; ++i)
                if (kMenuTimePresets[i].hours < s.timeOfDay - kSameTime) next = i;
        }
        s.timeOfDay = kMenuTimePresets[next].hours;
        break;
    }
    case M::TimeFlow: s.timeFlow = wrap(s.timeFlow + dir, 3); break;
    case M::DebugWindows: s.debugWindows = !s.debugWindows; break;
    case M::DriveKeys: s.driveKeys = wrap(s.driveKeys + dir, 2); break;
    case M::PadGlyphs: s.padGlyphs = wrap(s.padGlyphs + dir, 3); break;
    default: return false;
    }
    return !(s == before);
}

} // namespace tf
