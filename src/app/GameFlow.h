#pragma once

#include "app/GameSettings.h"
#include "sim/FixedStepClock.h"

#include <string>
#include <vector>

namespace tf {

// Top-level state of the game. Pure C++ (no SDL, no ImGui): the Application feeds it menu input and acts on the
// returned commands; the menu renderer only reads it.
// Garage: opened from the start or the pause menu; the simulation runs (the vehicle can be rebuilt in place) and
// closing it returns to the menu it came from.
// Map: the full-screen map, opened in gameplay (M / pad LB) or from the pause menu; the simulation is frozen and
// closing it returns to where it was opened.
enum class Screen { Intro, MainMenu, Playing, Paused, Garage, Map };

// Menu pages. Start opens the vehicle page when the catalog has more than one vehicle (else it starts directly).
// The level is a row of the start menu (applied at once, so the menu backdrop is the chosen world).
enum class MenuPage { Main, Pause, Settings, Controls, Vehicle };

enum class MenuItem {
    Start, LevelChoice, Garage, Settings, Quit,  // main
    Resume, Map, Restart, QuitToMenu, QuitGame,  // pause (+ Garage, Settings)
    Quality, RenderScale, GrassDistance, TreeDistance, VSync, TreeStyle, // settings: graphics
    MasterVolume, MusicVolume, EffectsVolume, UiVolume, // audio
    Abs, Traction, DiffLocks, AutoReverse,       // driving aid defaults
    Minimap, MinimapSize, MinimapNorthUp, Compass, // map
    TimeOfDay, TimePreset, TimeFlow,             // time of day (gameplay)
    DebugWindows,                                // developer
    DriveKeys, PadGlyphs,                        // controls page
    Controls, Back,
    VehicleChoice, Drive,                        // vehicle page
};

enum class MenuCommand {
    None, StartGame, Resume, Restart, QuitToMenu, QuitGame, SettingsChanged, SettingsClosed, OpenGarage, CloseGarage,
    VehicleChanged, // GameSettings::vehicle changed on the vehicle page
    LevelChanged,   // GameSettings::level changed on the start menu
    OpenMap         // the full-screen map opened from the pause menu
};
enum class MenuSound { None, Move, Confirm, Back, Adjust, Open, Close };

// One frame of menu input (keyboard, mouse and gamepad already merged).
struct MenuInput {
    bool up = false, down = false, left = false, right = false;
    bool accept = false, back = false;
    bool pause = false;  // Esc / gamepad Start
    bool any = false;    // any key, button or click (skips the intro)
    int hover = -1;      // row index under a moving mouse, -1 = none
    int click = -1;      // row index clicked
    int clickDir = 0;    // -1 / +1: a value arrow was clicked, 0: the row itself
    int pick = -1;       // vehicle page: vehicle card clicked (catalog index)
    int wheel = 0;       // vehicle page: mouse wheel steps (+ = previous vehicle, like scrolling up a list)
};

struct MenuResult {
    MenuCommand command = MenuCommand::None;
    MenuSound sound = MenuSound::None;
};

class GameFlow {
public:
    // Intro timeline (s): the logo fades in, holds, then the start menu fades in over the live scene.
    static constexpr double kLogoFadeIn = 1.2;
    static constexpr double kIntroLength = 4.6;
    static constexpr double kMenuFadeIn = 1.8;
    static constexpr double kSkipFade = 0.5;

    explicit GameFlow(Screen start = Screen::Intro);

    Screen screen() const { return m_screen; }
    MenuPage page() const { return m_stack.back().page; }
    int depth() const { return int(m_stack.size()); }
    int cursor() const { return m_stack.back().cursor; }
    MenuItem current() const { return items()[std::size_t(cursor())]; }
    const std::vector<MenuItem>& items() const { return items(page()); }
    static const std::vector<MenuItem>& items(MenuPage p);

    // A menu (or the intro) owns the input; gameplay controls are ignored.
    bool menuActive() const { return m_screen != Screen::Playing; }
    // The pause menu freezes the simulation: no ticks, the fixed-step accumulator keeps its value. The garage runs it,
    // also when opened from the pause menu.
    bool simulationRuns() const { return m_screen != Screen::Paused && m_screen != Screen::Map; }
    // Physics ticks for this frame: 0 (accumulator untouched) unless the simulation runs.
    int stepsForFrame(FixedStepClock& clock, double frameTime) const {
        return simulationRuns() ? clock.advance(frameTime) : 0;
    }

    double screenTime() const { return m_screenTime; } // s since the last screen change
    double introTime() const { return m_introTime; }
    // 0..1 opacity of the start menu (fades in after the intro or a skip); 1 when not coming from the intro.
    float menuFade() const;

    void update(double dt);
    MenuResult handle(const MenuInput& in, GameSettings& settings);

    // Vehicle catalog ids in display order (the vehicle page cycles through them).
    void setVehicles(std::vector<std::string> ids) { m_vehicleIds = std::move(ids); }
    const std::vector<std::string>& vehicles() const { return m_vehicleIds; }
    // Catalog index of settings.vehicle; 0 (the default vehicle) when it is empty or unknown.
    int vehicleIndex(const GameSettings& settings) const;
    void openVehiclePage(); // from the start menu (scenario command; Start does it through the input)
    // Level ids in display order (the LEVEL row cycles through them); settings.level empty = the first one.
    void setLevels(std::vector<std::string> ids) { m_levelIds = std::move(ids); }
    const std::vector<std::string>& levels() const { return m_levelIds; }
    int levelIndex(const GameSettings& settings) const;

    // Direct transitions (scenario commands and the application).
    void skipIntro();
    void startGame();
    void pause();
    void resume();
    void toMainMenu();
    void openSettings();
    void openGarage();  // from the start or the pause menu
    void closeGarage(); // back to that menu, cursor on GARAGE
    Screen garageReturn() const { return m_garageReturn; }
    void openMap();  // from gameplay or the pause menu
    void closeMap(); // back to gameplay, or to the pause menu with the cursor on MAP
    Screen mapReturn() const { return m_mapReturn; }
    bool back(); // pops a page; false at a root page
    bool select(MenuItem item); // moves the cursor to the item on the current page

private:
    struct PageState {
        MenuPage page;
        int cursor = 0;
    };
    void setScreen(Screen s, MenuPage root);
    MenuResult activate(MenuItem item, GameSettings& settings, int dir);
    MenuResult goBack();
    MenuResult pickVehicle(int index, GameSettings& settings);
    MenuResult pickLevel(int index, GameSettings& settings);

    Screen m_screen;
    std::vector<PageState> m_stack;
    double m_screenTime = 0.0;
    double m_introTime = 0.0;
    double m_menuFadeLength = 0.0; // > 0 while the start menu fades in after the intro
    Screen m_garageReturn = Screen::MainMenu;
    Screen m_mapReturn = Screen::Playing;
    std::vector<std::string> m_vehicleIds;
    std::vector<std::string> m_levelIds;
};

const char* menuLabel(MenuItem item);
const char* menuPageTitle(MenuPage page);
// Section heading drawn above this row of the Settings page, or nullptr.
const char* menuSection(MenuItem item);
// Rows with a value (left / right change it).
bool menuAdjustable(MenuItem item);
// Current value as shown in the menu ("ON", "HIGH", "80 %"); empty for plain actions. The level row shows the id in
// capitals with spaces ("EXPLORATION PEAKS"); an empty level shows `defaultLevel` that way.
std::string menuValue(MenuItem item, const GameSettings& s, const std::string& defaultLevel = {});
std::string levelDisplayName(const std::string& id);
// Changes the value one step (dir -1 / +1); returns false if nothing changed. TIME moves to the next half hour
// (wrapping at midnight), PRESET to the next or previous preset after the current time.
bool adjustSetting(MenuItem item, GameSettings& s, int dir);

// Time-of-day presets of the PRESET row (hours; Dawn, Noon, Sunset and Night are the kTimePresets of TimeOfDay.h).
struct MenuTimePreset {
    const char* name;
    float hours;
};
inline constexpr MenuTimePreset kMenuTimePresets[] = {{"DAWN", 5.75f},        {"MORNING", 9.0f}, {"NOON", 12.0f},
                                                      {"GOLDEN HOUR", 17.5f}, {"SUNSET", 18.2f}, {"NIGHT", 23.5f}};
// Game hours per real second of each TimeFlow value (off, real time, fast).
inline constexpr float kTimeFlowRates[] = {0.0f, 1.0f / 3600.0f, 1.0f / 60.0f};

} // namespace tf
