#pragma once

#include <string>

#if defined(__APPLE__)
#include <TargetConditionals.h>
#endif

namespace tf {

#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
constexpr int kDefaultQuality = 0; // iOS: Low until the port has been measured on a real device
#else
constexpr int kDefaultQuality = 2; // High
#endif

// Player settings edited in the Settings screen and stored as `settings.cfg` in the user data directory.
// Plain values only (no SDL, no renderer), so the file format is testable headless.
struct GameSettings {
    // Graphics
    int quality = kDefaultQuality; // QualityPreset Low (0) .. Ultra (3)
    float renderScale = 0.0f;  // 0: the preset's own scale, otherwise 0.5..1 of the window pixels
    float grassDistance = 0.0f; // m; 0: the preset's own, otherwise 20..250
    float treeDistance = 0.0f;  // m, tree detail (mesh LOD) distance; 0: the preset's own, otherwise 150..2000
    bool vsync = true;
    // Level trees: true = the procedural tiered firs (the default look), false = the CC0 tree models. Applied when a
    // level is baked (a change re-bakes the current level).
    bool stylizedTrees = true;
    // Audio (linear gains 0..1)
    float masterVolume = 0.8f;
    float musicVolume = 0.8f;
    float effectsVolume = 1.0f; // vehicle, surfaces, impacts, ambience
    float uiVolume = 1.0f;
    // Driving aid defaults (applied at start and when changed; B / T / X still toggle them while driving)
    bool abs = false;
    int traction = -1;         // -1: the vehicle's own default (vehicle.json aids.traction), else TractionMode 0 Off,
                               // 1 On, 2 Off-road
    int diffLocks = 0;         // 0 open, 1 rear, 2 rear + center, 3 all
    bool autoReverse = true;
    // Map: corner minimap (size 0 small, 1 medium, 2 large; turns with the vehicle unless north up), compass bar
    bool minimap = true;
    int minimapSize = 1;
    bool minimapNorthUp = false;
    bool compass = true;
    // Time of day in gameplay (render-only): local solar hours 0..24 (the live clock, also after it ran), and
    // whether the clock runs while driving (TimeFlow: 0 off, 1 real time, 2 fast = one game hour per real minute)
    float timeOfDay = 9.0f;
    int timeFlow = 0;
    // Developer windows: the top-left read-out, F1 telemetry + tuning, F2 graphics, F3 audio and the physics debug
    // lines. Off: only the player HUD (speed / gear, lap and drift panels, minimap, compass) and F1-F3 do nothing.
    bool debugWindows = false;
    // Controls
    int driveKeys = 0;         // DriveKeys: 0 W A S D, 1 arrows (the other cluster moves the free camera)
    int padGlyphs = 0;         // PadGlyphSetting: 0 auto (from the connected pad), 1 Xbox, 2 PlayStation
    // Vehicle chosen last on the vehicle page (catalog id, [a-z0-9_] only); empty = the default vehicle
    std::string vehicle;
    // Level chosen on the start menu ("proving_ground" or an assets/levels id, [a-z0-9_]); empty = the default level
    std::string level;

    bool operator==(const GameSettings&) const = default;
};

// Clamps every field into its valid range.
GameSettings sanitized(GameSettings s);

// `key = value` lines; '#' starts a comment. Unknown keys and malformed lines are skipped (counted in *skipped),
// missing keys keep their defaults, values are clamped.
std::string serializeSettings(const GameSettings& s);
GameSettings parseSettings(const std::string& text, int* skipped = nullptr);

// Writes through a temporary file and a rename, so a crash never leaves a half-written file.
bool saveSettingsFile(const std::string& path, const GameSettings& s, std::string* error = nullptr);
// false if the file does not exist or cannot be read (out keeps its value then).
bool loadSettingsFile(const std::string& path, GameSettings& out, int* skipped = nullptr);

} // namespace tf
