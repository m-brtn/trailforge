#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace tf {

// Every sound the game uses, in manifest id order (assets/audio/manifest.json). The audio thread refers to
// sounds only through this enum; names are resolved once when the manifest is loaded.
enum class SoundId : std::uint8_t {
    EngineExhaustIdle, EngineExhaustOffMid, EngineExhaustOffHigh,
    EngineExhaustOnLow, EngineExhaustOnMid, EngineExhaustOnHigh,
    EngineBayIdle, EngineBayOffMid, EngineBayOffHigh,
    EngineBayOnLow, EngineBayOnMid, EngineBayOnHigh,
    EngineStart, EngineStop,
    GearShift1, GearShift2, GearShift3,
    TireSkidLoop, TireSkidShort, TireGravelLoop, TireDirtLoop, TireMudLoop,
    MudSquelch1, MudSquelch2, MudSquelch3, MudSquelch4,
    WaterSplash1, WaterSplash2, WaterSplash3,
    GrassRustleLoop, GrassSwish1, GrassSwish2, GrassSwish3,
    SuspensionThump, BottomOut1, BottomOut2, BottomOut3,
    BodyImpactMetal1, BodyImpactMetal2, BodyImpactMetal3, BodyImpactClank,
    ChassisCreak1, ChassisCreak2, ChassisCreak3,
    WindLoop, AmbienceDayLoop, AmbienceNightLoop,
    UiClick, UiHover, UiConfirm, UiBack, UiSliderTick, UiMenuOpen, UiMenuClose, UiMenuAmbienceLoop,
    Count
};
inline constexpr std::size_t kSoundCount = std::size_t(SoundId::Count);

// Manifest id of each SoundId.
extern const std::array<const char*, kSoundCount> kSoundNames;

enum class EngineLoad : std::uint8_t { None, Off, Full };
enum class EngineMic : std::uint8_t { None, Exhaust, Bay };

struct SoundInfo {
    std::string id;
    std::string category;
    std::string file;      // relative to the manifest directory
    bool loop = false;
    int channels = 1;
    double durationS = 0.0;
    std::string license;
    // Engine loops only (manifest "engine" block).
    float rpmMeasured = 0.0f;         // rpm measured from the firing frequency of the recording
    EngineLoad load = EngineLoad::None;
    EngineMic mic = EngineMic::None;
    float sourceLevelDb = 0.0f;       // recorded level relative to the full-load high-rpm take (files are normalized)
};

struct AudioManifest {
    int sampleRate = 48000;
    std::array<SoundInfo, kSoundCount> sounds; // indexed by SoundId; id empty if missing in the file
    std::vector<std::string> unknownIds;       // ids in the file that the game does not use

    const SoundInfo& operator[](SoundId s) const { return sounds[std::size_t(s)]; }
    int missingCount() const;
};

// Reads manifest.json. Returns false (with a message) if the file cannot be read or parsed.
bool loadAudioManifest(const std::string& path, AudioManifest& out, std::string* error);

} // namespace tf
