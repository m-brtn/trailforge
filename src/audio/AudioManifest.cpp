#include "audio/AudioManifest.h"

#include "core/Json.h"

#include <fstream>
#include <sstream>

namespace tf {

const std::array<const char*, kSoundCount> kSoundNames = {
    "engine_exhaust_idle_loop", "engine_exhaust_offload_mid_loop", "engine_exhaust_offload_high_loop",
    "engine_exhaust_onload_low_loop", "engine_exhaust_onload_mid_loop", "engine_exhaust_onload_high_loop",
    "engine_bay_idle_loop", "engine_bay_offload_mid_loop", "engine_bay_offload_high_loop",
    "engine_bay_onload_low_loop", "engine_bay_onload_mid_loop", "engine_bay_onload_high_loop",
    "engine_start", "engine_stop",
    "gear_shift_01", "gear_shift_02", "gear_shift_03",
    "tire_skid_asphalt_loop", "tire_skid_short", "tire_gravel_roll_loop", "tire_dirt_roll_loop", "tire_mud_slosh_loop",
    "mud_squelch_01", "mud_squelch_02", "mud_squelch_03", "mud_squelch_04",
    "water_splash_01", "water_splash_02", "water_splash_03",
    "grass_rustle_loop", "grass_swish_01", "grass_swish_02", "grass_swish_03",
    "suspension_thump_01", "suspension_bottom_out_01", "suspension_bottom_out_02", "suspension_bottom_out_03",
    "body_impact_metal_01", "body_impact_metal_02", "body_impact_metal_03", "body_impact_clank",
    "chassis_creak_01", "chassis_creak_02", "chassis_creak_03",
    "wind_speed_loop", "ambience_day_meadow_loop", "ambience_night_crickets_loop",
    "ui_click", "ui_hover", "ui_confirm", "ui_back", "ui_slider_tick", "ui_menu_open", "ui_menu_close",
    "ui_menu_ambience_loop",
};

int AudioManifest::missingCount() const {
    int n = 0;
    for (const SoundInfo& s : sounds) n += s.id.empty() ? 1 : 0;
    return n;
}

bool loadAudioManifest(const std::string& path, AudioManifest& out, std::string* error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (error) *error = "cannot open " + path;
        return false;
    }
    std::stringstream ss;
    ss << in.rdbuf();
    json::Value doc;
    std::string err;
    if (!json::parse(ss.str(), doc, &err)) {
        if (error) *error = path + ": " + err;
        return false;
    }
    out = AudioManifest{};
    out.sampleRate = int(doc["format"]["sample_rate"].numberOr(48000));
    for (const json::Value& s : doc["sounds"].array) {
        const std::string id = s["id"].stringOr("");
        std::size_t index = kSoundCount;
        for (std::size_t i = 0; i < kSoundCount; ++i)
            if (id == kSoundNames[i]) index = i;
        if (index == kSoundCount) {
            out.unknownIds.push_back(id);
            continue;
        }
        SoundInfo& info = out.sounds[index];
        info.id = id;
        info.category = s["category"].stringOr("");
        info.file = s["file"].stringOr("");
        info.loop = s["loop"].boolOr(false);
        info.channels = int(s["channels"].numberOr(1));
        info.durationS = s["duration_s"].numberOr(0.0);
        info.license = s["license"].stringOr("");
        const json::Value& e = s["engine"];
        if (!e.isNull()) {
            info.rpmMeasured = float(e["rpm_measured"].numberOr(0.0));
            const std::string load = e["load"].stringOr("");
            info.load = load == "off" ? EngineLoad::Off : load == "full" ? EngineLoad::Full : EngineLoad::None;
            const std::string mic = e["mic"].stringOr("");
            info.mic = mic.rfind("exhaust", 0) == 0 ? EngineMic::Exhaust
                       : mic.rfind("engine bay", 0) == 0 ? EngineMic::Bay
                                                          : EngineMic::None;
            info.sourceLevelDb = float(e["source_level_vs_onload_high_db"].numberOr(0.0));
        }
    }
    return true;
}

} // namespace tf
