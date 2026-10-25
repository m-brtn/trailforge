#pragma once

#include <string>

namespace tf {

// How a vehicle's tires sound when they slide on hard ground (sound only, never physics). Selected per vehicle by
// `audioProfile` in vehicle.json; the optional `tireSound` object overrides single values. Both profiles play the
// same recorded squeal (tire_skid_asphalt_loop / tire_skid_short, a passenger car tire); the off-road profile plays
// it lower and duller and adds a synthesized scrub: band-limited noise, amplitude-modulated at the rate the tread
// blocks pass the contact patch (sliding speed / treadPitch), for big lugged tires that rumble more than they sing.
struct TireSoundProfile {
    float squealPitch = 1.0f;      // playback rate factor of the squeal recordings
    float squealGain = 1.0f;       // level factor of the squeal recordings
    float lowpassMinHz = 2500.0f;  // squeal low-pass cutoff at a faint squeal ...
    float lowpassMaxHz = 12000.0f; // ... and at full squeal
    float scrubGain = 0.0f;        // synthesized scrub level (0 = off)
    float scrubLowHz = 120.0f;     // scrub noise band (one-pole high-pass, two-pole low-pass)
    float scrubHighHz = 1400.0f;
    float treadPitch = 0.07f;      // m between tread blocks: scrub modulation rate = sliding speed / treadPitch
    // Rolling road hum on hard ground (synthesized, 0 = off): low band noise modulated at the tread block rate of the
    // rolling speed. Only the quiet electric profile uses it so far; with an engine it would be masked anyway.
    float roadGain = 0.0f;
    bool operator==(const TireSoundProfile&) const = default;
};

// Profile ids: "sporty" (the recordings as they are, for a road tire), "trailforge_suv" (mud-terrain tires: lower
// and duller squeal plus scrub) and "ev" (all-terrain tires, below). Returns false for an unknown id (out unchanged).
inline bool tireSoundPreset(const std::string& id, TireSoundProfile& out) {
    if (id == "sporty") {
        out = TireSoundProfile{};
        return true;
    }
    if (id == "trailforge_suv") {
        out = TireSoundProfile{};
        out.squealPitch = 0.78f;
        out.squealGain = 0.7f;
        out.lowpassMinHz = 1100.0f;
        out.lowpassMaxHz = 3200.0f;
        out.scrubGain = 1.0f;
        out.scrubLowHz = 120.0f;
        out.scrubHighHz = 900.0f;
        out.treadPitch = 0.07f;
        return true;
    }
    // Electric SUV on 285/70R17 all-terrain tires: like trailforge_suv, with a finer tread (shorter block pitch, a
    // little brighter scrub), a slightly higher squeal and the rolling road hum (no engine masks it).
    if (id == "ev") {
        tireSoundPreset("trailforge_suv", out);
        out.squealPitch = 0.84f;
        out.squealGain = 0.75f;
        out.lowpassMaxHz = 3800.0f;
        out.scrubGain = 0.85f;
        out.scrubHighHz = 1100.0f;
        out.treadPitch = 0.055f;
        out.roadGain = 1.0f;
        return true;
    }
    return false;
}

inline TireSoundProfile tireSoundPreset(const std::string& id) {
    TireSoundProfile p;
    tireSoundPreset(id, p);
    return p;
}

} // namespace tf
