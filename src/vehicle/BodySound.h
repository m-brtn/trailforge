#pragma once

#include <array>
#include <string>

namespace tf {

// How the vehicle body sounds when it is shaken (sound only, never physics): the synthesized body rattle after a hard
// landing or bump and the thud of a suspension reaching its bump stop or the body coming down on its springs.
// Selected per vehicle by `bodySound` in vehicle.json (default: from `audioProfile`, `sporty` -> "coupe", anything
// else -> "suv").
//
// Thud: one low mode that falls by 20 dB within thudCycles periods (no ringing tone) and a noise hit low-passed
// between thudDullHz (light hit) and thudBrightHz (full hit). Rattle: sparse noise ticks, each band-passed (low Q)
// around its own random frequency in [rattleLowHz, rattleHighHz], at most one per tickGap; the tick rate follows a
// shake envelope that the feed's events raise and that decays over rattleDecay.
struct BodySoundProfile {
    float thudHz = 130.0f;       // low body mode
    float thudCycles = 2.0f;     // periods to -20 dB
    float thudDullHz = 500.0f;   // noise hit low-pass at intensity 0 ...
    float thudBrightHz = 2600.0f; // ... and at intensity 1
    float thudNoiseDecay = 0.010f; // s, time constant of the noise hit
    float thudGain = 1.0f;
    float rattleLowHz = 450.0f, rattleHighHz = 2600.0f; // band of the tick centers (log-uniform)
    float rattleQ = 1.2f;
    float tickDecay = 0.0025f;   // s, time constant of one tick
    float tickGap = 0.018f;      // s, shortest interval between ticks
    float rattleRate = 28.0f;    // ticks per second at full shake (before the gap)
    float rattleDecay = 0.12f;   // s, shake envelope time constant at full intensity (shorter for light hits)
    float rattleGain = 1.0f;
    bool operator==(const BodySoundProfile&) const = default;
};

// Preset ids: "suv" (steel panels, doors and cargo: low thud, mid-band rattle), "tube_frame" (open tube-frame buggy:
// lighter, higher thud, sparser and brighter ticks) and "coupe" (stiff closed body: little rattle, tight thud).
inline bool bodySoundPreset(const std::string& id, BodySoundProfile& out) {
    if (id == "suv") {
        out = BodySoundProfile{};
        return true;
    }
    if (id == "tube_frame") {
        out = BodySoundProfile{};
        out.thudHz = 170.0f;
        out.thudDullHz = 800.0f;
        out.thudBrightHz = 4000.0f;
        out.thudNoiseDecay = 0.008f;
        out.rattleLowHz = 1200.0f;
        out.rattleHighHz = 6000.0f;
        out.rattleQ = 1.5f;
        out.rattleRate = 22.0f;
        out.rattleDecay = 0.10f;
        out.rattleGain = 0.85f;
        return true;
    }
    if (id == "coupe") {
        out = BodySoundProfile{};
        out.thudHz = 150.0f;
        out.thudDullHz = 550.0f;
        out.thudBrightHz = 2800.0f;
        out.thudNoiseDecay = 0.007f;
        out.thudGain = 0.8f;
        out.rattleLowHz = 700.0f;
        out.rattleHighHz = 3500.0f;
        out.rattleRate = 14.0f;
        out.rattleDecay = 0.09f;
        out.rattleGain = 0.7f;
        return true;
    }
    return false;
}

inline const char* defaultBodySound(const std::string& audioProfile) {
    return audioProfile == "sporty" ? "coupe" : "suv";
}

inline BodySoundProfile bodySoundPreset(const std::string& id) {
    BodySoundProfile p;
    bodySoundPreset(id, p);
    return p;
}

} // namespace tf
