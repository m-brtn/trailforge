#pragma once

#include <array>
#include <string>

namespace tf {

// How the vehicle body sounds when it is shaken (sound only, never physics): the synthesized body rattle after a hard
// landing or bump and the knock of a suspension reaching its bump stop. Selected per vehicle by `bodySound` in
// vehicle.json (default: from `audioProfile`, `sporty` -> "coupe", anything else -> "suv").
//
// Rattle: sparse random grains, each a short decaying sine on one of four body resonances (panels, brackets, loose
// items), plus a broadband click per grain; the grain rate follows a shake envelope that the feed's BodyRattle events
// raise and that decays over rattleDecay. Knock: three decaying modes and a short noise transient at the wheel.
struct BodySoundProfile {
    std::array<float, 4> rattleHz{520.0f, 870.0f, 1350.0f, 2300.0f}; // grain resonances
    float rattleRing = 0.018f;  // s, decay time constant of one grain
    float rattleRate = 140.0f;  // grains per second at full shake
    float rattleDecay = 0.22f;  // s, shake envelope time constant at full intensity (shorter for light hits)
    float rattleGain = 1.0f;
    float clickGain = 0.35f;    // broadband click per grain, relative to its resonance
    std::array<float, 3> knockHz{210.0f, 540.0f, 1250.0f};
    float knockDecay = 0.07f;   // s, time constant of the knock modes
    float knockGain = 1.0f;
    bool operator==(const BodySoundProfile&) const = default;
};

// Preset ids: "suv" (steel panels, doors and cargo: dense mid-band rattle), "tube_frame" (open tube-frame buggy:
// sparser, higher and longer-ringing metal) and "coupe" (stiff closed body: little rattle, tight knock).
inline bool bodySoundPreset(const std::string& id, BodySoundProfile& out) {
    if (id == "suv") {
        out = BodySoundProfile{};
        return true;
    }
    if (id == "tube_frame") {
        out = BodySoundProfile{};
        out.rattleHz = {950.0f, 1650.0f, 2750.0f, 4300.0f};
        out.rattleRing = 0.035f;
        out.rattleRate = 90.0f;
        out.rattleDecay = 0.20f;
        out.rattleGain = 0.85f;
        out.clickGain = 0.5f;
        out.knockHz = {380.0f, 1020.0f, 2350.0f};
        out.knockDecay = 0.10f;
        out.knockGain = 0.8f;
        return true;
    }
    if (id == "coupe") {
        out = BodySoundProfile{};
        out.rattleHz = {680.0f, 1150.0f, 1900.0f, 3100.0f};
        out.rattleRing = 0.012f;
        out.rattleRate = 80.0f;
        out.rattleDecay = 0.16f;
        out.rattleGain = 0.8f;
        out.clickGain = 0.3f;
        out.knockHz = {260.0f, 700.0f, 1600.0f};
        out.knockDecay = 0.05f;
        out.knockGain = 0.8f;
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
