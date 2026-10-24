#pragma once

#include <glm/glm.hpp>

namespace tf {

// Lighting and sky state for a time of day (render-only; nothing here touches the simulation).
// The sun follows a fixed latitude/declination path; `headingDeg` rotates the compass so the sunset can be
// placed in a chosen world direction. The moon is the shadow-casting light at night.
struct SkyState {
    glm::vec3 sunDir{0, 1, 0};      // toward the sun (can be below the horizon)
    glm::vec3 moonDir{0, 1, 0};     // toward the moon
    glm::vec3 lightDir{0, 1, 0};    // shadow-casting light: the sun by day, the moon at night (never below the horizon)
    glm::vec3 lightColor{0.0f};     // irradiance of that light (HDR)
    glm::vec3 sunGlow{0.0f};        // sky glow / in-scatter color around the sun (also when it is just below the horizon)
    glm::vec3 sunDisc{0.0f};        // radiance of the sun disc (0 below the horizon)
    float sunDiscCos = 0.9998f;     // cos of the disc angular radius (larger near the horizon, stylized)
    glm::vec3 zenith{0.0f}, horizon{0.0f};
    glm::vec3 fogColor{0.0f};
    glm::vec3 skyAmbient{0.0f};     // diffuse irradiance from the sky hemisphere (normal pointing up)
    glm::vec3 groundAmbient{0.0f};  // bounce light from below
    float fogMul = 1.0f;            // multiplier on the base fog density
    float exposure = 1.0f;          // eye adaptation multiplier on the user exposure
    float bloomMul = 1.0f;
    float stars = 0.0f;             // 0..1
    float moonDisc = 0.0f;          // brightness of the moon disc
    float sunElevationDeg = 0.0f;
    float sunAzimuthDeg = 0.0f;     // world azimuth: 0 = toward +Z, 90 = toward +X
    float night = 0.0f;             // 0 by day, 1 at full night (used for headlight glare and cloud tint)
};

// hours: 0..24 local solar time; headingDeg: world azimuth offset of the compass.
SkyState computeSky(float hours, float headingDeg);

struct TimePreset {
    const char* name;
    float hours;
};
inline constexpr TimePreset kTimePresets[] = {{"Dawn", 5.75f}, {"Noon", 12.0f}, {"Sunset", 18.2f}, {"Night", 23.5f}};

} // namespace tf
