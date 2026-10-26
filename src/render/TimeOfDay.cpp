#include "render/TimeOfDay.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace tf {
namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kLatitude = 45.0f;     // deg
constexpr float kDeclination = 10.0f;  // deg (late summer): sunrise ~5:20, sunset ~18:40, noon elevation 55 deg

float rad(float d) { return d * kPi / 180.0f; }
float smooth(float e0, float e1, float x) {
    const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Direction toward a body on the sky for local hour angle (hours from its transit) and declination.
glm::vec3 skyDirection(float hourAngleHours, float declDeg, float headingDeg, float& elevationDeg, float& worldAzDeg) {
    const float phi = rad(kLatitude), dec = rad(declDeg), h = rad(hourAngleHours * 15.0f);
    // Horizontal coordinates: east, north, up.
    const float east = -std::cos(dec) * std::sin(h);
    const float north = std::cos(phi) * std::sin(dec) - std::sin(phi) * std::cos(dec) * std::cos(h);
    const float up = std::sin(phi) * std::sin(dec) + std::cos(phi) * std::cos(dec) * std::cos(h);
    elevationDeg = std::asin(std::clamp(up, -1.0f, 1.0f)) * 180.0f / kPi;
    const float compassAz = std::atan2(east, north) * 180.0f / kPi; // 0 = north, 90 = east
    worldAzDeg = std::fmod(compassAz + headingDeg + 720.0f, 360.0f);
    const float az = rad(worldAzDeg), horiz = std::sqrt(std::max(0.0f, 1.0f - up * up));
    return glm::normalize(glm::vec3(horiz * std::sin(az), up, horiz * std::cos(az)));
}

// Palette keyed by sun elevation (deg). Linear HDR values; tuned by eye against the stylized banner look.
struct Key {
    float el;
    glm::vec3 sun, glow, disc, zenith, horizon, fog, ambient;
    float exposure, fogMul;
};

const Key kKeys[] = {
    {-18.0f, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0.0030f, 0.0050f, 0.0130f}, {0.0080f, 0.0115f, 0.0230f},
     {0.0080f, 0.0110f, 0.0210f}, {0.012f, 0.016f, 0.032f}, 2.7f, 1.0f},
    {-9.0f, {0, 0, 0}, {0.05f, 0.02f, 0.02f}, {0, 0, 0}, {0.0070f, 0.0110f, 0.0300f}, {0.040f, 0.032f, 0.050f},
     {0.032f, 0.028f, 0.045f}, {0.020f, 0.022f, 0.040f}, 2.4f, 1.1f},
    {-4.0f, {0, 0, 0}, {0.70f, 0.20f, 0.07f}, {0, 0, 0}, {0.040f, 0.040f, 0.110f}, {0.42f, 0.15f, 0.08f},
     {0.26f, 0.12f, 0.09f}, {0.075f, 0.060f, 0.085f}, 1.75f, 1.05f},
    {0.0f, {1.30f, 0.36f, 0.07f}, {1.60f, 0.48f, 0.10f}, {3.2f, 1.00f, 0.18f}, {0.200f, 0.110f, 0.200f},
     {1.40f, 0.48f, 0.12f}, {0.95f, 0.40f, 0.16f}, {0.26f, 0.16f, 0.14f}, 1.30f, 1.0f},
    {5.0f, {3.20f, 1.30f, 0.38f}, {2.60f, 1.05f, 0.26f}, {3.6f, 1.55f, 0.36f}, {0.340f, 0.200f, 0.260f},
     {1.85f, 0.78f, 0.22f}, {1.25f, 0.62f, 0.26f}, {0.40f, 0.26f, 0.22f}, 1.10f, 0.95f},
    {12.0f, {4.40f, 3.10f, 1.85f}, {2.40f, 1.70f, 1.00f}, {9.0f, 7.0f, 4.0f}, {0.200f, 0.300f, 0.640f},
     {1.05f, 0.80f, 0.58f}, {0.85f, 0.74f, 0.62f}, {0.28f, 0.30f, 0.38f}, 1.05f, 0.95f},
    {25.0f, {5.00f, 4.40f, 3.70f}, {2.60f, 2.40f, 2.10f}, {40.0f, 37.0f, 32.0f}, {0.170f, 0.420f, 1.100f},
     {0.68f, 0.86f, 1.12f}, {0.66f, 0.82f, 1.05f}, {0.21f, 0.34f, 0.62f}, 1.0f, 1.0f},
    {60.0f, {5.40f, 5.13f, 4.75f}, {2.60f, 2.50f, 2.30f}, {45.0f, 43.0f, 40.0f}, {0.150f, 0.380f, 1.150f},
     {0.62f, 0.80f, 1.12f}, {0.60f, 0.76f, 1.05f}, {0.21f, 0.35f, 0.66f}, 1.0f, 1.0f},
};

Key sample(float el) {
    const int n = int(std::size(kKeys));
    if (el <= kKeys[0].el) return kKeys[0];
    if (el >= kKeys[n - 1].el) return kKeys[n - 1];
    int i = 0;
    while (el > kKeys[i + 1].el) ++i;
    const Key& a = kKeys[i];
    const Key& b = kKeys[i + 1];
    const float t = smooth(a.el, b.el, el); // smooth between keys: no visible kinks while the time-lapse runs
    auto mix = [t](const glm::vec3& x, const glm::vec3& y) { return x + (y - x) * t; };
    Key k;
    k.el = el;
    k.sun = mix(a.sun, b.sun);
    k.glow = mix(a.glow, b.glow);
    k.disc = mix(a.disc, b.disc);
    k.zenith = mix(a.zenith, b.zenith);
    k.horizon = mix(a.horizon, b.horizon);
    k.fog = mix(a.fog, b.fog);
    k.ambient = mix(a.ambient, b.ambient);
    k.exposure = a.exposure + (b.exposure - a.exposure) * t;
    k.fogMul = a.fogMul + (b.fogMul - a.fogMul) * t;
    return k;
}

float luminance(const glm::vec3& c) { return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b; }

} // namespace

SkyState computeSky(float hours, float headingDeg) {
    SkyState s;
    hours = std::fmod(std::fmod(hours, 24.0f) + 24.0f, 24.0f);
    float sunEl = 0.0f, sunAz = 0.0f, moonEl = 0.0f, moonAz = 0.0f;
    s.sunDir = skyDirection(hours - 12.0f, kDeclination, headingDeg, sunEl, sunAz);
    // Full moon: opposite the sun in hour angle, with a lower path.
    s.moonDir = skyDirection(hours - 24.0f, -kDeclination + 4.0f, headingDeg + 8.0f, moonEl, moonAz);
    s.sunElevationDeg = sunEl;
    s.sunAzimuthDeg = sunAz;

    Key k = sample(sunEl);
    // Morning: cooler, pinker horizon and some extra haze (mist) while the sun is low.
    const float morning = (hours < 12.0f ? 1.0f : 0.0f) * (1.0f - smooth(4.0f, 20.0f, sunEl)) * smooth(-12.0f, -2.0f, sunEl);
    k.horizon *= glm::mix(glm::vec3(1.0f), glm::vec3(0.86f, 0.86f, 1.18f), morning);
    k.fog *= glm::mix(glm::vec3(1.0f), glm::vec3(0.92f, 0.90f, 1.12f), morning);
    k.glow *= glm::mix(glm::vec3(1.0f), glm::vec3(1.0f, 0.85f, 0.95f), morning);
    k.fogMul *= 1.0f + 0.5f * morning;

    s.zenith = k.zenith;
    s.horizon = k.horizon;
    s.fogColor = k.fog;
    s.sunGlow = k.glow;
    s.fogMul = k.fogMul;
    s.exposure = k.exposure;
    s.night = 1.0f - smooth(-12.0f, -2.0f, sunEl);
    s.stars = 1.0f - smooth(-14.0f, -5.0f, sunEl);
    s.bloomMul = 1.0f + 0.6f * s.night;

    // Sun disc: stylized, larger near the horizon (angular radius 1.0 deg high up, 2.6 deg at the horizon).
    const float discRadius = 1.0f + 1.6f * (1.0f - smooth(0.0f, 20.0f, sunEl));
    s.sunDiscCos = std::cos(rad(discRadius));
    s.sunDisc = k.disc * smooth(-discRadius, 0.0f, sunEl);

    // The shadow light switches from the sun to the moon while both are dark (sun 0..-4 deg below the horizon),
    // so the swap is invisible and nothing is ever lit or shadowed from below the horizon.
    const float moonUp = smooth(0.0f, 10.0f, moonEl);
    const float moonLight = (1.0f - smooth(-10.0f, -4.0f, sunEl)) * moonUp;
    const glm::vec3 moonColor = glm::vec3(0.50f, 0.62f, 0.95f) * 0.34f;
    if (sunEl > -4.0f) {
        s.lightDir = s.sunDir;
        s.lightColor = k.sun * smooth(-0.5f, 0.5f, sunEl);
    } else {
        s.lightDir = s.moonDir;
        s.lightColor = moonColor * moonLight;
    }
    if (s.lightDir.y < 0.02f) {
        s.lightDir.y = 0.02f;
        s.lightDir = glm::normalize(s.lightDir);
    }
    s.moonDisc = moonUp * (1.0f - smooth(-6.0f, 2.0f, sunEl));

    s.skyAmbient = k.ambient + moonColor * 0.12f * moonLight;
    const float bounce = luminance(s.lightColor) * 0.012f + luminance(s.skyAmbient) * 0.16f;
    s.groundAmbient = glm::vec3(1.0f, 0.82f, 0.58f) * bounce;
    return s;
}

SunScreen sunOnScreen(const glm::mat4& viewProj, const glm::vec3& cameraPos, const glm::vec3& sunDir, float night) {
    SunScreen r;
    const glm::vec4 c = viewProj * glm::vec4(cameraPos + sunDir * 1000.0f, 1.0f);
    if (c.w <= 0.0f) return r;
    r.uv = glm::vec2(c.x / c.w * 0.5f + 0.5f, 0.5f - c.y / c.w * 0.5f);
    const float edge = std::max(std::abs(r.uv.x - 0.5f), std::abs(r.uv.y - 0.5f));
    const float day = smooth(-0.03f, 0.06f, sunDir.y) * (1.0f - night);
    r.weight = day * (1.0f - smooth(0.45f, 1.1f, edge));
    return r;
}

} // namespace tf
