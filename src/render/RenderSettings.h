#pragma once

namespace tf {

enum class QualityPreset { Low = 0, Medium, High, Ultra, Custom };

inline const char* presetName(QualityPreset p) {
    switch (p) {
    case QualityPreset::Low: return "Low";
    case QualityPreset::Medium: return "Medium";
    case QualityPreset::High: return "High";
    case QualityPreset::Ultra: return "Ultra";
    default: return "Custom";
    }
}

// Renderer quality and look. Presets fill everything; the Graphics panel can edit single fields (-> Custom).
struct RenderSettings {
    // Performance-relevant
    float renderScale = 1.0f;      // 3D resolution relative to the window pixels (upscaled in the tonemap pass)
    int msaa = 2;                  // 1, 2 or 4 samples on the HDR target
    bool fxaa = false;             // post AA on the tonemapped image
    int shadowCascades = 3;        // 0..4 (0 = no sun shadows)
    int shadowMapSize = 2048;      // per cascade
    float shadowDistance = 80.0f;  // m
    int shadowFilter = 1;          // 0: 1 tap (2x2 hardware PCF), 1: 4 taps, 2: 12 taps
    bool reflections = true;       // planar reflection for the puddles (else sky only)
    float reflectionScale = 0.5f;  // of the render resolution
    bool bloom = true;
    int grassLevel = 3;            // 0 = off, 1..4: density levels (fine cell 0.2 / 0.15 / 0.1 / 0.075 m)
    float grassDistance = 45.0f;   // m
    float flowerDensity = 1.0f;    // 0..1: share of the flower clusters that bloom
    bool grassDetail = true;       // wheat awns, seed-head spikelets, clover leaf marks (alpha-cut shapes)
    bool clouds = true;
    int sprayMax = 3000;           // wheel spray particle cap (cosmetic)
    bool sprayDust = true;         // dust puffs / clouds in the spray

    // Look
    float exposure = 1.1f;
    float timeOfDay = 9.0f;        // hours 0..24, drives sun, moon, sky, fog and eye adaptation (render/TimeOfDay.h)
    float sunHeadingDeg = 81.0f;   // compass rotation: world azimuth = compass azimuth + heading (81: sunset toward +Z)
    float fogDensity = 1.0f;       // multiplier
    float bloomStrength = 0.05f;
    bool groundGrid = false;       // 1 m grid on the ground (reading speed / terrain)
};

// Look fields survive a quality preset change.
inline void copyLook(RenderSettings& dst, const RenderSettings& src) {
    dst.exposure = src.exposure;
    dst.timeOfDay = src.timeOfDay;
    dst.sunHeadingDeg = src.sunHeadingDeg;
    dst.fogDensity = src.fogDensity;
    dst.bloomStrength = src.bloomStrength;
    dst.groundGrid = src.groundGrid;
}

inline RenderSettings presetSettings(QualityPreset p) {
    RenderSettings s;
    switch (p) {
    case QualityPreset::Low:
        s.renderScale = 0.67f; s.msaa = 1; s.fxaa = true; s.shadowCascades = 1; s.shadowMapSize = 1024;
        s.shadowDistance = 40.0f; s.shadowFilter = 0; s.reflections = false; s.bloom = false; s.grassLevel = 1;
        s.grassDistance = 22.0f; s.flowerDensity = 0.5f; s.grassDetail = false; s.sprayMax = 600; s.sprayDust = false;
        break;
    case QualityPreset::Medium:
        s.renderScale = 0.85f; s.msaa = 1; s.fxaa = true; s.shadowCascades = 2; s.shadowMapSize = 1536;
        s.shadowDistance = 60.0f; s.shadowFilter = 1; s.reflections = true; s.reflectionScale = 0.35f; s.bloom = true;
        s.grassLevel = 2; s.grassDistance = 32.0f; s.flowerDensity = 0.8f; s.sprayMax = 1500;
        break;
    case QualityPreset::High:
        break; // defaults above
    case QualityPreset::Ultra:
        s.renderScale = 1.0f; s.msaa = 4; s.fxaa = false; s.shadowCascades = 4; s.shadowMapSize = 2048;
        s.shadowDistance = 110.0f; s.shadowFilter = 2; s.reflections = true; s.reflectionScale = 0.75f; s.bloom = true;
        s.grassLevel = 4; s.grassDistance = 60.0f; s.sprayMax = 6000;
        break;
    default: break;
    }
    return s;
}

} // namespace tf
