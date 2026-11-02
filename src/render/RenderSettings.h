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
    int msaa = 1;                  // 1, 2 or 4 samples on the HDR target
    bool fxaa = false;             // post AA on the tonemapped image
    int shadowCascades = 3;        // 0..4 (0 = no sun shadows)
    int shadowMapSize = 2048;      // per cascade
    float shadowDistance = 80.0f;  // m
    int shadowFilter = 1;          // 0: 1 tap (2x2 hardware PCF), 1: 4 taps, 2: 12 taps
    bool farShadows = true;        // terrain-only shadow of the whole level beyond the last cascade (needs cascades)
    bool reflections = true;       // planar reflection for the puddles (else sky only)
    int envProbe = 128;            // vehicle environment cube face size (0 = sky-only reflections on the vehicle)
    int envProbeFaces = 1;         // probe faces re-rendered per frame (1..6)
    int parallaxSteps = 8;         // ray-march steps of the level ground parallax near the camera (0 = off)
    float reflectionScale = 0.5f;  // of the render resolution
    bool bloom = true;
    int grassLevel = 3;            // 0 = off, 1..4: density levels (fine cell 0.2 / 0.15 / 0.1 / 0.075 m)
    float grassDistance = 110.0f;  // m, blades shrink away over the last 20 %
    float flowerDensity = 1.0f;    // 0..1: share of the flower clusters that bloom
    bool grassDetail = true;       // wheat awns, seed-head spikelets, clover leaf marks (alpha-cut shapes)
    bool clouds = true;
    int sprayMax = 3000;           // wheel spray particle cap (cosmetic)
    // Trees (generic for any tree model with mesh LODs and far cards): the mesh LODs reach out to the detail distance
    // (their boundaries scale with it and cross-fade), cards from there to the tree distance, nothing beyond.
    float treeDetailDistance = 650.0f; // m
    float treeDistance = 3500.0f;      // m
    float terrainDetail = 2.0f;    // level terrain: a chunk splits while the camera is closer than this times its size
    bool sprayDust = true;         // dust puffs / clouds in the spray
    int terrainTextureSize = 2048; // level ground textures: 512..4096 texels (normal maps at most 2048), 0 = off
    // Post effects. The depth-based ones (TAA, AO, motion blur, sun shafts, depth of field) need msaa == 1: they
    // sample the depth buffer, which stays multisampled with MSAA.
    bool taa = true;               // temporal AA (jittered projection, reprojected history), replaces MSAA / FXAA
    int ao = 1;                    // ground-truth AO at half resolution: 0 off, 1: 2 slices x 4 steps, 2: 3 x 6
    int contactShadows = 12;       // screen-space contact shadow steps toward the sun (half resolution), 0 = off
    float motionBlur = 0.5f;       // shutter as a share of the frame time (camera + vehicle parts), 0 = off
    float sunShafts = 0.6f;        // strength, 0 = off
    float lensFlare = 0.4f;        // ghosts of the bloom image (needs bloom), 0 = off
    bool autoExposure = true;      // eye adaptation on top of the time-of-day exposure
    bool dof = true;               // depth of field behind the vehicle in the menus and the garage
    float dofDriving = 0.0f;       // depth of field while driving (far blur, chase camera), 0 = off
    bool waterRefraction = true;   // lakes, rivers, sea: refracted, absorbed view of the bed (copy of the opaque scene)
    bool waterSsr = true;          // screen-space reflections on the water (needs waterRefraction), else sky only

    // Look
    float exposure = 1.1f;
    float timeOfDay = 9.0f;        // hours 0..24, drives sun, moon, sky, fog and eye adaptation (render/TimeOfDay.h)
    float sunHeadingDeg = 81.0f;   // compass rotation: world azimuth = compass azimuth + heading (81: sunset toward +Z)
    float fogDensity = 1.0f;       // multiplier
    float valleyFog = 1.0f;        // valley fog layer on open levels, multiplier
    float atmosphere = 1.0f;       // 0: palette sky only, 1: scattering sky mixed with the palette
    float bloomStrength = 0.05f;
    bool groundGrid = false;       // 1 m grid on the ground (reading speed / terrain)
    int tonemapper = 1;            // 0: ACES (fitted), 1: AgX (punchy)
    float grade = 1.0f;            // colour grade LUT amount (warm highlights, cool shadows, key-art mood)
    float aoStrength = 1.0f;       // exponent on the AO visibility
    float vignette = 0.28f;
    float grain = 0.025f;          // film grain amplitude (display units)
    float chromaticAberration = 0.0f;
    float sharpen = 0.3f;          // after TAA
};

// Look fields survive a quality preset change.
inline void copyLook(RenderSettings& dst, const RenderSettings& src) {
    dst.exposure = src.exposure;
    dst.timeOfDay = src.timeOfDay;
    dst.sunHeadingDeg = src.sunHeadingDeg;
    dst.fogDensity = src.fogDensity;
    dst.valleyFog = src.valleyFog;
    dst.atmosphere = src.atmosphere;
    dst.bloomStrength = src.bloomStrength;
    dst.groundGrid = src.groundGrid;
    dst.tonemapper = src.tonemapper;
    dst.grade = src.grade;
    dst.aoStrength = src.aoStrength;
    dst.vignette = src.vignette;
    dst.grain = src.grain;
    dst.chromaticAberration = src.chromaticAberration;
    dst.sharpen = src.sharpen;
}

inline RenderSettings presetSettings(QualityPreset p) {
    RenderSettings s;
    switch (p) {
    case QualityPreset::Low:
        s.renderScale = 0.67f; s.msaa = 1; s.fxaa = true; s.shadowCascades = 1; s.shadowMapSize = 1024;
        s.shadowDistance = 40.0f; s.shadowFilter = 0; s.farShadows = false; s.reflections = false; s.envProbe = 0; s.parallaxSteps = 0; s.bloom = false; s.grassLevel = 1;
        s.grassDistance = 40.0f; s.flowerDensity = 0.5f; s.grassDetail = false; s.sprayMax = 600; s.sprayDust = false;
        s.terrainDetail = 1.2f; s.treeDetailDistance = 380.0f; s.treeDistance = 2000.0f; s.terrainTextureSize = 512;
        s.taa = false; s.ao = 0; s.contactShadows = 0; s.motionBlur = 0.0f; s.sunShafts = 0.0f; s.lensFlare = 0.0f; s.waterSsr = false;
        break;
    case QualityPreset::Medium:
        s.renderScale = 0.85f; s.msaa = 1; s.fxaa = false; s.shadowCascades = 2; s.shadowMapSize = 1536;
        s.shadowDistance = 60.0f; s.shadowFilter = 1; s.reflections = true; s.reflectionScale = 0.35f; s.bloom = true;
        s.grassLevel = 2; s.grassDistance = 70.0f; s.flowerDensity = 0.8f; s.sprayMax = 1500; s.terrainDetail = 1.6f;
        s.treeDetailDistance = 500.0f; s.treeDistance = 3000.0f; s.terrainTextureSize = 1024;
        s.motionBlur = 0.4f; s.sunShafts = 0.5f; s.lensFlare = 0.3f; s.contactShadows = 8; s.envProbe = 64;
        s.parallaxSteps = 0;
        break;
    case QualityPreset::High:
        break; // defaults above
    case QualityPreset::Ultra:
        s.renderScale = 1.0f; s.msaa = 1; s.fxaa = false; s.shadowCascades = 4; s.shadowMapSize = 2048;
        s.shadowDistance = 110.0f; s.shadowFilter = 2; s.reflections = true; s.reflectionScale = 0.75f; s.bloom = true;
        s.grassLevel = 4; s.grassDistance = 160.0f; s.sprayMax = 6000; s.terrainDetail = 2.6f;
        s.treeDetailDistance = 900.0f; s.treeDistance = 4500.0f; s.terrainTextureSize = 4096; s.ao = 2;
        s.contactShadows = 16; s.envProbeFaces = 2; s.parallaxSteps = 16;
        break;
    default: break;
    }
    return s;
}

} // namespace tf
