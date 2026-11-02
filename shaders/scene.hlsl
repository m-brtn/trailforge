// TrailForge shaders (HLSL). The single shader source for every backend: each entry point is compiled at build
// time (tools/shaderc, see cmake/Shaders.cmake) to SPIR-V (Vulkan), MSL -> metallib (Metal) and, on Windows, DXIL
// (D3D12). The build defines TF_ENTRY_<name> and TF_STAGE_VERTEX or TF_STAGE_FRAGMENT for each compile, so every
// entry point declares only its own resources.
// Resource binding follows the SDL GPU rules (SDL_CreateGPUShader):
//   vertex:   textures t[n] + samplers s[n] in space0, read-only storage buffers t[samplers + i] in space0,
//             uniform slot n -> b[n] in space1;
//   fragment: the same in space2, uniforms in space3.
//   Vertex attributes use TEXCOORD<location> with a matching vk::location; varyings are matched by TEXCOORD index.
// Struct layouts must match src/render/Renderer.h (FrameUniforms, DrawUniforms, GrassUniforms, PostUniforms,
// MarkUniforms) and src/sim/TireMarks.h (TireMarkSegment); uniform and storage structs contain only float4 and
// float4x4 members, so HLSL, std140/std430 and MSL layouts agree. Matrices are column-major (glm), mul(M, v).
// Rendering is linear HDR; the tonemap pass converts to the (UNORM, non-sRGB) swapchain with a gamma curve.

#if defined(TF_STAGE_VERTEX)
#define TF_RES_SPACE space0
#define TF_UBO_SPACE space1
#else
#define TF_RES_SPACE space2
#define TF_UBO_SPACE space3
#endif
#define TEX(n) register(t##n, TF_RES_SPACE)
#define SMP(n) register(s##n, TF_RES_SPACE)
#define UBO(n) register(b##n, TF_UBO_SPACE)

static const float PI = 3.14159265;
// The fitted ACES curve (Narkowicz) expects scene values scaled by ~0.6; the user exposure multiplies this.
static const float ACES_PRE_EXPOSURE = 0.6;

// Material ids (src/render/Mesh.h, MatId). 0 = generic, 4 = low-grip slab (handled as fallbacks).
static const int MAT_DIRT = 1;
static const int MAT_CONCRETE = 2;
static const int MAT_ROCK = 3;
static const int MAT_MUD = 5;
static const int MAT_GRASS = 6;
static const int MAT_GLASS = 7;
static const int MAT_HEADLIGHT = 8;
static const int MAT_BRAKE = 9;
static const int MAT_REVERSE = 10;
static const int MAT_RUBBER = 11;
static const int MAT_AMBER = 12;
static const int MAT_FOLIAGE = 13;
static const int MAT_PAINT = 14;
static const int MAT_SAND = 15;
static const int MAT_SNOW = 16;
static const int MAT_TERRAIN = 17; // level terrain: splat weights rock / sand / snow in the color, the rest is dirt + mask
static const int MAT_CLADDING = 18; // grained unpainted plastic (body cladding, bumpers, flares)
static const int MAT_ASPHALT = 19;  // road finishes of the mini handling track: vertex color + aggregate noise
static const int MAT_HORIZON = 20;  // level horizon ring: palette from world-space noise, vertex colour = bake edge tone

// BodyMudRegion (src/sim/BodyMud.h).
static const int kMudArchRearShader = 0;
static const int kMudArchFrontShader = 4;
static const int kMudBehindShader = 8;
static const int kMudRearBumperShader = 12;
static const int kMudFrontBumperShader = 13;

struct FrameUniforms {
    float4x4 viewProj;
    float4x4 invViewProj;
    float4x4 shadowViewProj[4];
    float4x4 vehicleInv;    // world -> vehicle body space
    float4 cascadeSplits;   // view distance of the far end of each cascade
    float4 cascadeTexel;    // world size of one shadow texel per cascade (m)
    float4 cameraPos;       // xyz, w: time (s)
    float4 cameraFwd;       // xyz: view direction (for cascade selection)
    float4 sunDir;          // xyz: direction TO the sun
    float4 sunColor;        // rgb: sun irradiance (HDR)
    float4 skyZenith;       // rgb
    float4 skyHorizon;      // rgb
    float4 groundAmbient;   // rgb: bounce light from below
    float4 fogParams;       // x: density at y = 0, y: height falloff (1/m), z: sun inscatter, w: cloud coverage
    float4 shadowParams;    // x: cascades (0 = off), y: 1/map size, z: filter (0/1/2), w: normal offset scale
    float4 waterParams;     // x: water level, y: reflection texture valid, z/w: 1/render target size
    float4 groundRect;      // xMin, zMin, 1/sizeX, 1/sizeZ (ground maps)
    float4 vehicleParams;   // x: mud amount, y: brake light, z: reverse light, w: headlights
    float4 clipPlane;       // xyz normal, w offset; fragments with dot(n, p) + w < 0 are discarded (w > 1e5: off)
    float4 options;         // x: 1 m grid strength, y: exposure, z: wind strength, w: clouds on
    float4 skySun;          // xyz: true sun direction (sunDir is the shadow light: sun or moon), w: cos disc radius
    float4 skySunGlow;      // rgb: glow / in-scatter color around the sun, w: star visibility
    float4 sunDisc;         // rgb: sun disc radiance, w: moon disc brightness
    float4 moonDir;         // xyz, w: night factor
    float4 fogColor;        // rgb, w: world azimuth of the mountain valley (rad)
    float4 skyAmbient;      // rgb: sky diffuse irradiance, w: scattering sky amount
    float4 headlightL;      // xyz: lamp position, w: intensity (0 = off)
    float4 headlightR;      // xyz: lamp position
    float4 headlightDir;    // xyz: beam axis, w: cos of the outer cone angle
    float4 bodyMudAmount[4]; // BodyMud regions 0..13 (src/sim/BodyMud.h order), cosmetic
    float4 bodyMudColor[16]; // rgb linear, w: wetness
    float4 bodyMudFlecks[4]; // grass blades in the coat per region
    float4 grassParams;      // x: grass distance (m, 0 = grass off), y: canopy colour on the ground near the camera
    float4 trampleRect;      // xMin, zMin, 1/sizeX, 1/sizeZ of the grass trample mask
    float4 worldParams;      // x: sky-pass mountains (0/1), y: water outside the ground maps is deep (0/1),
                             // z: valley fog density at its base, w: valley fog base height
    float4 bodyMudArch;      // vehicle body: axle z front / rear, arch centre y, arch radius (BodyMud regions)
    float4 bodyMudBody;      // vehicle body: side |x|, bumper zones from z front / rear
    float4 screenFx;         // x: opaque scene copy + linear depth bound (0/1), y: water SSR (0/1)
    float4x4 farShadowViewProj;
    float4 farShadow;        // x: layer of the far terrain shadow (-1: off), y: texel size (m), z: bias toward the light (m)
    float4 envProbe;         // vehicle probe: x valid (0/1), y: diffuse mip, z: mip for roughness 1
    float4 parallax;         // level ground parallax: x steps (0 = off), y depth (m), z full until (m), w end (m)
    float4 groundFx;         // level ground: x tiling (0 plain, 1 macro tone, 2 hex tiling + macro tone), y detail
                             // normals, z wet ground, w specular AA + occlusion (0/1)
    float4 groundShadow;     // level ground micro-shadows: x steps (0 = off), y strength
};

struct DrawUniforms {
    float4x4 model;
    float4x4 normalMatrix;  // inverse transpose of model (upper 3x3)
    float4 tint;            // rgb multiplier
    float4 params;          // x: mud scale, y: mud mapping (0 body, 1 wheel), z: tread soil wetness, w: wheel radius
    float4 soil;            // wheels: soil in the tread, rgb linear colour, a amount 0..1
    float4 soilExtra;       // wheels: x grass flecks in the tread, y soil adhesion (dry sand dusts, mud cakes); z: vehicle probe (0/1)
};

// The frame uniforms are uniform slot 0 of almost every entry point. The entry points whose slot 0 holds other
// uniforms do not read them; there `f` is parked on an unused slot and the compiler drops it.
#if defined(TF_ENTRY_shadow_vs) || defined(TF_ENTRY_tree_shadow_vs) || defined(TF_ENTRY_terrain_shadow_vs) || \
    defined(TF_ENTRY_model_shadow_vs) || \
    defined(TF_ENTRY_bloom_down_fs) || \
    defined(TF_ENTRY_bloom_up_fs) || defined(TF_ENTRY_tonemap_fs) || defined(TF_ENTRY_fxaa_fs) || \
    defined(TF_ENTRY_blur_down_fs) || defined(TF_ENTRY_blur_up_fs) || defined(TF_ENTRY_blur_composite_fs) || \
    defined(TF_ENTRY_lindepth_fs) || defined(TF_ENTRY_gtao_fs) || defined(TF_ENTRY_ao_blur_fs) || \
    defined(TF_ENTRY_velocity_fs) || defined(TF_ENTRY_velocity_obj_vs) || defined(TF_ENTRY_velocity_obj_fs) || \
    defined(TF_ENTRY_taa_fs) || defined(TF_ENTRY_motion_blur_fs) || defined(TF_ENTRY_dof_blur_fs) || \
    defined(TF_ENTRY_dof_fs) || defined(TF_ENTRY_lum_fs) || defined(TF_ENTRY_adapt_fs) || defined(TF_ENTRY_shafts_fs)
ConstantBuffer<FrameUniforms> f : UBO(13);
#else
ConstantBuffer<FrameUniforms> f : UBO(0);
#endif

float length_squared(float3 v) { return dot(v, v); }

// ---------------------------------------------------------------- common helpers

float3 sampleNoise(Texture2D<float4> tex, SamplerState s, float2 uv) { return tex.Sample(s, uv).rgb; }

float luminance(float3 c) { return dot(c, float3(0.2126, 0.7152, 0.0722)); }

float hash12(float2 p) {
    float3 p3 = frac(((float3)(p.xyx)) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return frac((p3.x + p3.y) * p3.z);
}

// Single scattering in a flat exponential atmosphere (Hoffman and Preetham 2002): Rayleigh (per wavelength, scale
// height 8 km) and Mie (1.2 km) with Kasten-Young air mass along the view and toward the sun, so the sky turns blue
// overhead, pale at the horizon and red where sunlight crosses more air. Unit sun irradiance; skyRadiance scales it.
float airMass(float y) {
    const float zenithDeg = acos(clamp(y, 0.0, 1.0)) * 57.29578;
    return 1.0 / (max(y, 0.0) + 0.50572 * pow(max(96.07995 - zenithDeg, 1e-3), -1.6364));
}

float3 atmosphere(float3 dir, float3 sun) {
    const float3 bR = float3(5.8e-3, 13.5e-3, 33.1e-3); // 1/km
    const float3 ozone = float3(0.0098, 0.028, 0.0013); // vertical optical depth of the ozone layer
    const float bM = 3.0e-3, HR = 8.0, HM = 1.2;
    const float mv = airMass(dir.y), ms = airMass(sun.y);
    const float3 tauV = bR * HR * mv + bM * 1.1 * HM * mv;
    // Most scattering happens well above the ground, where sunlight has crossed only part of the air column (0.35 of
    // the ground-level air mass); ozone removes the green that the plain model leaves at low sun.
    const float3 sunT = exp(-((bR * HR + bM * 1.1 * HM) * 0.35 + ozone) * ms - ozone * 0.5 * mv);
    const float mu = dot(dir, sun);
    const float pR = 0.0596831 * (1.0 + mu * mu);
    const float g = 0.76;
    const float pM = 0.0795775 * (1.0 - g * g) / pow(max(1.0 + g * g - 2.0 * g * mu, 1e-4), 1.5);
    return sunT * (bR * pR + bM * pM) / (bR + bM) * (1.0 - exp(-tauV));
}

// Palette sky (TimeOfDay keys: night, twilight and the stylized banner colours).
float3 paletteSky(float3 dir) {
    const float3 sun = f.skySun.xyz;
    const float up = saturate(dir.y);
    // Toward the sun the warm horizon band reaches higher (sunset banding), away from it the sky stays cooler.
    const float mu = dot(dir, sun);
    const float toward = saturate(mu * 0.5 + 0.5);
    const float band = lerp(0.55, 0.28, toward * toward);
    float3 col = lerp(f.skyHorizon.rgb, f.skyZenith.rgb, pow(up, band));
    const float g = 0.76;
    const float hg = (1.0 - g * g) / pow(1.0 + g * g - 2.0 * g * mu, 1.5) / (4.0 * PI);
    col += f.skySunGlow.rgb * hg * 0.10;
    col += f.skySunGlow.rgb * 0.05 * pow(saturate(mu), 6.0) * (1.0 - up);
    return col;
}

// Sky radiance for a view direction (no sun disc), used by the sky pass and as environment light: the scattering
// model scaled to the palette's brightness at the zenith, mixed with the palette (which carries night and twilight).
float3 skyRadiance(float3 dir) {
    const float3 sun = f.skySun.xyz;
    const float3 d = float3(dir.x, max(dir.y, 0.0), dir.z);
    const float3 pal = paletteSky(d);
    const float3 zen = atmosphere(float3(0, 1, 0), sun);
    const float scale = luminance(f.skyZenith.rgb) / max(luminance(zen), 1e-6);
    const float day = smoothstep(-0.06, 0.05, sun.y);
    float3 col = lerp(pal, atmosphere(d, sun) * scale * 0.85 + pal * 0.25, day * f.skyAmbient.w);
    // Below the horizon: hazy ground color.
    const float3 below = lerp(f.fogColor.rgb * 0.85, f.groundAmbient.rgb * 1.4, saturate(-dir.y * 4.0));
    return dir.y < 0.0 ? lerp(col, below, saturate(-dir.y * 12.0)) : col;
}

// Haze color along a view ray: the fog color, warmed toward the sun; with the scattering sky, the sky colour just
// above the horizon in that direction (aerial perspective: distant land fades into the sky behind it).
float3 hazeColor(float3 rd) {
    const float sunAmt = pow(saturate(dot(rd, f.skySun.xyz)), 8.0);
    const float3 base = f.fogColor.rgb + f.skySunGlow.rgb * 0.22 * sunAmt * f.fogParams.z;
    if (f.skyAmbient.w <= 0.0) return base;
    const float3 h = normalize(float3(rd.x, 0.06, rd.z));
    const float3 sky = skyRadiance(h);
    return lerp(base, sky * (luminance(base) / max(luminance(sky), 1e-5)) * 0.5 + sky * 0.5, 0.6 * f.skyAmbient.w);
}

// Optical depth of an exponential height layer (density a at y = 0, falloff b) along a ray from camPos.
float fogLayer(float a, float b, float3 camPos, float3 rd, float dist) {
    const float k = b * rd.y;
    const float camTerm = exp(-b * camPos.y);
    return abs(k) > 1e-4 ? a * camTerm * (1.0 - exp(-k * dist)) / k : a * camTerm * dist;
}

// Analytic height fog (exponential in height) with sun in-scattering, plus a thin valley layer on open levels
// (worldParams z: density at its base, w: base height). Returns rgb: fog color, a: amount.
float4 fogAt(float3 worldPos, float3 camPos) {
    const float3 d = worldPos - camPos;
    const float dist = length(d);
    if (dist < 1e-3) return ((float4)(0.0));
    const float3 rd = d / dist;
    float optical = fogLayer(f.fogParams.x, f.fogParams.y, camPos, rd, dist);
    if (f.worldParams.z > 0.0) {
        const float3 c = float3(camPos.x, camPos.y - f.worldParams.w, camPos.z);
        optical += fogLayer(f.worldParams.z, 0.03, c, rd, dist);
    }
    const float fog = 1.0 - exp(-max(optical, 0.0));
    return float4(hazeColor(rd), fog);
}

// Cloud layer (render-only): a plane 2 km up drifting with the wind. Returns coverage 0..1 at a world position on
// the plane; the sky pass draws it and the ground receives its shadow along the sun.
static const float kCloudHeight = 2000.0;
float valueNoise(float2 p) {
    const float2 i = floor(p);
    const float2 u = frac(p);
    const float2 w = u * u * (3.0 - 2.0 * u);
    return lerp(lerp(hash12(i), hash12(i + float2(1, 0)), w.x), lerp(hash12(i + float2(0, 1)), hash12(i + float2(1, 1)), w.x), w.y);
}

// Large shapes of the cumulus layer: two value-noise octaves (the ground shadows use exactly these; the sky adds
// texture detail on top, see skyClouds).
float cloudShape(float2 xz) {
    const float t = f.cameraPos.w;
    float2 p = (xz + float2(9.0, 4.0) * t) * float2(1.0 / 2400.0, 1.0 / 1100.0);
    float n = valueNoise(p) * 0.5;
    p = float2(p.x * 1.6 - p.y * 1.2, p.x * 1.2 + p.y * 1.6) + 7.3;
    n += valueNoise(p) * 0.25;
    return n / 0.75;
}

float cloudCoverOf(float n) {
    const float cover = f.fogParams.w;
    return smoothstep(1.0 - cover, 1.0 - cover + 0.22, n);
}

float cloudCover(float2 xz) { return cloudCoverOf(cloudShape(xz)); }

// Henyey-Greenstein phase times 4 pi (1 = isotropic).
float phaseHG(float mu, float g) {
    const float g2 = g * g;
    return (1.0 - g2) / pow(max(1.0 + g2 - 2.0 * g * mu, 1e-4), 1.5);
}

float cloudShadow(float3 P) {
    if (f.options.w < 0.5 || f.skySun.y < 0.05) return 1.0;
    const float3 L = f.skySun.xyz;
    const float2 q = P.xz + L.xz * ((kCloudHeight - P.y) / L.y);
    return 1.0 - 0.55 * cloudCover(q) * smoothstep(0.05, 0.25, L.y);
}

// Low-beam headlights: two unshadowed spot lights on the vehicle (render-only). Returns irradiance.
// Stylized for readability: wrapped N.L so the grazing ground still catches light, and the beam keeps its
// brightness out to ~25 m (real low beams aim their hot spot far ahead) before the inverse-square falloff.
float3 headlightIrradiance(float3 P, float3 N) {
    const float I = f.headlightL.w;
    if (I <= 0.0) return ((float3)(0.0));
    const float3 axis = f.headlightDir.xyz;
    float sum = 0.0;
    for (int i = 0; i < 2; ++i) {
        const float3 d = (i == 0 ? f.headlightL.xyz : f.headlightR.xyz) - P;
        const float d2 = max(dot(d, d), 0.25);
        const float3 l = d * rsqrt(d2);
        const float cosA = dot(-l, axis);
        float beam = smoothstep(f.headlightDir.w, lerp(f.headlightDir.w, 1.0, 0.6), cosA) * 0.55 +
                     smoothstep(0.94, 0.995, cosA) * 1.2;
        beam *= 1.0 - smoothstep(0.015, 0.09, -l.y - axis.y); // sharp cut-off above the beam axis
        const float falloff = clamp(d2 / 25.0, 1.0, 30.0) / d2 * (1.0 - smoothstep(38.0, 60.0, sqrt(d2)));
        sum += beam * falloff * saturate(dot(N, l) * 0.75 + 0.25);
    }
    return float3(1.0, 0.93, 0.82) * I * sum;
}

float3 applyFog(float3 color, float3 worldPos, float3 camPos) {
    const float4 fog = fogAt(worldPos, camPos);
    return lerp(color, fog.rgb, fog.a);
}

// Terrain-only shadow of the whole level (layer after the cascades), used beyond the last cascade. Coarse texels:
// the bias toward the light hides the gap between the coarse caster grid and the rendered ground.
float farShadowFactor(Texture2DArray<float> sm, SamplerComparisonState cmp, float3 P, float3 N, int maxFilter) {
    if (f.farShadow.x < 0.0) return 1.0;
    const float3 Pn = P + N * f.farShadow.y + f.sunDir.xyz * f.farShadow.z;
    const float4 sc = mul(f.farShadowViewProj, float4(Pn, 1.0));
    float2 uv = sc.xy * 0.5 + 0.5;
    uv.y = 1.0 - uv.y;
    if (any(uv < 0.0) || any(uv > 1.0) || sc.z > 1.0) return 1.0;
    const float layer = f.farShadow.x;
    if (maxFilter == 0) return sm.SampleCmpLevelZero(cmp, float3(uv, layer), sc.z);
    const float o = f.shadowParams.y * 0.75;
    return 0.25 * (sm.SampleCmpLevelZero(cmp, float3(uv + float2(-o, -o), layer), sc.z) +
                   sm.SampleCmpLevelZero(cmp, float3(uv + float2(o, -o), layer), sc.z) +
                   sm.SampleCmpLevelZero(cmp, float3(uv + float2(-o, o), layer), sc.z) +
                   sm.SampleCmpLevelZero(cmp, float3(uv + float2(o, o), layer), sc.z));
}

// Cascaded shadow lookup. Returns 1 = lit. maxFilter caps the filter (0: one tap, for large translucent particles).
float shadowFactor(Texture2DArray<float> sm, SamplerComparisonState cmp, float3 P, float3 N,
                   int maxFilter = 2) {
    const int cascades = int(f.shadowParams.x);
    if (cascades <= 0) return 1.0;
    const float viewDepth = dot(P - f.cameraPos.xyz, f.cameraFwd.xyz);
    int c = 0;
    for (; c < cascades; ++c)
        if (viewDepth < f.cascadeSplits[c]) break;
    if (c >= cascades) return farShadowFactor(sm, cmp, P, N, maxFilter);
    const float texel = f.shadowParams.y;
    // Normal offset scales with the world size of one shadow texel in this cascade.
    const float3 Pn = P + N * f.cascadeTexel[c] * f.shadowParams.w;
    const float4 sc = mul(f.shadowViewProj[c], float4(Pn, 1.0));
    float2 uv = sc.xy * 0.5 + 0.5;
    uv.y = 1.0 - uv.y;
    const float z = sc.z;
    if (any(uv < 0.0) || any(uv > 1.0) || z > 1.0) return 1.0;
    const int filterMode = min(int(f.shadowParams.z), maxFilter);
    float shadowLit = 0.0;
    if (filterMode == 0) {
        shadowLit = sm.SampleCmpLevelZero(cmp, float3(uv, c), z);
    } else if (filterMode == 1) {
        const float2 o = ((float2)(texel)) * 1.2;
        shadowLit = sm.SampleCmpLevelZero(cmp, float3(uv + float2(-o.x, -o.y), c), z) + sm.SampleCmpLevelZero(cmp, float3(uv + float2(o.x, -o.y), c), z) +
              sm.SampleCmpLevelZero(cmp, float3(uv + float2(-o.x, o.y), c), z) + sm.SampleCmpLevelZero(cmp, float3(uv + float2(o.x, o.y), c), z);
        shadowLit *= 0.25;
    } else {
        const float2 taps[12] = {float2(-0.326, -0.406), float2(-0.840, -0.074), float2(-0.696, 0.457), float2(-0.203, 0.621),
                                 float2(0.962, -0.195), float2(0.473, -0.480), float2(0.519, 0.767), float2(0.185, -0.893),
                                 float2(0.507, 0.064), float2(0.896, 0.412), float2(-0.322, -0.933), float2(-0.792, -0.598)};
        for (int i = 0; i < 12; ++i) shadowLit += sm.SampleCmpLevelZero(cmp, float3(uv + taps[i] * texel * 2.2, c), z);
        shadowLit /= 12.0;
    }
    // Fade into the far terrain shadow (or lit) at the end of the last cascade.
    const float fadeStart = f.cascadeSplits[cascades - 1] * 0.85;
    const float fade = saturate((viewDepth - fadeStart) / (f.cascadeSplits[cascades - 1] - fadeStart));
    if (fade > 0.0) shadowLit = lerp(shadowLit, farShadowFactor(sm, cmp, P, N, maxFilter), fade);
    return shadowLit;
}

// Ground maps (mask: R grass, G mud weight, B wetness).
float2 groundUV(float2 xz) { return (xz - f.groundRect.xy) * f.groundRect.zw; }
float2 trampleUV(float2 xz) { return (xz - f.trampleRect.xy) * f.trampleRect.zw; }

float4 sampleMask(Texture2D<float4> mask, SamplerState s, float2 xz) {
    const float2 uv = groundUV(xz);
    if (any(uv < 0.0) || any(uv > 1.0)) return float4(0.55, 0.0, 0.0, 0.0); // backdrop outside the walls
    return mask.SampleLevel(s, uv, 0);
}

// Ambient occlusion under the vehicle (soft rounded box in body space).
float vehicleAO(float3 P) {
    const float3 lp = (mul(f.vehicleInv, float4(P, 1.0))).xyz;
    const float2 q = abs(lp.xz) - float2(0.80, 2.05);
    const float d = length(max(q, 0.0)) + min(max(q.x, q.y), 0.0);
    const float foot = 1.0 - smoothstep(-0.3, 1.1, d);
    const float h = saturate(1.0 - abs(lp.y + 0.78) / 1.0);
    return 1.0 - 0.65 * foot * h;
}

// Grass species zones, shared by the blades and the ground under them (so the far ground matches the blades).
struct GrassZones {
    float tall;   // tall wild grass with seed heads
    float clover; // low clover leaves
    float dry;    // straw-coloured blades: slopes and sparse margins (near rocks, tufts in the dirt)
    float patch;  // density multiplier: dense clumps and sparse spots
};

// slope: tangent of the ground slope angle.
GrassZones grassZones(Texture2D<float4> noiseTex, SamplerState ns, float2 p, float density, float slope) {
    const float4 a = noiseTex.SampleLevel(ns, p * 0.019, 0);
    const float4 b = noiseTex.SampleLevel(ns, p * 0.087 + 0.31, 0);
    GrassZones z;
    z.tall = smoothstep(0.47, 0.60, a.r);
    z.clover = smoothstep(0.50, 0.62, b.g) * (1.0 - z.tall);
    z.dry = saturate(smoothstep(0.54, 0.68, a.g) * 0.55 + smoothstep(0.12, 0.32, slope) * 0.7 +
                     (1.0 - smoothstep(0.35, 0.75, density)) * 0.6);
    z.patch = lerp(0.55, 1.20, smoothstep(0.28, 0.42, b.r));
    return z;
}

// Mean albedo of the blade canopy seen from afar, for the ground under distant grass and wheat.
float3 grassCanopyAlbedo(GrassZones z, float wheat) {
    float3 c = lerp(float3(0.080, 0.110, 0.022), float3(0.110, 0.115, 0.034), z.tall);
    c = lerp(c, float3(0.050, 0.085, 0.026), z.clover * 0.6);
    c = lerp(c, float3(0.22, 0.16, 0.065), z.dry * 0.5);
    return lerp(c, float3(0.36, 0.26, 0.09), wheat);
}

// GGX specular + Lambert diffuse under the sun, plus sky ambient and an environment lobe.
struct ShadeInput {
    float3 albedo;
    float roughness;
    float metallic;
    float3 N;
    float3 V;
    float shadow;
    float ao;
    float specOcclusion;
    float3 emissive;
    float clearCoat;
    float3 lamp; // headlight irradiance
};

// Diffuse irradiance from the sky hemisphere (blended with the ground bounce by the normal).
float3 ambientLight(float ny) {
    return lerp(f.groundAmbient.rgb, f.skyAmbient.rgb, ny * 0.5 + 0.5);
}

float3 envBRDFApprox(float3 F0, float rough, float NdotV) {
    const float4 c0 = float4(-1.0, -0.0275, -0.572, 0.022);
    const float4 c1 = float4(1.0, 0.0425, 1.04, -0.04);
    const float4 r = rough * c0 + c1;
    const float a004 = min(r.x * r.x, exp2(-9.28 * NdotV)) * r.x + r.y;
    const float2 AB = float2(-1.04, 1.04) * a004 + r.zw;
    return F0 * AB.x + AB.y;
}

float ggxSpec(float NdotH, float NdotV, float NdotL, float rough) {
    const float a = max(rough * rough, 0.002);
    const float a2 = a * a;
    const float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    const float D = a2 / (PI * d * d);
    const float k = (rough + 1.0) * (rough + 1.0) / 8.0;
    const float G = NdotV / (NdotV * (1.0 - k) + k) * NdotL / (NdotL * (1.0 - k) + k);
    return D * G / max(4.0 * NdotV * NdotL, 1e-4);
}

// Indirect light seen by a surface: diffuse irradiance (as radiance of a uniform surround) along N, the environment
// along R blurred for the surface roughness, and sharp along R for the clear coat.
struct EnvLight {
    float3 ambient;
    float3 specular;
    float3 coat;
};

EnvLight skyEnv(float3 N, float3 R, float roughness) {
    EnvLight e;
    e.ambient = ambientLight(N.y);
    e.coat = skyRadiance(R);
    e.specular = lerp(e.coat, e.ambient, saturate(roughness * 1.2));
    return e;
}

// direct: sun (or moon) light, indirect: sky / bounce ambient and the environment reflection (see lightSplit).
float3 shadeSplitEnv(ShadeInput s, EnvLight e, out float3 direct, out float3 indirect) {
    const float3 L = f.sunDir.xyz;
    const float3 N = s.N, V = s.V;
    const float3 H = normalize(L + V);
    const float NdotL = saturate(dot(N, L));
    const float NdotV = max(dot(N, V), 1e-3);
    const float NdotH = saturate(dot(N, H));
    const float VdotH = saturate(dot(V, H));
    const float3 F0 = lerp(((float3)(0.04)), s.albedo, s.metallic);
    const float3 F = F0 + (1.0 - F0) * pow(1.0 - VdotH, 5.0);
    const float3 spec = F * ggxSpec(NdotH, NdotV, NdotL, s.roughness);
    const float3 kd = (1.0 - F) * (1.0 - s.metallic);
    direct = (kd * s.albedo / PI + spec) * f.sunColor.rgb * NdotL * s.shadow;

    // Ambient: sky hemisphere + ground bounce (or the local probe).
    indirect = e.ambient * s.albedo * (1.0 - s.metallic) * s.ao;
    // Environment specular (blurred toward the ambient with roughness).
    float3 env = e.specular * lerp(0.35, 1.0, s.shadow) * s.specOcclusion; // sun-shadowed areas see less of the bright sky
    indirect += env * envBRDFApprox(F0, s.roughness, NdotV) * s.ao;
    float3 color = direct + indirect + kd * s.albedo / PI * s.lamp;
    if (s.clearCoat > 0.0) {
        const float Fc = 0.04 + 0.96 * pow(1.0 - NdotV, 5.0);
        const float3 coatSpec = s.clearCoat * ggxSpec(NdotH, NdotV, NdotL, 0.06) * 0.04 * f.sunColor.rgb * NdotL * s.shadow;
        const float3 coatEnv = s.clearCoat * e.coat * Fc * s.specOcclusion * lerp(0.4, 1.0, s.shadow);
        const float keep = 1.0 - Fc * s.clearCoat;
        color = color * keep + coatSpec + coatEnv;
        direct = direct * keep + coatSpec;
        indirect = indirect * keep + coatEnv;
    }
    return color + s.emissive;
}

float3 shadeSplit(ShadeInput s, out float3 direct, out float3 indirect) {
    return shadeSplitEnv(s, skyEnv(s.N, reflect(-s.V, s.N), s.roughness), direct, indirect);
}

float3 shade(ShadeInput s) {
    float3 direct, indirect;
    return shadeSplit(s, direct, indirect);
}

// The opaque passes keep the split of their radiance in the alpha of the HDR target, for ao_blur_fs, which darkens
// only the indirect light with the ambient occlusion and only the sun light with the contact shadows: the integer
// part is the sun / moon share in 1/15 steps, the fraction the sky / bounce share (both after the fog). Headlights,
// emissive surfaces and the fog keep their brightness. Grass blades store -1 - split (no contact shadows).
float lightSplit(float3 color, float3 direct, float3 indirect, float fogA) {
    const float k = (1.0 - fogA) / max(luminance(color), 1e-6);
    return round(saturate(luminance(direct) * k) * 15.0) + min(saturate(luminance(indirect) * k), 0.99);
}

// Fog over a lit opaque surface, with the light split in alpha.
float4 fogSplit(float3 lit, float3 direct, float3 indirect, float3 P) {
    const float4 fog = fogAt(P, f.cameraPos.xyz);
    const float3 color = lerp(lit, fog.rgb, fog.a);
    return float4(color, lightSplit(color, direct, indirect, fog.a));
}

// Bump mapping from screen-space derivatives of a height function (Mikkelsen).
float3 perturbNormal(float3 N, float3 P, float h, float strength) {
    const float3 dpdx = ddx(P), dpdy = ddy(P);
    const float dhdx = ddx(h) * strength, dhdy = ddy(h) * strength;
    const float3 r1 = cross(dpdy, N), r2 = cross(N, dpdx);
    const float det = dot(dpdx, r1);
    const float3 grad = sign(det) * (dhdx * r1 + dhdy * r2);
    return normalize(abs(det) * N - grad);
}

// ---------------------------------------------------------------- lit meshes

struct LitIn {
    [[vk::location(0)]] float3 position : TEXCOORD0;
    [[vk::location(1)]] float3 normal : TEXCOORD1;
    [[vk::location(2)]] float3 color : TEXCOORD2;
    [[vk::location(3)]] float4 material : TEXCOORD3;
};

struct LitOut {
    float4 position : SV_Position;
    [[vk::location(0)]] float3 worldPos : TEXCOORD0;
    [[vk::location(1)]] float3 normal : TEXCOORD1;
    [[vk::location(2)]] float3 color : TEXCOORD2;
    [[vk::location(3)]] nointerpolation float4 material : TEXCOORD3;
    [[vk::location(4)]] float3 localPos : TEXCOORD4;
    [[vk::location(5)]] float3 localNormal : TEXCOORD5;
    [[vk::location(6)]] nointerpolation float mudMode : TEXCOORD6; // DrawUniforms.params.y
    [[vk::location(7)]] nointerpolation float4 soil : TEXCOORD7;
    [[vk::location(8)]] nointerpolation float2 soilParams : TEXCOORD8; // x: wetness, y: wheel radius
    [[vk::location(9)]] nointerpolation float3 soilExtra : TEXCOORD9; // x: grass flecks, y: adhesion, z: local probe
};

#if defined(TF_ENTRY_lit_vs) || defined(TF_ENTRY_shadow_vs)
ConstantBuffer<DrawUniforms> draw : UBO(1);
#endif

#if defined(TF_ENTRY_lit_vs)
LitOut lit_vs(LitIn vin) {
    LitOut vout;
    const float4 world = mul(draw.model, float4(vin.position, 1.0));
    vout.position = mul(f.viewProj, world);
    vout.worldPos = world.xyz;
    vout.normal = (mul(draw.normalMatrix, float4(vin.normal, 0.0))).xyz;
    vout.color = vin.color * draw.tint.rgb;
    vout.material = float4(vin.material.xyz, vin.material.w * draw.params.x);
    vout.localPos = vin.position;
    vout.localNormal = vin.normal;
    vout.mudMode = draw.params.y;
    vout.soil = draw.soil;
    vout.soilParams = draw.params.zw;
    vout.soilExtra = draw.soilExtra.xyz;
    return vout;
}
#endif

struct GroundSurface {
    float3 albedo;
    float roughness;
    float height; // bump height (m-ish units for perturbNormal)
    float bump;
};

// How much of the blade canopy colour the ground under grass takes: some near the camera (seen between the blades),
// all of it where the blades have thinned out with distance, and all of it with grass off.
float grassFar(float3 P) {
    const float D = f.grassParams.x;
    if (D <= 0.0) return 1.0;
    return lerp(f.grassParams.y, 1.0, smoothstep(0.40 * D, 0.92 * D, length(P.xz - f.cameraPos.xz)));
}

GroundSurface groundMaterial(int mat, float3 P, float3 N, float4 mask, Texture2D<float4> noiseTex, SamplerState ns, float far) {
    GroundSurface g;
    const float2 uv = P.xz;
    const float3 nA = sampleNoise(noiseTex, ns, uv * 0.045);
    const float3 nB = sampleNoise(noiseTex, ns, uv * 0.23);
    const float3 nC = sampleNoise(noiseTex, ns, uv * 1.15);
    const float fine = noiseTex.Sample(ns, uv * 3.7).a;
    const float large = nA.r * 0.6 + nB.g * 0.4;
    g.bump = 0.6;
    if (mat == MAT_DIRT || mat == MAT_GRASS || mat == MAT_MUD) {
        // Soil: packed dirt with gravel, green undergrowth where the mask has grass.
        // Warm earthy palette (ochre/umber), matching the stylized banner look.
        const float3 dark = float3(0.100, 0.062, 0.035), light = float3(0.245, 0.160, 0.090);
        float3 dirt = lerp(dark, light, smoothstep(0.25, 0.75, large));
        const float stone = 1.0 - smoothstep(0.18, 0.30, nC.b);
        dirt = lerp(dirt, float3(0.25, 0.20, 0.155) * (0.7 + 0.6 * nC.r), stone * 0.5);
        float h = nC.r * 0.5 + fine * 0.35 + stone * 0.6;
        const float slope = sqrt(max(1.0 - N.y * N.y, 0.0)) / max(N.y, 0.1);
        const GrassZones zones = grassZones(noiseTex, ns, P.xz, mask.r, slope);
        const float grassAmt = saturate(mask.r * zones.patch * 1.2);
        float3 under = lerp(float3(0.030, 0.046, 0.011), float3(0.070, 0.088, 0.022), nB.r) * (0.8 + 0.4 * fine);
        under = lerp(under, float3(0.115, 0.085, 0.040), zones.dry * 0.5);
        float3 albedo = lerp(dirt, under, grassAmt * (0.8 + 0.2 * nA.g));
        // Sown wheat: light tilled soil with straw between the rows.
        albedo = lerp(albedo, lerp(dirt * 1.15, float3(0.20, 0.155, 0.075), 0.45 + 0.3 * nC.r), mask.a);
        albedo = lerp(albedo, grassCanopyAlbedo(zones, mask.a), max(grassAmt, mask.a) * far);
        float rough = 0.92;
        h = lerp(h, fine * 0.6, max(grassAmt, mask.a));

        // Mud: blended by the smooth mask (not per triangle), edges broken up with noise.
        const float mudBlend = smoothstep(0.30, 0.55, mask.g + (nB.r - 0.5) * 0.30 + (fine - 0.5) * 0.08);
        if (mudBlend > 0.0) {
            const float3 dry = float3(0.125, 0.078, 0.042), wetC = float3(0.045, 0.029, 0.016);
            const float wetN = smoothstep(0.45, 0.65, nB.g * 0.7 + nA.r * 0.3);
            const float wet = saturate(wetN * 0.8 + mask.b * 0.5);
            const float3 mudAlb = lerp(dry, wetC, wet) * (0.85 + 0.3 * nC.r);
            // Tire-rut streaks along the travel direction (z) and clumps.
            const float ruts = sampleNoise(noiseTex, ns, float2(P.x * 0.9, P.z * 0.08)).g;
            const float mudH = lerp(nC.r * 0.6 + fine * 0.3 + ruts * 0.7, ruts * 0.25 + fine * 0.05, wet);
            albedo = lerp(albedo, mudAlb, mudBlend);
            rough = lerp(rough, lerp(0.85, 0.40, wet), mudBlend);
            h = lerp(h, mudH, mudBlend);
            g.bump = lerp(0.6, 0.9, mudBlend);
        }
        g.albedo = albedo;
        g.roughness = rough;
        g.height = h;
    } else if (mat == MAT_CONCRETE) {
        float3 c = float3(0.30, 0.295, 0.28) * (0.85 + 0.25 * large + 0.12 * (fine - 0.5));
        if (abs(N.y) < 0.5) {
            // Walls: weathered concrete, rain streaks, grime and moss toward the ground.
            const float streak = sampleNoise(noiseTex, ns, float2((P.x + P.z) * 0.6, P.y * 0.05)).g;
            c *= 0.75 + 0.35 * streak;
            const float grime = 1.0 - smoothstep(0.0, 1.4, P.y + (nB.r - 0.5) * 0.6);
            c = lerp(c, float3(0.07, 0.075, 0.04), grime * 0.75);
            const float2 cell = abs(frac(float2(P.x + P.z, P.y) / float2(3.0, 100.0)) - 0.5);
            c *= 1.0 - 0.4 * smoothstep(0.490, 0.497, cell.x);
            // Moss and grass hanging over the top edge (the embankment behind the wall is at its top).
            const float top = smoothstep(3.9, 4.9, P.y + (nB.g - 0.5) * 0.9);
            c = lerp(c * 0.62, float3(0.035, 0.05, 0.015), top * 0.85);
            g.albedo = c;
            g.roughness = 0.85;
            g.height = fine * 0.5 + streak * 0.3;
            g.bump = 0.5;
        } else {
            // Slab seams every 4 m.
            const float2 cell = abs(frac(P.xz / 4.0) - 0.5);
            const float seam = smoothstep(0.492, 0.498, max(cell.x, cell.y));
            c *= 1.0 - 0.45 * seam;
            g.albedo = c;
            g.roughness = 0.78 - 0.1 * fine;
            g.height = fine * 0.4 - seam * 0.6;
            g.bump = 0.4;
        }
    } else if (mat == MAT_ROCK) {
        const float3 w = abs(N) / (abs(N).x + abs(N).y + abs(N).z);
        const float3 tx = sampleNoise(noiseTex, ns, P.zy * 0.7), ty = sampleNoise(noiseTex, ns, P.xz * 0.7), tz = sampleNoise(noiseTex, ns, P.xy * 0.7);
        const float3 t = tx * w.x + ty * w.y + tz * w.z;
        const float3 tf = sampleNoise(noiseTex, ns, P.zy * 3.1) * w.x + sampleNoise(noiseTex, ns, P.xz * 3.1) * w.y + sampleNoise(noiseTex, ns, P.xy * 3.1) * w.z;
        // Warm tan sandstone; N is the flat facet normal, so each facet gets its own slight tone (stylized low-poly).
        const float facet = hash12(floor(N.xz * 9.0 + N.y * 5.0) + 0.37);
        float3 c = lerp(float3(0.085, 0.060, 0.046), float3(0.235, 0.165, 0.118), smoothstep(0.3, 0.7, t.r)) * (0.85 + 0.3 * tf.g);
        c *= 0.70 + 0.60 * facet;
        // Cracks from the cellular channel.
        c *= 0.7 + 0.3 * smoothstep(0.05, 0.18, tf.b);
        const float moss = smoothstep(0.55, 0.9, N.y) * smoothstep(0.45, 0.65, t.g);
        c = lerp(c, float3(0.055, 0.065, 0.020), moss * 0.6);
        // Dirt splashed onto the steep sides.
        c = lerp(c, float3(0.10, 0.068, 0.04), (1.0 - smoothstep(0.0, 0.6, abs(N.y))) * 0.15);
        g.albedo = c;
        g.roughness = 0.78;
        g.height = t.r * 0.8 + tf.b * 0.6;
        g.bump = 0.35; // low, so the bump does not blur the facets
    } else if (mat == MAT_SAND) {
        // Light dry sand: wind ripples across the prevailing wind, fine grain speckle, darker damp hollows.
        const float ripple = sin(dot(P.xz, float2(0.55, 2.4)) + nB.r * 5.0 + nA.g * 3.0);
        const float3 lightS = float3(0.40, 0.315, 0.20), darkS = float3(0.26, 0.20, 0.125);
        float3 c = lerp(darkS, lightS, smoothstep(0.2, 0.8, large * 0.7 + 0.3 * (0.5 + 0.5 * ripple)));
        c *= 0.92 + 0.16 * fine;
        c = lerp(c, float3(0.20, 0.17, 0.12), (1.0 - smoothstep(0.08, 0.2, nC.b)) * 0.35); // grit
        const float grassAmt = saturate(mask.r * 1.2);
        c = lerp(c, float3(0.09, 0.09, 0.03), grassAmt * 0.6);
        g.albedo = c;
        g.roughness = 0.96;
        g.height = 0.35 * (0.5 + 0.5 * ripple) + fine * 0.25;
        g.bump = 0.35;
    } else if (mat == MAT_SNOW) {
        // Old wind-packed snow: soft drifts, sastrugi streaks across the wind, faint sparkle in the fine noise.
        const float drift = nA.r * 0.6 + nB.g * 0.4;
        const float streaks = sampleNoise(noiseTex, ns, float2(dot(P.xz, float2(0.8, 0.6)) * 0.35, dot(P.xz, float2(-0.6, 0.8)) * 0.06)).g;
        float3 c = lerp(float3(0.56, 0.60, 0.66), float3(0.78, 0.81, 0.86), smoothstep(0.25, 0.75, drift * 0.7 + streaks * 0.3));
        c *= 0.96 + 0.08 * fine;
        g.albedo = c;
        g.roughness = 0.62 - 0.12 * smoothstep(0.7, 0.95, fine);
        g.height = drift * 0.5 + streaks * 0.4 + fine * 0.1;
        g.bump = 0.3;
    } else if (mat == MAT_ASPHALT) {
        // Dense asphalt: dark binder with fine aggregate (tire mark relief only).
        g.albedo = float3(0.075, 0.075, 0.08) * (0.8 + 0.3 * fine + 0.2 * (large - 0.5));
        g.roughness = 0.82;
        g.height = fine * 0.5 + nC.r * 0.2;
        g.bump = 0.45;
    } else { // MAT_LOWGRIP: sealed slab, smooth
        g.albedo = float3(0.075, 0.095, 0.12) * (0.9 + 0.2 * large);
        g.roughness = 0.30 + 0.1 * fine;
        g.height = fine * 0.1;
        g.bump = 0.2;
    }
    // Wet ground near the water (dark, glossy, flatter).
    const float wet = mask.b * (mat == MAT_ROCK ? 0.5 : 1.0);
    g.albedo *= lerp(1.0, 0.6, wet);
    g.roughness = lerp(g.roughness, 0.38, wet * 0.8);
    g.bump *= 1.0 - 0.5 * wet;
    return g;
}

GroundSurface mixGround(GroundSurface a, GroundSurface b, float t) {
    GroundSurface r;
    r.albedo = lerp(a.albedo, b.albedo, t);
    r.roughness = lerp(a.roughness, b.roughness, t);
    r.height = lerp(a.height, b.height, t);
    r.bump = lerp(a.bump, b.bump, t);
    return r;
}

// Asphalt (levels with asphaltLayer: the snow splat weight): dark grey aggregate with a fine speckle, little bump.
float3 asphaltAlbedo(float3 P, float speckle) {
    return float3(0.068, 0.068, 0.072) * (0.8 + 0.4 * speckle);
}

// Level terrain: packed dirt / grass (from the mask) under rock, sand and snow (or asphalt) by the baked splat
// weights. The edges are sharpened with noise so the layers do not blur into each other over the 1 m splat cells.
GroundSurface terrainMaterial(float3 w, float road, float3 P, float3 N, float4 mask, Texture2D<float4> noiseTex, SamplerState ns,
                              float far, float asphalt) {
    const float2 nz = noiseTex.Sample(ns, P.xz * 0.13).rg;
    const float rock = saturate((w.x - 0.5) * 2.2 + 0.5 + (nz.r - 0.5) * 0.6) * step(0.01, w.x);
    const float sand = saturate((w.y - 0.5) * 1.6 + 0.5 + (nz.g - 0.5) * 0.4) * step(0.01, w.y);
    const float snow = saturate((w.z - 0.5) * 2.6 + 0.5 + (nz.g - 0.5) * 0.7) * step(0.01, w.z);
    GroundSurface g = groundMaterial(MAT_DIRT, P, N, mask, noiseTex, ns, far);
    // Road paint: lighter, sun-bleached packed dirt with a smoother, compacted top (edges broken up with noise).
    const float paint = saturate(road * 1.3 - 0.15 + (nz.r - 0.5) * 0.35);
    if (paint > 0.0) {
        const float3 packed = float3(0.300, 0.225, 0.150) * (0.85 + 0.3 * nz.g);
        g.albedo = lerp(g.albedo, lerp(g.albedo, packed, 0.7), paint);
        g.roughness = lerp(g.roughness, 0.86, paint);
        g.bump *= 1.0 - 0.4 * paint;
    }
    if (sand > 0.0) g = mixGround(g, groundMaterial(MAT_SAND, P, N, mask, noiseTex, ns, far), sand);
    if (rock > 0.0) g = mixGround(g, groundMaterial(MAT_ROCK, P, N, mask, noiseTex, ns, far), rock);
    if (snow > 0.0 && asphalt > 0.5) {
        const float sharp = saturate((w.z - 0.5) * 6.0 + 0.5) * step(0.01, w.z);
        const float speckle = noiseTex.Sample(ns, P.xz * 3.1).r;
        g.albedo = lerp(g.albedo, asphaltAlbedo(P, speckle), sharp);
        g.roughness = lerp(g.roughness, 0.78, sharp);
        g.bump *= 1.0 - 0.85 * sharp;
        g.height = lerp(g.height, 0.5, sharp);
    } else if (snow > 0.0) {
        g = mixGround(g, groundMaterial(MAT_SNOW, P, N, mask, noiseTex, ns, far), snow);
    }
    return g;
}

// Soil carried in the tread (TreadSoil, cosmetic): the grooves and the tread face load first, then the outer
// sidewall; the rim catches only a little. Wheel-local space: axle along X, radius in YZ.
// Poorly sticking soil (dry sand, stick ~0.1) only fills the grooves and the sidewall base and stays a thin film.
float treadSoilCover(float4 soil, float stick, float radius, float3 lp, float3 ln, Texture2D<float4> noiseTex, SamplerState ns) {
    if (soil.a <= 0.002 || radius <= 0.0) return 0.0;
    const float r = length(lp.yz) / radius;
    const float radial = abs(dot(normalize(ln), float3(0.0, normalize(lp.yz + 1e-5))));
    const float ang = atan2(lp.y, lp.z);
    const float n = noiseTex.Sample(ns, float2(ang * 1.9, lp.x * 6.0)).g;
    const float sp = noiseTex.Sample(ns, float2(ang * 7.0, lp.x * 23.0 + r * 9.0)).a;
    // Need: how much soil a spot needs before it is covered (0 = first to fill).
    const float face = smoothstep(0.90, 0.97, r) * smoothstep(0.35, 0.8, radial);
    const float groove = face * (1.0 - smoothstep(0.975, 0.995, r)); // recessed between the lugs
    const float faceNeed = 0.12 + 0.3 * (1.0 - saturate(stick));       // lug tops: sand needs far more than mud
    float side = 0.35 + 1.6 * saturate((0.97 - r) / 0.35);
    side = min(side, 0.08 + 3.0 * max(0.95 - r, 0.0));                  // sidewall base next to the shoulder
    float need = lerp(side, faceNeed, face) - faceNeed * groove;
    need = r < 0.62 ? 0.92 : need; // rim
    const float t = soil.a - need + (n - 0.5) * 0.3 + (sp - 0.5) * 0.25;
    return smoothstep(0.0, 0.08, t) * (r < 0.62 ? 0.5 : 1.0);
}

// Cosmetic mud on the vehicle (object-space noise). Body: a splash line that rises with the amount, thrown up
// from below; wheels (radial = 1): caked tread and lower rim. No effect on the simulation.
float mudCover(float amount, float receptivity, float radial, float3 lp, float3 ln, Texture2D<float4> noiseTex, SamplerState ns) {
    if (amount * receptivity <= 0.001) return 0.0;
    const float3 w = abs(ln) / (abs(ln).x + abs(ln).y + abs(ln).z + 1e-4);
    // Streaky splash noise (stretched vertically) plus fine droplets.
    const float n = noiseTex.Sample(ns, float2(lp.z * 1.6, lp.y * 0.6)).g * w.x + noiseTex.Sample(ns, lp.xz * 1.6).g * w.y +
                    noiseTex.Sample(ns, float2(lp.x * 1.6, lp.y * 0.6)).g * w.z;
    const float sp = noiseTex.Sample(ns, lp.zy * 7.3 + lp.x).a * w.x + noiseTex.Sample(ns, lp.xz * 7.3).a * w.y +
                     noiseTex.Sample(ns, lp.xy * 7.3).a * w.z;
    float height;
    if (radial > 0.5) {
        height = 1.0 - saturate(length(lp.yz) / 0.42); // 0 on the tread, 1 at the hub
    } else {
        height = saturate((lp.y + 0.45) / 1.6);        // 0 under the sills, 1 at the roof
    }
    const float t = amount * receptivity * 0.85 - height + (n - 0.5) * 0.35 + (sp - 0.5) * 0.45;
    return smoothstep(0.0, 0.06, t);
}

// Spray mud on the body (BodyMud regions, cosmetic). Body space: +X left, +Z forward; the arches, body side and
// bumper zones come from the vehicle model (f.bodyMudArch / bodyMudBody; distances below are for a 0.545 m arch and
// scale with it). Each region has a "need" field (how much coat a spot needs
// before it is covered, 0 = first): the rear part of an arch fills from the liner facing the tire, the panel behind a
// wheel from the sill upward, bumpers from the bottom edge. Returns rgb + cover, the wetness and grass flecks.
struct BodyMudAcc {
    float k, jitter;
    float best, sum, wet, flecks;
    float3 col;
};

void bodyMudAdd(inout BodyMudAcc acc, int r, float need) {
    const float a = f.bodyMudAmount[r / 4][r % 4];
    if (a <= 0.002) return;
    const float c = smoothstep(0.0, 0.07, a * acc.k - need + acc.jitter);
    if (c <= 0.0) return;
    acc.best = max(acc.best, c);
    acc.sum += c;
    acc.col += f.bodyMudColor[r].rgb * c;
    acc.wet += f.bodyMudColor[r].w * c;
    acc.flecks += f.bodyMudFlecks[r / 4][r % 4] * c;
}

float4 bodyMudCover(float3 P, float3 Nw, float receptivity, Texture2D<float4> noiseTex,
                   SamplerState ns, inout float wet, inout float flecks) {
    if (receptivity <= 0.001) return ((float4)(0.0));
    const float3 p = (mul(f.vehicleInv, float4(P, 1.0))).xyz;
    if (abs(p.x) > 1.15 || p.y > 0.9 || abs(p.z) > 2.6) return ((float4)(0.0));
    const float3 nb = normalize((mul(f.vehicleInv, float4(Nw, 0.0))).xyz);
    const float3 w3 = abs(nb) / (abs(nb).x + abs(nb).y + abs(nb).z + 1e-4);
    const float n = noiseTex.Sample(ns, float2(p.z * 1.9, p.y * 1.1)).g * w3.x + noiseTex.Sample(ns, p.xz * 1.9).g * w3.y +
                    noiseTex.Sample(ns, float2(p.x * 1.9, p.y * 1.1)).g * w3.z;
    const float sp = noiseTex.Sample(ns, p.zy * 9.1 + p.x).a * w3.x + noiseTex.Sample(ns, p.xz * 9.1).a * w3.y +
                     noiseTex.Sample(ns, p.xy * 9.1).a * w3.z;
    const float jitter = (n - 0.5) * 0.35 + (sp - 0.5) * 0.5;
    BodyMudAcc acc;
    acc.k = receptivity;
    acc.jitter = jitter;
    acc.best = 0.0;
    acc.sum = 0.0;
    acc.wet = 0.0;
    acc.flecks = 0.0;
    acc.col = ((float3)(0.0));
    const float archScale = 0.545 / f.bodyMudArch.w;
    const float skin = f.bodyMudBody.x;
    for (int w = 0; w < 4; ++w) {
        const float sx = (w % 2 == 0) ? 1.0 : -1.0; // FL / RL on the left (+X)
        const float zc = w < 2 ? f.bodyMudArch.x : f.bodyMudArch.y;
        if (p.x * sx < skin - 0.58) continue;
        const float2 d2 = float2(p.z - zc, p.y - f.bodyMudArch.z);
        const float d = length(d2) * archScale;
        const float facing = saturate(dot(nb, normalize(float3(sx * (skin + 0.02), f.bodyMudArch.z, zc) - p)));
        if (d > 0.38 && d < 1.0) {
            // Arch: angle 0 straight behind (or ahead) of the axle, pi/2 at the top; spray hits ~25 deg up first.
            const float ring = 1.8 * saturate((d - 0.60) / 0.45) + 0.35 * (1.0 - facing);
            const float angR = atan2(d2.y, -d2.x), angF = atan2(d2.y, d2.x);
            bodyMudAdd(acc, kMudArchRearShader + w, 0.04 + 0.8 * pow(saturate(abs(angR - 0.45) / 1.3), 1.5) + ring);
            bodyMudAdd(acc, kMudArchFrontShader + w, 0.04 + 0.8 * pow(saturate(abs(angF - 0.45) / 1.3), 1.5) + ring);
        }
        // Panel behind the wheel: from the arch edge backward (front: sill and lower rear door; rear: rear quarter).
        const float len = w < 2 ? 1.3 : 0.45;
        const float t = ((zc - 0.50) - p.z) / len;
        if (t > -0.15 && t < 1.2 && p.x * sx > skin - 0.43)
            bodyMudAdd(acc, kMudBehindShader + w, 0.08 + 1.3 * saturate((p.y - f.bodyMudArch.z - 0.08) / 0.75) + 0.5 * saturate(t) +
                                          2.0 * saturate(t - 1.0));
    }
    if (p.z < f.bodyMudBody.z) bodyMudAdd(acc, kMudRearBumperShader, 0.10 + 1.4 * saturate((p.y - f.bodyMudArch.z - 0.18) / 0.55) + (nb.z > -0.3 ? 0.15 : 0.0));
    if (p.z > f.bodyMudBody.y) bodyMudAdd(acc, kMudFrontBumperShader, 0.10 + 1.4 * saturate((p.y - f.bodyMudArch.z - 0.18) / 0.55) + (nb.z < 0.3 ? 0.15 : 0.0));
    if (acc.best <= 0.0) return ((float4)(0.0));
    wet = acc.wet / acc.sum;
    flecks = acc.flecks / acc.sum;
    return float4(acc.col / acc.sum, acc.best);
}

#if defined(TF_ENTRY_lit_fs)
Texture2DArray<float> shadowMap : TEX(0);
SamplerComparisonState shadowSampler : SMP(0);
Texture2D<float4> noiseTex : TEX(1);
SamplerState noiseSampler : SMP(1);
Texture2D<float4> mask : TEX(2);
SamplerState maskSampler : SMP(2);
TextureCube<float4> envProbe : TEX(3);
SamplerState envProbeSampler : SMP(3);

// The vehicle's local environment probe (a cube rendered around it without the vehicle, plain box-filtered mips):
// diffuse from the two coarsest mips along N, specular from the mip that matches the roughness along R.
EnvLight probeEnv(float3 N, float3 R, float roughness) {
    EnvLight e;
    e.ambient = envProbe.SampleLevel(envProbeSampler, N, f.envProbe.y).rgb;
    e.coat = envProbe.SampleLevel(envProbeSampler, R, 0.5).rgb;
    e.specular = envProbe.SampleLevel(envProbeSampler, R, sqrt(saturate(roughness)) * f.envProbe.z).rgb;
    return e;
}

float4 lit_fs(LitOut vin, bool front : SV_IsFrontFace) : SV_Target0 {
    if (dot(f.clipPlane.xyz, vin.worldPos) + f.clipPlane.w < 0.0) discard;
    // Tree LOD cross-fade (mudMode < 0, see tree_vs): the nearer LOD keeps the pixels with dither < k, the farther one
    // the rest, so both together cover the tree once while it moves through the band.
    if (vin.mudMode < 0.0) {
        const float code = -vin.mudMode;
        const float d = frac(52.9829189 * frac(dot(vin.position.xy, float2(0.06711056, 0.00583715))));
        if (code < 1.5 ? d >= code : d < code - 2.0) discard;
    }
    const int mat = int(vin.material.x + 0.5);
    float3 N = normalize(vin.normal);
    const float3 V = normalize(f.cameraPos.xyz - vin.worldPos);
    const float3 P = vin.worldPos;

    ShadeInput s;
    s.albedo = vin.color;
    s.roughness = vin.material.y;
    s.metallic = vin.material.z;
    s.ao = 1.0;
    s.specOcclusion = 1.0;
    s.emissive = ((float3)(0.0));
    s.clearCoat = 0.0;

    const bool ground = (mat >= MAT_DIRT && mat <= MAT_GRASS) || mat == MAT_SAND || mat == MAT_SNOW || mat == MAT_TERRAIN;
    if (ground) {
        const float4 m = sampleMask(mask, maskSampler, P.xz);
        // if / else, not ?: — DXC rejects a conditional operator with a struct result.
        GroundSurface g;
        if (mat == MAT_TERRAIN) g = terrainMaterial(vin.color, vin.localNormal.x, P, N, m, noiseTex, noiseSampler, grassFar(P), 0.0);
        else g = groundMaterial(mat, P, N, m, noiseTex, noiseSampler, grassFar(P));
        s.albedo = g.albedo;
        s.roughness = g.roughness;
        s.metallic = 0.0;
        N = perturbNormal(N, P, g.height * 0.035, g.bump);
        s.ao = vehicleAO(P);
        s.specOcclusion = 0.75; // clumps and blades occlude part of the sky reflection
        if (f.options.x > 0.0) {
            const float2 cell = abs(frac(P.xz) - 0.5);
            s.albedo *= 1.0 - f.options.x * smoothstep(0.47, 0.5, max(cell.x, cell.y));
        }
    } else {
        if (!front) N = -N;
        if (mat == MAT_GLASS) {
            // Dark tinted, nearly opaque glass with no interior behind it: a full-strength sky reflection would turn
            // the windows into light blue panels.
            s.roughness = 0.03;
            s.specOcclusion = 0.3;
        } else if (mat == MAT_HEADLIGHT) {
            s.emissive = vin.color * (0.6 + 7.0 * f.vehicleParams.w);
            s.albedo = ((float3)(0.8));
            s.roughness = 0.15;
        } else if (mat == MAT_BRAKE) {
            s.emissive = vin.color * (0.9 + 9.0 * f.vehicleParams.y);
            s.albedo = vin.color * 0.3;
        } else if (mat == MAT_REVERSE) {
            s.emissive = vin.color * (6.0 * f.vehicleParams.z);
            s.albedo = ((float3)(0.6));
        } else if (mat == MAT_AMBER) {
            s.emissive = vin.color * 1.2;
            s.albedo = vin.color * 0.4;
        } else if (mat == MAT_FOLIAGE) {
            const float n = noiseTex.Sample(noiseSampler, P.xz * 0.35 + P.y * 0.2).r;
            s.albedo = vin.color * (0.7 + 0.6 * n);
            s.roughness = 0.9;
            // Light through the needles when backlit.
            s.emissive = s.albedo * f.sunColor.rgb * 0.08 * pow(saturate(dot(-V, f.sunDir.xyz)), 3.0);
        } else if (mat == MAT_PAINT) {
            // Solid (flake-free) paint under a clear coat.
            s.clearCoat = 1.0;
        } else if (mat == MAT_RUBBER) {
            const float n = noiseTex.Sample(noiseSampler, vin.localPos.yz * 6.0).a;
            s.albedo *= 0.85 + 0.3 * n;
        } else if (mat == MAT_ASPHALT) {
            // Asphalt, road paint, curbs and gravel: the vertex color broken up by fine aggregate and coarse patches.
            const float grain = noiseTex.Sample(noiseSampler, P.xz * 2.3).a;
            const float patches = noiseTex.Sample(noiseSampler, P.xz * 0.07).r;
            s.albedo *= 0.80 + 0.30 * grain + 0.25 * (patches - 0.5);
            s.roughness = saturate(s.roughness + 0.15 * (grain - 0.5));
            N = perturbNormal(N, P, grain * 0.02, 0.3);
            s.ao = vehicleAO(P);
        } else if (mat == MAT_HORIZON) {
            // Per pixel, so the long radial ring quads do not smear a per-vertex palette into streaks: grass and
            // forest patches tinted to the mean tone of the bake edge (vertex colour; the tint fades out from 1.5 to 6 km),
            // rock on steep or high ground, snow on gentle faces above the level's snow line (material z), sand at
            // the waterline. Within ~900 m of the bake edge (along a noisy line) it starts from the edge tone itself,
            // so the ring begins in the colour the baked ground ends with.
            const float n1 = noiseTex.Sample(noiseSampler, P.xz / 5600.0).r * 0.65 + noiseTex.Sample(noiseSampler, P.xz / 1900.0).g * 0.35;
            const float n2 = noiseTex.Sample(noiseSampler, P.xz / 2100.0 + 0.37).g * 0.6 + noiseTex.Sample(noiseSampler, P.xz / 640.0).r * 0.4;
            const float n3 = noiseTex.Sample(noiseSampler, P.xz / 230.0 + 0.61).r;
            const float slope = 1.0 - N.y;
            const float r = max(abs(P.x), abs(P.z)) + f.groundRect.x; // past the bake edge
            float3 c = lerp(float3(0.085, 0.092, 0.040), float3(0.150, 0.122, 0.062), smoothstep(0.35, 0.75, n1));
            c = lerp(c, float3(0.045, 0.060, 0.030), 0.7 * smoothstep(0.50, 0.70, n2)); // forest
            // Tree crowns and clearings as dark specks (the bake's forest seen from above); where the specks are
            // below a pixel the mips would average them away, so they turn into their mean darkening instead.
            const float4 fine = noiseTex.Sample(noiseSampler, P.xz / 41.0 + 0.23);
            const float specks = smoothstep(0.52, 0.68, fine.a);
            const float speckMip = saturate(length(fwidth(P.xz)) / 41.0 * 3.0);
            c *= 1.0 - 0.45 * lerp(specks, 0.3, speckMip) * smoothstep(0.30, 0.60, n2 + 0.2 * n1);
            const float3 paletteMean = float3(0.098, 0.093, 0.045);
            c *= lerp(clamp(vin.color / paletteMean, 0.5, 2.0), (float3)1.0, smoothstep(1500.0, 6000.0, r));
            c = lerp(vin.color * (0.80 + 0.40 * n2), c, 0.6 * smoothstep(0.0, 300.0, r + (n2 - 0.5) * 400.0) + 0.4 * smoothstep(100.0, 900.0, r));
            const float3 rockC = lerp(float3(0.115, 0.105, 0.095), float3(0.205, 0.190, 0.170), n2);
            c = lerp(c, rockC, max(smoothstep(0.18, 0.32, slope), smoothstep(380.0, 560.0, P.y)));
            // Snow fields break up into rock and scree along a noisy line instead of fading over a wide band; the
            // far-view albedo of the bake's textured snow.
            const float n4 = fine.g;
            const float snowLine = vin.material.z + 60.0 * (n2 - 0.5) + 50.0 * (n3 - 0.5) + 24.0 * (n4 - 0.5);
            const float snow = smoothstep(snowLine, snowLine + 30.0, P.y) *
                               (1.0 - smoothstep(0.20, 0.30, slope + 0.12 * (n3 - 0.5) + 0.10 * (n4 - 0.5)));
            c = lerp(c, float3(0.38, 0.39, 0.42) * (0.85 + 0.2 * n3 + 0.1 * n4), snow);
            c = lerp(c, float3(0.36, 0.31, 0.21), 1.0 - smoothstep(1.5, 5.0, P.y));
            s.albedo = c;
            s.metallic = 0.0;
        } else if (mat == MAT_CLADDING) {
            // Stippled plastic: noise fixed to the vehicle, projected along the dominant local axis.
            const float3 an = abs(vin.localNormal);
            const float2 uv = an.x > max(an.y, an.z) ? vin.localPos.zy : (an.y > an.z ? vin.localPos.xz : vin.localPos.xy);
            const float g = noiseTex.Sample(noiseSampler, uv * 9.0).a;
            s.albedo *= 0.86 + 0.28 * g;
            s.roughness = saturate(s.roughness + 0.1 * (g - 0.5));
            N = perturbNormal(N, P, g, 0.002);
        }
        // Cosmetic mud layer (no effect on physics).
        const float mud = mudCover(f.vehicleParams.x, vin.material.w, vin.mudMode, vin.localPos, vin.localNormal, noiseTex,
                                   noiseSampler);
        if (mud > 0.0) {
            const float wetMud = noiseTex.Sample(noiseSampler, vin.localPos.zy * 1.7 + 0.3).g;
            const float3 mudC = lerp(float3(0.13, 0.090, 0.055), float3(0.050, 0.034, 0.020), smoothstep(0.4, 0.7, wetMud));
            s.albedo = lerp(s.albedo, mudC, mud);
            s.roughness = lerp(s.roughness, lerp(0.95, 0.35, smoothstep(0.4, 0.7, wetMud)), mud);
            s.metallic = lerp(s.metallic, 0.0, mud);
            s.clearCoat *= 1.0 - mud;
            s.emissive *= 1.0 - 0.85 * mud;
        }
        if (vin.mudMode > 0.5) {
            const float stick = vin.soilExtra.y;
            // Dry, poorly sticking soil is a thin dusting: the black rubber shows through.
            const float soilC = treadSoilCover(vin.soil, stick, vin.soilParams.y, vin.localPos, vin.localNormal, noiseTex,
                                               noiseSampler) * lerp(0.45, 1.0, saturate(stick));
            if (soilC > 0.0) {
                const float grain = noiseTex.Sample(noiseSampler, vin.localPos.zy * 9.0 + vin.localPos.x).a;
                const float wetS = vin.soilParams.x;
                s.albedo = lerp(s.albedo, vin.soil.rgb * (0.8 + 0.4 * grain), soilC);
                s.roughness = lerp(s.roughness, lerp(0.95, 0.35, wetS), soilC);
                s.metallic = lerp(s.metallic, 0.0, soilC);
                s.clearCoat *= 1.0 - soilC;
            }
            // A few torn grass blades stuck in the tread and on the shoulder (sparse specks, never a green tint).
            const float r = length(vin.localPos.yz) / max(vin.soilParams.y, 1e-3);
            if (vin.soilExtra.x > 0.01 && r > 0.88 && r < 1.02) {
                const float ang = atan2(vin.localPos.y, vin.localPos.z);
                const float fn = noiseTex.Sample(noiseSampler, float2(ang * 13.0, vin.localPos.x * 37.0)).a;
                const float fleck = smoothstep(0.0, 0.02, fn - (1.0 - 0.09 * vin.soilExtra.x));
                s.albedo = lerp(s.albedo, float3(0.05, 0.10, 0.02), fleck);
                s.roughness = lerp(s.roughness, 0.7, fleck);
            }
        } else {
            // Spray mud per body region (BodyMud): builds up where the wheels throw it.
            float wetB = 0.0, fleckB = 0.0;
            const float4 bm = bodyMudCover(P, normalize(vin.normal), vin.material.w, noiseTex, noiseSampler, wetB, fleckB);
            if (bm.a > 0.0) {
                // A coat on paint reads lighter than the soil in bulk (thin film, light scattered in it); dried lighter.
                float3 c = min(bm.rgb * lerp(4.0, 2.5, wetB), bm.rgb + 0.12);
                if (fleckB > 0.01) {
                    const float3 bp = (mul(f.vehicleInv, float4(P, 1.0))).xyz;
                    const float fn = noiseTex.Sample(noiseSampler, bp.zy * 21.0 + bp.x * 13.0).a;
                    c = lerp(c, float3(0.05, 0.10, 0.02), smoothstep(0.0, 0.02, fn - (1.0 - 0.07 * fleckB)));
                }
                s.albedo = lerp(s.albedo, c, bm.a);
                s.roughness = lerp(s.roughness, lerp(0.95, 0.3, wetB), bm.a);
                s.metallic = lerp(s.metallic, 0.0, bm.a);
                s.clearCoat *= 1.0 - bm.a;
                s.emissive *= 1.0 - 0.85 * bm.a;
            }
        }
        s.ao = 1.0;
    }
    s.N = N;
    s.V = V;
    s.lamp = headlightIrradiance(P, N);
    s.shadow = shadowFactor(shadowMap, shadowSampler, P, normalize(vin.normal) * (front ? 1.0 : -1.0)) * cloudShadow(P);
    const float3 R = reflect(-V, N);
    EnvLight e;
    if (vin.soilExtra.z > 0.5 && f.envProbe.x > 0.5) e = probeEnv(N, R, s.roughness);
    else e = skyEnv(N, R, s.roughness);
    float3 direct, indirect;
    const float3 color = shadeSplitEnv(s, e, direct, indirect);
    return fogSplit(color, direct, indirect, P);
}
#endif

// ---------------------------------------------------------------- level terrain (textured)
// Six CC0 material layers (assets/textures/terrain, src/core/TerrainTextureSet.h order) in two BC7 texture arrays:
// albedo (sRGB) + height in alpha, and normal x / y (DirectX convention) + roughness + AO. Layers are blended by
// the baked splat and the ground mask, sharpened by their heights. Without the textures (repository without
// `git lfs pull`) the procedural terrainMaterial is used.

#if defined(TF_ENTRY_terrain_fs)
Texture2DArray<float> shadowMap : TEX(0);
SamplerComparisonState shadowSampler : SMP(0);
Texture2D<float4> noiseTex : TEX(1);
SamplerState noiseSampler : SMP(1);
Texture2D<float4> mask : TEX(2);
SamplerState maskSampler : SMP(2);
#endif

// Layer sampling, shared with mark_fs (tire marks over the level ground).
#if defined(TF_ENTRY_terrain_fs) || defined(TF_ENTRY_mark_fs)
Texture2DArray<float4> layerAlbedo : TEX(3);
SamplerState layerAlbedoSampler : SMP(3);
Texture2DArray<float4> layerNra : TEX(4);
SamplerState layerNraSampler : SMP(4);

// Must match TerrainTextureUniforms in src/render/Renderer.h.
struct TerrainTextureUniforms {
    float4 invTile[2]; // 1 / repeat size (m): grass, soil, road, rock | sand, snow
    float4 tint[6];    // rgb albedo multiplier per layer
    float4 params;     // x: textures loaded (0 = procedural fallback), y: normal map strength, z: snow weight is asphalt
};
#if defined(TF_ENTRY_mark_fs)
ConstantBuffer<TerrainTextureUniforms> tt : UBO(2);
#else
ConstantBuffer<TerrainTextureUniforms> tt : UBO(1);
#endif

static const int LAYER_GRASS = 0;
static const int LAYER_SOIL = 1;
static const int LAYER_ROAD = 2;
static const int LAYER_ROCK = 3;
static const int LAYER_SAND = 4;
static const int LAYER_SNOW = 5;

struct LayerSample {
    float3 albedo;
    float height;
    float3 dn; // world-space normal offset (added to the geometric normal)
    float rough;
    float ao;
};

float layerInvTile(int i) { return i < 4 ? tt.invTile[0][i] : tt.invTile[1][i - 4]; }

// One planar projection: uv in repeats, gx / gy its screen derivatives, u / v the world directions of the texture
// axes. far: 0..1 from ~15 m, fade: 0..1 from ~60 m to ~250 m, where the layer turns into its mean (the 1x1 mip):
// seen from hundreds of meters the repeats of a 2.5 m tile line up into a regular pattern even in the small mips.
LayerSample sampleProjection(int layer, float2 uv, float2 gx, float2 gy, float3 u, float3 v, float far, float fade) {
    LayerSample r;
    const float3 at = float3(uv, float(layer));
    float4 a;
    float4 n;
    if (fade >= 1.0) {
        a = layerAlbedo.SampleLevel(layerAlbedoSampler, at, 16.0);
        n = layerNra.SampleLevel(layerNraSampler, at, 16.0);
    } else {
        a = layerAlbedo.SampleGrad(layerAlbedoSampler, at, gx, gy);
        // Beyond ~15 m a second, rotated sample at 0.29 x the frequency breaks up the repeats (near the camera
        // only a few repeats are in view).
        if (far > 0.0) {
            const float2x2 m = float2x2(0.81, -0.59, 0.59, 0.81) * 0.29;
            a = lerp(a, layerAlbedo.SampleGrad(layerAlbedoSampler, float3(mul(m, uv) + 0.37, float(layer)), mul(m, gx),
                                               mul(m, gy)), 0.55 * far);
        }
        n = layerNra.SampleGrad(layerNraSampler, at, gx, gy);
        if (fade > 0.0) {
            a = lerp(a, layerAlbedo.SampleLevel(layerAlbedoSampler, at, 16.0), fade);
            n = lerp(n, layerNra.SampleLevel(layerNraSampler, at, 16.0), fade);
        }
    }
    r.albedo = a.rgb * tt.tint[layer].rgb;
    r.height = lerp(a.a, 0.5, fade); // mean heights differ per layer and would tip the far blend
    const float2 nxy = n.rg * 2.0 - 1.0;
    r.dn = (u * nxy.x + v * nxy.y) * tt.params.y; // DirectX normal map: +y points along +v (down the image)
    r.rough = n.b;
    r.ao = n.a;
    return r;
}

// Hex tiling (Mikkelsen 2022, "Practical Real-Time Hex-Tiling"): a triangle grid whose vertices are the centres of
// hexagonal patches; each patch shows the texture rotated about its centre and shifted by a random amount, and a
// fragment blends the (at most three) patches of its triangle with barycentric weights to the 7th power, so most
// of a patch is one lookup and the blends are narrow. The grid is in world units on the projection plane, shared
// by every layer (each layer adds its own shift per patch). Normal maps are rotated back with their patch.
static const float kHexSpacing = 3.0; // m between patch centres
struct HexTap {
    float2 cen; // patch centre on the plane (m)
    float2 cs;  // cos / sin of the patch rotation
    float2 off; // random shift (repeats)
};
struct HexFrame {
    HexTap t[3];
    float3 w; // sharpened weights, sum 1
    HexTap d; // tap with the largest weight
};

float2 hash22(float2 p) {
    float3 p3 = frac(float3(p.xyx) * float3(0.1031, 0.1030, 0.0973));
    p3 += dot(p3, p3.yzx + 33.33);
    return frac((p3.xx + p3.yz) * p3.zy);
}

// cut: weights below it drop to 0 (continuously), so more of the plane is a single lookup; it grows with distance,
// where the narrower blend bands are smaller than the mip footprint.
HexFrame hexFrame(float2 xz, float rotStrength, float cut) {
    const float2 st = xz / kHexSpacing;
    const float2 skew = float2(st.x - 0.57735027 * st.y, 1.15470054 * st.y);
    const float2 base = floor(skew);
    const float2 fr = skew - base;
    const float z = 1.0 - fr.x - fr.y;
    const float s = z < 0.0 ? 1.0 : 0.0;
    const float s2 = 2.0 * s - 1.0;
    const float3 bw = float3(-z * s2, s - fr.y * s2, s - fr.x * s2);
    const float2 v[3] = {base + float2(s, s), base + float2(s, 1.0 - s), base + float2(1.0 - s, s)};
    HexFrame h;
    float3 w = pow(saturate(bw), 7.0);
    w /= w.x + w.y + w.z;
    w = saturate((w - cut) / (1.0 - cut));
    h.w = w / (w.x + w.y + w.z);
    [unroll] for (int i = 0; i < 3; ++i) {
        const float2 r = hash22(v[i] + 0.5);
        const float a = (r.x - 0.5) * 6.2831853 * rotStrength;
        h.t[i].cen = float2(v[i].x + 0.5 * v[i].y, v[i].y * 0.8660254) * kHexSpacing;
        h.t[i].cs = float2(cos(a), sin(a));
        h.t[i].off = float2(r.y, hash22(v[i] * 1.37 + 5.1).x * 0.71 + r.x * 0.29);
    }
    h.d = h.t[2];
    if (h.w.y >= h.w.z) h.d = h.t[1];
    if (h.w.x >= h.w.y && h.w.x >= h.w.z) h.d = h.t[0];
    return h;
}

float2 hexRot(HexTap t, float2 q) { return float2(t.cs.x * q.x - t.cs.y * q.y, t.cs.y * q.x + t.cs.x * q.y); }
float2 hexUnrot(HexTap t, float2 q) { return float2(t.cs.x * q.x + t.cs.y * q.y, -t.cs.y * q.x + t.cs.x * q.y); }
// Texture coordinate of plane position p in a patch (k: repeats per meter, layer: its own shift).
float2 hexUV(HexTap t, float2 p, float k, int layer) {
    return hexRot(t, p - t.cen) * k + frac(t.off + float2(0.618, 0.382) * float(layer));
}

// sampleProjection through the hex patches: p is the position on the plane (m), gx / gy its screen derivatives.
LayerSample sampleHex(int layer, HexFrame h, float2 p, float2 gx, float2 gy, float k, float3 u, float3 v, float fade) {
    LayerSample r;
    float4 a = (float4)0.0;
    float2 nxy = (float2)0.0;
    float2 ra = (float2)0.0;
    if (fade >= 1.0) {
        a = layerAlbedo.SampleLevel(layerAlbedoSampler, float3(0.5, 0.5, float(layer)), 16.0);
        const float4 n = layerNra.SampleLevel(layerNraSampler, float3(0.5, 0.5, float(layer)), 16.0);
        nxy = n.rg * 2.0 - 1.0;
        ra = n.ba;
    } else {
        float wsum = 0.0;
        [unroll] for (int i = 0; i < 3; ++i) {
            if (h.w[i] > 0.0) {
                const HexTap t = h.t[i];
                const float3 at = float3(hexUV(t, p, k, layer), float(layer));
                const float2 tgx = hexRot(t, gx) * k, tgy = hexRot(t, gy) * k;
                const float4 ai = layerAlbedo.SampleGrad(layerAlbedoSampler, at, tgx, tgy);
                const float4 ni = layerNra.SampleGrad(layerNraSampler, at, tgx, tgy);
                // Mikkelsen's luminance term: brighter texels win in the blend (no washed-out mix).
                const float wi = h.w[i] * lerp(1.0, sqrt(luminance(ai.rgb)), 0.6);
                a += ai * wi;
                nxy += hexUnrot(t, ni.rg * 2.0 - 1.0) * wi;
                ra += ni.ba * wi;
                wsum += wi;
            }
        }
        const float inv = 1.0 / max(wsum, 1e-5);
        a *= inv;
        nxy *= inv;
        ra *= inv;
        if (fade > 0.0) {
            const float3 at = float3(0.5, 0.5, float(layer));
            a = lerp(a, layerAlbedo.SampleLevel(layerAlbedoSampler, at, 16.0), fade);
            const float4 n = layerNra.SampleLevel(layerNraSampler, at, 16.0);
            nxy = lerp(nxy, n.rg * 2.0 - 1.0, fade);
            ra = lerp(ra, n.ba, fade);
        }
    }
    r.albedo = a.rgb * tt.tint[layer].rgb;
    r.height = lerp(a.a, 0.5, fade);
    r.dn = (u * nxy.x + v * nxy.y) * tt.params.y;
    r.rough = ra.x;
    r.ao = ra.y;
    return r;
}

// Where and how the ground layers are looked up: xz the (parallax-shifted) lookup position, gx / gy the screen
// derivatives of the world position (taken in uniform control flow: the layers are sampled in branches).
struct GroundCtx {
    float2 xz;
    float3 gx, gy;
    float far, fade;
    bool hex;
    float cut;  // hex weight cut (hexFrame)
    HexFrame h; // top-down patches (hex only)
};

GroundCtx groundCtx(float3 P, float dist) {
    GroundCtx c;
    c.xz = P.xz;
    c.gx = ddx(P);
    c.gy = ddy(P);
    c.hex = f.groundFx.x > 1.5;
    c.far = smoothstep(15.0, 120.0, dist);
    c.fade = smoothstep(60.0, 250.0, dist);
    c.cut = lerp(0.03, 0.2, smoothstep(8.0, 60.0, dist));
    c.h = (HexFrame)0;
    if (c.hex) c.h = hexFrame(P.xz, 1.0, c.cut);
    return c;
}

// Ground layers: top-down projection (x -> u, z -> v). scale multiplies the layer's frequency.
LayerSample sampleLayerScaled(int layer, GroundCtx c, float scale) {
    const float k = layerInvTile(layer) * scale;
    if (c.hex) return sampleHex(layer, c.h, c.xz, c.gx.xz, c.gy.xz, k, float3(1, 0, 0), float3(0, 0, 1), c.fade);
    return sampleProjection(layer, c.xz * k, c.gx.xz * k, c.gy.xz * k, float3(1, 0, 0), float3(0, 0, 1), c.far, c.fade);
}

LayerSample sampleLayer(int layer, GroundCtx c) { return sampleLayerScaled(layer, c, 1.0); }

// One projection plane of the rock: p / gx / gy the plane position and its derivatives.
LayerSample sampleRockPlane(GroundCtx c, float2 p, float2 gx, float2 gy, float3 u, float3 v, bool top) {
    const float k = layerInvTile(LAYER_ROCK);
    if (!c.hex) return sampleProjection(LAYER_ROCK, p * k, gx * k, gy * k, u, v, c.far, c.fade);
    // Walls keep the texture nearly upright (strata); the shift alone breaks up the repeats.
    return sampleHex(LAYER_ROCK, top ? c.h : hexFrame(p, 0.08, c.cut), p, gx, gy, k, u, v, c.fade);
}

// Rock: triplanar on steep faces (projections with a negligible weight are skipped). P: the lookup position.
LayerSample sampleRock(GroundCtx c, float3 P, float3 N, float2 warp) {
    float3 w = pow(abs(N), 4.0);
    w /= w.x + w.y + w.z;
    LayerSample r;
    r.albedo = ((float3)(0.0));
    r.height = 0.0;
    r.dn = ((float3)(0.0));
    r.rough = 0.0;
    r.ao = 0.0;
    if (w.y > 0.02) {
        const LayerSample s = sampleRockPlane(c, P.xz, c.gx.xz, c.gy.xz, float3(1, 0, 0), float3(0, 0, 1), true);
        r.albedo += s.albedo * w.y; r.height += s.height * w.y; r.dn += s.dn * w.y; r.rough += s.rough * w.y; r.ao += s.ao * w.y;
    }
    if (w.x > 0.02) {
        const LayerSample s = sampleRockPlane(c, float2(P.z, -P.y) + warp, float2(c.gx.z, -c.gx.y), float2(c.gy.z, -c.gy.y),
                                              float3(0, 0, 1), float3(0, -1, 0), false);
        r.albedo += s.albedo * w.x; r.height += s.height * w.x; r.dn += s.dn * w.x; r.rough += s.rough * w.x; r.ao += s.ao * w.x;
    }
    if (w.z > 0.02) {
        const LayerSample s = sampleRockPlane(c, float2(P.x, -P.y) + warp, float2(c.gx.x, -c.gx.y), float2(c.gy.x, -c.gy.y),
                                              float3(1, 0, 0), float3(0, -1, 0), false);
        r.albedo += s.albedo * w.z; r.height += s.height * w.z; r.dn += s.dn * w.z; r.rough += s.rough * w.z; r.ao += s.ao * w.z;
    }
    const float sum = (w.y > 0.02 ? w.y : 0.0) + (w.x > 0.02 ? w.x : 0.0) + (w.z > 0.02 ? w.z : 0.0);
    const float inv = 1.0 / max(sum, 1e-4);
    r.albedo *= inv; r.height *= inv; r.dn *= inv; r.rough *= inv; r.ao *= inv;
    return r;
}

// Large-scale tone of the ground. Tiling 0: one noise octave (repeats every 91 m); 1 and 2: three octaves at
// unrelated scales (213 m, 53 m, 16 m) vary brightness, hue, saturation and roughness, so neither the layer textures
// nor the noise texture line up over hundreds of meters.
void macroTone(Texture2D<float4> noiseT, SamplerState ns, float2 xz, inout float3 albedo, inout float rough) {
    if (f.groundFx.x < 0.5) {
        albedo *= 0.86 + 0.28 * noiseT.Sample(ns, xz * 0.011).r;
        return;
    }
    const float n1 = noiseT.Sample(ns, xz * 0.0047 + 0.13).r;
    const float n2 = noiseT.Sample(ns, xz * 0.0189 + 0.57).g;
    const float n3 = noiseT.Sample(ns, xz * 0.0613 + 0.29).a;
    const float tone = n1 * 0.5 + n2 * 0.35 + n3 * 0.15;
    albedo *= 0.80 + 0.40 * tone;
    const float hue = saturate((n2 - n1) * 2.0 + 0.5);
    albedo *= lerp(float3(1.05, 1.0, 0.90), float3(0.95, 1.0, 1.07), hue);
    const float l = luminance(albedo);
    albedo = max(lerp((float3)l, albedo, 0.85 + 0.3 * n1), 0.0);
    rough = saturate(rough * lerp(0.90, 1.10, n3));
}

// Blend factor of a layer of weight w over the current surface: the higher one wins near w = 0.5, and the
// result is exactly 0 at w = 0 and 1 at w = 1 (heights are 0..1).
float heightBlend(float w, float hLayer, float hBase) {
    return saturate((w * 2.0 - 1.0 + (hLayer - hBase) * 0.8) * 4.0 + 0.5);
}

void blendLayer(inout LayerSample g, LayerSample l, float t) {
    g.albedo = lerp(g.albedo, l.albedo, t);
    g.height = lerp(g.height, l.height, t);
    g.dn = lerp(g.dn, l.dn, t);
    g.rough = lerp(g.rough, l.rough, t);
    g.ao = lerp(g.ao, l.ao, t);
}

// Wet ground near the water (mask B: 1 under water, ~0.55 next to it; rock takes half). Render-only: grip and the
// surface type do not change. groundFx.z 0: one darkening and gloss for every layer. 1: porous layers (soil, sand)
// darken and saturate more, dense ones (rock) get a glossier film; the low parts of the texture height fill with
// water as the wetness rises toward the water edge (puddles: flat, smooth, mirror the sky through the environment
// lookup of the shading). h: the blended layer height, nzv: 0..1 noise breaking up the water line.
void groundWet(float wetMask, float rock, float road, float sand, float h, float Ny, float nzv, inout float3 albedo,
               inout float rough, inout float flat) {
    const float wetG = wetMask * lerp(1.0, 0.5, rock);
    if (f.groundFx.z < 0.5) {
        albedo *= lerp(1.0, 0.6, wetG);
        rough = lerp(rough, 0.38, wetG * 0.8);
        flat *= 1.0 - 0.5 * wetG;
        return;
    }
    if (wetG <= 0.0) return;
    const float por = lerp(lerp(lerp(0.85, 0.65, road), 0.80, sand), 0.20, rock);
    const float d = wetG * por;
    albedo *= lerp(1.0, 0.40, d);
    const float l = luminance(albedo);
    albedo = max(lerp((float3)l, albedo, 1.0 + 0.3 * d), 0.0);
    rough = lerp(rough, lerp(0.12, 0.34, por), saturate(wetG * 1.1));
    flat *= 1.0 - 0.4 * wetG;
    const float level = saturate((wetG - 0.25) * 2.0) * 0.6 + (nzv - 0.5) * 0.25;
    const float puddle = smoothstep(0.0, 0.06, level - h) * smoothstep(0.88, 0.96, Ny) * (1.0 - rock);
    albedo *= lerp(1.0, 0.8, puddle);
    rough = lerp(rough, 0.04, puddle);
    flat *= 1.0 - puddle;
}

// The layer with the largest weight (parallax, detail normals); -1 where the asphalt wins (no layer of its own).
int groundDominant(float grass, float road, float sand, float rock, float snow, bool asphalt) {
    int dom = grass >= 0.5 ? LAYER_GRASS : LAYER_SOIL;
    float best = max(grass, 1.0 - grass);
    if (road > best) { dom = LAYER_ROAD; best = road; }
    if (sand > best) { dom = LAYER_SAND; best = sand; }
    if (rock > best) { dom = LAYER_ROCK; best = rock; }
    if (snow > best) dom = asphalt ? -1 : LAYER_SNOW;
    return dom;
}

// Detail normals near the camera (groundFx.y): the dominant layer's own normal map at 3.7x its frequency through the
// same hex patches (no seams), added to the layer normal (whiteout: the slopes add), fading out from 4 to 12 m.
// Only the top-down projection: steep ground (rock walls) gets none.
float3 groundDetailDn(int layer, GroundCtx c, float dist, float Ny) {
    const float s = f.groundFx.y * (1.0 - smoothstep(4.0, 12.0, dist)) * smoothstep(0.65, 0.85, Ny);
    if (s <= 0.0 || layer < 0) return (float3)0.0;
    const float k = layerInvTile(layer) * 3.7;
    float2 nxy = (float2)0.0;
    if (c.hex) {
        [unroll] for (int i = 0; i < 3; ++i) {
            if (c.h.w[i] > 0.0) {
                const HexTap t = c.h.t[i];
                const float3 at = float3(hexUV(t, c.xz, k, layer) + 0.31, float(layer));
                const float4 n = layerNra.SampleGrad(layerNraSampler, at, hexRot(t, c.gx.xz) * k, hexRot(t, c.gy.xz) * k);
                nxy += hexUnrot(t, n.rg * 2.0 - 1.0) * c.h.w[i];
            }
        }
    } else {
        // Plain repeats: rotated against the layer so the two scales do not line up.
        const float2x2 m = float2x2(0.80, -0.60, 0.60, 0.80);
        const float4 n = layerNra.SampleGrad(layerNraSampler, float3(mul(m, c.xz) * k + 0.31, float(layer)),
                                             mul(m, c.gx.xz) * k, mul(m, c.gy.xz) * k);
        nxy = mul(n.rg * 2.0 - 1.0, m);
    }
    return float3(nxy.x, 0.0, nxy.y) * (tt.params.y * 0.6 * s);
}
#endif

#if defined(TF_ENTRY_terrain_fs)

// Height (albedo alpha) of one layer at plane position p: plain repeats, or the dominant hex patch of the fragment
// (the marches below stay within a few cm, inside one patch almost everywhere).
float layerHeightAt(int layer, GroundCtx c, float2 p, float k, float2 gx, float2 gy) {
    if (!c.hex) return layerAlbedo.SampleGrad(layerAlbedoSampler, float3(p * k, float(layer)), gx * k, gy * k).a;
    const HexTap t = c.h.d;
    return layerAlbedo.SampleGrad(layerAlbedoSampler, float3(hexUV(t, p, k, layer), float(layer)), hexRot(t, gx) * k,
                                  hexRot(t, gy) * k).a;
}

// Parallax occlusion of the dominant layer's height (albedo alpha) near the camera: the ground is a height field
// pushed down by up to depthM along the projection axis (world y). Returns the world xz shift for every layer's
// lookup (the loop samples with explicit gradients).
float2 parallaxShift(int layer, GroundCtx c, float3 V, float depthM, int steps) {
    const float k = layerInvTile(layer);
    const float2 gx = c.gx.xz, gy = c.gy.xz;
    const float2 stepXZ = -V.xz / max(V.y, 0.35) * (depthM / float(steps));
    const float dStep = 1.0 / float(steps);
    float2 cur = c.xz;
    float depth = 0.0;
    float prevGap = 0.0;
    float gap = 1.0 - layerHeightAt(layer, c, cur, k, gx, gy);
    int i = 0;
    [loop] for (; i < steps && gap > depth; ++i) {
        prevGap = gap - depth;
        cur += stepXZ;
        depth += dStep;
        gap = 1.0 - layerHeightAt(layer, c, cur, k, gx, gy);
    }
    if (i == 0) return float2(0.0, 0.0);
    // Linear refinement between the last step above the surface and the first below it.
    const float after = gap - depth;
    const float t = prevGap / max(prevGap - after, 1e-4);
    return (cur - stepXZ * (1.0 - saturate(t))) - c.xz;
}

float layerHeightLod(int layer, GroundCtx c, float2 p, float k, float lod) {
    const float2 uv = c.hex ? hexUV(c.h.d, p, k, layer) : p * k;
    return layerAlbedo.SampleLevel(layerAlbedoSampler, float3(uv, float(layer)), lod).a;
}

// Micro self-shadowing (groundShadow.x steps): from the (parallax-shifted) lookup position, a short march toward
// the sun through the dominant layer's height field (the same depth as the parallax); a texel above the ray shades
// the point, softer the further along the ray. Low sun makes the ray long and flat, so the shadows grow.
float microShadow(int layer, GroundCtx c, float3 L, float depthM, int steps) {
    const float k = layerInvTile(layer);
    // Trilinear at the mip of the longer screen axis (no anisotropic taps): the shadows are soft anyway.
    float tw, th, tl;
    layerAlbedo.GetDimensions(tw, th, tl);
    const float lod = 0.5 * log2(max(max(dot(c.gx.xz, c.gx.xz), dot(c.gy.xz, c.gy.xz)) * k * k * tw * tw, 1.0));
    const float h0 = layerHeightLod(layer, c, c.xz, k, lod);
    const float span = 1.0 - h0;
    if (span < 0.02) return 1.0;
    const float2 dirXZ = L.xz / max(L.y, 0.15) * depthM;
    float occl = 0.0;
    [loop] for (int j = 1; j <= steps && occl < 1.0; ++j) {
        const float t = span * float(j) / float(steps);
        const float hs = layerHeightLod(layer, c, c.xz + dirXZ * t, k, lod);
        occl = max(occl, (hs - (h0 + t)) * (1.0 - float(j - 1) / float(steps)) * 8.0);
    }
    return 1.0 - saturate(occl);
}

// Ground specular (groundFx.w): the roughness widens with the normal variance inside the pixel (Tokuyoshi and
// Kaplanyan 2019, from the screen derivatives of the shading normal: detail normals, puddle edges and wet rock do not
// sparkle; the mips are Toksvig-filtered offline as well), the environment reflection is occluded by the texture AO
// for the roughness and view angle (Lagarde 2014) instead of a constant, and reflections that point below the
// geometric surface (normal-mapped grazing views) fade out.
void groundSpecular(inout ShadeInput s, float3 Ngeo) {
    const float3 dx = ddx(s.N), dy = ddy(s.N);
    const float variance = 0.25 * (dot(dx, dx) + dot(dy, dy));
    const float a = s.roughness * s.roughness;
    s.roughness = sqrt(sqrt(saturate(a * a + min(2.0 * variance, 0.18))));
    const float NdotV = saturate(dot(s.N, s.V));
    const float ao = max(s.ao, 1e-3);
    const float so = saturate(pow(NdotV + ao, exp2(-16.0 * s.roughness - 1.0)) - 1.0 + ao) / ao;
    const float horizon = saturate(1.0 + 1.1 * dot(reflect(-s.V, s.N), Ngeo));
    s.specOcclusion = so * horizon * horizon;
}

float4 terrain_fs(LitOut vin) : SV_Target0 {
    float3 N = normalize(vin.normal);
    const float3 V = normalize(f.cameraPos.xyz - vin.worldPos);
    const float3 P = vin.worldPos;
    const float4 m = sampleMask(mask, maskSampler, P.xz);
    const float far = grassFar(P);

    ShadeInput s;
    s.metallic = 0.0;
    s.emissive = ((float3)(0.0));
    s.clearCoat = 0.0;
    s.specOcclusion = 0.75;
    s.ao = vehicleAO(P);
    float micro = 1.0; // level ground self-shadowing (direct light only)
    if (tt.params.x < 0.5) {
        const GroundSurface g = terrainMaterial(vin.color, vin.localNormal.x, P, N, m, noiseTex, noiseSampler, far, tt.params.z);
        s.albedo = g.albedo;
        s.roughness = g.roughness;
        N = perturbNormal(N, P, g.height * 0.035, g.bump);
    } else {
        const float dist = length(P - f.cameraPos.xyz);
        GroundCtx gc = groundCtx(P, dist);
        const float fadeTex = smoothstep(60.0, 250.0, dist);
        const float2 nz = noiseTex.Sample(noiseSampler, P.xz * 0.13).rg;
        const float slope = sqrt(max(1.0 - N.y * N.y, 0.0)) / max(N.y, 0.1);
        const GrassZones zones = grassZones(noiseTex, noiseSampler, P.xz, m.r, slope);
        // The sparse spots of the patch noise repeat every ~11.5 m; with the light soil texture under them they
        // read as a regular grid from far away, so they fade out with distance.
        const float grass = saturate(m.r * lerp(zones.patch, 1.0, fadeTex) * 1.2);
        const float road = saturate(vin.localNormal.x * 1.3 - 0.15 + (nz.r - 0.5) * 0.35) * step(0.01, vin.localNormal.x);
        const float3 w = vin.color;
        const float rock = saturate((w.x - 0.5) * 2.2 + 0.5 + (nz.r - 0.5) * 0.6) * step(0.01, w.x);
        const float sand = saturate((w.y - 0.5) * 1.6 + 0.5 + (nz.g - 0.5) * 0.4) * step(0.01, w.y);
        const float snow = saturate((w.z - 0.5) * 2.6 + 0.5 + (nz.g - 0.5) * 0.7) * step(0.01, w.z);
        const bool asphalt = tt.params.z > 0.5;

        // Parallax near the camera on flat-ish ground: one ray march in the dominant layer shifts every layer's
        // lookup (Pt); masks, shadows and lighting stay at P.
        float3 Pt = P;
        const float pom = f.parallax.x * (1.0 - smoothstep(f.parallax.z, f.parallax.w, dist)) * smoothstep(0.80, 0.92, N.y);
        const int dom = groundDominant(grass, road, sand, rock, snow, asphalt);
        if (pom > 0.0 && dom >= 0) {
            // Grazing views take the most steps and the least depth (stair steps between the slices).
            const float depthM = f.parallax.y * saturate(pom / f.parallax.x) * smoothstep(0.10, 0.45, V.y);
            const int steps = int(lerp(f.parallax.x, max(4.0, f.parallax.x * 0.5), saturate(V.y)));
            Pt.xz += parallaxShift(dom, gc, V, depthM, steps);
        }

        gc.xz = Pt.xz;
        const float microF = (1.0 - smoothstep(8.0, 16.0, dist)) * smoothstep(0.80, 0.92, N.y) *
                             smoothstep(0.02, 0.08, f.sunDir.y);
        if (f.groundShadow.x > 0.0 && microF > 0.0 && dom >= 0)
            micro = lerp(1.0, microShadow(dom, gc, f.sunDir.xyz, f.parallax.y, int(f.groundShadow.x)), f.groundShadow.y * microF);
        // Full grass hides the soil whatever the heights (heightBlend(1, ...) == 1), so the soil is not sampled.
        LayerSample g;
        if (grass >= 1.0) {
            g = sampleLayer(LAYER_GRASS, gc);
        } else {
            g = sampleLayer(LAYER_SOIL, gc);
            if (grass > 0.001) {
                const LayerSample l = sampleLayer(LAYER_GRASS, gc);
                blendLayer(g, l, heightBlend(grass, l.height, g.height));
            }
        }
        if (road > 0.001) {
            const LayerSample l = sampleLayer(LAYER_ROAD, gc);
            blendLayer(g, l, heightBlend(road, l.height, g.height));
        }
        if (sand > 0.001) {
            const LayerSample l = sampleLayer(LAYER_SAND, gc);
            blendLayer(g, l, heightBlend(sand, l.height, g.height));
        }
        if (rock > 0.001) {
            // Walls: the side projections are warped by ~1.5 m so the repeats of the texture do not stack into
            // regular vertical stripes. Broad weathering stays subtle so the scanned fractures remain legible.
            const float steepF = smoothstep(0.25, 0.65, 1.0 - N.y);
            const float2 warp = (noiseTex.Sample(noiseSampler, P.xz * 0.021 + P.y * 0.013).rg - 0.5) * 3.0 * steepF;
            LayerSample l = sampleRock(gc, Pt, N, warp);
            if (steepF > 0.0) {
                // Keep the scan's fractures and normals. High-contrast procedural height bands made
                // every steep wall look like stacked ribbons, independent of its actual rock geometry.
                const float stain = noiseTex.Sample(noiseSampler, P.xz * 0.017 + P.y * 0.009).r;
                const float broad = noiseTex.Sample(noiseSampler, float2(P.x + P.z, P.y * 1.7) * 0.006).g;
                const float luminance = dot(l.albedo, float3(0.2126, 0.7152, 0.0722));
                const float3 weathered = lerp(l.albedo, luminance * float3(1.02, 1.01, 1.0), 0.72);
                l.albedo = lerp(l.albedo, weathered * (1.12 + 0.18 * stain + 0.12 * broad), steepF);
            }
            blendLayer(g, l, heightBlend(rock, l.height, g.height));
        }
        if (snow > 0.001 && asphalt) {
            // Asphalt: the road layer's grain at 4x the frequency for the aggregate, desaturated and dark, flat.
            LayerSample l = sampleLayerScaled(LAYER_ROAD, gc, 4.0);
            const float grain = dot(l.albedo, float3(0.3, 0.55, 0.15));
            l.albedo = asphaltAlbedo(P, saturate(grain * 2.2));
            l.dn *= 0.25;
            l.rough = 0.78;
            l.height = 0.5;
            blendLayer(g, l, saturate((w.z - 0.5) * 6.0 + 0.5) * step(0.01, w.z));
        } else if (snow > 0.001) {
            const LayerSample l = sampleLayer(LAYER_SNOW, gc);
            blendLayer(g, l, heightBlend(snow, l.height + 0.3, g.height));
        }
        // Large-scale tone variation so the repeats do not line up over hundreds of meters.
        const float macro = noiseTex.Sample(noiseSampler, P.xz * 0.011).r;
        float3 albedo = g.albedo;
        float rough = g.rough;
        macroTone(noiseTex, noiseSampler, P.xz, albedo, rough);
        float flat = 1.0;

        // Mud near water (mask G, smooth, edges broken up by noise): dark, wet and smoother.
        const float mudBlend = smoothstep(0.30, 0.55, m.g + (nz.g - 0.5) * 0.30);
        if (mudBlend > 0.0) {
            const float wetN = smoothstep(0.45, 0.65, nz.r * 0.7 + macro * 0.3);
            const float wet = saturate(wetN * 0.8 + m.b * 0.5);
            const float3 mudAlb = lerp(float3(0.125, 0.078, 0.042), float3(0.045, 0.029, 0.016), wet) *
                                  (0.8 + 0.4 * g.height);
            albedo = lerp(albedo, mudAlb, mudBlend);
            rough = lerp(rough, lerp(0.85, 0.40, wet), mudBlend);
            flat = lerp(1.0, 0.5, mudBlend * wet);
        }
        // Sown wheat: light tilled soil with straw between the rows.
        albedo = lerp(albedo, lerp(albedo * 1.15, float3(0.20, 0.155, 0.075), 0.45 + 0.3 * nz.g), m.a);
        // Far from the camera the ground under the grass takes the blade canopy colour (the blades thin out).
        albedo = lerp(albedo, grassCanopyAlbedo(zones, m.a), max(grass * (1.0 - max(road, max(rock, snow))), m.a) * far);
        // Wet ground near the water (dark, glossy, flatter).
        groundWet(m.b, rock, road, sand, g.height, N.y, nz.g, albedo, rough, flat);

        // Under the crowns: needle and leaf litter (dark, brown-olive, broken up by noise) and less sky light.
        const float canopy = smoothstep(0.05, 0.6, vin.localNormal.y) * (1.0 - road) * (1.0 - snow);
        if (canopy > 0.0) {
            const float litterN = noiseTex.Sample(noiseSampler, P.xz * 0.37).r;
            const float lum = dot(albedo, float3(0.3, 0.5, 0.2));
            const float3 litter = lerp(float3(0.040, 0.032, 0.022), float3(0.085, 0.066, 0.042), litterN) *
                                  (0.55 + 2.2 * lum);
            albedo = lerp(albedo, litter, canopy * (0.8 + 0.2 * nz.g));
            rough = lerp(rough, 0.9, canopy * 0.5);
        }
        s.albedo = albedo;
        s.roughness = rough;
        s.ao *= lerp(1.0, g.ao, 0.8) * (1.0 - 0.35 * canopy);
        N = normalize(N + (g.dn + groundDetailDn(dom, gc, dist, N.y)) * flat);
    }
    if (f.options.x > 0.0) {
        const float2 cell = abs(frac(P.xz) - 0.5);
        s.albedo *= 1.0 - f.options.x * smoothstep(0.47, 0.5, max(cell.x, cell.y));
    }
    s.N = N;
    s.V = V;
    s.lamp = headlightIrradiance(P, N);
    s.shadow = shadowFactor(shadowMap, shadowSampler, P, normalize(vin.normal)) * cloudShadow(P) * micro;
    if (f.groundFx.w > 0.5) groundSpecular(s, normalize(vin.normal));
    float3 direct, indirect;
    const float3 color = shadeSplit(s, direct, indirect);
    return fogSplit(color, direct, indirect, P);
}
#endif

// ---------------------------------------------------------------- shadow pass (depth only)

struct ShadowUniforms {
    float4x4 lightViewProj;
};

#if defined(TF_ENTRY_shadow_vs) || defined(TF_ENTRY_tree_shadow_vs)
ConstantBuffer<ShadowUniforms> su : UBO(0);
#endif

#if defined(TF_ENTRY_shadow_vs)
float4 shadow_vs(LitIn vin) : SV_Position {
    return mul(su.lightViewProj, mul(draw.model, float4(vin.position, 1.0)));
}
#endif

#if defined(TF_ENTRY_shadow_fs)
void shadow_fs() {}
#endif

// ---------------------------------------------------------------- instanced trees (render-only backdrop)

struct TreeUniforms {
    float4 info; // x: first instance, y: time (s)
};

struct TreeInstance {
    float4 baseHeight; // xyz: trunk base, w: height (m)
    float4 params;     // x: yaw, y: color variation, z: width scale
};

// Unit-height tree vertex -> world, with a gentle sway that grows with height (same in the shadow pass).
float3 treeWorld(TreeInstance t, float3 lp, float time) {
    const float c = cos(t.params.x), s = sin(t.params.x);
    const float h = t.baseHeight.w, wsc = t.params.z;
    float3 p = float3((c * lp.x - s * lp.z) * wsc, lp.y, (s * lp.x + c * lp.z) * wsc) * h;
    const float sway = (sin(time * 0.9 + t.baseHeight.x * 0.13 + t.baseHeight.z * 0.11) * 0.6 +
                        sin(time * 1.7 + t.baseHeight.x * 0.31) * 0.25) * 0.010 * lp.y * lp.y * h;
    p.xz += float2(0.8, 0.6) * sway;
    return t.baseHeight.xyz + p;
}

#if defined(TF_ENTRY_tree_vs) || defined(TF_ENTRY_tree_shadow_vs)
ConstantBuffer<TreeUniforms> tu : UBO(1);
StructuredBuffer<TreeInstance> trees : TEX(0);
#endif

#if defined(TF_ENTRY_tree_vs)
LitOut tree_vs(LitIn vin, uint iid : SV_InstanceID) {
    const TreeInstance t = trees[uint(tu.info.x) + iid];
    LitOut vout;
    const float3 world = treeWorld(t, vin.position, f.cameraPos.w);
    vout.position = mul(f.viewProj, float4(world, 1.0));
    vout.worldPos = world;
    const float c = cos(t.params.x), s = sin(t.params.x);
    const float3 n = float3(c * vin.normal.x - s * vin.normal.z, vin.normal.y, s * vin.normal.x + c * vin.normal.z);
    vout.normal = normalize(float3(n.x / t.params.z, n.y, n.z / t.params.z));
    // Per-tree hue: from olive/yellow-green to deep blue-green.
    const float v = t.params.y;
    vout.color = vin.color * lerp(float3(1.18, 1.08, 0.70), float3(0.80, 0.95, 1.10), v) * (0.85 + 0.3 * frac(v * 7.31));
    vout.material = float4(vin.material.xyz, 0.0);
    vout.localPos = vin.position;
    vout.localNormal = vin.normal;
    // params.w: LOD cross-fade code (0 none, (0, 1] nearer LOD keeps that share, [2, 3] farther LOD from w - 2 up).
    vout.mudMode = -t.params.w;
    vout.soil = ((float4)(0.0));
    vout.soilParams = ((float2)(0.0));
    vout.soilExtra = ((float3)(0.0));
    return vout;
}
#endif

#if defined(TF_ENTRY_tree_shadow_vs)
float4 tree_shadow_vs(LitIn vin, uint iid : SV_InstanceID) : SV_Position {
    const TreeInstance t = trees[uint(tu.info.x) + iid];
    return mul(su.lightViewProj, float4(treeWorld(t, vin.position, tu.info.y), 1.0));
}
#endif

// ---------------------------------------------------------------- instanced level models (assets/models, render side)
// Trees, bushes, rocks and logs from core/ModelAsset .glb files: per (model, LOD, material) one indexed instanced
// draw; far trees as impostor cards from the baked 8-view atlas. The transform matches the collision of the same
// objects (track/Level.cpp addLevelRock / addLevelTrunk): scale per axis, pitch about the model x axis
// (y' = c y - s z, z' = s y + c z; cliff faces only), yaw about +Y (x' = c x + s z, z' = -s x + c z), then the position.

struct ModelUniforms {
    float4 info;   // x: first instance, y: time (s), z: wind strength, w: model height (m, unit scale)
    float4 params; // x: sway amplitude (share of the height), y: flutter amplitude (m), z: impostor views, w: radius (m)
};

struct ModelMaterialUniforms {
    float4 params; // x: alpha test (0/1), y: foliage (0/1), z: alpha to coverage (0/1), w: normal map strength
    float4 tint;   // rgb: albedo multiplier, w: per-instance hue variation
};

struct ModelInstance {
    float4 posFade;  // xyz: origin (tree base / rock center), w: LOD cross-fade code (as TreeInstance.params.w)
    float4 scaleYaw; // xyz: scale per model axis, w: yaw (rad)
    float4 tilt;     // x: pitch (rad) about the model x axis, after the scale and before the yaw
};

struct ModelIn {
    [[vk::location(0)]] float3 position : TEXCOORD0;
    [[vk::location(1)]] float3 normal : TEXCOORD1;
    [[vk::location(2)]] float2 uv : TEXCOORD2;
    [[vk::location(3)]] float2 wind : TEXCOORD3; // x: bend weight (0 at the base, 1 at the top), y: flutter weight
};

struct ModelOut {
    float4 position : SV_Position;
    [[vk::location(0)]] float3 worldPos : TEXCOORD0;
    [[vk::location(1)]] float3 normal : TEXCOORD1;
    [[vk::location(2)]] float2 uv : TEXCOORD2;
    [[vk::location(3)]] nointerpolation float2 fadeHue : TEXCOORD3; // x: cross-fade code, y: hue variation 0..1
    [[vk::location(4)]] float3 modelView : TEXCOORD4; // impostors: horizontal camera direction in model space
};

float3 modelRotate(float3 v, float c, float s) { return float3(c * v.x + s * v.z, v.y, -s * v.x + c * v.z); }
float3 modelPitch(float3 v, float c, float s) { return float3(v.x, c * v.y - s * v.z, s * v.y + c * v.z); }

#if defined(TF_ENTRY_model_vs) || defined(TF_ENTRY_model_shadow_vs) || defined(TF_ENTRY_impostor_vs)
ConstantBuffer<ModelUniforms> mu : UBO(1);
StructuredBuffer<ModelInstance> modelInstances : TEX(0);

// Wind: the whole crown bends with the gusts (weight wind.x grows with height), leaves and needle tips flutter.
float3 modelWind(float3 world, float2 w, float3 origin, float height) {
    const float t = mu.info.y;
    const float phase = origin.x * 0.13 + origin.z * 0.11;
    const float gust = sin(t * 0.9 + phase) * 0.6 + sin(t * 1.7 + origin.x * 0.31) * 0.25 + 0.35;
    float3 p = world;
    p.xz += float2(0.8, 0.6) * gust * mu.params.x * mu.info.z * height * w.x;
    const float flutter = sin(t * 6.3 + dot(world, float3(1.7, 2.3, 1.9))) * mu.params.y * mu.info.z * w.y;
    p += float3(0.6, 0.35, -0.7) * flutter;
    return p;
}
#endif

#if defined(TF_ENTRY_model_shadow_vs)
ConstantBuffer<ShadowUniforms> su : UBO(0);
#endif

#if defined(TF_ENTRY_model_vs) || defined(TF_ENTRY_model_shadow_vs)
ModelOut modelTransform(ModelIn vin, uint iid, float4x4 viewProj) {
    const ModelInstance m = modelInstances[uint(mu.info.x) + iid];
    const float c = cos(m.scaleYaw.w), s = sin(m.scaleYaw.w), cp = cos(m.tilt.x), sp = sin(m.tilt.x);
    const float3 world = m.posFade.xyz + modelRotate(modelPitch(vin.position * m.scaleYaw.xyz, cp, sp), c, s);
    const float3 p = modelWind(world, vin.wind, m.posFade.xyz, mu.info.w * m.scaleYaw.y);
    ModelOut o;
    o.position = mul(viewProj, float4(p, 1.0));
    o.worldPos = p;
    o.normal = modelRotate(modelPitch(vin.normal / m.scaleYaw.xyz, cp, sp), c, s);
    o.uv = vin.uv;
    o.fadeHue = float2(m.posFade.w, hash12(m.posFade.xz));
    o.modelView = ((float3)(0.0));
    return o;
}
#endif

#if defined(TF_ENTRY_model_vs)
ModelOut model_vs(ModelIn vin, uint iid : SV_InstanceID) { return modelTransform(vin, iid, f.viewProj); }
#endif

#if defined(TF_ENTRY_model_shadow_vs)
ModelOut model_shadow_vs(ModelIn vin, uint iid : SV_InstanceID) { return modelTransform(vin, iid, su.lightViewProj); }
#endif

// Impostor card (6 vertices per instance, no vertex buffer): upright, turned toward the camera about the trunk axis,
// width 2 x radius, height = model height (scaled). The atlas view nearest to the camera direction is chosen in the
// fragment shader from the card's facing.
#if defined(TF_ENTRY_impostor_vs)
ModelOut impostor_vs(uint vid : SV_VertexID, uint iid : SV_InstanceID) {
    const ModelInstance m = modelInstances[uint(mu.info.x) + iid];
    const float2 corners[6] = {float2(-1, 0), float2(1, 0), float2(1, 1), float2(-1, 0), float2(1, 1), float2(-1, 1)};
    const float2 k = corners[vid];
    float3 toCam = f.cameraPos.xyz - m.posFade.xyz;
    toCam.y = 0.0;
    toCam = length(toCam) > 1e-3 ? normalize(toCam) : float3(0, 0, 1);
    const float3 right = float3(toCam.z, 0.0, -toCam.x);
    const float w = mu.params.w * m.scaleYaw.x, h = mu.info.w * m.scaleYaw.y;
    float3 p = m.posFade.xyz + right * (k.x * w) + float3(0, k.y * h, 0);
    p = modelWind(p, float2(k.y * k.y, 0.0), m.posFade.xyz, h);
    ModelOut o;
    o.position = mul(f.viewProj, float4(p, 1.0));
    o.worldPos = p;
    o.normal = toCam;
    o.uv = float2(k.x * 0.5 + 0.5, 1.0 - k.y);
    o.fadeHue = float2(m.posFade.w, hash12(m.posFade.xz));
    // The camera direction turned back by the yaw (for the view selection).
    const float c = cos(m.scaleYaw.w), s = sin(m.scaleYaw.w);
    o.modelView = float3(c * toCam.x - s * toCam.z, 0.0, s * toCam.x + c * toCam.z);
    return o;
}
#endif

bool modelDitherDiscard(float code, float2 pixel) {
    if (code <= 0.0) return false;
    const float d = frac(52.9829189 * frac(dot(pixel, float2(0.06711056, 0.00583715))));
    return code < 1.5 ? d >= code : d < code - 2.0;
}

#if defined(TF_ENTRY_model_fs) || defined(TF_ENTRY_impostor_fs) || defined(TF_ENTRY_model_shadow_fs)
ConstantBuffer<ModelMaterialUniforms> mm : UBO(1);

// Alpha-tested coverage: with alpha to coverage the edge is sharpened to about one pixel (the MSAA samples blend it);
// without, a plain cut at 0.5.
float modelCoverage(float a) {
    if (mm.params.z > 0.5) {
        const float sharp = saturate((a - 0.5) / max(fwidth(a), 1e-4) + 0.5);
        if (sharp <= 0.0) discard;
        return sharp;
    }
    if (a < 0.5) discard;
    return 1.0;
}
#endif

#if defined(TF_ENTRY_model_shadow_fs)
Texture2D<float4> modelAlbedo : TEX(0);
SamplerState modelAlbedoSampler : SMP(0);
void model_shadow_fs(ModelOut vin) {
    if (mm.params.x > 0.5 && modelAlbedo.Sample(modelAlbedoSampler, vin.uv).a < 0.5) discard;
}
#endif

#if defined(TF_ENTRY_model_fs) || defined(TF_ENTRY_impostor_fs)
Texture2DArray<float> shadowMap : TEX(0);
SamplerComparisonState shadowSampler : SMP(0);
Texture2D<float4> noiseTex : TEX(1);
SamplerState noiseSampler : SMP(1);
Texture2D<float4> modelAlbedo : TEX(2);
SamplerState modelAlbedoSampler : SMP(2);
Texture2D<float4> modelNra : TEX(3);
SamplerState modelNraSampler : SMP(3);

// Foliage and bark share the lighting; leaves get a soft wrap and light through them when backlit.
// Returns the fogged colour and the light split (lightSplit).
float4 shadeModel(float3 albedo, float rough, float ao, float3 N, float3 Ng, float3 P, bool foliage) {
    const float3 V = normalize(f.cameraPos.xyz - P);
    ShadeInput s;
    s.albedo = albedo;
    s.roughness = rough;
    s.metallic = 0.0;
    s.N = N;
    s.V = V;
    s.ao = ao;
    s.specOcclusion = foliage ? 0.5 * ao : ao;
    s.emissive = ((float3)(0.0));
    s.clearCoat = 0.0;
    s.lamp = headlightIrradiance(P, N);
    s.shadow = shadowFactor(shadowMap, shadowSampler, P, Ng) * cloudShadow(P);
    float3 through = (float3)0.0;
    if (foliage) {
        const float back = pow(saturate(dot(-V, f.sunDir.xyz)), 4.0);
        through = albedo * f.sunColor.rgb * s.shadow * (0.05 + 0.35 * back) * ao;
    }
    float3 direct, indirect;
    const float3 color = shadeSplit(s, direct, indirect) + through;
    return fogSplit(color, direct + through, indirect, P);
}

float3 modelHue(float3 albedo, float hue) {
    const float k = mm.tint.w;
    return albedo * mm.tint.rgb * lerp((float3)(1.0), lerp(float3(1.12, 1.06, 0.82), float3(0.86, 0.97, 1.06), hue) *
                                                          (0.88 + 0.24 * frac(hue * 7.31)), k);
}
#endif

#if defined(TF_ENTRY_model_fs)
float4 model_fs(ModelOut vin, bool front : SV_IsFrontFace) : SV_Target0 {
    if (modelDitherDiscard(vin.fadeHue.x, vin.position.xy)) discard;
    const float4 a = modelAlbedo.Sample(modelAlbedoSampler, vin.uv);
    const float coverage = mm.params.x > 0.5 ? modelCoverage(a.a) : 1.0;
    const float4 nra = modelNra.Sample(modelNraSampler, vin.uv);
    float3 Ng = normalize(vin.normal);
    if (!front) Ng = -Ng;
    // Cotangent frame from the screen derivatives (no tangents in the vertex format); DirectX normal maps: +y
    // follows +v.
    const float3 dp1 = ddx(vin.worldPos), dp2 = ddy(vin.worldPos);
    const float2 duv1 = ddx(vin.uv), duv2 = ddy(vin.uv);
    const float3 dp2perp = cross(dp2, Ng), dp1perp = cross(Ng, dp1);
    const float3 T = dp2perp * duv1.x + dp1perp * duv2.x;
    const float3 B = dp2perp * duv1.y + dp1perp * duv2.y;
    const float inv = rsqrt(max(max(dot(T, T), dot(B, B)), 1e-20));
    const float2 nxy = (nra.rg * 2.0 - 1.0) * mm.params.w;
    const float3 N = normalize(T * inv * nxy.x + B * inv * nxy.y + Ng * sqrt(saturate(1.0 - dot(nxy, nxy))));
    const bool foliage = mm.params.y > 0.5;
    const float3 albedo = modelHue(a.rgb, vin.fadeHue.y);
    const float4 lit = shadeModel(albedo, nra.b, nra.a, N, Ng, vin.worldPos, foliage);
    return float4(lit.rgb, mm.params.z > 0.5 ? coverage : lit.a); // alpha to coverage only with MSAA (no AO there)
}
#endif

#if defined(TF_ENTRY_impostor_fs)
// Atlas: mu.params.z views side by side; view k was rendered from the azimuth a = 2 pi k / views (camera along
// (sin a, 0, cos a) in model space, card +u along (cos a, 0, -sin a)). Normals in the atlas: x along the card's +u,
// y up, z toward the viewer. mm.params.w: number of views.
float4 impostor_fs(ModelOut vin) : SV_Target0 {
    if (modelDitherDiscard(vin.fadeHue.x, vin.position.xy)) discard;
    const float views = max(1.0, round(mm.params.w));
    const float az = atan2(vin.modelView.x, vin.modelView.z);
    float k = round(az / (2.0 * PI) * views);
    k = k - views * floor(k / views);
    const float2 uv = float2((k + clamp(vin.uv.x, 0.002, 0.998)) / views, vin.uv.y);
    const float4 a = modelAlbedo.Sample(modelAlbedoSampler, uv);
    const float coverage = modelCoverage(a.a);
    const float4 nra = modelNra.Sample(modelNraSampler, uv);
    const float3 toCam = normalize(vin.normal);
    const float3 right = float3(toCam.z, 0.0, -toCam.x);
    const float2 nxy = nra.rg * 2.0 - 1.0;
    const float3 N = normalize(right * nxy.x + float3(0, nxy.y, 0) + toCam * sqrt(saturate(1.0 - dot(nxy, nxy))));
    const float3 albedo = modelHue(a.rgb, vin.fadeHue.y);
    const float4 lit = shadeModel(albedo, nra.b, nra.a, N, toCam, vin.worldPos, true);
    return float4(lit.rgb, mm.params.z > 0.5 ? coverage : lit.a);
}
#endif

// ---------------------------------------------------------------- level terrain (render copy of the physics grid)

// One shared patch of 64 x 64 cells (+ skirt) drawn per quadtree chunk. LitIn.position = (column, skirt 0/1, row).
// The heights are read with Load at the grid samples, so at step 1 the triangles are the physics heightfield
// (same diagonal: per cell {00, 01, 11} and {00, 11, 10}).
#if defined(TF_ENTRY_terrain_vs) || defined(TF_ENTRY_terrain_shadow_vs)
ConstantBuffer<DrawUniforms> draw : UBO(1); // params: first column, first row, step (samples), skirt depth (m); tint: samples, origin, cell
Texture2D<float> terrainHeight : TEX(0);
SamplerState terrainHeightSampler : SMP(0);
Texture2D<float4> terrainSplat : TEX(1);
SamplerState terrainSplatSampler : SMP(1);

float terrainSample(int i, int j) {
    const int last = int(draw.tint.x) - 1;
    return terrainHeight.Load(int3(clamp(i, 0, last), clamp(j, 0, last), 0));
}

int2 terrainIndex(float3 lp) {
    const int stepN = int(draw.params.z);
    return int2(int(draw.params.x) + int(lp.x + 0.5) * stepN, int(draw.params.y) + int(lp.z + 0.5) * stepN);
}

float3 terrainWorld(float3 lp, int2 ij) {
    return float3(draw.tint.y + float(ij.x) * draw.tint.z, terrainSample(ij.x, ij.y) - lp.y * draw.params.w,
                  draw.tint.y + float(ij.y) * draw.tint.z);
}
#endif

#if defined(TF_ENTRY_terrain_vs)
LitOut terrain_vs(LitIn vin) {
    const int2 ij = terrainIndex(vin.position);
    const float3 world = terrainWorld(vin.position, ij);
    // Normal from central differences over one LOD step (smooth at every LOD).
    const int d = int(draw.params.z);
    const float hx = terrainSample(ij.x - d, ij.y) - terrainSample(ij.x + d, ij.y);
    const float hz = terrainSample(ij.x, ij.y - d) - terrainSample(ij.x, ij.y + d);
    LitOut vout;
    vout.position = mul(f.viewProj, float4(world, 1.0));
    vout.worldPos = world;
    vout.normal = normalize(float3(hx, 2.0 * float(d) * draw.tint.z, hz));
    // Splat texels are the cells; a grid sample sits on the corner of four, the linear filter averages them.
    const float cells = draw.tint.x - 1.0;
    const float4 sp = terrainSplat.SampleLevel(terrainSplatSampler, float2(ij) / cells, 0);
    const float sandSnow = sp.b * (255.0 / 127.5) - 1.0; // splat b: sand above the middle, snow below
    vout.color = float3(sp.g, max(sandSnow, 0.0), max(-sandSnow, 0.0)); // rock, sand, snow
    vout.material = float4(float(MAT_TERRAIN), 0.0, 0.0, 0.0);
    vout.localPos = world;
    vout.localNormal = float3(sp.r, sp.a, 0.0); // terrain: x = road paint, y = tree canopy above the ground
    vout.mudMode = 0.0;
    vout.soil = ((float4)(0.0));
    vout.soilParams = ((float2)(0.0));
    vout.soilExtra = ((float3)(0.0));
    return vout;
}
#endif

#if defined(TF_ENTRY_terrain_shadow_vs)
ConstantBuffer<ShadowUniforms> su : UBO(0);
float4 terrain_shadow_vs(LitIn vin) : SV_Position {
    return mul(su.lightViewProj, float4(terrainWorld(vin.position, terrainIndex(vin.position)), 1.0));
}
#endif

// ---------------------------------------------------------------- sky

struct FullscreenOut {
    float4 position : SV_Position;
    [[vk::location(0)]] float2 uv : TEXCOORD0;
};

#if defined(TF_ENTRY_fullscreen_vs)
FullscreenOut fullscreen_vs(uint vid : SV_VertexID) {
    FullscreenOut vout;
    const float2 p = float2((vid << 1) & 2, vid & 2);
    vout.position = float4(p * 2.0 - 1.0, 0.0, 1.0); // z = 0: far plane of the reversed depth (sky behind everything)
    vout.uv = float2(p.x, 1.0 - p.y);
    return vout;
}
#endif

// Ridge vertex height (0..1) of mountain layer `seed`: jitter plus broader peaks every third vertex.
// n is a multiple of 3, so the pattern wraps around the full circle without a seam.
float ridgeHeight(float k, float seed) {
    const float r = hash12(float2(k, seed));
    const float big = hash12(float2(floor(k / 3.0), seed + 5.1));
    return 0.30 + 0.70 * (0.5 * r + 0.5 * big * big);
}

// Distant layered mountains, drawn in the sky pass (render-only, at infinity, no collision): faceted ridgelines
// from piecewise-linear hashed heights, two flat-shaded facets per ridge segment, aerial perspective toward the
// haze color (farther layers are lighter and bluer). A valley opens where the sun sets. Returns rgb + coverage.
float4 mountains(float3 dir) {
    const float el = dir.y;
    if (el > 0.205) return ((float4)(0.0)); // above the highest ridge (amp[0])
    const float az = atan2(dir.x, dir.z);
    const float dv = abs(fmod(az - f.fogColor.w + 3.0 * PI, 2.0 * PI) - PI); // angle from the valley center
    const float aa = max(fwidth(el), 1e-5) * 1.2;
    const float3 V0 = -float3(sin(az), 0.0, cos(az)); // facing the viewer
    const float3 T = float3(cos(az), 0.0, -sin(az));  // along the ridge (increasing azimuth)
    // Aerial perspective toward a cooler, violet version of the haze (stylized: distant ranges read blue-violet
    // even against a warm sunset sky).
    const float3 warmHaze = hazeColor(normalize(float3(dir.x, max(dir.y, 0.0), dir.z)));
    const float3 haze = lerp(warmHaze, luminance(warmHaze) * float3(0.78, 0.74, 1.12), 0.55);
    const float segs[4] = {24.0, 36.0, 54.0, 81.0};
    const float amp[4] = {0.200, 0.150, 0.110, 0.078};
    const float keep[4] = {0.40, 0.48, 0.60, 0.75};   // ridge height kept at the valley center
    const float hazeAmt[4] = {0.70, 0.55, 0.40, 0.26};
    const float3 albedo[4] = {float3(0.20, 0.20, 0.36), float3(0.17, 0.17, 0.32), float3(0.13, 0.14, 0.27),
                              float3(0.10, 0.12, 0.20)};
    float3 col = ((float3)(0.0));
    float cover = 0.0;
    for (int i = 0; i < 4; ++i) {
        const float n = segs[i];
        const float x = (az / (2.0 * PI) + 0.5) * n;
        const float k = floor(x), t = x - k;
        const float seed = float(i) * 17.3 + 3.0;
        const float k0 = fmod(k, n), k1 = fmod(k + 1.0, n);
        const float valley = lerp(keep[i], 1.0, smoothstep(0.05, 0.40, dv));
        const float h0 = amp[i] * ridgeHeight(k0, seed) * valley, h1 = amp[i] * ridgeHeight(k1, seed) * valley;
        const float h = lerp(h0, h1, t) - 0.003;
        const float c = 1.0 - smoothstep(h - aa, h, el);
        if (c <= 0.0) continue;
        // Crease from the higher ridge vertex down to a hashed point at the base splits the segment into two facets.
        const bool rightHigh = h1 > h0;
        const float tp = rightHigh ? 1.0 : 0.0, hp = max(h0, h1);
        const float tc = 0.15 + 0.7 * hash12(float2(k0, seed + 9.7));
        const float side = (t - tp) * (-0.03 - hp) - (el - hp) * (tc - tp);
        const bool slopeFacet = rightHigh ? side < 0.0 : side > 0.0;
        const float slope = (h1 - h0) * n / (2.0 * PI);
        const float facetId = k0 * 2.0 + (slopeFacet ? 1.0 : 0.0);
        float3 N = slopeFacet ? normalize(V0 - T * clamp(slope * 3.0, -1.5, 1.5) + float3(0, 0.55, 0))
                              : normalize(V0 + T * (rightHigh ? 0.5 : -0.5) + float3(0, 0.9, 0));
        const float tone = 0.85 + 0.3 * hash12(float2(facetId, seed + 2.3));
        float3 litCol = albedo[i] * tone * (f.skyAmbient.rgb * 1.1 + f.sunColor.rgb * saturate(dot(N, f.sunDir.xyz)) * 0.30);
        // Backlit rim along the ridge toward the sun (sunset), and mist settling in the valleys.
        const float rim = pow(saturate(dot(dir, f.skySun.xyz)), 5.0) * (1.0 - smoothstep(0.0, 0.006, h - el));
        litCol += f.skySunGlow.rgb * rim * 0.35;
        const float mist = 1.0 - smoothstep(-0.02, hp * 0.9, el);
        const float3 layer = lerp(litCol, haze, saturate(hazeAmt[i] + (1.0 - hazeAmt[i]) * 0.35 * mist));
        col = lerp(col, layer, c);
        cover = max(cover, c);
    }
    return float4(col, cover);
}

#if defined(TF_ENTRY_sky_fs)
Texture2D<float4> noiseTex : TEX(0);
SamplerState ns : SMP(0);

float4 sky_fs(FullscreenOut vin) : SV_Target0 {
    const float2 ndc = float2(vin.uv.x * 2.0 - 1.0, 1.0 - vin.uv.y * 2.0);
    const float4 w = mul(f.invViewProj, float4(ndc, 1.0, 1.0));
    const float3 dir = normalize(w.xyz / w.w - f.cameraPos.xyz);
    float3 col = skyRadiance(dir);
    const float mu = dot(dir, f.skySun.xyz);
    const float muMoon = dot(dir, f.moonDir.xyz);
    // Stars (hashed cells on a direction grid), fading toward the horizon.
    if (f.skySunGlow.w > 0.01 && dir.y > 0.0) {
        const float3 p = dir * 260.0;
        const float3 cell = floor(p);
        const float r = hash12(cell.xy + cell.z * 13.71);
        if (r > 0.982) {
            const float3 center = cell + 0.5 + (float3(hash12(cell.yz + 3.1), hash12(cell.zx + 7.7), hash12(cell.xy + 1.3)) - 0.5) * 0.5;
            const float star = smoothstep(0.42, 0.0, length(p - center)) * (0.3 + 0.7 * (r - 0.982) / 0.018);
            col += float3(0.75, 0.82, 1.0) * star * 0.30 * f.skySunGlow.w * smoothstep(0.02, 0.25, dir.y);
        }
    }
    // Sun disc: large and stylized (radius from skySun.w), slightly brighter core; the glow comes from skyRadiance.
    {
        const float cosR = f.skySun.w;
        const float edge = (1.0 - cosR) * 0.04;
        const float disc = smoothstep(cosR - edge, cosR + edge, mu);
        const float core = saturate((mu - cosR) / (1.0 - cosR));
        col = lerp(col, f.sunDisc.rgb * (0.85 + 0.35 * sqrt(core)), disc * saturate(luminance(f.sunDisc.rgb) * 4.0));
    }
    // Moon disc with a faint halo.
    if (f.sunDisc.w > 0.0) {
        const float cosM = 0.99975;
        const float disc = smoothstep(cosM - 0.00002, cosM + 0.00002, muMoon);
        const float maria = noiseTex.Sample(ns, (dir.xz - f.moonDir.xz) * 40.0 + 0.5).r;
        col = lerp(col, float3(0.80, 0.84, 0.95) * (0.9 + 0.4 * maria) * 1.4, disc * f.sunDisc.w);
        col += float3(0.10, 0.12, 0.18) * pow(saturate(muMoon), 400.0) * f.sunDisc.w * 0.6;
    }
    // Two cloud layers: thin cirrus streaks 8 km up behind the cumulus layer 2 km up (the layer whose shadow crosses
    // the ground). Detail comes from the tileable noise texture, the large cumulus shapes from cloudShape.
    if (f.options.w > 0.5 && dir.y > 0.01) {
        const float muL = dot(dir, f.sunDir.xyz);
        const float t = f.cameraPos.w;
        const float horizonFade = smoothstep(0.01, 0.20, dir.y);
        {
            const float2 q = f.cameraPos.xz + dir.xz * ((8000.0 - f.cameraPos.y) / dir.y) + float2(14.0, 3.0) * t;
            const float2 r = float2(q.x * 0.8 - q.y * 0.6, q.x * 0.6 + q.y * 0.8);
            const float patch = smoothstep(0.50, 0.75, noiseTex.Sample(ns, q * (1.0 / 70000.0)).r);
            const float streak = noiseTex.Sample(ns, r * float2(1.0 / 30000.0, 1.0 / 5000.0)).r;
            const float wisp = noiseTex.Sample(ns, r * float2(1.0 / 12000.0, 1.0 / 2600.0) + 0.37).g;
            const float ci = smoothstep(0.50, 0.85, streak * 0.8 + wisp * 0.35) * patch * horizonFade;
            const float3 ciCol = f.sunColor.rgb * 0.10 * (0.6 + phaseHG(muL, 0.6) * 0.5) + f.skyZenith.rgb * 0.55 +
                                 f.skySunGlow.rgb * 0.30;
            col = lerp(col, ciCol, ci * 0.35);
        }
        const float dist = (kCloudHeight - f.cameraPos.y) / dir.y;
        const float2 q = f.cameraPos.xz + dir.xz * dist;
        const float2 drift = float2(9.0, 4.0) * t;
        // Zero-mean texture detail on the shared shapes keeps the sky in step with the ground shadows.
        const float2 uv = (q + drift) * float2(1.0 / 4800.0, 1.0 / 2200.0);
        const float detail = noiseTex.Sample(ns, uv).r * 0.65 + noiseTex.Sample(ns, uv * 3.7 + 0.21).g * 0.35;
        const float n = cloudShape(q) + (detail - 0.5) * 0.36;
        const float c = cloudCoverOf(n) * horizonFade;
        // Optical depth: thin at the edges, thick in the cores and where the layer continues toward the sun.
        const float cover = f.fogParams.w;
        const float dens = smoothstep(1.0 - cover + 0.05, 1.0 - cover + 0.45, n);
        const float2 toSun = normalize(f.sunDir.xz + 1e-4) * 300.0;
        const float occl = cloudCover(q + toSun);
        const float tauSun = 1.6 * occl + 1.6 * dens;
        const float sunT = exp(-tauSun);
        // Beer-powder: thin parts scatter less back toward a viewer looking away from the sun.
        const float powder = lerp(1.0 - exp(-2.0 * (tauSun + 0.35)), 1.0, saturate(muL * 0.5 + 0.5));
        // Forward lobe only through thin parts (silver lining), backward lobe on single scattering, and multiple
        // scattering that the optical depth attenuates less.
        const float forward = 0.6 * phaseHG(muL, 0.6) * exp(-5.0 * dens);
        const float back = 0.4 * phaseHG(muL, -0.25) * powder;
        float3 cloudLit = f.sunColor.rgb * 0.5 * (sunT * (forward + back) + 0.45 * exp(-0.3 * tauSun));
        cloudLit += f.skyZenith.rgb * (0.30 + 0.20 * (1.0 - dens)) + f.skyHorizon.rgb * 0.25;
        cloudLit += f.skySunGlow.rgb * (0.06 + 0.25 * pow(saturate(mu), 3.0)); // sunset-lit undersides
        col = lerp(col, cloudLit, c * (0.75 + 0.2 * dens));
    }
    // Horizon haze matches the fog color.
    const float haze = pow(1.0 - saturate(abs(dir.y)), 6.0) * 0.6;
    col = lerp(col, hazeColor(dir), haze);
    if (f.worldParams.x > 0.5) {
        const float4 m = mountains(dir);
        col = lerp(col, m.rgb, m.a);
    }
    return float4(col, 0.0); // light split: nothing for AO or contact shadows
}
#endif

// ---------------------------------------------------------------- water (showcase puddles)

float rippleHeight(Texture2D<float4> noiseTex, SamplerState ns, float2 p, float t) {
    return noiseTex.Sample(ns, p * 0.11 + float2(t * 0.006, t * 0.004)).g +
           0.35 * noiseTex.Sample(ns, p * 0.37 - float2(t * 0.011, -t * 0.008)).g;
}

#if defined(TF_ENTRY_water_fs)
Texture2DArray<float> shadowMap : TEX(0);
SamplerComparisonState shadowSampler : SMP(0);
Texture2D<float4> noiseTex : TEX(1);
SamplerState ns : SMP(1);
Texture2D<float4> heightTex : TEX(2);
SamplerState cs : SMP(2);
Texture2D<float4> reflTex : TEX(3);
SamplerState rs : SMP(3);
Texture2D<float4> sceneTex : TEX(4); // opaque scene copy (screenFx.x)
SamplerState scs : SMP(4);
Texture2D<float4> depthLin : TEX(5); // view depth of the opaque scene (screenFx.x)
SamplerState dls : SMP(5);

float viewDepth(float3 P) { return dot(P - f.cameraPos.xyz, f.cameraFwd.xyz); }

float2 screenUV(float3 P, out bool inside) {
    const float4 c = mul(f.viewProj, float4(P, 1.0));
    const float2 n = c.xy / max(c.w, 1e-4);
    inside = c.w > 0.0 && all(abs(n) < 1.0);
    return float2(n.x * 0.5 + 0.5, 0.5 - n.y * 0.5);
}

// Screen-space reflection: march the reflected ray in growing steps against the opaque scene's view depth, refine
// the first crossing by bisection. Returns rgb and the confidence (0: no hit, use the sky).
float4 traceReflection(float3 P, float3 R, float dist) {
    float step = 0.3 + dist * 0.012;
    float3 prev = P, pos = P;
    for (int i = 0; i < 28; ++i) {
        pos = prev + R * step;
        bool inside;
        float2 uv = screenUV(pos, inside);
        if (!inside) break;
        const float zr = viewDepth(pos);
        const float zs = depthLin.SampleLevel(dls, uv, 0).r;
        if (zr > zs && zr - zs < step * 2.0 + 0.3) {
            float3 a = prev, b = pos;
            for (int k = 0; k < 5; ++k) {
                const float3 m = 0.5 * (a + b);
                bool mi;
                const float2 muv = screenUV(m, mi);
                if (viewDepth(m) > depthLin.SampleLevel(dls, muv, 0).r) b = m; else a = m;
            }
            uv = screenUV(b, inside);
            const float2 e = smoothstep(0.0, 0.08, uv) * smoothstep(1.0, 0.92, uv);
            const float conf = e.x * e.y * (1.0 - float(i) / 28.0);
            return float4(sceneTex.SampleLevel(scs, uv, 0).rgb, conf);
        }
        prev = pos;
        step *= 1.25;
    }
    return float4(0, 0, 0, 0);
}

float4 water_fs(LitOut vin) : SV_Target0 {
    const float3 P = vin.worldPos;
    const float2 guv = groundUV(P.xz);
    const bool outside = any(guv < 0.0) || any(guv > 1.0);
    const float terrain = heightTex.SampleLevel(cs, guv, 0).r;
    // Outside the baked square the open sea continues at about the depth the baked seabed ends with (no seam).
    const float depth = outside && f.worldParams.y > 0.5 ? 24.0 : P.y - terrain;
    if (depth <= 0.0) discard;
    const float t = f.cameraPos.w;
    const float dist = length(f.cameraPos.xyz - P);
    const bool open = vin.material.y > 1.0; // lakes, rivers, sea (puddles: ripples only)
    // Waves (render-only): wind swell as a sum of deep-water sine waves (phase speed sqrt(g L / 2 pi)), calmer in
    // the shallows, plus two scrolling noise ripple layers; detail fades with distance against shimmer.
    float2 slope = float2(0.0, 0.0);
    if (open) {
        const float calm = saturate(depth / 3.0);
        const float4 L = float4(9.0, 5.3, 3.1, 1.7);
        // Peak slopes, modulated by slow gust patches so the wave field does not read as a regular grid.
        const float gust = noiseTex.Sample(ns, P.xz * 0.0045 + float2(t * 0.0021, t * 0.0013)).r;
        const float4 S = float4(0.030, 0.026, 0.021, 0.017) * calm * (0.35 + 1.3 * gust);
        const float2 D[4] = {float2(0.87, 0.49), float2(0.57, 0.82), float2(0.98, -0.20), float2(0.34, 0.94)};
        [unroll] for (int w = 0; w < 4; ++w) {
            const float k = 6.2831853 / L[w];
            const float c = sqrt(9.81 / k);
            slope += D[w] * S[w] * cos(k * (dot(D[w], P.xz) - c * t) + float(w) * 1.7);
        }
        slope *= saturate(1.0 - dist / 500.0) * 0.85 + 0.15;
    }
    const float e = 0.08;
    const float h0 = rippleHeight(noiseTex, ns, P.xz, t), hx = rippleHeight(noiseTex, ns, P.xz + float2(e, 0), t),
                hz = rippleHeight(noiseTex, ns, P.xz + float2(0, e), t);
    const float rip = 0.035 * saturate(1.0 - dist / 250.0);
    const float3 N = normalize(float3(-(hx - h0) / e * rip - slope.x, 1.0, -(hz - h0) / e * rip - slope.y));
    const float3 V = normalize(f.cameraPos.xyz - P);
    const float NdotV = saturate(dot(N, V));
    const float F = 0.02 + 0.98 * pow(1.0 - NdotV, 5.0);
    const float2 suv = vin.position.xy * f.waterParams.zw;
    float3 refl;
    if (f.waterParams.y > 0.5) {
        refl = reflTex.Sample(rs, suv + N.xz * 0.06).rgb;
    } else {
        float3 R = reflect(-V, N);
        R.y = abs(R.y);
        refl = skyRadiance(R);
        if (f.screenFx.y > 0.5) {
            const float4 hit = traceReflection(P, R, dist);
            refl = lerp(refl, hit.rgb, hit.a);
        }
    }
    const float shadow = shadowFactor(shadowMap, shadowSampler, P, float3(0, 1, 0)) * cloudShadow(P);
    const float3 H = normalize(f.sunDir.xyz + V);
    const float spec = ggxSpec(saturate(dot(N, H)), max(NdotV, 1e-3), saturate(dot(N, f.sunDir.xyz)), 0.07);
    const float3 sunSpec = f.sunColor.rgb * spec * 0.02 * saturate(dot(N, f.sunDir.xyz)) * shadow;
    // Water body from the mesh: color (puddles muddy brown, lakes and sea blue-green), material.y = depth (m) at
    // which it turns opaque, material.z = opacity; see-through at the very edge.
    const float3 skyUp = lerp(f.skyHorizon.rgb, f.skyZenith.rgb, 0.5);
    const float3 body = vin.color * (skyUp + f.sunColor.rgb * 0.15 * shadow);
    // Soft shoreline: the last few cm of depth (height map, which is smoothed over a cell) and, with the scene depth,
    // of the view path to the ground behind fade the water out where it meets the bank.
    float edge = smoothstep(0.0, 0.06, depth);
    if (f.screenFx.x > 0.5) {
        const float zw = viewDepth(P);
        const float pathBehind = max(depthLin.SampleLevel(dls, suv, 0).r - zw, 0.0) * dist / max(zw, 1e-3);
        edge *= smoothstep(0.0, 0.25, pathBehind);
    }
    // Shore foam on lakes and the sea (opaque depth above 3 m): bands running up the beach, broken by noise, in the
    // last half metre of depth. Rivers get none: a white rim along both banks read as ice.
    float foam = 0.0;
    if (vin.material.y > 3.0) {
        const float n = noiseTex.Sample(ns, P.xz * 0.23 + float2(t * 0.013, -t * 0.009)).g;
        const float band = sin(depth * 16.0 - t * 1.3 + n * 7.0) * 0.5 + 0.5;
        foam = (1.0 - smoothstep(0.03, 0.5, depth)) * saturate(n * 1.6 - 0.45 + band * 0.55) * edge *
               saturate(1.5 - dist / 300.0);
    }
    const float3 foamCol = float3(0.80, 0.84, 0.86) * (skyUp * 0.9 + f.sunColor.rgb * 0.30 * saturate(f.sunDir.y) * shadow);
    const float4 fog = fogAt(P, f.cameraPos.xyz);
    if (f.screenFx.x > 0.5) {
        // Refraction: the bed seen through the surface, offset by the wave normal (undistorted where the offset
        // sample lies in front of the water), absorbed per channel along the view path through the water.
        const float zw = viewDepth(P);
        float2 ruv = suv + N.xz * min(depth, 2.0) * 0.35 / max(zw, 0.5);
        float zb = depthLin.SampleLevel(dls, ruv, 0).r;
        if (zb < zw) {
            ruv = suv;
            zb = depthLin.SampleLevel(dls, ruv, 0).r;
        }
        // Outside the baked square the horizon seabed is under the open sea quads, so the path is measured the same way.
        const float path = max(zb - zw, 0.0) * dist / max(zw, 1e-3);
        const float3 tint = vin.color / max(max(vin.color.r, vin.color.g), max(vin.color.b, 1e-4));
        const float3 sigma = ((1.0 - tint) * 2.5 + 1.2) / vin.material.y;
        const float3 T = lerp((float3)1.0, exp(-sigma * min(path, 1e3)), vin.material.z);
        const float3 bed = sceneTex.SampleLevel(scs, ruv, 0).rgb;
        const float3 scatter = body * (1.0 - T);
        float3 lit = ((scatter * (1.0 - F) + refl * F + sunSpec) * (1.0 - foam) + foamCol * foam);
        lit = lit * (1.0 - fog.a) + fog.rgb * fog.a;
        // The bed in the copy is already fogged.
        const float3 col = bed * T * (1.0 - F) * (1.0 - foam) + lit;
        return float4(col * edge, edge);
    }
    const float aBody = saturate(depth / vin.material.y) * vin.material.z;
    float3 src = (body * aBody * (1.0 - F) + refl * F + sunSpec) * edge;
    float srcAlpha = edge * (1.0 - (1.0 - F) * (1.0 - aBody));
    src = lerp(src, foamCol * edge, foam);
    srcAlpha = lerp(srcAlpha, edge, foam);
    // Fog over the premultiplied contribution (the background is already fogged).
    src = src * (1.0 - fog.a) + fog.rgb * fog.a * srcAlpha;
    return float4(src, srcAlpha);
}
#endif

// ---------------------------------------------------------------- waterfalls (render-only)
// Mesh from render/LevelScene buildLevelWaterfallMesh: color.xy piece coordinates, color.z kind (0 sheet, 1 foam,
// 2 mist); material x: drop (m), y: river width (m), z: seed. Premultiplied alpha, no depth write.

#if defined(TF_ENTRY_waterfall_fs)
Texture2DArray<float> shadowMap : TEX(0);
SamplerComparisonState shadowSampler : SMP(0);
Texture2D<float4> noiseTex : TEX(1);
SamplerState ns : SMP(1);
Texture2D<float4> heightTex : TEX(2);
SamplerState cs : SMP(2);
Texture2D<float4> depthLin : TEX(3); // view depth of the opaque scene (far plane when not available)
SamplerState dls : SMP(3);

float4 waterfall_fs(LitOut vin, bool front : SV_IsFrontFace) : SV_Target0 {
    const float3 P = vin.worldPos;
    const float t = f.cameraPos.w;
    const float kind = vin.color.z;
    const float drop = vin.material.x, width = vin.material.y, seed = vin.material.z;
    const float3 V = normalize(f.cameraPos.xyz - P);
    float3 N = normalize(vin.normal);
    if (!front) N = -N;
    float alpha, white, translucency, glassy = 0.0;
    if (kind < 0.5) {
        // Sheet: streaks stretched along the fall, scrolling with the water (the coordinate is the fall time, so the
        // pattern accelerates like the water). Thin film near the lip, aerated and torn toward the foot.
        const float u = vin.color.x, s = vin.color.y;
        const float T = sqrt(2.0 * (drop + 0.3) / 9.81);
        const float2 q = float2(u * width * 0.5 + seed * 13.0, (s * T - t) * 1.3);
        const float n1 = noiseTex.Sample(ns, float2(q.x * 0.09, q.y * 0.045)).g;
        const float n2 = noiseTex.Sample(ns, float2(q.x * 0.31 + 0.37, q.y * 0.16)).g;
        const float n3 = noiseTex.Sample(ns, float2(q.x * 0.9 + 0.11, q.y * 0.5)).g;
        const float streak = saturate(n1 * 0.7 + n2 * 0.45 + n3 * 0.25 - 0.2);
        const float rag = 0.08 + 0.22 * n2;
        const float edge = smoothstep(0.0, rag, u) * smoothstep(1.0, 1.0 - rag, u);
        const float torn = smoothstep(0.35, 1.0, s);
        const float lip = smoothstep(0.0, 0.05, s);
        alpha = saturate(lerp(0.35 + 0.45 * streak, streak * 1.4 - 0.25, torn)) * edge * lip;
        white = saturate(0.1 + 0.6 * streak + 0.8 * s);
        // Seen from above, the smooth upper part of the curtain is glassy dark water with light streaks, not foam.
        glassy = smoothstep(0.25, 0.75, V.y) * (1.0 - smoothstep(0.25, 0.75, s));
        white = lerp(white, 0.05 + 0.35 * streak * streak, glassy);
        alpha = lerp(alpha, (0.6 + 0.3 * streak) * edge * lip, glassy);
        translucency = 0.6;
    } else if (kind < 1.5) {
        // Foam: churned water spreading outward from the impact.
        const float2 l = vin.color.xy * 2.0 - 1.0;
        const float r = length(l);
        const float a = atan2(l.y, l.x);
        const float R = 0.55 * width + 0.12 * drop;
        const float n1 = noiseTex.Sample(ns, float2(a * 0.6 + seed, r * R * 0.08 - t * 0.12)).g;
        const float n2 = noiseTex.Sample(ns, P.xz * 0.21 + float2(t * 0.03, -t * 0.02)).g;
        const float foam = saturate(n1 * 0.8 + n2 * 0.6 - 0.25);
        alpha = saturate(foam * 1.3 + 0.25 * (1.0 - r) - 0.1) * smoothstep(1.0, 0.3, r) * 0.75;
        white = 0.7 + 0.25 * foam;
        translucency = 0.2;
        N = float3(0, 1, 0);
    } else {
        // Mist: soft rising spray, faded at the card edges, edge-on, against the ground and close to the camera.
        const float u = vin.color.x, v = vin.color.y;
        const float n1 = noiseTex.Sample(ns, float2(u * 0.55 + seed, v * 0.35 - t * 0.05)).g;
        const float n2 = noiseTex.Sample(ns, float2(u * 1.3 - t * 0.02, v * 0.9 - t * 0.11 + seed)).g;
        const float puff = saturate(n1 * 0.9 + n2 * 0.5 - 0.35);
        const float terrain = heightTex.SampleLevel(cs, groundUV(P.xz), 0).r;
        const float dist = length(f.cameraPos.xyz - P);
        const float facing = saturate(abs(dot(N, V)) * 1.6 - 0.2);
        // Soft particle: fades where the card meets the opaque scene (rocks, banks, the vehicle).
        const float zScene = depthLin.SampleLevel(dls, vin.position.xy * f.waterParams.zw, 0).r;
        const float soft = saturate((zScene - dot(P - f.cameraPos.xyz, f.cameraFwd.xyz)) / 2.5);
        alpha = puff * sin(3.14159 * u) * smoothstep(0.0, 0.2, v) * (1.0 - v) * (1.0 - v) * 0.32 * facing *
                saturate((P.y - terrain) * 0.5) * saturate((dist - 3.0) / 10.0) * saturate(drop / 12.0) * soft;
        white = 0.95;
        translucency = 1.0;
        N = V;
    }
    if (alpha < 0.002) discard;
    // Light: wrapped sun diffuse with forward scattering through the water, sky ambient, a small sun glint.
    const float shadow = shadowFactor(shadowMap, shadowSampler, P, N) * cloudShadow(P);
    const float3 L = f.sunDir.xyz;
    const float wrap = saturate((dot(N, L) + 0.5) / 1.5);
    const float forward = pow(saturate(dot(-V, L)), 6.0) * translucency;
    const float3 skyUp = lerp(f.skyHorizon.rgb, f.skyZenith.rgb, 0.5);
    const float3 albedo = lerp(float3(0.20, 0.30, 0.30), float3(0.86, 0.90, 0.92), white);
    float3 col = albedo * (skyUp * 0.9 + f.sunColor.rgb * shadow * (wrap * 0.30 + forward * 0.5));
    if (kind < 0.5) {
        const float3 H = normalize(L + V);
        col += f.sunColor.rgb * shadow * ggxSpec(saturate(dot(N, H)), max(saturate(dot(N, V)), 1e-3), saturate(dot(N, L)), 0.18) * 0.04;
        const float F = 0.02 + 0.98 * pow(1.0 - saturate(abs(dot(N, V))), 5.0);
        col += skyRadiance(reflect(-V, N)) * lerp(F, 0.25 + 0.6 * F, glassy);
    }
    const float4 fog = fogAt(P, f.cameraPos.xyz);
    col = col * (1.0 - fog.a) + fog.rgb * fog.a;
    return float4(col * alpha, alpha);
}
#endif

// ---------------------------------------------------------------- grass (GPU-instanced blades)

struct GrassUniforms {
    float4 lod;      // x: fine cell (m), y: LOD level, z: blades per chunk side, w: first chunk index
    float4 fade;     // x: this LOD's outer radius, y: fade band, z: max distance, w: blade segments
    float4 blade;    // x: height scale, y: width (m), z: wind, w: chunk size (m)
    float4 extra;    // x: radians per pixel, y: flower density, z: fine detail (0/1)
    float4 wind;     // xy: wind direction, z: gust speed (m/s), w: gust strength
    float4 wheels[4]; // xyz: wheel contact point, w: flatten radius (0 = none)
};

// Species. The cut kinds (seed head, clover, flower, wheat) get their outline in the fragment shader; their strip
// is a conservative envelope. The far LOD (one triangle) draws every kind as a plain blade.
#define GK_LAWN 0u
#define GK_TALL 1u
#define GK_SEED 2u
#define GK_CLOVER 3u
#define GK_FLOWER 4u
#define GK_WHEAT 5u
#define GK_CUT 16u    // flag: the fragment shader cuts the outline
#define GK_DETAIL 32u // flag: awns, spikelets, leaf marks

// Flower kinds: 0 daisy, 1 buttercup, 2 bellflower, 3 poppy, 4 clover head.
float3 flowerPetal(uint sub, float r) {
    switch (sub) {
    case 0: return lerp(float3(0.78, 0.78, 0.72), float3(0.80, 0.70, 0.74), r * 0.4);
    case 1: return lerp(float3(0.86, 0.58, 0.015), float3(0.92, 0.70, 0.04), r);
    case 2: return lerp(float3(0.26, 0.14, 0.62), float3(0.16, 0.22, 0.70), r);
    case 3: return lerp(float3(0.62, 0.035, 0.02), float3(0.75, 0.08, 0.03), r);
    default: return lerp(float3(0.50, 0.20, 0.42), float3(0.68, 0.60, 0.62), step(0.7, r));
    }
}

// Travelling gusts (0..1): bands moving downwind with wobbly fronts, modulated by a slower swell.
float windGust(float2 p, float time, float2 wd, float speed) {
    const float a = dot(p, wd), b = dot(p, float2(-wd.y, wd.x));
    const float front = sin((a - speed * time) * 0.42 + sin(b * 0.19 + time * 0.11) * 1.8) * 0.5 + 0.5;
    const float swell = sin((a - speed * 0.8 * time) * 0.13 + b * 0.07 + 1.3) * 0.5 + 0.5;
    const float ripple = sin((a * 0.7 + b * 0.7 - speed * 1.1 * time) * 0.8) * 0.5 + 0.5;
    return saturate(front * front * (0.45 + 0.75 * swell) + 0.2 * ripple * swell);
}

struct GrassOut {
    float4 position : SV_Position;
    [[vk::location(0)]] float3 worldPos : TEXCOORD0;
    [[vk::location(1)]] float3 normal : TEXCOORD1; // blade face normal (two-sided)
    [[vk::location(2)]] float3 across : TEXCOORD2; // world direction across the blade (rounded normal, flower head basis)
    [[vk::location(3)]] float3 color : TEXCOORD3;
    [[vk::location(4)]] float4 shape : TEXCOORD4; // x: side -1..1, y: t (0 root .. 1 tip, 2 flower head), z: offset from the axis (m), w: gust
    [[vk::location(5)]] float2 uv : TEXCOORD5; // flower head coordinates (-1..1)
    [[vk::location(6)]] nointerpolation float2 widths : TEXCOORD6; // half widths (m): stem, ear / head / leaf
    [[vk::location(7)]] nointerpolation uint kind : TEXCOORD7; // GK_* | flags | (flower kind << 8)
    [[vk::location(8)]] nointerpolation float ao : TEXCOORD8; // root occlusion of this clump (0 none .. 1 dense)
};

#if defined(TF_ENTRY_grass_vs)
ConstantBuffer<GrassUniforms> g : UBO(1);
Texture2D<float4> heightTex : TEX(0);
SamplerState cs : SMP(0);
Texture2D<float4> mask : TEX(1);
SamplerState ms : SMP(1);
Texture2D<float4> trample : TEX(2);
SamplerState ts : SMP(2);
Texture2D<float4> noiseTex : TEX(3);
SamplerState ns : SMP(3);
StructuredBuffer<float4> chunks : TEX(4);

GrassOut grass_vs(uint vid : SV_VertexID, uint iid : SV_InstanceID) {
    GrassOut vout;
    const int perSide = int(g.lod.z);
    const int perChunk = perSide * perSide;
    const int chunk = int(g.lod.w) + int(iid) / perChunk;
    const int cell = int(iid) % perChunk;
    const int ci = cell % perSide, cj = cell / perSide;
    const int lod = int(g.lod.y);
    const float levelScale = exp2(g.lod.y);
    const float fine = g.lod.x;
    const float4 ch = chunks[chunk];
    // Blade identity on the finest grid, so coarser LODs keep a subset of the same blades.
    const float2 gi = floor(ch.xy / fine + 0.5) + float2(ci, cj) * levelScale;
    const float r1 = hash12(gi), r2 = hash12(gi + 17.31), r3 = hash12(gi + 41.7), r4 = hash12(gi + 73.1);
    const float r5 = hash12(gi + 93.7), r6 = hash12(gi + 121.3), r7 = hash12(gi + 151.9);
    const float2 cellRoot = (gi + 0.5) * fine;
    const float2 jitter = float2(r1, r2) - 0.5;

    const float4 m = mask.SampleLevel(ms, groundUV(cellRoot), 0);
    const bool wheat = r6 < m.a * 1.1 - 0.05;
    // Grass grows in clumps (0.55 m cells): blades lean out of the clump centre and share a height; wheat is sown in
    // rows along x (small jitter across the rows).
    const float2 clumpCell = floor(cellRoot / 0.55);
    const float2 clumpC = (clumpCell + 0.25 + 0.5 * float2(hash12(clumpCell + 5.1), hash12(clumpCell + 9.7))) * 0.55;
    float2 root = cellRoot + jitter * (wheat ? float2(0.9, 0.2) : ((float2)(0.9))) * fine;
    if (!wheat) root = lerp(root, clumpC, 0.2);

    const float dist = length(root - f.cameraPos.xz);
    // Blades that do not exist in the next (coarser) LOD fade out before its boundary; the others widen.
    const bool inNext = (ci % 2 == 0) && (cj % 2 == 0);
    const float edgeT = smoothstep(g.fade.x - g.fade.y, g.fade.x, dist);
    float scale = inNext ? 1.0 : 1.0 - edgeT;
    const float widen = inNext ? lerp(1.0, 1.6, edgeT) : 1.0;
    scale *= 1.0 - smoothstep(g.fade.z * 0.8, g.fade.z, dist);

    const float2 uvRoot = groundUV(root);
    const float hC = heightTex.SampleLevel(cs, uvRoot, 0).r;
    const float hX = heightTex.SampleLevel(cs, groundUV(root + float2(0.5, 0.0)), 0).r;
    const float hZ = heightTex.SampleLevel(cs, groundUV(root + float2(0.0, 0.5)), 0).r;
    const float slope = length(float2(hX - hC, hZ - hC)) * 2.0;
    const GrassZones zones = grassZones(noiseTex, ns, root, m.r, slope);
    // Tire marks: R = flattened by a rolling wheel, G = torn out by a spinning or sliding wheel.
    const float2 uvTrample = trampleUV(root);
    const float2 trampled = all(uvTrample >= 0.0) && all(uvTrample <= 1.0) ? trample.SampleLevel(ts, uvTrample, 0).rg : ((float2)(0.0));
    const bool alive = (wheat || r3 < m.r * zones.patch * 1.15) && scale > 0.02 && r2 >= trampled.g * 1.3 - 0.2;

    // ---- species
    uint kind = GK_LAWN, sub = 0;
    if (wheat) {
        kind = GK_WHEAT;
    } else {
        // Flower clusters: at most one per 3.2 m cell, plus rare lone flowers.
        const float2 fc = floor(root / 3.2);
        const float2 fCentre = (fc + 0.3 + 0.4 * float2(hash12(fc + 7.3), hash12(fc + 13.9))) * 3.2;
        const float fRad = 0.45 + 0.5 * hash12(fc + 31.1);
        const bool cluster = hash12(fc + 211.0) < 0.55 * g.extra.y;
        float flowerP = cluster ? saturate(1.0 - length(root - fCentre) / fRad) * 0.65 : 0.0;
        flowerP = (flowerP + 0.006 * g.extra.y) * (1.0 - 0.6 * zones.dry);
        if (r5 < flowerP) {
            kind = GK_FLOWER;
            const float fs = cluster && flowerP > 0.01 ? hash12(fc + 57.7) : r4;
            sub = fs < 0.36 ? 0u : fs < 0.66 ? 1u : fs < 0.90 ? (zones.clover > 0.4 ? 4u : 2u) : 3u;
        } else if (r7 < zones.clover * 0.75) {
            kind = GK_CLOVER;
        } else if (r7 < zones.clover * 0.75 + zones.tall * 0.7) {
            kind = r2 < 0.35 ? GK_SEED : GK_TALL;
        }
    }

    // ---- size and lean
    const float clumpH = 0.75 + 0.5 * hash12(clumpCell + 3.3);
    const float dryBlade = saturate(zones.dry * 1.4 - 0.3 + (r4 - 0.5) * 0.6);
    const float wpx = dist * g.extra.x; // one pixel at this distance (m)
    const float bladeW = max(0.30 * fine * sqrt(fine / 0.1) * (0.7 + 0.6 * r3), 0.012) * levelScale * widen;
    float height, halfW, headW = 0.0, baseLean, flex;
    switch (kind) {
    case GK_WHEAT:
        height = (0.78 + 0.22 * r1) * (0.85 + 0.15 * m.a);
        halfW = max(0.0035, wpx * 0.4) * widen;
        headW = max(0.0075, wpx * 0.5) * widen;
        baseLean = 0.04 + 0.10 * r2;
        flex = 0.75;
        break;
    case GK_SEED:
        height = (0.50 + 0.40 * r1) * clumpH;
        halfW = max(bladeW * 0.12, wpx * 0.4);
        headW = max(0.012 + 0.008 * r3, wpx * 0.6) * widen;
        baseLean = 0.10 + 0.30 * r2;
        flex = 1.2;
        break;
    case GK_TALL:
        height = (0.38 + 0.42 * r1) * clumpH;
        halfW = bladeW * 0.40;
        baseLean = 0.15 + 0.40 * r2;
        flex = 1.0;
        break;
    case GK_CLOVER:
        height = 0.06 + 0.06 * r1;
        halfW = max(0.002, wpx * 0.4);
        headW = max(0.016 + 0.008 * r3, wpx * 0.6) * levelScale * widen;
        baseLean = 0.9 + 0.3 * r2;
        flex = 0.2;
        break;
    case GK_FLOWER:
        height = (sub == 3 ? 0.36 + 0.2 * r1 : sub == 4 ? 0.10 + 0.06 * r1 : 0.18 + 0.24 * r1) * (0.8 + 0.2 * clumpH);
        halfW = max(0.003, wpx * 0.4);
        headW = (sub == 3 ? 0.046 : sub == 1 ? 0.027 : sub == 4 ? 0.024 : 0.030) * (0.85 + 0.3 * r3);
        headW = max(headW, wpx * 1.2) * widen;
        baseLean = 0.05 + 0.2 * r2;
        flex = 0.6;
        break;
    default: // lawn
        height = (0.18 + 0.24 * r1) * (0.5 + 0.5 * m.r) * (1.0 + 0.5 * zones.tall) * clumpH;
        halfW = bladeW * 0.5;
        baseLean = 0.30 + 0.55 * r2;
        flex = 0.6;
        break;
    }
    if (kind != GK_WHEAT) {
        height *= 1.0 - 0.2 * dryBlade;
        baseLean += 0.25 * dryBlade;
    }
    height *= g.blade.x * scale;

    const float yaw = r4 * 2.0 * PI;
    const float2 facing = float2(cos(yaw), sin(yaw));
    const float2 fromClump = root - clumpC;
    const float2 leanDir = wheat ? facing : normalize(fromClump + facing * 0.25 + 1e-4);

    // Wheels flatten what they stand on (only when the contact is near this ground height).
    const float terrain = hC - 0.03;
    float2 push = ((float2)(0.0));
    float flatAmt = 0.0;
    for (int w = 0; w < 4; ++w) {
        if (g.wheels[w].w <= 0.0) continue;
        const float2 dlt = root - g.wheels[w].xz;
        const float dd = length(dlt);
        const float infl = (1.0 - smoothstep(g.wheels[w].w * 0.5, g.wheels[w].w * 1.4, dd)) *
                           (1.0 - smoothstep(0.3, 0.8, abs(g.wheels[w].y - terrain)));
        flatAmt = max(flatAmt, infl);
        push += dd > 1e-3 ? dlt / dd * infl : ((float2)(0.0));
    }
    flatAmt = max(flatAmt, trampled.r * 0.95);
    const float2 flatDir = length(push) > 0.01 ? normalize(push) : leanDir;

    // Wind: travelling gusts bend everything downwind; a small flutter across the wind.
    const float time = f.cameraPos.w;
    const float2 wd = g.wind.xy, wp = float2(-wd.y, wd.x);
    const float gust = windGust(root, time, wd, g.wind.z);
    const float windAmt = flex * g.wind.w * f.options.z * (0.08 + 0.55 * gust);
    const float flutter = sin(time * (5.0 + 4.0 * r3) + r1 * 30.0) * 0.07 * flex * (0.3 + gust);
    const float2 leanVec = leanDir * baseLean + wd * windAmt + wp * flutter + flatDir * flatAmt * 1.4;
    const float phi = min(length(leanVec), 1.45);
    const float2 dir = leanVec / max(length(leanVec), 1e-4);
    const float L = height * (1.0 - 0.2 * flatAmt);
    // Quadratic Bezier: the base leaves the ground at a fraction of the tip angle, so blades curve toward the tip.
    const float stiff = kind == GK_WHEAT ? 0.6 : 0.35;
    const float3 P1 = float3(dir.x * sin(phi * stiff), cos(phi * stiff), dir.y * sin(phi * stiff)) * (L * 0.5);
    const float3 P2 = float3(dir.x * sin(phi), cos(phi), dir.y * sin(phi)) * L;
    const float3 base = float3(root.x, terrain, root.y);

    // ---- vertex on the strip (or on the flower head quad)
    const bool cut = lod < 2 && (kind == GK_SEED || kind == GK_CLOVER || kind == GK_FLOWER || kind == GK_WHEAT);
    const int segs = int(g.fade.w);
    const int triCount = lod >= 2 ? 1 : 2 * segs;
    const int tri = int(vid) / 3, k = int(vid) % 3;
    const bool head = kind == GK_FLOWER && lod < 2 && tri >= triCount - 2;
    float t, side;
    if (lod >= 2) {
        t = k == 2 ? 1.0 : 0.0;
        side = k == 0 ? -1.0 : k == 1 ? 1.0 : 0.0;
    } else {
        const int sv = tri + k;
        const int stemSegs = kind == GK_FLOWER ? segs - 1 : segs;
        const int row = min(sv / 2, stemSegs);
        side = (sv % 2 == 0) ? -1.0 : 1.0;
        t = float(row) / float(stemSegs);
        if (kind == GK_SEED || kind == GK_WHEAT) {
            const float rows4[5] = {0.0, 0.45, 0.76, 0.88, 1.0};
            t = segs >= 4 ? rows4[row] : (row == 0 ? 0.0 : row == 1 ? 0.68 : 1.0);
        }
    }

    float3 p, T, across3;
    float hw = halfW;
    float2 uv = ((float2)(0.0));
    if (head) {
        // Flower head: a quad facing up and toward the camera, swaying with the stem tip.
        const float3 C = base + P2;
        float2 toCam = f.cameraPos.xz - C.xz;
        toCam = length(toCam) > 1e-3 ? normalize(toCam) : float2(1.0, 0.0);
        const float3 Nh = normalize(float3(toCam.x * 0.65 + dir.x * phi * 0.5, 0.75, toCam.y * 0.65 + dir.y * phi * 0.5));
        const float3 U = normalize(cross(Nh, float3(0.0, 0.0, 1.0)) + float3(1e-4, 0.0, 0.0));
        const float3 W = cross(Nh, U);
        const int hk = (tri == triCount - 2) ? k : (k == 0 ? 0 : k + 1);
        uv = float2(hk == 1 || hk == 2 ? 1.0 : -1.0, hk >= 2 ? 1.0 : -1.0);
        p = C + (U * uv.x + W * uv.y) * headW;
        T = W;
        across3 = U;
        t = 2.0;
        side = uv.x;
        vout.normal = Nh;
    } else {
        p = base + 2.0 * t * (1.0 - t) * P1 + t * t * P2;
        T = normalize(2.0 * (1.0 - t) * P1 + 2.0 * t * (P2 - P1) + float3(0.0, 1e-4, 0.0));
        if (kind == GK_CLOVER) {
            across3 = float3(-dir.y, 0.0, dir.x);
        } else if (kind == GK_LAWN || kind == GK_TALL) {
            across3 = float3(-facing.y, 0.0, facing.x);
        } else {
            // Thin stems, ears and heads turn their wide side to the camera.
            across3 = cross(T, f.cameraPos.xyz - p);
            across3 = length(across3) > 1e-5 ? normalize(across3) : float3(1.0, 0.0, 0.0);
        }
        across3 = normalize(across3 - T * dot(across3, T) + 1e-5);
        // Width profile: blades widest near the base with a pointed tip; cut kinds use an envelope.
        if (!cut) {
            hw = halfW * (1.0 - pow(t, 1.5)) * (1.0 + 0.25 * sin(PI * t));
        } else if (kind == GK_FLOWER) {
            hw = halfW;
        } else if (kind == GK_CLOVER) {
            hw = t < 0.4 ? halfW : headW;
        } else {
            hw = t < 0.6 ? halfW : max(headW, halfW) * (kind == GK_WHEAT ? 1.9 : 1.1);
        }
        p += across3 * side * hw;
        vout.normal = normalize(cross(across3, T));
    }

    if (!alive) p = float3(0.0, -1000.0, 0.0);
    vout.position = mul(f.viewProj, float4(p, 1.0));
    vout.worldPos = p;
    vout.across = across3;
    vout.shape = float4(side, t, side * hw, gust);
    vout.uv = uv;
    vout.widths = float2(halfW, headW);
    vout.kind = kind | (cut ? GK_CUT : 0u) | (g.extra.z > 0.5 ? GK_DETAIL : 0u) | (sub << 8);
    vout.ao = wheat ? 0.8 : saturate(m.r * zones.patch);

    // ---- colour: dark root, lighter tips, per-blade hue and value, per-clump tint, dry straw, golden wheat
    const float tc = min(t, 1.0);
    const float clumpTint = hash12(clumpCell + 23.9);
    float3 green = lerp(float3(0.060, 0.100, 0.016), float3(0.110, 0.128, 0.022), r1);
    green = lerp(green, green * float3(0.80, 1.02, 1.45), r4 * 0.6);  // blue-green to yellow-green
    green *= (0.80 + 0.40 * r3) * (0.9 + 0.2 * clumpTint);
    const float3 straw = lerp(float3(0.25, 0.18, 0.075), float3(0.38, 0.29, 0.13), r1);
    float3 rootC, tipC;
    if (kind == GK_WHEAT) {
        const float ripe = saturate(smoothstep(0.38, 0.60, noiseTex.SampleLevel(ns, root * 0.045 + 0.7, 0).g) * 0.85 +
                                    (r1 - 0.5) * 0.4);
        const float3 gold = lerp(float3(0.24, 0.26, 0.06), float3(0.50, 0.32, 0.08), ripe) * (0.85 + 0.3 * r3);
        rootC = lerp(float3(0.04, 0.06, 0.012), gold * 0.3, ripe);
        tipC = gold * (t >= 0.76 ? 1.12 : 0.95);
        // Bent stalks show their lighter sides: the gusts read as bright waves rolling over the field.
        tipC *= 1.0 + 0.30 * gust;
    } else if (kind == GK_FLOWER) {
        rootC = green * 0.4;
        tipC = head ? flowerPetal(sub, r3) : (lod >= 2 ? flowerPetal(sub, r3) : green * 1.1);
    } else if (kind == GK_CLOVER) {
        rootC = float3(0.020, 0.040, 0.012);
        tipC = float3(0.036, 0.082, 0.024) * (0.85 + 0.3 * r3);
    } else {
        rootC = green * 0.32;
        tipC = lerp(green * 1.35 + float3(0.020, 0.018, 0.0), straw, dryBlade * 0.85);
        if (kind == GK_TALL || kind == GK_SEED) tipC = lerp(tipC, float3(0.14, 0.135, 0.045), 0.3);
        if (kind == GK_SEED && t >= 0.7)
            tipC = lerp(lerp(float3(0.32, 0.25, 0.12), float3(0.26, 0.16, 0.13), r3 * r3), straw, dryBlade * 0.5);
    }
    vout.color = head ? tipC : lerp(rootC, tipC, smoothstep(0.0, 0.85, tc));
    // Flattened blades show their lighter, bruised sides; blades left standing in torn ground are soiled.
    vout.color = lerp(vout.color, vout.color * 1.1 + float3(0.010, 0.008, 0.0), trampled.r * 0.6);
    vout.color = lerp(vout.color, float3(0.070, 0.050, 0.025), trampled.g * 0.6);
    return vout;
}
#endif

#if defined(TF_ENTRY_grass_fs)
Texture2DArray<float> shadowMap : TEX(0);
SamplerComparisonState shadowSampler : SMP(0);

float4 grass_fs(GrassOut vin) : SV_Target0 {
    const float3 P = vin.worldPos;
    const float3 V = normalize(f.cameraPos.xyz - P);
    const uint kind = vin.kind & 15u, sub = vin.kind >> 8;
    const bool detail = (vin.kind & GK_DETAIL) != 0u;
    const float t = vin.shape.y, off = abs(vin.shape.z);
    float3 albedo = vin.color;
    float3 N = normalize(vin.normal);
    float thin = 1.0;  // light passing through
    float gloss = 0.0;
    if ((vin.kind & GK_CUT) != 0u) {
        if (kind == GK_FLOWER && t > 1.5) {
            const float2 uv = vin.uv;
            const float r = length(uv), a = atan2(uv.y, uv.x);
            float edge;
            float3 centre;
            float centreR;
            switch (sub) {
            case 0: edge = 0.60 + 0.40 * pow(abs(cos(a * 6.0)), 0.5); centre = float3(0.85, 0.52, 0.03); centreR = 0.30; break;
            case 1: edge = 0.74 + 0.26 * cos(a * 5.0); centre = float3(0.55, 0.50, 0.04); centreR = 0.18; break;
            case 2: edge = 0.60 + 0.40 * pow(abs(cos(a * 2.5)), 3.0); centre = float3(0.60, 0.55, 0.70); centreR = 0.15; break;
            case 3: edge = 0.84 + 0.16 * cos(a * 4.0); centre = float3(0.03, 0.02, 0.02); centreR = 0.20; break;
            default: edge = 0.80 + 0.20 * sin(a * 9.0) * sin(r * 13.0); centre = albedo; centreR = 0.0; break;
            }
            if (r > edge) discard;
            albedo = r < centreR ? centre : albedo * (0.75 + 0.3 * r);
            // A shallow dome so the petals catch the light unevenly.
            N = normalize(N + (vin.across * uv.x + cross(N, vin.across) * uv.y) * 0.35);
            thin = 0.8;
            gloss = sub == 1 ? 1.0 : 0.2;
        } else if (kind == GK_SEED) {
            if (t < 0.62) {
                if (off > vin.widths.x) discard;
            } else {
                const float u = saturate((t - 0.62) / 0.38);
                float prof = vin.widths.y * sqrt(sin(PI * u)) * (1.0 - 0.35 * u);
                if (detail) prof *= 0.62 + 0.38 * smoothstep(0.0, 0.6, frac(u * 15.0));
                if (off > max(prof, vin.widths.x)) discard;
                thin = 0.6;
            }
        } else if (kind == GK_CLOVER) {
            if (t < 0.4) {
                if (off > vin.widths.x) discard;
            } else {
                const float u = saturate((t - 0.4) / 0.6);
                const float prof = vin.widths.y * sqrt(saturate(sin(PI * min(u * 1.1, 1.0))));
                if (off > prof || (u > 0.82 && off < vin.widths.y * 0.3 * (u - 0.82) / 0.18)) discard;
                // The pale chevron of white clover.
                if (detail && abs(off / vin.widths.y * 0.7 - (u - 0.35)) < 0.07) albedo = albedo * 1.8 + float3(0.03, 0.035, 0.02);
                gloss = 0.4;
            }
        } else if (kind == GK_WHEAT) {
            const float earEnd = detail ? 0.92 : 0.97;
            if (t < 0.80) {
                if (off > vin.widths.x) discard;
            } else if (t < earEnd) {
                const float u = (t - 0.80) / (earEnd - 0.80);
                const float zig = abs(sin(u * PI * 9.0 + sign(vin.shape.z) * PI * 0.5));
                const float prof = vin.widths.y * sqrt(saturate(1.0 - pow(2.0 * u - 1.0, 4.0))) * (0.78 + 0.22 * zig);
                if (off > max(prof, vin.widths.x)) discard;
                albedo *= 0.85 + 0.3 * zig;
                thin = 0.5;
            } else {
                // Awns: a few hairs fanning out above the ear.
                const float u = (t - earEnd) / (1.0 - earEnd);
                const float spacing = vin.widths.y * (0.35 + 0.75 * u) * 0.5;
                const float q = vin.shape.z / spacing;
                const float lw = max(0.0010, fwidth(vin.shape.z) * 0.6);
                if (abs(q) > 2.4 || abs(q - round(q)) * spacing > lw) discard;
                albedo *= 1.15;
            }
        } else if (off > vin.widths.x * 1.05) { // flower stem
            discard;
        }
    }

    // Two-sided, rounded across the blade so a blade reads as a volume, softened toward the sky.
    if (dot(N, V) < 0.0) N = -N;
    if (t <= 1.0) N = normalize(N + vin.across * vin.shape.x * 0.6);
    const float3 Nl = normalize(lerp(N, float3(0.0, 1.0, 0.0), 0.3));
    const float3 L = f.sunDir.xyz;
    const float shadow = shadowFactor(shadowMap, shadowSampler, P, float3(0, 1, 0)) * cloudShadow(P);
    const float NdotL = saturate(dot(Nl, L) * 0.6 + 0.4); // wrapped diffuse
    // Root occlusion: the base of a dense clump sees little sky and little sun.
    const float rootAO = lerp(1.0 - 0.75 * vin.ao, 1.0, smoothstep(0.0, 0.7, min(t, 1.0)));
    const float vao = vehicleAO(P);
    const float3 indirect = albedo * ambientLight(Nl.y) * rootAO * vao;
    float3 direct = albedo * f.sunColor.rgb * NdotL * shadow * lerp(0.5, 1.0, rootAO) / PI;
    // Translucency: sunlight through the blades when looking toward the sun or at their shaded side (warm at sunset).
    const float toward = pow(saturate(dot(-V, L)), 3.0);
    const float through = saturate(-dot(N, L));
    const float trans = (toward * 0.9 + through * 0.4) * shadow * thin * smoothstep(0.05, 0.6, min(t, 1.0));
    direct += albedo * float3(1.05, 1.1, 0.55) * f.sunColor.rgb * trans * 0.75 / PI;
    // Sheen on the tips (glossy buttercups and clover).
    const float3 H = normalize(L + V);
    direct += f.sunColor.rgb * (0.015 + 0.06 * gloss) * pow(saturate(dot(N, H)), 24.0 + 40.0 * gloss) * shadow * min(t, 1.0);
    const float3 c = direct + indirect + albedo * headlightIrradiance(P, Nl) / PI;
    // Blades neither cast nor receive contact shadows (negative split): in the depth buffer every blade would shadow
    // its neighbours and the ground between them into a dark speckle.
    const float4 o = fogSplit(c, (float3)0.0, indirect, P);
    return float4(o.rgb, -1.0 - o.a);
}
#endif

// ---------------------------------------------------------------- tire marks (persistent, render-only)

// Must match TireMarkSegment in src/sim/TireMarks.h and MarkUniforms in src/render/Renderer.h.
struct TireMarkSegment {
    float4 p0u0;         // xyz: patch center at the start, w: distance along the ribbon (m)
    float4 p1u1;
    float4 side0Birth;   // xyz: half-width vector at p0, w: birth time (s)
    float4 side1Surface; // xyz: half-width vector at p1, w: SurfaceId + 16 * packed tread soil (packMarkSurface)
    float4 imprintSlip;  // x/y: imprint at p0/p1, z/w: slip at p0/p1
};

struct MarkUniforms {
    float4 info;   // x: first live segment, y: ring capacity, z: simulation time (s), w: fade time (s)
    float4 params; // x: lift along the contact normal (m)
    float4 gainA;  // per-surface intensity: Concrete, PackedDirt, Rock, LowGrip
    float4 gainB;  // Mud, Grass, Sand, Snow
    float4 gainC;  // Asphalt, Gravel
    float4 widthA; // mark width / tire width per surface, same order
    float4 widthB;
    float4 widthC;
    float4 ground; // level terrain under the marks: x 1 = textured level ground, y origin (m), z cell (m), w cells
};

// SurfaceId (src/track/SurfaceProperties.h).
static const int SURF_CONCRETE = 0;
static const int SURF_DIRT = 1;
static const int SURF_ROCK = 2;
static const int SURF_MUD = 4;
static const int SURF_GRASS = 5;
static const int SURF_SAND = 6;
static const int SURF_SNOW = 7;
static const int SURF_ASPHALT = 8;
static const int SURF_GRAVEL = 9;

bool softSurface(int s) {
    return s == SURF_DIRT || s == SURF_MUD || s == SURF_GRASS || s == SURF_SAND || s == SURF_SNOW || s == SURF_GRAVEL;
}

float surfaceParam(float4 a, float4 b, float4 c, int surface) {
    return surface < 4 ? a[surface] : surface < 8 ? b[surface - 4] : c[min(surface - 8, 3)];
}

struct MarkCorner {
    float3 world;
    float3 normal;
    float2 uv; // x: distance along the ribbon (m), y: -1..1 across
    float imprint;
    float slip;
    float fade;
    int surface;
    float4 soil;  // rgb: linear albedo of the soil on the tread, a: wetness
    bool visible; // whole segment
};

static const float2 kMarkCorners[6] = {float2(0, -1), float2(1, -1), float2(1, 1), float2(0, -1), float2(1, 1), float2(0, 1)};

#if defined(TF_ENTRY_mark_vs) || defined(TF_ENTRY_mark_mask_vs) || defined(TF_ENTRY_mark_fs)
ConstantBuffer<MarkUniforms> mu : UBO(1);
#endif

#if defined(TF_ENTRY_mark_vs) || defined(TF_ENTRY_mark_mask_vs)
StructuredBuffer<TireMarkSegment> segs : TEX(0);

MarkCorner markCorner(uint vid, uint iid) {
    const TireMarkSegment s = segs[(uint(mu.info.x) + iid) % uint(mu.info.y)];
    const float2 c = kMarkCorners[vid % 6];
    const bool end = c.x > 0.5;
    const float3 side = end ? s.side1Surface.xyz : s.side0Birth.xyz;
    float3 n = cross(s.p1u1.xyz - s.p0u0.xyz, side);
    n = length_squared(n) > 1e-10 ? normalize(n) : float3(0, 1, 0);
    n *= n.y < 0.0 ? -1.0 : 1.0;
    MarkCorner m;
    const uint word = uint(s.side1Surface.w + 0.5);
    m.surface = int(word & 15u);
    const uint code = word >> 4;
    const float3 q = float3(code & 31u, (code >> 5) & 31u, (code >> 10) & 31u) / 31.0;
    m.soil = float4(q * q * 0.5, float((code >> 15) & 7u) / 7.0);
    const float gain = surfaceParam(mu.gainA, mu.gainB, mu.gainC, m.surface);
    m.world = (end ? s.p1u1.xyz : s.p0u0.xyz) + side * c.y + n * mu.params.x;
    m.normal = n;
    m.uv = float2(end ? s.p1u1.w : s.p0u0.w, c.y);
    m.imprint = (end ? s.imprintSlip.y : s.imprintSlip.x) * gain;
    m.slip = (end ? s.imprintSlip.w : s.imprintSlip.z) * gain;
    m.fade = 1.0 - smoothstep(mu.info.w * 0.6, mu.info.w, mu.info.z - s.side0Birth.w);
    const float4 is = s.imprintSlip;
    m.visible = max(max(is.x, is.y), max(is.z, is.w)) * gain * m.fade > 0.003;
    return m;
}
#endif

struct MarkOut {
    float4 position : SV_Position;
    [[vk::location(0)]] float3 worldPos : TEXCOORD0;
    [[vk::location(1)]] float3 normal : TEXCOORD1;
    [[vk::location(2)]] float2 uv : TEXCOORD2;
    [[vk::location(3)]] float imprint : TEXCOORD3;
    [[vk::location(4)]] float slip : TEXCOORD4;
    [[vk::location(5)]] float fade : TEXCOORD5;
    [[vk::location(6)]] nointerpolation int surface : TEXCOORD6;
    [[vk::location(7)]] nointerpolation float4 soil : TEXCOORD7;
};

#if defined(TF_ENTRY_mark_vs)
MarkOut mark_vs(uint vid : SV_VertexID, uint iid : SV_InstanceID) {
    const MarkCorner c = markCorner(vid, iid);
    MarkOut vout;
    vout.position = c.visible ? mul(f.viewProj, float4(c.world, 1.0)) : float4(0.0, 0.0, -2.0, 1.0);
    vout.worldPos = c.world;
    vout.normal = c.normal;
    vout.uv = c.uv;
    vout.imprint = c.imprint;
    vout.slip = c.slip;
    vout.fade = c.fade;
    vout.surface = c.surface;
    vout.soil = c.soil;
    return vout;
}
#endif

// Off-road tread imprint: two staggered rows of angled lugs with a center groove. tv: -1..1 across the tire,
// u: m along the ribbon. Returns 1 where a lug pressed in.
float treadLugs(float u, float tv) {
    const float av = abs(tv);
    const float ph = frac(u / 0.085 + (tv < 0.0 ? 0.0 : 0.5) + av * 0.6);
    const float lug = smoothstep(0.05, 0.14, ph) * (1.0 - smoothstep(0.50, 0.59, ph));
    return lug * smoothstep(0.06, 0.14, av) * (1.0 - smoothstep(0.88, 0.98, av));
}

// Decal over the ground: the fragment reconstructs the ground material underneath, modifies it per surface (rubber,
// compacted or churned soil, wet rut with raised edges, pressed or torn grass) and lights it like the ground. The
// relief exists only in the shading normal; the collision ground stays flat.
#if defined(TF_ENTRY_mark_fs)
Texture2DArray<float> shadowMap : TEX(0);
SamplerComparisonState shadowSampler : SMP(0);
Texture2D<float4> noiseTex : TEX(1);
SamplerState ns : SMP(1);
Texture2D<float4> mask : TEX(2);
SamplerState maskSampler : SMP(2);
Texture2D<float4> terrainSplat : TEX(5);
SamplerState terrainSplatSampler : SMP(5);

// The textured level ground under a mark, built like terrain_fs (layers by the baked splat and the mask, macro
// tone, mud, wet ground, litter) without the rock wall beds and the far grass canopy, so a mark tints the ground it
// lies on instead of painting its own.
struct MarkGround {
    float3 albedo;
    float3 soil; // the soil layer alone (torn turf)
    float3 dn;
    float rough;
    float ao;
};

MarkGround levelGround(float3 P, float3 N, float4 m) {
    const float dist = length(P - f.cameraPos.xyz);
    const GroundCtx gc = groundCtx(P, dist);
    const float fadeTex = smoothstep(60.0, 250.0, dist);
    const float2 ij = (P.xz - mu.ground.y) / mu.ground.z;
    const float4 sp = terrainSplat.SampleLevel(terrainSplatSampler, ij / mu.ground.w, 0);
    const float sandSnow = sp.b * (255.0 / 127.5) - 1.0;
    const float2 nz = noiseTex.Sample(ns, P.xz * 0.13).rg;
    const float slope = sqrt(max(1.0 - N.y * N.y, 0.0)) / max(N.y, 0.1);
    const GrassZones zones = grassZones(noiseTex, ns, P.xz, m.r, slope);
    const float grass = saturate(m.r * lerp(zones.patch, 1.0, fadeTex) * 1.2);
    const float road = saturate(sp.r * 1.3 - 0.15 + (nz.r - 0.5) * 0.35) * step(0.01, sp.r);
    const float rock = saturate((sp.g - 0.5) * 2.2 + 0.5 + (nz.r - 0.5) * 0.6) * step(0.01, sp.g);
    const float sand = saturate((max(sandSnow, 0.0) - 0.5) * 1.6 + 0.5 + (nz.g - 0.5) * 0.4) * step(0.01, sandSnow);
    const float snow = saturate((max(-sandSnow, 0.0) - 0.5) * 2.6 + 0.5 + (nz.g - 0.5) * 0.7) * step(0.01, -sandSnow);
    LayerSample g = sampleLayer(LAYER_SOIL, gc);
    const float3 soil = g.albedo;
    if (grass > 0.001) {
        const LayerSample l = sampleLayer(LAYER_GRASS, gc);
        blendLayer(g, l, heightBlend(grass, l.height, g.height));
    }
    if (road > 0.001) {
        const LayerSample l = sampleLayer(LAYER_ROAD, gc);
        blendLayer(g, l, heightBlend(road, l.height, g.height));
    }
    if (sand > 0.001) {
        const LayerSample l = sampleLayer(LAYER_SAND, gc);
        blendLayer(g, l, heightBlend(sand, l.height, g.height));
    }
    if (rock > 0.001) {
        const LayerSample l = sampleLayer(LAYER_ROCK, gc);
        blendLayer(g, l, heightBlend(rock, l.height, g.height));
    }
    if (snow > 0.001) {
        const LayerSample l = sampleLayer(LAYER_SNOW, gc);
        blendLayer(g, l, heightBlend(snow, l.height + 0.3, g.height));
    }
    const float macro = noiseTex.Sample(ns, P.xz * 0.011).r;
    float3 albedo = g.albedo;
    float rough = g.rough;
    macroTone(noiseTex, ns, P.xz, albedo, rough);
    float flat = 1.0;
    const float mudBlend = smoothstep(0.30, 0.55, m.g + (nz.g - 0.5) * 0.30);
    if (mudBlend > 0.0) {
        const float wetN = smoothstep(0.45, 0.65, nz.r * 0.7 + macro * 0.3);
        const float wet = saturate(wetN * 0.8 + m.b * 0.5);
        const float3 mudAlb = lerp(float3(0.125, 0.078, 0.042), float3(0.045, 0.029, 0.016), wet) * (0.8 + 0.4 * g.height);
        albedo = lerp(albedo, mudAlb, mudBlend);
        rough = lerp(rough, lerp(0.85, 0.40, wet), mudBlend);
        flat = lerp(1.0, 0.5, mudBlend * wet);
    }
    groundWet(m.b, rock, road, sand, g.height, N.y, nz.g, albedo, rough, flat);
    const float canopy = smoothstep(0.05, 0.6, sp.a) * (1.0 - road) * (1.0 - snow);
    if (canopy > 0.0) {
        const float litterN = noiseTex.Sample(ns, P.xz * 0.37).r;
        const float lum = dot(albedo, float3(0.3, 0.5, 0.2));
        const float3 litter = lerp(float3(0.040, 0.032, 0.022), float3(0.085, 0.066, 0.042), litterN) * (0.55 + 2.2 * lum);
        albedo = lerp(albedo, litter, canopy * (0.8 + 0.2 * nz.g));
        rough = lerp(rough, 0.9, canopy * 0.5);
    }
    MarkGround r;
    r.albedo = albedo;
    float3 soilTone = soil;
    float soilRough = 0.0;
    macroTone(noiseTex, ns, P.xz, soilTone, soilRough);
    r.soil = soilTone;
    r.dn = (g.dn + groundDetailDn(groundDominant(grass, road, sand, rock, snow, false), gc, dist, N.y)) * flat;
    r.rough = rough;
    r.ao = lerp(1.0, g.ao, 0.8) * (1.0 - 0.35 * canopy);
    return r;
}

float4 mark_fs(MarkOut vin) : SV_Target0 {
    const float3 P = vin.worldPos;
    const float3 N0 = normalize(vin.normal);
    const float3 V = normalize(f.cameraPos.xyz - P);
    const int surface = vin.surface;
    const float u = vin.uv.x, v = vin.uv.y;
    const float tv = v * surfaceParam(mu.widthA, mu.widthB, mu.widthC, surface); // -1..1 under the tire
    const float atv = abs(tv);
    const float imprint = saturate(vin.imprint), slip = saturate(vin.slip);
    const float rag = noiseTex.Sample(ns, P.xz * 2.3).a;
    const float edge = 1.0 - smoothstep(0.80, 1.0, abs(v) + (rag - 0.5) * 0.25);
    const float lugAA = 1.0 - smoothstep(0.015, 0.05, fwidth(u)); // fade the tread pattern before it aliases
    const float4 m = sampleMask(mask, maskSampler, P.xz);
    const float clods = noiseTex.Sample(ns, P.xz * 5.3).b;
    const float streak = noiseTex.Sample(ns, float2(tv * 0.5, u * 0.05) + P.xz * 0.011).g;

    float3 albedo;
    float rough;
    float h;          // relief (m) for the bump normal
    float cover;      // blend weight over the ground
    float cavity = 0.0;
    const bool level = mu.ground.x > 0.5;
    float3 groundDn = ((float3)(0.0)); // level: normal offset of the ground texture (pressed flatter under the tire)
    float groundAo = 1.0;
    if (softSurface(surface)) {
        const int mat = surface == SURF_MUD ? MAT_MUD : surface == SURF_GRASS ? MAT_GRASS : surface == SURF_SAND ? MAT_SAND
                      : surface == SURF_SNOW ? MAT_SNOW : MAT_DIRT;
        GroundSurface ground = groundMaterial(mat, P, N0, m, noiseTex, ns, grassFar(P));
        GroundSurface soil = groundMaterial(MAT_DIRT, P, N0, float4(0.0, m.g, m.b, 0.0), noiseTex, ns, 0.0);
        const float under = 1.0 - smoothstep(0.82, 0.98, atv);
        const float lugs = treadLugs(u, tv) * lugAA * (1.0 - smoothstep(0.2, 0.7, slip));
        const float churn = smoothstep(0.05, 0.6, slip);
        if (level) {
            const MarkGround lg = levelGround(P, N0, m);
            ground.albedo = lg.albedo;
            ground.roughness = lg.rough;
            ground.bump = 0.0;
            soil.albedo = lg.soil;
            groundDn = lg.dn * lerp(1.0, 0.3, under * saturate(max(imprint, slip) * 1.5) * (1.0 - churn));
            groundAo = lg.ao;
        }
        const float baseH = ground.height * 0.035 * ground.bump;
        if (surface == SURF_MUD) {
            // Wet rut: dark glossy floor, lug pits holding water, edges pushed up into lighter berms.
            const float berm = smoothstep(0.88, 1.06, atv) * (1.0 - smoothstep(1.08, 1.3, atv));
            const float depth = 0.02 * imprint + 0.035 * slip;
            const float lumps = noiseTex.Sample(ns, P.xz * 2.6).b;
            h = -depth * under + (0.010 * imprint + 0.02 * slip) * berm * (0.6 + 0.8 * rag) - lugs * 0.007 * under +
                churn * (lumps - 0.5) * 0.02 * under + baseH * 0.25;
            // Water collects in the rut floor: darker and glossier than the wet mud around it.
            albedo = lerp(ground.albedo * 1.15, ground.albedo * 0.5, under) * (1.0 - 0.2 * lugs);
            albedo = lerp(albedo, ground.albedo * lerp(0.55, 1.05, lumps), churn * under * 0.7);
            rough = lerp(0.75, lerp(0.16, 0.30, lugs), under);
            rough = lerp(rough, lerp(0.20, 0.55, lumps), churn * under);
            cover = saturate(max(imprint, slip) * 1.2);
            cavity = under * (0.1 + 0.15 * lugs);
        } else if (surface == SURF_GRASS) {
            // Pressed undergrowth (lighter, yellower) or, under a spinning wheel, torn turf showing the soil.
            const float torn = smoothstep(0.08, 0.5, slip) * under;
            h = -0.006 * imprint * under - lugs * 0.004 + torn * (clods - 0.5) * 0.03 + baseH * 0.5;
            const float3 pressed = (ground.albedo * 1.12 + float3(0.006, 0.005, 0.0)) * (1.0 - 0.15 * lugs);
            const float shreds = smoothstep(0.55, 0.75, noiseTex.Sample(ns, P.xz * 9.1).a);
            const float3 tornC = lerp(soil.albedo * 0.85 * lerp(0.6, 1.15, clods), float3(0.05, 0.07, 0.015), shreds * 0.6);
            albedo = lerp(pressed, tornC, torn);
            rough = lerp(0.85, 0.95, torn);
            cover = saturate(max(imprint * 0.75, slip * 1.5));
            cavity = torn * 0.2;
        } else if (surface == SURF_SNOW) {
            // Snow: the tread packs a smoother, slightly darker and glossier track with crisp lugs; spinning throws up
            // loose granular snow at the sides.
            const float berm = smoothstep(0.85, 1.05, atv) * (1.0 - smoothstep(1.05, 1.3, atv));
            h = -(0.015 * imprint + 0.03 * slip) * under - lugs * 0.012 * under + (0.006 * imprint + 0.015 * slip) * berm +
                churn * (clods - 0.5) * 0.02 * under + baseH * lerp(1.0, 0.25, imprint);
            const float3 packed = ground.albedo * 0.84 * (1.0 - 0.1 * lugs);
            const float3 loose = ground.albedo * lerp(0.9, 1.05, clods);
            albedo = lerp(packed, loose, churn);
            rough = lerp(0.42, 0.7, churn);
            cover = saturate(max(imprint * 0.9, slip * 1.3));
            cavity = under * 0.08 * lugs;
        } else if (surface == SURF_GRAVEL) {
            // Loose gravel: the tread presses the stones into a flatter floor where the darker fines show and shoves
            // them aside into low ridges along both edges; wheelspin and slides scatter loose stones over the track.
            const float stones = noiseTex.Sample(ns, P.xz * 9.7).b;
            const float3 base = level ? ground.albedo : float3(0.30, 0.275, 0.23) * (0.8 + 0.35 * rag);
            const float berm = smoothstep(0.85, 1.05, atv) * (1.0 - smoothstep(1.05, 1.4, atv));
            const float stone = 1.0 - smoothstep(0.15, 0.35, stones); // round cells: single stones
            h = -(0.010 * imprint + 0.02 * slip) * under - lugs * 0.003 * under +
                (0.008 * imprint + 0.02 * slip) * berm * (0.6 + 0.8 * clods) + stone * 0.006 * (berm + churn * under);
            // On the textured level ground the stones are the ground's own: the track only darkens them a little
            // (fines pressed up between them) and the lugs a little more, with a stony, ragged edge.
            const float3 pressed = base * (level ? 0.66 : 0.72) * (1.0 - (level ? 0.22 : 0.08) * lugs) *
                                   (level ? 0.85 + 0.3 * stones : 1.0);
            const float3 ridge = base * lerp(0.95, level ? 1.08 : 1.2, clods);
            const float3 scattered = lerp(base * (level ? 0.75 : 0.62), base * (level ? 1.1 : 1.3), stone);
            albedo = lerp(lerp(pressed, ridge, berm), scattered, churn * under);
            rough = level ? ground.roughness : 0.93;
            cover = saturate(max(imprint * 0.8, slip * 1.3)) * (1.0 - smoothstep(0.7, 1.0, abs(v) + (stones - 0.5) * 0.4));
            cavity = under * 0.08 + berm * 0.05;
        } else if (surface == SURF_SAND) {
            // Dry sand: the tread presses crisp lugs into a slightly darker, smoother floor; spinning digs a loose
            // trench with sand heaped at the sides.
            const float berm = smoothstep(0.85, 1.05, atv) * (1.0 - smoothstep(1.05, 1.35, atv));
            h = -(0.012 * imprint + 0.03 * slip) * under - lugs * 0.010 * under + (0.006 * imprint + 0.02 * slip) * berm +
                churn * (clods - 0.5) * 0.02 * under + baseH * lerp(1.0, 0.3, imprint);
            const float3 pressed = ground.albedo * 0.86 * (1.0 - 0.12 * lugs);
            const float3 loose = ground.albedo * lerp(0.78, 1.08, clods);
            albedo = lerp(pressed, loose, churn);
            rough = 0.97;
            cover = saturate(max(imprint * 0.85, slip * 1.3));
            cavity = under * 0.12 * lugs;
        } else {
            // Packed dirt: compacted (darker, smoother) with the tread imprint; spinning smears it into loose soil.
            h = -(0.008 * imprint + 0.02 * slip) * under - lugs * 0.008 * under + churn * (clods - 0.5) * 0.03 * under +
                baseH * lerp(1.0, 0.4, imprint);
            // Level: the ground under it (a road layer on the trails, the soil layer off them), not a soil colour.
            const float3 dirtC = level ? ground.albedo : soil.albedo;
            const float3 compact = dirtC * (level ? 0.8 : 0.72) * (1.0 - 0.18 * lugs);
            const float3 loose = dirtC * lerp(0.75, 1.25, clods);
            albedo = lerp(compact, loose, churn);
            rough = lerp(level ? ground.roughness * 0.95 : 0.86, 0.97, churn);
            cover = saturate(max(imprint * 0.9, slip * 1.3));
            cavity = under * 0.1 * lugs;
        }
    } else {
        // Hard ground: rubber laid down by the sliding patch, streaky along the travel direction; the
        // circumferential grooves leave lighter lines.
        const int mat = surface == SURF_CONCRETE ? MAT_CONCRETE : surface == SURF_ROCK ? MAT_ROCK
                      : surface == SURF_ASPHALT ? MAT_ASPHALT : 4;
        const GroundSurface ground = groundMaterial(mat, P, N0, m, noiseTex, ns, grassFar(P));
        const float grooves = 1.0 - 0.6 * (1.0 - smoothstep(0.03, 0.08, abs(atv - 0.38))) -
                              0.6 * (1.0 - smoothstep(0.03, 0.08, atv));
        h = ground.height * 0.035 * ground.bump;
        const float rubber = saturate(slip * 1.1 * (0.7 + 0.6 * streak)) * lerp(1.0, grooves, 0.6 * lugAA);
        // Soil carried on the tread and printed back onto the hard ground: the lugs print first, the full width only
        // while the tread is still loaded with soil. Its colour and gloss come from the soil (wet mud dark and glossy,
        // dry dust light and matte); the print fades as the tread cleans.
        const float lugsPrint = treadLugs(u, tv) * lugAA;
        const float3 soilC = vin.soil.rgb * lerp(0.8, 1.15, clods);
        const float wetS = vin.soil.a;
        const float print = saturate(imprint * 1.3) * lerp(0.55 + 0.45 * lugsPrint, 1.0, smoothstep(0.55, 0.9, imprint)) *
                            (0.8 + 0.4 * rag);
        const float rubberW = rubber / max(rubber + print, 1e-4);
        albedo = lerp(soilC, float3(0.018, 0.017, 0.016) * (0.8 + 0.4 * streak), rubberW);
        rough = lerp(lerp(0.95, 0.35, wetS), mat == 4 ? 0.50 : 0.75, rubberW);
        h += print * (1.0 - rubberW) * (0.002 + 0.004 * lugsPrint);
        cover = max(rubber, print);
    }
    cover *= edge * vin.fade;
    float3 N = perturbNormal(N0, P, h, 1.0);
    if (level) N = normalize(N + groundDn);
    if (cover < 0.003) discard;
    if (f.options.x > 0.0) {
        const float2 cell = abs(frac(P.xz) - 0.5);
        albedo *= 1.0 - f.options.x * smoothstep(0.47, 0.5, max(cell.x, cell.y));
    }

    ShadeInput s;
    s.albedo = albedo;
    s.roughness = rough;
    s.metallic = 0.0;
    s.N = N;
    s.V = V;
    s.shadow = shadowFactor(shadowMap, shadowSampler, P, N0) * cloudShadow(P);
    s.ao = vehicleAO(P) * groundAo * (1.0 - cavity);
    s.specOcclusion = 0.75;
    s.emissive = ((float3)(0.0));
    s.clearCoat = 0.0;
    s.lamp = headlightIrradiance(P, N);
    // cover only blends the colour: the target keeps the ground's light split (lightSplit) in alpha.
    const float3 color = applyFog(shade(s), P, f.cameraPos.xyz);
    return float4(color, cover);
}
#endif

// Grass trample mask (R: flattened, G: torn) in ground-rect texture space; soft surfaces only.
struct MarkMaskOut {
    float4 position : SV_Position;
    [[vk::location(0)]] float2 amount : TEXCOORD0;
    [[vk::location(1)]] float v : TEXCOORD1;
};

#if defined(TF_ENTRY_mark_mask_vs)
MarkMaskOut mark_mask_vs(uint vid : SV_VertexID, uint iid : SV_InstanceID) {
    const MarkCorner c = markCorner(vid, iid);
    const bool soft = softSurface(c.surface);
    const float2 uv = trampleUV(c.world.xz);
    MarkMaskOut vout;
    vout.position = c.visible && soft ? float4(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, 0.5, 1.0) : float4(-2.0, -2.0, 0.5, 1.0);
    vout.amount = float2(c.imprint, c.slip) * c.fade;
    vout.v = c.uv.y;
    return vout;
}
#endif

#if defined(TF_ENTRY_mark_mask_fs)
float4 mark_mask_fs(MarkMaskOut vin) : SV_Target0 {
    const float edge = 1.0 - smoothstep(0.75, 1.0, abs(vin.v));
    return float4(saturate(vin.amount.x * 1.5) * edge, saturate(vin.amount.y * 2.0) * edge, 0.0, 1.0);
}
#endif

// ---------------------------------------------------------------- wheel spray (render-only particles)

// Must match SprayInstance in src/sim/WheelSpray.h.
struct SprayInstance {
    float4 posSize; // xyz, radius (m)
    float4 color;   // linear rgb, alpha
    float4 velKind; // xyz velocity (resting: ground normal), w: packSprayCode (kind * 2 + resting, 0.49 seed + 0.5 wet)
};

struct SprayOut {
    float4 position : SV_Position;
    [[vk::location(0)]] float3 worldPos : TEXCOORD0;
    [[vk::location(1)]] float2 uv : TEXCOORD1; // -1..1 over the billboard
    [[vk::location(2)]] float4 color : TEXCOORD2;
    [[vk::location(3)]] nointerpolation float3 axisA : TEXCOORD3; // world axes of the billboard (A: along the motion) or of the splat on the ground
    [[vk::location(4)]] nointerpolation float3 axisB : TEXCOORD4;
    [[vk::location(5)]] nointerpolation int kind : TEXCOORD5;
    [[vk::location(6)]] nointerpolation int flags : TEXCOORD6; // 1: resting splat, 2: wet
    [[vk::location(7)]] nointerpolation float seed : TEXCOORD7;
    [[vk::location(8)]] nointerpolation float3 dustBase : TEXCOORD8; // dust only: lighting per puff (ambient + headlights, sunlight before shadow), fog
    [[vk::location(9)]] nointerpolation float3 dustSun : TEXCOORD9;
    [[vk::location(10)]] nointerpolation float4 dustFog : TEXCOORD10;
};

#if defined(TF_ENTRY_spray_vs)
cbuffer SprayInfo : UBO(1) {
    float4 info;
};
StructuredBuffer<SprayInstance> ps : TEX(0);

SprayOut spray_vs(uint vid : SV_VertexID, uint iid : SV_InstanceID) {
    const SprayInstance p = ps[iid];
    const float2 c = kMarkCorners[vid % 6] * float2(2.0, 1.0) - float2(1.0, 0.0);
    const float3 P = p.posSize.xyz;
    const float3 toCam = normalize(f.cameraPos.xyz - P);
    const int code = int(p.velKind.w);
    const float fr = frac(p.velKind.w);
    const bool wet = fr >= 0.5;
    const float seed = saturate((fr - (wet ? 0.5 : 0.0)) / 0.49);
    const int kind = code / 2;
    const bool resting = (code & 1) != 0;
    float r = p.posSize.w;
    float alpha = p.color.a;
    float3 a, b, world;
    if (resting) {
        // Flat splat on the ground (normal in velKind.xyz), turned and stretched by the seed.
        const float3 N = normalize(p.velKind.xyz);
        const float ang = seed * 6.2832;
        const float3 t0 = normalize(cross(N, abs(N.z) < 0.9 ? float3(0, 0, 1) : float3(1, 0, 0)));
        const float3 t1 = cross(N, t0);
        a = t0 * cos(ang) + t1 * sin(ang);
        b = cross(N, a);
        world = P + a * (c.x * r) + b * (c.y * r * lerp(0.55, 1.0, frac(seed * 7.13)));
    } else {
        float3 right = cross(float3(0, 1, 0), toCam);
        right = length_squared(right) > 1e-6 ? normalize(right) : float3(1, 0, 0);
        const float3 up = cross(toCam, right);
        // Sub-pixel bits keep at least ~0.7 px on screen with less alpha. One particle stands for a cluster of real
        // droplets, so the alpha is not reduced all the way to the covered area.
        const float minR = info.y * length(f.cameraPos.xyz - P);
        if (kind != 2 && r < minR) {
            alpha *= max(sqrt(r / minR), 0.75);
            r = minR;
        }
        // Solid bits are stretched along their screen-plane velocity (1/40 s of travel, like a video camera shutter);
        // a longer streak is somewhat fainter.
        const float3 v = p.velKind.xyz - toCam * dot(p.velKind.xyz, toCam);
        const float vl = length(v);
        const bool stretch = kind != 2 && vl > 0.5;
        a = stretch ? v / vl : right;
        b = stretch ? normalize(cross(toCam, a)) : up;
        const float along = r + (stretch ? vl / 40.0 : 0.0);
        alpha *= max(lerp(1.0, r / along, 0.5), 0.75);
        world = P + a * (c.x * along) + b * (c.y * r);
    }
    SprayOut vout;
    vout.position = mul(f.viewProj, float4(world, 1.0));
    vout.worldPos = world;
    vout.uv = c;
    vout.color = float4(p.color.rgb, alpha);
    vout.axisA = a;
    vout.axisB = b;
    vout.kind = kind;
    vout.flags = (resting ? 1 : 0) | (wet ? 2 : 0);
    vout.seed = seed;
    vout.dustBase = ((float3)(0.0));
    vout.dustSun = ((float3)(0.0));
    vout.dustFog = ((float4)(0.0));
    if (kind == 2) {
        // A puff is soft and small on screen: one lighting value per puff, normal half toward the sky.
        const float3 N = normalize(float3(0, 0.6, 0) + toCam * 0.4);
        const float3 albedo = p.color.rgb;
        const float forward = pow(saturate(dot(-toCam, f.sunDir.xyz)), 4.0);
        vout.dustBase = albedo * (ambientLight(N.y) + headlightIrradiance(P, N) / PI);
        vout.dustSun = albedo * f.sunColor.rgb * (saturate(dot(N, f.sunDir.xyz)) / PI + 0.15 + 0.45 * forward);
        vout.dustFog = fogAt(P, f.cameraPos.xyz);
    }
    return vout;
}
#endif

#if defined(TF_ENTRY_spray_fs)
Texture2DArray<float> shadowMap : TEX(0);
SamplerComparisonState shadowSampler : SMP(0);
Texture2D<float4> noiseTex : TEX(1);
SamplerState ns : SMP(1);

float4 spray_fs(SprayOut vin) : SV_Target0 {
    const float3 P = vin.worldPos;
    const float3 V = normalize(f.cameraPos.xyz - P);
    const float2 uv = vin.uv;
    const float r2 = dot(uv, uv);
    const float n = noiseTex.Sample(ns, uv * 0.35 + vin.seed * 7.31).a;
    const bool resting = (vin.flags & 1) != 0, wet = (vin.flags & 2) != 0;
    float alpha = vin.color.a;
    float3 N;
    float rough = 0.9;
    float3 albedo = vin.color.rgb * (0.85 + 0.3 * n);
    if (vin.kind == 2) {
        // Dust: soft, lumpy puff, lit per puff (vertex stage); only the shape and a one-tap shadow per pixel.
        alpha *= exp(-2.6 * r2) * (0.55 + 0.9 * n);
        if (alpha < 0.004) discard;
        const float sh = shadowFactor(shadowMap, shadowSampler, P, float3(0, 1, 0), 0);
        const float3 c = vin.dustBase + vin.dustSun * sh;
        return float4(lerp(c, vin.dustFog.rgb, vin.dustFog.a), saturate(alpha));
    } else if (vin.kind == 3 && !resting) {
        // Grass bit: a thin torn blade.
        if (abs(uv.y) > 0.35 + 0.25 * n || r2 > 1.0) discard;
        N = normalize(V + float3(0, 0.5, 0));
        rough = 0.8;
    } else {
        // Droplets, specks, grains, clods and splats: irregular soft outline (the radius varies around the bit).
        const float rr = sqrt(r2);
        const float edgeN = noiseTex.Sample(ns, uv * 0.42 + float2(vin.seed * 17.0, vin.seed * 5.3)).a;
        const float edge = wet && vin.kind != 0 ? 0.78 + 0.22 * edgeN : 0.58 + 0.42 * edgeN;
        alpha *= 1.0 - smoothstep(edge - 0.2, edge, rr);
        if (alpha < 0.01) discard;
        if (resting) {
            N = normalize(cross(vin.axisA, vin.axisB));
            N = dot(N, V) < 0.0 ? -N : N;
            albedo *= wet ? 0.75 : 1.0;  // a wet splat darkens the ground under it
            rough = wet ? 0.2 : 0.9;
        } else {
            const float z = sqrt(saturate(1.0 - r2 / (edge * edge)));
            N = normalize(vin.axisA * uv.x + vin.axisB * uv.y + V * (z + 0.15));
            rough = vin.kind == 4 ? 0.08 : wet ? lerp(0.18, 0.42, vin.seed) : lerp(0.75, 0.95, vin.seed);
        }
    }
    ShadeInput s;
    s.albedo = albedo;
    s.roughness = rough;
    s.metallic = 0.0;
    s.N = N;
    s.V = V;
    s.shadow = shadowFactor(shadowMap, shadowSampler, P, N) * cloudShadow(P);
    s.ao = 1.0;
    s.specOcclusion = wet ? 1.0 : 0.5;
    s.emissive = ((float3)(0.0));
    s.clearCoat = 0.0;
    s.lamp = headlightIrradiance(P, N);
    if (!wet && vin.kind != 0 && !resting) {
        // Dry grains and grass bits are thin: they scatter sunlight through, strongest toward the sun.
        const float forward = pow(saturate(dot(-V, f.sunDir.xyz)), 4.0);
        s.emissive = s.albedo * f.sunColor.rgb * s.shadow * (0.10 + 0.30 * forward);
    }
    const float3 color = applyFog(shade(s), P, f.cameraPos.xyz);
    return float4(color, saturate(alpha));
}
#endif

// ---------------------------------------------------------------- debug lines

struct LineIn {
    [[vk::location(0)]] float3 position : TEXCOORD0;
    [[vk::location(1)]] float4 color : TEXCOORD1;
};

struct LineOut {
    float4 position : SV_Position;
    [[vk::location(0)]] float4 color : TEXCOORD0;
};

#if defined(TF_ENTRY_line_vs)
LineOut line_vs(LineIn vin) {
    LineOut vout;
    vout.position = mul(f.viewProj, float4(vin.position, 1.0));
    vout.color = vin.color;
    return vout;
}
#endif

// Inverse of the tonemap curve: debug line colors are given as display values and must come out unchanged.
float inverseAces(float y) {
    // y = (x(ax + b)) / (x(cx + d) + e) solved for x.
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    y = clamp(y, 0.0, 0.985);
    const float A = a - c * y, B = b - d * y, C = -e * y;
    return (-B + sqrt(max(B * B - 4.0 * A * C, 0.0))) / (2.0 * A);
}

#if defined(TF_ENTRY_line_fs)
float4 line_fs(LineOut vin) : SV_Target0 {
    const float3 display = pow(saturate(vin.color.rgb), ((float3)(2.2)));
    const float3 hdr = float3(inverseAces(display.r), inverseAces(display.g), inverseAces(display.b)) /
                       max(f.options.y * ACES_PRE_EXPOSURE, 1e-3);
    return float4(hdr, vin.color.a);
}
#endif

// ---------------------------------------------------------------- post: bloom, tonemap, FXAA

struct PostUniforms {
    float4 texel;   // xy: 1/source size, zw: 1/target size
    float4 params;  // x: bloom threshold (tonemap: sharpening), y: bloom strength, z: exposure, w: mode/flags
                    // (tonemap: auto exposure strength, 0 = off)
    float4 extra;   // x: vignette, y: FXAA on (tonemap writes luma to alpha), z: render scale u, w: v
    float4 look;    // tonemap: x: curve (0 ACES, 1 AgX), y: grade LUT amount, z: film grain, w: chromatic aberration
    float4 look2;   // tonemap: x: lens flare, y: sun shafts, z: auto exposure key (luminance), w: its range (EV)
    float4 look3;   // tonemap: x: grain seed (changes every frame)
};

#if defined(TF_ENTRY_bloom_down_fs) || defined(TF_ENTRY_bloom_up_fs) || defined(TF_ENTRY_tonemap_fs) || \
    defined(TF_ENTRY_fxaa_fs) || defined(TF_ENTRY_blur_down_fs) || defined(TF_ENTRY_blur_up_fs) || \
    defined(TF_ENTRY_blur_composite_fs)
ConstantBuffer<PostUniforms> p : UBO(0);
Texture2D<float4> src : TEX(0);
SamplerState ls : SMP(0);
#endif

float3 karisWeight(float3 c) { return c / (1.0 + luminance(c)); }

#if defined(TF_ENTRY_bloom_down_fs)
float4 bloom_down_fs(FullscreenOut vin) : SV_Target0 {
    const float2 uv = vin.uv * p.extra.zw;
    const float2 t = p.texel.xy;
    // 13-tap downsample (Jimenez 2014).
    const float3 a = src.Sample(ls, uv + t * float2(-2, -2)).rgb, b = src.Sample(ls, uv + t * float2(0, -2)).rgb,
                 c = src.Sample(ls, uv + t * float2(2, -2)).rgb, d = src.Sample(ls, uv + t * float2(-1, -1)).rgb,
                 e = src.Sample(ls, uv + t * float2(1, -1)).rgb, f0 = src.Sample(ls, uv + t * float2(-2, 0)).rgb,
                 g = src.Sample(ls, uv).rgb, h = src.Sample(ls, uv + t * float2(2, 0)).rgb,
                 i = src.Sample(ls, uv + t * float2(-1, 1)).rgb, j = src.Sample(ls, uv + t * float2(1, 1)).rgb,
                 k = src.Sample(ls, uv + t * float2(-2, 2)).rgb, l = src.Sample(ls, uv + t * float2(0, 2)).rgb,
                 m = src.Sample(ls, uv + t * float2(2, 2)).rgb;
    float3 col;
    if (p.params.w > 0.5) {
        // First pass: Karis average against fireflies, then a soft threshold.
        col = (karisWeight((d + e + i + j) * 0.25) * 0.5 + karisWeight((a + b + f0 + g) * 0.25) * 0.125 +
               karisWeight((b + c + g + h) * 0.25) * 0.125 + karisWeight((f0 + g + k + l) * 0.25) * 0.125 +
               karisWeight((g + h + l + m) * 0.25) * 0.125);
        col = col / max(1.0 - luminance(col), 1e-3);
        const float br = max(col.r, max(col.g, col.b));
        const float knee = p.params.x * 0.5;
        float soft = clamp(br - p.params.x + knee, 0.0, 2.0 * knee);
        soft = soft * soft / (4.0 * knee + 1e-4);
        col *= max(soft, br - p.params.x) / max(br, 1e-4);
    } else {
        col = (d + e + i + j) * 0.125 + (a + c + k + m) * 0.03125 + (b + f0 + h + l) * 0.0625 + g * 0.125;
    }
    return float4(col, 1.0);
}
#endif

#if defined(TF_ENTRY_bloom_up_fs)
float4 bloom_up_fs(FullscreenOut vin) : SV_Target0 {
    const float2 t = p.texel.xy;
    const float2 uv = vin.uv;
    // 3x3 tent upsample, added onto the larger level by blending.
    float3 c = src.Sample(ls, uv).rgb * 4.0;
    c += (src.Sample(ls, uv + float2(-t.x, 0)).rgb + src.Sample(ls, uv + float2(t.x, 0)).rgb +
          src.Sample(ls, uv + float2(0, -t.y)).rgb + src.Sample(ls, uv + float2(0, t.y)).rgb) * 2.0;
    c += src.Sample(ls, uv + float2(-t.x, -t.y)).rgb + src.Sample(ls, uv + float2(t.x, -t.y)).rgb +
         src.Sample(ls, uv + float2(-t.x, t.y)).rgb + src.Sample(ls, uv + float2(t.x, t.y)).rgb;
    return float4(c / 16.0, 1.0);
}
#endif

float3 aces(float3 x) {
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return saturate((x * (a * x + b)) / (x * (c * x + d) + e));
}

// AgX (Sobotka) with the "punchy" look, polynomial fit of the base contrast curve (Wrensch). Returns display values
// (gamma 2.2 encoded).
float3 agxContrast(float3 x) {
    const float3 x2 = x * x, x4 = x2 * x2;
    return 15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4 - 6.868 * x2 * x + 0.4298 * x2 + 0.1191 * x - 0.00232;
}

float3 agx(float3 c) {
    // HLSL float3x3 rows are the columns of the reference GLSL mat3, so mul(v, M) == M * v there.
    const float3x3 inset = float3x3(0.842479062253094, 0.0423282422610123, 0.0423756549057051,
                                    0.0784335999999992, 0.878468636469772, 0.0784336,
                                    0.0792237451477643, 0.0791661274605434, 0.879142973793104);
    const float3x3 outset = float3x3(1.19687900512017, -0.0528968517574562, -0.0529716355144438,
                                     -0.0980208811401368, 1.15190312990417, -0.0980434501171241,
                                     -0.0990297440797205, -0.0989611768448433, 1.15107367264116);
    const float minEv = -12.47393, maxEv = 4.026069;
    float3 v = mul(max(c, 1e-10), inset);
    v = (clamp(log2(v), minEv, maxEv) - minEv) / (maxEv - minEv);
    v = saturate(agxContrast(v));
    v = pow(v, ((float3)(1.35)));
    const float l = luminance(v);
    v = l + 1.4 * (v - l);
    return saturate(mul(v, outset));
}

// Tone-mapped domain for neighbourhood filters (reversible, keeps bright pixels from dominating).
float3 tmap(float3 c) { return c / (1.0 + max(c.r, max(c.g, c.b))); }
float3 itmap(float3 c) { return c / max(1.0 - max(c.r, max(c.g, c.b)), 1e-4); }

#if defined(TF_ENTRY_tonemap_fs)
Texture2D<float4> bloom : TEX(1);
SamplerState bs : SMP(1);
Texture2D<float4> adaptTex : TEX(2);
SamplerState ads : SMP(2);
Texture2D<float4> shaftTex : TEX(3);
SamplerState shs : SMP(3);
Texture3D<float4> lutTex : TEX(4);
SamplerState lts : SMP(4);

// Ghosts of the bloom image mirrored through the screen centre, tinted like lens coatings. Only the bloom around the
// sun feeds them (look3.yz: sun uv, w: sun weight): lamps and other small bright spots would ghost as enlarged
// copies of themselves far away from where they are.
float3 lensFlare(float2 suv) {
    const float k[4] = {-0.55, 0.42, -1.25, 0.82};
    const float3 tint[4] = {float3(1.0, 0.62, 0.30), float3(0.35, 0.65, 1.0), float3(0.55, 1.0, 0.55),
                            float3(1.0, 0.45, 0.75)};
    const float aspect = p.texel.y / p.texel.x;
    float3 acc = ((float3)(0.0));
    for (int i = 0; i < 4; ++i) {
        const float2 g = 0.5 + (suv - 0.5) * k[i];
        const float w = saturate(1.0 - length(g - 0.5) * 1.7);
        const float sun = 1.0 - smoothstep(0.04, 0.20, length((g - p.look3.yz) * float2(aspect, 1.0)));
        acc += bloom.SampleLevel(bs, g * p.extra.zw, 0).rgb * tint[i] * (w * w * sun);
    }
    return acc * p.look3.w;
}

float4 tonemap_fs(FullscreenOut vin) : SV_Target0 {
    const float2 uv = vin.uv * p.extra.zw; // the 3D image may use only part of the HDR texture (render scale)
    float3 c;
    if (p.look.w > 0.0) {
        // Chromatic aberration: red and blue sampled apart along the radius.
        const float2 o = (vin.uv - 0.5) * p.look.w * 0.010 * p.extra.zw;
        c = float3(src.Sample(ls, uv + o).r, src.Sample(ls, uv).g, src.Sample(ls, uv - o).b);
    } else {
        c = src.Sample(ls, uv).rgb;
    }
    if (p.params.x > 0.0) {
        // Sharpening after TAA (unsharp mask on the 4 neighbours, in the tone-mapped domain).
        const float2 t = p.texel.xy;
        const float3 n = tmap(src.Sample(ls, uv + float2(0, -t.y)).rgb) + tmap(src.Sample(ls, uv + float2(0, t.y)).rgb) +
                         tmap(src.Sample(ls, uv + float2(-t.x, 0)).rgb) + tmap(src.Sample(ls, uv + float2(t.x, 0)).rgb);
        const float3 tc = tmap(c);
        c = itmap(clamp(tc + (tc - n * 0.25) * p.params.x, 0.0, 0.999));
    }
    if (p.params.y > 0.0) c += bloom.Sample(bs, uv).rgb * p.params.y;
    if (p.look2.x > 0.0) c += lensFlare(vin.uv) * p.look2.x;
    if (p.look2.y > 0.0) c += shaftTex.Sample(shs, uv).rgb * p.look2.y;
    float exposure = p.params.z;
    if (p.params.w > 0.0) {
        // Auto exposure on top of the time-of-day exposure: towards the key luminance, within +-range EV.
        const float avgLog = adaptTex.Load(int3(0, 0, 0)).r;
        exposure *= exp2(clamp(log2(p.look2.z) - avgLog, -p.look2.w, p.look2.w) * p.params.w);
    }
    c *= exposure;
    const float2 q = vin.uv - 0.5;
    const float vignette = saturate(1.0 - p.extra.x * dot(q, q) * 1.6);
    float3 display;
    if (p.look.x > 0.5) {
        display = agx(c * vignette);
    } else {
        display = pow(saturate(aces(c * ACES_PRE_EXPOSURE) * vignette), ((float3)(1.0 / 2.2)));
    }
    if (p.look.y > 0.0) {
        const float3 graded = lutTex.SampleLevel(lts, saturate(display) * (31.0 / 32.0) + 0.5 / 32.0, 0).rgb;
        display = lerp(display, graded, p.look.y);
    }
    if (p.look.z > 0.0) {
        // Film grain: per-frame noise, stronger in the shadows and mid-tones.
        const float g = hash12(vin.position.xy + p.look3.x) + hash12(vin.position.yx * 1.37 - p.look3.x) - 1.0;
        display += g * p.look.z * (1.0 - 0.75 * luminance(saturate(display)));
    }
    // Dither against banding in the sky.
    const float n = hash12(vin.position.xy) - 0.5;
    display += n / 255.0;
    return float4(display, p.extra.y > 0.5 ? luminance(display) : 1.0);
}
#endif

// ---------------------------------------------------------------- post: depth-based effects
// Linear depth, GTAO, velocity, TAA, motion blur, depth of field, auto exposure, sun shafts. Only with a
// single-sample scene (the depth buffer is sampled); see Renderer::drawFrame.

struct PostFxUniforms {
    float4x4 viewProj;      // current view-projection without the TAA jitter
    float4x4 prevViewProj;  // previous frame's, without jitter
    float4x4 invView;       // view -> world
    float4 proj;            // x: P00, y: P11, z: P20, w: P21 of the (jittered) projection
    float4 texel;           // xy: 1/source size, zw: 1/target size
    float4 params;          // per pass
    float4 params2;         // per pass
    float4 camPos;          // xyz, w: time (s)
    float4 sun;             // xy: sun position (uv), z: shaft weight (0 = off screen / night)
    float4 sunColor;        // rgb: shaft tint
    float4 depthParams;     // x: near * far, y: near, z: far - near, w: far
};

struct VelocityUniforms {
    float4x4 viewProjJ;     // jittered (as the scene pass, for the depth test)
    float4x4 viewProj;
    float4x4 prevViewProj;
    float4x4 model;
    float4x4 prevModel;
};

#if defined(TF_ENTRY_lindepth_fs) || defined(TF_ENTRY_gtao_fs) || defined(TF_ENTRY_ao_blur_fs) || \
    defined(TF_ENTRY_velocity_fs) || defined(TF_ENTRY_taa_fs) || defined(TF_ENTRY_motion_blur_fs) || \
    defined(TF_ENTRY_dof_blur_fs) || defined(TF_ENTRY_dof_fs) || defined(TF_ENTRY_lum_fs) || \
    defined(TF_ENTRY_adapt_fs) || defined(TF_ENTRY_shafts_fs)
ConstantBuffer<PostFxUniforms> px : UBO(0);
#endif

#if defined(TF_ENTRY_gtao_fs) || defined(TF_ENTRY_ao_blur_fs) || defined(TF_ENTRY_velocity_fs) || \
    defined(TF_ENTRY_taa_fs) || defined(TF_ENTRY_motion_blur_fs) || defined(TF_ENTRY_dof_blur_fs) || \
    defined(TF_ENTRY_dof_fs) || defined(TF_ENTRY_shafts_fs)
// View-space position in screen axes (x right, y down, z forward = view distance) from uv and linear depth.
float3 viewPosSS(float2 uv, float z) {
    const float2 ndc = float2(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    return float3(z * (ndc.x + px.proj.z) / px.proj.x, -z * (ndc.y + px.proj.w) / px.proj.y, z);
}

float3 worldFromUV(float2 uv, float z) {
    const float3 v = viewPosSS(uv, z);
    return mul(px.invView, float4(v.x, -v.y, -v.z, 1.0)).xyz;
}
#endif

float2 clipToUV(float4 c) {
    const float2 n = c.xy / c.w;
    return float2(n.x * 0.5 + 0.5, 0.5 - n.y * 0.5);
}

// Interleaved gradient noise (Jimenez).
float ign(float2 pix) { return frac(52.9829189 * frac(dot(pix, float2(0.06711056, 0.00583715)))); }

#if defined(TF_ENTRY_lindepth_fs)
Texture2D<float4> depthTex : TEX(0);
SamplerState dps : SMP(0);

// Reversed device depth -> view distance (m); the sky (cleared depth 0) comes out as the far plane.
float4 lindepth_fs(FullscreenOut vin) : SV_Target0 {
    const float d = depthTex.Load(int3(int2(vin.position.xy), 0)).r;
    return float4(px.depthParams.x / (px.depthParams.y + d * px.depthParams.z), 0.0, 0.0, 1.0);
}
#endif

#if defined(TF_ENTRY_gtao_fs)
Texture2D<float4> depthLin : TEX(0);
SamplerState pts : SMP(0);
Texture2D<float4> sceneTex : TEX(1); // opaque scene, light split in alpha (grass blades < 0)
SamplerState sps : SMP(1);

// Snapped to a full-resolution texel centre: the depth and the position it is reconstructed at must belong to the same
// texel (half-resolution centres sit on texel corners, where point sampling rounds either way and fakes steps on
// surfaces seen at grazing angles).
float2 aoTexel(float2 uv) { return (floor(uv / px.texel.xy) + 0.5) * px.texel.xy; }
float3 aoPos(float2 uv) {
    const float2 c = aoTexel(uv);
    return viewPosSS(c, depthLin.SampleLevel(pts, c, 0).r);
}

float2 uvFromViewSS(float3 Q) {
    const float2 ndc = float2(Q.x * px.proj.x / Q.z - px.proj.z, -Q.y * px.proj.y / Q.z - px.proj.w);
    return float2(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
}

// Screen-space contact shadow: a short march from the surface toward the shadow light against the linear depth
// (sun: x y z in view axes, w: ray length (m); sunColor x: steps, y: fade distance (m)). Catches what the shadow
// cascades are too coarse for (wheels and rocks on the ground, small bumps) and the ground beyond the last cascade.
float contactShadow(float3 P, float2 uv, float2 pix) {
    const float steps = px.sunColor.x;
    if (steps < 0.5 || P.z > px.sunColor.y) return 1.0;
    // Only where sun light reaches the surface (light split >= 1: a sun share; grass blades are < 0).
    if (sceneTex.SampleLevel(sps, uv, 0).a < 1.0) return 1.0;
    const float len = px.sun.w * (1.0 + P.z * 0.02);
    const float3 dir = px.sun.xyz * len;
    const float bias = 0.015 + P.z * 0.0025;
    const float thick = 0.25 + P.z * 0.012;
    const float jitter = ign(pix + px.params2.x * float2(3.7, 1.3));
    float occl = 0.0;
    for (int i = 0; i < 16; ++i) {
        if (float(i) >= steps) break;
        const float k = (float(i) + jitter) / steps;
        const float3 Q = P + dir * (0.04 + 0.96 * k);
        if (Q.z < 0.1) break;
        const float2 quv = uvFromViewSS(Q);
        if (any(quv < 0.0) || any(quv > 1.0)) break;
        const float d = Q.z - depthLin.SampleLevel(pts, quv, 0).r;
        if (d > bias && d < thick && sceneTex.SampleLevel(sps, quv, 0).a > -0.5) {
            occl = 1.0 - k * k; // softer toward the end of the ray
            break;
        }
    }
    return 1.0 - occl * (1.0 - smoothstep(px.sunColor.y * 0.6, px.sunColor.y, P.z));
}

// Ground-truth ambient occlusion (Jimenez 2016, after XeGTAO) at half resolution: params x: radius (m), y: slices
// (0 = contact shadows only), z: steps per side, w: half-res pixels per metre at 1 m; params2 x: frame index (noise
// rotation), y: fade start (m). Output r: visibility, g: view distance (for the bilateral upsample), b: contact
// shadow (1 = lit).
float4 gtao_fs(FullscreenOut vin) : SV_Target0 {
    const float2 uv = aoTexel(vin.uv);
    const float z = depthLin.SampleLevel(pts, uv, 0).r;
    if (z > px.depthParams.w * 0.5) return float4(1.0, z, 1.0, 1.0);
    const float3 P = viewPosSS(uv, z);
    const float cs = contactShadow(P, uv, vin.position.xy);
    if (px.params.y < 0.5) return float4(1.0, z, cs, 1.0);
    const float2 t = px.texel.xy;
    const float3 pl = aoPos(uv - float2(t.x, 0)), pr = aoPos(uv + float2(t.x, 0));
    const float3 pu = aoPos(uv - float2(0, t.y)), pd = aoPos(uv + float2(0, t.y));
    const float3 dx = abs(pr.z - P.z) < abs(P.z - pl.z) ? pr - P : P - pl;
    const float3 dy = abs(pd.z - P.z) < abs(P.z - pu.z) ? pd - P : P - pu;
    const float3 V = normalize(-P);
    float3 N = normalize(cross(dy, dx));
    if (dot(N, V) < 0.0) N = -N;
    const float radius = px.params.x, slices = px.params.y, steps = px.params.z;
    const float screenRadius = min(radius * px.params.w / z, 80.0);
    if (screenRadius < 1.5) return float4(1.0, z, cs, 1.0);
    // No samples closer than 0.3 m: the terrain's height-grid steps on steep walls are not occluders worth showing.
    const float minPx = max(1.0, 0.3 * px.params.w / z);
    const float falloffRange = 0.6 * radius;
    const float falloffMul = -1.0 / falloffRange, falloffAdd = (radius - falloffRange) / falloffRange + 1.0;
    const float2 pix = vin.position.xy + px.params2.x * float2(5.588238, 2.71);
    const float nSlice = ign(pix), nStep = ign(pix.yx + 17.0);
    float vis = 0.0;
    for (int s = 0; s < 4; ++s) {
        if (float(s) >= slices) break;
        const float phi = (float(s) + nSlice) * PI / slices;
        const float2 omega = float2(cos(phi), sin(phi));
        const float3 dir = float3(omega, 0.0);
        const float3 ortho = dir - dot(dir, V) * V;
        const float3 axis = normalize(cross(ortho, V));
        const float3 projN = N - axis * dot(N, axis);
        const float projLen = length(projN);
        const float cosN = saturate(dot(projN, V) / max(projLen, 1e-4));
        const float n = (dot(ortho, projN) < 0.0 ? -1.0 : 1.0) * acos(cosN);
        const float low0 = cos(n + PI * 0.5), low1 = cos(n - PI * 0.5);
        float hc0 = low0, hc1 = low1;
        for (int j = 0; j < 8; ++j) {
            if (float(j) >= steps) break;
            float st = (float(j) + frac(nStep + float(j) * 0.618034)) / steps;
            st = st * st;
            const float2 off = omega * (st * screenRadius + minPx) * px.texel.zw;
            const float3 d0 = aoPos(uv + off) - P, d1 = aoPos(uv - off) - P;
            const float l0 = max(length(d0), 1e-4), l1 = max(length(d1), 1e-4);
            hc0 = max(hc0, lerp(low0, dot(d0 / l0, V), saturate(l0 * falloffMul + falloffAdd)));
            hc1 = max(hc1, lerp(low1, dot(d1 / l1, V), saturate(l1 * falloffMul + falloffAdd)));
        }
        float h0 = -acos(clamp(hc1, -1.0, 1.0)), h1 = acos(clamp(hc0, -1.0, 1.0));
        h0 = n + clamp(h0 - n, -PI * 0.5, PI * 0.5);
        h1 = n + clamp(h1 - n, -PI * 0.5, PI * 0.5);
        const float sinN = sin(n);
        vis += projLen * ((cosN + 2.0 * h0 * sinN - cos(2.0 * h0 - n)) + (cosN + 2.0 * h1 * sinN - cos(2.0 * h1 - n))) * 0.25;
    }
    vis = saturate(vis / slices);
    vis = lerp(vis, 1.0, smoothstep(px.params2.y, px.params2.y * 2.0, z));
    return float4(vis, z, cs, 1.0);
}
#endif

#if defined(TF_ENTRY_ao_blur_fs)
Texture2D<float4> aoTex : TEX(0);
SamplerState aps : SMP(0);
Texture2D<float4> depthLin : TEX(1);
SamplerState dls : SMP(1);

Texture2D<float4> sceneTex : TEX(2);
SamplerState sps : SMP(2);

// Jimenez et al. 2016: visibility with the light of further bounces between surfaces of this albedo.
float3 multiBounce(float v, float3 albedo) {
    const float3 a = 2.0404 * albedo - 0.3324;
    const float3 b = -4.7951 * albedo + 0.6417;
    const float3 c = 2.7552 * albedo + 0.6903;
    return max((float3)v, ((v * a + b) * v + c) * v);
}

// Full-resolution bilateral 4 x 4 gather of the half-resolution AO and contact shadows (covers their noise pattern),
// applied to the opaque scene by its light split (lightSplit in alpha): the AO darkens the indirect share, the
// contact shadow the sun share. params x: AO strength (exponent, 0 = no AO), y: depth tolerance.
float4 ao_blur_fs(FullscreenOut vin) : SV_Target0 {
    const float4 scene = sceneTex.SampleLevel(sps, vin.uv, 0);
    const float z = depthLin.SampleLevel(dls, vin.uv, 0).r;
    const float2 t = px.texel.zw; // 1 / AO size
    float2 sum = (float2)0.0;
    float wsum = 0.0;
    for (int y = -2; y < 2; ++y) {
        for (int x = -2; x < 2; ++x) {
            const float3 a = aoTex.SampleLevel(aps, vin.uv + (float2(x, y) + 0.5) * t, 0).rgb;
            const float w = exp(-abs(a.y - z) / max(z * px.params.y, 1e-3));
            sum += a.xz * w;
            wsum += w;
        }
    }
    const float2 v = wsum > 1e-4 ? sum / wsum : (float2)1.0;
    const float split = scene.a < 0.0 ? -1.0 - scene.a : scene.a;
    const float direct = floor(split) / 15.0, indirect = frac(split);
    // The albedo is not stored: the hue of the pixel stands in for it (bounce light between green grass is green).
    const float3 albedo = saturate(scene.rgb / max(max(scene.r, max(scene.g, scene.b)), 1e-5)) * 0.45;
    const float3 ao = px.params.x > 0.0 ? multiBounce(pow(saturate(v.x), px.params.x), albedo) : (float3)1.0;
    const float3 m = 1.0 - indirect * (1.0 - ao) - direct * (1.0 - saturate(v.y));
    return float4(scene.rgb * max(m, 0.0), 1.0);
}
#endif

#if defined(TF_ENTRY_velocity_fs)
Texture2D<float4> depthLin : TEX(0);
SamplerState dls : SMP(0);

// Camera motion: screen offset since the previous frame of the world point seen at this pixel (uv units).
float4 velocity_fs(FullscreenOut vin) : SV_Target0 {
    const float z = depthLin.SampleLevel(dls, vin.uv, 0).r;
    const float3 w = worldFromUV(vin.uv, z);
    const float2 v = clipToUV(mul(px.viewProj, float4(w, 1.0))) - clipToUV(mul(px.prevViewProj, float4(w, 1.0)));
    return float4(v, 0.0, 1.0);
}
#endif

struct VelocityOut {
    float4 position : SV_Position;
    [[vk::location(0)]] float4 cur : TEXCOORD0;
    [[vk::location(1)]] float4 prev : TEXCOORD1;
};

#if defined(TF_ENTRY_velocity_obj_vs)
ConstantBuffer<VelocityUniforms> vu : UBO(0);

// Moving meshes (vehicle body, spinning wheels) over the camera velocity, with their previous model matrix.
VelocityOut velocity_obj_vs(LitIn vin) {
    VelocityOut vout;
    const float4 world = mul(vu.model, float4(vin.position, 1.0));
    vout.position = mul(vu.viewProjJ, world);
    vout.cur = mul(vu.viewProj, world);
    vout.prev = mul(vu.prevViewProj, mul(vu.prevModel, float4(vin.position, 1.0)));
    return vout;
}
#endif

#if defined(TF_ENTRY_velocity_obj_fs)
float4 velocity_obj_fs(VelocityOut vin) : SV_Target0 {
    return float4(clipToUV(vin.cur) - clipToUV(vin.prev), 0.0, 1.0);
}
#endif

#if defined(TF_ENTRY_taa_fs)
Texture2D<float4> cur : TEX(0);
SamplerState cls : SMP(0);
Texture2D<float4> hist : TEX(1);
SamplerState hls : SMP(1);
Texture2D<float4> velTex : TEX(2);
SamplerState vps : SMP(2);
Texture2D<float4> depthLin : TEX(3);
SamplerState dls : SMP(3);

float3 rgbToYCoCg(float3 c) {
    return float3(dot(c, float3(0.25, 0.5, 0.25)), dot(c, float3(0.5, 0.0, -0.5)), dot(c, float3(-0.25, 0.5, -0.25)));
}
float3 yCoCgToRgb(float3 c) { return float3(c.x + c.y - c.z, c.x + c.z, c.x - c.y - c.z); }

// Catmull-Rom history fetch with 5 bilinear taps.
float3 historyCatmullRom(float2 uv, float2 size) {
    const float2 sp = uv * size;
    const float2 tc1 = floor(sp - 0.5) + 0.5;
    const float2 fr = sp - tc1;
    const float2 w0 = fr * (-0.5 + fr * (1.0 - 0.5 * fr));
    const float2 w1 = 1.0 + fr * fr * (-2.5 + 1.5 * fr);
    const float2 w2 = fr * (0.5 + fr * (2.0 - 1.5 * fr));
    const float2 w3 = fr * fr * (-0.5 + 0.5 * fr);
    const float2 w12 = w1 + w2;
    const float2 tc0 = (tc1 - 1.0) / size, tc3 = (tc1 + 2.0) / size, tc12 = (tc1 + w2 / w12) / size;
    const float a = w12.x * w0.y, b = w0.x * w12.y, c = w12.x * w12.y, d = w3.x * w12.y, e = w12.x * w3.y;
    const float3 r = hist.SampleLevel(hls, float2(tc12.x, tc0.y), 0).rgb * a +
                     hist.SampleLevel(hls, float2(tc0.x, tc12.y), 0).rgb * b + hist.SampleLevel(hls, tc12, 0).rgb * c +
                     hist.SampleLevel(hls, float2(tc3.x, tc12.y), 0).rgb * d +
                     hist.SampleLevel(hls, float2(tc12.x, tc3.y), 0).rgb * e;
    return max(r / (a + b + c + d + e), 0.0);
}

// Temporal AA: jittered current frame blended into the reprojected history, clipped to the variance box of the
// 3 x 3 neighbourhood (YCoCg, tone-mapped domain). params x: 1 = reset history, y: current-frame weight.
float4 taa_fs(FullscreenOut vin) : SV_Target0 {
    const float2 uv = vin.uv;
    const float2 t = px.texel.xy;
    // Velocity of the nearest depth in a cross (keeps edges of moving objects from trailing).
    float2 best = uv;
    float zMin = depthLin.SampleLevel(dls, uv, 0).r;
    const float2 offs[4] = {float2(-1, -1), float2(1, -1), float2(-1, 1), float2(1, 1)};
    for (int i = 0; i < 4; ++i) {
        const float2 o = uv + offs[i] * t;
        const float zo = depthLin.SampleLevel(dls, o, 0).r;
        if (zo < zMin) { zMin = zo; best = o; }
    }
    const float2 vel = velTex.SampleLevel(vps, best, 0).xy;
    float3 m1 = ((float3)(0.0)), m2 = ((float3)(0.0));
    float3 center = ((float3)(0.0));
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            const float3 c = rgbToYCoCg(tmap(cur.SampleLevel(cls, uv + float2(x, y) * t, 0).rgb));
            if (x == 0 && y == 0) center = c;
            m1 += c;
            m2 += c * c;
        }
    }
    const float2 huv = uv - vel;
    if (px.params.x > 0.5 || any(huv < 0.0) || any(huv > 1.0))
        return float4(itmap(yCoCgToRgb(center)), 1.0);
    const float3 mu = m1 / 9.0;
    const float3 sigma = sqrt(abs(m2 / 9.0 - mu * mu));
    const float3 bmin = mu - sigma * 1.1, bmax = mu + sigma * 1.1;
    float3 h = rgbToYCoCg(tmap(historyCatmullRom(huv, 1.0 / t)));
    // Clip toward the box centre (not a per-channel clamp: keeps the hue).
    const float3 hc = 0.5 * (bmax + bmin), he = 0.5 * (bmax - bmin) + 1e-5;
    const float3 dv = (h - hc) / he;
    const float ma = max(abs(dv.x), max(abs(dv.y), abs(dv.z)));
    if (ma > 1.0) h = hc + (h - hc) / ma;
    const float speed = length(vel / t);
    const float a = lerp(px.params.y, 0.3, saturate(speed / 40.0));
    return float4(max(itmap(yCoCgToRgb(lerp(h, center, a))), 0.0), 1.0);
}
#endif

#if defined(TF_ENTRY_motion_blur_fs)
Texture2D<float4> cur : TEX(0);
SamplerState cls : SMP(0);
Texture2D<float4> velTex : TEX(1);
SamplerState vps : SMP(1);
Texture2D<float4> depthLin : TEX(2);
SamplerState dls : SMP(2);

// Camera and per-object motion blur along the pixel's velocity. params x: shutter (share of the frame), y: max
// length (pixels), z: samples. Closer samples count only if their own motion reaches this pixel, so a sharp vehicle
// in front of a blurred background stays sharp and does not smear into it.
float4 motion_blur_fs(FullscreenOut vin) : SV_Target0 {
    const float2 uv = vin.uv;
    const float3 c0 = cur.SampleLevel(cls, uv, 0).rgb;
    const float2 tpx = px.texel.xy;
    float2 v = velTex.SampleLevel(vps, uv, 0).xy * px.params.x;
    float lenPx = length(v / tpx);
    if (lenPx < 0.75) return float4(c0, 1.0);
    if (lenPx > px.params.y) { v *= px.params.y / lenPx; lenPx = px.params.y; }
    const float z0 = depthLin.SampleLevel(dls, uv, 0).r;
    const float n = px.params.z;
    const float j = ign(vin.position.xy) - 0.5;
    float3 sum = c0;
    float wsum = 1.0;
    for (int i = 0; i < 16; ++i) {
        if (float(i) >= n) break;
        const float s = (float(i) + 0.5 + j) / n - 0.5;
        if (abs(s) < 1e-3) continue;
        const float2 suv = uv + v * s;
        const float zs = depthLin.SampleLevel(dls, suv, 0).r;
        const float distPx = abs(s) * lenPx;
        float w = 1.0;
        if (zs < z0 * 0.98) {
            const float vs = length(velTex.SampleLevel(vps, suv, 0).xy * px.params.x / tpx);
            w = saturate(vs - distPx + 1.0);
        }
        sum += cur.SampleLevel(cls, suv, 0).rgb * w;
        wsum += w;
    }
    return float4(sum / wsum, 1.0);
}
#endif

#if defined(TF_ENTRY_dof_blur_fs) || defined(TF_ENTRY_dof_fs)
// Circle of confusion (target pixels) at view distance z: params x: focus distance (m), y: aperture (blur at
// infinity, share of the max radius), z: max radius (half-res pixels), w: near blur scale.
float dofCoc(float z) {
    const float c = (z - px.params.x) / max(z, 1e-3);
    return clamp(c * px.params.y * (c < 0.0 ? px.params.w : 1.0), -1.0, 1.0) * px.params.z;
}
#endif

#if defined(TF_ENTRY_dof_blur_fs)
Texture2D<float4> cur : TEX(0);
SamplerState cls : SMP(0);
Texture2D<float4> depthLin : TEX(1);
SamplerState dls : SMP(1);

// Half-resolution gather over a golden-angle disc; a tap counts where its own blur reaches this pixel, background
// taps no farther than this pixel's own blur (no sharp foreground smeared outward). a: |CoC| / max radius.
float4 dof_blur_fs(FullscreenOut vin) : SV_Target0 {
    const float2 uv = vin.uv;
    const float c0 = abs(dofCoc(depthLin.SampleLevel(dls, uv, 0).r));
    const float z0 = depthLin.SampleLevel(dls, uv, 0).r;
    const float R = px.params.z;
    float3 sum = cur.SampleLevel(cls, uv, 0).rgb;
    float wsum = 1.0;
    const float rot = ign(vin.position.xy) * 6.2831853;
    for (int i = 0; i < 16; ++i) {
        const float r = sqrt((float(i) + 0.5) / 16.0) * R;
        const float a = float(i) * 2.39996323 + rot;
        const float2 suv = uv + float2(cos(a), sin(a)) * r * px.texel.zw;
        const float zs = depthLin.SampleLevel(dls, suv, 0).r;
        float cs = abs(dofCoc(zs));
        if (zs > z0) cs = min(cs, c0);
        const float w = saturate(cs - r + 1.0);
        sum += cur.SampleLevel(cls, suv, 0).rgb * w;
        wsum += w;
    }
    return float4(sum / wsum, c0 / max(R, 1e-3));
}
#endif

#if defined(TF_ENTRY_dof_fs)
Texture2D<float4> cur : TEX(0);
SamplerState cls : SMP(0);
Texture2D<float4> blurred : TEX(1);
SamplerState bls : SMP(1);
Texture2D<float4> depthLin : TEX(2);
SamplerState dls : SMP(2);

float4 dof_fs(FullscreenOut vin) : SV_Target0 {
    const float3 sharp = cur.SampleLevel(cls, vin.uv, 0).rgb;
    const float4 b = blurred.SampleLevel(bls, vin.uv, 0);
    const float coc = abs(dofCoc(depthLin.SampleLevel(dls, vin.uv, 0).r));
    const float a = smoothstep(0.4, 1.6, max(coc, b.a * px.params.z * 0.7));
    return float4(lerp(sharp, b.rgb, a), 1.0);
}
#endif

#if defined(TF_ENTRY_lum_fs)
Texture2D<float4> cur : TEX(0);
SamplerState cls : SMP(0);

// 64 x 64 log2 luminance after the base exposure (params x), 4 bilinear taps per texel.
float4 lum_fs(FullscreenOut vin) : SV_Target0 {
    const float2 h = px.texel.zw * 0.25;
    float s = 0.0;
    for (int i = 0; i < 4; ++i) {
        const float2 o = float2((i & 1) != 0 ? h.x : -h.x, (i & 2) != 0 ? h.y : -h.y);
        s += log2(max(luminance(cur.SampleLevel(cls, vin.uv + o, 0).rgb) * px.params.x, 1e-4));
    }
    return float4(s * 0.25, 0.0, 0.0, 1.0);
}
#endif

#if defined(TF_ENTRY_adapt_fs)
Texture2D<float4> lumTex : TEX(0);
SamplerState lls : SMP(0);
Texture2D<float4> prevTex : TEX(1);
SamplerState pls : SMP(1);

// 1 x 1: centre-weighted mean log luminance, adapted from the previous value. params x: blend (1 = snap).
float4 adapt_fs(FullscreenOut vin) : SV_Target0 {
    float s = 0.0, ws = 0.0;
    for (int y = 0; y < 16; ++y) {
        for (int x = 0; x < 16; ++x) {
            const float2 uv = (float2(x, y) + 0.5) / 16.0;
            const float2 q = uv - float2(0.5, 0.55);
            const float w = 1.0 - saturate(dot(q, q) * 2.2);
            s += lumTex.SampleLevel(lls, uv, 0).r * w;
            ws += w;
        }
    }
    const float target = s / ws;
    const float prev = prevTex.Load(int3(0, 0, 0)).r;
    return float4(lerp(prev, target, px.params.x), 0.0, 0.0, 1.0);
}
#endif

#if defined(TF_ENTRY_shafts_fs)
Texture2D<float4> cur : TEX(0);
SamplerState cls : SMP(0);
Texture2D<float4> depthLin : TEX(1);
SamplerState dls : SMP(1);

// Sun shafts (half resolution): radial march toward the sun over the bright sky only, so trees, ridges and the
// vehicle cut dark wedges into the glow. params x: threshold, y: decay per step, z: steps.
float4 shafts_fs(FullscreenOut vin) : SV_Target0 {
    const float2 uv = vin.uv;
    const float2 d = px.sun.xy - uv;
    const float n = px.params.z;
    const float2 stp = d / n;
    const float j = ign(vin.position.xy + px.params2.x * 3.17);
    float2 suv = uv + stp * j;
    float acc = 0.0, decay = 1.0;
    for (int i = 0; i < 32; ++i) {
        if (float(i) >= n) break;
        if (all(suv > 0.0) && all(suv < 1.0)) {
            const float sky = depthLin.SampleLevel(dls, suv, 0).r > px.depthParams.w * 0.5 ? 1.0 : 0.0;
            acc += sky * max(luminance(cur.SampleLevel(cls, suv, 0).rgb) - px.params.x, 0.0) * decay;
        }
        decay *= px.params.y;
        suv += stp;
    }
    const float fall = saturate(1.0 - length(d * float2(px.texel.y / px.texel.x, 1.0)) * 1.4);
    return float4(px.sunColor.rgb * (acc / n) * px.sun.z * fall, 1.0);
}
#endif

// FXAA (simplified 3.11 "quality" variant with 5 edge steps).
#if defined(TF_ENTRY_fxaa_fs)
float4 fxaa_fs(FullscreenOut vin) : SV_Target0 {
    const float2 uv = vin.uv;
    const float2 t = p.texel.xy;
    const float4 center = src.Sample(ls, uv);
    const float lM = center.a;
    const float lN = src.Sample(ls, uv + float2(0, -t.y)).a, lS = src.Sample(ls, uv + float2(0, t.y)).a;
    const float lE = src.Sample(ls, uv + float2(t.x, 0)).a, lW = src.Sample(ls, uv + float2(-t.x, 0)).a;
    const float lMax = max(lM, max(max(lN, lS), max(lE, lW))), lMin = min(lM, min(min(lN, lS), min(lE, lW)));
    const float range = lMax - lMin;
    if (range < max(0.0312, lMax * 0.125)) return float4(center.rgb, 1.0);
    const float lNW = src.Sample(ls, uv + float2(-t.x, -t.y)).a, lNE = src.Sample(ls, uv + float2(t.x, -t.y)).a;
    const float lSW = src.Sample(ls, uv + float2(-t.x, t.y)).a, lSE = src.Sample(ls, uv + float2(t.x, t.y)).a;
    const float edgeH = abs(lNW + lNE - 2.0 * lN) + 2.0 * abs(lW + lE - 2.0 * lM) + abs(lSW + lSE - 2.0 * lS);
    const float edgeV = abs(lNW + lSW - 2.0 * lW) + 2.0 * abs(lN + lS - 2.0 * lM) + abs(lNE + lSE - 2.0 * lE);
    const bool horizontal = edgeH >= edgeV;
    const float l1 = horizontal ? lN : lW, l2 = horizontal ? lS : lE;
    const float g1 = abs(l1 - lM), g2 = abs(l2 - lM);
    const bool steep1 = g1 >= g2;
    float stepLen = horizontal ? t.y : t.x;
    float lLocal;
    if (steep1) {
        stepLen = -stepLen;
        lLocal = 0.5 * (l1 + lM);
    } else {
        lLocal = 0.5 * (l2 + lM);
    }
    const float gScaled = 0.25 * max(g1, g2);
    float2 cuv = uv;
    if (horizontal) cuv.y += stepLen * 0.5; else cuv.x += stepLen * 0.5;
    const float2 off = horizontal ? float2(t.x, 0) : float2(0, t.y);
    float2 uv1 = cuv - off, uv2 = cuv + off;
    float e1 = src.Sample(ls, uv1).a - lLocal, e2 = src.Sample(ls, uv2).a - lLocal;
    bool d1 = abs(e1) >= gScaled, d2 = abs(e2) >= gScaled;
    const float steps[5] = {1.5, 2.0, 2.0, 4.0, 8.0};
    for (int i = 0; i < 5 && !(d1 && d2); ++i) {
        if (!d1) { uv1 -= off * steps[i]; e1 = src.Sample(ls, uv1).a - lLocal; d1 = abs(e1) >= gScaled; }
        if (!d2) { uv2 += off * steps[i]; e2 = src.Sample(ls, uv2).a - lLocal; d2 = abs(e2) >= gScaled; }
    }
    const float dist1 = horizontal ? uv.x - uv1.x : uv.y - uv1.y;
    const float dist2 = horizontal ? uv2.x - uv.x : uv2.y - uv.y;
    const bool closer1 = dist1 < dist2;
    const float dmin = min(dist1, dist2);
    const float edgeLen = dist1 + dist2;
    const bool correct = ((closer1 ? e1 : e2) < 0.0) != (lM < lLocal);
    float pixelOffset = correct ? -dmin / edgeLen + 0.5 : 0.0;
    // Sub-pixel aliasing.
    const float lAvg = (2.0 * (lN + lS + lE + lW) + lNW + lNE + lSW + lSE) / 12.0;
    const float sub = saturate(abs(lAvg - lM) / range);
    const float sub2 = (-2.0 * sub + 3.0) * sub * sub;
    pixelOffset = max(pixelOffset, sub2 * sub2 * 0.75);
    float2 fuv = uv;
    if (horizontal) fuv.y += pixelOffset * stepLen; else fuv.x += pixelOffset * stepLen;
    return float4(src.Sample(ls, fuv).rgb, 1.0);
}
#endif

// ---------------------------------------------------------------- menu blur (dual Kawase, display-referred LDR)

#if defined(TF_ENTRY_blur_down_fs)
// Half-resolution step: centre plus four diagonal taps at half-texel offsets (p.texel.xy: 1/source size,
// p.params.x: offset scale).
float4 blur_down_fs(FullscreenOut vin) : SV_Target0 {
    const float2 uv = vin.uv;
    const float2 h = p.texel.xy * 0.5 * p.params.x;
    float3 c = src.Sample(ls, uv).rgb * 4.0;
    c += src.Sample(ls, uv - h).rgb + src.Sample(ls, uv + h).rgb;
    c += src.Sample(ls, uv + float2(h.x, -h.y)).rgb + src.Sample(ls, uv - float2(h.x, -h.y)).rgb;
    return float4(c * 0.125, 1.0);
}
#endif

#if defined(TF_ENTRY_blur_up_fs)
// Double-resolution step: eight taps on a diamond around the pixel.
float4 blur_up_fs(FullscreenOut vin) : SV_Target0 {
    const float2 uv = vin.uv;
    const float2 h = p.texel.xy * 0.5 * p.params.x;
    float3 c = src.Sample(ls, uv + float2(-h.x * 2.0, 0.0)).rgb + src.Sample(ls, uv + float2(h.x * 2.0, 0.0)).rgb;
    c += src.Sample(ls, uv + float2(0.0, -h.y * 2.0)).rgb + src.Sample(ls, uv + float2(0.0, h.y * 2.0)).rgb;
    c += (src.Sample(ls, uv + float2(-h.x, h.y)).rgb + src.Sample(ls, uv + float2(h.x, h.y)).rgb +
          src.Sample(ls, uv + float2(h.x, -h.y)).rgb + src.Sample(ls, uv + float2(-h.x, -h.y)).rgb) * 2.0;
    return float4(c / 12.0, 1.0);
}
#endif

#if defined(TF_ENTRY_blur_composite_fs)
Texture2D<float4> blurred : TEX(1);
SamplerState bls : SMP(1);

// Sharp frame -> blurred, dimmed and slightly desaturated by p.params.x (0..1). p.params.y: dim factor at full blur,
// p.params.z: desaturation at full blur.
float4 blur_composite_fs(FullscreenOut vin) : SV_Target0 {
    const float a = saturate(p.params.x);
    const float3 sharp = src.Sample(ls, vin.uv).rgb;
    float3 c = blurred.Sample(bls, vin.uv).rgb;
    c = lerp(c, (float3)luminance(c), p.params.z);
    c *= p.params.y;
    // Dither: the blurred gradients are smooth enough to band in 8 bits.
    c += (hash12(vin.position.xy) - 0.5) / 255.0;
    return float4(lerp(sharp, c, a), 1.0);
}
#endif
