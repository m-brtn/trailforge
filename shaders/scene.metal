// TrailForge shaders (MSL, Metal). One file, compiled to one metallib at build time.
// Resource binding follows the SDL GPU rules for MSL:
//   uniform slot n -> [[buffer(n)]] (per stage); read-only storage buffers follow the uniform buffers;
//   sampled textures -> [[texture(n)]] with matching [[sampler(n)]];
//   vertex buffers via [[stage_in]] (SDL places them starting at buffer(14)).
// Struct layouts must match src/render/Renderer.h (FrameUniforms, DrawUniforms, GrassUniforms, PostUniforms).
// Rendering is linear HDR; the tonemap pass converts to the (UNORM, non-sRGB) swapchain with a gamma curve.

#include <metal_stdlib>
using namespace metal;

constant float PI = 3.14159265;
// The fitted ACES curve (Narkowicz) expects scene values scaled by ~0.6; the user exposure multiplies this.
constant float ACES_PRE_EXPOSURE = 0.6;

// Material ids (src/render/Mesh.h, MatId). 0 = generic, 4 = low-grip slab (handled as fallbacks).
constant int MAT_DIRT = 1;
constant int MAT_CONCRETE = 2;
constant int MAT_ROCK = 3;
constant int MAT_MUD = 5;
constant int MAT_GRASS = 6;
constant int MAT_GLASS = 7;
constant int MAT_HEADLIGHT = 8;
constant int MAT_BRAKE = 9;
constant int MAT_REVERSE = 10;
constant int MAT_RUBBER = 11;
constant int MAT_AMBER = 12;
constant int MAT_FOLIAGE = 13;
constant int MAT_PAINT = 14;

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
    float4 skyAmbient;      // rgb: sky diffuse irradiance
    float4 headlightL;      // xyz: lamp position, w: intensity (0 = off)
    float4 headlightR;      // xyz: lamp position
    float4 headlightDir;    // xyz: beam axis, w: cos of the outer cone angle
};

struct DrawUniforms {
    float4x4 model;
    float4x4 normalMatrix;  // inverse transpose of model (upper 3x3)
    float4 tint;            // rgb multiplier
    float4 params;          // x: mud scale, y: mud mapping (0 body, 1 wheel), zw: reserved
};

// ---------------------------------------------------------------- common helpers

float3 sampleNoise(texture2d<float> tex, sampler s, float2 uv) { return tex.sample(s, uv).rgb; }

float luminance(float3 c) { return dot(c, float3(0.2126, 0.7152, 0.0722)); }

float hash12(float2 p) {
    float3 p3 = fract(float3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

// Procedural sky radiance for a view direction (no sun disk), used by the sky pass and as environment light.
float3 skyRadiance(constant FrameUniforms& f, float3 dir) {
    const float3 sun = f.skySun.xyz;
    const float y = dir.y;
    const float up = saturate(y);
    // Toward the sun the warm horizon band reaches higher (sunset banding), away from it the sky stays cooler.
    const float mu = dot(dir, sun);
    const float toward = saturate(mu * 0.5 + 0.5);
    const float band = mix(0.55, 0.28, toward * toward);
    float3 col = mix(f.skyHorizon.rgb, f.skyZenith.rgb, pow(up, band));
    // Below the horizon: hazy ground color.
    const float3 below = mix(f.fogColor.rgb * 0.85, f.groundAmbient.rgb * 1.4, saturate(-y * 4.0));
    col = y < 0.0 ? below : col;
    // Mie-like glow around the sun and a warmer horizon toward it.
    const float g = 0.76;
    const float hg = (1.0 - g * g) / pow(1.0 + g * g - 2.0 * g * mu, 1.5) / (4.0 * PI);
    col += f.skySunGlow.rgb * hg * 0.10;
    col += f.skySunGlow.rgb * 0.05 * pow(saturate(mu), 6.0) * (1.0 - up);
    return col;
}

// Haze color along a view ray: the fog color, warmed toward the sun.
float3 hazeColor(constant FrameUniforms& f, float3 rd) {
    const float sunAmt = pow(saturate(dot(rd, f.skySun.xyz)), 8.0);
    return f.fogColor.rgb + f.skySunGlow.rgb * 0.22 * sunAmt * f.fogParams.z;
}

// Analytic height fog (exponential in height) with sun in-scattering. Returns rgb: fog color, a: amount.
float4 fogAt(constant FrameUniforms& f, float3 worldPos, float3 camPos) {
    const float3 d = worldPos - camPos;
    const float dist = length(d);
    if (dist < 1e-3) return float4(0.0);
    const float3 rd = d / dist;
    const float a = f.fogParams.x, b = f.fogParams.y;
    const float k = b * rd.y;
    const float camTerm = exp(-b * camPos.y);
    const float optical = abs(k) > 1e-4 ? a * camTerm * (1.0 - exp(-k * dist)) / k : a * camTerm * dist;
    const float fog = 1.0 - exp(-max(optical, 0.0));
    return float4(hazeColor(f, rd), fog);
}

// Low-beam headlights: two unshadowed spot lights on the vehicle (render-only). Returns irradiance.
// Stylized for readability: wrapped N.L so the grazing ground still catches light, and the beam keeps its
// brightness out to ~25 m (real low beams aim their hot spot far ahead) before the inverse-square falloff.
float3 headlightIrradiance(constant FrameUniforms& f, float3 P, float3 N) {
    const float I = f.headlightL.w;
    if (I <= 0.0) return float3(0.0);
    const float3 axis = f.headlightDir.xyz;
    float sum = 0.0;
    for (int i = 0; i < 2; ++i) {
        const float3 d = (i == 0 ? f.headlightL.xyz : f.headlightR.xyz) - P;
        const float d2 = max(dot(d, d), 0.25);
        const float3 l = d * rsqrt(d2);
        const float cosA = dot(-l, axis);
        float beam = smoothstep(f.headlightDir.w, mix(f.headlightDir.w, 1.0, 0.6), cosA) * 0.55 +
                     smoothstep(0.94, 0.995, cosA) * 1.2;
        beam *= 1.0 - smoothstep(0.015, 0.09, -l.y - axis.y); // sharp cut-off above the beam axis
        const float falloff = clamp(d2 / 25.0, 1.0, 30.0) / d2 * (1.0 - smoothstep(38.0, 60.0, sqrt(d2)));
        sum += beam * falloff * saturate(dot(N, l) * 0.75 + 0.25);
    }
    return float3(1.0, 0.93, 0.82) * I * sum;
}

float3 applyFog(constant FrameUniforms& f, float3 color, float3 worldPos, float3 camPos) {
    const float4 fog = fogAt(f, worldPos, camPos);
    return mix(color, fog.rgb, fog.a);
}

// Cascaded shadow lookup. Returns 1 = lit.
float shadowFactor(constant FrameUniforms& f, depth2d_array<float> sm, sampler cmp, float3 P, float3 N) {
    const int cascades = int(f.shadowParams.x);
    if (cascades <= 0) return 1.0;
    const float viewDepth = dot(P - f.cameraPos.xyz, f.cameraFwd.xyz);
    int c = 0;
    for (; c < cascades; ++c)
        if (viewDepth < f.cascadeSplits[c]) break;
    if (c >= cascades) return 1.0;
    const float texel = f.shadowParams.y;
    // Normal offset scales with the world size of one shadow texel in this cascade.
    const float3 Pn = P + N * f.cascadeTexel[c] * f.shadowParams.w;
    const float4 sc = f.shadowViewProj[c] * float4(Pn, 1.0);
    float2 uv = sc.xy * 0.5 + 0.5;
    uv.y = 1.0 - uv.y;
    const float z = sc.z;
    if (any(uv < 0.0) || any(uv > 1.0) || z > 1.0) return 1.0;
    const int filter = int(f.shadowParams.z);
    float lit = 0.0;
    if (filter == 0) {
        lit = sm.sample_compare(cmp, uv, uint(c), z);
    } else if (filter == 1) {
        const float2 o = float2(texel) * 1.2;
        lit = sm.sample_compare(cmp, uv + float2(-o.x, -o.y), uint(c), z) + sm.sample_compare(cmp, uv + float2(o.x, -o.y), uint(c), z) +
              sm.sample_compare(cmp, uv + float2(-o.x, o.y), uint(c), z) + sm.sample_compare(cmp, uv + float2(o.x, o.y), uint(c), z);
        lit *= 0.25;
    } else {
        const float2 taps[12] = {float2(-0.326, -0.406), float2(-0.840, -0.074), float2(-0.696, 0.457), float2(-0.203, 0.621),
                                 float2(0.962, -0.195), float2(0.473, -0.480), float2(0.519, 0.767), float2(0.185, -0.893),
                                 float2(0.507, 0.064), float2(0.896, 0.412), float2(-0.322, -0.933), float2(-0.792, -0.598)};
        for (int i = 0; i < 12; ++i) lit += sm.sample_compare(cmp, uv + taps[i] * texel * 2.2, uint(c), z);
        lit /= 12.0;
    }
    // Fade out at the end of the last cascade.
    const float fadeStart = f.cascadeSplits[cascades - 1] * 0.85;
    const float fade = saturate((viewDepth - fadeStart) / (f.cascadeSplits[cascades - 1] - fadeStart));
    return mix(lit, 1.0, fade);
}

// Ground maps (mask: R grass, G mud weight, B wetness).
float2 groundUV(constant FrameUniforms& f, float2 xz) { return (xz - f.groundRect.xy) * f.groundRect.zw; }

float4 sampleMask(constant FrameUniforms& f, texture2d<float> mask, sampler s, float2 xz) {
    const float2 uv = groundUV(f, xz);
    if (any(uv < 0.0) || any(uv > 1.0)) return float4(0.55, 0.0, 0.0, 1.0); // backdrop outside the walls
    return mask.sample(s, uv, level(0));
}

// Ambient occlusion under the vehicle (soft rounded box in body space).
float vehicleAO(constant FrameUniforms& f, float3 P) {
    const float3 lp = (f.vehicleInv * float4(P, 1.0)).xyz;
    const float2 q = abs(lp.xz) - float2(0.80, 2.05);
    const float d = length(max(q, 0.0)) + min(max(q.x, q.y), 0.0);
    const float foot = 1.0 - smoothstep(-0.3, 1.1, d);
    const float h = saturate(1.0 - abs(lp.y + 0.78) / 1.0);
    return 1.0 - 0.65 * foot * h;
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
float3 ambientLight(constant FrameUniforms& f, float ny) {
    return mix(f.groundAmbient.rgb, f.skyAmbient.rgb, ny * 0.5 + 0.5);
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

float3 shade(constant FrameUniforms& f, ShadeInput s) {
    const float3 L = f.sunDir.xyz;
    const float3 N = s.N, V = s.V;
    const float3 H = normalize(L + V);
    const float NdotL = saturate(dot(N, L));
    const float NdotV = max(dot(N, V), 1e-3);
    const float NdotH = saturate(dot(N, H));
    const float VdotH = saturate(dot(V, H));
    const float3 F0 = mix(float3(0.04), s.albedo, s.metallic);
    const float3 F = F0 + (1.0 - F0) * pow(1.0 - VdotH, 5.0);
    const float3 spec = F * ggxSpec(NdotH, NdotV, NdotL, s.roughness);
    const float3 kd = (1.0 - F) * (1.0 - s.metallic);
    float3 color = (kd * s.albedo / PI + spec) * f.sunColor.rgb * NdotL * s.shadow;
    color += kd * s.albedo / PI * s.lamp;

    // Ambient: sky hemisphere + ground bounce.
    const float3 amb = ambientLight(f, N.y);
    color += amb * s.albedo * (1.0 - s.metallic) * s.ao;
    // Environment specular from the procedural sky (blurred toward the ambient with roughness).
    const float3 R = reflect(-V, N);
    float3 env = mix(skyRadiance(f, R), amb, saturate(s.roughness * 1.2));
    env *= mix(0.35, 1.0, s.shadow) * s.specOcclusion; // sun-shadowed areas see less of the bright sky
    color += env * envBRDFApprox(F0, s.roughness, NdotV) * s.ao;
    if (s.clearCoat > 0.0) {
        const float Fc = 0.04 + 0.96 * pow(1.0 - NdotV, 5.0);
        const float3 coatSpec = ggxSpec(NdotH, NdotV, NdotL, 0.06) * 0.04 * f.sunColor.rgb * NdotL * s.shadow;
        color = color * (1.0 - Fc * s.clearCoat) + s.clearCoat * (coatSpec + skyRadiance(f, R) * Fc * s.specOcclusion * mix(0.4, 1.0, s.shadow));
    }
    return color + s.emissive;
}

// Bump mapping from screen-space derivatives of a height function (Mikkelsen).
float3 perturbNormal(float3 N, float3 P, float h, float strength) {
    const float3 dpdx = dfdx(P), dpdy = dfdy(P);
    const float dhdx = dfdx(h) * strength, dhdy = dfdy(h) * strength;
    const float3 r1 = cross(dpdy, N), r2 = cross(N, dpdx);
    const float det = dot(dpdx, r1);
    const float3 grad = sign(det) * (dhdx * r1 + dhdy * r2);
    return normalize(abs(det) * N - grad);
}

// ---------------------------------------------------------------- lit meshes

struct LitIn {
    float3 position [[attribute(0)]];
    float3 normal [[attribute(1)]];
    float3 color [[attribute(2)]];
    float4 material [[attribute(3)]];
};

struct LitOut {
    float4 position [[position]];
    float3 worldPos;
    float3 normal;
    float3 color;
    float4 material [[flat]];
    float3 localPos;
    float3 localNormal;
    float mudMode [[flat]]; // DrawUniforms.params.y
};

vertex LitOut lit_vs(LitIn in [[stage_in]], constant FrameUniforms& frame [[buffer(0)]],
                     constant DrawUniforms& draw [[buffer(1)]]) {
    LitOut out;
    const float4 world = draw.model * float4(in.position, 1.0);
    out.position = frame.viewProj * world;
    out.worldPos = world.xyz;
    out.normal = (draw.normalMatrix * float4(in.normal, 0.0)).xyz;
    out.color = in.color * draw.tint.rgb;
    out.material = float4(in.material.xyz, in.material.w * draw.params.x);
    out.localPos = in.position;
    out.localNormal = in.normal;
    out.mudMode = draw.params.y;
    return out;
}

struct GroundSurface {
    float3 albedo;
    float roughness;
    float height; // bump height (m-ish units for perturbNormal)
    float bump;
};

GroundSurface groundMaterial(int mat, float3 P, float3 N, float4 mask, texture2d<float> noise, sampler ns) {
    GroundSurface g;
    const float2 uv = P.xz;
    const float3 nA = sampleNoise(noise, ns, uv * 0.045);
    const float3 nB = sampleNoise(noise, ns, uv * 0.23);
    const float3 nC = sampleNoise(noise, ns, uv * 1.15);
    const float fine = noise.sample(ns, uv * 3.7).a;
    const float large = nA.r * 0.6 + nB.g * 0.4;
    g.bump = 0.6;
    if (mat == MAT_DIRT || mat == MAT_GRASS || mat == MAT_MUD) {
        // Soil: packed dirt with gravel, green undergrowth where the mask has grass.
        // Warm earthy palette (ochre/umber), matching the stylized banner look.
        const float3 dark = float3(0.100, 0.062, 0.035), light = float3(0.245, 0.160, 0.090);
        float3 dirt = mix(dark, light, smoothstep(0.25, 0.75, large));
        const float stone = 1.0 - smoothstep(0.18, 0.30, nC.b);
        dirt = mix(dirt, float3(0.25, 0.20, 0.155) * (0.7 + 0.6 * nC.r), stone * 0.5);
        float h = nC.r * 0.5 + fine * 0.35 + stone * 0.6;
        const float grassAmt = saturate(mask.r * 1.2);
        const float3 under = mix(float3(0.042, 0.048, 0.012), float3(0.100, 0.094, 0.028), nB.r) * (0.8 + 0.4 * fine);
        float3 albedo = mix(dirt, under, grassAmt * (0.8 + 0.2 * nA.g));
        float rough = 0.92;
        h = mix(h, fine * 0.6, grassAmt);

        // Mud: blended by the smooth mask (not per triangle), edges broken up with noise.
        const float mudBlend = smoothstep(0.30, 0.55, mask.g + (nB.r - 0.5) * 0.30 + (fine - 0.5) * 0.08);
        if (mudBlend > 0.0) {
            const float3 dry = float3(0.125, 0.078, 0.042), wetC = float3(0.045, 0.029, 0.016);
            const float wetN = smoothstep(0.45, 0.65, nB.g * 0.7 + nA.r * 0.3);
            const float wet = saturate(wetN * 0.8 + mask.b * 0.5);
            const float3 mudAlb = mix(dry, wetC, wet) * (0.85 + 0.3 * nC.r);
            // Tire-rut streaks along the travel direction (z) and clumps.
            const float ruts = sampleNoise(noise, ns, float2(P.x * 0.9, P.z * 0.08)).g;
            const float mudH = mix(nC.r * 0.6 + fine * 0.3 + ruts * 0.7, ruts * 0.25 + fine * 0.05, wet);
            albedo = mix(albedo, mudAlb, mudBlend);
            rough = mix(rough, mix(0.85, 0.40, wet), mudBlend);
            h = mix(h, mudH, mudBlend);
            g.bump = mix(0.6, 0.9, mudBlend);
        }
        g.albedo = albedo;
        g.roughness = rough;
        g.height = h;
    } else if (mat == MAT_CONCRETE) {
        float3 c = float3(0.30, 0.295, 0.28) * (0.85 + 0.25 * large + 0.12 * (fine - 0.5));
        if (abs(N.y) < 0.5) {
            // Walls: weathered concrete, rain streaks, grime and moss toward the ground.
            const float streak = sampleNoise(noise, ns, float2((P.x + P.z) * 0.6, P.y * 0.05)).g;
            c *= 0.75 + 0.35 * streak;
            const float grime = 1.0 - smoothstep(0.0, 1.4, P.y + (nB.r - 0.5) * 0.6);
            c = mix(c, float3(0.07, 0.075, 0.04), grime * 0.75);
            const float2 cell = abs(fract(float2(P.x + P.z, P.y) / float2(3.0, 100.0)) - 0.5);
            c *= 1.0 - 0.4 * smoothstep(0.490, 0.497, cell.x);
            // Moss and grass hanging over the top edge (the embankment behind the wall is at its top).
            const float top = smoothstep(3.9, 4.9, P.y + (nB.g - 0.5) * 0.9);
            c = mix(c * 0.62, float3(0.035, 0.05, 0.015), top * 0.85);
            g.albedo = c;
            g.roughness = 0.85;
            g.height = fine * 0.5 + streak * 0.3;
            g.bump = 0.5;
        } else {
            // Slab seams every 4 m.
            const float2 cell = abs(fract(P.xz / 4.0) - 0.5);
            const float seam = smoothstep(0.492, 0.498, max(cell.x, cell.y));
            c *= 1.0 - 0.45 * seam;
            // Tire marks.
            c *= 1.0 - 0.18 * smoothstep(0.55, 0.8, sampleNoise(noise, ns, float2(P.x * 0.35, P.z * 0.03)).r);
            g.albedo = c;
            g.roughness = 0.78 - 0.1 * fine;
            g.height = fine * 0.4 - seam * 0.6;
            g.bump = 0.4;
        }
    } else if (mat == MAT_ROCK) {
        const float3 w = abs(N) / (abs(N).x + abs(N).y + abs(N).z);
        const float3 tx = sampleNoise(noise, ns, P.zy * 0.7), ty = sampleNoise(noise, ns, P.xz * 0.7), tz = sampleNoise(noise, ns, P.xy * 0.7);
        const float3 t = tx * w.x + ty * w.y + tz * w.z;
        const float3 tf = sampleNoise(noise, ns, P.zy * 3.1) * w.x + sampleNoise(noise, ns, P.xz * 3.1) * w.y + sampleNoise(noise, ns, P.xy * 3.1) * w.z;
        // Warm tan sandstone; N is the flat facet normal, so each facet gets its own slight tone (stylized low-poly).
        const float facet = hash12(floor(N.xz * 9.0 + N.y * 5.0) + 0.37);
        float3 c = mix(float3(0.085, 0.060, 0.046), float3(0.235, 0.165, 0.118), smoothstep(0.3, 0.7, t.r)) * (0.85 + 0.3 * tf.g);
        c *= 0.70 + 0.60 * facet;
        // Cracks from the cellular channel.
        c *= 0.7 + 0.3 * smoothstep(0.05, 0.18, tf.b);
        const float moss = smoothstep(0.55, 0.9, N.y) * smoothstep(0.45, 0.65, t.g);
        c = mix(c, float3(0.055, 0.065, 0.020), moss * 0.6);
        // Dirt splashed onto the steep sides.
        c = mix(c, float3(0.10, 0.068, 0.04), (1.0 - smoothstep(0.0, 0.6, abs(N.y))) * 0.15);
        g.albedo = c;
        g.roughness = 0.78;
        g.height = t.r * 0.8 + tf.b * 0.6;
        g.bump = 0.35; // low, so the bump does not blur the facets
    } else { // MAT_LOWGRIP: sealed slab, smooth
        g.albedo = float3(0.075, 0.095, 0.12) * (0.9 + 0.2 * large);
        g.roughness = 0.30 + 0.1 * fine;
        g.height = fine * 0.1;
        g.bump = 0.2;
    }
    // Wet ground near the water (dark, glossy, flatter).
    const float wet = mask.b * (mat == MAT_ROCK ? 0.5 : 1.0);
    g.albedo *= mix(1.0, 0.6, wet);
    g.roughness = mix(g.roughness, 0.38, wet * 0.8);
    g.bump *= 1.0 - 0.5 * wet;
    return g;
}

// Cosmetic mud on the vehicle (object-space noise). Body: a splash line that rises with the amount, thrown up
// from below; wheels (radial = 1): caked tread and lower rim. No effect on the simulation.
float mudCover(float amount, float receptivity, float radial, float3 lp, float3 ln, texture2d<float> noise, sampler ns) {
    if (amount * receptivity <= 0.001) return 0.0;
    const float3 w = abs(ln) / (abs(ln).x + abs(ln).y + abs(ln).z + 1e-4);
    // Streaky splash noise (stretched vertically) plus fine droplets.
    const float n = noise.sample(ns, float2(lp.z * 1.6, lp.y * 0.6)).g * w.x + noise.sample(ns, lp.xz * 1.6).g * w.y +
                    noise.sample(ns, float2(lp.x * 1.6, lp.y * 0.6)).g * w.z;
    const float sp = noise.sample(ns, lp.zy * 7.3 + lp.x).a * w.x + noise.sample(ns, lp.xz * 7.3).a * w.y +
                     noise.sample(ns, lp.xy * 7.3).a * w.z;
    float height;
    if (radial > 0.5) {
        height = 1.0 - saturate(length(lp.yz) / 0.42); // 0 on the tread, 1 at the hub
    } else {
        height = saturate((lp.y + 0.45) / 1.6);        // 0 under the sills, 1 at the roof
    }
    const float t = amount * receptivity * 0.85 - height + (n - 0.5) * 0.35 + (sp - 0.5) * 0.45;
    return smoothstep(0.0, 0.06, t);
}

fragment float4 lit_fs(LitOut in [[stage_in]], bool front [[front_facing]], constant FrameUniforms& f [[buffer(0)]],
                       depth2d_array<float> shadowMap [[texture(0)]], texture2d<float> noise [[texture(1)]],
                       texture2d<float> mask [[texture(2)]], sampler shadowSampler [[sampler(0)]],
                       sampler noiseSampler [[sampler(1)]], sampler maskSampler [[sampler(2)]]) {
    if (dot(f.clipPlane.xyz, in.worldPos) + f.clipPlane.w < 0.0) discard_fragment();
    const int mat = int(in.material.x + 0.5);
    float3 N = normalize(in.normal);
    const float3 V = normalize(f.cameraPos.xyz - in.worldPos);
    const float3 P = in.worldPos;

    ShadeInput s;
    s.albedo = in.color;
    s.roughness = in.material.y;
    s.metallic = in.material.z;
    s.ao = 1.0;
    s.specOcclusion = 1.0;
    s.emissive = float3(0.0);
    s.clearCoat = 0.0;

    const bool ground = mat >= MAT_DIRT && mat <= MAT_GRASS;
    if (ground) {
        const float4 m = sampleMask(f, mask, maskSampler, P.xz);
        GroundSurface g = groundMaterial(mat, P, N, m, noise, noiseSampler);
        s.albedo = g.albedo;
        s.roughness = g.roughness;
        s.metallic = 0.0;
        N = perturbNormal(N, P, g.height * 0.035, g.bump);
        s.ao = vehicleAO(f, P);
        s.specOcclusion = 0.75; // clumps and blades occlude part of the sky reflection
        if (f.options.x > 0.0) {
            const float2 cell = abs(fract(P.xz) - 0.5);
            s.albedo *= 1.0 - f.options.x * smoothstep(0.47, 0.5, max(cell.x, cell.y));
        }
    } else {
        if (!front) N = -N;
        if (mat == MAT_GLASS) {
            s.roughness = 0.03;
            s.specOcclusion = 1.0;
        } else if (mat == MAT_HEADLIGHT) {
            s.emissive = in.color * (0.6 + 7.0 * f.vehicleParams.w);
            s.albedo = float3(0.8);
            s.roughness = 0.15;
        } else if (mat == MAT_BRAKE) {
            s.emissive = in.color * (0.9 + 9.0 * f.vehicleParams.y);
            s.albedo = in.color * 0.3;
        } else if (mat == MAT_REVERSE) {
            s.emissive = in.color * (6.0 * f.vehicleParams.z);
            s.albedo = float3(0.6);
        } else if (mat == MAT_AMBER) {
            s.emissive = in.color * 1.2;
            s.albedo = in.color * 0.4;
        } else if (mat == MAT_FOLIAGE) {
            const float n = noise.sample(noiseSampler, P.xz * 0.35 + P.y * 0.2).r;
            s.albedo = in.color * (0.7 + 0.6 * n);
            s.roughness = 0.9;
            // Light through the needles when backlit.
            s.emissive = s.albedo * f.sunColor.rgb * 0.08 * pow(saturate(dot(-V, f.sunDir.xyz)), 3.0);
        } else if (mat == MAT_PAINT) {
            s.clearCoat = 1.0;
            // Subtle metallic flake.
            const float flake = noise.sample(noiseSampler, in.localPos.xy * 37.0 + in.localPos.z * 11.0).a;
            s.albedo *= 0.92 + 0.16 * flake;
        } else if (mat == MAT_RUBBER) {
            const float n = noise.sample(noiseSampler, in.localPos.yz * 6.0).a;
            s.albedo *= 0.85 + 0.3 * n;
        }
        // Cosmetic mud layer (no effect on physics).
        const float mud = mudCover(f.vehicleParams.x, in.material.w, in.mudMode, in.localPos, in.localNormal, noise,
                                   noiseSampler);
        if (mud > 0.0) {
            const float wetMud = noise.sample(noiseSampler, in.localPos.zy * 1.7 + 0.3).g;
            const float3 mudC = mix(float3(0.13, 0.090, 0.055), float3(0.050, 0.034, 0.020), smoothstep(0.4, 0.7, wetMud));
            s.albedo = mix(s.albedo, mudC, mud);
            s.roughness = mix(s.roughness, mix(0.95, 0.35, smoothstep(0.4, 0.7, wetMud)), mud);
            s.metallic = mix(s.metallic, 0.0, mud);
            s.clearCoat *= 1.0 - mud;
            s.emissive *= 1.0 - 0.85 * mud;
        }
        s.ao = 1.0;
    }
    s.N = N;
    s.V = V;
    s.lamp = headlightIrradiance(f, P, N);
    s.shadow = shadowFactor(f, shadowMap, shadowSampler, P, normalize(in.normal) * (front ? 1.0 : -1.0));
    float3 color = shade(f, s);
    color = applyFog(f, color, P, f.cameraPos.xyz);
    return float4(color, 1.0);
}

// ---------------------------------------------------------------- shadow pass (depth only)

struct ShadowUniforms {
    float4x4 lightViewProj;
};

vertex float4 shadow_vs(LitIn in [[stage_in]], constant ShadowUniforms& su [[buffer(0)]],
                        constant DrawUniforms& draw [[buffer(1)]]) {
    return su.lightViewProj * (draw.model * float4(in.position, 1.0));
}

fragment void shadow_fs() {}

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

vertex LitOut tree_vs(LitIn in [[stage_in]], uint iid [[instance_id]], constant FrameUniforms& frame [[buffer(0)]],
                      constant TreeUniforms& tu [[buffer(1)]], const device TreeInstance* trees [[buffer(2)]]) {
    const TreeInstance t = trees[uint(tu.info.x) + iid];
    LitOut out;
    const float3 world = treeWorld(t, in.position, frame.cameraPos.w);
    out.position = frame.viewProj * float4(world, 1.0);
    out.worldPos = world;
    const float c = cos(t.params.x), s = sin(t.params.x);
    const float3 n = float3(c * in.normal.x - s * in.normal.z, in.normal.y, s * in.normal.x + c * in.normal.z);
    out.normal = normalize(float3(n.x / t.params.z, n.y, n.z / t.params.z));
    // Per-tree hue: from olive/yellow-green to deep blue-green.
    const float v = t.params.y;
    out.color = in.color * mix(float3(1.18, 1.08, 0.70), float3(0.80, 0.95, 1.10), v) * (0.85 + 0.3 * fract(v * 7.31));
    out.material = float4(in.material.xyz, 0.0);
    out.localPos = in.position;
    out.localNormal = in.normal;
    out.mudMode = 0.0;
    return out;
}

vertex float4 tree_shadow_vs(LitIn in [[stage_in]], uint iid [[instance_id]], constant ShadowUniforms& su [[buffer(0)]],
                             constant TreeUniforms& tu [[buffer(1)]], const device TreeInstance* trees [[buffer(2)]]) {
    const TreeInstance t = trees[uint(tu.info.x) + iid];
    return su.lightViewProj * float4(treeWorld(t, in.position, tu.info.y), 1.0);
}

// ---------------------------------------------------------------- sky

struct FullscreenOut {
    float4 position [[position]];
    float2 uv;
};

vertex FullscreenOut fullscreen_vs(uint vid [[vertex_id]]) {
    FullscreenOut out;
    const float2 p = float2((vid << 1) & 2, vid & 2);
    out.position = float4(p * 2.0 - 1.0, 1.0, 1.0); // z = 1: far plane (sky drawn behind everything)
    out.uv = float2(p.x, 1.0 - p.y);
    return out;
}

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
float4 mountains(constant FrameUniforms& f, float3 dir) {
    const float el = dir.y;
    if (el > 0.205) return float4(0.0); // above the highest ridge (amp[0])
    const float az = atan2(dir.x, dir.z);
    const float dv = abs(fmod(az - f.fogColor.w + 3.0 * PI, 2.0 * PI) - PI); // angle from the valley center
    const float aa = max(fwidth(el), 1e-5) * 1.2;
    const float3 V0 = -float3(sin(az), 0.0, cos(az)); // facing the viewer
    const float3 T = float3(cos(az), 0.0, -sin(az));  // along the ridge (increasing azimuth)
    // Aerial perspective toward a cooler, violet version of the haze (stylized: distant ranges read blue-violet
    // even against a warm sunset sky).
    const float3 warmHaze = hazeColor(f, normalize(float3(dir.x, max(dir.y, 0.0), dir.z)));
    const float3 haze = mix(warmHaze, luminance(warmHaze) * float3(0.78, 0.74, 1.12), 0.55);
    const float segs[4] = {24.0, 36.0, 54.0, 81.0};
    const float amp[4] = {0.200, 0.150, 0.110, 0.078};
    const float keep[4] = {0.40, 0.48, 0.60, 0.75};   // ridge height kept at the valley center
    const float hazeAmt[4] = {0.70, 0.55, 0.40, 0.26};
    const float3 albedo[4] = {float3(0.20, 0.20, 0.36), float3(0.17, 0.17, 0.32), float3(0.13, 0.14, 0.27),
                              float3(0.10, 0.12, 0.20)};
    float3 col = float3(0.0);
    float cover = 0.0;
    for (int i = 0; i < 4; ++i) {
        const float n = segs[i];
        const float x = (az / (2.0 * PI) + 0.5) * n;
        const float k = floor(x), t = x - k;
        const float seed = float(i) * 17.3 + 3.0;
        const float k0 = fmod(k, n), k1 = fmod(k + 1.0, n);
        const float valley = mix(keep[i], 1.0, smoothstep(0.05, 0.40, dv));
        const float h0 = amp[i] * ridgeHeight(k0, seed) * valley, h1 = amp[i] * ridgeHeight(k1, seed) * valley;
        const float h = mix(h0, h1, t) - 0.003;
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
        float3 lit = albedo[i] * tone * (f.skyAmbient.rgb * 1.1 + f.sunColor.rgb * saturate(dot(N, f.sunDir.xyz)) * 0.30);
        // Backlit rim along the ridge toward the sun (sunset), and mist settling in the valleys.
        const float rim = pow(saturate(dot(dir, f.skySun.xyz)), 5.0) * (1.0 - smoothstep(0.0, 0.006, h - el));
        lit += f.skySunGlow.rgb * rim * 0.35;
        const float mist = 1.0 - smoothstep(-0.02, hp * 0.9, el);
        const float3 layer = mix(lit, haze, saturate(hazeAmt[i] + (1.0 - hazeAmt[i]) * 0.35 * mist));
        col = mix(col, layer, c);
        cover = max(cover, c);
    }
    return float4(col, cover);
}

fragment float4 sky_fs(FullscreenOut in [[stage_in]], constant FrameUniforms& f [[buffer(0)]],
                       texture2d<float> noise [[texture(0)]], sampler ns [[sampler(0)]]) {
    const float2 ndc = float2(in.uv.x * 2.0 - 1.0, 1.0 - in.uv.y * 2.0);
    const float4 w = f.invViewProj * float4(ndc, 1.0, 1.0);
    const float3 dir = normalize(w.xyz / w.w - f.cameraPos.xyz);
    float3 col = skyRadiance(f, dir);
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
        col = mix(col, f.sunDisc.rgb * (0.85 + 0.35 * sqrt(core)), disc * saturate(luminance(f.sunDisc.rgb) * 4.0));
    }
    // Moon disc with a faint halo.
    if (f.sunDisc.w > 0.0) {
        const float cosM = 0.99975;
        const float disc = smoothstep(cosM - 0.00002, cosM + 0.00002, muMoon);
        const float maria = noise.sample(ns, (dir.xz - f.moonDir.xz) * 40.0 + 0.5).r;
        col = mix(col, float3(0.80, 0.84, 0.95) * (0.9 + 0.4 * maria) * 1.4, disc * f.sunDisc.w);
        col += float3(0.10, 0.12, 0.18) * pow(saturate(muMoon), 400.0) * f.sunDisc.w * 0.6;
    }
    // Clouds on a plane far above, stretched into streaks.
    if (f.options.w > 0.5 && dir.y > 0.01) {
        const float t = f.cameraPos.w;
        const float2 uv = dir.xz / (dir.y + 0.08) * float2(0.10, 0.24) + float2(t * 0.0012, t * 0.001);
        const float n0 = noise.sample(ns, uv).r * 0.65 + noise.sample(ns, uv * 2.7 + 0.3).g * 0.35;
        const float n = saturate((n0 - 0.5) * 2.6 + 0.5); // the fbm is centered at 0.5 with a small spread
        const float cover = f.fogParams.w;
        const float c = smoothstep(1.0 - cover, 1.0 - cover + 0.3, n) * smoothstep(0.01, 0.20, dir.y);
        const float thick = noise.sample(ns, uv * 1.3 + 0.17).r;
        const float muL = dot(dir, f.sunDir.xyz);
        float3 lit = f.sunColor.rgb * 0.11 * (0.75 + 0.6 * pow(saturate(muL), 4.0)) + f.skyZenith.rgb * 0.5;
        lit += f.skySunGlow.rgb * (0.06 + 0.25 * pow(saturate(mu), 3.0)); // sunset-lit undersides
        const float3 cloudCol = mix(lit, lit * 0.55 + f.skyHorizon.rgb * 0.2, thick * c);
        col = mix(col, cloudCol, c * 0.9);
    }
    // Horizon haze matches the fog color.
    const float haze = pow(1.0 - saturate(abs(dir.y)), 6.0) * 0.6;
    col = mix(col, hazeColor(f, dir), haze);
    const float4 m = mountains(f, dir);
    col = mix(col, m.rgb, m.a);
    return float4(col, 1.0);
}

// ---------------------------------------------------------------- water (showcase puddles)

float rippleHeight(texture2d<float> noise, sampler ns, float2 p, float t) {
    return noise.sample(ns, p * 0.11 + float2(t * 0.006, t * 0.004)).g +
           0.35 * noise.sample(ns, p * 0.37 - float2(t * 0.011, -t * 0.008)).g;
}

fragment float4 water_fs(LitOut in [[stage_in]], constant FrameUniforms& f [[buffer(0)]],
                         depth2d_array<float> shadowMap [[texture(0)]], texture2d<float> noise [[texture(1)]],
                         texture2d<float> heightTex [[texture(2)]], texture2d<float> reflTex [[texture(3)]],
                         sampler shadowSampler [[sampler(0)]], sampler ns [[sampler(1)]], sampler cs [[sampler(2)]],
                         sampler rs [[sampler(3)]]) {
    const float3 P = in.worldPos;
    const float terrain = heightTex.sample(cs, groundUV(f, P.xz), level(0)).r;
    const float depth = f.waterParams.x - terrain;
    if (depth <= 0.0) discard_fragment();
    const float t = f.cameraPos.w;
    // Ripples: two scrolling noise layers, normal from finite differences.
    const float e = 0.08;
    const float h0 = rippleHeight(noise, ns, P.xz, t), hx = rippleHeight(noise, ns, P.xz + float2(e, 0), t),
                hz = rippleHeight(noise, ns, P.xz + float2(0, e), t);
    const float rip = 0.035;
    const float3 N = normalize(float3(-(hx - h0) / e * rip, 1.0, -(hz - h0) / e * rip));
    const float3 V = normalize(f.cameraPos.xyz - P);
    const float NdotV = saturate(dot(N, V));
    const float F = 0.02 + 0.98 * pow(1.0 - NdotV, 5.0);
    float3 refl;
    if (f.waterParams.y > 0.5) {
        const float2 suv = in.position.xy * f.waterParams.zw + N.xz * 0.06;
        refl = reflTex.sample(rs, suv).rgb;
    } else {
        refl = skyRadiance(f, reflect(-V, N));
    }
    const float shadow = shadowFactor(f, shadowMap, shadowSampler, P, float3(0, 1, 0));
    const float3 H = normalize(f.sunDir.xyz + V);
    const float spec = ggxSpec(saturate(dot(N, H)), max(NdotV, 1e-3), saturate(dot(N, f.sunDir.xyz)), 0.07);
    const float3 sunSpec = f.sunColor.rgb * spec * 0.02 * saturate(dot(N, f.sunDir.xyz)) * shadow;
    // Muddy water body: opaque brown with depth, see-through at the very edge.
    const float3 skyUp = mix(f.skyHorizon.rgb, f.skyZenith.rgb, 0.5);
    const float3 body = float3(0.055, 0.040, 0.026) * (skyUp + f.sunColor.rgb * 0.15 * shadow);
    const float aBody = saturate(depth / 0.10) * 0.88;
    const float edge = smoothstep(0.0, 0.025, depth);
    float3 src = (body * aBody * (1.0 - F) + refl * F + sunSpec) * edge;
    const float srcAlpha = edge * (1.0 - (1.0 - F) * (1.0 - aBody));
    // Fog over the premultiplied contribution (the background is already fogged).
    const float4 fog = fogAt(f, P, f.cameraPos.xyz);
    src = src * (1.0 - fog.a) + fog.rgb * fog.a * srcAlpha;
    return float4(src, srcAlpha);
}

// ---------------------------------------------------------------- grass (GPU-instanced blades)

struct GrassUniforms {
    float4 lod;      // x: fine cell (m), y: LOD level, z: blades per chunk side, w: first chunk index
    float4 fade;     // x: this LOD's outer radius, y: fade band, z: max distance, w: blade segments
    float4 blade;    // x: height scale, y: width (m), z: wind, w: chunk size (m)
    float4 wheels[4]; // xyz: wheel contact point, w: flatten radius (0 = none)
};

struct GrassOut {
    float4 position [[position]];
    float3 worldPos;
    float3 normal;
    float3 color;
    float t; // 0 at the root, 1 at the tip
};

vertex GrassOut grass_vs(uint vid [[vertex_id]], uint iid [[instance_id]], constant FrameUniforms& f [[buffer(0)]],
                         constant GrassUniforms& g [[buffer(1)]], const device float4* chunks [[buffer(2)]],
                         texture2d<float> heightTex [[texture(0)]], texture2d<float> mask [[texture(1)]],
                         sampler cs [[sampler(0)]], sampler ms [[sampler(1)]]) {
    GrassOut out;
    const int perSide = int(g.lod.z);
    const int perChunk = perSide * perSide;
    const int chunk = int(g.lod.w) + int(iid) / perChunk;
    const int cell = int(iid) % perChunk;
    const int ci = cell % perSide, cj = cell / perSide;
    const float levelScale = exp2(g.lod.y);
    const float fine = g.lod.x;
    const float4 ch = chunks[chunk];
    // Blade identity on the finest grid, so coarser LODs keep a subset of the same blades.
    const float2 gi = floor(ch.xy / fine + 0.5) + float2(ci, cj) * levelScale;
    const float r1 = hash12(gi), r2 = hash12(gi + 17.31), r3 = hash12(gi + 41.7), r4 = hash12(gi + 73.1);
    float2 root = (gi + 0.5 + (float2(r1, r2) - 0.5) * 0.9) * fine;

    const float4 m = mask.sample(ms, groundUV(f, root), level(0));
    const float density = m.r;
    const float dist = length(root - f.cameraPos.xz);
    // Blades that do not exist in the next (coarser) LOD fade out before its boundary; the others widen.
    const bool inNext = (ci % 2 == 0) && (cj % 2 == 0);
    const float edgeT = smoothstep(g.fade.x - g.fade.y, g.fade.x, dist);
    float scale = inNext ? 1.0 : 1.0 - edgeT;
    const float widen = inNext ? mix(1.0, 1.6, edgeT) : 1.0;
    scale *= 1.0 - smoothstep(g.fade.z * 0.8, g.fade.z, dist);
    const bool alive = r3 < density * 1.15 && scale > 0.02;

    const int segs = int(g.fade.w);
    const int tri = int(vid) / 3, k = int(vid) % 3;
    const int sv = tri + k; // strip vertex index: 0..2*segs
    const int row = sv / 2;
    const float side = (sv % 2 == 0) ? -1.0 : 1.0;
    const float t = min(float(row) / float(segs), 1.0);
    const bool tip = sv >= 2 * segs;

    const float terrain = heightTex.sample(cs, groundUV(f, root), level(0)).r - 0.03;
    const float yaw = r4 * 2.0 * PI;
    const float2 facing = float2(cos(yaw), sin(yaw));
    const float2 across = float2(-facing.y, facing.x);
    float height = (0.22 + 0.33 * r1) * (0.55 + 0.45 * density) * g.blade.x * scale;
    // Wheels flatten the grass they stand on (only when the contact is near this ground height).
    float2 push = float2(0.0);
    float flatten = 0.0;
    for (int w = 0; w < 4; ++w) {
        if (g.wheels[w].w <= 0.0) continue;
        const float2 dlt = root - g.wheels[w].xz;
        const float dd = length(dlt);
        const float infl = (1.0 - smoothstep(g.wheels[w].w * 0.5, g.wheels[w].w * 1.4, dd)) *
                           (1.0 - smoothstep(0.3, 0.8, abs(g.wheels[w].y - terrain)));
        flatten = max(flatten, infl);
        push += dd > 1e-3 ? dlt / dd * infl : float2(0.0);
    }
    height *= 1.0 - 0.75 * flatten;
    const float time = f.cameraPos.w;
    const float wind = g.blade.z * (sin(time * 1.7 + root.x * 0.35 + root.y * 0.22) * 0.5 +
                                    sin(time * 3.1 + root.x * 1.3 - root.y * 0.7) * 0.18);
    const float bendAmt = (0.25 + 0.35 * r2) + wind;
    const float2 bendDir = normalize(facing * 0.6 + float2(0.7, 0.3) + push * 2.5);
    const float width = g.blade.y * levelScale * 0.6 * widen * (0.75 + 0.5 * r3);

    const float tt = tip ? 1.0 : t;
    float3 p = float3(root.x, terrain, root.y);
    p.y += height * tt * (1.0 - 0.35 * bendAmt * tt);
    p.xz += bendDir * (bendAmt * tt * tt * height * (1.0 + flatten * 2.0));
    if (!tip) p.xz += across * side * width * 0.5 * (1.0 - t * 0.85);

    if (!alive) p = float3(0.0, -1000.0, 0.0);
    out.position = f.viewProj * float4(p, 1.0);
    out.worldPos = p;
    const float3 bladeN = normalize(float3(bendDir.x, 0.0, bendDir.y) * -1.0 + float3(0, 0.6, 0));
    out.normal = normalize(mix(bladeN, float3(0, 1, 0), 0.55));
    // Color: dark root, lighter yellow-green tips, per-blade hue variation and dry patches.
    const float dry = smoothstep(0.55, 0.85, hash12(floor(root * 0.5)) * 0.6 + r2 * 0.4);
    const float patchHue = hash12(floor(root * 0.25) + 3.7);
    const float3 base = mix(float3(0.022, 0.030, 0.006), float3(0.048, 0.050, 0.012), r3);
    const float3 green = mix(float3(0.085, 0.118, 0.018), float3(0.140, 0.150, 0.030), r1);
    const float3 tipC = mix(mix(green, float3(0.110, 0.125, 0.032), patchHue * 0.6), float3(0.24, 0.17, 0.06), dry * 0.8);
    out.color = mix(base, tipC, tt);
    out.t = tt;
    return out;
}

fragment float4 grass_fs(GrassOut in [[stage_in]], constant FrameUniforms& f [[buffer(0)]],
                         depth2d_array<float> shadowMap [[texture(0)]], sampler shadowSampler [[sampler(0)]]) {
    const float3 P = in.worldPos;
    const float3 V = normalize(f.cameraPos.xyz - P);
    const float3 N = normalize(in.normal);
    const float shadow = shadowFactor(f, shadowMap, shadowSampler, P, float3(0, 1, 0));
    const float NdotL = saturate(dot(N, f.sunDir.xyz) * 0.6 + 0.4); // wrapped diffuse
    const float ao = mix(0.3, 1.0, in.t) * vehicleAO(f, P);
    float3 c = in.color * ((f.sunColor.rgb * NdotL * shadow + headlightIrradiance(f, P, N)) / PI + ambientLight(f, 1.0) * ao);
    // Translucency when looking toward the sun.
    const float back = pow(saturate(dot(-V, f.sunDir.xyz)), 4.0);
    c += in.color * f.sunColor.rgb * back * 0.25 * shadow * in.t;
    // Sheen on the tips.
    const float3 H = normalize(f.sunDir.xyz + V);
    c += f.sunColor.rgb * 0.015 * pow(saturate(dot(N, H)), 24.0) * shadow * in.t;
    c = applyFog(f, c, P, f.cameraPos.xyz);
    return float4(c, 1.0);
}

// ---------------------------------------------------------------- debug lines

struct LineIn {
    float3 position [[attribute(0)]];
    float4 color [[attribute(1)]];
};

struct LineOut {
    float4 position [[position]];
    float4 color;
};

vertex LineOut line_vs(LineIn in [[stage_in]], constant FrameUniforms& frame [[buffer(0)]]) {
    LineOut out;
    out.position = frame.viewProj * float4(in.position, 1.0);
    out.color = in.color;
    return out;
}

// Inverse of the tonemap curve: debug line colors are given as display values and must come out unchanged.
float inverseAces(float y) {
    // y = (x(ax + b)) / (x(cx + d) + e) solved for x.
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    y = clamp(y, 0.0, 0.985);
    const float A = a - c * y, B = b - d * y, C = -e * y;
    return (-B + sqrt(max(B * B - 4.0 * A * C, 0.0))) / (2.0 * A);
}

fragment float4 line_fs(LineOut in [[stage_in]], constant FrameUniforms& f [[buffer(0)]]) {
    const float3 display = pow(saturate(in.color.rgb), float3(2.2));
    const float3 hdr = float3(inverseAces(display.r), inverseAces(display.g), inverseAces(display.b)) /
                       max(f.options.y * ACES_PRE_EXPOSURE, 1e-3);
    return float4(hdr, in.color.a);
}

// ---------------------------------------------------------------- post: bloom, tonemap, FXAA

struct PostUniforms {
    float4 texel;   // xy: 1/source size, zw: 1/target size
    float4 params;  // x: bloom threshold, y: bloom strength, z: exposure, w: mode/flags
    float4 extra;   // x: vignette, y: FXAA on (tonemap writes luma to alpha), z: render scale u, w: v
};

float3 karisWeight(float3 c) { return c / (1.0 + luminance(c)); }

fragment float4 bloom_down_fs(FullscreenOut in [[stage_in]], constant PostUniforms& p [[buffer(0)]],
                              texture2d<float> src [[texture(0)]], sampler ls [[sampler(0)]]) {
    const float2 uv = in.uv * p.extra.zw;
    const float2 t = p.texel.xy;
    // 13-tap downsample (Jimenez 2014).
    const float3 a = src.sample(ls, uv + t * float2(-2, -2)).rgb, b = src.sample(ls, uv + t * float2(0, -2)).rgb,
                 c = src.sample(ls, uv + t * float2(2, -2)).rgb, d = src.sample(ls, uv + t * float2(-1, -1)).rgb,
                 e = src.sample(ls, uv + t * float2(1, -1)).rgb, f0 = src.sample(ls, uv + t * float2(-2, 0)).rgb,
                 g = src.sample(ls, uv).rgb, h = src.sample(ls, uv + t * float2(2, 0)).rgb,
                 i = src.sample(ls, uv + t * float2(-1, 1)).rgb, j = src.sample(ls, uv + t * float2(1, 1)).rgb,
                 k = src.sample(ls, uv + t * float2(-2, 2)).rgb, l = src.sample(ls, uv + t * float2(0, 2)).rgb,
                 m = src.sample(ls, uv + t * float2(2, 2)).rgb;
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

fragment float4 bloom_up_fs(FullscreenOut in [[stage_in]], constant PostUniforms& p [[buffer(0)]],
                            texture2d<float> src [[texture(0)]], sampler ls [[sampler(0)]]) {
    const float2 t = p.texel.xy;
    const float2 uv = in.uv;
    // 3x3 tent upsample, added onto the larger level by blending.
    float3 c = src.sample(ls, uv).rgb * 4.0;
    c += (src.sample(ls, uv + float2(-t.x, 0)).rgb + src.sample(ls, uv + float2(t.x, 0)).rgb +
          src.sample(ls, uv + float2(0, -t.y)).rgb + src.sample(ls, uv + float2(0, t.y)).rgb) * 2.0;
    c += src.sample(ls, uv + float2(-t.x, -t.y)).rgb + src.sample(ls, uv + float2(t.x, -t.y)).rgb +
         src.sample(ls, uv + float2(-t.x, t.y)).rgb + src.sample(ls, uv + float2(t.x, t.y)).rgb;
    return float4(c / 16.0, 1.0);
}

float3 aces(float3 x) {
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return saturate((x * (a * x + b)) / (x * (c * x + d) + e));
}

fragment float4 tonemap_fs(FullscreenOut in [[stage_in]], constant PostUniforms& p [[buffer(0)]],
                           texture2d<float> hdr [[texture(0)]], texture2d<float> bloom [[texture(1)]],
                           sampler ls [[sampler(0)]], sampler bs [[sampler(1)]]) {
    const float2 uv = in.uv * p.extra.zw; // the 3D image may use only part of the HDR texture (render scale)
    float3 c = hdr.sample(ls, uv).rgb;
    if (p.params.y > 0.0) c += bloom.sample(bs, uv).rgb * p.params.y;
    c *= p.params.z * ACES_PRE_EXPOSURE;
    float3 mapped = aces(c);
    // Vignette.
    const float2 q = in.uv - 0.5;
    mapped *= 1.0 - p.extra.x * dot(q, q) * 1.6;
    float3 display = pow(saturate(mapped), float3(1.0 / 2.2));
    // Dither against banding in the sky.
    const float n = hash12(in.position.xy) - 0.5;
    display += n / 255.0;
    return float4(display, p.extra.y > 0.5 ? luminance(display) : 1.0);
}

// FXAA (simplified 3.11 "quality" variant with 5 edge steps).
fragment float4 fxaa_fs(FullscreenOut in [[stage_in]], constant PostUniforms& p [[buffer(0)]],
                        texture2d<float> src [[texture(0)]], sampler ls [[sampler(0)]]) {
    const float2 uv = in.uv;
    const float2 t = p.texel.xy;
    const float4 center = src.sample(ls, uv);
    const float lM = center.a;
    const float lN = src.sample(ls, uv + float2(0, -t.y)).a, lS = src.sample(ls, uv + float2(0, t.y)).a;
    const float lE = src.sample(ls, uv + float2(t.x, 0)).a, lW = src.sample(ls, uv + float2(-t.x, 0)).a;
    const float lMax = max(lM, max(max(lN, lS), max(lE, lW))), lMin = min(lM, min(min(lN, lS), min(lE, lW)));
    const float range = lMax - lMin;
    if (range < max(0.0312, lMax * 0.125)) return float4(center.rgb, 1.0);
    const float lNW = src.sample(ls, uv + float2(-t.x, -t.y)).a, lNE = src.sample(ls, uv + float2(t.x, -t.y)).a;
    const float lSW = src.sample(ls, uv + float2(-t.x, t.y)).a, lSE = src.sample(ls, uv + float2(t.x, t.y)).a;
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
    float e1 = src.sample(ls, uv1).a - lLocal, e2 = src.sample(ls, uv2).a - lLocal;
    bool d1 = abs(e1) >= gScaled, d2 = abs(e2) >= gScaled;
    const float steps[5] = {1.5, 2.0, 2.0, 4.0, 8.0};
    for (int i = 0; i < 5 && !(d1 && d2); ++i) {
        if (!d1) { uv1 -= off * steps[i]; e1 = src.sample(ls, uv1).a - lLocal; d1 = abs(e1) >= gScaled; }
        if (!d2) { uv2 += off * steps[i]; e2 = src.sample(ls, uv2).a - lLocal; d2 = abs(e2) >= gScaled; }
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
    return float4(src.sample(ls, fuv).rgb, 1.0);
}
