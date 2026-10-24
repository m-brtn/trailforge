// TrailForge scene shaders (MSL, Metal).
// Resource binding follows the SDL GPU rules for MSL:
//   vertex shader: uniform slot 0 -> [[buffer(0)]], slot 1 -> [[buffer(1)]];
//   vertex buffers via [[stage_in]] (SDL places them starting at buffer(14));
//   fragment shader: uniform slot 0 -> [[buffer(0)]], slot 1 -> [[buffer(1)]].
// Struct layout must match FrameUniforms/DrawUniforms in src/render/Renderer.h.

#include <metal_stdlib>
using namespace metal;

struct FrameUniforms {
    float4x4 viewProj;
    float4 lightDir;    // xyz: direction light TRAVELS (away from the sun), world space
    float4 lightColor;  // rgb
    float4 skyColor;    // rgb: hemispherical ambient from above
    float4 groundColor; // rgb: ambient from below
    float4 cameraPos;   // xyz
    float4 fogParams;   // x: fog density, yzw: color
};

struct DrawUniforms {
    float4x4 model;  // rigid transforms only (no non-uniform scale)
    float4 tint;     // rgb color multiplier, a: unused
    float4 params;   // x: 1 m grid strength (for ground), yzw: reserved
};

// ---------------------------------------------------------------- lit

struct LitIn {
    float3 position [[attribute(0)]];
    float3 normal [[attribute(1)]];
    float3 color [[attribute(2)]];
};

struct LitOut {
    float4 position [[position]];
    float3 worldPos;
    float3 normal;
    float3 color;
};

vertex LitOut lit_vs(LitIn in [[stage_in]],
                     constant FrameUniforms& frame [[buffer(0)]],
                     constant DrawUniforms& draw [[buffer(1)]]) {
    LitOut out;
    const float4 world = draw.model * float4(in.position, 1.0);
    out.position = frame.viewProj * world;
    out.worldPos = world.xyz;
    out.normal = (draw.model * float4(in.normal, 0.0)).xyz;
    out.color = in.color * draw.tint.rgb;
    return out;
}

fragment float4 lit_fs(LitOut in [[stage_in]],
                       constant FrameUniforms& frame [[buffer(0)]],
                       constant DrawUniforms& draw [[buffer(1)]]) {
    const float3 n = normalize(in.normal);
    const float3 l = normalize(-frame.lightDir.xyz);
    const float diffuse = max(dot(n, l), 0.0);
    const float3 ambient = mix(frame.groundColor.rgb, frame.skyColor.rgb, n.y * 0.5 + 0.5);

    float3 albedo = in.color;
    if (draw.params.x > 0.0) {
        // A 1 m grid helps read speed and terrain.
        const float2 cell = abs(fract(in.worldPos.xz) - 0.5);
        const float line = smoothstep(0.47, 0.5, max(cell.x, cell.y));
        albedo *= 1.0 - draw.params.x * line;
    }

    float3 color = albedo * (ambient + frame.lightColor.rgb * diffuse);
    const float dist = length(in.worldPos - frame.cameraPos.xyz);
    const float fog = 1.0 - exp(-frame.fogParams.x * dist);
    color = mix(color, frame.fogParams.yzw, saturate(fog));
    // SDR swapchain (UNORM, not sRGB): manual gamma.
    return float4(pow(saturate(color), float3(1.0 / 2.2)), 1.0);
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

fragment float4 line_fs(LineOut in [[stage_in]]) {
    return in.color;
}
