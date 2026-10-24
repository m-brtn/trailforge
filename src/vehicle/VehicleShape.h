#pragma once

// Body boxes in body space (m). Used both for the collision shape
// (Vehicle.cpp) and for the visual mesh (render/Mesh.cpp), so they match.

namespace tf {

struct BodyBox {
    float center[3];
    float halfExtents[3];
    float color[3];
};

inline constexpr BodyBox kBodyBoxes[] = {
    // Lower "tub"/frame: narrower than the track, wheels outside.
    {{0.0f, 0.0f, 0.0f}, {0.62f, 0.30f, 2.30f}, {0.18f, 0.18f, 0.19f}},
    // Fenders/sides above the wheels.
    {{0.0f, 0.45f, 0.0f}, {0.95f, 0.15f, 2.30f}, {0.72f, 0.42f, 0.12f}},
    // Cab.
    {{0.0f, 0.95f, -0.35f}, {0.85f, 0.35f, 1.15f}, {0.78f, 0.48f, 0.15f}},
};

inline constexpr float kBodyConvexRadius = 0.05f; // m, Jolt box rounding

} // namespace tf
