#pragma once

#include <span>
#include <string>

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

// Tube-frame patrol buggy (visualModel "trailforge_patrol"): the kit body of render/VehicleModel is built around
// these. Floor tub between the axles, the cage up to the roof tubes, the nose and the engine bay between the wheels,
// the cargo bed above the rear wheels.
inline constexpr BodyBox kPatrolBoxes[] = {
    {{0.0f, -0.18f, -0.08f}, {0.60f, 0.11f, 0.70f}, {0.10f, 0.10f, 0.10f}}, // floor tub, sills
    {{0.0f, 0.36f, -0.10f}, {0.60f, 0.47f, 0.36f}, {0.70f, 0.58f, 0.40f}},  // cage
    {{0.0f, 0.02f, 1.05f}, {0.34f, 0.24f, 0.42f}, {0.10f, 0.10f, 0.10f}},   // nose, front guard
    {{0.0f, -0.10f, -1.00f}, {0.40f, 0.17f, 0.47f}, {0.10f, 0.10f, 0.10f}}, // engine bay
    {{0.0f, 0.23f, -0.98f}, {0.68f, 0.13f, 0.50f}, {0.70f, 0.58f, 0.40f}},  // cargo bed
};

// Boxes of the kit body of a visual model.
inline std::span<const BodyBox> kitBodyBoxes(const std::string& visualModel) {
    if (visualModel == "trailforge_patrol") return kPatrolBoxes;
    return kBodyBoxes;
}

inline constexpr float kBodyConvexRadius = 0.05f; // m, Jolt box rounding

} // namespace tf
