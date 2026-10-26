#pragma once

#include <array>

// The space a physics wheel can occupy in body space: the tire cylinder (outer radius and width of the physics
// wheel) moved over the whole suspension travel (hub y from attachHeight - maxLength to attachHeight - minLength)
// and turned over the steering range about the vertical through the hub. Body geometry must stay outside it: the
// procedural bodies are fitted around it, Blender-authored bodies (vehicle/VehicleBody.h) are validated against it.
// No SDL, no rendering.

namespace tf {

struct VehicleConfig;

struct WheelSweep {
    float x = 0.0f, z = 0.0f;            // hub position in the body (m)
    float halfWidth = 0.0f, radius = 0.0f;
    float hubLow = 0.0f, hubHigh = 0.0f; // hub y over the suspension travel (full droop, full compression)
    float steer = 0.0f;                  // steering lock (rad, both directions)

    // Signed clearance (m) of a body-space point: negative inside the tire at some hub height and steering angle.
    // Outside, the larger of the axial and the radial gap, minimized over the steering range.
    float clearance(float px, float py, float pz) const;
};

std::array<WheelSweep, 4> wheelSweeps(const VehicleConfig& c); // wheel order FL, FR, RL, RR

// Smallest clearance of a point to the four wheels; `wheel` receives the closest one.
float wheelSweepClearance(const std::array<WheelSweep, 4>& sweeps, float x, float y, float z, int* wheel = nullptr);

} // namespace tf
