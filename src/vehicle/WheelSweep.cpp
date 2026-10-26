#include "vehicle/WheelSweep.h"

#include "vehicle/VehicleConfig.h"

#include <algorithm>
#include <cmath>

namespace tf {

float WheelSweep::clearance(float px, float py, float pz) const {
    const float hub = std::clamp(py, hubLow, hubHigh);
    const float dy = py - hub;
    const float dx = px - x, dz = pz - z;
    const int steps = steer > 0.0f ? 15 : 1;
    float best = 1e9f;
    for (int s = 0; s < steps; ++s) {
        const float a = steps > 1 ? -steer + 2.0f * steer * float(s) / float(steps - 1) : 0.0f;
        const float axial = std::abs(dx * std::cos(a) - dz * std::sin(a)) - halfWidth;
        const float along = dx * std::sin(a) + dz * std::cos(a);
        const float radial = std::hypot(along, dy) - radius;
        best = std::min(best, std::max(axial, radial));
    }
    return best;
}

std::array<WheelSweep, 4> wheelSweeps(const VehicleConfig& c) {
    std::array<WheelSweep, 4> out{};
    for (int w = 0; w < 4; ++w) {
        const AxleConfig& ax = c.axles[std::size_t(w / 2)];
        WheelSweep& s = out[std::size_t(w)];
        s.x = (w % 2 == 0 ? 0.5f : -0.5f) * ax.track;
        s.z = (w < 2 ? 0.5f : -0.5f) * c.wheelBase;
        s.halfWidth = 0.5f * ax.wheelWidth;
        s.radius = ax.wheelRadius;
        s.hubLow = c.attachHeight - ax.suspensionMaxLength;
        s.hubHigh = c.attachHeight - ax.suspensionMinLength;
        s.steer = ax.maxSteerAngleDeg * 3.14159265f / 180.0f;
    }
    return out;
}

float wheelSweepClearance(const std::array<WheelSweep, 4>& sweeps, float x, float y, float z, int* wheel) {
    float best = 1e9f;
    for (int w = 0; w < 4; ++w) {
        const float c = sweeps[std::size_t(w)].clearance(x, y, z);
        if (c < best) {
            best = c;
            if (wheel) *wheel = w;
        }
    }
    return best;
}

} // namespace tf
