#pragma once

#include "track/TestTrack.h"

#include <Jolt/Jolt.h>

#include <functional>
#include <optional>

namespace tf {

// Water of the current level: the surface height over (x, z), or nullopt where there is no water. The surface is
// flat at each sample (no waves). It drives the water forces on the vehicle (sim/WaterForces.h), the visuals and
// sounds (wet tread, splashes) and, on open-world levels, the deep-water recovery.
using WaterSurface = std::function<std::optional<float>(float x, float z)>;

// The proving ground puddles: one common level inside the TestTrack::water() rectangle.
inline WaterSurface provingGroundWater() {
    return [](float x, float z) -> std::optional<float> {
        const WaterInfo w = TestTrack::water();
        if (x > w.x0 && x < w.x1 && z > w.z0 && z < w.z1) return w.level;
        return std::nullopt;
    };
}

// A contact point counts as in the water when it is more than `margin` below the surface.
inline bool underWater(const WaterSurface& water, JPH::Vec3Arg p, float margin = 0.03f) {
    if (!water) return false;
    const std::optional<float> s = water(p.GetX(), p.GetZ());
    return s && p.GetY() < *s - margin;
}

} // namespace tf
