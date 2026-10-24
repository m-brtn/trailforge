#pragma once

#include "track/SurfaceProperties.h"

#include <Jolt/Jolt.h>

#include <cmath>

namespace tf {

// What one wheel does to the ground during the last physics tick, in ground terms. Filled by Vehicle::postPhysics
// from the Jolt wheel state; read-only consumers (tire marks today, a deformable terrain later) use only this.
// The ground is static, so "relative to the ground" is the body point velocity at the patch.
struct WheelContactSample {
    bool contact = false;
    SurfaceId surface = SurfaceId::PackedDirt;
    JPH::Vec3 patchCenter = JPH::Vec3::sZero();  // wheel center projected onto the contact plane, world, m
    JPH::Vec3 normal = JPH::Vec3::sAxisY();       // contact normal (from the ground toward the wheel)
    JPH::Vec3 longitudinal = JPH::Vec3::sAxisZ(); // wheel heading in the contact plane, unit
    JPH::Vec3 lateral = JPH::Vec3::sAxisX();      // in the contact plane, unit
    JPH::Vec3 velocity = JPH::Vec3::sZero();      // patch velocity over the ground, in the contact plane, m/s
    float load = 0.0f;                  // N, suspension force along the normal
    float longitudinalForce = 0.0f;     // N, tire force along `longitudinal`
    float lateralForce = 0.0f;          // N, tire force along `lateral`
    float rollingSpeed = 0.0f;          // m/s, wheel angular velocity * radius (+ = rolling forward)
    float longitudinalSlipSpeed = 0.0f; // m/s, rollingSpeed - velocity.longitudinal (+ wheelspin, - lock-up)
    float lateralSlipSpeed = 0.0f;      // m/s, velocity.lateral (sideways sliding of the patch)
    float width = 0.0f;                 // m, tire width
    float radius = 0.0f;                // m

    // m/s, magnitude of the patch sliding velocity
    float slipSpeed() const {
        return std::sqrt(longitudinalSlipSpeed * longitudinalSlipSpeed + lateralSlipSpeed * lateralSlipSpeed);
    }
};

} // namespace tf
