#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/ContactListener.h>

#include <mutex>
#include <vector>

namespace tf {

// A new contact of the vehicle body (not the wheels: they are shape casts) during the last tick.
struct BodyImpact {
    JPH::Vec3 position = JPH::Vec3::sZero(); // world, mean of the manifold contact points
    float approachSpeed = 0.0f;              // m/s, relative normal velocity toward the other body at first contact
    // N*s: impulse that stops that approach for the vehicle's effective mass at the contact point
    // (perfectly inelastic estimate; Jolt's listener runs before the solver and reports no impulses).
    float impulse = 0.0f;
};

// Observes contacts only; it never changes contact settings, so the physics are unaffected.
// Jolt calls it from its worker threads, hence the mutex.
class BodyImpactListener final : public JPH::ContactListener {
public:
    void setBody(JPH::BodyID id) { m_body = id; }
    void beginTick();
    // Impacts recorded during the last tick (read between ticks).
    const std::vector<BodyImpact>& impacts() const { return m_impacts; }

    void OnContactAdded(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold& manifold,
                        JPH::ContactSettings& settings) override;

private:
    JPH::BodyID m_body;
    std::mutex m_mutex;
    std::vector<BodyImpact> m_impacts;
};

} // namespace tf
