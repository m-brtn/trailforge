#include "sim/BodyImpacts.h"

#include <Jolt/Physics/Body/Body.h>

namespace tf {

void BodyImpactListener::beginTick() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_impacts.clear();
}

void BodyImpactListener::OnContactAdded(const JPH::Body& body1, const JPH::Body& body2,
                                        const JPH::ContactManifold& manifold, JPH::ContactSettings&) {
    using namespace JPH;
    const bool first = body1.GetID() == m_body;
    if (!first && body2.GetID() != m_body) return;
    if (manifold.mRelativeContactPointsOn1.empty()) return;
    const Body& vehicle = first ? body1 : body2;
    const Body& other = first ? body2 : body1;

    Vec3 sum = Vec3::sZero();
    for (uint i = 0; i < manifold.mRelativeContactPointsOn1.size(); ++i) sum += manifold.mRelativeContactPointsOn1[i];
    const RVec3 point = manifold.mBaseOffset + sum / float(manifold.mRelativeContactPointsOn1.size());
    // The normal points from body 1 toward body 2; turn it into "from the vehicle toward the other body".
    const Vec3 n = first ? manifold.mWorldSpaceNormal : -manifold.mWorldSpaceNormal;
    const Vec3 vRel = vehicle.GetPointVelocity(point) - (other.IsStatic() ? Vec3::sZero() : other.GetPointVelocity(point));
    const float approach = vRel.Dot(n);
    if (approach <= 0.0f || !vehicle.IsDynamic()) return;

    // Effective mass of the vehicle along n at the contact point: 1 / (1/m + n . ((I^-1 (r x n)) x r)).
    const Vec3 r = Vec3(point - vehicle.GetCenterOfMassPosition());
    const Vec3 rxn = r.Cross(n);
    const Mat44 invI = vehicle.GetInverseInertia();
    const float invMass = vehicle.GetMotionPropertiesUnchecked()->GetInverseMass() + rxn.Dot(invI.Multiply3x3(rxn));
    if (invMass <= 0.0f) return;

    BodyImpact impact;
    impact.position = Vec3(point);
    impact.approachSpeed = approach;
    impact.impulse = approach / invMass;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_impacts.push_back(impact);
}

} // namespace tf
