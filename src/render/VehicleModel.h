#pragma once

#include "render/Mesh.h"

#include <glm/glm.hpp>

namespace tf {

// Visual-only vehicle model in body space (+Z forward, +X left, +Y up). The collision shape stays the
// three boxes from vehicle/VehicleShape.h; the visual body fits around them (rack, mirrors, snorkel and
// bed rails stick out by a few centimeters).
MeshData buildVehicleBodyMesh();
// Wheel with lug tread, sidewall, beadlock rim and brake disc. Axle along local X, outer face at +X.
MeshData buildWheelMesh(float radius, float width);

// Suspension parts, drawn per wheel with transforms computed from the Jolt wheel position each frame.
// "Unit" meshes span local z = 0..1 and are stretched between two points.
MeshData buildUnitSpringMesh();             // coil spring
MeshData buildUnitTubeMesh(float radius);   // arms, half shafts
MeshData buildShockBodyMesh();              // fixed length kShockBodyLength along +Z
MeshData buildShockShaftMesh();             // fixed length kShockShaftLength along +Z

inline constexpr float kShockBodyLength = 0.32f;  // drawn from the lower mount upward
inline constexpr float kShockShaftLength = 0.46f; // drawn from the upper mount downward
inline constexpr float kSpringPerch = 0.105f;     // lower spring seat above the lower mount

// Body-space suspension mount points for a wheel at body-space hub position `hub`.
struct SuspensionPose {
    glm::vec3 shockTop;   // on the body, under the wheel arch
    glm::vec3 shockLow;   // on the lower arm
    glm::vec3 armPivotA;  // frame pivots of the lower A-arm
    glm::vec3 armPivotB;
    glm::vec3 armOuter;   // ball joint near the hub
    glm::vec3 shaftInner; // differential output
    glm::vec3 shaftOuter; // hub side
};
SuspensionPose suspensionPose(const glm::vec3& hubBody, float attachZ);

// Matrix that maps local z = 0..1 of a unit mesh onto the segment a..b (non-uniform scale along z).
glm::mat4 segmentMatrix(const glm::vec3& a, const glm::vec3& b);
// Rigid matrix with +Z pointing from a toward b, origin at a.
glm::mat4 aimMatrix(const glm::vec3& a, const glm::vec3& b);

// Spare wheel on the roof rack (body space; wheel axle along body Y).
glm::mat4 spareWheelMatrix(float radius, float width);

} // namespace tf
