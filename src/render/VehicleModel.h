#pragma once

#include "render/Mesh.h"
#include "render/StyleKit.h"
#include "vehicle/VehicleConfig.h"

#include <glm/glm.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace tf {

// Named part of a vehicle model. Parts are built separately and merged into one mesh (one draw); the flags
// mark what later damage stages may bend or knock off (there is no damage logic yet).
struct VehiclePart {
    std::string name;
    std::string parent;      // part it is bolted / hinged to ("" = the body root)
    glm::vec3 attach{0.0f};  // attachment point in body space (hinge line center, bolt-on point)
    bool deformable = false;
    bool detachable = false;
    std::size_t firstVertex = 0; // vertex range in VehicleVisual::mesh
    std::size_t vertexCount = 0;
};

// Spare wheel slot, drawn with the rear wheel mesh: the wheel's outer face points along `outward`.
struct SpareMount {
    std::string name;
    std::string parent;
    glm::vec3 seat{0.0f};          // body-space point the inner face of the tire rests on
    glm::vec3 outward{0, 1, 0};
    bool detachable = true;
};

// Coil-over and lower-arm mounts of one axle where a model's frame carries them (body space, x as the distance from
// the centre line, mirrored per side; z relative to the hub). Unset = the procedural layout under the SUV bodies.
struct SuspensionMounts {
    bool set = false;
    float shockTopX = 0.585f, shockTopY = 0.29f, shockTopDZ = 0.0f; // upper eye on the frame (shock tower)
    float shockLowX = 0.585f;                                          // lower eye on the lower arm
    float armPivotX = 0.22f, armPivotY = -0.27f, armPivotHalfZ = 0.24f; // the two frame pivots of the lower arm
    float armOuterInset = 0.11f, armOuterDrop = 0.11f;                  // ball joint: inboard of / below the hub
};

// Visual-only vehicle model in body space (+Z forward, +X left, +Y up). The kit body (bodyShape "kit", the default) is
// lofted from StyleKit panels around the collision boxes of vehicle/VehicleShape.h; the traced body (bodyShape
// "traced") is meshed from the blueprint (BlueprintBody), the same data its collision hulls come from; see README
// "Vehicle rendering" and "Vehicle blueprints".
struct VehicleVisual {
    MeshData mesh;                 // all parts, in `parts` order
    std::vector<VehiclePart> parts;
    std::vector<SpareMount> spares;
    style::WheelStyle wheel;       // rim / tire materials and spokes; size and tread come from the axle
    glm::vec3 headlightLens{0.67f, 0.40f, 2.24f}; // left low-beam lens center (body space; right is mirrored)
    glm::vec4 mudArch{1.40f, -1.40f, -0.20f, 0.545f}; // BodyMud shading: axle z front / rear, arch centre y, radius
    glm::vec4 mudBody{0.88f, 2.15f, -2.10f, 0.0f};    // BodyMud shading: body side |x|, bumper zones from z front / rear
    Material coil{MatId::Generic, {0.16f, 0.025f, 0.012f}, 0.40f, 0.1f, 0.8f}; // coil spring colour
    SuspensionMounts mounts[2];    // front, rear
    const VehiclePart* find(const std::string& name) const;
};

// Builds `config.visualModel` (known: "trailforge_suv", gasoline expedition SUV, Cars.png car #1;
// "trailforge_ev", dual-motor electric SUV, Cars.png car #2; "trailforge_patrol", tube-frame patrol buggy). Arches and
// flares are fitted to the physics wheels (radius, width, track, travel, steering), so rebuild on a wheel change.
VehicleVisual buildVehicleVisual(const VehicleConfig& config);
// Wheel style of an axle: the model's materials with the axle's visual rim ratio and tread.
// The tread follows the fitted tire set; the off-road set shows the axle's own tread (vehicle.json).
style::WheelStyle axleWheelStyle(const VehicleVisual& visual, const AxleConfig& axle, TireSet set = TireSet::Offroad);
// Wheel mesh of an axle: outer radius == wheelRadius, width == wheelWidth (the physics wheel).
MeshData buildAxleWheelMesh(const VehicleVisual& visual, const AxleConfig& axle, TireSet set = TireSet::Offroad);
// Measured size of a wheel mesh (axle along local X): (max distance from the axle, axial extent).
glm::vec2 wheelMeshSize(const MeshData& wheel);
// True when two configs give the same body fit and wheel meshes (no rebuild needed).
bool sameWheelFit(const VehicleConfig& a, const VehicleConfig& b);
// Body-space matrix of a spare wheel (wheel mesh: axle along local X, outer face at +X).
glm::mat4 spareWheelMatrix(const SpareMount& mount, float radius, float width);

// Suspension parts, drawn per wheel with transforms computed from the Jolt wheel position each frame.
// "Unit" meshes span local z = 0..1 and are stretched between two points.
MeshData buildUnitSpringMesh(const Material& coil); // coil spring (VehicleVisual::coil)
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
SuspensionPose suspensionPose(const glm::vec3& hubBody, float attachZ, const SuspensionMounts& mounts = {});

// Matrix that maps local z = 0..1 of a unit mesh onto the segment a..b (non-uniform scale along z).
glm::mat4 segmentMatrix(const glm::vec3& a, const glm::vec3& b);
// Rigid matrix with +Z pointing from a toward b, origin at a.
glm::mat4 aimMatrix(const glm::vec3& a, const glm::vec3& b);

} // namespace tf
