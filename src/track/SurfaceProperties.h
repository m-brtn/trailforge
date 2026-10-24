#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>

#include <array>
#include <cstdint>

namespace tf {

enum class SurfaceId : std::uint8_t {
    Concrete = 0, // start pad and steps
    PackedDirt,   // main hard ground of the proving ground
    Rock,         // rocks
    LowGrip,      // low-grip section (hard, only lower mu)
    Count
};

inline constexpr std::size_t kSurfaceCount = static_cast<std::size_t>(SurfaceId::Count);

// Surface properties that affect the tire. In the MVP the ground is hard: the surface
// only changes friction coefficients, which multiply the tire grip curve.
//
// Placeholder for the next stage (deformable terrain): sinkage parameters
// (stiffness/bearing capacity limit), rolling resistance and soil state
// will be added here; they will be applied in Vehicle::combineFriction and
// in the future tire contact model. These fields are intentionally absent for now.
struct SurfaceProperties {
    const char* name = "";
    float longitudinalFriction = 1.0f; // multiplier on the tire longitudinal grip curve, dimensionless
    float lateralFriction = 1.0f;      // multiplier on the tire lateral grip curve, dimensionless
    float color[3] = {0.5f, 0.5f, 0.5f}; // render color (linear RGB)
};

// Surface table + matching Jolt PhysicsMaterials.
// Material at a contact point: Shape::GetMaterial(SubShapeID) -> PhysicsMaterial* -> SurfaceId.
class SurfaceTable {
public:
    SurfaceTable();

    SurfaceProperties& get(SurfaceId id) { return m_props[static_cast<std::size_t>(id)]; }
    const SurfaceProperties& get(SurfaceId id) const { return m_props[static_cast<std::size_t>(id)]; }

    const JPH::PhysicsMaterial* material(SurfaceId id) const { return m_materials[static_cast<std::size_t>(id)]; }

    // Returns the SurfaceId for a contact material; unknown material -> PackedDirt.
    SurfaceId lookup(const JPH::PhysicsMaterial* material) const;

private:
    std::array<SurfaceProperties, kSurfaceCount> m_props;
    std::array<JPH::RefConst<JPH::PhysicsMaterial>, kSurfaceCount> m_materials;
};

} // namespace tf
