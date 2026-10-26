#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>

#include <array>
#include <cstdint>
#include <string_view>

namespace tf {

// New surfaces are appended: the numeric ids are stored in tire marks, audio frames and vehicle files.
enum class SurfaceId : std::uint8_t {
    Concrete = 0, // start pad and steps
    PackedDirt,   // main hard ground of the proving ground
    Rock,         // rocks
    LowGrip,      // low-grip section (hard, only lower mu)
    Mud,          // showcase mud path: rigid, only lower mu (no sinkage; visuals are separate)
    Grass,        // showcase meadow: rigid, only mu
    Sand,         // sand area: rigid, only a lower mu (no sinkage, no ruts that change the ground)
    Snow,         // level snowfields: rigid, only a low mu (no sinkage, no compaction, no melting)
    Asphalt,      // dense dry asphalt road (mini handling track, circuits)
    Gravel,       // loose gravel: rigid, lower and flatter mu, extra rolling resistance (no sinkage, no displacement)
    Count
};

inline constexpr std::size_t kSurfaceCount = static_cast<std::size_t>(SurfaceId::Count);

// Surface properties that affect the tire. The ground is hard: the surface scales the tire grip curves, may stretch
// and flatten them (loose surfaces), and may add rolling resistance. No sinkage, bearing capacity or soil state
// (deformable terrain is a later stage).
struct SurfaceProperties {
    const char* name = "";
    float longitudinalFriction = 1.0f; // multiplier on the tire longitudinal grip curve, dimensionless
    float lateralFriction = 1.0f;      // multiplier on the tire lateral grip curve, dimensionless
    float color[3] = {0.5f, 0.5f, 0.5f}; // render color (linear RGB)
    // Loose surfaces: the tire curves peak at peakSlipScale x the tire's peak slip ratio / slip angle, and beyond the
    // peak only slideDrop x the tire's peak-to-slide drop remains (material piles up ahead of a locked or sliding
    // tire). 1 / 1 = the tire curve as measured on a paved surface.
    float peakSlipScale = 1.0f;
    float slideDrop = 1.0f;
    // Rolling resistance coefficient on top of a paved road (force = coefficient x wheel load, against the patch's
    // rolling direction). Paved-road rolling resistance itself is not modeled, so hard surfaces have 0.
    float extraRollingResistance = 0.0f;
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

// Data-file key of a surface ("concrete", "packed_dirt", ..., "asphalt", "gravel"): vehicle.json tires.surfaceGrip,
// level.json road / flat "surface" (which also accepts "dirt" for packed_dirt).
const char* surfaceKey(SurfaceId id);
// SurfaceId::Count for an unknown key.
SurfaceId surfaceFromKey(std::string_view key);

} // namespace tf
