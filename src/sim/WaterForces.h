#pragma once

#include "sim/WaterQuery.h"
#include "vehicle/VehicleWater.h"

#include <Jolt/Jolt.h>
#include <Jolt/Geometry/AABox.h>

#include <functional>
#include <span>
#include <vector>

namespace JPH {
class Shape;
}

namespace tf {

// Hydrodynamic forces on a vehicle in water: explicit forces on the chassis each physics tick, computed from the
// water surface height at each sample (WaterSurface) and the body's motion relative to the water.
//
// - Buoyancy: the body volume is split into cells of about `cell` m; each cell's submerged fraction (from the depth at
//   its center and its vertical extent in the current pose) displaces ρ g V x VehicleWater::displacement, applied at
//   the submerged part. The tires are cylinders and displace their submerged segment fully.
// - Pressure drag: the outer surface of the body (faces not inside another box) is split into patches; a wetted
//   patch facing into the relative flow gets ½ ρ Cd v_n² x wetted area against its normal, plus skin friction along
//   it. Yaw and roll moments come only from where these forces act (one side deeper, entering at an angle, flow).
// - Bow lift: the front and rear faces also get tan(frontSlopeDeg) x their pressure force upward in body axes (the
//   sloped bumper chin and underbody, which the boxes do not have).
// - Tires: displacement drag ½ ρ wheelCd v² (width x submerged height) along the rolling direction and side drag on
//   the submerged disc segment, at the center of the submerged part of the wheel (Jolt's wheels are casts, so the
//   force acts on the chassis there).
// Not modelled: waves, the bow wave's run-up and spray, added mass, flooding, water in the intake, hydroplaning of
// the tread (the tire grip is the ground's), and the river current unless a flow field is given.
//
// No SDL; used by Simulation and testable on its own.

// Axis-aligned box in body space (the frame of the body's world transform, Vehicle::bodyTransform).
struct WaterBox {
    JPH::Vec3 center = JPH::Vec3::sZero();
    JPH::Vec3 halfExtents = JPH::Vec3::sZero();
};

// Horizontal water velocity at (x, z), m/s (world); empty = still water. Hook for river flow from the level bake.
using WaterFlow = std::function<JPH::Vec3(float x, float z)>;

// A physics wheel in world space: hub center, unit axle direction, outer radius and width (m).
struct WaterWheel {
    JPH::Vec3 hub = JPH::Vec3::sZero();
    JPH::Vec3 axle = JPH::Vec3::sAxisX();
    float radius = 0.4f;
    float width = 0.3f;
};

struct WaterBodyState {
    JPH::RMat44 bodyToWorld = JPH::RMat44::sIdentity();
    JPH::Vec3 centerOfMass = JPH::Vec3::sZero(); // world
    JPH::Vec3 linearVelocity = JPH::Vec3::sZero();
    JPH::Vec3 angularVelocity = JPH::Vec3::sZero();
};

struct WaterForceResult {
    bool wet = false;                           // any body cell, patch or wheel below the surface
    JPH::Vec3 force = JPH::Vec3::sZero();       // N, world, total
    JPH::Vec3 torque = JPH::Vec3::sZero();      // N m, world, about the center of mass
    float buoyancy = 0.0f;                      // N, upward, body + tires
    float bowLift = 0.0f;                       // N, upward part of the front / rear face pressure (body up axis)
    float bodyDrag = 0.0f;                      // N, body pressure + skin force against the horizontal velocity
    float wheelDrag = 0.0f;                     // N, tire force against the horizontal velocity
    float displacedVolume = 0.0f;               // m^3 (after the displacement fraction)
    float bodyDepth = 0.0f;                     // m, water surface above the lowest point of the body (0 = dry)
    float wheelDepth = 0.0f;                    // m, deepest submerged tire height
    float wetArea = 0.0f;                       // m^2, wetted body surface
};

class WaterForces {
public:
    // Body boxes (body space); cell = target size of the volume cells and surface patches, m.
    void setBody(std::span<const WaterBox> boxes, const VehicleWater& params, float cell = 0.2f);
    WaterForceResult compute(const WaterBodyState& state, std::span<const WaterWheel> wheels, const WaterSurface& water,
                             const WaterFlow& flow = {}) const;

    const VehicleWater& params() const { return m_params; }
    std::size_t cellCount() const { return m_cells.size(); }
    std::size_t patchCount() const { return m_patches.size(); }
    float bodyVolume() const { return m_volume; } // m^3, union of the boxes (cell resolution)
    float surfaceArea() const { return m_area; }  // m^2, outer surface (patch resolution)

private:
    enum class Face : unsigned char { End, Side, Heave };
    struct Cell {
        JPH::Vec3 center, half;
        float volume;
    };
    struct Patch {
        JPH::Vec3 center, normal, t1, t2; // body space; t1 / t2 in-plane unit axes
        float h1, h2, area;               // half sizes along t1 / t2, m^2
        Face face;
    };
    VehicleWater m_params;
    std::vector<Cell> m_cells;
    std::vector<Patch> m_patches;
    JPH::AABox m_bounds;
    float m_volume = 0.0f, m_area = 0.0f;
};

// Body-space boxes of a body shape: an OffsetCenterOfMassShape is unwrapped, a compound gives one box per sub-shape
// (a convex hull by its bounding box), anything else its bounding box.
std::vector<WaterBox> waterBoxesFromShape(const JPH::Shape& shape);

} // namespace tf
