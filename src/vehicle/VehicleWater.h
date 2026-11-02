#pragma once

namespace tf {

// Hydrodynamic coefficients of a vehicle in water (vehicle.json "water", all optional; sim/WaterForces.h applies
// them). The body is the collision shape (boxes or the bounding boxes of its convex pieces), the wheels are cylinders
// of the physics wheel size. Defaults are textbook values for bluff bodies and tires, not measurements of these
// vehicles.
struct VehicleWater {
    float density = 1000.0f; // kg/m^3, fresh water
    // Pressure drag ½ ρ Cd v_n² per wetted surface patch facing into the flow (v_n: the relative velocity along the
    // patch normal). frontalCd on the faces facing forward / backward, sideCd on the sides, heaveCd on the bottom
    // and top. A square-edged box at high Reynolds number is ~1.05-1.2 (Hoerner, Fluid-Dynamic Drag, ch. 3).
    float frontalCd = 1.0f;
    float sideCd = 1.1f;
    float heaveCd = 1.2f;
    // Skin friction on every wetted patch, ½ ρ Cf |v_t| v_t (v_t: the velocity along the surface). A rough plate is
    // 0.003-0.008; the underbody is rougher than a hull.
    float skinCf = 0.005f;
    // The collision boxes have vertical ends; a real bumper chin and the underbody behind it slope up at the approach
    // (and departure) angle. The pressure on such a slope is normal to it, so a front or rear face pushing into the
    // water also gets an upward force of tan(slope) x its horizontal pressure force: the bow lift. 0 = vertical ends.
    float frontSlopeDeg = 35.0f;
    // Fraction of the collision volume that displaces water (buoyancy ρ g V). Doors, floor plugs and the engine bay
    // are not sealed and the cabin is glass and trim; the value is an estimate, and flooding over time is not
    // simulated. The tires (sealed air) always displace their full cylinder.
    float displacement = 0.7f;
    // Tire displacement drag ½ ρ Cd v² (width x submerged height) along the rolling direction: the contaminated-runway
    // model for aircraft tires in standing water (ESDU 83042: Cd ~0.75). Sideways the submerged disc segment gets
    // sideCd.
    float wheelCd = 0.75f;

    bool operator==(const VehicleWater&) const = default;
};

} // namespace tf
