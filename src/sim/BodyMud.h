#pragma once

#include "sim/TireMarks.h"
#include "vehicle/WheelContact.h"

#include <array>
#include <span>

namespace tf {

// Cosmetic mud on the body, per region, driven by the material each wheel throws (wheelThrowRate: slip-driven, the
// same model as the spray particles but independent of the particle cap). A wheel throwing forward-rolling spray
// coats the rear part of its own arch and the inner liner first, then the panel behind it (sills and lower rear
// doors behind a front wheel, the rear quarter behind a rear wheel) and, for the rear wheels, the rear bumper;
// reversing mirrors that. The coat builds toward a soil-dependent maximum (wet mud up to 1, dirt and grass ~0.35, dry
// sand ~0.12) at a rate set by the soil's adhesion and wetness; it dries slowly, falls off very slowly, and a puddle
// washes the regions of the wheels standing in it. Visual only: no mass, no drag, no effect on the simulation.
enum BodyMudRegion : int {
    kMudArchRear = 0,   // + wheel index (0 FL, 1 FR, 2 RL, 3 RR): rear part of the arch, inner liner
    kMudArchFront = 4,  // + wheel index: front part of the arch (reversing)
    kMudBehind = 8,     // + wheel index: panel behind the wheel (front: sill, lower rear door; rear: rear quarter)
    kMudRearBumper = 12,
    kMudFrontBumper = 13,
    kBodyMudRegions = 14,
};

struct BodyMudCoat {
    float amount = 0.0f;  // 0 clean .. 1 caked
    float wetness = 0.0f; // 0 dry (lighter, matte) .. 1 wet (dark, glossy)
    float color[3] = {0.045f, 0.031f, 0.018f};
    float flecks = 0.0f;  // grass blades stuck in it
};

class BodyMud {
public:
    // One step from the wheel contact samples and the soil each tread carries (thrown off on hard ground).
    void update(std::span<const WheelContactSample> wheels, std::span<const TreadSoil> soil, float dt);
    void clear() { m_coat = {}; }
    void setAll(float amount); // wet dark mud on every region (scenario / UI)

    const BodyMudCoat& coat(int region) const { return m_coat[std::size_t(region)]; }
    const std::array<BodyMudCoat, kBodyMudRegions>& coats() const { return m_coat; }
    float maxAmount() const;

private:
    void deposit(int region, float exposure, const SoilType& soil, float target);
    std::array<BodyMudCoat, kBodyMudRegions> m_coat{};
};

} // namespace tf
