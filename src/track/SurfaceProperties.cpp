#include "track/SurfaceProperties.h"

#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>

namespace tf {

SurfaceTable::SurfaceTable() {
    // Multipliers on the tire curve (peak 1.00 long / 1.00 lat), i.e. the effective peak mu on each surface:
    // asphalt 1.00, dry concrete 0.95, rock 0.75, dry packed dirt 0.70, gravel 0.62, sand 0.55, grass 0.50, mud 0.35,
    // snow 0.32; the low-grip patch keeps its original 0.42 / 0.40. All surfaces are rigid: mud, grass, sand, snow and
    // gravel have no sinkage or rutting; only gravel has extra rolling resistance and a stretched, flatter curve.
    m_props[size_t(SurfaceId::Concrete)] = {"Concrete", 0.95f, 0.95f, {0.55f, 0.55f, 0.52f}};
    m_props[size_t(SurfaceId::PackedDirt)] = {"Packed dirt", 0.70f, 0.68f, {0.42f, 0.34f, 0.22f}};
    m_props[size_t(SurfaceId::Rock)] = {"Rock", 0.75f, 0.75f, {0.38f, 0.37f, 0.36f}};
    m_props[size_t(SurfaceId::LowGrip)] = {"Low grip (hard)", 0.42f, 0.40f, {0.20f, 0.35f, 0.55f}};
    m_props[size_t(SurfaceId::Mud)] = {"Mud look (rigid, low mu)", 0.35f, 0.32f, {0.16f, 0.11f, 0.07f}};
    m_props[size_t(SurfaceId::Grass)] = {"Grass (rigid)", 0.50f, 0.48f, {0.20f, 0.26f, 0.09f}};
    m_props[size_t(SurfaceId::Sand)] = {"Sand (rigid, low mu)", 0.55f, 0.52f, {0.62f, 0.52f, 0.36f}};
    m_props[size_t(SurfaceId::Snow)] = {"Snow (rigid, low mu)", 0.32f, 0.30f, {0.82f, 0.85f, 0.90f}};
    // Dry dense asphalt: peak mu 1.0-1.1 for road tires (Bosch Automotive Handbook; Wong, Theory of Ground Vehicles,
    // Table 1.3 gives 0.8-0.9 for asphalt and concrete with older tires). The tire curve is an all-terrain tread
    // (peak 1.00, EV longitudinal 1.10), so 1.00 here puts asphalt 5 % above the broom-finished concrete pad.
    m_props[size_t(SurfaceId::Asphalt)] = {"Asphalt", 1.00f, 1.00f, {0.075f, 0.075f, 0.080f}};
    // Loose gravel: peak 0.6 / sliding 0.55 (Wong Table 1.3), 0.55-0.7 in braking tests on gravel roads. Loose
    // surfaces peak at a larger slip and drop less beyond it (stones pile up ahead of a sliding tire): peak slip x2
    // (0.20 ratio, 12 deg) and 0.4 of the tire's drop (sliding/peak 0.92 like Wong's 0.55 / 0.60) are assumptions
    // shaped on that, not fitted to a measured curve. Rolling resistance of car tires on a gravel road is commonly
    // tabulated at 0.02-0.03 against 0.010-0.015 on concrete / asphalt (Wong, Gillespie): +0.015.
    m_props[size_t(SurfaceId::Gravel)] = {"Gravel (rigid, loose mu)", 0.62f, 0.58f, {0.34f, 0.31f, 0.26f}, 2.0f, 0.4f, 0.015f};

    for (size_t i = 0; i < kSurfaceCount; ++i) {
        const auto& p = m_props[i];
        m_materials[i] = new JPH::PhysicsMaterialSimple(
            p.name, JPH::Color(JPH::uint8(p.color[0] * 255), JPH::uint8(p.color[1] * 255), JPH::uint8(p.color[2] * 255)));
    }
}

SurfaceId SurfaceTable::lookup(const JPH::PhysicsMaterial* material) const {
    for (size_t i = 0; i < kSurfaceCount; ++i)
        if (m_materials[i] == material)
            return static_cast<SurfaceId>(i);
    return SurfaceId::PackedDirt;
}

namespace {
constexpr const char* kSurfaceKeys[kSurfaceCount] = {"concrete", "packed_dirt", "rock", "low_grip", "mud",
                                                     "grass",    "sand",        "snow", "asphalt",  "gravel"};
} // namespace

const char* surfaceKey(SurfaceId id) {
    return std::size_t(id) < kSurfaceCount ? kSurfaceKeys[std::size_t(id)] : "";
}

SurfaceId surfaceFromKey(std::string_view key) {
    if (key == "dirt") return SurfaceId::PackedDirt;
    for (std::size_t i = 0; i < kSurfaceCount; ++i)
        if (key == kSurfaceKeys[i]) return SurfaceId(i);
    return SurfaceId::Count;
}

} // namespace tf
