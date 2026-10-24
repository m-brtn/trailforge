#include "track/SurfaceProperties.h"

#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>

namespace tf {

SurfaceTable::SurfaceTable() {
    // Multipliers on the tire curve (peak 1.00 long / 1.00 lat), i.e. the effective peak mu on each surface:
    // dry concrete 0.95, rock 0.75, dry packed dirt 0.70, sand 0.55, grass 0.50, mud 0.35; the low-grip patch keeps
    // its original 0.42 / 0.40. All surfaces are rigid: mud, grass and sand only have a lower mu (no sinkage, no
    // rutting, no extra rolling resistance).
    m_props[size_t(SurfaceId::Concrete)] = {"Concrete", 0.95f, 0.95f, {0.55f, 0.55f, 0.52f}};
    m_props[size_t(SurfaceId::PackedDirt)] = {"Packed dirt", 0.70f, 0.68f, {0.42f, 0.34f, 0.22f}};
    m_props[size_t(SurfaceId::Rock)] = {"Rock", 0.75f, 0.75f, {0.38f, 0.37f, 0.36f}};
    m_props[size_t(SurfaceId::LowGrip)] = {"Low grip (hard)", 0.42f, 0.40f, {0.20f, 0.35f, 0.55f}};
    m_props[size_t(SurfaceId::Mud)] = {"Mud look (rigid, low mu)", 0.35f, 0.32f, {0.16f, 0.11f, 0.07f}};
    m_props[size_t(SurfaceId::Grass)] = {"Grass (rigid)", 0.50f, 0.48f, {0.20f, 0.26f, 0.09f}};
    m_props[size_t(SurfaceId::Sand)] = {"Sand (rigid, low mu)", 0.55f, 0.52f, {0.62f, 0.52f, 0.36f}};

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

} // namespace tf
