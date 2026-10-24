#include "track/SurfaceProperties.h"

#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>

namespace tf {

SurfaceTable::SurfaceTable() {
    m_props[size_t(SurfaceId::Concrete)] = {"Concrete", 1.00f, 1.00f, {0.55f, 0.55f, 0.52f}};
    m_props[size_t(SurfaceId::PackedDirt)] = {"Packed dirt", 0.85f, 0.85f, {0.42f, 0.34f, 0.22f}};
    m_props[size_t(SurfaceId::Rock)] = {"Rock", 0.90f, 0.90f, {0.38f, 0.37f, 0.36f}};
    m_props[size_t(SurfaceId::LowGrip)] = {"Low grip (hard)", 0.35f, 0.35f, {0.20f, 0.35f, 0.55f}};

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
