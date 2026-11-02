#include "sim/WaterForces.h"

#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

#include <algorithm>
#include <cmath>

namespace tf {
namespace {

constexpr float kG = 9.81f;

bool insideBox(const WaterBox& b, JPH::Vec3Arg p, float eps) {
    const JPH::Vec3 d = (p - b.center).Abs();
    return d.GetX() <= b.halfExtents.GetX() + eps && d.GetY() <= b.halfExtents.GetY() + eps &&
           d.GetZ() <= b.halfExtents.GetZ() + eps;
}

int divisions(float extent, float cell) { return std::max(1, int(std::ceil(extent / cell - 1e-4f))); }

// Area of the segment of a circle of radius r below a chord at height h above its lowest point (0 <= h <= 2r).
float segmentArea(float r, float h) {
    const float d = r - h;
    return r * r * std::acos(std::clamp(d / r, -1.0f, 1.0f)) - d * std::sqrt(std::max(0.0f, 2.0f * r * h - h * h));
}

// Vertical half extent in world space of a body-space box (half sizes along body axes) under the transform.
float verticalHalf(const JPH::RMat44& xf, JPH::Vec3Arg half) {
    return std::abs(xf.GetAxisX().GetY()) * half.GetX() + std::abs(xf.GetAxisY().GetY()) * half.GetY() +
           std::abs(xf.GetAxisZ().GetY()) * half.GetZ();
}

void collectBoxes(const JPH::Shape& shape, JPH::Mat44Arg toBody, std::vector<WaterBox>& out) {
    using namespace JPH;
    if (shape.GetSubType() == EShapeSubType::OffsetCenterOfMass) {
        // Only the center of mass moves; the inner shape's frame is the body frame.
        collectBoxes(*static_cast<const OffsetCenterOfMassShape&>(shape).GetInnerShape(), toBody, out);
        return;
    }
    if (shape.GetType() == EShapeType::Compound) {
        const auto& compound = static_cast<const CompoundShape&>(shape);
        const Vec3 com = compound.GetCenterOfMass();
        for (const CompoundShape::SubShape& s : compound.GetSubShapes()) {
            const Vec3 pos = com + s.GetPositionCOM();
            const AABox local = s.mShape->GetLocalBounds(); // around the sub-shape's center of mass
            const AABox box = local.Transformed(toBody * Mat44::sRotationTranslation(s.GetRotation(), pos));
            out.push_back({box.GetCenter(), box.GetExtent()});
        }
        return;
    }
    AABox box = shape.GetLocalBounds();
    box.Translate(shape.GetCenterOfMass());
    box = box.Transformed(toBody);
    out.push_back({box.GetCenter(), box.GetExtent()});
}

} // namespace

std::vector<WaterBox> waterBoxesFromShape(const JPH::Shape& shape) {
    std::vector<WaterBox> out;
    collectBoxes(shape, JPH::Mat44::sIdentity(), out);
    return out;
}

void WaterForces::setBody(std::span<const WaterBox> boxes, const VehicleWater& params, float cell) {
    m_params = params;
    m_cells.clear();
    m_patches.clear();
    m_bounds = JPH::AABox();
    m_volume = m_area = 0.0f;
    for (std::size_t bi = 0; bi < boxes.size(); ++bi) {
        const WaterBox& b = boxes[bi];
        const JPH::Vec3 size = 2.0f * b.halfExtents;
        m_bounds.Encapsulate(b.center - b.halfExtents);
        m_bounds.Encapsulate(b.center + b.halfExtents);
        // Volume cells; a cell whose center lies in an earlier box is already counted there.
        const int nx = divisions(size.GetX(), cell), ny = divisions(size.GetY(), cell), nz = divisions(size.GetZ(), cell);
        const JPH::Vec3 step(size.GetX() / nx, size.GetY() / ny, size.GetZ() / nz);
        for (int i = 0; i < nx; ++i)
            for (int j = 0; j < ny; ++j)
                for (int k = 0; k < nz; ++k) {
                    const JPH::Vec3 c = b.center - b.halfExtents + step * JPH::Vec3(i + 0.5f, j + 0.5f, k + 0.5f);
                    bool counted = false;
                    for (std::size_t o = 0; o < bi && !counted; ++o) counted = insideBox(boxes[o], c, 0.0f);
                    if (counted) continue;
                    const float v = step.GetX() * step.GetY() * step.GetZ();
                    m_cells.push_back({c, 0.5f * step, v});
                    m_volume += v;
                }
        // Surface patches of the six faces; a patch on or inside another box is an inner face of the union.
        for (int axis = 0; axis < 3; ++axis)
            for (int side = -1; side <= 1; side += 2) {
                const int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
                JPH::Vec3 n = JPH::Vec3::sZero(), t1 = JPH::Vec3::sZero(), t2 = JPH::Vec3::sZero();
                n.SetComponent(axis, float(side));
                t1.SetComponent(a1, 1.0f);
                t2.SetComponent(a2, 1.0f);
                const float len1 = size[a1], len2 = size[a2];
                const int n1 = divisions(len1, cell), n2 = divisions(len2, cell);
                const Face face = axis == 2 ? Face::End : axis == 0 ? Face::Side : Face::Heave;
                for (int u = 0; u < n1; ++u)
                    for (int w = 0; w < n2; ++w) {
                        const JPH::Vec3 c = b.center + n * b.halfExtents[axis] + t1 * (len1 * ((u + 0.5f) / n1 - 0.5f)) +
                                            t2 * (len2 * ((w + 0.5f) / n2 - 0.5f));
                        bool inner = false;
                        for (std::size_t o = 0; o < boxes.size() && !inner; ++o)
                            inner = o != bi && insideBox(boxes[o], c, 1e-3f);
                        if (inner) continue;
                        const float area = (len1 / n1) * (len2 / n2);
                        m_patches.push_back({c, n, t1, t2, 0.5f * len1 / n1, 0.5f * len2 / n2, area, face});
                        m_area += area;
                    }
            }
    }
}

WaterForceResult WaterForces::compute(const WaterBodyState& s, std::span<const WaterWheel> wheels,
                                      const WaterSurface& water, const WaterFlow& flow) const {
    WaterForceResult r;
    if (!water || m_cells.empty()) return r;
    const VehicleWater& p = m_params;
    const JPH::RMat44& xf = s.bodyToWorld;

    // Early out: no water above the lowest point of the body or the wheels anywhere on a <= 1 m grid over the footprint.
    {
        const JPH::AABox world = m_bounds.Transformed(JPH::Mat44(xf));
        float lowest = world.mMin.GetY();
        float x0 = world.mMin.GetX(), x1 = world.mMax.GetX(), z0 = world.mMin.GetZ(), z1 = world.mMax.GetZ();
        for (const WaterWheel& w : wheels) {
            lowest = std::min(lowest, w.hub.GetY() - w.radius);
            x0 = std::min(x0, w.hub.GetX() - w.radius), x1 = std::max(x1, w.hub.GetX() + w.radius);
            z0 = std::min(z0, w.hub.GetZ() - w.radius), z1 = std::max(z1, w.hub.GetZ() + w.radius);
        }
        const int nx = divisions(x1 - x0, 1.0f), nz = divisions(z1 - z0, 1.0f);
        bool any = false;
        for (int i = 0; i <= nx && !any; ++i)
            for (int k = 0; k <= nz && !any; ++k) {
                const std::optional<float> h = water(x0 + (x1 - x0) * i / nx, z0 + (z1 - z0) * k / nz);
                any = h && *h > lowest;
            }
        if (!any) return r;
    }

    const JPH::Vec3 up = JPH::Vec3::sAxisY();
    const JPH::Vec3 bodyUp = xf.GetAxisY();
    auto relVelocity = [&](JPH::Vec3Arg at) {
        JPH::Vec3 v = s.linearVelocity + s.angularVelocity.Cross(at - s.centerOfMass);
        if (flow) v -= flow(at.GetX(), at.GetZ());
        return v;
    };
    JPH::Vec3 bodyForce = JPH::Vec3::sZero(), wheelForce = JPH::Vec3::sZero();
    auto add = [&](JPH::Vec3Arg at, JPH::Vec3Arg f, JPH::Vec3& part) {
        r.force += f;
        r.torque += (at - s.centerOfMass).Cross(f);
        part += f;
    };

    // Buoyancy of the body cells.
    const float rhoG = p.density * kG;
    for (const Cell& c : m_cells) {
        const JPH::Vec3 w(xf * c.center);
        const std::optional<float> h = water(w.GetX(), w.GetZ());
        if (!h) continue;
        const float e = std::max(verticalHalf(xf, c.half), 1e-4f);
        const float bottom = w.GetY() - e;
        if (*h <= bottom) continue;
        r.wet = true;
        r.bodyDepth = std::max(r.bodyDepth, *h - bottom);
        const float f = std::min((*h - bottom) / (2.0f * e), 1.0f);
        const float v = f * c.volume * p.displacement;
        r.displacedVolume += v;
        r.buoyancy += rhoG * v;
        JPH::Vec3 force = JPH::Vec3::sZero(), unused = JPH::Vec3::sZero();
        force.SetY(rhoG * v);
        add(JPH::Vec3(w.GetX(), bottom + f * e, w.GetZ()), force, unused);
    }

    // Pressure drag, skin friction and bow lift on the wetted surface patches.
    const float slope = std::tan(std::clamp(p.frontSlopeDeg, 0.0f, 80.0f) * 0.0174532925f);
    for (const Patch& q : m_patches) {
        const JPH::Vec3 c(xf * q.center);
        const std::optional<float> h = water(c.GetX(), c.GetZ());
        if (!h) continue;
        const JPH::Vec3 t1 = xf.Multiply3x3(q.t1), t2 = xf.Multiply3x3(q.t2);
        const float e = std::abs(t1.GetY()) * q.h1 + std::abs(t2.GetY()) * q.h2;
        const float bottom = c.GetY() - e;
        if (*h <= bottom) continue;
        const float f = e > 1e-4f ? std::min((*h - bottom) / (2.0f * e), 1.0f) : 1.0f;
        const float area = f * q.area;
        r.wetArea += area;
        // Center of the wetted part (along world Y; exact for vertical patches, a small offset for tilted ones).
        const JPH::Vec3 at = c - up * ((1.0f - f) * e);
        const JPH::Vec3 n = xf.Multiply3x3(q.normal);
        const JPH::Vec3 v = relVelocity(at);
        const float vn = v.Dot(n);
        JPH::Vec3 force = JPH::Vec3::sZero();
        if (vn > 0.0f) {
            const float cd = q.face == Face::End ? p.frontalCd : q.face == Face::Side ? p.sideCd : p.heaveCd;
            const float pressure = 0.5f * p.density * cd * vn * vn * area;
            force -= n * pressure;
            if (q.face == Face::End) {
                const float lift = pressure * slope;
                force += bodyUp * lift;
                r.bowLift += lift;
            }
        }
        const JPH::Vec3 vt = v - n * vn;
        force -= vt * (0.5f * p.density * p.skinCf * vt.Length() * area);
        add(at, force, bodyForce);
    }

    // Tires: buoyancy of the submerged segment, displacement drag along the rolling direction, side drag.
    for (const WaterWheel& wh : wheels) {
        const std::optional<float> h = water(wh.hub.GetX(), wh.hub.GetZ());
        if (!h) continue;
        const float bottom = wh.hub.GetY() - wh.radius;
        const float depth = std::min(*h - bottom, 2.0f * wh.radius);
        if (depth <= 0.0f) continue;
        r.wet = true;
        r.wheelDepth = std::max(r.wheelDepth, depth);
        const float segment = segmentArea(wh.radius, depth);
        const float volume = segment * wh.width;
        r.displacedVolume += volume;
        r.buoyancy += rhoG * volume;
        const JPH::Vec3 at(wh.hub.GetX(), bottom + 0.5f * depth, wh.hub.GetZ());
        JPH::Vec3 force(0.0f, rhoG * volume, 0.0f);
        JPH::Vec3 axle(wh.axle.GetX(), 0.0f, wh.axle.GetZ());
        if (axle.LengthSq() > 1e-6f) {
            axle = axle.Normalized();
            const JPH::Vec3 roll = up.Cross(axle);
            const JPH::Vec3 v = relVelocity(at);
            const float vl = v.Dot(roll), vs = v.Dot(axle);
            force -= roll * (0.5f * p.density * p.wheelCd * wh.width * depth * std::abs(vl) * vl);
            force -= axle * (0.5f * p.density * p.sideCd * segment * std::abs(vs) * vs);
        }
        add(at, force, wheelForce);
    }

    // Forces against the horizontal travel direction (telemetry).
    JPH::Vec3 vh(s.linearVelocity.GetX(), 0.0f, s.linearVelocity.GetZ());
    if (vh.LengthSq() > 1e-6f) {
        vh = vh.Normalized();
        r.bodyDrag = -bodyForce.Dot(vh);
        r.wheelDrag = -wheelForce.Dot(vh);
    }
    return r;
}

} // namespace tf
