#include "sim/TireMarks.h"

#include <algorithm>
#include <cmath>

namespace tf {
namespace {

constexpr float kMinIntensity = 0.02f;  // segments below this at both ends are not stored
constexpr float kJumpDistance = 1.5f;   // m: a larger patch jump in one tick (teleport, wheel hopping) breaks the ribbon
constexpr float kPatchLength = 0.25f;   // m: contact patch length, widens the mark when the tire slides sideways
constexpr float kBurnoutInterval = 0.1; // s: a spinning wheel that does not travel stamps a patch this often

float saturate(float x) { return std::clamp(x, 0.0f, 1.0f); }

float smoothstep(float e0, float e1, float x) {
    const float t = saturate((x - e0) / (e1 - e0));
    return t * t * (3.0f - 2.0f * t);
}

void store(float* dst, JPH::Vec3Arg v) {
    dst[0] = v.GetX();
    dst[1] = v.GetY();
    dst[2] = v.GetZ();
}

} // namespace

SurfaceMarkResponse surfaceMarkResponse(SurfaceId id) {
    switch (id) {
    case SurfaceId::Concrete: return {0.0f, 1.0f, 1.0f};
    case SurfaceId::PackedDirt: return {0.85f, 1.0f, 1.05f};
    case SurfaceId::Rock: return {0.0f, 0.35f, 1.0f};
    case SurfaceId::LowGrip: return {0.0f, 0.8f, 1.0f};
    case SurfaceId::Mud: return {1.0f, 1.0f, 1.3f};
    case SurfaceId::Grass: return {0.9f, 1.0f, 1.1f};
    default: return {0.5f, 1.0f, 1.0f};
    }
}

MarkIntensity markIntensity(const WheelContactSample& c, float referenceLoad) {
    MarkIntensity m;
    if (!c.contact) return m;
    const SurfaceMarkResponse r = surfaceMarkResponse(c.surface);
    const float load = saturate(c.load / std::max(referenceLoad, 1.0f));
    const float vs = c.slipSpeed();
    // Sliding speed relative to the faster of the patch travel and the tread speed: ~0.1 while a driven tire works
    // near its friction peak, 1 for a locked wheel or a wheel spinning in place.
    const float ratio = vs / std::max({c.velocity.Length(), std::abs(c.rollingSpeed), 1.0f});
    if (r.imprint > 0.0f) {
        // Soft ground: the tread presses in whenever the wheel is loaded; sliding churns it up.
        m.imprint = r.imprint * std::pow(load, 0.75f);
        m.slip = r.slip * smoothstep(0.5f, 3.5f, vs) * smoothstep(0.08f, 0.3f, ratio) * std::sqrt(load);
    } else {
        // Hard ground: rubber is laid down only when the patch really slides (lock-up, wheelspin, drift), darker
        // with more sliding speed and more load. Rolling and hard acceleration below ~15 % slip leave nothing.
        m.slip = r.slip * smoothstep(0.8f, 5.0f, vs) * smoothstep(0.15f, 0.45f, ratio) * load;
    }
    // Swept width perpendicular to the travel direction: a sliding patch drags its length sideways.
    const float vLong = std::abs(c.velocity.Dot(c.longitudinal)), vLat = std::abs(c.velocity.Dot(c.lateral));
    const float angle = (vLong + vLat) > 0.3f ? std::atan2(vLat, vLong) : 0.0f;
    const float swept = c.width * std::cos(angle) + kPatchLength * std::sin(angle);
    m.halfWidth = 0.5f * std::min(swept, 0.7f) * r.widthScale;
    return m;
}

TireMarks::TireMarks(const TireMarkSettings& settings) { setSettings(settings); }

void TireMarks::setSettings(const TireMarkSettings& in) {
    TireMarkSettings s = in;
    s.maxSegments = std::clamp(s.maxSegments, 256, 262144);
    s.fadeTime = std::clamp(s.fadeTime, 1.0f, 3600.0f);
    s.segmentLength = std::clamp(s.segmentLength, 0.05f, 1.0f);
    const bool resize = m_ring.size() != std::size_t(s.maxSegments);
    m_settings = s;
    if (resize) {
        m_ring.assign(std::size_t(s.maxSegments), TireMarkSegment{});
        clear();
    }
}

void TireMarks::clear() {
    m_head = 0;
    m_count = 0;
    ++m_generation;
    breakRibbons();
}

void TireMarks::breakRibbons() {
    for (Trail& t : m_trails) t.active = false;
}

void TireMarks::push(const TireMarkSegment& s) {
    m_ring[m_head] = s;
    m_head = (m_head + 1) % m_ring.size();
    m_count = std::min(m_count + 1, m_ring.size());
    ++m_written;
}

void TireMarks::expire(double time) {
    while (m_count > 0 && time - double(oldest(0).birth) > double(m_settings.fadeTime)) --m_count;
}

void TireMarks::start(Trail& t, const WheelContactSample& c, const MarkIntensity& m, double time) const {
    t.active = true;
    t.lastPos = c.patchCenter;
    t.lastSide = c.normal.Cross(c.longitudinal).NormalizedOr(c.lateral) * m.halfWidth;
    t.lastU = 0.0f;
    t.lastImprint = m.imprint;
    t.lastSlip = m.slip;
    t.lastEmit = time;
    t.accImprint = t.accSlip = t.accHalfWidth = 0.0f;
    t.accCount = 0;
}

void TireMarks::update(std::span<const WheelContactSample> wheels, double time) {
    expire(time);
    if (!m_settings.enabled) {
        breakRibbons();
        return;
    }
    const float segLen = m_settings.segmentLength;
    for (std::size_t i = 0; i < wheels.size() && i < m_trails.size(); ++i) {
        const WheelContactSample& c = wheels[i];
        Trail& t = m_trails[i];
        if (!c.contact) {
            t.active = false;
            continue;
        }
        const MarkIntensity m = markIntensity(c, m_settings.referenceLoad);
        if (!t.active) {
            start(t, c, m, time);
            continue;
        }
        t.accImprint += m.imprint;
        t.accSlip += m.slip;
        t.accHalfWidth += m.halfWidth;
        ++t.accCount;

        const JPH::Vec3 d = c.patchCenter - t.lastPos;
        const float dist = d.Length();
        if (dist > kJumpDistance) {
            start(t, c, m, time);
            continue;
        }
        const float inv = 1.0f / float(t.accCount);
        const float imprint = t.accImprint * inv, slip = t.accSlip * inv, halfWidth = t.accHalfWidth * inv;
        TireMarkSegment s{};
        s.birth = float(time);
        s.surface = float(c.surface);
        if (dist >= segLen) {
            const JPH::Vec3 dir = d / dist;
            const JPH::Vec3 side = c.normal.Cross(dir).NormalizedOr(c.lateral) * halfWidth;
            // Reversing over the same spot: keep the side vectors on the same side so the quad does not twist.
            const JPH::Vec3 side0 = t.lastSide.Dot(side) < 0.0f ? -t.lastSide : t.lastSide;
            store(s.p0, t.lastPos);
            store(s.p1, c.patchCenter);
            store(s.side0, side0);
            store(s.side1, side);
            s.u0 = t.lastU;
            s.u1 = t.lastU + dist;
            s.imprint0 = t.lastImprint;
            s.imprint1 = imprint;
            s.slip0 = t.lastSlip;
            s.slip1 = slip;
            if (std::max({s.imprint0, s.imprint1, s.slip0, s.slip1}) >= kMinIntensity) push(s);
            t.lastPos = c.patchCenter;
            t.lastSide = side;
            t.lastU = s.u1;
            t.lastImprint = imprint;
            t.lastSlip = slip;
            t.lastEmit = time;
            t.accImprint = t.accSlip = t.accHalfWidth = 0.0f;
            t.accCount = 0;
        } else if (time - t.lastEmit >= kBurnoutInterval && slip > 0.25f && dist < 0.5f * segLen) {
            // Wheelspin (or a locked wheel being pushed) without travel: stamp the patch in place; repeated stamps
            // darken the spot. The ribbon itself continues from its last point.
            const JPH::Vec3 fwd = c.longitudinal * (0.5f * kPatchLength);
            const JPH::Vec3 side = c.normal.Cross(c.longitudinal).NormalizedOr(c.lateral) * halfWidth;
            store(s.p0, c.patchCenter - fwd);
            store(s.p1, c.patchCenter + fwd);
            store(s.side0, side);
            store(s.side1, side);
            s.u0 = 0.0f;
            s.u1 = kPatchLength;
            s.imprint0 = s.imprint1 = imprint;
            s.slip0 = s.slip1 = slip;
            push(s);
            t.lastEmit = time;
            t.accImprint = t.accSlip = t.accHalfWidth = 0.0f;
            t.accCount = 0;
        }
    }
}

} // namespace tf
