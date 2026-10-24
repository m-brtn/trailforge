#include "sim/TireMarks.h"

#include "track/TestTrack.h"

#include <algorithm>
#include <cmath>

namespace tf {
namespace {

constexpr float kMinIntensity = 0.02f;  // segments below this at both ends are not stored
constexpr float kJumpDistance = 1.5f;   // m: a larger patch jump in one tick (teleport, wheel hopping) breaks the ribbon
constexpr float kPatchLength = 0.25f;   // m: contact patch length, widens the mark when the tire slides sideways
constexpr float kBurnoutInterval = 0.1; // s: a spinning wheel that does not travel stamps a patch this often
constexpr float kHeatRise = 0.35f;      // s: a sliding patch needs this long to lay full-strength rubber
constexpr float kHeatFall = 0.08f;      // s: time constant of the fade once the patch stops sliding

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
    case SurfaceId::Sand: return {0.9f, 1.0f, 1.2f};
    default: return {0.5f, 1.0f, 1.0f};
    }
}

float packMarkSurface(SurfaceId surface, const float c[3], float wet) {
    auto q = [](float v, float levels) { return std::lround(std::sqrt(saturate(v / 0.5f)) * levels); };
    const long code = q(c[0], 31.0f) | (q(c[1], 31.0f) << 5) | (q(c[2], 31.0f) << 10) | (q(wet * wet * 0.5f, 7.0f) << 15);
    return float(long(surface) + 8 * code);
}

SurfaceId markSurface(const TireMarkSegment& s) { return SurfaceId(long(s.surface + 0.5f) & 7); }

void markSoil(const TireMarkSegment& s, float c[3], float& wet) {
    const long code = long(s.surface + 0.5f) >> 3;
    auto d = [](long v, float levels) { const float t = float(v) / levels; return t * t * 0.5f; };
    c[0] = d(code & 31, 31.0f);
    c[1] = d((code >> 5) & 31, 31.0f);
    c[2] = d((code >> 10) & 31, 31.0f);
    wet = std::sqrt(d((code >> 15) & 7, 7.0f) * 2.0f);
}

float rubberSlide(const WheelContactSample& c) {
    if (!c.contact) return 0.0f;
    const float ref = std::max({c.velocity.Length(), std::abs(c.rollingSpeed), 1.0f});
    const float vl = std::abs(c.longitudinalSlipSpeed), vt = std::abs(c.lateralSlipSpeed);
    const float spin = smoothstep(1.0f, 5.0f, vl) * smoothstep(0.2f, 0.55f, vl / ref);
    const float scrub = smoothstep(2.5f, 7.0f, vt) * smoothstep(0.4f, 0.8f, vt / ref);
    return std::max(spin, scrub);
}

float markSlide(const WheelContactSample& c) {
    if (!c.contact) return 0.0f;
    const float ref = std::max({c.velocity.Length(), std::abs(c.rollingSpeed), 1.0f});
    const float vl = std::abs(c.longitudinalSlipSpeed);
    // Locked (or nearly locked) wheel: the patch slides at the full ground speed; below ~1 m/s the ratio to the
    // 1 m/s floor of `ref` ends the mark.
    const float locked = smoothstep(0.82f, 0.95f, vl / ref) * smoothstep(0.3f, 1.0f, vl) *
                         (std::abs(c.rollingSpeed) < 0.3f * c.velocity.Length() ? 1.0f : 0.0f);
    return std::max(rubberSlide(c), locked);
}

SoilType soilType(SurfaceId id) {
    switch (id) {
    case SurfaceId::Mud: return {1.0f, 0.6f, 1.0f, {0.045f, 0.031f, 0.018f}};
    case SurfaceId::PackedDirt: return {0.5f, 1.5f, 0.15f, {0.20f, 0.135f, 0.075f}};
    case SurfaceId::Sand: return {0.45f, 1.0f, 0.0f, {0.42f, 0.34f, 0.22f}};
    case SurfaceId::Grass: return {0.35f, 2.5f, 0.35f, {0.10f, 0.095f, 0.042f}};
    default: return {};
    }
}

float soilPickup(SurfaceId id) { return soilType(id).pickup; }

void TreadSoil::update(const WheelContactSample& c, float dt, bool inWater) {
    if (!c.contact || dt <= 0.0f) return;
    // Rolling and sliding both move tread over the ground; a sliding or spinning patch scrubs it off faster.
    const float d = (c.velocity.Length() + std::abs(c.rollingSpeed) * 0.25f + 3.0f * c.slipSpeed()) * dt;
    SoilType ground = soilType(c.surface);
    if (inWater) ground = {0.2f, 0.8f, 1.0f, {0.05f, 0.036f, 0.022f}}; // rinsed to a thin film of muddy water
    if (ground.pickup > 0.0f) {
        const float f = 1.0f - std::exp(-d / (ground.pickup > amount ? ground.pickupLength : 4.0f));
        const float before = amount;
        amount += (ground.pickup - amount) * f;
        // The tread exchanges soil with the ground it rolls on: the colour and wetness move toward it.
        const float mixW = before <= 1e-4f ? 1.0f : std::max(f, (amount - before) / std::max(amount, 1e-4f));
        for (int k = 0; k < 3; ++k) color[k] += (ground.color[k] - color[k]) * mixW;
        wetness += (ground.wetness - wetness) * mixW;
    } else {
        // Hard ground: dry dust falls off within a few meters, wet mud sticks longer.
        const float shed = 3.0f + 3.0f * wetness;
        amount *= std::exp(-d / shed);
    }
}

MarkIntensity markIntensity(const WheelContactSample& c, float referenceLoad, const TreadSoil& soil, float slide) {
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
        // with more sliding speed and (weakly) more load; a tread full of soil leaves none.
        const float sl = slide < 0.0f ? markSlide(c) : slide;
        m.slip = r.slip * sl * std::sqrt(load) * (1.0f - saturate(soil.amount));
        // A dirty tread prints its soil: dark and strong after wet mud, faint and dusty after dry dirt or sand.
        m.imprint = saturate(soil.amount * (0.35f + 0.6f * soil.wetness)) * std::sqrt(load);
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
    for (Trail& t : m_trails) {
        t.active = false;
        t.soil = {};
        t.heat = 0.0f;
    }
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
    t.surface = c.surface;
    t.lastPos = c.patchCenter;
    t.lastSide = c.normal.Cross(c.longitudinal).NormalizedOr(c.lateral) * m.halfWidth;
    t.lastU = 0.0f;
    t.lastImprint = m.imprint;
    t.lastSlip = m.slip;
    t.lastEmit = time;
    t.accImprint = t.accSlip = t.accHalfWidth = 0.0f;
    t.accCount = 0;
}

void TireMarks::emit(Trail& t, const WheelContactSample& c, float imprint, float slip, float halfWidth,
                     float surfaceWord, double time) {
    const JPH::Vec3 d = c.patchCenter - t.lastPos;
    const float dist = d.Length();
    TireMarkSegment s{};
    s.birth = float(time);
    s.surface = surfaceWord;
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
}

void TireMarks::update(std::span<const WheelContactSample> wheels, double time) {
    expire(time);
    const float dt = m_lastUpdate >= 0.0 ? float(std::clamp(time - m_lastUpdate, 0.0, 0.1)) : 0.0f;
    m_lastUpdate = time;
    if (!m_settings.enabled) {
        breakRibbons();
        return;
    }
    const float segLen = m_settings.segmentLength;
    const WaterInfo water = TestTrack::water();
    for (std::size_t i = 0; i < wheels.size() && i < m_trails.size(); ++i) {
        const WheelContactSample& c = wheels[i];
        Trail& t = m_trails[i];
        if (!c.contact) {
            t.active = false;
            t.heat = 0.0f;
            continue;
        }
        const JPH::Vec3 pc = c.patchCenter;
        const bool inWater = pc.GetY() < water.level - 0.03f && pc.GetX() > water.x0 && pc.GetX() < water.x1 &&
                             pc.GetZ() > water.z0 && pc.GetZ() < water.z1;
        t.soil.update(c, dt, inWater);
        // Patch heating: rubber builds up over the first ~0.35 s of sliding and fades as soon as the slide ends.
        const float target = surfaceMarkResponse(c.surface).imprint > 0.0f ? 0.0f : markSlide(c);
        t.heat = target > t.heat ? std::min(target, t.heat + dt / kHeatRise)
                                 : target + (t.heat - target) * std::exp(-dt / kHeatFall);
        const MarkIntensity m = markIntensity(c, m_settings.referenceLoad, t.soil, t.heat);
        if (!t.active) {
            start(t, c, m, time);
            continue;
        }
        if (c.surface != t.surface) {
            // Surface boundary: the ribbon so far ends here in its own surface's style and with its own values, and
            // a new ribbon starts. A segment never carries the churn or imprint of one surface into another one's
            // style, where mud churn would read as a short wide rubber blob on concrete.
            const float dist = (c.patchCenter - t.lastPos).Length();
            if (dist > 0.01f && dist <= kJumpDistance) {
                const bool acc = t.accCount > 0;
                const float inv = acc ? 1.0f / float(t.accCount) : 0.0f;
                emit(t, c, acc ? t.accImprint * inv : t.lastImprint, acc ? t.accSlip * inv : t.lastSlip,
                     acc ? t.accHalfWidth * inv : t.lastSide.Length(), packMarkSurface(t.surface, t.soil.color, t.soil.wetness),
                     time);
            }
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
        s.surface = packMarkSurface(c.surface, t.soil.color, t.soil.wetness);
        if (dist >= segLen) {
            emit(t, c, imprint, slip, halfWidth, s.surface, time);
        } else if (time - t.lastEmit >= kBurnoutInterval && slip > 0.25f && dist < 0.5f * segLen &&
                   c.longitudinalSlipSpeed > 2.0f && std::abs(c.rollingSpeed) > c.velocity.Length() + 2.0f) {
            // A driven wheel spinning in place (burnout): stamp the patch; repeated stamps darken the spot. A
            // braking wheel never stamps. The ribbon itself continues from its last point.
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
