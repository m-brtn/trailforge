#include "sim/Autopilot.h"

#include <algorithm>
#include <cmath>

namespace tf {

void Autopilot::setRoute(const DriveRoute& route, float wheelBase, float maxSteerAngleRad) {
    m_route = route;
    m_wheelBase = std::max(wheelBase, 0.5f);
    m_maxSteer = std::max(maxSteerAngleRad, 0.05f);
    const std::size_t n = m_route.points.size();
    m_arc.assign(n, 0.0f);
    m_limit.assign(n, m_route.cruiseSpeed);
    m_curvature.assign(n, 0.0f);
    m_length = 0.0f;
    if (!m_route.valid()) {
        restart();
        return;
    }
    for (std::size_t i = 0; i < n; ++i) {
        m_arc[i] = m_length;
        const RoutePoint& a = m_route.points[i];
        const RoutePoint& b = m_route.points[(i + 1) % n];
        m_length += std::hypot(b.x - a.x, b.z - a.z);
    }
    // Curvature from the heading change over +-6 m, then the speed that keeps the lateral acceleration.
    const float spacing = m_length / float(n);
    const int k = std::max(1, int(std::lround(6.0f / std::max(spacing, 0.1f))));
    for (std::size_t i = 0; i < n; ++i) {
        const RoutePoint& p0 = m_route.points[(i + n - std::size_t(k) % n) % n];
        const RoutePoint& p1 = m_route.points[i];
        const RoutePoint& p2 = m_route.points[(i + std::size_t(k)) % n];
        const float ax = p1.x - p0.x, az = p1.z - p0.z, bx = p2.x - p1.x, bz = p2.z - p1.z;
        const float la = std::hypot(ax, az), lb = std::hypot(bx, bz);
        if (la < 1e-3f || lb < 1e-3f) continue;
        const float turn = std::abs(std::atan2(ax * bz - az * bx, ax * bx + az * bz));
        const float curvature = turn / (0.5f * (la + lb));
        m_curvature[i] = curvature;
        if (curvature > 1e-4f)
            m_limit[i] = std::min(m_limit[i], std::max(2.5f, std::sqrt(m_settings.lateralAccel / curvature)));
    }
    // Slow down ahead of curves and speed up gently after them: walk the loop backwards and forwards twice so the
    // ramps wrap around the start.
    auto segment = [&](std::size_t j) { return j + 1 == n ? m_length - m_arc[j] : m_arc[j + 1] - m_arc[j]; };
    for (int pass = 0; pass < 2; ++pass) {
        for (std::size_t j = n; j-- > 0;) {
            const std::size_t next = (j + 1) % n;
            m_limit[j] = std::min(m_limit[j], std::sqrt(m_limit[next] * m_limit[next] + 2.0f * m_settings.planDecel * segment(j)));
        }
        for (std::size_t j = 0; j < n; ++j) {
            const std::size_t next = (j + 1) % n;
            m_limit[next] = std::min(m_limit[next], std::sqrt(m_limit[j] * m_limit[j] + 2.0f * m_settings.planAccel * segment(j)));
        }
    }
    restart();
}

void Autopilot::restart() {
    m_tracking = false;
    m_s = m_driven = m_lateral = m_maxLateral = m_target = 0.0f;
    m_integral = 0.0f;
    m_offRoute = m_stuck = m_flipped = 0.0f;
    m_wantsReset = false;
    m_resetReason = "";
}

float Autopilot::wrapS(float s) const {
    if (m_length <= 0.0f) return 0.0f;
    s = std::fmod(s, m_length);
    return s < 0.0f ? s + m_length : s;
}

RoutePoint Autopilot::pointAt(float s) const {
    if (!m_route.valid()) return {};
    s = wrapS(s);
    const std::size_t n = m_route.points.size();
    const std::size_t i = std::size_t(std::upper_bound(m_arc.begin(), m_arc.end(), s) - m_arc.begin()) - 1;
    const RoutePoint& a = m_route.points[i];
    const RoutePoint& b = m_route.points[(i + 1) % n];
    const float segEnd = i + 1 < n ? m_arc[i + 1] : m_length;
    const float t = std::clamp((s - m_arc[i]) / std::max(segEnd - m_arc[i], 1e-4f), 0.0f, 1.0f);
    return {a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t};
}

float Autopilot::speedLimitAt(float s) const {
    if (!m_route.valid()) return 0.0f;
    s = wrapS(s);
    const std::size_t n = m_route.points.size();
    const std::size_t i = std::size_t(std::upper_bound(m_arc.begin(), m_arc.end(), s) - m_arc.begin()) - 1;
    const float segEnd = i + 1 < n ? m_arc[i + 1] : m_length;
    const float t = std::clamp((s - m_arc[i]) / std::max(segEnd - m_arc[i], 1e-4f), 0.0f, 1.0f);
    return m_limit[i] + (m_limit[(i + 1) % n] - m_limit[i]) * t;
}

namespace {
// Closest point of segment a-b to p: parameter 0..1 and squared distance.
float segmentProject(const RoutePoint& a, const RoutePoint& b, float x, float z, float& d2) {
    const float ex = b.x - a.x, ez = b.z - a.z;
    const float l2 = ex * ex + ez * ez;
    const float t = l2 > 1e-8f ? std::clamp(((x - a.x) * ex + (z - a.z) * ez) / l2, 0.0f, 1.0f) : 0.0f;
    const float px = a.x + ex * t - x, pz = a.z + ez * t - z;
    d2 = px * px + pz * pz;
    return t;
}
} // namespace

float Autopilot::nearestArc(float x, float z, float* distance) const {
    if (!m_route.valid()) return 0.0f;
    const std::size_t n = m_route.points.size();
    float best = 1e30f, bestS = 0.0f;
    for (std::size_t i = 0; i < n; ++i) {
        float d2 = 0.0f;
        const std::size_t j = (i + 1) % n;
        const float t = segmentProject(m_route.points[i], m_route.points[j], x, z, d2);
        if (d2 < best) {
            best = d2;
            const float segEnd = j == 0 ? m_length : m_arc[j];
            bestS = m_arc[i] + (segEnd - m_arc[i]) * t;
        }
    }
    if (distance) *distance = std::sqrt(best);
    return wrapS(bestS);
}

// Nearest point within a window around the current progress (a loop that passes close to itself elsewhere, or a
// junction, cannot make the progress jump).
float Autopilot::localNearest(float x, float z, float* distance) const {
    const std::size_t n = m_route.points.size();
    const float spacing = m_length / float(n);
    const int back = int(10.0f / spacing) + 1, ahead = int(30.0f / spacing) + 2;
    const int i0 = int(std::upper_bound(m_arc.begin(), m_arc.end(), m_s) - m_arc.begin()) - 1;
    float best = 1e30f, bestS = m_s;
    for (int k = -back; k <= ahead; ++k) {
        const std::size_t i = std::size_t(((i0 + k) % int(n) + int(n)) % int(n));
        const std::size_t j = (i + 1) % n;
        float d2 = 0.0f;
        const float t = segmentProject(m_route.points[i], m_route.points[j], x, z, d2);
        if (d2 < best) {
            best = d2;
            const float segEnd = j == 0 ? m_length : m_arc[j];
            bestS = m_arc[i] + (segEnd - m_arc[i]) * t;
        }
    }
    if (distance) *distance = std::sqrt(best);
    return wrapS(bestS);
}

RoutePoint Autopilot::startPoint() const { return m_route.valid() ? m_route.points.front() : RoutePoint{}; }

float Autopilot::startYaw() const {
    if (!m_route.valid()) return 0.0f;
    const RoutePoint a = pointAt(0.0f), b = pointAt(4.0f);
    return std::atan2(b.x - a.x, b.z - a.z);
}

AutopilotOutput Autopilot::update(const AutopilotInput& in, float dt) {
    AutopilotOutput out;
    if (!m_route.valid()) {
        out.brake = 1.0f;
        return out;
    }
    const AutopilotSettings& c = m_settings;
    const float v = in.forwardSpeed;

    // Progress along the route.
    float dist = 0.0f;
    if (!m_tracking) {
        m_s = nearestArc(in.x, in.z, &dist);
        m_tracking = true;
    } else {
        const float s = localNearest(in.x, in.z, &dist);
        float ds = s - m_s;
        if (ds > 0.5f * m_length) ds -= m_length;
        if (ds < -0.5f * m_length) ds += m_length;
        m_driven += ds;
        m_s = s;
    }
    m_lateral = dist;
    m_maxLateral = std::max(m_maxLateral, dist);

    // Pure pursuit: the arc through the target point ahead on the route, as a front wheel angle.
    float lookahead = std::clamp(c.lookaheadMin + c.lookaheadPerSpeed * std::max(v, 0.0f), c.lookaheadMin, c.lookaheadMax);
    {
        const std::size_t n = m_route.points.size();
        const int i0 = int(std::upper_bound(m_arc.begin(), m_arc.end(), m_s) - m_arc.begin()) - 1;
        const int steps = int(lookahead / std::max(m_length / float(n), 0.1f)) + 2;
        float kMax = 0.0f;
        for (int k = 0; k <= steps; ++k) kMax = std::max(kMax, m_curvature[std::size_t(i0 + k) % n]);
        if (kMax > 1e-4f) lookahead = std::min(lookahead, std::max(c.lookaheadCornerMin, c.cornerLookahead / kMax));
    }
    const RoutePoint target = pointAt(m_s + lookahead);
    const float dx = target.x - in.x, dz = target.z - in.z;
    const float ahead = dx * in.fx + dz * in.fz;
    const float left = dx * in.fz - dz * in.fx; // vehicle +X is left
    const float d2 = std::max(ahead * ahead + left * left, 1.0f);
    const float curvature = 2.0f * left / d2;
    const float wheelAngle = std::atan(curvature * m_wheelBase);
    out.steer = std::clamp(-wheelAngle / m_maxSteer, -1.0f, 1.0f);

    // Target speed: the route profile, lower on descents and when far off the line.
    float target_v = speedLimitAt(m_s);
    const float grade = -in.forwardY; // + downhill
    if (grade > c.downhillGrade) target_v *= std::max(0.55f, 1.0f - c.downhillSlowdown * (grade - c.downhillGrade));
    if (dist > 3.0f) target_v *= std::clamp(1.0f - (dist - 3.0f) / 10.0f, 0.4f, 1.0f);
    m_target = target_v;

    // PI on the throttle; brake when clearly too fast. The integral holds the throttle needed on climbs.
    const float e = target_v - v;
    if (e > -0.5f) {
        m_integral = std::clamp(m_integral + e * dt, 0.0f, 6.0f);
        const float cap = c.maxThrottle * (1.0f - c.steerEase * std::abs(out.steer));
        out.throttle = std::clamp(0.06f + c.kp * e + c.ki * m_integral, 0.0f, cap);
    } else {
        m_integral = std::max(0.0f, m_integral - 2.0f * dt);
        out.brake = std::clamp(0.05f + c.kb * (-e - 0.5f), 0.0f, c.maxBrake);
    }

    // Recovery conditions.
    m_offRoute = dist > c.offRouteDistance ? m_offRoute + dt : 0.0f;
    m_stuck = std::abs(v) < c.stuckSpeed && out.throttle > 0.2f ? m_stuck + dt : 0.0f;
    m_flipped = in.upDot < c.flippedUpDot ? m_flipped + dt : 0.0f;
    if (!m_wantsReset) {
        if (m_flipped > c.flippedTime) m_resetReason = "flipped";
        else if (m_offRoute > c.offRouteTime) m_resetReason = "off the route";
        else if (m_stuck > c.stuckTime) m_resetReason = "stuck";
        m_wantsReset = m_resetReason[0] != '\0';
    }
    return out;
}

} // namespace tf
