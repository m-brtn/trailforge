#include "sim/DriftScore.h"

#include <quench/core/Json.h>

#include <algorithm>
#include <cmath>

namespace tf {
namespace json = quench::json;
namespace {

constexpr float kRadToDeg = 57.2957795f;
constexpr float kMinSlipSpeed = 3.0f; // m/s: below this the slip angle is undefined (0)

bool insidePolygon(const std::vector<DriftPoint>& poly, float x, float z) {
    bool in = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const DriftPoint& a = poly[i];
        const DriftPoint& b = poly[j];
        if ((a.z > z) != (b.z > z) && x < (b.x - a.x) * (z - a.z) / (b.z - a.z) + a.x) in = !in;
    }
    return in;
}

bool readPoint(const json::Value& v, DriftPoint& p) {
    if (!v.isArray() || v.array.size() != 2 || !v.array[0].isNumber() || !v.array[1].isNumber()) return false;
    p = {float(v.array[0].number), float(v.array[1].number)};
    return true;
}

bool readPoints(const json::Value& v, std::vector<DriftPoint>& out) {
    out.clear();
    if (!v.isArray()) return false;
    for (const json::Value& e : v.array) {
        DriftPoint p;
        if (!readPoint(e, p)) return false;
        out.push_back(p);
    }
    return true;
}

} // namespace

bool DriftZone::contains(float x, float z) const {
    if (polygon.size() >= 3) return insidePolygon(polygon, x, z);
    const float ex = b.x - a.x, ez = b.z - a.z, l2 = ex * ex + ez * ez;
    const float t = l2 > 1e-8f ? std::clamp(((x - a.x) * ex + (z - a.z) * ez) / l2, 0.0f, 1.0f) : 0.0f;
    return std::hypot(x - (a.x + ex * t), z - (a.z + ez * t)) <= reach;
}

bool parseDriftConfig(const json::Value& v, DriftConfig& out, std::string& err) {
    out = DriftConfig{};
    if (v.isNull()) return true;
    if (!v.isObject()) {
        err = "drift: must be an object";
        return false;
    }
    out.enabled = v["enabled"].boolOr(true);
    auto num = [&](const char* key, float& f) { f = float(v[key].numberOr(f)); };
    num("minSpeedKmh", out.minSpeedKmh);
    num("minAngleDeg", out.minAngleDeg);
    num("spinAngleDeg", out.spinAngleDeg);
    num("maxScoredAngleDeg", out.maxScoredAngleDeg);
    num("linkSeconds", out.linkSeconds);
    num("wallSpeed", out.wallSpeed);
    num("pointsScale", out.pointsScale);
    if (!(out.minSpeedKmh >= 0.0f) || !(out.minAngleDeg > 0.0f) || !(out.spinAngleDeg > out.minAngleDeg) ||
        !(out.maxScoredAngleDeg >= out.minAngleDeg) || !(out.linkSeconds >= 0.0f) || !(out.wallSpeed > 0.0f) ||
        !(out.pointsScale > 0.0f)) {
        err = "drift: limits out of range (minAngleDeg > 0, spinAngleDeg > minAngleDeg, maxScoredAngleDeg >= minAngleDeg, "
              "linkSeconds >= 0, wallSpeed > 0, pointsScale > 0)";
        return false;
    }
    if (v.has("area") && (!readPoints(v["area"], out.area) || out.area.size() < 3)) {
        err = "drift: area must be a polygon of [x, z] points (>= 3)";
        return false;
    }
    if (v.has("zones") && !v["zones"].isArray()) {
        err = "drift: zones must be an array";
        return false;
    }
    for (const json::Value& z : v["zones"].array) {
        DriftZone zone;
        zone.name = z["name"].stringOr("zone " + std::to_string(out.zones.size() + 1));
        zone.multiplier = float(z["multiplier"].numberOr(2.0));
        zone.reach = float(z["reach"].numberOr(2.0));
        bool ok = zone.multiplier >= 1.0f;
        if (z.has("polygon")) {
            ok = ok && readPoints(z["polygon"], zone.polygon) && zone.polygon.size() >= 3;
        } else {
            const json::Value& line = z["line"];
            ok = ok && line.isArray() && line.array.size() == 2 && readPoint(line.array[0], zone.a) &&
                 readPoint(line.array[1], zone.b) && zone.reach > 0.0f;
        }
        if (!ok) {
            err = "drift: zone '" + zone.name + "' needs \"polygon\" (>= 3 [x, z]) or \"line\" ([[x0, z0], [x1, z1]] with reach > 0)"
                  " and multiplier >= 1";
            return false;
        }
        out.zones.push_back(std::move(zone));
    }
    return true;
}

void DriftScorer::setConfig(DriftConfig config) {
    m_config = std::move(config);
    clearSession();
}

void DriftScorer::abort() {
    if (m_running) m_lastEnd = End::Aborted;
    m_running = false;
    m_drifting = false;
    m_score = m_runTime = m_driftTime = m_zoneTime = m_idle = 0.0;
    m_maxAngle = 0.0f;
}

void DriftScorer::clearSession() {
    abort();
    m_runs = m_failed = 0;
    m_last = m_lost = m_best = 0.0;
    m_lastEnd = End::None;
    m_zone = -1;
    m_mult = 1.0f;
}

const char* DriftScorer::endName(End e) {
    switch (e) {
    case End::Completed: return "completed";
    case End::LeftArea: return "left the course";
    case End::Spin: return "spin";
    case End::Wall: return "wall hit";
    case End::Aborted: return "reset";
    case End::None: break;
    }
    return "";
}

DriftScorer::Event DriftScorer::finish(End why) {
    m_running = false;
    m_lastEnd = why;
    if (why == End::Spin || why == End::Wall) {
        m_lost = m_score;
        m_last = 0.0;
        ++m_failed;
        return Event::Failed;
    }
    m_last = m_score;
    m_best = std::max(m_best, m_score);
    ++m_runs;
    return Event::Completed;
}

DriftScorer::Event DriftScorer::update(const DriftInput& in) {
    m_drifting = false;
    if (!m_config.enabled || !(in.dt > 0.0f)) return Event::None;
    const float speed = std::hypot(in.vx, in.vz);
    m_speed = speed * 3.6f;
    // Velocity direction relative to the heading: forward component along (fx, fz), left = (fz, -fx) (body +X).
    const float vf = in.vx * in.fx + in.vz * in.fz, vl = in.vx * in.fz - in.vz * in.fx;
    m_slip = speed > kMinSlipSpeed ? std::atan2(vl, vf) * kRadToDeg : 0.0f;
    const float angle = std::abs(m_slip);

    m_zone = -1;
    m_mult = 1.0f;
    for (std::size_t k = 0; k < m_config.zones.size(); ++k) {
        const DriftZone& z = m_config.zones[k];
        if ((m_zone < 0 || z.multiplier > m_mult) && (z.contains(in.x, in.z) || z.contains(in.rearX, in.rearZ)))
            m_zone = int(k), m_mult = z.multiplier;
    }
    const bool inArea = m_config.area.size() < 3 || insidePolygon(m_config.area, in.x, in.z);
    m_inArea = inArea;

    if (m_running) {
        if (in.impactSpeed >= m_config.wallSpeed) return finish(End::Wall);
        if (angle > m_config.spinAngleDeg) return finish(End::Spin);
        if (!inArea) return finish(End::LeftArea);
    }
    m_drifting = inArea && in.wheelsOnGround >= 2 && angle >= m_config.minAngleDeg && angle <= m_config.spinAngleDeg &&
                 m_speed >= m_config.minSpeedKmh;
    Event e = Event::None;
    if (m_drifting && !m_running) {
        m_running = true;
        m_score = m_runTime = m_driftTime = m_zoneTime = m_idle = 0.0;
        m_maxAngle = 0.0f;
        e = Event::Started;
    }
    if (!m_running) return e;
    m_runTime += in.dt;
    if (m_drifting) {
        m_idle = 0.0;
        m_driftTime += in.dt;
        if (m_zone >= 0) m_zoneTime += in.dt;
        m_maxAngle = std::max(m_maxAngle, angle);
        m_score += double(m_config.pointsScale * std::min(angle, m_config.maxScoredAngleDeg) * m_speed * in.dt * m_mult);
    } else {
        m_idle += in.dt;
        if (m_idle >= m_config.linkSeconds) return finish(End::Completed);
    }
    return e;
}

} // namespace tf
