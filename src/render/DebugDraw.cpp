#include "render/DebugDraw.h"

#include "render/MathConvert.h"
#include "vehicle/Vehicle.h"

namespace tf {

void DebugDraw::line(const glm::vec3& a, const glm::vec3& b, const glm::vec4& color) {
    m_lines.push_back({a, color});
    m_lines.push_back({b, color});
}

void DebugDraw::cross(const glm::vec3& p, float s, const glm::vec4& c) {
    line(p - glm::vec3(s, 0, 0), p + glm::vec3(s, 0, 0), c);
    line(p - glm::vec3(0, s, 0), p + glm::vec3(0, s, 0), c);
    line(p - glm::vec3(0, 0, s), p + glm::vec3(0, 0, s), c);
}

void DebugDraw::axes(const glm::mat4& m, float s) {
    const glm::vec3 o(m[3]);
    line(o, o + glm::vec3(m[0]) * s, {1, 0.2f, 0.2f, 1});
    line(o, o + glm::vec3(m[1]) * s, {0.2f, 1, 0.2f, 1});
    line(o, o + glm::vec3(m[2]) * s, {0.3f, 0.5f, 1, 1});
}

void drawVehicleDebug(DebugDraw& dd, const VehicleTelemetry& t, float sMin, float sMax, const VehicleDebugOptions& o) {
    for (const WheelTelemetry& w : t.wheels) {
        const glm::vec3 top = toGlm(w.suspensionTop);
        const glm::vec3 dir = toGlm(w.suspensionDir);
        if (o.suspension) {
            // Suspension axis: from the attachment point to full droop; min/max ticks; current wheel center.
            dd.line(top, top + dir * sMax, {1.0f, 0.85f, 0.1f, 0.9f});
            dd.cross(top + dir * sMin, 0.05f, {1.0f, 0.3f, 0.1f, 1});
            dd.cross(top + dir * sMax, 0.05f, {0.3f, 0.8f, 1.0f, 1});
            const glm::vec4 c = w.bumpStop ? glm::vec4(1, 0, 0, 1) : glm::vec4(1, 1, 1, 1);
            dd.cross(top + dir * w.suspensionLength, 0.09f, c);
        }
        if (!w.contact) continue;
        const glm::vec3 p = toGlm(w.contactPosition);
        if (o.contacts) {
            dd.cross(p, 0.07f, {1, 0.2f, 1, 1});
            dd.line(p, p + toGlm(w.contactNormal) * 0.6f, {0.2f, 1.0f, 0.3f, 1});
            // Load: a vertical bar proportional to the suspension force.
            dd.line(p + glm::vec3(0, 0.02f, 0), p + toGlm(w.contactNormal) * (w.load * o.forceScale),
                    {1.0f, 1.0f, 1.0f, 0.6f});
        }
        if (o.tireForces) {
            dd.line(p, p + toGlm(w.contactLongitudinal) * (w.longitudinalForce * o.forceScale), {1.0f, 0.25f, 0.2f, 1});
            dd.line(p, p + toGlm(w.contactLateral) * (w.lateralForce * o.forceScale), {0.25f, 0.45f, 1.0f, 1});
        }
    }
}

} // namespace tf
