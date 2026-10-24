#pragma once

#include "render/Mesh.h"

#include <glm/glm.hpp>

#include <vector>

namespace tf {

struct VehicleTelemetry;

// Line accumulator for physics diagnostics (rebuilt every frame).
class DebugDraw {
public:
    void clear() { m_lines.clear(); }
    void line(const glm::vec3& a, const glm::vec3& b, const glm::vec4& color);
    void cross(const glm::vec3& p, float size, const glm::vec4& color);
    void axes(const glm::mat4& m, float size);

    const std::vector<LineVertex>& lines() const { return m_lines; }

private:
    std::vector<LineVertex> m_lines;
};

struct VehicleDebugOptions {
    bool contacts = true;      // contact points and normals
    bool suspension = true;    // suspension axis, min/max travel, current wheel position
    bool tireForces = true;    // longitudinal/lateral tire forces
    bool bodyAxes = false;
    float forceScale = 0.0002f; // m per N
};

void drawVehicleDebug(DebugDraw& dd, const VehicleTelemetry& t, float suspensionMin, float suspensionMax,
                      const VehicleDebugOptions& opt);

} // namespace tf
