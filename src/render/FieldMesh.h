#pragma once

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace tf {

// Polygonizes the zero level of a scalar field (negative inside) on a regular lattice with naive surface nets: one
// vertex per lattice cell the surface crosses, one quad per crossed lattice edge. The vertices are then projected
// onto the surface along the field gradient, and the normals are the normalized gradient.
struct FieldMesh {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<std::array<std::uint32_t, 4>> quads; // counter-clockwise seen from outside
};

FieldMesh polygonizeField(const std::function<float(const glm::vec3&)>& field, const glm::vec3& lo, const glm::vec3& hi,
                          float cell);

} // namespace tf
