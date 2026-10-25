#pragma once

#include "render/Mesh.h"

#include <array>
#include <vector>

namespace tf {

// Render-only backdrop outside the perimeter walls (unreachable, so it has no collision):
// faceted foothills with a view corridor toward the sunset (the distant mountains are drawn in the sky pass).
MeshData buildSceneryMesh();
// Stylized pine at unit height, one mesh per LOD (near / mid / far / two crossed double-sided triangle cards with
// the same silhouette, no texture), drawn instanced.
std::array<MeshData, 4> buildPineMeshes();
// Pine instances on the backdrop (outside the walls).
std::vector<TreeInstance> buildTreeInstances();
// Water surface quad over the showcase puddles (render-only; the puddle bowls are real collision geometry).
MeshData buildWaterMesh();

} // namespace tf
