#pragma once

#include "render/Mesh.h"

#include <vector>

namespace tf {

class Level;

// Render content of an open-world level. The terrain itself is drawn by the renderer from the level's height grid
// (Renderer::setTerrain); these are the meshes around it.

// Static props (bridges, POI blockouts) in their style colors and the rocks, from the same triangles as their
// collision (Level::props(), addLevelRock).
MeshData buildLevelPropsMesh(const Level& level);
// Render-only water: 16 m quads over flat sea / lake areas, refined to 4 m and 1 m cells near shores and along
// the rivers (per-corner heights), and the open sea around the baked square out to kHorizonSea. Body color per kind; water_fs fades it by the depth
// to the ground maps' heights.
MeshData buildLevelWaterMesh(const Level& level);
// Render-only horizon around the baked square (no collision; the play area ends well inside the bake): a ring of
// low-detail terrain that starts at the bake edge heights and eases into hills, a mountain range to the north and
// a seabed under the sea elsewhere, out to ~14 km. Fog and distance hide its coarseness.
MeshData buildLevelHorizonMesh(const Level& level);
// Open sea plane half extent around the level (m).
inline constexpr float kHorizonSea = 80000.0f;
// Pine instances of the level (the trunks are collision in the terrain tiles).
std::vector<TreeInstance> buildLevelTreeInstances(const Level& level);

} // namespace tf
