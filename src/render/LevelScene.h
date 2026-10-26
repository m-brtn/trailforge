#pragma once

#include "render/Mesh.h"

#include <string>
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
// Render-only waterfalls at the steps of the river surfaces (Level::waterfalls): a ballistic sheet from the lip, foam
// on the pool and crossed mist cards, for waterfall_fs. Vertex color: x / y piece coordinates, z kind (0 sheet,
// 1 foam, 2 mist); material: drop (m), river width (m), seed.
MeshData buildLevelWaterfallMesh(const Level& level);
// Render-only horizon around the baked square (no collision; the play area ends well inside the bake): a ring of
// low-detail terrain that starts at the bake edge heights and eases into hills, a mountain range to the north and
// a seabed under the sea elsewhere, out to ~14 km. Fog and distance hide its coarseness.
MeshData buildLevelHorizonMesh(const Level& level);
// Open sea plane half extent around the level (m).
inline constexpr float kHorizonSea = 80000.0f;
// Procedural pine instances of the level: the trees without a model (the trunks are collision in the terrain tiles).
std::vector<TreeInstance> buildLevelTreeInstances(const Level& level);
// Level model files (Level::models() paths) and the placements of the trees, rocks, logs and ground cover that use
// them, with the transforms of their collision (track/Level.h addLevelRock / addLevelTrunk).
void buildLevelModelPlacements(const Level& level, std::vector<std::string>& paths, std::vector<ModelPlacement>& out);

} // namespace tf
