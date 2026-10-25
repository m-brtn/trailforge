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
// Render-only water: 16 m quads over flat sea / lake areas, 4 m quads with per-corner heights along the rivers,
// and a sea ring outside the baked square on the coast side. Body color per kind; water_fs fades it by the depth
// to the ground maps' heights.
MeshData buildLevelWaterMesh(const Level& level);
// Pine instances of the level (the trunks are collision in the terrain tiles).
std::vector<TreeInstance> buildLevelTreeInstances(const Level& level);

} // namespace tf
