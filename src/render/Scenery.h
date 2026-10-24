#pragma once

#include "render/Mesh.h"

namespace tf {

// Render-only backdrop outside the perimeter walls (unreachable, so it has no collision):
// rolling hills, distant ridges and a pine tree line.
MeshData buildSceneryMesh();
// Water surface quad over the showcase puddles (render-only; the puddle bowls are real collision geometry).
MeshData buildWaterMesh();

} // namespace tf
