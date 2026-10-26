#pragma once

#include "vehicle/VehicleBody.h"

#include <string>
#include <vector>

// Reading a Blender glTF export of a vehicle body (README "Vehicle data format", body.glb contract) into a
// BodyModel, and the simplified LODs. Used by trailforge_vehiclec and the body tests.

namespace tf {

// Node transforms are applied (everything ends up in body space), node names become part names, the node
// hierarchy the part parents. COL_* nodes are collision pieces, SPARE_* empties spare wheel seats, <part>_LOD1 /
// <part>_LOD2 authored LODs; REF_ / PROXY_ / CUT_ subtrees (Blender helpers) are skipped with a warning. Materials
// map to roles by the extras key "tf_role" or by their name up to the first '.'.
bool readBodySource(const std::string& path, BodyModel& out, std::vector<std::string>& errors,
                    std::vector<std::string>& warnings);

// Fills the missing LODs 1 and 2 of every part by simplification (about 1/2 and 1/4 of the triangles, absolute
// error limits 10 mm and 30 mm; material borders are locked) and records the largest error per LOD.
void buildBodyLods(BodyModel& model);

} // namespace tf
