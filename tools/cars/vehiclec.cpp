// trailforge_vehiclec: converts a Blender glTF export of a vehicle body into the game's body.glb
// (docs/vehicle-format.md, body.glb) and validates it against the vehicle definition: part names, material roles, convex
// collision pieces, no body point within the wheel sweep (tire over the full suspension travel and steering lock
// of every kit and garage ride height), visual vs collision bounds.
//
// Usage:
//   trailforge_vehiclec --vehicle assets/vehicles/<id>/vehicle.json SOURCE.glb OUT.glb [--clearance M]
//   trailforge_vehiclec --vehicle assets/vehicles/<id>/vehicle.json --check body.glb [--clearance M]
// Exit code 0: written / valid; 1: validation errors (nothing written); 2: usage or read errors.

#include "BodySource.h"

#include "core/Json.h"
#include "vehicle/VehicleDefinition.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

using namespace tf;

namespace {

// The definition without its body model (the file may not exist yet, or be the one being replaced).
bool loadDefinition(const std::string& path, VehicleDefinition& def) {
    std::ifstream f(path, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    json::Value doc;
    std::string err;
    if (!f || !json::parse(ss.str(), doc, &err)) {
        std::fprintf(stderr, "%s: cannot read (%s)\n", path.c_str(), err.c_str());
        return false;
    }
    if (doc.has("body")) {
        doc.object["body"].object.erase("model");
        doc.object["body"].object.erase("paint");
    }
    std::vector<std::string> errors;
    if (!parseVehicleDefinition(json::write(doc), path, def, errors)) {
        for (const std::string& e : errors) std::fprintf(stderr, "%s\n", e.c_str());
        return false;
    }
    return true;
}

void report(const BodyModel& m, const BodyValidation& v, std::size_t configs) {
    std::size_t tris[kBodyLodCount] = {};
    for (const BodyPart& p : m.parts)
        for (std::size_t l = 0; l < p.lods.size() && l < std::size_t(kBodyLodCount); ++l) tris[l] += p.lods[l].triangles();
    std::printf("parts %zu, materials %zu, collision pieces %zu, spares %zu\n", m.parts.size(), m.materials.size(),
                m.hulls.size(), m.spares.size());
    std::printf("triangles LOD0 %zu, LOD1 %zu, LOD2 %zu", tris[0], tris[1], tris[2]);
    if (m.lodError.size() == std::size_t(kBodyLodCount))
        std::printf(" (simplification error LOD1 %.1f mm, LOD2 %.1f mm)", 1000.0f * m.lodError[1], 1000.0f * m.lodError[2]);
    std::printf("\nbounds (%.3f, %.3f, %.3f) .. (%.3f, %.3f, %.3f)\n", m.boundsMin[0], m.boundsMin[1], m.boundsMin[2],
                m.boundsMax[0], m.boundsMax[1], m.boundsMax[2]);
    std::printf("wheel sweep (%zu configurations): body clearance %.1f mm at %s, collision clearance %.1f mm\n", configs,
                1000.0f * v.minClearance, v.clearanceWhere.c_str(), 1000.0f * v.hullClearance);
    std::printf("visual vs collision: bounds deviation %.1f mm, %.1f%% of the vertices within 100 mm of a piece\n",
                1000.0f * v.boundsDeviation, 100.0f * v.hullCoverage);
    for (const BodyMaterial& mat : m.materials)
        std::printf("  material %-24s role %-16s colour (%.3f %.3f %.3f) rough %.2f metal %.2f coat %.2f trans %.2f\n",
                    mat.name.c_str(), bodyRoleName(mat.role), mat.color[0], mat.color[1], mat.color[2], mat.roughness,
                    mat.metallic, mat.clearcoat, mat.transmission);
    for (const std::string& w : v.warnings) std::printf("WARNING: %s\n", w.c_str());
    for (const std::string& e : v.errors) std::printf("ERROR: %s\n", e.c_str());
}

int usage() {
    std::fprintf(stderr,
                 "usage: trailforge_vehiclec --vehicle VEHICLE.json SOURCE.glb OUT.glb [--clearance M]\n"
                 "       trailforge_vehiclec --vehicle VEHICLE.json --check BODY.glb [--clearance M]\n");
    return 2;
}

} // namespace

int main(int argc, char** argv) {
    std::string vehicle, check;
    std::vector<std::string> files;
    float clearance = kBodyClearance;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--vehicle" && i + 1 < argc) vehicle = argv[++i];
        else if (a == "--check" && i + 1 < argc) check = argv[++i];
        else if (a == "--clearance" && i + 1 < argc) clearance = std::strtof(argv[++i], nullptr);
        else if (!a.empty() && a[0] != '-') files.push_back(a);
        else return usage();
    }
    if (vehicle.empty() || (check.empty() && files.size() != 2) || (!check.empty() && !files.empty())) return usage();
    VehicleDefinition def;
    if (!loadDefinition(vehicle, def)) return 2;
    const std::vector<VehicleConfig> configs = bodyCheckConfigs(def);

    BodyModel model;
    if (!check.empty()) {
        std::string err;
        if (!loadBodyModel(check, model, true, &err)) {
            std::fprintf(stderr, "%s\n", err.c_str());
            return 2;
        }
        const BodyValidation v = validateBodyModel(model, configs, clearance);
        report(model, v, configs.size());
        std::printf("%s: %s\n", check.c_str(), v.errors.empty() ? "valid" : "INVALID");
        return v.errors.empty() ? 0 : 1;
    }

    std::vector<std::string> errors, warnings;
    const bool read = readBodySource(files[0], model, errors, warnings);
    for (const std::string& w : warnings) std::printf("WARNING: %s\n", w.c_str());
    for (const std::string& e : errors) std::printf("ERROR: %s\n", e.c_str());
    if (!read) return model.parts.empty() ? 2 : 1;
    buildBodyLods(model);
    model.computeBounds();
    const BodyValidation v = validateBodyModel(model, configs, clearance);
    report(model, v, configs.size());
    if (!v.errors.empty()) {
        std::printf("%s: INVALID, nothing written\n", files[0].c_str());
        return 1;
    }
    std::string err;
    BodyModel back;
    if (!writeBodyModel(files[1], model, &err) || !loadBodyModel(files[1], back, true, &err)) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 2;
    }
    std::printf("wrote %s\n", files[1].c_str());
    return 0;
}
