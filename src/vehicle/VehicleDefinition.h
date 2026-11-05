#pragma once

#include "vehicle/VehicleConfig.h"

#include <quench/core/Json.h>

#include <map>
#include <string>
#include <vector>

// Vehicle definitions as data: assets/vehicles/<id>/vehicle.json (format reference in docs/vehicle-format.md).
// The loader validates every field (type, unit range, cross-field rules) and rejects unknown keys, with messages of
// the form "<file>: <path>: <problem>".

namespace tf {

namespace json = quench::json;

struct ParamRange {
    float min = 0.0f, max = 0.0f;
    bool operator==(const ParamRange&) const = default;
};

// A named package on top of the base definition (e.g. a lift kit, mud tires). Its overrides are a partial
// vehicle.json document merged over the base before parsing; massDelta adds to the body mass; ranges replace the
// garage slider ranges of the base. visualParts names model parts for later (not drawn yet).
struct VehicleKit {
    std::string id, name, description;
    float massDelta = 0.0f; // kg
    json::Value overrides;
    std::map<std::string, ParamRange> ranges;
    std::vector<std::string> visualParts;
};

struct VehicleDefinition {
    std::string id, name, description;
    std::string sourcePath;
    json::Value document;   // the parsed file; kits are merged over it
    VehicleConfig config;   // base configuration (no kit, no setup)
    std::map<std::string, ParamRange> garageRanges; // per setup parameter key (VehicleSetup.h)
    std::vector<VehicleKit> kits;

    const VehicleKit* findKit(const std::string& kitId) const;
};

// Parses the physics part of a vehicle document into a configuration. Returns false with messages on any error.
bool parseVehicleConfig(const json::Value& doc, const std::string& source, VehicleConfig& out,
                        std::vector<std::string>& errors);

// Parses a whole definition (config, garage ranges, kits; every kit must produce a valid configuration).
bool parseVehicleDefinition(const std::string& text, const std::string& source, VehicleDefinition& out,
                            std::vector<std::string>& errors);
bool loadVehicleDefinition(const std::string& path, VehicleDefinition& out, std::vector<std::string>& errors);

// Vehicle the game starts with when nothing else is chosen (identical to the built-in configuration).
inline constexpr const char* kDefaultVehicleId = "trailforge_suv";

// Every <dir>/<id>/vehicle.json: kDefaultVehicleId first, the others sorted by id. Broken definitions are skipped and reported in errors; the folder
// name must equal the id.
std::vector<VehicleDefinition> loadVehicleCatalog(const std::string& dir, std::vector<std::string>& errors);

// Base configuration with a kit applied (empty kit id = base).
bool configWithKit(const VehicleDefinition& def, const std::string& kitId, VehicleConfig& out,
                   std::vector<std::string>& errors);

// Known references (the procedural body model and the sound set that exist).
bool knownVisualModel(const std::string& id);
bool knownAudioProfile(const std::string& id);

} // namespace tf
