#pragma once

#include "vehicle/VehicleDefinition.h"

#include <map>
#include <string>
#include <vector>

// Garage setups: base definition -> kit -> fine-tune values within allowed ranges. A setup stores only the values
// of the parameters below (by key); everything else comes from the definition and the kit.

namespace tf {

enum class SetupParamKind { Number, Choice };

struct SetupParamInfo {
    const char* key;      // stable id used in setup files and garage.ranges ("suspension.front.frequency")
    const char* group;    // garage section heading
    const char* label;    // row label
    const char* unit;     // shown after the value ("Hz", "N/m", "%")
    SetupParamKind kind;
    float displayScale;   // shown value = stored value x displayScale (m -> mm, share -> %)
    int decimals;         // shown decimals
    float step;           // stored-unit step for left / right
    float hardMin, hardMax; // limits for any range (definition, kit)
    // Default range when the definition gives none: base x [relMin, relMax] if relative, else [absMin, absMax].
    bool relative;
    float defMin, defMax;
    bool rebuild;          // needs the vehicle recreated (in place)
    std::vector<const char*> choices; // Choice: names, stored value = index
    float (*get)(const VehicleConfig& c, const VehicleConfig& base);
    void (*set)(VehicleConfig& c, const VehicleConfig& base, float v);
};

const std::vector<SetupParamInfo>& setupParams();
const SetupParamInfo* findSetupParam(const std::string& key);

// Allowed range of a number parameter for this definition and kit (kit ranges > definition ranges > defaults),
// always containing the base value.
ParamRange setupParamRange(const SetupParamInfo& p, const VehicleDefinition& def, const VehicleKit* kit,
                           const VehicleConfig& base);

struct VehicleSetup {
    std::string name = "Factory";
    std::string kit;                      // empty = no kit
    std::map<std::string, float> values;  // only the parameters changed from the base (+ kit)
    bool operator==(const VehicleSetup&) const = default;
};

// base -> kit -> values (clamped to the ranges; unknown keys and bad choices are dropped with a warning).
bool buildSetupConfig(const VehicleDefinition& def, const VehicleSetup& setup, VehicleConfig& out,
                      std::vector<std::string>& warnings);
// Current value of a parameter in a configuration built from base.
float setupValue(const SetupParamInfo& p, const VehicleConfig& config, const VehicleConfig& base);

// All setups of one vehicle, persisted as JSON (setups/<vehicle id>.json in the user data directory).
struct SetupFile {
    std::string vehicleId;
    std::string selected;               // name of the active setup; empty = factory
    std::vector<VehicleSetup> setups;
    VehicleSetup* find(const std::string& name);
    const VehicleSetup* find(const std::string& name) const;
};

std::string writeSetupFile(const SetupFile& f);
bool parseSetupFile(const std::string& text, SetupFile& out, std::string* error);
bool loadSetupFile(const std::string& path, SetupFile& out, std::string* error);
// Written to a temporary file and renamed.
bool saveSetupFile(const std::string& path, const SetupFile& f, std::string* error);

} // namespace tf
