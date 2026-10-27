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
    // Whether the parameter exists for this vehicle (engine and gearbox rows only for a combustion drivetrain,
    // motor rows only for an electric one); nullptr = every vehicle.
    bool (*applies)(const VehicleConfig& base) = nullptr;
};

bool setupParamApplies(const SetupParamInfo& p, const VehicleConfig& base);

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

// Textbook equivalents of the suspension values for one corner of an axle, shown next to them in the garage.
// The springs are tuned by natural frequency f (mass independent); with the static sprung corner mass m
// (body mass x axle weight fraction / 2) the wheel rate is k = (2 pi f)^2 m and the critical damping 2 sqrt(k m).
// Jolt applies f and the damping ratio to the effective mass of the wheel constraint, which also contains the body's
// rotational inertia, so the rate in the simulation is close to but not exactly this estimate.
float sprungCornerMass(const VehicleConfig& c, int axle);        // kg
float wheelRateFromFrequency(float frequencyHz, float cornerMass); // N/m
float frequencyFromWheelRate(float rateNPerM, float cornerMass);   // Hz
float criticalDamping(float frequencyHz, float cornerMass);        // N*s/m

// All setups of one vehicle, persisted as JSON (setups/<vehicle id>.json in the user data directory).
// Schema 2 (v0.28.0): anti-roll bars are N/m at the wheel. Schema 1 stored them in the units of Jolt's anti-roll
// bar, whose effective rate depended on the damping and the physics step, so they have no fixed conversion: loading
// a schema 1 file drops its anti-roll values (the vehicle's own bars apply) and lists them in migrationNotes.
constexpr int kSetupSchema = 2;

struct SetupFile {
    std::string vehicleId;
    std::string selected;               // name of the active setup; empty = factory
    std::vector<VehicleSetup> setups;
    int loadedSchema = kSetupSchema;           // schema of the parsed text
    std::vector<std::string> migrationNotes;   // what parsing an older schema changed (not written)
    VehicleSetup* find(const std::string& name);
    const VehicleSetup* find(const std::string& name) const;
};

std::string writeSetupFile(const SetupFile& f);
bool parseSetupFile(const std::string& text, SetupFile& out, std::string* error);
bool loadSetupFile(const std::string& path, SetupFile& out, std::string* error);
// Written to a temporary file and renamed.
bool saveSetupFile(const std::string& path, const SetupFile& f, std::string* error);

} // namespace tf
