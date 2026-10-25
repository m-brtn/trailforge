#include "vehicle/VehicleSetup.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace tf {
namespace {

using C = VehicleConfig;

AxleTuning& axle(C& c, int i) { return c.tuning.axles[std::size_t(i)]; }
const AxleTuning& axle(const C& c, int i) { return c.tuning.axles[std::size_t(i)]; }

float brakeTotal(const C& c) { return axle(c, kFrontAxle).brakeTorque + axle(c, kRearAxle).brakeTorque; }
float brakeBias(const C& c) {
    const float total = brakeTotal(c);
    return total > 0.0f ? axle(c, kFrontAxle).brakeTorque / total : 0.5f;
}
void setBrakes(C& c, float total, float bias) {
    axle(c, kFrontAxle).brakeTorque = total * bias;
    axle(c, kRearAxle).brakeTorque = total * (1.0f - bias);
}

constexpr float kTwoPi = 6.28318530718f;

std::vector<SetupParamInfo> buildParams() {
    using K = SetupParamKind;
    std::vector<SetupParamInfo> p;
    // Suspension
    p.push_back({"suspension.front.frequency", "SUSPENSION", "Front spring", "Hz", K::Number, 1.0f, 2, 0.05f, 0.5f, 5.0f,
                 true, 0.6f, 1.6f, false, {},
                 [](const C& c, const C&) { return axle(c, 0).springFrequency; },
                 [](C& c, const C&, float v) { axle(c, 0).springFrequency = v; }});
    p.push_back({"suspension.rear.frequency", "SUSPENSION", "Rear spring", "Hz", K::Number, 1.0f, 2, 0.05f, 0.5f, 5.0f,
                 true, 0.6f, 1.6f, false, {},
                 [](const C& c, const C&) { return axle(c, 1).springFrequency; },
                 [](C& c, const C&, float v) { axle(c, 1).springFrequency = v; }});
    p.push_back({"suspension.front.compression", "SUSPENSION", "Front compression damping", "", K::Number, 1.0f, 2, 0.02f,
                 0.05f, 2.0f, true, 0.4f, 2.0f, false, {},
                 [](const C& c, const C&) { return axle(c, 0).compressionDamping; },
                 [](C& c, const C&, float v) { axle(c, 0).compressionDamping = v; }});
    p.push_back({"suspension.front.rebound", "SUSPENSION", "Front rebound damping", "", K::Number, 1.0f, 2, 0.02f, 0.05f,
                 2.0f, true, 0.4f, 2.0f, false, {},
                 [](const C& c, const C&) { return axle(c, 0).reboundDamping; },
                 [](C& c, const C&, float v) { axle(c, 0).reboundDamping = v; }});
    p.push_back({"suspension.rear.compression", "SUSPENSION", "Rear compression damping", "", K::Number, 1.0f, 2, 0.02f,
                 0.05f, 2.0f, true, 0.4f, 2.0f, false, {},
                 [](const C& c, const C&) { return axle(c, 1).compressionDamping; },
                 [](C& c, const C&, float v) { axle(c, 1).compressionDamping = v; }});
    p.push_back({"suspension.rear.rebound", "SUSPENSION", "Rear rebound damping", "", K::Number, 1.0f, 2, 0.02f, 0.05f,
                 2.0f, true, 0.4f, 2.0f, false, {},
                 [](const C& c, const C&) { return axle(c, 1).reboundDamping; },
                 [](C& c, const C&, float v) { axle(c, 1).reboundDamping = v; }});
    p.push_back({"suspension.front.preload", "SUSPENSION", "Front preload", "mm", K::Number, 1000.0f, 0, 0.005f, 0.0f,
                 0.3f, false, 0.0f, 0.08f, true, {},
                 [](const C& c, const C&) { return c.axles[0].suspensionPreload; },
                 [](C& c, const C&, float v) { c.axles[0].suspensionPreload = v; }});
    p.push_back({"suspension.rear.preload", "SUSPENSION", "Rear preload", "mm", K::Number, 1000.0f, 0, 0.005f, 0.0f, 0.3f,
                 false, 0.0f, 0.08f, true, {},
                 [](const C& c, const C&) { return c.axles[1].suspensionPreload; },
                 [](C& c, const C&, float v) { c.axles[1].suspensionPreload = v; }});
    p.push_back({"suspension.front.antiRoll", "SUSPENSION", "Front anti-roll bar", "N/m", K::Number, 1.0f, 0, 1000.0f,
                 0.0f, 300000.0f, false, 0.0f, 40000.0f, false, {},
                 [](const C& c, const C&) { return axle(c, 0).antiRoll; },
                 [](C& c, const C&, float v) { axle(c, 0).antiRoll = v; }});
    p.push_back({"suspension.rear.antiRoll", "SUSPENSION", "Rear anti-roll bar", "N/m", K::Number, 1.0f, 0, 1000.0f, 0.0f,
                 300000.0f, false, 0.0f, 40000.0f, false, {},
                 [](const C& c, const C&) { return axle(c, 1).antiRoll; },
                 [](C& c, const C&, float v) { axle(c, 1).antiRoll = v; }});
    p.push_back({"suspension.rideHeight", "SUSPENSION", "Ride height (vs. base)", "mm", K::Number, 1000.0f, 0, 0.01f,
                 -0.15f, 0.15f, false, -0.05f, 0.08f, true, {},
                 [](const C& c, const C& b) { return b.attachHeight - c.attachHeight; },
                 [](C& c, const C& b, float v) { c.attachHeight = b.attachHeight - v; }});
    // Tires
    p.push_back({"tires.gripScale", "TIRES", "Grip scale (pressure proxy)", "x", K::Number, 1.0f, 2, 0.02f, 0.5f, 1.5f,
                 false, 0.85f, 1.15f, false, {},
                 [](const C& c, const C& b) { return c.tuning.tirePeakLongFriction / b.tuning.tirePeakLongFriction; },
                 [](C& c, const C& b, float v) {
                     c.tuning.tirePeakLongFriction = b.tuning.tirePeakLongFriction * v;
                     c.tuning.tireSlideLongFriction = b.tuning.tireSlideLongFriction * v;
                     c.tuning.tirePeakLatFriction = b.tuning.tirePeakLatFriction * v;
                     c.tuning.tireSlideLatFriction = b.tuning.tireSlideLatFriction * v;
                 }});
    p.push_back({"tires.peakSlip", "TIRES", "Peak slip ratio (long. stiffness proxy)", "", K::Number, 1.0f, 3, 0.005f,
                 0.02f, 0.5f, true, 0.6f, 1.8f, false, {},
                 [](const C& c, const C&) { return c.tuning.tirePeakSlipRatio; },
                 [](C& c, const C&, float v) { c.tuning.tirePeakSlipRatio = v; }});
    p.push_back({"tires.peakSlipAngle", "TIRES", "Peak slip angle (cornering stiffness proxy)", "deg", K::Number, 1.0f,
                 1, 0.5f, 1.0f, 20.0f, true, 0.6f, 1.8f, false, {},
                 [](const C& c, const C&) { return c.tuning.tirePeakSlipAngleDeg; },
                 [](C& c, const C&, float v) { c.tuning.tirePeakSlipAngleDeg = v; }});
    p.push_back({"tires.staticGrip", "TIRES", "Static grip below breakaway", "", K::Choice, 1.0f, 0, 1.0f, 0.0f, 1.0f,
                 false, 0.0f, 1.0f, false, {"OFF", "ON"},
                 [](const C& c, const C&) { return c.tuning.tireStaticGrip ? 1.0f : 0.0f; },
                 [](C& c, const C&, float v) { c.tuning.tireStaticGrip = v > 0.5f; }});
    // Engine and gearbox
    p.push_back({"engine.torqueScale", "ENGINE AND GEARBOX", "Engine torque", "%", K::Number, 100.0f, 0, 0.02f, 0.3f,
                 2.0f, false, 0.7f, 1.3f, false, {},
                 [](const C& c, const C& b) { return c.tuning.engineMaxTorque / b.tuning.engineMaxTorque; },
                 [](C& c, const C& b, float v) { c.tuning.engineMaxTorque = b.tuning.engineMaxTorque * v; }});
    p.push_back({"engine.revLimit", "ENGINE AND GEARBOX", "Rev limit", "rpm", K::Number, 1.0f, 0, 100.0f, 1000.0f,
                 12000.0f, true, 0.7f, 1.0f, false, {},
                 [](const C& c, const C&) { return c.tuning.revLimitRPM; },
                 [](C& c, const C&, float v) { c.tuning.revLimitRPM = std::min(v, c.engineMaxRPM); }});
    p.push_back({"gearbox.finalDrive", "ENGINE AND GEARBOX", "Final drive", "", K::Number, 1.0f, 2, 0.05f, 1.5f, 8.0f, true,
                 0.8f, 1.25f, false, {},
                 [](const C& c, const C&) { return c.tuning.finalDrive; },
                 [](C& c, const C&, float v) { c.tuning.finalDrive = v; }});
    // Brakes
    p.push_back({"brakes.torqueScale", "BRAKES", "Brake torque", "%", K::Number, 100.0f, 0, 0.05f, 0.3f, 2.0f, false,
                 0.6f, 1.4f, false, {},
                 [](const C& c, const C& b) { return brakeTotal(c) / std::max(brakeTotal(b), 1.0f); },
                 [](C& c, const C& b, float v) { setBrakes(c, brakeTotal(b) * v, brakeBias(c)); }});
    p.push_back({"brakes.frontBias", "BRAKES", "Front brake share", "%", K::Number, 100.0f, 0, 0.01f, 0.2f, 0.8f, false,
                 0.4f, 0.7f, false, {},
                 [](const C& c, const C&) { return brakeBias(c); },
                 [](C& c, const C&, float v) { setBrakes(c, brakeTotal(c), v); }});
    // Drivetrain
    p.push_back({"drivetrain.layout", "DRIVETRAIN", "Layout", "", K::Choice, 1.0f, 0, 1.0f, 0.0f, 4.0f, false, 0.0f, 4.0f,
                 true, {"RWD", "FWD", "AWD", "PART-TIME 4WD", "ON-DEMAND AWD"},
                 [](const C& c, const C&) { return float(int(c.layout)); },
                 [](C& c, const C&, float v) { c.layout = DriveLayout(std::clamp(int(std::lround(v)), 0, 4)); }});
    p.push_back({"drivetrain.frontShare", "DRIVETRAIN", "Front torque share (AWD / on-demand)", "%", K::Number, 100.0f,
                 0, 0.05f, 0.05f, 0.95f, false, 0.1f, 0.9f, false, {},
                 [](const C& c, const C&) { return c.tuning.frontTorqueShare; },
                 [](C& c, const C&, float v) { c.tuning.frontTorqueShare = v; }});
    static const std::vector<const char*> kDiffs = {"OPEN", "LSD", "LOCKED"};
    p.push_back({"drivetrain.frontDiff", "DRIVETRAIN", "Front differential", "", K::Choice, 1.0f, 0, 1.0f, 0.0f, 2.0f,
                 false, 0.0f, 2.0f, false, kDiffs,
                 [](const C& c, const C&) { return float(int(c.tuning.frontDiff)); },
                 [](C& c, const C&, float v) { c.tuning.frontDiff = DiffMode(std::clamp(int(std::lround(v)), 0, 2)); }});
    p.push_back({"drivetrain.rearDiff", "DRIVETRAIN", "Rear differential", "", K::Choice, 1.0f, 0, 1.0f, 0.0f, 2.0f, false,
                 0.0f, 2.0f, false, kDiffs,
                 [](const C& c, const C&) { return float(int(c.tuning.rearDiff)); },
                 [](C& c, const C&, float v) { c.tuning.rearDiff = DiffMode(std::clamp(int(std::lround(v)), 0, 2)); }});
    p.push_back({"drivetrain.centerDiff", "DRIVETRAIN", "Center differential", "", K::Choice, 1.0f, 0, 1.0f, 0.0f, 2.0f,
                 false, 0.0f, 2.0f, false, kDiffs,
                 [](const C& c, const C&) { return float(int(c.tuning.centerDiff)); },
                 [](C& c, const C&, float v) { c.tuning.centerDiff = DiffMode(std::clamp(int(std::lround(v)), 0, 2)); }});
    p.push_back({"drivetrain.lsdRatio", "DRIVETRAIN", "LSD lock ratio (front / rear)", "", K::Number, 1.0f, 2, 0.05f,
                 1.01f, 5.0f, false, 1.1f, 3.0f, false, {},
                 [](const C& c, const C&) { return c.tuning.limitedSlipRatio; },
                 [](C& c, const C&, float v) { c.tuning.limitedSlipRatio = v; }});
    // Driving aids
    p.push_back({"aids.absReleaseSlip", "DRIVING AIDS", "ABS release slip", "", K::Number, 1.0f, 2, 0.01f, 0.05f, 0.6f,
                 false, 0.10f, 0.30f, false, {},
                 [](const C& c, const C&) { return c.tuning.aids.absReleaseSlip; },
                 [](C& c, const C&, float v) {
                     c.tuning.aids.absReleaseSlip = std::max(v, c.tuning.aids.absApplySlip + 0.01f);
                 }});
    p.push_back({"aids.tcThreshold", "DRIVING AIDS", "TC slip threshold (on)", "", K::Number, 1.0f, 2, 0.01f, 0.05f, 1.0f,
                 false, 0.10f, 0.40f, false, {},
                 [](const C& c, const C&) { return c.tuning.aids.tcSlipThreshold; },
                 [](C& c, const C&, float v) { c.tuning.aids.tcSlipThreshold = v; }});
    p.push_back({"aids.tcOffRoadThreshold", "DRIVING AIDS", "TC slip threshold (off-road)", "", K::Number, 1.0f, 2, 0.01f,
                 0.05f, 1.0f, false, 0.20f, 0.60f, false, {},
                 [](const C& c, const C&) { return c.tuning.aids.tcOffRoadSlipThreshold; },
                 [](C& c, const C&, float v) { c.tuning.aids.tcOffRoadSlipThreshold = v; }});
    return p;
}

std::string fmt(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", v);
    return buf;
}

} // namespace

float sprungCornerMass(const VehicleConfig& c, int axleIndex) {
    const float front = c.wheelBase > 0.0f ? std::clamp(0.5f + c.comForward / c.wheelBase, 0.0f, 1.0f) : 0.5f;
    return 0.5f * c.mass * (axleIndex == kFrontAxle ? front : 1.0f - front);
}

float wheelRateFromFrequency(float frequencyHz, float cornerMass) {
    const float w = kTwoPi * frequencyHz;
    return w * w * cornerMass;
}

float frequencyFromWheelRate(float rateNPerM, float cornerMass) {
    return cornerMass > 0.0f && rateNPerM > 0.0f ? std::sqrt(rateNPerM / cornerMass) / kTwoPi : 0.0f;
}

float criticalDamping(float frequencyHz, float cornerMass) { return 2.0f * kTwoPi * frequencyHz * cornerMass; }

const std::vector<SetupParamInfo>& setupParams() {
    static const std::vector<SetupParamInfo> kParams = buildParams();
    return kParams;
}

const SetupParamInfo* findSetupParam(const std::string& key) {
    for (const SetupParamInfo& p : setupParams())
        if (key == p.key) return &p;
    return nullptr;
}

float setupValue(const SetupParamInfo& p, const VehicleConfig& config, const VehicleConfig& base) {
    return p.get(config, base);
}

ParamRange setupParamRange(const SetupParamInfo& p, const VehicleDefinition& def, const VehicleKit* kit,
                           const VehicleConfig& base) {
    if (p.kind == SetupParamKind::Choice) return {0.0f, float(p.choices.size() - 1)};
    const float b = p.get(base, base);
    ParamRange r;
    if (kit && kit->ranges.count(p.key)) r = kit->ranges.at(p.key);
    else if (def.garageRanges.count(p.key)) r = def.garageRanges.at(p.key);
    else if (p.relative) r = {b * p.defMin, b * p.defMax};
    else r = {p.defMin, p.defMax};
    r.min = std::min(std::max(r.min, p.hardMin), b);
    r.max = std::max(std::min(r.max, p.hardMax), b);
    return r;
}

bool buildSetupConfig(const VehicleDefinition& def, const VehicleSetup& setup, VehicleConfig& out,
                      std::vector<std::string>& warnings) {
    VehicleConfig base;
    std::vector<std::string> errors;
    if (!configWithKit(def, setup.kit, base, errors)) {
        for (const std::string& e : errors) warnings.push_back(e);
        warnings.push_back("setup '" + setup.name + "': kit '" + setup.kit + "' ignored");
        base = def.config;
    }
    const VehicleKit* kit = def.findKit(setup.kit);
    for (const auto& [key, v] : setup.values)
        if (!findSetupParam(key)) warnings.push_back("setup '" + setup.name + "': unknown parameter '" + key + "' ignored");
    VehicleConfig c = base;
    for (const SetupParamInfo& p : setupParams()) {
        const auto it = setup.values.find(p.key);
        if (it == setup.values.end()) continue;
        float v = it->second;
        if (!std::isfinite(v)) {
            warnings.push_back("setup '" + setup.name + "': " + p.key + " is not a number, ignored");
            continue;
        }
        const ParamRange r = setupParamRange(p, def, kit, base);
        if (p.kind == SetupParamKind::Choice) v = std::round(v);
        if (v < r.min || v > r.max) {
            warnings.push_back("setup '" + setup.name + "': " + p.key + " = " + fmt(v) + " clamped to [" + fmt(r.min) +
                               ", " + fmt(r.max) + "]");
            v = std::clamp(v, r.min, r.max);
        }
        p.set(c, base, v);
    }
    out = std::move(c);
    return true;
}

VehicleSetup* SetupFile::find(const std::string& name) {
    for (VehicleSetup& s : setups)
        if (s.name == name) return &s;
    return nullptr;
}

const VehicleSetup* SetupFile::find(const std::string& name) const {
    for (const VehicleSetup& s : setups)
        if (s.name == name) return &s;
    return nullptr;
}

std::string writeSetupFile(const SetupFile& f) {
    json::Value root = json::Value::makeObject();
    root.object["schema"] = json::Value::makeNumber(1);
    root.object["vehicle"] = json::Value::makeString(f.vehicleId);
    root.object["selected"] = json::Value::makeString(f.selected);
    json::Value list = json::Value::makeArray();
    for (const VehicleSetup& s : f.setups) {
        json::Value o = json::Value::makeObject();
        o.object["name"] = json::Value::makeString(s.name);
        o.object["kit"] = json::Value::makeString(s.kit);
        json::Value values = json::Value::makeObject();
        for (const auto& [k, v] : s.values) values.object[k] = json::Value::makeNumber(v);
        o.object["values"] = values;
        list.array.push_back(o);
    }
    root.object["setups"] = list;
    return json::write(root);
}

bool parseSetupFile(const std::string& text, SetupFile& out, std::string* error) {
    json::Value root;
    std::string e;
    if (!json::parse(text, root, &e)) {
        if (error) *error = "invalid JSON: " + e;
        return false;
    }
    if (!root.isObject() || root["schema"].numberOr(0) != 1 || root["vehicle"].type != json::Value::Type::String ||
        !root["setups"].isArray()) {
        if (error) *error = "expected {\"schema\": 1, \"vehicle\": ..., \"setups\": [...]}";
        return false;
    }
    SetupFile f;
    f.vehicleId = root["vehicle"].string;
    f.selected = root["selected"].stringOr("");
    for (const json::Value& s : root["setups"].array) {
        VehicleSetup v;
        v.name = s["name"].stringOr("");
        v.kit = s["kit"].stringOr("");
        if (v.name.empty() || f.find(v.name)) {
            if (error) *error = "setup names must be non-empty and unique";
            return false;
        }
        for (const auto& [k, x] : s["values"].object)
            if (x.isNumber()) v.values[k] = float(x.number);
        f.setups.push_back(std::move(v));
    }
    if (!f.selected.empty() && !f.find(f.selected)) f.selected.clear();
    out = std::move(f);
    return true;
}

bool loadSetupFile(const std::string& path, SetupFile& out, std::string* error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (error) *error = "cannot read " + path;
        return false;
    }
    std::stringstream ss;
    ss << in.rdbuf();
    return parseSetupFile(ss.str(), out, error);
}

bool saveSetupFile(const std::string& path, const SetupFile& f, std::string* error) {
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path p(path);
    if (p.has_parent_path()) fs::create_directories(p.parent_path(), ec);
    const std::string tmp = path + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) {
            if (error) *error = "cannot write " + tmp;
            return false;
        }
        out << writeSetupFile(f);
        if (!out) {
            if (error) *error = "write failed: " + tmp;
            return false;
        }
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        if (error) *error = "cannot rename " + tmp + ": " + ec.message();
        return false;
    }
    return true;
}

} // namespace tf
