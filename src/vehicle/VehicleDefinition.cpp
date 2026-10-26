#include "vehicle/VehicleDefinition.h"

#include "vehicle/VehicleBody.h"
#include "vehicle/VehicleSetup.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

namespace tf {
namespace {

constexpr const char* kSurfaceKeys[kSurfaceCount] = {"concrete", "packed_dirt", "rock", "low_grip",
                                                     "mud",      "grass",       "sand",     "snow"};

std::string fmt(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", v);
    return buf;
}

// Reads one JSON object, records which members were read and reports missing, mistyped, out-of-range and unknown
// members with their full path.
class Obj {
public:
    Obj(const json::Value* v, std::string path, std::string source, std::vector<std::string>& errors)
        : m_v(v), m_path(std::move(path)), m_source(std::move(source)), m_errors(&errors) {
        // A missing member was already reported by member().
        if (m_v != nullptr && !m_v->isObject()) {
            error(m_path.empty() ? std::string("document") : m_path, "must be an object");
            m_v = nullptr;
        }
    }
    ~Obj() {
        if (m_v == nullptr) return;
        for (const auto& [key, value] : m_v->object)
            if (!m_used.count(key)) error(child(key), "unknown key");
    }
    Obj(const Obj&) = delete;
    Obj& operator=(const Obj&) = delete;

    bool valid() const { return m_v != nullptr; }
    const std::string& path() const { return m_path; }
    std::string child(const std::string& key) const { return m_path.empty() ? key : m_path + "." + key; }
    void error(const std::string& path, const std::string& what) const {
        m_errors->push_back(m_source + ": " + path + ": " + what);
    }

    // Returns the member or nullptr (with an error when required).
    const json::Value* member(const std::string& key, bool required = true) {
        m_used.insert(key);
        if (m_v == nullptr) return nullptr;
        const auto it = m_v->object.find(key);
        if (it == m_v->object.end()) {
            if (required) error(child(key), "missing");
            return nullptr;
        }
        return &it->second;
    }

    void num(const std::string& key, float& out, double lo, double hi, const char* unit, bool required = true) {
        const json::Value* v = member(key, required);
        if (v == nullptr) return;
        if (!v->isNumber()) {
            error(child(key), "must be a number");
            return;
        }
        if (!(v->number >= lo && v->number <= hi)) {
            error(child(key), "= " + fmt(v->number) + " is out of range [" + fmt(lo) + ", " + fmt(hi) + "]" +
                                  (unit && *unit ? std::string(" ") + unit : std::string()));
            return;
        }
        out = float(v->number);
    }
    // Same, but keeps the double (for values derived in double precision).
    void numD(const std::string& key, double& out, double lo, double hi, const char* unit) {
        float tmp = 0.0f;
        const std::size_t before = m_errors->size();
        num(key, tmp, lo, hi, unit);
        if (m_errors->size() == before) {
            const json::Value* v = member(key);
            if (v) out = v->number;
        }
    }
    void boolean(const std::string& key, bool& out) {
        const json::Value* v = member(key);
        if (v == nullptr) return;
        if (v->type != json::Value::Type::Bool) error(child(key), "must be true or false");
        else out = v->boolean;
    }
    void str(const std::string& key, std::string& out, bool required = true) {
        const json::Value* v = member(key, required);
        if (v == nullptr) return;
        if (v->type != json::Value::Type::String) error(child(key), "must be a string");
        else out = v->string;
    }
    template <typename E>
    void choice(const std::string& key, E& out, std::initializer_list<std::pair<const char*, E>> names) {
        const json::Value* v = member(key);
        if (v == nullptr) return;
        std::string list;
        for (const auto& [n, e] : names) {
            if (v->type == json::Value::Type::String && v->string == n) {
                out = e;
                return;
            }
            list += list.empty() ? n : std::string(", ") + n;
        }
        error(child(key), "must be one of: " + list);
    }
    const json::Value* raw() const { return m_v; }

private:
    const json::Value* m_v;
    std::string m_path, m_source;
    std::vector<std::string>* m_errors;
    std::set<std::string> m_used;
};

const std::initializer_list<std::pair<const char*, DiffMode>> kDiffNames = {
    {"open", DiffMode::Open}, {"lsd", DiffMode::LimitedSlip}, {"locked", DiffMode::Locked}};

void readAxle(Obj& axles, const char* key, AxleConfig& a, AxleTuning& t, bool front, const std::string& source,
              std::vector<std::string>& errors) {
    Obj o(axles.member(key), axles.child(key), source, errors);
    if (!o.valid()) return;
    o.num("track", a.track, 1.0, 2.6, "m");
    {
        Obj w(o.member("wheel"), o.child("wheel"), source, errors);
        w.num("radius", a.wheelRadius, 0.2, 0.8, "m");
        w.num("width", a.wheelWidth, 0.1, 0.6, "m");
        w.num("mass", a.wheelMass, 5.0, 200.0, "kg");
        w.num("inertia", a.wheelInertia, 0.0, 50.0, "kg*m^2 (0 = solid disc)");
        w.num("angularDamping", a.wheelAngularDamping, 0.0, 5.0, "1/s");
        w.num("rimRatio", a.rimRatio, 0.35, 0.85, "(rim / tire diameter, visual)", false);
        w.str("tread", a.tread, false);
        if (a.tread != "mud_terrain" && a.tread != "all_terrain")
            w.error(w.child("tread"), "unknown tread '" + a.tread + "' (known: mud_terrain, all_terrain)");
    }
    {
        Obj s(o.member("suspension"), o.child("suspension"), source, errors);
        s.num("frequency", t.springFrequency, 0.5, 5.0, "Hz");
        s.num("compressionDamping", t.compressionDamping, 0.05, 2.0, "(damping ratio)");
        s.num("reboundDamping", t.reboundDamping, 0.05, 2.0, "(damping ratio)");
        s.num("preload", a.suspensionPreload, 0.0, 0.3, "m");
        s.num("minLength", a.suspensionMinLength, 0.0, 1.0, "m");
        s.num("maxLength", a.suspensionMaxLength, 0.05, 1.5, "m");
        s.num("antiRoll", t.antiRoll, 0.0, 300000.0, "N/m");
        if (a.suspensionMaxLength <= a.suspensionMinLength + 0.02f)
            s.error(s.child("maxLength"), "must exceed minLength by at least 0.02 m (travel)");
        Obj b(s.member("bumpStop"), s.child("bumpStop"), source, errors);
        b.num("range", t.bumpStopRange, 0.0, 0.5, "m");
        b.num("frequencyScale", t.bumpStopFrequencyScale, 1.0, 10.0, "");
        if (t.bumpStopRange > a.suspensionMaxLength - a.suspensionMinLength)
            b.error(b.child("range"), "must not exceed the suspension travel");
    }
    o.num("brakeTorque", t.brakeTorque, 0.0, 20000.0, "N*m per wheel");
    o.num("handBrakeTorque", t.handBrakeTorque, 0.0, 20000.0, "N*m per wheel");
    a.maxSteerAngleDeg = 0.0f;
    (void)front;
}

std::string readFile(const std::string& path, bool& ok) {
    std::ifstream f(path, std::ios::binary);
    ok = bool(f);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool readRanges(const json::Value* v, const std::string& path, const std::string& source,
                std::map<std::string, ParamRange>& out, std::vector<std::string>& errors) {
    if (v == nullptr) return true;
    if (!v->isObject()) {
        errors.push_back(source + ": " + path + ": must be an object");
        return false;
    }
    bool ok = true;
    for (const auto& [key, r] : v->object) {
        const SetupParamInfo* p = findSetupParam(key);
        const std::string where = source + ": " + path + "." + key + ": ";
        if (p == nullptr) {
            errors.push_back(where + "unknown garage parameter");
            ok = false;
            continue;
        }
        if (p->kind != SetupParamKind::Number) {
            errors.push_back(where + "has fixed choices, no range");
            ok = false;
            continue;
        }
        if (!r.isArray() || r.array.size() != 2 || !r.array[0].isNumber() || !r.array[1].isNumber() ||
            !(r.array[0].number < r.array[1].number)) {
            errors.push_back(where + "must be [min, max] with min < max");
            ok = false;
            continue;
        }
        if (r.array[0].number < p->hardMin || r.array[1].number > p->hardMax) {
            errors.push_back(where + "[" + fmt(r.array[0].number) + ", " + fmt(r.array[1].number) +
                             "] exceeds the allowed [" + fmt(p->hardMin) + ", " + fmt(p->hardMax) + "]");
            ok = false;
            continue;
        }
        out[key] = {float(r.array[0].number), float(r.array[1].number)};
    }
    return ok;
}

void readCombustion(Obj& e, VehicleConfig& c) {
    VehicleTuning& t = c.tuning;
    e.num("maxTorque", t.engineMaxTorque, 50.0, 3000.0, "N*m");
    e.num("idleRPM", c.engineMinRPM, 400.0, 1500.0, "rpm");
    e.num("maxRPM", c.engineMaxRPM, 2000.0, 12000.0, "rpm");
    e.num("revLimitRPM", t.revLimitRPM, 1000.0, 12000.0, "rpm");
    e.num("inertia", c.engineInertia, 0.05, 5.0, "kg*m^2");
    e.num("angularDamping", c.engineAngularDamping, 0.0, 2.0, "");
    if (c.engineMaxRPM < c.engineMinRPM + 1000.0f) e.error("engine.maxRPM", "must be at least idleRPM + 1000");
    if (t.revLimitRPM <= c.engineMinRPM + 500.0f || t.revLimitRPM > c.engineMaxRPM)
        e.error("engine.revLimitRPM", "must be in (idleRPM + 500, maxRPM]");
    if (const json::Value* curve = e.member("torqueCurve")) {
        bool ok = curve->isArray() && curve->array.size() >= 2;
        std::vector<std::pair<float, float>> pts;
        double lastX = -1.0;
        if (ok) {
            for (const json::Value& p : curve->array) {
                if (!p.isArray() || p.array.size() != 2 || !p.array[0].isNumber() || !p.array[1].isNumber()) {
                    ok = false;
                    break;
                }
                const double x = p.array[0].number, y = p.array[1].number;
                if (!(x > lastX) || x < 0.0 || x > 1.0 || y < 0.0 || y > 1.5) {
                    ok = false;
                    break;
                }
                lastX = x;
                pts.emplace_back(float(x), float(y));
            }
        }
        if (ok && (pts.front().first != 0.0f || pts.back().first != 1.0f)) ok = false;
        if (!ok)
            e.error("engine.torqueCurve", "must be >= 2 points [rpmFraction, torqueFraction] with rpmFraction rising "
                                          "from 0 to 1 and torqueFraction in [0, 1.5]");
        else
            c.torqueCurve = std::move(pts);
    }
}

void readMotor(Obj& motors, const char* key, MotorConfig& m, const std::string& source, std::vector<std::string>& errors) {
    Obj o(motors.member(key), motors.child(key), source, errors);
    if (!o.valid()) return;
    o.num("maxTorque", m.maxTorque, 20.0, 2000.0, "N*m at the motor shaft");
    float kw = m.maxPower * 1e-3f;
    o.num("maxPowerKW", kw, 5.0, 1000.0, "kW at the motor shaft");
    m.maxPower = kw * 1000.0f;
    o.num("maxRPM", m.maxRPM, 3000.0, 30000.0, "rpm");
    o.num("reduction", m.reduction, 3.0, 30.0, "(motor / axle speed)");
    o.num("rotorInertia", m.rotorInertia, 0.0, 1.0, "kg*m^2 at the motor shaft");
    o.num("regenTorque", m.regenTorque, 0.0, 2000.0, "N*m at the motor shaft");
    if (m.regenTorque > m.maxTorque) o.error(o.child("regenTorque"), "must not exceed maxTorque");
    // The constant-power range must start below the maximum speed.
    if (m.maxPower / m.maxTorque >= m.maxRPM * 2.0f * 3.14159265f / 60.0f)
        o.error(o.child("maxPowerKW"), "base speed maxPower / maxTorque must be below maxRPM");
}

void readElectric(Obj& e, ElectricConfig& c, const std::string& source, std::vector<std::string>& errors) {
    {
        Obj m(e.member("motors"), e.child("motors"), source, errors);
        if (m.valid()) {
            readMotor(m, "front", c.motors[kFrontAxle], source, errors);
            readMotor(m, "rear", c.motors[kRearAxle], source, errors);
        }
    }
    {
        Obj b(e.member("battery"), e.child("battery"), source, errors);
        b.num("capacityKWh", c.batteryCapacityKWh, 5.0, 300.0, "kWh (usable)");
        b.num("initialSoC", c.initialSoC, 0.0, 1.0, "(0..1)");
        b.num("auxPowerKW", c.auxPowerKW, 0.0, 20.0, "kW");
        b.num("maxChargePowerKW", c.regenMaxPowerKW, 0.0, 1000.0, "kW (regen limit of both motors)");
    }
    {
        Obj f(e.member("efficiency"), e.child("efficiency"), source, errors);
        f.num("drive", c.driveEfficiency, 0.5, 1.0, "(battery -> shaft)");
        f.num("regen", c.regenEfficiency, 0.0, 1.0, "(shaft -> battery)");
    }
    {
        Obj r(e.member("regen"), e.child("regen"), source, errors);
        r.num("liftOffDecel", c.liftOffDecel, 0.0, 4.0, "m/s^2 (both pedals released)");
        r.num("fadeSpeed", c.regenFadeSpeed, 0.0, 10.0, "m/s");
    }
}

void readGearbox(Obj& root, VehicleConfig& c, const std::string& source, std::vector<std::string>& errors) {
    VehicleTuning& t = c.tuning;
    Obj g(root.member("gearbox"), "gearbox", source, errors);
    std::string mode;
    g.str("mode", mode);
    if (mode != "auto") g.error("gearbox.mode", "only \"auto\" is supported");
    auto ratios = [&](const char* key, std::vector<float>& out, bool reverse) {
        const json::Value* v = g.member(key);
        if (v == nullptr) return;
        std::vector<float> r;
        bool ok = v->isArray() && !v->array.empty() && v->array.size() <= 10;
        for (std::size_t i = 0; ok && i < v->array.size(); ++i) {
            const double x = v->array[i].numberOr(0.0);
            if (reverse ? !(x <= -0.3 && x >= -8.0) : !(x >= 0.3 && x <= 8.0)) ok = false;
            else if (!reverse && i > 0 && !(x < r.back())) ok = false;
            else r.push_back(float(x));
        }
        if (!ok)
            g.error(g.child(key), reverse ? "must be 1-10 negative ratios in [-8, -0.3]"
                                          : "must be 1-10 falling ratios in [0.3, 8]");
        else
            out = std::move(r);
    };
    ratios("ratios", c.gearRatios, false);
    ratios("reverseRatios", c.reverseRatios, true);
    g.num("finalDrive", t.finalDrive, 1.5, 8.0, "");
    g.num("lowRange", c.lowRangeRatio, 1.0, 4.0, "(1 = no low range)");
    g.num("shiftUpRPM", c.shiftUpRPM, 1000.0, 12000.0, "rpm");
    g.num("shiftDownRPM", c.shiftDownRPM, 500.0, 12000.0, "rpm");
    g.num("shiftTime", c.shiftTime, 0.0, 2.0, "s");
    g.num("clutchReleaseTime", c.clutchReleaseTime, 0.0, 2.0, "s");
    g.num("shiftLatency", c.shiftLatency, 0.0, 3.0, "s");
    g.num("clutchStrength", t.clutchStrength, 1.0, 100.0, "kg*m^2/s");
    g.boolean("launchClutchLimit", t.launchClutchLimit);
    g.num("creepTorque", t.creepTorque, 0.0, 200.0, "N*m");
    if (c.shiftUpRPM > c.engineMaxRPM || c.shiftUpRPM <= c.shiftDownRPM)
        g.error("gearbox.shiftUpRPM", "must be in (shiftDownRPM, engine.maxRPM]");
    if (c.shiftDownRPM < c.engineMinRPM) g.error("gearbox.shiftDownRPM", "must be at least engine.idleRPM");
}

void readDrivetrain(Obj& d, VehicleConfig& c) {
    VehicleTuning& t = c.tuning;
    d.choice("layout", c.layout,
             {{"rwd", DriveLayout::Rwd}, {"fwd", DriveLayout::Fwd}, {"awd", DriveLayout::Awd},
              {"part_time", DriveLayout::PartTime}, {"on_demand", DriveLayout::OnDemand},
              {"dual_motor", DriveLayout::DualMotor}});
    const bool electric = c.engineType == EngineType::Electric;
    if (electric != (c.layout == DriveLayout::DualMotor))
        d.error("drivetrain.layout", electric ? "an electric vehicle needs \"dual_motor\""
                                              : "\"dual_motor\" needs engine.type \"electric\"");
    d.num("frontTorqueShare", t.frontTorqueShare, 0.0, 1.0, "");
    d.choice("frontDiff", t.frontDiff, kDiffNames);
    d.choice("rearDiff", t.rearDiff, kDiffNames);
    d.num("limitedSlipRatio", t.limitedSlipRatio, 1.01, 5.0, "");
    d.num("lockedSlipRatio", t.lockedSlipRatio, 1.001, 1.5, "");
    if (electric) {
        // No center differential: the split between the motors is electronic.
        d.num("offRoadFrontTorqueShare", c.electric.offRoadFrontShare, 0.0, 1.0, "(drive mode OFF-ROAD)");
        if (t.frontTorqueShare <= 0.0f || t.frontTorqueShare >= 1.0f)
            d.error("drivetrain.frontTorqueShare", "must be strictly between 0 and 1 for dual_motor");
    } else {
        d.choice("centerDiff", t.centerDiff, kDiffNames);
        d.num("centerLimitedSlipRatio", t.centerLimitedSlipRatio, 1.01, 5.0, "");
    }
    if ((c.layout == DriveLayout::Awd || c.layout == DriveLayout::OnDemand) &&
        (t.frontTorqueShare <= 0.0f || t.frontTorqueShare >= 1.0f))
        d.error("drivetrain.frontTorqueShare", "must be strictly between 0 and 1 for awd / on_demand");
}

void readAids(Obj& a, DrivingAidTuning& g) {
    a.boolean("absEnabled", g.absEnabled);
    a.choice("traction", g.traction,
             {{"off", TractionMode::Off}, {"on", TractionMode::On}, {"off_road", TractionMode::OffRoad}});
    a.num("absReleaseSlip", g.absReleaseSlip, 0.02, 0.6, "");
    a.num("absApplySlip", g.absApplySlip, 0.0, 0.5, "");
    if (g.absApplySlip >= g.absReleaseSlip) a.error("aids.absApplySlip", "must be below absReleaseSlip");
    a.num("absReleaseRate", g.absReleaseRate, 1.0, 200.0, "1/s");
    a.num("absApplyRate", g.absApplyRate, 1.0, 200.0, "1/s");
    a.num("absMinPressure", g.absMinPressure, 0.0, 1.0, "");
    a.num("absMinSpeed", g.absMinSpeed, 0.0, 10.0, "m/s");
    a.num("tcSlipThreshold", g.tcSlipThreshold, 0.02, 2.0, "");
    a.num("tcOffRoadSlipThreshold", g.tcOffRoadSlipThreshold, 0.02, 2.0, "");
    a.num("tcMinSpeed", g.tcMinSpeed, 0.1, 10.0, "m/s");
    a.num("tcSlipFilterTime", g.tcSlipFilterTime, 0.0, 1.0, "s");
    a.num("tcBrakeGain", g.tcBrakeGain, 0.0, 50000.0, "N*m");
    a.num("tcMaxBrakeTorque", g.tcMaxBrakeTorque, 0.0, 20000.0, "N*m");
    a.num("tcBrakeRate", g.tcBrakeRate, 1.0, 1.0e6, "N*m/s");
    a.num("tcCutRate", g.tcCutRate, 0.0, 50.0, "1/s");
    a.num("tcRecoverRate", g.tcRecoverRate, 0.0, 50.0, "1/s");
    a.num("tcMinThrottle", g.tcMinThrottle, 0.0, 1.0, "");
    a.num("tcOffRoadMinThrottle", g.tcOffRoadMinThrottle, 0.0, 1.0, "");
    a.num("tcSpinSpeed", g.tcSpinSpeed, 0.0, 5.0, "m/s");
    a.num("tcRollbackSpeed", g.tcRollbackSpeed, 0.0, 5.0, "m/s");
}

} // namespace

bool knownVisualModel(const std::string& id) { return id == "trailforge_suv" || id == "trailforge_ev"; }
bool knownAudioProfile(const std::string& id) {
    TireSoundProfile p;
    return tireSoundPreset(id, p);
}

const VehicleKit* VehicleDefinition::findKit(const std::string& kitId) const {
    for (const VehicleKit& k : kits)
        if (k.id == kitId) return &k;
    return nullptr;
}

bool parseVehicleConfig(const json::Value& doc, const std::string& source, VehicleConfig& out,
                        std::vector<std::string>& errors) {
    const std::size_t errorsBefore = errors.size();
    VehicleConfig c;
    VehicleTuning& t = c.tuning;
    {
        Obj root(&doc, "", source, errors);
        if (!root.valid()) return false;
        float schema = 0.0f;
        root.num("schema", schema, 1.0, 1.0, "(format version)");
        root.str("id", c.id);
        root.str("name", c.name);
        std::string description;
        root.str("description", description, false);
        root.str("visualModel", c.visualModel);
        root.str("bodyShape", c.bodyShape, false);
        root.str("audioProfile", c.audioProfile);
        if (c.id.empty() || c.id.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_") != std::string::npos)
            root.error("id", "must be non-empty lowercase letters, digits and '_'");
        if (!c.visualModel.empty() && !knownVisualModel(c.visualModel))
            root.error("visualModel", "unknown model '" + c.visualModel + "' (known: trailforge_suv, trailforge_ev)");
        if (c.bodyShape != "kit" && c.bodyShape != "traced")
            root.error("bodyShape", "unknown shape '" + c.bodyShape + "' (known: kit, traced)");
        if (!c.audioProfile.empty() && !knownAudioProfile(c.audioProfile))
            root.error("audioProfile", "unknown profile '" + c.audioProfile + "' (known: trailforge_suv, ev, sporty)");
        c.tireSound = tireSoundPreset(c.audioProfile);
        if (root.member("tireSound", false)) {
            TireSoundProfile& ts = c.tireSound;
            Obj s(root.member("tireSound", false), "tireSound", source, errors);
            s.num("squealPitch", ts.squealPitch, 0.5, 1.5, "(playback rate factor)", false);
            s.num("squealGain", ts.squealGain, 0.0, 2.0, "", false);
            s.num("lowpassMinHz", ts.lowpassMinHz, 300.0, 20000.0, "Hz", false);
            s.num("lowpassMaxHz", ts.lowpassMaxHz, 300.0, 20000.0, "Hz", false);
            s.num("scrubGain", ts.scrubGain, 0.0, 2.0, "", false);
            s.num("scrubLowHz", ts.scrubLowHz, 20.0, 2000.0, "Hz", false);
            s.num("scrubHighHz", ts.scrubHighHz, 100.0, 8000.0, "Hz", false);
            s.num("treadPitch", ts.treadPitch, 0.01, 0.3, "m", false);
            s.num("roadGain", ts.roadGain, 0.0, 2.0, "", false);
            if (ts.lowpassMaxHz < ts.lowpassMinHz) s.error(s.child("lowpassMaxHz"), "must not be below lowpassMinHz");
            if (ts.scrubHighHz <= ts.scrubLowHz) s.error(s.child("scrubHighHz"), "must exceed scrubLowHz");
        }
        if (const json::Value* cam = root.member("camera", false)) {
            Obj co(cam, "camera", source, errors);
            if (const json::Value* hv = co.member("hood", false)) {
                Obj h(hv, "camera.hood", source, errors);
                if (const json::Value* pos = h.member("position", false)) {
                    if (!pos->isArray() || pos->array.size() != 3) {
                        h.error("camera.hood.position", "must be [x, y, z] in m (body space)");
                    } else {
                        const double lim[3][2] = {{-1.5, 1.5}, {0.0, 3.0}, {-3.0, 3.0}};
                        for (int i = 0; i < 3; ++i) {
                            const double v = pos->array[std::size_t(i)].numberOr(1e9);
                            if (!(v >= lim[i][0] && v <= lim[i][1]))
                                h.error("camera.hood.position", "x in [-1.5, 1.5], y in [0, 3], z in [-3, 3] m");
                            else c.hoodCamera[std::size_t(i)] = float(v);
                        }
                    }
                }
                h.num("pitch", c.hoodCameraPitch, -30.0, 30.0, "deg", false);
                h.num("nearPlane", c.hoodCameraNear, 0.01, 0.3, "m", false);
            }
        }
        root.member("garage", false); // read by parseVehicleDefinition
        root.member("kits", false);

        // Geometry first: the center of mass is given relative to the axles.
        double wheelBase = c.wheelBase;
        {
            Obj g(root.member("geometry"), "geometry", source, errors);
            g.numD("wheelBase", wheelBase, 1.5, 5.0, "m");
            c.wheelBase = float(wheelBase);
            g.num("attachHeight", c.attachHeight, -0.5, 0.5, "m (body space)");
        }
        {
            Obj b(root.member("body"), "body", source, errors);
            b.num("mass", c.mass, 300.0, 20000.0, "kg");
            b.num("inertiaScale", c.inertiaScale, 0.2, 5.0, "");
            if (const json::Value* in = b.member("inertia")) {
                if (in->isNull()) {
                    c.inertia = {0.0f, 0.0f, 0.0f};
                } else if (!in->isArray() || in->array.size() != 3) {
                    b.error("body.inertia", "must be null or [Ixx, Iyy, Izz] in kg*m^2");
                } else {
                    for (int i = 0; i < 3; ++i) {
                        const double v = in->array[std::size_t(i)].numberOr(-1.0);
                        if (!(v >= 10.0 && v <= 1.0e6)) b.error("body.inertia", "each moment must be in [10, 1e6] kg*m^2");
                        else c.inertia[std::size_t(i)] = float(v);
                    }
                }
            }
            {
                Obj m(b.member("centerOfMass"), "body.centerOfMass", source, errors);
                double front = 0.5;
                m.numD("frontWeightFraction", front, 0.2, 0.8, "(static load share of the front axle)");
                // Static axle loads with the center of mass at z: front share = (z + wb/2) / wb.
                c.comForward = float(front * wheelBase - 0.5 * wheelBase);
                m.num("height", c.comHeight, -1.0, 1.0, "m (body space Y)");
            }
            b.num("friction", c.bodyFriction, 0.0, 2.0, "");
            b.num("angularDamping", c.bodyAngularDamping, 0.0, 1.0, "1/s");
            std::string model;
            b.str("model", model, false);
            if (!model.empty()) {
                // Next to the vehicle.json; no paths outside its folder.
                const std::filesystem::path rel(model);
                if (rel.is_absolute() || model.find("..") != std::string::npos || rel.extension() != ".glb") {
                    b.error("body.model", "must be a .glb file name in the vehicle folder");
                } else {
                    c.bodyModel = (std::filesystem::path(source).parent_path() / rel).lexically_normal().string();
                    BodyModel body;
                    std::string err;
                    if (!loadBodyModel(c.bodyModel, body, false, &err)) b.error("body.model", err);
                    else if (body.hulls.empty()) b.error("body.model", c.bodyModel + ": no collision pieces (COL_*)");
                    for (const BodyHull& h : body.hulls) c.bodyHulls.push_back(h.points);
                }
            }
            if (const json::Value* paint = b.member("paint", false)) {
                if (!paint->isObject()) b.error("body.paint", "must be an object");
                else
                    for (const auto& [role, v] : paint->object) {
                        if (role != "paint_primary" && role != "paint_secondary" && role != "paint_accent") {
                            b.error("body.paint." + role, "unknown role (paint_primary, paint_secondary, paint_accent)");
                            continue;
                        }
                        std::array<float, 3> rgb{};
                        bool ok = v.isArray() && v.array.size() == 3;
                        for (std::size_t i = 0; ok && i < 3; ++i) {
                            rgb[i] = float(v.array[i].numberOr(-1.0));
                            ok = rgb[i] >= 0.0f && rgb[i] <= 1.0f;
                        }
                        if (!ok) b.error("body.paint." + role, "must be [r, g, b] in sRGB 0..1");
                        else c.bodyPaint[role] = rgb;
                    }
                if (c.bodyModel.empty() && !c.bodyPaint.empty())
                    b.error("body.paint", "needs body.model (the procedural bodies have fixed liveries)");
            }
        }
        {
            Obj ax(root.member("axles"), "axles", source, errors);
            if (ax.valid()) {
                readAxle(ax, "front", c.axles[kFrontAxle], t.axles[kFrontAxle], true, source, errors);
                readAxle(ax, "rear", c.axles[kRearAxle], t.axles[kRearAxle], false, source, errors);
            }
        }
        {
            Obj ti(root.member("tires"), "tires", source, errors);
            {
                Obj l(ti.member("longitudinal"), "tires.longitudinal", source, errors);
                l.num("peakSlip", t.tirePeakSlipRatio, 0.02, 0.5, "(slip ratio)");
                l.num("peakFriction", t.tirePeakLongFriction, 0.3, 2.0, "(mu)");
                l.num("slideFriction", t.tireSlideLongFriction, 0.1, 2.0, "(mu)");
                if (t.tireSlideLongFriction > t.tirePeakLongFriction) l.error(l.child("slideFriction"), "must not exceed peakFriction");
            }
            {
                Obj l(ti.member("lateral"), "tires.lateral", source, errors);
                l.num("peakSlipAngleDeg", t.tirePeakSlipAngleDeg, 1.0, 20.0, "deg");
                l.num("peakFriction", t.tirePeakLatFriction, 0.3, 2.0, "(mu)");
                l.num("slideFriction", t.tireSlideLatFriction, 0.1, 2.0, "(mu)");
                if (t.tireSlideLatFriction > t.tirePeakLatFriction) l.error(l.child("slideFriction"), "must not exceed peakFriction");
            }
            ti.boolean("staticGrip", t.tireStaticGrip);
            ti.num("slipSpeedFloor", t.tireSlipSpeedFloor, 0.0, 5.0, "m/s");
            ti.boolean("frictionEllipse", t.tireFrictionEllipse);
            Obj sg(ti.member("surfaceGrip"), "tires.surfaceGrip", source, errors);
            for (std::size_t i = 0; i < kSurfaceCount; ++i) sg.num(kSurfaceKeys[i], t.tireSurfaceGrip[i], 0.2, 2.0, "(multiplier)");
        }
        {
            Obj s(root.member("steering"), "steering", source, errors);
            s.num("maxAngleDeg", c.axles[kFrontAxle].maxSteerAngleDeg, 10.0, 60.0, "deg");
            s.num("rate", t.steerRate, 0.3, 20.0, "1/s");
            s.num("returnRate", t.steerReturnRate, 0.3, 20.0, "1/s");
            s.num("ackermann", t.ackermann, 0.0, 1.0, "(0 parallel .. 1 full Ackermann)");
        }
        {
            Obj e(root.member("engine"), "engine", source, errors);
            e.choice("type", c.engineType, {{"combustion", EngineType::Combustion}, {"electric", EngineType::Electric}});
            if (c.engineType == EngineType::Electric) readElectric(e, c.electric, source, errors);
            else readCombustion(e, c);
        }
        if (c.engineType == EngineType::Electric) {
            if (root.member("gearbox", false))
                root.error("gearbox", "an electric drivetrain has no gearbox (single fixed reduction per motor)");
        } else {
            readGearbox(root, c, source, errors);
        }
        {
            Obj d(root.member("drivetrain"), "drivetrain", source, errors);
            readDrivetrain(d, c);
        }
        {
            Obj a(root.member("aero"), "aero", source, errors);
            a.num("dragArea", t.dragArea, 0.0, 3.0, "m^2 (Cd x frontal area)");
            a.num("airDensity", t.airDensity, 0.9, 1.4, "kg/m^3");
        }
        {
            Obj a(root.member("aids"), "aids", source, errors);
            readAids(a, t.aids);
        }
    }
    if (errors.size() != errorsBefore) return false;
    out = std::move(c);
    return true;
}

bool parseVehicleDefinition(const std::string& text, const std::string& source, VehicleDefinition& out,
                            std::vector<std::string>& errors) {
    const std::size_t errorsBefore = errors.size();
    VehicleDefinition d;
    d.sourcePath = source;
    std::string parseError;
    if (!json::parse(text, d.document, &parseError)) {
        errors.push_back(source + ": invalid JSON: " + parseError);
        return false;
    }
    if (!parseVehicleConfig(d.document, source, d.config, errors)) return false;
    d.id = d.config.id;
    d.name = d.config.name;
    d.description = d.document["description"].stringOr("");

    const json::Value& garage = d.document["garage"];
    if (!garage.isNull()) {
        Obj g(&garage, "garage", source, errors);
        readRanges(g.member("ranges", false), "garage.ranges", source, d.garageRanges, errors);
    }
    const json::Value& kits = d.document["kits"];
    if (!kits.isNull() && !kits.isArray()) errors.push_back(source + ": kits: must be an array");
    if (kits.isArray()) {
        std::set<std::string> ids;
        for (std::size_t i = 0; i < kits.array.size(); ++i) {
            const std::string path = "kits[" + std::to_string(i) + "]";
            VehicleKit k;
            {
                Obj o(&kits.array[i], path, source, errors);
                if (!o.valid()) continue;
                o.str("id", k.id);
                o.str("name", k.name);
                o.str("description", k.description, false);
                o.num("massDelta", k.massDelta, -2000.0, 5000.0, "kg");
                if (const json::Value* ov = o.member("overrides", false)) {
                    if (!ov->isObject()) o.error(path + ".overrides", "must be an object (partial vehicle.json)");
                    else k.overrides = *ov;
                    if (ov->has("id") || ov->has("kits") || ov->has("garage"))
                        o.error(path + ".overrides", "must not change id, kits or garage");
                }
                readRanges(o.member("ranges", false), path + ".ranges", source, k.ranges, errors);
                if (const json::Value* vp = o.member("visualParts", false)) {
                    if (!vp->isArray()) o.error(path + ".visualParts", "must be an array of strings");
                    else
                        for (const json::Value& s : vp->array) k.visualParts.push_back(s.stringOr(""));
                }
            }
            if (k.id.empty() || !ids.insert(k.id).second) {
                errors.push_back(source + ": " + path + ".id: must be non-empty and unique");
                continue;
            }
            d.kits.push_back(std::move(k));
        }
    }
    if (errors.size() != errorsBefore) return false;
    for (const VehicleKit& k : d.kits) {
        VehicleConfig tmp;
        configWithKit(d, k.id, tmp, errors);
    }
    if (errors.size() != errorsBefore) return false;
    out = std::move(d);
    return true;
}

bool configWithKit(const VehicleDefinition& def, const std::string& kitId, VehicleConfig& out,
                   std::vector<std::string>& errors) {
    if (kitId.empty()) {
        out = def.config;
        return true;
    }
    const VehicleKit* k = def.findKit(kitId);
    if (k == nullptr) {
        errors.push_back(def.sourcePath + ": unknown kit '" + kitId + "'");
        return false;
    }
    json::Value doc = def.document;
    if (k->overrides.isObject()) json::merge(doc, k->overrides);
    VehicleConfig c;
    if (!parseVehicleConfig(doc, def.sourcePath + " (kit " + k->id + ")", c, errors)) return false;
    c.mass += k->massDelta;
    if (c.mass < 300.0f) {
        errors.push_back(def.sourcePath + " (kit " + k->id + "): body.mass + massDelta is below 300 kg");
        return false;
    }
    out = std::move(c);
    return true;
}

bool loadVehicleDefinition(const std::string& path, VehicleDefinition& out, std::vector<std::string>& errors) {
    bool ok = false;
    const std::string text = readFile(path, ok);
    if (!ok) {
        errors.push_back(path + ": cannot read");
        return false;
    }
    return parseVehicleDefinition(text, path, out, errors);
}

std::vector<VehicleDefinition> loadVehicleCatalog(const std::string& dir, std::vector<std::string>& errors) {
    namespace fs = std::filesystem;
    std::vector<VehicleDefinition> out;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) {
        errors.push_back(dir + ": not a directory");
        return out;
    }
    std::vector<fs::path> files;
    for (const fs::directory_entry& e : fs::directory_iterator(dir, ec)) {
        const fs::path f = e.path() / "vehicle.json";
        if (e.is_directory() && fs::is_regular_file(f)) files.push_back(f);
    }
    std::sort(files.begin(), files.end());
    for (const fs::path& f : files) {
        VehicleDefinition d;
        if (!loadVehicleDefinition(f.string(), d, errors)) continue;
        const std::string folder = f.parent_path().filename().string();
        if (d.id != folder) {
            errors.push_back(f.string() + ": id '" + d.id + "' must match its folder name '" + folder + "'");
            continue;
        }
        out.push_back(std::move(d));
    }
    std::stable_partition(out.begin(), out.end(), [](const VehicleDefinition& d) { return d.id == kDefaultVehicleId; });
    return out;
}

} // namespace tf
