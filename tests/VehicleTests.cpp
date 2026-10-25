// Vehicle definitions, drivetrain layouts and garage setups (headless). Usage: trailforge_vehicle_tests <test_name>

#include "sim/Simulation.h"
#include "vehicle/VehicleDefinition.h"
#include "vehicle/VehicleSetup.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace tf;

namespace {

int g_failures = 0;

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("  FAIL %s:%d: %s — ", __FILE__, __LINE__, #cond);      \
            std::printf(__VA_ARGS__);                                           \
            std::printf("\n");                                                  \
            ++g_failures;                                                       \
        }                                                                       \
    } while (0)

const std::string kAssets = TRAILFORGE_SOURCE_DIR "/assets/vehicles";
const std::string kTestData = TRAILFORGE_SOURCE_DIR "/tests/data/vehicles";

std::vector<VehicleDefinition> allDefinitions() {
    std::vector<std::string> errors;
    std::vector<VehicleDefinition> defs = loadVehicleCatalog(kAssets, errors);
    const std::size_t shipped = defs.size();
    for (VehicleDefinition& d : loadVehicleCatalog(kTestData, errors)) defs.push_back(std::move(d));
    for (const std::string& e : errors) std::printf("    %s\n", e.c_str());
    CHECK(errors.empty(), "%zu definition errors", errors.size());
    CHECK(shipped >= 1, "no shipped vehicle in %s", kAssets.c_str());
    return defs;
}

VehicleDefinition loadOne(const std::string& dir, const std::string& id) {
    VehicleDefinition d;
    std::vector<std::string> errors;
    const bool ok = loadVehicleDefinition(dir + "/" + id + "/vehicle.json", d, errors);
    for (const std::string& e : errors) std::printf("    %s\n", e.c_str());
    CHECK(ok, "cannot load %s", id.c_str());
    return d;
}

SimulationSettings settings(const VehicleConfig& c, TrackLayout layout) {
    SimulationSettings s;
    s.layout = layout;
    s.vehicle = c;
    return s;
}

void run(Simulation& sim, double seconds, const DriverIntent& intent = {}) {
    sim.setIntent(intent);
    const int n = int(std::lround(seconds / sim.fixedStep()));
    for (int i = 0; i < n; ++i) sim.tick();
}

JPH::Vec3 position(const Simulation& sim) { return sim.currentSnapshot().body.position; }
float headingDeg(const Simulation& sim) {
    const JPH::Vec3 f = sim.currentSnapshot().body.rotation.RotateAxisZ();
    return std::atan2(f.GetX(), f.GetZ()) * 57.2958f;
}

// ---------------------------------------------------------------------------

// Every shipped and test definition: settle, acceleration, braking, a 15 degree climb, suspension travel.
void vehicleCatalogSweep() {
    for (const VehicleDefinition& def : allDefinitions()) {
        const VehicleConfig& c = def.config;
        std::printf("  %s (%s, %s, %.0f kg)\n", def.id.c_str(), def.name.c_str(), driveLayoutName(c.layout), c.mass);
        const float g = 9.81f;
        {
            Simulation sim(settings(c, TrackLayout::Flat));
            const JPH::Vec3 start = position(sim);
            run(sim, 4.0);
            const JPH::Vec3 p = position(sim);
            const float v = sim.world().bodies().GetLinearVelocity(sim.vehicle().bodyId()).Length();
            const float drift = std::hypot(p.GetX() - start.GetX(), p.GetZ() - start.GetZ());
            std::printf("    settle: drift %.4f m, |v| %.4f m/s, upDot %.5f, ride y %.3f m\n", drift, v,
                        sim.vehicle().telemetry().upDot, p.GetY());
            CHECK(!sim.hadPhysicsError(), "%s: Jolt update error", def.id.c_str());
            CHECK(drift < 0.05f && v < 0.05f, "%s: no clean settle (drift %.3f m, v %.3f)", def.id.c_str(), drift, v);
            CHECK(sim.vehicle().telemetry().upDot > 0.999f, "%s: not level", def.id.c_str());
            for (const auto& w : sim.vehicle().telemetry().wheels) CHECK(w.contact, "%s: wheel without contact", def.id.c_str());
        }
        {
            // Launch on the concrete lane up to 50 km/h, then a full stop.
            Simulation sim(settings(c, TrackLayout::SurfaceLanes));
            sim.requestTeleport(0);
            run(sim, 1.0);
            const float mu = sim.surfaces().get(SurfaceId::Concrete).longitudinalFriction * c.tuning.tirePeakLongFriction;
            double t = 0.0, t30 = -1.0;
            float vPrev = 0.0f, maxAccel = 0.0f;
            while (t < 15.0 && sim.vehicle().telemetry().speed < 50.0f / 3.6f) {
                run(sim, 0.25, {.throttle = 1.0f});
                t += 0.25;
                const float v = sim.vehicle().telemetry().speed;
                maxAccel = std::max(maxAccel, (v - vPrev) / 0.25f);
                vPrev = v;
                if (t30 < 0.0 && v >= 30.0f / 3.6f) t30 = t;
            }
            std::printf("    launch: 0-30 km/h %.2f s, 0-50 km/h %.2f s, max accel %.2f m/s^2 (mu g %.2f)\n", t30, t,
                        maxAccel, mu * g);
            CHECK(t30 > 0.0 && t30 < 8.0, "%s: 0-30 km/h too slow (%.2f s)", def.id.c_str(), t30);
            CHECK(maxAccel <= mu * g * 1.05f, "%s: acceleration %.2f above mu g", def.id.c_str(), maxAccel);
            const float v0 = sim.vehicle().telemetry().speed;
            const JPH::Vec3 p0 = position(sim);
            const float h0 = headingDeg(sim);
            double tb = 0.0;
            while (sim.vehicle().telemetry().speed > 0.05f && tb < 10.0) {
                run(sim, sim.fixedStep(), {.brake = 1.0f});
                tb += sim.fixedStep();
            }
            const JPH::Vec3 p1 = position(sim);
            const float d = p1.GetZ() - p0.GetZ();
            const float ideal = v0 * v0 / (2.0f * mu * g);
            std::printf("    brake: %.1f km/h -> 0 in %.2f m / %.2f s (mu-limit %.2f m), heading change %.1f deg, "
                        "sideways %.2f m\n", v0 * 3.6f, d, tb, ideal, headingDeg(sim) - h0, p1.GetX() - p0.GetX());
            CHECK(tb < 10.0, "%s: did not stop", def.id.c_str());
            CHECK(d > 0.9f * ideal && d < 2.5f * ideal, "%s: stop %.2f m vs mu-limit %.2f m", def.id.c_str(), d, ideal);
            CHECK(std::abs(headingDeg(sim) - h0) < 10.0f && std::abs(p1.GetX() - p0.GetX()) < 1.5f,
                  "%s: unstable straight stop", def.id.c_str());
        }
        {
            // 15 degree concrete ramp, rolling start in the lowest available range.
            Simulation sim(settings(c, TrackLayout::SurfaceRamps));
            sim.requestTeleport(0);
            run(sim, 1.0);
            if (c.lowRangeRatio > 1.0f) {
                sim.requestToggleLowRange();
                sim.tick();
            }
            double t = 0.0;
            while (t < 15.0 && position(sim).GetZ() < 24.0f) {
                run(sim, 0.25, {.throttle = 1.0f});
                t += 0.25;
            }
            std::printf("    climb 15 deg concrete (%s): %s in %.2f s\n",
                        sim.vehicle().telemetry().lowRange ? "low range" : "high range",
                        position(sim).GetZ() >= 24.0f ? "top" : "stalled", t);
            CHECK(position(sim).GetZ() >= 24.0f, "%s: did not climb the 15 degree ramp", def.id.c_str());
        }
        {
            // Diagonal twister on the proving ground: travel stays inside [min, max] and is used.
            Simulation sim(settings(c, TrackLayout::ProvingGround));
            sim.requestTeleport(2);
            run(sim, 2.0);
            if (c.lowRangeRatio > 1.0f) {
                sim.requestToggleLowRange();
                sim.tick();
            }
            float minL[4] = {9, 9, 9, 9}, maxL[4] = {0, 0, 0, 0};
            for (int i = 0; i < int(8.0 / sim.fixedStep()); ++i) {
                sim.setIntent({.throttle = 0.4f});
                sim.tick();
                for (int w = 0; w < 4; ++w) {
                    minL[w] = std::min(minL[w], sim.vehicle().telemetry().wheels[w].suspensionLength);
                    maxL[w] = std::max(maxL[w], sim.vehicle().telemetry().wheels[w].suspensionLength);
                }
            }
            std::printf("    twister suspension length min..max (m): FL %.3f..%.3f FR %.3f..%.3f RL %.3f..%.3f RR "
                        "%.3f..%.3f\n", minL[0], maxL[0], minL[1], maxL[1], minL[2], maxL[2], minL[3], maxL[3]);
            int used = 0;
            for (int w = 0; w < 4; ++w) {
                const AxleConfig& a = c.axles[w < 2 ? 0 : 1];
                // Jolt lets the wheel pass the hard point by a few mm on a hard hit before its contact constraint
                // pushes back.
                CHECK(minL[w] >= a.suspensionMinLength - 0.02f && maxL[w] <= a.suspensionMaxLength + 1e-3f,
                      "%s: wheel %d length outside [%.3f, %.3f]", def.id.c_str(), w, a.suspensionMinLength,
                      a.suspensionMaxLength);
                used += maxL[w] - minL[w] > 0.10f;
            }
            CHECK(used >= 2, "%s: suspension barely moved on the twister", def.id.c_str());
            CHECK(!sim.hadPhysicsError(), "%s: Jolt update error", def.id.c_str());
        }
    }
}

// Malformed documents produce clear messages; kits and garage ranges are validated.
void vehicleDefinitionValidation() {
    std::ifstream in(kTestData + "/rwd_test/vehicle.json");
    const std::string good((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    json::Value doc;
    CHECK(json::parse(good, doc), "test document does not parse");
    auto expect = [&](const char* what, auto mutate, const char* needle) {
        json::Value d = doc;
        mutate(d);
        VehicleDefinition def;
        std::vector<std::string> errors;
        const bool ok = parseVehicleDefinition(json::write(d), "case.json", def, errors);
        bool found = false;
        for (const std::string& e : errors) found = found || e.find(needle) != std::string::npos;
        std::printf("    %-28s -> %s\n", what, errors.empty() ? "(accepted)" : errors.front().c_str());
        CHECK(!ok && found, "%s: expected an error containing '%s'", what, needle);
    };
    using V = json::Value;
    expect("missing mass", [](V& d) { d.object["body"].object.erase("mass"); }, "case.json: body.mass: missing");
    expect("mass out of range", [](V& d) { d.object["body"].object["mass"] = V::makeNumber(-5); },
           "body.mass: = -5 is out of range [300, 20000] kg");
    expect("unknown key", [](V& d) { d.object["axles"].object["front"].object["springs"] = V::makeNumber(1); },
           "axles.front.springs: unknown key");
    expect("wrong type", [](V& d) { d.object["tires"].object["staticGrip"] = V::makeString("yes"); },
           "tires.staticGrip: must be true or false");
    expect("bad layout", [](V& d) { d.object["drivetrain"].object["layout"] = V::makeString("4x4"); },
           "drivetrain.layout: must be one of: rwd, fwd, awd, part_time, on_demand");
    expect("travel inverted",
           [](V& d) { d.object["axles"].object["rear"].object["suspension"].object["maxLength"] = V::makeNumber(0.1); },
           "axles.rear.suspension.maxLength: must exceed minLength");
    expect("electric motor", [](V& d) { d.object["engine"].object["type"] = V::makeString("electric"); },
           "engine.type: electric motors are reserved");
    expect("falling gears", [](V& d) { d.object["gearbox"].object["ratios"].array[2] = V::makeNumber(5.0); },
           "gearbox.ratios: must be 1-10 falling ratios");
    expect("torque curve", [](V& d) { d.object["engine"].object["torqueCurve"].array.pop_back(); },
           "engine.torqueCurve: must be");
    expect("unknown surface", [](V& d) { d.object["tires"].object["surfaceGrip"].object["ice"] = V::makeNumber(1); },
           "tires.surfaceGrip.ice: unknown key");
    expect("unknown visual model", [](V& d) { d.object["visualModel"] = V::makeString("truck"); },
           "visualModel: unknown model 'truck'");
    expect("unknown audio profile", [](V& d) { d.object["audioProfile"] = V::makeString("truck"); },
           "audioProfile: unknown profile 'truck'");
    expect("tire sound out of range",
           [](V& d) {
               d.object["tireSound"] = V::makeObject();
               d.object["tireSound"].object["squealPitch"] = V::makeNumber(3);
           },
           "tireSound.squealPitch: = 3 is out of range");
    expect("tire sound band inverted",
           [](V& d) {
               d.object["tireSound"] = V::makeObject();
               d.object["tireSound"].object["scrubLowHz"] = V::makeNumber(1000);
               d.object["tireSound"].object["scrubHighHz"] = V::makeNumber(500);
           },
           "tireSound.scrubHighHz: must exceed scrubLowHz");
    {
        json::Value d = doc;
        d.object["audioProfile"] = V::makeString("sporty");
        d.object["tireSound"] = V::makeObject();
        d.object["tireSound"].object["scrubGain"] = V::makeNumber(0.5);
        VehicleDefinition def;
        std::vector<std::string> errors;
        const bool ok = parseVehicleDefinition(json::write(d), "case.json", def, errors);
        TireSoundProfile want = tireSoundPreset("sporty");
        want.scrubGain = 0.5f;
        CHECK(ok && def.config.tireSound == want, "sporty profile with a scrubGain override: %s",
              errors.empty() ? "wrong values" : errors.front().c_str());
    }
    expect("unknown garage key",
           [](V& d) {
               V r = V::makeArray();
               r.array = {V::makeNumber(1), V::makeNumber(2)};
               d.object["garage"].object["ranges"].object["engine.boost"] = r;
           },
           "garage.ranges.engine.boost: unknown garage parameter");
    expect("garage range too wide",
           [](V& d) {
               V r = V::makeArray();
               r.array = {V::makeNumber(0.1), V::makeNumber(2)};
               d.object["garage"].object["ranges"].object["suspension.front.frequency"] = r;
           },
           "exceeds the allowed [0.5, 5]");
    expect("bad kit override",
           [](V& d) {
               d.object["kits"].array[0].object["overrides"].object["body"] = V::makeObject();
               d.object["kits"].array[0].object["overrides"].object["body"].object["mass"] = V::makeNumber(1.0e6);
           },
           "(kit test_lift): body.mass: = 1e+06 is out of range");
    expect("invalid JSON", [](V& d) { d = V::makeString("x"); }, "document: must be an object");
    {
        VehicleDefinition def;
        std::vector<std::string> errors;
        CHECK(!parseVehicleDefinition("{ \"schema\": 1, ", "broken.json", def, errors) && !errors.empty() &&
                  errors[0].find("broken.json: invalid JSON") == 0,
              "truncated JSON not reported");
    }
    // The folder name must equal the id.
    {
        namespace fs = std::filesystem;
        const fs::path dir = fs::temp_directory_path() / "trailforge_vehicle_catalog_test";
        fs::remove_all(dir);
        fs::create_directories(dir / "wrong_folder");
        std::ofstream(dir / "wrong_folder" / "vehicle.json") << good;
        std::vector<std::string> errors;
        const auto defs = loadVehicleCatalog(dir.string(), errors);
        CHECK(defs.empty() && errors.size() == 1 && errors[0].find("must match its folder name") != std::string::npos,
              "folder / id mismatch not reported");
        fs::remove_all(dir);
    }
    // Kit: overrides, mass delta, ranges.
    const VehicleDefinition def = loadOne(kTestData, "rwd_test");
    CHECK(def.kits.size() == 1 && def.kits[0].visualParts.size() == 1, "kit not loaded");
    VehicleConfig kit;
    std::vector<std::string> errors;
    CHECK(configWithKit(def, "test_lift", kit, errors), "kit config failed");
    std::printf("    kit test_lift: mass %.0f -> %.0f kg, attach %.3f -> %.3f m, front spring %.2f -> %.2f Hz, "
                "max length %.2f -> %.2f m\n", def.config.mass, kit.mass, def.config.attachHeight, kit.attachHeight,
                def.config.tuning.axles[0].springFrequency, kit.tuning.axles[0].springFrequency,
                def.config.axles[0].suspensionMaxLength, kit.axles[0].suspensionMaxLength);
    CHECK(kit.mass == def.config.mass + 60.0f && kit.attachHeight == 0.03f &&
              kit.tuning.axles[0].springFrequency == 1.4f && kit.axles[1].suspensionMaxLength == 0.5f &&
              kit.tuning.axles[1].springFrequency == def.config.tuning.axles[1].springFrequency,
          "kit overrides not applied as expected");
    CHECK(!configWithKit(def, "nope", kit, errors), "unknown kit accepted");
}

// Which wheels the engine drives in each layout and transfer mode.
void drivetrainLayouts() {
    struct Case {
        const char* name;
        DriveLayout layout;
        int transfer; // -1 = none, else TransferMode
        bool front, rear;
    };
    const Case cases[] = {
        {"RWD", DriveLayout::Rwd, -1, false, true},
        {"FWD", DriveLayout::Fwd, -1, true, false},
        {"AWD", DriveLayout::Awd, -1, true, true},
        {"ON-DEMAND", DriveLayout::OnDemand, -1, true, true},
        {"PART-TIME 2H", DriveLayout::PartTime, int(TransferMode::TwoHigh), false, true},
        {"PART-TIME 4H", DriveLayout::PartTime, int(TransferMode::FourHigh), true, true},
        {"PART-TIME 4L", DriveLayout::PartTime, int(TransferMode::FourLow), true, true},
    };
    for (const Case& k : cases) {
        VehicleConfig c;
        c.layout = k.layout;
        if (k.layout == DriveLayout::OnDemand) c.tuning.frontTorqueShare = 0.1f;
        // Low-grip lane: full throttle spins the driven wheels, so driven and free wheels separate clearly.
        Simulation sim(settings(c, TrackLayout::SurfaceLanes));
        sim.requestTeleport(5);
        run(sim, 1.0);
        if (k.transfer >= 0) {
            sim.requestTransfer(TransferMode(k.transfer));
            sim.tick();
        }
        const VehicleTelemetry& t = sim.vehicle().telemetry();
        CHECK(t.frontDriven == k.front && t.rearDriven == k.rear, "%s: driven axles %d/%d", k.name, t.frontDriven,
              t.rearDriven);
        float maxSlip[4] = {0, 0, 0, 0};
        for (int i = 0; i < int(1.5 / sim.fixedStep()); ++i) {
            sim.setIntent({.throttle = 1.0f});
            sim.tick();
            for (int w = 0; w < 4; ++w) {
                const auto& wt = sim.vehicle().telemetry().wheels[w];
                maxSlip[w] = std::max(maxSlip[w], wt.rollingSpeed - wt.groundSpeed);
            }
        }
        const float v = sim.vehicle().telemetry().speed;
        std::printf("    %-13s driven F%d R%d, front share %.2f, center %d, low %d: v(1.5 s) %.2f m/s, max wheel slip "
                    "FL %.2f FR %.2f RL %.2f RR %.2f m/s\n", k.name, t.frontDriven, t.rearDriven, t.frontTorqueShare,
                    int(t.centerDiff), t.lowRange, v, maxSlip[0], maxSlip[1], maxSlip[2], maxSlip[3]);
        CHECK(v > 1.0f, "%s: did not move", k.name);
        for (int w = 0; w < 4; ++w) {
            const bool driven = w < 2 ? k.front : k.rear;
            if (!driven) CHECK(maxSlip[w] < 0.3f, "%s: undriven wheel %d slips %.2f m/s", k.name, w, maxSlip[w]);
        }
        if (k.transfer == int(TransferMode::FourLow)) CHECK(t.lowRange, "4L without low range");
        if (k.transfer == int(TransferMode::FourHigh) || k.transfer == int(TransferMode::FourLow))
            CHECK(t.centerDiff == DiffMode::Locked, "%s: center not locked", k.name);
    }
    // Transfer case rules: 2H <-> 4H on the move, 4L only near a standstill, cycling order, other layouts refuse.
    {
        VehicleConfig c;
        c.layout = DriveLayout::PartTime;
        Simulation sim(settings(c, TrackLayout::SurfaceLanes));
        sim.requestTeleport(0);
        run(sim, 1.0);
        CHECK(sim.vehicle().telemetry().transfer == TransferMode::TwoHigh, "part-time does not start in 2H");
        run(sim, 3.0, {.throttle = 0.6f});
        const float v = sim.vehicle().telemetry().speed;
        sim.requestCycleTransfer();
        run(sim, sim.fixedStep(), {.throttle = 0.6f});
        CHECK(sim.vehicle().telemetry().transfer == TransferMode::FourHigh, "2H -> 4H refused at %.1f m/s", v);
        sim.requestCycleTransfer();
        run(sim, sim.fixedStep(), {.throttle = 0.6f});
        CHECK(sim.vehicle().telemetry().transfer == TransferMode::FourHigh && std::string(sim.lastMessage()).find("4L") == 0,
              "4L accepted while moving (%s)", sim.lastMessage());
        while (sim.vehicle().telemetry().speed > 0.05f) run(sim, sim.fixedStep(), {.brake = 1.0f});
        sim.requestCycleTransfer();
        sim.tick();
        CHECK(sim.vehicle().telemetry().transfer == TransferMode::FourLow, "4L refused at a standstill");
        sim.requestCycleTransfer();
        sim.tick();
        CHECK(sim.vehicle().telemetry().transfer == TransferMode::TwoHigh && !sim.vehicle().telemetry().lowRange,
              "4L -> 2H failed");
        sim.requestToggleLowRange();
        sim.tick();
        CHECK(sim.vehicle().telemetry().transfer == TransferMode::FourLow, "L in 2H does not select 4L");
        std::printf("    transfer case: 2H -> 4H at %.1f m/s ok, 4L refused while moving, 4L / 2H / L at a stop ok\n", v);
        Simulation awd(settings(VehicleConfig{}, TrackLayout::Flat));
        awd.requestCycleTransfer();
        awd.tick();
        CHECK(std::string(awd.lastMessage()).find("No transfer case") == 0, "AWD accepted a transfer switch");
    }
}

// Garage setups: values within ranges, live vs rebuild, in-place respawn, file round trip.
void garageSetups() {
    const VehicleDefinition def = loadOne(kTestData, "rwd_test");
    const VehicleConfig& base = def.config;
    for (const SetupParamInfo& p : setupParams()) {
        const ParamRange r = setupParamRange(p, def, nullptr, base);
        const float b = p.get(base, base);
        CHECK(r.min <= b && b <= r.max, "%s: base %.3f outside [%.3f, %.3f]", p.key, b, r.min, r.max);
        VehicleConfig c = base;
        p.set(c, base, b);
        CHECK(std::abs(p.get(c, base) - b) <= 1e-4f * std::max(1.0f, std::abs(b)), "%s: set(get) changes the value", p.key);
    }
    const ParamRange fr = setupParamRange(*findSetupParam("suspension.front.frequency"), def, nullptr, base);
    CHECK(fr.min == 1.2f && fr.max == 2.2f, "definition range not used (%.2f..%.2f)", fr.min, fr.max);
    const ParamRange rh = setupParamRange(*findSetupParam("suspension.rideHeight"), def, def.findKit("test_lift"), base);
    CHECK(rh.min == -0.02f && rh.max == 0.12f, "kit range not used");

    VehicleSetup s;
    s.name = "Test";
    s.values["suspension.front.frequency"] = 9.0f; // clamped to 2.2
    s.values["brakes.frontBias"] = 0.65f;
    s.values["brakes.torqueScale"] = 1.2f;
    s.values["engine.boost"] = 1.0f;               // unknown, ignored
    s.values["drivetrain.layout"] = float(int(DriveLayout::PartTime));
    VehicleConfig c;
    std::vector<std::string> warnings;
    CHECK(buildSetupConfig(def, s, c, warnings), "setup build failed");
    for (const std::string& w : warnings) std::printf("    warning: %s\n", w.c_str());
    const float total = c.tuning.axles[0].brakeTorque + c.tuning.axles[1].brakeTorque;
    std::printf("    setup: front spring %.2f Hz, brakes F %.0f / R %.0f N*m (bias %.2f, total x%.2f), layout %s\n",
                c.tuning.axles[0].springFrequency, c.tuning.axles[0].brakeTorque, c.tuning.axles[1].brakeTorque,
                c.tuning.axles[0].brakeTorque / total, total / 4400.0f, driveLayoutName(c.layout));
    CHECK(warnings.size() == 2, "expected 2 warnings (clamp, unknown), got %zu", warnings.size());
    CHECK(c.tuning.axles[0].springFrequency == 2.2f, "range clamp failed");
    CHECK(std::abs(c.tuning.axles[0].brakeTorque / total - 0.65f) < 1e-4f && std::abs(total - 4400.0f * 1.2f) < 0.1f,
          "brake bias / scale wrong");
    CHECK(c.layout == DriveLayout::PartTime && Vehicle::needsRebuild(base, c), "layout change must rebuild");
    VehicleConfig live = base;
    live.tuning.axles[0].springFrequency = 1.9f;
    live.tuning.finalDrive = 4.1f;
    CHECK(!Vehicle::needsRebuild(base, live), "tuning change must be live");

    // Live change keeps the body; a rebuild respawns in place (same x/z within a few cm, same heading).
    Simulation sim(settings(base, TrackLayout::Flat));
    run(sim, 3.0, {.throttle = 0.3f, .steer = 0.4f});
    run(sim, 3.0, {.brake = 1.0f});
    const unsigned spawns = sim.spawnCount();
    sim.requestVehicleConfig(live);
    sim.tick();
    CHECK(sim.spawnCount() == spawns && sim.vehicleConfig().tuning.finalDrive == 4.1f, "live change respawned");
    const JPH::Vec3 p0 = position(sim);
    const float h0 = headingDeg(sim);
    VehicleConfig lifted = live;
    lifted.attachHeight -= 0.05f;
    sim.requestVehicleConfig(lifted);
    sim.tick();
    run(sim, 3.0);
    const JPH::Vec3 p1 = position(sim);
    std::printf("    in-place respawn: moved %.3f m, heading %.2f -> %.2f deg, body y %.3f -> %.3f m\n",
                std::hypot(p1.GetX() - p0.GetX(), p1.GetZ() - p0.GetZ()), h0, headingDeg(sim), p0.GetY(), p1.GetY());
    CHECK(sim.spawnCount() == spawns + 1, "rebuild did not respawn");
    CHECK(std::hypot(p1.GetX() - p0.GetX(), p1.GetZ() - p0.GetZ()) < 0.1f && std::abs(headingDeg(sim) - h0) < 1.0f,
          "respawn not in place");
    CHECK(p1.GetY() > p0.GetY() + 0.03f, "50 mm lower attachment did not raise the body");

    // File round trip.
    SetupFile f;
    f.vehicleId = def.id;
    f.selected = "Test";
    f.setups.push_back(s);
    VehicleSetup s2;
    s2.name = "Rock \"crawl\"";
    s2.kit = "test_lift";
    f.setups.push_back(s2);
    const std::string path = (std::filesystem::temp_directory_path() / "trailforge_setup_test" / "rwd_test.json").string();
    std::string error;
    CHECK(saveSetupFile(path, f, &error), "save failed: %s", error.c_str());
    SetupFile back;
    CHECK(loadSetupFile(path, back, &error), "load failed: %s", error.c_str());
    CHECK(back.vehicleId == f.vehicleId && back.selected == f.selected && back.setups == f.setups, "round trip differs");
    std::filesystem::remove_all(std::filesystem::path(path).parent_path());
}

// Migration check: the shipped SUV definition reproduces the v0.7 C++ configuration exactly.
void suvDefinitionMatchesBuiltin() {
    const VehicleDefinition def = loadOne(kAssets, "trailforge_suv");
    // The built-in configuration is the v0.7 SUV (full-time AWD, no drag, 0.80 m tires); the definition differs
    // from it only in the part-time transfer case and the drag area (v0.8.0) and in the 315/70R17 tires with the
    // matching final drive (v0.9.0), so every other value is still the migrated one.
    VehicleConfig expected;
    expected.id = def.config.id;
    expected.name = def.config.name;
    expected.layout = DriveLayout::PartTime;
    expected.tuning.dragArea = 1.3f;
    for (AxleConfig& a : expected.axles) {
        a.wheelRadius = 0.436f;
        a.wheelWidth = 0.315f;
        a.wheelMass = 43.0f;
        a.wheelInertia = 4.7f;
        a.rimRatio = 0.495f;
    }
    expected.tuning.finalDrive = 4.27f;
    std::printf("    comForward %.9g vs %.9g, layout %s, drag area %.2f m^2\n", def.config.comForward,
                expected.comForward, driveLayoutName(def.config.layout), double(def.config.tuning.dragArea));
    CHECK(def.config == expected, "vehicle.json differs from the built-in configuration beyond layout, drag, tires and final drive");
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"vehicle_catalog_sweep", vehicleCatalogSweep},
        {"vehicle_definition_validation", vehicleDefinitionValidation},
        {"drivetrain_layouts", drivetrainLayouts},
        {"garage_setups", garageSetups},
        {"suv_definition_matches_builtin", suvDefinitionMatchesBuiltin},
    };
    int ran = 0;
    for (const auto& [name, fn] : tests) {
        if (argc > 1 && std::strcmp(argv[1], name) != 0) continue;
        std::printf("[ RUN ] %s\n", name);
        const int before = g_failures;
        fn();
        std::printf("[%s] %s\n", g_failures == before ? "  OK " : "FAIL ", name);
        ++ran;
    }
    if (ran == 0) {
        std::printf("unknown test '%s'\n", argc > 1 ? argv[1] : "");
        return 2;
    }
    return g_failures == 0 ? 0 : 1;
}
