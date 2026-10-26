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
            // Collision bounds in body space (the close follow camera is sized from them): around the body origin,
            // the roof above it, the bottom below it.
            const JPH::AABox& b = sim.vehicle().bodyBounds();
            std::printf("    bounds x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f m\n", b.mMin.GetX(), b.mMax.GetX(),
                        b.mMin.GetY(), b.mMax.GetY(), b.mMin.GetZ(), b.mMax.GetZ());
            CHECK(b.mMin.GetZ() < -1.0f && b.mMax.GetZ() > 1.0f && b.mMin.GetY() < 0.0f && b.mMax.GetY() > 0.6f &&
                      b.mMax.GetZ() - b.mMin.GetZ() < 7.0f,
                  "%s: body bounds not in body space", def.id.c_str());
            if (c.bodyHulls.empty() && c.bodyShape == "kit")
                CHECK(std::abs(b.mMax.GetY() - 1.30f) < 1e-3f && std::abs(b.mMin.GetZ() + 2.30f) < 1e-3f,
                      "%s: kit bounds top %.4f rear %.4f (boxes 1.30 / -2.30 m)", def.id.c_str(), b.mMax.GetY(),
                      b.mMin.GetZ());
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
           "drivetrain.layout: must be one of: rwd, fwd, awd, part_time, on_demand, dual_motor");
    expect("travel inverted",
           [](V& d) { d.object["axles"].object["rear"].object["suspension"].object["maxLength"] = V::makeNumber(0.1); },
           "axles.rear.suspension.maxLength: must exceed minLength");
    expect("electric with a gearbox", [](V& d) { d.object["engine"].object["type"] = V::makeString("electric"); },
           "gearbox: an electric drivetrain has no gearbox");
    expect("dual motor on combustion", [](V& d) { d.object["drivetrain"].object["layout"] = V::makeString("dual_motor"); },
           "drivetrain.layout: \"dual_motor\" needs engine.type \"electric\"");
    expect("falling gears", [](V& d) { d.object["gearbox"].object["ratios"].array[2] = V::makeNumber(5.0); },
           "gearbox.ratios: must be 1-10 falling ratios");
    expect("torque curve", [](V& d) { d.object["engine"].object["torqueCurve"].array.pop_back(); },
           "engine.torqueCurve: must be");
    expect("unknown surface", [](V& d) { d.object["tires"].object["surfaceGrip"].object["ice"] = V::makeNumber(1); },
           "tires.surfaceGrip.ice: unknown key");
    expect("unknown visual model", [](V& d) { d.object["visualModel"] = V::makeString("truck"); },
           "visualModel: unknown model 'truck'");
    expect("unknown body shape", [](V& d) { d.object["bodyShape"] = V::makeString("voxel"); },
           "bodyShape: unknown shape 'voxel'");
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
    // v0.24: front-biased brakes (4200 / 2200 N*m per wheel; 3000 / 3000 left the front axle short of its grip).
    expected.tuning.axles[0].brakeTorque = 4200.0f;
    expected.tuning.axles[1].brakeTorque = 2200.0f;
    std::printf("    comForward %.9g vs %.9g, layout %s, drag area %.2f m^2\n", def.config.comForward,
                expected.comForward, driveLayoutName(def.config.layout), double(def.config.tuning.dragArea));
    CHECK(def.config == expected, "vehicle.json differs from the built-in configuration beyond layout, drag, tires, final drive and brakes");
}


// Electric definition: the new fields are validated with their paths.
void electricDefinitionValidation() {
    std::ifstream in(kAssets + "/trailforge_ev/vehicle.json");
    const std::string good((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    json::Value doc;
    CHECK(json::parse(good, doc), "EV document does not parse");
    {
        VehicleDefinition def;
        std::vector<std::string> errors;
        CHECK(parseVehicleDefinition(good, "ev.json", def, errors), "shipped EV rejected");
        const ElectricConfig& e = def.config.electric;
        CHECK(def.config.isElectric() && def.config.layout == DriveLayout::DualMotor, "not a dual-motor EV");
        CHECK(std::abs(e.motors[0].maxPower - 180000.0f) < 1.0f && std::abs(e.motors[1].maxTorque - 450.0f) < 1e-3f,
              "motor values not read: %.0f W, %.0f N*m", e.motors[0].maxPower, e.motors[1].maxTorque);
        CHECK(std::abs(e.offRoadFrontShare - 0.5f) < 1e-6f && std::abs(def.config.tuning.frontTorqueShare - 0.4f) < 1e-6f,
              "torque shares not read");
    }
    auto expect = [&](const char* what, auto mutate, const char* needle) {
        json::Value d = doc;
        mutate(d);
        VehicleDefinition def;
        std::vector<std::string> errors;
        const bool ok = parseVehicleDefinition(json::write(d), "ev.json", def, errors);
        bool found = false;
        for (const std::string& e : errors) found = found || e.find(needle) != std::string::npos;
        std::printf("    %-28s -> %s\n", what, errors.empty() ? "(accepted)" : errors.front().c_str());
        CHECK(!ok && found, "%s: expected an error containing '%s'", what, needle);
    };
    using V = json::Value;
    auto motor = [](V& d, const char* axle) -> V& { return d.object["engine"].object["motors"].object[axle]; };
    expect("missing rear motor", [](V& d) { d.object["engine"].object["motors"].object.erase("rear"); },
           "engine.motors.rear: missing");
    expect("motor power range", [&](V& d) { motor(d, "front").object["maxPowerKW"] = V::makeNumber(5000); },
           "engine.motors.front.maxPowerKW: = 5000 is out of range");
    expect("regen above drive", [&](V& d) { motor(d, "rear").object["regenTorque"] = V::makeNumber(900); },
           "engine.motors.rear.regenTorque: must not exceed maxTorque");
    expect("base speed past max",
           [&](V& d) {
               motor(d, "front").object["maxTorque"] = V::makeNumber(50);
               motor(d, "front").object["regenTorque"] = V::makeNumber(50);
           },
           "engine.motors.front.maxPowerKW: base speed maxPower / maxTorque must be below maxRPM");
    expect("battery capacity", [](V& d) { d.object["engine"].object["battery"].object["capacityKWh"] = V::makeNumber(0); },
           "engine.battery.capacityKWh: = 0 is out of range");
    expect("SoC above 1", [](V& d) { d.object["engine"].object["battery"].object["initialSoC"] = V::makeNumber(1.5); },
           "engine.battery.initialSoC: = 1.5 is out of range");
    expect("efficiency", [](V& d) { d.object["engine"].object["efficiency"].object["drive"] = V::makeNumber(1.2); },
           "engine.efficiency.drive: = 1.2 is out of range");
    expect("combustion key on EV", [](V& d) { d.object["engine"].object["idleRPM"] = V::makeNumber(800); },
           "engine.idleRPM: unknown key");
    expect("gearbox on EV", [](V& d) { d.object["gearbox"] = V::makeObject(); },
           "gearbox: an electric drivetrain has no gearbox");
    expect("center diff on EV", [](V& d) { d.object["drivetrain"].object["centerDiff"] = V::makeString("open"); },
           "drivetrain.centerDiff: unknown key");
    expect("EV not dual motor", [](V& d) { d.object["drivetrain"].object["layout"] = V::makeString("awd"); },
           "drivetrain.layout: an electric vehicle needs \"dual_motor\"");
    expect("EV split at 1", [](V& d) { d.object["drivetrain"].object["frontTorqueShare"] = V::makeNumber(1.0); },
           "drivetrain.frontTorqueShare: must be strictly between 0 and 1 for dual_motor");
}

// Motor envelope: constant torque to the base speed, constant power above, zero at the maximum speed.
void evMotorCurve() {
    MotorConfig m;
    m.maxTorque = 450.0f;
    m.maxPower = 220000.0f;
    m.maxRPM = 15000.0f;
    m.regenTorque = 300.0f;
    const float base = motorBaseRPM(m);
    std::printf("    base speed %.0f rpm\n", base);
    CHECK(std::abs(base - 220000.0f / 450.0f / kRpmToRadPerSec) < 0.5f, "base speed %.1f", base);
    CHECK(std::abs(motorDriveLimit(m, 0.0f) - 450.0f) < 1e-3f, "stall torque %.1f", motorDriveLimit(m, 0.0f));
    CHECK(std::abs(motorDriveLimit(m, 0.5f * base * kRpmToRadPerSec) - 450.0f) < 1e-3f, "constant torque range");
    for (float rpm : {6000.0f, 9000.0f, 12000.0f, 14000.0f}) {
        const float w = rpm * kRpmToRadPerSec, t = motorDriveLimit(m, w);
        std::printf("    %5.0f rpm: %.1f N*m, %.1f kW\n", rpm, t, t * w * 1e-3f);
        CHECK(std::abs(t * w - 220000.0f) < 50.0f, "constant power at %.0f rpm: %.0f W", rpm, t * w);
    }
    CHECK(motorDriveLimit(m, 15000.0f * kRpmToRadPerSec) == 0.0f, "torque at max rpm");
    CHECK(motorDriveLimit(m, -0.5f * base * kRpmToRadPerSec) == 450.0f, "envelope symmetric in the rotation sign");
    const float half = motorDriveLimit(m, 0.99f * 15000.0f * kRpmToRadPerSec);
    CHECK(half > 0.0f && half < 220000.0f / (0.99f * 15000.0f * kRpmToRadPerSec), "fade over the last 2 %%");
    CHECK(motorRegenLimit(m, 0.0f) == 300.0f && std::abs(motorRegenLimit(m, 1000.0f) - 220.0f) < 1e-3f,
          "regen limit %.1f / %.1f", motorRegenLimit(m, 0.0f), motorRegenLimit(m, 1000.0f));
}

ElectricConfig evConfig() {
    VehicleDefinition d = loadOne(kAssets, "trailforge_ev");
    return d.config.electric;
}

EvInput evInput(float speed) {
    EvInput in;
    in.speed = speed;
    in.mass = 2650.0f;
    in.soc = 0.8f;
    in.frontShare = 0.4f;
    in.dt = 1.0f / 120.0f;
    for (int a = 0; a < 2; ++a) {
        in.axles[std::size_t(a)].wheelRadius = 0.4155f;
        in.axles[std::size_t(a)].frictionBrakeTorque = a == 0 ? 3800.0f : 3200.0f;
        in.axles[std::size_t(a)].motorOmega = speed / 0.4155f * 12.0f;
    }
    return in;
}

// Electronic split: part throttle follows the drive mode share, full throttle uses both motors fully, an axle cut
// by traction control hands its torque to the other.
void evTorqueSplit() {
    const ElectricConfig c = evConfig();
    const DrivingAidTuning aids;
    {
        ElectricDrive d;
        EvInput in = evInput(0.0f);
        in.throttle = 0.5f;
        const EvOutput o = d.update(c, aids, in);
        const float f = o.driveTorque[0] * 12.0f, r = o.driveTorque[1] * 12.0f;
        std::printf("    half pedal at rest: front %.0f, rear %.0f N*m at the wheels (share %.3f)\n", f, r, f / (f + r));
        CHECK(std::abs(f / (f + r) - 0.4f) < 1e-3f, "part-throttle share %.3f", f / (f + r));
        CHECK(std::abs(f + r - 0.5f * (400.0f + 450.0f) * 12.0f) < 1.0f, "half of both motors: %.0f", f + r);
        in.frontShare = 0.5f;
        const EvOutput o2 = d.update(c, aids, in);
        CHECK(std::abs(o2.driveTorque[0] - o2.driveTorque[1]) < 1e-3f, "OFF-ROAD 50 / 50 split");
    }
    {
        ElectricDrive d;
        EvInput in = evInput(5.0f);
        in.throttle = 1.0f;
        const EvOutput o = d.update(c, aids, in);
        CHECK(std::abs(o.driveTorque[0] - 400.0f) < 1e-2f && std::abs(o.driveTorque[1] - 450.0f) < 1e-2f,
              "full pedal: both motors at their limit (%.1f / %.1f)", o.driveTorque[0], o.driveTorque[1]);
        // At 120 km/h both motors are in their constant-power range.
        EvInput fast = evInput(120.0f / 3.6f);
        fast.throttle = 1.0f;
        const EvOutput of = d.update(c, aids, fast);
        const float w = fast.axles[0].motorOmega;
        std::printf("    full pedal at 120 km/h: %.1f + %.1f N*m, %.0f kW\n", of.driveTorque[0], of.driveTorque[1],
                    (of.driveTorque[0] + of.driveTorque[1]) * w * 1e-3f);
        CHECK(std::abs((of.driveTorque[0] + of.driveTorque[1]) * w - 400000.0f) < 100.0f, "400 kW at 120 km/h");
    }
    {
        ElectricDrive d;
        EvInput in = evInput(3.0f);
        in.throttle = 0.4f; // 4080 N*m: the rear motor (5400 at the wheels) can take the front's share
        in.traction = true;
        in.tcSlipThreshold = aids.tcSlipThreshold;
        in.tcMinFactor = aids.tcMinThrottle;
        in.axles[0].driveSlip = 1.0f; // front axle spinning
        EvOutput o;
        for (int i = 0; i < 120; ++i) o = d.update(c, aids, in);
        const float f = o.driveTorque[0] * 12.0f, r = o.driveTorque[1] * 12.0f;
        std::printf("    front spinning, TC: factors %.2f / %.2f, front %.0f, rear %.0f N*m\n", o.tcFactor[0], o.tcFactor[1], f, r);
        CHECK(o.tcFactor[0] < 0.5f && o.tcFactor[1] == 1.0f && o.tcActive, "front factor not cut");
        CHECK(r > 0.6f * 0.4f * (400.0f + 450.0f) * 12.0f + 500.0f, "rear did not take over the front's torque (%.0f)", r);
        CHECK(std::abs(f + r - 0.4f * (400.0f + 450.0f) * 12.0f) < 1.0f, "request not kept: %.0f", f + r);
        in.soc = 0.0f;
        CHECK(d.update(c, aids, in).driveTorque[1] == 0.0f, "an empty battery still drives");
    }
}

// Regen blend: lift-off target, the pedal taken by regen first, the friction brakes for the rest; fade-out at low
// speed, cut on wheel slip and with a full battery.
void evRegenBlend() {
    const ElectricConfig c = evConfig();
    const DrivingAidTuning aids;
    ElectricDrive d;
    EvInput in = evInput(15.0f);
    EvOutput o = d.update(c, aids, in);
    const float liftOff = (o.regenTorque[0] + o.regenTorque[1]) * 12.0f / 0.4155f;
    std::printf("    lift-off at 15 m/s: %.0f N = %.2f m/s^2\n", liftOff, liftOff / 2650.0f);
    CHECK(std::abs(liftOff / 2650.0f - c.liftOffDecel) < 1e-3f, "lift-off decel %.3f", liftOff / 2650.0f);
    CHECK(o.frictionScale[0] == 1.0f && o.frictionScale[1] == 1.0f, "friction untouched without the pedal");

    in.brake = 0.1f; // 760 + 640 N*m per axle requested
    o = d.update(c, aids, in);
    std::printf("    pedal 0.1: regen %.0f / %.0f N*m at the shaft, friction scale %.2f / %.2f\n", o.regenTorque[0],
                o.regenTorque[1], o.frictionScale[0], o.frictionScale[1]);
    CHECK(o.frictionScale[0] < 0.01f && o.frictionScale[1] < 0.01f, "light pedal not fully regenerative");
    in.brake = 1.0f;
    o = d.update(c, aids, in);
    const float regenWheel = (o.regenTorque[0] + o.regenTorque[1]) * 12.0f;
    std::printf("    full pedal: regen %.0f N*m at the wheels, friction scale %.2f / %.2f\n", regenWheel, o.frictionScale[0],
                o.frictionScale[1]);
    CHECK(o.frictionScale[0] > 0.3f && o.frictionScale[0] < 1.0f, "full pedal: friction share %.2f", o.frictionScale[0]);
    const float power = (o.regenTorque[0] + o.regenTorque[1]) * in.axles[0].motorOmega;
    CHECK(power <= c.regenMaxPowerKW * 1000.0f + 1.0f, "charge power limit exceeded: %.0f W", power);

    EvInput slow = evInput(0.5f * c.regenFadeSpeed);
    slow.brake = 0.5f;
    const EvOutput os = d.update(c, aids, slow);
    EvInput stop = evInput(0.0f);
    stop.brake = 0.5f;
    const EvOutput o0 = d.update(c, aids, stop);
    CHECK(os.frictionScale[0] > 0.0f && os.frictionScale[0] < 1.0f, "half fade: friction %.2f", os.frictionScale[0]);
    CHECK(o0.regenTorque[0] == 0.0f && o0.frictionScale[0] == 1.0f && o0.frictionScale[1] == 1.0f,
          "standstill: friction only");

    EvInput full = evInput(15.0f);
    full.soc = 1.0f;
    full.brake = 0.5f;
    const EvOutput of = d.update(c, aids, full);
    CHECK(of.regenTorque[0] == 0.0f && of.frictionScale[0] == 1.0f, "a full battery still regenerates");

    ElectricDrive ds;
    EvInput slip = evInput(15.0f);
    slip.brake = 0.3f;
    slip.axles[1].brakeSlip = 0.5f;
    EvOutput sl;
    for (int i = 0; i < 6; ++i) sl = ds.update(c, aids, slip);
    std::printf("    rear slipping: regen factor %.2f / %.2f, friction scale rear %.2f\n", sl.regenFactor[0],
                sl.regenFactor[1], sl.frictionScale[1]);
    CHECK(sl.regenFactor[1] == 0.0f && sl.regenFactor[0] == 1.0f && sl.frictionScale[1] == 1.0f,
          "slipping axle keeps its regen");
}

// Battery: drive energy / efficiency + auxiliary load out, regen x efficiency in.
void evBattery() {
    ElectricConfig c;
    c.batteryCapacityKWh = 100.0f;
    c.initialSoC = 0.5f;
    c.driveEfficiency = 0.9f;
    c.regenEfficiency = 0.8f;
    c.auxPowerKW = 1.0f;
    Battery b;
    b.init(c);
    CHECK(std::abs(b.soc() - 0.5f) < 1e-6f, "initial SoC %.4f", b.soc());
    float p = 0.0f;
    for (int i = 0; i < 3600; ++i) p = b.step(c, 90000.0f, 1.0f); // 90 kW at the shaft for one hour
    std::printf("    1 h at 90 kW shaft: battery %.1f kW, SoC %.4f\n", p * 1e-3f, b.soc());
    CHECK(std::abs(p - 101000.0f) < 1.0f, "battery power %.0f W", p);
    CHECK(b.soc() == 0.0f, "101 kWh out of 50 kWh left: SoC %.4f", b.soc());
    b.init(c);
    for (int i = 0; i < 120 * 60; ++i) b.step(c, -50000.0f, 1.0f / 120.0f); // one minute of 50 kW regen
    const double expected = 50.0 + (50000.0 * 0.8 - 1000.0) * 60.0 / 3.6e6;
    std::printf("    1 min at 50 kW regen: %.4f kWh (expected %.4f)\n", b.energyJ / 3.6e6, expected);
    CHECK(std::abs(b.energyJ / 3.6e6 - expected) < 1e-4, "regen bookkeeping");
}

// The EV in the simulation: launch, regen stop, SoC, drive modes, no low range, mass and center of mass.
void evSimulation() {
    const VehicleDefinition def = loadOne(kAssets, "trailforge_ev");
    const VehicleDefinition suv = loadOne(kAssets, "trailforge_suv");
    const float dm = def.config.mass - suv.config.mass;
    std::printf("    mass %.0f kg (SUV %.0f, +%.0f), COM height %.2f (SUV %.2f)\n", def.config.mass, suv.config.mass, dm,
                def.config.comHeight, suv.config.comHeight);
    CHECK(dm >= 400.0f && dm <= 700.0f, "EV must be 400-700 kg heavier than the SUV: %+.0f", dm);
    CHECK(def.config.comHeight < suv.config.comHeight - 0.02f, "battery in the floor must lower the COM");

    CHECK(def.config.tuning.aids.traction == TractionMode::On, "the EV ships with traction control on");
    {
        // Full-pedal launch with the defaults: TC keeps the front wheels from spinning up, so no wheelspin marks.
        Simulation tc(settings(def.config, TrackLayout::Flat));
        for (int i = 0; i < 120; ++i) tc.tick();
        float slip = 0.0f, kmh1 = 0.0f, minKW = 1e9f;
        for (int i = 0; i < 360; ++i) {
            tc.setIntent({.throttle = 1.0f});
            tc.tick();
            if (i == 119) kmh1 = tc.vehicle().telemetry().forwardSpeed * 3.6f;
            if (i >= 30 && i < 120) minKW = std::min(minKW, tc.vehicle().telemetry().ev.motorPowerKW);
            if (i > 60)
                for (const auto& w : tc.vehicle().telemetry().wheels)
                    if (w.contact && w.slipValid) slip = std::max(slip, w.longitudinalSlip);
        }
        std::printf("    full pedal, TC on: %.1f km/h at 1 s, %.1f km/h at 3 s, min power 0.25-1 s %.0f kW, max slip after "
                    "0.5 s %.2f, %zu mark segments\n",
                    kmh1, tc.vehicle().telemetry().forwardSpeed * 3.6f, minKW, slip, tc.tireMarks().size());
        CHECK(slip < 0.5f && tc.tireMarks().size() < 10, "TC-on launch spins: slip %.2f, %zu marks", slip,
              tc.tireMarks().size());
    }
    VehicleConfig raw = def.config;
    raw.tuning.aids.traction = TractionMode::Off; // full motor envelope, wheelspin included
    Simulation sim(settings(raw, TrackLayout::Flat));
    for (int i = 0; i < 120; ++i) sim.tick();
    const float soc0 = sim.vehicle().telemetry().ev.soc;
    CHECK(std::abs(soc0 - def.config.electric.initialSoC) < 1e-3f, "spawn SoC %.4f", soc0);
    sim.requestToggleLowRange();
    sim.tick();
    CHECK(!sim.vehicle().telemetry().lowRange, "an EV has no low range");
    CHECK(std::string(sim.lastMessage()).find("No low range") != std::string::npos, "message '%s'", sim.lastMessage());
    float maxF = 0.0f, maxR = 0.0f, maxKW = 0.0f;
    for (int i = 0; i < 360; ++i) {
        sim.setIntent({.throttle = 1.0f});
        sim.tick();
        const EvTelemetry& e = sim.vehicle().telemetry().ev;
        maxF = std::max(maxF, e.motorTorque[0]);
        maxR = std::max(maxR, e.motorTorque[1]);
        maxKW = std::max(maxKW, e.motorPowerKW);
    }
    const auto& t = sim.vehicle().telemetry();
    std::printf("    3 s full pedal: %.1f km/h, motor torque max %.0f / %.0f N*m, %.0f kW, SoC %.4f -> %.4f\n",
                t.forwardSpeed * 3.6f, maxF, maxR, maxKW, soc0, t.ev.soc);
    CHECK(t.forwardSpeed * 3.6f > 60.0f, "slow launch: %.1f km/h", t.forwardSpeed * 3.6f);
    CHECK(maxF > 390.0f && maxF <= 400.5f && maxR > 440.0f && maxR <= 450.5f, "motor torques %.0f / %.0f", maxF, maxR);
    CHECK(maxKW > 380.0f && maxKW < 410.0f, "peak power %.0f kW", maxKW);
    CHECK(t.ev.soc < soc0 && t.ev.energyOutKWh > 0.2, "SoC did not drop");
    CHECK(t.engineRPM > 4000.0f && t.gear == 1, "motor speed / direction telemetry");

    const double in0 = t.ev.energyInKWh;
    const float soc1 = t.ev.soc;
    bool regenSeen = false, negativePower = false;
    for (int i = 0; i < 600 && sim.vehicle().telemetry().speed > 0.05f; ++i) {
        sim.setIntent({.brake = 0.3f});
        sim.tick();
        regenSeen |= sim.vehicle().telemetry().ev.regenActive;
        negativePower |= sim.vehicle().telemetry().ev.motorPowerKW < -20.0f;
    }
    const auto& b = sim.vehicle().telemetry();
    std::printf("    stop at pedal 0.3: %.2f m/s left, recovered %.4f kWh, SoC %.5f -> %.5f\n", b.speed,
                b.ev.energyInKWh - in0, soc1, b.ev.soc);
    CHECK(regenSeen && negativePower, "no regen while braking");
    CHECK(b.ev.energyInKWh - in0 > 0.01 && b.speed < 0.1f, "no energy recovered or not stopped");
    CHECK(b.ev.soc > soc1, "SoC did not rise under regen (aux load included)");

    sim.requestCycleTransfer();
    sim.tick();
    CHECK(sim.vehicle().evDriveMode() == EvDriveMode::OffRoad &&
              std::abs(sim.vehicle().telemetry().ev.frontShare - def.config.electric.offRoadFrontShare) < 1e-6f,
          "V did not switch to OFF-ROAD");
    sim.requestReset();
    sim.tick();
    CHECK(std::abs(sim.vehicle().telemetry().ev.soc - def.config.electric.initialSoC) < 1e-3f, "reset keeps the SoC");
    CHECK(!sim.hadPhysicsError(), "Jolt error");
}

} // namespace

// Tire thermal model on its own: energy balance of the heat input, exponential cooling with the expected time constant,
// steady state (input = losses), and the compound grip windows.
void tireThermalModel() {
    const TireThermalParams base = tireThermalParams(0.40f, 0.30f, 35.0f, tireCompound(TireSet::Offroad));
    std::printf("    SUV tire: surface %.0f J/K, core %.0f J/K, surface-core %.1f W/K, tread %.3f m^2, sidewalls %.3f m^2\n",
                base.surfaceCapacity, base.coreCapacity, base.surfaceCoreConductance, base.treadArea, base.sidewallArea);
    CHECK(base.surfaceCapacity > 3000.0f && base.surfaceCapacity < 6000.0f, "surface capacity %.0f", base.surfaceCapacity);

    // 1. Energy: no losses, 10 kW of sliding for 10 s -> the stored heat is the tire's share of 100 kJ.
    {
        TireThermalParams p = base;
        p.convection = false;
        p.rimConductance = 0.0f;
        TireThermalState s;
        s.reset(20.0f);
        TireThermalInput in{.contact = true, .load = 5000.0f, .slidingPower = 10000.0f, .rollingSpeed = 0.0f};
        const float dt = 1.0f / 120.0f;
        for (int i = 0; i < 1200; ++i) stepTireThermal(p, in, dt, s);
        const double stored = double(p.surfaceCapacity) * (s.surfaceC - 20.0) + double(p.coreCapacity) * (s.coreC - 20.0);
        const double expected = double(p.slidingHeatShare) * 10000.0 * 10.0;
        std::printf("    energy: stored %.0f J, expected %.0f J (surface %.1f C, core %.1f C)\n", stored, expected, s.surfaceC, s.coreC);
        CHECK(std::abs(stored - expected) < 0.002 * expected, "energy balance off: %.0f vs %.0f J", stored, expected);
        CHECK(s.surfaceC > s.coreC, "the patch heats the surface first");
    }
    // 2. Cooling: surface only (no core coupling), still air at 20 m/s: excess falls to 1/e after C / (h A).
    {
        TireThermalParams p = base;
        p.surfaceCoreConductance = 0.0f;
        const float h = p.naturalConvection + tireForcedConvection(p.diameter, 20.0f);
        const float tau = p.surfaceCapacity / (h * p.treadArea);
        TireThermalState s;
        s.reset(20.0f);
        s.surfaceC = 80.0f;
        TireThermalInput in{.contact = false, .airSpeed = 20.0f, .ambientC = 20.0f};
        const float dt = 1.0f / 120.0f;
        for (int i = 0; i < int(std::lround(tau / dt)); ++i) stepTireThermal(p, in, dt, s);
        const float ratio = (s.surfaceC - 20.0f) / 60.0f;
        std::printf("    cooling at 20 m/s: h %.1f W/(m^2 K), tau %.0f s, excess after tau %.4f (1/e = 0.3679)\n", h, tau, ratio);
        CHECK(h > 15.0f && h < 80.0f, "convection coefficient %.1f out of the expected range", h);
        CHECK(std::abs(ratio - 0.3679f) < 0.003f, "cooling does not follow exp(-t/tau): %.4f", ratio);
    }
    // 3. Steady state: constant cornering-like input on asphalt for 2 h of simulated time; input = losses.
    {
        TireThermalState s;
        s.reset(20.0f);
        TireThermalInput in{.contact = true, .load = 5000.0f, .slidingPower = 3000.0f, .rollingSpeed = 20.0f,
                            .airSpeed = 20.0f, .ambientC = 20.0f, .groundC = 20.0f,
                            .groundEffusivity = surfaceEffusivity(SurfaceId::Asphalt)};
        const float dt = 1.0f / 120.0f;
        for (int i = 0; i < 7200 * 120; ++i) stepTireThermal(base, in, dt, s);
        const float in_ = s.heatSliding + s.heatRolling, out = s.toAir + s.toGround + s.toRim;
        std::printf("    steady: surface %.1f C core %.1f C, in %.0f W, out %.0f W (air %.0f, ground %.0f, rim %.0f), %.0f kPa\n",
                    s.surfaceC, s.coreC, in_, out, s.toAir, s.toGround, s.toRim, s.pressureKPa(base));
        CHECK(std::abs(in_ - out) < 0.01f * in_, "not in balance: in %.0f W, out %.0f W", in_, out);
        CHECK(s.surfaceC > 30.0f && s.coreC > 30.0f, "surface %.1f core %.1f", s.surfaceC, s.coreC);
        CHECK(s.pressureKPa(base) > base.coldPressureKPa, "hot pressure not above cold");
    }
    // 4. Grip windows.
    const TireCompound& off = tireCompound(TireSet::Offroad);
    const TireCompound& road = tireCompound(TireSet::Road);
    const TireCompound& slick = tireCompound(TireSet::Slick);
    CHECK(compoundGripFactor(off, 20.0f) == 1.0f && compoundGripFactor(off, 90.0f) == 1.0f, "off-road not flat in its window");
    CHECK(compoundGripFactor(slick, 95.0f) == 1.0f, "slick not at full grip in its window");
    CHECK(compoundGripFactor(slick, 20.0f) < 0.65f, "cold slick %.3f", compoundGripFactor(slick, 20.0f));
    CHECK(compoundGripFactor(road, 20.0f) > compoundGripFactor(slick, 20.0f), "cold road below cold slick");
    CHECK(compoundGripFactor(slick, 140.0f) < 0.9f && compoundGripFactor(slick, 140.0f) > 0.7f, "overheated slick %.3f",
          compoundGripFactor(slick, 140.0f));
    for (float t = -40.0f; t < 200.0f; t += 1.0f)
        for (const TireCompound* c : {&off, &road, &slick})
            CHECK(compoundGripFactor(*c, t) > 0.5f && compoundGripFactor(*c, t) <= 1.0f, "grip out of range at %.0f C", t);
    CHECK(tireSetFromKey("slick") == TireSet::Slick && tireSetFromKey("summer") == TireSet::Count, "tire set keys");
}

// Tire sets on the vehicle: off-road leaves the vehicle's tire exactly as it is; road / slick change the stop on
// asphalt and gravel in the expected direction; temperature acts through the grip; garage and vehicle.json.
void tireSets() {
    const VehicleDefinition suv = loadOne(kAssets, "trailforge_suv");
    CHECK(Vehicle::tireSetTuning(suv.config.tuning, TireSet::Offroad) == suv.config.tuning, "off-road set changes the tire");

    // 60-0 km/h with ABS on the asphalt (lane 0) and gravel (lane 2) lanes; tires preheated or at the air temperature.
    auto stop = [&](TireSet set, int lane, bool warm, float* surfaceAfter) {
        VehicleConfig c = suv.config;
        c.tireSet = set;
        SimulationSettings s = settings(c, TrackLayout::RoadLanes);
        Simulation sim(s);
        sim.requestTeleport(lane);
        for (int i = 0; i < 120; ++i) sim.tick();
        sim.requestAidControls({true, TractionMode::On, 0});
        const TireCompound& tc = tireCompound(set);
        const float t = warm ? 0.5f * (std::max(tc.optimumLowC, 20.0f) + tc.optimumHighC) : 20.0f;
        sim.setTireTemperatures(t, t);
        while (sim.vehicle().telemetry().speed < 60.0f / 3.6f && sim.simTime() < 30.0) {
            sim.setIntent({.throttle = 1.0f});
            sim.tick();
        }
        sim.setTireTemperatures(t, t);
        const float v0 = sim.vehicle().telemetry().speed;
        float path = 0.0f;
        for (int i = 0; i < 2400 && sim.vehicle().telemetry().speed > 0.05f; ++i) {
            sim.setIntent({.brake = 1.0f});
            sim.tick();
            path += sim.vehicle().telemetry().speed * float(sim.fixedStep());
        }
        float hottest = 0.0f;
        for (const WheelTelemetry& w : sim.vehicle().telemetry().wheels) hottest = std::max(hottest, w.tireSurfaceC);
        if (surfaceAfter) *surfaceAfter = hottest;
        return path * (60.0f / 3.6f) * (60.0f / 3.6f) / (v0 * v0);
    };
    float offHot = 0.0f;
    const float offAsphalt = stop(TireSet::Offroad, 0, false, &offHot);
    const float roadAsphalt = stop(TireSet::Road, 0, true, nullptr);
    const float slickWarm = stop(TireSet::Slick, 0, true, nullptr);
    const float slickCold = stop(TireSet::Slick, 0, false, nullptr);
    const float offGravel = stop(TireSet::Offroad, 2, false, nullptr);
    const float slickGravel = stop(TireSet::Slick, 2, true, nullptr);
    std::printf("    60-0 asphalt: off-road %.2f m (hottest tire after %.1f C), road warm %.2f m, slick warm %.2f m, "
                "slick cold %.2f m; gravel: off-road %.2f m, slick warm %.2f m\n",
                offAsphalt, offHot, roadAsphalt, slickWarm, slickCold, offGravel, slickGravel);
    CHECK(roadAsphalt < 0.95f * offAsphalt, "road tires do not stop shorter on asphalt");
    CHECK(slickWarm < roadAsphalt, "warm slicks do not stop shorter than road tires");
    CHECK(slickCold > 1.3f * slickWarm, "cold slicks not clearly worse than warm ones");
    CHECK(slickGravel > 1.2f * offGravel, "slicks not worse on gravel");
    CHECK(offHot > 20.5f, "braking did not heat the tires (%.2f C)", offHot);

    // Garage: the set is a rebuild choice; vehicle.json tires.set.
    VehicleSetup setup;
    setup.values["tires.set"] = 2.0f;
    VehicleConfig c;
    std::vector<std::string> warnings;
    CHECK(buildSetupConfig(suv, setup, c, warnings) && c.tireSet == TireSet::Slick, "garage tire set");
    CHECK(Vehicle::needsRebuild(suv.config, c), "a tire set change must rebuild the vehicle");
    const std::string dir = (std::filesystem::temp_directory_path() / "tf_tire_set_test").string();
    std::filesystem::create_directories(dir + "/rwd_test");
    std::ifstream src(kTestData + "/rwd_test/vehicle.json");
    std::string text((std::istreambuf_iterator<char>(src)), std::istreambuf_iterator<char>());
    const std::size_t at = text.find("\"tires\": {");
    CHECK(at != std::string::npos, "rwd_test has no tires block");
    for (const char* value : {"road", "summer"}) {
        std::string t = text;
        t.insert(at + 10, std::string("\"set\": \"") + value + "\", ");
        std::ofstream(dir + "/rwd_test/vehicle.json") << t;
        VehicleDefinition d;
        std::vector<std::string> errors;
        const bool ok = loadVehicleDefinition(dir + "/rwd_test/vehicle.json", d, errors);
        if (std::string(value) == "road") CHECK(ok && d.config.tireSet == TireSet::Road, "tires.set road not read");
        else CHECK(!ok, "unknown tire set accepted");
    }
    std::filesystem::remove_all(dir);
}

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"tire_thermal_model", tireThermalModel},
        {"tire_sets", tireSets},
        {"vehicle_catalog_sweep", vehicleCatalogSweep},
        {"vehicle_definition_validation", vehicleDefinitionValidation},
        {"drivetrain_layouts", drivetrainLayouts},
        {"garage_setups", garageSetups},
        {"suv_definition_matches_builtin", suvDefinitionMatchesBuiltin},
        {"electric_definition_validation", electricDefinitionValidation},
        {"ev_motor_curve", evMotorCurve},
        {"ev_torque_split", evTorqueSplit},
        {"ev_regen_blend", evRegenBlend},
        {"ev_battery", evBattery},
        {"ev_simulation", evSimulation},
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
