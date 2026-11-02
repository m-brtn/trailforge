// trailforge_probe water [entry|rest|all] - the vehicle driving into water on the flat pad (see tools/SimProbe.cpp).
//   entry  still water of 0.3 / 0.5 / 0.7 m over the concrete pad for z > 0 (a vertical water front, a test
//          setup: real water would run out); the vehicle reaches 20 / 40 / 60 km/h on the dry pad, then coasts
//          (throttle and brake released) into it straight and at 20 deg. Reported from the first wet tick: peak and
//          mean deceleration over 0.5 s / 1 s (g, 50 ms average), speed after 1 s, nose-up pitch change (max, min),
//          peak yaw rate and the heading change after 2 s, peak bow lift, tire and body drag (kN). A dry coast row
//          is the baseline (rolling, engine braking, air drag).
//   rest   standing in still water 0.3 .. 1.4 m deep: buoyancy / weight, wheels on the ground, pitch.
//   all    both
// Vehicles: TRAILFORGE_VEHICLE, else the SUV. WATER_TRACE=1: per-tick trace of the 40 km/h / 0.5 m straight entry.

#include "sim/Simulation.h"
#include "vehicle/VehicleDefinition.h"

#include <Jolt/Physics/Body/BodyInterface.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <string>

using namespace tf;

namespace {

constexpr float kRad = 57.2957795f;
constexpr float kG = 9.81f;

VehicleConfig waterVehicle(std::string& name) {
    const char* env = std::getenv("TRAILFORGE_VEHICLE");
    const std::string path = env ? env : TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_suv/vehicle.json";
    if (path == "builtin") {
        name = "builtin";
        return VehicleConfig{};
    }
    VehicleDefinition def;
    std::vector<std::string> errors;
    if (!loadVehicleDefinition(path, def, errors)) {
        for (const auto& e : errors) std::fprintf(stderr, "%s\n", e.c_str());
        std::exit(2);
    }
    name = def.id;
    return def.config;
}

std::unique_ptr<Simulation> pool(const VehicleConfig& cfg, float depth) {
    SimulationSettings s;
    s.vehicle = cfg;
    s.layout = TrackLayout::Flat;
    s.water = [depth](float, float z) -> std::optional<float> {
        if (depth <= 0.0f || z <= 0.0f) return std::nullopt;
        return depth;
    };
    return std::make_unique<Simulation>(s);
}

float pitchDeg(const Simulation& sim) {
    return std::asin(std::clamp(float(sim.vehicle().bodyTransform().GetAxisZ().GetY()), -1.0f, 1.0f)) * kRad;
}
float headingDeg(const Simulation& sim) {
    const JPH::Vec3 f = sim.vehicle().bodyTransform().GetAxisZ();
    return std::atan2(f.GetX(), f.GetZ()) * kRad;
}

struct EntryResult {
    bool entered = false;
    float speed0 = 0, peakG = 0, meanG05 = 0, meanG1 = 0, speed1 = 0;
    float pitchUp = 0, pitchDown = 0, yawRate = 0, dHeading = 0;
    float bowLift = 0, wheelDrag = 0, bodyDrag = 0;
};

EntryResult entry(const VehicleConfig& cfg, float depth, float kmh, float angleDeg, bool trace) {
    auto sim = pool(cfg, depth);
    const float yaw = angleDeg / kRad;
    const float run = 160.0f; // m of dry run-up
    TrackFeature f;
    f.name = "probe";
    f.position = JPH::Vec3(-run * std::sin(yaw), 0.0f, -run * std::cos(yaw));
    f.yaw = yaw;
    sim->requestPlace(f);
    sim->setIntent({.brake = 1.0f});
    for (int i = 0; i < 120; ++i) sim->tick();
    const float target = kmh / 3.6f, dt = float(sim->fixedStep());
    JPH::BodyInterface& bi = sim->world().bodies();
    const JPH::BodyID id = sim->vehicle().bodyId();
    // Reach the speed; release the pedals 0.5 s before the front bumper (~2.4 m ahead of the origin) reaches the water,
    // so the coast-down transient is over at the entry.
    float integral = 0.0f;
    for (int i = 0; i < 120 * 40; ++i) {
        const JPH::Vec3 p(sim->vehicle().bodyTransform().GetTranslation());
        const float v = sim->vehicle().telemetry().forwardSpeed;
        const float toWater = -p.GetZ() / std::max(std::cos(yaw), 0.1f) - 2.4f;
        if (toWater < 0.5f * target) break;
        const float err = target - v;
        integral = std::clamp(integral + err * dt, -20.0f, 20.0f);
        sim->setIntent({.throttle = std::clamp(0.1f + 0.4f * err + 0.05f * integral, 0.0f, 1.0f)});
        sim->tick();
    }
    sim->setIntent({});
    EntryResult r;
    std::deque<float> speeds; // horizontal speed history for the 50 ms average deceleration
    float pitch0 = 0, heading0 = 0, t = -1.0f, sum05 = 0, sum1 = 0;
    int n05 = 0, n1 = 0;
    auto hspeed = [&] {
        const JPH::Vec3 v = bi.GetLinearVelocity(id);
        return std::hypot(v.GetX(), v.GetZ());
    };
    for (int i = 0; i < 120 * 6; ++i) {
        sim->tick();
        const WaterForceResult& w = sim->waterForces();
        const float s = hspeed();
        speeds.push_back(s);
        if (speeds.size() > 7) speeds.pop_front();
        if (t < 0.0f) {
            const JPH::Vec3 p(sim->vehicle().bodyTransform().GetTranslation());
            const bool started = depth > 0.0f ? w.wet : p.GetZ() + 2.4f * std::cos(yaw) > 0.0f;
            if (!started) {
                pitch0 = pitchDeg(*sim);
                heading0 = headingDeg(*sim);
                r.speed0 = s;
                continue;
            }
            t = 0.0f;
            r.entered = true;
        }
        t += dt;
        const float g = speeds.size() == 7 ? (speeds.front() - speeds.back()) / (6.0f * dt) / kG : 0.0f;
        r.peakG = std::max(r.peakG, g);
        if (t <= 0.5f) sum05 += g, ++n05;
        if (t <= 1.0f) sum1 += g, ++n1;
        if (t <= 1.0f) r.speed1 = s;
        const float dp = pitchDeg(*sim) - pitch0;
        r.pitchUp = std::max(r.pitchUp, dp);
        r.pitchDown = std::min(r.pitchDown, dp);
        const float wy = bi.GetAngularVelocity(id).GetY() * kRad;
        if (t <= 2.0f) {
            if (std::abs(wy) > std::abs(r.yawRate)) r.yawRate = wy;
            r.dHeading = headingDeg(*sim) - heading0;
        }
        r.bowLift = std::max(r.bowLift, w.bowLift);
        r.wheelDrag = std::max(r.wheelDrag, w.wheelDrag);
        r.bodyDrag = std::max(r.bodyDrag, w.bodyDrag);
        if (trace && t <= 2.0f)
            std::printf("    t %.3f  v %5.2f m/s  decel %5.2f g  pitch %+5.2f deg  wheel drag %5.1f kN  body drag %5.1f kN  bow "
                        "lift %5.2f kN  buoyancy %5.2f kN  body depth %.2f m  tire depth %.2f m\n",
                        double(t), double(s), double(g), double(dp), double(w.wheelDrag / 1000), double(w.bodyDrag / 1000),
                        double(w.bowLift / 1000), double(w.buoyancy / 1000), double(w.bodyDepth), double(w.wheelDepth));
        if (t > 2.0f && s < 0.3f) break;
    }
    r.meanG05 = n05 ? sum05 / n05 : 0.0f;
    r.meanG1 = n1 ? sum1 / n1 : 0.0f;
    return r;
}

void entryTable(const VehicleConfig& cfg) {
    std::printf("\nentry (coasting into still water, flat concrete pad; peak = max 50 ms average; pitch + = nose up "
                "relative to the pose before the entry; yaw + = left)\n");
    std::printf("  depth  speed  angle | peak g  mean 0.5s  mean 1s  v@1s km/h | pitch up  down deg | yaw rate deg/s  "
                "dHead 2s deg | bow lift  tire drag  body drag kN\n");
    for (float depth : {0.0f, 0.3f, 0.5f, 0.7f})
        for (float kmh : {20.0f, 40.0f, 60.0f})
            for (float angle : {0.0f, 20.0f}) {
                if (depth == 0.0f && angle != 0.0f) continue;
                // depth 0: the dry baseline, measured from where the water front would be.
                const EntryResult r = entry(cfg, depth, kmh, angle, false);
                std::printf("  %4.1f m  %3.0f   %4.0f  | %5.2f   %5.2f     %5.2f    %5.1f    | %+6.2f  %+6.2f   | %+7.1f        "
                            "%+6.1f      | %6.2f    %6.1f     %6.1f\n",
                            double(depth), double(kmh), double(angle), double(r.peakG), double(r.meanG05), double(r.meanG1),
                            double(r.speed1 * 3.6f), double(r.pitchUp), double(r.pitchDown), double(r.yawRate),
                            double(r.dHeading), double(r.bowLift / 1000), double(r.wheelDrag / 1000),
                            double(r.bodyDrag / 1000));
            }
}

void restTable(const VehicleConfig& cfg) {
    std::printf("\nrest (standing in still water, brakes held; weight %.1f kN)\n", double(cfg.mass * kG / 1000));
    std::printf("  depth | buoyancy kN  / weight | wheels on ground | pitch deg | displaced m^3 | body depth m | step ms\n");
    for (float depth : {0.0f, 0.3f, 0.5f, 0.7f, 0.9f, 1.1f, 1.4f}) {
        auto sim = pool(cfg, depth);
        TrackFeature f;
        f.name = "probe";
        f.position = JPH::Vec3(0.0f, 0.0f, 20.0f);
        sim->requestPlace(f);
        sim->setIntent({.brake = 1.0f});
        // 6 s; the buoyancy is the mean of the last second (a floating body still bobs: the heave drag is quadratic).
        float buoyancy = 0.0f;
        for (int i = 0; i < 120 * 6; ++i) {
            sim->tick();
            if (i >= 120 * 5) buoyancy += sim->waterForces().buoyancy / 120.0f;
        }
        WaterForceResult w = sim->waterForces();
        w.buoyancy = buoyancy;
        int contacts = 0;
        for (const auto& c : sim->vehicle().contacts()) contacts += c.contact;
        std::printf("  %3.1f m |   %6.2f    %5.2f   |        %d         |  %+5.2f   |    %5.2f      |   %4.2f       | %.3f\n",
                    double(depth), double(w.buoyancy / 1000), double(w.buoyancy / (cfg.mass * kG)), contacts,
                    double(pitchDeg(*sim)), double(w.displacedVolume), double(w.bodyDepth), sim->timing().avgStepMs);
    }
}

} // namespace

void waterProbe(int argc, char** argv) {
    const std::string section = argc > 2 ? argv[2] : "all";
    std::string name;
    const VehicleConfig cfg = waterVehicle(name);
    {
        auto sim = pool(cfg, 0.5f);
        std::printf("water probe: %s, %.0f kg, body %zu volume cells (%.2f m^3), %zu surface patches (%.1f m^2)\n",
                    name.c_str(), double(cfg.mass), sim->waterModel().cellCount(), double(sim->waterModel().bodyVolume()),
                    sim->waterModel().patchCount(), double(sim->waterModel().surfaceArea()));
    }
    if (std::getenv("WATER_TRACE")) entry(cfg, 0.5f, 40.0f, 0.0f, true);
    if (section == "entry" || section == "all") entryTable(cfg);
    if (section == "rest" || section == "all") restTable(cfg);
}
