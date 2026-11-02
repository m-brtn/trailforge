// Water forces on the vehicle (sim/WaterForces.h). Usage: trailforge_water_tests <test_name>
// (CTest registers each test separately). Tolerances are engineering, not bitwise.

#include "sim/Simulation.h"
#include "sim/WaterForces.h"
#include "vehicle/VehicleDefinition.h"

#include <Jolt/Physics/Body/BodyInterface.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
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

constexpr float kG = 9.81f;
constexpr float kRad = 57.2957795f;

bool near(float a, float b, float tol) { return std::abs(a - b) <= tol; }

// A 2 x 1 x 4 m box (body origin at its center) in still water at the given surface height.
WaterForceResult boxIn(float surface, JPH::Vec3Arg velocity, const VehicleWater& p,
                       const std::vector<WaterBox>& boxes = {{JPH::Vec3::sZero(), JPH::Vec3(1.0f, 0.5f, 2.0f)}},
                       WaterSurface water = {}) {
    WaterForces wf;
    wf.setBody(boxes, p);
    WaterBodyState s;
    s.linearVelocity = velocity;
    if (!water) water = [surface](float, float) -> std::optional<float> { return surface; };
    return wf.compute(s, {}, water);
}

void waterForcesBox() {
    VehicleWater p;
    p.displacement = 1.0f;
    p.skinCf = 0.0f;
    p.frontSlopeDeg = 0.0f;
    // Buoyancy: half submerged = rho g x 4 m^3, at the center (no torque, no side force).
    WaterForceResult r = boxIn(0.0f, JPH::Vec3::sZero(), p);
    std::printf("  half submerged: buoyancy %.0f N (expected %.0f), torque %.1f N m\n", double(r.buoyancy),
                double(1000.0f * kG * 4.0f), double(r.torque.Length()));
    CHECK(near(r.buoyancy, 1000.0f * kG * 4.0f, 1.0f), "buoyancy %.1f", double(r.buoyancy));
    CHECK(r.torque.Length() < 1.0f && std::abs(r.force.GetX()) < 1e-3f && std::abs(r.force.GetZ()) < 1e-3f,
          "still water pushes sideways: force %.3f %.3f, torque %.3f", double(r.force.GetX()), double(r.force.GetZ()),
          double(r.torque.Length()));
    // Overlapping boxes count their common volume once.
    WaterForceResult twice = boxIn(0.0f, JPH::Vec3::sZero(), p,
                                   {{JPH::Vec3::sZero(), JPH::Vec3(1.0f, 0.5f, 2.0f)}, {JPH::Vec3::sZero(), JPH::Vec3(1.0f, 0.5f, 2.0f)}});
    CHECK(near(twice.buoyancy, r.buoyancy, 1.0f), "overlap counted twice: %.1f vs %.1f", double(twice.buoyancy), double(r.buoyancy));
    // Dry: nothing.
    r = boxIn(-1.0f, JPH::Vec3(0.0f, 0.0f, 5.0f), p);
    CHECK(!r.wet && r.force.Length() == 0.0f, "dry box gets %.1f N", double(r.force.Length()));
    // Fully submerged at 5 m/s forward: the front face takes 1/2 rho Cd v^2 x 2 m^2 (the sides are tangential).
    r = boxIn(2.0f, JPH::Vec3(0.0f, 0.0f, 5.0f), p);
    const float drag = 0.5f * 1000.0f * p.frontalCd * 25.0f * 2.0f;
    std::printf("  submerged at 5 m/s: drag %.0f N (expected %.0f), lift %.0f N\n", double(-r.force.GetZ()), double(drag),
                double(r.bowLift));
    CHECK(near(-r.force.GetZ(), drag, 1.0f) && r.bowLift == 0.0f, "drag %.1f, lift %.1f", double(-r.force.GetZ()),
          double(r.bowLift));
    // A sloped front lifts by tan(slope) x the front pressure force.
    p.frontSlopeDeg = 35.0f;
    r = boxIn(2.0f, JPH::Vec3(0.0f, 0.0f, 5.0f), p);
    CHECK(near(r.bowLift, drag * std::tan(35.0f / kRad), 1.0f), "bow lift %.1f", double(r.bowLift));
    CHECK(r.torque.GetX() < 0.0f, "bow lift raises the nose: torque x %.1f", double(r.torque.GetX()));
    // Deeper on the left (+X): more drag on the left side turns the body left (yaw torque about +Y > 0); level water
    // gives no yaw.
    p.frontSlopeDeg = 0.0f;
    const WaterSurface tilted = [](float x, float) -> std::optional<float> { return 0.1f * x; };
    r = boxIn(0.0f, JPH::Vec3(0.0f, 0.0f, 3.0f), p, {{JPH::Vec3::sZero(), JPH::Vec3(1.0f, 0.5f, 2.0f)}}, tilted);
    const WaterForceResult level = boxIn(0.0f, JPH::Vec3(0.0f, 0.0f, 3.0f), p);
    std::printf("  deeper on the left: yaw torque %.0f N m (level water %.2f)\n", double(r.torque.GetY()),
                double(level.torque.GetY()));
    CHECK(r.torque.GetY() > 50.0f && std::abs(level.torque.GetY()) < 1.0f, "yaw torque %.1f / %.1f", double(r.torque.GetY()),
          double(level.torque.GetY()));
    // Flow: a box at rest in a 2 m/s current along +X is pushed downstream.
    WaterForces wf;
    wf.setBody(std::vector<WaterBox>{{JPH::Vec3::sZero(), JPH::Vec3(1.0f, 0.5f, 2.0f)}}, p);
    r = wf.compute({}, {}, [](float, float) -> std::optional<float> { return 0.0f; },
                   [](float, float) { return JPH::Vec3(2.0f, 0.0f, 0.0f); });
    CHECK(r.force.GetX() > 0.0f, "current pushes %.1f N", double(r.force.GetX()));
    // Tires: a 0.4 m wheel half submerged displaces half its cylinder.
    WaterWheel wheel;
    wheel.hub = JPH::Vec3(10.0f, 5.0f, 0.0f);
    r = wf.compute({}, std::span<const WaterWheel>(&wheel, 1), [](float x, float) -> std::optional<float> {
        if (x < 8.0f) return std::nullopt; // the box stays dry
        return 5.0f;
    });
    const float half = 0.5f * 3.14159265f * 0.16f * 0.3f * 1000.0f * kG;
    CHECK(near(r.buoyancy, half, 1.0f) && near(r.wheelDepth, 0.4f, 1e-4f), "tire buoyancy %.1f (expected %.1f)",
          double(r.buoyancy), double(half));
}

VehicleConfig suv() {
    VehicleDefinition d;
    std::vector<std::string> errors;
    if (!loadVehicleDefinition(TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_suv/vehicle.json", d, errors)) {
        for (const auto& e : errors) std::printf("  %s\n", e.c_str());
        std::exit(1);
    }
    return d.config;
}

struct Entry {
    float speed0 = 0, peakG = 0, speed1 = 0, pitchUp = 0, pitchDown = 0, dHeading = 0, bowLift = 0;
};

// Coasting at kmh into still water `depth` deep for z > 0 on the flat pad, heading angleDeg off the water front's
// normal (left positive); measured from the first wet tick (same procedure as trailforge_probe water).
Entry enter(float depth, float kmh, float angleDeg) {
    SimulationSettings s;
    s.vehicle = suv();
    s.layout = TrackLayout::Flat;
    s.water = [depth](float, float z) -> std::optional<float> {
        if (z <= 0.0f) return std::nullopt;
        return depth;
    };
    Simulation sim(s);
    const float yaw = angleDeg / kRad;
    TrackFeature f;
    f.name = "test";
    f.position = JPH::Vec3(-120.0f * std::sin(yaw), 0.0f, -120.0f * std::cos(yaw));
    f.yaw = yaw;
    sim.requestPlace(f);
    sim.setIntent({.brake = 1.0f});
    for (int i = 0; i < 120; ++i) sim.tick();
    const float target = kmh / 3.6f, dt = float(sim.fixedStep());
    float integral = 0.0f;
    for (int i = 0; i < 120 * 40; ++i) {
        const JPH::Vec3 p(sim.vehicle().bodyTransform().GetTranslation());
        if (-p.GetZ() / std::cos(yaw) - 2.4f < 0.5f * target) break;
        const float err = target - sim.vehicle().telemetry().forwardSpeed;
        integral = std::clamp(integral + err * dt, -20.0f, 20.0f);
        sim.setIntent({.throttle = std::clamp(0.1f + 0.4f * err + 0.05f * integral, 0.0f, 1.0f)});
        sim.tick();
    }
    sim.setIntent({});
    JPH::BodyInterface& bi = sim.world().bodies();
    const JPH::BodyID id = sim.vehicle().bodyId();
    auto pitch = [&] { return std::asin(float(sim.vehicle().bodyTransform().GetAxisZ().GetY())) * kRad; };
    auto heading = [&] {
        const JPH::Vec3 fw = sim.vehicle().bodyTransform().GetAxisZ();
        return std::atan2(fw.GetX(), fw.GetZ()) * kRad;
    };
    auto hspeed = [&] {
        const JPH::Vec3 v = bi.GetLinearVelocity(id);
        return std::hypot(v.GetX(), v.GetZ());
    };
    Entry e;
    std::deque<float> speeds;
    float pitch0 = 0, heading0 = 0, t = -1.0f;
    for (int i = 0; i < 120 * 5 && t < 2.0f; ++i) {
        sim.tick();
        speeds.push_back(hspeed());
        if (speeds.size() > 7) speeds.pop_front();
        if (t < 0.0f) {
            if (!sim.waterForces().wet) {
                pitch0 = pitch();
                heading0 = heading();
                e.speed0 = hspeed();
                continue;
            }
            t = 0.0f;
        }
        t += dt;
        if (speeds.size() == 7) e.peakG = std::max(e.peakG, (speeds.front() - speeds.back()) / (6.0f * dt) / kG);
        if (t <= 1.0f) e.speed1 = hspeed();
        e.pitchUp = std::max(e.pitchUp, pitch() - pitch0);
        e.pitchDown = std::min(e.pitchDown, pitch() - pitch0);
        e.dHeading = heading() - heading0;
        e.bowLift = std::max(e.bowLift, sim.waterForces().bowLift);
    }
    CHECK(!sim.hadPhysicsError(), "physics error");
    return e;
}

void waterEntry() {
    // Straight into 0.5 m at 40 km/h: hard deceleration, no yaw. Into 0.7 m at 40 km/h: the body front is under water,
    // the bow lift raises the nose. At 20 deg the side that enters first is held back: the heading turns toward the
    // water front's normal (to the right here).
    const Entry a = enter(0.5f, 40.0f, 0.0f);
    std::printf("  0.5 m, 40 km/h straight: %.1f km/h, peak %.2f g, %.1f km/h after 1 s, pitch %+.2f / %+.2f deg, "
                "heading %+.2f deg\n",
                double(a.speed0 * 3.6f), double(a.peakG), double(a.speed1 * 3.6f), double(a.pitchUp), double(a.pitchDown),
                double(a.dHeading));
    CHECK(a.peakG > 0.5f && a.peakG < 2.5f, "peak deceleration %.2f g", double(a.peakG));
    CHECK(a.speed1 < 0.75f * a.speed0, "speed after 1 s %.1f of %.1f m/s", double(a.speed1), double(a.speed0));
    CHECK(std::abs(a.dHeading) < 0.3f, "straight entry turns %.2f deg", double(a.dHeading));
    const Entry b = enter(0.7f, 40.0f, 0.0f);
    std::printf("  0.7 m, 40 km/h straight: peak %.2f g, pitch %+.2f / %+.2f deg, bow lift %.1f kN\n", double(b.peakG),
                double(b.pitchUp), double(b.pitchDown), double(b.bowLift / 1000.0f));
    CHECK(b.peakG > a.peakG, "0.7 m decelerates no more than 0.5 m: %.2f vs %.2f g", double(b.peakG), double(a.peakG));
    CHECK(b.bowLift > 1000.0f && b.pitchUp > 0.3f, "bow lift %.0f N, nose up %.2f deg", double(b.bowLift), double(b.pitchUp));
    const Entry c = enter(0.5f, 40.0f, 20.0f);
    std::printf("  0.5 m, 40 km/h at 20 deg: peak %.2f g, heading %+.2f deg\n", double(c.peakG), double(c.dHeading));
    CHECK(c.dHeading < -0.5f, "angled entry heading change %.2f deg", double(c.dHeading));
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"water_forces_box", waterForcesBox},
        {"water_entry", waterEntry},
    };
    if (argc < 2) {
        std::printf("usage: trailforge_water_tests <test>\n");
        return 2;
    }
    for (const auto& [name, fn] : tests)
        if (std::strcmp(argv[1], name) == 0) {
            fn();
            std::printf("%s: %s (%d failures)\n", name, g_failures ? "FAILED" : "passed", g_failures);
            return g_failures ? 1 : 0;
        }
    std::printf("unknown test %s\n", argv[1]);
    return 2;
}
