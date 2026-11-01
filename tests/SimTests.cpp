// Headless TrailForge physics checks. Usage: trailforge_tests <test_name>
// (CTest registers each test separately). Tolerances are engineering, not bitwise.

#include "input/KeyboardPedals.h"
#include "sim/BodyMud.h"
#include "sim/DriftScore.h"
#include "sim/Autopilot.h"
#include "sim/FixedStepClock.h"
#include "sim/LapTimer.h"
#include "sim/Simulation.h"
#include "sim/WheelSpray.h"
#include "track/MiniTrack.h"
#include "track/TestTrack.h"
#include "vehicle/VehicleDefinition.h"
#include "core/Json.h"

#include <algorithm>
#include <array>
#include <span>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
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

bool finite(JPH::Vec3Arg v) { return std::isfinite(v.GetX()) && std::isfinite(v.GetY()) && std::isfinite(v.GetZ()); }

SimulationSettings settingsFor(TrackLayout layout) {
    SimulationSettings s;
    s.layout = layout;
    return s;
}

void run(Simulation& sim, double seconds, const DriverIntent& intent = {}) {
    sim.setIntent(intent);
    const int n = int(std::lround(seconds / sim.fixedStep()));
    for (int i = 0; i < n; ++i) sim.tick();
}

JPH::Vec3 position(const Simulation& sim) { return sim.currentSnapshot().body.position; }
JPH::Vec3 velocity(Simulation& sim) { return sim.world().bodies().GetLinearVelocity(sim.vehicle().bodyId()); }

void printWheels(const VehicleTelemetry& t) {
    for (int i = 0; i < kWheelCount; ++i) {
        const auto& w = t.wheels[i];
        std::printf("    %s contact=%d len=%.3f m load=%.0f N slipL=%.3f slipA=%.1f deg\n", kWheelNames[i], w.contact,
                    w.suspensionLength, w.load, w.longitudinalSlip, w.lateralSlipDeg);
    }
}

// ---------------------------------------------------------------------------

void settlesOnFlatGround() {
    Simulation sim(settingsFor(TrackLayout::Flat));
    const JPH::Vec3 start = position(sim);
    run(sim, 4.0);
    const auto& t = sim.vehicle().telemetry();
    const JPH::Vec3 p = position(sim);
    const JPH::Vec3 v = velocity(sim);
    printWheels(t);
    std::printf("    pos=(%.3f %.3f %.3f) |v|=%.4f m/s upDot=%.4f\n", p.GetX(), p.GetY(), p.GetZ(), v.Length(), t.upDot);

    CHECK(finite(p) && finite(v), "NaN/Inf in body state");
    CHECK(!sim.hadPhysicsError(), "Jolt reported update error");
    CHECK(v.Length() < 0.05f, "body still moving: %.4f m/s", v.Length());
    CHECK(std::abs(p.GetX() - start.GetX()) < 0.05f && std::abs(p.GetZ() - start.GetZ()) < 0.05f,
          "drifted horizontally");
    CHECK(p.GetY() > 0.4f && p.GetY() < 1.2f, "unexpected ride height %.3f", p.GetY());
    CHECK(t.upDot > 0.999f, "body not level: %.4f", t.upDot);
    float total = 0.0f;
    for (const auto& w : t.wheels) {
        CHECK(w.contact, "wheel without contact");
        CHECK(!w.bumpStop, "suspension on bump stop at rest");
        total += w.load;
    }
    const float weight = sim.vehicleConfig().mass * 9.81f;
    std::printf("    sum(load)=%.0f N, m*g=%.0f N\n", total, weight);
    CHECK(std::abs(total - weight) < 0.05f * weight, "sum of wheel loads %.0f N vs weight %.0f N", total, weight);
}

void throttleMovesVehicle() {
    Simulation sim(settingsFor(TrackLayout::Flat));
    run(sim, 1.0);
    const JPH::Vec3 p0 = position(sim);
    run(sim, 4.0, DriverIntent{.throttle = 1.0f});
    const auto& t = sim.vehicle().telemetry();
    const JPH::Vec3 p1 = position(sim);
    std::printf("    forwardSpeed=%.2f m/s rpm=%.0f gear=%d dz=%.2f m\n", t.forwardSpeed, t.engineRPM, t.gear,
                p1.GetZ() - p0.GetZ());
    printWheels(t);
    CHECK(finite(p1), "NaN");
    CHECK(t.forwardSpeed > 5.0f, "too slow after 4 s full throttle: %.2f m/s", t.forwardSpeed);
    CHECK(p1.GetZ() - p0.GetZ() > 8.0f, "did not travel forward (+Z)");
    CHECK(std::abs(p1.GetX() - p0.GetX()) < 0.5f, "pulled sideways: %.2f m", p1.GetX() - p0.GetX());
    CHECK(t.gear >= 1, "not in a forward gear: %d", t.gear);
}

void brakeReducesSpeed() {
    Simulation sim(settingsFor(TrackLayout::Flat));
    run(sim, 0.5);
    run(sim, 4.0, DriverIntent{.throttle = 1.0f});
    const float v0 = sim.vehicle().telemetry().forwardSpeed;
    run(sim, 1.0, DriverIntent{.brake = 1.0f});
    const float v1 = sim.vehicle().telemetry().forwardSpeed;
    run(sim, 3.0, DriverIntent{.brake = 1.0f});
    const float v2 = sim.vehicle().telemetry().forwardSpeed;
    std::printf("    v before=%.2f m/s, after 1 s brake=%.2f, after 4 s=%.3f\n", v0, v1, v2);
    CHECK(v0 > 5.0f, "precondition: speed %.2f", v0);
    // 1 s of braking with mu~1: dv ~ 5..10 m/s. Require noticeable deceleration.
    CHECK(v1 < v0 - 3.0f, "brake too weak: %.2f -> %.2f", v0, v1);
    CHECK(std::abs(v2) < 0.2f, "did not stop: %.3f", v2);
}

// Brake without throttle in low range: the idling engine must not turn the wheels through the clutch
// (previously speed under full brake grew until the automatic reached neutral).
void brakeHoldsInLowRange() {
    Simulation sim(settingsFor(TrackLayout::Flat));
    run(sim, 0.5);
    sim.requestToggleLowRange();
    run(sim, 3.0, DriverIntent{.throttle = 1.0f});
    const float v0 = sim.vehicle().telemetry().forwardSpeed;
    sim.setIntent(DriverIntent{.brake = 1.0f});
    float vMin = v0, worstRise = 0.0f, worstOverspin = 0.0f;
    double stopTime = -1.0;
    for (int i = 0; i < int(4.0 / sim.fixedStep()); ++i) {
        sim.tick();
        const auto& t = sim.vehicle().telemetry();
        if (t.command.throttle > 0.0f) { vMin = t.forwardSpeed; continue; } // throttle is still ramping down
        if (t.forwardSpeed > 0.5f) {
            vMin = std::fmin(vMin, t.forwardSpeed);
            worstRise = std::fmax(worstRise, t.forwardSpeed - vMin);
        }
        // A wheel the engine drives against the brake spins faster than rolling.
        for (const auto& w : t.wheels)
            worstOverspin = std::fmax(worstOverspin, w.angularVelocity * sim.vehicleConfig().axles[0].wheelRadius - t.forwardSpeed);
        if (stopTime < 0.0 && std::abs(t.forwardSpeed) < 0.05f) stopTime = (i + 1) * sim.fixedStep();
    }
    const auto& t = sim.vehicle().telemetry();
    std::printf("    low range: v0=%.2f m/s, stop after %.2f s, worst speed rise %.3f m/s, worst wheel overspin %.2f m/s, "
                "final v=%.3f gear=%d\n",
                v0, stopTime, worstRise, worstOverspin, t.forwardSpeed, t.gear);
    CHECK(t.lowRange && v0 > 3.0f, "precondition: low range, speed %.2f", v0);
    CHECK(worstRise < 0.05f, "speed rose by %.3f m/s while braking", worstRise);
    CHECK(stopTime > 0.0 && stopTime < 2.5, "stop time %.2f s", stopTime);
    CHECK(worstOverspin < 0.5f, "wheel driven against the brake: w*r - v = %.2f m/s", worstOverspin);
    CHECK(std::abs(t.forwardSpeed) < 0.05f, "still moving: %.3f", t.forwardSpeed);
}

void liftedWheelLosesLoad() {
    Simulation sim(settingsFor(TrackLayout::FlatWithWheelPit));
    run(sim, 3.0);
    const auto& t = sim.vehicle().telemetry();
    printWheels(t);
    CHECK(finite(position(sim)), "NaN");
    CHECK(!t.wheels[0].contact, "front-left wheel should hang over the pit");
    CHECK(t.wheels[0].load == 0.0f, "hanging wheel has load %.1f N", t.wheels[0].load);
    CHECK(t.wheels[0].compression < 0.02f, "hanging wheel should be at full droop, compression %.3f m",
          t.wheels[0].compression);
    // Without anti-roll bars the vehicle rests on the FR-RL diagonal, so RR also unloads;
    // this is expected. Check that the other wheels carry the weight.
    float total = 0.0f;
    for (int i = 1; i < kWheelCount; ++i) total += t.wheels[i].load;
    const float weight = sim.vehicleConfig().mass * 9.81f;
    CHECK(t.wheels[1].contact && t.wheels[2].contact, "diagonal FR/RL must be in contact");
    CHECK(std::abs(total - weight) < 0.1f * weight, "remaining wheels carry %.0f N of %.0f N", total, weight);
}

void resetRestoresState() {
    Simulation sim(settingsFor(TrackLayout::Flat));
    run(sim, 0.5);
    const JPH::Vec3 spawnPos = position(sim);
    run(sim, 4.0, DriverIntent{.throttle = 1.0f, .steer = 0.4f});
    const auto before = sim.vehicle().telemetry();
    std::printf("    before reset: v=%.2f m/s gear=%d rpm=%.0f\n", before.forwardSpeed, before.gear, before.engineRPM);
    CHECK(before.gear >= 1 && before.speed > 3.0f, "precondition failed");

    sim.requestReset();
    sim.setIntent({});
    sim.tick(); // the command runs at the start of the tick, then one step
    const auto& t = sim.vehicle().telemetry();
    const JPH::Vec3 v = velocity(sim);
    const JPH::Vec3 p = position(sim);
    std::printf("    after reset: |v|=%.3f m/s gear=%d rpm=%.0f pos=(%.2f %.2f %.2f)\n", v.Length(), t.gear, t.engineRPM,
                p.GetX(), p.GetY(), p.GetZ());
    CHECK(v.Length() < 0.2f, "velocity not reset: %.3f", v.Length());
    CHECK(t.gear == 0, "gear not reset: %d", t.gear);
    CHECK(t.engineRPM <= sim.vehicleConfig().engineMinRPM + 50.0f, "rpm not reset: %.0f", t.engineRPM);
    CHECK(t.direction == DriveDirection::Forward && !t.lowRange, "direction/range not reset");
    CHECK(std::abs(p.GetX() - spawnPos.GetX()) < 0.1f && std::abs(p.GetZ() - spawnPos.GetZ()) < 0.1f,
          "not at spawn position");
    for (const auto& w : t.wheels) CHECK(std::abs(w.angularVelocity) < 0.5f, "wheel still spinning %.2f", w.angularVelocity);
}

// Steady-state cornering at ~40 km/h on flat concrete: the time-averaged sum of wheel loads
// must equal m*g (no average vertical acceleration, normals are vertical) for any
// number of collision steps; Jolt lambdas refer to the last substep.
void cornerLoadBalance() {
    for (int steps : {1, 2, 4}) {
        SimulationSettings s = settingsFor(TrackLayout::Flat);
        s.collisionSteps = steps;
        Simulation sim(s);
        const float target = 40.0f / 3.6f;
        double sum = 0.0;
        int n = 0;
        for (int i = 0; i < int(10.0 / sim.fixedStep()); ++i) {
            const double time = i * sim.fixedStep();
            const auto& t = sim.vehicle().telemetry();
            DriverIntent in;
            in.throttle = std::fmin(1.0f, std::fmax(0.0f, 0.3f + 0.4f * (target - t.speed)));
            in.steer = time > 2.5 ? -0.45f : 0.0f;
            in.analog = true;
            run(sim, sim.fixedStep(), in);
            if (time < 6.0) continue;
            for (const auto& w : t.wheels) sum += w.load;
            ++n;
        }
        const auto& t = sim.vehicle().telemetry();
        const double mean = sum / n, weight = sim.vehicleConfig().mass * 9.81;
        std::printf("    collisionSteps=%d: speed %.1f km/h, mean sum(load) %.0f N, m*g %.0f N\n", steps, t.speed * 3.6f,
                    mean, weight);
        CHECK(std::abs(t.speed * 3.6f - 40.0f) < 4.0f, "speed controller off: %.1f km/h", t.speed * 3.6f);
        CHECK(std::abs(mean - weight) < 0.01 * weight, "collisionSteps=%d: mean load %.0f N vs %.0f N", steps, mean,
              weight);
        CHECK(t.wheels[1].load > 2.0f * t.wheels[0].load, "outer front should carry more than inner (left turn = right outer)");
    }
}

// Command schedule by physics tick number.
DriverIntent scriptedIntent(unsigned long long tick) {
    DriverIntent in;
    if (tick < 360) in.throttle = 1.0f;
    if (tick >= 180 && tick < 420) in.steer = 0.6f;
    if (tick >= 420 && tick < 520) in.brake = 0.7f;
    if (tick >= 520) { in.throttle = 0.6f; in.steer = -0.5f; }
    return in;
}

struct ScheduleResult { JPH::Vec3 pos; JPH::Quat rot; float speed; };

ScheduleResult runSchedule(const std::function<double(int)>& frameTime, unsigned long long totalTicks) {
    SimulationSettings s = settingsFor(TrackLayout::ProvingGround);
    Simulation sim(s);
    FixedStepClock clock(sim.fixedStep());
    clock.maxStepsPerFrame = 1000; // nothing is dropped in the test
    clock.maxFrameTime = 10.0;
    for (int frame = 0; sim.timing().ticks < totalTicks; ++frame) {
        int steps = clock.advance(frameTime(frame));
        for (int i = 0; i < steps && sim.timing().ticks < totalTicks; ++i) {
            sim.setIntent(scriptedIntent(sim.timing().ticks)); // input is tied to the tick, not the frame
            sim.tick();
        }
    }
    return {position(sim), sim.currentSnapshot().body.rotation, sim.vehicle().telemetry().speed};
}

void frameScheduleIndependence() {
    const unsigned long long ticks = 720; // 6 s at 120 Hz
    const auto a = runSchedule([](int) { return 1.0 / 60.0; }, ticks);
    const auto b = runSchedule([](int f) { // uneven frames of 4..45 ms
        std::uint32_t x = std::uint32_t(f) * 2654435761u + 12345u;
        x ^= x >> 13;
        return 0.004 + 0.041 * double(x % 1000) / 1000.0;
    }, ticks);
    const auto c = runSchedule([](int) { return 1.0 / 144.0; }, ticks);
    const float dab = (a.pos - b.pos).Length(), dac = (a.pos - c.pos).Length();
    std::printf("    A=(%.4f %.4f %.4f) B=(%.4f %.4f %.4f) C=(%.4f %.4f %.4f)\n", a.pos.GetX(), a.pos.GetY(), a.pos.GetZ(),
                b.pos.GetX(), b.pos.GetY(), b.pos.GetZ(), c.pos.GetX(), c.pos.GetY(), c.pos.GetZ());
    std::printf("    |A-B|=%.6f m |A-C|=%.6f m, speeds %.3f %.3f %.3f\n", dab, dac, a.speed, b.speed, c.speed);
    CHECK(finite(a.pos) && finite(b.pos) && finite(c.pos), "NaN");
    CHECK(dab < 0.05f && dac < 0.05f, "results diverge between frame schedules");
    CHECK(std::abs(a.speed - b.speed) < 0.05f && std::abs(a.speed - c.speed) < 0.05f, "speed differs");
    CHECK(std::abs(a.rot.Dot(b.rot)) > 0.9999f, "orientation differs");
}

// Tire marks from the per-wheel contact samples (visual only, generated headless).
struct MarkStats {
    int segments = 0;
    float maxImprint = 0.0f, maxSlip = 0.0f;
    int surfaceSegments[kSurfaceCount] = {};
};

MarkStats marksSince(const TireMarks& m, std::uint64_t writtenBefore) {
    MarkStats st;
    const std::uint64_t fresh = std::min<std::uint64_t>(m.written() - writtenBefore, m.size());
    for (std::size_t i = m.size() - fresh; i < m.size(); ++i) {
        const TireMarkSegment& s = m.oldest(i);
        ++st.segments;
        st.maxImprint = std::fmax(st.maxImprint, std::fmax(s.imprint0, s.imprint1));
        st.maxSlip = std::fmax(st.maxSlip, std::fmax(s.slip0, s.slip1));
        ++st.surfaceSegments[int(markSurface(s))];
    }
    return st;
}

void printMarks(const char* what, const MarkStats& s, const Simulation& sim) {
    std::printf("    %-34s v=%5.1f km/h segments=%4d maxImprint=%.2f maxSlip=%.2f (C/D/M/G %d/%d/%d/%d)\n", what,
                sim.vehicle().telemetry().speed * 3.6f, s.segments, s.maxImprint, s.maxSlip,
                s.surfaceSegments[int(SurfaceId::Concrete)], s.surfaceSegments[int(SurfaceId::PackedDirt)],
                s.surfaceSegments[int(SurfaceId::Mud)], s.surfaceSegments[int(SurfaceId::Grass)]);
}

// Cruise at a target speed with a proportional throttle (no wheelspin).
void cruise(Simulation& sim, double seconds, float targetKmh, float steer = 0.0f) {
    for (int i = 0; i < int(seconds / sim.fixedStep()); ++i) {
        const float v = sim.vehicle().telemetry().speed * 3.6f;
        DriverIntent in;
        in.throttle = std::fmin(0.6f, std::fmax(0.0f, 0.25f + 0.05f * (targetKmh - v)));
        in.steer = steer;
        run(sim, sim.fixedStep(), in);
    }
}

void tireMarksSurfaces() {
    Simulation sim(settingsFor(TrackLayout::SurfaceLanes));
    TireMarks& marks = sim.tireMarks();
    run(sim, 1.0);

    // Concrete: rolling at a steady 30 km/h leaves (almost) nothing, a locked-wheel stop leaves dark rubber.
    cruise(sim, 6.0, 30.0f);
    std::uint64_t w0 = marks.written();
    cruise(sim, 3.0, 30.0f);
    const MarkStats rolling = marksSince(marks, w0);
    printMarks("concrete, rolling 30 km/h:", rolling, sim);
    w0 = marks.written();
    run(sim, 2.5, DriverIntent{.brake = 1.0f});
    const MarkStats lockup = marksSince(marks, w0);
    printMarks("concrete, full brake:", lockup, sim);
    CHECK(rolling.maxSlip < 0.05f && rolling.segments <= 4, "rolling on concrete left marks: %d segments, slip %.2f",
          rolling.segments, rolling.maxSlip);
    CHECK(rolling.maxImprint == 0.0f, "concrete must not take a rolling imprint");
    CHECK(lockup.segments >= 20 && lockup.maxSlip > 0.5f, "lock-up on concrete: %d segments, slip %.2f",
          lockup.segments, lockup.maxSlip);
    CHECK(lockup.surfaceSegments[int(SurfaceId::Concrete)] == lockup.segments, "lock-up marks not tagged Concrete");

    // Soft lanes: rolling leaves a load-driven imprint; a full-lock low-range launch churns (the unloaded inner
    // wheels spin through the open differentials; a straight low-range launch keeps grip on packed dirt).
    const struct {
        int feature;
        SurfaceId surface;
        const char* name;
    } lanes[] = {{1, SurfaceId::PackedDirt, "dirt"}, {2, SurfaceId::Mud, "mud"}, {3, SurfaceId::Grass, "grass"}};
    for (const auto& lane : lanes) {
        sim.requestTeleport(lane.feature);
        run(sim, 1.0);
        cruise(sim, 4.0, 20.0f);
        w0 = marks.written();
        cruise(sim, 3.0, 20.0f);
        const MarkStats roll = marksSince(marks, w0);
        char label[64];
        std::snprintf(label, sizeof(label), "%s, rolling 20 km/h:", lane.name);
        printMarks(label, roll, sim);
        CHECK(roll.segments >= 50 && roll.surfaceSegments[int(lane.surface)] == roll.segments,
              "%s: %d segments, %d on the lane surface", lane.name, roll.segments,
              roll.surfaceSegments[int(lane.surface)]);
        CHECK(roll.maxImprint > 0.5f, "%s: weak imprint %.2f", lane.name, roll.maxImprint);
        CHECK(roll.maxSlip < 0.3f, "%s: rolling counted as churn %.2f", lane.name, roll.maxSlip);

        sim.requestTeleport(lane.feature);
        run(sim, 1.0);
        sim.requestToggleLowRange();
        run(sim, 0.1);
        w0 = marks.written();
        run(sim, 1.7, DriverIntent{.throttle = 1.0f, .steer = 1.0f});
        const MarkStats spin = marksSince(marks, w0);
        std::snprintf(label, sizeof(label), "%s, power turn (low range):", lane.name);
        printMarks(label, spin, sim);
        CHECK(spin.segments >= 8 && spin.maxSlip > 0.3f, "%s: wheelspin churn %d segments, slip %.2f", lane.name,
              spin.segments, spin.maxSlip);
        run(sim, 2.0, DriverIntent{.brake = 1.0f});
        sim.requestToggleLowRange();
        run(sim, 0.1);
    }

    // Hard cap and fade: a small ring keeps only its capacity; old segments expire after the fade time.
    TireMarkSettings s = marks.settings();
    s.maxSegments = 256;
    s.fadeTime = 3.0f;
    marks.setSettings(s);
    sim.requestTeleport(1);
    run(sim, 1.0);
    cruise(sim, 8.0, 30.0f);
    std::printf("    cap 256: size=%zu written=%llu\n", marks.size(), (unsigned long long)marks.written());
    CHECK(marks.size() <= 256, "ring exceeded its capacity: %zu", marks.size());
    run(sim, 6.0, DriverIntent{.brake = 1.0f});
    run(sim, 3.5);
    std::printf("    after %.1f s parked: size=%zu\n", 3.5, marks.size());
    CHECK(marks.size() == 0, "segments did not expire: %zu left", marks.size());
}

// Live-input pedals (DriverIntent::autoReverse): W = forward pedal, S = backward pedal.
DriverIntent pedals(float w, float s) { return DriverIntent{.throttle = w, .brake = s, .autoReverse = true}; }

struct SwitchTrace {
    double stopTime = -1.0;   // first tick with |v| < stopSpeed while the opposite pedal is held
    double switchTime = -1.0; // tick at which the direction changed
    float speedAtSwitch = 0.0f;
    bool switchedWhileMoving = false;
};

// Holds the given pedals until the direction changes (or the timeout) and records when the vehicle stopped.
SwitchTrace holdUntilSwitch(Simulation& sim, const DriverIntent& in, double timeout) {
    SwitchTrace tr;
    const DriveDirection start = sim.vehicle().telemetry().direction;
    const float stopSpeed = sim.autoReverse().stopSpeed;
    sim.setIntent(in);
    for (int i = 0; i < int(timeout / sim.fixedStep()); ++i) {
        const float vBefore = sim.vehicle().telemetry().forwardSpeed;
        sim.tick();
        const double time = (i + 1) * sim.fixedStep();
        if (tr.stopTime < 0.0 && std::abs(vBefore) < stopSpeed) tr.stopTime = time;
        if (sim.vehicle().telemetry().direction != start) {
            tr.switchTime = time;
            tr.speedAtSwitch = vBefore;
            tr.switchedWhileMoving = std::abs(vBefore) >= stopSpeed;
            break;
        }
    }
    return tr;
}

void autoReverse() {
    // Flat ground at 60/120/240 Hz: S pressed while moving brakes to a stop and keeps holding; released and pressed
    // again at the stop it engages reverse after the hold time (simulated time, so the same at any tick rate) and
    // drives backwards. W behaves the same way against the backward motion.
    for (double hz : {60.0, 120.0, 240.0}) {
        SimulationSettings s = settingsFor(TrackLayout::Flat);
        s.fixedStep = 1.0 / hz;
        Simulation sim(s);
        const double dt = sim.fixedStep(), hold = sim.autoReverse().holdTime;
        run(sim, 0.5);
        run(sim, 3.0, pedals(1, 0));
        const float v0 = sim.vehicle().telemetry().forwardSpeed;
        const SwitchTrace held = holdUntilSwitch(sim, pedals(0, 1), 3.0 + hold + 1.0);
        const float vHeld = sim.vehicle().telemetry().forwardSpeed;
        run(sim, 0.1, pedals(0, 0));
        const SwitchTrace toRev = holdUntilSwitch(sim, pedals(0, 1), 2.0);
        run(sim, 3.0, pedals(0, 1));
        const auto& t = sim.vehicle().telemetry();
        const float vRev = t.forwardSpeed;
        const int gearRev = t.gear;
        const SwitchTrace heldRev = holdUntilSwitch(sim, pedals(1, 0), 3.0 + hold + 1.0);
        run(sim, 0.1, pedals(0, 0));
        const SwitchTrace toFwd = holdUntilSwitch(sim, pedals(1, 0), 2.0);
        run(sim, 3.0, pedals(1, 0));
        const float vFwd = sim.vehicle().telemetry().forwardSpeed;
        std::printf("    %3.0f Hz: v0=%.2f m/s, S held: stop %.3f s, switch %.3f, v %.3f; S again: reverse after %.3f s, "
                    "after 3 s v=%.2f gear=%d; W held: switch %.3f; W again: drive after %.3f s, after 3 s v=%.2f\n",
                    hz, v0, held.stopTime, held.switchTime, vHeld, toRev.switchTime, vRev, gearRev, heldRev.switchTime,
                    toFwd.switchTime, vFwd);
        CHECK(v0 > 5.0f, "precondition: forward speed %.2f", v0);
        CHECK(held.stopTime > 0.0 && held.switchTime < 0.0 && std::abs(vHeld) < 0.05f,
              "%.0f Hz: S held through the stop must keep braking (switch %.3f s, v %.3f)", hz, held.switchTime, vHeld);
        CHECK(toRev.switchTime > 0.0 && !toRev.switchedWhileMoving, "%.0f Hz: reverse engaged while moving or never", hz);
        CHECK(toRev.switchTime >= hold - 1e-6 && toRev.switchTime < hold + dt + 1e-6,
              "%.0f Hz: reverse delay %.3f s vs hold %.3f s", hz, toRev.switchTime, hold);
        CHECK(vRev < -2.0f && gearRev < 0, "%.0f Hz: not driving backwards: v=%.2f gear=%d", hz, vRev, gearRev);
        CHECK(heldRev.stopTime > 0.0 && heldRev.switchTime < 0.0, "%.0f Hz: W held through the stop switched", hz);
        CHECK(toFwd.switchTime > 0.0 && !toFwd.switchedWhileMoving, "%.0f Hz: forward engaged while moving or never", hz);
        CHECK(toFwd.switchTime < hold + dt + 1e-6, "%.0f Hz: forward delay %.3f s", hz, toFwd.switchTime);
        CHECK(vFwd > 2.0f, "%.0f Hz: not driving forward again: v=%.2f", hz, vFwd);
    }

    // Low range from a standstill, then R reset with S still held: the reset selects forward, S engages reverse again.
    {
        Simulation sim(settingsFor(TrackLayout::Flat));
        run(sim, 0.5);
        sim.requestToggleLowRange();
        run(sim, 0.1);
        const SwitchTrace tr = holdUntilSwitch(sim, pedals(0, 1), 2.0);
        run(sim, 3.0, pedals(0, 1));
        const auto& t = sim.vehicle().telemetry();
        std::printf("    low range: reverse after %.3f s, after 3 s v=%.2f m/s gear=%d low=%d\n", tr.switchTime,
                    t.forwardSpeed, t.gear, t.lowRange);
        CHECK(t.lowRange && t.direction == DriveDirection::Reverse, "low range reverse not engaged");
        CHECK(t.forwardSpeed < -1.0f && t.gear < 0, "low range: not driving backwards: v=%.2f", t.forwardSpeed);
        sim.requestReset();
        sim.tick();
        CHECK(sim.vehicle().telemetry().direction == DriveDirection::Forward, "reset did not select forward");
        run(sim, 1.0, pedals(0, 1));
        run(sim, 2.0, pedals(0, 1));
        const auto& r = sim.vehicle().telemetry();
        std::printf("    after reset: dir=%s v=%.2f m/s gear=%d\n", r.direction == DriveDirection::Reverse ? "R" : "D",
                    r.forwardSpeed, r.gear);
        CHECK(r.direction == DriveDirection::Reverse && r.forwardSpeed < -1.0f, "no reverse after reset: v=%.2f",
              r.forwardSpeed);
    }

    // Both pedals: no switch. Classic mapping (autoReverse off): S only brakes and holds at a standstill.
    {
        Simulation sim(settingsFor(TrackLayout::Flat));
        run(sim, 0.5);
        run(sim, 2.0, pedals(1, 1));
        CHECK(sim.vehicle().telemetry().direction == DriveDirection::Forward, "W+S switched the direction");
        run(sim, 2.0, DriverIntent{.brake = 1.0f});
        const auto& t = sim.vehicle().telemetry();
        CHECK(t.direction == DriveDirection::Forward && std::abs(t.forwardSpeed) < 0.05f,
              "classic mapping: S must only brake (dir %d, v %.3f)", int(t.direction), t.forwardSpeed);
    }

    // Slope: climb the 12 degree hill in low range; S held from the climb stops the vehicle and holds it on the slope.
    // Released and pressed again, S engages reverse and the vehicle backs down.
    {
        Simulation sim(settingsFor(TrackLayout::ProvingGround));
        sim.requestTeleport(4);
        run(sim, 1.0);
        sim.requestToggleLowRange();
        run(sim, 0.1);
        sim.setIntent(pedals(1, 0));
        for (int i = 0; i < int(12.0 / sim.fixedStep()) && position(sim).GetZ() < -36.0f; ++i) sim.tick();
        const JPH::Vec3 onSlope = position(sim);
        const SwitchTrace held = holdUntilSwitch(sim, pedals(0, 1), 2.0);
        const float zHeld = position(sim).GetZ();
        const SwitchTrace held2 = holdUntilSwitch(sim, pedals(0, 1), 6.0);
        const float creep = position(sim).GetZ() - zHeld;
        run(sim, 2.0 * sim.fixedStep(), pedals(0, 0));
        const SwitchTrace tr = holdUntilSwitch(sim, pedals(0, 1), 2.0);
        const float zSwitch = position(sim).GetZ();
        run(sim, 2.0, pedals(0, 1));
        const auto& t = sim.vehicle().telemetry();
        const float pitch = std::asin(sim.currentSnapshot().body.rotation.RotateAxisZ().GetY()) * 57.2958f;
        std::printf("    slope: on slope z=%.2f y=%.2f pitch %.1f deg, S held: stop %.3f s, creep over 6 s %.4f m; "
                    "S again: reverse after %.3f s, 2 s later dz=%.2f m v=%.2f gear=%d\n",
                    onSlope.GetZ(), onSlope.GetY(), pitch, held.stopTime, creep, tr.switchTime,
                    position(sim).GetZ() - zSwitch, t.forwardSpeed, t.gear);
        CHECK(onSlope.GetZ() >= -36.0f && onSlope.GetY() > 0.6f, "precondition: not on the slope (z %.2f y %.2f)",
              onSlope.GetZ(), onSlope.GetY());
        CHECK(held.switchTime < 0.0 && held2.switchTime < 0.0, "slope: S held from the climb engaged reverse");
        CHECK(std::abs(creep) < 0.05f, "slope: vehicle moved %.3f m while S was held", creep);
        CHECK(tr.switchTime > 0.0 && !tr.switchedWhileMoving, "slope: reverse engaged while moving or never");
        CHECK(t.direction == DriveDirection::Reverse && t.gear < 0, "slope: not in reverse (gear %d)", t.gear);
        CHECK(position(sim).GetZ() - zSwitch < -0.5f, "slope: did not back down (dz %.2f)", position(sim).GetZ() - zSwitch);
    }
}

// Full-throttle high-range launches on the surface lanes: concrete keeps grip, the low-mu surfaces spin up and their
// acceleration stays below mu * g.
void surfaceLaunchGrip() {
    const struct {
        int feature;
        SurfaceId surface;
        const char* name;
    } lanes[] = {{0, SurfaceId::Concrete, "concrete"},
                 {1, SurfaceId::PackedDirt, "dirt"},
                 {3, SurfaceId::Grass, "grass"},
                 {2, SurfaceId::Mud, "mud"}};
    float prevV2 = 1e9f;
    for (const auto& lane : lanes) {
        for (double hz : {60.0, 120.0, 240.0}) {
            SimulationSettings s = settingsFor(TrackLayout::SurfaceLanes);
            s.fixedStep = 1.0 / hz;
            Simulation sim(s);
            sim.requestTeleport(lane.feature);
            run(sim, 1.0);
            const float mu = sim.surfaces().get(lane.surface).longitudinalFriction *
                             sim.vehicleConfig().tuning.tirePeakLongFriction;
            float maxSlip = 0.0f, v1 = 0.0f;
            double spin = 0.0;
            sim.setIntent({.throttle = 1.0f});
            const int n = int(std::lround(2.0 / sim.fixedStep()));
            for (int i = 1; i <= n; ++i) {
                sim.tick();
                float slip = 0.0f;
                for (const WheelContactSample& c : sim.vehicle().contacts())
                    if (c.contact) slip = std::max(slip, std::abs(c.longitudinalSlipSpeed));
                maxSlip = std::max(maxSlip, slip);
                if (slip > 1.0f) spin += sim.fixedStep();
                if (i == n / 2) v1 = sim.vehicle().telemetry().speed;
            }
            const float v2 = sim.vehicle().telemetry().speed;
            std::printf("    %-8s %3.0f Hz: mu %.2f, v(1 s) %.2f, v(2 s) %.2f m/s, max slip %.2f m/s, wheelspin %.2f s\n",
                        lane.name, hz, mu, v1, v2, maxSlip, spin);
            CHECK(v1 < mu * 9.81f * 1.05f, "%s: %.2f m/s^2 in the first second exceeds mu * g", lane.name, v1);
            if (lane.surface == SurfaceId::Concrete) {
                // At 60 Hz the lighter front wheels still slip for one or two steps (~3 m/s) near the peak.
                CHECK(spin <= 0.05 && (hz < 100.0 || maxSlip < 0.5f), "concrete launch spins: max slip %.2f m/s, %.2f s",
                      maxSlip, spin);
                CHECK(v1 > 4.5f, "concrete launch too slow: %.2f m/s after 1 s", v1);
            }
            if (lane.surface == SurfaceId::Mud || lane.surface == SurfaceId::Grass)
                CHECK(maxSlip > 2.0f && spin > 0.5, "%s launch keeps grip: max slip %.2f m/s, %.2f s", lane.name,
                      maxSlip, spin);
            if (hz == 120.0) {
                CHECK(v2 < prevV2, "%s: v(2 s) %.2f not below the grippier surface's %.2f", lane.name, v2, prevV2);
                prevV2 = v2;
            }
        }
    }
}

// Asphalt and gravel: table values in their literature ranges, data keys, and on the road lanes (builtin vehicle):
// 80-0 km/h stops asphalt < concrete < gravel within mu x g, gravel rolling resistance while coasting.
void asphaltGravelSurfaces() {
    Simulation sim(settingsFor(TrackLayout::RoadLanes));
    const SurfaceTable& table = sim.surfaces();
    for (std::size_t i = 0; i < kSurfaceCount; ++i) {
        const SurfaceId id = SurfaceId(i);
        CHECK(surfaceFromKey(surfaceKey(id)) == id, "key '%s' does not map back to surface %zu", surfaceKey(id), i);
        const SurfaceProperties& p = table.get(id);
        if (id != SurfaceId::Gravel)
            CHECK(p.peakSlipScale == 1.0f && p.slideDrop == 1.0f && p.extraRollingResistance == 0.0f,
                  "%s: loose-surface parameters on a surface that keeps the plain tire curve", p.name);
    }
    CHECK(surfaceFromKey("dirt") == SurfaceId::PackedDirt && surfaceFromKey("tarmac") == SurfaceId::Count, "key aliases");
    const SurfaceProperties& asphalt = table.get(SurfaceId::Asphalt);
    const SurfaceProperties& gravel = table.get(SurfaceId::Gravel);
    CHECK(asphalt.longitudinalFriction >= 1.0f && asphalt.longitudinalFriction <= 1.1f &&
              asphalt.longitudinalFriction > table.get(SurfaceId::Concrete).longitudinalFriction,
          "asphalt mu %.2f", asphalt.longitudinalFriction);
    CHECK(gravel.longitudinalFriction >= 0.55f && gravel.longitudinalFriction <= 0.7f && gravel.lateralFriction >= 0.55f &&
              gravel.lateralFriction <= 0.7f,
          "gravel mu %.2f / %.2f", gravel.longitudinalFriction, gravel.lateralFriction);
    CHECK(gravel.peakSlipScale > 1.0f && gravel.slideDrop < 1.0f && gravel.extraRollingResistance >= 0.01f &&
              gravel.extraRollingResistance <= 0.02f,
          "gravel curve x%.1f, drop %.2f, rolling resistance %.3f", gravel.peakSlipScale, gravel.slideDrop,
          gravel.extraRollingResistance);

    const struct {
        int feature;
        SurfaceId surface;
        const char* name;
    } lanes[] = {{0, SurfaceId::Asphalt, "asphalt"}, {1, SurfaceId::Concrete, "concrete"}, {2, SurfaceId::Gravel, "gravel"}};
    float prev = 0.0f;
    for (const auto& lane : lanes) {
        for (int abs = 0; abs < 2; ++abs) {
            Simulation s(settingsFor(TrackLayout::RoadLanes));
            s.requestTeleport(lane.feature);
            run(s, 1.0);
            s.requestAidControls({abs != 0, TractionMode::Off, 0});
            for (int i = 0; i < int(30.0 / s.fixedStep()) && s.vehicle().telemetry().speed * 3.6f < 80.0f; ++i) {
                s.setIntent({.throttle = 1.0f});
                s.tick();
            }
            int onLane = 0;
            for (const auto& w : s.vehicle().telemetry().wheels) onLane += w.contact && w.surface == lane.surface;
            const float v0 = s.vehicle().telemetry().speed;
            float path = 0.0f;
            double t = 0.0;
            while (s.vehicle().telemetry().speed > 0.05f && t < 15.0) {
                s.setIntent({.brake = 1.0f});
                s.tick();
                t += s.fixedStep();
                path += s.vehicle().telemetry().speed * float(s.fixedStep());
            }
            const float mu = s.surfaces().get(lane.surface).longitudinalFriction * s.vehicleConfig().tuning.tirePeakLongFriction;
            const float decel = v0 * v0 / (2.0f * path);
            std::printf("    %-8s ABS %s: from %.1f km/h %.2f m (%.2f g, peak mu %.2f), %d wheels on the lane\n", lane.name,
                        abs ? "on " : "off", v0 * 3.6f, path, decel / 9.81f, mu, onLane);
            CHECK(onLane == kWheelCount, "%s: %d wheels on the lane surface", lane.name, onLane);
            CHECK(decel < mu * 9.81f * 1.03f && decel > 0.75f * mu * 9.81f, "%s: %.2f g against peak mu %.2f", lane.name,
                  decel / 9.81f, mu);
            if (abs) {
                CHECK(path > prev, "%s ABS stop %.2f m not longer than the grippier surface's %.2f m", lane.name, path, prev);
                prev = path;
            }
        }
    }

    // Coasting from 50 km/h with no pedals: the same engine braking, gravel adds ~0.015 x weight.
    float loss[2] = {};
    for (int k = 0; k < 2; ++k) {
        Simulation s(settingsFor(TrackLayout::RoadLanes));
        s.requestTeleport(k == 0 ? 0 : 2);
        run(s, 1.0);
        for (int i = 0; i < int(30.0 / s.fixedStep()) && s.vehicle().telemetry().speed * 3.6f < 50.0f; ++i) {
            s.setIntent({.throttle = 0.6f});
            s.tick();
        }
        const float v0 = s.vehicle().telemetry().speed;
        run(s, 0.5);
        const float rr = s.vehicle().telemetry().rollingResistanceForce;
        run(s, 2.5);
        loss[k] = v0 - s.vehicle().telemetry().speed;
        const float weight = s.vehicleConfig().mass * 9.81f;
        std::printf("    coast %s: %.2f m/s lost in 3 s, rolling resistance %.0f N (%.4f x weight)\n", k ? "gravel " : "asphalt",
                    loss[k], rr, rr / weight);
        if (k == 0) CHECK(rr == 0.0f, "asphalt rolling resistance %.0f N", rr);
        else CHECK(std::abs(rr / weight - gravel.extraRollingResistance) < 0.003f, "gravel rolling resistance %.4f x weight",
                   rr / weight);
    }
    CHECK(loss[1] - loss[0] > 0.25f && loss[1] - loss[0] < 0.7f, "gravel coast loses %.2f m/s more than asphalt (0.44 expected)",
          loss[1] - loss[0]);
}

// Rubber mark profile of a locked-wheel stop from 100 km/h on concrete, per vehicle side (the rear wheel runs in the
// front wheel's track, so both ribbons of a side are composited like the shader does: 1 - (1 - a)(1 - b)). Every
// wheel also feeds its own TireMarks so the ribbons can be told apart.
// A wheel leaving soft ground for concrete: the boundary must not carry soft-ground churn into a rubber mark, and
// the soil print fades smoothly over the following meters (longest after wet mud). Synthetic contact samples, no
// physics: the mark model alone is under test.
void tireMarksBoundary() {
    struct Drive {
        const char* name;
        float v, roll; // ground speed, tread speed [m/s]
    };
    const Drive drives[] = {{"rolling", 5.0f, 5.0f}, {"spinning", 4.0f, 9.0f}, {"locked", 5.0f, 0.0f}};
    const SurfaceId soils[] = {SurfaceId::Mud, SurfaceId::PackedDirt, SurfaceId::Sand};
    constexpr float kBoundary = 6.0f, kMin = 0.02f;
    float mudEnd = 0.0f, dirtEnd = 0.0f;
    for (SurfaceId soft : soils) {
        const char* name = soft == SurfaceId::Mud ? "mud" : soft == SurfaceId::Sand ? "sand" : "dirt";
        for (const Drive& k : drives) {
            TireMarks marks;
            TireMarkSettings st = marks.settings();
            st.referenceLoad = 4900.0f;
            marks.setSettings(st);
            double t = 0.0;
            float z = 0.0f;
            for (int i = 0; i < 2400 && z < 40.0f; ++i) {
                WheelContactSample c;
                c.contact = true;
                c.surface = z < kBoundary ? soft : SurfaceId::Concrete;
                c.patchCenter = JPH::Vec3(-300.0f, 0.0f, z); // far from the showcase water
                c.velocity = JPH::Vec3(0.0f, 0.0f, k.v);
                c.load = 4900.0f;
                c.rollingSpeed = k.roll;
                c.longitudinalSlipSpeed = k.roll - k.v;
                c.width = 0.3f;
                c.radius = 0.4f;
                t += 1.0 / 120.0;
                z += k.v / 120.0f;
                marks.update(std::span(&c, 1), t);
            }
            float firstPrint = -1.0f, printEnd = kBoundary, maxRise = 0.0f, prev = 2.0f, blob = -1.0f, maxHalf = 0.0f;
            float prevSlip = 0.0f, slipDrop = 0.0f;
            int crossing = 0;
            for (std::size_t i = 0; i < marks.size(); ++i) {
                const TireMarkSegment& s = marks.oldest(i);
                const bool hard = markSurface(s) == SurfaceId::Concrete;
                if (!hard) continue;
                const float half = std::hypot(s.side0[0], s.side0[2]);
                maxHalf = std::max({maxHalf, half, std::hypot(s.side1[0], s.side1[2])});
                if (s.p0[2] < kBoundary - 0.01f) ++crossing;
                // Rubber may build up once the tread is clean, but never starts dark and never spikes and drops.
                if (blob < 0.0f) blob = std::max(s.slip0, s.slip1);
                if (s.p1[2] < kBoundary + 8.0f) slipDrop = std::max(slipDrop, prevSlip - s.slip1);
                prevSlip = s.slip1;
                if (firstPrint < 0.0f) firstPrint = s.imprint0;
                maxRise = std::max(maxRise, s.imprint1 - prev);
                prev = s.imprint1;
                if (s.imprint1 >= kMin) printEnd = s.p1[2];
            }
            const float printLen = printEnd - kBoundary;
            std::printf("  %-4s %-8s first print %.2f, print to +%.1f m, rubber at boundary %.2f, largest rubber drop "
                        "%.2f, end rubber %.2f, half width %.3f, crossing segments %d\n", name, k.name, firstPrint,
                        printLen, blob, slipDrop, prevSlip, maxHalf, crossing);
            CHECK(crossing == 0, "%s/%s: %d concrete-style segments start on the soft side", name, k.name,
                  crossing);
            CHECK(blob < 0.1f, "%s/%s: rubber mark %.2f at the boundary", name, k.name, blob);
            CHECK(slipDrop < 0.05f, "%s/%s: rubber mark drops by %.2f (blob)", name, k.name, slipDrop);
            CHECK(maxHalf < 0.151f, "%s/%s: concrete mark half width %.3f m (soft-ground width carried over)",
                  name, k.name, maxHalf);
            CHECK(maxRise < 0.02f, "%s/%s: print rises by %.3f along the concrete", name, k.name, maxRise);
            if (k.roll == k.v) {
                if (soft == SurfaceId::Mud) {
                    mudEnd = printLen;
                    CHECK(firstPrint > 0.6f, "mud print starts at %.2f", firstPrint);
                    CHECK(printLen > 10.0f && printLen < 22.0f, "mud print length %.1f m (want 10..20)", printLen);
                } else if (soft == SurfaceId::PackedDirt) {
                    dirtEnd = printLen;
                    CHECK(firstPrint > 0.08f && firstPrint < 0.4f, "dirt print starts at %.2f", firstPrint);
                    CHECK(printLen > 2.0f && printLen < mudEnd, "dirt print length %.1f m vs mud %.1f m", printLen,
                          mudEnd);
                } else { // dry sand barely sticks: a faint dusting gone within a meter or two
                    CHECK(firstPrint > 0.02f && firstPrint < 0.08f, "sand print starts at %.2f", firstPrint);
                    CHECK(printLen > 0.2f && printLen < 2.5f && printLen < dirtEnd,
                          "sand print length %.1f m vs dirt %.1f m", printLen, dirtEnd);
                }
            }
        }
    }
}

// The shipped SUV (2H part-time transfer case, ABS off) on the flat concrete pad: rubber marks for rolling, a full
// brake and an instant full-throttle launch, and the keyboard pedals (KeyboardPedals): a held W pulls away without
// wheelspin or marks and still reaches full throttle; a 0.25 s S tap does not lock; a held S reaches full brake.
void suvMarksKeyboard() {
    VehicleDefinition def;
    std::vector<std::string> errors;
    CHECK(loadVehicleDefinition(TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_suv/vehicle.json", def, errors),
          "SUV definition: %s", errors.empty() ? "?" : errors.front().c_str());
    SimulationSettings st = settingsFor(TrackLayout::Flat);
    st.vehicle = def.config;
    Simulation sim(st);
    TireMarks& marks = sim.tireMarks();
    struct Drive {
        MarkStats marks;
        float maxSlip = 0.0f, maxPedal = 0.0f;
        double t30 = -1.0, stop = -1.0;
    };
    // keyboard: the pedal comes from KeyboardPedals with the key held for `hold` s; otherwise intent is used as is.
    auto drive = [&](double seconds, const DriverIntent& intent, bool keyboard, bool brakeKey, double hold) {
        Drive d;
        KeyboardPedals pedals;
        const std::uint64_t w0 = marks.written();
        const float dt = float(sim.fixedStep());
        for (int i = 0; i < int(std::lround(seconds / sim.fixedStep())); ++i) {
            const double t = i * sim.fixedStep();
            DriverIntent in = intent;
            if (keyboard) {
                pedals.update(t < hold && !brakeKey, t < hold && brakeKey, dt);
                in.throttle = pedals.throttle();
                in.brake = pedals.brake();
                d.maxPedal = std::max(d.maxPedal, brakeKey ? in.brake : in.throttle);
            }
            run(sim, sim.fixedStep(), in);
            for (const WheelContactSample& c : sim.vehicle().contacts())
                if (c.contact) d.maxSlip = std::max(d.maxSlip, c.slipSpeed());
            const float v = sim.vehicle().telemetry().speed;
            if (d.t30 < 0.0 && v * 3.6f >= 30.0f) d.t30 = t;
            if (d.stop < 0.0 && v < 0.5f) d.stop = t;
        }
        d.marks = marksSince(marks, w0);
        return d;
    };
    auto spawn = [&](float kmh) {
        sim.requestReset();
        run(sim, 1.0);
        if (kmh > 0.0f) cruise(sim, 8.0, kmh);
    };
    auto report = [&](const char* what, const Drive& d) {
        printMarks(what, d.marks, sim);
        std::printf("      max slip %.2f m/s, pedal max %.2f, 0-30 km/h %.2f s, stop %.2f s\n", d.maxSlip, d.maxPedal,
                    d.t30, d.stop);
    };

    spawn(30.0f);
    const Drive rolling = drive(3.0, {}, false, false, 0.0);
    report("SUV coasting from 30 km/h:", rolling);
    spawn(0.0f);
    const Drive instant = drive(4.0, {.throttle = 1.0f}, false, false, 0.0);
    report("SUV instant full throttle (2H):", instant);
    spawn(0.0f);
    const Drive keyW = drive(4.0, {}, true, false, 4.0);
    report("SUV keyboard W held 4 s:", keyW);
    spawn(50.0f);
    const Drive keyTap = drive(2.0, {}, true, true, 0.25);
    report("SUV keyboard S tap 0.25 s at 50 km/h:", keyTap);
    spawn(50.0f);
    const Drive keyHold = drive(5.0, {}, true, true, 5.0);
    report("SUV keyboard S held at 50 km/h:", keyHold);
    spawn(50.0f);
    const Drive full = drive(5.0, {.brake = 1.0f}, false, false, 0.0);
    report("SUV full brake at 50 km/h:", full);

    CHECK(rolling.marks.maxSlip < 0.05f, "SUV rolling left rubber: %.2f", rolling.marks.maxSlip);
    CHECK(instant.maxSlip > 2.0f && instant.marks.maxSlip > 0.05f,
          "instant full throttle no longer spins the 2H rears (%.2f m/s, mark %.2f): update KeyboardPedals' notes",
          instant.maxSlip, instant.marks.maxSlip);
    CHECK(keyW.maxPedal > 0.99f && keyW.maxSlip < 1.5f && keyW.marks.maxSlip < 0.05f,
          "keyboard launch: pedal %.2f, slip %.2f m/s, mark %.2f", keyW.maxPedal, keyW.maxSlip, keyW.marks.maxSlip);
    CHECK(keyW.t30 > 0.0 && keyW.t30 <= instant.t30 + 0.2, "keyboard 0-30 km/h %.2f s vs instant %.2f s", keyW.t30,
          instant.t30);
    CHECK(keyTap.maxSlip < 2.0f && keyTap.marks.maxSlip < 0.05f, "keyboard tap locks: %.2f m/s, mark %.2f",
          keyTap.maxSlip, keyTap.marks.maxSlip);
    CHECK(keyHold.maxPedal > 0.99f && keyHold.stop > 0.0 && keyHold.stop < full.stop + 0.3,
          "keyboard brake hold: pedal %.2f, stop %.2f s vs %.2f s", keyHold.maxPedal, keyHold.stop, full.stop);
    CHECK(full.marks.segments >= 20 && full.marks.maxSlip > 0.5f, "SUV lock-up marks: %d segments, %.2f",
          full.marks.segments, full.marks.maxSlip);
}

// Keyboard steering range (Simulation::keyboardSteerRange) on the shipped SUV, flat pad: at a crawl a held key reaches
// full lock; at 60 km/h it stops at the corner angle of the front tires' peak friction plus keyboardSteerSlip x their
// peak slip angle, on the concrete pad and with the pad turned into grass at speed (lower mu, same peak slip), so the
// front tires work near their peak (6 deg) one second into the hold instead of sliding at 30+ deg (full lock at 60
// km/h); the stick is not limited.
void keyboardSteerRange() {
    VehicleDefinition def;
    std::vector<std::string> errors;
    CHECK(loadVehicleDefinition(TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_suv/vehicle.json", def, errors),
          "SUV definition: %s", errors.empty() ? "?" : errors.front().c_str());
    SimulationSettings st = settingsFor(TrackLayout::Flat);
    st.vehicle = def.config;
    const float maxDeg = def.config.axles[kFrontAxle].maxSteerAngleDeg;
    const VehicleTuning& t = def.config.tuning;
    {
        Simulation sim(st);
        run(sim, 1.0);
        cruise(sim, 1.0, 5.0f);
        cruise(sim, 1.0, 5.0f, 1.0f);
        const float crawl = std::abs(sim.vehicle().telemetry().steerAngleDeg);
        std::printf("  held key at %.1f km/h: steer %.1f deg (full lock %.0f)\n", sim.vehicle().telemetry().speed * 3.6f, crawl, maxDeg);
        CHECK(crawl > 0.97f * maxDeg, "full lock at a crawl: %.1f deg", crawl);
    }
    for (const SurfaceId surface : {SurfaceId::Concrete, SurfaceId::Grass}) {
        Simulation sim(st);
        const SurfaceProperties pad = sim.surfaces().get(surface);
        run(sim, 1.0);
        cruise(sim, 10.0, 60.0f);
        sim.surfaces().get(SurfaceId::Concrete) = pad;
        run(sim, 1.0, {.throttle = 0.3f, .steer = 1.0f});
        const auto& tel = sim.vehicle().telemetry();
        const float v = tel.speed;
        const float mu = t.tirePeakLatFriction * pad.lateralFriction;
        const float expected = std::atan(def.config.wheelBase * mu * 9.81f / (v * v)) * 57.2958f +
                               st.smoothing.keyboardSteerSlip * t.tirePeakSlipAngleDeg;
        const float steer = std::abs(tel.steerAngleDeg);
        const float front = 0.5f * (std::abs(tel.wheels[0].lateralSlipDeg) + std::abs(tel.wheels[1].lateralSlipDeg));
        std::printf("  %s, held key at %.0f km/h: steer %.1f deg (range %.1f), front slip %.1f deg, up %.3f\n", pad.name,
                    v * 3.6f, steer, expected, front, tel.upDot);
        CHECK(std::abs(steer - expected) < 0.5f, "%s: keyboard range at speed %.1f deg vs %.1f", pad.name, steer, expected);
        CHECK(front > 4.0f && front < 10.0f, "%s: front tires near their 6 deg peak: %.1f deg", pad.name, front);
        CHECK(tel.upDot > 0.9f, "upright");
    }
    {
        Simulation sim(st);
        run(sim, 1.0);
        cruise(sim, 10.0, 60.0f);
        run(sim, 0.5, {.throttle = 0.3f, .steer = 1.0f, .analog = true});
        const float stick = std::abs(sim.vehicle().telemetry().steerAngleDeg);
        std::printf("  stick at 60 km/h: steer %.1f deg\n", stick);
        CHECK(stick > 0.97f * maxDeg, "the stick is not limited: %.1f deg", stick);
    }
}

// Launch rubber follows how much and how long the tire spins (shipped SUV, flat concrete): normal pull-aways leave
// none, a full-pedal 2H launch (~2 s of spin at < 5 m/s) a faint, narrow mark, 4H none; a stationary burnout
// (brake 0.6 + full throttle, rears at ~10 m/s) builds up from light and narrow after 2 s to dark and full width
// after 5 s. Burnout stamps are composited per spot like the shader (cover = 1 - prod(1 - 1.1 x slip)).
void launchMarksProportional() {
    VehicleDefinition def;
    std::vector<std::string> errors;
    CHECK(loadVehicleDefinition(TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_suv/vehicle.json", def, errors),
          "SUV definition: %s", errors.empty() ? "?" : errors.front().c_str());
    SimulationSettings st = settingsFor(TrackLayout::Flat);
    st.vehicle = def.config;
    Simulation sim(st);
    const float tireWidth = def.config.axles[kRearAxle].wheelWidth;
    struct Result {
        float maxSlip = 0.0f, ribbonMax = 0.0f, ribbonWidth = 0.0f, spotCover = 0.0f, spotWidth = 0.0f;
        double ribbonLength = 0.0;
    };
    auto drive = [&](TransferMode mode, float throttle, float brake, double seconds, bool keyboard) {
        sim.requestReset();
        run(sim, 1.0);
        sim.requestTransfer(mode);
        run(sim, 0.2);
        sim.tireMarks().clear();
        KeyboardPedals pedals;
        Result r;
        for (int i = 0; i < int(std::lround(seconds / sim.fixedStep())); ++i) {
            pedals.update(keyboard, false, float(sim.fixedStep()));
            run(sim, sim.fixedStep(), {.throttle = keyboard ? pedals.throttle() : throttle, .brake = brake});
            for (const WheelContactSample& c : sim.vehicle().contacts())
                if (c.contact) r.maxSlip = std::max(r.maxSlip, c.longitudinalSlipSpeed);
        }
        const TireMarks& m = sim.tireMarks();
        struct Spot {
            float x, z, keep, width;
        };
        std::vector<Spot> spots;
        double widthSum = 0.0;
        for (std::size_t i = 0; i < m.size(); ++i) {
            const TireMarkSegment& s = m.oldest(i);
            const float slip = 0.5f * (s.slip0 + s.slip1);
            const float w = 2.0f * std::sqrt(s.side1[0] * s.side1[0] + s.side1[1] * s.side1[1] + s.side1[2] * s.side1[2]);
            if (s.u0 == 0.0f && s.u1 == 0.25f) { // burnout stamp
                const float x = 0.5f * (s.p0[0] + s.p1[0]), z = 0.5f * (s.p0[2] + s.p1[2]);
                Spot* spot = nullptr;
                for (Spot& q : spots)
                    if (std::hypot(q.x - x, q.z - z) < 0.3f) spot = &q;
                if (!spot) spot = &spots.emplace_back(Spot{x, z, 1.0f, 0.0f});
                spot->keep *= 1.0f - std::min(1.0f, 1.1f * slip);
                spot->width = std::max(spot->width, w);
                continue;
            }
            if (slip < 0.02f) continue;
            const float dx = s.p1[0] - s.p0[0], dz = s.p1[2] - s.p0[2];
            const double l = std::sqrt(dx * dx + dz * dz);
            r.ribbonLength += l;
            widthSum += w * l;
            r.ribbonMax = std::max(r.ribbonMax, slip);
        }
        if (r.ribbonLength > 0.0) r.ribbonWidth = float(widthSum / r.ribbonLength);
        for (const Spot& q : spots)
            if (1.0f - q.keep > r.spotCover) r.spotCover = 1.0f - q.keep, r.spotWidth = q.width;
        return r;
    };
    auto print = [](const char* what, const Result& r) {
        std::printf("    %-24s spin peak %5.2f m/s | ribbon %5.1f m, max %.2f, width %.2f m | burnout spot cover %.2f, "
                    "width %.2f m\n",
                    what, r.maxSlip, r.ribbonLength, r.ribbonMax, r.ribbonWidth, r.spotCover, r.spotWidth);
    };
    const Result gentle = drive(TransferMode::TwoHigh, 0.3f, 0.0f, 5.0, false);
    const Result key = drive(TransferMode::TwoHigh, 0.0f, 0.0f, 5.0, true);
    const Result hard = drive(TransferMode::TwoHigh, 0.7f, 0.0f, 5.0, false);
    const Result full = drive(TransferMode::TwoHigh, 1.0f, 0.0f, 5.0, false);
    const Result full4 = drive(TransferMode::FourHigh, 1.0f, 0.0f, 5.0, false);
    const Result burn2 = drive(TransferMode::TwoHigh, 1.0f, 0.6f, 1.0, false);
    const Result burn5 = drive(TransferMode::TwoHigh, 1.0f, 0.6f, 5.0, false);
    print("2H gentle 0.3", gentle);
    print("2H keyboard W", key);
    print("2H hard 0.7", hard);
    print("2H full pedal", full);
    print("4H full pedal", full4);
    print("2H burnout 1 s", burn2);
    print("2H burnout 5 s", burn5);
    for (const Result* r : {&gentle, &key, &hard, &full4})
        CHECK(r->ribbonMax < 0.05f && r->spotCover < 0.05f, "normal launch left rubber: %.2f / %.2f", r->ribbonMax,
              r->spotCover);
    CHECK(full.maxSlip > 3.0f, "the 2H full-pedal launch no longer spins (%.2f m/s): update the test", full.maxSlip);
    CHECK(full.ribbonLength > 2.0 && full.ribbonMax > 0.05f && full.ribbonMax < 0.3f,
          "2H full-pedal launch mark: %.1f m, max %.2f (want faint)", full.ribbonLength, full.ribbonMax);
    CHECK(full.ribbonWidth < 0.6f * tireWidth, "2H full-pedal launch mark %.2f m wide (tire %.2f m, want narrow)",
          full.ribbonWidth, tireWidth);
    CHECK(burn5.maxSlip > 8.0f && burn5.spotCover > 0.85f && burn5.spotWidth > 0.9f * tireWidth,
          "5 s burnout: spin %.2f m/s, cover %.2f, width %.2f m (want dark, full width)", burn5.maxSlip,
          burn5.spotCover, burn5.spotWidth);
    CHECK(burn2.spotCover < burn5.spotCover - 0.3f && burn2.spotWidth < 0.7f * tireWidth,
          "1 s burnout cover %.2f width %.2f m vs 5 s %.2f (want lighter, narrower)", burn2.spotCover,
          burn2.spotWidth, burn5.spotCover);
}

void skidMarkProfile() {
    for (double hz : {60.0, 120.0, 240.0}) {
        SimulationSettings st = settingsFor(TrackLayout::Flat);
        st.fixedStep = 1.0 / hz;
        Simulation sim(st);
        run(sim, 0.5);
        for (int i = 0; i < int(40.0 / sim.fixedStep()) && sim.vehicle().telemetry().speed < 100.0f / 3.6f; ++i)
            run(sim, sim.fixedStep(), DriverIntent{.throttle = 1.0f});
        cruise(sim, 1.0, 100.0f);
        const float v0 = sim.vehicle().telemetry().speed;
        std::array<TireMarks, kWheelCount> perWheel;
        const float zBrake = position(sim).GetZ();
        sim.setIntent(DriverIntent{.brake = 1.0f});
        int stamps = 0;
        const std::uint64_t w0 = sim.tireMarks().written();
        for (int i = 0; i < int(8.0 / sim.fixedStep()); ++i) {
            sim.tick();
            for (int w = 0; w < kWheelCount; ++w)
                perWheel[std::size_t(w)].update(std::span(&sim.vehicle().contacts()[std::size_t(w)], 1), sim.simTime());
        }
        const float zStop = position(sim).GetZ();
        const TireMarks& all = sim.tireMarks();
        for (std::size_t i = all.size() - std::min<std::uint64_t>(all.written() - w0, all.size()); i < all.size(); ++i)
            stamps += all.oldest(i).u0 == 0.0f && all.oldest(i).u1 > 0.2f && all.oldest(i).u1 < 0.3f ? 1 : 0;

        // 0.5 m bins along z over the stop; per side the composited cover (shader: saturate(slip * 1.1)).
        constexpr float kBin = 0.5f;
        const int bins = int(std::ceil((zStop + 3.0f - zBrake) / kBin)) + 1;
        float worst = 0.0f;
        for (int side = 0; side < 2; ++side) {
            std::vector<float> keep(std::size_t(bins), 1.0f);
            std::vector<int> count(std::size_t(bins), 0);
            for (int w : {side, side + 2}) {
                const TireMarks& m = perWheel[std::size_t(w)];
                std::vector<float> wheelCover(std::size_t(bins), 0.0f);
                for (std::size_t i = 0; i < m.size(); ++i) {
                    const TireMarkSegment& s = m.oldest(i);
                    const float z = 0.5f * (s.p0[2] + s.p1[2]);
                    const int b = int((z - zBrake) / kBin);
                    if (b < 0 || b >= bins) continue;
                    const float a = std::clamp(0.5f * (s.slip0 + s.slip1) * 1.1f, 0.0f, 1.0f);
                    wheelCover[std::size_t(b)] = std::max(wheelCover[std::size_t(b)], a);
                    ++count[std::size_t(b)];
                }
                for (int b = 0; b < bins; ++b) keep[std::size_t(b)] *= 1.0f - wheelCover[std::size_t(b)];
                if (std::getenv("TF_PROFILE") && hz == 120.0) {
                    std::printf("    wheel %d:", w);
                    for (int b = 0; b < bins; b += 2) std::printf(" %.2f", wheelCover[std::size_t(b)]);
                    std::printf("\n");
                }
            }
            std::vector<float> cover(std::size_t(bins), 0.0f);
            int first = -1, last = -1;
            for (int b = 0; b < bins; ++b) {
                cover[std::size_t(b)] = 1.0f - keep[std::size_t(b)];
                if (cover[std::size_t(b)] > 0.05f) {
                    if (first < 0) first = b;
                    last = b;
                }
            }
            if (first < 0) {
                CHECK(false, "%.0f Hz side %d: no rubber", hz, side);
                continue;
            }
            const float len = float(last - first + 1) * kBin;
            auto mean = [&](float from, float to) { // meters from the start of the mark
                float sum = 0.0f;
                int n = 0;
                for (int b = first; b <= last; ++b) {
                    const float d = float(b - first) * kBin;
                    if (d >= from && d < to) sum += cover[std::size_t(b)], ++n;
                }
                return n ? sum / float(n) : 0.0f;
            };
            // The patch heats up over ~0.5 s of sliding, so the even middle starts after that build-up distance.
            const float build = v0 * 0.5f;
            const float start = mean(0.0f, 1.0f), middle = mean(build, len - 4.0f), end = mean(len - 2.0f, len + 1.0f);
            int maxCount = 0;
            for (int b = first; b <= last; ++b) maxCount = std::max(maxCount, count[std::size_t(b)]);
            float minMid = 1.0f, maxMid = 0.0f;
            for (int b = first; b <= last; ++b) {
                const float d = float(b - first) * kBin;
                if (d < build || d >= len - 4.0f) continue;
                minMid = std::min(minMid, cover[std::size_t(b)]);
                maxMid = std::max(maxMid, cover[std::size_t(b)]);
            }
            std::printf("    %3.0f Hz side %d: v0 %.1f km/h, stop %.1f m, mark %.1f m (from +%.1f m), cover start %.2f "
                        "middle %.2f [%.2f..%.2f] last 2 m %.2f, max segments per 0.5 m %d, stamps %d\n",
                        hz, side, v0 * 3.6f, zStop - zBrake, len, float(first) * kBin, start, middle, minMid, maxMid, end,
                        maxCount, stamps);
            worst = std::max(worst, end / std::max(middle, 1e-3f));
            CHECK(start < 0.75f * middle, "%.0f Hz side %d: the mark does not start faint (%.2f vs %.2f)", hz, side,
                  start, middle);
            CHECK(middle > 0.4f && maxMid - minMid < 0.25f, "%.0f Hz side %d: uneven middle %.2f..%.2f", hz, side,
                  minMid, maxMid);
            CHECK(end < 1.1f * middle && end > 0.6f * middle, "%.0f Hz side %d: end %.2f vs middle %.2f", hz, side,
                  end, middle);
            CHECK(maxCount <= 2 * int(std::ceil(kBin / sim.tireMarks().settings().segmentLength)) + 2,
                  "%.0f Hz side %d: %d segments stacked in one 0.5 m bin", hz, side, maxCount);
        }
        CHECK(stamps == 0, "%.0f Hz: %d in-place stamps during a braking stop", hz, stamps);
        (void)worst;
    }
}

// Cosmetic wheel spray from synthetic contact samples on flat ground: rolling throws almost nothing, wheelspin throws
// material backward and up, a locked wheel pushes it forward, concrete throws nothing, particles land and expire, and
// the cap and the dust switch hold.
void wheelSpray() {
    struct Run {
        std::size_t live = 0;
        std::uint64_t emitted = 0, dropped = 0;
        float meanVz = 0.0f, meanVy = 0.0f;
        int dust = 0, resting = 0, below = 0;
        int solids = 0, fine = 0; // mud/dirt bits airborne at the end of the drive, those under 1.5 cm radius
        float maxSolid = 0.0f;
    };
    auto drive = [](SurfaceId surface, float v, float roll, double seconds, WheelSpraySettings st = {},
                    double coast = 0.0) {
        WheelSpray spray;
        spray.setSettings(st);
        spray.setGround([](float, float) { return 0.0f; });
        WheelContactSample c;
        c.contact = true;
        c.surface = surface;
        c.patchCenter = JPH::Vec3(-300.0f, 0.0f, 0.0f); // far from the showcase water
        c.velocity = JPH::Vec3(0.0f, 0.0f, v);
        c.load = 4900.0f;
        c.rollingSpeed = roll;
        c.longitudinalSlipSpeed = roll - v;
        c.width = 0.3f;
        c.radius = 0.4f;
        const TreadSoil soil{};
        Run r;
        constexpr float dt = 1.0f / 60.0f;
        for (int i = 0; i < int(seconds / dt); ++i) {
            spray.update(std::span(&c, 1), std::span(&soil, 1), dt);
            c.patchCenter += c.velocity * dt;
            if (i == int(0.2 / dt)) { // fresh particles: direction of the throw
                int n = 0;
                for (const SprayParticle& p : spray.particles())
                    if (p.kind != SprayKind::Dust && p.age < 0.1f) r.meanVz += p.vel.GetZ(), r.meanVy += p.vel.GetY(), ++n;
                if (n) r.meanVz /= float(n), r.meanVy /= float(n);
            }
        }
        for (const SprayParticle& p : spray.particles()) {
            if (p.resting || (p.kind != SprayKind::Clod && p.kind != SprayKind::Crumb)) continue;
            ++r.solids;
            r.fine += p.size < 0.015f ? 1 : 0;
            r.maxSolid = std::max(r.maxSolid, p.size);
        }
        c.contact = false;
        for (int i = 0; i < int(coast / dt); ++i) {
            spray.update(std::span(&c, 1), std::span(&soil, 1), dt);
            if (i == int(1.0 / dt))
                for (const SprayParticle& p : spray.particles()) r.resting += p.resting ? 1 : 0;
        }
        for (const SprayParticle& p : spray.particles()) {
            r.dust += p.kind == SprayKind::Dust ? 1 : 0;
            r.below += p.pos.GetY() < -0.01f ? 1 : 0;
        }
        r.live = spray.size();
        r.emitted = spray.emitted();
        r.dropped = spray.dropped();
        return r;
    };

    const Run calm = drive(SurfaceId::PackedDirt, 5.0f, 5.0f, 2.0);
    const Run spin = drive(SurfaceId::Mud, 1.0f, 9.0f, 1.0);
    const Run locked = drive(SurfaceId::PackedDirt, 5.0f, 0.0f, 1.0);
    const Run concrete = drive(SurfaceId::Concrete, 1.0f, 9.0f, 1.0);
    const Run land = drive(SurfaceId::Mud, 1.0f, 9.0f, 1.0, {}, 6.0);
    WheelSpraySettings capped;
    capped.maxParticles = 50;
    const Run cap = drive(SurfaceId::Mud, 1.0f, 9.0f, 1.0, capped);
    WheelSpraySettings noDust;
    noDust.dust = false;
    const Run sand = drive(SurfaceId::Sand, 1.0f, 9.0f, 1.0);
    const Run sandNoDust = drive(SurfaceId::Sand, 1.0f, 9.0f, 1.0, noDust);
    std::printf("  calm dirt roll 2 s: %llu emitted; mud spin 1 s: %llu emitted, fresh vz %.2f vy %.2f m/s; "
                "dirt locked: vz %.2f m/s; concrete spin: %llu\n",
                (unsigned long long)calm.emitted, (unsigned long long)spin.emitted, spin.meanVz, spin.meanVy,
                locked.meanVz, (unsigned long long)concrete.emitted);
    std::printf("  after 1 s coasting %d resting, after 6 s %zu live (%d below ground); cap 50: %zu live, %llu "
                "dropped; sand dust %d, without dust %d\n",
                land.resting, land.live, land.below, cap.live, (unsigned long long)cap.dropped, sand.dust,
                sandNoDust.dust);
    std::printf("  mud spin sizes: %d airborne bits, %.0f %% under 1.5 cm radius, largest %.3f m\n", spin.solids,
                spin.solids ? 100.0f * float(spin.fine) / float(spin.solids) : 0.0f, spin.maxSolid);
    CHECK(spin.solids > 0 && spin.fine >= int(0.85f * float(spin.solids)) && spin.maxSolid <= 0.03f,
          "spray sizes: %d of %d fine, largest radius %.3f m (want mostly fine, no clod above 6 cm across)", spin.fine,
          spin.solids, spin.maxSolid);
    CHECK(calm.emitted <= 3, "calm roll emitted %llu", (unsigned long long)calm.emitted);
    CHECK(spin.emitted > 100, "mud wheelspin emitted only %llu", (unsigned long long)spin.emitted);
    CHECK(spin.meanVz < -1.0f && spin.meanVy > 1.0f, "wheelspin throw vz %.2f vy %.2f (want backward and up)",
          spin.meanVz, spin.meanVy);
    CHECK(locked.emitted > 0 && locked.meanVz > 1.0f, "locked wheel push vz %.2f (want forward)", locked.meanVz);
    CHECK(concrete.emitted == 0, "concrete emitted %llu", (unsigned long long)concrete.emitted);
    CHECK(land.resting > 0 && land.live == 0 && land.below == 0, "landing: %d resting, %zu live, %d below",
          land.resting, land.live, land.below);
    CHECK(cap.live <= 50 && cap.dropped > 0, "cap: %zu live, %llu dropped", cap.live,
          (unsigned long long)cap.dropped);
    CHECK(sand.dust > 0 && sandNoDust.dust == 0, "dust %d, with dust off %d", sand.dust, sandNoDust.dust);
}

float yawDeg(const Simulation& sim) {
    const JPH::Vec3 f = sim.currentSnapshot().body.rotation.RotateAxisZ();
    return std::atan2(f.GetX(), f.GetZ()) * 57.2958f;
}

// Full brake from 50 km/h with 40 % right steer on the dirt and grass lanes: without ABS the wheels lock and the
// vehicle barely turns; with ABS the wheels keep turning, the vehicle turns and stops no later on the straight.
void absPreventsLock() {
    const struct {
        int feature;
        const char* name;
    } lanes[] = {{1, "dirt"}, {3, "grass"}};
    for (const auto& lane : lanes) {
        for (double hz : {60.0, 120.0, 240.0}) {
            float dist[2][2] = {}, yaw[2] = {};
            double lock[2] = {};
            for (int steer = 0; steer < 2; ++steer) {
                for (int abs = 0; abs < 2; ++abs) {
                    SimulationSettings s = settingsFor(TrackLayout::SurfaceLanes);
                    s.fixedStep = 1.0 / hz;
                    Simulation sim(s);
                    sim.requestTeleport(lane.feature);
                    run(sim, 1.0);
                    sim.requestAidControls({abs != 0, TractionMode::Off, 0});
                    sim.setIntent({.throttle = 0.6f});
                    for (int i = 0; i < int(12.0 / sim.fixedStep()) && sim.vehicle().telemetry().speed < 50.0f / 3.6f; ++i)
                        sim.tick();
                    const JPH::Vec3 p0 = position(sim);
                    const float yaw0 = yawDeg(sim);
                    double t = 0.0, locked = 0.0;
                    sim.setIntent({.brake = 1.0f, .steer = steer ? 0.4f : 0.0f, .analog = true});
                    while (sim.vehicle().telemetry().speed > 0.05f && t < 10.0) {
                        sim.tick();
                        t += sim.fixedStep();
                        for (const WheelTelemetry& w : sim.vehicle().telemetry().wheels)
                            if (w.contact && std::abs(w.groundSpeed) > 2.0f &&
                                std::abs(w.rollingSpeed) < 0.2f * std::abs(w.groundSpeed))
                                locked += sim.fixedStep();
                    }
                    dist[steer][abs] = (position(sim) - p0).Length();
                    if (steer) {
                        yaw[abs] = yawDeg(sim) - yaw0;
                        lock[abs] = locked;
                    }
                }
            }
            std::printf("    %-5s %3.0f Hz: straight %.2f m (ABS %.2f m); steered: yaw %.1f deg, lock %.2f wheel-s "
                        "(ABS: yaw %.1f deg, lock %.2f wheel-s)\n",
                        lane.name, hz, dist[0][0], dist[0][1], yaw[0], lock[0], yaw[1], lock[1]);
            CHECK(lock[0] > 3.0, "%s %.0f Hz: wheels did not lock without ABS (%.2f wheel-s)", lane.name, hz, lock[0]);
            // At 60 Hz the controller runs at 60 Hz too and overshoots: brief locks, and the stop is no shorter.
            CHECK(lock[1] < (hz < 100.0 ? 1.5 : 0.5), "%s %.0f Hz: wheels locked with ABS (%.2f wheel-s)", lane.name, hz,
                  lock[1]);
            CHECK(std::abs(yaw[0]) < 25.0f, "%s %.0f Hz: locked wheels still steer (%.1f deg)", lane.name, hz, yaw[0]);
            CHECK(yaw[1] < -35.0f, "%s %.0f Hz: no steering with ABS (%.1f deg)", lane.name, hz, yaw[1]);
            CHECK(dist[0][1] <= dist[0][0] * (hz < 100.0 ? 1.1f : 1.0f), "%s %.0f Hz: ABS stop %.2f m longer than locked %.2f m", lane.name,
                  hz, dist[0][1], dist[0][0]);
        }
    }
}

// Hill start on the 15 degree concrete slope with the front-left wheel over a bottomless pit: open differentials send
// the torque to the hanging wheel and the vehicle does not climb; with all diffs locked it drives out.
void diffLocksClimb() {
    for (double hz : {60.0, 120.0, 240.0}) {
        float climbed[2] = {}, airSpin[2] = {};
        for (int locked = 0; locked < 2; ++locked) {
            SimulationSettings s = settingsFor(TrackLayout::SlopeWithWheelPit);
            s.fixedStep = 1.0 / hz;
            s.vehicle.tuning.frontDiff = s.vehicle.tuning.rearDiff = s.vehicle.tuning.centerDiff = DiffMode::Open;
            Simulation sim(s);
            run(sim, 2.0, {.brake = 1.0f});
            sim.requestToggleLowRange();
            sim.requestAidControls({false, TractionMode::Off, locked ? AidControls::kDiffLockStages - 1 : 0});
            run(sim, 0.1, {.brake = 1.0f});
            const float z0 = position(sim).GetZ();
            sim.setIntent({.throttle = 1.0f});
            for (int i = 0; i < int(8.0 / sim.fixedStep()); ++i) {
                sim.tick();
                const WheelTelemetry& fl = sim.vehicle().telemetry().wheels[0];
                if (!fl.contact) airSpin[locked] = std::max(airSpin[locked], std::abs(fl.rollingSpeed));
            }
            climbed[locked] = position(sim).GetZ() - z0;
            CHECK(sim.vehicle().telemetry().controls.diffLocks == (locked ? AidControls::kDiffLockStages - 1 : 0),
                  "diff lock stage not applied");
        }
        std::printf("    %3.0f Hz: open: %.2f m, hanging wheel up to %.1f m/s; all locked: %.2f m\n", hz, climbed[0],
                    airSpin[0], climbed[1]);
        CHECK(climbed[0] < 0.5f && airSpin[0] > 5.0f, "%.0f Hz: open diffs climbed %.2f m (air spin %.1f m/s)", hz,
              climbed[0], airSpin[0]);
        CHECK(climbed[1] > 4.0f, "%.0f Hz: locked diffs climbed only %.2f m", hz, climbed[1]);
    }
}

// Full-throttle launch on the flat mud lane: traction control cuts the wheelspin.
void tractionControlMud() {
    for (double hz : {60.0, 120.0, 240.0}) {
        float maxSlip[3] = {};
        double spin[3] = {}, active[3] = {};
        float v3[3] = {};
        for (int m = 0; m < 3; ++m) {
            SimulationSettings s = settingsFor(TrackLayout::SurfaceLanes);
            s.fixedStep = 1.0 / hz;
            Simulation sim(s);
            sim.requestTeleport(2);
            run(sim, 1.0);
            sim.requestAidControls({false, TractionMode(m), 0});
            sim.setIntent({.throttle = 1.0f});
            for (int i = 0; i < int(3.0 / sim.fixedStep()); ++i) {
                sim.tick();
                float slip = 0.0f;
                for (const WheelContactSample& c : sim.vehicle().contacts())
                    if (c.contact) slip = std::max(slip, std::abs(c.longitudinalSlipSpeed));
                maxSlip[m] = std::max(maxSlip[m], slip);
                if (slip > 1.0f) spin[m] += sim.fixedStep();
                if (sim.vehicle().telemetry().tcActive) active[m] += sim.fixedStep();
            }
            v3[m] = sim.vehicle().telemetry().speed;
        }
        std::printf("    %3.0f Hz: off: max slip %.2f m/s, spin %.2f s, v(3 s) %.2f; on: %.2f m/s, %.2f s, v %.2f, "
                    "active %.2f s; off-road: %.2f m/s, %.2f s, v %.2f\n",
                    hz, maxSlip[0], spin[0], v3[0], maxSlip[1], spin[1], v3[1], active[1], maxSlip[2], spin[2], v3[2]);
        CHECK(active[0] == 0.0, "TC active while off");
        CHECK(active[1] > 0.5, "%.0f Hz: TC never engaged", hz);
        // The peak is a single tick; at 60 Hz Jolt's wheel speeds alternate near the grip limit, so the peak there
        // only has to drop by 20 %. The spin time must drop by 30 % at every rate.
        CHECK(maxSlip[1] < (hz < 100.0 ? 0.8f : 0.7f) * maxSlip[0] && spin[1] < 0.7 * spin[0],
              "%.0f Hz: TC on did not cut wheelspin", hz);
        CHECK(maxSlip[2] < 0.8f * maxSlip[0], "%.0f Hz: TC off-road did not cut wheelspin", hz);
        CHECK(v3[1] > 0.8f * v3[0], "%.0f Hz: TC on bogged down (v %.2f vs %.2f)", hz, v3[1], v3[0]);
    }
}

// DrivingAids alone, on synthetic wheel states (no Jolt): ABS dumps a locked wheel to zero torque and builds up again
// once it rolls; TC brakes only the wheel that spins ahead of the others, cuts the throttle when all spin, and lets
// the throttle recover while the vehicle rolls back against the selected direction.
void drivingAidsLogic() {
    const DrivingAidTuning t;
    const float dt = 1.0f / 120.0f;
    auto wheels = [](float ground, std::array<float, 4> rolling) {
        AidInput in;
        for (int i = 0; i < 4; ++i) in.wheels[i] = {true, ground, rolling[i]};
        return in;
    };
    {
        DrivingAids aids;
        AidInput in = wheels(10.0f, {0.0f, 10.0f, 10.0f, 10.0f});
        in.brake = 1.0f;
        in.maxBrakeTorque.fill(3000.0f);
        in.dt = dt;
        const AidOutput off = aids.update(t, {false, TractionMode::Off, 0}, in);
        CHECK(off.brakeTorque[0] == 3000.0f && !off.absActive, "ABS off: torque %.0f", off.brakeTorque[0]);
        double dumpTime = -1.0;
        AidOutput out;
        for (int i = 0; i < 30 && dumpTime < 0.0; ++i) {
            out = aids.update(t, {true, TractionMode::Off, 0}, in);
            if (out.brakeTorque[0] == 0.0f) dumpTime = (i + 1) * dt;
        }
        std::printf("    ABS: locked wheel dumped to 0 N*m after %.3f s; rolling wheels %.0f N*m\n", dumpTime,
                    out.brakeTorque[1]);
        CHECK(dumpTime > 0.0 && dumpTime <= 0.05, "ABS did not dump the locked wheel to zero (%.3f s)", dumpTime);
        CHECK(out.brakeTorque[1] == 3000.0f && out.absActive, "ABS touched the rolling wheels (%.0f)", out.brakeTorque[1]);
        in.wheels[0].rollingSpeed = 9.5f; // slip 0.05 < absApplySlip
        double fullTime = -1.0;
        for (int i = 0; i < 60 && fullTime < 0.0; ++i)
            if (aids.update(t, {true, TractionMode::Off, 0}, in).absPressure[0] >= 1.0f) fullTime = (i + 1) * dt;
        std::printf("    ABS: full pressure again %.3f s after the wheel rolls\n", fullTime);
        CHECK(fullTime > 0.05 && fullTime < 0.2, "ABS pressure build-up %.3f s", fullTime);
    }
    {
        DrivingAids aids;
        AidInput in = wheels(1.0f, {8.0f, 1.0f, 1.0f, 1.0f});
        in.throttle = 1.0f;
        in.dt = dt;
        AidOutput out;
        for (int i = 0; i < 60; ++i) out = aids.update(t, {false, TractionMode::On, 0}, in);
        std::printf("    TC: one wheel spinning: brake %.0f / %.0f / %.0f / %.0f N*m, throttle factor %.2f\n",
                    out.tcBrake[0], out.tcBrake[1], out.tcBrake[2], out.tcBrake[3], out.throttleFactor);
        CHECK(out.tcBrake[0] > 500.0f && out.tcBrake[1] == 0.0f && out.tcBrake[2] == 0.0f && out.tcBrake[3] == 0.0f,
              "TC must brake only the spinning wheel");
        CHECK(out.throttleFactor > 0.99f, "TC cut the throttle with three wheels gripping (%.2f)", out.throttleFactor);
        in = wheels(1.0f, {8.0f, 8.0f, 8.0f, 8.0f});
        in.throttle = 1.0f;
        in.dt = dt;
        for (int i = 0; i < 120; ++i) out = aids.update(t, {false, TractionMode::On, 0}, in);
        std::printf("    TC: all wheels spinning: throttle factor %.2f after 1 s, engine throttle %.2f\n",
                    out.throttleFactor, out.throttle);
        CHECK(out.throttleFactor < 0.2f && out.tcActive, "TC did not cut the throttle (%.2f)", out.throttleFactor);
        const float cut = out.throttleFactor;
        in = wheels(-1.0f, {8.0f, 8.0f, 8.0f, 8.0f}); // rolling back with the wheels spinning forward
        in.throttle = 1.0f;
        in.dt = dt;
        for (int i = 0; i < 60; ++i) out = aids.update(t, {false, TractionMode::On, 0}, in);
        std::printf("    TC: rolling back: throttle factor %.2f -> %.2f in 0.5 s\n", cut, out.throttleFactor);
        CHECK(out.throttleFactor > cut + 0.2f, "TC kept cutting while rolling back (%.2f)", out.throttleFactor);
        const AidOutput off = aids.update(t, {false, TractionMode::Off, 0}, in);
        CHECK(off.throttle == 1.0f && off.tcBrake[0] == 0.0f && !off.tcActive, "TC off still acts");
    }
}

// Friction ellipse: a full brake with 40 % steer from 50 km/h on concrete without ABS. With Jolt's independent limits
// the locked wheels still steer; with the ellipse they lose side grip and the vehicle keeps going straighter. The
// straight stop and moderate cornering stay the same.
void frictionEllipse() {
    float heading[2] = {}, offset[2] = {}, straight[2] = {}, corner[2] = {};
    for (int ellipse = 0; ellipse < 2; ++ellipse) {
        for (int steer = 0; steer < 2; ++steer) {
            SimulationSettings s = settingsFor(TrackLayout::Flat);
            s.vehicle.tuning.tireFrictionEllipse = ellipse != 0;
            Simulation sim(s);
            run(sim, 0.5);
            sim.setIntent({.throttle = 0.6f});
            for (int i = 0; i < int(12.0 / sim.fixedStep()) && sim.vehicle().telemetry().speed < 50.0f / 3.6f; ++i)
                sim.tick();
            const JPH::Vec3 p0 = position(sim);
            const float yaw0 = yawDeg(sim);
            const JPH::Vec3 fwd = sim.currentSnapshot().body.rotation.RotateAxisZ();
            run(sim, 1.0, {.brake = 1.0f, .steer = steer ? 0.4f : 0.0f});
            if (steer) {
                heading[ellipse] = yawDeg(sim) - yaw0;
                const JPH::Vec3 d = position(sim) - p0;
                offset[ellipse] = (d - fwd * d.Dot(fwd)).Length();
            }
            run(sim, 8.0, {.brake = 1.0f, .steer = steer ? 0.4f : 0.0f});
            if (!steer) straight[ellipse] = (position(sim) - p0).Length();
        }
        // Steady circle at ~40 km/h with 30 % steer (below the limit).
        SimulationSettings s = settingsFor(TrackLayout::Flat);
        s.vehicle.tuning.tireFrictionEllipse = ellipse != 0;
        Simulation sim(s);
        const JPH::BodyID id = sim.vehicle().bodyId();
        double t = 0, sum = 0;
        int n = 0;
        while (t < 10.0) {
            const auto& tel = sim.vehicle().telemetry();
            sim.setIntent({.throttle = std::clamp(0.3f + 0.4f * (40.0f / 3.6f - tel.speed), 0.0f, 1.0f),
                           .steer = t > 2.5 ? 0.3f : 0.0f});
            sim.tick();
            t += sim.fixedStep();
            if (t < 7.0) continue;
            sum += std::abs(tel.forwardSpeed * sim.world().bodies().GetAngularVelocity(id).GetY());
            ++n;
        }
        corner[ellipse] = float(sum / n);
    }
    std::printf("    locked wheels, 1 s after a steered full brake: heading %.1f -> %.1f deg, sideways %.2f -> %.2f m "
                "(ellipse off -> on); straight stop %.2f -> %.2f m; 30 %% steer circle %.2f -> %.2f m/s^2\n",
                heading[0], heading[1], offset[0], offset[1], straight[0], straight[1], corner[0], corner[1]);
    CHECK(std::abs(heading[1]) < 0.75f * std::abs(heading[0]) && offset[1] < 0.75f * offset[0],
          "ellipse: locked wheels still steer (heading %.1f vs %.1f deg)", heading[1], heading[0]);
    CHECK(std::abs(straight[1] - straight[0]) < 0.02f * straight[0], "ellipse changed the straight stop %.2f vs %.2f m",
          straight[1], straight[0]);
    CHECK(std::abs(corner[1] - corner[0]) < 0.03f * corner[0], "ellipse changed moderate cornering %.2f vs %.2f",
          corner[1], corner[0]);
}

WheelContactSample syntheticWheel(SurfaceId surface, float v, float roll, JPH::Vec3 at) {
    WheelContactSample c;
    c.contact = true;
    c.surface = surface;
    c.patchCenter = at;
    c.velocity = JPH::Vec3(0.0f, 0.0f, v);
    c.load = 4900.0f;
    c.rollingSpeed = roll;
    c.longitudinalSlipSpeed = roll - v;
    c.width = 0.3f;
    c.radius = 0.4f;
    return c;
}

// Tread soil per surface (cosmetic): mud loads the most and stays longest, dirt in between, dry sand a light tan
// dusting capped at ~0.15 of mud that sheds within a meter or two, grass the brown earth under the turf with a few
// blades (no green coat).
void treadSoilSurfaces() {
    struct Result {
        TreadSoil soil;
        float shed = 0.0f; // m of concrete to 1/e
    };
    auto drive = [](SurfaceId s) {
        Result r;
        constexpr float dt = 1.0f / 120.0f, v = 5.0f;
        WheelContactSample c = syntheticWheel(s, v, v, JPH::Vec3(-300.0f, 0.0f, 0.0f));
        for (int i = 0; i < int(8.0f / dt); ++i) r.soil.update(c, dt); // 40 m
        TreadSoil t = r.soil;
        c.surface = SurfaceId::Concrete;
        float d = 0.0f;
        while (t.amount > r.soil.amount / 2.718f && d < 100.0f) {
            t.update(c, dt);
            d += v * dt;
        }
        r.shed = d;
        return r;
    };
    const Result mud = drive(SurfaceId::Mud), dirt = drive(SurfaceId::PackedDirt), sand = drive(SurfaceId::Sand),
                 grass = drive(SurfaceId::Grass);
    auto print = [](const char* n, const Result& r) {
        std::printf("  %-5s amount %.3f wet %.2f stick %.2f flecks %.2f colour %.3f %.3f %.3f, sheds to 1/e in %.1f m\n",
                    n, r.soil.amount, r.soil.wetness, r.soil.stick, r.soil.flecks, r.soil.color[0], r.soil.color[1],
                    r.soil.color[2], r.shed);
    };
    print("mud", mud);
    print("dirt", dirt);
    print("sand", sand);
    print("grass", grass);
    CHECK(mud.soil.amount > 0.9f && mud.soil.amount > dirt.soil.amount && dirt.soil.amount > sand.soil.amount,
          "pickup order mud %.2f > dirt %.2f > sand %.2f", mud.soil.amount, dirt.soil.amount, sand.soil.amount);
    CHECK(sand.soil.amount <= 0.2f * mud.soil.amount, "sand coverage %.3f above 0.2 of mud %.3f", sand.soil.amount,
          mud.soil.amount);
    CHECK(mud.shed > dirt.shed && dirt.shed > sand.shed && sand.shed < 2.0f,
          "shedding mud %.1f > dirt %.1f > sand %.1f m (sand < 2 m)", mud.shed, dirt.shed, sand.shed);
    const float* sc = sand.soil.color;
    const float sandMax = std::max({sc[0], sc[1], sc[2]}), sandMin = std::min({sc[0], sc[1], sc[2]});
    CHECK(sandMax < 0.35f && sandMin > 0.4f * sandMax, "sand colour %.3f %.3f %.3f is not a muted tan", sc[0], sc[1],
          sc[2]);
    const float* gc = grass.soil.color;
    CHECK(gc[0] > gc[1] && gc[1] > gc[2], "grass soil colour %.3f %.3f %.3f is not brown earth", gc[0], gc[1], gc[2]);
    CHECK(grass.soil.flecks > 0.05f && grass.soil.flecks < 0.5f, "grass blade flecks %.2f (want sparse)",
          grass.soil.flecks);
    CHECK(grass.soil.amount < 0.5f * mud.soil.amount, "grass load %.2f vs mud %.2f (want modest)", grass.soil.amount,
          mud.soil.amount);
    CHECK(mud.soil.flecks == 0.0f && sand.soil.flecks == 0.0f, "flecks off grass: mud %.2f sand %.2f",
          mud.soil.flecks, sand.soil.flecks);
}

// Cosmetic body mud from four synthetic wheels: rear wheelspin in mud coats the rear arches, the rear quarters and
// the rear bumper within seconds, the slowly rolling front wheels leave the front clean, concrete adds nothing, dry
// sand stays a thin dust film, grass leaves brown earth with sparse blades, and a puddle washes the wet regions.
void bodyMud() {
    auto run = [](BodyMud& mud, SurfaceId s, float frontRoll, float rearRoll, float seconds, JPH::Vec3 at,
                  std::array<TreadSoil, 4> soil = {}) {
        constexpr float dt = 1.0f / 60.0f;
        std::array<WheelContactSample, 4> w;
        for (int i = 0; i < 4; ++i)
            w[std::size_t(i)] = syntheticWheel(s, 1.0f, i < 2 ? frontRoll : rearRoll,
                                               at + JPH::Vec3(i % 2 ? -0.8f : 0.8f, 0.0f, i < 2 ? 1.4f : -1.4f));
        for (int i = 0; i < int(seconds / dt); ++i) mud.update(w, soil, dt);
    };
    const JPH::Vec3 dry(-300.0f, 0.0f, 0.0f);
    BodyMud m;
    run(m, SurfaceId::Mud, 1.0f, 9.0f, 1.0f, dry);
    const float arch1 = m.coat(kMudArchRear + 2).amount;
    run(m, SurfaceId::Mud, 1.0f, 9.0f, 2.0f, dry);
    const BodyMudCoat& arch = m.coat(kMudArchRear + 2);
    const float quarter = m.coat(kMudBehind + 3).amount, bumper = m.coat(kMudRearBumper).amount;
    const float front = std::max({m.coat(kMudArchRear + 0).amount, m.coat(kMudBehind + 0).amount,
                                  m.coat(kMudFrontBumper).amount, m.coat(kMudArchFront + 2).amount});
    std::printf("  mud rear wheelspin: rear arch %.2f after 1 s, %.2f after 3 s (wet %.2f), rear quarter %.2f, "
                "rear bumper %.2f, front regions max %.2f\n",
                arch1, arch.amount, arch.wetness, quarter, bumper, front);
    CHECK(arch1 > 0.3f && arch.amount > 0.7f, "rear arch builds %.2f / %.2f (want fast)", arch1, arch.amount);
    CHECK(quarter > 0.3f && bumper > 0.2f && quarter < arch.amount, "rear quarter %.2f bumper %.2f", quarter, bumper);
    CHECK(front < 0.5f * quarter, "front regions %.2f while only the rear wheels spin", front);

    BodyMud concrete;
    run(concrete, SurfaceId::Concrete, 9.0f, 9.0f, 3.0f, dry);
    CHECK(concrete.maxAmount() == 0.0f, "clean tread on concrete coats %.3f", concrete.maxAmount());

    BodyMud sand;
    run(sand, SurfaceId::Sand, 9.0f, 9.0f, 10.0f, dry);
    BodyMud grass;
    run(grass, SurfaceId::Grass, 9.0f, 9.0f, 10.0f, dry);
    const BodyMudCoat& g = grass.coat(kMudArchRear + 2);
    std::printf("  10 s wheelspin: sand max %.3f, grass arch %.2f colour %.3f %.3f %.3f flecks %.2f\n",
                sand.maxAmount(), g.amount, g.color[0], g.color[1], g.color[2], g.flecks);
    CHECK(sand.maxAmount() < 0.15f, "sand coat %.3f (want a light dust film)", sand.maxAmount());
    CHECK(g.amount > 0.1f && g.amount < 0.5f, "grass coat %.2f", g.amount);
    CHECK(g.color[0] > g.color[1] && g.color[1] > g.color[2], "grass coat colour %.3f %.3f %.3f is not brown",
          g.color[0], g.color[1], g.color[2]);
    CHECK(g.flecks > 0.0f && g.flecks < 0.5f, "grass coat flecks %.2f (want sparse)", g.flecks);

    const WaterInfo water = TestTrack::water();
    const JPH::Vec3 puddle(0.5f * (water.x0 + water.x1), water.level - 0.3f, 0.5f * (water.z0 + water.z1));
    const float before = m.coat(kMudArchRear + 2).amount;
    std::array<WheelContactSample, 4> w;
    for (int i = 0; i < 4; ++i) {
        w[std::size_t(i)] = syntheticWheel(SurfaceId::Concrete, 3.0f, 3.0f, puddle);
    }
    for (int i = 0; i < 300; ++i) m.update(w, std::array<TreadSoil, 4>{}, 1.0f / 60.0f);
    const float after = m.coat(kMudArchRear + 2).amount;
    std::printf("  puddle 5 s at 3 m/s: rear arch %.2f -> %.2f\n", before, after);
    CHECK(after < 0.4f * before, "puddle wash %.2f -> %.2f", before, after);
}


// Lap timer on synthetic gates: a square loop with the start/finish line and two sector gates.
void lapTimerLogic() {
    LapTimer lt;
    // Gates on the sides of a 100 x 100 m square driven counterclockwise seen from +Y... in x/z: (0,0) -> (100,0) ->
    // (100,100) -> (0,100) -> back. Start/finish on the first side, sectors on the second and third.
    lt.setGates({{50.0f, 0.0f, 1.0f, 0.0f, 5.0f, false}, {100.0f, 50.0f, 0.0f, 1.0f, 5.0f, true},
                 {50.0f, 100.0f, -1.0f, 0.0f, 5.0f, true}});
    struct P { float x, z; };
    std::vector<P> loop;
    for (int k = 0; k < 100; ++k) loop.push_back({float(k), 0.0f});
    for (int k = 0; k < 100; ++k) loop.push_back({100.0f, float(k)});
    for (int k = 0; k < 100; ++k) loop.push_back({100.0f - float(k), 100.0f});
    for (int k = 0; k < 100; ++k) loop.push_back({0.0f, 100.0f - float(k)});
    double t = 0.0;
    auto drive = [&](int laps, double dt) {
        for (int l = 0; l < laps; ++l)
            for (std::size_t i = 0; i < loop.size(); ++i) {
                const P& a = loop[i];
                const P& b = loop[(i + 1) % loop.size()];
                lt.update(a.x, a.z, b.x, b.z, t, t + dt);
                t += dt;
            }
    };
    drive(1, 0.1); // the first pass of the line starts the clock at x = 50 (t = 5.0 s)
    CHECK(lt.running() && lt.laps() == 0, "running %d laps %d", lt.running(), lt.laps());
    drive(1, 0.1);
    CHECK(lt.laps() == 1, "laps %d", lt.laps());
    CHECK(std::abs(lt.lastLap() - 40.0) < 1e-6, "lap %.6f s (400 m at 10 m/s)", lt.lastLap());
    const auto& sec = lt.lastSectors();
    CHECK(sec.size() == 3 && std::abs(sec[0] - 10.0) < 1e-6 && std::abs(sec[1] - 10.0) < 1e-6 && std::abs(sec[2] - 20.0) < 1e-6,
          "sectors %zu: %.3f %.3f %.3f", sec.size(), sec.size() > 0 ? sec[0] : -1.0, sec.size() > 1 ? sec[1] : -1.0,
          sec.size() > 2 ? sec[2] : -1.0);
    drive(1, 0.05); // half of this lap was driven at 10 m/s, half at 20 m/s
    CHECK(lt.laps() == 2 && std::abs(lt.lastLap() - 37.5) < 1e-6, "laps %d last %.3f", lt.laps(), lt.lastLap());
    drive(1, 0.05);
    CHECK(lt.laps() == 3 && std::abs(lt.lastLap() - 20.0) < 1e-6 && std::abs(lt.bestLap() - 20.0) < 1e-6,
          "laps %d last %.3f best %.3f", lt.laps(), lt.lastLap(), lt.bestLap());
    // Both sectors of the running lap are done: the next forward crossing of the line completes it.
    lt.update(55.0f, 0.0f, 45.0f, 0.0f, t, t + 0.1); // backwards: nothing
    CHECK(lt.laps() == 3 && lt.running() && lt.nextGate() == 0, "backward crossing changed the state");
    lt.update(45.0f, 0.0f, 55.0f, 0.0f, t, t + 0.1);
    CHECK(lt.laps() == 4 && lt.nextGate() == 1, "laps %d next gate %d", lt.laps(), lt.nextGate());
    // A cut back to the line without the sectors restarts the clock and counts nothing.
    lt.update(45.0f, 0.0f, 55.0f, 0.0f, t, t + 0.1);
    CHECK(lt.laps() == 4 && lt.running() && lt.nextGate() == 1, "a lap without sectors counted (%d)", lt.laps());
    lt.update(49.0f, 100.0f, 51.0f, 100.0f, t, t + 0.1); // wrong direction over gate 2: ignored
    CHECK(lt.running(), "backward sector crossing voided the lap");
    lt.update(51.0f, 100.0f, 49.0f, 100.0f, t, t + 0.1); // gate 2 before gate 1: not counted, gate 1 still next
    CHECK(lt.running() && lt.nextGate() == 1, "out-of-order sector counted (next gate %d)", lt.nextGate());
    CHECK(lt.update(45.0f, 0.0f, 55.0f, 0.0f, t, t + 0.1) == LapTimer::Event::Invalid && lt.laps() == 4,
          "a lap with a missed sector counted (%d)", lt.laps());
    // Outside the gate half width: no crossing.
    lt.reset();
    lt.update(45.0f, 20.0f, 55.0f, 20.0f, t, t + 0.1);
    CHECK(!lt.running(), "crossing 20 m beside the line counted");
}

// Mini handling track: layout, clear asphalt, the teleport, and a scripted lap timed by the simulation.
void miniTrackLap() {
    const MiniTrackInfo& mt = miniTrack();
    const std::size_t n = mt.center.size();
    float maxGap = 0.0f;
    for (std::size_t i = 0; i < n; ++i)
        maxGap = std::max(maxGap, std::hypot(mt.center[(i + 1) % n].x - mt.center[i].x, mt.center[(i + 1) % n].z - mt.center[i].z));
    std::printf("  lap %.1f m, %zu center points, max spacing %.2f m, gates at %.0f / %.0f m\n", mt.length, n, maxGap,
                mt.gateArc[1], mt.gateArc[2]);
    CHECK(mt.length > 800.0f && mt.length < 1500.0f, "lap length %.1f m", mt.length);
    CHECK(maxGap < 1.5f, "center line spacing %.2f m", maxGap);

    Simulation sim(settingsFor(TrackLayout::ProvingGround));
    const auto& features = sim.track().features();
    CHECK(features.size() == 18 && features.back().name == "Mini handling track", "%zu features, last '%s'",
          features.size(), features.back().name.c_str());
    // The asphalt is clear: a ray down every 5 m along the center line lands on the ribbon (no tyres, walls, rocks).
    int blocked = 0;
    for (std::size_t i = 0; i < n; i += 5) {
        const float x = mt.center[i].x, z = mt.center[i].z, ground = miniTrackGround(x, z);
        const auto hit = sim.castRayIgnoringVehicle(JPH::Vec3(x, ground + 5.0f, z), JPH::Vec3(x, ground - 5.0f, z));
        const float y = hit ? ground + 5.0f - 10.0f * *hit : -1e9f;
        if (std::abs(y - (ground + 0.03f)) > 0.02f) ++blocked;
        if (TestTrack::groundSurfaceAt(x, z) != SurfaceId::Asphalt) ++blocked;
    }
    CHECK(blocked == 0, "%d center line samples not on bare asphalt", blocked);
    // The traps are the Gravel surface (no Sand or Concrete left on the circuit ground).
    int gravel = 0, sandOrConcrete = 0;
    for (float x = mt.area.xMin; x <= mt.area.xMax; x += 2.0f)
        for (float z = mt.area.zMin; z <= mt.area.zMax; z += 2.0f) {
            const SurfaceId s = miniTrackSurface(x, z);
            gravel += s == SurfaceId::Gravel;
            sandOrConcrete += s == SurfaceId::Sand || s == SurfaceId::Concrete;
        }
    std::printf("  ground samples every 2 m: %d gravel, %d sand or concrete\n", gravel, sandOrConcrete);
    CHECK(gravel > 100 && sandOrConcrete == 0, "%d gravel, %d sand / concrete samples", gravel, sandOrConcrete);

    sim.requestTeleport(int(features.size()) - 1);
    run(sim, 1.0, {.brake = 1.0f});
    const JPH::Vec3 p0 = position(sim);
    CHECK(std::hypot(p0.GetX() - mt.start.position.GetX(), p0.GetZ() - mt.start.position.GetZ()) < 0.5f,
          "spawn at (%.1f, %.1f)", p0.GetX(), p0.GetZ());
    CHECK(sim.lapTimer().hasGates() && !sim.lapTimer().running(), "lap timer before the line");
    int onAsphalt = 0;
    for (const auto& w : sim.vehicle().telemetry().wheels) onAsphalt += w.contact && w.surface == SurfaceId::Asphalt;
    CHECK(onAsphalt == kWheelCount, "%d wheels report the Asphalt contact on the grid", onAsphalt);

    AutopilotSettings a;
    a.lateralAccel = 5.0f;
    a.planDecel = 5.0f;
    a.planAccel = 40.0f;
    a.maxThrottle = 1.0f;
    a.steerEase = 0.6f;
    a.kp = 0.5f;
    a.kb = 0.6f;
    a.maxBrake = 1.0f;
    a.lookaheadMin = 6.0f;
    a.lookaheadPerSpeed = 0.45f;
    a.lookaheadMax = 26.0f;
    a.cornerLookahead = 0.8f;
    a.lookaheadCornerMin = 4.0f;
    Autopilot driver(a);
    driver.setRoute(mt.route(60.0f), sim.vehicleConfig().wheelBase,
                    sim.vehicleConfig().axles[kFrontAxle].maxSteerAngleDeg * 0.0174532925f);
    float maxLine = 0.0f, maxKmh = 0.0f;
    for (double t = 0.0; t < 200.0 && sim.lapTimer().laps() < 1; t += sim.fixedStep()) {
        const JPH::RMat44 xf = sim.vehicle().bodyTransform();
        const JPH::Vec3 p(xf.GetTranslation()), fwd = xf.GetAxisZ();
        AutopilotInput in;
        in.x = p.GetX();
        in.z = p.GetZ();
        const float h = std::hypot(fwd.GetX(), fwd.GetZ());
        in.fx = fwd.GetX() / h;
        in.fz = fwd.GetZ() / h;
        in.forwardY = fwd.GetY();
        in.upDot = xf.GetAxisY().GetY();
        in.forwardSpeed = sim.vehicle().telemetry().forwardSpeed;
        const AutopilotOutput out = driver.update(in, float(sim.fixedStep()));
        sim.setIntent({.throttle = out.throttle, .brake = out.brake, .steer = out.steer, .analog = true});
        sim.tick();
        maxLine = std::max(maxLine, driver.lateralError());
        maxKmh = std::max(maxKmh, sim.vehicle().telemetry().speed * 3.6f);
    }
    const LapTimer& lt = sim.lapTimer();
    std::printf("  builtin, 5 m/s^2 driver: lap %.2f s, sectors %zu, max %.0f km/h, line error max %.2f m\n",
                lt.lastLap(), lt.lastSectors().size(), maxKmh, maxLine);
    CHECK(lt.laps() == 1, "no complete lap (%d)", lt.laps());
    CHECK(lt.lastLap() > 45.0 && lt.lastLap() < 90.0, "lap %.2f s", lt.lastLap());
    CHECK(lt.lastSectors().size() == 3, "%zu sectors", lt.lastSectors().size());
    CHECK(maxLine < 3.0f, "left the line by %.2f m", maxLine);
    CHECK(maxKmh > 85.0f, "max %.0f km/h", maxKmh);
}

// Drift scoring arithmetic: slip x speed x time, zone multiplier, linking, completion, spin, wall hit, area, abort.
void driftScoreLogic() {
    DriftConfig c;
    c.enabled = true;
    DriftZone wall;
    wall.name = "wall";
    wall.multiplier = 2.0f;
    wall.a = {0.0f, 10.0f};
    wall.b = {100.0f, 10.0f};
    wall.reach = 3.0f;
    c.zones.push_back(wall);
    DriftScorer d;
    d.setConfig(c);
    const float dt = 0.01f;
    // Body moving +x at v m/s with the heading turned by deg (slip = deg).
    auto tick = [&](float x, float z, float v, float deg, int wheels = 4, float impact = 0.0f) {
        DriftInput in;
        in.dt = dt;
        in.x = x, in.z = z;
        in.rearX = x - 2.0f, in.rearZ = z;
        in.fx = std::cos(deg / 57.2957795f), in.fz = std::sin(deg / 57.2957795f);
        in.vx = v;
        in.wheelsOnGround = wheels;
        in.impactSpeed = impact;
        return d.update(in);
    };
    CHECK(tick(0, 0, 15.0f, 30.0f) == DriftScorer::Event::Started && d.running(), "run starts");
    CHECK(std::abs(d.slipAngleDeg() - 30.0f) < 0.01f && std::abs(d.speedKmh() - 54.0f) < 0.01f, "slip %.2f speed %.2f",
          double(d.slipAngleDeg()), double(d.speedKmh()));
    for (int k = 1; k < 200; ++k) tick(0, 0, 15.0f, 30.0f);
    const double plain = 0.1 * 30.0 * 54.0 * 2.0; // 2 s at 30 deg, 54 km/h
    CHECK(std::abs(d.score() - plain) < 0.5, "plain %.2f expected %.2f", d.score(), plain);
    for (int k = 0; k < 100; ++k) tick(50, 11.0f, 15.0f, -30.0f); // 1 s in the wall zone, the other way round
    CHECK(d.zone() == 0 && d.multiplier() == 2.0f, "zone %d", d.zone());
    CHECK(std::abs(d.score() - 2.0 * plain) < 0.5 && std::abs(d.zoneTime() - 1.0) < 0.02, "zone score %.2f", d.score());
    for (int k = 0; k < 100; ++k) tick(0, 0, 15.0f, 80.0f); // capped at 70 deg
    CHECK(std::abs(d.score() - 2.0 * plain - 0.1 * 70.0 * 54.0) < 0.5 && d.maxAngleDeg() > 79.9f, "cap %.2f", d.score());
    const double run1 = d.score();
    DriftScorer::Event e = DriftScorer::Event::None;
    int ticks = 0;
    while (e == DriftScorer::Event::None && ticks < 1000) e = tick(0, 0, 15.0f, 2.0f), ++ticks;
    CHECK(e == DriftScorer::Event::Completed && std::abs(ticks * dt - 1.5f) < 0.02f, "completed after %d ticks", ticks);
    CHECK(d.runs() == 1 && d.lastScore() == run1 && d.bestScore() == run1 && !d.running(), "result");
    // Straight driving, slow slides and one wheel on the ground do not start a run.
    for (int k = 0; k < 100; ++k) tick(0, 0, 15.0f, 5.0f), tick(0, 0, 4.0f, 30.0f), tick(0, 0, 15.0f, 30.0f, 1);
    CHECK(!d.running() && d.score() == run1, "no run");
    // Spin and wall hit: failed, 0 points, session best kept.
    for (int k = 0; k < 50; ++k) tick(0, 0, 15.0f, 40.0f);
    CHECK(tick(0, 0, 15.0f, 120.0f) == DriftScorer::Event::Failed && d.lastEnd() == DriftScorer::End::Spin, "spin");
    CHECK(d.lastScore() == 0.0 && d.lastLost() > 0.0 && d.bestScore() == run1 && d.failedRuns() == 1, "spin result");
    for (int k = 0; k < 50; ++k) tick(0, 0, 15.0f, 40.0f);
    CHECK(tick(0, 0, 15.0f, 40.0f, 4, 2.0f) == DriftScorer::Event::Failed && d.lastEnd() == DriftScorer::End::Wall, "wall");
    // A light touch (below wallSpeed) does not end it; a reset aborts without a result.
    for (int k = 0; k < 50; ++k) tick(0, 0, 15.0f, 40.0f, 4, 0.5f);
    CHECK(d.running(), "light touch");
    d.abort();
    CHECK(!d.running() && d.lastEnd() == DriftScorer::End::Aborted && d.runs() == 1 && d.failedRuns() == 2, "abort");
    // Course area: leaving it completes the run.
    c.area = {{-10, -10}, {10, -10}, {10, 10}, {-10, 10}};
    d.setConfig(c);
    CHECK(d.runs() == 0 && d.bestScore() == 0.0, "new config clears the session");
    for (int k = 0; k < 50; ++k) tick(0, 0, 15.0f, 40.0f);
    CHECK(tick(20, 0, 15.0f, 40.0f) == DriftScorer::Event::Completed && d.lastEnd() == DriftScorer::End::LeftArea &&
              d.runs() == 1 && d.lastScore() > 0.0, "left the area");
    CHECK(tick(20, 0, 15.0f, 40.0f) == DriftScorer::Event::None && !d.running(), "outside: no run");
    // Disabled: nothing.
    c.enabled = false;
    d.setConfig(c);
    CHECK(tick(0, 0, 15.0f, 40.0f) == DriftScorer::Event::None && !d.running(), "disabled");
    // Polygon zone, rear point inside only.
    DriftZone box;
    box.polygon = {{-3, -1}, {-1, -1}, {-1, 1}, {-3, 1}};
    CHECK(box.contains(-2, 0) && !box.contains(0, 0) && wall.contains(50, 12.9f) && !wall.contains(50, 13.1f) &&
              wall.contains(-2, 10) && !wall.contains(-4, 10), "contains");
}

void driftConfigParse() {
    auto parse = [](const char* text, DriftConfig& c, std::string& err) {
        json::Value v;
        if (!json::parse(text, v, &err)) return false;
        return parseDriftConfig(v, c, err);
    };
    DriftConfig c;
    std::string err;
    CHECK(parseDriftConfig(json::Value{}, c, err) && !c.enabled, "null: disabled");
    CHECK(parse(R"({"minAngleDeg": 15, "linkSeconds": 2, "area": [[0,0],[10,0],[10,10]],
                    "zones": [{"name": "Outer wall", "multiplier": 2.5, "line": [[0, 0], [10, 0]], "reach": 3},
                              {"name": "Clip", "polygon": [[1,1],[2,1],[2,2]]}]})", c, err),
          "%s", err.c_str());
    CHECK(c.enabled && c.minAngleDeg == 15.0f && c.linkSeconds == 2.0f && c.area.size() == 3 && c.zones.size() == 2, "values");
    CHECK(c.zones[0].name == "Outer wall" && c.zones[0].multiplier == 2.5f && c.zones[0].reach == 3.0f &&
              c.zones[0].polygon.empty() && c.zones[0].b.x == 10.0f, "line zone");
    CHECK(c.zones[1].polygon.size() == 3 && c.zones[1].multiplier == 2.0f, "polygon zone");
    CHECK(parse(R"({"enabled": false})", c, err) && !c.enabled, "enabled false");
    CHECK(!parse(R"({"zones": [{"name": "x", "polygon": [[1,1],[2,1]]}]})", c, err) && err.find("'x'") != std::string::npos,
          "short polygon: %s", err.c_str());
    CHECK(!parse(R"({"zones": [{"line": [[0,0],[1,0]], "multiplier": 0.5}]})", c, err), "multiplier < 1");
    CHECK(!parse(R"({"spinAngleDeg": 5})", c, err), "spin below min angle");
    CHECK(!parse(R"({"area": [[0,0],[1,1]]})", c, err), "short area");
    CHECK(!parse(R"([1, 2])", c, err), "not an object");
}

// The proving ground test bed: scoring on the mini handling track with zones on the constant-radius corner; the
// override switches it; a respawn aborts the run.
void miniTrackDrift() {
    const DriftConfig c = tf::miniTrackDrift();
    const MiniTrackInfo& info = miniTrack();
    CHECK(c.enabled && c.zones.size() == 2 && c.area.size() == 4, "config");
    // Outer zone points lie 2.5 .. 6.2 m outside the center line radius, the apex line on the inner edge. Corners 5
    // (160, -145) and 6 (160, -209), R 32, share the arc center (128, -177).
    CHECK(c.zones[0].polygon.size() > 40, "outer zone points %zu", c.zones[0].polygon.size());
    float rMin = 1e9f, rMax = 0.0f;
    const float acx = 128.0f, acz = -177.0f;
    for (const DriftPoint& p : c.zones[0].polygon) {
        const float r = std::hypot(p.x - acx, p.z - acz);
        rMin = std::min(rMin, r), rMax = std::max(rMax, r);
    }
    std::printf("  outer zone radius %.2f .. %.2f m (center line %.1f m), apex %.2f / %.2f m\n", double(rMin), double(rMax),
                double(info.steadyRadius), double(std::hypot(c.zones[1].a.x - acx, c.zones[1].a.z - acz)),
                double(std::hypot(c.zones[1].b.x - acx, c.zones[1].b.z - acz)));
    CHECK(rMin > info.steadyRadius + 2.0f && rMax < info.steadyRadius + 6.5f, "outer zone radius %.2f..%.2f", double(rMin), double(rMax));
    CHECK(std::abs(std::hypot(c.zones[1].a.x - acx, c.zones[1].a.z - acz) - (info.steadyRadius - info.halfWidth)) < 0.5f,
          "apex on the inner edge");

    Simulation sim(settingsFor(TrackLayout::ProvingGround));
    CHECK(sim.drift().active() && sim.drift().config().zones.size() == 2, "proving ground: active");
    sim.setDriftOverride(false);
    CHECK(!sim.drift().active(), "override off");
    sim.setDriftOverride(std::nullopt);
    CHECK(sim.drift().active(), "world setting");
    // Handbrake turn on the straight before the corner: the builtin vehicle slides, a run starts; R aborts it.
    const std::size_t i0 = [&] {
        for (std::size_t i = 0; i < info.center.size(); ++i)
            if (info.arc[i] > info.steadyArc0 - 140.0f) return i;
        return std::size_t(0);
    }();
    const std::size_t i1 = (i0 + 1) % info.center.size();
    TrackFeature at;
    at.position = JPH::Vec3(info.center[i0].x, miniTrackGround(info.center[i0].x, info.center[i0].z) + 0.03f, info.center[i0].z);
    at.yaw = std::atan2(info.center[i1].x - info.center[i0].x, info.center[i1].z - info.center[i0].z);
    sim.requestPlace(at);
    run(sim, 1.0);
    run(sim, 4.0, {.throttle = 1.0f});
    const float kmh = sim.vehicle().telemetry().speed * 3.6f;
    float maxSlip = 0.0f;
    bool started = false;
    for (int k = 0; k < 120; ++k) {
        run(sim, 1.0 / 120.0, {.steer = -1.0f, .handBrake = 1.0f, .analog = true});
        maxSlip = std::max(maxSlip, std::abs(sim.drift().slipAngleDeg()));
        started = started || sim.drift().running();
    }
    std::printf("  handbrake turn from %.0f km/h: max slip %.1f deg, run %s, score %.0f\n", double(kmh), double(maxSlip),
                started ? "started" : "not started", sim.drift().score());
    CHECK(kmh > 45.0f && maxSlip > 20.0f && started, "slide %.0f km/h %.1f deg", double(kmh), double(maxSlip));
    sim.requestReset();
    run(sim, 0.1);
    CHECK(!sim.drift().running(), "reset aborts");
}
} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"settles_on_flat_ground", settlesOnFlatGround},
        {"throttle_moves_vehicle", throttleMovesVehicle},
        {"brake_reduces_speed", brakeReducesSpeed},
        {"lifted_wheel_loses_load", liftedWheelLosesLoad},
        {"reset_restores_state", resetRestoresState},
        {"frame_schedule_independence", frameScheduleIndependence},
        {"corner_load_balance", cornerLoadBalance},
        {"brake_holds_in_low_range", brakeHoldsInLowRange},
        {"tire_marks_surfaces", tireMarksSurfaces},
        {"auto_reverse", autoReverse},
        {"surface_launch_grip", surfaceLaunchGrip},
        {"skid_mark_profile", skidMarkProfile},
        {"suv_marks_keyboard", suvMarksKeyboard},
        {"keyboard_steer_range", keyboardSteerRange},
        {"launch_marks_proportional", launchMarksProportional},
        {"tire_marks_boundary", tireMarksBoundary},
        {"wheel_spray", wheelSpray},
        {"abs_prevents_lock", absPreventsLock},
        {"diff_locks_climb", diffLocksClimb},
        {"traction_control_mud", tractionControlMud},
        {"driving_aids_logic", drivingAidsLogic},
        {"friction_ellipse", frictionEllipse},
        {"tread_soil_surfaces", treadSoilSurfaces},
        {"body_mud", bodyMud},
        {"lap_timer_logic", lapTimerLogic},
        {"mini_track_lap", miniTrackLap},
        {"asphalt_gravel_surfaces", asphaltGravelSurfaces},
        {"drift_score_logic", driftScoreLogic},
        {"drift_config_parse", driftConfigParse},
        {"mini_track_drift", miniTrackDrift},
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
