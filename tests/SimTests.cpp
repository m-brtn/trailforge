// Headless TrailForge physics checks. Usage: trailforge_tests <test_name>
// (CTest registers each test separately). Tolerances are engineering, not bitwise.

#include "sim/FixedStepClock.h"
#include "sim/Simulation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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
            worstOverspin = std::fmax(worstOverspin, w.angularVelocity * sim.vehicleConfig().wheelRadius - t.forwardSpeed);
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
        ++st.surfaceSegments[int(s.surface)];
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
        run(sim, 1.5, DriverIntent{.throttle = 1.0f, .steer = 1.0f});
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
    // Flat ground at 60/120/240 Hz: S brakes to a stop, then engages reverse after the hold time (simulated time,
    // so the same at any tick rate) and drives backwards; W brakes the backward roll, then engages forward.
    for (double hz : {60.0, 120.0, 240.0}) {
        SimulationSettings s = settingsFor(TrackLayout::Flat);
        s.fixedStep = 1.0 / hz;
        Simulation sim(s);
        const double dt = sim.fixedStep(), hold = sim.autoReverse().holdTime;
        run(sim, 0.5);
        run(sim, 3.0, pedals(1, 0));
        const float v0 = sim.vehicle().telemetry().forwardSpeed;
        const SwitchTrace toRev = holdUntilSwitch(sim, pedals(0, 1), 8.0);
        run(sim, 3.0, pedals(0, 1));
        const auto& t = sim.vehicle().telemetry();
        const float vRev = t.forwardSpeed;
        const int gearRev = t.gear;
        const SwitchTrace toFwd = holdUntilSwitch(sim, pedals(1, 0), 8.0);
        run(sim, 3.0, pedals(1, 0));
        const float vFwd = sim.vehicle().telemetry().forwardSpeed;
        std::printf("    %3.0f Hz: v0=%.2f m/s, stop %.3f s -> reverse %.3f s (delay %.3f, v %.2f), after 3 s v=%.2f gear=%d; "
                    "W: stop %.3f s -> drive %.3f s (v %.2f), after 3 s v=%.2f\n",
                    hz, v0, toRev.stopTime, toRev.switchTime, toRev.switchTime - toRev.stopTime, toRev.speedAtSwitch, vRev,
                    gearRev, toFwd.stopTime, toFwd.switchTime, toFwd.speedAtSwitch, vFwd);
        CHECK(v0 > 5.0f, "precondition: forward speed %.2f", v0);
        CHECK(toRev.switchTime > 0.0 && !toRev.switchedWhileMoving, "%.0f Hz: reverse engaged while moving or never", hz);
        CHECK(std::abs(toRev.switchTime - toRev.stopTime - hold) < dt + 1e-6 || toRev.switchTime - toRev.stopTime > hold,
              "%.0f Hz: reverse delay %.3f s vs hold %.3f s", hz, toRev.switchTime - toRev.stopTime, hold);
        CHECK(toRev.switchTime - toRev.stopTime < hold + 0.1, "%.0f Hz: reverse delay %.3f s", hz,
              toRev.switchTime - toRev.stopTime);
        CHECK(vRev < -2.0f && gearRev < 0, "%.0f Hz: not driving backwards: v=%.2f gear=%d", hz, vRev, gearRev);
        CHECK(toFwd.switchTime > 0.0 && !toFwd.switchedWhileMoving, "%.0f Hz: forward engaged while moving or never", hz);
        CHECK(toFwd.switchTime - toFwd.stopTime < hold + 0.1, "%.0f Hz: forward delay %.3f s", hz,
              toFwd.switchTime - toFwd.stopTime);
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

    // Slope: climb the 12 degree hill in low range, S stops the vehicle, engages reverse and it backs down.
    {
        Simulation sim(settingsFor(TrackLayout::ProvingGround));
        sim.requestTeleport(4);
        run(sim, 1.0);
        sim.requestToggleLowRange();
        run(sim, 0.1);
        sim.setIntent(pedals(1, 0));
        for (int i = 0; i < int(12.0 / sim.fixedStep()) && position(sim).GetZ() < -36.0f; ++i) sim.tick();
        const JPH::Vec3 onSlope = position(sim);
        const SwitchTrace tr = holdUntilSwitch(sim, pedals(0, 1), 6.0);
        const float zSwitch = position(sim).GetZ();
        run(sim, 2.0, pedals(0, 1));
        const auto& t = sim.vehicle().telemetry();
        const float pitch = std::asin(sim.currentSnapshot().body.rotation.RotateAxisZ().GetY()) * 57.2958f;
        std::printf("    slope: on slope z=%.2f y=%.2f pitch %.1f deg, reverse after %.3f s (stop %.3f s), 2 s later "
                    "dz=%.2f m v=%.2f gear=%d\n",
                    onSlope.GetZ(), onSlope.GetY(), pitch, tr.switchTime, tr.stopTime, position(sim).GetZ() - zSwitch,
                    t.forwardSpeed, t.gear);
        CHECK(onSlope.GetZ() >= -36.0f && onSlope.GetY() > 0.6f, "precondition: not on the slope (z %.2f y %.2f)",
              onSlope.GetZ(), onSlope.GetY());
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
