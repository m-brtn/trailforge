// Headless tool: scenario tracing and physics step cost measurement.
//   trailforge_probe brake            - braking trace on a flat pad
//   trailforge_probe course [hz]      - proving ground drive, telemetry every 0.25 s
//   trailforge_probe bench            - step cost and stability at 60/120/240 Hz

#include "sim/Simulation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

using namespace tf;

namespace {

void printRow(Simulation& sim, double time) {
    const auto& t = sim.vehicle().telemetry();
    const auto p = sim.currentSnapshot().body.position;
    std::printf("t=%5.2f pos=(%6.1f %5.2f %6.1f) fwd=%6.2f up=%.3f g=%2d rpm=%4.0f thr=%.2f brk=%.2f str=%+.2f |", time,
                p.GetX(), p.GetY(), p.GetZ(), t.forwardSpeed, t.upDot, t.gear, t.engineRPM, t.command.throttle,
                t.command.brake, t.command.steer);
    for (const auto& w : t.wheels)
        std::printf(" %c%5.0fN s%.2f a%4.1f w%5.1f", w.contact ? 'C' : '-', w.load, w.longitudinalSlip, w.lateralSlipDeg,
                    w.angularVelocity);
    std::printf("\n");
}

void brake() {
    SimulationSettings s;
    s.layout = TrackLayout::Flat;
    Simulation sim(s);
    double time = 0.0;
    auto runFor = [&](double secs, DriverIntent in, int every) {
        sim.setIntent(in);
        const int n = int(secs / sim.fixedStep());
        for (int i = 0; i < n; ++i) {
            sim.tick();
            time += sim.fixedStep();
            if (i % every == 0) printRow(sim, time);
        }
    };
    runFor(0.5, {}, 1000);
    runFor(4.0, {.throttle = 1.0f}, 60);
    runFor(4.0, {.brake = 1.0f}, 12);
}

void course(double hz) {
    SimulationSettings s;
    s.fixedStep = 1.0 / hz;
    Simulation sim(s);
    double time = 0.0;
    const int every = int(0.25 * hz);
    // Start -> bumps -> diagonal -> low grip
    sim.setIntent({.throttle = 0.6f});
    for (int i = 0; time < 30.0; ++i) {
        sim.tick();
        time += sim.fixedStep();
        if (i % every == 0) printRow(sim, time);
    }
}

// Steady-state cornering at ~40 km/h: time-averaged sum of wheel loads versus m*g.
// Vertical balance: sum(F_i . y) - m*g = m * a_y, where F_i is the total force of wheel i
// (normal + longitudinal + lateral; all Jolt lambdas are impulses per step, F = lambda / dt).
void corner(TrackLayout layout, float steer, double holdSeconds, int collisionSteps) {
    SimulationSettings s;
    s.layout = layout;
    s.collisionSteps = collisionSteps;
    Simulation sim(s);
    const double dt = sim.fixedStep();
    const float targetSpeed = 40.0f / 3.6f;
    const JPH::BodyID id = sim.vehicle().bodyId();
    auto& bi = sim.world().bodies();
    double time = 0.0;
    double sumLoad = 0.0, sumVert = 0.0, sumAy = 0.0, minLoad = 1e9, maxLoad = 0.0, sumTilt = 0.0;
    int samples = 0;
    JPH::Vec3 prevV = bi.GetLinearVelocity(id);
    const double settle = 6.0;
    while (time < settle + holdSeconds) {
        const auto& t = sim.vehicle().telemetry();
        DriverIntent in;
        // Simple P speed controller: throttle up to 40 km/h, then steering.
        const float err = targetSpeed - t.speed;
        in.throttle = std::clamp(0.3f + 0.4f * err, 0.0f, 1.0f);
        in.steer = time > 2.5 ? steer : 0.0f;
        sim.setIntent(in);
        sim.tick();
        time += dt;
        const JPH::Vec3 v = bi.GetLinearVelocity(id);
        const float ay = float((v.GetY() - prevV.GetY()) / dt);
        prevV = v;
        if (time < settle) continue;
        double load = 0.0, vert = 0.0;
        for (const auto& w : t.wheels) {
            if (!w.contact) continue;
            load += w.load;
            // Jolt impulses are applied to the body along -(-n) = +n, -(-long), -(-lat).
            vert += w.load * w.contactNormal.GetY() + w.longitudinalForce * w.contactLongitudinal.GetY() +
                    w.lateralForce * w.contactLateral.GetY();
        }
        sumLoad += load;
        sumVert += vert;
        sumAy += ay;
        sumTilt += std::acos(std::fmin(1.0f, t.upDot)) * 57.2958;
        minLoad = std::min(minLoad, load);
        maxLoad = std::max(maxLoad, load);
        ++samples;
    }
    const auto& t = sim.vehicle().telemetry();
    const double weight = sim.vehicleConfig().mass * 9.81;
    std::printf("corner layout=%d steer=%.2f collisionSteps=%d: %d samples over %.1f s, speed now %.1f km/h, mean tilt %.1f deg\n",
                int(layout), steer, collisionSteps, samples, holdSeconds, t.speed * 3.6f, sumTilt / samples);
    std::printf("  m*g = %.0f N\n  sum(load) mean = %.0f N (%.1f%% of m*g), min %.0f, max %.0f (single ticks)\n", weight,
                sumLoad / samples, 100.0 * sumLoad / samples / weight, minLoad, maxLoad);
    std::printf("  sum(F·y) mean = %.0f N, m*(g + mean a_y) = %.0f N\n", sumVert / samples,
                sim.vehicleConfig().mass * (9.81 + sumAy / samples));
    for (int i = 0; i < kWheelCount; ++i)
        std::printf("  %s load %6.0f N lat %+6.0f N long %+6.0f N comp %.0f%%%s\n", kWheelNames[i], t.wheels[i].load,
                    t.wheels[i].lateralForce, t.wheels[i].longitudinalForce, 100.0f * t.wheels[i].compressionRatio,
                    t.wheels[i].bumpStop ? " BUMP" : "");
}

// Same input as --demo (throttle 0.35, steering -0.45 after 3 s) on the proving ground: 1 s windows.
void demoLoads() {
    Simulation sim;
    const double dt = sim.fixedStep();
    const JPH::BodyID id = sim.vehicle().bodyId();
    auto& bi = sim.world().bodies();
    const double weight = sim.vehicleConfig().mass * 9.81;
    JPH::Vec3 prevV = bi.GetLinearVelocity(id);
    double sLoad = 0, sVert = 0, sAy = 0, maxLoad = 0, maxBump = 0;
    int n = 0, bumpTicks = 0;
    for (int i = 1; i <= int(20.0 / dt); ++i) {
        const double time = i * dt;
        DriverIntent in;
        in.throttle = 0.35f;
        in.steer = time < 3.0 ? 0.0f : -0.45f;
        sim.setIntent(in);
        sim.tick();
        const auto& t = sim.vehicle().telemetry();
        const JPH::Vec3 v = bi.GetLinearVelocity(id);
        sAy += (v.GetY() - prevV.GetY()) / dt;
        prevV = v;
        double load = 0, vert = 0;
        bool bump = false;
        for (const auto& w : t.wheels) {
            if (!w.contact) continue;
            load += w.load;
            vert += w.load * w.contactNormal.GetY() + w.longitudinalForce * w.contactLongitudinal.GetY() +
                    w.lateralForce * w.contactLateral.GetY();
            bump = bump || w.bumpStop;
        }
        sLoad += load;
        sVert += vert;
        maxLoad = std::max(maxLoad, load);
        if (bump) { ++bumpTicks; maxBump = std::max(maxBump, load); }
        ++n;
        if (i % int(1.0 / dt) == 0) {
            const auto p = sim.currentSnapshot().body.position;
            std::printf("t=%4.1f pos=(%6.1f %5.2f %6.1f) v=%4.1f km/h  mean sum(load)=%6.0f N (%5.1f%%)  max tick %6.0f  "
                        "mean F·y=%6.0f  m(g+a_y)=%6.0f  bumpstop ticks %d (max %0.f)\n",
                        time, p.GetX(), p.GetY(), p.GetZ(), t.speed * 3.6f, sLoad / n, 100.0 * sLoad / n / weight,
                        maxLoad, sVert / n, sim.vehicleConfig().mass * (9.81 + sAy / n), bumpTicks, maxBump);
            sLoad = sVert = sAy = maxLoad = maxBump = 0;
            n = bumpTicks = 0;
        }
    }
}

// 12 degree climb in low range from a standstill; on the plateau the throttle is released and full brake applied (as in the scenario).
void hillBrake() {
    Simulation sim;
    sim.requestTeleport(4);
    for (int i = 0; i < 120; ++i) sim.tick();
    sim.requestToggleLowRange();
    double time = 0.0;
    bool braking = false;
    for (int i = 0; time < 9.0; ++i) {
        const auto& t = sim.vehicle().telemetry();
        const float z = sim.currentSnapshot().body.position.GetZ();
        if (!braking && z > -33.0f) braking = true;
        DriverIntent in;
        in.throttle = braking ? 0.0f : 1.0f;
        in.brake = braking ? 1.0f : 0.0f;
        sim.setIntent(in);
        sim.tick();
        time += sim.fixedStep();
        if (i % 12 == 0) {
            printRow(sim, time);
            std::printf("        clutch=%.2f shifting=%d\n", t.clutchFriction, t.shifting);
        }
        if (braking && t.speed < 0.1f) { printRow(sim, time); break; }
    }
}

void bench() {
    for (double hz : {60.0, 120.0, 240.0}) {
        SimulationSettings s;
        s.fixedStep = 1.0 / hz;
        Simulation sim(s);
        const int n = int(20.0 * hz);
        double total = 0.0, worst = 0.0;
        float maxPen = 0.0f;
        bool finiteAll = true;
        for (int i = 0; i < n; ++i) {
            const double t = i / hz;
            DriverIntent in;
            in.throttle = t < 12.0 ? 0.7f : 0.0f;
            in.steer = std::sin(t * 0.7) * 0.4f;
            in.brake = t > 14.0 ? 0.5f : 0.0f;
            sim.setIntent(in);
            sim.tick();
            total += sim.timing().lastStepMs;
            worst = std::max(worst, sim.timing().lastStepMs);
            const auto p = sim.currentSnapshot().body.position;
            finiteAll = finiteAll && std::isfinite(p.GetX()) && std::isfinite(p.GetY());
            for (const auto& w : sim.vehicle().telemetry().wheels)
                maxPen = std::max(maxPen, w.bumpStop ? 1.0f : 0.0f);
        }
        const auto p = sim.currentSnapshot().body.position;
        std::printf("%5.0f Hz: avg step %.3f ms, worst %.3f ms, CPU per sim-second %.1f ms, final pos (%.2f %.2f %.2f) "
                    "speed %.2f finite=%d bumpstop=%d\n",
                    hz, total / n, worst, total / 20.0, p.GetX(), p.GetY(), p.GetZ(),
                    sim.vehicle().telemetry().speed, finiteAll, maxPen > 0.0f);
    }
}

} // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "bench";
    if (mode == "brake") brake();
    else if (mode == "course") course(argc > 2 ? std::atof(argv[2]) : 120.0);
    else if (mode == "demo") demoLoads();
    else if (mode == "hillbrake") hillBrake();
    else if (mode == "corner") corner(TrackLayout::Flat, argc > 2 ? float(std::atof(argv[2])) : -0.45f, 4.0,
                                         argc > 3 ? std::atoi(argv[3]) : 1);
    else bench();
    return 0;
}
