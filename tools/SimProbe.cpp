// Headless tool: scenario tracing and physics step cost measurement.
//   trailforge_probe brake            - braking trace on a flat pad
//   trailforge_probe course [hz]      - proving ground drive, telemetry every 0.25 s
//   trailforge_probe bench            - step cost and stability at 60/120/240 Hz
//   trailforge_probe susp [feature throttle low secs steer] - per-wheel suspension travel over a track feature

#include "audio/AudioMixer.h"
#include "audio/VehicleAudioFeed.h"
#include "sim/Simulation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

// Suspension articulation trace: teleport to a feature, drive slowly, print per-wheel travel.
void susp(int feature, float throttle, bool low, double secs, float steer) {
    Simulation sim;
    sim.requestTeleport(feature);
    for (int i = 0; i < 240; ++i) sim.tick();
    if (low) { sim.requestToggleLowRange(); sim.tick(); }
    const JPH::BodyID vid = sim.vehicle().bodyId(), tid = sim.track().bodyId();
    auto& sys = sim.world().system();
    double time = 0.0;
    int bodyContactTicks = 0, ticks = 0;
    float minL[4] = {9, 9, 9, 9}, maxL[4] = {0, 0, 0, 0}, maxDiag = 0.0f;
    std::printf("feature %d '%s'\n", feature, sim.track().features()[feature].name.c_str());
    std::printf("   t     z   roll  pitch body| L(m) comp%% c  hubY_body  per wheel FL FR RL RR | diag\n");
    for (int i = 0; time < secs; ++i) {
        sim.setIntent({.throttle = throttle, .steer = steer});
        sim.tick();
        time += sim.fixedStep();
        const auto& t = sim.vehicle().telemetry();
        const auto& s = sim.currentSnapshot();
        const JPH::RMat44 bodyXf = JPH::RMat44::sRotationTranslation(s.body.rotation, JPH::RVec3(s.body.position));
        const JPH::RMat44 inv = bodyXf.InversedRotationTranslation();
        float hubY[4];
        for (int w = 0; w < 4; ++w) {
            hubY[w] = JPH::Vec3(inv * JPH::RVec3(s.wheels[w].position)).GetY();
            minL[w] = std::min(minL[w], t.wheels[w].suspensionLength);
            maxL[w] = std::max(maxL[w], t.wheels[w].suspensionLength);
        }
        const float diag = (hubY[0] + hubY[3]) - (hubY[1] + hubY[2]);
        maxDiag = std::max(maxDiag, std::abs(diag));
        const bool bc = sys.WereBodiesInContact(vid, tid);
        bodyContactTicks += bc;
        ++ticks;
        if (i % 12 == 0) {
            const JPH::Vec3 fwd = s.body.rotation.RotateAxisZ(), left = s.body.rotation.RotateAxisX();
            std::printf("%5.2f %6.2f %+5.1f %+5.1f %s |", time, s.body.position.GetZ(),
                        std::asin(left.GetY()) * 57.2958f, std::asin(fwd.GetY()) * 57.2958f, bc ? "HIT " : "    ");
            for (int w = 0; w < 4; ++w)
                std::printf(" %.3f %3.0f%% %c %+.3f |", t.wheels[w].suspensionLength,
                            100.0f * t.wheels[w].compressionRatio, t.wheels[w].contact ? 'C' : '-', hubY[w]);
            std::printf(" %+.3f\n", diag);
        }
    }
    std::printf("summary: body-terrain contact %d/%d ticks; max |diag hubY| %.3f m\n", bodyContactTicks, ticks, maxDiag);
    for (int w = 0; w < 4; ++w)
        std::printf("  %s L min %.3f max %.3f (range %.3f of 0.40 travel)\n", kWheelNames[w], minL[w], maxL[w],
                    maxL[w] - minL[w]);
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

// Tire slip and the sound/mark response for standard maneuvers: straight ones on the 12 m surface lanes (concrete,
// dirt, mud, grass), turns on the flat 500 m concrete pad (turning circles would leave a lane).
// The mixer runs with an empty sound bank: no samples are mixed, but the smoothed layer gains are the real ones.
struct SkidRig {
    explicit SkidRig(int lane) : lane(lane) {}
    int lane; // -1 = flat concrete pad
    Simulation sim{[this] {
        SimulationSettings s;
        s.layout = lane < 0 ? TrackLayout::Flat : TrackLayout::SurfaceLanes;
        return s;
    }()};
    SoundBank bank;
    AudioMixer mixer{bank};
    VehicleAudioFeed feed;
    std::vector<float> buf = std::vector<float>(800);

    struct Stats {
        float longSlip = 0, latSlip = 0, ratio = 0, joltSlip = 0, slipDeg = 0, skid = 0, skidRaw = 0, mud = 0,
              mark = 0, maxSpeed = 0;
        double skidTime = 0, dist = 0, time = 0, t30 = -1;
    } st;

    void tick(const DriverIntent& in, bool record) {
        sim.setIntent(in);
        sim.tick();
        feed.onTick(sim);
        AudioFrame f = feed.frame();
        f.listenerPos = f.vehiclePos - f.vehicleForward * 8.0f + AudioVec3{0, 2.6f, 0};
        mixer.setFrame(f);
        for (const AudioEvent& e : feed.events()) mixer.trigger(e);
        feed.clearEvents();
        mixer.render(buf.data(), 400);
        if (!record) return;
        const auto& t = sim.vehicle().telemetry();
        st.maxSpeed = std::max(st.maxSpeed, t.speed);
        st.time += sim.fixedStep();
        st.dist += std::abs(t.speed) * sim.fixedStep();
        if (st.t30 < 0 && t.speed * 3.6f >= 30.0f) st.t30 = st.time;
        for (int i = 0; i < kWheelCount; ++i) {
            const WheelContactSample& c = sim.vehicle().contacts()[std::size_t(i)];
            const WheelTelemetry& w = t.wheels[std::size_t(i)];
            if (!c.contact) continue;
            st.longSlip = std::max(st.longSlip, std::abs(c.longitudinalSlipSpeed));
            st.latSlip = std::max(st.latSlip, std::abs(c.lateralSlipSpeed));
            st.ratio = std::max(st.ratio, f.wheels[std::size_t(i)].slipRatio);
            if (w.contactSpeed > 2.0f) {
                st.joltSlip = std::max(st.joltSlip, w.longitudinalSlip);
                st.slipDeg = std::max(st.slipDeg, w.lateralSlipDeg);
            }
            st.mark = std::max(st.mark, markIntensity(c, sim.tireMarks().settings().referenceLoad,
                                                      sim.tireMarks().treadSoil(std::size_t(i))).slip);
        }
        const TireLayerMix raw = tireLayerMix(f.wheels, f.referenceLoad);
        st.skidRaw = std::max(st.skidRaw, raw.gain[0]);
        st.skid = std::max(st.skid, mixer.stats().skid);
        st.mud = std::max(st.mud, mixer.stats().mud);
        if (mixer.stats().skid > 0.15f) st.skidTime += sim.fixedStep();
    }
    void run(double s, const DriverIntent& in, bool record = true) {
        for (int i = 0; i < int(s * 120.0); ++i) tick(in, record);
    }
    // Hold a speed with a P controller (steering optional).
    void cruise(double s, float kmh, float steer, bool record) {
        for (int i = 0; i < int(s * 120.0); ++i) {
            const float v = sim.vehicle().telemetry().speed * 3.6f;
            DriverIntent in;
            in.throttle = std::clamp(0.25f + 0.08f * (kmh - v), 0.0f, 1.0f);
            in.steer = steer;
            tick(in, record);
        }
    }
    void spawn() {
        if (lane >= 0) sim.requestTeleport(lane);
        else sim.requestReset();
        run(1.0, {}, false);
        st = {};
    }
    void lowRange() {
        sim.requestToggleLowRange();
        run(0.1, {}, false);
    }
    void report(const char* lane, const char* what) {
        std::printf("%-8s %-26s vmax %5.1f km/h | slip long %5.2f lat %5.2f m/s ratio %4.2f jolt %4.2f angle %5.1f deg"
                    " | skid raw %4.2f mixed %4.2f (%.2f s > 0.15) mud %4.2f | mark %4.2f | dist %5.1f m, 0-30 %4.2f s\n",
                    lane, what, st.maxSpeed * 3.6f, st.longSlip, st.latSlip, st.ratio, st.joltSlip, st.slipDeg,
                    st.skidRaw, st.skid, st.skidTime, st.mud, st.mark, st.dist, st.t30);
    }
};

void skid() {
    const char* lanes[] = {"concrete", "dirt", "mud", "grass"};
    for (int lane = 0; lane < 4; ++lane) {
        SkidRig r(lane);
        auto go = [&](const char* what, auto&& fn) {
            r.spawn();
            fn();
            r.report(lanes[lane], what);
        };
        go("launch full throttle", [&] { r.run(5.0, {.throttle = 1.0f}); });
        go("launch half throttle", [&] { r.run(5.0, {.throttle = 0.5f}); });
        go("launch low range full", [&] {
            r.lowRange();
            r.run(3.0, {.throttle = 1.0f});
        });
        go("cruise 40 km/h", [&] {
            r.cruise(6.0, 40.0f, 0.0f, false);
            r.cruise(3.0, 40.0f, 0.0f, true);
        });
        go("full brake from 50 km/h", [&] {
            r.cruise(8.0, 50.0f, 0.0f, false);
            r.run(4.0, {.brake = 1.0f});
        });
        go("handbrake from 40 km/h", [&] {
            r.cruise(7.0, 40.0f, 0.0f, false);
            r.run(2.5, {.handBrake = 1.0f});
        });
    }
    SkidRig r(-1);
    auto go = [&](const char* what, auto&& fn) {
        r.spawn();
        fn();
        r.report("pad", what);
    };
    go("turn 15 km/h steer 1.0", [&] {
        r.cruise(5.0, 15.0f, 0.0f, false);
        r.cruise(4.0, 15.0f, 1.0f, true);
    });
    go("turn 30 km/h steer 0.5", [&] {
        r.cruise(6.0, 30.0f, 0.0f, false);
        r.cruise(4.0, 30.0f, 0.5f, true);
    });
    go("turn 30 km/h steer 1.0", [&] {
        r.cruise(6.0, 30.0f, 0.0f, false);
        r.cruise(4.0, 30.0f, 1.0f, true);
    });
    go("turn 50 km/h steer 0.5", [&] {
        r.cruise(8.0, 50.0f, 0.0f, false);
        r.cruise(4.0, 50.0f, 0.5f, true);
    });
    go("launch while steering 1.0", [&] { r.run(5.0, {.throttle = 1.0f, .steer = 1.0f}); });
    go("power turn low range", [&] {
        r.lowRange();
        r.run(4.0, {.throttle = 1.0f, .steer = 1.0f});
    });
    go("handbrake turn 40 km/h", [&] {
        r.cruise(7.0, 40.0f, 0.0f, false);
        r.run(2.5, {.steer = 0.6f, .handBrake = 1.0f});
    });
}

// Per-tick trace of a full-throttle launch on one lane: wheel slip, Jolt slip ratio and the skid gains.
void launchTrace(int lane, float throttle) {
    SkidRig r{lane};
    r.spawn();
    if (std::getenv("PROBE_LOW_RANGE")) r.lowRange();
    for (int i = 0; i < 4 * 120; ++i) {
        r.tick({.throttle = throttle}, true);
        if (i % 6 != 0 && i > 60 && !std::getenv("PROBE_ALL_TICKS")) continue;
        const auto& t = r.sim.vehicle().telemetry();
        std::printf("t=%4.2f v=%5.2f g=%d rpm=%4.0f thr=%.2f clutch=%.2f |", i / 120.0, t.speed, t.gear, t.engineRPM,
                    t.command.throttle, t.clutchFriction);
        for (int k = 0; k < kWheelCount; ++k) {
            const WheelContactSample& c = r.sim.vehicle().contacts()[std::size_t(k)];
            const WheelAudio& w = r.feed.frame().wheels[std::size_t(k)];
            std::printf(" %c%d vs%5.2f/%5.2f r%4.2f q%4.2f", w.contact ? 'C' : '-', int(w.surface),
                        c.longitudinalSlipSpeed, c.lateralSlipSpeed, w.slipRatio, w.squeal);
        }
        const TireLayerMix raw = tireLayerMix(r.feed.frame().wheels, r.feed.frame().referenceLoad);
        std::printf(" | raw %.2f skid %.2f\n", raw.gain[0], r.mixer.stats().skid);
    }
}

// Drives around the mud & puddles showcase (real proving ground surfaces) with wheelspin and turns, and reports
// every stretch where the skid layer is audible with the surfaces under the wheels.
void showcase() {
    Simulation sim;
    SoundBank bank;
    AudioMixer mixer{bank};
    VehicleAudioFeed feed;
    std::vector<float> buf(800);
    const float steers[] = {0.0f, 0.4f, -0.4f, 1.0f, -1.0f, 0.7f};
    double skidTime = 0.0, total = 0.0;
    std::array<double, kSurfaceCount> skidBySurface{};
    for (int run = 0; run < 6; ++run) {
        sim.requestTeleport(8);
        for (int i = 0; i < 12 * 120; ++i) {
            const double t = i / 120.0;
            DriverIntent in;
            in.throttle = (t > 1.0 && t < 7.0) || t > 9.0 ? 1.0f : 0.0f;
            in.brake = t > 7.0 && t < 8.0 ? 1.0f : 0.0f;
            in.steer = t > 2.0 ? steers[run] * (std::fmod(t, 4.0) < 2.0 ? 1.0f : -0.5f) : 0.0f;
            sim.setIntent(in);
            sim.tick();
            feed.onTick(sim);
            AudioFrame f = feed.frame();
            f.listenerPos = f.vehiclePos - f.vehicleForward * 8.0f + AudioVec3{0, 2.6f, 0};
            mixer.setFrame(f);
            for (const AudioEvent& e : feed.events()) mixer.trigger(e);
            feed.clearEvents();
            mixer.render(buf.data(), 400);
            if (t < 1.0) continue;
            total += 1.0 / 120.0;
            const float skid = mixer.stats().skid;
            if (skid > 0.05f) {
                skidTime += 1.0 / 120.0;
                for (const WheelAudio& w : f.wheels)
                    if (w.contact) skidBySurface[w.surface] += 0.25 / 120.0;
            }
            if (skid > 0.05f && i % 30 == 0) {
                const auto p = sim.currentSnapshot().body.position;
                std::printf("run %d t=%5.2f pos (%6.1f %6.1f) skid %.2f mud %.2f wheels:", run, t, p.GetX(), p.GetZ(),
                            skid, mixer.stats().mud);
                for (const WheelAudio& w : f.wheels)
                    std::printf(" %s%d slip %.1f r %.2f", w.contact ? "" : "air:", int(w.surface), w.slipSpeed,
                                w.slipRatio);
                std::printf("\n");
            }
        }
    }
    std::printf("showcase: skid > 0.05 for %.2f of %.1f s; wheel-seconds by surface while skid audible:", skidTime,
                total);
    for (std::size_t s = 0; s < kSurfaceCount; ++s) std::printf(" %zu:%.2f", s, skidBySurface[s]);
    std::printf("\n");
}

} // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "bench";
    if (mode == "brake") brake();
    else if (mode == "course") course(argc > 2 ? std::atof(argv[2]) : 120.0);
    else if (mode == "demo") demoLoads();
    else if (mode == "hillbrake") hillBrake();
    else if (mode == "susp") susp(argc > 2 ? std::atoi(argv[2]) : 2, argc > 3 ? float(std::atof(argv[3])) : 0.4f,
                                  argc > 4 ? std::atoi(argv[4]) != 0 : true, argc > 5 ? std::atof(argv[5]) : 10.0,
                                  argc > 6 ? float(std::atof(argv[6])) : 0.0f);
    else if (mode == "corner") corner(TrackLayout::Flat, argc > 2 ? float(std::atof(argv[2])) : -0.45f, 4.0,
                                         argc > 3 ? std::atoi(argv[3]) : 1);
    else if (mode == "skid") skid();
    else if (mode == "showcase") showcase();
    else if (mode == "launch") launchTrace(argc > 2 ? std::atoi(argv[2]) : 0, argc > 3 ? float(std::atof(argv[3])) : 1.0f);
    else bench();
    return 0;
}
