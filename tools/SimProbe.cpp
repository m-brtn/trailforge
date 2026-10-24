// Headless tool: scenario tracing and physics step cost measurement.
//   trailforge_probe brake            - braking trace on a flat pad
//   trailforge_probe course [hz]      - proving ground drive, telemetry every 0.25 s
//   trailforge_probe bench            - step cost and stability at 60/120/240 Hz
//   trailforge_probe susp [feature throttle low secs steer] - per-wheel suspension travel over a track feature
//   trailforge_probe surfaces         - launches, braking, 15/20 degree climbs per surface
//   trailforge_probe abs              - 50 km/h stops per surface, straight and with full lock, ABS off / on
//   trailforge_probe tc               - hill starts on the ramps with traction control off / on / off-road
//   trailforge_probe diff             - differential modes with a wheel in the air (pit, diagonal twister)

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
        if (const char* hz = std::getenv("PROBE_HZ")) s.fixedStep = 1.0 / std::atof(hz);
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
                                                      sim.tireMarks().soil(std::size_t(i))).slip);
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
    if (std::getenv("PROBE_NO_STATIC_GRIP") || std::getenv("PROBE_NO_CLUTCH_LIMIT")) {
        // Same path as the Tuning checkboxes: applied between physics steps.
        VehicleTuning t = r.sim.vehicleConfig().tuning;
        if (std::getenv("PROBE_NO_STATIC_GRIP")) t.tireStaticGrip = false;
        if (std::getenv("PROBE_NO_CLUTCH_LIMIT")) t.launchClutchLimit = false;
        r.sim.requestTuning(t);
    }
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

// Effective friction per surface: launches (high and low range) and braking on the flat surface lanes, rolling climbs
// and hill starts on the 15 / 20 degree ramps. Wheelspin = a wheel in contact with |longitudinal slip| > 1 m/s.
void surfaces() {
    const char* names[] = {"concrete", "dirt", "mud", "grass", "rock", "lowgrip"};
    std::printf("effective peak mu (tire curve x surface): long %.2f, lat %.2f x multipliers\n",
                VehicleTuning{}.tirePeakLongFriction, VehicleTuning{}.tirePeakLatFriction);
    {
        SimulationSettings flat;
        flat.layout = TrackLayout::Flat;
        Simulation sim(flat); // the surface table needs the Jolt runtime
        const SurfaceTable& table = sim.surfaces();
        for (std::size_t i = 0; i < kSurfaceCount; ++i) {
            const SurfaceProperties& p = table.get(SurfaceId(i));
            std::printf("  %-28s long %.2f (peak %.2f) lat %.2f (peak %.2f)\n", p.name, p.longitudinalFriction,
                        p.longitudinalFriction * VehicleTuning{}.tirePeakLongFriction, p.lateralFriction,
                        p.lateralFriction * VehicleTuning{}.tirePeakLatFriction);
        }
    }
    struct Launch {
        double v1 = 0, v2 = 0, tTarget = -1, spin = 0;
        float maxSlip = 0;
    };
    auto launch = [](Simulation& sim, bool low, float targetKmh, double secs) {
        Launch r;
        if (low) { sim.requestToggleLowRange(); sim.tick(); }
        double t = 0;
        for (int i = 0; t < secs; ++i) {
            sim.setIntent({.throttle = 1.0f});
            sim.tick();
            t += sim.fixedStep();
            const auto& tel = sim.vehicle().telemetry();
            float slip = 0;
            for (const WheelContactSample& c : sim.vehicle().contacts())
                if (c.contact) slip = std::max(slip, std::abs(c.longitudinalSlipSpeed));
            r.maxSlip = std::max(r.maxSlip, slip);
            if (slip > 1.0f) r.spin += sim.fixedStep();
            if (std::abs(t - 1.0) < 0.5 * sim.fixedStep()) r.v1 = tel.speed;
            if (std::abs(t - 2.0) < 0.5 * sim.fixedStep()) r.v2 = tel.speed;
            if (r.tTarget < 0 && tel.speed * 3.6f >= targetKmh) r.tTarget = t;
        }
        return r;
    };
    for (int lane = 0; lane < 6; ++lane) {
        for (int low = 0; low < 2; ++low) {
            SimulationSettings s;
            s.layout = TrackLayout::SurfaceLanes;
            Simulation sim(s);
            sim.requestTeleport(lane);
            for (int i = 0; i < 120; ++i) sim.tick();
            const float target = low ? 15.0f : 30.0f;
            const Launch r = launch(sim, low != 0, target, low ? 3.0 : 5.0);
            std::printf("launch %-8s %s: v(1s) %5.2f v(2s) %5.2f m/s (a ~%.2f m/s^2), 0-%.0f km/h %5.2f s, max slip "
                        "%5.2f m/s, wheelspin %.2f s\n",
                        names[lane], low ? "low " : "high", r.v1, r.v2, r.v1, target, r.tTarget, r.maxSlip, r.spin);
        }
    }
    for (int lane = 0; lane < 6; ++lane) {
        SimulationSettings s;
        s.layout = TrackLayout::SurfaceLanes;
        Simulation sim(s);
        sim.requestTeleport(lane);
        for (int i = 0; i < 120; ++i) sim.tick();
        for (int i = 0; i < int(10.0 / sim.fixedStep()) && sim.vehicle().telemetry().speed * 3.6f < 50.0f; ++i) {
            sim.setIntent({.throttle = 0.6f});
            sim.tick();
        }
        const float v0 = sim.vehicle().telemetry().speed;
        const float z0 = sim.currentSnapshot().body.position.GetZ();
        double t = 0;
        while (sim.vehicle().telemetry().speed > 0.05f && t < 10.0) {
            sim.setIntent({.brake = 1.0f});
            sim.tick();
            t += sim.fixedStep();
        }
        const float d = sim.currentSnapshot().body.position.GetZ() - z0;
        std::printf("brake  %-8s from %4.1f km/h: %5.2f m in %.2f s (mean %.2f m/s^2)\n", names[lane], v0 * 3.6f, d, t,
                    v0 / t);
    }
    const char* rampNames[] = {"concrete", "dirt", "rock", "grass", "mud", "lowgrip"};
    for (int k = 0; k < 12; ++k) {
        for (int mode = 0; mode < 2; ++mode) {
            SimulationSettings s;
            s.layout = TrackLayout::SurfaceRamps;
            Simulation sim(s);
            sim.requestTeleport(k);
            for (int i = 0; i < 120; ++i) sim.tick();
            sim.requestToggleLowRange();
            sim.tick();
            auto z = [&] { return sim.currentSnapshot().body.position.GetZ(); };
            double t = 0, spin = 0, top = -1, vTop = 0;
            float zStart = z(), slide = 0;
            auto drive = [&](const DriverIntent& in, double secs) {
                for (double e = 0; e < secs; e += sim.fixedStep()) {
                    sim.setIntent(in);
                    sim.tick();
                    t += sim.fixedStep();
                    float slip = 0;
                    for (const WheelContactSample& c : sim.vehicle().contacts())
                        if (c.contact) slip = std::max(slip, std::abs(c.longitudinalSlipSpeed));
                    if (in.throttle > 0.9f && slip > 1.0f) spin += sim.fixedStep();
                    if (top < 0 && z() > 24.0f) { top = t; vTop = sim.vehicle().telemetry().speed; }
                }
            };
            if (mode == 1) {
                // Hill start: creep onto the ramp, hold with the foot brake, then full throttle.
                for (int i = 0; i < int(10.0 / sim.fixedStep()) && z() < 8.0f; ++i) {
                    sim.setIntent({.throttle = 0.4f});
                    sim.tick();
                }
                for (int i = 0; i < int(3.0 / sim.fixedStep()) && sim.vehicle().telemetry().speed > 0.05f; ++i)
                    drive({.brake = 1.0f}, sim.fixedStep() * 0.5);
                const float zHold = z();
                drive({.brake = 1.0f}, 2.0);
                slide = zHold - z();
                zStart = z();
                t = 0;
                top = -1;
            }
            float zMax = zStart;
            for (double e = 0; e < 10.0 && top < 0; e += 0.25) {
                drive({.throttle = 1.0f}, 0.25);
                zMax = std::max(zMax, z());
            }
            std::printf("climb  %-8s %2d deg %s: %s", rampNames[k / 2], k % 2 ? 20 : 15,
                        mode ? "hill start" : "rolling   ", top >= 0 ? "top" : "no ");
            if (top >= 0) std::printf(" in %5.2f s at %4.1f km/h", top, vTop * 3.6);
            else std::printf("                      ");
            std::printf(", max progress %5.2f m, end %5.2f m, wheelspin %.2f s", zMax - zStart, z() - zStart, spin);
            if (mode) std::printf(", brake-hold slide %.2f m", slide);
            const auto pos = sim.currentSnapshot().body.position;
            std::printf(", lateral drift %+.2f m, height %.2f m\n", pos.GetX() - sim.track().features()[k].position.GetX(),
                        pos.GetY());
        }
    }
}

// Rolling low-range climb on one ramp lane of TrackLayout::SurfaceRamps: force balance every 0.25 s.
void climbTrace(int lane) {
    SimulationSettings s;
    s.layout = TrackLayout::SurfaceRamps;
    Simulation sim(s);
    sim.requestTeleport(lane);
    for (int i = 0; i < 120; ++i) sim.tick();
    sim.requestToggleLowRange();
    sim.tick();
    for (int i = 0; i < 12 * 120; ++i) {
        sim.setIntent({.throttle = 1.0f});
        sim.tick();
        if (i % 30 != 0) continue;
        const auto& t = sim.vehicle().telemetry();
        const auto& snap = sim.currentSnapshot();
        const float pitch = std::asin(snap.body.rotation.RotateAxisZ().GetY()) * 57.2958f;
        float load = 0, fl = 0;
        for (const auto& w : t.wheels) {
            if (!w.contact) continue;
            load += w.load;
            fl += w.longitudinalForce;
        }
        std::printf("t=%5.2f z=%6.2f y=%5.2f v=%5.2f pitch=%5.1f | sum load %6.0f N, sum long %+6.0f N, ratio %.3f "
                    "(tan %.3f) |",
                    i / 120.0, snap.body.position.GetZ(), snap.body.position.GetY(), t.forwardSpeed, pitch, load, fl,
                    load > 0 ? fl / load : 0.0f, std::tan(pitch / 57.2958f));
        for (const auto& w : t.wheels)
            std::printf(" %c%5.0f %+5.0f s%5.2f", w.contact ? 'C' : '-', w.load, w.longitudinalForce, w.longitudinalSlip);
        std::printf("\n");
        if (snap.body.position.GetZ() > 24.0f) break;
    }
}

void setAids(Simulation& sim, bool abs, TractionMode tc, int locks) {
    sim.requestAidControls({abs, tc, locks});
    sim.tick();
}

const char* tcName(TractionMode m) { return m == TractionMode::Off ? "off" : m == TractionMode::On ? "on " : "off-road"; }

// ABS: full stops from 50 km/h per surface lane (straight, and with full right lock applied with the brake),
// ABS off / on. Lock time = wheel-seconds with a wheel in contact turning below 20 % of its ground speed.
void absProbe() {
    const char* names[] = {"concrete", "dirt", "mud", "grass", "rock", "lowgrip"};
    for (int lane : {0, 4, 1, 3, 2, 5}) {
        for (int steer = 0; steer < 2; ++steer) {
            for (int abs = 0; abs < 2; ++abs) {
                SimulationSettings s;
                s.layout = TrackLayout::SurfaceLanes;
                Simulation sim(s);
                sim.requestTeleport(lane);
                for (int i = 0; i < 120; ++i) sim.tick();
                setAids(sim, abs != 0, TractionMode::Off, 0);
                for (int i = 0; i < int(12.0 / sim.fixedStep()) && sim.vehicle().telemetry().speed * 3.6f < 50.0f; ++i) {
                    sim.setIntent({.throttle = 0.6f});
                    sim.tick();
                }
                const float v0 = sim.vehicle().telemetry().speed;
                const auto p0 = sim.currentSnapshot().body.position;
                const float yaw0 = std::atan2(sim.currentSnapshot().body.rotation.RotateAxisZ().GetX(),
                                              sim.currentSnapshot().body.rotation.RotateAxisZ().GetZ());
                double t = 0, lock = 0, absTime = 0, frontLock = 0;
                int releases = 0;
                std::array<float, kWheelCount> prevP{1, 1, 1, 1};
                std::array<bool, kWheelCount> releasing{};
                float path = 0;
                while (sim.vehicle().telemetry().speed > 0.05f && t < 15.0) {
                    sim.setIntent({.brake = 1.0f, .steer = steer ? 0.4f : 0.0f});
                    sim.tick();
                    t += sim.fixedStep();
                    const auto& tel = sim.vehicle().telemetry();
                    path += tel.speed * float(sim.fixedStep());
                    bool any = false;
                    for (int i = 0; i < kWheelCount; ++i) {
                        const auto& w = tel.wheels[i];
                        if (w.contact && std::abs(w.groundSpeed) > 2.0f && std::abs(w.rollingSpeed) < 0.2f * std::abs(w.groundSpeed)) {
                            lock += sim.fixedStep();
                            if (i < 2) frontLock += sim.fixedStep();
                        }
                        const bool down = w.absPressure < prevP[i] - 1e-4f;
                        if (down && !releasing[i]) ++releases;
                        releasing[i] = down;
                        prevP[i] = w.absPressure;
                        any |= w.absPressure < 0.999f;
                    }
                    if (any) absTime += sim.fixedStep();
                }
                const auto p1 = sim.currentSnapshot().body.position;
                const float yaw1 = std::atan2(sim.currentSnapshot().body.rotation.RotateAxisZ().GetX(),
                                              sim.currentSnapshot().body.rotation.RotateAxisZ().GetZ());
                std::printf("abs %-8s %s ABS %s: from %4.1f km/h %5.2f m path (dz %5.2f, dx %+5.2f) in %.2f s, heading %+6.1f deg, "
                            "lock %.2f wheel-s (front %.2f), ABS active %.2f s, %d releases (%.1f Hz per wheel)\n",
                            names[lane], steer ? "turn    " : "straight", abs ? "on " : "off", v0 * 3.6f, path,
                            p1.GetZ() - p0.GetZ(), p1.GetX() - p0.GetX(), t, (yaw1 - yaw0) * 57.2958f, lock, frontLock,
                            absTime, releases, absTime > 0 ? releases / (4.0 * absTime) : 0.0);
            }
        }
    }
}

// Traction control: hill starts (creep 8 m up, foot brake to a stop, hold 2 s, full throttle) on the ramps, low range.
void tcProbe() {
    const char* laneNames[] = {"concrete", "dirt", "mud", "grass", "rock", "lowgrip"};
    for (int lane : {0, 1, 3, 2}) {
        for (TractionMode tc : {TractionMode::Off, TractionMode::On, TractionMode::OffRoad}) {
            SimulationSettings s;
            s.layout = TrackLayout::SurfaceLanes;
            Simulation sim(s);
            sim.requestTeleport(lane);
            for (int i = 0; i < 120; ++i) sim.tick();
            setAids(sim, false, tc, 0);
            double t = 0, t30 = -1, spin = 0, active = 0;
            float maxSlip = 0, v2 = 0;
            while (t < 6.0) {
                sim.setIntent({.throttle = 1.0f});
                sim.tick();
                t += sim.fixedStep();
                float slip = 0;
                for (const WheelContactSample& c : sim.vehicle().contacts())
                    if (c.contact) slip = std::max(slip, std::abs(c.longitudinalSlipSpeed));
                maxSlip = std::max(maxSlip, slip);
                if (slip > 1.0f) spin += sim.fixedStep();
                if (sim.vehicle().telemetry().tcActive) active += sim.fixedStep();
                if (std::abs(t - 2.0) < 0.5 * sim.fixedStep()) v2 = sim.vehicle().telemetry().speed;
                if (t30 < 0 && sim.vehicle().telemetry().speed * 3.6f >= 30.0f) t30 = t;
            }
            std::printf("tc  launch %-8s high range TC %-8s: v(2 s) %5.2f m/s, 0-30 km/h %5.2f s, max slip %5.2f m/s, "
                        "wheelspin (>1 m/s) %.2f s, TC active %.2f s\n",
                        laneNames[lane], tcName(tc), v2, t30, maxSlip, spin, active);
        }
    }
    const char* rampNames[] = {"concrete", "dirt", "rock", "grass", "mud", "lowgrip"};
    for (int k : {8, 9, 7, 6, 2}) {
        for (TractionMode tc : {TractionMode::Off, TractionMode::On, TractionMode::OffRoad}) {
            SimulationSettings s;
            s.layout = TrackLayout::SurfaceRamps;
            Simulation sim(s);
            sim.requestTeleport(k);
            for (int i = 0; i < 120; ++i) sim.tick();
            sim.requestToggleLowRange();
            setAids(sim, false, tc, 0);
            auto z = [&] { return sim.currentSnapshot().body.position.GetZ(); };
            for (int i = 0; i < int(10.0 / sim.fixedStep()) && z() < 8.0f; ++i) {
                sim.setIntent({.throttle = 0.4f});
                sim.tick();
            }
            for (int i = 0; i < int(5.0 / sim.fixedStep()); ++i) {
                sim.setIntent({.brake = 1.0f});
                sim.tick();
            }
            const float zStart = z();
            double t = 0, top = -1, spin = 0, tcTime = 0;
            float maxSlip = 0, minFactor = 1;
            while (t < 15.0 && top < 0) {
                sim.setIntent({.throttle = 1.0f});
                sim.tick();
                t += sim.fixedStep();
                const auto& tel = sim.vehicle().telemetry();
                float slip = 0;
                for (const WheelContactSample& c : sim.vehicle().contacts())
                    if (c.contact) slip = std::max(slip, std::abs(c.longitudinalSlipSpeed));
                maxSlip = std::max(maxSlip, slip);
                if (slip > 1.0f) spin += sim.fixedStep();
                if (tel.tcActive) tcTime += sim.fixedStep();
                minFactor = std::min(minFactor, tel.throttleFactor);
                if (z() > 24.0f) top = t;
                if (std::getenv("TF_TC_TRACE") && sim.timing().ticks % 60 == 0 && t < 15.0) {
                    std::printf("  t=%.2f z=%.2f v=%+.2f f=%.2f thr=%.2f rpm=%.0f g=%d |", t, z(), tel.forwardSpeed,
                                tel.throttleFactor, tel.engineThrottle, tel.engineRPM, tel.gear);
                    for (const auto& w : tel.wheels)
                        std::printf(" %c v%+.2f r%+.2f tb%4.0f F%+5.0f", w.contact ? 'C' : '-', w.groundSpeed, w.rollingSpeed,
                                    w.tcBrake, w.longitudinalForce);
                    std::printf("\n");
                }
            }
            std::printf("tc  %-8s %2d deg TC %-8s: %s", rampNames[k / 2], k % 2 ? 20 : 15, tcName(tc), top >= 0 ? "top" : "no ");
            if (top >= 0) std::printf(" in %5.2f s", top);
            else std::printf("  %5.2f m in 15 s", z() - zStart);
            const auto pos = sim.currentSnapshot().body.position;
            std::printf(", max slip %5.2f m/s, wheelspin (>1 m/s) %.2f s, TC active %.2f s, min throttle factor %.2f, "
                        "lateral drift %+.2f m\n",
                        maxSlip, spin, tcTime, minFactor, pos.GetX() - sim.track().features()[k].position.GetX());
        }
    }
}

// Differentials: one wheel without support. Pad with a bottomless pit under the front-left wheel and the
// proving ground diagonal twister, low range, full throttle.
void diffProbe() {
    struct Setup {
        const char* name;
        DiffMode f, r, c;
        int locks;
        TractionMode tc;
    } setups[] = {
        {"all open              ", DiffMode::Open, DiffMode::Open, DiffMode::Open, 0, TractionMode::Off},
        {"default (F open, R/C LSD)", DiffMode::Open, DiffMode::LimitedSlip, DiffMode::LimitedSlip, 0, TractionMode::Off},
        {"all open + rear lock  ", DiffMode::Open, DiffMode::Open, DiffMode::Open, 1, TractionMode::Off},
        {"all open + rear+center", DiffMode::Open, DiffMode::Open, DiffMode::Open, 2, TractionMode::Off},
        {"all locked            ", DiffMode::Open, DiffMode::Open, DiffMode::Open, 3, TractionMode::Off},
        {"all open + TC on      ", DiffMode::Open, DiffMode::Open, DiffMode::Open, 0, TractionMode::On},
        {"all open + TC off-road", DiffMode::Open, DiffMode::Open, DiffMode::Open, 0, TractionMode::OffRoad},
    };
    for (int track = 0; track < 2; ++track) {
        for (const Setup& su : setups) {
            SimulationSettings s;
            s.layout = track == 0 ? TrackLayout::SlopeWithWheelPit : TrackLayout::ProvingGround;
            s.vehicle.tuning.frontDiff = su.f;
            s.vehicle.tuning.rearDiff = su.r;
            s.vehicle.tuning.centerDiff = su.c;
            Simulation sim(s);
            if (track == 1) sim.requestTeleport(2);
            sim.setIntent({.brake = 1.0f});
            for (int i = 0; i < 240; ++i) sim.tick();
            sim.requestToggleLowRange();
            setAids(sim, false, su.tc, su.locks);
            auto z = [&] { return sim.currentSnapshot().body.position.GetZ(); };
            const float z0 = z();
            const double secs = track == 0 ? 8.0 : 20.0;
            const float goal = track == 0 ? 4.0f : 16.0f;
            double t = 0, done = -1;
            float maxAirSpin = 0;
            while (t < secs && done < 0) {
                sim.setIntent({.throttle = 1.0f});
                sim.tick();
                t += sim.fixedStep();
                for (const auto& w : sim.vehicle().telemetry().wheels)
                    if (!w.contact) maxAirSpin = std::max(maxAirSpin, std::abs(w.rollingSpeed));
                if ((track == 0 ? z() - z0 : z()) > goal) done = t;
                if (std::getenv("TF_DIFF_TRACE") && sim.timing().ticks % 12 == 0) printRow(sim, t);
            }
            std::printf("diff %-12s %s: ", track == 0 ? "slope pit" : "twister", su.name);
            if (done >= 0) std::printf("out (%.0f m) in %5.2f s", goal, done);
            else std::printf("stuck: %5.2f m in %.0f s", z() - z0, secs);
            std::printf(", max rim speed of a wheel in the air %5.1f m/s\n", maxAirSpin);
        }
    }
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
    else if (mode == "surfaces") surfaces();
    else if (mode == "abs") absProbe();
    else if (mode == "tc") tcProbe();
    else if (mode == "diff") diffProbe();
    else if (mode == "climb") climbTrace(argc > 2 ? std::atoi(argv[2]) : 9);
    else if (mode == "launch") launchTrace(argc > 2 ? std::atoi(argv[2]) : 0, argc > 3 ? float(std::atof(argv[3])) : 1.0f);
    else bench();
    return 0;
}
