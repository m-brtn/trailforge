// Headless tool: scenario tracing and physics step cost measurement.
//   trailforge_probe brake            - braking trace on a flat pad
//   trailforge_probe course [hz]      - proving ground drive, telemetry every 0.25 s
//   trailforge_probe bench            - step cost and stability at 60/120/240 Hz
//   trailforge_probe susp [feature throttle low secs steer] - per-wheel suspension travel over a track feature
//   trailforge_probe surfaces         - launches, braking, 15/20 degree climbs per surface
//   trailforge_probe abs [ellipse]    - 50 / 100 km/h stops per surface, straight and steered, ABS off / on
//   trailforge_probe brakedetail      - 100 km/h concrete stop, ABS off / on: per axle pressure, torque, grip use, slip
//   trailforge_probe surfacecmp       - asphalt / concrete / gravel: 100-0 stops, max lateral g, 60 km/h coast
//   trailforge_probe ellipse          - cornering and launches with the friction ellipse off / on
//   trailforge_probe tc               - hill starts on the ramps with traction control off / on / off-road
//   trailforge_probe tcmud            - 15 degree mud hill start sweep (60/120/240 Hz x 3 creep points) per TC mode
//   trailforge_probe diff             - differential modes with a wheel in the air (pit, diagonal twister)
//   trailforge_probe layouts          - drivetrain layouts / transfer modes: launches, climbs, the pit slope
//   trailforge_probe squeal           - squeal peak, length and release tail: launches, brakes, slides per surface
//   trailforge_probe keyboard         - keyboard W / S as instant keys vs. the progressive KeyboardPedals
//   trailforge_probe compare          - launches, stops, lift-off, cornering, climbs, articulation (+ EV energy)
//   trailforge_probe minitrack [vehicle.json ...] - scripted laps of the mini handling track (path-following driver):
//                                       lap times, max speed, lateral g; SUV, EV, builtin, rwd_test + the given files
//   trailforge_probe drift [vehicle.json ...] - scripted drift attempts on the mini track's R 32 corner (traction
//                                       control off, entry 45 / 55 / 65 km/h, power or hand brake initiation):
//                                       DriftScorer score, max / mean slip angle; rwd_test, builtin, SUV, EV
//   trailforge_probe tires [brake|minitrack|suncoast|circle|limit|cooldown] - tire sets (SUV, EV): 60-0 stops per
//                                       surface at the air temperature and warm; racing-autopilot laps from cold tires
//                                       (lap times, tread and core temperatures per lap), warm-start laps; 150 s on a
//                                       40 m circle at the limit; limit-driver laps (budget = k x g x current mu) with
//                                       grip use; one slow lap from the middle of the window
//   trailforge_probe bump [all|quick [hold|cruise|coast|full]] | bump trace offset h kmh driver shape
//                                     - straight runs over one bump on the flat pad: 1 m kicker, 2.5 m mound and
//                                       60 degree step, full width and under the left wheels, 0.2 / 0.3 /
//                                       0.4 m at 20 / 30 / 40 km/h (quick: 0.30 / 0.35 / 0.40 m at 30 km/h): CoM rise,
//                                       airtime, roll / pitch, suspension speed, hard stop hits, rollovers; per
//                                       vehicle the suspension numbers Jolt uses (effective mass, rates, ratios)
//   trailforge_probe handling [ramp|step|sine|straight|arb|slope|user30|all] - steer ramp (beta, slip angles,
//                                       understeer gradient), step / sine steer, straight line, anti-roll bar balance,
//                                       cross-slope launch, keyboard turn at 30 km/h (tools/HandlingProbe.cpp)
//   trailforge_probe hill all [hold|cruise|coast|full] | hill H L KMH [driver] [trace] - straight runs over a cosine
//                                       hill (0.5x8, 0.9x13, 1x12, 1x15, 1.5x20 m at 80 / 100 km/h): takeoff vertical
//                                       speed and pitch rate, airtime, landing, rollovers
//   trailforge_probe drop [H] [pitchDeg] - body lifted H (default 1.5 m) and released on the flat pad: rebound share
//   trailforge_probe water [entry|rest|all] - coasting into still water 0.3 / 0.5 / 0.7 m deep at 20 / 40 / 60 km/h,
//                                       straight and at 20 deg: deceleration, pitch, yaw; buoyancy standing in water
//                                       (tools/WaterProbe.cpp)
//   trailforge_probe levelmap [level.json] [out.ppm] - bake a level, print stats, write a shaded top view
//   trailforge_probe levelslope [level.json] [out.ppm] - slope statistics, steepest 32 m tiles, slope map
//   trailforge_probe levelprofile x0 z0 x1 z1 steps [level.json] - ground and water height along a line
// Every mode drives the vehicle of TRAILFORGE_VEHICLE (path to a vehicle.json), by default
// assets/vehicles/trailforge_suv/vehicle.json; TRAILFORGE_VEHICLE=builtin uses the C++ defaults of VehicleConfig.

#include "audio/AudioMixer.h"
#include "audio/VehicleAudioFeed.h"
#include "input/KeyboardPedals.h"
#include "sim/Autopilot.h"
#include "sim/Simulation.h"
#include "track/BuildingKit.h"
#include "track/Level.h"
#include "track/MiniTrack.h"
#include "vehicle/VehicleDefinition.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

using namespace tf;

void handlingProbe(int argc, char** argv); // tools/HandlingProbe.cpp
void waterProbe(int argc, char** argv);    // tools/WaterProbe.cpp

namespace {

// Settings with the probe vehicle (see the header comment).
SimulationSettings probeSettings() {
    static const VehicleConfig vehicle = [] {
        const char* env = std::getenv("TRAILFORGE_VEHICLE");
        std::string path = env ? env : TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_suv/vehicle.json";
        if (path == "builtin") {
            std::fprintf(stderr, "probe vehicle: C++ defaults\n");
            return VehicleConfig{};
        }
        VehicleDefinition def;
        std::vector<std::string> errors;
        if (!loadVehicleDefinition(path, def, errors)) {
            for (const auto& e : errors) std::fprintf(stderr, "%s\n", e.c_str());
            std::exit(2);
        }
        std::fprintf(stderr, "probe vehicle: %s (%s)\n", def.id.c_str(), path.c_str());
        return def.config;
    }();
    SimulationSettings s;
    s.vehicle = vehicle;
    return s;
}

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
    SimulationSettings s = probeSettings();
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
    SimulationSettings s = probeSettings();
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
    SimulationSettings s = probeSettings();
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
    Simulation sim(probeSettings());
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
    Simulation sim(probeSettings());
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
    Simulation sim(probeSettings());
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
        SimulationSettings s = probeSettings();
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

// Only the two squeal sounds, as sines with the lengths of the real files (loop 3.6 s, short squeal 1.686 s) and
// ~-18 dBFS RMS: the mixed output is then the squeal alone, with the short squeal's real length.
SoundBank squealOnlyBank() {
    SoundBank b;
    auto sine = [&](SoundId id, double seconds, double hz, bool loop) {
        SoundData& d = b.sounds[std::size_t(id)];
        d.frames = std::size_t(seconds * 48000.0);
        d.loop = loop;
        d.pcm.resize(d.frames);
        for (std::size_t k = 0; k < d.frames; ++k)
            d.pcm[k] = float(0.18 * std::sqrt(2.0) * std::sin(6.283185307 * hz * double(k) / 48000.0));
    };
    sine(SoundId::TireSkidLoop, 3.6, 1700.0, true);
    sine(SoundId::TireSkidShort, 1.686, 1900.0, false);
    return b;
}

// Tire slip and the sound/mark response for standard maneuvers: straight ones on the 12 m surface lanes (concrete,
// dirt, mud, grass), turns on the flat 500 m concrete pad (turning circles would leave a lane).
// The mixer only has the two squeal sounds (squealOnlyBank): the output is the squeal, the layer gains are the real
// ones.
struct SkidRig {
    explicit SkidRig(int lane) : lane(lane) {}
    int lane; // -1 = flat concrete pad
    Simulation sim{[this] {
        SimulationSettings s = probeSettings();
        s.layout = lane < 0 ? TrackLayout::Flat : TrackLayout::SurfaceLanes;
        if (const char* hz = std::getenv("PROBE_HZ")) s.fixedStep = 1.0 / std::atof(hz);
        return s;
    }()};
    SoundBank bank = squealOnlyBank();
    AudioMixer mixer{bank};
    VehicleAudioFeed feed;
    std::vector<float> buf = std::vector<float>(800);
    float outDb = -120.0f; // RMS of the last tick's output, dBFS

    struct Stats {
        float longSlip = 0, latSlip = 0, ratio = 0, joltSlip = 0, slipDeg = 0, skid = 0, skidRaw = 0, mud = 0,
              mark = 0, maxSpeed = 0;
        double skidTime = 0, dist = 0, time = 0, t30 = -1;
        std::array<double, kSurfaceCount> squealOn{}; // wheel-seconds with squeal > 0.05, by the surface under the wheel
    } st;

    void tick(const DriverIntent& in, bool record) {
        sim.setIntent(in);
        sim.tick();
        feed.onTick(sim);
        AudioFrame f = feed.frame();
        f.listenerPos = f.vehiclePos - f.vehicleForward * 8.0f + AudioVec3{0, 2.6f, 0};
        f.settings.whine = 0.0f; // synthesized, needs no sample: would be the only other output
        mixer.setFrame(f);
        for (const AudioEvent& e : feed.events()) mixer.trigger(e);
        feed.clearEvents();
        mixer.render(buf.data(), 400);
        double power = 0.0;
        for (float x : buf) power += double(x) * double(x);
        outDb = gainToDb(float(std::sqrt(power / double(buf.size()))));
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
            if (f.wheels[std::size_t(i)].squeal > 0.05f) st.squealOn[f.wheels[std::size_t(i)].surface] += sim.fixedStep();
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
        for (std::size_t k = 0; k < kSurfaceCount; ++k)
            if (st.squealOn[k] > 0.0)
                std::printf("         squeal on %s: %.2f wheel-s\n", sim.surfaces().get(SurfaceId(k)).name, st.squealOn[k]);
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

// Squeal length and level for everyday and hard maneuvers on the concrete and dirt lanes and the flat pad: peak
// mixed skid gain, peak squeal output (squealOnlyBank: loop and short squeal, nothing else), time the output is above
// -60 dBFS, the tail after the last tick any wheel slid faster than 2 m/s, and onset chirps (short squeals).
void squealCases() {
    constexpr float kAudibleDb = -60.0f, kSlideSpeed = 2.0f;
    struct Case {
        const char* name;
        int lane; // -1 = flat pad
        bool abs;
        float cruiseKmh;
        double seconds, holdSeconds; // total recorded time; the maneuver input is held for holdSeconds
        DriverIntent in;
    };
    const Case cases[] = {
        {"a gentle launch thr 0.3", 0, false, 0, 4.0, 4.0, {.throttle = 0.3f}},
        {"a launch thr 0.5", 0, false, 0, 4.0, 4.0, {.throttle = 0.5f}},
        {"b launch full throttle", 0, false, 0, 5.0, 5.0, {.throttle = 1.0f}},
        {"c light brake 0.3 50 km/h", 0, false, 50, 5.0, 5.0, {.brake = 0.3f}},
        {"c brake 0.5 50 km/h", 0, false, 50, 5.0, 5.0, {.brake = 0.5f}},
        {"c brake tap 0.25 s 50 km/h", 0, false, 50, 2.5, 0.25, {.brake = 1.0f}},
        {"c brake tap 0.5 s 50 km/h", 0, false, 50, 2.5, 0.5, {.brake = 1.0f}},
        {"d full brake 50 km/h", 0, false, 50, 5.0, 5.0, {.brake = 1.0f}},
        {"d full brake ABS 50 km/h", 0, true, 50, 5.0, 5.0, {.brake = 1.0f}},
        {"e handbrake 40 km/h", 0, false, 40, 3.0, 3.0, {.handBrake = 1.0f}},
        {"e handbrake turn 40 km/h", -1, false, 40, 3.0, 3.0, {.steer = 0.6f, .handBrake = 1.0f}},
        {"e power slide full lock", -1, false, 0, 5.0, 5.0, {.throttle = 1.0f, .steer = 1.0f}},
        {"a gentle launch thr 0.3", 1, false, 0, 4.0, 4.0, {.throttle = 0.3f}},
        {"b launch full throttle", 1, false, 0, 5.0, 5.0, {.throttle = 1.0f}},
        {"c light brake 0.3 50 km/h", 1, false, 50, 5.0, 5.0, {.brake = 0.3f}},
        {"d full brake 50 km/h", 1, false, 50, 5.0, 5.0, {.brake = 1.0f}},
        {"e handbrake 40 km/h", 1, false, 40, 3.0, 3.0, {.handBrake = 1.0f}},
    };
    for (const Case& c : cases) {
        SkidRig r(c.lane);
        r.spawn();
        if (c.abs) {
            r.sim.requestToggleAbs();
            r.run(0.1, {}, false);
        }
        if (c.cruiseKmh > 0) r.cruise(8.0, c.cruiseKmh, 0.0f, false);
        const int chirps0 = r.mixer.stats().skidChirps;
        float peak = 0.0f, maxSlip = 0.0f, peakDb = -120.0f;
        double audible = 0.0, lastAudible = -1.0, lastSlide = -1.0;
        const int n = int(c.seconds * 120.0);
        for (int i = 0; i < n; ++i) {
            const double t = i / 120.0;
            r.tick(t < c.holdSeconds ? c.in : DriverIntent{}, false);
            const float skid = r.mixer.stats().skid;
            peak = std::max(peak, skid);
            peakDb = std::max(peakDb, r.outDb);
            if (r.outDb > kAudibleDb) {
                audible += 1.0 / 120.0;
                lastAudible = t;
            }
            for (const WheelContactSample& s : r.sim.vehicle().contacts()) {
                if (!s.contact) continue;
                maxSlip = std::max(maxSlip, s.slipSpeed());
                if (s.slipSpeed() > kSlideSpeed) lastSlide = t;
            }
            if (std::getenv("PROBE_TRACE") && std::strstr(c.name, std::getenv("PROBE_TRACE")) && i % 3 == 0) {
                const auto& tel = r.sim.vehicle().telemetry();
                std::printf("  t=%4.2f v=%5.2f brk=%.2f |", t, tel.speed, tel.command.brake);
                for (int k = 0; k < kWheelCount; ++k) {
                    const WheelContactSample& s = r.sim.vehicle().contacts()[std::size_t(k)];
                    const WheelAudio& w = r.feed.frame().wheels[std::size_t(k)];
                    std::printf(" vl%6.2f vt%5.2f sl%4.2f q%4.2f", s.longitudinalSlipSpeed, s.lateralSlipSpeed,
                                w.slipRatio, w.squeal);
                }
                std::printf(" | skid %.2f out %6.1f dB\n", skid, r.outDb);
            }
        }
        std::printf("%-8s %-28s max slip %5.2f m/s, last > %.0f m/s at %5.2f s | skid peak %4.2f, out peak %6.1f dBFS,"
                    " > %.0f dBFS for %4.2f s, until %5.2f s (tail after the slide %5.2f s) | chirps %d\n",
                    c.lane < 0 ? "pad" : c.lane == 0 ? "concrete" : "dirt", c.name, maxSlip, kSlideSpeed, lastSlide,
                    peak, peakDb, kAudibleDb, audible, lastAudible,
                    lastAudible < 0 || lastSlide < 0 ? 0.0 : lastAudible - lastSlide,
                    r.mixer.stats().skidChirps - chirps0);
    }
}

// Keyboard driving with instant keys (W / S = pedal 1.0, the behavior before KeyboardPedals) and with the progressive
// keyboard pedals: a held W from a standstill, S taps of 0.15 / 0.25 / 0.4 s and a held S at 50 km/h. Reports the
// wheelspin / lock-up (max slip, time with a wheel sliding > 1 m/s), rubber marks, squeal and the acceleration or stop.
void keyboardPedals() {
    struct Result {
        float maxSlip = 0.0f, mark = 0.0f, skid = 0.0f, maxPedal = 0.0f;
        double slideTime = 0.0, t30 = -1.0, stop = -1.0, dist = 0.0;
    };
    auto drive = [](int lane, bool ramp, bool brake, float cruiseKmh, double hold, double seconds) {
        SkidRig r(lane);
        r.spawn();
        if (cruiseKmh > 0.0f) r.cruise(8.0, cruiseKmh, 0.0f, false);
        KeyboardPedals pedals;
        Result res;
        const float dt = float(r.sim.fixedStep());
        for (int i = 0; i < int(seconds * 120.0); ++i) {
            const double t = i / 120.0;
            const bool key = t < hold;
            pedals.update(key && !brake, key && brake, dt);
            const float pedal = ramp ? (brake ? pedals.brake() : pedals.throttle()) : (key ? 1.0f : 0.0f);
            res.maxPedal = std::max(res.maxPedal, pedal);
            r.tick(brake ? DriverIntent{.brake = pedal} : DriverIntent{.throttle = pedal}, true);
            const auto& tel = r.sim.vehicle().telemetry();
            bool sliding = false;
            for (int k = 0; k < kWheelCount; ++k) {
                const WheelContactSample& c = r.sim.vehicle().contacts()[std::size_t(k)];
                if (!c.contact) continue;
                res.maxSlip = std::max(res.maxSlip, c.slipSpeed());
                sliding = sliding || c.slipSpeed() > 1.0f;
                res.mark = std::max(res.mark, markIntensity(c, r.sim.tireMarks().settings().referenceLoad,
                                                            r.sim.tireMarks().soil(std::size_t(k))).slip);
            }
            if (sliding) res.slideTime += 1.0 / 120.0;
            res.skid = std::max(res.skid, r.mixer.stats().skid);
            if (res.t30 < 0.0 && tel.speed * 3.6f >= 30.0f) res.t30 = t;
            if (res.stop < 0.0 && cruiseKmh > 0.0f && tel.speed < 0.5f) res.stop = t;
            if (res.stop < 0.0) res.dist += std::abs(tel.speed) / 120.0;
        }
        return res;
    };
    auto print = [](const char* lane, const char* what, bool ramp, const Result& r, bool brake) {
        std::printf("%-8s %-22s %-8s pedal max %4.2f | max slip %5.2f m/s, sliding > 1 m/s %4.2f s, mark %4.2f, "
                    "skid %4.2f | ",
                    lane, what, ramp ? "ramp" : "instant", r.maxPedal, r.maxSlip, r.slideTime, r.mark, r.skid);
        if (brake) std::printf("stop %4.2f s / %5.1f m\n", r.stop, r.stop >= 0.0 ? r.dist : -1.0);
        else std::printf("0-30 km/h %4.2f s\n", r.t30);
    };
    for (int lane : {0, -1}) {
        const char* name = lane < 0 ? "pad" : "concrete";
        for (bool ramp : {false, true}) print(name, "hold W 5 s", ramp, drive(lane, ramp, false, 0.0f, 5.0, 5.0), false);
        for (double tap : {0.15, 0.25, 0.4}) {
            char what[32];
            std::snprintf(what, sizeof(what), "tap S %.2f s at 50", tap);
            for (bool ramp : {false, true}) print(name, what, ramp, drive(lane, ramp, true, 50.0f, tap, 2.0), true);
        }
        for (bool ramp : {false, true}) print(name, "hold S at 50", ramp, drive(lane, ramp, true, 50.0f, 6.0, 6.0), true);
    }
}

// Rubber marks of launches on the concrete lane, read from the mark ring: gentle (0.3), keyboard W (ramp), hard
// (0.7) and full-pedal launches in 2H and 4H, and stationary burnouts (brake 0.6 + full throttle: the rear wheels
// spin at ~10 m/s while the car stays put; with brake 1.0 the open differential alternates a slow spin between the
// rear wheels). Reports the time a wheel spins (> 1 m/s) and the peak spinning speed; over the ribbon segments the
// marked length, peak and length-weighted mean intensity (the shader's rubber cover is ~1.1 x this), the length
// darker than 0.5 and the mean width; burnout stamps composited per spot (cover = 1 - prod(1 - 1.1 x slip)).
void launchMarks() {
    struct Case {
        const char* name;
        TransferMode mode;
        float throttle; // < 0: keyboard W through KeyboardPedals
        float brake;
        double seconds;
    };
    const Case cases[] = {
        {"2H gentle 0.3", TransferMode::TwoHigh, 0.3f, 0.0f, 5.0},
        {"2H keyboard W", TransferMode::TwoHigh, -1.0f, 0.0f, 5.0},
        {"2H hard 0.7", TransferMode::TwoHigh, 0.7f, 0.0f, 5.0},
        {"2H full pedal", TransferMode::TwoHigh, 1.0f, 0.0f, 5.0},
        {"4H gentle 0.3", TransferMode::FourHigh, 0.3f, 0.0f, 5.0},
        {"4H keyboard W", TransferMode::FourHigh, -1.0f, 0.0f, 5.0},
        {"4H full pedal", TransferMode::FourHigh, 1.0f, 0.0f, 5.0},
        {"2H burnout 1 s", TransferMode::TwoHigh, 1.0f, 0.6f, 1.0},
        {"2H burnout 2 s", TransferMode::TwoHigh, 1.0f, 0.6f, 2.0},
        {"2H burnout 5 s", TransferMode::TwoHigh, 1.0f, 0.6f, 5.0},
    };
    for (const Case& k : cases) {
        SkidRig r(0);
        r.spawn();
        r.sim.requestTransfer(k.mode);
        r.run(0.2, {}, false);
        r.sim.tireMarks().clear();
        KeyboardPedals pedals;
        double spin = 0.0;
        float peak = 0.0f;
        const JPH::RVec3 start = r.sim.vehicle().bodyTransform().GetTranslation();
        for (int i = 0; i < int(k.seconds * 120.0); ++i) {
            pedals.update(true, false, float(r.sim.fixedStep()));
            r.tick({.throttle = k.throttle < 0.0f ? pedals.throttle() : k.throttle, .brake = k.brake}, true);
            bool spinning = false;
            for (const WheelContactSample& c : r.sim.vehicle().contacts()) {
                if (!c.contact) continue;
                peak = std::max(peak, c.longitudinalSlipSpeed);
                spinning = spinning || c.longitudinalSlipSpeed > 1.0f;
            }
            if (spinning) spin += r.sim.fixedStep();
        }
        const float travel = float((r.sim.vehicle().bodyTransform().GetTranslation() - start).Length());
        const TireMarks& m = r.sim.tireMarks();
        double len = 0.0, dark = 0.0, sumSlip = 0.0, sumWidth = 0.0;
        float maxSlip = 0.0f;
        struct Spot {
            float x, z, keep, width;
            int stamps;
        };
        std::vector<Spot> spots;
        for (std::size_t i = 0; i < m.size(); ++i) {
            const TireMarkSegment& s = m.oldest(i);
            if (markSurface(s) != SurfaceId::Concrete) continue;
            const float slip = 0.5f * (s.slip0 + s.slip1);
            const float w = 2.0f * std::sqrt(s.side1[0] * s.side1[0] + s.side1[1] * s.side1[1] + s.side1[2] * s.side1[2]);
            if (s.u0 == 0.0f && s.u1 == 0.25f) { // burnout stamp (patch length)
                const float x = 0.5f * (s.p0[0] + s.p1[0]), z = 0.5f * (s.p0[2] + s.p1[2]);
                Spot* spot = nullptr;
                for (Spot& q : spots)
                    if (std::hypot(q.x - x, q.z - z) < 0.3f) spot = &q;
                if (!spot) spot = &spots.emplace_back(Spot{x, z, 1.0f, 0.0f, 0});
                spot->keep *= 1.0f - std::min(1.0f, 1.1f * slip);
                spot->width = std::max(spot->width, w);
                ++spot->stamps;
                continue;
            }
            if (slip < 0.02f) continue;
            const float dx = s.p1[0] - s.p0[0], dz = s.p1[2] - s.p0[2];
            const double l = std::sqrt(dx * dx + dz * dz);
            len += l;
            sumSlip += slip * l;
            sumWidth += w * l;
            if (slip > 0.5f) dark += l;
            maxSlip = std::max(maxSlip, slip);
        }
        std::printf("%-15s spin %4.2f s, peak %5.2f m/s, moved %5.1f m | ribbon %5.1f m, max %4.2f, mean %4.2f, "
                    "dark(>0.5) %5.1f m, width %4.2f m",
                    k.name, spin, peak, travel, len, maxSlip, len > 0.0 ? sumSlip / len : 0.0, dark,
                    len > 0.0 ? sumWidth / len : 0.0);
        for (const Spot& q : spots)
            if (q.stamps >= 2) std::printf(" | spot: %d stamps, cover %4.2f, width %4.2f m", q.stamps, 1.0f - q.keep, q.width);
        std::printf("\n");
    }
}

// Drives around the mud & puddles showcase (real proving ground surfaces) with wheelspin and turns, and reports
// every stretch where the skid layer is audible with the surfaces under the wheels.
void showcase() {
    Simulation sim(probeSettings());
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

// Probe lane numbers: 0-6 are the TrackLayout::SurfaceLanes features, 7 (asphalt) and 8 (gravel) the RoadLanes ones.
void useLane(SimulationSettings& s, int lane, int& feature) {
    s.layout = lane >= 7 ? TrackLayout::RoadLanes : TrackLayout::SurfaceLanes;
    feature = lane == 7 ? 0 : lane == 8 ? 2 : lane;
}

// Effective friction per surface: launches (high and low range) and braking on the flat surface lanes, rolling climbs
// and hill starts on the 15 / 20 degree ramps. Wheelspin = a wheel in contact with |longitudinal slip| > 1 m/s.
void surfaces() {
    const char* names[] = {"concrete", "dirt", "mud", "grass", "rock", "lowgrip", "sand", "asphalt", "gravel"};
    const int flatLanes[] = {0, 1, 2, 3, 4, 5, 7, 8};
    std::printf("effective peak mu (tire curve x surface): long %.2f, lat %.2f x multipliers\n",
                VehicleTuning{}.tirePeakLongFriction, VehicleTuning{}.tirePeakLatFriction);
    {
        SimulationSettings flat = probeSettings();
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
    for (int lane : flatLanes) {
        for (int low = 0; low < 2; ++low) {
            SimulationSettings s = probeSettings();
            int feature = 0;
            useLane(s, lane, feature);
            Simulation sim(s);
            sim.requestTeleport(feature);
            for (int i = 0; i < 120; ++i) sim.tick();
            const float target = low ? 15.0f : 30.0f;
            const Launch r = launch(sim, low != 0, target, low ? 3.0 : 5.0);
            std::printf("launch %-8s %s: v(1s) %5.2f v(2s) %5.2f m/s (a ~%.2f m/s^2), 0-%.0f km/h %5.2f s, max slip "
                        "%5.2f m/s, wheelspin %.2f s\n",
                        names[lane], low ? "low " : "high", r.v1, r.v2, r.v1, target, r.tTarget, r.maxSlip, r.spin);
        }
    }
    for (int lane : flatLanes) {
        SimulationSettings s = probeSettings();
        int feature = 0;
        useLane(s, lane, feature);
        Simulation sim(s);
        sim.requestTeleport(feature);
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
            SimulationSettings s = probeSettings();
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

// Drivetrain layouts and transfer modes with the probe vehicle: launches on concrete, mud and low grip, rolling
// climbs on the 20 degree dirt and the 15 degree mud ramps (all in high range except 4L).
void layoutsProbe() {
    struct Variant {
        const char* name;
        DriveLayout layout;
        TransferMode transfer;
    };
    const Variant variants[] = {{"RWD", DriveLayout::Rwd, TransferMode::TwoHigh},
                                {"FWD", DriveLayout::Fwd, TransferMode::TwoHigh},
                                {"AWD", DriveLayout::Awd, TransferMode::TwoHigh},
                                {"on-demand", DriveLayout::OnDemand, TransferMode::TwoHigh},
                                {"part-time 2H", DriveLayout::PartTime, TransferMode::TwoHigh},
                                {"part-time 4H", DriveLayout::PartTime, TransferMode::FourHigh},
                                {"part-time 4L", DriveLayout::PartTime, TransferMode::FourLow}};
    auto make = [](const Variant& v, TrackLayout track) {
        SimulationSettings s = probeSettings();
        s.layout = track;
        s.vehicle.layout = v.layout;
        return s;
    };
    auto engage = [](Simulation& sim, const Variant& v) {
        if (v.layout == DriveLayout::PartTime && v.transfer != TransferMode::TwoHigh) {
            sim.requestTransfer(v.transfer);
            sim.tick();
        }
    };
    const char* lanes[] = {"concrete", "mud", "lowgrip"};
    const int laneIndex[] = {0, 2, 5};
    for (const Variant& v : variants) {
        std::printf("%-13s", v.name);
        for (int l = 0; l < 3; ++l) {
            Simulation sim(make(v, TrackLayout::SurfaceLanes));
            sim.requestTeleport(laneIndex[l]);
            for (int i = 0; i < 120; ++i) sim.tick();
            engage(sim, v);
            double t = 0, t30 = -1, spin = 0, v2 = 0;
            for (int i = 0; t < 5.0; ++i) {
                sim.setIntent({.throttle = 1.0f});
                sim.tick();
                t += sim.fixedStep();
                float slip = 0;
                for (const WheelContactSample& c : sim.vehicle().contacts())
                    if (c.contact) slip = std::max(slip, std::abs(c.longitudinalSlipSpeed));
                if (slip > 1.0f) spin += sim.fixedStep();
                const float kmh = sim.vehicle().telemetry().speed * 3.6f;
                if (std::abs(t - 2.0) < 0.5 * sim.fixedStep()) v2 = sim.vehicle().telemetry().speed;
                if (t30 < 0 && kmh >= 30.0f) t30 = t;
            }
            char t30Text[16];
            if (t30 >= 0) std::snprintf(t30Text, sizeof(t30Text), "%5.2f s", t30);
            else std::snprintf(t30Text, sizeof(t30Text), "  > 5 s");
            std::printf(" | %-8s v(2s) %5.2f m/s, 0-30 %s, spin %4.2f s", lanes[l], v2, t30Text, spin);
        }
        std::printf("\n%-13s", "");
        for (int k : {3, 8}) {
            Simulation sim(make(v, TrackLayout::SurfaceRamps));
            sim.requestTeleport(k);
            for (int i = 0; i < 120; ++i) sim.tick();
            engage(sim, v);
            const float z0 = sim.currentSnapshot().body.position.GetZ();
            float zMax = z0;
            double t = 0, top = -1;
            while (t < 12.0 && top < 0) {
                sim.setIntent({.throttle = 1.0f});
                sim.tick();
                t += sim.fixedStep();
                const float z = sim.currentSnapshot().body.position.GetZ();
                zMax = std::max(zMax, z);
                if (z > 24.0f) top = t;
            }
            std::printf(" | climb %s: %s", k == 3 ? "dirt 20 deg" : "mud 15 deg ", top >= 0 ? "top" : "no ");
            if (top >= 0) std::printf(" in %5.2f s", top);
            else std::printf(", max progress %5.2f m", zMax - z0);
        }
        std::printf("\n");
    }
}

// Rolling low-range climb on one ramp lane of TrackLayout::SurfaceRamps: force balance every 0.25 s.
void climbTrace(int lane) {
    SimulationSettings s = probeSettings();
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

// ABS: full stops from 50 and 100 km/h per surface lane (straight, and with 40 % right steer applied with the
// brake), ABS off / on. Lock time = wheel-seconds with a wheel in contact turning below 20 % of its ground speed.
// Heading after 1 s = steering response under full braking before the vehicle can leave its 12 m lane.
// ellipse: 0/1 friction ellipse (tireFrictionEllipse), -1 = default tuning.
void absProbe(int ellipse) {
    const char* names[] = {"concrete", "dirt", "mud", "grass", "rock", "lowgrip", "sand", "asphalt", "gravel"};
    for (float kmh : {50.0f, 100.0f})
    for (int lane : {7, 0, 4, 1, 8, 6, 3, 2}) {
        for (int steer = 0; steer < 2; ++steer) {
            for (int abs = 0; abs < 2; ++abs) {
                SimulationSettings s = probeSettings();
                int feature = 0;
                useLane(s, lane, feature);
                if (ellipse >= 0) s.vehicle.tuning.tireFrictionEllipse = ellipse != 0;
                Simulation sim(s);
                sim.requestTeleport(feature);
                for (int i = 0; i < 120; ++i) sim.tick();
                setAids(sim, abs != 0, TractionMode::Off, 0);
                for (int i = 0; i < int(60.0 / sim.fixedStep()) && sim.vehicle().telemetry().speed * 3.6f < kmh; ++i) {
                    sim.setIntent({.throttle = 1.0f});
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
                float path = 0, heading1 = 0;
                bool leftLane = false;
                while (sim.vehicle().telemetry().speed > 0.05f && t < 20.0) {
                    if (std::abs(sim.currentSnapshot().body.position.GetX() - p0.GetX()) > 5.0f) {
                        leftLane = true;
                        break;
                    }
                    sim.setIntent({.brake = 1.0f, .steer = steer ? 0.4f : 0.0f});
                    sim.tick();
                    t += sim.fixedStep();
                    if (std::abs(t - 1.0) < 0.5 * sim.fixedStep()) {
                        const auto f = sim.currentSnapshot().body.rotation.RotateAxisZ();
                        heading1 = (std::atan2(f.GetX(), f.GetZ()) - yaw0) * 57.2958f;
                    }
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
                std::printf("abs %-8s %3.0f %s ABS %s: from %5.1f km/h %6.2f m path (dz %6.2f, dx %+6.2f) in %5.2f s, "
                            "heading 1 s %+6.1f deg, end %+6.1f deg, lock %5.2f wheel-s (front %.2f), ABS active %.2f s, "
                            "%d releases (%.1f Hz per wheel)%s\n",
                            names[lane], kmh, steer ? "turn    " : "straight", abs ? "on " : "off", v0 * 3.6f, path,
                            p1.GetZ() - p0.GetZ(), p1.GetX() - p0.GetX(), t, heading1, (yaw1 - yaw0) * 57.2958f, lock,
                            frontLock, absTime, releases, absTime > 0 ? releases / (4.0 * absTime) : 0.0,
                            leftLane ? " LEFT THE LANE (|dx| > 5 m, run cut)" : "");
            }
        }
    }
}

// Where a full stop from 100 km/h on the concrete lane loses distance against the ideal mu_peak x g stop: per axle
// time-averaged ABS pressure, commanded brake torque, |longitudinal force| / (combined mu x load) (use of the grip
// available) and slip, from brake on to 5 km/h. abs: 0 / 1.
void brakeDetail(int abs) {
    SimulationSettings s = probeSettings();
    s.layout = TrackLayout::SurfaceLanes;
    Simulation sim(s);
    sim.requestTeleport(0);
    for (int i = 0; i < 120; ++i) sim.tick();
    setAids(sim, abs != 0, TractionMode::Off, 0);
    for (int i = 0; i < int(60.0 / sim.fixedStep()) && sim.vehicle().telemetry().speed * 3.6f < 100.0f; ++i) {
        sim.setIntent({.throttle = 1.0f});
        sim.tick();
    }
    const float v0 = sim.vehicle().telemetry().speed;
    double t = 0, path = 0, n = 0;
    std::array<double, 2> p{}, torque{}, use{}, slip{}, mu{}, load{}, force{};
    double sumMuMax = 0;
    while (sim.vehicle().telemetry().speed > 0.05f && t < 20.0) {
        sim.setIntent({.brake = 1.0f});
        sim.tick();
        t += sim.fixedStep();
        const auto& tel = sim.vehicle().telemetry();
        path += tel.speed * sim.fixedStep();
        if (tel.speed * 3.6f < 5.0f) continue;
        double fx = 0, avail = 0;
        for (int i = 0; i < kWheelCount; ++i) {
            const auto& w = tel.wheels[i];
            const int a = i < 2 ? 0 : 1;
            p[a] += 0.5 * w.absPressure;
            torque[a] += 0.5 * w.brakeTorque;
            slip[a] += 0.5 * w.longitudinalSlip;
            if (!w.contact) continue;
            mu[a] += 0.5 * w.combinedLongFriction;
            load[a] += w.load;
            force[a] += std::abs(w.longitudinalForce);
            use[a] += 0.5 * std::abs(w.longitudinalForce) / std::max(1.0f, w.combinedLongFriction * w.load);
            fx += std::abs(w.longitudinalForce);
            avail += w.load;
        }
        sumMuMax += fx / std::max(1.0, avail);
        ++n;
    }
    const double g = 9.81, ideal = v0 * v0 / (2.0 * g * 0.95);
    std::printf("brakedetail ABS %s from %.1f km/h: %.2f m in %.2f s (ideal at mu 0.95: %.2f m); mean sum|Fx| / sum(load) %.3f\n",
                abs ? "on " : "off", v0 * 3.6f, path, t, ideal, sumMuMax / n);
    for (int a = 0; a < 2; ++a)
        std::printf("  %s axle: pressure %.3f, brake torque %6.0f N*m/wheel, load %6.0f N, |Fx| %6.0f N, Fx/load %.3f, "
                    "mu(curve x surface) %.3f, Fx/(mu load) %.3f, slip %.3f\n",
                    a ? "rear " : "front", p[a] / n, torque[a] / n, load[a] / n, force[a] / n, force[a] / std::max(1.0, load[a]),
                    mu[a] / n, use[a] / n, slip[a] / n);
}

// Asphalt vs concrete vs gravel for the vehicle of TRAILFORGE_VEHICLE: 100-0 km/h stops ABS off / on on the surface
// lanes, the maximum lateral acceleration (1 s mean) at a fixed 35 % steer while the speed ramps from 30 to 110 km/h
// on the flat pad (its surface given the lane surface's properties), and a 5 s coast from 60 km/h without pedals.
void surfaceCompare() {
    const struct {
        int lane;
        SurfaceId surface;
        const char* name;
    } lanes[] = {{0, SurfaceId::Asphalt, "asphalt"}, {1, SurfaceId::Concrete, "concrete"}, {2, SurfaceId::Gravel, "gravel"}};
    const VehicleConfig cfg = probeSettings().vehicle;
    std::printf("surfacecmp: peak mu long %.2f lat %.2f, ABS default %s\n", cfg.tuning.tirePeakLongFriction,
                cfg.tuning.tirePeakLatFriction, cfg.tuning.aids.absEnabled ? "on" : "off");
    for (const auto& lane : lanes) {
        float stop[2] = {}, muLong = 0, muLat = 0;
        for (int abs = 0; abs < 2; ++abs) {
            SimulationSettings s = probeSettings();
            s.layout = TrackLayout::RoadLanes;
            Simulation sim(s);
            sim.requestTeleport(lane.lane);
            for (int i = 0; i < 120; ++i) sim.tick();
            // Traction control on for the run-up (a two-wheel-drive car spins up on gravel otherwise), off for the stop.
            setAids(sim, abs != 0, TractionMode::On, 0);
            muLong = sim.surfaces().get(lane.surface).longitudinalFriction;
            muLat = sim.surfaces().get(lane.surface).lateralFriction;
            for (int i = 0; i < int(60.0 / sim.fixedStep()) && sim.vehicle().telemetry().speed * 3.6f < 100.0f; ++i) {
                sim.setIntent({.throttle = 1.0f});
                sim.tick();
            }
            setAids(sim, abs != 0, TractionMode::Off, 0);
            const float v0 = sim.vehicle().telemetry().speed;
            if (v0 * 3.6f < 95.0f) std::printf("surfacecmp %s: run-up reached only %.1f km/h\n", lane.name, v0 * 3.6f);
            double t = 0;
            float path = 0;
            while (sim.vehicle().telemetry().speed > 0.05f && t < 20.0) {
                sim.setIntent({.brake = 1.0f});
                sim.tick();
                t += sim.fixedStep();
                path += sim.vehicle().telemetry().speed * float(sim.fixedStep());
            }
            // Distance scaled to exactly 100 km/h (v^2) so the run-up overshoot does not count.
            stop[abs] = path * (100.0f / 3.6f) * (100.0f / 3.6f) / (v0 * v0);
        }
        float maxLat = 0, atKmh = 0;
        {
            SimulationSettings s = probeSettings();
            s.layout = TrackLayout::Flat;
            Simulation sim(s);
            sim.surfaces().get(SurfaceId::Concrete) = sim.surfaces().get(lane.surface);
            const JPH::BodyID id = sim.vehicle().bodyId();
            JPH::Vec3 prevV = sim.world().bodies().GetLinearVelocity(id);
            std::vector<float> window;
            const int n = int(std::lround(1.0 / sim.fixedStep()));
            double sum = 0;
            for (double t = 0; t < 58.0; t += sim.fixedStep()) {
                const float target = float(30.0 + std::max(0.0, t - 4.0) * 1.5) / 3.6f;
                const float err = target - sim.vehicle().telemetry().speed;
                sim.setIntent({.throttle = std::clamp(0.3f + 0.5f * err, 0.0f, 1.0f), .steer = t > 2.0 ? 0.35f : 0.0f});
                sim.tick();
                JPH::Vec3 v = sim.world().bodies().GetLinearVelocity(id);
                JPH::Vec3 acc = (v - prevV) / float(sim.fixedStep());
                prevV = v;
                v.SetY(0.0f);
                acc.SetY(0.0f);
                const float a = v.Length() > 0.5f ? std::abs(v.Cross(acc).GetY()) / v.Length() : 0.0f;
                window.push_back(a);
                sum += a;
                if (int(window.size()) > n) {
                    sum -= window[window.size() - std::size_t(n) - 1];
                }
                if (t > 4.0 && int(window.size()) > n && sum / n > maxLat) {
                    maxLat = float(sum / n);
                    atKmh = sim.vehicle().telemetry().speed * 3.6f;
                }
            }
        }
        float lost = 0, rr = 0;
        {
            SimulationSettings s = probeSettings();
            s.layout = TrackLayout::RoadLanes;
            Simulation sim(s);
            sim.requestTeleport(lane.lane);
            for (int i = 0; i < 120; ++i) sim.tick();
            for (int i = 0; i < int(60.0 / sim.fixedStep()) && sim.vehicle().telemetry().speed * 3.6f < 60.0f; ++i) {
                sim.setIntent({.throttle = 0.7f});
                sim.tick();
            }
            const float v0 = sim.vehicle().telemetry().speed;
            sim.setIntent({});
            for (int i = 0; i < int(5.0 / sim.fixedStep()); ++i) {
                sim.tick();
                if (i == 60) rr = sim.vehicle().telemetry().rollingResistanceForce;
            }
            lost = (v0 - sim.vehicle().telemetry().speed) * 3.6f;
        }
        std::printf("surfacecmp %-8s (mu %.2f / %.2f): 100-0 ABS off %6.2f m, ABS on %6.2f m (%.2f g); max lateral %.2f g at "
                    "%.0f km/h (steer 0.35); coast 60 km/h 5 s: -%.1f km/h, rolling resistance %.0f N\n",
                    lane.name, muLong, muLat, stop[0], stop[1],
                    (100.0f / 3.6f) * (100.0f / 3.6f) / (2.0f * stop[1] * 9.81f), maxLat / 9.81f, atKmh, lost, rr);
    }
}

// Traction control: hill starts (creep 8 m up, foot brake to a stop, hold 2 s, full throttle) on the ramps, low range.
void tcProbe() {
    const char* laneNames[] = {"concrete", "dirt", "mud", "grass", "rock", "lowgrip"};
    for (int lane : {0, 1, 3, 2}) {
        for (TractionMode tc : {TractionMode::Off, TractionMode::On, TractionMode::OffRoad}) {
            SimulationSettings s = probeSettings();
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
            SimulationSettings s = probeSettings();
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

// The 15 degree mud hill start is close to the grip limit, so one run says little: the same start at 60/120/240 Hz
// and three creep points (7, 8, 9 m up the ramp), TC off / on / off-road. Time to the plateau (15 s limit).
void tcMudSweep() {
    for (TractionMode tc : {TractionMode::Off, TractionMode::On, TractionMode::OffRoad}) {
        int tops = 0, runs = 0;
        double sum = 0, worst = 0;
        std::printf("tcmud TC %-8s:", tcName(tc));
        for (double hz : {60.0, 120.0, 240.0}) {
            for (float creepZ : {7.0f, 8.0f, 9.0f}) {
                SimulationSettings s = probeSettings();
                s.layout = TrackLayout::SurfaceRamps;
                s.fixedStep = 1.0 / hz;
                Simulation sim(s);
                sim.requestTeleport(8);
                for (int i = 0; i < int(1.0 / sim.fixedStep()); ++i) sim.tick();
                sim.requestToggleLowRange();
                setAids(sim, false, tc, 0);
                auto z = [&] { return sim.currentSnapshot().body.position.GetZ(); };
                for (int i = 0; i < int(10.0 / sim.fixedStep()) && z() < creepZ; ++i) {
                    sim.setIntent({.throttle = 0.4f});
                    sim.tick();
                }
                for (int i = 0; i < int(5.0 / sim.fixedStep()); ++i) {
                    sim.setIntent({.brake = 1.0f});
                    sim.tick();
                }
                const float z0 = z();
                double t = 0, top = -1;
                while (t < 15.0 && top < 0) {
                    sim.setIntent({.throttle = 1.0f});
                    sim.tick();
                    t += sim.fixedStep();
                    if (z() > 24.0f) top = t;
                    const auto& tel = sim.vehicle().telemetry();
                    if (std::getenv("TF_TC_TRACE") && hz == std::atof(std::getenv("TF_TC_TRACE")) && creepZ == 8.0f &&
                        sim.timing().ticks % int(hz / 4) == 0) {
                        std::printf("\n  t=%.2f z=%.2f v=%+.2f f=%.2f rpm=%.0f g=%d |", t, z(), tel.forwardSpeed,
                                    tel.throttleFactor, tel.engineRPM, tel.gear);
                        for (const auto& w : tel.wheels)
                            std::printf(" %c v%+.2f r%+.2f tb%4.0f F%+5.0f", w.contact ? 'C' : '-', w.groundSpeed,
                                        w.rollingSpeed, w.tcBrake, w.longitudinalForce);
                    }
                }
                ++runs;
                if (top >= 0) {
                    ++tops;
                    sum += top;
                    worst = std::max(worst, top);
                    std::printf(" %.1f", top);
                } else {
                    std::printf(" no(%+.1fm)", z() - z0);
                }
            }
        }
        std::printf(" | top %d/%d, mean %.2f s, worst %.2f s\n", tops, runs, tops ? sum / tops : 0.0, worst);
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
            SimulationSettings s = probeSettings();
            s.layout = track == 0 ? TrackLayout::SlopeWithWheelPit : TrackLayout::ProvingGround;
            s.vehicle.tuning.frontDiff = su.f;
            s.vehicle.tuning.rearDiff = su.r;
            s.vehicle.tuning.centerDiff = su.c;
            if (const char* hz = std::getenv("PROBE_HZ")) s.fixedStep = 1.0 / std::atof(hz);
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
                if (std::getenv("TF_DIFF_TRACE") && sim.timing().ticks % int(0.1 / sim.fixedStep()) == 0) printRow(sim, t);
            }
            std::printf("diff %-12s %s: ", track == 0 ? "slope pit" : "twister", su.name);
            if (done >= 0) std::printf("out (%.0f m) in %5.2f s", goal, done);
            else std::printf("stuck: %5.2f m in %.0f s", z() - z0, secs);
            std::printf(", max rim speed of a wheel in the air %5.1f m/s\n", maxAirSpin);
        }
    }
}

// Friction ellipse off / on: steady cornering on the flat pad (concrete, and the pad set to dirt and mud friction),
// full-throttle launches per lane. Braking distances and locked-wheel steering: `abs 0` / `abs 1`.
void ellipseProbe() {
    struct Mu {
        const char* name;
        float lon, lat, kmh;
    } mus[] = {{"concrete", 0.95f, 0.95f, 40.0f}, {"dirt", 0.70f, 0.68f, 40.0f}, {"mud", 0.35f, 0.32f, 25.0f}};
    for (int ellipse = 0; ellipse < 2; ++ellipse) {
        for (const Mu& mu : mus) {
            for (float steer : {0.3f, 0.6f, 1.0f}) {
                SimulationSettings s = probeSettings();
                s.layout = TrackLayout::Flat;
                s.vehicle.tuning.tireFrictionEllipse = ellipse != 0;
                Simulation sim(s);
                SurfaceProperties& pad = sim.surfaces().get(SurfaceId::Concrete);
                pad.longitudinalFriction = mu.lon;
                pad.lateralFriction = mu.lat;
                const JPH::BodyID id = sim.vehicle().bodyId();
                double t = 0, sumA = 0, sumV = 0, sumSlip = 0, maxA = 0;
                int n = 0;
                JPH::Vec3 prevV = sim.world().bodies().GetLinearVelocity(id);
                while (t < 12.0) {
                    const auto& tel = sim.vehicle().telemetry();
                    const float err = mu.kmh / 3.6f - tel.speed;
                    sim.setIntent({.throttle = std::clamp(0.3f + 0.4f * err, 0.0f, 1.0f), .steer = t > 2.5 ? -steer : 0.0f});
                    sim.tick();
                    t += sim.fixedStep();
                    // Centripetal acceleration of the body: horizontal acceleration perpendicular to the velocity.
                    JPH::Vec3 v = sim.world().bodies().GetLinearVelocity(id);
                    JPH::Vec3 acc = (v - prevV) / float(sim.fixedStep());
                    prevV = v;
                    if (t < 8.0) continue;
                    v.SetY(0.0f);
                    acc.SetY(0.0f);
                    const float a = v.Length() > 0.5f ? std::abs(v.Cross(acc).GetY()) / v.Length() : 0.0f;
                    float slip = 0;
                    for (const auto& w : tel.wheels) slip = std::max(slip, std::abs(w.lateralSlipDeg));
                    sumA += a;
                    sumV += tel.speed;
                    sumSlip += slip;
                    maxA = std::max(maxA, double(a));
                    ++n;
                }
                std::printf("ellipse %s corner %-8s steer %.1f: mean speed %5.1f km/h, lateral accel mean %.2f m/s^2 "
                            "(%.2f g, max %.2f), max wheel slip angle mean %4.1f deg\n",
                            ellipse ? "on " : "off", mu.name, steer, sumV / n * 3.6, sumA / n, sumA / n / 9.81, maxA,
                            sumSlip / n);
            }
        }
        const char* names[] = {"concrete", "dirt", "mud", "grass", "rock", "lowgrip", "sand"};
        for (int lane : {0, 1, 6, 3, 2}) {
            SimulationSettings s = probeSettings();
            s.layout = TrackLayout::SurfaceLanes;
            s.vehicle.tuning.tireFrictionEllipse = ellipse != 0;
            Simulation sim(s);
            sim.requestTeleport(lane);
            for (int i = 0; i < 120; ++i) sim.tick();
            double t = 0, spin = 0;
            float maxSlip = 0, v1 = 0, v2 = 0;
            while (t < 3.0) {
                sim.setIntent({.throttle = 1.0f});
                sim.tick();
                t += sim.fixedStep();
                float slip = 0;
                for (const WheelContactSample& c : sim.vehicle().contacts())
                    if (c.contact) slip = std::max(slip, std::abs(c.longitudinalSlipSpeed));
                maxSlip = std::max(maxSlip, slip);
                if (slip > 1.0f) spin += sim.fixedStep();
                if (std::abs(t - 1.0) < 0.5 * sim.fixedStep()) v1 = sim.vehicle().telemetry().speed;
                if (std::abs(t - 2.0) < 0.5 * sim.fixedStep()) v2 = sim.vehicle().telemetry().speed;
            }
            const auto p = sim.currentSnapshot().body.position;
            std::printf("ellipse %s launch %-8s: v(1 s) %5.2f m/s, v(2 s) %5.2f m/s, max slip %5.2f m/s, wheelspin "
                        "(>1 m/s) %.2f s, lateral drift %+.2f m\n",
                        ellipse ? "on " : "off", names[lane], v1, v2, maxSlip, spin,
                        p.GetX() - sim.track().features()[lane].position.GetX());
        }
    }
}

} // namespace

// Vehicle comparison (run once per TRAILFORGE_VEHICLE): launches, stops, cornering, climbs, articulation, energy.
// The combustion SUV runs its default 2H for the flat tests and 4H / 4L on the ramps; an electric vehicle runs the
// NORMAL drive mode on the flat and NORMAL / OFF-ROAD on the ramps.
void compareProbe() {
    const bool electric = probeSettings().vehicle.isElectric();
    auto prep = [&](TrackLayout track, int feature) {
        SimulationSettings s = probeSettings();
        s.layout = track;
        auto sim = std::make_unique<Simulation>(s);
        if (feature >= 0) sim->requestTeleport(feature);
        for (int i = 0; i < 120; ++i) sim->tick();
        return sim;
    };
    std::printf("compare vehicle %s, %s, %.0f kg\n", probeSettings().vehicle.id.c_str(),
                driveLayoutName(probeSettings().vehicle.layout), probeSettings().vehicle.mass);
    // Launch on the concrete lane, full pedal.
    {
        auto sim = prep(TrackLayout::SurfaceLanes, 0);
        double t = 0, t30 = -1, t60 = -1, t100 = -1;
        float peakKW = 0.0f;
        const double e0 = sim->vehicle().telemetry().ev.energyOutKWh;
        while (t < 30.0 && t100 < 0) {
            sim->setIntent({.throttle = 1.0f});
            sim->tick();
            t += sim->fixedStep();
            const float kmh = sim->vehicle().telemetry().forwardSpeed * 3.6f;
            peakKW = std::max(peakKW, sim->vehicle().telemetry().ev.motorPowerKW);
            if (t30 < 0 && kmh >= 30.0f) t30 = t;
            if (t60 < 0 && kmh >= 60.0f) t60 = t;
            if (t100 < 0 && kmh >= 100.0f) t100 = t;
        }
        std::printf("compare launch concrete: 0-30 %.2f s, 0-60 %.2f s, 0-100 %.2f s", t30, t60, t100);
        if (electric)
            std::printf(", peak motor power %.0f kW, battery energy %.3f kWh, SoC %.4f",
                        peakKW, sim->vehicle().telemetry().ev.energyOutKWh - e0, sim->vehicle().telemetry().ev.soc);
        std::printf("\n");
    }
    // Stops from 50 and 100 km/h on concrete, ABS off / on.
    for (float kmh : {50.0f, 100.0f}) {
        for (int abs = 0; abs < 2; ++abs) {
            auto sim = prep(TrackLayout::SurfaceLanes, 0);
            setAids(*sim, abs != 0, TractionMode::Off, 0);
            for (int i = 0; i < 6000 && sim->vehicle().telemetry().forwardSpeed * 3.6f < kmh; ++i) {
                sim->setIntent({.throttle = 1.0f});
                sim->tick();
            }
            const float v0 = sim->vehicle().telemetry().forwardSpeed;
            const float z0 = sim->currentSnapshot().body.position.GetZ();
            const double in0 = sim->vehicle().telemetry().ev.energyInKWh;
            double t = 0, lock = 0, regenJ = 0;
            while (sim->vehicle().telemetry().speed > 0.05f && t < 20.0) {
                sim->setIntent({.brake = 1.0f});
                sim->tick();
                t += sim->fixedStep();
                const auto& tel = sim->vehicle().telemetry();
                for (const auto& w : tel.wheels)
                    if (w.contact && std::abs(w.groundSpeed) > 2.0f && std::abs(w.rollingSpeed) < 0.2f * std::abs(w.groundSpeed))
                        lock += sim->fixedStep();
                regenJ += -std::min(tel.ev.motorPowerKW, 0.0f) * 1000.0 * sim->fixedStep();
            }
            // Distance scaled to exactly the nominal speed (d ~ v^2) so both vehicles compare at the same speed.
            const float d = sim->currentSnapshot().body.position.GetZ() - z0;
            const float scale = (kmh / 3.6f) * (kmh / 3.6f) / (v0 * v0);
            std::printf("compare brake %3.0f-0 ABS %s: %.2f m (raw %.2f m from %.1f km/h) in %.2f s, lock %.2f wheel-s",
                        kmh, abs ? "on " : "off", d * scale, d, v0 * 3.6f, t, lock);
            if (electric)
                std::printf(", regen shaft energy %.0f kJ (%.0f %% of the kinetic energy), battery +%.4f kWh", regenJ * 1e-3,
                            100.0 * regenJ / (0.5 * sim->vehicleConfig().mass * v0 * v0),
                            sim->vehicle().telemetry().ev.energyInKWh - in0);
            std::printf("\n");
        }
    }
    // Lift-off from 80 km/h with both pedals released (after the 0.17 s pedal release ramp).
    {
        auto sim = prep(TrackLayout::SurfaceLanes, 0);
        for (int i = 0; i < 6000 && sim->vehicle().telemetry().forwardSpeed * 3.6f < 80.0f; ++i) {
            sim->setIntent({.throttle = 1.0f});
            sim->tick();
        }
        float v0 = 0.0f;
        for (int i = 0; i < int(3.0 / sim->fixedStep()); ++i) {
            sim->setIntent({});
            sim->tick();
            if (i == int(0.5 / sim->fixedStep())) v0 = sim->vehicle().telemetry().forwardSpeed;
        }
        const float v1 = sim->vehicle().telemetry().forwardSpeed;
        std::printf("compare lift-off 80 km/h: mean decel %.2f m/s^2 from 0.5 to 3 s after the release (%.1f -> %.1f "
                    "km/h)\n", (v0 - v1) / 2.5f, v0 * 3.6f, v1 * 3.6f);
    }
    // Steady cornering on the flat pad: speed held at 40 km/h, steer 0.45 right; lateral acceleration and body roll.
    for (float steer : {0.25f, 0.45f}) {
        SimulationSettings s = probeSettings();
        s.layout = TrackLayout::Flat;
        Simulation sim(s);
        const double dt = sim.fixedStep();
        double t = 0, sumAy = 0, sumRoll = 0;
        int n = 0;
        while (t < 10.0) {
            const auto& tel = sim.vehicle().telemetry();
            const float err = 40.0f / 3.6f - tel.speed;
            sim.setIntent({.throttle = std::clamp(0.3f + 0.4f * err, 0.0f, 1.0f), .steer = t > 2.5 ? steer : 0.0f});
            sim.tick();
            t += dt;
            if (t < 7.0) continue;
            const JPH::Vec3 w = sim.world().bodies().GetAngularVelocity(sim.vehicle().bodyId());
            sumAy += std::abs(w.GetY()) * sim.vehicle().telemetry().speed;
            const JPH::Vec3 left = sim.currentSnapshot().body.rotation.RotateAxisX();
            sumRoll += std::abs(std::asin(left.GetY())) * 57.2958;
            ++n;
        }
        std::printf("compare corner 40 km/h steer %.2f: lateral accel %.2f m/s^2, roll %.2f deg, speed %.1f km/h\n", steer,
                    sumAy / n, sumRoll / n, sim.vehicle().telemetry().speed * 3.6f);
    }
    // Climbs from 6 m before the ramp, full pedal and half pedal, TC off / off-road.
    struct Ramp {
        const char* name;
        int feature;
    };
    for (Ramp r : {Ramp{"dirt 20 deg", 3}, Ramp{"mud 15 deg", 8}}) {
        for (int mode = 0; mode < 2; ++mode) {
            for (float pedal : {1.0f, 0.5f}) {
                for (TractionMode tc : {TractionMode::Off, TractionMode::OffRoad}) {
                    auto sim = prep(TrackLayout::SurfaceRamps, r.feature);
                    if (electric) {
                        if (mode == 1) sim->requestCycleTransfer();
                    } else {
                        sim->requestTransfer(mode == 0 ? TransferMode::FourHigh : TransferMode::FourLow);
                    }
                    setAids(*sim, false, tc, 0);
                    const float z0 = sim->currentSnapshot().body.position.GetZ();
                    float zMax = z0;
                    double t = 0, top = -1;
                    while (t < 15.0 && top < 0) {
                        sim->setIntent({.throttle = pedal});
                        sim->tick();
                        t += sim->fixedStep();
                        const float z = sim->currentSnapshot().body.position.GetZ();
                        zMax = std::max(zMax, z);
                        if (z > 24.0f) top = t;
                    }
                    const char* modeName = electric ? (mode == 0 ? "NORMAL  " : "OFF-ROAD") : (mode == 0 ? "4H      " : "4L      ");
                    std::printf("compare climb %-11s %s pedal %.1f TC %-8s: ", r.name, modeName, pedal, tcName(tc));
                    if (top >= 0) std::printf("top in %.2f s\n", top);
                    else std::printf("no, max progress %.2f m\n", zMax - z0);
                }
            }
        }
    }
    // Articulation: the diagonal twister of the proving ground at a crawl.
    {
        Simulation sim(probeSettings());
        sim.requestTeleport(2);
        for (int i = 0; i < 240; ++i) sim.tick();
        float minL[4] = {9, 9, 9, 9}, maxL[4] = {0, 0, 0, 0};
        double airborne = 0, t = 0;
        while (t < 8.0) {
            sim.setIntent({.throttle = 0.4f});
            sim.tick();
            t += sim.fixedStep();
            const auto& tel = sim.vehicle().telemetry();
            for (int w = 0; w < 4; ++w) {
                minL[w] = std::min(minL[w], tel.wheels[w].suspensionLength);
                maxL[w] = std::max(maxL[w], tel.wheels[w].suspensionLength);
                if (!tel.wheels[w].contact) airborne += sim.fixedStep();
            }
        }
        std::printf("compare susp '%s': travel used FL %.3f FR %.3f RL %.3f RR %.3f m, wheel-s off the ground %.2f\n",
                    sim.track().features()[2].name.c_str(), maxL[0] - minL[0], maxL[1] - minL[1], maxL[2] - minL[2],
                    maxL[3] - minL[3], airborne);
    }
    // Energy at a steady 50 and 90 km/h on the concrete lane (electric only).
    if (electric) {
        for (float kmh : {50.0f, 90.0f}) {
            auto sim = prep(TrackLayout::SurfaceLanes, 0);
            double t = 0, e0 = 0, d0 = 0, d = 0;
            bool measuring = false;
            while (t < 40.0) {
                const auto& tel = sim->vehicle().telemetry();
                const float err = kmh / 3.6f - tel.forwardSpeed;
                sim->setIntent({.throttle = std::clamp(0.2f + 0.3f * err, 0.0f, 1.0f)});
                sim->tick();
                t += sim->fixedStep();
                d += tel.forwardSpeed * sim->fixedStep();
                if (!measuring && t > 20.0) {
                    measuring = true;
                    e0 = sim->vehicle().telemetry().ev.energyOutKWh - sim->vehicle().telemetry().ev.energyInKWh;
                    d0 = d;
                }
            }
            const double e = sim->vehicle().telemetry().ev.energyOutKWh - sim->vehicle().telemetry().ev.energyInKWh - e0;
            std::printf("compare energy steady %.0f km/h: %.1f kWh/100 km (drag, wheel angular damping, aux)\n", kmh,
                        e / ((d - d0) / 1000.0) * 100.0);
        }
    }
}

// Bakes a level and writes a shaded top view (PPM, 2 m per pixel): surfaces, water, roads, trees, POIs.
void levelMap(const char* path, const char* out) {
    std::string err;
    const auto level = Level::load(path, &err);
    if (!level) {
        std::printf("levelmap: %s\n", err.c_str());
        return;
    }
    const LevelBakeStats& st = level->bakeStats();
    std::printf("levelmap %s: bake %.0f ms, %d x %d samples, height %.1f .. %.1f m, %zu trees, %zu rocks, %zu prop "
                "triangles, %zu roads, %zu rivers\n",
                level->id().c_str(), st.totalMs, st.samples, st.samples, st.minHeight, st.maxHeight, st.trees, st.rocks,
                st.propTriangles, level->roads().size(), level->rivers().size());
    int surfaceCount[kSurfaceCount] = {};
    for (SurfaceId s : level->cellSurfaces()) ++surfaceCount[std::size_t(s)];
    const quench::JoltRuntime runtime; // SurfaceTable creates Jolt materials
    const SurfaceTable table;
    for (std::size_t i = 0; i < kSurfaceCount; ++i)
        std::printf("levelmap surface %-28s %5.1f %%\n", table.get(SurfaceId(i)).name,
                    100.0 * surfaceCount[i] / double(level->cellSurfaces().size()));
    for (const LevelRoad& r : level->roads()) {
        float maxGrade = 0.0f;
        int fords = 0;
        for (std::size_t i = 1; i < r.samples.size(); ++i) {
            const float ds = r.samples[i].s - r.samples[i - 1].s;
            if (ds > 0.1f) maxGrade = std::max(maxGrade, std::abs(r.samples[i].y - r.samples[i - 1].y) / ds);
            fords += r.samples[i].ford && !r.samples[i - 1].ford;
        }
        std::printf("levelmap road %-18s %6.0f m, max grade %4.1f %% (limit %4.1f %%), %d fords\n", r.name.c_str(),
                    r.samples.back().s, 100.0f * maxGrade, 100.0f * r.maxGrade, fords);
    }
    const int W = (level->samples() - 1) / 2;
    std::vector<unsigned char> img(std::size_t(W) * W * 3);
    auto put = [&](int x, int y, float r, float g, float b) {
        if (x < 0 || y < 0 || x >= W || y >= W) return;
        unsigned char* p = &img[(std::size_t(y) * W + x) * 3];
        p[0] = (unsigned char)std::clamp(r * 255.0f, 0.0f, 255.0f);
        p[1] = (unsigned char)std::clamp(g * 255.0f, 0.0f, 255.0f);
        p[2] = (unsigned char)std::clamp(b * 255.0f, 0.0f, 255.0f);
    };
    const float o = level->origin();
    for (int py = 0; py < W; ++py)
        for (int px = 0; px < W; ++px) {
            const float x = o + 2.0f * px + 1.0f, z = o + 2.0f * py + 1.0f;
            const float h = level->heightAt(x, z);
            const float hx = level->heightAt(x + 1.0f, z) - level->heightAt(x - 1.0f, z);
            const float hz = level->heightAt(x, z + 1.0f) - level->heightAt(x, z - 1.0f);
            const float shade = std::clamp(0.75f + 0.35f * (-hx + hz) * 0.5f, 0.35f, 1.25f);
            const SurfaceProperties& sp = table.get(level->surfaceAt(x, z));
            float c[3] = {sp.color[0], sp.color[1], sp.color[2]};
            if (auto w = level->waterAt(x, z)) {
                const float depth = std::clamp((*w - h) / 6.0f, 0.0f, 1.0f);
                c[0] = 0.15f - 0.1f * depth;
                c[1] = 0.45f - 0.2f * depth;
                c[2] = 0.65f - 0.15f * depth;
            }
            const float tint = 0.85f + 0.25f * std::clamp(h / 300.0f, 0.0f, 1.0f);
            put(px, py, c[0] * shade * tint, c[1] * shade * tint, c[2] * shade * tint);
        }
    for (const LevelTree& t : level->trees()) {
        const int px = int((t.x - o) / 2.0f), py = int((t.z - o) / 2.0f);
        put(px, py, 0.05f, 0.18f, 0.06f);
    }
    for (const LevelBridge& b : level->bridges())
        for (int k = 0; k <= 100; ++k) {
            const float t = k / 100.0f;
            put(int((b.x0 + (b.x1 - b.x0) * t - o) / 2.0f), int((b.z0 + (b.z1 - b.z0) * t - o) / 2.0f), 0.9f, 0.6f, 0.2f);
        }
    for (const LevelPoi& p : level->pois()) {
        const int px = int((p.x - o) / 2.0f), py = int((p.z - o) / 2.0f);
        for (int dy = -5; dy <= 5; ++dy)
            for (int dx = -5; dx <= 5; ++dx)
                if (dx * dx + dy * dy <= 25) put(px + dx, py + dy, dx * dx + dy * dy > 12 ? 1.0f : 0.55f, 0.1f, 0.1f);
    }
    // Kit buildings: footprints on the map, and per type the instance count, render LOD and collision triangles.
    std::vector<int> uses(level->buildingTypes().size(), 0);
    for (const LevelBuilding& b : level->buildings()) {
        ++uses[b.type];
        const LevelBuildingType& t = level->buildingTypes()[b.type];
        if (!buildingIsStructure(t.spec.kind)) continue;
        const float c = std::cos(b.yaw), s = std::sin(b.yaw);
        for (float u = -t.footprintX; u <= t.footprintX; u += 0.5f)
            for (float v = -t.footprintZ; v <= t.footprintZ; v += 0.5f)
                put(int((b.x + c * u + s * v - o) / 2.0f), int((b.z - s * u + c * v - o) / 2.0f), 0.55f, 0.3f, 0.2f);
    }
    for (const LevelBuilding& b : level->buildings())
        std::printf("levelmap place %-12s %8.1f %6.1f %8.1f yaw %6.1f deg\n", buildingKindName(level->buildingTypes()[b.type].spec.kind),
                    b.x, b.y, b.z, b.yaw * 57.29578f);
    std::size_t drawn[3] = {}, collision = 0;
    for (std::size_t k = 0; k < uses.size(); ++k) {
        const BuildingSpec& sp = level->buildingTypes()[k].spec;
        const BuildingGeometry g = buildBuilding(sp, true);
        std::size_t tris[3] = {};
        for (std::size_t l = 0; l < g.model.lods.size() && l < 3; ++l) tris[l] = g.model.lods[l].triangles();
        for (int l = 0; l < 3; ++l) drawn[l] += tris[l] * std::size_t(uses[k]);
        collision += g.collision.size() / 3 * std::size_t(uses[k]);
        std::printf("levelmap building %-28s x%-3d lod %5zu / %5zu / %4zu tris, collision %4zu tris in %3zu pieces, %zu materials\n",
                    g.model.id.c_str(), uses[k], tris[0], tris[1], tris[2], g.collision.size() / 3, g.pieceStarts.size(),
                    g.model.materials.size());
    }
    std::printf("levelmap buildings: %zu placed, %zu types; all at lod0 %zu / lod1 %zu / lod2 %zu tris, collision %zu tris\n",
                level->buildings().size(), uses.size(), drawn[0], drawn[1], drawn[2], collision);
    for (const LevelPoi& p : level->pois()) {
        int n = 0;
        for (const LevelBuilding& b : level->buildings()) n += std::hypot(b.x - p.x, b.z - p.z) < 130.0f;
        std::printf("levelmap poi %2d %-22s %-12s marker %7.1f %7.1f (ground %6.1f), teleport %7.1f %7.1f, %d buildings within 130 m\n",
                    p.id, p.name.c_str(), p.kind.c_str(), p.x, p.z, level->heightAt(p.x, p.z), p.atX, p.atZ, n);
    }
    const int pl = int((-level->playHalfExtent() - o) / 2.0f), ph = int((level->playHalfExtent() - o) / 2.0f);
    for (int k = pl; k <= ph; ++k) {
        put(k, pl, 1, 1, 1);
        put(k, ph, 1, 1, 1);
        put(pl, k, 1, 1, 1);
        put(ph, k, 1, 1, 1);
    }
    FILE* f = std::fopen(out, "wb");
    if (!f) return;
    std::fprintf(f, "P6\n%d %d\n255\n", W, W);
    std::fwrite(img.data(), 1, img.size(), f);
    std::fclose(f);
    std::printf("levelmap wrote %s (%d x %d, 2 m per pixel)\n", out, W, W);
}

void levelSlope(const char* path, const char* out) {
    std::string err;
    const auto level = Level::load(path, &err);
    if (!level) {
        std::printf("levelslope: %s\n", err.c_str());
        return;
    }
    const int n = level->samples();
    const float o = level->origin(), cell = level->cell(), play = level->playHalfExtent();
    const std::vector<float>& h = level->heights();
    auto H = [&](int i, int j) { return h[std::size_t(std::clamp(j, 0, n - 1)) * n + std::size_t(std::clamp(i, 0, n - 1))]; };
    std::vector<float> slope(std::size_t(n) * n, 0.0f);
    long total = 0, c45 = 0, c60 = 0, c70 = 0, c80 = 0;
    const int tile = int(32.0f / cell), tn = (n + tile - 1) / tile;
    std::vector<int> tiles(std::size_t(tn) * tn, 0);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const float gx = (H(i + 1, j) - H(i - 1, j)) / (2.0f * cell), gz = (H(i, j + 1) - H(i, j - 1)) / (2.0f * cell);
            const float deg = std::atan(std::hypot(gx, gz)) * 57.29578f;
            slope[std::size_t(j) * n + i] = deg;
            const float x = o + i * cell, z = o + j * cell;
            if (std::abs(x) > play || std::abs(z) > play) continue;
            if (level->waterAt(x, z)) continue;
            ++total;
            c45 += deg > 45.0f;
            c60 += deg > 60.0f;
            c70 += deg > 70.0f;
            c80 += deg > 80.0f;
            if (deg > 60.0f) ++tiles[std::size_t(j / tile) * tn + i / tile];
        }
    std::printf("levelslope %s: dry cells in the play area %ld: >45 deg %.2f %%, >60 %.2f %%, >70 %.2f %%, >80 %.3f %%\n",
                level->id().c_str(), total, 100.0 * c45 / total, 100.0 * c60 / total, 100.0 * c70 / total, 100.0 * c80 / total);
    std::vector<std::pair<int, int>> order;
    for (int k = 0; k < tn * tn; ++k)
        if (tiles[std::size_t(k)] > 0) order.push_back({tiles[std::size_t(k)], k});
    std::sort(order.rbegin(), order.rend());
    for (std::size_t k = 0; k < std::min<std::size_t>(order.size(), 25); ++k) {
        const int ti = order[k].second % tn, tj = order[k].second / tn;
        std::printf("levelslope tile centre (%.0f, %.0f): %d cells > 60 deg\n", o + (ti + 0.5f) * tile * cell,
                    o + (tj + 0.5f) * tile * cell, order[k].first);
    }
    std::vector<int> treeTiles(std::size_t(tn) * tn, 0);
    for (const LevelTree& t : level->trees()) {
        const int ti = int((t.x - o) / (tile * cell)), tj = int((t.z - o) / (tile * cell));
        if (ti >= 0 && tj >= 0 && ti < tn && tj < tn) ++treeTiles[std::size_t(tj) * tn + ti];
    }
    const int dense = int(std::max_element(treeTiles.begin(), treeTiles.end()) - treeTiles.begin());
    std::printf("levelslope densest forest tile centre (%.0f, %.0f): %d trees\n", o + (dense % tn + 0.5f) * tile * cell,
                o + (dense / tn + 0.5f) * tile * cell, treeTiles[std::size_t(dense)]);
    {
        double sum = 0.0;
        int cnt = 0;
        const int ti = dense % tn, tj = dense / tn;
        for (int j = tj * tile; j < (tj + 1) * tile && j < n - 1; ++j)
            for (int i = ti * tile; i < (ti + 1) * tile && i < n - 1; ++i, ++cnt)
                sum += double((level->splatTexels()[std::size_t(j) * std::size_t(n - 1) + std::size_t(i)] >> 24) & 0xffu) / 255.0;
        std::printf("levelslope mean canopy weight there %.2f\n", sum / std::max(cnt, 1));
    }
    const int W = (n - 1) / 2;
    std::vector<unsigned char> img(std::size_t(W) * W * 3);
    for (int py = 0; py < W; ++py)
        for (int px = 0; px < W; ++px) {
            const int i = px * 2, j = py * 2;
            const float deg = std::max({slope[std::size_t(j) * n + i], slope[std::size_t(j) * n + i + 1],
                                        slope[std::size_t(j + 1) * n + i], slope[std::size_t(j + 1) * n + i + 1]});
            float r = 0.2f + deg / 120.0f, g = r, b = r;
            if (level->waterAt(o + i * cell, o + j * cell)) { r = 0.1f; g = 0.2f; b = 0.5f; }
            if (deg > 45.0f) { r = 0.9f; g = 0.9f; b = 0.2f; }
            if (deg > 60.0f) { r = 1.0f; g = 0.55f; b = 0.1f; }
            if (deg > 70.0f) { r = 1.0f; g = 0.1f; b = 0.1f; }
            unsigned char* p = &img[(std::size_t(py) * W + px) * 3];
            p[0] = (unsigned char)std::clamp(r * 255.0f, 0.0f, 255.0f);
            p[1] = (unsigned char)std::clamp(g * 255.0f, 0.0f, 255.0f);
            p[2] = (unsigned char)std::clamp(b * 255.0f, 0.0f, 255.0f);
        }
    FILE* f = std::fopen(out, "wb");
    if (!f) return;
    std::fprintf(f, "P6\n%d %d\n255\n", W, W);
    std::fwrite(img.data(), 1, img.size(), f);
    std::fclose(f);
    std::printf("levelslope wrote %s (%d x %d, 2 m per pixel)\n", out, W, W);
}


// ------------------------------------------------------------------ mini handling track laps
struct MiniLap {
    double standing = -1.0, flying = -1.0; // s, from the simulation's lap timer
    std::vector<double> sectors;           // of the flying lap
    float maxKmh = 0.0f, maxLatG = 0.0f;   // flying lap; lateral g low-passed over ~0.2 s
    float steadyLatG = 0.0f, steadyKmh = 0.0f; // mean in the middle of the constant-radius corner (flying lap)
    float maxLateral = 0.0f;               // m from the center line (flying lap)
    double offTrack = 0.0;                 // s with two or more wheels off the asphalt and curbs (flying lap)
    int flips = 0;
};

MiniLap miniTrackLaps(const VehicleConfig& vehicle, float lateralAccel) {
    SimulationSettings s;
    s.vehicle = vehicle;
    Simulation sim(s);
    const auto& features = sim.track().features();
    int feature = -1;
    for (std::size_t k = 0; k < features.size(); ++k)
        if (features[k].name == "Mini handling track") feature = int(k);
    sim.requestTeleport(feature);
    sim.setIntent({.brake = 1.0f});
    for (int i = 0; i < 120; ++i) sim.tick();

    // The menu autopilot with a racing setup: full pedals, a lateral acceleration budget, braking planned at the
    // same budget, a longer pure-pursuit lookahead at speed. No resets.
    AutopilotSettings a;
    a.lateralAccel = lateralAccel;
    a.planDecel = std::min(lateralAccel, 8.0f);
    a.planAccel = 40.0f;
    a.maxThrottle = 1.0f;
    a.steerEase = 0.6f;
    a.kp = 0.5f;
    a.ki = 0.2f;
    a.kb = 0.6f;
    a.maxBrake = 1.0f;
    a.lookaheadMin = 6.0f;
    a.lookaheadPerSpeed = 0.45f;
    a.lookaheadMax = 26.0f;
    a.cornerLookahead = 0.8f;
    a.lookaheadCornerMin = 4.0f;
    a.downhillGrade = 1.0f;
    a.offRouteDistance = a.stuckTime = a.flippedTime = 1e9f;
    Autopilot driver(a);
    const MiniTrackInfo& mt = miniTrack();
    driver.setRoute(mt.route(60.0f), vehicle.wheelBase, vehicle.axles[kFrontAxle].maxSteerAngleDeg * 0.0174532925f);

    MiniLap r;
    const double dt = sim.fixedStep();
    auto& bi = sim.world().bodies();
    JPH::Vec3 prevV = bi.GetLinearVelocity(sim.vehicle().bodyId());
    float latFilt = 0.0f;
    double steadySumG = 0.0, steadySumV = 0.0;
    int steadyN = 0;
    for (double t = 0.0; t < 400.0 && sim.lapTimer().laps() < 2; t += dt) {
        const JPH::RMat44 xf = sim.vehicle().bodyTransform();
        const JPH::Vec3 p(xf.GetTranslation()), fwd = xf.GetAxisZ(), left = xf.GetAxisX();
        AutopilotInput in;
        in.x = p.GetX();
        in.z = p.GetZ();
        const float h = std::hypot(fwd.GetX(), fwd.GetZ());
        if (h > 1e-3f) in.fx = fwd.GetX() / h, in.fz = fwd.GetZ() / h;
        in.forwardY = fwd.GetY();
        in.upDot = xf.GetAxisY().GetY();
        in.forwardSpeed = sim.vehicle().telemetry().forwardSpeed;
        AutopilotOutput out = driver.update(in, float(dt));
        // Like a driver feeling the rear step out: lift as the body slip angle grows past 3 degrees.
        {
            const JPH::Vec3 vel = bi.GetLinearVelocity(sim.vehicle().bodyId());
            const float vh = std::hypot(vel.GetX(), vel.GetZ());
            if (vh > 3.0f) {
                const float beta = std::acos(std::clamp((vel.GetX() * in.fx + vel.GetZ() * in.fz) / vh, -1.0f, 1.0f));
                out.throttle *= std::clamp(1.0f - (beta * 57.3f - 3.0f) / 5.0f, 0.0f, 1.0f);
            }
        }
        sim.setIntent({.throttle = out.throttle, .brake = out.brake, .steer = out.steer, .analog = true});
        sim.tick();

        const JPH::Vec3 v = bi.GetLinearVelocity(sim.vehicle().bodyId());
        const float lat = float(((v - prevV) / float(dt)).Dot(left));
        prevV = v;
        latFilt += (lat - latFilt) * float(dt / 0.2);
        const auto& tel = sim.vehicle().telemetry();
        if (tel.upDot < 0.3f) ++r.flips;
        if (std::getenv("TRAILFORGE_MINITRACK_TRACE") && std::fmod(t, 0.5) < dt)
            std::printf("  t=%5.1f s=%6.1f pos=(%6.1f %5.2f %6.1f) %5.1f km/h target %5.1f thr %.2f brk %.2f str %+.2f "
                        "line %.1f m up %.2f lap %d\n",
                        t, driver.progress(), p.GetX(), p.GetY(), p.GetZ(), tel.speed * 3.6f, driver.targetSpeed() * 3.6f,
                        out.throttle, out.brake, out.steer, driver.lateralError(), tel.upDot, sim.lapTimer().laps());
        if (sim.lapTimer().laps() == 1) { // the flying lap is running
            r.maxKmh = std::max(r.maxKmh, tel.speed * 3.6f);
            r.maxLatG = std::max(r.maxLatG, std::abs(latFilt) / 9.81f);
            r.maxLateral = std::max(r.maxLateral, driver.lateralError());
            int off = 0;
            for (const auto& w : tel.wheels) off += w.contact && w.surface != SurfaceId::Asphalt;
            if (off >= 2) r.offTrack += dt;
            const float sArc = driver.progress();
            if (sArc > mt.steadyArc0 && sArc < mt.steadyArc1) {
                steadySumG += std::abs(lat) / 9.81;
                steadySumV += tel.speed * 3.6;
                ++steadyN;
            }
        }
        if (sim.lapTimer().laps() >= 1 && r.standing < 0.0) r.standing = sim.lapTimer().lastLap();
    }
    if (sim.lapTimer().laps() >= 2) {
        r.flying = sim.lapTimer().lastLap();
        r.sectors = sim.lapTimer().lastSectors();
    }
    if (steadyN > 0) {
        r.steadyLatG = float(steadySumG / steadyN);
        r.steadyKmh = float(steadySumV / steadyN);
    }
    return r;
}

void miniTrackProbe(int argc, char** argv) {
    const MiniTrackInfo& mt = miniTrack();
    std::printf("minitrack layout: lap %.1f m, sectors end at %.1f / %.1f m, constant-radius corner R %.0f m "
                "(steady window %.0f..%.0f m), start (%.1f, %.1f)\n",
                mt.length, mt.gateArc[1], mt.gateArc[2], mt.steadyRadius, mt.steadyArc0, mt.steadyArc1,
                mt.start.position.GetX(), mt.start.position.GetZ());
    std::vector<std::string> paths;
    if (const char* env = std::getenv("TRAILFORGE_VEHICLE")) {
        paths.push_back(env);
    } else {
        paths = {TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_suv/vehicle.json",
                 TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_ev/vehicle.json", "builtin",
                 TRAILFORGE_SOURCE_DIR "/tests/data/vehicles/rwd_test/vehicle.json"};
    }
    for (int k = 2; k < argc; ++k) paths.push_back(argv[k]);
    auto fmt = [](double s) {
        static char buf[4][24];
        static int slot = 0;
        char* b = buf[slot++ % 4];
        if (s < 0.0) std::snprintf(b, 24, "   --   ");
        else std::snprintf(b, 24, "%d:%06.3f", int(s / 60.0), std::fmod(s, 60.0));
        return b;
    };
    for (const std::string& path : paths) {
        VehicleConfig cfg;
        std::string id = "builtin";
        if (path != "builtin") {
            VehicleDefinition def;
            std::vector<std::string> errors;
            if (!loadVehicleDefinition(path, def, errors)) {
                for (const auto& e : errors) std::fprintf(stderr, "%s\n", e.c_str());
                continue;
            }
            cfg = def.config;
            id = def.id;
        }
        std::printf("minitrack vehicle %s: %s, %.0f kg\n", id.c_str(), driveLayoutName(cfg.layout), cfg.mass);
        std::vector<float> budgets = {5.0f, 5.5f, 6.0f, 6.5f, 7.0f, 8.0f, 9.0f, 10.0f};
        if (const char* b = std::getenv("TRAILFORGE_MINITRACK_BUDGET")) budgets = {float(std::atof(b))};
        float best = -1.0f;
        MiniLap bestLap;
        for (float b : budgets) {
            const MiniLap r = miniTrackLaps(cfg, b);
            const bool clean = r.flying > 0.0 && r.offTrack < 0.25 && r.maxLateral < 4.0f && r.flips == 0;
            std::printf("minitrack   budget %4.1f m/s^2: standing %s flying %s", b, fmt(r.standing), fmt(r.flying));
            for (std::size_t s = 0; s < r.sectors.size(); ++s) std::printf(" S%zu %.2f", s + 1, r.sectors[s]);
            std::printf(" | max %.0f km/h, max lat %.2f g, steady R%.0f %.2f g at %.0f km/h, line error max %.1f m, "
                        "off %.1f s%s%s\n",
                        r.maxKmh, r.maxLatG, mt.steadyRadius, r.steadyLatG, r.steadyKmh, r.maxLateral, r.offTrack,
                        r.flips ? ", ROLLED" : "", clean ? "" : "  (not clean)");
            if (clean && (best < 0 || r.flying < bestLap.flying)) {
                best = b;
                bestLap = r;
            }
        }
        if (best > 0.0f)
            std::printf("minitrack RESULT %-14s best clean flying lap %s (budget %.1f m/s^2), standing %s, max %.0f km/h, "
                        "max lat %.2f g, steady %.2f g\n",
                        id.c_str(), fmt(bestLap.flying), best, fmt(bestLap.standing), bestLap.maxKmh, bestLap.maxLatG,
                        bestLap.steadyLatG);
        else std::printf("minitrack RESULT %-14s no clean lap\n", id.c_str());
    }
}

// Scripted drift attempts on the mini handling track's constant-radius corner (R 32, the proving ground drift test
// bed, miniTrackDrift): traction control off, approach on the center line at an entry speed, initiation (full lock
// into the corner, optional hand brake pulse, full throttle), then a slip angle hold (countersteer toward the
// velocity, throttle on the angle error) through the corner, then straighten. The simulation's DriftScorer judges.
struct DriftAttempt {
    float entryKmh = 0.0f, targetDeg = 0.0f;
    bool handBrake = false;
    double score = 0.0, driftTime = 0.0, zoneTime = 0.0;
    float maxAngle = 0.0f, meanAngle = 0.0f, meanKmh = 0.0f;
    const char* end = "";
    bool failed = false;
};

DriftAttempt driftAttempt(const VehicleConfig& cfg, float entryKmh, bool handBrake, float targetDeg) {
    const MiniTrackInfo& mt = miniTrack();
    SimulationSettings s;
    s.vehicle = cfg;
    Simulation sim(s);
    AidControls aids = sim.aidControls();
    aids.traction = TractionMode::Off;
    sim.requestAidControls(aids);
    const std::size_t n = mt.center.size();
    const float s0 = mt.steadyArc0 - 0.25f * (mt.steadyArc1 - mt.steadyArc0); // corner entry
    auto indexAt = [&](float arc) {
        arc = std::fmod(arc + mt.length, mt.length);
        return std::size_t(std::upper_bound(mt.arc.begin(), mt.arc.end(), arc) - mt.arc.begin() - 1) % n;
    };
    const std::size_t i0 = indexAt(s0 - 70.0f), i1 = (i0 + 2) % n;
    TrackFeature at;
    at.position = JPH::Vec3(mt.center[i0].x, miniTrackGround(mt.center[i0].x, mt.center[i0].z), mt.center[i0].z);
    at.yaw = std::atan2(mt.center[i1].x - mt.center[i0].x, mt.center[i1].z - mt.center[i0].z);
    sim.requestPlace(at);
    for (int k = 0; k < 120; ++k) sim.tick();
    const float sigma = mt.curvature[indexAt(0.5f * (mt.steadyArc0 + mt.steadyArc1))] > 0.0f ? 1.0f : -1.0f; // + left turn
    const float maxSteer = cfg.axles[kFrontAxle].maxSteerAngleDeg;
    const float dt = float(sim.fixedStep());
    enum { Approach, Initiate, Hold, Exit } phase = Approach;
    float phaseTime = 0.0f, angleSum = 0.0f, kmhSum = 0.0f;
    int samples = 0;
    DriftAttempt r;
    r.entryKmh = entryKmh;
    r.targetDeg = targetDeg;
    r.handBrake = handBrake;
    bool ran = false;
    for (int tick = 0; tick < 40 * 120; ++tick) {
        const JPH::RMat44 xf = sim.vehicle().bodyTransform();
        const JPH::Vec3 p(xf.GetTranslation()), fwd = xf.GetAxisZ();
        const float yaw = std::atan2(fwd.GetX(), fwd.GetZ());
        float dist = 0.0f;
        const float arc = mt.nearestArc(p.GetX(), p.GetZ(), &dist);
        const float kmh = sim.vehicle().telemetry().speed * 3.6f;
        // Pure pursuit toward the center line 10 m ahead; steer + is right.
        auto pursuit = [&](float ahead) {
            const RoutePoint& q = mt.center[indexAt(arc + ahead)];
            float a = std::atan2(q.x - p.GetX(), q.z - p.GetZ()) - yaw;
            a = std::atan2(std::sin(a), std::cos(a)); // + toward +X (left)
            return std::clamp(-a * 57.3f * 1.4f / maxSteer, -1.0f, 1.0f);
        };
        DriverIntent in;
        in.analog = true;
        const float drift = -sigma * sim.drift().slipAngleDeg(); // + when sliding the right way
        phaseTime += dt;
        if (phase == Approach) {
            in.steer = pursuit(10.0f);
            in.throttle = std::clamp(0.3f * (entryKmh - kmh), 0.0f, 1.0f);
            in.brake = kmh > entryKmh + 3.0f ? 0.4f : 0.0f;
            const float ahead = std::fmod(s0 - arc + mt.length, mt.length);
            if (ahead < 2.0f || ahead > mt.length - 20.0f) phase = Initiate, phaseTime = 0.0f;
        } else if (phase == Initiate) {
            in.steer = -sigma;
            in.throttle = handBrake ? 0.5f : 1.0f;
            in.handBrake = handBrake && phaseTime < 0.4f ? 1.0f : 0.0f;
            if (drift > 0.6f * targetDeg || phaseTime > 1.0f) phase = Hold, phaseTime = 0.0f;
        } else if (phase == Hold) {
            // Front wheels along the velocity plus turn-in that grows when the vehicle runs wide of the center line,
            // more countersteer over the target angle; throttle on the angle error, less when wide or fast.
            const std::size_t c = indexAt(arc), c1 = (c + 1) % n;
            const float tx = mt.center[c1].x - mt.center[c].x, tz = mt.center[c1].z - mt.center[c].z;
            const float tl = std::max(1e-4f, std::hypot(tx, tz));
            const float left = ((p.GetX() - mt.center[c].x) * tz - (p.GetZ() - mt.center[c].z) * tx) / tl;
            const float wide = -sigma * left; // m outside the center line
            const float beta = sim.drift().slipAngleDeg();
            const float frontLeft = beta + sigma * (5.0f - 0.8f * (drift - targetDeg) + 1.5f * wide);
            in.steer = std::clamp(-frontLeft / maxSteer, -1.0f, 1.0f);
            in.throttle = std::clamp(0.5f + 0.06f * (targetDeg - drift) - 0.08f * wide -
                                         0.05f * std::max(0.0f, kmh - entryKmh), 0.0f, 1.0f);
            const bool past = std::fmod(arc - mt.steadyArc1 + mt.length, mt.length) < 0.5f * mt.length;
            if (past || phaseTime > 12.0f) phase = Exit, phaseTime = 0.0f;
        } else {
            in.steer = pursuit(12.0f);
            in.throttle = 0.15f;
            if ((!sim.drift().running() && ran) || phaseTime > 5.0f) break;
        }
        sim.setIntent(in);
        sim.tick();
        if (std::getenv("TF_DRIFT_DEBUG"))
            for (const BodyImpact& b : sim.bodyImpacts()) {
                const JPH::Vec3 bp = sim.currentSnapshot().body.position;
                std::printf("   impact t=%.2f phase %d at (%.1f %.2f %.1f) body (%.1f %.2f %.1f) ground %.2f approach %.2f m/s "
                            "impulse %.0f, center dist %.1f\n", tick * dt, int(phase), b.position.GetX(), b.position.GetY(),
                            b.position.GetZ(), bp.GetX(), bp.GetY(), bp.GetZ(),
                            miniTrackGround(b.position.GetX(), b.position.GetZ()), b.approachSpeed, b.impulse, dist);
            }
        const DriftScorer& d = sim.drift();
        if (d.running()) {
            ran = true;
            r.score = d.score();
            r.driftTime = d.driftTime();
            r.zoneTime = d.zoneTime();
            r.maxAngle = d.maxAngleDeg();
            if (d.drifting()) angleSum += std::abs(d.slipAngleDeg()), kmhSum += d.speedKmh(), ++samples;
        } else if (ran && phase != Exit) {
            break; // ended early (spin, wall, link timeout)
        }
    }
    // Straighten and roll on until the run is judged (link timeout).
    for (int tick = 0; tick < 6 * 120 && sim.drift().running(); ++tick) {
        DriverIntent in;
        in.analog = true;
        in.brake = 0.2f;
        sim.setIntent(in);
        sim.tick();
        if (sim.drift().running()) r.score = sim.drift().score();
    }
    const DriftScorer& d = sim.drift();
    r.failed = d.lastEnd() == DriftScorer::End::Spin || d.lastEnd() == DriftScorer::End::Wall;
    r.end = !ran ? "no run" : d.running() ? "still running" : DriftScorer::endName(d.lastEnd());
    if (ran && !d.running() && !r.failed) r.score = d.lastScore();
    if (r.failed) r.score = 0.0;
    r.meanAngle = samples ? angleSum / float(samples) : 0.0f;
    r.meanKmh = samples ? kmhSum / float(samples) : 0.0f;
    return r;
}

void driftProbe(int argc, char** argv) {
    const MiniTrackInfo& mt = miniTrack();
    std::printf("drift test bed: mini handling track R %.0f corner, steady %.0f..%.0f m\n", mt.steadyRadius, mt.steadyArc0,
                mt.steadyArc1);
    std::vector<std::string> paths;
    if (const char* env = std::getenv("TRAILFORGE_VEHICLE")) paths.push_back(env);
    else
        paths = {TRAILFORGE_SOURCE_DIR "/tests/data/vehicles/rwd_test/vehicle.json", "builtin",
                 TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_suv/vehicle.json",
                 TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_ev/vehicle.json"};
    for (int k = 2; k < argc; ++k) paths.push_back(argv[k]);
    for (const std::string& path : paths) {
        VehicleConfig cfg;
        std::string id = "builtin";
        if (path != "builtin") {
            VehicleDefinition def;
            std::vector<std::string> errors;
            if (!loadVehicleDefinition(path, def, errors)) {
                for (const auto& e : errors) std::fprintf(stderr, "%s\n", e.c_str());
                continue;
            }
            cfg = def.config;
            id = def.id;
        }
        const bool verbose = std::getenv("TF_DRIFT_VERBOSE") != nullptr;
        DriftAttempt best;
        float maxAngle = 0.0f;
        int attempts = 0, scored = 0, spins = 0, walls = 0;
        auto line = [&](const char* tag, const DriftAttempt& a) {
            std::printf("drift %s %-14s entry %2.0f km/h %-9s target %2.0f deg: score %6.0f, max %5.1f deg, mean %5.1f deg "
                        "at %3.0f km/h, drift %4.1f s (zones %4.1f s), end %s\n",
                        tag, id.c_str(), a.entryKmh, a.handBrake ? "handbrake" : "power", a.targetDeg, a.score, a.maxAngle,
                        a.meanAngle, a.meanKmh, a.driftTime, a.zoneTime, a.end);
        };
        for (float target : {20.0f, 30.0f})
            for (float kmh : {40.0f, 45.0f, 50.0f, 55.0f, 60.0f, 65.0f})
                for (bool hb : {false, true}) {
                    const DriftAttempt a = driftAttempt(cfg, kmh, hb, target);
                    ++attempts;
                    scored += a.score > 0.0 ? 1 : 0;
                    spins += std::strcmp(a.end, "spin") == 0 ? 1 : 0;
                    walls += std::strcmp(a.end, "wall hit") == 0 ? 1 : 0;
                    if (!a.failed) maxAngle = std::max(maxAngle, a.maxAngle);
                    if (verbose) line("try", a);
                    if (a.score > best.score) best = a;
                }
        if (best.score > 0.0) line("best", best);
        std::printf("drift RESULT %-14s %s, %.0f kg: best score %.0f, max angle of a scored run %.1f deg; %d attempts: "
                    "%d scored, %d spins, %d wall hits\n",
                    id.c_str(), driveLayoutName(cfg.layout), cfg.mass, best.score, maxAngle, attempts, scored, spins, walls);
    }
}

// Autopilot laps of a circuit level (its attractRoute pace): lap and sector times, top and mean speed, the
// largest lateral acceleration and the share of ticks with all four wheels on asphalt.
void lapProbe(const char* path, int laps) {
    std::string err;
    auto level = Level::load(path, &err);
    if (!level || level->circuit().gates.empty()) {
        std::fprintf(stderr, "lap: %s: %s\n", path, level ? "no circuit gates" : err.c_str());
        std::exit(1);
    }
    SimulationSettings s = probeSettings();
    s.level = level;
    s.terrain.async = false;
    Simulation sim(s);
    const LapTimer& lt = sim.lapTimer();
    sim.setAutopilot(AutopilotMode::Drive);
    double t = 0.0, maxKmh = 0.0, maxLat = 0.0;
    long ticks = 0, onAsphalt = 0, traced = 0;
    int done = 0;
    const bool trace = std::getenv("TF_LAP_TRACE") != nullptr; // body position once per 0.5 s
    // Lateral acceleration from the body path: velocity and acceleration by differences, smoothed over ~0.2 s.
    JPH::Vec3 prevPos = sim.currentSnapshot().body.position, prevVel = JPH::Vec3::sZero(), acc = JPH::Vec3::sZero();
    std::printf("lap %s: circuit %.0f m, %zu gates, %d sectors\n", level->id().c_str(), double(level->circuit().length),
                lt.gates().size(), lt.sectorCount());
    while (t < 300.0 * laps + 60.0 && lt.laps() < laps && sim.autopilotResets() == 0) {
        sim.tick();
        t += sim.fixedStep();
        const VehicleTelemetry& tel = sim.vehicle().telemetry();
        const JPH::Vec3 pos = sim.currentSnapshot().body.position;
        const float dt = sim.fixedStep();
        const JPH::Vec3 vel = (pos - prevPos) / dt;
        acc += ((vel - prevVel) / dt - acc) * std::min(1.0f, dt / 0.2f);
        prevPos = pos;
        prevVel = vel;
        if (trace && ++traced % 60 == 0)
            std::printf("  t %6.2f pos %7.2f %6.2f %7.2f  %5.1f km/h  steer %+.2f throttle %.2f brake %.2f\n", t,
                        double(pos.GetX()), double(pos.GetY()), double(pos.GetZ()), double(tel.forwardSpeed) * 3.6,
                        double(sim.resolvedIntent().steer), double(sim.resolvedIntent().throttle),
                        double(sim.resolvedIntent().brake));
        if (lt.running()) {
            maxKmh = std::max(maxKmh, double(tel.forwardSpeed) * 3.6);
            const JPH::Vec3 vh(vel.GetX(), 0.0f, vel.GetZ());
            if (vh.Length() > 3.0f) {
                const JPH::Vec3 side = JPH::Vec3(0, 1, 0).Cross(vh).Normalized();
                maxLat = std::max(maxLat, double(std::abs(acc.Dot(side))));
            }
            int n = 0;
            for (const auto& w : tel.wheels) n += w.contact && w.surface == SurfaceId::Asphalt;
            onAsphalt += n == 4;
            ++ticks;
        }
        if (lt.laps() > done) {
            done = lt.laps();
            std::printf("lap %d: %.3f s", done, lt.lastLap());
            for (int k = 0; k < lt.sectorCount(); ++k) std::printf("  S%d %.3f", k + 1, lt.lastSectorTime(k));
            std::printf("\n");
        }
    }
    const double best = lt.bestLap();
    std::printf("lap best %.3f s (%.1f km/h mean), max %.1f km/h, max lateral %.2f m/s^2, all wheels on asphalt %.1f %% "
                "of the ticks, resets %d (%s), recoveries %d\n",
                best, best > 0.0 ? double(level->circuit().length) / best * 3.6 : 0.0, maxKmh, maxLat,
                100.0 * double(onAsphalt) / double(std::max(ticks, 1L)), sim.autopilotResets(), sim.autopilotResetReason(),
                sim.recoveries());
    if (done < laps || sim.autopilotResets() > 0) std::exit(1);
}

// ---- Tire sets and tire temperature (vehicle/TireModel.h)

VehicleConfig loadProbeVehicle(const std::string& path) {
    VehicleDefinition def;
    std::vector<std::string> errors;
    if (!loadVehicleDefinition(path, def, errors)) {
        for (const auto& e : errors) std::fprintf(stderr, "%s\n", e.c_str());
        std::exit(2);
    }
    return def.config;
}

float windowMiddle(TireSet set, float airC) {
    const TireCompound& c = tireCompound(set);
    return 0.5f * (std::max(c.optimumLowC, airC) + c.optimumHighC);
}

struct TireLap {
    double time = 0.0;
    float surfaceAvg = 0.0f, surfaceMax = 0.0f, coreAvg = 0.0f, gripMin = 1.0f; // at the end of the lap
    float maxLatG = 0.0f, maxLateral = 0.0f;
    double offTrack = 0.0, heatJ = 0.0, slideJ = 0.0, groundJ = 0.0, airJ = 0.0; // all four tires over the lap
    // Grip use: per tick the largest combined slip of the four tires relative to the peak of the fitted curves,
    // u = sqrt((slip ratio / peak ratio)^2 + (slip angle / peak angle)^2) (1 = at the peak of the curve); lap mean and
    // the time with u >= 0.8 (u capped at 2 in the mean).
    double useSum = 0.0, limitTime = 0.0, budgetSum = 0.0;
    long ticks = 0;
    bool flipped = false;
    double useMean() const { return ticks ? useSum / double(ticks) : 0.0; }
    double budgetMean() const { return ticks ? budgetSum / double(ticks) : 0.0; }
    bool clean() const { return offTrack < 0.25 && maxLateral < 4.0f && !flipped; }
};

float tireUse(const Vehicle& v) {
    const VehicleTuning& t = v.effectiveTireTuning();
    float u = 0.0f;
    for (const WheelTelemetry& w : v.telemetry().wheels) {
        if (!w.contact || !w.slipValid) continue;
        const float a = w.longitudinalSlip / std::max(t.tirePeakSlipRatio, 1e-3f);
        const float b = std::abs(w.lateralSlipDeg) / std::max(t.tirePeakSlipAngleDeg, 1e-3f);
        u = std::max(u, std::sqrt(a * a + b * b));
    }
    return u;
}

// Peak lateral mu the tires offer on asphalt now: curve peak x asphalt x set x mean temperature grip factor.
float tireLimitMu(Simulation& sim) {
    const Vehicle& v = sim.vehicle();
    const VehicleTuning& t = v.effectiveTireTuning();
    float grip = 0.0f;
    for (const WheelTelemetry& w : v.telemetry().wheels) grip += 0.25f * w.tireGrip;
    return t.tirePeakLatFriction * sim.surfaces().get(SurfaceId::Asphalt).lateralFriction *
           t.tireSurfaceGrip[std::size_t(SurfaceId::Asphalt)] * grip;
}

// Laps with the racing autopilot of miniTrackLaps at one lateral acceleration budget: the mini handling track (level
// nullptr) or a circuit level's attract route. Tires start at the air temperature (or warm: the middle of the set's
// window). Per completed lap: time and the tire state at the line. adaptK > 0: a limit driver instead of a fixed
// budget: every second the budget is adaptK x g x tireLimitMu (the same share of the grip the tires offer now, so
// cold tires are driven slower and warm slicks faster).
std::vector<TireLap> budgetLaps(const VehicleConfig& vehicle, std::shared_ptr<const Level> level, float budget, int laps,
                                bool warm, float adaptK = 0.0f) {
    SimulationSettings s;
    s.vehicle = vehicle;
    s.level = level;
    s.terrain.async = false;
    Simulation sim(s);
    DriveRoute route;
    if (level) {
        route = level->attractRoute();
    } else {
        const auto& features = sim.track().features();
        for (std::size_t k = 0; k < features.size(); ++k)
            if (features[k].name == "Mini handling track") sim.requestTeleport(int(k));
        route = miniTrack().route(60.0f);
    }
    // The mini track has no straight long enough to need a cap; the circuit keeps its route speed and braking but
    // corners at the budget of the run instead of its showcase pace.
    if (level) route.lateralAccel = budget;
    else route.cruiseSpeed = 70.0f;
    auto adapt = [&] {
        budget = adaptK * 9.81f * tireLimitMu(sim);
        route.lateralAccel = budget;
        if (!level) route.brakeDecel = std::min(budget, 8.0f);
    };
    sim.setIntent({.brake = 1.0f});
    for (int i = 0; i < 120; ++i) sim.tick();
    if (warm) sim.setTireTemperatures(windowMiddle(vehicle.tireSet, sim.ambientTemperature()),
                                      windowMiddle(vehicle.tireSet, sim.ambientTemperature()));
    if (adaptK > 0.0f) adapt();
    AutopilotSettings a;
    a.lateralAccel = budget;
    a.planDecel = std::min(budget, 8.0f);
    a.planAccel = 40.0f;
    a.maxThrottle = 1.0f;
    a.steerEase = 0.6f;
    a.kp = 0.5f;
    a.ki = 0.2f;
    a.kb = 0.6f;
    a.maxBrake = 1.0f;
    a.lookaheadMin = 6.0f;
    a.lookaheadPerSpeed = 0.45f;
    a.lookaheadMax = 26.0f;
    a.cornerLookahead = 0.8f;
    a.lookaheadCornerMin = 4.0f;
    a.downhillGrade = 1.0f;
    a.offRouteDistance = a.stuckTime = a.flippedTime = 1e9f;
    Autopilot driver(a);
    driver.setRoute(route, vehicle.wheelBase, vehicle.axles[kFrontAxle].maxSteerAngleDeg * 0.0174532925f);
    const double dt = sim.fixedStep();
    auto& bi = sim.world().bodies();
    JPH::Vec3 prevV = bi.GetLinearVelocity(sim.vehicle().bodyId());
    float latFilt = 0.0f;
    std::vector<TireLap> out;
    TireLap cur;
    const double limit = (level ? 400.0 : 120.0) * laps;
    double nextAdapt = 1.0;
    for (double t = 0.0; t < limit && int(out.size()) < laps; t += dt) {
        if (adaptK > 0.0f && t >= nextAdapt) {
            nextAdapt += 1.0;
            adapt();
            driver.setRoute(route, vehicle.wheelBase, vehicle.axles[kFrontAxle].maxSteerAngleDeg * 0.0174532925f);
        }
        const JPH::RMat44 xf = sim.vehicle().bodyTransform();
        const JPH::Vec3 p(xf.GetTranslation()), fwd = xf.GetAxisZ(), left = xf.GetAxisX();
        AutopilotInput in;
        in.x = p.GetX();
        in.z = p.GetZ();
        const float h = std::hypot(fwd.GetX(), fwd.GetZ());
        if (h > 1e-3f) in.fx = fwd.GetX() / h, in.fz = fwd.GetZ() / h;
        in.forwardY = fwd.GetY();
        in.upDot = xf.GetAxisY().GetY();
        in.forwardSpeed = sim.vehicle().telemetry().forwardSpeed;
        AutopilotOutput o = driver.update(in, float(dt));
        const JPH::Vec3 vel = bi.GetLinearVelocity(sim.vehicle().bodyId());
        const float vh = std::hypot(vel.GetX(), vel.GetZ());
        if (vh > 3.0f) {
            const float beta = std::acos(std::clamp((vel.GetX() * in.fx + vel.GetZ() * in.fz) / vh, -1.0f, 1.0f));
            o.throttle *= std::clamp(1.0f - (beta * 57.3f - 3.0f) / 5.0f, 0.0f, 1.0f);
        }
        sim.setIntent({.throttle = o.throttle, .brake = o.brake, .steer = o.steer, .analog = true});
        const int before = sim.lapTimer().laps();
        sim.tick();
        const JPH::Vec3 v = bi.GetLinearVelocity(sim.vehicle().bodyId());
        latFilt += (float(((v - prevV) / float(dt)).Dot(left)) - latFilt) * float(dt / 0.2);
        prevV = v;
        const auto& tel = sim.vehicle().telemetry();
        if (sim.lapTimer().running()) {
            cur.maxLatG = std::max(cur.maxLatG, std::abs(latFilt) / 9.81f);
            cur.maxLateral = std::max(cur.maxLateral, driver.lateralError());
            int off = 0;
            for (const auto& w : tel.wheels) off += w.contact && w.surface != SurfaceId::Asphalt;
            if (off >= 2) cur.offTrack += dt;
            cur.flipped = cur.flipped || tel.upDot < 0.3f;
            const float u = tireUse(sim.vehicle());
            cur.useSum += std::min(u, 2.0f); // a spin reads as u >> 1: capped so one spin does not set the lap mean
            if (u >= 0.8f) cur.limitTime += dt;
            cur.budgetSum += budget;
            ++cur.ticks;
            for (int i = 0; i < kWheelCount; ++i) {
                const TireThermalState& th = sim.vehicle().tireThermal(i);
                cur.heatJ += tel.wheels[std::size_t(i)].tireHeatW * dt;
                cur.slideJ += th.heatSliding * dt;
                cur.groundJ += th.toGround * dt;
                cur.airJ += th.toAir * dt;
            }
        }
        if (sim.lapTimer().laps() > before) {
            cur.time = sim.lapTimer().lastLap();
            cur.surfaceMax = 0.0f;
            for (const auto& w : tel.wheels) {
                cur.surfaceAvg += 0.25f * w.tireSurfaceC;
                cur.coreAvg += 0.25f * w.tireCoreC;
                cur.surfaceMax = std::max(cur.surfaceMax, w.tireSurfaceC);
                cur.gripMin = std::min(cur.gripMin, w.tireGrip);
            }
            out.push_back(cur);
            cur = TireLap{};
        }
    }
    return out;
}

// One straight stop on a proving ground lane from kTireStopKmh with the tires at the air temperature or in the middle
// of their window (set before the run-up and again before the stop). The distance is scaled to exactly the start
// speed; ok = the run-up reached it and the vehicle stayed within 2.5 m of the lane line.
constexpr float kTireStopKmh = 60.0f;
struct TireStop {
    float distance = 0.0f, hottest = 0.0f;
    bool ok = false;
};
TireStop tireStop(const VehicleConfig& vehicle, int lane, bool abs, bool warm) {
    SimulationSettings s;
    s.vehicle = vehicle;
    int feature = 0;
    useLane(s, lane, feature);
    Simulation sim(s);
    sim.requestTeleport(feature);
    for (int i = 0; i < 120; ++i) sim.tick();
    const float t0 = warm ? windowMiddle(vehicle.tireSet, sim.ambientTemperature()) : sim.ambientTemperature();
    sim.setTireTemperatures(t0, t0);
    setAids(sim, abs, TractionMode::On, 0);
    const JPH::RMat44 start = sim.vehicle().bodyTransform();
    const JPH::Vec3 origin(start.GetTranslation()), heading = start.GetAxisZ();
    auto offLine = [&] {
        const JPH::Vec3 d = JPH::Vec3(sim.vehicle().bodyTransform().GetTranslation()) - origin;
        return std::hypot(d.GetX() - heading.GetX() * d.Dot(heading), d.GetZ() - heading.GetZ() * d.Dot(heading));
    };
    for (int i = 0; i < int(30.0 / sim.fixedStep()) && sim.vehicle().telemetry().speed * 3.6f < kTireStopKmh; ++i) {
        sim.setIntent({.throttle = 1.0f});
        sim.tick();
    }
    setAids(sim, abs, TractionMode::Off, 0);
    sim.setTireTemperatures(t0, t0);
    const float v0 = sim.vehicle().telemetry().speed;
    float path = 0.0f, worst = offLine();
    for (double t = 0.0; sim.vehicle().telemetry().speed > 0.05f && t < 30.0; t += sim.fixedStep()) {
        sim.setIntent({.brake = 1.0f});
        sim.tick();
        path += sim.vehicle().telemetry().speed * float(sim.fixedStep());
        worst = std::max(worst, offLine());
    }
    TireStop r;
    for (const auto& w : sim.vehicle().telemetry().wheels) r.hottest = std::max(r.hottest, w.tireSurfaceC);
    const float target = kTireStopKmh / 3.6f;
    r.distance = path * target * target / std::max(v0 * v0, 1.0f);
    r.ok = v0 >= 0.98f * target && worst < 2.5f;
    return r;
}

// Steady cornering at the grip limit on the flat concrete pad: the racing autopilot on a 40 m radius circle with its
// lateral acceleration raised by 0.1 m/s^2 per second while it holds the line (radial error < 1 m) and lowered by
// 0.5 m/s^2 per second while it runs wide (> 2 m), i.e. it keeps the vehicle just at the limit. Tires start at the air
// temperature. Prints the state every 15 s.
void tireLimitCircle(const std::string& id, const VehicleConfig& vehicle, double seconds) {
    SimulationSettings s;
    s.vehicle = vehicle;
    s.layout = TrackLayout::Flat;
    Simulation sim(s);
    for (int i = 0; i < 120; ++i) sim.tick();
    constexpr float kR = 40.0f;
    DriveRoute route;
    route.cruiseSpeed = 60.0f;
    for (int k = 0; k < 126; ++k) {
        const float a = float(k) / 126.0f * 6.2831853f;
        route.points.push_back({kR - kR * std::cos(a), kR * std::sin(a)});
    }
    float accel = 5.0f;
    route.lateralAccel = accel;
    route.brakeDecel = 4.0f;
    AutopilotSettings a;
    a.planAccel = 40.0f;
    a.maxThrottle = 1.0f;
    a.steerEase = 0.0f;
    a.kp = 0.5f;
    a.ki = 0.2f;
    a.kb = 0.6f;
    a.maxBrake = 1.0f;
    a.lookaheadMin = 6.0f;
    a.lookaheadPerSpeed = 0.45f;
    a.lookaheadMax = 26.0f;
    a.offRouteDistance = a.stuckTime = a.flippedTime = 1e9f;
    Autopilot driver(a);
    const float maxSteer = vehicle.axles[kFrontAxle].maxSteerAngleDeg * 0.0174532925f;
    driver.setRoute(route, vehicle.wheelBase, maxSteer);
    setAids(sim, false, TractionMode::Off, 0);
    const double dt = sim.fixedStep();
    auto& bi = sim.world().bodies();
    JPH::Vec3 prevV = bi.GetLinearVelocity(sim.vehicle().bodyId());
    float latFilt = 0.0f;
    double useSum = 0.0, latSum = 0.0, heat = 0.0, slide = 0.0;
    long n = 0;
    double nextAdapt = 1.0, nextPrint = 15.0;
    for (double t = 0.0; t < seconds; t += dt) {
        if (t >= nextAdapt) {
            nextAdapt += 1.0;
            const float err = driver.lateralError();
            if (t > 10.0 && err < 1.0f) accel += 0.1f;
            else if (err > 2.0f) accel = std::max(3.0f, accel - 0.5f);
            route.lateralAccel = accel;
            driver.setRoute(route, vehicle.wheelBase, maxSteer);
        }
        const JPH::RMat44 xf = sim.vehicle().bodyTransform();
        const JPH::Vec3 p(xf.GetTranslation()), fwd = xf.GetAxisZ(), left = xf.GetAxisX();
        AutopilotInput in;
        in.x = p.GetX();
        in.z = p.GetZ();
        const float h = std::hypot(fwd.GetX(), fwd.GetZ());
        if (h > 1e-3f) in.fx = fwd.GetX() / h, in.fz = fwd.GetZ() / h;
        in.forwardY = fwd.GetY();
        in.upDot = xf.GetAxisY().GetY();
        in.forwardSpeed = sim.vehicle().telemetry().forwardSpeed;
        const AutopilotOutput o = driver.update(in, float(dt));
        sim.setIntent({.throttle = o.throttle, .brake = o.brake, .steer = o.steer, .analog = true});
        sim.tick();
        const JPH::Vec3 v = bi.GetLinearVelocity(sim.vehicle().bodyId());
        latFilt += (float(((v - prevV) / float(dt)).Dot(left)) - latFilt) * float(dt / 0.5);
        prevV = v;
        useSum += tireUse(sim.vehicle());
        latSum += std::abs(latFilt);
        for (int i = 0; i < kWheelCount; ++i) {
            heat += sim.vehicle().telemetry().wheels[std::size_t(i)].tireHeatW * dt;
            slide += sim.vehicle().tireThermal(i).heatSliding * dt;
        }
        ++n;
        if (t + dt >= nextPrint) {
            nextPrint += 15.0;
            const auto& tel = sim.vehicle().telemetry();
            float sAvg = 0.0f, sMax = 0.0f, cAvg = 0.0f, grip = 0.0f;
            for (const auto& w : tel.wheels) {
                sAvg += 0.25f * w.tireSurfaceC;
                sMax = std::max(sMax, w.tireSurfaceC);
                cAvg += 0.25f * w.tireCoreC;
                grip += 0.25f * w.tireGrip;
            }
            std::printf("tires circle %-14s %-8s t %4.0f s: %5.1f km/h, lat %.2f g (limit mu %.2f), grip use %.2f, tread avg "
                        "%5.1f max %5.1f C, core %5.1f C, grip x%.3f, heat in %.2f kW per tire (sliding %.2f), line error "
                        "%.1f m\n",
                        id.c_str(), tireSetName(vehicle.tireSet), t + dt, tel.speed * 3.6f, latSum / n / 9.81,
                        tireLimitMu(sim), useSum / n, sAvg, sMax, cAvg, grip, heat / (15.0 * 4000.0),
                        slide / (15.0 * 4000.0), driver.lateralError());
            useSum = latSum = heat = slide = 0.0;
            n = 0;
        }
    }
}

void tiresProbe(int argc, char** argv) {
    const std::string section = argc > 2 ? argv[2] : "all";
    std::vector<std::pair<std::string, std::string>> vehicles = {
        {"trailforge_suv", TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_suv/vehicle.json"},
        {"trailforge_ev", TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_ev/vehicle.json"}};
    if (const char* env = std::getenv("TRAILFORGE_VEHICLE")) vehicles = {{loadProbeVehicle(env).id, env}};
    const TireSet sets[] = {TireSet::Offroad, TireSet::Road, TireSet::Slick};
    for (TireSet set : sets) {
        const TireCompound& c = tireCompound(set);
        const TireThermalParams p = tireThermalParams(0.40f, 0.30f, 35.0f, c);
        std::printf("tires set %-8s window %3.0f..%3.0f C, grip at 20 C x%.2f, asphalt x%.2f gravel x%.2f mud x%.2f, "
                    "Crr %.3f; SUV tire C surface %.0f J/K core %.0f J/K\n",
                    tireSetName(set), c.optimumLowC, c.optimumHighC, compoundGripFactor(c, 20.0f),
                    c.surfaceGrip[std::size_t(SurfaceId::Asphalt)], c.surfaceGrip[std::size_t(SurfaceId::Gravel)],
                    c.surfaceGrip[std::size_t(SurfaceId::Mud)], c.rollingResistance, p.surfaceCapacity, p.coreCapacity);
    }
    if (section == "all" || section == "brake") {
        const char* names[] = {"concrete", "dirt", "mud", "grass", "rock", "lowgrip", "sand", "asphalt", "gravel"};
        for (const auto& [id, path] : vehicles) {
            for (TireSet set : sets) {
                VehicleConfig cfg = loadProbeVehicle(path);
                cfg.tireSet = set;
                for (int lane : {7, 0, 8, 1, 3, 2}) {
                    const TireStop cold = tireStop(cfg, lane, true, false);
                    const TireStop warm = tireStop(cfg, lane, true, true);
                    auto cell = [](const TireStop& r) {
                        char buf[48];
                        if (r.ok) std::snprintf(buf, sizeof buf, "%6.2f m (%.2f g)", r.distance,
                                                (kTireStopKmh / 3.6f) * (kTireStopKmh / 3.6f) / (2.0f * r.distance * 9.81f));
                        else std::snprintf(buf, sizeof buf, "%-17s", "  -- (run-up)");
                        return std::string(buf);
                    };
                    std::printf("tires brake %-14s %-8s %-8s %.0f-0 ABS on: air 20 C %s, warm %s", id.c_str(),
                                tireSetName(set), names[lane], kTireStopKmh, cell(cold).c_str(), cell(warm).c_str());
                    if (lane == 7) {
                        const TireStop locked = tireStop(cfg, lane, false, false);
                        std::printf("; ABS off from 20 C %s, hottest tread after %.1f C (ABS on: %.1f C)",
                                    cell(locked).c_str(), locked.hottest, cold.hottest);
                    }
                    std::printf("\n");
                }
            }
        }
    }
    auto lapRows = [&](const char* where, const std::vector<std::pair<std::string, std::string>>& list,
                       std::shared_ptr<const Level> level, const std::vector<float>& budgets, int laps) {
        for (const auto& [id, path] : list) {
            for (TireSet set : sets) {
                VehicleConfig cfg = loadProbeVehicle(path);
                cfg.tireSet = set;
                double best = -1.0;
                float bestBudget = 0.0f;
                std::vector<TireLap> bestRun;
                for (float b : budgets) {
                    const std::vector<TireLap> run = budgetLaps(cfg, level, b, laps, false);
                    std::printf("tires laps %-10s %-14s %-8s budget %4.1f:", where, id.c_str(), tireSetName(set), b);
                    for (std::size_t k = 0; k < run.size(); ++k)
                        std::printf(" L%zu %.2f s%s", k + 1, run[k].time, run[k].clean() ? "" : "*");
                    std::printf("\n");
                    for (std::size_t k = 1; k < run.size(); ++k)
                        if (run[k].clean() && (best < 0.0 || run[k].time < best)) {
                            best = run[k].time;
                            bestBudget = b;
                            bestRun = run;
                        }
                }
                if (best < 0.0) {
                    std::printf("tires RESULT %-10s %-14s %-8s no clean flying lap\n", where, id.c_str(), tireSetName(set));
                    continue;
                }
                std::printf("tires RESULT %-10s %-14s %-8s best clean flying lap %.3f s (budget %.1f m/s^2)\n", where,
                            id.c_str(), tireSetName(set), best, bestBudget);
                for (std::size_t k = 0; k < bestRun.size(); ++k) {
                    const TireLap& l = bestRun[k];
                    std::printf("tires warmup %-10s %-14s %-8s lap %zu %8.3f s%s  tread avg %5.1f max %5.1f C, core avg %5.1f C, "
                                "min grip x%.3f, max lat %.2f g, off %.2f s; per tire: heat in %.2f kW (sliding %.2f), to ground %.2f kW, "
                                "to air %.2f kW; grip use mean %.2f, at >= 0.8 for %.0f %% of the lap\n",
                                where, id.c_str(), tireSetName(set), k + 1, l.time, l.clean() ? " " : "*", l.surfaceAvg,
                                l.surfaceMax, l.coreAvg, l.gripMin, l.maxLatG, l.offTrack, l.heatJ / l.time / 4000.0,
                                l.slideJ / l.time / 4000.0, l.groundJ / l.time / 4000.0, l.airJ / l.time / 4000.0,
                                l.useMean(), 100.0 * l.limitTime / l.time);
                }
                const std::vector<TireLap> warm = budgetLaps(cfg, level, bestBudget, 2, true);
                if (warm.size() == 2)
                    std::printf("tires warmstart %-10s %-14s %-8s budget %.1f: L1 %.3f s L2 %.3f s, tread avg after L1 %.1f C\n",
                                where, id.c_str(), tireSetName(set), bestBudget, warm[0].time, warm[1].time, warm[0].surfaceAvg);
            }
        }
    };
    if (section == "all" || section == "minitrack") {
        // TRAILFORGE_TIRES_BUDGET=b: the SUV at that budget only (quick look).
        if (const char* b = std::getenv("TRAILFORGE_TIRES_BUDGET")) lapRows("minitrack", {vehicles[0]}, nullptr, {float(std::atof(b))}, 5);
        else lapRows("minitrack", vehicles, nullptr, {6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f}, 5);
    }
    if (section == "all" || section == "cooldown") {
        // One slow lap of the mini track (lateral budget 3 m/s^2, about a third of the grip) with tires starting in
        // the middle of their window: how much they cool.
        for (const auto& [id, path] : vehicles)
            for (TireSet set : sets) {
                VehicleConfig cfg = loadProbeVehicle(path);
                cfg.tireSet = set;
                const std::vector<TireLap> run = budgetLaps(cfg, nullptr, 3.0f, 1, true);
                if (run.empty()) continue;
                const TireLap& l = run[0];
                std::printf("tires cooldown %-14s %-8s from %5.1f C: slow lap %7.2f s, tread avg %5.1f max %5.1f C at the "
                            "line, core %5.1f C, min grip x%.3f, heat in %.2f kW per tire, to air %.2f, to ground %.2f\n",
                            id.c_str(), tireSetName(set), windowMiddle(set, kDefaultAmbientC), l.time, l.surfaceAvg,
                            l.surfaceMax, l.coreAvg, l.gripMin, l.heatJ / l.time / 4000.0, l.airJ / l.time / 4000.0,
                            l.groundJ / l.time / 4000.0);
            }
    }
    if (section == "all" || section == "circle") {
        for (const auto& [id, path] : vehicles)
            for (TireSet set : sets) {
                VehicleConfig cfg = loadProbeVehicle(path);
                cfg.tireSet = set;
                tireLimitCircle(id, cfg, 150.0);
            }
    }
    if (section == "all" || section == "limit") {
        // The limit driver (budget = k x g x the mu the tires offer now) on the mini track, 5 laps from cold, and on
        // Suncoast, 3 laps; k swept, the best clean flying lap per set reported like the fixed budgets.
        auto limitRows = [&](const char* where, const std::vector<std::pair<std::string, std::string>>& list,
                             std::shared_ptr<const Level> level, const std::vector<float>& ks, int laps) {
            for (const auto& [id, path] : list)
                for (TireSet set : sets) {
                    VehicleConfig cfg = loadProbeVehicle(path);
                    cfg.tireSet = set;
                    double best = -1.0;
                    float bestK = 0.0f;
                    std::vector<TireLap> bestRun;
                    for (float k : ks) {
                        const std::vector<TireLap> run = budgetLaps(cfg, level, 0.0f, laps, false, k);
                        std::printf("tires limitlaps %-10s %-14s %-8s k %.2f:", where, id.c_str(), tireSetName(set), k);
                        for (std::size_t j = 0; j < run.size(); ++j)
                            std::printf(" L%zu %.2f s%s", j + 1, run[j].time, run[j].clean() ? "" : "*");
                        std::printf("\n");
                        for (std::size_t j = 1; j < run.size(); ++j)
                            if (run[j].clean() && (best < 0.0 || run[j].time < best)) {
                                best = run[j].time;
                                bestK = k;
                                bestRun = run;
                            }
                    }
                    if (best < 0.0) {
                        std::printf("tires LIMIT %-10s %-14s %-8s no clean flying lap\n", where, id.c_str(), tireSetName(set));
                        continue;
                    }
                    std::printf("tires LIMIT %-10s %-14s %-8s best clean flying lap %.3f s (k %.2f)\n", where, id.c_str(),
                                tireSetName(set), best, bestK);
                    for (std::size_t j = 0; j < bestRun.size(); ++j) {
                        const TireLap& l = bestRun[j];
                        std::printf("tires limitlap %-10s %-14s %-8s lap %zu %8.3f s%s budget %.1f m/s^2, tread avg %5.1f max "
                                    "%5.1f C, core %5.1f C, min grip x%.3f, max lat %.2f g, grip use mean %.2f, >= 0.8 %3.0f %%, "
                                    "heat in %.2f kW per tire (sliding %.2f)\n",
                                    where, id.c_str(), tireSetName(set), j + 1, l.time, l.clean() ? " " : "*", l.budgetMean(),
                                    l.surfaceAvg, l.surfaceMax, l.coreAvg, l.gripMin, l.maxLatG, l.useMean(),
                                    100.0 * l.limitTime / l.time, l.heatJ / l.time / 4000.0, l.slideJ / l.time / 4000.0);
                    }
                }
        };
        limitRows("minitrack", vehicles, nullptr, {0.65f, 0.70f, 0.75f, 0.80f, 0.85f, 0.90f}, 5);
        std::string err;
        auto level = Level::load(TRAILFORGE_SOURCE_DIR "/assets/levels/suncoast_raceway/level.json", &err);
        if (level) limitRows("suncoast", {vehicles[0]}, level, {0.55f, 0.60f, 0.65f, 0.70f, 0.75f}, 3);
    }
    if (section == "all" || section == "suncoast") {
        std::string err;
        auto level = Level::load(TRAILFORGE_SOURCE_DIR "/assets/levels/suncoast_raceway/level.json", &err);
        if (!level) std::fprintf(stderr, "tires: %s\n", err.c_str());
        else lapRows("suncoast", {vehicles[0]}, level, {5.0f, 6.0f, 7.0f, 8.0f}, 3);
    }
}

// ---- bump: straight runs over one bump on the flat pad (TrackGeometry::addBump), per vehicle.
// From 1 m before the bump: Hold keeps the speed controller (it opens the throttle up to full as the bump slows the
// vehicle), Cruise freezes the throttle at its cruise value, Coast lifts off, Full floors it.
enum class BumpDriver { Hold, Cruise, Coast, Full };

// Kicker: a 1 m long round bump (steepest face atan(pi h / 1 m): 43 deg at 0.30 m, 51 deg at 0.40 m), a ramp at
// speed. Mound: the same cosine over 2.5 m (21 / 27 deg), a hummock. Step: a ledge with 60 degree faces.
enum class BumpShape { Kicker, Mound, Step };
const char* bumpShapeName(BumpShape s) { return s == BumpShape::Kicker ? "kicker" : s == BumpShape::Mound ? "mound" : "step"; }
BumpShape bumpShape(const std::string& s) {
    return s == "mound" ? BumpShape::Mound : s == "step" ? BumpShape::Step : BumpShape::Kicker;
}

struct BumpRun {
    BumpShape shape = BumpShape::Kicker;
    bool offset = false; // under the left wheels only
    float height = 0.3f; // m
    float kmh = 30.0f;   // target speed at the bump
    BumpDriver driver = BumpDriver::Hold;
};

struct BumpOutcome {
    float entryKmh = 0.0f;
    float gain = 0.0f;       // m, max center-of-mass rise over the cruise height
    float air = 0.0f;        // s, longest time with no wheel in contact
    float lift = 0.0f;       // s, longest time with two or more wheels off the ground
    float roll = 0.0f;       // deg, max |roll|
    float pitch = 0.0f;      // deg, max |pitch|
    float suspVel = 0.0f;    // m/s, max compression speed of a wheel in contact
    int stops = 0;           // suspension solid (minLength), rising edges summed over the wheels
    int rims = 0;            // Jolt's rigid hard point (tire at its rim, or minLength with a rigid tire)
    float peakLoad = 0.0f;   // N, max single-wheel suspension force
    float nz = 0.0f;         // g, max vertical specific force of the body (50 ms mean; 1 = at rest)
    bool bodyHit = false;    // chassis touched the bump
    bool rolled = false;     // body upside down at any time or more than 60 degrees tilted at the end
};

float bumpBase(BumpShape p, float h) {
    if (p == BumpShape::Step) return 2.0f * h / std::tan(60.0f * 3.14159265f / 180.0f) + 0.3f;
    return p == BumpShape::Kicker ? 1.0f : 2.5f;
}

BumpOutcome bumpRun(const VehicleConfig& cfg, const BumpRun& run, bool trace = false) {
    SimulationSettings s;
    s.layout = TrackLayout::Flat;
    s.vehicle = cfg;
    Simulation sim(s);
    const float zc = 40.0f, base = bumpBase(run.shape, run.height);
    const float zIn = zc - 0.5f * base, zOut = zc + 0.5f * base;
    TrackGeometry g;
    g.addBump(run.offset ? 0.35f : -3.0f, 3.0f, zc, base, run.height,
              run.shape == BumpShape::Step ? BumpProfile::Step : BumpProfile::Round, 0.0f, SurfaceId::Concrete);
    JPH::TriangleList tris;
    for (std::size_t i = 0; i < g.triangleCount(); ++i)
        tris.emplace_back(g.positions[3 * i], g.positions[3 * i + 1], g.positions[3 * i + 2], 0u);
    JPH::PhysicsMaterialList mats;
    mats.push_back(sim.surfaces().material(SurfaceId::Concrete));
    auto& bi = sim.world().bodies();
    const JPH::BodyID bumpId =
        bi.CreateAndAddBody(JPH::BodyCreationSettings(JPH::MeshShapeSettings(tris, mats).Create().Get(), JPH::RVec3::sZero(),
                                                      JPH::Quat::sIdentity(), JPH::EMotionType::Static, quench::PhysicsLayers::kStatic),
                            JPH::EActivation::DontActivate);
    const JPH::BodyID vid = sim.vehicle().bodyId();
    const double dt = sim.fixedStep();
    const float target = run.kmh / 3.6f;
    BumpOutcome o;
    double restSum = 0.0;
    int restN = 0;
    float rest = 0.0f, prevL[4] = {0, 0, 0, 0};
    bool prevStop[4] = {false, false, false, false}, prevRim[4] = {false, false, false, false};
    bool prevContact[4] = {false, false, false, false};
    std::vector<float> vy;
    double airRun = 0.0, liftRun = 0.0, tOut = -1.0;
    float cruiseThrottle = -1.0f;
    bool measuring = false;
    VehicleAudioFeed feed;
    for (int i = 0; i < int(30.0 / dt); ++i) {
        const auto& t = sim.vehicle().telemetry();
        const float frontZ = float(sim.currentSnapshot().wheels[0].position.GetZ());
        const float rearZ = float(sim.currentSnapshot().wheels[2].position.GetZ());
        DriverIntent in;
        const float err = target - t.forwardSpeed;
        in.throttle = std::clamp(0.15f + 0.5f * err, 0.0f, 1.0f);
        if (frontZ < zIn - 3.0f && err < -1.0f) in.brake = std::clamp(-0.2f * err, 0.0f, 0.5f);
        if (frontZ > zIn - 1.0f && run.driver != BumpDriver::Hold) {
            if (cruiseThrottle < 0.0f) cruiseThrottle = in.throttle;
            in.throttle = run.driver == BumpDriver::Coast ? 0.0f : run.driver == BumpDriver::Full ? 1.0f : cruiseThrottle;
        }
        sim.setIntent(in);
        sim.tick();
        if (trace) {
            feed.onTick(sim);
            static const char* kEventNames[] = {"start", "gear", "bump-stop", "rattle", "impact", "splash", "creak", "ui"};
            for (const AudioEvent& e : feed.events())
                std::printf("  t=%5.2f audio event %s %.2f\n", i * dt, kEventNames[int(e.type)], e.intensity);
            feed.clearEvents();
            const AudioFrame& af = feed.frame();
            bool any = false;
            for (const WheelAudio& w : af.wheels) any = any || w.squeal > 0.0f;
            if (any) {
                std::printf("  t=%5.2f squeal", i * dt);
                for (const WheelAudio& w : af.wheels)
                    std::printf(" | %.2f c%d slip %.1f m/s long %.2f lat %.2f roll %.1f load %.0f", w.squeal, int(w.contact),
                                w.slipSpeed, w.longRatio, w.latRatio, w.rollSpeed, w.load);
                std::printf("\n");
            }
        }
        const auto& tw = sim.vehicle().telemetry();
        const JPH::RVec3 com = bi.GetCenterOfMassPosition(vid);
        const JPH::Quat q = bi.GetRotation(vid);
        const JPH::Vec3 up = q.RotateAxisY(), left = q.RotateAxisX(), fwd = q.RotateAxisZ();
        vy.push_back(bi.GetLinearVelocity(vid).GetY());
        if (frontZ > zIn - 15.0f && frontZ < zIn - 5.0f) {
            restSum += com.GetY();
            ++restN;
        }
        if (!measuring && frontZ > zIn - 1.0f) {
            measuring = true;
            rest = restN > 0 ? float(restSum / restN) : float(com.GetY());
            o.entryKmh = tw.forwardSpeed * 3.6f;
            for (int w = 0; w < 4; ++w) prevL[w] = tw.wheels[w].suspensionLength;
        }
        if (tOut < 0.0 && rearZ > zOut + 0.5f) tOut = i * dt;
        if (measuring) {
            o.gain = std::max(o.gain, float(com.GetY()) - rest);
            int contacts = 0;
            for (int w = 0; w < 4; ++w) {
                const auto& wt = tw.wheels[w];
                contacts += wt.contact;
                if (wt.contact && prevContact[w])
                    o.suspVel = std::max(o.suspVel, float((prevL[w] - wt.suspensionLength) / dt));
                if (wt.bumpStop && !prevStop[w]) ++o.stops;
                if (wt.rimHit && !prevRim[w]) ++o.rims;
                prevRim[w] = wt.rimHit;
                o.peakLoad = std::max(o.peakLoad, wt.load);
                prevStop[w] = wt.bumpStop;
                prevContact[w] = wt.contact;
                prevL[w] = wt.suspensionLength;
            }
            airRun = contacts == 0 ? airRun + dt : 0.0;
            liftRun = contacts <= 2 ? liftRun + dt : 0.0;
            o.air = std::max(o.air, float(airRun));
            o.lift = std::max(o.lift, float(liftRun));
            o.roll = std::max(o.roll, std::abs(std::atan2(left.GetY(), up.GetY())) * 57.2958f);
            o.pitch = std::max(o.pitch, std::abs(std::atan2(fwd.GetY(), up.GetY())) * 57.2958f);
            if (vy.size() > 6) {
                const float a = float((vy.back() - vy[vy.size() - 7]) / (6.0 * dt));
                o.nz = std::max(o.nz, (a + 9.81f) / 9.81f);
            }
            o.bodyHit = o.bodyHit || sim.world().system().WereBodiesInContact(vid, bumpId);
            o.rolled = o.rolled || up.GetY() < 0.0f;
            if (trace && i % 6 == 0)
                std::printf("  t=%5.2f z=%6.2f dy=%+.3f roll=%+6.1f pitch=%+6.1f v=%5.1f | L %.3f %.3f %.3f %.3f | %c%c%c%c\n",
                            i * dt, frontZ, float(com.GetY()) - rest, std::atan2(left.GetY(), up.GetY()) * 57.2958f,
                            std::atan2(fwd.GetY(), up.GetY()) * 57.2958f, tw.forwardSpeed * 3.6f,
                            tw.wheels[0].suspensionLength, tw.wheels[1].suspensionLength, tw.wheels[2].suspensionLength,
                            tw.wheels[3].suspensionLength, tw.wheels[0].contact ? 'C' : '-', tw.wheels[1].contact ? 'C' : '-',
                            tw.wheels[2].contact ? 'C' : '-', tw.wheels[3].contact ? 'C' : '-');
        }
        if (tOut >= 0.0 && i * dt > tOut + 4.0) break;
    }
    o.rolled = o.rolled || bi.GetRotation(vid).RotateAxisY().GetY() < 0.5f;
    return o;
}

// Suspension numbers as Jolt uses them: effective mass of each wheel constraint (body mass and inertia at the force
// point), spring rate and damper coefficient from frequency and ratio, and the ratio against the static corner load.
void bumpVehicleHeader(const std::string& id, const VehicleConfig& cfg) {
    SimulationSettings s;
    s.layout = TrackLayout::Flat;
    s.vehicle = cfg;
    Simulation sim(s);
    for (int i = 0; i < 360; ++i) sim.tick();
    const auto& t = sim.vehicle().telemetry();
    JPH::Vec3 inertia, invI;
    float invMass = 0.0f;
    {
        JPH::BodyLockRead lock(sim.world().system().GetBodyLockInterface(), sim.vehicle().bodyId());
        const JPH::MotionProperties* mp = lock.GetBody().GetMotionProperties();
        invMass = mp->GetInverseMass();
        invI = mp->GetInverseInertiaDiagonal();
        inertia = JPH::Vec3(1.0f / invI.GetX(), 1.0f / invI.GetY(), 1.0f / invI.GetZ());
    }
    auto& bi = sim.world().bodies();
    const float comY = float(bi.GetCenterOfMassPosition(sim.vehicle().bodyId()).GetY());
    std::printf("bump vehicle %s: %.0f kg, principal inertia %.0f / %.0f / %.0f kg*m^2 (radii of gyration %.2f / %.2f / "
                "%.2f m), CoM %.2f m above ground, SSF track/(2h) %.2f\n",
                id.c_str(), cfg.mass, inertia.GetX(), inertia.GetY(), inertia.GetZ(), std::sqrt(inertia.GetX() / cfg.mass),
                std::sqrt(inertia.GetY() / cfg.mass), std::sqrt(inertia.GetZ() / cfg.mass), comY,
                cfg.axles[kFrontAxle].track / (2.0f * comY));
    for (int axle = 0; axle < 2; ++axle) {
        const AxleConfig& a = cfg.axles[axle];
        const AxleTuning& at = cfg.tuning.axles[axle];
        const int w = 2 * axle;
        const float hz = 0.5f * cfg.wheelBase * (axle == 0 ? 1.0f : -1.0f);
        const JPH::Vec3 fp(0.5f * a.track, cfg.attachHeight - 0.5f * (a.suspensionMinLength + a.suspensionMaxLength), hz);
        const JPH::Vec3 r = fp.Cross(JPH::Vec3(0, -1, 0));
        const float meff = 1.0f / (invMass + r.Dot(invI * r));
        const float omega = 2.0f * 3.14159265f * at.springFrequency;
        const float k = meff * omega * omega;
        const float cC = 2.0f * meff * at.compressionDamping * omega, cR = 2.0f * meff * at.reboundDamping * omega;
        const float corner = 0.5f * (t.wheels[w].load + t.wheels[w + 1].load) / 9.81f;
        const float crit = 2.0f * std::sqrt(k * corner);
        const float travel = a.suspensionMaxLength - a.suspensionMinLength;
        const float restL = 0.5f * (t.wheels[w].suspensionLength + t.wheels[w + 1].suspensionLength);
        std::printf("  %s: corner %.0f kg (static), Jolt m_eff %.0f kg, %.2f Hz -> k %.1f N/mm, c %.2f / %.2f kN*s/m "
                    "(ratio %.2f / %.2f of the corner's critical), travel %.3f m, rest %.0f%% compressed, bump travel "
                    "left %.3f m, anti-roll %.0f, bump stop %.3f m x%.1f (%.1f kN at full), damper knee %.2f m/s "
                    "slopes %.2f / %.2f\n",
                    axle == 0 ? "front" : "rear ", corner, meff, at.springFrequency, k / 1000.0f, cC / 1000.0f,
                    cR / 1000.0f, cC / crit, cR / crit, travel, 100.0f * (a.suspensionMaxLength - restL) / travel,
                    restL - a.suspensionMinLength, at.antiRoll, at.bumpStopRange, at.bumpStopFrequencyScale,
                    k * bumpStopForceRatio(at, at.bumpStopRange) / 1000.0f, at.damperKneeSpeed,
                    at.compressionHighSpeedSlope, at.reboundHighSpeedSlope);
    }
}

void bumpProbe(int argc, char** argv) {
    const std::string section = argc > 2 ? argv[2] : "all";
    const auto driver = [](const std::string& d) {
        return d == "cruise" ? BumpDriver::Cruise : d == "coast" ? BumpDriver::Coast : d == "full" ? BumpDriver::Full
                                                                                    : BumpDriver::Hold;
    };
    std::vector<std::string> paths;
    if (const char* env = std::getenv("TRAILFORGE_VEHICLE")) {
        paths.push_back(env);
    } else {
        for (const char* id : {"trailforge_suv", "trailforge_ev", "trailforge_patrol"})
            paths.push_back(std::string(TRAILFORGE_SOURCE_DIR "/assets/vehicles/") + id + "/vehicle.json");
        paths.push_back("builtin");
    }
    std::vector<float> heights = {0.2f, 0.3f, 0.4f}, speeds = {20.0f, 30.0f, 40.0f};
    if (section == "quick") {
        heights = {0.3f, 0.35f, 0.4f};
        speeds = {30.0f};
    }
    for (const std::string& path : paths) {
        VehicleConfig cfg;
        std::string id = "builtin";
        if (path != "builtin") {
            VehicleDefinition def;
            std::vector<std::string> errors;
            if (!loadVehicleDefinition(path, def, errors)) {
                for (const auto& e : errors) std::fprintf(stderr, "%s\n", e.c_str());
                continue;
            }
            cfg = def.config;
            id = def.id;
        }
        bumpVehicleHeader(id, cfg);
        if (section == "trace") {
            const BumpRun run{argc > 7 ? bumpShape(argv[7]) : BumpShape::Kicker,
                              argc > 3 && std::atoi(argv[3]) != 0, argc > 4 ? float(std::atof(argv[4])) : 0.35f,
                              argc > 5 ? float(std::atof(argv[5])) : 30.0f, argc > 6 ? driver(argv[6]) : BumpDriver::Hold};
            bumpRun(cfg, run, true);
            continue;
        }
        int rolls = 0, runs = 0;
        std::printf("  driver %s\n", argc > 3 ? argv[3] : "hold");
        std::printf("  shape  side  h(m) km/h | entry | gain(m) air(s) lift(s) | roll  pitch (deg) | susp(m/s) solid rim "
                    "peak(kN) nz(g) | body\n");
        for (BumpShape p : {BumpShape::Kicker, BumpShape::Mound, BumpShape::Step})
            for (bool offset : {false, true})
                for (float h : heights)
                    for (float v : speeds) {
                        const BumpOutcome o = bumpRun(cfg, {p, offset, h, v, argc > 3 ? driver(argv[3]) : BumpDriver::Hold});
                        rolls += o.rolled;
                        ++runs;
                        std::printf("  %-6s %-5s %.2f %4.0f | %5.1f | %6.3f %6.2f %6.2f | %5.1f %5.1f | %6.2f %5d %3d "
                                    "%7.1f %5.2f | %s%s\n",
                                    bumpShapeName(p), offset ? "left" : "full", h, v, o.entryKmh,
                                    o.gain, o.air, o.lift, o.roll, o.pitch, o.suspVel, o.stops, o.rims, o.peakLoad / 1000.0f,
                                    o.nz,
                                    o.bodyHit ? "hit" : "-", o.rolled ? "  ROLLED" : "");
                    }
        std::printf("bump RESULT %-18s %-6s %d of %d runs rolled over\n", id.c_str(), argc > 3 ? argv[3] : "hold", rolls, runs);
    }
}

// ---- hill / drop: hill-crest launches and drop rebound (the tire-stage damping, AxleConfig::tireDamping).
std::vector<std::pair<std::string, VehicleConfig>> probeVehicles() {
    std::vector<std::pair<std::string, VehicleConfig>> out;
    std::vector<std::string> paths;
    if (const char* env = std::getenv("TRAILFORGE_VEHICLE")) {
        paths.push_back(env);
    } else {
        for (const char* id : {"trailforge_suv", "trailforge_ev", "trailforge_patrol"})
            paths.push_back(std::string(TRAILFORGE_SOURCE_DIR "/assets/vehicles/") + id + "/vehicle.json");
    }
    for (const std::string& path : paths) {
        if (path == "builtin") {
            out.push_back({"builtin", VehicleConfig{}});
            continue;
        }
        VehicleDefinition def;
        std::vector<std::string> errors;
        if (!loadVehicleDefinition(path, def, errors)) {
            for (const auto& e : errors) std::fprintf(stderr, "%s\n", e.c_str());
            continue;
        }
        out.push_back({def.id, def.config});
    }
    return out;
}

struct HillOutcome {
    float entry = 0, gain = 0, air = 0, takeoffVy = 0, takeoffRate = 0, maxRate = 0, minPitch = 0, maxPitch = 0,
          landRate = 0, peakLoad = 0;
    int stops = 0, rims = 0;
    bool rolled = false;
};

// Straight run over a cosine hill (height H, length L, across the whole width) on the flat pad, speed held by a
// throttle controller up to 2 m before the hill; then the driver holds that throttle (hold), keeps the controller
// (cruise), coasts or floors it.
HillOutcome hillRun(const VehicleConfig& cfg, float H, float L, float kmh, const std::string& drv, bool trace) {
    SimulationSettings s;
    s.layout = TrackLayout::Flat;
    s.vehicle = cfg;
    Simulation sim(s);
    const float zc = 150.0f, zIn = zc - 0.5f * L;
    JPH::TriangleList tris;
    const int n = 64;
    const float x0 = -40.0f, x1 = 40.0f;
    auto yAt = [&](int k) { return 0.5f * H * (1.0f - std::cos(2.0f * 3.14159265f * float(k) / float(n))); };
    for (int k = 0; k < n; ++k) {
        const float za = zIn + L * float(k) / float(n), zb = za + L / float(n);
        const JPH::Float3 a(x0, yAt(k), za), b(x1, yAt(k), za), c(x1, yAt(k + 1), zb), d(x0, yAt(k + 1), zb);
        tris.emplace_back(a, d, c, 0u);
        tris.emplace_back(a, c, b, 0u);
    }
    JPH::PhysicsMaterialList mats;
    mats.push_back(sim.surfaces().material(SurfaceId::Concrete));
    auto& bi = sim.world().bodies();
    bi.CreateAndAddBody(JPH::BodyCreationSettings(JPH::MeshShapeSettings(tris, mats).Create().Get(), JPH::RVec3::sZero(),
                                                  JPH::Quat::sIdentity(), JPH::EMotionType::Static, quench::PhysicsLayers::kStatic),
                        JPH::EActivation::DontActivate);
    const JPH::BodyID vid = sim.vehicle().bodyId();
    const double dt = sim.fixedStep();
    const float target = kmh / 3.6f;
    HillOutcome o;
    float cruise = -1.0f, rest = 0.0f, airRun = 0.0f;
    bool measuring = false, airborne = false;
    bool prevStop[kWheelCount] = {}, prevRim[kWheelCount] = {};
    double restSum = 0.0;
    int restN = 0;
    for (int i = 0; i < int(40.0 / dt); ++i) {
        const auto& t = sim.vehicle().telemetry();
        const float frontZ = float(sim.currentSnapshot().wheels[0].position.GetZ());
        DriverIntent in;
        const float err = target - t.forwardSpeed;
        in.throttle = std::clamp(0.15f + 0.5f * err, 0.0f, 1.0f);
        if (frontZ < zIn - 10.0f && err < -1.0f) in.brake = std::clamp(-0.2f * err, 0.0f, 0.5f);
        if (frontZ > zIn - 2.0f && drv != "cruise") {
            if (cruise < 0.0f) cruise = in.throttle;
            in.throttle = drv == "coast" ? 0.0f : drv == "full" ? 1.0f : cruise;
        }
        sim.setIntent(in);
        sim.tick();
        const auto& tw = sim.vehicle().telemetry();
        const JPH::RVec3 com = bi.GetCenterOfMassPosition(vid);
        const JPH::Quat q = bi.GetRotation(vid);
        const JPH::Vec3 up = q.RotateAxisY(), left = q.RotateAxisX(), fwd = q.RotateAxisZ();
        const JPH::Vec3 w = bi.GetAngularVelocity(vid), v = bi.GetLinearVelocity(vid);
        const float rate = w.Dot(left) * 57.2958f;                       // + = nose down
        const float pitch = std::atan2(fwd.GetY(), up.GetY()) * 57.2958f; // + = nose up
        if (frontZ > zIn - 30.0f && frontZ < zIn - 10.0f) {
            restSum += com.GetY();
            ++restN;
        }
        if (!measuring && frontZ > zIn - 1.5f) {
            measuring = true;
            rest = restN ? float(restSum / restN) : float(com.GetY());
            o.entry = tw.forwardSpeed * 3.6f;
        }
        if (!measuring) continue;
        int contacts = 0;
        for (int k = 0; k < kWheelCount; ++k) {
            const auto& wt = tw.wheels[std::size_t(k)];
            contacts += wt.contact;
            if (wt.bumpStop && !prevStop[k]) ++o.stops;
            if (wt.rimHit && !prevRim[k]) ++o.rims;
            prevStop[k] = wt.bumpStop;
            prevRim[k] = wt.rimHit;
            o.peakLoad = std::max(o.peakLoad, wt.load);
        }
        if (contacts == 0 && !airborne) {
            airborne = true;
            o.takeoffRate = rate;
            o.takeoffVy = v.GetY();
        }
        if (contacts > 0 && airborne && airRun > 0.0f && o.landRate == 0.0f) o.landRate = rate;
        airRun = contacts == 0 ? airRun + float(dt) : 0.0f;
        o.air = std::max(o.air, airRun);
        o.maxRate = std::max(o.maxRate, std::abs(rate));
        o.maxPitch = std::max(o.maxPitch, pitch);
        o.minPitch = std::min(o.minPitch, pitch);
        o.gain = std::max(o.gain, float(com.GetY()) - rest);
        o.rolled = o.rolled || up.GetY() < 0.0f;
        if (trace && i % 3 == 0)
            std::printf("  t=%5.2f z=%6.2f y=%+.2f vy=%+5.2f pitch=%+6.1f rate=%+6.1f v=%5.1f | L %.3f %.3f %.3f %.3f | "
                        "F %5.1f %5.1f %5.1f %5.1f kN | g%d %5.0f rpm Fx %+5.2f %+5.2f kN\n",
                        i * dt, frontZ, float(com.GetY()) - rest, v.GetY(), pitch, rate, tw.forwardSpeed * 3.6f,
                        tw.wheels[0].suspensionLength, tw.wheels[1].suspensionLength, tw.wheels[2].suspensionLength,
                        tw.wheels[3].suspensionLength, tw.wheels[0].load / 1000, tw.wheels[1].load / 1000,
                        tw.wheels[2].load / 1000, tw.wheels[3].load / 1000, tw.gear, tw.engineRPM,
                        (tw.wheels[0].longitudinalForce + tw.wheels[1].longitudinalForce) / 1000,
                        (tw.wheels[2].longitudinalForce + tw.wheels[3].longitudinalForce) / 1000);
        if (frontZ > zc + 0.5f * L + 60.0f) break;
    }
    o.rolled = o.rolled || bi.GetRotation(vid).RotateAxisY().GetY() < 0.5f;
    return o;
}

// hill all [driver] | hill H L kmh [driver] [trace]
void hillProbe(int argc, char** argv) {
    const bool all = argc > 2 && std::string(argv[2]) == "all";
    const std::string drv = all ? (argc > 3 ? argv[3] : "hold") : (argc > 5 ? argv[5] : "hold");
    struct Hill {
        float h, l;
    };
    std::vector<Hill> hills = {{0.5f, 8.0f}, {0.9f, 13.0f}, {1.0f, 12.0f}, {1.0f, 15.0f}, {1.5f, 20.0f}};
    std::vector<float> speeds = {80.0f, 100.0f};
    if (!all) {
        hills = {{argc > 2 ? float(std::atof(argv[2])) : 1.0f, argc > 3 ? float(std::atof(argv[3])) : 15.0f}};
        speeds = {argc > 4 ? float(std::atof(argv[4])) : 80.0f};
    }
    for (const auto& [id, cfg] : probeVehicles()) {
        int rolls = 0, runs = 0;
        for (const Hill& h : hills)
            for (float kmh : speeds) {
                const HillOutcome o = hillRun(cfg, h.h, h.l, kmh, drv, !all && argc > 6);
                rolls += o.rolled;
                ++runs;
                std::printf("hill %-18s H %.2f L %4.1f %-6s %3.0f km/h entry %5.1f | gain %.2f m air %.2f s | takeoff vy %+.2f "
                            "rate %+6.1f deg/s | max |rate| %6.1f pitch %+5.1f / %+5.1f | land rate %+6.1f | solid %d rim %d "
                            "peak %.1f kN %s\n",
                            id.c_str(), h.h, h.l, drv.c_str(), kmh, o.entry, o.gain, o.air, o.takeoffVy, o.takeoffRate,
                            o.maxRate, o.minPitch, o.maxPitch, o.landRate, o.stops, o.rims, o.peakLoad / 1000.0f,
                            o.rolled ? "ROLLED" : "");
            }
        if (all) std::printf("hill RESULT %-18s %-6s %d of %d runs rolled over\n", id.c_str(), drv.c_str(), rolls, runs);
    }
}

// drop H [pitchDeg]: settle on the flat pad, lift the body by H (the wheels hang at full droop) and release; rebound
// = the highest COM rise after the first landing, above rest, as a share of the drop.
void dropProbe(int argc, char** argv) {
    const float H = argc > 2 ? float(std::atof(argv[2])) : 1.5f;
    const float pitchDeg = argc > 3 ? float(std::atof(argv[3])) : 0.0f;
    for (const auto& [id, cfg] : probeVehicles()) {
        SimulationSettings s;
        s.layout = TrackLayout::Flat;
        s.vehicle = cfg;
        Simulation sim(s);
        auto& bi = sim.world().bodies();
        const JPH::BodyID vid = sim.vehicle().bodyId();
        for (int i = 0; i < int(5.0 / sim.fixedStep()); ++i) {
            sim.setIntent(DriverIntent{});
            sim.tick();
        }
        const float rest = float(bi.GetCenterOfMassPosition(vid).GetY());
        bi.SetPositionAndRotation(vid, bi.GetPosition(vid) + JPH::Vec3(0, H, 0),
                                  JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -pitchDeg / 57.2958f), JPH::EActivation::Activate);
        bi.SetLinearAndAngularVelocity(vid, JPH::Vec3::sZero(), JPH::Vec3::sZero());
        const float y0 = float(bi.GetCenterOfMassPosition(vid).GetY());
        bool landed = false, rising = false;
        float minY = 1e9f, maxAfter = -1e9f, peakLoad = 0.0f;
        int solid = 0;
        bool prevStop[kWheelCount] = {};
        for (int i = 0; i < int(4.0 / sim.fixedStep()); ++i) {
            sim.setIntent(DriverIntent{});
            sim.tick();
            const auto& t = sim.vehicle().telemetry();
            const float y = float(bi.GetCenterOfMassPosition(vid).GetY());
            int c = 0;
            for (int k = 0; k < kWheelCount; ++k) {
                c += t.wheels[std::size_t(k)].contact;
                peakLoad = std::max(peakLoad, t.wheels[std::size_t(k)].load);
                if (t.wheels[std::size_t(k)].bumpStop && !prevStop[k]) ++solid;
                prevStop[k] = t.wheels[std::size_t(k)].bumpStop;
            }
            if (c > 0) landed = true;
            if (landed) minY = std::min(minY, y);
            if (landed && bi.GetLinearVelocity(vid).GetY() > 0.0f) rising = true;
            if (rising) maxAfter = std::max(maxAfter, y);
        }
        std::printf("drop %-18s H %.2f pitch %+.0f: fall %.2f m (%.2f m/s) | lowest %+.3f m vs rest | rebound %+.3f m "
                    "(%.0f %% of the drop) | solid %d peak %.1f kN\n",
                    id.c_str(), H, pitchDeg, y0 - rest, std::sqrt(2.0f * 9.81f * std::max(0.0f, y0 - rest)), minY - rest,
                    maxAfter - rest, 100.0f * (maxAfter - rest) / std::max(y0 - rest, 1e-3f), solid, peakLoad / 1000.0f);
    }
}

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "bench";
    if (mode == "levelprofile" && argc > 6) {
        std::string err;
        const auto level = Level::load(argc > 7 ? argv[7] : "assets/levels/exploration_peaks/level.json", &err);
        if (!level) return 1;
        const float x0 = float(std::atof(argv[2])), z0 = float(std::atof(argv[3])), x1 = float(std::atof(argv[4])),
                    z1 = float(std::atof(argv[5]));
        const int steps = std::max(1, std::atoi(argv[6]));
        for (int k = 0; k <= steps; ++k) {
            const float t = float(k) / float(steps), x = x0 + (x1 - x0) * t, z = z0 + (z1 - z0) * t;
            const auto w = level->waterAt(x, z);
            std::printf("levelprofile %8.1f %8.1f  h %7.2f  water %7.2f\n", x, z, level->heightAt(x, z), w ? *w : -1.0f);
        }
        return 0;
    }
    if (mode == "levelrocks" && argc > 4) {
        std::string err;
        const auto level = Level::load("assets/levels/exploration_peaks/level.json", &err);
        if (!level) return 1;
        const float x = float(std::atof(argv[2])), z = float(std::atof(argv[3])), r = float(std::atof(argv[4]));
        for (const LevelRock& k : level->rocks())
            if (std::hypot(k.x - x, k.z - z) < r)
                std::printf("levelrocks %7.1f %6.1f %7.1f  ext %.1f %.1f %.1f pitch %5.1f  %s\n", k.x, k.y, k.z, k.rx, k.ry, k.rz,
                            k.pitch * 57.3f, k.model >= 0 ? level->models()[std::size_t(k.model)].path.c_str() : "procedural");
        return 0;
    }
    if (mode == "levelslope") {
        levelSlope(argc > 2 ? argv[2] : "assets/levels/exploration_peaks/level.json", argc > 3 ? argv[3] : "/tmp/trailforge_levelslope.ppm");
        return 0;
    }
    if (mode == "levelmap") {
        levelMap(argc > 2 ? argv[2] : "assets/levels/exploration_peaks/level.json", argc > 3 ? argv[3] : "/tmp/trailforge_levelmap.ppm");
        return 0;
    }
    if (mode == "lap") lapProbe(argc > 2 ? argv[2] : TRAILFORGE_SOURCE_DIR "/assets/levels/suncoast_raceway/level.json",
                                argc > 3 ? std::max(1, std::atoi(argv[3])) : 2);
    else if (mode == "brake") brake();
    else if (mode == "course") course(argc > 2 ? std::atof(argv[2]) : 120.0);
    else if (mode == "demo") demoLoads();
    else if (mode == "hillbrake") hillBrake();
    else if (mode == "susp") susp(argc > 2 ? std::atoi(argv[2]) : 2, argc > 3 ? float(std::atof(argv[3])) : 0.4f,
                                  argc > 4 ? std::atoi(argv[4]) != 0 : true, argc > 5 ? std::atof(argv[5]) : 10.0,
                                  argc > 6 ? float(std::atof(argv[6])) : 0.0f);
    else if (mode == "corner") corner(TrackLayout::Flat, argc > 2 ? float(std::atof(argv[2])) : -0.45f, 4.0,
                                         argc > 3 ? std::atoi(argv[3]) : 1);
    else if (mode == "skid") skid();
    else if (mode == "squeal") squealCases();
    else if (mode == "keyboard") keyboardPedals();
    else if (mode == "launchmarks") launchMarks();
    else if (mode == "showcase") showcase();
    else if (mode == "surfaces") surfaces();
    else if (mode == "abs") absProbe(argc > 2 ? std::atoi(argv[2]) : -1);
    else if (mode == "brakedetail") { brakeDetail(0); brakeDetail(1); }
    else if (mode == "surfacecmp") surfaceCompare();
    else if (mode == "ellipse") ellipseProbe();
    else if (mode == "tc") tcProbe();
    else if (mode == "tcmud") tcMudSweep();
    else if (mode == "diff") diffProbe();
    else if (mode == "layouts") layoutsProbe();
    else if (mode == "compare") compareProbe();
    else if (mode == "minitrack") miniTrackProbe(argc, argv);
    else if (mode == "drift") driftProbe(argc, argv);
    else if (mode == "tires") tiresProbe(argc, argv);
    else if (mode == "bump") bumpProbe(argc, argv);
    else if (mode == "hill") hillProbe(argc, argv);
    else if (mode == "drop") dropProbe(argc, argv);
    else if (mode == "handling") handlingProbe(argc, argv);
    else if (mode == "climb") climbTrace(argc > 2 ? std::atoi(argv[2]) : 9);
    else if (mode == "launch") launchTrace(argc > 2 ? std::atoi(argv[2]) : 0, argc > 3 ? float(std::atof(argv[3])) : 1.0f);
    else if (mode == "water") waterProbe(argc, argv);
    else bench();
    return 0;
}
