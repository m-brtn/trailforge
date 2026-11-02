// trailforge_probe handling [section] - vehicle handling measurements on the flat pad (see tools/SimProbe.cpp).
//   ramp     quasi-steady steer ramp at 60 km/h: steer, body slip angle (beta), axle slip angles, yaw rate and the
//            understeer angle at 0.1 .. 0.9 g; understeer gradient; axle cornering stiffness (Fy / Fz per degree)
//   step     step steer to the 0.4 g steer of the ramp at 60 / 90 km/h: yaw rate 50 / 90 % times, overshoot, beta
//   sine     one-period 0.5 Hz sine steer (lane change) at 60 / 90 km/h, 0.4 g amplitude: peak yaw rate, beta, lag
//   straight 90 km/h constant throttle 6 s, then lift-off 3 s: heading drift, beta, yaw rate
//   arb      steer ramp with the front or rear anti-roll bar raised: understeer gradient per setting
//   slope    2 deg cross slope (gravity tilted): parked creep with the brake, launch at 0.45 throttle (heading change)
//   user30   the steering key held 4 s at ~30 km/h (coast, W pressed with the turn, W held from 15 km/h) with the
//            KeyboardPedals ramp, TC off / on / off-road, locks 0 / 3: max body slip, max rear slip angle against the
//            tire's peak on that surface (STEP-OUT beyond 1.5x), per-wheel means; HANDLING_TRACE=1 per-tick trace,
//            HANDLING_LOCKS=n only that lock setting
//   all      everything above
// Surface: HANDLING_SURFACE=asphalt|gravel|grass|packed_dirt|concrete (default asphalt) replaces the pad's concrete
// (the tire set's per-surface grip of that surface id is not applied: the shipped off-road sets have 1.0 everywhere).
// Vehicles: TRAILFORGE_VEHICLE, else the three shipped vehicles. HANDLING_AIDS=user: ABS on, off-road TC.

#include "input/KeyboardPedals.h"
#include "sim/Simulation.h"
#include "vehicle/VehicleDefinition.h"

#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace tf;

namespace {

constexpr float kRad = 57.2957795f;
constexpr float kG = 9.81f;

struct NamedVehicle {
    std::string id;
    VehicleConfig config;
};

std::vector<NamedVehicle> handlingVehicles() {
    std::vector<std::string> paths;
    if (const char* env = std::getenv("TRAILFORGE_VEHICLE")) paths.push_back(env);
    else
        for (const char* id : {"trailforge_suv", "trailforge_ev", "trailforge_patrol"})
            paths.push_back(std::string(TRAILFORGE_SOURCE_DIR "/assets/vehicles/") + id + "/vehicle.json");
    std::vector<NamedVehicle> out;
    for (const auto& p : paths) {
        if (p == "builtin") {
            out.push_back({"builtin", VehicleConfig{}});
            continue;
        }
        VehicleDefinition def;
        std::vector<std::string> errors;
        if (!loadVehicleDefinition(p, def, errors)) {
            for (const auto& e : errors) std::fprintf(stderr, "%s\n", e.c_str());
            std::exit(2);
        }
        out.push_back({def.id, def.config});
    }
    if (const char* e = std::getenv("HANDLING_ELLIPSE"))
        for (auto& v : out) v.config.tuning.tireFrictionEllipse = std::atoi(e) != 0;
    return out;
}

SurfaceId handlingSurface() {
    const char* env = std::getenv("HANDLING_SURFACE");
    if (!env) return SurfaceId::Asphalt;
    const SurfaceId id = surfaceFromKey(env);
    return id == SurfaceId::Count ? SurfaceId::Asphalt : id;
}

// One tick of measurements in the body frame. Turn direction normalized: a right turn (steer > 0) counts positive.
struct Sample {
    float t = 0, speed = 0, steerDeg = 0, ay = 0, yawRate = 0, beta = 0, alphaF = 0, alphaR = 0;
    float fyF = 0, fyR = 0, fzF = 0, fzR = 0; // axle lateral forces (toward the turn center) and loads, N
    float heading = 0;                        // deg, world yaw (left positive)
    float x = 0, z = 0;
};

class Rig {
public:
    Rig(const VehicleConfig& cfg, SurfaceId surface, float gravityTiltDeg = 0.0f) {
        SimulationSettings s;
        s.vehicle = cfg;
        s.layout = TrackLayout::Flat;
        m_sim = std::make_unique<Simulation>(s);
        m_sim->surfaces().get(SurfaceId::Concrete) = m_sim->surfaces().get(surface);
        if (gravityTiltDeg != 0.0f) {
            // Cross slope: gravity tilted toward +X (the vehicle's left when it faces +Z) is a slope falling to the left.
            const float a = gravityTiltDeg / kRad;
            m_sim->world().system().SetGravity(JPH::Vec3(kG * std::sin(a), -kG * std::cos(a), 0.0f));
        }
        if (const char* aids = std::getenv("HANDLING_AIDS"); aids && std::string(aids) == "user")
            m_sim->requestAidControls({true, TractionMode::OffRoad, 0});
    }
    Simulation& sim() { return *m_sim; }

    void place(float x, float z, float yawRad) {
        TrackFeature f;
        f.name = "probe";
        f.position = JPH::Vec3(x, 0.0f, z);
        f.yaw = yawRad;
        m_sim->requestPlace(f);
        m_sim->setIntent({.brake = 1.0f});
        for (int i = 0; i < 120; ++i) m_sim->tick();
        m_prevV = velocity();
        m_t = 0.0f;
    }

    JPH::Vec3 velocity() { return m_sim->world().bodies().GetLinearVelocity(m_sim->vehicle().bodyId()); }

    // Throttle for a target speed (PI), steer as an analog stick.
    void drive(float targetSpeed, float steer, float throttleOverride = -1.0f) {
        const float v = m_sim->vehicle().telemetry().forwardSpeed;
        const float err = targetSpeed - v;
        m_integral = std::clamp(m_integral + err * float(m_sim->fixedStep()), -20.0f, 20.0f);
        float thr = std::clamp(0.1f + 0.3f * err + 0.03f * m_integral, 0.0f, 1.0f);
        if (throttleOverride >= 0.0f) thr = throttleOverride;
        m_sim->setIntent({.throttle = thr, .steer = steer, .analog = true});
        tick();
    }

    void tick() {
        m_sim->tick();
        m_t += float(m_sim->fixedStep());
        const JPH::Vec3 v = velocity();
        const JPH::Vec3 acc = (v - m_prevV) / float(m_sim->fixedStep());
        m_prevV = v;
        const JPH::RMat44 xf = m_sim->vehicle().bodyTransform();
        const JPH::Vec3 fwd = xf.GetAxisZ(), left = xf.GetAxisX(), up = xf.GetAxisY();
        const JPH::Vec3 w = m_sim->world().bodies().GetAngularVelocity(m_sim->vehicle().bodyId());
        const auto& tel = m_sim->vehicle().telemetry();
        const float dir = m_turnSign;
        Sample s;
        s.t = m_t;
        s.speed = v.Length();
        // Commanded road-wheel angle (the driver's input; the front compliance steer is part of the vehicle's response).
        s.steerDeg = -dir * (tel.steerAngleDeg - tel.complianceSteerDeg[0]); // telemetry: + = left
        s.ay = -dir * acc.Dot(left);
        s.yawRate = -dir * w.Dot(up) * kRad;
        const float vx = v.Dot(fwd), vy = v.Dot(left);
        s.beta = std::abs(vx) > 0.5f ? -dir * std::atan2(vy, std::abs(vx)) * kRad : 0.0f;
        float aF = 0, aR = 0;
        int nF = 0, nR = 0;
        for (int i = 0; i < kWheelCount; ++i) {
            const WheelTelemetry& wt = tel.wheels[std::size_t(i)];
            if (!wt.contact) continue;
            const JPH::Vec3 pv = m_sim->world().bodies().GetPointVelocity(m_sim->vehicle().bodyId(), JPH::RVec3(wt.contactPosition));
            const float lon = pv.Dot(wt.contactLongitudinal), lat = pv.Dot(wt.contactLateral);
            // Slip angle sign: the contact patch velocity pointing out of the turn is positive.
            const float latLeft = wt.contactLateral.Dot(left) >= 0.0f ? lat : -lat;
            const float a = std::abs(lon) > 0.3f ? dir * std::atan2(latLeft, std::abs(lon)) * kRad : 0.0f;
            // Lateral force on the body toward the turn center: Jolt applies -lambda along -lateral, i.e. +lateral.
            const float fy = -dir * wt.lateralForce * wt.contactLateral.Dot(left);
            if (i < 2) aF += a, ++nF, s.fyF += fy, s.fzF += wt.load;
            else aR += a, ++nR, s.fyR += fy, s.fzR += wt.load;
        }
        s.alphaF = nF ? aF / float(nF) : 0.0f;
        s.alphaR = nR ? aR / float(nR) : 0.0f;
        s.heading = std::atan2(fwd.GetX(), fwd.GetZ()) * kRad;
        const JPH::Vec3 p(xf.GetTranslation());
        s.x = p.GetX();
        s.z = p.GetZ();
        last = s;
    }

    void setTurnSign(float s) { m_turnSign = s; }
    Sample last;

private:
    std::unique_ptr<Simulation> m_sim;
    JPH::Vec3 m_prevV = JPH::Vec3::sZero();
    float m_t = 0.0f, m_integral = 0.0f, m_turnSign = 1.0f;
};

// Moving average over a window of samples.
struct Avg {
    std::vector<Sample> buf;
    std::size_t n;
    explicit Avg(std::size_t window) : n(window) {}
    void add(const Sample& s) {
        buf.push_back(s);
        if (buf.size() > n) buf.erase(buf.begin());
    }
    Sample mean() const {
        Sample m;
        for (const auto& s : buf) {
            m.speed += s.speed, m.steerDeg += s.steerDeg, m.ay += s.ay, m.yawRate += s.yawRate, m.beta += s.beta;
            m.alphaF += s.alphaF, m.alphaR += s.alphaR, m.fyF += s.fyF, m.fyR += s.fyR, m.fzF += s.fzF, m.fzR += s.fzR;
        }
        const float k = buf.empty() ? 0.0f : 1.0f / float(buf.size());
        m.speed *= k, m.steerDeg *= k, m.ay *= k, m.yawRate *= k, m.beta *= k, m.alphaF *= k, m.alphaR *= k;
        m.fyF *= k, m.fyR *= k, m.fzF *= k, m.fzR *= k;
        if (!buf.empty()) m.t = buf.back().t;
        return m;
    }
};

struct RampResult {
    float steerAt[10]{};  // steer fraction of full lock at 0.1 .. 0.9 g (index = g x 10)
    float maxG = 0.0f;
    float gradient = 0.0f; // deg/g understeer gradient over 0.1-0.4 g
};

const char* surfaceName(SurfaceId s) { return surfaceKey(s); }

// Quasi-steady steer ramp at a constant speed: the road wheel angle rises at 0.5 deg/s.
RampResult steerRamp(const NamedVehicle& v, SurfaceId surface, float kmh, bool print, const char* label = nullptr) {
    Rig rig(v.config, surface);
    rig.place(0.0f, -200.0f, 0.0f);
    const float target = kmh / 3.6f;
    for (int i = 0; i < int(12.0 / rig.sim().fixedStep()); ++i) rig.drive(target, 0.0f);
    const float maxDeg = v.config.axles[kFrontAxle].maxSteerAngleDeg;
    const float rate = 0.5f / maxDeg; // steer fraction per second
    Avg avg(std::size_t(0.5 / rig.sim().fixedStep()));
    RampResult r;
    float steer = 0.0f;
    int nextG = 1;
    float k0 = 0.0f;
    float bestG = 0.0f;
    std::vector<std::pair<float, float>> us; // (ay g, understeer angle deg) for the gradient
    if (print)
        std::printf("  %-6s ay g | steer deg  beta deg  alphaF  alphaR  yaw deg/s  under deg | CF Fy/Fz/deg  CR | speed\n",
                    label ? label : "");
    while (steer < 0.9f) {
        steer += rate * float(rig.sim().fixedStep());
        rig.drive(target, steer);
        avg.add(rig.last);
        const Sample m = avg.mean();
        const float g = m.ay / kG;
        if (g > bestG) bestG = g;
        if (std::abs(m.beta) > 20.0f || (g < bestG - 0.05f && bestG > 0.2f)) break;
        // Understeer angle: road wheel angle minus the kinematic angle L / R, R = v / yaw rate.
        const float ack = std::atan(v.config.wheelBase * m.yawRate / kRad / std::max(m.speed, 1.0f)) * kRad;
        const float under = m.steerDeg - ack;
        if (g > 0.05f && g < 0.45f) us.push_back({g, under});
        if (nextG < 10 && g >= 0.1f * float(nextG)) {
            r.steerAt[nextG] = steer;
            const float cf = m.fzF > 0 && std::abs(m.alphaF) > 0.05f ? m.fyF / m.fzF / m.alphaF : 0.0f;
            const float cr = m.fzR > 0 && std::abs(m.alphaR) > 0.05f ? m.fyR / m.fzR / m.alphaR : 0.0f;
            if (nextG == 2) k0 = cf;
            if (print)
                std::printf("  %-6s %.2f | %8.2f  %8.2f  %6.2f  %6.2f  %9.2f  %9.2f | %8.3f  %6.3f | %5.1f\n", "", g,
                            m.steerDeg, m.beta, m.alphaF, m.alphaR, m.yawRate, under, cf, cr, m.speed * 3.6f);
            ++nextG;
        }
    }
    (void)k0;
    r.maxG = bestG;
    if (us.size() > 10) {
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (const auto& [x, y] : us) sx += x, sy += y, sxx += x * x, sxy += x * y;
        const double n = double(us.size());
        r.gradient = float((n * sxy - sx * sy) / (n * sxx - sx * sx));
    }
    if (print)
        std::printf("  %-6s max %.2f g, understeer gradient %.2f deg/g (0.05-0.45 g)\n", label ? label : "", r.maxG, r.gradient);
    return r;
}

float steerFor(const RampResult& r, float g) {
    const int i = std::clamp(int(std::lround(g * 10.0f)), 1, 9);
    return r.steerAt[i];
}

void stepSteer(const NamedVehicle& v, SurfaceId surface, float kmh, float steer) {
    Rig rig(v.config, surface);
    rig.place(0.0f, -230.0f, 0.0f);
    const float target = kmh / 3.6f;
    for (int i = 0; i < int(14.0 / rig.sim().fixedStep()); ++i) rig.drive(target, 0.0f);
    std::vector<Sample> trace;
    const float t0 = rig.last.t;
    // Throttle held at the straight-line value: no speed control in the transient (ISO 7401 style).
    float thr = rig.sim().vehicle().telemetry().command.throttle;
    for (int i = 0; i < int(4.0 / rig.sim().fixedStep()); ++i) {
        rig.drive(target, steer, thr);
        Sample s = rig.last;
        s.t -= t0;
        trace.push_back(s);
    }
    // Steady state = mean over the last 1 s.
    float rSteady = 0, aySteady = 0, bSteady = 0;
    int n = 0;
    for (const auto& s : trace)
        if (s.t > 3.0f) rSteady += s.yawRate, aySteady += s.ay, bSteady += s.beta, ++n;
    rSteady /= float(n), aySteady /= float(n), bSteady /= float(n);
    float steerT = -1, r50 = -1, r90 = -1, rPeak = 0, tPeak = 0, ay90 = -1, bPeak = 0, settle = 0;
    for (const auto& s : trace) {
        if (steerT < 0 && s.steerDeg >= 0.5f * steer * v.config.axles[0].maxSteerAngleDeg) steerT = s.t;
        if (r50 < 0 && s.yawRate >= 0.5f * rSteady) r50 = s.t;
        if (r90 < 0 && s.yawRate >= 0.9f * rSteady) r90 = s.t;
        if (ay90 < 0 && s.ay >= 0.9f * aySteady) ay90 = s.t;
        if (s.yawRate > rPeak) rPeak = s.yawRate, tPeak = s.t;
        if (std::abs(s.beta) > std::abs(bPeak)) bPeak = s.beta;
        if (std::abs(s.yawRate - rSteady) > 0.05f * std::abs(rSteady)) settle = s.t;
    }
    std::printf("  step %3.0f km/h steer %.3f (%.1f deg): yaw rate steady %.1f deg/s, 50%% %.3f s, 90%% %.3f s (from 50%% "
                "steer %.3f s), peak %.1f at %.2f s, overshoot %.0f%%, settle 5%% %.2f s; ay steady %.2f g, 90%% %.3f s; "
                "beta steady %.2f peak %.2f deg\n",
                kmh, steer, steer * v.config.axles[0].maxSteerAngleDeg, rSteady, r50 - steerT, r90 - steerT, steerT, rPeak,
                tPeak, 100.0f * (rPeak / rSteady - 1.0f), settle, aySteady / kG, ay90 - steerT, bSteady, bPeak);
}

void sineSteer(const NamedVehicle& v, SurfaceId surface, float kmh, float amp, float hz, bool trace) {
    Rig rig(v.config, surface);
    rig.place(0.0f, -230.0f, 0.0f);
    const float target = kmh / 3.6f;
    for (int i = 0; i < int(14.0 / rig.sim().fixedStep()); ++i) rig.drive(target, 0.0f);
    const float thr = rig.sim().vehicle().telemetry().command.throttle;
    const float t0 = rig.last.t, h0 = rig.last.heading;
    float rMax = 0, rMin = 0, bMax = 0, ayMax = 0, tSteerPk = 0, tYawPk = 0, steerPk = 0, after = 0, headAfter = 0;
    const float period = 1.0f / hz;
    for (int i = 0; i < int((period + 3.0f) / rig.sim().fixedStep()); ++i) {
        const float t = rig.last.t - t0;
        const float steer = t < period ? amp * std::sin(6.2831853f * hz * t) : 0.0f;
        rig.drive(target, steer, thr);
        const Sample& s = rig.last;
        const float ts = s.t - t0;
        if (s.steerDeg > steerPk) steerPk = s.steerDeg, tSteerPk = ts;
        if (s.yawRate > rMax) rMax = s.yawRate, tYawPk = ts;
        rMin = std::min(rMin, s.yawRate);
        bMax = std::max(bMax, std::abs(s.beta));
        ayMax = std::max(ayMax, std::abs(s.ay));
        if (ts > period + 1.0f && ts < period + 1.0f + 1.0f / 120.0f) after = s.yawRate;
        headAfter = s.heading - h0;
        if (trace && i % 6 == 0)
            std::printf("    t %.2f steer %+6.2f yaw %+6.2f beta %+6.2f ay %+5.2f g aF %+5.2f aR %+5.2f\n", ts, s.steerDeg,
                        s.yawRate, s.beta, s.ay / kG, s.alphaF, s.alphaR);
    }
    std::printf("  sine %3.0f km/h %.2f Hz amp %.3f: yaw rate +%.1f / %.1f deg/s, yaw lag %.3f s, max |beta| %.2f deg, "
                "max |ay| %.2f g; 1 s after: yaw rate %.2f deg/s, heading change %.2f deg\n",
                kmh, hz, amp, rMax, rMin, tYawPk - tSteerPk, bMax, ayMax / kG, after, headAfter);
}

void straightRun(const NamedVehicle& v, SurfaceId surface) {
    Rig rig(v.config, surface);
    rig.place(0.0f, -240.0f, 0.0f);
    for (int i = 0; i < int(14.0 / rig.sim().fixedStep()); ++i) rig.drive(90.0f / 3.6f, 0.0f);
    const float h0 = rig.last.heading, x0 = rig.last.x;
    float rMax = 0, bMax = 0;
    const float thr = rig.sim().vehicle().telemetry().command.throttle;
    for (int i = 0; i < int(5.0 / rig.sim().fixedStep()); ++i) {
        rig.drive(0, 0.0f, thr);
        rMax = std::max(rMax, std::abs(rig.last.yawRate));
        bMax = std::max(bMax, std::abs(rig.last.beta));
    }
    std::printf("  straight 90 km/h, throttle %.2f held 5 s: heading %+.3f deg, x drift %+.3f m, max |yaw rate| %.3f deg/s, "
                "max |beta| %.3f deg\n",
                thr, rig.last.heading - h0, rig.last.x - x0, rMax, bMax);
    const float h1 = rig.last.heading, x1 = rig.last.x;
    rMax = bMax = 0;
    for (int i = 0; i < int(3.0 / rig.sim().fixedStep()); ++i) {
        rig.drive(0, 0.0f, 0.0f);
        rMax = std::max(rMax, std::abs(rig.last.yawRate));
        bMax = std::max(bMax, std::abs(rig.last.beta));
    }
    std::printf("  lift-off 3 s: heading %+.3f deg, x drift %+.3f m, max |yaw rate| %.3f deg/s, max |beta| %.3f deg, "
                "%.0f km/h\n",
                rig.last.heading - h1, rig.last.x - x1, rMax, bMax, rig.last.speed * 3.6f);
}

// Throttle or brake applied in a steady corner (0.5 g at 60 km/h): yaw rate and body slip angle over 2 s.
void combinedRuns(const NamedVehicle& v, SurfaceId surface, const RampResult& ramp, float g) {
    for (int kind = 0; kind < 2; ++kind)
        for (float pedal : {0.3f, 0.6f, 1.0f}) {
            Rig rig(v.config, surface);
            rig.place(-60.0f, -200.0f, 0.0f);
            const float target = 60.0f / 3.6f;
            for (int i = 0; i < int(10.0 / rig.sim().fixedStep()); ++i) rig.drive(target, 0.0f);
            const float steer = steerFor(ramp, g);
            for (int i = 0; i < int(5.0 / rig.sim().fixedStep()); ++i) rig.drive(target, steer);
            const Sample s0 = rig.last;
            float bMax = 0, rMax = 0, rMin = 1e9f, ayMin = 1e9f;
            bool spun = false;
            for (int i = 0; i < int(2.0 / rig.sim().fixedStep()); ++i) {
                if (kind == 0) rig.sim().setIntent({.throttle = pedal, .steer = steer, .analog = true});
                else rig.sim().setIntent({.brake = pedal, .steer = steer, .analog = true});
                rig.tick();
                const Sample& s = rig.last;
                if (std::abs(s.beta) > std::abs(bMax)) bMax = s.beta;
                rMax = std::max(rMax, s.yawRate);
                rMin = std::min(rMin, s.yawRate);
                ayMin = std::min(ayMin, s.ay);
                spun = spun || std::abs(s.beta) > 20.0f;
            }
            std::printf("  %s %.1f in a %.1f g corner: beta %.2f -> max %.2f deg, yaw rate %.1f -> %.1f..%.1f deg/s, "
                        "%.0f km/h after 2 s%s\n",
                        kind == 0 ? "throttle" : "brake   ", pedal, s0.ay / kG, s0.beta, bMax, s0.yawRate, rMin, rMax,
                        rig.last.speed * 3.6f, spun ? " SPIN" : "");
        }
}

// The user's report: keyboard, 2H, ABS + TC + lockers, dirt, ~30 km/h. A steering key is held from 30 km/h; the
// throttle key (through KeyboardPedals, as in the app) is: 0 released (coast), 1 pressed together with the steering
// key, 2 held from 15 km/h (the steering key goes down when 30 km/h is passed). Per-wheel trace with HANDLING_TRACE=1.
// tc: 0 Off, 1 On, 2 OffRoad; locks 0..3.
void userTurn(const NamedVehicle& v, SurfaceId surface, int tc, int locks, int throttleKey, bool trace) {
    Rig rig(v.config, surface);
    rig.place(-60.0f, -200.0f, 0.0f);
    rig.sim().requestAidControls({true, tc == 0 ? TractionMode::Off : tc == 1 ? TractionMode::On : TractionMode::OffRoad, locks});
    rig.sim().tick();
    const float dt = rig.sim().fixedStep();
    const float target = 30.0f / 3.6f;
    KeyboardPedals pedals;
    if (throttleKey == 2) {
        for (int i = 0; i < int(8.0 / dt); ++i) rig.drive(15.0f / 3.6f, 0.0f);
        for (int i = 0; i < int(10.0 / dt) && rig.last.speed < target; ++i) {
            pedals.update(true, false, dt);
            rig.sim().setIntent({.throttle = pedals.throttle()});
            rig.tick();
        }
    } else {
        for (int i = 0; i < int(8.0 / dt); ++i) rig.drive(target, 0.0f);
    }
    const float entry = rig.last.speed;
    float bMax = 0, ayMax = 0, steerSum = 0, arMax = 0;
    int n = 0;
    struct W {
        double brake = 0, tc = 0, fx = 0, slip = 0, alpha = 0, load = 0;
    } w[4];
    const int ticks = int(4.0 / dt);
    for (int i = 0; i < ticks; ++i) {
        pedals.update(throttleKey != 0, false, dt);
        rig.sim().setIntent({.throttle = pedals.throttle(), .steer = 1.0f});
        rig.tick();
        const auto& tel = rig.sim().vehicle().telemetry();
        bMax = std::max(bMax, std::abs(rig.last.beta));
        ayMax = std::max(ayMax, std::abs(rig.last.ay));
        if (rig.last.speed > 2.0f) arMax = std::max(arMax, std::abs(rig.last.alphaR));
        steerSum += rig.last.steerDeg;
        ++n;
        for (int k = 0; k < 4; ++k) {
            const WheelTelemetry& wt = tel.wheels[std::size_t(k)];
            w[k].brake += wt.brakeTorque, w[k].tc += wt.tcBrake, w[k].fx += wt.longitudinalForce;
            w[k].slip += wt.longitudinalSlip, w[k].alpha += wt.lateralSlipDeg, w[k].load += wt.load;
        }
        if (trace && i % 6 == 0) {
            std::printf("    t %.2f v %4.1f steer %5.1f beta %+6.1f yaw %+6.1f thr %.2f tcf %.2f gear %d |", rig.last.t,
                        rig.last.speed * 3.6f, rig.last.steerDeg, rig.last.beta, rig.last.yawRate, tel.engineThrottle,
                        tel.throttleFactor, tel.gear);
            for (const auto& wt : tel.wheels)
                std::printf(" B%4.0f/%4.0f Fx%+5.0f s%.2f a%4.1f Fz%5.0f", wt.brakeTorque, wt.tcBrake,
                            wt.longitudinalForce, wt.longitudinalSlip, wt.lateralSlipDeg, wt.load);
            std::printf("\n");
        }
    }
    // Step-out: the rear past 1.5x the tire's peak slip angle on this surface (loose surfaces stretch the curve).
    const float peak = v.config.tuning.tirePeakSlipAngleDeg * rig.sim().surfaces().get(surface).peakSlipScale;
    static const char* kKey[] = {"coast", "W+turn", "W held"};
    std::printf("  user30 %-8s tc %d locks %d %-6s: entry %2.0f km/h, max |beta| %5.1f, max rear slip angle %5.1f deg (peak %4.1f), max ay %.2f g, mean steer %4.1f deg, end %2.0f km/h%s\n",
                surfaceName(surface), tc, locks, kKey[throttleKey], entry * 3.6f, bMax, arMax, peak, ayMax / kG,
                steerSum / float(n), rig.last.speed * 3.6f, arMax > 1.5f * peak ? "  STEP-OUT" : "");
    if (trace)
        for (int k = 0; k < 4; ++k)
            std::printf("     %s mean brake %5.0f (TC %5.0f) N*m, Fx %+6.0f N, slip %.3f, alpha %4.1f deg, load %5.0f N\n",
                        kWheelNames[k], w[k].brake / n, w[k].tc / n, w[k].fx / n, w[k].slip / n, w[k].alpha / n, w[k].load / n);
}

void slopeRuns(const NamedVehicle& v, SurfaceId surface, float deg) {
    for (float side : {1.0f, -1.0f}) {
        // Parked: brake held 10 s on the cross slope.
        Rig rig(v.config, surface, side * deg);
        rig.place(0.0f, -200.0f, 0.0f);
        const float x0 = rig.last.x, h0 = rig.last.heading;
        rig.sim().setIntent({.brake = 1.0f});
        for (int i = 0; i < int(10.0 / rig.sim().fixedStep()); ++i) rig.tick();
        std::printf("  slope %+.1f deg %s, brake held 10 s: x %+.4f m, heading %+.3f deg\n", side * deg,
                    surfaceName(surface), rig.last.x - x0, rig.last.heading - h0);
        // Launch from rest, throttle 0.45, no steering, 16 m.
        const float x1 = rig.last.x, z1 = rig.last.z, h1 = rig.last.heading;
        float tAt = 0;
        while (rig.last.z - z1 < 16.0f && rig.last.t < 40.0f) {
            rig.sim().setIntent({.throttle = 0.45f});
            rig.tick();
            tAt += float(rig.sim().fixedStep());
        }
        std::printf("  slope %+.1f deg %s, launch 0.45 throttle 16 m: heading %+.3f deg (+ = turned left), x %+.3f m, "
                    "%.1f km/h after %.2f s\n",
                    side * deg, surfaceName(surface), rig.last.heading - h1, rig.last.x - x1, rig.last.speed * 3.6f, tAt);
    }
}

} // namespace

void handlingProbe(int argc, char** argv) {
    const std::string section = argc > 2 ? argv[2] : "all";
    const SurfaceId surface = handlingSurface();
    for (const NamedVehicle& v : handlingVehicles()) {
        {
            Rig rig(v.config, surface);
            const JPH::BodyLockRead lock(rig.sim().world().system().GetBodyLockInterface(), rig.sim().vehicle().bodyId());
            const JPH::Mat44 inertia = lock.GetBody().GetMotionProperties()->GetLocalSpaceInverseInertia().Inversed3x3();
            const float a = v.config.wheelBase * (0.5f - v.config.comForward / v.config.wheelBase);
            const float b = v.config.wheelBase - a;
            std::printf("handling %s on %s (mass %.0f kg, wheelbase %.2f m, front lock %.0f deg, inertia pitch / yaw / roll "
                        "%.0f %.0f %.0f kg m^2, yaw dynamic index k^2/ab %.2f)\n",
                        v.id.c_str(), surfaceName(surface), v.config.mass, v.config.wheelBase,
                        v.config.axles[0].maxSteerAngleDeg, inertia(0, 0), inertia(1, 1), inertia(2, 2),
                        inertia(1, 1) / v.config.mass / (a * b));
        }
        RampResult ramp;
        const bool needRamp =
            section == "all" || section == "ramp" || section == "step" || section == "sine" || section == "combined";
        if (needRamp) ramp = steerRamp(v, surface, 60.0f, section == "all" || section == "ramp", "60");
        if (section == "all" || section == "step" || section == "sine") {
            const RampResult ramp90 = steerRamp(v, surface, 90.0f, section == "all", "90");
            for (float g : {0.4f}) {
                if (section != "sine") {
                    stepSteer(v, surface, 60.0f, steerFor(ramp, g));
                    stepSteer(v, surface, 90.0f, steerFor(ramp90, g));
                }
                if (section != "step") {
                    sineSteer(v, surface, 60.0f, steerFor(ramp, g), 0.5f, false);
                    sineSteer(v, surface, 90.0f, steerFor(ramp90, g), 0.5f, false);
                    sineSteer(v, surface, 90.0f, steerFor(ramp90, g), 1.0f, false);
                }
            }
        }
        if (section == "trace") {
            const RampResult r90 = steerRamp(v, surface, 90.0f, false);
            sineSteer(v, surface, 90.0f, steerFor(r90, 0.4f), 0.5f, true);
        }
        if (section == "all" || section == "combined") combinedRuns(v, surface, ramp, std::min(0.5f, 0.8f * ramp.maxG));
        if (section == "all" || section == "straight") straightRun(v, surface);
        if (section == "all" || section == "arb") {
            for (int which = 0; which < 3; ++which) {
                NamedVehicle m = v;
                const float base = std::max({v.config.tuning.axles[0].antiRoll, v.config.tuning.axles[1].antiRoll, 10000.0f});
                const char* label = which == 0 ? "base" : which == 1 ? "F-ARB+" : "R-ARB+";
                if (which == 1) m.config.tuning.axles[0].antiRoll += 2.0f * base;
                if (which == 2) m.config.tuning.axles[1].antiRoll += 2.0f * base;
                const RampResult r = steerRamp(m, surface, 60.0f, false);
                std::printf("  arb %-6s F %6.0f R %6.0f N/m: understeer gradient %.2f deg/g, max %.2f g, steer at 0.4 g "
                            "%.3f\n",
                            label, m.config.tuning.axles[0].antiRoll, m.config.tuning.axles[1].antiRoll, r.gradient, r.maxG,
                            steerFor(r, 0.4f));
            }
        }
        if (section == "all" || section == "slope") slopeRuns(v, surface, 2.0f);
        if (section == "user30") {
            const bool trace = std::getenv("HANDLING_TRACE") != nullptr;
            const int onlyLocks = std::getenv("HANDLING_LOCKS") ? std::atoi(std::getenv("HANDLING_LOCKS")) : -1;
            for (int tc : {0, 1, 2})
                for (int locks : {0, 3})
                    for (int key : {0, 1, 2})
                        if (onlyLocks < 0 || onlyLocks == locks) userTurn(v, surface, tc, locks, key, trace);
        }
    }
}
