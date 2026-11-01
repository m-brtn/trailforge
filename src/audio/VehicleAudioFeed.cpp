#include "audio/VehicleAudioFeed.h"

#include "audio/AudioMapping.h"
#include "sim/Simulation.h"

#include <algorithm>
#include <cmath>

namespace tf {
namespace {

AudioVec3 toAudio(JPH::Vec3Arg v) { return {v.GetX(), v.GetY(), v.GetZ()}; }

} // namespace

void VehicleAudioFeed::onTick(const Simulation& sim) {
    const AudioFeedSettings& s = m_settings;
    const Vehicle& vehicle = sim.vehicle();
    const VehicleTelemetry& t = vehicle.telemetry();
    const VehicleConfig& cfg = vehicle.config();
    const auto& contacts = vehicle.contacts();
    m_time = sim.simTime();

    const Pose& body = sim.currentSnapshot().body;
    AudioFrame& f = m_frame;
    f.vehiclePos = toAudio(body.position);
    f.vehicleForward = toAudio(body.rotation.RotateAxisZ());
    f.vehicleLeft = toAudio(body.rotation.RotateAxisX());
    f.vehicleUp = toAudio(body.rotation.RotateAxisY());
    f.speed = t.speed;
    f.rpm = t.engineRPM;
    f.throttle = t.command.throttle;
    f.clutch = t.clutchFriction;
    f.gear = t.gear;
    f.referenceLoad = 0.25f * cfg.mass * 9.81f;
    f.tireSound = cfg.tireSound;
    f.bodySound = cfg.bodySound;
    float wheelOmega = 0.0f;
    for (const auto& w : t.wheels) wheelOmega += 0.25f * w.angularVelocity;
    const float finalDrive = cfg.tuning.finalDrive * (t.lowRange ? cfg.lowRangeRatio : 1.0f);
    f.driveshaftHz = std::abs(wheelOmega) * finalDrive / 6.2831853f;
    f.electric = cfg.isElectric();
    if (f.electric) {
        f.driveshaftHz = 0.0f; // no propeller shaft; the reduction gears are part of the motor tones
        for (int m = 0; m < 2; ++m) {
            MotorAudio& ma = f.motors[std::size_t(m)];
            ma.rpm = t.ev.motorRPM[std::size_t(m)];
            ma.torque = t.ev.motorTorque[std::size_t(m)];
            ma.maxTorque = cfg.electric.motors[std::size_t(m)].maxTorque;
        }
    }
    f.vehicleValid = true;

    const WaterSurface& water = sim.waterSurface();
    const bool newSpawn = !m_seen || sim.spawnCount() != m_spawnCount;
    const float dt = float(sim.fixedStep());
    const auto smoothstep = [](float a, float b, float x) {
        const float u = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
        return u * u * (3.0f - 2.0f * u);
    };
    const float upright = smoothstep(s.squealUprightMin, s.squealUprightFull, f.vehicleUp.y);
    for (int i = 0; i < kAudioWheels; ++i) {
        const WheelContactSample& c = contacts[std::size_t(i)];
        const WheelTelemetry& wt = t.wheels[std::size_t(i)];
        WheelAudio& w = f.wheels[std::size_t(i)];
        w.contact = c.contact;
        w.surface = std::uint8_t(c.surface);
        w.load = c.contact ? c.load : 0.0f;
        const float patchSpeed = c.velocity.Length();
        w.rollSpeed = c.contact ? patchSpeed : 0.0f;
        w.slipSpeed = c.contact ? c.slipSpeed() : 0.0f;
        w.slipRatio = w.slipSpeed / c.slipReference();
        w.longRatio = c.contact ? c.longSlipRatio() : 0.0f;
        w.latRatio = c.contact ? c.latSlipRatio() : 0.0f;
        w.position = toAudio(c.contact ? c.patchCenter : wt.suspensionTop + wt.suspensionDir * wt.suspensionLength);
        const JPH::Vec3 p = c.patchCenter;
        w.inWater = c.contact && underWater(water, p, 0.02f);

        const std::size_t k = std::size_t(i);
        if (newSpawn) {
            m_soil[k] = {};
            m_squealEnv[k] = 0.0f;
            m_squealOpen[k] = false;
            m_airTime[k] = 0.0f;
            m_sinceLanding[k] = 1e3f;
        }
        m_soil[k].update(c, dt);
        w.soil = m_soil[k].amount;
        if (c.contact) {
            if (m_airTime[k] >= s.squealAirMin) m_sinceLanding[k] = 0.0f;
            m_airTime[k] = 0.0f;
            m_sinceLanding[k] = std::min(m_sinceLanding[k] + dt, 1e3f);
        } else {
            m_airTime[k] += dt;
        }
        const float landed = smoothstep(s.squealLandedMin, s.squealLandedFull, m_sinceLanding[k]);
        const float drive = wheelSquealDrive(w, f.referenceLoad) * upright * landed;
        const bool hard = c.contact && squealSurface(c.surface) > 0.0f;
        float& env = m_squealEnv[k];
        const float tau = drive > env ? s.squealAttack : (hard ? s.squealRelease : s.squealReleaseLoose);
        env += (drive - env) * (1.0f - std::exp(-dt / tau));
        if (env > s.squealOn) m_squealOpen[k] = true;
        else if (env < s.squealOff) m_squealOpen[k] = false;
        w.squeal = m_squealOpen[k] ? (env - s.squealOff) / (1.0f - s.squealOff) : 0.0f;
    }

    if (newSpawn) {
        m_seen = true;
        m_spawnCount = sim.spawnCount();
        if (!f.electric) {
            AudioEvent e;
            e.type = AudioEventType::EngineStart;
            e.position = f.vehiclePos + f.vehicleForward * 1.6f;
            m_events.push_back(e);
        }
        m_gear = t.gear;
        for (int i = 0; i < kAudioWheels; ++i) {
            m_prevLength[std::size_t(i)] = t.wheels[std::size_t(i)].suspensionLength;
            m_prevBumpStop[std::size_t(i)] = t.wheels[std::size_t(i)].bumpStop;
            m_prevWater[std::size_t(i)] = f.wheels[std::size_t(i)].inWater;
        }
        m_prevVelocity = t.velocity;
        m_feltG = 1.0f;
        m_rattleArmed = true;
        m_rattlePending = false;
        return;
    }

    // Gear engaged (shifts into neutral while braking to a stop are silent; an electric drive has no gears).
    if (!f.electric && t.gear != m_gear && t.gear != 0 && m_time - m_lastShift > 0.25) {
        AudioEvent e;
        e.type = AudioEventType::GearShift;
        e.intensity = m_gear == 0 ? 0.6f : 1.0f;
        e.position = f.vehiclePos + f.vehicleUp * -0.3f + f.vehicleForward * 0.3f;
        m_events.push_back(e);
        m_lastShift = m_time;
    }
    m_gear = t.gear;

    // Bump-stop knock: the tick a suspension reaches its compression limit, if it was still compressing fast.
    for (int i = 0; i < kAudioWheels; ++i) {
        const std::size_t k = std::size_t(i);
        const WheelAudio& w = f.wheels[k];
        const WheelTelemetry& wt = t.wheels[k];
        const float compressionSpeed = (m_prevLength[k] - wt.suspensionLength) / dt;
        m_prevLength[k] = wt.suspensionLength;
        const bool bump = wt.bumpStop && w.contact;
        if (bump && !m_prevBumpStop[k] && compressionSpeed >= s.bumpStopMinSpeed &&
            m_time - m_lastBump[k] > s.bumpStopInterval && m_time - m_lastKnock > s.bumpStopGap) {
            AudioEvent e;
            e.type = AudioEventType::BumpStop;
            e.intensity = std::clamp((compressionSpeed - s.bumpStopMinSpeed) / (s.bumpStopFullSpeed - s.bumpStopMinSpeed),
                                     0.0f, 1.0f);
            e.position = toAudio(wt.suspensionTop);
            m_events.push_back(e);
            m_lastBump[k] = m_time;
            m_lastKnock = m_time;
        }
        if (w.inWater && !m_prevWater[k] && w.rollSpeed > s.splashMinSpeed && m_time - m_lastSplash[k] > 0.3) {
            AudioEvent e;
            e.type = AudioEventType::Splash;
            e.intensity = std::clamp(w.rollSpeed / 8.0f, 0.2f, 1.0f);
            e.position = w.position;
            m_events.push_back(e);
            m_lastSplash[k] = m_time;
        }
        m_prevBumpStop[k] = bump;
        m_prevWater[k] = w.inWater;
    }

    // Body rattle: felt vertical acceleration along the body's up axis (velocity change of the center of mass plus
    // gravity), from the solver's velocities of consecutive ticks.
    {
        const JPH::Vec3 up = body.rotation.RotateAxisY();
        const float felt = ((t.velocity - m_prevVelocity) / dt + JPH::Vec3(0.0f, 9.81f, 0.0f)).Dot(up) / 9.81f;
        m_prevVelocity = t.velocity;
        m_feltG += (felt - m_feltG) * (1.0f - std::exp(-dt / s.rattleSmoothing));
        if (m_rattleArmed && m_feltG > s.rattleOnG && m_time - m_lastRattle > s.rattleInterval) {
            m_rattleArmed = false;
            m_rattlePending = true;
            m_rattleStart = m_time;
            m_rattlePeak = m_feltG;
        }
        // The event waits for the peak of the jolt (at most rattlePeakWait) so its intensity is the peak's.
        if (m_rattlePending) {
            m_rattlePeak = std::max(m_rattlePeak, m_feltG);
            if (m_feltG < m_rattlePeak * 0.97f || m_time - m_rattleStart >= s.rattlePeakWait) {
                AudioEvent e;
                e.type = AudioEventType::BodyRattle;
                e.intensity = std::clamp((m_rattlePeak - s.rattleOnG) / (s.rattleFullG - s.rattleOnG), 0.0f, 1.0f);
                e.position = f.vehiclePos;
                m_events.push_back(e);
                m_rattlePending = false;
                m_lastRattle = m_rattleStart;
            }
        }
        if (!m_rattlePending && m_feltG < s.rattleRearmG) m_rattleArmed = true;
    }

    // Strongest body contact of the tick.
    const BodyImpact* strongest = nullptr;
    for (const BodyImpact& b : sim.bodyImpacts())
        if (!strongest || b.impulse > strongest->impulse) strongest = &b;
    if (strongest && strongest->impulse > s.impactMinImpulse && m_time - m_lastImpact > s.impactInterval) {
        AudioEvent e;
        e.type = AudioEventType::BodyImpact;
        e.intensity = std::clamp(strongest->impulse / s.impactFullImpulse, 0.1f, 1.0f);
        e.position = toAudio(strongest->position);
        m_events.push_back(e);
        m_lastImpact = m_time;
    }

    // Articulation: diagonal difference of the suspension compression (twisting the frame).
    const auto cr = [&](int i) { return t.wheels[std::size_t(i)].compressionRatio; };
    const float articulation = std::abs((cr(0) - cr(1)) - (cr(2) - cr(3)));
    int grounded = 0;
    for (const WheelAudio& w : f.wheels) grounded += w.contact ? 1 : 0;
    if (articulation > s.creakArticulation && std::abs(articulation - m_prevArticulation) > 1e-4f &&
        t.speed < 6.0f && f.vehicleUp.y > s.creakUprightMin && grounded >= s.creakMinWheels &&
        m_time - m_lastCreak > s.creakInterval) {
        AudioEvent e;
        e.type = AudioEventType::ChassisCreak;
        e.intensity = std::clamp((articulation - s.creakArticulation) / 0.5f + 0.4f, 0.4f, 1.0f);
        e.position = f.vehiclePos;
        m_events.push_back(e);
        m_lastCreak = m_time;
    }
    m_prevArticulation = articulation;
}

} // namespace tf
