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
    float wheelOmega = 0.0f;
    for (const auto& w : t.wheels) wheelOmega += 0.25f * w.angularVelocity;
    const float finalDrive = cfg.tuning.finalDrive * (t.lowRange ? cfg.lowRangeRatio : 1.0f);
    f.driveshaftHz = std::abs(wheelOmega) * finalDrive / 6.2831853f;
    f.vehicleValid = true;

    const WaterInfo water = TestTrack::water();
    const bool newSpawn = !m_seen || sim.spawnCount() != m_spawnCount;
    const float dt = float(sim.fixedStep());
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
        w.inWater = c.contact && p.GetY() < water.level - 0.02f && p.GetX() > water.x0 && p.GetX() < water.x1 &&
                    p.GetZ() > water.z0 && p.GetZ() < water.z1;

        const std::size_t k = std::size_t(i);
        if (newSpawn) {
            m_soil[k] = {};
            m_squealEnv[k] = 0.0f;
            m_squealOpen[k] = false;
        }
        m_soil[k].update(c, dt);
        w.soil = m_soil[k].amount;
        const float drive = wheelSquealDrive(w, f.referenceLoad);
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
        AudioEvent e;
        e.type = AudioEventType::EngineStart;
        e.position = f.vehiclePos + f.vehicleForward * 1.6f;
        m_events.push_back(e);
        m_gear = t.gear;
        for (int i = 0; i < kAudioWheels; ++i) {
            m_prevLoad[std::size_t(i)] = f.wheels[std::size_t(i)].load;
            m_prevBumpStop[std::size_t(i)] = t.wheels[std::size_t(i)].bumpStop;
            m_prevWater[std::size_t(i)] = f.wheels[std::size_t(i)].inWater;
        }
        return;
    }

    // Gear engaged (shifts into neutral while braking to a stop are silent).
    if (t.gear != m_gear && t.gear != 0 && m_time - m_lastShift > 0.25) {
        AudioEvent e;
        e.type = AudioEventType::GearShift;
        e.intensity = m_gear == 0 ? 0.6f : 1.0f;
        e.position = f.vehiclePos + f.vehicleUp * -0.3f + f.vehicleForward * 0.3f;
        m_events.push_back(e);
        m_lastShift = m_time;
    }
    m_gear = t.gear;

    for (int i = 0; i < kAudioWheels; ++i) {
        const std::size_t k = std::size_t(i);
        const WheelAudio& w = f.wheels[k];
        const bool bump = t.wheels[k].bumpStop && w.contact;
        if (bump && !m_prevBumpStop[k] && m_time - m_lastBump[k] > s.wheelEventInterval) {
            AudioEvent e;
            e.type = AudioEventType::BumpStop;
            e.intensity = std::clamp(w.load / s.bumpStopFullLoad, 0.25f, 1.0f);
            e.position = toAudio(t.wheels[k].suspensionTop);
            m_events.push_back(e);
            m_lastBump[k] = m_time;
            m_lastThump[k] = m_time;
        } else {
            const float step = w.load - m_prevLoad[k];
            if (step > s.thumpLoadStep && m_time - m_lastThump[k] > s.wheelEventInterval) {
                AudioEvent e;
                e.type = AudioEventType::SuspensionThump;
                e.intensity = std::clamp(step / s.thumpFullStep, 0.15f, 1.0f);
                e.position = w.position;
                m_events.push_back(e);
                m_lastThump[k] = m_time;
            }
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
        m_prevLoad[k] = w.load;
        m_prevWater[k] = w.inWater;
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
    if (articulation > s.creakArticulation && std::abs(articulation - m_prevArticulation) > 1e-4f &&
        t.speed < 6.0f && m_time - m_lastCreak > s.creakInterval) {
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
