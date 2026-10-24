#include "sim/Simulation.h"

#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace tf {
namespace {

float approach(float current, float target, float rate, float dt) {
    const float delta = target - current;
    const float maxDelta = rate * dt;
    return current + std::clamp(delta, -maxDelta, maxDelta);
}

} // namespace

Simulation::Simulation(const SimulationSettings& settings) : m_settings(settings), m_tireMarks(settings.tireMarks) {
    m_world = std::make_unique<PhysicsWorld>(settings.physics);
    m_world->system().SetContactListener(&m_impactListener);
    m_track = std::make_unique<TestTrack>(*m_world, m_surfaces, settings.layout);
    spawnVehicle(settings.vehicle, m_track->spawn());
}

Simulation::~Simulation() {
    m_vehicle.reset();
    m_track.reset();
    m_world.reset();
}

void Simulation::spawnVehicle(VehicleConfig config, const TrackFeature& at) {
    m_vehicle.reset(); // delete the old body/constraint first
    // Height: wheels at full droop slightly above the ground.
    const float y = at.position.GetY() + config.wheelRadius + config.suspensionMaxLength - config.attachHeight + 0.05f;
    const JPH::RVec3 pos(at.position.GetX(), y, at.position.GetZ());
    const JPH::Quat rot = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), at.yaw);
    m_vehicle = std::make_unique<Vehicle>(*m_world, m_surfaces, config, pos, rot);
    m_impactListener.setBody(m_vehicle->bodyId());
    ++m_spawnCount;
    m_smoothed = {};
    m_autoReverseHold = 0.0f;
    TireMarkSettings marks = m_tireMarks.settings();
    marks.referenceLoad = 0.25f * config.mass * 9.81f;
    m_tireMarks.setSettings(marks);
    m_tireMarks.breakRibbons();
    m_curr = captureSnapshot();
    m_prev = m_curr;
}

RenderSnapshot Simulation::captureSnapshot() const {
    RenderSnapshot s;
    const JPH::RMat44 b = m_vehicle->bodyTransform();
    s.body.position = JPH::Vec3(b.GetTranslation());
    s.body.rotation = b.GetQuaternion().Normalized();
    for (int i = 0; i < kWheelCount; ++i) {
        const JPH::RMat44 w = m_vehicle->wheelTransform(i);
        s.wheels[i].position = JPH::Vec3(w.GetTranslation());
        s.wheels[i].rotation = w.GetQuaternion().Normalized();
    }
    return s;
}

void Simulation::processCommands() {
    if (m_pendingRebuild) {
        spawnVehicle(*m_pendingRebuild, m_track->features()[m_spawnFeature]);
        m_pendingRebuild.reset();
        m_pendingReset = false;
        setMessage("Vehicle rebuilt");
    }
    if (m_pendingTeleport) {
        const int idx = std::clamp(*m_pendingTeleport, 0, int(m_track->features().size()) - 1);
        m_spawnFeature = idx;
        m_pendingTeleport.reset();
        m_pendingReset = true;
    }
    if (m_pendingReset) {
        // Recreation resets the body, wheels, engine and gearbox to the initial state.
        spawnVehicle(m_vehicle->config(), m_track->features()[m_spawnFeature]);
        m_pendingReset = false;
        setMessage("Vehicle reset to spawn point");
    }
    if (m_pendingTuning) {
        m_vehicle->applyTuning(*m_pendingTuning);
        m_pendingTuning.reset();
    }
    if (m_pendingToggleDirection) {
        m_pendingToggleDirection = false;
        if (m_vehicle->toggleDirection())
            setMessage(m_vehicle->telemetry().direction == DriveDirection::Forward ? "Drive (forward) selected"
                                                                                   : "Reverse selected");
        else
            setMessage("Stop to change direction (below 1 m/s)");
    }
    if (m_pendingToggleLowRange) {
        m_pendingToggleLowRange = false;
        setMessage(m_vehicle->toggleLowRange() ? "Transfer case range changed"
                                               : "Low range change only near standstill (< 1 m/s)");
    }
}

void Simulation::setMessage(const char* text) {
    m_lastMessage = text;
    m_lastMessageTime = m_simTime;
}

void Simulation::resolvePedals(float dt) {
    m_resolved = m_intent;
    if (!m_intent.autoReverse) {
        m_autoReverseHold = 0.0f;
        return;
    }
    const AutoReverseSettings& a = m_settings.autoReverse;
    const VehicleTelemetry& t = m_vehicle->telemetry();
    bool forward = t.direction == DriveDirection::Forward;
    auto map = [&] {
        m_resolved.throttle = forward ? m_intent.throttle : m_intent.brake;
        m_resolved.brake = forward ? m_intent.brake : m_intent.throttle;
    };
    map();
    // The pedal against the selected direction brakes; held near a standstill (and without the other pedal)
    // for holdTime it selects the opposite direction and becomes the throttle.
    if (m_resolved.brake > a.pedalThreshold && m_resolved.throttle < 0.05f && std::abs(t.forwardSpeed) < a.stopSpeed) {
        m_autoReverseHold += dt;
        if (m_autoReverseHold >= a.holdTime && m_vehicle->toggleDirection()) {
            m_autoReverseHold = 0.0f;
            forward = !forward;
            setMessage(forward ? "Drive engaged" : "Reverse engaged");
            map();
        }
    } else {
        m_autoReverseHold = 0.0f;
    }
}

void Simulation::smoothInput(float dt) {
    const InputSmoothing& s = m_settings.smoothing;
    const DriverIntent& in = m_resolved;
    auto pedal = [&](float cur, float target) {
        return approach(cur, std::clamp(target, 0.0f, 1.0f), target > cur ? s.pedalRise : s.pedalFall, dt);
    };
    m_smoothed.throttle = pedal(m_smoothed.throttle, in.throttle);
    m_smoothed.brake = pedal(m_smoothed.brake, in.brake);
    m_smoothed.handBrake = pedal(m_smoothed.handBrake, in.handBrake);
    const float target = std::clamp(in.steer, -1.0f, 1.0f);
    const bool returning = std::abs(target) < std::abs(m_smoothed.steer) || target * m_smoothed.steer < 0.0f;
    // The analog stick sets the angle directly, but with a steering rate limit (faster than the keyboard).
    const float rate = (returning ? s.steerReturnRate : s.steerRate) * (in.analog ? 2.0f : 1.0f);
    m_smoothed.steer = approach(m_smoothed.steer, target, rate, dt);
}

void Simulation::tick() {
    using Clock = std::chrono::steady_clock;
    const float dt = float(m_settings.fixedStep);

    processCommands();
    resolvePedals(dt);
    smoothInput(dt);

    const auto t0 = Clock::now();
    m_vehicle->prePhysics(m_smoothed);
    m_impactListener.beginTick();
    const JPH::EPhysicsUpdateError err = m_world->step(dt, m_settings.collisionSteps);
    m_physicsError = m_physicsError || err != JPH::EPhysicsUpdateError::None;
    // Jolt splits the step into collisionSteps substeps; constraint lambdas are impulses of the last substep.
    m_vehicle->postPhysics(dt / float(m_settings.collisionSteps));
    const auto t1 = Clock::now();

    m_simTime += m_settings.fixedStep;
    m_tireMarks.update(m_vehicle->contacts(), m_simTime);
    const auto t2 = Clock::now();
    m_timing.marksUs = std::chrono::duration<double, std::micro>(t2 - t1).count();

    m_prev = m_curr;
    m_curr = captureSnapshot();

    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    m_timing.lastStepMs = ms;
    m_timing.avgStepMs = m_timing.ticks == 0 ? ms : m_timing.avgStepMs * 0.98 + ms * 0.02;
    m_timing.maxStepMs = std::max(ms, m_timing.maxStepMs * (1.0 - dt / 2.0));
    ++m_timing.ticks;
}

std::optional<float> Simulation::castRayIgnoringVehicle(JPH::Vec3Arg from, JPH::Vec3Arg to) const {
    const JPH::RRayCast ray(JPH::RVec3(from), to - from);
    JPH::RayCastResult hit;
    const JPH::IgnoreSingleBodyFilter filter(m_vehicle->bodyId());
    if (m_world->system().GetNarrowPhaseQuery().CastRay(ray, hit, {}, {}, filter))
        return hit.mFraction;
    return std::nullopt;
}

} // namespace tf
