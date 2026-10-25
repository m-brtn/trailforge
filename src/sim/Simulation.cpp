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
    buildWorld();
    spawnVehicle(settings.vehicle, m_track->spawn());
}

Simulation::~Simulation() {
    m_vehicle.reset();
    m_terrain.reset();
    m_track.reset();
    m_world.reset();
}

namespace {
// Body origin height above the ground with the wheels at full droop slightly above it.
float spawnHeight(const VehicleConfig& config) {
    float reach = 0.0f;
    for (const AxleConfig& a : config.axles) reach = std::max(reach, a.wheelRadius + a.suspensionMaxLength);
    return reach - config.attachHeight + 0.05f;
}
} // namespace

void Simulation::buildWorld() {
    m_terrain.reset();
    m_track.reset();
    if (m_settings.level) {
        m_track = std::make_unique<TestTrack>(*m_world, m_surfaces, *m_settings.level);
        m_terrain = std::make_unique<TerrainTiles>(*m_world, m_surfaces, m_settings.level, m_settings.terrain);
        std::shared_ptr<const Level> level = m_settings.level;
        m_water = [level](float x, float z) { return level->waterAt(x, z); };
    } else {
        m_track = std::make_unique<TestTrack>(*m_world, m_surfaces, m_settings.layout);
        m_water = provingGroundWater();
    }
    m_tireMarks.setWater(m_water);
    m_tireMarks.clear();
    m_spawnFeature = 0;
    m_submersion = m_submergedTime = 0.0f;
}

TrackFeature Simulation::roadRecoveryPoint() const {
    const JPH::RMat44 xf = m_vehicle->bodyTransform();
    const JPH::Vec3 p(xf.GetTranslation());
    TrackFeature at = m_settings.level->nearestRoadPoint(p.GetX(), p.GetZ());
    // Keep the travel direction along the road closest to the current heading.
    const JPH::Vec3 fwd = xf.GetAxisZ();
    if (fwd.GetX() * std::sin(at.yaw) + fwd.GetZ() * std::cos(at.yaw) < 0.0f) at.yaw += 3.14159265f;
    return at;
}

void Simulation::checkRecovery(float dt) {
    if (!m_settings.level) return;
    const Level& level = *m_settings.level;
    const JPH::Vec3 p(m_vehicle->bodyTransform().GetTranslation());
    const std::optional<float> water = level.waterAt(p.GetX(), p.GetZ());
    m_submersion = water ? *water - p.GetY() : 0.0f;
    m_submergedTime = m_submersion > 0.0f ? m_submergedTime + dt : 0.0f;
    const bool nearEdge = !level.insidePlayArea(p.GetX(), p.GetZ(), 40.0f);
    if (nearEdge && !m_nearEdge) setMessage("Edge of the map ahead: driving on recovers the vehicle to the nearest road");
    m_nearEdge = nearEdge;
    const char* message = nullptr;
    if (!level.insidePlayArea(p.GetX(), p.GetZ(), -8.0f) || p.GetY() < level.bakeStats().minHeight - 30.0f)
        message = "Left the map: vehicle recovered to the nearest road";
    else if (m_submergedTime > m_settings.deepWaterTime)
        message = "Deep water: vehicle recovered to the nearest road (no buoyancy or water simulated)";
    if (!message) return;
    const TrackFeature at = roadRecoveryPoint();
    const AidControls controls = m_vehicle->aidControls();
    spawnVehicle(m_vehicle->config(), at);
    m_vehicle->setAidControls(controls);
    m_submersion = m_submergedTime = 0.0f;
    ++m_recoveries;
    setMessage(message);
}

void Simulation::spawnVehicle(VehicleConfig config, const TrackFeature& at) {
    m_vehicle.reset(); // delete the old body/constraint first
    if (m_terrain) m_terrain->ensureLoaded(at.position.GetX(), at.position.GetZ(), m_settings.terrain.requiredRadius);
    const float y = at.position.GetY() + spawnHeight(config);
    const JPH::RVec3 pos(at.position.GetX(), y, at.position.GetZ());
    const JPH::Quat rot = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), at.yaw);
    m_vehicle = std::make_unique<Vehicle>(*m_world, m_surfaces, config, pos, rot);
    if (m_aidSwitches) {
        AidControls c = m_vehicle->aidControls();
        c.abs = m_aidSwitches->abs;
        c.traction = m_aidSwitches->traction;
        m_vehicle->setAidControls(c);
    }
    m_impactListener.setBody(m_vehicle->bodyId());
    ++m_spawnCount;
    m_smoothed = {};
    m_autoReverseHold = 0.0f;
    m_brakeLatch = false;
    TireMarkSettings marks = m_tireMarks.settings();
    marks.referenceLoad = 0.25f * config.mass * 9.81f;
    m_tireMarks.setSettings(marks);
    m_tireMarks.breakRibbons();
    m_curr = captureSnapshot();
    m_prev = m_curr;
}

void Simulation::respawnInPlace(VehicleConfig config) {
    const JPH::RMat44 xf = m_vehicle->bodyTransform();
    const JPH::Vec3 p(xf.GetTranslation());
    const JPH::Vec3 fwd = xf.GetAxisZ();
    const float yaw = std::atan2(fwd.GetX(), fwd.GetZ());
    const AidControls controls = m_vehicle->aidControls();
    const VehicleTelemetry old = m_vehicle->telemetry();
    // Ground below the body: a ray from 3 m above to 10 m below, ignoring the vehicle.
    const JPH::Vec3 above = p + JPH::Vec3(0.0f, 3.0f, 0.0f), below = p - JPH::Vec3(0.0f, 10.0f, 0.0f);
    float ground = p.GetY() - 1.0f;
    if (auto hit = castRayIgnoringVehicle(above, below)) ground = above.GetY() + (below.GetY() - above.GetY()) * *hit;
    TrackFeature at;
    at.position = JPH::Vec3(p.GetX(), ground, p.GetZ());
    at.yaw = yaw;
    spawnVehicle(std::move(config), at);
    m_vehicle->setAidControls(controls);
    if (old.layout == DriveLayout::PartTime) m_vehicle->setTransfer(old.transfer);
    else if (old.lowRange) m_vehicle->toggleLowRange();
    // A rebuild in place keeps the battery charge and the drive mode (the R reset restores the initial charge).
    if (old.ev.electric) {
        m_vehicle->setEvDriveMode(old.ev.mode);
        m_vehicle->setStateOfCharge(old.ev.soc);
    }
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
    if (m_pendingLevel) {
        VehicleConfig config = m_vehicle->config();
        m_vehicle.reset();
        m_settings.level = std::move(*m_pendingLevel);
        m_pendingLevel.reset();
        buildWorld();
        spawnVehicle(std::move(config), m_track->spawn());
        m_pendingReset = false;
        m_pendingTeleport.reset();
        m_pendingPlace.reset();
        setMessage(m_settings.level ? "Level loaded" : "Proving ground loaded");
    }
    bool teleported = false;
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
        teleported = true;
    }
    if (m_pendingPlace) {
        spawnVehicle(m_vehicle->config(), *m_pendingPlace);
        m_pendingPlace.reset();
        m_pendingReset = false;
        teleported = false;
        setMessage("Vehicle placed");
    }
    if (m_pendingReset) {
        // Recreation resets the body, wheels, engine and gearbox to the initial state. On an open-world level the R
        // reset puts the vehicle on the nearest road point instead of the far-away spawn.
        if (m_settings.level && !teleported) {
            spawnVehicle(m_vehicle->config(), roadRecoveryPoint());
            setMessage("Vehicle reset to the nearest road");
        } else {
            spawnVehicle(m_vehicle->config(), m_track->features()[m_spawnFeature]);
            setMessage("Vehicle reset to spawn point");
        }
        m_pendingReset = false;
    }
    if (m_pendingConfig) {
        if (Vehicle::needsRebuild(m_vehicle->config(), *m_pendingConfig)) {
            respawnInPlace(*m_pendingConfig);
            setMessage("Vehicle rebuilt in place");
        } else {
            m_vehicle->applyTuning(m_pendingConfig->tuning);
        }
        m_pendingConfig.reset();
    }
    if (m_pendingTuning) {
        m_vehicle->applyTuning(*m_pendingTuning);
        m_pendingTuning.reset();
    }
    if (m_pendingSoC) {
        m_vehicle->setStateOfCharge(*m_pendingSoC);
        m_pendingSoC.reset();
    }
    if (m_pendingTransferCycle || m_pendingTransfer) {
        const bool ok = m_pendingTransfer ? m_vehicle->setTransfer(*m_pendingTransfer) : m_vehicle->cycleTransfer();
        m_pendingTransferCycle = false;
        m_pendingTransfer.reset();
        const VehicleTelemetry& t = m_vehicle->telemetry();
        if (m_vehicle->config().isElectric())
            setMessage(m_vehicle->evDriveMode() == EvDriveMode::OffRoad ? "Drive mode OFF-ROAD (off-road torque split)"
                                                                        : "Drive mode NORMAL (road torque split)");
        else if (m_vehicle->config().layout != DriveLayout::PartTime)
            setMessage("No transfer case switch on this drivetrain");
        else if (!ok)
            setMessage("4L in or out only near standstill (< 1 m/s)");
        else
            setMessage(t.transfer == TransferMode::TwoHigh ? "2H: rear-wheel drive"
                       : t.transfer == TransferMode::FourHigh ? "4H: front axle engaged, center locked"
                                                              : "4L: low range, center locked");
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
        const bool ok = m_vehicle->toggleLowRange();
        if (m_vehicle->config().isElectric())
            setMessage("No low range: single fixed reduction per motor");
        else if (ok && m_vehicle->config().layout == DriveLayout::PartTime)
            setMessage(m_vehicle->telemetry().transfer == TransferMode::FourLow ? "4L: low range, center locked"
                                                                                : "4H: front axle engaged, center locked");
        else
            setMessage(ok ? "Transfer case range changed" : "Low range change only near standstill (< 1 m/s)");
    }
    if (m_pendingAbsToggle || m_pendingTractionCycle || m_pendingDiffCycle || m_pendingAids || m_pendingDefaultTraction) {
        AidControls c = m_pendingAids ? *m_pendingAids : m_vehicle->aidControls();
        if (m_pendingDefaultTraction) c.traction = m_vehicle->config().tuning.aids.traction;
        if (m_pendingAbsToggle) {
            c.abs = !c.abs;
            setMessage(c.abs ? "ABS on" : "ABS off");
        }
        if (m_pendingTractionCycle) {
            c.traction = TractionMode((int(c.traction) + 1) % 3);
            setMessage(c.traction == TractionMode::Off ? "Traction control off"
                       : c.traction == TractionMode::On ? "Traction control on"
                                                        : "Traction control off-road");
        }
        if (m_pendingDiffCycle) {
            c.diffLocks = (c.diffLocks + 1) % AidControls::kDiffLockStages;
            const char* names[] = {"Lockers off", "Rear diff locked", "Rear + center diffs locked", "All diffs locked"};
            setMessage(names[c.diffLocks]);
        }
        m_pendingAbsToggle = m_pendingTractionCycle = m_pendingDiffCycle = m_pendingDefaultTraction = false;
        m_pendingAids.reset();
        m_vehicle->setAidControls(c);
        m_aidSwitches = m_vehicle->aidControls();
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
        m_brakeLatch = false;
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
    // The pedal against the selected direction brakes. A press that started while moving keeps braking through the
    // stop and holds the vehicle (also on a slope) until it is released; a fresh press near a standstill (without
    // the other pedal), held for holdTime, selects the opposite direction and becomes the throttle.
    const bool held = m_resolved.brake > a.pedalThreshold;
    const bool stopped = std::abs(t.forwardSpeed) < a.stopSpeed;
    if (!held) m_brakeLatch = false;
    else if (!stopped) m_brakeLatch = true;
    if (held && !m_brakeLatch && m_resolved.throttle < 0.05f && stopped) {
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
    const VehicleTuning& vt = m_vehicle->config().tuning;
    const float rate = (returning ? vt.steerReturnRate : vt.steerRate) * (in.analog ? 2.0f : 1.0f);
    m_smoothed.steer = approach(m_smoothed.steer, target, rate, dt);
}

void Simulation::tick() {
    using Clock = std::chrono::steady_clock;
    const float dt = float(m_settings.fixedStep);

    processCommands();
    if (m_terrain) {
        const JPH::Vec3 p(m_vehicle->bodyTransform().GetTranslation());
        m_terrain->update(p.GetX(), p.GetZ());
    }
    resolvePedals(dt);
    smoothInput(dt);

    const auto t0 = Clock::now();
    m_vehicle->prePhysics(m_smoothed, dt);
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
    checkRecovery(dt);

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
