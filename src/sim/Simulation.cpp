#include "sim/Simulation.h"

#include "track/MiniTrack.h"

#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

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
    // Lap timing: the circuit of a level, or the proving ground's mini handling track (start / finish and two sector
    // lines).
    std::vector<LevelGate> gates;
    if (m_settings.level) {
        gates = m_settings.level->circuit().gates;
    } else if (m_settings.layout == TrackLayout::ProvingGround) {
        for (const TimingGate& g : miniTrack().gates)
            gates.push_back({g.x, g.z, g.dirX, g.dirZ, g.halfWidth, !gates.empty()});
    }
    m_lapTimer.setGates(std::move(gates));
    applyDriftConfig();
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
    m_lapTimer.reset();
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
    m_drift.abort();
    ++m_spawnCount;
    // The attract route of this world, steered with this vehicle's geometry; the progress starts over.
    DriveRoute route;
    if (m_settings.level) route = m_settings.level->attractRoute();
    else if (m_track->layout() == TrackLayout::ProvingGround) route = TestTrack::attractRoute();
    m_autopilot.setRoute(route, config.wheelBase, config.axles[kFrontAxle].maxSteerAngleDeg * 0.0174532925f);
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
    m_lapTimer.reset();
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
        m_lapTimer.clearAll();
        m_drift.clearSession();
        m_pendingReset = false;
        setMessage("Vehicle rebuilt");
    }
    if (m_pendingTeleport) {
        const int idx = std::clamp(*m_pendingTeleport, 0, int(m_track->features().size()) - 1);
        m_spawnFeature = idx;
        m_pendingTeleport.reset();
        m_pendingReset = true;
        teleported = true;
        m_lapTimer.reset();
    }
    if (m_pendingPlace) {
        spawnVehicle(m_vehicle->config(), *m_pendingPlace);
        m_pendingPlace.reset();
        m_pendingReset = false;
        teleported = false;
        m_lapTimer.reset();
        setMessage("Vehicle placed");
    }
    if (m_pendingReset) {
        // Recreation resets the body, wheels, engine and gearbox to the initial state. On an open-world level the R
        // reset puts the vehicle on the nearest road point instead of the far-away spawn.
        if (m_settings.level && !teleported && m_lapTimer.running() && m_lapTimer.lastGate() >= 0) {
            spawnVehicle(m_vehicle->config(), gateResetPoint(m_lapTimer.lastGate()));
            std::snprintf(m_lapMessage, sizeof(m_lapMessage), m_lapTimer.lastGate() == 0
                              ? "Vehicle reset to the start line (lap timer running)"
                              : "Vehicle reset to checkpoint %d (lap timer running)", m_lapTimer.lastGate());
            setMessage(m_lapMessage);
        } else if (m_settings.level && !teleported) {
            m_lapTimer.reset();
            spawnVehicle(m_vehicle->config(), roadRecoveryPoint());
            setMessage("Vehicle reset to the nearest road");
        } else {
            m_lapTimer.reset();
            spawnVehicle(m_vehicle->config(), m_track->features()[m_spawnFeature]);
            setMessage("Vehicle reset to spawn point");
        }
        m_pendingReset = false;
    }
    if (m_pendingConfig) {
        if (Vehicle::needsRebuild(m_vehicle->config(), *m_pendingConfig)) {
            respawnInPlace(*m_pendingConfig);
            m_lapTimer.reset();
            m_drift.clearSession();
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
    auto pedal = [&](float cur, float target, float rise) {
        return approach(cur, std::clamp(target, 0.0f, 1.0f), target > cur ? rise : s.pedalFall, dt);
    };
    m_smoothed.throttle = pedal(m_smoothed.throttle, in.throttle, s.pedalRise);
    m_smoothed.brake = pedal(m_smoothed.brake, in.brake, s.brakeRise);
    m_smoothed.handBrake = pedal(m_smoothed.handBrake, in.handBrake, s.pedalRise);
    const float target = std::clamp(in.steer, -1.0f, 1.0f);
    const bool returning = std::abs(target) < std::abs(m_smoothed.steer) || target * m_smoothed.steer < 0.0f;
    // The analog stick sets the angle directly, but with a steering rate limit (faster than the keyboard).
    const VehicleTuning& vt = m_vehicle->config().tuning;
    const float rate = (returning ? vt.steerReturnRate : vt.steerRate) * (in.analog ? 2.0f : 1.0f);
    m_smoothed.steer = approach(m_smoothed.steer, target, rate, dt);
}

void Simulation::runAutopilot(float dt) {
    if (m_autopilotMode == AutopilotMode::Off) return;
    DriverIntent intent;
    if (m_autopilotMode == AutopilotMode::Hold || !m_autopilot.hasRoute()) {
        intent.brake = 1.0f;
        m_intent = intent;
        return;
    }
    const JPH::RMat44 xf = m_vehicle->bodyTransform();
    const JPH::Vec3 p(xf.GetTranslation()), fwd = xf.GetAxisZ(), up = xf.GetAxisY();
    AutopilotInput in;
    in.x = p.GetX();
    in.z = p.GetZ();
    const float h = std::hypot(fwd.GetX(), fwd.GetZ());
    if (h > 1e-3f) {
        in.fx = fwd.GetX() / h;
        in.fz = fwd.GetZ() / h;
    }
    in.forwardY = fwd.GetY();
    in.upDot = up.GetY();
    in.forwardSpeed = m_vehicle->telemetry().forwardSpeed;
    const AutopilotOutput out = m_autopilot.update(in, dt);
    if (m_autopilot.wantsReset()) {
        // Quiet recovery to the route start, facing along the route (the vehicle is recreated, like the R reset).
        const RoutePoint s = m_autopilot.startPoint();
        TrackFeature at;
        at.name = "route start";
        const float y = m_settings.level ? m_settings.level->heightAt(s.x, s.z) : TestTrack::provingGroundHeight(s.x, s.z);
        at.position = JPH::Vec3(s.x, y, s.z);
        at.yaw = m_autopilot.startYaw();
        m_autopilotResetReason = m_autopilot.resetReason();
        spawnVehicle(m_vehicle->config(), at);
        ++m_autopilotResets;
        m_lapTimer.reset();
        intent.brake = 1.0f;
        m_intent = intent;
        return;
    }
    intent.throttle = out.throttle;
    intent.brake = out.brake;
    intent.handBrake = out.handBrake;
    intent.steer = out.steer;
    intent.analog = true;
    m_intent = intent;
}

void Simulation::tick() {
    using Clock = std::chrono::steady_clock;
    const float dt = float(m_settings.fixedStep);

    processCommands();
    if (m_terrain) {
        const JPH::Vec3 p(m_vehicle->bodyTransform().GetTranslation());
        m_terrain->update(p.GetX(), p.GetZ());
    }
    runAutopilot(dt);
    resolvePedals(dt);
    smoothInput(dt);

    const JPH::Vec3 before(m_vehicle->bodyTransform().GetTranslation());
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
    updateLapTimer(before);
    updateDrift(before, dt);
    checkRecovery(dt);

    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    m_timing.lastStepMs = ms;
    m_timing.avgStepMs = m_timing.ticks == 0 ? ms : m_timing.avgStepMs * 0.98 + ms * 0.02;
    m_timing.maxStepMs = std::max(ms, m_timing.maxStepMs * (1.0 - dt / 2.0));
    ++m_timing.ticks;
}

void Simulation::updateLapTimer(const JPH::Vec3& before) {
    if (!m_lapTimer.active()) return;
    const JPH::Vec3 p(m_vehicle->bodyTransform().GetTranslation());
    const LapTimer::Event e = m_lapTimer.update(before.GetX(), before.GetZ(), p.GetX(), p.GetZ(),
                                                m_simTime - m_settings.fixedStep, m_simTime);
    auto clock = [](double t, char* out, std::size_t n) {
        const int m = int(t / 60.0);
        std::snprintf(out, n, "%d:%06.3f", m, t - 60.0 * m);
    };
    char a[24], b[24];
    switch (e) {
    case LapTimer::Event::Started: setMessage("Lap started"); break;
    case LapTimer::Event::Checkpoint:
        clock(m_lapTimer.lastSplit(), a, sizeof(a));
        std::snprintf(m_lapMessage, sizeof(m_lapMessage), "Checkpoint %d  %s", m_lapTimer.lastGate(), a);
        setMessage(m_lapMessage);
        break;
    case LapTimer::Event::Lap:
        clock(m_lapTimer.lastLap(), a, sizeof(a));
        clock(m_lapTimer.bestLap(), b, sizeof(b));
        std::snprintf(m_lapMessage, sizeof(m_lapMessage), "Lap %d  %s  (best %s)", m_lapTimer.laps(), a, b);
        setMessage(m_lapMessage);
        break;
    case LapTimer::Event::Invalid: setMessage("Lap not counted: a checkpoint was missed (new lap started)"); break;
    case LapTimer::Event::None: break;
    }
}

void Simulation::applyDriftConfig() {
    DriftConfig c;
    if (m_settings.level) c = m_settings.level->drift();
    else if (m_settings.layout == TrackLayout::ProvingGround) c = miniTrackDrift();
    if (m_driftOverride) c.enabled = *m_driftOverride;
    m_drift.setConfig(std::move(c));
}

void Simulation::setDriftOverride(std::optional<bool> enabled) {
    m_driftOverride = enabled;
    applyDriftConfig();
}

void Simulation::updateDrift(const JPH::Vec3& before, float dt) {
    if (!m_drift.active()) return;
    const JPH::RMat44 xf = m_vehicle->bodyTransform();
    const JPH::Vec3 p(xf.GetTranslation()), fwd = xf.GetAxisZ();
    DriftInput in;
    in.dt = dt;
    in.x = p.GetX();
    in.z = p.GetZ();
    const float h = std::hypot(fwd.GetX(), fwd.GetZ());
    if (h > 1e-3f) in.fx = fwd.GetX() / h, in.fz = fwd.GetZ() / h;
    // Rear bumper center: half the wheelbase plus about a metre of overhang behind the body origin.
    const float rear = 0.5f * m_vehicle->config().wheelBase + 1.0f;
    in.rearX = in.x - in.fx * rear;
    in.rearZ = in.z - in.fz * rear;
    in.vx = (p.GetX() - before.GetX()) / dt;
    in.vz = (p.GetZ() - before.GetZ()) / dt;
    in.wheelsOnGround = 0;
    for (const WheelTelemetry& w : m_vehicle->telemetry().wheels) in.wheelsOnGround += w.contact ? 1 : 0;
    for (const BodyImpact& b : m_impactListener.impacts()) in.impactSpeed = std::max(in.impactSpeed, b.approachSpeed);
    switch (m_drift.update(in)) {
    case DriftScorer::Event::Completed:
        std::snprintf(m_driftMessage, sizeof(m_driftMessage), "Drift %.0f pts, max %.0f deg (session best %.0f)",
                      m_drift.lastScore(), double(m_drift.maxAngleDeg()), m_drift.bestScore());
        setMessage(m_driftMessage);
        break;
    case DriftScorer::Event::Failed:
        std::snprintf(m_driftMessage, sizeof(m_driftMessage), "Drift lost: %s (%.0f pts)",
                      DriftScorer::endName(m_drift.lastEnd()), m_drift.lastLost());
        setMessage(m_driftMessage);
        break;
    case DriftScorer::Event::Started:
    case DriftScorer::Event::None: break;
    }
}

// On the track 4 m past the gate, facing along it (the gate itself is behind, so the reset does not cross it).
TrackFeature Simulation::gateResetPoint(int gate) const {
    const LevelGate& g = m_lapTimer.gates()[std::size_t(gate)];
    const float x = g.x + 4.0f * g.dx, z = g.z + 4.0f * g.dz;
    TrackFeature at;
    at.name = "checkpoint";
    at.position = JPH::Vec3(x, m_settings.level->heightAt(x, z), z);
    at.yaw = std::atan2(g.dx, g.dz);
    return at;
}

std::optional<float> Simulation::castRayIgnoringVehicle(JPH::Vec3Arg from, JPH::Vec3Arg to) const {
    const JPH::RRayCast ray(JPH::RVec3(from), to - from);
    JPH::RayCastResult hit;
    const JPH::IgnoreSingleBodyFilter filter(m_vehicle->bodyId());
    if (m_world->system().GetNarrowPhaseQuery().CastRay(ray, hit, {}, {}, filter))
        return hit.mFraction;
    return std::nullopt;
}

std::optional<float> Simulation::castRayAtVehicle(JPH::Vec3Arg from, JPH::Vec3Arg to) const {
    const JPH::BodyLockRead lock(m_world->system().GetBodyLockInterface(), m_vehicle->bodyId());
    if (!lock.Succeeded()) return std::nullopt;
    const JPH::RRayCast ray(JPH::RVec3(from), to - from);
    JPH::RayCastResult hit;
    if (lock.GetBody().GetTransformedShape().CastRay(ray, hit)) return hit.mFraction;
    return std::nullopt;
}

} // namespace tf
