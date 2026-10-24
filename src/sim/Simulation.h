#pragma once

#include "physics/PhysicsWorld.h"
#include "sim/BodyImpacts.h"
#include "sim/TireMarks.h"
#include "track/SurfaceProperties.h"
#include "track/TestTrack.h"
#include "vehicle/Vehicle.h"
#include "vehicle/VehicleConfig.h"

#include <Jolt/Jolt.h>

#include <array>
#include <memory>
#include <optional>

namespace tf {

// Raw driver intent from the input device (keyboard: 0/1, gamepad: analog).
struct DriverIntent {
    float throttle = 0.0f;  // 0..1
    float brake = 0.0f;     // 0..1
    float steer = 0.0f;     // -1..1 (right is +)
    float handBrake = 0.0f; // 0..1
    bool analog = false;    // analog source: steering is not smoothed with the keyboard rate
    // Pedal mapping. false: throttle drives in the selected direction and brake always brakes (tests, tools).
    // true (live keyboard/gamepad): throttle is the "forward" pedal (W/RT) and brake the "backward" pedal (S/LT);
    // the pedal against the selected direction brakes; released and pressed again at a standstill it switches the
    // direction.
    bool autoReverse = false;
};

// Automatic forward/reverse switching for DriverIntent::autoReverse, evaluated on physics ticks. A pedal that was
// already held while moving only brakes, also after the stop; the switch needs a fresh press at a standstill.
struct AutoReverseSettings {
    float stopSpeed = 0.5f;      // m/s, |forward speed| below which a held opposite pedal counts as "stopped"
    float holdTime = 0.25f;      // s of simulated time a fresh press must be held while stopped
    float pedalThreshold = 0.5f; // opposite pedal travel that counts as held (gamepad triggers are analog)
};

// Pedal and steering smoothing. Updated on every physics tick with a fixed dt,
// so it does not depend on FPS.
struct InputSmoothing {
    float steerRate = 2.2f;       // 1/s, steering rate toward the target (keyboard)
    float steerReturnRate = 3.5f; // 1/s, return-to-center rate
    float pedalRise = 3.0f;       // 1/s
    float pedalFall = 6.0f;       // 1/s
};

struct Pose {
    JPH::Vec3 position = JPH::Vec3::sZero();
    JPH::Quat rotation = JPH::Quat::sIdentity();
};

// Pose snapshot for rendering (body + 4 wheels) after a physics tick.
struct RenderSnapshot {
    Pose body;
    std::array<Pose, kWheelCount> wheels;
};

struct SimulationSettings {
    TrackLayout layout = TrackLayout::ProvingGround;
    double fixedStep = 1.0 / 120.0; // s
    int collisionSteps = 1;
    PhysicsWorld::Settings physics;
    VehicleConfig vehicle;
    InputSmoothing smoothing;
    AutoReverseSettings autoReverse;
    TireMarkSettings tireMarks;
};

struct StepTiming {
    double lastStepMs = 0.0;   // last physics step time (CPU, ms)
    double marksUs = 0.0;      // tire mark update of the last tick (CPU, microseconds), not part of lastStepMs
    double avgStepMs = 0.0;    // moving average, ms
    double maxStepMs = 0.0;    // maximum over the last ~2 s, ms
    unsigned long long ticks = 0;
};

// Windowless game physics: Jolt world, proving ground, vehicle and fixed step.
// Used by the application and the headless tests.
class Simulation {
public:
    explicit Simulation(const SimulationSettings& settings = {});
    ~Simulation();

    // Driver intent; applied on the following ticks.
    void setIntent(const DriverIntent& intent) { m_intent = intent; }
    const DriverIntent& intent() const { return m_intent; }

    // Discrete commands run at the start of the next tick.
    void requestToggleDirection() { m_pendingToggleDirection = true; }
    void requestToggleLowRange() { m_pendingToggleLowRange = true; }
    void requestReset() { m_pendingReset = true; }
    void requestRebuild(const VehicleConfig& config) { m_pendingRebuild = config; }
    void requestTuning(const VehicleTuning& tuning) { m_pendingTuning = tuning; }
    void requestTeleport(int featureIndex) { m_pendingTeleport = featureIndex; }
    // Driver aid switches: B (ABS on/off), T (TC Off -> On -> Off-road), X (lockers none -> rear -> rear + center ->
    // all), or a direct setting (UI, scenarios). ABS and TC survive a reset/rebuild; the lockers reset like low range.
    void requestToggleAbs() { m_pendingAbsToggle = true; }
    void requestCycleTraction() { m_pendingTractionCycle = true; }
    void requestCycleDiffLock() { m_pendingDiffCycle = true; }
    void requestAidControls(const AidControls& c) { m_pendingAids = c; }
    // Requested switches if a request is pending, otherwise the vehicle's.
    AidControls aidControls() const { return m_pendingAids.value_or(m_vehicle->aidControls()); }

    // One fixed physics step.
    void tick();

    double fixedStep() const { return m_settings.fixedStep; }
    void setFixedStep(double dt) { m_settings.fixedStep = dt; }
    int collisionSteps() const { return m_settings.collisionSteps; }
    void setCollisionSteps(int n) { m_settings.collisionSteps = n; }
    InputSmoothing& smoothing() { return m_settings.smoothing; }
    AutoReverseSettings& autoReverse() { return m_settings.autoReverse; }

    PhysicsWorld& world() { return *m_world; }
    SurfaceTable& surfaces() { return m_surfaces; }
    const TestTrack& track() const { return *m_track; }
    const Vehicle& vehicle() const { return *m_vehicle; }
    const VehicleConfig& vehicleConfig() const { return m_vehicle->config(); }
    const StepTiming& timing() const { return m_timing; }
    double simTime() const { return m_simTime; } // s, sum of all fixed steps
    // Visual-only tire marks written from the wheel contact samples after every tick.
    TireMarks& tireMarks() { return m_tireMarks; }
    const TireMarks& tireMarks() const { return m_tireMarks; }
    // New contacts of the vehicle body during the last tick (observation only, for impact sounds).
    const std::vector<BodyImpact>& bodyImpacts() const { return m_impactListener.impacts(); }
    // Incremented whenever the vehicle is (re)created: start, R reset, teleport, rebuild.
    unsigned spawnCount() const { return m_spawnCount; }
    const RenderSnapshot& previousSnapshot() const { return m_prev; }
    const RenderSnapshot& currentSnapshot() const { return m_curr; }
    // Latest messages about discrete commands (e.g. "reverse only near a stop").
    const char* lastMessage() const { return m_lastMessage; }
    double lastMessageTime() const { return m_lastMessageTime; } // simTime() when the message was set
    // Pedals after the forward/reverse mapping of the last tick (before smoothing).
    const DriverIntent& resolvedIntent() const { return m_resolved; }
    bool hadPhysicsError() const { return m_physicsError; }

    // Ray against static geometry and other bodies except the vehicle. Returns the path fraction [0,1] to the hit.
    std::optional<float> castRayIgnoringVehicle(JPH::Vec3Arg from, JPH::Vec3Arg to) const;

private:
    // config by value: the old vehicle (and its config) is destroyed inside.
    void spawnVehicle(VehicleConfig config, const TrackFeature& at);
    RenderSnapshot captureSnapshot() const;
    void processCommands();
    void setMessage(const char* text);
    void resolvePedals(float dt);
    void smoothInput(float dt);

    JoltRuntime m_joltRuntime; // first: SurfaceTable creates Jolt objects
    SimulationSettings m_settings;
    SurfaceTable m_surfaces;
    BodyImpactListener m_impactListener; // before m_world: registered with its PhysicsSystem
    std::unique_ptr<PhysicsWorld> m_world;
    std::unique_ptr<TestTrack> m_track;
    std::unique_ptr<Vehicle> m_vehicle;
    int m_spawnFeature = 0;
    unsigned m_spawnCount = 0;

    DriverIntent m_intent;
    DriverIntent m_resolved;
    float m_autoReverseHold = 0.0f; // s the opposite pedal has been held while stopped
    // The opposite pedal has been held since the vehicle was moving: it keeps braking (holds on a slope) until it
    // is released; only a fresh press at a standstill switches the direction.
    bool m_brakeLatch = false;
    DriverCommand m_smoothed;
    bool m_pendingToggleDirection = false;
    bool m_pendingToggleLowRange = false;
    bool m_pendingReset = false;
    std::optional<VehicleConfig> m_pendingRebuild;
    std::optional<VehicleTuning> m_pendingTuning;
    std::optional<int> m_pendingTeleport;
    bool m_pendingAbsToggle = false;
    bool m_pendingTractionCycle = false;
    bool m_pendingDiffCycle = false;
    std::optional<AidControls> m_pendingAids;
    std::optional<AidControls> m_aidSwitches; // ABS/TC state carried over to a respawned vehicle
    const char* m_lastMessage = "";
    double m_lastMessageTime = -1.0e9;
    bool m_physicsError = false;

    RenderSnapshot m_prev, m_curr;
    StepTiming m_timing;
    double m_simTime = 0.0;
    TireMarks m_tireMarks;
};

} // namespace tf
