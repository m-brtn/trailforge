#pragma once

#include "track/SurfaceProperties.h"
#include "vehicle/DrivingAids.h"
#include "vehicle/ElectricDrive.h"
#include "vehicle/VehicleConfig.h"
#include "vehicle/WheelContact.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

#include <array>

namespace tf {

class PhysicsWorld;

inline constexpr int kWheelCount = 4;
inline constexpr const char* kWheelNames[kWheelCount] = {"FL", "FR", "RL", "RR"};

// Driver commands for one physics tick (already smoothed).
struct DriverCommand {
    float throttle = 0.0f;  // 0..1
    float brake = 0.0f;     // 0..1
    float steer = 0.0f;     // -1 (left) .. 1 (right)
    float handBrake = 0.0f; // 0..1
};

enum class DriveDirection { Forward, Reverse };

struct WheelTelemetry {
    bool contact = false;
    bool bumpStop = false;            // suspension hit the bump stop (min length)
    float suspensionLength = 0.0f;    // m, from the attachment point to the wheel center
    float compression = 0.0f;         // m, maxLength - length
    float compressionRatio = 0.0f;    // 0 = full droop, 1 = full compression
    float load = 0.0f;                // N, suspension force = suspension impulse / dt
    float longitudinalForce = 0.0f;   // N, tire longitudinal force = impulse / dt
    float lateralForce = 0.0f;        // N
    float longitudinalSlip = 0.0f;    // dimensionless |(wr - v)/v| (Jolt definition)
    float lateralSlipDeg = 0.0f;      // degrees, slip angle
    float contactSpeed = 0.0f;        // m/s, body velocity at the contact point (ground is static)
    bool slipValid = false;           // for |v| and |wr| < 0.5 m/s Jolt's slip is numerically undefined
    float angularVelocity = 0.0f;     // rad/s
    float steerAngleDeg = 0.0f;       // degrees, + is left
    float combinedLongFriction = 0.0f; // mu tire x surface (longitudinal)
    float groundSpeed = 0.0f;  // m/s, ground velocity under the wheel along its heading (hub velocity in the air)
    float rollingSpeed = 0.0f; // m/s, w * r
    float brakeTorque = 0.0f;  // N*m commanded on the last tick (pedal x ABS pressure + TC brake)
    float absPressure = 1.0f;  // 0..1
    float tcBrake = 0.0f;      // N*m
    SurfaceId surface = SurfaceId::PackedDirt;
    JPH::Vec3 contactPosition = JPH::Vec3::sZero(); // world, m
    JPH::Vec3 contactNormal = JPH::Vec3::sZero();
    JPH::Vec3 contactLongitudinal = JPH::Vec3::sZero();
    JPH::Vec3 contactLateral = JPH::Vec3::sZero();
    JPH::Vec3 suspensionTop = JPH::Vec3::sZero();    // world: attachment point
    JPH::Vec3 suspensionDir = JPH::Vec3::sZero();    // world: travel direction (down)
};

// Dual-motor electric state of the last tick (electric = false for combustion vehicles).
struct EvTelemetry {
    bool electric = false;
    EvDriveMode mode = EvDriveMode::Normal;
    float frontShare = 0.5f;          // part-throttle front share of the active drive mode
    float soc = 0.0f;                 // 0..1
    float batteryPowerKW = 0.0f;      // + discharge, - charge (regen), incl. the auxiliary load
    float motorPowerKW = 0.0f;        // shaft power of both motors, + drive, - regen
    std::array<float, 2> motorRPM{};    // front, rear (+ = forward rotation)
    std::array<float, 2> motorTorque{}; // N*m at the shaft, signed like the rotation it drives (regen opposes it)
    std::array<float, 2> motorPowerAxleKW{};
    std::array<float, 2> tcFactor{1.0f, 1.0f};
    std::array<float, 2> regenFactor{1.0f, 1.0f};
    std::array<float, 2> frictionScale{1.0f, 1.0f}; // pedal friction torque left after regen, per axle
    bool regenActive = false;
    double energyOutKWh = 0.0;        // battery energy drawn since spawn
    double energyInKWh = 0.0;         // battery energy recovered since spawn
};

struct VehicleTelemetry {
    float speed = 0.0f;         // m/s, magnitude
    float forwardSpeed = 0.0f;  // m/s, projection onto the body +Z axis
    float engineRPM = 0.0f;     // rpm
    int gear = 0;               // -1 = R, 0 = N, 1.. = gears
    float clutchFriction = 0.0f; // 0..1
    bool shifting = false;
    DriveDirection direction = DriveDirection::Forward;
    bool lowRange = false;
    DriveLayout layout = DriveLayout::Awd;
    TransferMode transfer = TransferMode::FourHigh; // part-time layouts only
    bool frontDriven = true, rearDriven = true;     // axles connected to the engine on the last tick
    float frontTorqueShare = 0.5f;                  // base engine torque share of the front axle (before the center LSD)
    bool revLimited = false;                        // rev limiter cut the throttle on the last tick
    float dragForce = 0.0f;                         // N, aerodynamic drag of the last tick
    DriverCommand command;      // commands applied on the last tick (driver pedals, before the aids)
    AidControls controls;       // ABS / TC / locker switches
    bool absActive = false;     // ABS modulated at least one wheel on the last tick
    bool tcActive = false;      // TC cut the throttle or braked a wheel on the last tick
    float throttleFactor = 1.0f; // TC throttle factor (1 = no cut)
    float engineThrottle = 0.0f; // throttle passed to Jolt after the TC cut
    DiffMode frontDiff = DiffMode::Open, rearDiff = DiffMode::Open, centerDiff = DiffMode::Open; // effective modes
    float steerAngleDeg = 0.0f; // average front wheel angle, degrees
    float upDot = 1.0f;         // cosine of the angle between the body Y axis and vertical
    std::array<WheelTelemetry, kWheelCount> wheels;
    EvTelemetry ev;
};

class ElectricVehicleController;

// Four-wheeled vehicle based on Jolt VehicleConstraint + WheeledVehicleController; the drive layout comes from the
// configuration (VehicleConfig::layout).
// Moves only via simulation forces/torques; position/velocity are never set directly
// (except when spawning at the start position).
class Vehicle {
public:
    Vehicle(PhysicsWorld& world, const SurfaceTable& surfaces, const VehicleConfig& config,
            JPH::RVec3Arg position, JPH::QuatArg rotation);
    ~Vehicle();
    Vehicle(const Vehicle&) = delete;
    Vehicle& operator=(const Vehicle&) = delete;

    // Before the physics step: runs the driver aids and passes commands to the controller. dt = fixed step.
    void prePhysics(const DriverCommand& command, float dt);
    // After the physics step: collects telemetry. dt is the length of the last Jolt substep
    // (fixedStep / collisionSteps), which the constraint lambdas refer to.
    void postPhysics(float dt);

    // Forward/reverse only near a stop. Returns true if switched.
    bool toggleDirection();
    // Low range only near a stop. On a part-time layout this switches 4H <-> 4L (2H -> 4L engages the front axle).
    bool toggleLowRange();
    // Part-time layouts: 2H -> 4H -> 4L -> 2H. 2H <-> 4H also while moving (shift on the fly), 4L only near a stop.
    // Returns false (and changes nothing) for other layouts or when the speed is too high.
    // Dual-motor layouts: V cycles the drive mode Normal <-> OffRoad (also while moving).
    bool cycleTransfer();
    bool setTransfer(TransferMode mode);
    bool setEvDriveMode(EvDriveMode mode); // false for non-electric vehicles
    EvDriveMode evDriveMode() const { return m_evMode; }
    // Battery state of charge (electric only; R reset restores ElectricConfig::initialSoC).
    void setStateOfCharge(float soc);
    static constexpr float kShiftSpeedLimit = 1.0f; // m/s

    // Driver aid switches (ABS, TC, lockers); take effect on the next tick.
    void setAidControls(const AidControls& controls);
    const AidControls& aidControls() const { return m_controls; }

    // Apply settings that do not require recreation. Call between steps.
    void applyTuning(const VehicleTuning& tuning);
    // True if switching from config a to b needs a new Jolt body / constraint (geometry, wheels, engine, gearbox,
    // layout, mass); false if b differs only in VehicleTuning.
    static bool needsRebuild(const VehicleConfig& a, const VehicleConfig& b);

    const VehicleConfig& config() const { return m_config; }
    const VehicleTelemetry& telemetry() const { return m_telemetry; }
    // Per-wheel ground contact of the last tick (see WheelContact.h).
    const std::array<WheelContactSample, kWheelCount>& contacts() const { return m_contacts; }
    JPH::BodyID bodyId() const { return m_bodyId; }

    // Body pose (body origin position, not the center of mass).
    JPH::RMat44 bodyTransform() const;
    // Wheel world matrix: wheel axis = model local X, Y is up.
    JPH::RMat44 wheelTransform(int wheel) const;

private:
    void buildTireCurves(const VehicleTuning& t);
    float staticGripFriction(float curveFriction, float slip) const;
    float curveSlip(JPH::uint wheel) const;
    void updateEllipseSlip(JPH::uint wheel, float slip);
    void frictionEllipse(JPH::uint wheel, float& ioLongImpulse, float& ioLatImpulse) const;
    DiffMode effectiveDiff(int which) const; // 0 front, 1 rear, 2 center
    float clutchStrength(float throttle) const;
    void applyDifferentials();
    void applySuspensionPerTick(); // compression / rebound damping, bump stops
    void applyAckermann(float steer);
    void applyDrag();
    void prePhysicsElectric(const DriverCommand& cmd, float dt);

    PhysicsWorld& m_world;
    const SurfaceTable& m_surfaces;
    VehicleConfig m_config;
    JPH::BodyID m_bodyId;
    JPH::Body* m_body = nullptr;
    JPH::Ref<JPH::VehicleConstraint> m_constraint;
    JPH::WheeledVehicleController* m_controller = nullptr;
    ElectricVehicleController* m_electric = nullptr; // same object as m_controller for electric vehicles
    ElectricDrive m_evDrive;
    EvOutput m_evOut;
    EvDriveMode m_evMode = EvDriveMode::Normal;
    Battery m_battery;
    JPH::Ref<JPH::VehicleCollisionTester> m_tester;
    std::array<JPH::Ref<JPH::WheelSettingsWV>, kWheelCount> m_wheelSettings;
    // Written from CombineFriction (inside the physics step), read after the step.
    std::array<SurfaceId, kWheelCount> m_contactSurface{};
    // Friction ellipse input per wheel, written in CombineFriction and read by the tire max impulse callback of the
    // same step: normalized slip ratio, normalized slip angle, low-speed blend (0..1).
    struct EllipseSlip {
        float sx = 0.0f, sy = 0.0f, blend = 0.0f;
    };
    std::array<EllipseSlip, kWheelCount> m_ellipseSlip{};

    DriveDirection m_direction = DriveDirection::Forward;
    bool m_lowRange = false;
    TransferMode m_transfer = TransferMode::TwoHigh;
    bool m_revCut = false;
    std::array<float, kWheelCount> m_prevSuspLength{};
    std::array<float, kWheelCount> m_suspVelocity{}; // m/s, + = extending (rebound)
    DriverCommand m_command;
    AidControls m_controls;
    DrivingAids m_aids;
    AidOutput m_aidOut;
    VehicleTelemetry m_telemetry;
    std::array<WheelContactSample, kWheelCount> m_contacts;
};

} // namespace tf
