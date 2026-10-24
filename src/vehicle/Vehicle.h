#pragma once

#include "track/SurfaceProperties.h"
#include "vehicle/DrivingAids.h"
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

struct VehicleTelemetry {
    float speed = 0.0f;         // m/s, magnitude
    float forwardSpeed = 0.0f;  // m/s, projection onto the body +Z axis
    float engineRPM = 0.0f;     // rpm
    int gear = 0;               // -1 = R, 0 = N, 1.. = gears
    float clutchFriction = 0.0f; // 0..1
    bool shifting = false;
    DriveDirection direction = DriveDirection::Forward;
    bool lowRange = false;
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
};

// Off-road vehicle based on Jolt VehicleConstraint + WheeledVehicleController (four-wheel drive).
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
    // Low range only near a stop.
    bool toggleLowRange();
    static constexpr float kShiftSpeedLimit = 1.0f; // m/s

    // Driver aid switches (ABS, TC, lockers); take effect on the next tick.
    void setAidControls(const AidControls& controls);
    const AidControls& aidControls() const { return m_controls; }

    // Apply settings that do not require recreation. Call between steps.
    void applyTuning(const VehicleTuning& tuning);

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
    float staticGripFriction(JPH::uint wheel, float curveFriction, float slip) const;
    float curveSlip(JPH::uint wheel) const;
    void combinedSlip(JPH::uint wheel, float slip, float& ioLong, float& ioLat) const;
    DiffMode effectiveDiff(int which) const; // 0 front, 1 rear, 2 center
    float clutchStrength(float throttle) const;
    void applyDifferentials();

    PhysicsWorld& m_world;
    const SurfaceTable& m_surfaces;
    VehicleConfig m_config;
    JPH::BodyID m_bodyId;
    JPH::Body* m_body = nullptr;
    JPH::Ref<JPH::VehicleConstraint> m_constraint;
    JPH::WheeledVehicleController* m_controller = nullptr;
    JPH::Ref<JPH::VehicleCollisionTester> m_tester;
    std::array<JPH::Ref<JPH::WheelSettingsWV>, kWheelCount> m_wheelSettings;
    // Written from CombineFriction (inside the physics step), read after the step.
    std::array<SurfaceId, kWheelCount> m_contactSurface{};

    DriveDirection m_direction = DriveDirection::Forward;
    bool m_lowRange = false;
    DriverCommand m_command;
    AidControls m_controls;
    DrivingAids m_aids;
    AidOutput m_aidOut;
    VehicleTelemetry m_telemetry;
    std::array<WheelContactSample, kWheelCount> m_contacts;
};

} // namespace tf
