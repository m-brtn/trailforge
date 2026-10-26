#pragma once

#include "track/DriveRoute.h"

#include <vector>

namespace tf {

// What the autopilot sees of the vehicle each tick: the same things a driver sees or feels (no forces, no contacts).
struct AutopilotInput {
    float x = 0.0f, z = 0.0f;     // body position
    float fx = 0.0f, fz = 1.0f;   // horizontal forward direction (unit)
    float forwardY = 0.0f;        // vertical component of the body forward axis (sine of the pitch, + nose up)
    float upDot = 1.0f;           // body up axis . world up
    float forwardSpeed = 0.0f;    // m/s, + forward
};

// Pedals and steering, like DriverIntent (throttle drives in the selected direction, brake always brakes).
struct AutopilotOutput {
    float throttle = 0.0f, brake = 0.0f, handBrake = 0.0f;
    float steer = 0.0f; // -1..1, right is +
};

struct AutopilotSettings {
    float lookaheadMin = 7.0f;       // m, pure pursuit target distance at a standstill
    float lookaheadPerSpeed = 0.8f;  // s: + speed * this
    float lookaheadMax = 16.0f;      // m
    float cornerLookahead = 0.6f;    // in tight curves at most this times the curve radius (pure pursuit cuts corners
    float lookaheadCornerMin = 3.5f; // by about lookahead^2 / 8 R), but not below this, m
    float lateralAccel = 2.0f;       // m/s^2 allowed in curves (gentle, passengers keep their coffee)
    float planDecel = 1.6f;          // m/s^2 used to slow down before curves
    float planAccel = 1.0f;          // m/s^2 used to speed up after them
    float maxThrottle = 0.65f;       // a calm driver; less while steering (1 - steerEase * |steer|)
    float steerEase = 0.5f;
    float downhillGrade = 0.06f;     // descents steeper than this lower the target speed
    float downhillSlowdown = 3.0f;   // target speed factor 1 - this * (grade - downhillGrade)
    float kp = 0.30f, ki = 0.10f;    // throttle per m/s of speed error, per m of integrated error
    float kb = 0.30f;                // brake per m/s above the target
    // Reset conditions (honest recovery: the vehicle is put back at the route start).
    float offRouteDistance = 12.0f;  // m from the route center line ...
    float offRouteTime = 4.0f;       // ... for this long
    float stuckSpeed = 0.6f;         // m/s below which a vehicle with throttle counts as stuck ...
    float stuckTime = 10.0f;         // ... for this long
    float flippedUpDot = 0.4f;       // body up . world up below this ...
    float flippedTime = 2.5f;        // ... for this long
};

// A simple driver that follows a closed DriveRoute: pure-pursuit steering toward a point ahead on the route, a speed
// profile from the route curvature (slower into curves, a planning deceleration before them) and the pitch (slower
// downhill), and a PI speed controller on the pedals that brakes when too fast. Pure logic: the simulation feeds it
// once per tick and applies the output through the normal pedal and steering smoothing.
class Autopilot {
public:
    explicit Autopilot(const AutopilotSettings& settings = {}) : m_settings(settings) {}

    // A new route (copied) and the vehicle's geometry; forgets the progress.
    void setRoute(const DriveRoute& route, float wheelBase, float maxSteerAngleRad);
    const DriveRoute& route() const { return m_route; }
    bool hasRoute() const { return m_route.valid(); }
    // Forget the progress and the controller state (after a respawn).
    void restart();

    AutopilotOutput update(const AutopilotInput& in, float dt);

    // A reset is wanted (stuck, flipped or off the route for too long); cleared by restart().
    bool wantsReset() const { return m_wantsReset; }
    const char* resetReason() const { return m_resetReason; }

    // Route start: position and heading (rad, 0 = +Z) along the route.
    RoutePoint startPoint() const;
    float startYaw() const;

    // Telemetry (tests, scenario values).
    float progress() const { return m_s; }               // m along the route from its first point
    float routeLength() const { return m_length; }
    float distanceDriven() const { return m_driven; }    // m of route progress since restart()
    float lateralError() const { return m_lateral; }     // m from the center line
    float targetSpeed() const { return m_target; }       // m/s
    float maxLateralError() const { return m_maxLateral; }

    // Route geometry (also used by the tests): nearest arc length to a point, point and heading at an arc length.
    float nearestArc(float x, float z, float* distance = nullptr) const;
    RoutePoint pointAt(float s) const;
    float speedLimitAt(float s) const; // m/s from the curvature profile

private:
    float wrapS(float s) const;
    float localNearest(float x, float z, float* distance) const;

    AutopilotSettings m_settings;
    DriveRoute m_route;
    std::vector<float> m_arc;   // cumulative arc length per point
    std::vector<float> m_limit; // speed limit per point (curvature + planning deceleration), m/s
    std::vector<float> m_curvature; // 1/m per point
    float m_length = 0.0f;
    float m_wheelBase = 2.8f, m_maxSteer = 0.6f;

    bool m_tracking = false; // m_s is valid (local search from it)
    float m_s = 0.0f, m_driven = 0.0f, m_lateral = 0.0f, m_maxLateral = 0.0f, m_target = 0.0f;
    float m_integral = 0.0f;
    float m_offRoute = 0.0f, m_stuck = 0.0f, m_flipped = 0.0f;
    bool m_wantsReset = false;
    const char* m_resetReason = "";
};

} // namespace tf
