#pragma once

#include <glm/glm.hpp>

#include <functional>
#include <optional>

namespace tf {

// Ray against the physics world (excluding the vehicle): from -> to, returns the hit fraction.
using CameraRayCast = std::function<std::optional<float>(const glm::vec3& from, const glm::vec3& to)>;

struct CameraInput {
    glm::vec2 mouseDelta{0.0f}; // pixels, only while RMB is held
    float wheel = 0.0f;
    glm::vec3 move{0.0f};       // free camera: x right, y up, z forward (-1..1)
    bool fast = false;
    glm::vec2 stick{0.0f};      // gamepad right stick after the dead zone, -1..1 (x right, y down like SDL)
    float vehicleSpeed = 0.0f;  // m/s, the follow camera recenters behind a moving vehicle
};

// Hood camera mount in body space (vehicle.json camera.hood), +Z forward, +X left, +Y up.
struct HoodMount {
    glm::vec3 position{0.0f, 1.0f, 0.9f};
    float pitchDeg = -3.0f;  // look down a little so the hood edge is in view
    float nearPlane = 0.05f; // m
};

// Close chase view, sized from the vehicle's collision bounds in body space (+Z forward, +Y up): the camera sits
// gapPerLength body lengths behind the rear bumper and roofClearance above the roof and looks over the roof at a
// point lookAhead body lengths ahead of the front bumper, lookHeight of the roof height above the body origin.
struct CloseChaseTuning {
    float gapPerLength = 0.8f;
    float roofClearance = 0.12f; // m
    float lookAhead = 0.5f;
    float lookHeight = 0.3f;
    float pivotHeight = 0.5f;    // orbit pivot above the body origin, fraction of the roof height
};

// Right-stick behaviour (gamepad orbit / hood look-around).
struct StickCameraTuning {
    float yawRate = 2.6f;        // rad/s at full deflection (follow orbit)
    float pitchRate = 1.6f;      // rad/s
    float recenterDelay = 1.75f; // s without stick input before the follow camera swings back behind a moving car
    float recenterRate = 2.0f;   // 1/s, exponential return of the orbit offset
    float recenterSpeed = 1.0f;  // m/s, the vehicle counts as driving above this
    float hoodYaw = 1.2f;        // rad, hood look-around at full deflection
    float hoodPitch = 0.4f;      // rad
    float hoodFollow = 12.0f;    // 1/s, how fast the hood look follows the stick and springs back
};

// Perspective camera: follow (vehicle), close follow (lower and nearer chase), hood (rigid on the body) and free. The camera only reads the physics world
// (rays) and does not affect the simulation.
class Camera {
public:
    enum class Mode { Follow, CloseFollow, Hood, Free };

    // C / gamepad camera button: follow -> close follow -> hood -> free -> follow.
    void cycleMode();
    void setMode(Mode m);
    Mode mode() const { return m_mode; }
    // Switch to the free camera at a fixed pose (scripted screenshots).
    void setFreePose(const glm::vec3& position, const glm::vec3& target);
    void setHoodMount(const HoodMount& m) { m_hood = m; }
    const HoodMount& hoodMount() const { return m_hood; }
    // Vehicle collision bounds in body space (m), for the close follow view.
    void setVehicleBounds(const glm::vec3& min, const glm::vec3& max) {
        m_boundsMin = min;
        m_boundsMax = max;
    }
    bool chaseMode() const { return m_mode == Mode::Follow || m_mode == Mode::CloseFollow; }

    // body: interpolated vehicle body transform (world from body space).
    void update(float dt, const CameraInput& input, const glm::mat4& body, const CameraRayCast& rayCast);

    glm::mat4 view() const;
    glm::mat4 projection(float aspect) const;
    glm::vec3 position() const { return m_position; }
    // Near plane of the current mode (the hood camera sits a few centimetres above the hood).
    float activeNearPlane() const { return m_mode == Mode::Hood ? m_hood.nearPlane : nearPlane; }

    float orbitYaw() const { return m_orbitYaw; }
    float orbitPitch() const { return m_orbitPitch; }
    float hoodLookYaw() const { return m_hoodYaw; }
    float hoodLookPitch() const { return m_hoodPitch; }

    float fovYDeg = 60.0f;
    float nearPlane = 0.1f;
    float farPlane = 600.0f;
    float followDistance = 7.5f; // m
    float followHeight = 1.9f;   // m above the body
    float followStiffness = 6.0f; // 1/s, catch-up rate (exponential smoothing)
    StickCameraTuning stickTuning;
    CloseChaseTuning closeChase;

private:
    void updateFollow(float dt, const CameraInput& in, const glm::mat4& body, const CameraRayCast& rayCast);
    void updateHood(float dt, const CameraInput& in, const glm::mat4& body);

    Mode m_mode = Mode::Follow;
    glm::vec3 m_position{0.0f, 5.0f, -50.0f};
    glm::vec3 m_lookAt{0.0f, 1.0f, -40.0f};
    glm::vec3 m_up{0.0f, 1.0f, 0.0f};
    float m_followYaw = 0.0f;    // rad, current smoothed follow direction
    float m_orbitYaw = 0.0f;     // rad, mouse / stick offset
    float m_orbitPitch = 0.0f;   // rad
    float m_stickIdle = 1e9f;    // s since the right stick last moved the follow orbit
    bool m_stickOrbit = false;   // the orbit offset came from the stick (only that one recenters)
    float m_hoodYaw = 0.0f, m_hoodPitch = 0.0f; // rad, hood look-around
    bool m_initialized = false;
    float m_freeYaw = 0.0f, m_freePitch = -0.3f;
    HoodMount m_hood;
    glm::vec3 m_boundsMin{-0.95f, -0.30f, -2.30f}, m_boundsMax{0.95f, 1.30f, 2.30f}; // kit boxes of VehicleShape.h
};

} // namespace tf
