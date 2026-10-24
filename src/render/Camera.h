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
};

// Perspective camera: follow (vehicle) and free. The camera only reads
// the physics world (rays) and does not affect the simulation.
class Camera {
public:
    enum class Mode { Follow, Free };

    void toggleMode();
    Mode mode() const { return m_mode; }
    // Switch to the free camera at a fixed pose (scripted screenshots).
    void setFreePose(const glm::vec3& position, const glm::vec3& target);

    // target is the interpolated body position, forward is its +Z axis (world).
    void update(float dt, const CameraInput& input, const glm::vec3& target, const glm::vec3& forward,
                const CameraRayCast& rayCast);

    glm::mat4 view() const;
    glm::mat4 projection(float aspect) const;
    glm::vec3 position() const { return m_position; }

    float fovYDeg = 60.0f;
    float nearPlane = 0.1f;
    float farPlane = 600.0f;
    float followDistance = 8.0f; // m
    float followHeight = 2.6f;   // m above the body
    float followStiffness = 6.0f; // 1/s, catch-up rate (exponential smoothing)

private:
    Mode m_mode = Mode::Follow;
    glm::vec3 m_position{0.0f, 5.0f, -50.0f};
    glm::vec3 m_lookAt{0.0f, 1.0f, -40.0f};
    float m_followYaw = 0.0f;    // rad, current smoothed follow direction
    float m_orbitYaw = 0.0f;     // rad, mouse offset
    float m_orbitPitch = 0.0f;   // rad
    bool m_initialized = false;
    float m_freeYaw = 0.0f, m_freePitch = -0.3f;
};

} // namespace tf
