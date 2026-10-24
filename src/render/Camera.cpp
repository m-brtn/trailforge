#include "render/Camera.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>

namespace tf {
namespace {

constexpr float kPi = 3.14159265358979f;

float wrapAngle(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

glm::vec3 dirFromYawPitch(float yaw, float pitch) {
    // yaw = 0 looks toward +Z, positive yaw turns toward +X.
    return {std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
}

} // namespace

void Camera::toggleMode() {
    if (m_mode == Mode::Follow) {
        m_mode = Mode::Free;
        const glm::vec3 d = glm::normalize(m_lookAt - m_position);
        m_freeYaw = std::atan2(d.x, d.z);
        m_freePitch = std::asin(std::clamp(d.y, -1.0f, 1.0f));
    } else {
        m_mode = Mode::Follow;
    }
}

void Camera::update(float dt, const CameraInput& in, const glm::vec3& target, const glm::vec3& forward,
                    const CameraRayCast& rayCast) {
    const float sensitivity = 0.005f;
    if (m_mode == Mode::Free) {
        m_freeYaw -= in.mouseDelta.x * sensitivity;
        m_freePitch = std::clamp(m_freePitch - in.mouseDelta.y * sensitivity, -1.5f, 1.5f);
        const glm::vec3 fwd = dirFromYawPitch(m_freeYaw, m_freePitch);
        const glm::vec3 right = glm::normalize(glm::cross(fwd, glm::vec3(0, 1, 0)));
        const float speed = (in.fast ? 40.0f : 12.0f) * dt;
        m_position += (fwd * in.move.z + right * in.move.x + glm::vec3(0, 1, 0) * in.move.y) * speed;
        m_position += fwd * in.wheel * 2.0f;
        m_lookAt = m_position + fwd;
        return;
    }

    // ---- Follow camera
    followDistance = std::clamp(followDistance - in.wheel * 0.8f, 3.5f, 30.0f);
    m_orbitYaw = wrapAngle(m_orbitYaw - in.mouseDelta.x * sensitivity);
    m_orbitPitch = std::clamp(m_orbitPitch + in.mouseDelta.y * sensitivity, -0.25f, 1.2f);

    glm::vec3 flat(forward.x, 0.0f, forward.z);
    const float flatLen = glm::length(flat);
    const float carYaw = flatLen > 0.2f ? std::atan2(flat.x, flat.z) : m_followYaw;
    const float k = 1.0f - std::exp(-followStiffness * dt); // independent of FPS
    if (!m_initialized) {
        m_followYaw = carYaw;
    } else {
        m_followYaw = wrapAngle(m_followYaw + wrapAngle(carYaw - m_followYaw) * k);
    }

    const float yaw = m_followYaw + m_orbitYaw;
    const float basePitch = std::atan2(followHeight, followDistance);
    const float pitch = std::clamp(basePitch + m_orbitPitch, -0.2f, 1.35f);
    const glm::vec3 pivot = target + glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 desired = pivot - dirFromYawPitch(yaw, -pitch) * followDistance;

    // Do not pass through the ground and obstacles: ray from the vehicle to the camera.
    if (rayCast) {
        if (auto hit = rayCast(pivot, desired)) {
            const float len = glm::length(desired - pivot);
            const float safe = std::max(0.0f, *hit * len - 0.35f);
            desired = pivot + (desired - pivot) * (len > 0.0f ? safe / len : 0.0f);
        }
        // Keep the camera above the surface.
        const glm::vec3 above = desired + glm::vec3(0, 30.0f, 0);
        const glm::vec3 below = desired - glm::vec3(0, 2.0f, 0);
        if (auto hit = rayCast(above, below)) {
            const float groundY = above.y + (below.y - above.y) * *hit;
            desired.y = std::max(desired.y, groundY + 0.5f);
        }
    }

    if (!m_initialized) {
        m_position = desired;
        m_lookAt = pivot;
        m_initialized = true;
    } else {
        m_position = glm::mix(m_position, desired, k);
        m_lookAt = glm::mix(m_lookAt, pivot, 1.0f - std::exp(-15.0f * dt));
        // Smoothing may have moved the camera under the surface, so check again.
        if (rayCast) {
            if (auto hit = rayCast(m_lookAt, m_position)) {
                const float len = glm::length(m_position - m_lookAt);
                m_position = m_lookAt + (m_position - m_lookAt) * std::max(0.0f, (*hit * len - 0.35f) / len);
            }
        }
    }
}

glm::mat4 Camera::view() const {
    glm::vec3 dir = m_lookAt - m_position;
    if (glm::length(dir) < 1e-4f) dir = glm::vec3(0, 0, 1);
    return glm::lookAt(m_position, m_position + glm::normalize(dir), glm::vec3(0, 1, 0));
}

glm::mat4 Camera::projection(float aspect) const {
    return glm::perspective(glm::radians(fovYDeg), aspect, nearPlane, farPlane);
}

} // namespace tf
