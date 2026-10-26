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

void Camera::setMode(Mode m) {
    if (m == m_mode) return;
    if (m == Mode::Free) {
        const glm::vec3 d = glm::normalize(m_lookAt - m_position);
        m_freeYaw = std::atan2(d.x, d.z);
        m_freePitch = std::asin(std::clamp(d.y, -1.0f, 1.0f));
    }
    // Back to the chase view: start from the current pose behind the vehicle again (no sweep from the hood).
    if ((m == Mode::Follow || m == Mode::CloseFollow) && m_mode == Mode::Hood) m_initialized = false;
    m_hoodYaw = m_hoodPitch = 0.0f;
    m_up = glm::vec3(0.0f, 1.0f, 0.0f);
    m_mode = m;
}

void Camera::cycleMode() {
    setMode(m_mode == Mode::Follow        ? Mode::CloseFollow
            : m_mode == Mode::CloseFollow ? Mode::Hood
            : m_mode == Mode::Hood        ? Mode::Free
                                          : Mode::Follow);
}

void Camera::setFreePose(const glm::vec3& position, const glm::vec3& target) {
    m_mode = Mode::Free;
    m_up = glm::vec3(0.0f, 1.0f, 0.0f);
    m_position = position;
    const glm::vec3 d = glm::normalize(target - position);
    m_freeYaw = std::atan2(d.x, d.z);
    m_freePitch = std::asin(std::clamp(d.y, -1.0f, 1.0f));
    m_lookAt = position + d;
}

void Camera::update(float dt, const CameraInput& in, const glm::mat4& body, const CameraRayCast& rayCast) {
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
    if (m_mode == Mode::Hood) {
        updateHood(dt, in, body);
        return;
    }
    updateFollow(dt, in, body, rayCast);
}

void Camera::updateHood(float dt, const CameraInput& in, const glm::mat4& body) {
    // Rigid on the body: the interpolated body transform already moves smoothly between physics ticks.
    const StickCameraTuning& t = stickTuning;
    const float k = 1.0f - std::exp(-t.hoodFollow * dt);
    if (in.stick != glm::vec2(0.0f)) {
        // Look-around: the deflection is the look offset, it springs back when the stick is released.
        m_hoodYaw += (-in.stick.x * t.hoodYaw - m_hoodYaw) * k;
        m_hoodPitch += (-in.stick.y * t.hoodPitch - m_hoodPitch) * k;
    } else if (in.mouseDelta != glm::vec2(0.0f)) {
        m_hoodYaw = std::clamp(m_hoodYaw - in.mouseDelta.x * 0.005f, -t.hoodYaw, t.hoodYaw);
        m_hoodPitch = std::clamp(m_hoodPitch - in.mouseDelta.y * 0.005f, -t.hoodPitch, t.hoodPitch);
    } else {
        m_hoodYaw -= m_hoodYaw * k;
        m_hoodPitch -= m_hoodPitch * k;
    }
    const glm::vec3 local = dirFromYawPitch(m_hoodYaw, m_hoodPitch + glm::radians(m_hood.pitchDeg));
    m_position = glm::vec3(body * glm::vec4(m_hood.position, 1.0f));
    m_lookAt = m_position + glm::normalize(glm::vec3(body * glm::vec4(local, 0.0f)));
    m_up = glm::normalize(glm::vec3(body[1]));
    m_initialized = false; // the follow camera starts behind the vehicle when it comes back
}

void Camera::updateFollow(float dt, const CameraInput& in, const glm::mat4& body, const CameraRayCast& rayCast) {
    const float sensitivity = 0.005f;
    const StickCameraTuning& st = stickTuning;
    const glm::vec3 target(body[3]);
    const glm::vec3 forward(body[2]);
    const bool close = m_mode == Mode::CloseFollow;
    if (!close) followDistance = std::clamp(followDistance - in.wheel * 0.8f, 3.5f, 30.0f);
    if (in.mouseDelta != glm::vec2(0.0f)) m_stickOrbit = false;
    m_orbitYaw = wrapAngle(m_orbitYaw - in.mouseDelta.x * sensitivity);
    m_orbitPitch = std::clamp(m_orbitPitch + in.mouseDelta.y * sensitivity, -0.25f, 1.2f);
    // Right stick: the same directions as dragging the mouse (right swings the camera to the left of the car).
    if (in.stick != glm::vec2(0.0f)) {
        m_orbitYaw = wrapAngle(m_orbitYaw - in.stick.x * st.yawRate * dt);
        m_orbitPitch = std::clamp(m_orbitPitch + in.stick.y * st.pitchRate * dt, -0.25f, 1.2f);
        m_stickOrbit = true;
        m_stickIdle = 0.0f;
    } else {
        m_stickIdle += dt;
    }
    if (m_stickOrbit && m_stickIdle > st.recenterDelay && in.vehicleSpeed > st.recenterSpeed) {
        const float r = std::exp(-st.recenterRate * dt);
        m_orbitYaw *= r;
        m_orbitPitch *= r;
    }

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
    // Rig: the orbit pivot, the camera's distance and elevation from it, and the look target.
    glm::vec3 pivot = target + glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 look = pivot;
    float distance = followDistance;
    float basePitch = std::atan2(followHeight, followDistance);
    if (close) {
        const CloseChaseTuning& c = closeChase;
        const float length = m_boundsMax.z - m_boundsMin.z;
        const float pivotY = c.pivotHeight * m_boundsMax.y;
        const float back = -m_boundsMin.z + c.gapPerLength * length;  // behind the body origin
        const float rise = m_boundsMax.y + c.roofClearance - pivotY; // above the pivot
        pivot = target + glm::vec3(0.0f, pivotY, 0.0f);
        distance = std::hypot(back, rise);
        basePitch = std::atan2(rise, back);
        // The look target ahead of the car moves back onto the car as the orbit swings away from behind it.
        const float ahead = (m_boundsMax.z + c.lookAhead * length) * std::max(0.0f, std::cos(m_orbitYaw)) *
                            std::cos(m_orbitPitch);
        look = target + glm::vec3(0.0f, c.lookHeight * m_boundsMax.y, 0.0f) + dirFromYawPitch(yaw, 0.0f) * ahead;
    }
    const float pitch = std::clamp(basePitch + m_orbitPitch, -0.2f, 1.35f);
    glm::vec3 desired = pivot - dirFromYawPitch(yaw, -pitch) * distance;

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
        m_lookAt = look;
        m_initialized = true;
    } else {
        m_position = glm::mix(m_position, desired, k);
        m_lookAt = glm::mix(m_lookAt, look, 1.0f - std::exp(-15.0f * dt));
        // Smoothing may have moved the camera under the surface, so check again.
        if (rayCast) {
            const glm::vec3 from = close ? pivot : m_lookAt;
            if (auto hit = rayCast(from, m_position)) {
                const float len = glm::length(m_position - from);
                m_position = from + (m_position - from) * std::max(0.0f, (*hit * len - 0.35f) / len);
            }
        }
    }
}

glm::mat4 Camera::view() const {
    glm::vec3 dir = m_lookAt - m_position;
    if (glm::length(dir) < 1e-4f) dir = glm::vec3(0, 0, 1);
    return glm::lookAt(m_position, m_position + glm::normalize(dir), m_up);
}

glm::mat4 Camera::projection(float aspect) const {
    return glm::perspective(glm::radians(fovYDeg), aspect, activeNearPlane(), farPlane);
}

} // namespace tf
