// Headless tests of the camera math: the hood mount follows the body rigidly, the right stick orbits the follow
// camera (frame-rate independent, pitch limits, recentering behind a moving vehicle) and looks around in the hood
// view with a spring back. No window, no gamepad: the stick values are what Input passes after the dead zone.

#include "input/ControlLayout.h"
#include "render/Camera.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <utility>
#include <vector>

using namespace tf;

namespace {

int g_failures = 0;

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("  FAIL %s:%d: %s - ", __FILE__, __LINE__, #cond);      \
            std::printf(__VA_ARGS__);                                           \
            std::printf("\n");                                                  \
            ++g_failures;                                                       \
        }                                                                       \
    } while (0)

glm::mat4 bodyPose(const glm::vec3& p, float yawDeg, float pitchDeg, float rollDeg) {
    const glm::quat q = glm::angleAxis(glm::radians(yawDeg), glm::vec3(0, 1, 0)) *
                        glm::angleAxis(glm::radians(pitchDeg), glm::vec3(1, 0, 0)) *
                        glm::angleAxis(glm::radians(rollDeg), glm::vec3(0, 0, 1));
    glm::mat4 m = glm::mat4_cast(q);
    m[3] = glm::vec4(p, 1.0f);
    return m;
}

// Camera forward / up from the view matrix (rows of the rotation part).
glm::vec3 viewForward(const glm::mat4& v) { return -glm::vec3(v[0][2], v[1][2], v[2][2]); }
glm::vec3 viewUp(const glm::mat4& v) { return glm::vec3(v[0][1], v[1][1], v[2][1]); }

void run(Camera& cam, const CameraInput& in, const glm::mat4& body, double seconds, float hz) {
    const int n = int(std::lround(seconds * hz));
    for (int i = 0; i < n; ++i) cam.update(1.0f / hz, in, body, {});
}

void hoodMount() {
    Camera cam;
    HoodMount m;
    m.position = {0.0f, 0.95f, 0.95f};
    m.pitchDeg = -4.0f;
    m.nearPlane = 0.05f;
    cam.setHoodMount(m);
    CHECK(cam.mode() == Camera::Mode::Follow, "starts in follow");
    cam.cycleMode();
    CHECK(cam.mode() == Camera::Mode::Hood, "C: follow -> hood");
    float worstPos = 0.0f, worstDir = 1.0f, worstUp = 1.0f;
    for (const auto& [yaw, pitch, roll] : {std::tuple{0.0f, 0.0f, 0.0f}, std::tuple{35.0f, 12.0f, -8.0f},
                                           std::tuple{-120.0f, -20.0f, 25.0f}, std::tuple{179.0f, 5.0f, 40.0f}}) {
        const glm::mat4 body = bodyPose({12.0f, 3.0f, -40.0f}, yaw, pitch, roll);
        cam.update(1.0f / 60.0f, CameraInput{}, body, {});
        const glm::vec3 want = glm::vec3(body * glm::vec4(m.position, 1.0f));
        const glm::vec3 local(0.0f, std::sin(glm::radians(m.pitchDeg)), std::cos(glm::radians(m.pitchDeg)));
        const glm::vec3 wantDir = glm::normalize(glm::vec3(body * glm::vec4(local, 0.0f)));
        const glm::mat4 v = cam.view();
        worstPos = std::max(worstPos, glm::length(cam.position() - want));
        worstDir = std::min(worstDir, glm::dot(viewForward(v), wantDir));
        worstUp = std::min(worstUp, glm::dot(viewUp(v), glm::normalize(glm::vec3(body[1]))));
    }
    // Rigid: exactly on the mount, looking along the body (the view's up is the body's up up to the pitch tilt).
    CHECK(worstPos < 1e-4f, "hood camera %.6f m off the mount", worstPos);
    CHECK(worstDir > 0.99999f, "hood view direction dot %.6f", worstDir);
    CHECK(worstUp > std::cos(glm::radians(4.5f)), "hood view up dot %.5f", worstUp);
    CHECK(cam.activeNearPlane() == 0.05f, "hood near plane %.3f", cam.activeNearPlane());
    cam.cycleMode();
    CHECK(cam.mode() == Camera::Mode::Free && cam.activeNearPlane() == cam.nearPlane, "hood -> free, near plane back");
    CHECK(viewUp(cam.view()).y > 0.0f, "free camera upright");
    cam.cycleMode();
    CHECK(cam.mode() == Camera::Mode::Follow, "free -> follow");
    std::printf("  hood mount: %.1e m, direction dot %.6f over 4 body poses\n", worstPos, worstDir);
}

void stickDeadZone() {
    const StickValue z = rightStickValue(0.1f, 0.08f);
    CHECK(z.x == 0.0f && z.y == 0.0f, "inside the dead zone: %.3f %.3f", z.x, z.y);
    const StickValue f = rightStickValue(1.0f, 0.0f);
    CHECK(std::abs(f.x - 1.0f) < 1e-6f && f.y == 0.0f, "full right %.3f %.3f", f.x, f.y);
    const StickValue h = rightStickValue(0.0f, -0.5f);
    const float want = std::pow((0.5f - kRightStickDeadZone) / (1.0f - kRightStickDeadZone), 2.0f);
    CHECK(std::abs(h.y + want) < 1e-5f && h.x == 0.0f, "half up %.4f (want %.4f)", h.y, -want);
    const StickValue d = rightStickValue(0.9f, 0.9f); // corner of a square gate: clamped to 1
    CHECK(std::abs(std::hypot(d.x, d.y) - 1.0f) < 1e-5f, "diagonal magnitude %.4f", std::hypot(d.x, d.y));
}

void stickOrbit() {
    const glm::mat4 body = bodyPose({0.0f, 1.0f, 0.0f}, 0.0f, 0.0f, 0.0f);
    CameraInput right;
    right.stick = {1.0f, 0.0f};
    // Rate in rad/s, independent of the frame rate.
    float yaw[2];
    int k = 0;
    for (float hz : {60.0f, 144.0f}) {
        Camera cam;
        run(cam, CameraInput{}, body, 0.2, hz);
        run(cam, right, body, 0.5, hz);
        yaw[k++] = cam.orbitYaw();
    }
    const float want = -Camera{}.stickTuning.yawRate * 0.5f;
    CHECK(std::abs(yaw[0] - want) < 1e-3f && std::abs(yaw[1] - want) < 1e-3f, "yaw %.4f / %.4f, want %.4f", yaw[0],
          yaw[1], want);
    // Pitch limits like the mouse orbit.
    Camera cam;
    CameraInput down;
    down.stick = {0.0f, 1.0f};
    run(cam, down, body, 3.0, 60.0f);
    CHECK(std::abs(cam.orbitPitch() - 1.2f) < 1e-4f, "pitch upper limit %.3f", cam.orbitPitch());
    CameraInput up;
    up.stick = {0.0f, -1.0f};
    run(cam, up, body, 3.0, 60.0f);
    CHECK(std::abs(cam.orbitPitch() + 0.25f) < 1e-4f, "pitch lower limit %.3f", cam.orbitPitch());

    // Recenter: parked, the offset stays; driving, it holds for the delay and then swings back behind the car.
    Camera rc;
    run(rc, right, body, 0.4, 60.0f);
    const float held = rc.orbitYaw();
    CameraInput parked;
    run(rc, parked, body, 5.0, 60.0f);
    CHECK(std::abs(rc.orbitYaw() - held) < 1e-5f, "parked: yaw %.4f moved from %.4f", rc.orbitYaw(), held);
    CameraInput driving;
    driving.vehicleSpeed = 10.0f;
    run(rc, right, body, 0.1, 60.0f);
    const float start = rc.orbitYaw();
    run(rc, driving, body, 1.5, 60.0f);
    CHECK(std::abs(rc.orbitYaw() - start) < 1e-5f, "driving: yaw %.4f moved before the delay (%.4f)", rc.orbitYaw(), start);
    run(rc, driving, body, 2.5, 60.0f);
    CHECK(std::abs(rc.orbitYaw()) < 0.05f * std::abs(start), "driving: yaw %.4f after 4 s (from %.4f)", rc.orbitYaw(), start);
    // A held stick never recenters, even while driving.
    CameraInput holdDrive = right;
    holdDrive.vehicleSpeed = 10.0f;
    holdDrive.stick = {0.3f, 0.0f};
    Camera hd;
    run(hd, holdDrive, body, 4.0, 60.0f);
    CHECK(hd.orbitYaw() < -1.0f, "held stick while driving: yaw %.3f", hd.orbitYaw());
    // The mouse orbit is not recentered (it stays where the player left it).
    Camera ms;
    CameraInput mouse;
    mouse.mouseDelta = {100.0f, 0.0f};
    ms.update(1.0f / 60.0f, mouse, body, {});
    const float mouseYaw = ms.orbitYaw();
    run(ms, driving, body, 5.0, 60.0f);
    CHECK(std::abs(ms.orbitYaw() - mouseYaw) < 1e-5f && mouseYaw < -0.4f, "mouse orbit %.3f -> %.3f", mouseYaw,
          ms.orbitYaw());
    std::printf("  orbit %.3f rad after 0.5 s at 60 / 144 Hz, recentered to %.4f rad after 4 s at 10 m/s\n", yaw[0],
                rc.orbitYaw());
}

void hoodLook() {
    const glm::mat4 body = bodyPose({0.0f, 1.0f, 0.0f}, 30.0f, 0.0f, 0.0f);
    Camera cam;
    cam.setMode(Camera::Mode::Hood);
    CameraInput in;
    in.stick = {1.0f, 0.0f};
    run(cam, in, body, 1.0, 60.0f);
    const float limit = cam.stickTuning.hoodYaw;
    CHECK(std::abs(cam.hoodLookYaw() + limit) < 1e-3f, "hood look right %.4f (limit %.3f)", cam.hoodLookYaw(), limit);
    // Looking right: the view turns toward the body's right (-X in body space).
    const glm::vec3 f = viewForward(cam.view());
    const glm::vec3 bodyRight = -glm::vec3(body[0]);
    CHECK(glm::dot(f, bodyRight) > 0.8f, "hood look direction %.3f", glm::dot(f, bodyRight));
    in.stick = {0.0f, -1.0f};
    run(cam, in, body, 1.0, 60.0f);
    CHECK(std::abs(cam.hoodLookPitch() - cam.stickTuning.hoodPitch) < 1e-3f && std::abs(cam.hoodLookYaw()) < 1e-3f,
          "hood look up %.4f yaw %.4f", cam.hoodLookPitch(), cam.hoodLookYaw());
    run(cam, CameraInput{}, body, 1.0, 60.0f);
    CHECK(std::abs(cam.hoodLookPitch()) < 1e-3f && std::abs(cam.hoodLookYaw()) < 1e-3f, "springs back: %.4f %.4f",
          cam.hoodLookPitch(), cam.hoodLookYaw());
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"camera_hood_mount", hoodMount},
        {"camera_stick_dead_zone", stickDeadZone},
        {"camera_stick_orbit", stickOrbit},
        {"camera_hood_look", hoodLook},
    };
    int ran = 0;
    for (const auto& [name, fn] : tests) {
        if (argc > 1 && std::strcmp(argv[1], name) != 0) continue;
        std::printf("[ RUN ] %s\n", name);
        const int before = g_failures;
        fn();
        std::printf("[%s] %s\n", g_failures == before ? "  OK " : "FAIL ", name);
        ++ran;
    }
    if (ran == 0) {
        std::printf("unknown test '%s'\n", argc > 1 ? argv[1] : "");
        return 2;
    }
    return g_failures == 0 ? 0 : 1;
}
