// Headless checks of the procedural vehicle model (render data only, no GPU): the part list and its flags, and
// that no body vertex enters the space the physics wheels sweep (suspension travel, steering).
// Usage: trailforge_model_tests <test_name>

#include "render/VehicleModel.h"
#include "vehicle/VehicleConfig.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
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

void modelParts() {
    const VehicleVisual v = buildVehicleVisual(VehicleConfig{});
    CHECK(!v.parts.empty(), "no parts");
    std::set<std::string> names;
    std::size_t next = 0;
    for (const VehiclePart& p : v.parts) {
        CHECK(names.insert(p.name).second, "duplicate part '%s'", p.name.c_str());
        CHECK(p.vertexCount > 0, "part '%s' is empty", p.name.c_str());
        CHECK(p.firstVertex == next, "part '%s' range does not follow the previous one", p.name.c_str());
        next = p.firstVertex + p.vertexCount;
    }
    CHECK(next == v.mesh.vertices.size(), "part ranges cover %zu of %zu vertices", next, v.mesh.vertices.size());
    for (const VehiclePart& p : v.parts)
        CHECK(p.parent.empty() || names.count(p.parent), "part '%s' has unknown parent '%s'", p.name.c_str(),
              p.parent.c_str());
    for (const char* n : {"hood", "tailgate", "door_front_left", "door_rear_right", "bumper_front", "fender_front_left",
                          "flare_rear_right", "roof_rails"}) {
        const VehiclePart* p = v.find(n);
        CHECK(p && p->deformable && p->detachable, "'%s' should be deformable and detachable", n);
    }
    for (const char* n : {"mirror_left", "headlight_right", "winch", "tow_hook_front_left", "jerry_can_left"}) {
        const VehiclePart* p = v.find(n);
        CHECK(p && p->detachable, "'%s' should be detachable", n);
    }
    CHECK(v.find("chassis") && !v.find("chassis")->deformable && !v.find("chassis")->detachable, "chassis flags");
    CHECK(v.spares.size() == 1, "%zu spare mounts", v.spares.size());
    for (const SpareMount& s : v.spares) CHECK(names.count(s.parent), "spare '%s' parent", s.name.c_str());
    std::printf("  %zu parts, %zu triangles\n", v.parts.size(), v.mesh.vertices.size() / 3);
}

// Tire size changes a tire kit could make (the body is rebuilt around them).
std::vector<VehicleConfig> wheelVariants() {
    std::vector<VehicleConfig> out(1);
    for (const auto& [r, w] : {std::pair{0.436f, 0.315f}, std::pair{0.43f, 0.33f}, std::pair{0.45f, 0.30f}, std::pair{0.36f, 0.26f}}) {
        VehicleConfig c;
        for (AxleConfig& ax : c.axles) {
            ax.wheelRadius = r;
            ax.wheelWidth = w;
        }
        out.push_back(c);
    }
    return out;
}

// The wheel mesh is exactly the physics wheel: outer radius == wheelRadius, width == wheelWidth, for every
// size, rim ratio and tread, and it follows a wheel change.
void modelWheelSize() {
    const VehicleConfig base;
    const VehicleVisual v = buildVehicleVisual(base);
    for (VehicleConfig c : wheelVariants()) {
        for (const char* tread : {"mud_terrain", "all_terrain"})
            for (float rim : {0.45f, 0.58f, 0.75f}) {
                for (AxleConfig& ax : c.axles) {
                    ax.tread = tread;
                    ax.rimRatio = rim;
                }
                CHECK(c == base || !sameWheelFit(base, c), "a wheel change must rebuild the visual");
                const AxleConfig& ax = c.axles[1];
                const glm::vec2 size = wheelMeshSize(buildAxleWheelMesh(v, ax));
                CHECK(size.x <= ax.wheelRadius + 1e-4f && size.x >= ax.wheelRadius - 0.003f,
                      "r %.3f w %.3f %s rim %.2f: visual radius %.4f", ax.wheelRadius, ax.wheelWidth, tread, rim, size.x);
                CHECK(std::abs(size.y - ax.wheelWidth) <= 1e-4f, "r %.3f w %.3f %s rim %.2f: visual width %.4f",
                      ax.wheelRadius, ax.wheelWidth, tread, rim, size.y);
            }
    }
    const glm::vec2 s = wheelMeshSize(buildAxleWheelMesh(v, base.axles[1]));
    std::printf("  default wheel: visual radius %.4f width %.4f (physics %.3f / %.3f)\n", s.x, s.y,
                base.axles[1].wheelRadius, base.axles[1].wheelWidth);
}

// No body triangle (vertices, edge midpoints, centroid) enters the space a wheel sweeps over the whole suspension travel and steering range, for the
// default wheels and for bigger / smaller tires (the arches and flares are fitted to the wheels).
void modelWheelClearance() {
    for (const VehicleConfig& c : wheelVariants()) {
        const VehicleVisual v = buildVehicleVisual(c);
        const float margin = 0.01f;
        int hits = 0;
        for (int w = 0; w < 4; ++w) {
            const AxleConfig& ax = c.axles[std::size_t(w / 2)];
            const float xw = (w % 2 == 0 ? 0.5f : -0.5f) * ax.track;
            const float zw = (w < 2 ? 0.5f : -0.5f) * c.wheelBase;
            const float hw = 0.5f * ax.wheelWidth + margin, r = ax.wheelRadius + margin;
            const float yHigh = c.attachHeight - ax.suspensionMinLength, yLow = c.attachHeight - ax.suspensionMaxLength;
            const float steer = glm::radians(ax.maxSteerAngleDeg);
            // Vertices plus edge midpoints and centroids, so large triangles cannot span the sweep unseen.
            for (std::size_t si = 0; si < 7 * v.mesh.vertices.size() / 3; ++si) {
                const std::size_t vi = 3 * (si / 7);
                const glm::vec3 &t0 = v.mesh.vertices[vi].position, &t1 = v.mesh.vertices[vi + 1].position,
                                &t2 = v.mesh.vertices[vi + 2].position;
                const glm::vec3 samples[7] = {t0, t1, t2, 0.5f * (t0 + t1), 0.5f * (t1 + t2), 0.5f * (t2 + t0),
                                              (t0 + t1 + t2) / 3.0f};
                const glm::vec3 p = samples[si % 7];
                const float hub = std::fmin(std::fmax(p.y, yLow), yHigh);
                const float dy = p.y - hub;
                if (std::abs(dy) >= r) continue;
                const float reach = std::sqrt(r * r - dy * dy);
                bool inside = false;
                const int steps = steer > 0.0f ? 15 : 1;
                for (int s = 0; s < steps && !inside; ++s) {
                    const float a = steps > 1 ? -steer + 2.0f * steer * float(s) / float(steps - 1) : 0.0f;
                    const float dx = p.x - xw, dz = p.z - zw;
                    const float axial = dx * std::cos(a) - dz * std::sin(a);
                    const float along = dx * std::sin(a) + dz * std::cos(a);
                    inside = std::abs(axial) < hw && std::abs(along) < reach;
                }
                if (inside && hits++ < 5) {
                    const char* part = "?";
                    for (const VehiclePart& vp : v.parts)
                        if (vi >= vp.firstVertex && vi < vp.firstVertex + vp.vertexCount) part = vp.name.c_str();
                    std::printf("  r %.2f w %.2f: point (%.3f, %.3f, %.3f) of '%s' inside the sweep of wheel %d\n",
                                ax.wheelRadius, ax.wheelWidth, p.x, p.y, p.z, part, w);
                }
            }
        }
        CHECK(hits == 0, "r %.2f w %.2f: %d body points inside the wheel sweep", c.axles[0].wheelRadius,
              c.axles[0].wheelWidth, hits);
    }
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"model_parts", modelParts},
        {"model_wheel_size", modelWheelSize},
        {"model_wheel_clearance", modelWheelClearance},
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
        std::printf("unknown test: %s\n", argc > 1 ? argv[1] : "");
        return 2;
    }
    return g_failures == 0 ? 0 : 1;
}
