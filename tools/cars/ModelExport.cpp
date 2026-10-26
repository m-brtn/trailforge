// Writes the procedural vehicle model at rest as a Wavefront OBJ in body space (+X left, +Y up, +Z forward, m):
// body parts as named objects, the four road wheels at the hub positions the simulation settles to on flat
// ground, and the spare wheels. Used by tools/cars/compare_silhouettes.py (silhouette IoU against the traced
// concept views) without a window or GPU.
// Usage: trailforge_model_export <vehicle.json | builtin> <out.obj>

#include "render/VehicleModel.h"
#include "sim/Simulation.h"
#include "vehicle/VehicleDefinition.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyLock.h>

#include <glm/gtc/matrix_transform.hpp>

#include <cstdio>
#include <string>
#include <vector>

using namespace tf;

namespace {

void writeMesh(std::FILE* f, const MeshData& m, std::size_t first, std::size_t count, const glm::mat4& xf,
               const std::string& name, std::size_t& base) {
    std::fprintf(f, "o %s\n", name.c_str());
    for (std::size_t i = first; i < first + count; ++i) {
        const glm::vec3 p = glm::vec3(xf * glm::vec4(m.vertices[i].position, 1.0f));
        const glm::vec3 c = m.vertices[i].color;
        std::fprintf(f, "v %.5f %.5f %.5f %.3f %.3f %.3f\n", p.x, p.y, p.z, c.r, c.g, c.b);
    }
    for (std::size_t i = 0; i + 2 < count; i += 3)
        std::fprintf(f, "f %zu %zu %zu\n", base + i + 1, base + i + 2, base + i + 3);
    base += count;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: trailforge_model_export <vehicle.json | builtin> <out.obj>\n");
        return 2;
    }
    VehicleConfig config;
    if (std::string(argv[1]) != "builtin") {
        VehicleDefinition def;
        std::vector<std::string> errors;
        if (!loadVehicleDefinition(argv[1], def, errors)) {
            for (const std::string& e : errors) std::fprintf(stderr, "%s\n", e.c_str());
            return 2;
        }
        config = def.config;
    }
    SimulationSettings settings;
    settings.layout = TrackLayout::Flat;
    settings.vehicle = config;
    Simulation sim(settings);
    for (int i = 0; i < 480; ++i) sim.tick(); // 4 s: settled on the springs
    const RenderSnapshot& s = sim.currentSnapshot();
    const JPH::RMat44 bodyXf = JPH::RMat44::sRotationTranslation(s.body.rotation, JPH::RVec3(s.body.position));
    const JPH::RMat44 inv = bodyXf.InversedRotationTranslation();
    {
        const JPH::BodyLockRead lock(sim.world().system().GetBodyLockInterface(), sim.vehicle().bodyId());
        const JPH::Body& body = lock.GetBody();
        const JPH::MotionProperties* mp = body.GetMotionProperties();
        const JPH::Mat44 I = mp->GetLocalSpaceInverseInertia().Inversed3x3(); // body axes
        const JPH::Vec3 com = JPH::Vec3(inv * body.GetCenterOfMassPosition());
        std::printf("mass %.1f kg, COM body (%.3f %.3f %.3f), inertia Ixx Iyy Izz %.0f %.0f %.0f kg m^2, ride height of "
                    "the body origin %.3f m\n",
                    1.0f / mp->GetInverseMass(), com.GetX(), com.GetY(), com.GetZ(), I(0, 0), I(1, 1), I(2, 2), s.body.position.GetY());
    }

    const VehicleVisual visual = buildVehicleVisual(config);
    std::FILE* f = std::fopen(argv[2], "w");
    if (!f) {
        std::fprintf(stderr, "cannot write %s\n", argv[2]);
        return 1;
    }
    std::fprintf(f, "# trailforge_model_export %s (%s)\n", config.id.c_str(), config.visualModel.c_str());
    std::size_t base = 0;
    const MeshData arm = buildUnitTubeMesh(0.021f), shaft = buildUnitTubeMesh(0.028f);
    const MeshData shockBody = buildShockBodyMesh(), shockShaft = buildShockShaftMesh();
    const MeshData spring = buildUnitSpringMesh(visual.coil);
    for (const VehiclePart& p : visual.parts)
        writeMesh(f, visual.mesh, p.firstVertex, p.vertexCount, glm::mat4(1.0f), p.name, base);
    for (int w = 0; w < 4; ++w) {
        const JPH::Vec3 hub = JPH::Vec3(inv * JPH::RVec3(s.wheels[std::size_t(w)].position));
        const AxleConfig& ax = config.axles[std::size_t(w / 2)];
        const MeshData wheel = buildAxleWheelMesh(visual, ax);
        glm::mat4 xf = glm::translate(glm::mat4(1.0f), glm::vec3(hub.GetX(), hub.GetY(), hub.GetZ()));
        if (hub.GetX() < 0.0f) xf = glm::rotate(xf, glm::pi<float>(), glm::vec3(0, 1, 0));
        writeMesh(f, wheel, 0, wheel.vertices.size(), xf, "wheel_" + std::to_string(w), base);
        std::printf("wheel %d hub %.4f %.4f %.4f\n", w, hub.GetX(), hub.GetY(), hub.GetZ());
        // Suspension as the app draws it (Application::buildDraws).
        const glm::vec3 h(hub.GetX(), hub.GetY(), hub.GetZ());
        const SuspensionPose p = suspensionPose(h, h.z, visual.mounts[std::size_t(w / 2)]);
        const glm::vec3 dir = glm::normalize(p.shockTop - p.shockLow);
        const std::string n = std::to_string(w);
        writeMesh(f, arm, 0, arm.vertices.size(), segmentMatrix(p.armPivotA, p.armOuter), "arm_a_" + n, base);
        writeMesh(f, arm, 0, arm.vertices.size(), segmentMatrix(p.armPivotB, p.armOuter), "arm_b_" + n, base);
        writeMesh(f, shaft, 0, shaft.vertices.size(), segmentMatrix(p.shaftInner, p.shaftOuter), "half_shaft_" + n, base);
        writeMesh(f, shockBody, 0, shockBody.vertices.size(), aimMatrix(p.shockLow, p.shockTop), "shock_" + n, base);
        writeMesh(f, shockShaft, 0, shockShaft.vertices.size(), aimMatrix(p.shockTop, p.shockLow), "shock_shaft_" + n,
                  base);
        writeMesh(f, spring, 0, spring.vertices.size(), segmentMatrix(p.shockTop, p.shockLow + dir * kSpringPerch),
                  "spring_" + n, base);
    }
    for (const SpareMount& sp : visual.spares) {
        const AxleConfig& ax = config.axles[1];
        const MeshData wheel = buildAxleWheelMesh(visual, ax);
        writeMesh(f, wheel, 0, wheel.vertices.size(), spareWheelMatrix(sp, ax.wheelRadius, ax.wheelWidth), sp.name,
                  base);
    }
    std::fclose(f);
    std::printf("wrote %s: %zu vertices\n", argv[2], base);
    return 0;
}
