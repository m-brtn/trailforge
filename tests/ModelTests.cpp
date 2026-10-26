// Headless checks of the vehicle models (render data only, no GPU): the part list and its flags, that no body vertex
// enters the space the physics wheels sweep (suspension travel, steering), and the Blender body (body.glb) path.
// Usage: trailforge_model_tests <test_name>

#include "render/VehicleModel.h"
#include "vehicle/VehicleBody.h"
#include "vehicle/VehicleConfig.h"
#include "vehicle/VehicleDefinition.h"
#include "vehicle/WheelSweep.h"

#include <algorithm>

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

// Base configuration of every catalog vehicle (one per visual model at least), each also with the traced body
// (bodyShape "traced"; the shipped vehicles use the kit body).
std::vector<VehicleConfig> catalogConfigs() {
    std::vector<std::string> errors;
    std::vector<VehicleConfig> out;
    for (const VehicleDefinition& d : loadVehicleCatalog(TRAILFORGE_SOURCE_DIR "/assets/vehicles", errors)) {
        CHECK(d.config.bodyShape == "kit", "%s: bodyShape '%s'", d.config.id.c_str(), d.config.bodyShape.c_str());
        out.push_back(d.config);
        out.push_back(d.config);
        out.back().bodyShape = "traced";
    }
    CHECK(errors.empty() && out.size() >= 4, "catalog: %zu vehicles, %zu errors", out.size(), errors.size());
    return out;
}

void checkParts(const VehicleConfig& c) {
    const VehicleVisual v = buildVehicleVisual(c);
    const bool ev = c.visualModel == "trailforge_ev";
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
    for (const char* n : {"mirror_left", "headlight_right", "tow_hook_front_left", "taillight_left", "light_bar"}) {
        const VehiclePart* p = v.find(n);
        CHECK(p && p->detachable, "'%s' should be detachable", n);
    }
    for (const char* n : ev ? std::vector<const char*>{"front_panel", "battery_pack", "motor_front", "motor_rear"}
                            : std::vector<const char*>{"winch", "jerry_can_left", "grille", "skid_plate"})
        CHECK(v.find(n), "%s: missing part '%s'", c.visualModel.c_str(), n);
    // Only what the concept sheet shows: no snorkel or ladder on either model (v0.15).
    for (const char* n : ev ? std::vector<const char*>{"winch", "snorkel", "grille", "jerry_can_left", "ladder"}
                            : std::vector<const char*>{"battery_pack", "motor_front", "snorkel", "ladder"})
        CHECK(!v.find(n), "%s: unexpected part '%s'", c.visualModel.c_str(), n);
    // The bumpers are the front- and rearmost body panels on the center line (outward-facing triangles spanning
    // x = 0), ahead of the grille / front panel and behind the tailgate.
    auto reach = [&](const char* name, float zs) {
        const VehiclePart* p = v.find(name);
        float r = -1e9f;
        for (std::size_t i = p ? p->firstVertex : 0; p && i + 2 < p->firstVertex + p->vertexCount; i += 3) {
            const glm::vec3 &a = v.mesh.vertices[i].position, &b = v.mesh.vertices[i + 1].position,
                            &d = v.mesh.vertices[i + 2].position;
            const glm::vec3 n = glm::cross(b - a, d - a);
            if (std::fmin(a.x, std::fmin(b.x, d.x)) > 0.0f || std::fmax(a.x, std::fmax(b.x, d.x)) < 0.0f) continue;
            if (zs * n.z <= 0.0f) continue;
            r = std::fmax(r, zs * (a.z + b.z + d.z) / 3.0f);
        }
        return r;
    };
    const float front = reach("bumper_front", 1.0f), rear = reach("bumper_rear", -1.0f);
    for (const char* n : {"body_shell", "hood", ev ? "front_panel" : "grille"})
        CHECK(front >= reach(n, 1.0f) - 0.01f, "%s: bumper_front (%.3f) behind '%s' (%.3f)", c.visualModel.c_str(),
              front, n, reach(n, 1.0f));
    for (const char* n : {"body_shell", "tailgate"})
        CHECK(rear >= reach(n, -1.0f) - 0.01f, "%s: bumper_rear (%.3f) ahead of '%s' (%.3f)", c.visualModel.c_str(), rear,
              n, reach(n, -1.0f));
    // No antennas or mud flaps on any model (style rule).
    for (const VehiclePart& p : v.parts)
        CHECK(p.name.find("antenna") == std::string::npos && p.name.find("mud_flap") == std::string::npos,
              "%s: part '%s' is not allowed", c.visualModel.c_str(), p.name.c_str());
    CHECK(v.find("chassis") && !v.find("chassis")->deformable && !v.find("chassis")->detachable, "chassis flags");
    CHECK(v.spares.size() == (ev ? 0u : 1u), "%zu spare mounts (the concept has one on the SUV tailgate only)",
          v.spares.size());
    for (const SpareMount& s : v.spares) CHECK(names.count(s.parent), "spare '%s' parent", s.name.c_str());
    std::printf("  %s (%s): %zu parts, %zu triangles\n", c.visualModel.c_str(), c.bodyShape.c_str(), v.parts.size(),
                v.mesh.vertices.size() / 3);
}

void modelParts() {
    checkParts(VehicleConfig{});
    for (const VehicleConfig& c : catalogConfigs()) checkParts(c);
}

// Every catalog vehicle with its own wheels and with the tire sizes a tire kit could fit (the body is rebuilt
// around them).
std::vector<VehicleConfig> wheelVariants() {
    std::vector<VehicleConfig> out{VehicleConfig{}};
    for (const VehicleConfig& base : catalogConfigs()) {
        out.push_back(base);
        for (const auto& [r, w] :
             {std::pair{0.436f, 0.315f}, std::pair{0.43f, 0.33f}, std::pair{0.45f, 0.30f}, std::pair{0.36f, 0.26f}}) {
            VehicleConfig c = base;
            for (AxleConfig& ax : c.axles) {
                ax.wheelRadius = r;
                ax.wheelWidth = w;
            }
            out.push_back(c);
        }
    }
    return out;
}

// The wheel mesh is exactly the physics wheel: outer radius == wheelRadius, width == wheelWidth, for every
// size, rim ratio and tread, and it follows a wheel change.
void modelWheelSize() {
    const VehicleConfig base;
    const VehicleVisual v = buildVehicleVisual(base);
    for (VehicleConfig c : wheelVariants()) {
        const VehicleVisual model = buildVehicleVisual(c);
        for (const char* tread : {"mud_terrain", "all_terrain"})
            for (float rim : {0.45f, 0.58f, 0.75f}) {
                for (AxleConfig& ax : c.axles) {
                    ax.tread = tread;
                    ax.rimRatio = rim;
                }
                CHECK(c == base || !sameWheelFit(base, c), "a wheel change must rebuild the visual");
                const AxleConfig& ax = c.axles[1];
                const glm::vec2 size = wheelMeshSize(buildAxleWheelMesh(model, ax));
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
        const auto sweeps = wheelSweeps(c);
        int hits = 0;
        // Vertices plus edge midpoints and centroids, so large triangles cannot span the sweep unseen.
        for (std::size_t si = 0; si < 7 * v.mesh.vertices.size() / 3; ++si) {
            const std::size_t vi = 3 * (si / 7);
            const glm::vec3 &t0 = v.mesh.vertices[vi].position, &t1 = v.mesh.vertices[vi + 1].position,
                            &t2 = v.mesh.vertices[vi + 2].position;
            const glm::vec3 samples[7] = {t0, t1, t2, 0.5f * (t0 + t1), 0.5f * (t1 + t2), 0.5f * (t2 + t0),
                                          (t0 + t1 + t2) / 3.0f};
            const glm::vec3 p = samples[si % 7];
            int w = 0;
            if (wheelSweepClearance(sweeps, p.x, p.y, p.z, &w) >= kBodyClearance) continue;
            if (hits++ < 5) {
                const char* part = "?";
                for (const VehiclePart& vp : v.parts)
                    if (vi >= vp.firstVertex && vi < vp.firstVertex + vp.vertexCount) part = vp.name.c_str();
                std::printf("  %s r %.2f w %.2f: point (%.3f, %.3f, %.3f) of '%s' inside the sweep of wheel %d\n",
                            c.visualModel.c_str(), c.axles[0].wheelRadius, c.axles[0].wheelWidth, p.x, p.y, p.z, part, w);
            }
        }
        CHECK(hits == 0, "%s r %.2f w %.2f: %d body points inside the wheel sweep", c.visualModel.c_str(),
              c.axles[0].wheelRadius, c.axles[0].wheelWidth, hits);
    }
}

// The test vehicle with a Blender body (tests/data/vehicles/suv_glb_test): the visual is the body.glb (parts,
// light roles on the shader's light ids, paint from vehicle.json), clear of every wheel sweep it can be driven with;
// wheels and suspension stay procedural; a missing file falls back to the procedural body.
void modelBodyGlb() {
    VehicleDefinition def;
    std::vector<std::string> errors;
    const bool ok = loadVehicleDefinition(TRAILFORGE_SOURCE_DIR "/tests/data/vehicles/suv_glb_test/vehicle.json", def, errors);
    CHECK(ok && errors.empty(), "suv_glb_test: %s", errors.empty() ? "load failed" : errors[0].c_str());
    if (!ok) return;
    const VehicleConfig& c = def.config;
    BodyModel body;
    CHECK(loadBodyModel(c.bodyModel, body, true), "body.glb");
    const VehicleVisual v = buildVehicleVisual(c);
    CHECK(v.parts.size() == body.parts.size(), "%zu parts drawn, %zu in body.glb", v.parts.size(), body.parts.size());
    CHECK(!v.find("chassis") && v.find("hood") && v.find("door_front_left"), "parts come from body.glb");
    std::size_t next = 0;
    for (const VehiclePart& p : v.parts) {
        const BodyPart* bp = body.find(p.name);
        CHECK(bp && bp->parent == p.parent && bp->deformable == p.deformable && bp->detachable == p.detachable,
              "part '%s' flags / parent differ from body.glb", p.name.c_str());
        CHECK(p.firstVertex == next && p.vertexCount > 0, "part '%s' range", p.name.c_str());
        next = p.firstVertex + p.vertexCount;
    }
    CHECK(next == v.mesh.vertices.size(), "part ranges cover %zu of %zu vertices", next, v.mesh.vertices.size());
    CHECK(v.spares.size() == body.spares.size() && v.spares.size() == 1, "%zu spare mounts", v.spares.size());
    std::set<int> ids;
    // Same olive as the procedural SUV (Cars.png car #1).
    const glm::vec3 paint = style::oliveExpedition().body.color;
    const auto& own = c.bodyPaint.at("paint_primary");
    CHECK(glm::length(style::srgb(own[0], own[1], own[2]) - paint) < 1e-4f, "paint_primary is not the SUV olive");
    int paintVerts = 0, wrongPaint = 0;
    for (const LitVertex& lv : v.mesh.vertices) {
        const int id = int(lv.material.x + 0.5f);
        ids.insert(id);
        if (id == int(MatId::Paint)) {
            ++paintVerts;
            // paint_primary takes vehicle.json body.paint; secondary / accent keep the body.glb colours.
            if (glm::length(lv.color - paint) > 1e-4f) ++wrongPaint;
        }
    }
    for (MatId m : {MatId::Paint, MatId::Glass, MatId::Headlight, MatId::BrakeLight, MatId::ReverseLight, MatId::Amber,
                    MatId::Cladding})
        CHECK(ids.count(int(m)), "no vertex with material id %d", int(m));
    CHECK(paintVerts > 0 && wrongPaint < paintVerts, "paint: %d vertices, %d not primary", paintVerts, wrongPaint);
    int primary = 0;
    for (const BodyMaterial& m : body.materials) primary += m.role == BodyRole::PaintPrimary;
    CHECK(primary > 0, "body.glb has no paint_primary material");
    CHECK(v.headlightLens.x > 0.3f && v.headlightLens.z > 1.8f, "headlight lens (%.2f, %.2f, %.2f)", v.headlightLens.x,
          v.headlightLens.y, v.headlightLens.z);
    float worst = 1e9f;
    for (const VehicleConfig& k : bodyCheckConfigs(def)) {
        const VehicleVisual kv = buildVehicleVisual(k);
        const auto sweeps = wheelSweeps(k);
        for (const LitVertex& lv : kv.mesh.vertices)
            worst = std::min(worst, wheelSweepClearance(sweeps, lv.position.x, lv.position.y, lv.position.z));
    }
    CHECK(worst >= kBodyClearance, "body.glb vertex %.1f mm from a wheel sweep", 1000.0f * worst);
    // Same procedural wheels as the SUV it is based on.
    const AxleConfig& ax = c.axles[1];
    const glm::vec2 size = wheelMeshSize(buildAxleWheelMesh(v, ax));
    CHECK(std::abs(size.x - ax.wheelRadius) < 0.003f && std::abs(size.y - ax.wheelWidth) < 1e-4f, "wheel mesh size");
    VehicleConfig repaint = c;
    repaint.bodyPaint["paint_primary"] = {0.2f, 0.3f, 0.4f};
    int repainted = 0;
    for (const LitVertex& lv : buildVehicleVisual(repaint).mesh.vertices)
        repainted += int(lv.material.x + 0.5f) == int(MatId::Paint) && glm::length(lv.color - style::srgb(0.2f, 0.3f, 0.4f)) < 1e-4f;
    CHECK(repainted > 0, "body.paint override not applied");
    VehicleConfig procedural = c;
    procedural.bodyModel.clear();
    procedural.bodyPaint.clear();
    CHECK(!sameWheelFit(c, repaint) && !sameWheelFit(c, procedural), "a body or paint change must rebuild the visual");
    VehicleConfig missing = c;
    missing.bodyModel += ".missing";
    const VehicleVisual fallback = buildVehicleVisual(missing);
    CHECK(fallback.find("chassis") && fallback.find("winch"), "missing body.glb: procedural body expected");
    std::printf("  suv_glb_test: %zu parts, %zu triangles, closest wheel sweep %.1f mm over %zu configurations\n",
                v.parts.size(), v.mesh.vertices.size() / 3, 1000.0f * worst, bodyCheckConfigs(def).size());
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"model_parts", modelParts},
        {"model_wheel_size", modelWheelSize},
        {"model_wheel_clearance", modelWheelClearance},
        {"model_body_glb", modelBodyGlb},
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
