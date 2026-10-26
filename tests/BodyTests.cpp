// Blender-authored vehicle bodies (body.glb, README "Vehicle data format"), headless: the converter round trip on the
// committed test source, validation (convex collision pieces, wheel sweep, names, roles, bounds), the vehicle.json
// reference and the Jolt collision built from the COL_* pieces. Usage: trailforge_body_tests <test_name>

#include "BodySource.h"
#include "sim/Simulation.h"
#include "vehicle/VehicleBody.h"
#include "vehicle/VehicleDefinition.h"
#include "vehicle/WheelSweep.h"

#include <Jolt/Physics/Body/BodyLockInterface.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
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

const std::string kSuv = TRAILFORGE_SOURCE_DIR "/assets/vehicles/trailforge_suv/vehicle.json";
const std::string kTestDir = TRAILFORGE_SOURCE_DIR "/tests/data/vehicles/suv_glb_test";

VehicleDefinition load(const std::string& path) {
    VehicleDefinition d;
    std::vector<std::string> errors;
    const bool ok = loadVehicleDefinition(path, d, errors);
    for (const std::string& e : errors) std::printf("    %s\n", e.c_str());
    CHECK(ok, "cannot load %s", path.c_str());
    return d;
}

bool contains(const std::vector<std::string>& list, const std::string& text) {
    return std::any_of(list.begin(), list.end(), [&](const std::string& s) { return s.find(text) != std::string::npos; });
}

std::size_t triangles(const BodyModel& m, int lod) {
    std::size_t n = 0;
    for (const BodyPart& p : m.parts)
        if (std::size_t(lod) < p.lods.size()) n += p.lods[std::size_t(lod)].triangles();
    return n;
}

// Triangle corners (position, normal, material) in primitive order: the written file stores one vertex array per
// primitive, so the vertex arrays themselves may differ.
std::vector<float> corners(const BodyMesh& m) {
    std::vector<float> out;
    for (const BodyMesh::Primitive& p : m.primitives)
        for (std::uint32_t k = p.firstIndex; k < p.firstIndex + p.indexCount; ++k) {
            const std::uint32_t v = m.indices[k];
            out.insert(out.end(), {m.positions[3 * v], m.positions[3 * v + 1], m.positions[3 * v + 2], m.normals[3 * v],
                                   m.normals[3 * v + 1], m.normals[3 * v + 2], float(p.material)});
        }
    return out;
}

// Axis-aligned box as a body mesh (outward winding) and as a collision piece.
BodyMesh boxMesh(const std::array<float, 3>& lo, const std::array<float, 3>& hi, int material = 0) {
    BodyMesh m;
    const int faces[6][4] = {{0, 2, 6, 4}, {1, 5, 7, 3}, {0, 4, 5, 1}, {2, 3, 7, 6}, {0, 1, 3, 2}, {4, 6, 7, 5}};
    const float normals[6][3] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
    for (int f = 0; f < 6; ++f) {
        const std::uint32_t base = std::uint32_t(m.vertexCount());
        for (int k = 0; k < 4; ++k) {
            const int c = faces[f][k];
            m.positions.insert(m.positions.end(), {c & 4 ? hi[0] : lo[0], c & 2 ? hi[1] : lo[1], c & 1 ? hi[2] : lo[2]});
            m.normals.insert(m.normals.end(), {normals[f][0], normals[f][1], normals[f][2]});
        }
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    m.primitives.push_back({0, std::uint32_t(m.indices.size()), material});
    return m;
}

BodyHull boxHull(const std::string& name, const std::array<float, 3>& lo, const std::array<float, 3>& hi) {
    const BodyMesh m = boxMesh(lo, hi);
    BodyHull h{name, {}, m.indices};
    for (std::size_t i = 0; i < m.vertexCount(); ++i)
        h.points.push_back({m.positions[3 * i], m.positions[3 * i + 1], m.positions[3 * i + 2]});
    return h;
}

// A box above the SUV wheels with a matching collision box: valid against the SUV configurations.
BodyModel boxBody() {
    BodyModel m;
    BodyMaterial paint;
    paint.name = "paint_primary";
    paint.role = BodyRole::PaintPrimary;
    m.materials.push_back(paint);
    BodyPart part;
    part.name = "body_shell";
    part.lods.push_back(boxMesh({-0.9f, 0.45f, -2.0f}, {0.9f, 1.2f, 2.0f}));
    m.parts.push_back(part);
    m.hulls.push_back(boxHull("COL_body", {-0.9f, 0.45f, -2.0f}, {0.9f, 1.2f, 2.0f}));
    m.computeBounds();
    return m;
}

std::vector<VehicleConfig> suvConfigs() {
    VehicleDefinition def = load(kSuv);
    return bodyCheckConfigs(def);
}

std::string readText(const std::string& path) {
    std::ifstream f(path);
    std::stringstream s;
    s << f.rdbuf();
    return s.str();
}

// ---------------------------------------------------------------------------

// Hull convexity: a box is convex, a dent shows as its depth, a flat piece is degenerate.
void bodyHullConvexity() {
    BodyHull box = boxHull("COL_box", {-1, 0, -1}, {1, 1, 1});
    CHECK(hullConvexityError(box) < 1e-4f, "box: %.5f", hullConvexityError(box));
    BodyHull dented = box;
    dented.points.push_back({0.0f, 0.8f, 0.0f}); // 0.2 m below the top face
    CHECK(std::abs(hullConvexityError(dented) - 0.2f) < 0.005f, "dent: %.4f", hullConvexityError(dented));
    BodyHull subdivided = box;
    subdivided.points.push_back({0.3f, 1.0f, -0.2f}); // on the top face: still convex
    CHECK(hullConvexityError(subdivided) < 1e-4f, "point on a face: %.5f", hullConvexityError(subdivided));
    const BodyHull flat = boxHull("COL_flat", {-1, 0, -1}, {1, 0.005f, 1});
    CHECK(hullConvexityError(flat) > 1e8f, "flat piece not rejected");
}

// The committed Blender export (body_src.glb, written by tools/cars/blender/export_body.py) converts without errors
// and gives the committed body.glb; the converted file round-trips through write / load.
void bodyConvertSource() {
    BodyModel src;
    std::vector<std::string> errors, warnings;
    CHECK(readBodySource(kTestDir + "/body_src.glb", src, errors, warnings), "read body_src.glb");
    for (const std::string& e : errors) std::printf("    error: %s\n", e.c_str());
    buildBodyLods(src);
    VehicleDefinition def = load(kTestDir + "/vehicle.json");
    const BodyValidation v = validateBodyModel(src, bodyCheckConfigs(def));
    for (const std::string& e : v.errors) std::printf("    error: %s\n", e.c_str());
    for (const std::string& w : v.warnings) std::printf("    warning: %s\n", w.c_str());
    CHECK(errors.empty() && v.errors.empty(), "%zu source errors, %zu validation errors", errors.size(), v.errors.size());
    CHECK(v.minClearance >= kBodyClearance && v.hullClearance >= kBodyClearance, "clearance %.1f / %.1f mm",
          1000.0f * v.minClearance, 1000.0f * v.hullClearance);

    BodyModel shipped;
    std::string err;
    CHECK(loadBodyModel(kTestDir + "/body.glb", shipped, true, &err), "body.glb: %s", err.c_str());
    CHECK(shipped.parts.size() == src.parts.size() && shipped.hulls.size() == src.hulls.size() &&
              shipped.materials.size() == src.materials.size() && shipped.spares.size() == src.spares.size(),
          "body.glb differs from body_src.glb: %zu/%zu parts, %zu/%zu pieces", shipped.parts.size(), src.parts.size(),
          shipped.hulls.size(), src.hulls.size());
    // LOD0 is the authored mesh; the simplified LODs may differ by a few triangles between compilers.
    CHECK(triangles(shipped, 0) == triangles(src, 0), "LOD0: %zu triangles in body.glb, %zu converted",
          triangles(shipped, 0), triangles(src, 0));
    for (int lod = 1; lod < kBodyLodCount; ++lod)
        CHECK(std::abs(double(triangles(shipped, lod)) - double(triangles(src, lod))) <= 0.05 * double(triangles(src, lod)),
              "LOD%d: %zu triangles in body.glb, %zu converted", lod, triangles(shipped, lod), triangles(src, lod));

    const std::string out = (std::filesystem::temp_directory_path() / "trailforge_body_roundtrip.glb").string();
    CHECK(writeBodyModel(out, src, &err), "write: %s", err.c_str());
    BodyModel back;
    CHECK(loadBodyModel(out, back, true, &err), "reload: %s", err.c_str());
    CHECK(back.parts.size() == src.parts.size(), "parts after round trip");
    for (std::size_t i = 0; i < std::min(back.parts.size(), src.parts.size()); ++i) {
        const BodyPart &a = src.parts[i], &b = back.parts[i];
        CHECK(a.name == b.name && a.parent == b.parent && a.deformable == b.deformable && a.detachable == b.detachable &&
                  a.collides == b.collides && b.lods.size() == std::size_t(kBodyLodCount),
              "part '%s' changed in the round trip", a.name.c_str());
        for (std::size_t l = 0; l < std::min(a.lods.size(), b.lods.size()); ++l)
            CHECK(corners(a.lods[l]) == corners(b.lods[l]),
                  "part '%s' LOD%zu mesh changed", a.name.c_str(), l);
    }
    for (std::size_t i = 0; i < std::min(back.materials.size(), src.materials.size()); ++i) {
        const BodyMaterial &a = src.materials[i], &b = back.materials[i];
        CHECK(a.name == b.name && a.role == b.role && a.color == b.color && a.emissive == b.emissive &&
                  a.clearcoat == b.clearcoat && a.transmission == b.transmission && a.mud == b.mud,
              "material '%s' changed in the round trip", a.name.c_str());
    }
    for (std::size_t i = 0; i < std::min(back.hulls.size(), src.hulls.size()); ++i)
        CHECK(back.hulls[i].points == src.hulls[i].points, "piece '%s' changed", src.hulls[i].name.c_str());
    std::filesystem::remove(out);
    std::printf("  body_src.glb: %zu parts, %zu materials, %zu pieces, LOD0 %zu / LOD1 %zu / LOD2 %zu triangles, "
                "clearance %.1f mm, bounds deviation %.1f mm, coverage %.1f%%\n",
                src.parts.size(), src.materials.size(), src.hulls.size(), triangles(src, 0), triangles(src, 1),
                triangles(src, 2), 1000.0f * v.minClearance, 1000.0f * v.boundsDeviation, 100.0f * v.hullCoverage);
}

// Each rule rejects the body that breaks it.
void bodyValidationRules() {
    const std::vector<VehicleConfig> configs = suvConfigs();
    CHECK(configs.size() == 3, "SUV: %zu configurations (base at both ride height limits expected)", configs.size());
    {
        const BodyValidation v = validateBodyModel(boxBody(), configs);
        for (const std::string& e : v.errors) std::printf("    error: %s\n", e.c_str());
        CHECK(v.errors.empty() && v.boundsDeviation < 1e-4f && v.hullCoverage > 0.99f, "box body: %zu errors",
              v.errors.size());
    }
    {
        BodyModel m = boxBody();
        m.hulls[0].points.push_back({0.0f, 1.0f, 0.0f}); // dent 0.2 m under the roof
        CHECK(contains(validateBodyModel(m, configs).errors, "COL_body is not convex"), "concave piece accepted");
    }
    {
        BodyModel m = boxBody();
        BodyPart fender;
        fender.name = "fender_front_left";
        fender.lods.push_back(boxMesh({0.80f, 0.0f, 1.20f}, {0.95f, 0.10f, 1.36f})); // inside the FL tire at full bump
        m.parts.push_back(fender);
        m.computeBounds();
        CHECK(contains(validateBodyModel(m, configs).errors, "fender_front_left"), "body inside the wheel sweep accepted");
    }
    {
        BodyModel m = boxBody();
        m.hulls.push_back(boxHull("COL_low", {0.80f, 0.0f, 1.20f}, {0.95f, 0.10f, 1.36f}));
        CHECK(contains(validateBodyModel(m, configs).errors, "COL_low"), "collision inside the wheel sweep accepted");
    }
    {
        BodyModel m = boxBody();
        m.parts[0].name = "wheel_front_left";
        CHECK(contains(validateBodyModel(m, configs).errors, "drawn by the game"), "reserved part name accepted");
        CHECK(bodyNameIsReserved("spare_tire") && bodyNameIsReserved("shock_rear") && !bodyNameIsReserved("hood") &&
                  !bodyNameIsReserved("spare_carrier"),
              "reserved names");
    }
    {
        BodyModel m = boxBody();
        m.parts[0].parent = "chassis";
        CHECK(contains(validateBodyModel(m, configs).errors, "unknown parent"), "unknown parent accepted");
    }
    {
        BodyModel m = boxBody();
        m.hulls[0] = boxHull("COL_body", {-0.6f, 0.45f, -1.6f}, {0.6f, 1.2f, 1.6f}); // 0.3 / 0.4 m short of the body
        const BodyValidation v = validateBodyModel(m, configs);
        CHECK(contains(v.errors, "bounds differ") && v.boundsDeviation > 0.39f, "collision smaller than the body accepted");
    }
    {
        BodyModel m = boxBody();
        m.hulls.clear();
        CHECK(contains(validateBodyModel(m, configs).errors, "no collision pieces"), "body without collision accepted");
    }
    {
        BodyModel m = boxBody();
        BodyMaterial lamp;
        lamp.name = "light_head";
        lamp.role = BodyRole::LightHead;
        lamp.color = {0, 0, 0};
        m.materials.push_back(lamp);
        CHECK(contains(validateBodyModel(m, configs).warnings, "is black"), "black lamp not reported");
    }
    {
        // A source material without a role: written with a name that is no role, then read as a Blender export.
        BodyModel m = boxBody();
        m.materials[0].name = "Material.001";
        const std::string path = (std::filesystem::temp_directory_path() / "trailforge_body_norole.glb").string();
        std::string err;
        CHECK(writeBodyModel(path, m, &err), "write: %s", err.c_str());
        BodyModel src;
        std::vector<std::string> errors, warnings;
        readBodySource(path, src, errors, warnings);
        CHECK(contains(errors, "no role"), "material without a role accepted");
        std::filesystem::remove(path);
    }
    std::string role;
    for (int r = 0; r < int(BodyRole::Count); ++r) {
        BodyRole back;
        CHECK(bodyRoleFromName(bodyRoleName(BodyRole(r)), back) && back == BodyRole(r), "role %d name", r);
    }
}

// vehicle.json body.model / body.paint: resolved next to the definition, collision pieces loaded; bad references
// rejected.
void bodyDefinition() {
    const VehicleDefinition def = load(kTestDir + "/vehicle.json");
    const VehicleConfig& c = def.config;
    std::error_code ec;
    CHECK(std::filesystem::equivalent(c.bodyModel, kTestDir + "/body.glb", ec), "body.model resolved to '%s'",
          c.bodyModel.c_str());
    CHECK(c.bodyHulls.size() == 7, "%zu collision pieces", c.bodyHulls.size());
    CHECK(c.bodyPaint.count("paint_primary") && c.bodyPaint.at("paint_primary")[0] == 0.45f, "body.paint");
    const VehicleDefinition suv = load(kSuv);
    CHECK(suv.config.bodyModel.empty() && suv.config.bodyHulls.empty() && suv.config.bodyPaint.empty(),
          "the shipped SUV keeps its procedural body");

    const std::string text = readText(kTestDir + "/vehicle.json");
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "trailforge_body_def";
    std::filesystem::create_directories(dir / "suv_glb_test");
    std::filesystem::copy_file(kTestDir + "/body.glb", dir / "suv_glb_test" / "body.glb",
                               std::filesystem::copy_options::overwrite_existing);
    auto variant = [&](const std::string& from, const std::string& to, const char* expect) {
        std::string t = text;
        const std::size_t at = t.find(from);
        CHECK(at != std::string::npos, "'%s' not in the test definition", from.c_str());
        if (at == std::string::npos) return;
        t.replace(at, from.size(), to);
        const std::string path = (dir / "suv_glb_test" / "vehicle.json").string();
        std::ofstream(path) << t;
        VehicleDefinition d;
        std::vector<std::string> errors;
        const bool ok = loadVehicleDefinition(path, d, errors);
        if (!expect) {
            CHECK(ok && errors.empty(), "%s: %s", to.c_str(), errors.empty() ? "load failed" : errors[0].c_str());
            return;
        }
        CHECK(!ok && contains(errors, expect), "'%s' accepted (expected an error with '%s')", to.c_str(), expect);
    };
    variant("\"model\": \"body.glb\"", "\"model\": \"body.glb\"", nullptr);
    variant("\"model\": \"body.glb\"", "\"model\": \"../trailforge_suv/body.glb\"", "body.model");
    variant("\"model\": \"body.glb\"", "\"model\": \"body.obj\"", "body.model");
    variant("\"model\": \"body.glb\"", "\"model\": \"missing.glb\"", "missing.glb");
    variant("\"model\": \"body.glb\",", "", "body.paint");
    variant("\"paint_primary\"", "\"paint_tertiary\"", "paint_tertiary");
    variant("[0.45, 0.50, 0.32]", "[1.45, 0.50, 0.32]", "body.paint");
    std::filesystem::remove_all(dir);
}

// Jolt collision from the COL_* pieces: the body shape spans the pieces (not the procedural body's boxes), the vehicle
// settles at the same ride height as the same definition with the procedural body (the body never touches flat
// ground). The twister run is in
// vehicle_catalog_sweep (tests/data/vehicles is part of the catalog there).
void bodyPhysics() {
    const VehicleDefinition glb = load(kTestDir + "/vehicle.json");
    VehicleConfig procedural = glb.config;
    procedural.bodyModel.clear();
    procedural.bodyHulls.clear();
    procedural.bodyPaint.clear();
    BodyModel body;
    CHECK(loadBodyModel(glb.config.bodyModel, body, false), "body.glb");
    float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    for (const BodyHull& h : body.hulls)
        for (const auto& p : h.points)
            for (int k = 0; k < 3; ++k) lo[k] = std::min(lo[k], p[std::size_t(k)]), hi[k] = std::max(hi[k], p[std::size_t(k)]);
    float ride[2] = {0, 0};
    for (int i = 0; i < 2; ++i) {
        SimulationSettings s;
        s.layout = TrackLayout::Flat;
        s.vehicle = i == 0 ? glb.config : procedural;
        Simulation sim(s);
        for (int t = 0; t < 480; ++t) sim.tick();
        ride[i] = sim.currentSnapshot().body.position.GetY();
        CHECK(!sim.hadPhysicsError(), "Jolt update error");
        if (i == 0) {
            JPH::BodyLockRead lock(sim.world().system().GetBodyLockInterface(), sim.vehicle().bodyId());
            CHECK(lock.Succeeded(), "body lock");
            if (lock.Succeeded()) {
                const JPH::Shape* shape = lock.GetBody().GetShape();
                // Local bounds of the shape relative to the body origin (the COM offset is undone by the decorator).
                const JPH::AABox b = shape->GetLocalBounds();
                const JPH::Vec3 com = shape->GetCenterOfMass();
                const JPH::Vec3 mn = b.mMin + com, mx = b.mMax + com;
                std::printf("  collision bounds (%.3f, %.3f, %.3f) .. (%.3f, %.3f, %.3f), pieces (%.3f, %.3f, %.3f) .. "
                            "(%.3f, %.3f, %.3f)\n", mn.GetX(), mn.GetY(), mn.GetZ(), mx.GetX(), mx.GetY(), mx.GetZ(),
                            lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]);
                for (int k = 0; k < 3; ++k)
                    CHECK(std::abs(mn[k] - lo[k]) < 0.01f && std::abs(mx[k] - hi[k]) < 0.01f,
                          "axis %d: shape %.3f..%.3f vs pieces %.3f..%.3f", k, mn[k], mx[k], lo[k], hi[k]);
            }
        }
    }
    std::printf("  ride height: body.glb %.4f m, procedural %.4f m\n", ride[0], ride[1]);
    CHECK(std::abs(ride[0] - ride[1]) < 0.002f, "ride height differs: %.4f vs %.4f", ride[0], ride[1]);
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"body_hull_convexity", bodyHullConvexity},
        {"body_convert_source", bodyConvertSource},
        {"body_validation_rules", bodyValidationRules},
        {"body_definition", bodyDefinition},
        {"body_physics", bodyPhysics},
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
