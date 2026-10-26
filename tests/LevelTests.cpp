// Headless tests of the open-world level: the bake (height and material sampling, roads, rivers, water), the
// physics tiles (heightfield == baked grid, streaming bookkeeping), both vehicles on Exploration Peaks and the
// terrain texture files (KTX2 container, shipped BC7 maps).

#include "core/Ktx2.h"
#include "core/TerrainTextureSet.h"
#include "physics/PhysicsWorld.h"
#include "sim/Simulation.h"
#include "track/Level.h"
#include "track/TerrainTiles.h"
#include "vehicle/VehicleDefinition.h"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Body/BodyFilter.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
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

const std::string kLevel = TRAILFORGE_SOURCE_DIR "/assets/levels/exploration_peaks/level.json";

const std::shared_ptr<const Level>& level() {
    static const std::shared_ptr<const Level> l = [] {
        std::string err;
        auto lv = Level::load(kLevel, &err);
        if (!lv) {
            std::printf("  cannot load the level: %s\n", err.c_str());
            std::exit(1);
        }
        std::printf("  bake %.0f ms: %d^2 samples, %.1f..%.1f m, %zu trees, %zu rocks, %zu prop triangles\n",
                    lv->bakeStats().totalMs, lv->samples(), lv->bakeStats().minHeight, lv->bakeStats().maxHeight,
                    lv->bakeStats().trees, lv->bakeStats().rocks, lv->bakeStats().propTriangles);
        return lv;
    }();
    return l;
}

struct Lcg {
    std::uint32_t s;
    float next() {
        s = s * 1664525u + 1013904223u;
        return float(s >> 8) / float(1u << 24);
    }
    float range(float a, float b) { return a + (b - a) * next(); }
};

// --------------------------------------------------------------------------------------------------- bake + tiles
void levelHeightfieldSampling() {
    const Level& L = *level();
    CHECK(L.samples() == 2561, "samples %d", L.samples());
    CHECK(L.bakeStats().maxHeight > 270.0f && L.bakeStats().maxHeight < 320.0f, "max height %.1f",
          L.bakeStats().maxHeight);
    // Grid samples are exact; the triangle interpolation is continuous across the cell diagonal.
    int bad = 0;
    for (int k = 0; k < 2000; ++k) {
        const int i = 300 + (k * 37) % 1900, j = 300 + (k * 53) % 1900;
        const float x = L.origin() + float(i), z = L.origin() + float(j);
        bad += std::abs(L.heightAt(x, z) - L.heightSample(i, j)) > 1e-4f;
        const float a = L.heightAt(x + 0.5f, z + 0.4999f), b = L.heightAt(x + 0.5f, z + 0.5001f);
        bad += std::abs(a - b) > 0.01f;
    }
    CHECK(bad == 0, "%d samples differ from the grid", bad);

    // Physics tiles: rays straight down hit the heightfield at heightAt() and report the cell's surface.
    JoltRuntime runtime;
    PhysicsWorld world;
    SurfaceTable surfaces;
    TerrainTiles::Settings ts;
    ts.async = false;
    TerrainTiles tiles(world, surfaces, level(), ts);
    tiles.ensureLoaded(0.0f, 0.0f, 3000.0f);
    CHECK(tiles.tilesPerSide() == 8, "tiles per side %d", tiles.tilesPerSide());
    CHECK(tiles.stats().builtSync == 64, "builtSync %d", tiles.stats().builtSync);
    world.optimizeBroadPhase();
    Lcg rng{17u};
    int rays = 0, heightBad = 0, surfaceBad = 0, seamBad = 0;
    float maxErr = 0.0f;
    auto cast = [&](float x, float z, float& hitY, SurfaceId& s) {
        const JPH::RRayCast ray(JPH::RVec3(x, 600.0f, z), JPH::Vec3(0.0f, -800.0f, 0.0f));
        JPH::RayCastResult hit;
        if (!world.system().GetNarrowPhaseQuery().CastRay(ray, hit)) return false;
        hitY = 600.0f - 800.0f * hit.mFraction;
        const int tx = tiles.tileIndexOf(x), tz = tiles.tileIndexOf(z);
        if (tx < 0 || tz < 0 || hit.mBodyID != tiles.terrainBody(tx, tz)) return false; // trunk or boulder on top
        JPH::BodyLockRead lock(world.system().GetBodyLockInterface(), hit.mBodyID);
        s = surfaces.lookup(lock.GetBody().GetShape()->GetMaterial(hit.mSubShapeID2));
        return true;
    };
    for (int k = 0; k < 4000; ++k) {
        const float x = rng.range(-990.0f, 990.0f), z = rng.range(-990.0f, 990.0f);
        float y;
        SurfaceId s;
        if (!cast(x, z, y, s)) continue;
        ++rays;
        const float err = std::abs(y - L.heightAt(x, z));
        maxErr = std::max(maxErr, err);
        heightBad += err > 0.005f;
        surfaceBad += s != L.surfaceAt(x, z);
    }
    // Tile seams: both sides of every interior tile edge.
    for (int t = 1; t < tiles.tilesPerSide(); ++t) {
        const float edge = tiles.tileOrigin() + float(t) * tiles.tileSize();
        for (int k = 0; k < 40; ++k) {
            const float along = rng.range(-990.0f, 990.0f);
            for (float o : {-0.02f, 0.02f})
                for (int axis = 0; axis < 2; ++axis) {
                    const float x = axis ? along : edge + o, z = axis ? edge + o : along;
                    float y;
                    SurfaceId s;
                    if (cast(x, z, y, s)) seamBad += std::abs(y - L.heightAt(x, z)) > 0.005f;
                }
        }
    }
    std::printf("  %d terrain rays: max |hit - heightAt| %.4f m, %d height and %d surface mismatches, %d seam\n", rays,
                maxErr, heightBad, surfaceBad, seamBad);
    CHECK(rays > 3500, "rays %d", rays);
    CHECK(heightBad == 0, "%d", heightBad);
    CHECK(surfaceBad == 0, "%d", surfaceBad);
    CHECK(seamBad == 0, "%d", seamBad);

    // Deterministic bake: a second bake with a different thread count is identical.
    std::string err;
    auto again = Level::load(kLevel, &err, 3);
    CHECK(again && again->heights() == L.heights(), "heights differ");
    CHECK(again && again->cellSurfaces() == L.cellSurfaces(), "surfaces differ");
    CHECK(again && again->trees().size() == L.trees().size() && again->rocks().size() == L.rocks().size(), "objects");
}

void levelTileStreaming() {
    JoltRuntime runtime;
    PhysicsWorld world;
    SurfaceTable surfaces;
    TerrainTiles::Settings ts;
    TerrainTiles tiles(world, surfaces, level(), ts);
    const int side = tiles.tilesPerSide();
    // Drive the focus diagonally across the map at ~30 m/s, updating at 120 Hz.
    int violations = 0, farLoaded = 0, maxLoaded = 0;
    const float speed = 30.0f, dt = 1.0f / 120.0f;
    float x = -950.0f, z = -950.0f;
    for (int step = 0; x < 950.0f; ++step) {
        x += speed * dt * 0.7071f;
        z += speed * dt * 0.7071f;
        tiles.update(x, z);
        for (int tz = 0; tz < side; ++tz)
            for (int tx = 0; tx < side; ++tx) {
                const float d = tiles.distanceToTile(tx, tz, x, z);
                if (d <= ts.requiredRadius && !tiles.isLoaded(tx, tz)) ++violations;
                if (d > ts.keepRadius + 1.0f && tiles.isLoaded(tx, tz)) ++farLoaded;
            }
        maxLoaded = std::max(maxLoaded, tiles.stats().loaded);
        if (step % 4 == 0) std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    // Let the worker finish, then everything within the load radius is present.
    for (int k = 0; k < 400 && tiles.stats().queued > 0; ++k) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        tiles.update(x, z);
    }
    int missing = 0;
    for (int tz = 0; tz < side; ++tz)
        for (int tx = 0; tx < side; ++tx)
            if (tiles.distanceToTile(tx, tz, x, z) <= ts.loadRadius && !tiles.isLoaded(tx, tz)) ++missing;
    const TerrainStreamingStats& st = tiles.stats();
    std::printf("  streaming: max %d tiles loaded, %d async + %d sync builds, %d unloads, build max %.1f ms, "
                "update max %.2f ms, sync stall max %.1f ms\n",
                maxLoaded, st.builtAsync, st.builtSync, st.unloaded, st.maxBuildMs, st.maxUpdateMs, st.maxSyncStallMs);
    CHECK(violations == 0, "%d required tiles missing after update()", violations);
    CHECK(farLoaded == 0, "%d tiles beyond keepRadius stayed loaded", farLoaded);
    CHECK(missing == 0, "%d tiles within loadRadius missing after the worker drained", missing);
    CHECK(st.unloaded > 0 && maxLoaded < side * side, "unloads %d, max loaded %d", st.unloaded, maxLoaded);
    CHECK(st.builtAsync > st.builtSync, "async %d sync %d", st.builtAsync, st.builtSync);

    // Destruction with work queued must not hang or leak bodies.
    {
        TerrainTiles busy(world, surfaces, level(), ts);
        busy.update(0.0f, 0.0f);
        busy.update(900.0f, 900.0f);
    }
    CHECK(true, "");
}

void levelRoadCarving() {
    // Grade limiter: random targets with pins, slope limit everywhere, pins exact.
    Lcg rng{5u};
    for (int trial = 0; trial < 20; ++trial) {
        const std::size_t n = 300;
        std::vector<float> target(n), pin(n, std::numeric_limits<float>::quiet_NaN());
        float y = 0.0f;
        for (std::size_t i = 0; i < n; ++i) {
            y += rng.range(-2.0f, 2.0f);
            target[i] = y;
        }
        pin[0] = target[0];
        pin[150] = target[0] + 10.0f;
        const std::vector<float> out = level_math::gradeLimitedProfile(target, pin, 2.0f, 0.12f);
        float worst = 0.0f;
        for (std::size_t i = 1; i < n; ++i) worst = std::max(worst, std::abs(out[i] - out[i - 1]) / 2.0f);
        CHECK(worst <= 0.12f + 1e-4f, "trial %d grade %.4f", trial, worst);
        CHECK(std::abs(out[0] - pin[0]) < 1e-4f && std::abs(out[150] - pin[150]) < 1e-4f, "pins moved");
    }

    const Level& L = *level();
    int fords = 0, bridges = 0;
    for (const LevelRoad& r : L.roads()) {
        float worstGrade = 0.0f, worstCenter = 0.0f, worstCross = 0.0f;
        int offRoad = 0, samples = 0;
        float wx = 0.0f, wz = 0.0f;
        const float hw = 0.5f * r.width;
        for (std::size_t i = 0; i < r.samples.size(); ++i) {
            const RoadSample& s = r.samples[i];
            if (i > 0) {
                const float ds = s.s - r.samples[i - 1].s;
                worstGrade = std::max(worstGrade, std::abs(s.y - r.samples[i - 1].y) / ds);
                if (s.ford && !r.samples[i - 1].ford) {
                    ++fords;
                    std::printf("  ford on %s at %.0f %.0f (s = %.0f m, heading %.0f deg)\n", r.name.c_str(), double(s.x),
                                double(s.z), double(s.s), double(std::atan2(s.dx, s.dz) * 57.2958f));
                }
            }
            if (s.bridge || std::abs(s.x) > L.playHalfExtent() || std::abs(s.z) > L.playHalfExtent()) continue;
            // Junctions blend two profiles; they are checked by the vehicle drives, not against one profile.
            bool junction = false;
            for (const LevelRoad& o : L.roads())
                if (&o != &r)
                    for (const RoadSample& q : o.samples)
                        junction = junction || std::hypot(q.x - s.x, q.z - s.z) < 0.5f * (o.width + r.width) + 8.0f;
            if (junction) continue;
            // The deck ramps (3 m into the banks) are the running surface there, checked by level_bridges.
            bool ramp = false;
            for (const LevelBridge& b : L.bridges()) {
                const float len = b.length(), dx = (b.x1 - b.x0) / len, dz = (b.z1 - b.z0) / len;
                const float along = (s.x - b.x0) * dx + (s.z - b.z0) * dz;
                const float lat = std::abs((s.x - b.x0) * dz - (s.z - b.z0) * dx);
                ramp = ramp || (along > -LevelBridge::kRamp - 2.0f && along < len + LevelBridge::kRamp + 2.0f && lat < b.halfWidth);
            }
            if (ramp) continue;
            ++samples;
            if (std::abs(L.heightAt(s.x, s.z) - s.y) > worstCenter) {
                wx = s.x;
                wz = s.z;
            }
            worstCenter = std::max(worstCenter, std::abs(L.heightAt(s.x, s.z) - s.y));
            // Across the road: the banked plane within 0.15 m.
            for (float o : {-1.0f, 1.0f}) {
                const float lat = o * (hw - 0.6f);
                const float px = s.x + s.dz * lat, pz = s.z - s.dx * lat;
                const float expect = s.y + lat * std::tan(s.bank);
                worstCross = std::max(worstCross, std::abs(L.heightAt(px, pz) - expect));
            }
            const SurfaceId sf = L.surfaceAt(s.x, s.z);
            offRoad += !(sf == SurfaceId::PackedDirt || sf == SurfaceId::Mud || sf == SurfaceId::Snow ||
                         (s.ford && sf == SurfaceId::Rock));
        }
        std::printf("  road %-17s %5.0f m: grade %.1f %% (limit %.1f %%), center %.3f m (at %.0f %.0f), across %.3f m, "
                    "%d/%d not dirt\n",
                    r.name.c_str(), r.samples.back().s, 100.0f * worstGrade, 100.0f * r.maxGrade, worstCenter, wx, wz,
                    worstCross, offRoad, samples);
        CHECK(worstGrade <= r.maxGrade + 0.005f, "%s grade %.3f", r.name.c_str(), worstGrade);
        CHECK(worstCenter < 0.08f, "%s center %.3f", r.name.c_str(), worstCenter);
        CHECK(worstCross < 0.12f, "%s across %.3f", r.name.c_str(), worstCross);
        CHECK(offRoad <= samples / 50, "%s: %d samples off the road surface", r.name.c_str(), offRoad);
    }
    // Fords are shallow, and bridges carry the road: a ray from above lands on the deck.
    for (const LevelRoad& r : L.roads())
        for (const RoadSample& s : r.samples) {
            if (!s.ford) continue;
            const auto w = L.waterAt(s.x, s.z);
            const float depth = w ? *w - L.heightAt(s.x, s.z) : 0.0f;
            CHECK(depth < 0.45f, "%s ford depth %.2f at %.0f %.0f", r.name.c_str(), depth, s.x, s.z);
        }
    JoltRuntime runtime;
    PhysicsWorld world;
    SurfaceTable surfaces;
    TestTrack props(world, surfaces, L);
    world.optimizeBroadPhase();
    for (const LevelBridge& b : L.bridges()) {
        ++bridges;
        const float mx = 0.5f * (b.x0 + b.x1), mz = 0.5f * (b.z0 + b.z1), my = 0.5f * (b.y0 + b.y1);
        const JPH::RRayCast ray(JPH::RVec3(mx, my + 5.0f, mz), JPH::Vec3(0.0f, -10.0f, 0.0f));
        JPH::RayCastResult hit;
        const bool ok = world.system().GetNarrowPhaseQuery().CastRay(ray, hit);
        const float y = my + 5.0f - 10.0f * hit.mFraction;
        CHECK(ok && std::abs(y - my) < 0.05f, "bridge deck at %.0f %.0f: hit %d y %.2f expected %.2f", mx, mz, ok, y, my);
        CHECK(L.heightAt(mx, mz) < my - 2.0f, "bridge %.0f %.0f does not span a gap", mx, mz);
    }
    std::printf("  %d fords, %d bridges\n", fords, bridges);
    CHECK(fords >= 3, "fords %d", fords);
    CHECK(bridges == 2, "bridges %d", bridges);
}

void levelSurfacesAndWater() {
    const Level& L = *level();
    int counts[kSurfaceCount] = {};
    for (SurfaceId s : L.cellSurfaces()) ++counts[std::size_t(s)];
    for (SurfaceId s : {SurfaceId::PackedDirt, SurfaceId::Rock, SurfaceId::Mud, SurfaceId::Grass, SurfaceId::Sand, SurfaceId::Snow})
        CHECK(counts[std::size_t(s)] > 20000, "surface %d only %d cells", int(s), counts[std::size_t(s)]);
    // Snow only high up, sand only low.
    Lcg rng{9u};
    int snowLow = 0, sandHigh = 0;
    for (int k = 0; k < 20000; ++k) {
        const float x = rng.range(-1000, 1000), z = rng.range(-1000, 1000);
        const SurfaceId s = L.surfaceAt(x, z);
        const float h = L.heightAt(x, z);
        snowLow += s == SurfaceId::Snow && h < 150.0f;
        sandHigh += s == SurfaceId::Sand && h > 8.0f && !L.waterAt(x, z);
    }
    CHECK(snowLow == 0 && sandHigh == 0, "snow below 150 m: %d, dry sand above 8 m: %d", snowLow, sandHigh);
    // Lake: level 70, deep in the middle; sea at 0 in the south.
    const auto lake = L.waterAt(-115.0f, -135.0f);
    CHECK(lake && std::abs(*lake - 70.0f) < 1e-3f && *lake - L.heightAt(-115.0f, -135.0f) > 5.0f, "lake");
    CHECK(L.waterKindAt(-115.0f, -135.0f) == WaterKind::Lake, "lake kind");
    const auto sea = L.waterAt(400.0f, 900.0f);
    CHECK(sea && std::abs(*sea) < 1e-3f && L.waterKindAt(400.0f, 900.0f) == WaterKind::Sea, "sea");
    // Rivers never flow uphill; the water surface follows the samples.
    for (const LevelRiver& r : L.rivers()) {
        float rise = 0.0f;
        for (std::size_t i = 1; i < r.samples.size(); ++i) rise = std::max(rise, r.samples[i].y - r.samples[i - 1].y);
        CHECK(rise <= 1e-4f, "%s rises by %.3f m", r.name.c_str(), rise);
    }
    // Points of interest: inside the play area, teleport points on dry, drivable ground.
    CHECK(L.pois().size() == 13, "%zu POIs", L.pois().size());
    for (const TrackFeature& f : L.features()) {
        const float x = f.position.GetX(), z = f.position.GetZ();
        const float gx = L.heightAt(x + 1.5f, z) - L.heightAt(x - 1.5f, z), gz = L.heightAt(x, z + 1.5f) - L.heightAt(x, z - 1.5f);
        const float slope = std::atan(std::hypot(gx, gz) / 3.0f) * 57.2958f;
        CHECK(L.insidePlayArea(x, z, 20.0f), "%s outside", f.name.c_str());
        CHECK(!L.waterAt(x, z), "%s in water", f.name.c_str());
        CHECK(slope < 14.0f, "%s slope %.1f deg", f.name.c_str(), slope);
    }
}

// ------------------------------------------------------------------------------------------------ vehicles on it
VehicleConfig vehicle(const char* id) {
    VehicleDefinition d;
    std::vector<std::string> errors;
    const std::string path = std::string(TRAILFORGE_SOURCE_DIR "/assets/vehicles/") + id + "/vehicle.json";
    if (!loadVehicleDefinition(path, d, errors)) {
        std::printf("  cannot load %s\n", path.c_str());
        std::exit(1);
    }
    return d.config;
}

void runFor(Simulation& sim, double seconds) {
    const int n = int(seconds / sim.fixedStep());
    for (int i = 0; i < n; ++i) sim.tick();
}

int wheelsInContact(const Simulation& sim) {
    int n = 0;
    for (const auto& c : sim.vehicle().contacts()) n += c.contact;
    return n;
}

// The vehicle body shape touches static geometry (both triangle faces count; the wheels are casts, not part of it).
bool chassisTouches(Simulation& sim) {
    PhysicsWorld& w = sim.world();
    const JPH::BodyID id = sim.vehicle().bodyId();
    JPH::RefConst<JPH::Shape> shape;
    JPH::RMat44 xf;
    {
        JPH::BodyLockRead lock(w.system().GetBodyLockInterface(), id);
        if (!lock.Succeeded()) return false;
        shape = lock.GetBody().GetShape();
        xf = lock.GetBody().GetCenterOfMassTransform();
    }
    JPH::CollideShapeSettings cs;
    cs.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;
    JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector> hit;
    w.system().GetNarrowPhaseQuery().CollideShape(shape, JPH::Vec3::sReplicate(1.0f), xf, cs, xf.GetTranslation(), hit, {},
                                                  {}, JPH::IgnoreSingleBodyFilter(id));
    return hit.HadHit();
}

// A dry, gentle point near the Alpine Lake marker facing at least 32 m of lake deeper than 1.5 m.
std::optional<TrackFeature> lakeShore(const Level& L) {
    const LevelPoi* lake = nullptr;
    for (const LevelPoi& p : L.pois())
        if (p.name == "Alpine Lake") lake = &p;
    if (!lake) return std::nullopt;
    for (float r = 0.0f; r < 400.0f; r += 4.0f)
        for (int a = 0; a < 72; ++a) {
            const float ang = float(a) * 0.0872665f;
            const float x = lake->x + r * std::sin(ang), z = lake->z + r * std::cos(ang);
            if (L.waterAt(x, z)) continue;
            const float gx = L.heightAt(x + 1.5f, z) - L.heightAt(x - 1.5f, z), gz = L.heightAt(x, z + 1.5f) - L.heightAt(x, z - 1.5f);
            if (std::hypot(gx, gz) / 3.0f > 0.22f) continue;
            for (int d = 0; d < 36; ++d) {
                const float yaw = float(d) * 0.174533f, dx = std::sin(yaw), dz = std::cos(yaw);
                bool deep = true;
                // A gentle approach (no river bank or channel step on the way in).
                for (float k = 0.0f; k < 10.0f && deep; k += 1.0f)
                    deep = std::abs(L.heightAt(x + dx * (k + 1.0f), z + dz * (k + 1.0f)) - L.heightAt(x + dx * k, z + dz * k)) < 0.45f;
                for (float k = 10.0f; k <= 42.0f && deep; k += 2.0f) {
                    const std::optional<float> w = L.waterAt(x + dx * k, z + dz * k);
                    deep = w && L.waterKindAt(x + dx * k, z + dz * k) == WaterKind::Lake &&
                           *w - L.heightAt(x + dx * k, z + dz * k) > 1.5f;
                }
                if (!deep) continue;
                TrackFeature f;
                f.name = "lake shore";
                f.position = JPH::Vec3(x, L.heightAt(x, z), z);
                f.yaw = yaw;
                return f;
            }
        }
    return std::nullopt;
}

void levelVehicles() {
    for (const char* id : {"trailforge_suv", "trailforge_ev"}) {
        SimulationSettings s;
        s.level = level();
        s.vehicle = vehicle(id);
        s.terrain.async = false; // deterministic tile set for the test
        Simulation sim(s);
        DriverIntent hold;
        hold.brake = 1.0f;
        sim.setIntent(hold);
        runFor(sim, 2.0);
        const JPH::Vec3 p0 = sim.currentSnapshot().body.position;
        CHECK(wheelsInContact(sim) == 4, "%s: %d wheels on the ground at the spawn", id, wheelsInContact(sim));
        CHECK(sim.vehicle().telemetry().speed < 0.2f, "%s settles", id);
        // Up the valley road from the village (the spawn faces along it).
        DriverIntent in;
        in.throttle = 0.45f;
        sim.setIntent(in);
        runFor(sim, 6.0);
        const JPH::Vec3 p1 = sim.currentSnapshot().body.position;
        const float moved = (p1 - p0).Length();
        CHECK(moved > 25.0f && !sim.hadPhysicsError(), "%s moved %.1f m", id, moved);
        // Every point of interest: settles with the wheels down (brakes held), no recovery.
        sim.setIntent(hold);
        const auto features = sim.track().features();
        for (int f = 1; f < int(features.size()); ++f) {
            sim.requestTeleport(f);
            runFor(sim, 2.5);
            const float speed = sim.vehicle().telemetry().speed;
            CHECK(wheelsInContact(sim) >= 3 && speed < 0.5f, "%s at %s: %d wheels, %.2f m/s", id,
                  features[std::size_t(f)].name.c_str(), wheelsInContact(sim), speed);
        }
        CHECK(sim.recoveries() == 0, "%s: %d recoveries while parked", id, sim.recoveries());
        // Deep water: from the lake shore straight into the lake; the vehicle is put back on a road.
        const std::optional<TrackFeature> shore = lakeShore(*sim.level());
        CHECK(shore.has_value(), "no lake shore start found");
        if (!shore) return;
        sim.requestPlace(*shore);
        runFor(sim, 2.0);
        in.throttle = 0.6f;
        sim.setIntent(in);
        double t = 0.0;
        float maxSub = 0.0f;
        while (t < 40.0 && sim.recoveries() == 0) {
            sim.tick();
            t += sim.fixedStep();
            maxSub = std::max(maxSub, sim.vehicleSubmersion());
        }
        sim.setIntent({});
        CHECK(sim.recoveries() == 1 && std::strstr(sim.lastMessage(), "Deep water") != nullptr,
              "%s: recoveries %d after %.1f s, message '%s'", id, sim.recoveries(), t, sim.lastMessage());
        runFor(sim, 2.0);
        const JPH::Vec3 p = sim.currentSnapshot().body.position;
        CHECK(!sim.level()->waterAt(p.GetX(), p.GetZ()) && wheelsInContact(sim) >= 3, "%s: recovered onto dry ground", id);
        // R reset: nearest road, not the spawn.
        sim.requestReset();
        runFor(sim, 1.5);
        const JPH::Vec3 r = sim.currentSnapshot().body.position;
        const TrackFeature road = sim.level()->nearestRoadPoint(r.GetX(), r.GetZ());
        CHECK((r - road.position).Length() < 3.0f, "%s: reset %.1f m from the nearest road point", id,
              (r - road.position).Length());
        std::printf("  %s: drove %.0f m, 13 POIs parked, deep-water recovery after %.1f s (max submersion %.2f m)\n",
                    id, moved, t, maxSub);
    }
    // Level switch back to the proving ground and again to the level.
    SimulationSettings s;
    s.vehicle = vehicle("trailforge_suv");
    Simulation sim(s);
    runFor(sim, 0.5);
    sim.requestLevel(level());
    runFor(sim, 1.5);
    CHECK(sim.level() != nullptr && sim.terrain() != nullptr && wheelsInContact(sim) == 4, "switch to the level");
    sim.requestLevel(nullptr);
    runFor(sim, 1.5);
    CHECK(sim.level() == nullptr && sim.track().layout() == TrackLayout::ProvingGround && wheelsInContact(sim) == 4,
          "switch back");
}

// Triangles of one convex piece that face toward its vertex centroid. Jolt's wheel casts ignore back faces, so an
// inverted piece carries the chassis but not the wheels.
int inwardTriangles(const TrackGeometry& g, std::size_t first, std::size_t last) {
    JPH::Vec3 c = JPH::Vec3::sZero();
    for (std::size_t v = 3 * first; v < 3 * last; ++v) c += JPH::Vec3(g.positions[v]);
    c /= float(3 * (last - first));
    int bad = 0;
    for (std::size_t t = first; t < last; ++t) {
        const JPH::Vec3 a(g.positions[3 * t]), b(g.positions[3 * t + 1]), d(g.positions[3 * t + 2]);
        const JPH::Vec3 n = (b - a).Cross(d - a);
        if (n.Length() < 1e-6f) continue;
        bad += n.Normalized().Dot(a - c) < -1e-3f;
    }
    return bad;
}

void levelPropWinding() {
    const Level& L = *level();
    const LevelProps& props = L.props();
    int pieces = 0, inverted = 0, badTris = 0;
    for (std::size_t k = 0; k < props.pieceStarts.size(); ++k) {
        const std::size_t first = props.pieceStarts[k];
        const std::size_t last = k + 1 < props.pieceStarts.size() ? props.pieceStarts[k + 1] : props.geometry.triangleCount();
        if (last <= first) continue;
        ++pieces;
        const int bad = inwardTriangles(props.geometry, first, last);
        if (bad > 0 && inverted < 5) {
            const JPH::Vec3 p(props.geometry.positions[3 * first]);
            std::printf("  piece %zu at %.0f %.1f %.0f: %d of %zu triangles face inward\n", k, p.GetX(), p.GetY(),
                        p.GetZ(), bad, last - first);
        }
        inverted += bad > 0;
        badTris += bad;
    }
    CHECK(inverted == 0, "%d of %d prop pieces have %d inward triangles", inverted, pieces, badTris);
    int rocks = 0, trunks = 0;
    for (const LevelRock& r : L.rocks()) {
        TrackGeometry g;
        addLevelRock(g, r);
        rocks += inwardTriangles(g, 0, g.triangleCount()) > 0;
    }
    for (std::size_t i = 0; i < L.trees().size(); i += 7) {
        TrackGeometry g;
        addLevelTrunk(g, L.trees()[i]);
        trunks += inwardTriangles(g, 0, g.triangleCount()) > 0;
    }
    CHECK(rocks == 0 && trunks == 0, "inverted rocks %d, trunks %d", rocks, trunks);
    std::printf("  %d prop pieces, %zu rocks, %zu trunks (every 7th) wound outward\n", pieces, L.rocks().size(),
                (L.trees().size() + 6) / 7);
}

// Placement: no building, prop, rock or tree trunk within its margin of a road (bridges excepted: they carry the
// roads), no building in water or on a spawn / teleport point.
struct Xz {
    float x, z;
};

std::vector<Xz> convexHull(std::vector<Xz> p) {
    std::sort(p.begin(), p.end(), [](const Xz& a, const Xz& b) { return a.x < b.x || (a.x == b.x && a.z < b.z); });
    const auto cross = [](const Xz& o, const Xz& a, const Xz& b) { return (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x); };
    std::vector<Xz> h(2 * p.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0.0f) --k;
        h[k++] = p[i];
    }
    for (std::size_t i = p.size() - 1, t = k + 1; i > 0; --i) {
        while (k >= t && cross(h[k - 2], h[k - 1], p[i - 1]) <= 0.0f) --k;
        h[k++] = p[i - 1];
    }
    h.resize(k > 1 ? k - 1 : k);
    return h;
}

// Distance from a point to a convex polygon (counter-clockwise), 0 inside.
float polygonDistance(const std::vector<Xz>& poly, Xz q) {
    bool inside = poly.size() >= 3;
    float best = 1e30f;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const Xz a = poly[i], b = poly[(i + 1) % poly.size()];
        const float ex = b.x - a.x, ez = b.z - a.z, px = q.x - a.x, pz = q.z - a.z;
        if (ex * pz - ez * px < 0.0f) inside = false;
        const float t = std::clamp((px * ex + pz * ez) / std::max(1e-9f, ex * ex + ez * ez), 0.0f, 1.0f);
        best = std::min(best, std::hypot(px - ex * t, pz - ez * t));
    }
    return inside ? 0.0f : best;
}

// Road center line points every 0.5 m with the half width.
struct RoadPoint {
    float x, z, hw;
};
std::vector<RoadPoint> roadPoints(const Level& L) {
    std::vector<RoadPoint> pts;
    for (const LevelRoad& r : L.roads())
        for (std::size_t i = 0; i + 1 < r.samples.size(); ++i) {
            const RoadSample &a = r.samples[i], &b = r.samples[i + 1];
            const int n = std::max(1, int(std::hypot(b.x - a.x, b.z - a.z) / 0.5f));
            for (int k = 0; k < n; ++k) {
                const float t = float(k) / float(n);
                pts.push_back({a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t, 0.5f * r.width});
            }
        }
    return pts;
}

void levelPlacement() {
    const Level& L = *level();
    const LevelProps& props = L.props();
    const std::vector<RoadPoint> road = roadPoints(L);
    const auto roadClearance = [&](const std::vector<Xz>& hull, float x0, float z0, float x1, float z1) {
        float best = 1e30f;
        for (const RoadPoint& p : road) {
            if (p.x < x0 - 60.0f || p.x > x1 + 60.0f || p.z < z0 - 60.0f || p.z > z1 + 60.0f) continue;
            best = std::min(best, polygonDistance(hull, {p.x, p.z}) - p.hw);
        }
        return best;
    };
    int counts[4] = {}, bad[4] = {};
    const char* names[4] = {"bridge", "building", "prop", "rock"};
    for (std::size_t k = 0; k < props.pieceStarts.size(); ++k) {
        const std::size_t first = props.pieceStarts[k];
        const std::size_t last = k + 1 < props.pieceStarts.size() ? props.pieceStarts[k + 1] : props.geometry.triangleCount();
        const PropRole role = props.pieceRoles[k];
        if (last <= first || role == PropRole::Bridge) continue;
        std::vector<Xz> pts;
        float x0 = 1e30f, z0 = 1e30f, x1 = -1e30f, z1 = -1e30f;
        for (std::size_t v = 3 * first; v < 3 * last; ++v) {
            const Xz p{props.geometry.positions[v].x, props.geometry.positions[v].z};
            pts.push_back(p);
            x0 = std::min(x0, p.x), z0 = std::min(z0, p.z), x1 = std::max(x1, p.x), z1 = std::max(z1, p.z);
        }
        const std::vector<Xz> hull = convexHull(pts);
        const float margin = role == PropRole::Building ? 3.0f : role == PropRole::Rock ? 2.0f : 1.5f;
        const float clear = roadClearance(hull, x0, z0, x1, z1);
        bool ok = clear >= margin;
        if (role == PropRole::Building) {
            for (const Xz& p : hull) ok = ok && !L.waterAt(p.x, p.z);
            for (const TrackFeature& f : L.features())
                ok = ok && polygonDistance(hull, {f.position.GetX(), f.position.GetZ()}) > 6.0f;
        }
        ++counts[int(role)];
        if (!ok) {
            if (bad[int(role)] < 6)
                std::printf("  %s piece %zu at %.0f %.0f: %.1f m from a road (margin %.1f)\n", names[int(role)], k,
                            0.5f * (x0 + x1), 0.5f * (z0 + z1), clear, margin);
            ++bad[int(role)];
        }
    }
    for (int r = 1; r < 4; ++r) CHECK(bad[r] == 0, "%d of %d %s pieces too close to a road, in water or on a spawn point", bad[r], counts[r], names[r]);
    int trees = 0, rocks = 0;
    for (const LevelTree& t : L.trees()) {
        const float trunk = 0.022f * t.height * t.width;
        std::vector<Xz> c{{t.x, t.z}};
        trees += roadClearance(c, t.x, t.z, t.x, t.z) - trunk < 2.0f;
    }
    for (const LevelRock& r : L.rocks()) {
        std::vector<Xz> c{{r.x, r.z}};
        rocks += roadClearance(c, r.x, r.z, r.x, r.z) - std::max(r.rx, r.rz) < 2.0f;
    }
    CHECK(trees == 0 && rocks == 0, "%d trees, %d rocks within 2 m of a road", trees, rocks);
    std::printf("  clear of the roads: %d buildings, %d props, %d rock props, %zu trees, %zu rocks (%zu road points); "
                "placement moved %d, removed %d\n",
                counts[1], counts[2], counts[3], L.trees().size(), L.rocks().size(), road.size(),
                L.bakeStats().propsMoved, L.bakeStats().propsRemoved);
}

// Every bridge, both vehicles: from the road 30 m before the deck across to 15 m past it, on the wheels.
void levelBridges() {
    const Level& L = *level();
    struct Crossing {
        const LevelRoad* road;
        std::size_t first, last; // deck samples
    };
    std::vector<Crossing> crossings;
    for (const LevelRoad& r : L.roads())
        for (std::size_t i = 0; i < r.samples.size(); ++i) {
            if (!r.samples[i].bridge || (i > 0 && r.samples[i - 1].bridge)) continue;
            std::size_t j = i;
            while (j + 1 < r.samples.size() && r.samples[j + 1].bridge) ++j;
            crossings.push_back({&r, i, j});
        }
    // No terrain above the deck surface (ramps included; the outer 1.5 m of each ramp meets the road).
    for (const LevelBridge& b : L.bridges()) {
        const float len = b.length(), dx = (b.x1 - b.x0) / len, dz = (b.z1 - b.z0) / len;
        float worst = -1e9f;
        for (float sa = -LevelBridge::kRamp + 1.5f; sa <= len + LevelBridge::kRamp - 1.5f; sa += 0.5f)
            for (float lat = -b.halfWidth; lat <= b.halfWidth; lat += 0.5f)
                worst = std::max(worst, L.heightAt(b.x0 + dx * sa + dz * lat, b.z0 + dz * sa - dx * lat) - b.deckY(sa));
        CHECK(worst < 0.0f, "bridge %.0f %.0f: terrain %.3f m above the deck", b.x0, b.z0, worst);
        std::printf("  bridge %.0f m: terrain at least %.3f m below the deck, ramps %.2f / %.2f m\n", len, -worst,
                    b.ya - b.y0, b.ye - b.y1);
    }
    CHECK(crossings.size() == L.bridges().size(), "%zu bridge crossings, %zu bridges", crossings.size(), L.bridges().size());
    for (const char* id : {"trailforge_suv", "trailforge_ev"}) {
        SimulationSettings s;
        s.level = level();
        s.vehicle = vehicle(id);
        s.terrain.async = false;
        Simulation sim(s);
        for (const Crossing& c : crossings) {
            for (int dir = 0; dir < 2; ++dir) {
                const std::vector<RoadSample>& rs = c.road->samples;
                // Sample index stepping in the driving direction.
                auto at = [&](long k) { return &rs[std::size_t(std::clamp<long>(k, 0, long(rs.size()) - 1))]; };
                const long step = dir == 0 ? 1 : -1;
                const long deckIn = dir == 0 ? long(c.first) : long(c.last);
                const long deckOut = dir == 0 ? long(c.last) : long(c.first);
                const RoadSample* start = at(deckIn - step * 15); // ~30 m before the deck
                const RoadSample* next = at(deckIn - step * 14);
                TrackFeature f;
                f.name = "bridge approach";
                f.position = JPH::Vec3(start->x, start->y, start->z);
                f.yaw = std::atan2(next->x - start->x, next->z - start->z);
                sim.requestPlace(f);
                DriverIntent hold;
                hold.brake = 1.0f;
                sim.setIntent(hold);
                runFor(sim, 2.0);
                long cursor = deckIn - step * 15;
                const RoadSample* exit = at(deckOut + step * 8);
                int impacts = 0, deckTicks = 0, deckFullContact = 0, chassisTicks = 0;
                float minDeckSpeed = 1e9f, minAbove = 1e9f;
                const float rideHeight = sim.currentSnapshot().body.position.GetY() - start->y;
                bool crossed = false;
                double t = 0.0;
                while (t < 40.0) {
                    const JPH::Vec3 p = sim.currentSnapshot().body.position;
                    const JPH::Quat q = sim.currentSnapshot().body.rotation;
                    // Pure pursuit on the road center line, 8 m ahead.
                    while (cursor != deckOut + step * 8) {
                        const RoadSample* n = at(cursor);
                        if (std::hypot(n->x - p.GetX(), n->z - p.GetZ()) > 8.0f &&
                            (JPH::Vec3(n->x, 0, n->z) - JPH::Vec3(p.GetX(), 0, p.GetZ())).Dot(q * JPH::Vec3::sAxisZ()) > 0.0f)
                            break;
                        cursor += step;
                    }
                    const RoadSample* target = at(cursor);
                    const JPH::Vec3 d(target->x - p.GetX(), 0.0f, target->z - p.GetZ());
                    const float lateral = d.Dot(q * JPH::Vec3::sAxisX()), ahead = d.Dot(q * JPH::Vec3::sAxisZ());
                    DriverIntent in;
                    in.throttle = 0.5f;
                    in.steer = std::clamp(-2.0f * std::atan2(lateral, std::max(ahead, 1.0f)), -1.0f, 1.0f);
                    in.analog = true;
                    sim.setIntent(in);
                    sim.tick();
                    t += sim.fixedStep();
                    impacts += int(sim.bodyImpacts().size());
                    // Over the deck (not the ramps into the banks): along the bridge between its end samples.
                    const RoadSample* a = &rs[c.first];
                    const RoadSample* b = &rs[c.last];
                    const JPH::Vec3 ab(b->x - a->x, 0.0f, b->z - a->z);
                    const float u = JPH::Vec3(p.GetX() - a->x, 0.0f, p.GetZ() - a->z).Dot(ab) / ab.LengthSq();
                    if (u > 0.05f && u < 0.95f) {
                        ++deckTicks;
                        deckFullContact += wheelsInContact(sim) == 4;
                        chassisTicks += chassisTouches(sim);
                        minDeckSpeed = std::min(minDeckSpeed, sim.vehicle().telemetry().speed);
                        const float deckY = a->y + (b->y - a->y) * u;
                        minAbove = std::min(minAbove, p.GetY() - deckY);
                    }
                    if ((JPH::Vec3(p.GetX() - exit->x, 0, p.GetZ() - exit->z)).Dot(JPH::Vec3(exit->dx, 0, exit->dz)) *
                            float(step) > 0.0f) {
                        crossed = true;
                        break;
                    }
                }
                const float len = std::hypot(rs[c.last].x - rs[c.first].x, rs[c.last].z - rs[c.first].z);
                CHECK(crossed && !sim.hadPhysicsError(), "%s: %s bridge (%.0f m) dir %d not crossed in %.1f s", id,
                      c.road->name.c_str(), len, dir, t);
                CHECK(deckTicks > 0 && deckFullContact >= deckTicks * 98 / 100,
                      "%s: %s bridge dir %d: 4 wheels down %d of %d deck ticks", id, c.road->name.c_str(), dir,
                      deckFullContact, deckTicks);
                CHECK(impacts == 0 && chassisTicks == 0, "%s: %s bridge dir %d: %d chassis impacts, %d deck ticks touching",
                      id, c.road->name.c_str(), dir, impacts, chassisTicks);
                CHECK(minDeckSpeed > 4.0f, "%s: %s bridge dir %d: min speed on the deck %.1f m/s", id,
                      c.road->name.c_str(), dir, minDeckSpeed);
                CHECK(minAbove > rideHeight - 0.15f, "%s: %s bridge dir %d: body %.2f m above the deck (%.2f m on the road)",
                      id, c.road->name.c_str(), dir, minAbove, rideHeight);
                std::printf("  %s: %s bridge %.0f m dir %d: %.1f s, 4 wheels %d/%d deck ticks, min %.1f m/s, "
                            "height %.2f (road %.2f) m, chassis impacts %d / touching %d ticks\n",
                            id, c.road->name.c_str(), len, dir, t, deckFullContact, deckTicks, minDeckSpeed, minAbove,
                            rideHeight, impacts, chassisTicks);
            }
        }
        CHECK(sim.recoveries() == 0, "%s: %d recoveries on the bridges", id, sim.recoveries());
    }
}

// KTX2 container: write / read round trip of a full BC7 mip chain, skipping the largest levels, structural errors
// and the Git LFS pointer check; the level skip used by the quality presets.
void terrainTexturesKtx2() {
    ktx2::Image img;
    img.vkFormat = ktx2::kVkFormatBc7Srgb;
    img.width = img.height = 64;
    for (std::uint32_t s = 64; s >= 1; s /= 2) {
        ktx2::Level l;
        l.width = l.height = s;
        l.data.resize(ktx2::bc7LevelBytes(s, s));
        for (std::size_t i = 0; i < l.data.size(); ++i) l.data[i] = std::uint8_t(i * 7 + s);
        img.levels.push_back(std::move(l));
    }
    CHECK(img.levels.size() == 7 && ktx2::bc7LevelBytes(2, 2) == 16 && ktx2::bc7LevelBytes(64, 64) == 4096,
          "levels %zu", img.levels.size());
    const std::vector<std::uint8_t> file = ktx2::write(img, "trailforge test");

    ktx2::Image back;
    std::string err;
    CHECK(ktx2::read(file, back, err), "read: %s", err.c_str());
    bool same = back.vkFormat == img.vkFormat && back.width == 64 && back.levels.size() == img.levels.size();
    for (std::size_t i = 0; same && i < img.levels.size(); ++i)
        same = back.levels[i].width == img.levels[i].width && back.levels[i].data == img.levels[i].data;
    CHECK(same, "round trip differs");

    ktx2::Image skipped;
    CHECK(ktx2::read(file, skipped, err, 2), "read with skip: %s", err.c_str());
    CHECK(skipped.width == 16 && skipped.levels.size() == 5 && skipped.levels[0].data == img.levels[2].data &&
              skipped.levels.back().width == 1,
          "skip 2: width %u, %zu levels", skipped.width, skipped.levels.size());

    const std::string path = (std::filesystem::temp_directory_path() / "trailforge_test_texture.ktx2").string();
    {
        std::FILE* fp = std::fopen(path.c_str(), "wb");
        CHECK(fp != nullptr, "cannot write %s", path.c_str());
        if (fp) {
            std::fwrite(file.data(), 1, file.size(), fp);
            std::fclose(fp);
        }
    }
    ktx2::Image fromFile;
    CHECK(ktx2::readFile(path, fromFile, err, 1) && fromFile.width == 32 && fromFile.levels.size() == 6 &&
              fromFile.levels[0].data == img.levels[1].data,
          "readFile with skip: %s", err.c_str());
    std::remove(path.c_str());

    std::vector<std::uint8_t> cut(file.begin(), file.end() - 100);
    CHECK(!ktx2::read(cut, back, err), "truncated file accepted");
    std::vector<std::uint8_t> bad = file;
    bad[5] ^= 0xff;
    CHECK(!ktx2::read(bad, back, err), "bad identifier accepted");
    const std::string ptr = "version https://git-lfs.github.com/spec/v1\noid sha256:00\nsize 22370144\n";
    CHECK(ktx2::isLfsPointer(std::vector<std::uint8_t>(ptr.begin(), ptr.end())) && !ktx2::isLfsPointer(file),
          "LFS pointer detection");

    CHECK(terrainTextureSkip(4096, 4096) == 0 && terrainTextureSkip(4096, 2048) == 1 &&
              terrainTextureSkip(4096, 512) == 3 && terrainTextureSkip(2048, 4096) == 0 &&
              terrainTextureSkip(2048, 1024) == 1 && terrainTextureSkip(16, 1) == 2,
          "terrainTextureSkip");
}

// The shipped terrain textures (assets/textures/terrain): manifest in the shader's layer order, every map a
// square BC7 texture of the manifest size with a full mip chain down to 1x1. Needs the LFS objects (`git lfs pull`).
void terrainTexturesAssets() {
    const std::string dir = std::string(TRAILFORGE_SOURCE_DIR) + "/assets/textures/terrain";
    TerrainTextureSet set;
    std::string err;
    CHECK(loadTerrainTextureSet(dir, set, err), "%s", err.c_str());
    if (set.layers.size() != std::size_t(TerrainTextureSet::kLayers)) return;
    std::printf("  terrain textures: albedo %u, normal / roughness / AO %u texels\n", set.albedoSize, set.nraSize);
    std::size_t total = 0;
    for (const TerrainTextureSet::Layer& l : set.layers) {
        for (int k = 0; k < 2; ++k) {
            const bool albedo = k == 0;
            const std::string& path = albedo ? l.albedoPath : l.nraPath;
            const std::uint32_t size = albedo ? set.albedoSize : set.nraSize;
            ktx2::Image img;
            const bool ok = loadTerrainTextureMap(path, albedo, 0, img, err);
            CHECK(ok, "%s", err.c_str());
            if (!ok) continue;
            std::uint32_t mips = 0;
            for (std::uint32_t s = size; s >= 1; s /= 2) ++mips;
            bool chain = img.width == size && img.levels.size() == mips;
            for (std::size_t i = 0; chain && i < img.levels.size(); ++i)
                chain = img.levels[i].width == std::max(size >> i, 1u) &&
                        img.levels[i].data.size() == ktx2::bc7LevelBytes(img.levels[i].width, img.levels[i].height);
            CHECK(chain, "%s: %u texels, %zu levels (want %u, %u)", path.c_str(), img.width, img.levels.size(), size,
                  mips);
            for (const ktx2::Level& lv : img.levels) total += lv.data.size();
        }
    }
    std::printf("  terrain textures: %.1f MB of BC7 data in 12 maps\n", double(total) / (1024.0 * 1024.0));
}

// Grass, flowers and wheat (mask R and A, sampled bilinearly like the grass shader) are zero on every road surface and
// on every bridge deck with its ramps.
void levelGrassClear() {
    const Level& L = *level();
    const int n = L.cells();
    const auto& mask = L.maskTexels();
    const float c0 = L.origin() + 0.5f * L.cell();
    const auto sample = [&](float x, float z, int ch) {
        const float fx = (x - c0) / L.cell(), fz = (z - c0) / L.cell();
        const int i = std::clamp(int(std::floor(fx)), 0, n - 2), j = std::clamp(int(std::floor(fz)), 0, n - 2);
        const float tx = std::clamp(fx - float(i), 0.0f, 1.0f), tz = std::clamp(fz - float(j), 0.0f, 1.0f);
        const auto v = [&](int a, int b) {
            return float((mask[std::size_t(b) * std::size_t(n) + std::size_t(a)] >> (8 * ch)) & 0xffu) / 255.0f;
        };
        return (v(i, j) * (1 - tx) + v(i + 1, j) * tx) * (1 - tz) + (v(i, j + 1) * (1 - tx) + v(i + 1, j + 1) * tx) * tz;
    };
    float worstRoad = 0.0f, worstDeck = 0.0f;
    int roadPts = 0, deckPts = 0;
    for (const LevelRoad& r : L.roads())
        for (const RoadSample& s : r.samples) {
            if (s.bridge || !L.insidePlayArea(s.x, s.z, 2.0f)) continue;
            for (int k = -2; k <= 2; ++k) {
                const float lat = 0.5f * r.width * float(k) / 2.0f;
                const float x = s.x + s.dz * lat, z = s.z - s.dx * lat;
                worstRoad = std::max({worstRoad, sample(x, z, 0), sample(x, z, 3)});
                ++roadPts;
            }
        }
    for (const LevelBridge& b : L.bridges()) {
        const float len = b.length(), dx = (b.x1 - b.x0) / len, dz = (b.z1 - b.z0) / len;
        for (float a = -LevelBridge::kRamp; a <= len + LevelBridge::kRamp; a += 0.5f)
            for (int k = -2; k <= 2; ++k) {
                const float lat = b.halfWidth * float(k) / 2.0f;
                const float x = b.x0 + dx * a + dz * lat, z = b.z0 + dz * a - dx * lat;
                worstDeck = std::max({worstDeck, sample(x, z, 0), sample(x, z, 3)});
                ++deckPts;
            }
    }
    CHECK(worstRoad < 0.5f / 255.0f, "grass / wheat %.4f on a road surface", worstRoad);
    CHECK(worstDeck < 0.5f / 255.0f, "grass / wheat %.4f on a bridge deck", worstDeck);
    std::printf("  grass clear: %d road points (max %.4f), %d deck points of %zu bridges (max %.4f)\n", roadPts,
                worstRoad, deckPts, L.bridges().size(), worstDeck);
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"level_heightfield_sampling", levelHeightfieldSampling},
        {"level_tile_streaming", levelTileStreaming},
        {"level_road_carving", levelRoadCarving},
        {"level_surfaces_water", levelSurfacesAndWater},
        {"level_vehicles", levelVehicles},
        {"level_prop_winding", levelPropWinding},
        {"level_bridges", levelBridges},
        {"level_placement", levelPlacement},
        {"level_grass_clear", levelGrassClear},
        {"terrain_textures_ktx2", terrainTexturesKtx2},
        {"terrain_textures_assets", terrainTexturesAssets},
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
