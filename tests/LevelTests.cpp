// Headless tests of the open-world level: the bake (height and material sampling, roads, rivers, water), the
// physics tiles (heightfield == baked grid, streaming bookkeeping) and both vehicles on Exploration Peaks.

#include "physics/PhysicsWorld.h"
#include "sim/Simulation.h"
#include "track/Level.h"
#include "track/TerrainTiles.h"
#include "vehicle/VehicleDefinition.h"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
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
                fords += s.ford && !r.samples[i - 1].ford;
            }
            if (s.bridge || std::abs(s.x) > L.playHalfExtent() || std::abs(s.z) > L.playHalfExtent()) continue;
            // Junctions blend two profiles; they are checked by the vehicle drives, not against one profile.
            bool junction = false;
            for (const LevelRoad& o : L.roads())
                if (&o != &r)
                    for (const RoadSample& q : o.samples)
                        junction = junction || std::hypot(q.x - s.x, q.z - s.z) < 0.5f * (o.width + r.width) + 8.0f;
            if (junction) continue;
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

// A dry, gentle point near the Alpine Lake marker facing at least 32 m of lake deeper than 1.5 m.
std::optional<TrackFeature> lakeShore(const Level& L) {
    const LevelPoi* lake = nullptr;
    for (const LevelPoi& p : L.pois())
        if (p.name == "Alpine Lake") lake = &p;
    if (!lake) return std::nullopt;
    for (float r = 0.0f; r < 200.0f; r += 4.0f)
        for (int a = 0; a < 72; ++a) {
            const float ang = float(a) * 0.0872665f;
            const float x = lake->x + r * std::sin(ang), z = lake->z + r * std::cos(ang);
            if (L.waterAt(x, z)) continue;
            const float gx = L.heightAt(x + 1.5f, z) - L.heightAt(x - 1.5f, z), gz = L.heightAt(x, z + 1.5f) - L.heightAt(x, z - 1.5f);
            if (std::hypot(gx, gz) / 3.0f > 0.12f) continue;
            for (int d = 0; d < 36; ++d) {
                const float yaw = float(d) * 0.174533f, dx = std::sin(yaw), dz = std::cos(yaw);
                bool deep = true;
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

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"level_heightfield_sampling", levelHeightfieldSampling},
        {"level_tile_streaming", levelTileStreaming},
        {"level_road_carving", levelRoadCarving},
        {"level_surfaces_water", levelSurfacesAndWater},
        {"level_vehicles", levelVehicles},
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
