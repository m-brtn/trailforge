// Headless tests of the open-world level: the bake (height and material sampling, roads, rivers, water), the
// physics tiles (heightfield == baked grid, streaming bookkeeping), both vehicles on Exploration Peaks and the
// terrain texture files (KTX2 container, shipped BC7 maps) and the building kit (collision pieces vs render LODs,
// materials vs assets/textures/buildings).

#include "core/Json.h"
#include "core/Ktx2.h"
#include "core/TerrainTextureSet.h"
#include "physics/PhysicsWorld.h"
#include "sim/Simulation.h"
#include "track/BuildingKit.h"
#include "track/Level.h"
#include "track/LevelFiles.h"
#include "track/TerrainTiles.h"
#include "track/TestTrack.h"
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
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
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

    // Model rocks, logs and cliffs: the tile compound (shared model mesh, scaled and turned) is the triangles of
    // addLevelRock. Rays straight down near the instance centre hit both at the same height.
    int rockRays = 0, rockSame = 0, rockBelow = 0, rockCovered = 0;
    for (std::size_t i = 0; i < L.rocks().size() && rockRays < 300; i += 2) {
        const LevelRock& r = L.rocks()[i];
        if (r.model < 0) continue;
        TrackGeometry g;
        addLevelRock(g, r, L.models());
        const float x = r.x + rng.range(-0.3f, 0.3f) * r.rx, z = r.z + rng.range(-0.3f, 0.3f) * r.rz;
        float triY = -1e9f;
        for (std::size_t t = 0; t < g.triangleCount(); ++t) {
            const JPH::Vec3 a(g.positions[3 * t]), b(g.positions[3 * t + 1]), c(g.positions[3 * t + 2]);
            // Vertical ray: barycentric coordinates in the xz projection.
            const float d = (b.GetX() - a.GetX()) * (c.GetZ() - a.GetZ()) - (c.GetX() - a.GetX()) * (b.GetZ() - a.GetZ());
            if (std::abs(d) < 1e-9f) continue;
            const float u = ((x - a.GetX()) * (c.GetZ() - a.GetZ()) - (c.GetX() - a.GetX()) * (z - a.GetZ())) / d;
            const float v = ((b.GetX() - a.GetX()) * (z - a.GetZ()) - (x - a.GetX()) * (b.GetZ() - a.GetZ())) / d;
            if (u < 0.0f || v < 0.0f || u + v > 1.0f) continue;
            triY = std::max(triY, a.GetY() + u * (b.GetY() - a.GetY()) + v * (c.GetY() - a.GetY()));
        }
        if (triY < -1e8f) continue;
        const JPH::RRayCast ray(JPH::RVec3(x, 600.0f, z), JPH::Vec3(0.0f, -800.0f, 0.0f));
        JPH::RayCastResult hit;
        if (!world.system().GetNarrowPhaseQuery().CastRay(ray, hit)) continue;
        const float y = 600.0f - 800.0f * hit.mFraction;
        ++rockRays;
        rockSame += std::abs(y - triY) < 0.01f;
        rockBelow += y < triY - 0.01f;
        rockCovered += y > triY + 0.01f && std::abs(y - L.heightAt(x, z)) < 0.01f;
    }
    std::printf("  %d model rock rays: %d at the addLevelRock height, %d under the ground, %d below it\n", rockRays,
                rockSame, rockCovered, rockBelow);
    CHECK(rockRays > 50, "model rock rays %d", rockRays);
    CHECK(rockBelow == 0, "%d rays passed through a model rock", rockBelow);
    CHECK(rockSame * 10 >= rockRays * 8, "only %d of %d rays at the rock surface", rockSame, rockRays);
    CHECK((rockSame + rockCovered) * 20 >= rockRays * 19, "only %d + %d of %d rays at the rock or the ground", rockSame,
          rockCovered, rockRays);

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
            offRoad += !(sf == r.surface || (r.surface == SurfaceId::PackedDirt && sf == SurfaceId::Mud) ||
                         sf == SurfaceId::Snow || (s.ford && sf == SurfaceId::Rock));
        }
        std::printf("  road %-17s %5.0f m: grade %.1f %% (limit %.1f %%), center %.3f m (at %.0f %.0f), across %.3f m, "
                    "%d/%d off its surface\n",
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
    // Road running surfaces follow level.json: the main roads are gravel, the trails packed dirt with mud patches
    // (snow above the snow line and the rock bars of fords override both).
    for (const LevelRoad& r : L.roads()) {
        int own = 0, n = 0;
        for (const RoadSample& s : r.samples) {
            const SurfaceId at = L.surfaceAt(s.x, s.z);
            if (s.bridge || at == SurfaceId::Snow || at == SurfaceId::Rock) continue;
            ++n;
            own += r.trail ? at == SurfaceId::PackedDirt || at == SurfaceId::Mud : at == SurfaceId::Gravel;
        }
        std::printf("  %-18s %s: %d / %d center samples on its surface\n", r.name.c_str(), surfaceKey(r.surface), own, n);
        CHECK(r.surface == (r.trail ? SurfaceId::PackedDirt : SurfaceId::Gravel), "%s surface %s", r.name.c_str(),
              surfaceKey(r.surface));
        CHECK(own >= n * 9 / 10, "%s: %d of %d center samples on its surface", r.name.c_str(), own, n);
    }
    CHECK(counts[std::size_t(SurfaceId::Gravel)] > 5000, "gravel only %d cells", counts[std::size_t(SurfaceId::Gravel)]);
    CHECK(counts[std::size_t(SurfaceId::Asphalt)] == 0, "asphalt %d cells (no asphalt road in this level)",
          counts[std::size_t(SurfaceId::Asphalt)]);
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
    int rocks = 0, trunks = 0, modelRocks = 0, cliffs = 0;
    for (const LevelRock& r : L.rocks()) {
        TrackGeometry g;
        addLevelRock(g, r, L.models());
        if (r.model < 0) {
            rocks += inwardTriangles(g, 0, g.triangleCount()) > 0;
            continue;
        }
        ++modelRocks;
        if (L.models()[std::size_t(r.model)].kind == "cliff") {
            // Cliff faces are open shells: their area-weighted normal must face out of the wall (the placement's
            // pitched +Z, which leans toward the downhill side) and away from the ground.
            JPH::Vec3 n = JPH::Vec3::sZero();
            for (std::size_t t = 0; t < g.triangleCount(); ++t) {
                const JPH::Vec3 a(g.positions[3 * t]), b(g.positions[3 * t + 1]), d(g.positions[3 * t + 2]);
                n += (b - a).Cross(d - a);
            }
            n = n.NormalizedOr(JPH::Vec3::sZero());
            const JPH::Vec3 face(std::sin(r.yaw) * std::cos(r.pitch), -std::sin(r.pitch), std::cos(r.yaw) * std::cos(r.pitch));
            ++cliffs;
            rocks += n.Dot(face) < 0.6f || n.GetY() < 0.0f;
            continue;
        }
        // Model rocks are not convex: the enclosed signed volume must be positive (wound outward) and close to
        // the scaled model volume order of magnitude (not a flat or inverted shell).
        const JPH::Vec3 c(r.x, r.y, r.z);
        double vol = 0.0;
        for (std::size_t t = 0; t < g.triangleCount(); ++t) {
            const JPH::Vec3 a = JPH::Vec3(g.positions[3 * t]) - c, b = JPH::Vec3(g.positions[3 * t + 1]) - c,
                            d = JPH::Vec3(g.positions[3 * t + 2]) - c;
            vol += double(a.Dot(b.Cross(d))) / 6.0;
        }
        rocks += vol < 0.05 * double(r.rx * r.ry * r.rz);
    }
    for (std::size_t i = 0; i < L.trees().size(); i += 7) {
        TrackGeometry g;
        addLevelTrunk(g, L.trees()[i], L.models());
        trunks += inwardTriangles(g, 0, g.triangleCount()) > 0;
    }
    CHECK(rocks == 0 && trunks == 0, "inverted rocks %d, trunks %d", rocks, trunks);
    std::printf("  %d prop pieces, %zu rocks (%d models, %d cliff faces), %zu trunks (every 7th) wound outward\n", pieces,
                L.rocks().size(), modelRocks, cliffs, (L.trees().size() + 6) / 7);
    CHECK(L.models().empty() || cliffs > 50, "only %d cliff faces", cliffs);
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

// ------------------------------------------------------------------------------------------- menu attract autopilot
// One full loop of the attract route from the spawn, per world and vehicle (the SUV in 2H and in 4H), without an
// autopilot reset or a recovery, close to the center line and at a gentle speed.
void autopilotLoops() {
    struct Case {
        const char* vehicle;
        std::optional<TransferMode> transfer;
    };
    const Case cases[] = {{"trailforge_suv", TransferMode::TwoHigh}, {"trailforge_suv", TransferMode::FourHigh},
                          {"trailforge_ev", std::nullopt}};
    for (const bool open : {true, false}) {
        for (const Case& c : cases) {
            SimulationSettings s;
            if (open) s.level = level();
            s.vehicle = vehicle(c.vehicle);
            s.terrain.async = false;
            Simulation sim(s);
            if (c.transfer) sim.requestTransfer(*c.transfer);
            sim.setAutopilot(AutopilotMode::Drive);
            const Autopilot& ap = sim.autopilot();
            CHECK(ap.hasRoute(), "%s: no attract route", open ? "level" : "proving ground");
            if (!ap.hasRoute()) return;
            const float length = ap.routeLength();
            const auto t0 = std::chrono::steady_clock::now();
            double t = 0.0, speedSum = 0.0, maxKmh = 0.0;
            float minSoc = 1.0f, minUp = 1.0f;
            long ticks = 0;
            bool regen = false;
            float driven = 0.0f;
            JPH::Vec3 last = sim.currentSnapshot().body.position;
            std::vector<std::string> trace;
            while (ap.distanceDriven() < length && t < length / 2.5 + 30.0 && sim.autopilotResets() == 0) {
                driven = ap.distanceDriven();
                last = sim.currentSnapshot().body.position;
                if (ticks % 60 == 0) {
                    const VehicleTelemetry& tel = sim.vehicle().telemetry();
                    char buf[256];
                    std::snprintf(buf, sizeof(buf), "    t %.1f s %.0f (%.1f %.1f %.1f) v %.1f tgt %.1f lat %.2f thr %.2f brk %.2f steer %+.2f up %.2f wheels %d",
                                  t, double(ap.progress()), double(last.GetX()), double(last.GetY()), double(last.GetZ()),
                                  double(tel.forwardSpeed), double(ap.targetSpeed()), double(ap.lateralError()),
                                  double(tel.command.throttle), double(tel.command.brake), double(tel.command.steer),
                                  double(tel.upDot), wheelsInContact(sim));
                    trace.push_back(buf);
                }
                sim.tick();
                t += sim.fixedStep();
                ++ticks;
                const VehicleTelemetry& tel = sim.vehicle().telemetry();
                speedSum += tel.forwardSpeed;
                maxKmh = std::max(maxKmh, double(tel.forwardSpeed) * 3.6);
                minUp = std::min(minUp, tel.upDot);
                if (tel.ev.electric) {
                    minSoc = std::min(minSoc, tel.ev.soc);
                    regen = regen || tel.ev.regenActive;
                }
            }
            if (std::getenv("TF_AP_DEBUG") && sim.autopilotResets() > 0)
                for (std::size_t k = trace.size() > 40 ? trace.size() - 40 : 0; k < trace.size(); ++k)
                    std::printf("%s\n", trace[k].c_str());
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            const char* name = c.transfer ? (*c.transfer == TransferMode::TwoHigh ? "SUV 2H" : "SUV 4H") : "EV";
            std::printf("  %s %s: route '%s' %.0f m, driven %.0f m in %.1f s (avg %.1f km/h, max %.1f km/h), max lateral "
                        "%.2f m, min up %.2f, resets %d (%s), recoveries %d%s, %.2f s wall\n",
                        open ? "Exploration Peaks" : "proving ground", name, ap.route().name.c_str(), double(length),
                        double(ap.distanceDriven()), t, speedSum / double(std::max(ticks, 1L)) * 3.6, maxKmh,
                        double(ap.maxLateralError()), double(minUp), sim.autopilotResets(), ap.resetReason(),
                        sim.recoveries(), regen ? ", regen used" : "", wall);
            CHECK(ap.distanceDriven() >= length, "%s: only %.0f of %.0f m in %.0f s", name, double(ap.distanceDriven()),
                  double(length), t);
            CHECK(sim.autopilotResets() == 0 && sim.recoveries() == 0 && !sim.hadPhysicsError(),
                  "%s: resets %d (%s after %.0f m at %.0f, %.0f), recoveries %d", name, sim.autopilotResets(),
                  sim.autopilotResetReason(), double(driven), double(last.GetX()), double(last.GetZ()), sim.recoveries());
            CHECK(ap.maxLateralError() < 2.5f, "%s: %.2f m off the center line", name, double(ap.maxLateralError()));
            CHECK(maxKmh < 45.0, "%s: %.1f km/h", name, maxKmh);
            if (open) CHECK(length > 1000.0f, "level route only %.0f m", double(length));
        }
    }
}

// No vertical cuts in the dry ground: lake outlines and the bake's coast cap used to leave straight walls of 6 to 60 m
// (one-cell steps). Canyon walls are steep on purpose, so the limit is on near-vertical cells and on single steps;
// the rock steps of the waterfalls are left out.
void levelSlopes() {
    const Level& L = *level();
    const int n = L.samples();
    const auto& h = L.heights();
    long dry = 0, over80 = 0;
    float worstStep = 0.0f, stepX = 0.0f, stepZ = 0.0f;
    for (int j = 1; j + 1 < n; ++j)
        for (int i = 1; i + 1 < n; ++i) {
            const float x = L.origin() + float(i) * L.cell(), z = L.origin() + float(j) * L.cell();
            if (!L.insidePlayArea(x, z, 0.0f) || L.waterAt(x, z)) continue;
            bool fall = false; // the rock step behind a waterfall is vertical on purpose
            for (const LevelWaterfall& w : L.waterfalls())
                fall = fall || std::hypot(x - w.x0, z - w.z0) < w.width + 6.0f;
            if (fall) continue;
            const std::size_t k = std::size_t(j) * std::size_t(n) + std::size_t(i);
            const float gx = (h[k + 1] - h[k - 1]) / (2.0f * L.cell()), gz = (h[k + std::size_t(n)] - h[k - std::size_t(n)]) / (2.0f * L.cell());
            ++dry;
            if (std::hypot(gx, gz) > 5.67f) ++over80; // tan 80 deg
            const float step = std::max(std::abs(h[k + 1] - h[k]), std::abs(h[k + std::size_t(n)] - h[k]));
            if (step > worstStep) {
                worstStep = step;
                stepX = x;
                stepZ = z;
            }
        }
    const double share = 100.0 * double(over80) / double(std::max(dry, 1L));
    std::printf("  dry cells %ld, steeper than 80 deg %.3f %%, largest 1 m step %.2f m at (%.0f, %.0f)\n", dry, share,
                double(worstStep), double(stepX), double(stepZ));
    CHECK(share < 0.04, "%.3f %% of the dry cells are steeper than 80 deg", share);
    CHECK(worstStep < 8.0f, "a %.2f m step between neighbouring samples at (%.0f, %.0f)", double(worstStep), double(stepX),
          double(stepZ));
}

// One spec per kind plus the house variants placed on the level.
std::vector<BuildingSpec> kitSpecs() {
    using BK = BuildingKind;
    std::vector<BuildingSpec> v = {
        {BK::House, 9.0f, 8.0f, 0.0f, 2, kHouseStoneBase | kHouseBalcony | kHouseOchre},
        {BK::House, 10.0f, 8.5f, 0.0f, 3, kHouseStoneBase | kHouseUpperPlaster | kHouseSideBalcony | (2 << kHouseShutterShift)},
        {BK::House, 6.0f, 4.5f, 0.0f, 1, kHouseTimber | kHouseShingles | (3 << kHouseShutterShift)},
        {BK::FuelCanopy, 10.0f, 8.0f, 4.8f, 0, 0}, {BK::FuelShop, 8.0f, 6.0f, 0, 0, 0}, {BK::Pylon, 0, 0, 6.0f, 0, 0},
        {BK::Garage, 12.0f, 9.0f, 4.4f, 0, 0}, {BK::Garage, 10.0f, 8.0f, 4.0f, 0, 1}, {BK::Lighthouse, 0, 0, 18.0f, 0, 0},
        {BK::RadioMast, 0, 0, 34.0f, 0, 0}, {BK::RadioHut, 4.0f, 3.0f, 0, 0, 0}, {BK::RuinWall, 14.0f, 0, 5.0f, 0, 3},
        {BK::RuinTower, 6.0f, 0, 9.0f, 0, 1}, {BK::RuinGate, 5.0f, 0, 0, 0, 0}, {BK::Crusher, 0, 0, 9.0f, 0, 0},
        {BK::Conveyor, 12.0f, 0, 3.0f, 0, 0}, {BK::Silo, 3.0f, 0, 8.0f, 0, 0}, {BK::Dock, 20.0f, 2.4f, 4.0f, 0, 0},
        {BK::Tent, 0, 0, 0, 0, 0}, {BK::Tent, 0, 0, 0, 0, 1}, {BK::Fence, 9.0f, 0, 0, 0, 0}, {BK::Fence, 9.0f, 0, 0, 0, 1},
        {BK::Fence, 9.0f, 0, 0, 0, 2}, {BK::Garden, 6.0f, 5.0f, 0, 0, 1}, {BK::PowerPole, 0, 0, 8.0f, 0, 0},
        {BK::Wire, 18.0f, 0, 0.5f, 0, 0}, {BK::Woodpile, 5.0f, 0, 0, 0, 0}, {BK::Cairn, 0, 0, 0, 0, 0},
        {BK::Signpost, 0, 0, 0, 0, 0}, {BK::Well, 0, 0, 0, 0, 0}, {BK::Shelter, 5.0f, 4.0f, 0, 0, 0},
        {BK::Bench, 1.8f, 0, 0, 0, 0}, {BK::Barrier, 4.0f, 0, 0, 0, 0}, {BK::Barrier, 6.0f, 0, 0, 0, 1},
        {BK::TyreWall, 6.0f, 0, 0, 0, 0}, {BK::TyreWall, 4.0f, 0, 0, 0, 1}, {BK::Flag, 0, 0, 5.0f, 0, 2},
        {BK::Grandstand, 24.0f, 9.0f, 0, 0, 0}, {BK::StartGantry, 14.0f, 0, 6.0f, 0, 0}, {BK::RaceControl, 12.0f, 8.0f, 0, 0, 0}, {BK::MarshalPost, 0, 0, 0, 0, 0},
    };
    return v;
}

struct KitPiece {
    std::vector<JPH::Float3> verts;
    std::vector<std::pair<JPH::Float3, float>> planes; // outward normal, offset
};

std::vector<KitPiece> kitPieces(const BuildingGeometry& g) {
    std::vector<KitPiece> out;
    const std::size_t tris = g.collision.size() / 3;
    for (std::size_t k = 0; k < g.pieceStarts.size(); ++k) {
        const std::size_t first = g.pieceStarts[k], last = k + 1 < g.pieceStarts.size() ? g.pieceStarts[k + 1] : tris;
        KitPiece p;
        for (std::size_t t = first; t < last; ++t) {
            const JPH::Vec3 a(g.collision[3 * t]), b(g.collision[3 * t + 1]), c(g.collision[3 * t + 2]);
            for (int i = 0; i < 3; ++i) p.verts.push_back(g.collision[3 * t + std::size_t(i)]);
            const JPH::Vec3 n = (b - a).Cross(c - a);
            if (n.Length() < 1e-9f) continue;
            const JPH::Vec3 u = n.Normalized();
            p.planes.push_back({JPH::Float3(u.GetX(), u.GetY(), u.GetZ()), u.Dot(a)});
        }
        out.push_back(std::move(p));
    }
    return out;
}

// Largest plane distance of a point outside a convex piece (<= 0 inside; a lower bound of the true distance).
float pieceDistance(const KitPiece& p, const JPH::Float3& q) {
    float d = -1e30f;
    for (const auto& [n, o] : p.planes) d = std::max(d, n.x * q.x + n.y * q.y + n.z * q.z - o);
    return d;
}

// Every collision piece is convex and wound outward; the far LOD (LOD 2) has every vertex below 6 m on the
// collision (within 3 cm: decals 12 mm proud), the near LODs within kKitTrim (trims standing proud of the solid parts).
void buildingKitParts() {
    int specs = 0;
    for (const BuildingSpec& spec : kitSpecs()) {
        const BuildingGeometry g = buildBuilding(spec, true);
        const char* name = buildingKindName(spec.kind);
        ++specs;
        const bool renderOnly = spec.kind == BuildingKind::Wire;
        CHECK(renderOnly == g.collision.empty(), "%s: %zu collision triangles", name, g.collision.size() / 3);
        CHECK(g.model.lods.size() == 3, "%s: %zu LODs", name, g.model.lods.size());
        if (g.model.lods.size() != 3) continue;
        const std::vector<KitPiece> pieces = kitPieces(g);
        int concave = 0, inward = 0;
        for (const KitPiece& p : pieces) {
            JPH::Float3 c(0, 0, 0);
            for (const JPH::Float3& v : p.verts) c = JPH::Float3(c.x + v.x, c.y + v.y, c.z + v.z);
            const float inv = 1.0f / float(std::max<std::size_t>(p.verts.size(), 1));
            c = JPH::Float3(c.x * inv, c.y * inv, c.z * inv);
            bool convex = true;
            for (const JPH::Float3& v : p.verts) convex = convex && pieceDistance(p, v) < 1e-3f;
            concave += !convex;
            inward += pieceDistance(p, c) >= 0.0f;
        }
        CHECK(concave == 0 && inward == 0, "%s: %d of %zu pieces not convex, %d wound inward", name, concave,
              pieces.size(), inward);
        for (int lod = 0; lod < 3 && !renderOnly; ++lod) {
            const float tol = lod == 2 ? 0.03f : kKitTrim + 0.03f;
            int far = 0, checked = 0;
            float worst = 0.0f;
            for (const ModelVertex& mv : g.model.lods[std::size_t(lod)].vertices) {
                const JPH::Float3 q(mv.pos[0], mv.pos[1], mv.pos[2]);
                if (q.y > 6.0f) continue;
                float d = 1e30f;
                for (const KitPiece& p : pieces) d = std::min(d, pieceDistance(p, q));
                ++checked;
                if (d > tol) ++far, worst = std::max(worst, d);
            }
            CHECK(far == 0, "%s LOD%d: %d of %d vertices below 6 m farther than %.2f m from the collision (max %.2f)",
                  name, lod, far, checked, double(tol), double(worst));
        }
    }
    std::printf("  %d kit specs checked\n", specs);
}

// Every textured kit material names a layer of buildings.json with the same tile size and both maps on disk, and
// the set keeps alpha free of height (the level model shader cuts alpha below 0.5).
void buildingKitMaterials() {
    const std::string dir = std::string(TRAILFORGE_SOURCE_DIR) + "/assets/textures/buildings";
    std::ifstream f(dir + "/buildings.json");
    std::stringstream ss;
    ss << f.rdbuf();
    json::Value doc;
    std::string err;
    CHECK(json::parse(ss.str(), doc, &err), "buildings.json: %s", err.c_str());
    CHECK(!doc["heightInAlpha"].boolOr(true), "buildings.json must set heightInAlpha false");
    int textured = 0;
    for (int m = 0; m < kKitMatCount; ++m) {
        const KitMaterial& km = kitMaterial(KitMat(m));
        if (!km.layer) {
            CHECK(km.roughness >= 0.0f, "%s: untextured without a constant roughness", km.name);
            continue;
        }
        ++textured;
        const json::Value* layer = nullptr;
        for (const json::Value& l : doc["layers"].array)
            if (l["id"].stringOr("") == km.layer) layer = &l;
        CHECK(layer != nullptr, "%s: layer %s not in buildings.json", km.name, km.layer);
        if (!layer) continue;
        CHECK(std::abs((*layer)["tile"].numberOr(0.0) - double(km.tile)) < 1e-4, "%s: tile %.2f, buildings.json %.2f",
              km.name, double(km.tile), (*layer)["tile"].numberOr(0.0));
        for (const char* map : {"_albedo.ktx2", "_nra.ktx2"}) {
            const std::string path = dir + "/" + km.layer + map;
            ktx2::Image img;
            CHECK(loadTerrainTextureMap(path, std::strcmp(map, "_albedo.ktx2") == 0, 0, img, err), "%s", err.c_str());
        }
    }
    std::printf("  %d of %d kit materials textured\n", textured, kKitMatCount);
}

// The level places the kit: houses in the village, every structure on a floor at or above the ground under its
// footprint corners and no plinth bottom above the ground there.
void levelBuildings() {
    const Level& L = *level();
    int houses = 0, structures = 0, sunk = 0, hung = 0;
    for (const LevelBuilding& b : L.buildings()) {
        const LevelBuildingType& t = L.buildingTypes()[b.type];
        houses += t.spec.kind == BuildingKind::House;
        if (!buildingIsStructure(t.spec.kind)) continue;
        ++structures;
        const float c = std::cos(b.yaw), s = std::sin(b.yaw);
        for (const float sx : {-1.0f, 1.0f})
            for (const float sz : {-1.0f, 1.0f}) {
                const float lx = sx * t.footprintX, lz = sz * t.footprintZ;
                const float x = b.x + c * lx + s * lz, z = b.z - s * lx + c * lz;
                const bool low = b.y < L.heightAt(x, z) - 0.05f;
                if (low)
                    std::printf("  %s at %.1f %.1f: floor %.2f, ground %.2f at a footprint corner\n",
                                buildingKindName(t.spec.kind), double(b.x), double(b.z), double(b.y),
                                double(L.heightAt(x, z)));
                sunk += low;
                const bool hanging = L.heightAt(x, z) < b.y - buildingPlinthDepth(t.spec.kind) + 0.1f;
                if (hanging)
                    std::printf("  %s at %.1f %.1f: floor %.2f, ground %.2f below the plinth at a footprint corner\n",
                                buildingKindName(t.spec.kind), double(b.x), double(b.z), double(b.y),
                                double(L.heightAt(x, z)));
                hung += hanging;
            }
    }
    CHECK(L.buildings().size() > 80 && houses >= 18, "%zu buildings, %d houses", L.buildings().size(), houses);
    CHECK(sunk == 0, "ground above the floor at %d footprint corners of %d structures", sunk, structures);
    CHECK(hung == 0, "ground below the plinth at %d footprint corners of %d structures", hung, structures);
    std::printf("  %zu buildings (%d houses, %d structures) of %zu types\n", L.buildings().size(), houses,
                structures, L.buildingTypes().size());
}

// POI building groups stay on their sites (the placement may shift a group off a road, not across the map), no
// structure stands below the surface of a lake or the sea next to it, and neither ground cover nor cliff slabs reach
// into water.
void levelPoiBuildings() {
    const Level& L = *level();
    const auto poi = [&](const char* kind) -> const LevelPoi* {
        for (const LevelPoi& p : L.pois())
            if (p.kind == kind) return &p;
        return nullptr;
    };
    const auto kindOf = [&](const LevelBuilding& b) { return L.buildingTypes()[b.type].spec.kind; };
    // Castle ruins: walls, towers and the gate are one group laid out around the POI, the gate 14 m north of it.
    if (const LevelPoi* p = poi("ruins")) {
        int n = 0;
        float shift = 1e9f;
        for (const LevelBuilding& b : L.buildings()) {
            const BuildingKind k = kindOf(b);
            if (k != BuildingKind::RuinWall && k != BuildingKind::RuinTower && k != BuildingKind::RuinGate) continue;
            ++n;
            if (k == BuildingKind::RuinGate) shift = std::hypot(b.x - (p->x + 0.5f), b.z - (p->z + 14.0f));
        }
        std::printf("  castle ruins: %d parts, shifted %.1f m from their layout around the POI\n", n, double(shift));
        CHECK(n == 8 && shift < 6.0f, "%d ruin parts, shifted %.1f m from the POI layout", n, double(shift));
    }
    // Single landmarks stand on their marker.
    for (const auto& [kind, bk] : {std::pair{"lighthouse", BuildingKind::Lighthouse}, std::pair{"radio_tower", BuildingKind::RadioMast}}) {
        const LevelPoi* p = poi(kind);
        if (!p) continue;
        float best = 1e9f;
        for (const LevelBuilding& b : L.buildings())
            if (kindOf(b) == bk) best = std::min(best, std::hypot(b.x - p->x, b.z - p->z));
        CHECK(best < 6.0f, "%s %.1f m from its POI", kind, double(best));
    }
    // Structures by a lake or the sea: the floor is not below the water surface within 40 m.
    int below = 0, nearWater = 0;
    for (const LevelBuilding& b : L.buildings()) {
        if (!buildingIsStructure(kindOf(b))) continue;
        float top = -1e30f;
        for (float dz = -40.0f; dz <= 40.0f; dz += 1.0f)
            for (float dx = -40.0f; dx <= 40.0f; dx += 1.0f) {
                if (dx * dx + dz * dz > 1600.0f) continue;
                const WaterKind wk = L.waterKindAt(b.x + dx, b.z + dz);
                if (wk != WaterKind::Lake && wk != WaterKind::Sea) continue;
                if (const auto w = L.waterAt(b.x + dx, b.z + dz)) top = std::max(top, *w);
            }
        if (top < -1e29f) continue;
        ++nearWater;
        if (b.y < top - 0.3f) {
            std::printf("  %s at %.1f %.1f: floor %.2f below the water at %.2f\n", buildingKindName(kindOf(b)),
                        double(b.x), double(b.z), double(b.y), double(top));
            ++below;
        }
    }
    std::printf("  %d structures within 40 m of a lake or the sea\n", nearWater);
    CHECK(below == 0, "%d structures below the water next to them", below);
    // Ground cover stands with its whole footprint on dry ground (no fronds floating over a pool).
    int overWater = 0;
    for (const LevelScatter& c : L.scatter()) {
        const float r = L.models()[std::size_t(c.model)].radius * c.scale;
        for (int k = 0; k < 8; ++k)
            if (L.waterAt(c.x + r * std::cos(0.785398f * float(k)), c.z + r * std::sin(0.785398f * float(k)))) {
                ++overWater;
                break;
            }
    }
    std::printf("  %zu ground cover models, %d reaching over water\n", L.scatter().size(), overWater);
    CHECK(overWater == 0, "%d ground cover models reach over water", overWater);
    // Cliff slabs (pitched rock models on the walls) do not dip into a pool or show their back.
    int dipping = 0, slabs = 0, exposed = 0;
    for (const LevelRock& r : L.rocks()) {
        if (r.model < 0 || std::abs(r.pitch) < 0.05f) continue;
        ++slabs;
        const float reach = std::max(r.rx, r.ry);
        for (int k = 0; k < 8; ++k) {
            const float ax = r.x + reach * std::cos(0.785398f * float(k)), az = r.z + reach * std::sin(0.785398f * float(k));
            if (const auto w = L.waterAt(ax, az); w && r.y - r.ry < *w) {
                ++dipping;
                break;
            }
        }
        // The models are open shells: the back stays inside the wall everywhere over the face.
        const float cy = std::cos(r.yaw), sy = std::sin(r.yaw), cp = std::cos(r.pitch), sp = std::sin(r.pitch);
        const float slope = r.pitch + 0.5f * 3.14159265f;
        bool buried = true;
        for (int a = -2; a <= 2; ++a)
            for (int b = -2; b <= 2; ++b) {
                const float u = 0.5f * float(a) * r.rx, v = 0.5f * float(b) * r.ry;
                const float px = r.x + cy * u + sp * sy * v, py = r.y + cp * v, pz = r.z - sy * u + sp * cy * v;
                if ((py - L.heightAt(px, pz)) * std::cos(slope) > 0.9f * r.rz + 0.05f) buried = false;
            }
        if (!buried) ++exposed;
    }
    std::printf("  %d cliff slabs, %d dipping into water, %d with the back exposed\n", slabs, dipping, exposed);
    CHECK(exposed == 0, "%d cliff slabs show their open back", exposed);
    CHECK(dipping == 0, "%d cliff slabs dip into water", dipping);
}

// ------------------------------------------------------------------------------------------------- baked levels
const std::string kCircuit = TRAILFORGE_SOURCE_DIR "/assets/levels/suncoast_raceway/level.json";

std::shared_ptr<const Level> loadOrExit(const std::string& path) {
    std::string err;
    auto lv = Level::load(path, &err);
    if (!lv) {
        std::printf("  cannot load %s: %s\n", path.c_str(), err.c_str());
        std::exit(1);
    }
    return lv;
}

const std::shared_ptr<const Level>& circuitLevel() {
    static const std::shared_ptr<const Level> l = [] {
        auto lv = loadOrExit(kCircuit);
        std::printf("  bake %.0f ms: %d^2 samples, %.1f..%.1f m, %zu trees, %zu rocks, %zu buildings, %zu scatter\n",
                    lv->bakeStats().totalMs, lv->samples(), lv->bakeStats().minHeight, lv->bakeStats().maxHeight,
                    lv->bakeStats().trees, lv->bakeStats().rocks, lv->bakeStats().buildings, lv->bakeStats().scatter);
        return lv;
    }();
    return l;
}

// Differences between two bakes (empty: identical heights, surfaces, water, roads, placements, gates and spawn).
std::vector<std::string> bakeDifferences(const Level& a, const Level& b) {
    std::vector<std::string> d;
    auto note = [&](bool same, const char* what) {
        if (!same) d.push_back(what);
    };
    note(a.samples() == b.samples() && a.heights() == b.heights(), "heights");
    note(a.cellSurfaces() == b.cellSurfaces() && a.surfaceClasses() == b.surfaceClasses(), "surfaces");
    note(a.waterSurfaces() == b.waterSurfaces(), "water");
    note(a.maskTexels() == b.maskTexels() && a.splatTexels() == b.splatTexels(), "ground masks");
    bool roads = a.roads().size() == b.roads().size();
    for (std::size_t r = 0; roads && r < a.roads().size(); ++r) {
        const LevelRoad &ra = a.roads()[r], &rb = b.roads()[r];
        roads = ra.name == rb.name && ra.width == rb.width && ra.samples.size() == rb.samples.size();
        for (std::size_t k = 0; roads && k < ra.samples.size(); ++k)
            roads = ra.samples[k].x == rb.samples[k].x && ra.samples[k].z == rb.samples[k].z &&
                    ra.samples[k].y == rb.samples[k].y && ra.samples[k].bank == rb.samples[k].bank;
    }
    note(roads, "roads");
    bool trees = a.trees().size() == b.trees().size();
    for (std::size_t k = 0; trees && k < a.trees().size(); ++k) {
        const LevelTree &ta = a.trees()[k], &tb = b.trees()[k];
        trees = ta.x == tb.x && ta.y == tb.y && ta.z == tb.z && ta.yaw == tb.yaw && ta.scale == tb.scale && ta.model == tb.model;
    }
    note(trees, "trees");
    bool rocks = a.rocks().size() == b.rocks().size();
    for (std::size_t k = 0; rocks && k < a.rocks().size(); ++k) {
        const LevelRock &ra = a.rocks()[k], &rb = b.rocks()[k];
        rocks = ra.x == rb.x && ra.y == rb.y && ra.z == rb.z && ra.yaw == rb.yaw && ra.sx == rb.sx && ra.model == rb.model;
    }
    note(rocks, "rocks");
    bool bld = a.buildings().size() == b.buildings().size();
    for (std::size_t k = 0; bld && k < a.buildings().size(); ++k) {
        const LevelBuilding &ba = a.buildings()[k], &bb = b.buildings()[k];
        bld = ba.x == bb.x && ba.y == bb.y && ba.z == bb.z && ba.yaw == bb.yaw &&
              a.buildingTypes()[ba.type].spec == b.buildingTypes()[bb.type].spec;
    }
    note(bld, "buildings");
    note(a.props().geometry.triangleCount() == b.props().geometry.triangleCount(), "prop collision");
    note(a.scatter().size() == b.scatter().size(), "ground cover");
    bool gates = a.circuit().gates.size() == b.circuit().gates.size() && a.circuit().length == b.circuit().length;
    for (std::size_t k = 0; gates && k < a.circuit().gates.size(); ++k) {
        const LevelGate &ga = a.circuit().gates[k], &gb = b.circuit().gates[k];
        gates = ga.x == gb.x && ga.z == gb.z && ga.dx == gb.dx && ga.dz == gb.dz && ga.halfWidth == gb.halfWidth;
    }
    note(gates, "checkpoints");
    const TrackFeature sa = a.spawn(), sb = b.spawn();
    note(sa.position == sb.position && sa.yaw == sb.yaw, "spawn");
    return d;
}

// The shipped Blender-authored level: no layout issues, the model library, the circuit and its gates, the surface
// classes, the heights exactly the 16-bit heightmap, the physics tiles on the same grid, a deterministic bake.
void levelBakedLayout() {
    const Level& lv = *circuitLevel();
    CHECK(lv.baked() && lv.asphaltLayer(), "baked %d, asphaltLayer %d", int(lv.baked()), int(lv.asphaltLayer()));
    for (const std::string& issue : lv.layoutIssues()) std::printf("    layout: %s\n", issue.c_str());
    CHECK(lv.layoutIssues().empty(), "%zu layout issues", lv.layoutIssues().size());
    CHECK(lv.modelError().empty() && !lv.models().empty(), "model library: %s", lv.modelError().c_str());
    const LevelCircuit& c = lv.circuit();
    std::printf("  circuit '%s' %.0f m, %zu gates; %zu roads, %zu POIs\n", c.road.c_str(), double(c.length), c.gates.size(),
                lv.roads().size(), lv.pois().size());
    // Suncoast Raceway full course: 1979 m as mapped in OpenStreetMap, the smoothed center line a little longer.
    CHECK(c.length > 1950.0f && c.length < 2050.0f, "circuit %.0f m", double(c.length));
    CHECK(c.gates.size() >= 6, "%zu gates", c.gates.size());
    CHECK(lv.trees().size() > 2000 && lv.rocks().size() > 50 && lv.buildings().size() > 100, "trees %zu rocks %zu buildings %zu",
          lv.trees().size(), lv.rocks().size(), lv.buildings().size());
    CHECK(lv.attractRoute().points.size() > 500, "attract route %zu points", lv.attractRoute().points.size());
    int count[int(LevelSurfaceClass::Count)] = {};
    for (std::uint8_t k : lv.surfaceClasses()) ++count[k];
    std::printf("  cells:");
    for (int k = 0; k < int(LevelSurfaceClass::Count); ++k)
        std::printf(" %s %d", levelSurfaceClassName(LevelSurfaceClass(k)), count[k]);
    std::printf("\n");
    for (LevelSurfaceClass k : {LevelSurfaceClass::Asphalt, LevelSurfaceClass::Gravel, LevelSurfaceClass::Dirt,
                                LevelSurfaceClass::Grass})
        CHECK(count[int(k)] > 500, "%d %s cells", count[int(k)], levelSurfaceClassName(k));
    CHECK(count[int(LevelSurfaceClass::Mud)] == 0, "%d mud cells on a race circuit", count[int(LevelSurfaceClass::Mud)]);
    // Every circuit sample on asphalt.
    int offTrack = 0;
    for (const LevelRoad& r : lv.roads())
        if (r.name == c.road)
            for (const RoadSample& p : r.samples)
                for (float u : {-0.4f, 0.0f, 0.4f})
                {
                    const float x = p.x + p.dz * u * r.width, z = p.z - p.dx * u * r.width;
                    if (lv.surfaceAt(x, z) == SurfaceId::Asphalt) continue;
                    if (++offTrack <= 5) std::printf("    off asphalt at (%.1f, %.1f) s %.0f u %.2f\n", double(x), double(z), double(p.s), double(u));
                }
    CHECK(offTrack == 0, "%d circuit points not on asphalt", offTrack);
    // Painted lines and curbs.
    int paint = 0, curbs = 0;
    for (PropStyle st : lv.props().styles) {
        paint += st == PropStyle::LinePaint;
        curbs += st == PropStyle::CurbRed || st == PropStyle::CurbWhite;
    }
    std::printf("  markings: %d paint triangles, %d curb triangles\n", paint, curbs);
    CHECK(paint > 5000 && curbs > 5000, "paint %d curbs %d triangles", paint, curbs);
    // Asphalt is the Asphalt surface, gravel Gravel, dirt packed dirt, mud mud.
    int mism = 0;
    const int cn = lv.samples() - 1;
    for (int j = 0; j < cn; ++j)
        for (int i = 0; i < cn; ++i) {
            const auto k = LevelSurfaceClass(lv.surfaceClasses()[std::size_t(j) * std::size_t(cn) + std::size_t(i)]);
            const SurfaceId s = lv.cellSurface(i, j);
            const SurfaceId want = k == LevelSurfaceClass::Asphalt ? SurfaceId::Asphalt
                                   : k == LevelSurfaceClass::Gravel ? SurfaceId::Gravel
                                   : k == LevelSurfaceClass::Dirt ? SurfaceId::PackedDirt
                                   : k == LevelSurfaceClass::Mud ? SurfaceId::Mud : k == LevelSurfaceClass::Sand ? SurfaceId::Sand
                                   : k == LevelSurfaceClass::Rock ? SurfaceId::Rock : k == LevelSurfaceClass::Snow ? SurfaceId::Snow
                                                                                                           : SurfaceId::Grass;
            mism += s != want;
        }
    CHECK(mism == 0, "%d cells with a physics surface not of their class", mism);
    // Heights: exactly the decoded heightmap.
    PgmImage hm;
    std::string err;
    CHECK(readPgm(TRAILFORGE_SOURCE_DIR "/assets/levels/suncoast_raceway/heightmap.pgm", hm, err), "%s", err.c_str());
    std::ifstream lf(TRAILFORGE_SOURCE_DIR "/assets/levels/suncoast_raceway/layout.json");
    std::stringstream ls;
    ls << lf.rdbuf();
    json::Value lay;
    CHECK(json::parse(ls.str(), lay, &err), "layout: %s", err.c_str());
    const float lo = float(lay["grid"]["heightMin"].numberOr(0)), hi = float(lay["grid"]["heightMax"].numberOr(0));
    int off = 0;
    for (std::size_t k = 0; k < hm.data.size() && k < lv.heights().size(); ++k)
        off += lv.heights()[k] != lo + float(hm.data[k]) * ((hi - lo) / 65535.0f);
    CHECK(hm.data.size() == lv.heights().size() && off == 0, "%d of %zu heights differ from the heightmap", off, hm.data.size());
    int wet = 0;
    for (const LevelRoad& r : lv.roads())
        for (const RoadSample& p : r.samples) wet += lv.waterAt(p.x, p.z).has_value();
    CHECK(wet == 0, "%d road samples under water", wet);
    // Physics tiles: rays onto the heightfield hit the baked surface.
    {
        JoltRuntime runtime;
        PhysicsWorld world;
        SurfaceTable surfaces;
        TerrainTiles::Settings ts;
        ts.async = false;
        TerrainTiles tiles(world, surfaces, circuitLevel(), ts);
        float worst = 0.0f;
        int rays = 0;
        Lcg rng{7u};
        for (int k = 0; k < 400; ++k) {
            const float x = -470.0f + 940.0f * rng.next(), z = -470.0f + 940.0f * rng.next();
            tiles.update(x, z);
            const JPH::RRayCast ray(JPH::RVec3(x, 400.0f, z), JPH::Vec3(0, -800.0f, 0));
            JPH::RayCastResult hit;
            if (!world.system().GetNarrowPhaseQuery().CastRay(ray, hit)) continue;
            const float y = 400.0f - 800.0f * hit.mFraction;
            if (y < lv.heightAt(x, z) - 0.05f) continue;
            if (y > lv.heightAt(x, z) + 0.05f) continue; // a prop or rock above the ground
            worst = std::max(worst, std::abs(y - lv.heightAt(x, z)));
            ++rays;
        }
        CHECK(rays > 300 && worst < 0.02f, "%d rays, worst %.3f m", rays, double(worst));
    }
    auto again = loadOrExit(kCircuit);
    const std::vector<std::string> diff = bakeDifferences(lv, *again);
    for (const std::string& what : diff) std::printf("    second bake differs: %s\n", what.c_str());
    CHECK(diff.empty(), "%zu differences between two bakes", diff.size());
}

// Gate crossing: start, checkpoints in order, an interpolated lap time, a missed checkpoint voids the lap.
void lapTimerGates() {
    LapTimer t;
    // Square loop 100 m: start at (0, 0) heading +z, gates on each side.
    t.setGates({{0, 0, 0, 1, 6}, {50, 50, 1, 0, 6}, {0, 100, 0, -1, 6}, {-50, 50, -1, 0, 6}});
    CHECK(t.update(0, -1, 0, 1, 0.5, 1.0) == LapTimer::Event::Started, "start");
    CHECK(t.running() && t.nextGate() == 1 && t.lastGate() == 0, "next %d", t.nextGate());
    CHECK(t.update(49, 50, 51, 50, 10.5, 11.0) == LapTimer::Event::Checkpoint, "checkpoint 1");
    CHECK(t.update(0, 99, 0, 101, 20.5, 21.0) == LapTimer::Event::None, "a gate crossed backwards does not count");
    CHECK(t.update(1, 101, -1, 99, 20.5, 21.0) == LapTimer::Event::Checkpoint, "checkpoint 2");
    CHECK(t.update(-20, 50, -20, 50, 24.5, 25.0) == LapTimer::Event::None, "no move");
    CHECK(t.update(-49, 50, -51, 50, 30.5, 31.0) == LapTimer::Event::Checkpoint && t.checkpointsPassed() == 3, "checkpoint 3");
    // Finish at t = 40.875 s (crossing at 3/4 of the tick 40.5 .. 41.0 s); the lap started at 0.75 s.
    CHECK(t.update(0, -3, 0, 1, 40.5, 41.0) == LapTimer::Event::Lap, "lap");
    CHECK(std::abs(t.lastLap() - 40.125) < 1e-6 && t.bestLap() == t.lastLap() && t.laps() == 1, "lap %.4f best %.4f",
          t.lastLap(), t.bestLap());
    CHECK(t.update(20, 0, 20, 2, 44.5, 45.0) == LapTimer::Event::None, "outside the half width");
    CHECK(t.update(49, 50, 51, 50, 49.5, 50.0) == LapTimer::Event::Checkpoint, "checkpoint 1, lap 2");
    CHECK(t.update(0, -1, 0, 1, 59.5, 60.0) == LapTimer::Event::Invalid && t.laps() == 1, "short cut voids the lap");
    CHECK(t.running() && t.nextGate() == 1, "a new lap started at the line");
    t.reset();
    CHECK(!t.running() && t.bestLap() > 0.0, "reset keeps the best lap");
}

// The racing surface as the tires meet it (rays onto the terrain tiles and the props): along the circuit every
// 0.5 m, across it every 0.5 m up to 0.6 m inside the edges (the curbs start beyond), the height deviates from the
// chord between the points 1 m before and after by a few millimetres at most (painted lines are 4 mm).
void levelBakedTrackSmooth() {
    const Level& lv = *circuitLevel();
    const LevelRoad* road = nullptr;
    for (const LevelRoad& r : lv.roads())
        if (r.name == lv.circuit().road) road = &r;
    CHECK(road && road->samples.size() > 100, "circuit road");
    if (!road) return;
    JoltRuntime runtime;
    PhysicsWorld world;
    SurfaceTable surfaces;
    TerrainTiles::Settings ts;
    ts.async = false;
    TerrainTiles tiles(world, surfaces, circuitLevel(), ts);
    TestTrack props(world, surfaces, lv);
    world.system().OptimizeBroadPhase();
    auto heightAt = [&](float x, float z) {
        // From 1 m above the ground: under the start gantry and the bridges of other levels.
        const float top = lv.heightAt(x, z) + 1.0f;
        const JPH::RRayCast ray(JPH::RVec3(x, top, z), JPH::Vec3(0, -2.0f, 0));
        JPH::RayCastResult hit;
        return world.system().GetNarrowPhaseQuery().CastRay(ray, hit) ? top - 2.0f * hit.mFraction : -1000.0f;
    };
    const float hw = 0.5f * road->width - 0.6f;
    const int lanes = int(2.0f * hw / 0.5f) + 1;
    float worst = 0.0f, worstX = 0.0f, worstZ = 0.0f, grade = 0.0f;
    int points = 0;
    const std::vector<RoadSample>& rs = road->samples;
    for (std::size_t k = 0; k < rs.size(); ++k) {
        const RoadSample &a = rs[k], &b = rs[(k + 1) % rs.size()];
        tiles.update(a.x, a.z);
        const float len = std::hypot(b.x - a.x, b.z - a.z);
        for (float t = 0.0f; t < len; t += 0.5f) {
            const float f = t / len, x = a.x + (b.x - a.x) * f, z = a.z + (b.z - a.z) * f;
            float dx = a.dx + (b.dx - a.dx) * f, dz = a.dz + (b.dz - a.dz) * f;
            const float n = std::hypot(dx, dz);
            dx /= n;
            dz /= n;
            for (int l = 0; l < lanes; ++l) {
                const float u = -hw + 0.5f * float(l), px = x + dz * u, pz = z - dx * u;
                const float h0 = heightAt(px - dx, pz - dz), h1 = heightAt(px, pz), h2 = heightAt(px + dx, pz + dz);
                const float dev = std::abs(h1 - 0.5f * (h0 + h2));
                grade = std::max(grade, std::abs(h2 - h0) * 0.5f);
                if (dev > worst) worst = dev, worstX = px, worstZ = pz;
                ++points;
            }
        }
    }
    std::printf("  %d surface points: largest bump %.1f mm at (%.1f, %.1f), max grade %.2f %%\n", points,
                double(worst) * 1000.0, double(worstX), double(worstZ), double(grade) * 100.0);
    CHECK(points > 50000 && worst < 0.006f, "%d points, bump %.1f mm", points, double(worst) * 1000.0);
}

// The autopilot drives one lap of the circuit: the timer starts at the line, passes every checkpoint and sector
// line in order and counts the lap with its sector times; R then puts the vehicle 4 m past the last gate, facing
// along it.
void levelBakedLap(const char* id) {
    SimulationSettings s;
    s.level = circuitLevel();
    s.vehicle = vehicle(id);
    s.terrain.async = false;
    Simulation sim(s);
    const LapTimer& lt = sim.lapTimer();
    CHECK(lt.active() && !lt.running(), "timer active %d running %d", int(lt.active()), int(lt.running()));
    sim.setAutopilot(AutopilotMode::Drive);
    double t = 0.0, maxKmh = 0.0;
    int lastGate = -1, order = 0;
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<std::string> trace;
    while (t < 600.0 && lt.laps() == 0 && sim.autopilotResets() == 0) {
        if (int(t * 120.0) % 60 == 0) {
            const Autopilot& ap = sim.autopilot();
            const VehicleTelemetry& tel = sim.vehicle().telemetry();
            const JPH::Vec3 q = sim.currentSnapshot().body.position;
            char buf[256];
            std::snprintf(buf, sizeof(buf), "    t %.1f s %.0f (%.1f %.1f %.1f) v %.1f tgt %.1f lat %.2f steer %+.2f up %.2f wheels %d",
                          t, double(ap.progress()), double(q.GetX()), double(q.GetY()), double(q.GetZ()),
                          double(tel.forwardSpeed), double(ap.targetSpeed()), double(ap.lateralError()),
                          double(tel.command.steer), double(tel.upDot), wheelsInContact(sim));
            trace.push_back(buf);
        }
        sim.tick();
        t += sim.fixedStep();
        maxKmh = std::max(maxKmh, double(sim.vehicle().telemetry().forwardSpeed) * 3.6);
        if (lt.running() && lt.lastGate() != lastGate) {
            order += lt.lastGate() == (lastGate + 1) % int(lt.gates().size()) ? 0 : 1;
            lastGate = lt.lastGate();
        }
    }
    if (sim.autopilotResets() > 0 || std::getenv("TF_AP_DEBUG"))
        for (std::size_t k = trace.size() > 30 ? trace.size() - 30 : 0; k < trace.size(); ++k) std::printf("%s\n", trace[k].c_str());
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("  %s autopilot lap: %.3f s (sim %.1f s, max %.1f km/h), %d laps, resets %d, recoveries %d, %.1f s wall\n",
                id, lt.lastLap(), t, maxKmh, lt.laps(), sim.autopilotResets(), sim.recoveries(), wall);
    double sectors = 0.0;
    for (int k = 0; k < lt.sectorCount(); ++k) {
        std::printf("    S%d %.3f s\n", k + 1, lt.lastSectorTime(k));
        sectors += lt.lastSectorTime(k);
    }
    CHECK(lt.sectorCount() == 3 && std::abs(sectors - lt.lastLap()) < 1e-6, "%d sectors, sum %.4f lap %.4f",
          lt.sectorCount(), sectors, lt.lastLap());
    CHECK(lt.laps() == 1 && lt.lastLap() > 60.0 && order == 0, "laps %d, lap %.1f s, %d out of order", lt.laps(), lt.lastLap(), order);
    CHECK(sim.autopilotResets() == 0 && sim.recoveries() == 0 && !sim.hadPhysicsError(), "resets %d (%s), recoveries %d",
          sim.autopilotResets(), sim.autopilotResetReason(), sim.recoveries());
    // Drive on past the next gate, then reset: back to that gate, the lap timer keeps running.
    const int target = lt.nextGate();
    while (t < 700.0 && lt.lastGate() != target) {
        sim.tick();
        t += sim.fixedStep();
    }
    runFor(sim, 3.0);
    sim.requestReset();
    runFor(sim, 0.05);
    const LevelGate& g = lt.gates()[std::size_t(target)];
    const JPH::Vec3 p = sim.currentSnapshot().body.position;
    const float along = LapTimer::along(g, p.GetX(), p.GetZ()), across = LapTimer::across(g, p.GetX(), p.GetZ());
    CHECK(lt.running() && lt.lastGate() == target && along > 2.0f && along < 6.0f && std::abs(across) < 1.0f,
          "reset to gate %d: running %d, last %d, %.1f m along, %.1f m across", target, int(lt.running()), lt.lastGate(),
          double(along), double(across));
    // Handing the wheel from the autopilot to the player stops the lap (no lap is driven by both).
    sim.setAutopilot(AutopilotMode::Off);
    CHECK(!lt.running() && lt.laps() == 1, "after autopilot off: running %d, laps %d", int(lt.running()), lt.laps());
}

// Round trip (CTest level_blender_roundtrip): the bake of a level.json with the original files and with the files
// exported again from Blender are identical.
int levelBakedCompare(const char* a, const char* b) {
    auto la = loadOrExit(a), lb = loadOrExit(b);
    const std::vector<std::string> diff = bakeDifferences(*la, *lb);
    for (const std::string& what : diff) std::printf("  differs: %s\n", what.c_str());
    std::printf("  %s vs %s: %s (%zu trees, %zu rocks, %zu buildings, %zu layout issues)\n", a, b,
                diff.empty() ? "identical bakes" : "DIFFERENT", lb->trees().size(), lb->rocks().size(),
                lb->buildings().size(), lb->layoutIssues().size());
    return diff.empty() && lb->layoutIssues().empty() ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
    if (argc == 4 && std::strcmp(argv[1], "level_baked_compare") == 0) return levelBakedCompare(argv[2], argv[3]);
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
        {"level_slopes", levelSlopes},
        {"autopilot_loops", autopilotLoops},
        {"terrain_textures_ktx2", terrainTexturesKtx2},
        {"terrain_textures_assets", terrainTexturesAssets},
        {"building_kit_parts", buildingKitParts},
        {"building_kit_materials", buildingKitMaterials},
        {"level_buildings", levelBuildings},
        {"level_poi_buildings", levelPoiBuildings},
        {"level_baked_layout", levelBakedLayout},
        {"lap_timer_gates", lapTimerGates},
        {"level_baked_track_smooth", levelBakedTrackSmooth},
        {"level_baked_lap", [] { levelBakedLap("trailforge_suv"); }},
        {"level_baked_lap_ev", [] { levelBakedLap("trailforge_ev"); }},
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
