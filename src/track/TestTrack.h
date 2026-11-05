#pragma once

#include "track/DriveRoute.h"
#include "track/SurfaceProperties.h"

#include <Jolt/Jolt.h>
#include <Jolt/Math/Float3.h>
#include <Jolt/Physics/Body/BodyID.h>

#include <string>
#include <vector>

namespace quench {
class PhysicsWorld;
}

namespace tf {

class Level;

// Procedural static geometry: the same triangles go
// to rendering (mesh) and to the Jolt MeshShape (collision), so they match by construction.
// Render-only finish of a triangle; the physics surface stays TrackGeometry::surfaces (paint never changes grip).
enum class Paint : std::uint8_t {
    None = 0,     // the surface's own procedural material
    Asphalt,
    Line,         // white road paint
    CurbRed,
    CurbWhite,
    Gravel,
    Tyre,
    BarrierRed,
    BarrierWhite,
};

// Bump profiles along the driving direction. Round: h * (1 - cos(2 pi u)) / 2 over the base (a hump with no edge, the
// steepest face at mid-height). Step: a block with 60 degree faces and a flat top (base - 2 * h / tan 60 long), like a
// squared-off rock or a kerb-stone ledge.
enum class BumpProfile : std::uint8_t { Round, Step };

// Triangles are stored non-indexed (3 vertices per triangle), wound CCW
// when viewed from outside (Jolt defines the front face the same way).
struct TrackGeometry {
    std::vector<JPH::Float3> positions; // 3 per triangle, meters, world coordinates
    std::vector<SurfaceId> surfaces;    // 1 per triangle
    std::vector<Paint> paints;          // empty or shorter than the triangles: the missing ones are Paint::None

    std::size_t triangleCount() const { return surfaces.size(); }
    Paint paintOf(std::size_t t) const { return t < paints.size() ? paints[t] : Paint::None; }
    // Paints triangles [firstTriangle, triangleCount()).
    void paintFrom(std::size_t firstTriangle, Paint p);

    void addTriangle(JPH::Vec3Arg a, JPH::Vec3Arg b, JPH::Vec3Arg c, SurfaceId s);
    // Quad a-b-c-d, CCW when viewed from outside.
    void addQuad(JPH::Vec3Arg a, JPH::Vec3Arg b, JPH::Vec3Arg c, JPH::Vec3Arg d, SurfaceId s);
    // Box without a bottom face; rotation around Y (rad).
    void addBox(JPH::Vec3Arg center, JPH::Vec3Arg halfExtents, float yaw, SurfaceId s);
    // Irregular rock: a sphere with deterministic radius noise, scaled per axis.
    void addRock(JPH::Vec3Arg center, JPH::Vec3Arg radii, float yaw, std::uint32_t seed, SurfaceId s);
    // Bump across the driving direction (+Z) on flat ground at height y0, over x0..x1, closed at both ends; see
    // BumpProfile for the shapes. length = base length along Z, m.
    void addBump(float x0, float x1, float zCenter, float length, float height, BumpProfile profile, float y0,
                 SurfaceId s);
};

enum class TrackLayout {
    ProvingGround,    // full proving ground, ~100x100 m
    Flat,             // flat pad (for tests)
    FlatWithWheelPit, // flat pad with a pit under the front left wheel (wheel lift test)
    SurfaceLanes,     // flat pad of parallel 12 m lanes: concrete, packed dirt, mud, grass, rock, low grip, sand
    SurfaceRamps,     // 15 and 20 degree ramps per surface, 10 m lanes, 22 m long (climb measurements)
    SlopeWithWheelPit, // 15 degree concrete slope, hole under the front left wheel (differential tests)
    Level,            // open-world level: props of a baked Level (bridges, blockouts); terrain in TerrainTiles
    RoadLanes,        // flat pad of 12 m lanes like SurfaceLanes: asphalt, concrete, gravel (features 0, 1, 2)
};

struct TrackFeature {
    std::string name;
    JPH::Vec3 position; // vehicle spawn point (on the ground)
    float yaw = 0.0f;   // rad, 0 = vehicle faces +Z
};

// Footprint of a rock (for render-side masks: no grass on rocks).
struct RockFootprint {
    float x = 0.0f, z = 0.0f, radius = 0.0f;
};

// Walled proving ground area (heightfield extent), meters.
struct TrackBounds {
    float xMin, xMax, zMin, zMax;
};

// Showcase puddles: the terrain dips below one common water level; the water surface itself is render-only.
struct WaterInfo {
    float level;          // m, world Y of the water surface
    float x0, z0, x1, z1; // rectangle that contains every puddle
};

class TestTrack {
public:
    TestTrack(quench::PhysicsWorld& world, const SurfaceTable& surfaces, TrackLayout layout);
    // Static props of an open-world level (the heightfield tiles are streamed separately); features: the level spawn
    // and its points of interest.
    TestTrack(quench::PhysicsWorld& world, const SurfaceTable& surfaces, const Level& level);
    ~TestTrack();
    TestTrack(const TestTrack&) = delete;
    TestTrack& operator=(const TestTrack&) = delete;

    const TrackGeometry& geometry() const { return m_geometry; }
    const std::vector<TrackFeature>& features() const { return m_features; }
    const TrackFeature& spawn() const { return m_features.front(); }
    JPH::BodyID bodyId() const { return m_body; }
    TrackLayout layout() const { return m_layout; }

    const std::vector<RockFootprint>& rocks() const { return m_rocks; }

    // Procedural terrain height (without rocks/steps), meters; the mini handling track area has its own ground.
    static float provingGroundHeight(float x, float z);
    // Ground surface of the heightfield at (x, z) (rocks and steps are separate).
    static SurfaceId groundSurfaceAt(float x, float z);
    // 0..1: how "muddy" the showcase ground is at (x, z); Mud surface where > 0.45 or inside a puddle.
    static float showcaseMudWeight(float x, float z);
    static TrackBounds bounds();
    // The proving ground plus the mini handling track area south of it (map extent).
    static TrackBounds worldBounds();
    static WaterInfo water();
    // Wheat test patch in the west meadow: render-only stalks, the ground under them is the Grass surface.
    static TrackBounds wheatPatch();
    // Menu attract loop of the proving ground: flat ground around lane A (start pad, between the lanes, north end).
    static DriveRoute attractRoute();

private:
    void buildProvingGround();
    void buildFlat(bool withPit);
    void buildSurfaceLanes();
    void buildSurfaceRamps();
    void buildRoadLanes();
    void buildSlopeWithWheelPit();
    void createBody(const SurfaceTable& surfaces);

    quench::PhysicsWorld& m_world;
    TrackLayout m_layout;
    TrackGeometry m_geometry;
    std::vector<TrackFeature> m_features;
    std::vector<RockFootprint> m_rocks;
    JPH::BodyID m_body;
};

} // namespace tf
