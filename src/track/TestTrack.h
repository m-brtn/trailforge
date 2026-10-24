#pragma once

#include "track/SurfaceProperties.h"

#include <Jolt/Jolt.h>
#include <Jolt/Math/Float3.h>
#include <Jolt/Physics/Body/BodyID.h>

#include <string>
#include <vector>

namespace tf {

class PhysicsWorld;

// Procedural static geometry: the same triangles go
// to rendering (mesh) and to the Jolt MeshShape (collision), so they match by construction.
// Triangles are stored non-indexed (3 vertices per triangle), wound CCW
// when viewed from outside (Jolt defines the front face the same way).
struct TrackGeometry {
    std::vector<JPH::Float3> positions; // 3 per triangle, meters, world coordinates
    std::vector<SurfaceId> surfaces;    // 1 per triangle

    std::size_t triangleCount() const { return surfaces.size(); }

    void addTriangle(JPH::Vec3Arg a, JPH::Vec3Arg b, JPH::Vec3Arg c, SurfaceId s);
    // Quad a-b-c-d, CCW when viewed from outside.
    void addQuad(JPH::Vec3Arg a, JPH::Vec3Arg b, JPH::Vec3Arg c, JPH::Vec3Arg d, SurfaceId s);
    // Box without a bottom face; rotation around Y (rad).
    void addBox(JPH::Vec3Arg center, JPH::Vec3Arg halfExtents, float yaw, SurfaceId s);
    // Irregular rock: a sphere with deterministic radius noise, scaled per axis.
    void addRock(JPH::Vec3Arg center, JPH::Vec3Arg radii, float yaw, std::uint32_t seed, SurfaceId s);
};

enum class TrackLayout {
    ProvingGround,    // full proving ground, ~100x100 m
    Flat,             // flat pad (for tests)
    FlatWithWheelPit, // flat pad with a pit under the front left wheel (wheel lift test)
    SurfaceLanes,     // flat pad of parallel 12 m lanes: concrete, packed dirt, mud, grass, rock, low grip
    SurfaceRamps,     // 15 and 20 degree ramps per surface, 10 m lanes, 22 m long (climb measurements)
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
    TestTrack(PhysicsWorld& world, const SurfaceTable& surfaces, TrackLayout layout);
    ~TestTrack();
    TestTrack(const TestTrack&) = delete;
    TestTrack& operator=(const TestTrack&) = delete;

    const TrackGeometry& geometry() const { return m_geometry; }
    const std::vector<TrackFeature>& features() const { return m_features; }
    const TrackFeature& spawn() const { return m_features.front(); }
    JPH::BodyID bodyId() const { return m_body; }
    TrackLayout layout() const { return m_layout; }

    const std::vector<RockFootprint>& rocks() const { return m_rocks; }

    // Procedural terrain height (without rocks/steps), meters.
    static float provingGroundHeight(float x, float z);
    // Ground surface of the heightfield at (x, z) (rocks and steps are separate).
    static SurfaceId groundSurfaceAt(float x, float z);
    // 0..1: how "muddy" the showcase ground is at (x, z); Mud surface where > 0.45 or inside a puddle.
    static float showcaseMudWeight(float x, float z);
    static TrackBounds bounds();
    static WaterInfo water();

private:
    void buildProvingGround();
    void buildFlat(bool withPit);
    void buildSurfaceLanes();
    void buildSurfaceRamps();

    PhysicsWorld& m_world;
    TrackLayout m_layout;
    TrackGeometry m_geometry;
    std::vector<TrackFeature> m_features;
    std::vector<RockFootprint> m_rocks;
    JPH::BodyID m_body;
};

} // namespace tf
