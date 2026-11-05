#pragma once

#include "track/Level.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace quench {
class PhysicsWorld;
}

namespace tf {

class SurfaceTable;

struct TerrainStreamingStats {
    int loaded = 0;          // tiles with bodies in the physics world
    int queued = 0;          // requested from the worker, not added yet
    int builtAsync = 0;      // tiles built on the worker thread (total)
    int builtSync = 0;       // tiles built on the calling thread because they were needed at once (total)
    int unloaded = 0;        // tiles removed again (total)
    double lastBuildMs = 0.0, maxBuildMs = 0.0; // shape build time of one tile (either thread)
    double maxSyncStallMs = 0.0;                // longest synchronous build in one update()
    double lastUpdateMs = 0.0, maxUpdateMs = 0.0; // main-thread cost of update() (adds, removes, sync builds)
};

// Physics of an open-world level in square tiles: one HeightFieldShape per tile (the baked height grid, per-cell
// surface materials) plus one StaticCompoundShape with the tree trunks and boulders whose centers are inside the tile
// (model rocks, logs and cliffs: the model's shared collision MeshShape, scaled and placed per instance). Shapes are
// built on a worker thread; bodies are added and removed only by update(), which the simulation calls between physics
// steps. Tiles near the focus that are not ready are built synchronously (counted as stalls), so the vehicle never
// drives onto a missing tile.
class TerrainTiles {
public:
    static constexpr int kTileCells = 255; // 256 x 256 samples per tile, 1 m cells; neighbors share the edge samples

    struct Settings {
        float loadRadius = 420.0f;     // m from the focus to the tile rectangle: request asynchronously
        float keepRadius = 620.0f;     // m: tiles farther than this are removed (hysteresis against reloading)
        float requiredRadius = 110.0f; // m: tiles this close must be present after update() (sync build if needed)
        bool async = true;             // false: build everything on the calling thread (deterministic tests)
    };

    TerrainTiles(quench::PhysicsWorld& world, const SurfaceTable& surfaces, std::shared_ptr<const Level> level,
                 const Settings& settings);
    TerrainTiles(quench::PhysicsWorld& world, const SurfaceTable& surfaces, std::shared_ptr<const Level> level)
        : TerrainTiles(world, surfaces, std::move(level), Settings{}) {}
    ~TerrainTiles();
    TerrainTiles(const TerrainTiles&) = delete;
    TerrainTiles& operator=(const TerrainTiles&) = delete;

    // Main thread, never during a physics step.
    void update(float focusX, float focusZ);
    // Synchronously loads every tile within radius of (x, z) (teleports, spawn).
    void ensureLoaded(float x, float z, float radius);

    int tilesPerSide() const { return m_side; }
    float tileOrigin() const { return m_x0; } // world x / z of the first tile's first sample
    float tileSize() const { return float(kTileCells) * m_level->cell(); }
    int tileIndexOf(float coord) const; // -1 outside
    bool isLoaded(int tx, int tz) const;
    JPH::BodyID terrainBody(int tx, int tz) const { return m_tiles[std::size_t(tz) * std::size_t(m_side) + std::size_t(tx)].terrain; }
    const Level& level() const { return *m_level; }
    const TerrainStreamingStats& stats() const { return m_stats; }
    Settings& settings() { return m_settings; }
    std::size_t trunkCount() const { return m_trunkCount; }

    // Distance (m) from a point to tile (tx, tz)'s rectangle, 0 inside.
    float distanceToTile(int tx, int tz, float x, float z) const;

private:
    struct Built {
        int index = -1;
        JPH::Ref<JPH::Shape> terrain, props;
        double ms = 0.0;
    };
    enum class State : unsigned char { Unloaded, Queued, Loaded };
    struct Tile {
        State state = State::Unloaded;
        JPH::BodyID terrain, props;
        bool cancel = false; // unloaded while queued: discard the result
    };

    Built build(int index) const; // thread-safe: reads only the level and the materials
    void addBodies(Built&& b);
    void removeBodies(int index);
    void workerLoop();
    void drainFinished();

    quench::PhysicsWorld& m_world;
    const SurfaceTable& m_surfaces;
    std::shared_ptr<const Level> m_level;
    Settings m_settings;
    int m_side = 0;
    float m_x0 = 0.0f;
    int m_sampleOffset = 0; // level sample index of the first tile sample
    std::vector<Tile> m_tiles;
    std::vector<std::vector<std::uint32_t>> m_tileTrees, m_tileRocks; // per tile: indices into the level lists
    std::size_t m_trunkCount = 0;
    std::vector<JPH::Ref<JPH::Shape>> m_modelShapes; // per level model: unit-scale collision mesh (null: none)
    JPH::BodyID m_staticScene; // static scene meshes (LevelRock::staticMesh): one body for the level's lifetime
    TerrainStreamingStats m_stats;

    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::deque<int> m_requests;
    std::vector<Built> m_finished;
    bool m_stop = false;
    int m_inFlight = -1; // tile index the worker is building
    std::thread m_worker;
};

} // namespace tf
