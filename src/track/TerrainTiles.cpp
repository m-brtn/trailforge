#include "track/TerrainTiles.h"

#include "physics/PhysicsWorld.h"
#include "track/SurfaceProperties.h"

#include <Jolt/Geometry/Triangle.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace tf {

namespace {
using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }

// Common quantization range for every tile, so shared edge samples compress the same way.
constexpr float kMinHeight = -64.0f, kMaxHeight = 448.0f;
} // namespace

TerrainTiles::TerrainTiles(PhysicsWorld& world, const SurfaceTable& surfaces, std::shared_ptr<const Level> level,
                           const Settings& settings)
    : m_world(world), m_surfaces(surfaces), m_level(std::move(level)), m_settings(settings) {
    const Level& L = *m_level;
    const float cell = L.cell();
    const float tile = float(kTileCells) * cell;
    m_side = int(std::ceil((2.0f * L.playHalfExtent() + 2.0f * cell) / tile));
    const int samplesNeeded = m_side * kTileCells + 1;
    m_sampleOffset = (L.samples() - samplesNeeded) / 2;
    if (m_sampleOffset < 0) throw std::runtime_error("TerrainTiles: the baked grid is smaller than the play area");
    m_x0 = L.origin() + float(m_sampleOffset) * cell;
    m_tiles.resize(std::size_t(m_side) * std::size_t(m_side));
    m_tileTrees.resize(m_tiles.size());
    m_tileRocks.resize(m_tiles.size());
    auto bucket = [&](float x, float z) -> int {
        const int tx = tileIndexOf(x), tz = tileIndexOf(z);
        return tx < 0 || tz < 0 ? -1 : tz * m_side + tx;
    };
    for (std::size_t i = 0; i < L.trees().size(); ++i)
        if (const int b = bucket(L.trees()[i].x, L.trees()[i].z); b >= 0) {
            m_tileTrees[std::size_t(b)].push_back(std::uint32_t(i));
            ++m_trunkCount;
        }
    for (std::size_t i = 0; i < L.rocks().size(); ++i)
        if (const int b = bucket(L.rocks()[i].x, L.rocks()[i].z); b >= 0) m_tileRocks[std::size_t(b)].push_back(std::uint32_t(i));
    if (m_settings.async) m_worker = std::thread([this] { workerLoop(); });
}

TerrainTiles::~TerrainTiles() {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stop = true;
        m_requests.clear();
    }
    m_cv.notify_all();
    if (m_worker.joinable()) m_worker.join();
    for (int i = 0; i < int(m_tiles.size()); ++i)
        if (m_tiles[std::size_t(i)].state == State::Loaded) removeBodies(i);
}

int TerrainTiles::tileIndexOf(float coord) const {
    const float t = (coord - m_x0) / tileSize();
    if (t < 0.0f) return -1;
    const int i = int(t);
    return i < m_side ? i : -1;
}

bool TerrainTiles::isLoaded(int tx, int tz) const {
    if (tx < 0 || tz < 0 || tx >= m_side || tz >= m_side) return false;
    return m_tiles[std::size_t(tz) * std::size_t(m_side) + std::size_t(tx)].state == State::Loaded;
}

float TerrainTiles::distanceToTile(int tx, int tz, float x, float z) const {
    const float s = tileSize();
    const float x0 = m_x0 + float(tx) * s, z0 = m_x0 + float(tz) * s;
    const float dx = std::max({x0 - x, 0.0f, x - (x0 + s)});
    const float dz = std::max({z0 - z, 0.0f, z - (z0 + s)});
    return std::hypot(dx, dz);
}

TerrainTiles::Built TerrainTiles::build(int index) const {
    const auto t0 = Clock::now();
    const Level& L = *m_level;
    const int tx = index % m_side, tz = index / m_side;
    const int n = kTileCells + 1;
    const int si = m_sampleOffset + tx * kTileCells, sj = m_sampleOffset + tz * kTileCells;
    std::vector<float> samples(std::size_t(n) * std::size_t(n));
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) samples[std::size_t(j) * n + i] = L.heightSample(si + i, sj + j);
    std::vector<JPH::uint8> mats(std::size_t(n - 1) * std::size_t(n - 1));
    for (int j = 0; j < n - 1; ++j)
        for (int i = 0; i < n - 1; ++i) mats[std::size_t(j) * (n - 1) + i] = JPH::uint8(L.cellSurface(si + i, sj + j));
    JPH::PhysicsMaterialList materials;
    for (std::size_t i = 0; i < kSurfaceCount; ++i) materials.push_back(m_surfaces.material(SurfaceId(i)));

    const float cell = L.cell();
    const JPH::Vec3 offset(L.origin() + float(si) * cell, 0.0f, L.origin() + float(sj) * cell);
    JPH::HeightFieldShapeSettings hs(samples.data(), offset, JPH::Vec3(cell, 1.0f, cell), JPH::uint32(n), mats.data(),
                                     materials);
    hs.mBlockSize = 4;
    hs.mBitsPerSample = 16;
    hs.mMinHeightValue = kMinHeight;
    hs.mMaxHeightValue = kMaxHeight;
    JPH::ShapeSettings::ShapeResult hr = hs.Create();
    if (hr.HasError()) throw std::runtime_error(std::string("HeightFieldShape: ") + hr.GetError().c_str());

    Built b;
    b.index = index;
    b.terrain = hr.Get();

    TrackGeometry g;
    for (std::uint32_t t : m_tileTrees[std::size_t(index)]) addLevelTrunk(g, L.trees()[t]);
    for (std::uint32_t r : m_tileRocks[std::size_t(index)]) addLevelRock(g, L.rocks()[r]);
    if (g.triangleCount() > 0) {
        JPH::TriangleList tris;
        tris.reserve(g.triangleCount());
        for (std::size_t i = 0; i < g.triangleCount(); ++i)
            tris.emplace_back(g.positions[3 * i], g.positions[3 * i + 1], g.positions[3 * i + 2], JPH::uint32(g.surfaces[i]));
        JPH::MeshShapeSettings ms(tris, materials);
        JPH::ShapeSettings::ShapeResult mr = ms.Create();
        if (mr.HasError()) throw std::runtime_error(std::string("MeshShape (tile props): ") + mr.GetError().c_str());
        b.props = mr.Get();
    }
    b.ms = msSince(t0);
    return b;
}

void TerrainTiles::addBodies(Built&& b) {
    Tile& t = m_tiles[std::size_t(b.index)];
    auto make = [&](const JPH::Shape* shape) {
        JPH::BodyCreationSettings bcs(shape, JPH::RVec3::sZero(), JPH::Quat::sIdentity(), JPH::EMotionType::Static,
                                      Layers::Static);
        bcs.mFriction = 0.8f;
        return m_world.bodies().CreateAndAddBody(bcs, JPH::EActivation::DontActivate);
    };
    t.terrain = make(b.terrain.GetPtr());
    t.props = b.props ? make(b.props.GetPtr()) : JPH::BodyID();
    t.state = State::Loaded;
    t.cancel = false;
    m_stats.lastBuildMs = b.ms;
    m_stats.maxBuildMs = std::max(m_stats.maxBuildMs, b.ms);
}

void TerrainTiles::removeBodies(int index) {
    Tile& t = m_tiles[std::size_t(index)];
    for (JPH::BodyID id : {t.terrain, t.props}) {
        if (id.IsInvalid()) continue;
        m_world.bodies().RemoveBody(id);
        m_world.bodies().DestroyBody(id);
    }
    t.terrain = t.props = JPH::BodyID();
    t.state = State::Unloaded;
}

void TerrainTiles::workerLoop() {
    for (;;) {
        int index = -1;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [&] { return m_stop || !m_requests.empty(); });
            if (m_stop) return;
            index = m_requests.front();
            m_requests.pop_front();
            m_inFlight = index;
        }
        Built b = build(index);
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_finished.push_back(std::move(b));
            m_inFlight = -1;
        }
        m_cv.notify_all();
    }
}

void TerrainTiles::drainFinished() {
    std::vector<Built> done;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        done.swap(m_finished);
    }
    for (Built& b : done) {
        Tile& t = m_tiles[std::size_t(b.index)];
        if (t.state != State::Queued) continue; // already built synchronously
        if (t.cancel) {
            t.state = State::Unloaded;
            t.cancel = false;
            continue;
        }
        ++m_stats.builtAsync;
        addBodies(std::move(b));
    }
}

void TerrainTiles::ensureLoaded(float x, float z, float radius) {
    const auto t0 = Clock::now();
    for (int tz = 0; tz < m_side; ++tz)
        for (int tx = 0; tx < m_side; ++tx) {
            if (distanceToTile(tx, tz, x, z) > radius) continue;
            const int index = tz * m_side + tx;
            Tile& t = m_tiles[std::size_t(index)];
            if (t.state == State::Loaded) {
                t.cancel = false;
                continue;
            }
            if (t.state == State::Queued) {
                std::unique_lock<std::mutex> lock(m_mutex);
                const auto it = std::find(m_requests.begin(), m_requests.end(), index);
                if (it != m_requests.end()) {
                    m_requests.erase(it);
                } else {
                    // In flight or finished: wait for the worker's result instead of building it twice.
                    m_cv.wait(lock, [&] {
                        return m_inFlight != index;
                    });
                    lock.unlock();
                    t.cancel = false;
                    drainFinished();
                    if (t.state == State::Loaded) continue;
                }
            }
            Built b = build(index);
            ++m_stats.builtSync;
            addBodies(std::move(b));
        }
    m_stats.maxSyncStallMs = std::max(m_stats.maxSyncStallMs, msSince(t0));
}

void TerrainTiles::update(float x, float z) {
    const auto t0 = Clock::now();
    drainFinished();
    // Remove far tiles (hysteresis: keepRadius > loadRadius), request near ones by distance.
    std::vector<std::pair<float, int>> wanted;
    for (int tz = 0; tz < m_side; ++tz)
        for (int tx = 0; tx < m_side; ++tx) {
            const int index = tz * m_side + tx;
            Tile& t = m_tiles[std::size_t(index)];
            const float d = distanceToTile(tx, tz, x, z);
            if (d > m_settings.keepRadius) {
                if (t.state == State::Loaded) {
                    removeBodies(index);
                    ++m_stats.unloaded;
                } else if (t.state == State::Queued) {
                    t.cancel = true;
                }
            } else if (d <= m_settings.loadRadius) {
                if (t.state == State::Unloaded) wanted.push_back({d, index});
                else if (t.state == State::Queued) t.cancel = false;
            }
        }
    std::sort(wanted.begin(), wanted.end());
    if (m_settings.async) {
        if (!wanted.empty()) {
            std::lock_guard<std::mutex> lock(m_mutex);
            for (const auto& w : wanted) {
                m_tiles[std::size_t(w.second)].state = State::Queued;
                m_requests.push_back(w.second);
            }
        }
        m_cv.notify_all();
    } else {
        for (const auto& w : wanted) {
            Built b = build(w.second);
            ++m_stats.builtSync;
            addBodies(std::move(b));
        }
    }
    // Anything still missing close to the focus is built now.
    bool missing = false;
    for (int tz = 0; tz < m_side && !missing; ++tz)
        for (int tx = 0; tx < m_side; ++tx)
            if (distanceToTile(tx, tz, x, z) <= m_settings.requiredRadius && !isLoaded(tx, tz)) {
                missing = true;
                break;
            }
    if (missing) ensureLoaded(x, z, m_settings.requiredRadius);

    int loaded = 0, queued = 0;
    for (const Tile& t : m_tiles) {
        loaded += t.state == State::Loaded;
        queued += t.state == State::Queued;
    }
    m_stats.loaded = loaded;
    m_stats.queued = queued;
    m_stats.lastUpdateMs = msSince(t0);
    m_stats.maxUpdateMs = std::max(m_stats.maxUpdateMs, m_stats.lastUpdateMs);
}

} // namespace tf
