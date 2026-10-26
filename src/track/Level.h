#pragma once

#include "track/SurfaceProperties.h"
#include "track/TestTrack.h"

#include <Jolt/Jolt.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace tf {

// Open-world level baked from a hand-authored description (assets/levels/<id>/level.json): control points, peak
// bumps, coast polygons, lakes, river and road splines, flats, forest and field polygons, points of interest.
// The bake is deterministic (no std::*_distribution, fixed hashes) and produces one height grid that is the
// physics heightfield (TerrainTiles) and the render terrain at the same samples.

enum class WaterKind : std::uint8_t { None = 0, Sea, Lake, River };

// Visual style of a level prop triangle (render colors only; the surface id decides the physics).
enum class PropStyle : std::uint8_t {
    Wood = 0, DarkWood, Stone, DarkStone, Plaster, RoofRed, RoofDark, Metal, Rust, Canvas, Marker, Beacon, Count
};

struct LevelPoi {
    int id = 0;
    std::string name, kind;
    float x = 0.0f, z = 0.0f;   // map marker
    float atX = 0.0f, atZ = 0.0f; // teleport / spawn point on drivable ground
};

struct RoadSample {
    float x = 0.0f, z = 0.0f, y = 0.0f; // center line, y = road surface height
    float dx = 0.0f, dz = 1.0f;         // unit direction along the road
    float bank = 0.0f;                  // rad, positive: the left (+side) edge is higher
    float s = 0.0f;                     // m along the road
    bool bridge = false;                // on a bridge deck (props carry the vehicle, the terrain is untouched)
    bool ford = false;                  // the road crosses water at this sample
};

struct LevelRoad {
    std::string name;
    bool trail = false;
    float width = 6.0f;
    float maxGrade = 0.12f;
    std::vector<RoadSample> samples; // every ~2 m
};

struct LevelRiverSample {
    float x = 0.0f, z = 0.0f, y = 0.0f; // y = water surface
};

struct LevelRiver {
    std::string name;
    float width = 8.0f, depth = 1.0f, bankSlope = 0.7f;
    std::vector<LevelRiverSample> samples;
};

struct LevelTree {
    float x = 0.0f, y = 0.0f, z = 0.0f; // trunk base
    float height = 10.0f, yaw = 0.0f, hue = 0.5f, width = 1.0f;
};

struct LevelRock {
    float x = 0.0f, y = 0.0f, z = 0.0f; // center
    float rx = 1.0f, ry = 1.0f, rz = 1.0f, yaw = 0.0f;
    std::uint32_t seed = 0;
    PropStyle style = PropStyle::DarkStone;
};

// Collision and render geometry of level objects; both sides call these, so the triangles are identical.
void addLevelRock(TrackGeometry& g, const LevelRock& r);
// Tree trunk: the hexagonal prism of the LOD0 pine trunk (render/Scenery buildPineMeshes, shader treeWorld):
// radius 0.022 h w, from -0.02 h to 0.32 h above the base. Wind sway moves it by < 2 cm and is ignored.
void addLevelTrunk(TrackGeometry& g, const LevelTree& t);

// What a prop piece belongs to (placement rules: bridges sit on the roads, everything else keeps off them).
enum class PropRole : std::uint8_t { Bridge, Building, Prop, Rock };

struct LevelProps {
    TrackGeometry geometry;          // static props: bridges, POI blockouts (collision == render)
    std::vector<PropStyle> styles;   // one per triangle
    // First triangle of every prop piece (a box, slab, prism, roof or rock; each one convex, normals outward).
    std::vector<std::uint32_t> pieceStarts;
    std::vector<PropRole> pieceRoles; // one per piece
};

struct LevelBridge {
    float x0, z0, x1, z1; // deck end points (center line)
    float y0, y1;         // deck surface height at the ends
    float halfWidth;
    // The deck reaches kRamp m into both banks as a ramp down onto the carved road (ya before x0, ye past x1).
    float ya = 0.0f, ye = 0.0f;
    static constexpr float kRamp = 3.0f;
    // Deck surface height at s m along the center line from (x0, z0) (-kRamp .. length + kRamp).
    float deckY(float s) const;
    float length() const;
};

struct LevelBakeStats {
    double totalMs = 0.0;
    int samples = 0;       // per side
    std::size_t trees = 0, rocks = 0, propTriangles = 0;
    int propsMoved = 0, propsRemoved = 0; // placed objects shifted / dropped to keep off the roads
    float minHeight = 0.0f, maxHeight = 0.0f;
};

class Level {
public:
    static constexpr float kNoWater = -1.0e30f;

    // Loads and bakes. Returns nullptr and an error message on failure. threads <= 0: hardware concurrency.
    static std::shared_ptr<const Level> load(const std::string& path, std::string* error = nullptr, int threads = 0);
    static std::shared_ptr<const Level> fromJsonText(const std::string& text, std::string* error = nullptr,
                                                     int threads = 0);

    const std::string& id() const { return m_id; }
    const std::string& name() const { return m_name; }

    // Height grid: samples() x samples() at cell() spacing, sample (0, 0) at (origin(), origin()).
    int samples() const { return m_n; }
    float origin() const { return m_origin; }
    float cell() const { return m_cell; }
    float bakeHalfExtent() const { return -m_origin; }
    float playHalfExtent() const { return m_playHalf; }
    const std::vector<float>& heights() const { return m_height; }
    float heightSample(int i, int j) const { return m_height[std::size_t(j) * m_n + i]; }
    // Height of the triangulated surface at (x, z), the same triangles as Jolt's HeightFieldShape:
    // per cell (x, z)-(x+1, z+1) triangles {00, 01, 11} and {00, 11, 10}.
    float heightAt(float x, float z) const;
    // Per sample water surface (kNoWater where dry) and kind.
    float waterSample(int i, int j) const { return m_water[std::size_t(j) * m_n + i]; }
    const std::vector<float>& waterSurfaces() const { return m_water; }
    // Water surface over (x, z) if the ground there is under water (nearest sample), else nullopt.
    std::optional<float> waterAt(float x, float z) const;
    WaterKind waterKindAt(float x, float z) const;

    // Per cell ((samples()-1)^2, row-major by z): physics surface and render masks.
    int cells() const { return m_n - 1; }
    SurfaceId cellSurface(int i, int j) const { return m_surface[std::size_t(j) * (m_n - 1) + i]; }
    const std::vector<SurfaceId>& cellSurfaces() const { return m_surface; }
    SurfaceId surfaceAt(float x, float z) const;
    // RGBA8 per cell: grass blade density, mud, wet, wheat (the proving ground mask semantics).
    const std::vector<std::uint32_t>& maskTexels() const { return m_mask; }
    // RGBA8 per cell: road paint (render-only, includes the worn shoulder), rock, sand, snow weights.
    const std::vector<std::uint32_t>& splatTexels() const { return m_splat; }

    const std::vector<LevelPoi>& pois() const { return m_pois; }
    const std::vector<LevelRoad>& roads() const { return m_roads; }
    const std::vector<LevelRiver>& rivers() const { return m_rivers; }
    const std::vector<LevelBridge>& bridges() const { return m_bridges; }
    const std::vector<LevelTree>& trees() const { return m_trees; }
    const std::vector<LevelRock>& rocks() const { return m_rocks; }
    const LevelProps& props() const { return m_props; }
    float snowLine() const { return m_snowLine; }

    // Spawn point (on the ground, facing along the road) and the point-of-interest teleports, in map order.
    TrackFeature spawn() const;
    std::vector<TrackFeature> features() const;
    // Nearest point on a road center line (not on a bridge or ford), oriented along the road.
    TrackFeature nearestRoadPoint(float x, float z) const;
    bool insidePlayArea(float x, float z, float margin = 0.0f) const;

    const LevelBakeStats& bakeStats() const { return m_stats; }

private:
    Level() = default;
    friend class LevelBaker;

    std::string m_id, m_name;
    int m_n = 0;
    float m_origin = 0.0f, m_cell = 1.0f, m_playHalf = 1000.0f, m_snowLine = 200.0f;
    std::vector<float> m_height, m_water;
    std::vector<WaterKind> m_waterKind;
    std::vector<SurfaceId> m_surface;
    std::vector<std::uint32_t> m_mask, m_splat;
    std::vector<LevelPoi> m_pois;
    std::vector<LevelRoad> m_roads;
    std::vector<LevelRiver> m_rivers;
    std::vector<LevelBridge> m_bridges;
    std::vector<LevelTree> m_trees;
    std::vector<LevelRock> m_rocks;
    LevelProps m_props;
    float m_spawnX = 0.0f, m_spawnZ = 0.0f, m_spawnTowardX = 0.0f, m_spawnTowardZ = 1.0f;
    LevelBakeStats m_stats;
};

// Small deterministic helpers shared with the tests.
namespace level_math {
// Uniform Catmull-Rom resampling of a polyline at about `step` meters.
std::vector<JPH::Float3> resampleSpline(const std::vector<JPH::Float3>& points, float step);
// Grade-limited profile: the cut/fill average of the largest function below and the smallest function above
// `target` with |dy/ds| <= grade, clamped to the slope envelopes of the pins (pinned samples are exact).
std::vector<float> gradeLimitedProfile(const std::vector<float>& target, const std::vector<float>& pin,
                                       float ds, float grade);
} // namespace level_math

} // namespace tf
