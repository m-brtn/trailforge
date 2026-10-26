#pragma once

#include "track/DriveRoute.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace tf {

class Level;
struct TrackGeometry;

// Top-down map of a world, made once per level from the same data as the terrain: a hill-shaded raster (surfaces,
// forests, water depth, contour lines) and vector lines and points drawn on top at any zoom. Pure C++ (no SDL), built
// on a worker thread.
struct MapRaster {
    int size = 0;              // pixels per side
    float x0 = 0.0f, z0 = 0.0f; // world position of the top-left (north-west) corner of pixel (0, 0)
    float metersPerPixel = 1.0f;
    std::vector<std::uint32_t> rgba; // row-major from the north edge, R in the low byte, opaque
};

enum class MapLineKind : std::uint8_t { Road, Trail, River, Bridge };

struct MapLine {
    MapLineKind kind = MapLineKind::Road;
    float width = 6.0f; // m
    std::string name;
    std::vector<RoutePoint> points;
};

struct MapPoi {
    int id = 0;          // the number on the marker
    std::string name, kind;
    float x = 0.0f, z = 0.0f;
};

struct MapData {
    std::string title;          // "Exploration Peaks"
    std::string subtitle;       // "2 x 2 km"
    float playX0 = 0.0f, playZ0 = 0.0f, playX1 = 0.0f, playZ1 = 0.0f; // drivable area (the initial full view)
    float contourStep = 10.0f;  // m between contour lines (every fifth one is drawn stronger)
    MapRaster raster;
    std::vector<MapLine> lines;
    std::vector<MapPoi> pois;
    double buildMs = 0.0;       // wall time of the build
};

// Exploration level: `pixels` per side over the play area plus a margin (threads <= 0: hardware concurrency).
MapData buildLevelMap(const Level& level, int pixels = 2048, int threads = 0);

// Proving ground: the track triangles rasterized from above (collision == render == map), its puddle water level
// and the named features (start pad, ramps, ...) as points of interest.
struct ProvingGroundMapInput {
    const TrackGeometry* geometry = nullptr;
    float x0 = -70.0f, z0 = -52.0f, x1 = 70.0f, z1 = 88.0f; // map extent
    float waterLevel = -1.0e30f;
    float waterX0 = 0.0f, waterZ0 = 0.0f, waterX1 = 0.0f, waterZ1 = 0.0f;
    std::vector<MapPoi> pois;
};
MapData buildProvingGroundMap(const ProvingGroundMapInput& in, int pixels = 1024);

} // namespace tf
