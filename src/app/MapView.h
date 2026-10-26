#pragma once

#include <optional>
#include <string>

namespace tf {

// Map math shared by the full-screen map, the minimap and the compass bar. Pure C++ (no SDL, no ImGui).
// World: +X east, -Z north (Y up). Screen: x right, y down, in ImGui points. Bearings are degrees clockwise from
// north (0 north, 90 east), like a compass.

struct MapPoint {
    float x = 0.0f, y = 0.0f;
};

// World <-> screen transform: the world point (cx, cz) sits at the screen point (sx, sy); `scale` is points per
// meter; `rotationDeg` is the bearing that points up (0: north up; the heading for a map that turns with the car).
struct MapView {
    float cx = 0.0f, cz = 0.0f;
    float sx = 0.0f, sy = 0.0f;
    float scale = 1.0f;
    float rotationDeg = 0.0f;

    MapPoint toScreen(float x, float z) const;
    MapPoint toWorld(float px, float py) const; // MapPoint::y holds the world z
    // Zooms by `factor` keeping the world point under the screen point (px, py) in place.
    void zoomAround(float factor, float px, float py, float minScale, float maxScale);
    // Moves the map by a screen delta (a drag of the content).
    void pan(float dxPoints, float dyPoints);
};

// Bearing of the direction (dx, dz) in degrees [0, 360).
float bearingDeg(float dx, float dz);
inline float bearingDeg(float fromX, float fromZ, float toX, float toZ) { return bearingDeg(toX - fromX, toZ - fromZ); }
// Signed difference target - heading, wrapped to (-180, 180].
float relativeBearingDeg(float targetDeg, float headingDeg);
// Horizontal position on a compass bar showing headingDeg in the middle and halfSpanDeg to each side, as a fraction
// -1..1 of the half width; nullopt when the bearing is outside the bar.
std::optional<float> compassOffset(float bearing, float headingDeg, float halfSpanDeg);
// Eight-point name of a bearing ("N", "NE", ...).
const char* compassPointName(float bearing);

// "85 m", "950 m", "1.2 km", "12 km".
std::string formatDistance(float meters);
// Largest 1 / 2 / 5 x 10^k meter length whose bar fits into maxPoints at `scale` points per meter.
float scaleBarMeters(float scale, float maxPoints);

// Map waypoint set by the player (world x, z); the map toggles it: a click near the current marker clears it.
struct MapMarker {
    bool set = false;
    float x = 0.0f, z = 0.0f;
};
// Sets the marker at (x, z), or clears it when it is set and within `clearRadius` meters of (x, z). Returns true
// when the marker is set afterwards.
bool toggleMarker(MapMarker& marker, float x, float z, float clearRadius);

} // namespace tf
