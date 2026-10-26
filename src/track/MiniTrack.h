#pragma once

#include "track/DriveRoute.h"
#include "track/TestTrack.h"

#include <array>
#include <vector>

namespace tf {

// A timing line across the track: center, driving direction (unit, x/z) and half length. It counts when the vehicle
// passes from behind to in front of it within the half length.
struct TimingGate {
    float x = 0.0f, z = 0.0f;
    float dirX = 0.0f, dirZ = 1.0f;
    float halfWidth = 10.0f;
};

// Mini handling track: a ~1 km asphalt circuit in its own walled area south of the proving ground (outside the
// proving ground walls, reached by teleport). Procedural; its triangles are part of the proving ground mesh, so the
// rendered asphalt, curbs, runoff and tyre walls are the collision. Asphalt and curbs are the Concrete surface, gravel
// traps the rigid Sand surface, the runoff Grass.
struct MiniTrackInfo {
    std::vector<RoutePoint> center; // closed center line ~1 m apart in driving order; point 0 is on the start line
    std::vector<float> arc;         // arc length of each point, m
    std::vector<float> curvature;   // signed, 1/m (+ = turning toward the vehicle's left, +X)
    float length = 0.0f;            // lap length along the center line, m
    float halfWidth = 5.0f;         // asphalt half width, m (curb or shoulder up to kEdge beyond)
    TrackBounds area{};             // walled area (ground grid extent)
    std::array<TimingGate, 3> gates{}; // start/finish, end of sector 1, end of sector 2
    std::array<float, 3> gateArc{};    // arc length of each gate, m
    TrackFeature start;                // grid spot behind the start/finish line, facing along the track
    float steadyArc0 = 0.0f, steadyArc1 = 0.0f; // the middle of the constant-radius corner, m
    float steadyRadius = 0.0f;                  // its center line radius, m

    // The center line as a closed route (~2 m spacing) for path-following drivers.
    DriveRoute route(float cruiseSpeed) const;
    // Nearest center line arc length and the distance to it (brute force).
    float nearestArc(float x, float z, float* distance = nullptr) const;
};

const MiniTrackInfo& miniTrack();
// The circuit area contains (x, z) (its ground grid; the proving ground is outside it).
bool insideMiniTrackArea(float x, float z);
// Ground height of the circuit area (gentle slope and edge berms), without the asphalt ribbon and curbs.
float miniTrackGround(float x, float z);
// Physics surface of the ground at (x, z): Concrete on the asphalt ribbon, Sand in gravel traps, Grass elsewhere.
SurfaceId miniTrackSurface(float x, float z);
// Appends the circuit (ground grid, asphalt ribbon with curbs and lines, tyre walls, barriers, area walls).
void buildMiniTrack(TrackGeometry& g);

} // namespace tf
