#pragma once

#include <string>
#include <vector>

namespace tf {

struct RoutePoint {
    float x = 0.0f, z = 0.0f;
};

// A closed driving loop over the ground of a world (the menu attract drive): center line points about 2 m apart in
// driving order; the last point connects back to the first.
struct DriveRoute {
    std::string name;
    float cruiseSpeed = 9.0f; // m/s on straight stretches
    std::vector<RoutePoint> points;

    bool valid() const { return points.size() >= 8; }
};

} // namespace tf
