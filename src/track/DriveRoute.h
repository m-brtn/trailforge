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
    // Pace of the autopilot on this route (0: its gentle defaults): curve m/s^2, planned braking m/s^2, throttle cap.
    float lateralAccel = 0.0f, brakeDecel = 0.0f, maxThrottle = 0.0f;
    std::vector<RoutePoint> points;

    bool valid() const { return points.size() >= 8; }
};

} // namespace tf
