#include "app/MapView.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tf {
namespace {

constexpr float kDegToRad = 0.0174532925f;

} // namespace

// A world offset (east e, south s) turns by -rotation (the bearing `rotationDeg` points up):
// screen = scale * (e cos r + s sin r, s cos r - e sin r).
MapPoint MapView::toScreen(float x, float z) const {
    const float r = rotationDeg * kDegToRad;
    const float c = std::cos(r), s = std::sin(r);
    const float e = x - cx, so = z - cz;
    return {sx + scale * (e * c + so * s), sy + scale * (so * c - e * s)};
}

MapPoint MapView::toWorld(float px, float py) const {
    const float r = rotationDeg * kDegToRad;
    const float c = std::cos(r), s = std::sin(r);
    const float u = (px - sx) / scale, v = (py - sy) / scale;
    return {cx + u * c - v * s, cz + u * s + v * c};
}

void MapView::zoomAround(float factor, float px, float py, float minScale, float maxScale) {
    const MapPoint before = toWorld(px, py);
    scale = std::clamp(scale * factor, minScale, maxScale);
    const MapPoint after = toWorld(px, py);
    cx += before.x - after.x;
    cz += before.y - after.y;
}

void MapView::pan(float dxPoints, float dyPoints) {
    const MapPoint a = toWorld(sx, sy), b = toWorld(sx + dxPoints, sy + dyPoints);
    cx -= b.x - a.x;
    cz -= b.y - a.y;
}

float bearingDeg(float dx, float dz) {
    float b = std::atan2(dx, -dz) / kDegToRad;
    if (b < 0.0f) b += 360.0f;
    return b >= 360.0f ? 0.0f : b;
}

float relativeBearingDeg(float targetDeg, float headingDeg) {
    float d = std::fmod(targetDeg - headingDeg, 360.0f);
    if (d <= -180.0f) d += 360.0f;
    if (d > 180.0f) d -= 360.0f;
    return d;
}

std::optional<float> compassOffset(float bearing, float headingDeg, float halfSpanDeg) {
    const float d = relativeBearingDeg(bearing, headingDeg);
    if (std::abs(d) > halfSpanDeg) return std::nullopt;
    return d / halfSpanDeg;
}

const char* compassPointName(float bearing) {
    static const char* kNames[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    const int i = int(std::floor(std::fmod(bearing + 22.5f + 720.0f, 360.0f) / 45.0f)) % 8;
    return kNames[i];
}

std::string formatDistance(float meters) {
    char buf[32];
    meters = std::max(meters, 0.0f);
    if (meters < 99.5f) std::snprintf(buf, sizeof(buf), "%d m", int(std::lround(meters)));
    else if (meters < 995.0f) std::snprintf(buf, sizeof(buf), "%d m", 10 * int(std::lround(meters / 10.0f)));
    else if (meters < 9950.0f) std::snprintf(buf, sizeof(buf), "%.1f km", double(meters) / 1000.0);
    else std::snprintf(buf, sizeof(buf), "%d km", int(std::lround(meters / 1000.0f)));
    return buf;
}

float scaleBarMeters(float scale, float maxPoints) {
    if (scale <= 0.0f || maxPoints <= 0.0f) return 0.0f;
    const float maxMeters = maxPoints / scale;
    float best = 0.0f;
    for (float decade = 0.1f; decade <= 1.0e5f; decade *= 10.0f)
        for (float m : {1.0f, 2.0f, 5.0f})
            if (m * decade <= maxMeters) best = m * decade;
    return best;
}

bool toggleMarker(MapMarker& marker, float x, float z, float clearRadius) {
    if (marker.set && std::hypot(marker.x - x, marker.z - z) <= clearRadius) {
        marker.set = false;
        return false;
    }
    marker = {true, x, z};
    return true;
}

} // namespace tf
