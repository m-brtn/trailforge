#include "app/MapData.h"

#include "track/Level.h"
#include "track/TestTrack.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <thread>

namespace tf {
namespace {

struct Rgb {
    float r, g, b;
};

Rgb mix(Rgb a, Rgb b, float t) { return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t}; }
Rgb scaled(Rgb a, float s) { return {a.r * s, a.g * s, a.b * s}; }

std::uint32_t pack(Rgb c) {
    auto u = [](float v) { return std::uint32_t(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); };
    return u(c.r) | (u(c.g) << 8) | (u(c.b) << 16) | 0xFF000000u;
}
Rgb unpack(std::uint32_t p) {
    return {float(p & 0xFF) / 255.0f, float((p >> 8) & 0xFF) / 255.0f, float((p >> 16) & 0xFF) / 255.0f};
}

constexpr Rgb kGrassLow{0.42f, 0.55f, 0.29f};
constexpr Rgb kGrassHigh{0.52f, 0.55f, 0.37f};
constexpr Rgb kDirt{0.60f, 0.49f, 0.35f};
constexpr Rgb kMud{0.43f, 0.36f, 0.26f};
constexpr Rgb kRock{0.52f, 0.50f, 0.47f};
constexpr Rgb kSnow{0.93f, 0.94f, 0.96f};
constexpr Rgb kSand{0.84f, 0.77f, 0.58f};
constexpr Rgb kConcrete{0.67f, 0.67f, 0.65f};
constexpr Rgb kLowGrip{0.50f, 0.58f, 0.64f};
constexpr Rgb kAsphalt{0.30f, 0.30f, 0.32f};
constexpr Rgb kGravel{0.71f, 0.67f, 0.59f};
constexpr Rgb kWheat{0.78f, 0.71f, 0.43f};
constexpr Rgb kWaterShallow{0.38f, 0.67f, 0.77f};
constexpr Rgb kWaterDeep{0.10f, 0.32f, 0.50f};
constexpr Rgb kCrownDark{0.16f, 0.29f, 0.17f};
constexpr Rgb kCrownLit{0.27f, 0.42f, 0.24f};
constexpr Rgb kContour{0.30f, 0.24f, 0.17f};
constexpr Rgb kOutside{0.08f, 0.12f, 0.19f};

Rgb surfaceColor(SurfaceId s) {
    switch (s) {
    case SurfaceId::Concrete: return kConcrete;
    case SurfaceId::PackedDirt: return kDirt;
    case SurfaceId::Rock: return kRock;
    case SurfaceId::LowGrip: return kLowGrip;
    case SurfaceId::Mud: return kMud;
    case SurfaceId::Sand: return kSand;
    case SurfaceId::Snow: return kSnow;
    case SurfaceId::Asphalt: return kAsphalt;
    case SurfaceId::Gravel: return kGravel;
    case SurfaceId::Grass:
    case SurfaceId::Count: break;
    }
    return kGrassLow;
}

// Mini handling track finishes.
Rgb paintColor(Paint p) {
    switch (p) {
    case Paint::Asphalt: return kAsphalt;
    case Paint::Line:
    case Paint::CurbWhite:
    case Paint::BarrierWhite: return {0.92f, 0.92f, 0.90f};
    case Paint::CurbRed:
    case Paint::BarrierRed: return {0.78f, 0.18f, 0.14f};
    case Paint::Gravel: return kGravel;
    case Paint::Tyre: return {0.12f, 0.12f, 0.13f};
    case Paint::None: break;
    }
    return kConcrete;
}

// Runs fn(row0, row1) over [0, rows) split across threads.
void parallelRows(int rows, int threads, const std::function<void(int, int)>& fn) {
    if (threads <= 0) threads = int(std::max(1u, std::thread::hardware_concurrency()));
    threads = std::clamp(threads, 1, std::max(1, rows / 16));
    std::vector<std::thread> pool;
    const int chunk = (rows + threads - 1) / threads;
    for (int t = 1; t < threads; ++t) {
        const int a = t * chunk, b = std::min(rows, a + chunk);
        if (a < b) pool.emplace_back(fn, a, b);
    }
    fn(0, std::min(rows, chunk));
    for (std::thread& t : pool) t.join();
}

// Hill shade from a height grid (light from the north-west, 45 degrees up; flat ground = 1) and contour lines every
// `step` m (every fifth stronger). `wet` pixels get neither contours nor shade beyond a hint.
void shadeAndContour(MapRaster& r, const std::vector<float>& h, const std::vector<std::uint8_t>& wet, float step,
                     float relief, int threads) {
    const int n = r.size;
    const float m = r.metersPerPixel;
    std::vector<std::uint32_t> out(r.rgba.size());
    parallelRows(n, threads, [&](int j0, int j1) {
        for (int j = j0; j < j1; ++j) {
            for (int i = 0; i < n; ++i) {
                const std::size_t k = std::size_t(j) * n + i;
                Rgb c = unpack(r.rgba[k]);
                const int il = std::max(i - 1, 0), ir = std::min(i + 1, n - 1);
                const int ju = std::max(j - 1, 0), jd = std::min(j + 1, n - 1);
                const float gx = relief * (h[std::size_t(j) * n + ir] - h[std::size_t(j) * n + il]) / (float(ir - il) * m);
                const float gz = relief * (h[std::size_t(jd) * n + i] - h[std::size_t(ju) * n + i]) / (float(jd - ju) * m);
                // n = normalize(-gx, 1, -gz), light (-0.5, 0.7071, -0.5) (towards the north-west, +z is south).
                const float len = std::sqrt(gx * gx + 1.0f + gz * gz);
                float shade = (0.5f * gx + 0.7071f + 0.5f * gz) / (len * 0.7071f);
                if (wet[k]) shade = 1.0f + 0.15f * (shade - 1.0f);
                c = scaled(c, std::clamp(0.35f + 0.65f * shade, 0.45f, 1.3f));
                if (!wet[k] && step > 0.0f) {
                    const float a = h[k];
                    bool minor = false, major = false;
                    for (const std::size_t nb : {std::size_t(j) * n + ir, std::size_t(jd) * n + i}) {
                        if (nb == k || wet[nb]) continue;
                        const float b = h[nb];
                        if (std::floor(a / step) != std::floor(b / step)) minor = true;
                        if (std::floor(a / (5.0f * step)) != std::floor(b / (5.0f * step))) major = true;
                    }
                    if (major) c = mix(c, kContour, 0.45f);
                    else if (minor) c = mix(c, kContour, 0.2f);
                }
                out[k] = pack(c);
            }
        }
    });
    r.rgba.swap(out);
}

// Soft disc blended over the raster (tree crowns lit from the north-west, rocks).
void stampDisc(MapRaster& r, float x, float z, float radiusM, Rgb dark, Rgb lit, float opacity) {
    const float cx = (x - r.x0) / r.metersPerPixel, cy = (z - r.z0) / r.metersPerPixel;
    const float rad = std::max(radiusM / r.metersPerPixel, 0.8f);
    const int i0 = std::max(int(std::floor(cx - rad)), 0), i1 = std::min(int(std::ceil(cx + rad)), r.size - 1);
    const int j0 = std::max(int(std::floor(cy - rad)), 0), j1 = std::min(int(std::ceil(cy + rad)), r.size - 1);
    for (int j = j0; j <= j1; ++j)
        for (int i = i0; i <= i1; ++i) {
            const float dx = (float(i) + 0.5f - cx) / rad, dy = (float(j) + 0.5f - cy) / rad;
            const float d2 = dx * dx + dy * dy;
            if (d2 >= 1.0f) continue;
            const float edge = std::clamp((1.0f - std::sqrt(d2)) * rad, 0.0f, 1.0f); // ~1 px soft edge
            const float light = std::clamp(0.5f - 0.5f * (dx + dy) * 0.7071f, 0.0f, 1.0f);
            std::uint32_t& p = r.rgba[std::size_t(j) * r.size + i];
            p = pack(mix(unpack(p), mix(dark, lit, light), opacity * edge));
        }
}

// Building footprint: rectangle of half extents (hx, hz) turned by yaw (building +X -> world (cos, -sin)) with a
// darker 1 px rim, blended over the raster.
void stampRect(MapRaster& r, float x, float z, float yaw, float hx, float hz, Rgb roof, float opacity) {
    const float c = std::cos(yaw), s = std::sin(yaw), m = r.metersPerPixel;
    const float reach = std::hypot(hx, hz) / m + 1.0f;
    const float cx = (x - r.x0) / m, cy = (z - r.z0) / m;
    const int i0 = std::max(int(std::floor(cx - reach)), 0), i1 = std::min(int(std::ceil(cx + reach)), r.size - 1);
    const int j0 = std::max(int(std::floor(cy - reach)), 0), j1 = std::min(int(std::ceil(cy + reach)), r.size - 1);
    const float ex = std::max(hx / m, 0.6f), ez = std::max(hz / m, 0.6f);
    for (int j = j0; j <= j1; ++j)
        for (int i = i0; i <= i1; ++i) {
            const float dx = float(i) + 0.5f - cx, dz = float(j) + 0.5f - cy;
            const float lx = std::abs(c * dx - s * dz), lz = std::abs(s * dx + c * dz);
            const float inside = std::min(ex - lx, ez - lz); // px to the nearest edge
            if (inside <= -0.5f) continue;
            const float cover = std::clamp(inside + 0.5f, 0.0f, 1.0f);
            const Rgb col = inside < 1.0f ? scaled(roof, 0.55f) : roof;
            std::uint32_t& p = r.rgba[std::size_t(j) * r.size + i];
            p = pack(mix(unpack(p), col, opacity * cover));
        }
}

Rgb buildingMapColor(const BuildingSpec& b) {
    switch (b.kind) {
    case BuildingKind::House:
        return (b.variant & kHouseShingles) ? Rgb{0.40f, 0.37f, 0.34f} : Rgb{0.72f, 0.33f, 0.21f};
    case BuildingKind::FuelCanopy:
    case BuildingKind::FuelShop: return {0.90f, 0.90f, 0.88f};
    case BuildingKind::Dock: return {0.46f, 0.35f, 0.24f};
    case BuildingKind::Tent: return {0.92f, 0.52f, 0.16f};
    default: return {0.56f, 0.56f, 0.54f};
    }
}

std::vector<RoutePoint> decimate(const std::vector<RoutePoint>& pts, std::size_t every) {
    std::vector<RoutePoint> out;
    for (std::size_t k = 0; k < pts.size(); k += every) out.push_back(pts[k]);
    if (!pts.empty() && (pts.size() - 1) % every != 0) out.push_back(pts.back());
    return out;
}

double msSince(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

} // namespace

MapData buildLevelMap(const Level& L, int pixels, int threads) {
    const auto t0 = std::chrono::steady_clock::now();
    MapData d;
    d.title = L.name();
    const float half = L.playHalfExtent();
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.0f x %.0f km", double(2.0f * half / 1000.0f), double(2.0f * half / 1000.0f));
    d.subtitle = buf;
    d.playX0 = d.playZ0 = -half;
    d.playX1 = d.playZ1 = half;
    d.contourStep = 10.0f;

    MapRaster& r = d.raster;
    const float extent = std::min(half * 1.12f, L.bakeHalfExtent());
    r.size = pixels;
    r.x0 = r.z0 = -extent;
    r.metersPerPixel = 2.0f * extent / float(pixels);
    const int n = pixels;
    r.rgba.assign(std::size_t(n) * n, 0u);
    std::vector<float> h(std::size_t(n) * n);
    std::vector<std::uint8_t> wet(std::size_t(n) * n, 0);
    const int cells = L.cells();
    const std::vector<std::uint32_t>& mask = L.maskTexels();
    parallelRows(n, threads, [&](int j0, int j1) {
        for (int j = j0; j < j1; ++j) {
            const float z = r.z0 + (float(j) + 0.5f) * r.metersPerPixel;
            for (int i = 0; i < n; ++i) {
                const float x = r.x0 + (float(i) + 0.5f) * r.metersPerPixel;
                const std::size_t k = std::size_t(j) * n + i;
                const float ground = L.heightAt(x, z);
                h[k] = ground;
                Rgb c;
                if (const auto water = L.waterAt(x, z)) {
                    const float depth = std::max(*water - ground, 0.0f);
                    const float deep = L.waterKindAt(x, z) == WaterKind::Sea ? 6.0f : 3.5f;
                    c = mix(kWaterShallow, kWaterDeep, std::clamp(depth / deep, 0.0f, 1.0f));
                    wet[k] = 1;
                    h[k] = *water;
                } else {
                    const SurfaceId s = L.surfaceAt(x, z);
                    c = s == SurfaceId::Grass ? mix(kGrassLow, kGrassHigh, std::clamp((ground - 40.0f) / 160.0f, 0.0f, 1.0f))
                                              : surfaceColor(s);
                    const int ci = std::clamp(int((x - L.origin()) / L.cell()), 0, cells - 1);
                    const int cj = std::clamp(int((z - L.origin()) / L.cell()), 0, cells - 1);
                    const float wheat = float(mask[std::size_t(cj) * cells + ci] >> 24) / 255.0f;
                    if (wheat > 0.0f) c = mix(c, kWheat, 0.8f * wheat);
                }
                if (!L.insidePlayArea(x, z)) c = mix(c, kOutside, 0.45f);
                r.rgba[k] = pack(c);
            }
        }
    });
    shadeAndContour(r, h, wet, d.contourStep, 1.6f, threads);
    for (const LevelRock& rock : L.rocks())
        if (!rock.staticMesh) stampDisc(r, rock.x, rock.z, std::max(rock.rx, rock.rz), kRock, kSnow, 0.7f);
    for (const LevelTree& t : L.trees()) stampDisc(r, t.x, t.z, 0.22f * t.height * t.width, kCrownDark, kCrownLit, 0.9f);
    for (const LevelBuilding& b : L.buildings()) {
        const LevelBuildingType& t = L.buildingTypes()[b.type];
        const BuildingKind k = t.spec.kind;
        if (!buildingIsStructure(k) && k != BuildingKind::Dock && k != BuildingKind::Tent) continue;
        // Footprint around the bounds center (a dock starts at its origin and runs along +Z).
        const float mx = 0.5f * (t.boundsMin.x + t.boundsMax.x), mz = 0.5f * (t.boundsMin.z + t.boundsMax.z);
        const float c = std::cos(b.yaw), s = std::sin(b.yaw);
        stampRect(r, b.x + c * mx + s * mz, b.z - s * mx + c * mz, b.yaw, t.footprintX, t.footprintZ,
                  buildingMapColor(t.spec), 0.95f);
    }

    for (const LevelRiver& river : L.rivers()) {
        MapLine line{MapLineKind::River, river.width, river.name, {}};
        for (const LevelRiverSample& s : river.samples) line.points.push_back({s.x, s.z});
        line.points = decimate(line.points, 2);
        d.lines.push_back(std::move(line));
    }
    for (const LevelRoad& road : L.roads()) {
        MapLine line{road.trail ? MapLineKind::Trail : MapLineKind::Road, road.width, road.name, {}};
        for (const RoadSample& s : road.samples) line.points.push_back({s.x, s.z});
        line.points = decimate(line.points, 2);
        d.lines.push_back(std::move(line));
    }
    for (const LevelBridge& b : L.bridges())
        d.lines.push_back({MapLineKind::Bridge, 2.0f * b.halfWidth, {}, {{b.x0, b.z0}, {b.x1, b.z1}}});
    for (const LevelPoi& p : L.pois()) d.pois.push_back({p.id, p.name, p.kind, p.x, p.z});
    d.buildMs = msSince(t0);
    return d;
}

MapData buildProvingGroundMap(const ProvingGroundMapInput& in, int pixels) {
    const auto t0 = std::chrono::steady_clock::now();
    MapData d;
    d.title = "Proving Ground";
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.0f x %.0f m", double(in.x1 - in.x0), double(in.z1 - in.z0));
    d.subtitle = buf;
    d.playX0 = in.x0;
    d.playZ0 = in.z0;
    d.playX1 = in.x1;
    d.playZ1 = in.z1;
    d.contourStep = 1.0f;
    MapRaster& r = d.raster;
    const float side = std::max(in.x1 - in.x0, in.z1 - in.z0);
    r.size = pixels;
    r.metersPerPixel = side / float(pixels);
    r.x0 = 0.5f * (in.x0 + in.x1) - 0.5f * side;
    r.z0 = 0.5f * (in.z0 + in.z1) - 0.5f * side;
    const int n = pixels;
    constexpr float kEmpty = -1.0e30f;
    std::vector<float> h(std::size_t(n) * n, kEmpty);
    std::vector<SurfaceId> surf(std::size_t(n) * n, SurfaceId::Count);
    std::vector<Paint> paint(std::size_t(n) * n, Paint::None);
    // Top-down rasterization of the upward-facing track triangles, highest surface wins.
    if (in.geometry) {
        const auto& p = in.geometry->positions;
        for (std::size_t t = 0; t + 2 < p.size(); t += 3) {
            const JPH::Float3 a = p[t], b = p[t + 1], c = p[t + 2];
            const float ax = (a.x - r.x0) / r.metersPerPixel, az = (a.z - r.z0) / r.metersPerPixel;
            const float bx = (b.x - r.x0) / r.metersPerPixel, bz = (b.z - r.z0) / r.metersPerPixel;
            const float cx = (c.x - r.x0) / r.metersPerPixel, cz = (c.z - r.z0) / r.metersPerPixel;
            const int i0 = std::max(int(std::floor(std::min({ax, bx, cx}))), 0);
            const int i1 = std::min(int(std::ceil(std::max({ax, bx, cx}))), n - 1);
            const int j0 = std::max(int(std::floor(std::min({az, bz, cz}))), 0);
            const int j1 = std::min(int(std::ceil(std::max({az, bz, cz}))), n - 1);
            const float pa = (bx - ax) * (cz - az) - (bz - az) * (cx - ax); // twice the signed area in pixels
            if (std::abs(pa) < 1e-6f) continue; // vertical faces
            for (int j = j0; j <= j1; ++j)
                for (int i = i0; i <= i1; ++i) {
                    const float px = float(i) + 0.5f, pz = float(j) + 0.5f;
                    const float w0 = ((bx - px) * (cz - pz) - (bz - pz) * (cx - px)) / pa;
                    const float w1 = ((cx - px) * (az - pz) - (cz - pz) * (ax - px)) / pa;
                    const float w2 = 1.0f - w0 - w1;
                    if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;
                    const float y = w0 * a.y + w1 * b.y + w2 * c.y;
                    const std::size_t k = std::size_t(j) * n + i;
                    if (y > h[k]) {
                        h[k] = y;
                        surf[k] = in.geometry->surfaces[t / 3];
                        paint[k] = in.geometry->paintOf(t / 3);
                    }
                }
        }
    }
    r.rgba.assign(std::size_t(n) * n, 0u);
    std::vector<std::uint8_t> wet(std::size_t(n) * n, 0);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const std::size_t k = std::size_t(j) * n + i;
            const float x = r.x0 + (float(i) + 0.5f) * r.metersPerPixel, z = r.z0 + (float(j) + 0.5f) * r.metersPerPixel;
            Rgb c = kOutside;
            if (h[k] > kEmpty) {
                c = paint[k] != Paint::None ? paintColor(paint[k]) : surfaceColor(surf[k]);
                if (h[k] < in.waterLevel && x >= in.waterX0 && x <= in.waterX1 && z >= in.waterZ0 && z <= in.waterZ1) {
                    c = mix(kWaterShallow, kWaterDeep, std::clamp((in.waterLevel - h[k]) / 0.6f, 0.0f, 1.0f));
                    wet[k] = 1;
                    h[k] = in.waterLevel;
                }
            } else {
                h[k] = 0.0f;
                wet[k] = 1; // no contours outside the ground
            }
            r.rgba[k] = pack(c);
        }
    shadeAndContour(r, h, wet, d.contourStep, 1.0f, 0);
    d.pois = in.pois;
    d.buildMs = msSince(t0);
    return d;
}

} // namespace tf
