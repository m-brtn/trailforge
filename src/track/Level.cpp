#include "track/Level.h"

#include "core/Json.h"
#include "core/ModelAsset.h"
#include "track/LevelFiles.h"


#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <thread>
#include <utility>

namespace tf {

using JPH::Float3;
using JPH::Vec3;

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }
float lerp(float a, float b, float t) { return a + (b - a) * t; }

struct P2 {
    float x, z;
};
float smoothstep(float e0, float e1, float x) {
    const float t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3.0f - 2.0f * t);
}

std::uint32_t hash3(std::int32_t x, std::int32_t y, std::uint32_t seed) {
    std::uint32_t h = std::uint32_t(x) * 0x8da6b343u ^ std::uint32_t(y) * 0xd8163841u ^ seed * 0xcb1ab31fu;
    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return h;
}
float hashUnit(std::int32_t x, std::int32_t y, std::uint32_t seed) { return float(hash3(x, y, seed) >> 8) / 16777216.0f; }

// Value noise in [0, 1] with quintic interpolation (deterministic on every platform).
float valueNoise(float x, float y, std::uint32_t seed) {
    const float fx = std::floor(x), fy = std::floor(y);
    const std::int32_t ix = std::int32_t(fx), iy = std::int32_t(fy);
    const float tx = x - fx, ty = y - fy;
    const float ux = tx * tx * tx * (tx * (tx * 6.0f - 15.0f) + 10.0f);
    const float uy = ty * ty * ty * (ty * (ty * 6.0f - 15.0f) + 10.0f);
    const float a = hashUnit(ix, iy, seed), b = hashUnit(ix + 1, iy, seed);
    const float c = hashUnit(ix, iy + 1, seed), d = hashUnit(ix + 1, iy + 1, seed);
    return lerp(lerp(a, b, ux), lerp(c, d, ux), uy);
}

float fbm(float x, float y, int octaves, std::uint32_t seed) {
    float sum = 0.0f, amp = 0.5f, norm = 0.0f;
    for (int o = 0; o < octaves; ++o) {
        // Rotate each octave a little so the lattice does not line up.
        const float rx = x * 0.8f - y * 0.6f, ry = x * 0.6f + y * 0.8f;
        sum += amp * valueNoise(rx, ry, seed + std::uint32_t(o) * 101u);
        norm += amp;
        x = rx * 2.03f;
        y = ry * 2.03f;
        amp *= 0.5f;
    }
    return sum / norm;
}

float ridged(float x, float y, int octaves, std::uint32_t seed) {
    float sum = 0.0f, amp = 0.5f, norm = 0.0f;
    for (int o = 0; o < octaves; ++o) {
        const float rx = x * 0.8f - y * 0.6f, ry = x * 0.6f + y * 0.8f;
        const float n = 1.0f - std::abs(valueNoise(rx, ry, seed + std::uint32_t(o) * 131u) * 2.0f - 1.0f);
        sum += amp * n * n;
        norm += amp;
        x = rx * 2.1f;
        y = ry * 2.1f;
        amp *= 0.5f;
    }
    return sum / norm;
}

struct Vec2 {
    float x = 0.0f, z = 0.0f;
};

struct Polygon {
    std::vector<Vec2> p;
    float x0 = 1e9f, z0 = 1e9f, x1 = -1e9f, z1 = -1e9f;
    void finish() {
        for (const Vec2& v : p) {
            x0 = std::min(x0, v.x);
            x1 = std::max(x1, v.x);
            z0 = std::min(z0, v.z);
            z1 = std::max(z1, v.z);
        }
    }
    bool inside(float x, float z) const {
        if (x < x0 || x > x1 || z < z0 || z > z1) return false;
        bool in = false;
        for (std::size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
            const Vec2 &a = p[i], &b = p[j];
            if ((a.z > z) != (b.z > z) && x < (b.x - a.x) * (z - a.z) / (b.z - a.z) + a.x) in = !in;
        }
        return in;
    }
    float edgeDistance(float x, float z) const {
        float best = 1e30f;
        for (std::size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
            const Vec2 &a = p[j], &b = p[i];
            const float ex = b.x - a.x, ez = b.z - a.z;
            const float t = clamp01(((x - a.x) * ex + (z - a.z) * ez) / std::max(1e-6f, ex * ex + ez * ez));
            const float dx = x - (a.x + ex * t), dz = z - (a.z + ez * t);
            best = std::min(best, dx * dx + dz * dz);
        }
        return std::sqrt(best);
    }
    // Positive inside.
    float signedDistance(float x, float z) const {
        const float d = edgeDistance(x, z);
        return inside(x, z) ? d : -d;
    }
};

template <typename F> void parallelFor(int count, int threads, F&& fn) {
    threads = std::max(1, std::min(threads, count));
    if (threads == 1) {
        for (int i = 0; i < count; ++i) fn(i);
        return;
    }
    std::atomic<int> next{0};
    std::vector<std::thread> pool;
    pool.reserve(std::size_t(threads));
    for (int t = 0; t < threads; ++t)
        pool.emplace_back([&] {
            for (int i = next.fetch_add(1); i < count; i = next.fetch_add(1)) fn(i);
        });
    for (auto& th : pool) th.join();
}

bool readPoint(const json::Value& v, float& x, float& z, float* y = nullptr) {
    if (!v.isArray() || v.array.size() < 2) return false;
    x = float(v.array[0].numberOr(0.0));
    z = float(v.array[1].numberOr(0.0));
    if (y) *y = v.array.size() >= 3 ? float(v.array[2].numberOr(0.0)) : kNaN;
    return true;
}

Polygon readPolygon(const json::Value& v) {
    Polygon poly;
    for (const json::Value& p : v.array) {
        Vec2 q;
        if (readPoint(p, q.x, q.z)) poly.p.push_back(q);
    }
    poly.finish();
    return poly;
}

// Chaikin corner cutting: each pass replaces every edge by its 1/4 and 3/4 points (organic outlines from a few
// authored vertices; the shape shrinks slightly at the corners).
Polygon smoothedPolygon(Polygon poly, int passes) {
    for (int k = 0; k < passes && poly.p.size() >= 3; ++k) {
        std::vector<Vec2> out;
        out.reserve(poly.p.size() * 2);
        for (std::size_t i = 0; i < poly.p.size(); ++i) {
            const Vec2 a = poly.p[i], b = poly.p[(i + 1) % poly.p.size()];
            out.push_back({0.75f * a.x + 0.25f * b.x, 0.75f * a.z + 0.25f * b.z});
            out.push_back({0.25f * a.x + 0.75f * b.x, 0.25f * a.z + 0.75f * b.z});
        }
        poly.p = std::move(out);
    }
    poly = Polygon{poly.p};
    poly.finish();
    return poly;
}


std::uint32_t packRgba(float r, float g, float b, float a) {
    auto q = [](float v) { return std::uint32_t(std::lround(clamp01(v) * 255.0f)); };
    return q(r) | (q(g) << 8) | (q(b) << 16) | (q(a) << 24);
}
float channel(std::uint32_t v, int c) { return float((v >> (8 * c)) & 0xffu) / 255.0f; }

} // namespace

// ------------------------------------------------------------------------------------------------------------------
namespace level_math {

std::vector<Float3> resampleSpline(const std::vector<Float3>& pts, float step) {
    if (pts.size() < 2) return pts;
    // Dense Catmull-Rom polyline, then samples at equal arc length; y carries the pins (NaN = none) of the control
    // points, moved to the nearest output sample.
    auto P = [&](int i) { return pts[std::size_t(std::clamp(i, 0, int(pts.size()) - 1))]; };
    std::vector<Float3> dense;
    std::vector<float> len;
    std::vector<std::pair<std::size_t, float>> pins; // dense index, pin
    for (int i = 0; i + 1 < int(pts.size()); ++i) {
        const Float3 p0 = P(i - 1), p1 = P(i), p2 = P(i + 1), p3 = P(i + 2);
        const float seg = std::hypot(p2.x - p1.x, p2.z - p1.z);
        const int m = std::max(4, int(std::ceil(seg / step * 8.0f)));
        if (!std::isnan(p1.y)) pins.push_back({dense.size(), p1.y});
        for (int k = 0; k < m; ++k) {
            const float t = float(k) / float(m), t2 = t * t, t3 = t2 * t;
            auto cr = [&](float a, float b, float c, float d) {
                return 0.5f * (2.0f * b + (-a + c) * t + (2.0f * a - 5.0f * b + 4.0f * c - d) * t2 +
                               (-a + 3.0f * b - 3.0f * c + d) * t3);
            };
            dense.push_back(Float3(cr(p0.x, p1.x, p2.x, p3.x), kNaN, cr(p0.z, p1.z, p2.z, p3.z)));
        }
    }
    if (!std::isnan(pts.back().y)) pins.push_back({dense.size(), pts.back().y});
    dense.push_back(Float3(pts.back().x, kNaN, pts.back().z));
    len.resize(dense.size(), 0.0f);
    for (std::size_t i = 1; i < dense.size(); ++i)
        len[i] = len[i - 1] + std::hypot(dense[i].x - dense[i - 1].x, dense[i].z - dense[i - 1].z);
    const float total = len.back();
    const int n = std::max(1, int(std::lround(total / step)));
    std::vector<Float3> out;
    out.reserve(std::size_t(n) + 1);
    std::size_t k = 0;
    for (int i = 0; i <= n; ++i) {
        const float s = total * float(i) / float(n);
        while (k + 2 < dense.size() && len[k + 1] < s) ++k;
        const float t = clamp01((s - len[k]) / std::max(1e-6f, len[k + 1] - len[k]));
        out.push_back(Float3(lerp(dense[k].x, dense[k + 1].x, t), kNaN, lerp(dense[k].z, dense[k + 1].z, t)));
    }
    for (const auto& [di, y] : pins) {
        const std::size_t oi = std::size_t(std::lround(len[di] / total * float(n)));
        out[std::min(oi, out.size() - 1)].y = y;
    }
    return out;
}

std::vector<float> gradeLimitedProfile(const std::vector<float>& target, const std::vector<float>& pin, float ds,
                                       float grade) {
    const std::size_t n = target.size();
    const float g = grade * ds;
    // Slope envelopes of the pins.
    std::vector<float> lo(n, -1e30f), hi(n, 1e30f);
    for (std::size_t i = 0; i < n; ++i) {
        if (std::isnan(pin[i])) continue;
        for (std::size_t k = 0; k < n; ++k) {
            const float d = g * float(k > i ? k - i : i - k);
            lo[k] = std::max(lo[k], pin[i] - d);
            hi[k] = std::min(hi[k], pin[i] + d);
        }
    }
    std::vector<float> t(n);
    for (std::size_t i = 0; i < n; ++i) t[i] = std::clamp(target[i], lo[i], hi[i]);
    // Largest Lipschitz function below t (cut) and smallest above t (fill).
    std::vector<float> a = t, b = t;
    for (std::size_t i = 1; i < n; ++i) {
        a[i] = std::min(a[i], a[i - 1] + g);
        b[i] = std::max(b[i], b[i - 1] - g);
    }
    for (std::size_t i = n - 1; i-- > 0;) {
        a[i] = std::min(a[i], a[i + 1] + g);
        b[i] = std::max(b[i], b[i + 1] - g);
    }
    std::vector<float> y(n);
    for (std::size_t i = 0; i < n; ++i) y[i] = std::clamp(0.5f * (a[i] + b[i]), lo[i], hi[i]);
    // Light smoothing (moving average keeps the slope limit), then the pins again.
    const int r = 4;
    std::vector<float> s(n);
    for (int pass = 0; pass < 2; ++pass) {
        for (std::size_t i = 0; i < n; ++i) {
            float sum = 0.0f;
            int c = 0;
            for (int k = -r; k <= r; ++k) {
                const long j = long(i) + k;
                if (j < 0 || j >= long(n)) continue;
                sum += y[std::size_t(j)];
                ++c;
            }
            s[i] = std::clamp(sum / float(c), lo[i], hi[i]);
        }
        y.swap(s);
    }
    return y;
}

} // namespace level_math

// ------------------------------------------------------------------------------------------------------------------
class LevelBaker {
public:
    LevelBaker(Level& out, int threads, std::string baseDir)
        : L(out), m_threads(threads > 0 ? threads : int(std::max(1u, std::thread::hardware_concurrency()))),
          m_baseDir(std::move(baseDir)) {}

    bool parse(const json::Value& doc, std::string& err);
    void bake();
    // After bake(): the attract route from the baked road center lines.
    bool buildAttractRoute(std::string& err);

private:
    struct RouteLeg { std::string road; float x0, z0, x1, z1; };
    std::vector<RouteLeg> m_routeLegs;
    std::string m_routeName;
    float m_routeSpeed = 9.0f;
    float m_routeLateral = 0.0f, m_routeDecel = 0.0f, m_routeThrottle = 0.0f;
    struct Peak { float x, z, h, r; };
    struct Beach { float x, z, r; };
    struct Lake { std::string name; float level, depth; Polygon poly; float shore = 0.0f; };
    struct RiverDef { std::string name; float width, depth, bank; std::vector<Float3> points; };
    struct BridgeDef { float x0, z0, x1, z1; };
    struct RoadDef { std::string name; bool trail; float width, grade; std::vector<Float3> points; std::vector<BridgeDef> bridges; SurfaceId surface; };
    struct Flat { std::string name; float x, z, r, h, fade; SurfaceId surface; };
    struct Forest { Polygon poly; float density; };
    struct Field { Polygon poly; };

    std::size_t idx(int i, int j) const { return std::size_t(j) * std::size_t(m_n) + std::size_t(i); }
    float X(int i) const { return m_origin + float(i) * m_cell; }
    float sampleGrid(const std::vector<float>& g, float x, float z) const;
    float coastDistance(float x, float z) const;
    float beachFactor(float x, float z) const;
    float forestDensity(float x, float z) const;
    float treeDensity(float x, float z, float forest) const;
    float snowLineAt(float x, float z) const {
        return m_snowLine + m_snowEastRise * std::max(0.0f, x - m_snowEastFrom) +
               m_snowNoise * (fbm(x / 70.0f, z / 70.0f, 3, m_seed + 71u) - 0.5f) * 2.0f;
    }

    void bakeBase();
    void bakeCoast();
    void bakeFlats();
    void bakeLakes();
    void bakeRivers();
    void bakeRoads();
    void bakeWater();
    void bakeSurfaces();
    void bakeTrees();
    void bakeRocks();
    void bakeProps();
    void bakeLogs();
    void bakeCliffs();
    void bakeTalus();
    void bakeCutBanks();
    void bakeCanopy();
    void bakeBankSmoothing(const std::vector<float>& uncarved);
    void bakeScatter();
    // Model library: "vegetation" lists model paths (relative to "dir", relative to the level file) per role.
    bool loadModels(const json::Value& veg, std::string& err);
    void bakeWaterDistance();
    float waterDistance(float x, float z) const; // m to the nearest water, 8 m grid, capped at 120 m

    // Props helpers.
    void addProp(const std::function<void(TrackGeometry&)>& build, PropStyle style);
    void box(float x, float y, float z, float hx, float hy, float hz, float yaw, PropStyle style,
             SurfaceId s = SurfaceId::Concrete);
    void groundedBox(float x, float z, float hx, float hy, float hz, float yaw, PropStyle style,
                     SurfaceId s = SurfaceId::Concrete, float lift = 0.0f);
    void slab(Vec3 a, Vec3 b, float halfWidth, float thickness, PropStyle style, SurfaceId s);
    void rock(Vec3 center, Vec3 radii, float yaw, std::uint32_t seed, PropStyle style);
    // Kit buildings (track/BuildingKit). building() places one at (x, z) turned by yaw, with its floor at floorY or,
    // when floorY is kGroundFloor, from the ground under its bounds (structures: above the highest sample, the plinth
    // reaches down; tents: on the lowest; small props: the middle). Its collision pieces form one Site.
    static constexpr float kGroundFloor = -1.0e30f;
    std::uint16_t buildingType(const BuildingSpec& spec);
    void building(const BuildingSpec& spec, float x, float z, float yaw, float floorY = kGroundFloor);
    float groundFloor(std::uint16_t type, float x, float z, float yaw, float* spread = nullptr) const;
    void dock(float x0, float z0, float x1, float z1, float deckY, float width);
    std::optional<P2> shoreDock(float x, float z, float dirX, float dirZ, float reach, float width);
    void powerLine(const std::vector<P2>& poles, float height);
    std::vector<BuildingGeometry> m_kitGeometry; // per L.m_buildingTypes entry (collision only)
    std::vector<std::size_t> m_structureSites;   // m_sites indices of placed structures (tree / rock clearing)
    std::vector<bool> m_onGround;                // per L.m_buildings entry: floor from groundFloor()
    std::vector<std::size_t> m_buildingTri;      // per L.m_buildings entry: its first props triangle
    float footprintMin(float x, float z, float hx, float hz, float yaw) const;
    float roadClearance(float x, float z) const;   // distance to the nearest road edge or bridge deck (m), large when far
    float bridgeClearance(float x, float z) const; // distance to the nearest bridge deck with its ramps (m)
    PropRole m_role = PropRole::Prop; // role of the pieces addProp adds

    // Placement. Props added inside one Site (or a single addProp outside any) form one object: if it is too close to
    // a road (or a bridge), a building is in water, or it covers a spawn / teleport point, the object moves as a whole
    // (horizontally, plus the terrain height difference at its center) to the first clear spot on rings of 2..40 m
    // around it, starting at a hashed angle (same seed, same layout), and is dropped if there is none.
    struct Site {
        explicit Site(LevelBaker& baker)
            : b(baker), tri(baker.L.m_props.geometry.triangleCount()), piece(baker.L.m_props.pieceStarts.size()),
              building(baker.L.m_buildings.size()), outer(!baker.m_inSite) {
            b.m_inSite = true;
        }
        ~Site() {
            if (!outer) return;
            b.m_inSite = false;
            b.settle(tri, piece, building);
        }
        Site(const Site&) = delete;
        Site& operator=(const Site&) = delete;
        LevelBaker& b;
        std::size_t tri, piece, building;
        bool outer;
    };
    void settle(std::size_t firstTriangle, std::size_t firstPiece, std::size_t firstBuilding);
    bool siteClear(const std::vector<P2>& hull, float margin, bool building, bool moved) const;
    void buildRoadPoints();
    static constexpr float kRoadMargin = 2.0f;     // m from the road edge: trees, rocks
    static constexpr float kPropMargin = 1.5f;     // small props (posts, signs, fences)
    static constexpr float kBuildingMargin = 3.0f; // buildings
    static constexpr float kGridSlack = 0.75f;     // roadClearance reads the nearest 1 m grid sample
    bool m_inSite = false;
    struct RoadPoint { float x, z, hw; };
    std::vector<std::vector<RoadPoint>> m_roadBuckets; // road center line every 0.5 m, in 32 m buckets
    int m_bucketSide = 0;
    std::vector<std::vector<P2>> m_sites;             // footprints of the placed objects
    std::vector<TrackFeature> m_features;             // spawn and teleport points

    Level& L;
    int m_threads;
    std::uint32_t m_seed = 1;
    float m_half = 1280.0f, m_cell = 1.0f, m_origin = -1280.0f;
    int m_n = 0;
    float m_snowLine = 200.0f, m_snowNoise = 15.0f, m_rockSlope = 36.0f;
    float m_snowEastRise = 0.0f, m_snowEastFrom = 0.0f; // snow line rises by eastRise m per m east of eastFrom
    float m_mtLow = 110, m_mtHigh = 190, m_ridgeAmp = 20, m_ridgeScale = 150, m_hillAmp = 5, m_hillScale = 95,
          m_fineAmp = 1.2f, m_fineScale = 24;
    std::vector<Float3> m_controls;
    std::vector<Peak> m_peaks;
    std::vector<Polygon> m_land;
    std::vector<Beach> m_beaches;
    float m_seaLevel = 0.0f, m_maxDepth = 20.0f, m_cliffRise = 1.0f, m_beachRise = 0.035f;
    std::vector<Lake> m_lakes;
    std::vector<RiverDef> m_riverDefs;
    std::vector<RoadDef> m_roadDefs;
    std::vector<Flat> m_flats;
    std::vector<Forest> m_forests;
    float m_meadowTrees = 0.03f;
    std::vector<Field> m_fields;

    // Bake state.
    std::vector<float> m_h;          // heights
    std::vector<float> m_coastD;     // coast signed distance on a 2 m grid
    int m_cn = 0;                    // coast grid samples per side
    std::vector<std::uint8_t> m_lakeId; // 1 + lake index where inside a lake polygon
    std::vector<float> m_riverD, m_riverY, m_riverReach;
    std::vector<float> m_roadness, m_roadEdge; // 0..1 road surface, distance to the nearest road edge (shoulder only)
    std::vector<SurfaceId> m_roadSurface;      // surface of the road with the largest roadness
    std::vector<float> m_roadClear;            // distance to the nearest road edge within 48 m of a road
    std::vector<SurfaceId> m_flatSurface;      // per sample: forced surface of a flat (Count = none)
    std::vector<std::uint8_t> m_nearWater;     // 0..255: water within ~3 m
    std::vector<float> m_forest;               // per cell forest density
    std::string m_baseDir;
    std::vector<int> m_conifers, m_saplings, m_broadleaf, m_bushes, m_ferns, m_debris, m_rockModels, m_logs, m_cliffModels;
    std::vector<float> m_waterDist;            // 8 m grid
    int m_wn = 0;

    // Baked levels (level.json "baked", files from tools/levels/blender/export_level.py).
    // Painted lines and curbs of a race circuit road (layout road "markings"); s in m along the exported samples.
    struct BakedMarkings {
        float edgeLine = 0.0f, edgeInset = 0.3f; // edge line width (0: none) and its gap to the road edge
        float startLine = -1.0f;                 // s of the start / finish line (< 0: none)
        int gridSlots = 0;                       // staggered start grid behind the start line
        float gridPitch = 8.0f;
        struct Curb { float s0, s1, side; };     // side +1 left, -1 right of the direction of travel
        std::vector<Curb> curbs;
    };
    struct BakedRoad {
        std::string name, kind;
        float width;
        bool closed;
        std::vector<Float3> samples; // x, z, bank
        BakedMarkings markings;
    };
    void bakeMarkings(const LevelRoad& road, const BakedMarkings& mk);
    struct BakedPlacement {
        std::string kind, model;
        BuildingSpec spec;
        float x = 0, z = 0, yaw = 0, scale = 1, width = 1, sx = 1, sy = 1, sz = 1, sink = 0.3f;
    };
    bool parseBaked(const json::Value& doc, std::string& err);
    void bakeBaked(std::chrono::steady_clock::time_point t0);
    void bakeBakedSurfaces();
    void placeBaked();
    void checkBakedLayout();
    float roadGap(const std::vector<P2>& hull) const; // m from a footprint to the nearest road edge
    bool m_baked = false;
    std::vector<float> m_bakedHeight;
    std::vector<std::uint8_t> m_bakedClass;
    std::vector<BakedRoad> m_bakedRoads;
    std::vector<BakedPlacement> m_placements;
    std::vector<LevelGate> m_gates;
    std::string m_circuitRoad;
    float m_spawnYaw = 0.0f;
    std::map<std::string, int> m_modelByRef; // vegetation entry ("fir_tree_01/fir_tree_01_a") -> L.m_models index
};

float LevelBaker::sampleGrid(const std::vector<float>& g, float x, float z) const {
    const float fx = std::clamp((x - m_origin) / m_cell, 0.0f, float(m_n - 1) - 0.001f);
    const float fz = std::clamp((z - m_origin) / m_cell, 0.0f, float(m_n - 1) - 0.001f);
    const int i = int(fx), j = int(fz);
    const float tx = fx - float(i), tz = fz - float(j);
    const float a = g[idx(i, j)], b = g[idx(i + 1, j)], c = g[idx(i, j + 1)], d = g[idx(i + 1, j + 1)];
    return lerp(lerp(a, b, tx), lerp(c, d, tx), tz);
}

float LevelBaker::coastDistance(float x, float z) const {
    const float step = 2.0f;
    const float fx = std::clamp((x - m_origin) / step, 0.0f, float(m_cn - 1) - 0.001f);
    const float fz = std::clamp((z - m_origin) / step, 0.0f, float(m_cn - 1) - 0.001f);
    const int i = int(fx), j = int(fz);
    const float tx = fx - float(i), tz = fz - float(j);
    auto at = [&](int a, int b) { return m_coastD[std::size_t(b) * std::size_t(m_cn) + std::size_t(a)]; };
    return lerp(lerp(at(i, j), at(i + 1, j), tx), lerp(at(i, j + 1), at(i + 1, j + 1), tx), tz);
}

float LevelBaker::beachFactor(float x, float z) const {
    float b = 0.0f;
    for (const Beach& c : m_beaches) b = std::max(b, smoothstep(c.r, c.r * 0.55f, std::hypot(x - c.x, z - c.z)));
    return b;
}

float LevelBaker::forestDensity(float x, float z) const {
    // Polygon edges are warped by noise (up to ~45 m) so the forests have ragged, natural borders.
    const float wx = x + 45.0f * (fbm(x / 140.0f, z / 140.0f, 3, m_seed + 301u) - 0.5f) * 2.0f;
    const float wz = z + 45.0f * (fbm(x / 140.0f, z / 140.0f, 3, m_seed + 307u) - 0.5f) * 2.0f;
    float d = 0.0f;
    for (const Forest& f : m_forests)
        if (f.poly.inside(wx, wz)) d = std::max(d, f.density * smoothstep(0.0f, 30.0f, f.poly.edgeDistance(wx, wz)));
    return d;
}

float LevelBaker::treeDensity(float x, float z, float forest) const {
    const float cluster = smoothstep(0.30f, 0.62f, fbm(x / 45.0f, z / 45.0f, 3, m_seed + 107u));
    return std::max(forest * (0.35f + 0.65f * cluster), m_meadowTrees * cluster);
}

bool LevelBaker::parse(const json::Value& doc, std::string& err) {
    if (!doc.isObject()) {
        err = "level: the document is not an object";
        return false;
    }
    if (int(doc["format"].numberOr(0)) != 1) {
        err = "level: unsupported format (expected 1)";
        return false;
    }
    L.m_id = doc["id"].stringOr("");
    L.m_name = doc["name"].stringOr(L.m_id);
    if (L.m_id.empty()) {
        err = "level: missing id";
        return false;
    }
    m_seed = std::uint32_t(doc["seed"].numberOr(1));
    m_half = float(doc["bakeHalfExtent"].numberOr(1280));
    L.m_playHalf = float(doc["playHalfExtent"].numberOr(1000));
    if (m_half < 256.0f || m_half > 4096.0f || L.m_playHalf <= 0.0f || L.m_playHalf > m_half - 16.0f) {
        err = "level: bakeHalfExtent must be 256..4096 m and larger than playHalfExtent + 16 m";
        return false;
    }
    if (!parseDriftConfig(doc["drift"], L.m_drift, err)) {
        err = "level: " + err;
        return false;
    }
    if (doc.has("baked")) return parseBaked(doc, err);
    const json::Value& snow = doc["snow"];
    m_snowLine = float(snow["line"].numberOr(1e9));
    m_snowNoise = float(snow["noise"].numberOr(0));
    m_snowEastRise = float(snow["eastRise"].numberOr(0));
    m_snowEastFrom = float(snow["eastFrom"].numberOr(0));
    m_rockSlope = float(doc["rockSlopeDeg"].numberOr(36));
    const json::Value& det = doc["detail"];
    m_mtLow = float(det["mountainLow"].numberOr(m_mtLow));
    m_mtHigh = float(det["mountainHigh"].numberOr(m_mtHigh));
    m_ridgeAmp = float(det["ridgeAmplitude"].numberOr(m_ridgeAmp));
    m_ridgeScale = float(det["ridgeScale"].numberOr(m_ridgeScale));
    m_hillAmp = float(det["hillAmplitude"].numberOr(m_hillAmp));
    m_hillScale = float(det["hillScale"].numberOr(m_hillScale));
    m_fineAmp = float(det["fineAmplitude"].numberOr(m_fineAmp));
    m_fineScale = float(det["fineScale"].numberOr(m_fineScale));

    for (const json::Value& p : doc["elevation"].array) {
        if (!p.isArray() || p.array.size() != 3) {
            err = "level: elevation entries are [x, z, height]";
            return false;
        }
        m_controls.push_back(Float3(float(p.array[0].number), float(p.array[2].number), float(p.array[1].number)));
    }
    if (m_controls.size() < 4) {
        err = "level: at least 4 elevation control points are needed";
        return false;
    }
    for (const json::Value& p : doc["peaks"].array)
        m_peaks.push_back({float(p["x"].numberOr(0)), float(p["z"].numberOr(0)), float(p["height"].numberOr(0)),
                           std::max(1.0f, float(p["radius"].numberOr(50)))});
    const json::Value& coast = doc["coast"];
    m_seaLevel = float(coast["seaLevel"].numberOr(0));
    m_maxDepth = float(coast["maxDepth"].numberOr(20));
    m_cliffRise = float(coast["cliffRise"].numberOr(1.0));
    m_beachRise = float(coast["beachRise"].numberOr(0.035));
    for (const json::Value& poly : coast["land"].array) m_land.push_back(readPolygon(poly));
    if (m_land.empty()) {
        err = "level: coast.land needs at least one polygon";
        return false;
    }
    for (const json::Value& b : coast["beaches"].array)
        m_beaches.push_back({float(b["x"].numberOr(0)), float(b["z"].numberOr(0)), float(b["radius"].numberOr(50))});
    for (const json::Value& l : doc["lakes"].array) {
        Lake lake{l["name"].stringOr("lake"), float(l["level"].numberOr(0)), float(l["depth"].numberOr(4)),
                  readPolygon(l["polygon"]), std::max(0.0f, float(l["shore"].numberOr(0)))};
        if (lake.poly.p.size() < 3) {
            err = "level: lake " + lake.name + " needs a polygon";
            return false;
        }
        lake.poly = smoothedPolygon(std::move(lake.poly), 3);
        m_lakes.push_back(std::move(lake));
    }
    for (const json::Value& r : doc["rivers"].array) {
        RiverDef d{r["name"].stringOr("river"), float(r["width"].numberOr(8)), float(r["depth"].numberOr(1)),
                   std::max(0.1f, float(r["bankSlope"].numberOr(0.7))), {}};
        for (const json::Value& p : r["points"].array) {
            float x, z, y;
            if (!readPoint(p, x, z, &y)) continue;
            d.points.push_back(Float3(x, y, z));
        }
        if (d.points.size() < 2) {
            err = "level: river " + d.name + " needs at least 2 points";
            return false;
        }
        m_riverDefs.push_back(std::move(d));
    }
    for (const json::Value& r : doc["roads"].array) {
        RoadDef d{r["name"].stringOr("road"), r["kind"].stringOr("main") == "trail", float(r["width"].numberOr(6)),
                  float(r["maxGrade"].numberOr(0.12)), {}, {}, surfaceFromKey(r["surface"].stringOr("dirt"))};
        for (const json::Value& p : r["points"].array) {
            float x, z;
            if (readPoint(p, x, z)) d.points.push_back(Float3(x, kNaN, z));
        }
        for (const json::Value& b : r["bridges"].array) {
            BridgeDef bd{};
            if (readPoint(b["from"], bd.x0, bd.z0) && readPoint(b["to"], bd.x1, bd.z1)) d.bridges.push_back(bd);
        }
        if (d.surface != SurfaceId::PackedDirt && d.surface != SurfaceId::Gravel && d.surface != SurfaceId::Asphalt) {
            err = "level: road " + d.name + " surface must be dirt, gravel or asphalt";
            return false;
        }
        if (d.points.size() < 2 || d.width < 2.0f || d.grade <= 0.0f) {
            err = "level: road " + d.name + " needs 2 points, width >= 2 m and maxGrade > 0";
            return false;
        }
        m_roadDefs.push_back(std::move(d));
    }
    for (const json::Value& f : doc["flats"].array) {
        Flat fl{f["name"].stringOr("flat"), float(f["x"].numberOr(0)), float(f["z"].numberOr(0)),
                float(f["radius"].numberOr(10)), float(f["height"].numberOr(kNaN)), float(f["fade"].numberOr(20)),
                surfaceFromKey(f["surface"].stringOr(""))};
        if (!f.has("height")) fl.h = kNaN;
        m_flats.push_back(fl);
    }
    for (const json::Value& f : doc["forests"].array) m_forests.push_back({readPolygon(f["polygon"]), float(f["density"].numberOr(0.5))});
    m_meadowTrees = float(doc["meadowTreeDensity"].numberOr(0.03));
    for (const json::Value& f : doc["fields"].array) m_fields.push_back({readPolygon(f["polygon"])});
    for (const json::Value& p : doc["pois"].array) {
        LevelPoi poi;
        poi.id = int(p["id"].numberOr(0));
        poi.name = p["name"].stringOr("POI");
        poi.kind = p["kind"].stringOr("");
        poi.x = float(p["x"].numberOr(0));
        poi.z = float(p["z"].numberOr(0));
        if (!readPoint(p["at"], poi.atX, poi.atZ)) {
            poi.atX = poi.x;
            poi.atZ = poi.z;
        }
        L.m_pois.push_back(poi);
    }
    const json::Value& route = doc["attractRoute"];
    m_routeName = route["name"].stringOr("attract route");
    m_routeSpeed = float(route["speedKmh"].numberOr(32.0)) / 3.6f;
    for (const json::Value& leg : route["legs"].array) {
        RouteLeg l{leg["road"].stringOr(""), 0, 0, 0, 0};
        if (!readPoint(leg["from"], l.x0, l.z0) || !readPoint(leg["to"], l.x1, l.z1)) {
            err = "level: attractRoute leg on " + l.road + " needs from and to";
            return false;
        }
        m_routeLegs.push_back(l);
    }
    const json::Value& sp = doc["spawn"];
    L.m_spawnX = float(sp["x"].numberOr(0));
    L.m_spawnZ = float(sp["z"].numberOr(0));
    if (!readPoint(sp["towards"], L.m_spawnTowardX, L.m_spawnTowardZ)) {
        L.m_spawnTowardX = L.m_spawnX;
        L.m_spawnTowardZ = L.m_spawnZ + 1.0f;
    }
    if (std::abs(L.m_spawnX) > L.m_playHalf || std::abs(L.m_spawnZ) > L.m_playHalf) {
        err = "level: spawn outside the play area";
        return false;
    }
    return loadModels(doc["vegetation"], err);
}

bool LevelBaker::loadModels(const json::Value& veg, std::string& err) {
    if (veg.isNull()) return true;
    if (!veg.isObject()) {
        err = "level: vegetation must be an object";
        return false;
    }
    if (m_baseDir.empty()) return true; // no file location (text-only bake): procedural trees and rocks
    const std::string vdir = veg["dir"].stringOr("../../models");
    const bool absolute = !vdir.empty() && (vdir[0] == '/' || (vdir.size() > 1 && vdir[1] == ':'));
    const std::string dir = (absolute ? vdir : m_baseDir + "/" + vdir) + "/";
    const std::pair<const char*, std::vector<int>*> roles[] = {
        {"conifers", &m_conifers}, {"saplings", &m_saplings}, {"broadleaf", &m_broadleaf}, {"bushes", &m_bushes},
        {"ferns", &m_ferns},       {"debris", &m_debris},     {"rocks", &m_rockModels},   {"logs", &m_logs},
        {"cliffs", &m_cliffModels}};
    std::string warning;
    for (const auto& [key, list] : roles) {
        for (const json::Value& v : veg[key].array) {
            if (v.type != json::Value::Type::String) {
                err = std::string("level: vegetation.") + key + " entries must be model paths";
                return false;
            }
            LevelModel m;
            m.path = dir + v.string + ".glb";
            ModelAsset a;
            std::string e;
            if (!a.load(m.path, false, &e)) {
                warning = e;
                break;
            }
            m.id = a.id;
            m.kind = a.kind;
            m.height = a.height;
            m.radius = a.radius;
            m.trunkRadius = a.trunkRadius;
            m.trunkTop = Float3(a.trunkTop[0], a.trunkTop[1], a.trunkTop[2]);
            m.boundsMin = Float3(a.boundsMin[0], a.boundsMin[1], a.boundsMin[2]);
            m.boundsMax = Float3(a.boundsMax[0], a.boundsMax[1], a.boundsMax[2]);
            if (m.kind == "rock" || m.kind == "log" || m.kind == "cliff") {
                if (a.collisionIndices.empty() || a.collisionLod < 0 || std::size_t(a.collisionLod) >= a.lodError.size()) {
                    warning = m.path + ": no collision mesh";
                    break;
                }
                for (std::uint32_t i : a.collisionIndices)
                    m.collision.push_back(Float3(a.collisionPositions[3 * i], a.collisionPositions[3 * i + 1],
                                                 a.collisionPositions[3 * i + 2]));
                m.collisionLod = a.collisionLod;
                m.collisionError = a.lodError[std::size_t(a.collisionLod)];
            }
            if ((m.kind == "tree") != (list == &m_conifers || list == &m_saplings || list == &m_broadleaf) ||
                ((m.kind == "rock") != (list == &m_rockModels)) || ((m.kind == "cliff") != (list == &m_cliffModels))) {
                err = "level: " + m.path + " is a " + m.kind + ", not one of the " + key;
                return false;
            }
            list->push_back(int(L.m_models.size()));
            m_modelByRef[v.string] = int(L.m_models.size());
            L.m_models.push_back(std::move(m));
        }
        if (!warning.empty()) break;
    }
    if (!warning.empty() || m_conifers.empty()) {
        // Missing library (for example Git LFS objects not pulled): the procedural trees and rocks stand in.
        L.m_modelError = warning.empty() ? "vegetation: no conifers" : warning;
        L.m_models.clear();
        for (const auto& [key, list] : roles) list->clear();
        m_modelByRef.clear();
    }
    return true;
}

// ------------------------------------------------------------------------------------------------------------------
void LevelBaker::bakeBase() {
    // Coarse normalized Gaussian interpolation of the control points on an 8 m grid, then bicubic upsampling.
    const float cstep = 8.0f;
    const int cn = int(std::lround(2.0f * m_half / cstep)) + 1;
    std::vector<float> coarse(std::size_t(cn) * std::size_t(cn));
    const float sigma = 190.0f, inv2s2 = 1.0f / (2.0f * sigma * sigma);
    parallelFor(cn, m_threads, [&](int j) {
        for (int i = 0; i < cn; ++i) {
            const float x = m_origin + float(i) * cstep, z = m_origin + float(j) * cstep;
            double sw = 0.0, sh = 0.0;
            for (const Float3& c : m_controls) {
                const float d2 = (x - c.x) * (x - c.x) + (z - c.z) * (z - c.z);
                const double w = std::exp(-double(d2) * inv2s2) + 1e-7 / (1.0 + double(d2));
                sw += w;
                sh += w * double(c.y);
            }
            coarse[std::size_t(j) * std::size_t(cn) + std::size_t(i)] = float(sh / sw);
        }
    });
    auto C = [&](int i, int j) {
        return coarse[std::size_t(std::clamp(j, 0, cn - 1)) * std::size_t(cn) + std::size_t(std::clamp(i, 0, cn - 1))];
    };
    auto cubic = [](float a, float b, float c, float d, float t) {
        return b + 0.5f * t * (c - a + t * (2.0f * a - 5.0f * b + 4.0f * c - d + t * (3.0f * (b - c) + d - a)));
    };
    parallelFor(m_n, m_threads, [&](int j) {
        for (int i = 0; i < m_n; ++i) {
            const float x = X(i), z = X(j);
            const float fx = (x - m_origin) / cstep, fz = (z - m_origin) / cstep;
            const int ci = int(std::floor(fx)), cj = int(std::floor(fz));
            const float tx = fx - float(ci), tz = fz - float(cj);
            float row[4];
            for (int k = 0; k < 4; ++k)
                row[k] = cubic(C(ci - 1, cj - 1 + k), C(ci, cj - 1 + k), C(ci + 1, cj - 1 + k), C(ci + 2, cj - 1 + k), tx);
            float base = cubic(row[0], row[1], row[2], row[3], tz);
            for (const Peak& p : m_peaks) {
                const float d2 = ((x - p.x) * (x - p.x) + (z - p.z) * (z - p.z)) / (p.r * p.r);
                if (d2 < 9.0f) base += p.h * std::exp(-d2);
            }
            const float m = smoothstep(m_mtLow, m_mtHigh, base);
            float h = base;
            if (m > 0.0f) h += m * m_ridgeAmp * (ridged(x / m_ridgeScale, z / m_ridgeScale, 5, m_seed + 11u) - 0.45f);
            h += m_hillAmp * 2.0f * (fbm(x / m_hillScale, z / m_hillScale, 4, m_seed + 23u) - 0.5f);
            h += m_fineAmp * 2.0f * (fbm(x / m_fineScale, z / m_fineScale, 3, m_seed + 37u) - 0.5f);
            m_h[idx(i, j)] = h;
        }
    });
}

void LevelBaker::bakeCoast() {
    const float step = 2.0f;
    m_cn = int(std::lround(2.0f * m_half / step)) + 1;
    m_coastD.assign(std::size_t(m_cn) * std::size_t(m_cn), 0.0f);
    parallelFor(m_cn, m_threads, [&](int j) {
        for (int i = 0; i < m_cn; ++i) {
            const float x = m_origin + float(i) * step, z = m_origin + float(j) * step;
            float d = -1e30f;
            for (const Polygon& p : m_land) d = std::max(d, p.signedDistance(x, z));
            m_coastD[std::size_t(j) * std::size_t(m_cn) + std::size_t(i)] = d;
        }
    });
    parallelFor(m_n, m_threads, [&](int j) {
        for (int i = 0; i < m_n; ++i) {
            const float x = X(i), z = X(j);
            const float d = coastDistance(x, z);
            if (d > 400.0f) continue;
            const float b = beachFactor(x, z);
            float& h = m_h[idx(i, j)];
            if (d >= 0.0f) {
                const float rise = lerp(m_cliffRise, m_beachRise, b);
                // The cap fades out toward the 400 m reach instead of ending in a cliff where high ground is cut.
                const float cap = m_seaLevel + 0.4f + d * rise;
                h = std::max(0.4f + 0.012f * d + m_seaLevel, lerp(std::min(h, cap), h, smoothstep(220.0f, 400.0f, d)));
            } else {
                const float slope = lerp(0.3f, 0.03f, b);
                const float floor = std::max(-m_maxDepth, 0.4f + d * slope);
                h = m_seaLevel + floor + 0.8f * (fbm(x / 30.0f, z / 30.0f, 3, m_seed + 41u) - 0.5f) * smoothstep(0.0f, -20.0f, d);
            }
        }
    });
}

void LevelBaker::bakeFlats() {
    for (Flat& f : m_flats) {
        if (std::isnan(f.h)) f.h = sampleGrid(m_h, f.x, f.z);
        const float reach = f.r + f.fade;
        const int i0 = std::max(0, int((f.x - reach - m_origin) / m_cell)), i1 = std::min(m_n - 1, int((f.x + reach - m_origin) / m_cell) + 1);
        const int j0 = std::max(0, int((f.z - reach - m_origin) / m_cell)), j1 = std::min(m_n - 1, int((f.z + reach - m_origin) / m_cell) + 1);
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                const float d = std::hypot(X(i) - f.x, X(j) - f.z);
                const float w = 1.0f - smoothstep(f.r, reach, d);
                if (w <= 0.0f) continue;
                float& h = m_h[idx(i, j)];
                h = lerp(h, f.h, w);
                if (f.surface != SurfaceId::Count && d < f.r + 2.0f) m_flatSurface[idx(i, j)] = f.surface;
            }
    }
}

void LevelBaker::bakeLakes() {
    for (std::size_t k = 0; k < m_lakes.size(); ++k) {
        const Lake& lake = m_lakes[k];
        const float lip = 12.0f, embankment = 60.0f, pad = std::max(lip + embankment, lake.shore);
        const int i0 = std::max(0, int((lake.poly.x0 - pad - m_origin) / m_cell)), i1 = std::min(m_n - 1, int((lake.poly.x1 + pad - m_origin) / m_cell) + 1);
        const int j0 = std::max(0, int((lake.poly.z0 - pad - m_origin) / m_cell)), j1 = std::min(m_n - 1, int((lake.poly.z1 + pad - m_origin) / m_cell) + 1);
        parallelFor(j1 - j0 + 1, m_threads, [&](int jj) {
            const int j = j0 + jj;
            for (int i = i0; i <= i1; ++i) {
                const float x = X(i), z = X(j);
                const float d = lake.poly.signedDistance(x, z);
                float& h = m_h[idx(i, j)];
                if (d > 0.0f) {
                    const float bed = lake.level + 0.3f - std::min(lake.depth + 0.3f, d * 0.2f) +
                                      0.4f * (fbm(x / 12.0f, z / 12.0f, 2, m_seed + 53u) - 0.5f) * smoothstep(2.0f, 10.0f, d);
                    // Where the land inside the outline is already deep, the bed still shelves down from the
                    // shore at most ~27 deg instead of dropping off at the outline.
                    h = std::max(std::min(h, bed), lake.level + 0.3f - d * 0.5f);
                    m_lakeId[idx(i, j)] = std::uint8_t(k + 1);
                } else if (-d < pad) {
                    // Shore band: higher ground eases down to the waterline (about 7 deg at the water) instead
                    // of meeting the lake in a cut; then the low lip that keeps the water in. Where the land
                    // outside is lower than the lip, it falls away as an uneven embankment of about 15 to 27 deg.
                    if (-d < lake.shore) {
                        const float lowered = std::min(h, lake.level + 0.3f - d * 0.12f);
                        h = lowered + (h - lowered) * smoothstep(0.0f, lake.shore, -d);
                    }
                    // Lakes in a gorge: the walls rise from the outline at most ~63 deg, not as a step.
                    h = std::min(h, lake.level + 1.0f - 2.0f * d);
                    const float out = std::max(0.0f, -d - lip);
                    const float fall = 0.27f + 0.24f * fbm(x / 22.0f, z / 22.0f, 3, m_seed + 57u);
                    const float lump = 0.6f * (fbm(x / 7.0f, z / 7.0f, 2, m_seed + 59u) - 0.5f) * smoothstep(0.0f, 3.0f, out);
                    h = std::max(h, lake.level + 0.3f - 0.06f * std::min(-d, lip) - fall * out + lump);
                }
            }
        });
    }
}

void LevelBaker::bakeRivers() {
    std::vector<float> carved;
    for (const RiverDef& def : m_riverDefs) {
        LevelRiver river;
        river.name = def.name;
        river.width = def.width;
        river.depth = def.depth;
        river.bankSlope = def.bank;
        const std::vector<Float3> pts = level_math::resampleSpline(def.points, 2.0f);
        const float hw = 0.5f * def.width;
        const std::size_t n = pts.size();
        // Automatic surface: below the lowest terrain across the channel, never rising downstream; pins exact.
        std::vector<float> autoY(n), pin(n), y(n);
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t a = i > 0 ? i - 1 : i, b = i + 1 < n ? i + 1 : i;
            float dx = pts[b].x - pts[a].x, dz = pts[b].z - pts[a].z;
            const float len = std::max(1e-4f, std::hypot(dx, dz));
            dx /= len;
            dz /= len;
            float lo = sampleGrid(m_h, pts[i].x, pts[i].z);
            for (float o : {-hw, -0.5f * hw, 0.5f * hw, hw})
                lo = std::min(lo, sampleGrid(m_h, pts[i].x + dz * o, pts[i].z - dx * o));
            autoY[i] = lo - 0.7f;
            pin[i] = pts[i].y;
        }
        std::vector<float> nextPin(n, -1e30f);
        float np = -1e30f;
        for (std::size_t i = n; i-- > 0;) {
            if (!std::isnan(pin[i])) np = pin[i];
            nextPin[i] = np;
        }
        for (std::size_t i = 0; i < n; ++i) {
            if (!std::isnan(pin[i])) y[i] = pin[i];
            else y[i] = std::max(i > 0 ? std::min(y[i - 1], autoY[i]) : autoY[i], nextPin[i]);
        }
        for (std::size_t i = 0; i < n; ++i) river.samples.push_back({pts[i].x, pts[i].z, y[i]});
        for (std::size_t i = 1; i < n; ++i)
            if (y[i - 1] - y[i] >= 2.0f)
                L.m_waterfalls.push_back({pts[i - 1].x, pts[i - 1].z, y[i - 1], pts[i].x, pts[i].z, y[i], def.width});

        // Carve: channel bed, then banks rising at bankSlope until they meet the terrain. Steep banks (bankSlope
        // >= 1.5) get a scree apron at the foot, spurs and gullies (the slope varies along the wall), ledges and a
        // rounded rim. The bank line wanders a few meters off the spline so the walls are not parallel.
        const bool steep = def.bank >= 1.5f;
        const float reach = std::clamp((steep ? 64.0f : 32.0f) / def.bank, 10.0f, 60.0f) + (steep ? 9.0f : 3.0f);
        const float waterReach = hw + 0.25f / def.bank + 1.0f;
        carved.assign(m_h.size(), 1e30f);
        const int bands = (m_n + 15) / 16;
        parallelFor(bands, m_threads, [&](int band) {
            const int j0 = band * 16, j1 = std::min(m_n - 1, j0 + 15);
            const float zb0 = X(j0) - hw - reach, zb1 = X(j1) + hw + reach;
            for (std::size_t s = 0; s + 1 < n; ++s) {
                const LevelRiverSample &a = river.samples[s], &b = river.samples[s + 1];
                if (std::max(a.z, b.z) < zb0 || std::min(a.z, b.z) > zb1) continue;
                const float ex = b.x - a.x, ez = b.z - a.z, el2 = std::max(1e-6f, ex * ex + ez * ez);
                const float r = hw + reach;
                const int i0 = std::max(0, int((std::min(a.x, b.x) - r - m_origin) / m_cell));
                const int i1 = std::min(m_n - 1, int((std::max(a.x, b.x) + r - m_origin) / m_cell) + 1);
                const int jj0 = std::max(j0, int((std::min(a.z, b.z) - r - m_origin) / m_cell));
                const int jj1 = std::min(j1, int((std::max(a.z, b.z) + r - m_origin) / m_cell) + 1);
                for (int j = jj0; j <= jj1; ++j)
                    for (int i = i0; i <= i1; ++i) {
                        const float px = X(i) - a.x, pz = X(j) - a.z;
                        const float t = clamp01((px * ex + pz * ez) / el2);
                        const float d = std::hypot(px - ex * t, pz - ez * t);
                        if (d > r) continue;
                        const float ys = lerp(a.y, b.y, t);
                        const float x = X(i), z = X(j);
                        const float e = d - hw + (steep ? 6.0f : 3.0f) * (fbm(x / 34.0f, z / 34.0f, 2, m_seed + 73u) - 0.5f) *
                                                     smoothstep(0.0f, 4.0f, d - hw);
                        float rise = e * def.bank;
                        if (steep && e > 0.0f) {
                            // Scree apron of 1.5 to 5.5 m at about 37 deg, then the wall; steep walls are terraced:
                            // a ledge about every 16 m of height, steeper rock between.
                            const float apron = 1.5f + 4.0f * fbm(x / 18.0f, z / 18.0f, 2, m_seed + 79u);
                            const float bank = def.bank * (0.75f + 0.5f * ridged(x / 21.0f, z / 21.0f, 2, m_seed + 83u));
                            const float wall = std::max(0.0f, e - apron) * bank;
                            const float band = 16.0f;
                            const float phase = fbm(x / 45.0f, z / 45.0f, 2, m_seed + 71u) * 1.7f;
                            const auto steps = [](float q) {
                                const float f = q - std::floor(q);
                                return std::floor(q) + 0.18f * f + 0.82f * smoothstep(0.45f, 0.95f, f);
                            };
                            rise = 0.75f * std::min(e, apron) + band * (steps(wall / band + phase) - steps(phase));
                        }
                        const float carve = d < hw ? ys - 0.25f - (def.depth - 0.25f) * (1.0f - (d / hw) * (d / hw))
                                                   : ys - 0.25f + rise;
                        const std::size_t k = idx(i, j);
                        carved[k] = std::min(carved[k], carve);
                        if (d < m_riverD[k]) {
                            m_riverD[k] = d;
                            m_riverY[k] = ys;
                            m_riverReach[k] = waterReach;
                        }
                    }
            }
        });
        // Rounded rim: a smooth minimum of the terrain and the carved profile (radius up to ~4 m on steep walls).
        const float rim = steep ? 6.0f : 2.0f;
        parallelFor(m_n, m_threads, [&](int j) {
            for (int i = 0; i < m_n; ++i) {
                const std::size_t k = idx(i, j);
                if (carved[k] > 1e29f) continue;
                const float a = m_h[k], b = carved[k];
                const float q = std::max(rim - std::abs(a - b), 0.0f);
                m_h[k] = std::min(a, b) - q * q / (4.0f * rim);
            }
        });
        L.m_rivers.push_back(std::move(river));
    }
}

void LevelBaker::bakeBankSmoothing(const std::vector<float>& uncarved) {
    if (m_lakes.empty() && m_riverDefs.empty()) return;
    // Smooth the carved banks: the per-segment river cones meet in a crease at every sample on the inside of a
    // bend, lakes end in a cut at their outline, and banks meet the terrain in a hard rim. River channels and
    // lake water deeper than 0.5 m keep their exact profile.
    std::vector<std::uint8_t> region(m_h.size(), 0);
    parallelFor(m_n, m_threads, [&](int j) {
        for (int i = 0; i < m_n; ++i) {
            const std::size_t k = idx(i, j);
            if (m_riverD[k] < m_riverReach[k] + 1.0f) continue;
            if (m_lakeId[k] && m_h[k] < m_lakes[std::size_t(m_lakeId[k] - 1)].level - 0.5f) continue;
            bool near = false;
            for (int dj = -2; dj <= 2 && !near; ++dj)
                for (int di = -2; di <= 2 && !near; ++di) {
                    const int ii = std::clamp(i + di, 0, m_n - 1), jj = std::clamp(j + dj, 0, m_n - 1);
                    near = uncarved[idx(ii, jj)] - m_h[idx(ii, jj)] > 0.02f;
                }
            region[k] = near ? 1 : 0;
        }
    });
    for (int pass = 0; pass < 2; ++pass) {
        const std::vector<float> src = m_h;
        parallelFor(m_n, m_threads, [&](int j) {
            for (int i = 0; i < m_n; ++i) {
                const std::size_t k = idx(i, j);
                if (!region[k]) continue;
                float sum = 0.0f;
                for (int dj = -1; dj <= 1; ++dj)
                    for (int di = -1; di <= 1; ++di)
                        sum += src[idx(std::clamp(i + di, 0, m_n - 1), std::clamp(j + dj, 0, m_n - 1))];
                m_h[k] = sum / 9.0f;
            }
        });
    }
}

void LevelBaker::bakeRoads() {
    // Provisional water for the ford pins (the roads below change the beds only inside their width).
    auto waterAtSample = [&](float x, float z, float h) -> float {
        const int i = std::clamp(int(std::lround((x - m_origin) / m_cell)), 0, m_n - 1);
        const int j = std::clamp(int(std::lround((z - m_origin) / m_cell)), 0, m_n - 1);
        const std::size_t k = idx(i, j);
        float w = kNaN;
        if (m_riverD[k] < m_riverReach[k] && h < m_riverY[k]) w = m_riverY[k];
        else if (m_lakeId[k] && h < m_lakes[m_lakeId[k] - 1].level) w = m_lakes[m_lakeId[k] - 1].level;
        else if (h < m_seaLevel && coastDistance(x, z) < 80.0f) w = m_seaLevel;
        return w;
    };

    const float ds = 2.0f;
    for (const RoadDef& def : m_roadDefs) {
        LevelRoad road;
        road.name = def.name;
        road.trail = def.trail;
        road.surface = def.surface;
        road.width = def.width;
        road.maxGrade = def.grade;
        const std::vector<Float3> pts = level_math::resampleSpline(def.points, ds);
        const std::size_t n = pts.size();
        road.samples.resize(n);
        float s = 0.0f;
        for (std::size_t i = 0; i < n; ++i) {
            RoadSample& r = road.samples[i];
            r.x = pts[i].x;
            r.z = pts[i].z;
            if (i > 0) s += std::hypot(r.x - pts[i - 1].x, r.z - pts[i - 1].z);
            r.s = s;
            const std::size_t a = i > 0 ? i - 1 : i, b = i + 1 < n ? i + 1 : i;
            const float dx = pts[b].x - pts[a].x, dz = pts[b].z - pts[a].z, len = std::max(1e-4f, std::hypot(dx, dz));
            r.dx = dx / len;
            r.dz = dz / len;
        }
        const float step = n > 1 ? s / float(n - 1) : ds;
        // Target: terrain under the road, smoothed over +-20 m.
        std::vector<float> h0(n), target(n), pin(n, kNaN);
        for (std::size_t i = 0; i < n; ++i) h0[i] = sampleGrid(m_h, road.samples[i].x, road.samples[i].z);
        const int r = 10;
        for (std::size_t i = 0; i < n; ++i) {
            float sum = 0.0f;
            int c = 0;
            for (int k = -r; k <= r; ++k) {
                const long j = long(i) + k;
                if (j < 0 || j >= long(n)) continue;
                sum += h0[std::size_t(j)];
                ++c;
            }
            target[i] = sum / float(c);
        }
        // Fords: water under the center line pins the road just below the surface.
        const float fordDepth = def.trail ? 0.3f : 0.25f;
        for (std::size_t i = 0; i < n; ++i) {
            const float w = waterAtSample(road.samples[i].x, road.samples[i].z, h0[i]);
            if (!std::isnan(w)) {
                pin[i] = w - fordDepth;
                road.samples[i].ford = true;
            }
        }
        // Bridges: the deck runs straight between the bank heights at both ends.
        auto nearest = [&](float x, float z) {
            std::size_t best = 0;
            float bd = 1e30f;
            for (std::size_t i = 0; i < n; ++i) {
                const float d = std::hypot(road.samples[i].x - x, road.samples[i].z - z);
                if (d < bd) {
                    bd = d;
                    best = i;
                }
            }
            return best;
        };
        for (const BridgeDef& b : def.bridges) {
            std::size_t i0 = nearest(b.x0, b.z0), i1 = nearest(b.x1, b.z1);
            if (i0 > i1) std::swap(i0, i1);
            const float y0 = h0[i0] + 0.05f, y1 = h0[i1] + 0.05f;
            for (std::size_t i = i0; i <= i1; ++i) {
                const float t = i1 > i0 ? float(i - i0) / float(i1 - i0) : 0.0f;
                pin[i] = lerp(y0, y1, t);
                road.samples[i].bridge = i > i0 && i < i1;
                road.samples[i].ford = false;
            }
            LevelBridge lb{road.samples[i0].x, road.samples[i0].z, road.samples[i1].x, road.samples[i1].z, y0, y1,
                           0.5f * def.width + 0.4f};
            L.m_bridges.push_back(lb);
        }
        // Junctions: the ends of a road meet the earlier roads at their profile height.
        for (std::size_t end : {std::size_t(0), n - 1}) {
            float bd = 10.0f;
            float y = kNaN;
            for (const LevelRoad& other : L.m_roads)
                for (const RoadSample& o : other.samples) {
                    const float d = std::hypot(o.x - road.samples[end].x, o.z - road.samples[end].z);
                    if (d < bd) {
                        bd = d;
                        y = o.y;
                    }
                }
            if (!std::isnan(y)) pin[end] = y;
        }
        const std::vector<float> y = level_math::gradeLimitedProfile(target, pin, step, def.grade);
        for (std::size_t i = 0; i < n; ++i) road.samples[i].y = y[i];
        // Banking from the signed curvature over +-8 m: the inside of a bend is lower, at most 4 degrees.
        std::vector<float> bank(n, 0.0f);
        const int kb = 4;
        for (std::size_t i = 0; i < n; ++i) {
            const RoadSample& a = road.samples[i >= std::size_t(kb) ? i - kb : 0];
            const RoadSample& b = road.samples[std::min(n - 1, i + kb)];
            const float cross = a.dx * b.dz - a.dz * b.dx, dot = a.dx * b.dx + a.dz * b.dz;
            const float span = std::max(1.0f, b.s - a.s);
            const float kappa = std::atan2(cross, dot) / span;
            bank[i] = std::clamp(kappa * 160.0f, -4.0f, 4.0f) * kPi / 180.0f;
        }
        for (std::size_t i = 0; i < n; ++i) {
            float sum = 0.0f;
            int c = 0;
            bool flat = false;
            for (int k = -6; k <= 6; ++k) {
                const long j = long(i) + k;
                if (j < 0 || j >= long(n)) continue;
                sum += bank[std::size_t(j)];
                ++c;
                flat = flat || road.samples[std::size_t(j)].bridge || road.samples[std::size_t(j)].ford;
            }
            road.samples[i].bank = flat ? 0.0f : sum / float(c);
        }
        L.m_roads.push_back(std::move(road));
    }

    // Carve every road into the terrain. Roads are split into ~60 m pieces; per sample the nearest segment of each
    // piece gives a height and a weight (1 on the road, falling to 0 across the shoulder), and the pieces blend by
    // weight^16, so switchback legs and junctions meet without steps.
    struct Piece { std::size_t road, s0, s1; float x0, z0, x1, z1; };
    std::vector<Piece> pieces;
    const float maxReach = 48.0f;
    for (std::size_t r = 0; r < L.m_roads.size(); ++r) {
        const auto& sm = L.m_roads[r].samples;
        // Pieces overlap by 10 samples on both sides, so every point whose nearest point on a piece is that piece's
        // inner end (a joint, not a road end) is covered by the neighbour piece; such points ignore the piece.
        for (std::size_t s0 = 0; s0 + 1 < sm.size(); s0 += 30) {
            Piece p{r, s0 >= 10 ? s0 - 10 : 0, std::min(sm.size() - 1, s0 + 40), 1e9f, 1e9f, -1e9f, -1e9f};
            for (std::size_t s = p.s0; s <= p.s1; ++s) {
                p.x0 = std::min(p.x0, sm[s].x);
                p.x1 = std::max(p.x1, sm[s].x);
                p.z0 = std::min(p.z0, sm[s].z);
                p.z1 = std::max(p.z1, sm[s].z);
            }
            const float reach = 0.5f * L.m_roads[r].width + maxReach;
            p.x0 -= reach;
            p.x1 += reach;
            p.z0 -= reach;
            p.z1 += reach;
            pieces.push_back(p);
        }
    }
    const std::vector<float> h0 = m_h;
    const int bands = (m_n + 15) / 16;
    parallelFor(bands, m_threads, [&](int band) {
        const int j0 = band * 16, j1 = std::min(m_n - 1, j0 + 15);
        const float zb0 = X(j0), zb1 = X(j1);
        const std::size_t rowN = std::size_t(m_n);
        std::vector<float> sumW(rowN * 16, 0.0f), sumWY(rowN * 16, 0.0f), maxW(rowN * 16, 0.0f);
        std::vector<float> pd(rowN * 16), py(rowN * 16);
        std::vector<std::uint8_t> pend(rowN * 16);
        for (const Piece& p : pieces) {
            if (p.z1 < zb0 || p.z0 > zb1) continue;
            const LevelRoad& road = L.m_roads[p.road];
            const float hw = 0.5f * road.width;
            const bool innerStart = p.s0 > 0, innerEnd = p.s1 + 1 < road.samples.size();
            const int i0 = std::max(0, int((p.x0 - m_origin) / m_cell)), i1 = std::min(m_n - 1, int((p.x1 - m_origin) / m_cell) + 1);
            const int jj0 = std::max(j0, int((p.z0 - m_origin) / m_cell)), jj1 = std::min(j1, int((p.z1 - m_origin) / m_cell) + 1);
            if (jj0 > jj1) continue;
            for (int j = jj0; j <= jj1; ++j)
                for (int i = i0; i <= i1; ++i) pd[std::size_t(j - j0) * rowN + std::size_t(i)] = 1e30f;
            for (std::size_t s = p.s0; s < p.s1; ++s) {
                const RoadSample &a = road.samples[s], &b = road.samples[s + 1];
                if (a.bridge && b.bridge) continue;
                const float ex = b.x - a.x, ez = b.z - a.z, el2 = std::max(1e-6f, ex * ex + ez * ez);
                const float rr = hw + maxReach;
                const int si0 = std::max(i0, int((std::min(a.x, b.x) - rr - m_origin) / m_cell));
                const int si1 = std::min(i1, int((std::max(a.x, b.x) + rr - m_origin) / m_cell) + 1);
                const int sj0 = std::max(jj0, int((std::min(a.z, b.z) - rr - m_origin) / m_cell));
                const int sj1 = std::min(jj1, int((std::max(a.z, b.z) + rr - m_origin) / m_cell) + 1);
                for (int j = sj0; j <= sj1; ++j)
                    for (int i = si0; i <= si1; ++i) {
                        const float px = X(i) - a.x, pz = X(j) - a.z;
                        const float t = clamp01((px * ex + pz * ez) / el2);
                        const float qx = px - ex * t, qz = pz - ez * t;
                        const float d = std::hypot(qx, qz);
                        const std::size_t k = std::size_t(j - j0) * rowN + std::size_t(i);
                        if (d >= pd[k]) continue;
                        pd[k] = d;
                        pend[k] = (innerStart && s == p.s0 && t <= 0.0f) || (innerEnd && s + 1 == p.s1 && t >= 1.0f);
                        const float dx = lerp(a.dx, b.dx, t), dz = lerp(a.dz, b.dz, t);
                        // Lateral offset, + to the left of the driving direction (left = (dz, -dx)).
                        const float lat = std::clamp(qx * dz - qz * dx, -hw, hw);
                        py[k] = lerp(a.y, b.y, t) + lat * std::tan(lerp(a.bank, b.bank, t));
                    }
            }
            for (int j = jj0; j <= jj1; ++j)
                for (int i = i0; i <= i1; ++i) {
                    const std::size_t k = std::size_t(j - j0) * rowN + std::size_t(i);
                    const float d = pd[k];
                    if (d > 1e29f || pend[k]) continue;
                    const std::size_t g = idx(i, j);
                    m_roadClear[g] = std::min(m_roadClear[g], d - hw);
                    const float dy = std::abs(h0[g] - py[k]);
                    const float shoulder = (road.trail ? 1.2f : 1.6f) + 1.45f * dy;
                    const float w = d <= hw ? 1.0f : 1.0f - smoothstep(0.0f, 1.0f, (d - hw) / shoulder);
                    if (w <= 0.0f) continue;
                    // weight^16: the nearest piece dominates, two legs blend only where they are about equally near.
                    float wp = w * w;
                    wp *= wp;
                    wp *= wp;
                    wp *= wp;
                    sumW[k] += wp + 1e-30f;
                    sumWY[k] += (wp + 1e-30f) * py[k];
                    maxW[k] = std::max(maxW[k], w);
                    const float roadness = 1.0f - smoothstep(hw - 0.6f, hw + 0.5f, d);
                    if (roadness > m_roadness[g]) {
                        m_roadness[g] = roadness;
                        m_roadSurface[g] = road.surface;
                    }
                    m_roadEdge[g] = std::min(m_roadEdge[g], d - hw);
                }
        }
        for (int j = j0; j <= j1; ++j)
            for (int i = 0; i < m_n; ++i) {
                const std::size_t k = std::size_t(j - j0) * rowN + std::size_t(i);
                if (sumW[k] <= 0.0f || m_baked) continue; // baked heights are final (carved in Blender)
                const std::size_t g = idx(i, j);
                m_h[g] = lerp(h0[g], sumWY[k] / sumW[k], maxW[k]);
            }
    });

    // Bridges: the deck ramps meet the carved road 3 m into the banks, and no terrain reaches above the deck surface
    // (a bank hump through the deck would be a bump the wheels hit on the deck).
    for (LevelBridge& b : L.m_bridges) {
        const float len = b.length(), dx = (b.x1 - b.x0) / len, dz = (b.z1 - b.z0) / len;
        const float r = LevelBridge::kRamp;
        b.ya = sampleGrid(m_h, b.x0 - dx * r, b.z0 - dz * r) - 0.02f;
        b.ye = sampleGrid(m_h, b.x1 + dx * r, b.z1 + dz * r) - 0.02f;
        const float reach = b.halfWidth + 1.5f; // every grid cell under the deck, with its corners
        const float xa = std::min(b.x0, b.x1) - r - reach, xb = std::max(b.x0, b.x1) + r + reach;
        const float za = std::min(b.z0, b.z1) - r - reach, zb = std::max(b.z0, b.z1) + r + reach;
        for (int j = std::max(0, int((za - m_origin) / m_cell)); j <= std::min(m_n - 1, int((zb - m_origin) / m_cell) + 1); ++j)
            for (int i = std::max(0, int((xa - m_origin) / m_cell)); i <= std::min(m_n - 1, int((xb - m_origin) / m_cell) + 1); ++i) {
                const float px = X(i) - b.x0, pz = X(j) - b.z0;
                const float along = px * dx + pz * dz, lat = std::abs(px * dz - pz * dx);
                if (along < -r || along > len + r || lat > reach) continue;
                float& h = m_h[idx(i, j)];
                // Lowest deck point within a cell of here: the interpolated terrain between samples stays below too.
                const float top = std::min({b.deckY(along - m_cell), b.deckY(along), b.deckY(along + m_cell)});
                h = std::min(h, top - 0.03f);
            }
    }
}

// Cut and fill next to the roads: where the cut of one switchback leg meets the fill of the next (or a cut meets steep
// ground), the blend between the legs can leave a wall. Off the road surface, within 40 m of a road and away from the
// river banks and lakes, neighbouring samples are relaxed to at most 1 m apart (~45 deg along the grid).
void LevelBaker::bakeCutBanks() {
    if (m_roadDefs.empty()) return;
    const std::size_t total = m_h.size();
    std::vector<std::uint8_t> free(total, 0);
    parallelFor(m_n, m_threads, [&](int j) {
        for (int i = 0; i < m_n; ++i) {
            const std::size_t k = idx(i, j);
            free[k] = m_roadClear[k] < 40.0f && m_roadClear[k] > 0.6f && m_roadness[k] <= 0.0f && m_riverD[k] > 60.0f &&
                      !m_lakeId[k] && i > 0 && j > 0 && i + 1 < m_n && j + 1 < m_n;
        }
    });
    const float limit = 1.0f * m_cell;
    std::vector<float> src = m_h;
    for (int pass = 0; pass < 64; ++pass) {
        std::atomic<int> moved{0};
        parallelFor(m_n, m_threads, [&](int j) {
            int local = 0;
            for (int i = 0; i < m_n; ++i) {
                const std::size_t k = idx(i, j);
                if (!free[k]) continue;
                const float h = src[k];
                float lo = -1e30f, hi = 1e30f;
                for (const std::size_t n : {k - 1, k + 1, k - std::size_t(m_n), k + std::size_t(m_n)}) {
                    // A free neighbour moves too, so only half of the excess is taken here.
                    const float share = free[n] ? 0.5f : 1.0f;
                    if (h - src[n] > limit) hi = std::min(hi, h - share * (h - src[n] - limit));
                    if (src[n] - h > limit) lo = std::max(lo, h + share * (src[n] - h - limit));
                }
                const float v = hi < 1e29f && lo > -1e29f ? 0.5f * (lo + hi) : hi < 1e29f ? hi : lo > -1e29f ? lo : h;
                m_h[k] = v;
                local += v != h;
            }
            moved += local;
        });
        if (moved == 0) break;
        std::swap(src, m_h);
        // Fixed cells are equal in both buffers; free ones are all rewritten each pass.
    }
    m_h = src;
}

void LevelBaker::bakeWater() {
    L.m_water.assign(m_h.size(), Level::kNoWater);
    L.m_waterKind.assign(m_h.size(), WaterKind::None);
    parallelFor(m_n, m_threads, [&](int j) {
        for (int i = 0; i < m_n; ++i) {
            const std::size_t k = idx(i, j);
            const float h = m_h[k];
            if (m_riverD[k] < m_riverReach[k] && h < m_riverY[k]) {
                L.m_water[k] = m_riverY[k];
                L.m_waterKind[k] = WaterKind::River;
            } else if (m_lakeId[k] && h < m_lakes[m_lakeId[k] - 1].level) {
                L.m_water[k] = m_lakes[m_lakeId[k] - 1].level;
                L.m_waterKind[k] = WaterKind::Lake;
            } else if (h < m_seaLevel && coastDistance(X(i), X(j)) < 80.0f) {
                L.m_water[k] = m_seaLevel;
                L.m_waterKind[k] = WaterKind::Sea;
            }
        }
    });
    // Near-water mask (wet banks, mud): water within 3 samples, by a separable max filter.
    std::vector<std::uint8_t> wet(m_h.size(), 0), tmp(m_h.size(), 0);
    for (std::size_t k = 0; k < m_h.size(); ++k) wet[k] = L.m_water[k] > Level::kNoWater ? 255 : 0;
    const int r = 3;
    parallelFor(m_n, m_threads, [&](int j) {
        for (int i = 0; i < m_n; ++i) {
            std::uint8_t m = 0;
            for (int k = -r; k <= r; ++k) m = std::max(m, wet[idx(std::clamp(i + k, 0, m_n - 1), j)]);
            tmp[idx(i, j)] = m;
        }
    });
    parallelFor(m_n, m_threads, [&](int j) {
        for (int i = 0; i < m_n; ++i) {
            std::uint8_t m = 0;
            for (int k = -r; k <= r; ++k) m = std::max(m, tmp[idx(i, std::clamp(j + k, 0, m_n - 1))]);
            m_nearWater[idx(i, j)] = m;
        }
    });
}

void LevelBaker::bakeSurfaces() {
    const int cn = m_n - 1;
    const std::size_t cells = std::size_t(cn) * std::size_t(cn);
    L.m_surface.assign(cells, SurfaceId::Grass);
    std::vector<std::uint32_t> mask(cells), splat(cells);
    // Grass, flowers and wheat stay off the roads (none within 1.5 m of the edge, full 2.7 m out) and off the bridge
    // decks with their ramps (none within 1.5 m, full 3 m out): blades rooted on the terrain just under a deck would
    // grow through it. 1.5 m covers the bilinear lookup of the 1 m mask. Applied after the blur so it cannot bleed back.
    std::vector<float> keep(cells, 1.0f);
    m_forest.assign(cells, 0.0f);
    const float rockTan = std::tan(m_rockSlope * kPi / 180.0f);
    parallelFor(cn, m_threads, [&](int j) {
        for (int i = 0; i < cn; ++i) {
            const float x = X(i) + 0.5f * m_cell, z = X(j) + 0.5f * m_cell;
            const std::size_t a = idx(i, j), b = idx(i + 1, j), c = idx(i, j + 1), d = idx(i + 1, j + 1);
            const float h = 0.25f * (m_h[a] + m_h[b] + m_h[c] + m_h[d]);
            const float gx = 0.5f * ((m_h[b] - m_h[a]) + (m_h[d] - m_h[c])) / m_cell;
            const float gz = 0.5f * ((m_h[c] - m_h[a]) + (m_h[d] - m_h[b])) / m_cell;
            const float grad = std::hypot(gx, gz);
            // Rock by the slope over ~4 m: 1 m detail bumps alone do not turn a grassy bank into rock.
            const int il = std::max(i - 1, 0), ir = std::min(i + 2, m_n - 1), jl = std::max(j - 1, 0), jr = std::min(j + 2, m_n - 1);
            const float gx4 = 0.5f * ((m_h[idx(ir, j)] - m_h[idx(il, j)]) + (m_h[idx(ir, j + 1)] - m_h[idx(il, j + 1)])) / (float(ir - il) * m_cell);
            const float gz4 = 0.5f * ((m_h[idx(i, jr)] - m_h[idx(i, jl)]) + (m_h[idx(i + 1, jr)] - m_h[idx(i + 1, jl)])) / (float(jr - jl) * m_cell);
            const float slope4 = std::hypot(gx4, gz4);
            const float road = 0.25f * (m_roadness[a] + m_roadness[b] + m_roadness[c] + m_roadness[d]);
            float wd = 0.0f;
            WaterKind wk = WaterKind::None;
            for (std::size_t s : {a, b, c, d})
                if (L.m_water[s] > Level::kNoWater) {
                    wd += 0.25f * std::max(0.0f, L.m_water[s] - m_h[s]);
                    wk = L.m_waterKind[s];
                }
            const float near = float(std::max({m_nearWater[a], m_nearWater[b], m_nearWater[c], m_nearWater[d]})) / 255.0f;
            const SurfaceId flatS = m_flatSurface[a];
            const float coastD = coastDistance(x, z);
            const float beach = beachFactor(x, z);
            const float forest = forestDensity(x, z);
            m_forest[std::size_t(j) * std::size_t(cn) + std::size_t(i)] = forest;
            const float snowAt = snowLineAt(x, z);
            const float patch = fbm(x / 16.0f, z / 16.0f, 3, m_seed + 61u);
            const float meadow = fbm(x / 40.0f, z / 40.0f, 3, m_seed + 67u);
            bool wheat = false;
            for (const Field& f : m_fields) wheat = wheat || f.poly.inside(x, z);

            SurfaceId s = SurfaceId::Grass;
            if (road > 0.5f) {
                const SurfaceId roadS = m_roadSurface[std::max({a, b, c, d}, [&](std::size_t p, std::size_t q) {
                    return m_roadness[p] < m_roadness[q];
                })];
                if (h > snowAt) s = SurfaceId::Snow;
                else if (wd > 0.02f) s = SurfaceId::Rock; // gravel ford bar
                else if (roadS != SurfaceId::PackedDirt) s = roadS;
                else if (patch > (L.m_roads.empty() ? 1.0f : 0.70f) && grad < 0.12f) s = SurfaceId::Mud;
                else s = SurfaceId::PackedDirt;
            } else if (wd > 0.05f) {
                s = wk == WaterKind::River ? SurfaceId::Rock : wk == WaterKind::Lake ? SurfaceId::Mud : SurfaceId::Sand;
            } else if (flatS != SurfaceId::Count) {
                s = flatS;
            } else if (slope4 > rockTan) {
                s = SurfaceId::Rock;
            } else if (h > snowAt) {
                s = SurfaceId::Snow;
            } else if ((coastD < 35.0f && beach > 0.3f && h < 4.0f + m_seaLevel) || (coastD < 8.0f && h < 1.4f + m_seaLevel)) {
                s = SurfaceId::Sand;
            } else if (near > 0.5f && grad < 0.25f && patch > 0.55f) {
                s = SurfaceId::Mud;
            } else if (treeDensity(x, z, forest) > 0.55f && !wheat) {
                s = SurfaceId::PackedDirt; // needle floor under dense trees
            }
            L.m_surface[std::size_t(j) * std::size_t(cn) + std::size_t(i)] = s;

            const float rock = s == SurfaceId::Rock ? 1.0f : 0.0f;
            const float sand = s == SurfaceId::Sand ? 1.0f : 0.0f;
            const float snow = s == SurfaceId::Snow ? 1.0f : 0.0f;
            const float mud = s == SurfaceId::Mud ? 1.0f : 0.0f;
            // Render-only road paint: the packed running surface plus a worn shoulder of about 1.8 m that fades
            // into the grass (the physics class above keeps the sharp road edge).
            const float edge = std::min({m_roadEdge[a], m_roadEdge[b], m_roadEdge[c], m_roadEdge[d]});
            const float roadPaint = std::max(road, 1.0f - smoothstep(-0.2f, 1.8f, edge));
            const float clear = std::min({m_roadClear[a], m_roadClear[b], m_roadClear[c], m_roadClear[d]});
            const float k = std::min(smoothstep(1.5f, 2.7f, clear), smoothstep(1.5f, 3.0f, bridgeClearance(x, z)));
            keep[std::size_t(j) * std::size_t(cn) + std::size_t(i)] = k;
            // Gravel and asphalt roads are painted with the road layer too (the terrain set has no asphalt texture).
            const float dirt = s == SurfaceId::PackedDirt || s == SurfaceId::Grass || s == SurfaceId::Gravel ||
                                       s == SurfaceId::Asphalt
                                   ? roadPaint
                                   : 0.0f;
            float grass = 0.0f;
            if (s == SurfaceId::Grass) grass = (0.35f + 0.65f * smoothstep(0.3f, 0.7f, meadow)) * (1.0f - 0.85f * dirt);
            else if (s == SurfaceId::PackedDirt && road < 0.2f) grass = 0.12f * smoothstep(0.4f, 0.7f, patch);
            const float wet = wd > 0.0f ? 1.0f : 0.55f * near;
            mask[std::size_t(j) * std::size_t(cn) + std::size_t(i)] = packRgba(grass, mud, wet, wheat ? 1.0f : 0.0f);
            // b: sand above 0.5, snow below (never on the same ground); a: canopy, filled in by bakeCanopy.
            splat[std::size_t(j) * std::size_t(cn) + std::size_t(i)] = packRgba(dirt, rock, 0.5f + 0.5f * (sand - snow), 0.0f);
        }
    });
    // 3x3 box blur of the masks: soft transitions of about 1.5 m, the physics class keeps the largest weight.
    auto blur = [&](const std::vector<std::uint32_t>& src, std::vector<std::uint32_t>& dst) {
        dst.resize(src.size());
        parallelFor(cn, m_threads, [&](int j) {
            for (int i = 0; i < cn; ++i) {
                float acc[4] = {0, 0, 0, 0};
                for (int dj = -1; dj <= 1; ++dj)
                    for (int di = -1; di <= 1; ++di) {
                        const std::uint32_t v = src[std::size_t(std::clamp(j + dj, 0, cn - 1)) * std::size_t(cn) +
                                                    std::size_t(std::clamp(i + di, 0, cn - 1))];
                        for (int c = 0; c < 4; ++c) acc[c] += channel(v, c);
                    }
                dst[std::size_t(j) * std::size_t(cn) + std::size_t(i)] =
                    packRgba(acc[0] / 9.0f, acc[1] / 9.0f, acc[2] / 9.0f, acc[3] / 9.0f);
            }
        });
    };
    blur(mask, L.m_mask);
    blur(splat, L.m_splat);
    for (std::size_t c = 0; c < cells; ++c)
        if (keep[c] < 1.0f) {
            const std::uint32_t v = L.m_mask[c];
            L.m_mask[c] = packRgba(channel(v, 0) * keep[c], channel(v, 1), channel(v, 2), channel(v, 3) * keep[c]);
        }
}

float LevelBaker::roadClearance(float x, float z) const {
    const int i = std::clamp(int(std::lround((x - m_origin) / m_cell)), 0, m_n - 1);
    const int j = std::clamp(int(std::lround((z - m_origin) / m_cell)), 0, m_n - 1);
    return std::min(m_roadClear[idx(i, j)], bridgeClearance(x, z));
}

float LevelBaker::bridgeClearance(float x, float z) const {
    float best = 1e30f;
    for (const LevelBridge& b : L.m_bridges) {
        const float len = b.length(), dx = (b.x1 - b.x0) / len, dz = (b.z1 - b.z0) / len;
        const float px = x - b.x0, pz = z - b.z0;
        const float along = px * dx + pz * dz, lat = std::abs(px * dz - pz * dx);
        const float r = LevelBridge::kRamp;
        const float beyond = std::max({-r - along, along - len - r, 0.0f});
        best = std::min(best, std::hypot(beyond, std::max(0.0f, lat - b.halfWidth)));
    }
    return best;
}

void LevelBaker::bakeWaterDistance() {
    const float step = 8.0f;
    m_wn = int(2.0f * m_half / step) + 1;
    m_waterDist.assign(std::size_t(m_wn) * std::size_t(m_wn), 120.0f);
    for (int j = 0; j < m_wn; ++j)
        for (int i = 0; i < m_wn; ++i) {
            const float x = m_origin + float(i) * step, z = m_origin + float(j) * step;
            if (L.waterAt(x, z)) m_waterDist[std::size_t(j) * std::size_t(m_wn) + std::size_t(i)] = 0.0f;
        }
    // Two-pass chamfer distance (8 m straight, 11.3 m diagonal steps).
    const float d1 = step, d2 = step * 1.41421356f;
    auto at = [&](int i, int j) -> float& { return m_waterDist[std::size_t(j) * std::size_t(m_wn) + std::size_t(i)]; };
    for (int j = 0; j < m_wn; ++j)
        for (int i = 0; i < m_wn; ++i) {
            float& d = at(i, j);
            if (i > 0) d = std::min(d, at(i - 1, j) + d1);
            if (j > 0) d = std::min(d, at(i, j - 1) + d1);
            if (i > 0 && j > 0) d = std::min(d, at(i - 1, j - 1) + d2);
            if (i + 1 < m_wn && j > 0) d = std::min(d, at(i + 1, j - 1) + d2);
        }
    for (int j = m_wn - 1; j >= 0; --j)
        for (int i = m_wn - 1; i >= 0; --i) {
            float& d = at(i, j);
            if (i + 1 < m_wn) d = std::min(d, at(i + 1, j) + d1);
            if (j + 1 < m_wn) d = std::min(d, at(i, j + 1) + d1);
            if (i + 1 < m_wn && j + 1 < m_wn) d = std::min(d, at(i + 1, j + 1) + d2);
            if (i > 0 && j + 1 < m_wn) d = std::min(d, at(i - 1, j + 1) + d2);
        }
}

float LevelBaker::waterDistance(float x, float z) const {
    if (m_wn == 0) return 120.0f;
    const int i = std::clamp(int(std::lround((x - m_origin) / 8.0f)), 0, m_wn - 1);
    const int j = std::clamp(int(std::lround((z - m_origin) / 8.0f)), 0, m_wn - 1);
    return m_waterDist[std::size_t(j) * std::size_t(m_wn) + std::size_t(i)];
}

void LevelBaker::bakeTrees() {
    const float spacing = 6.0f;
    const int g = int(2.0f * m_half / spacing);
    const int cn = m_n - 1;
    std::vector<std::vector<LevelTree>> rows(static_cast<std::size_t>(g));
    parallelFor(g, m_threads, [&](int gj) {
        for (int gi = 0; gi < g; ++gi) {
            const float x = m_origin + (float(gi) + 0.15f + 0.7f * hashUnit(gi, gj, m_seed + 101u)) * spacing;
            const float z = m_origin + (float(gj) + 0.15f + 0.7f * hashUnit(gi, gj, m_seed + 103u)) * spacing;
            const int ci = std::clamp(int((x - m_origin) / m_cell), 0, cn - 1), cj = std::clamp(int((z - m_origin) / m_cell), 0, cn - 1);
            const std::size_t c = std::size_t(cj) * std::size_t(cn) + std::size_t(ci);
            const SurfaceId s = L.m_surface[c];
            if (s != SurfaceId::Grass && s != SurfaceId::PackedDirt) continue;
            if (m_nearWater[idx(ci, cj)] > 0 || channel(L.m_mask[c], 3) > 0.0f) continue;
            const float treeHeight = 7.0f + 8.5f * hashUnit(gi, gj, m_seed + 113u) + (m_forest[c] > 0.4f ? 2.0f : 0.0f);
            const float treeWidth = 0.85f + 0.3f * hashUnit(gi, gj, m_seed + 137u);
            if (roadClearance(x, z) < kRoadMargin + 0.022f * treeHeight * treeWidth + kGridSlack) continue;
            const std::size_t k = idx(ci, cj);
            const float gx = m_h[idx(ci + 1, cj)] - m_h[k], gz = m_h[idx(ci, cj + 1)] - m_h[k];
            if (std::hypot(gx, gz) > 0.65f) continue;
            bool clear = true;
            for (const Flat& f : m_flats)
                if (std::hypot(x - f.x, z - f.z) < f.r + 4.0f) clear = false;
            for (const LevelPoi& p : L.m_pois)
                if (std::hypot(x - p.atX, z - p.atZ) < 14.0f) clear = false;
            if (!clear) continue;
            const float h = m_h[k];
            const float snowFade = 1.0f - smoothstep(snowLineAt(x, z) - 40.0f, snowLineAt(x, z) - 5.0f, h);
            const float forest = m_forest[c];
            // Past the play area the forest thins out toward the bake edge along a noisy line, so the horizon ring
            // (no trees) does not meet a straight wall of trees.
            const float edge = std::max(std::abs(x), std::abs(z)) + 150.0f * (fbm(x / 240.0f, z / 240.0f, 3, m_seed + 151u) - 0.5f);
            const float outer = 1.0f - smoothstep(L.m_playHalf + 80.0f, m_half - 30.0f, edge);
            const float density = treeDensity(x, z, forest) * snowFade * outer;
            if (hashUnit(gi, gj, m_seed + 109u) >= density) continue;
            LevelTree t;
            t.x = x;
            t.z = z;
            t.height = treeHeight;
            t.y = L.heightAt(x, z) - 0.3f;
            t.yaw = 2.0f * kPi * hashUnit(gi, gj, m_seed + 127u);
            t.hue = hashUnit(gi, gj, m_seed + 131u);
            t.width = treeWidth;
            if (!m_conifers.empty()) {
                // Species: broadleaf trees near water and the village and on low ground, fir saplings among the
                // smaller trees, firs elsewhere. The model scale keeps the tree near the drawn height.
                const float water = 1.0f - smoothstep(12.0f, 70.0f, waterDistance(x, z));
                float village = 0.0f;
                for (const Flat& f : m_flats)
                    if (f.name.find("village") != std::string::npos)
                        village = std::max(village, 1.0f - smoothstep(f.r, f.r + 160.0f, std::hypot(x - f.x, z - f.z)));
                const float low = 1.0f - smoothstep(55.0f, 120.0f, h);
                const float broad = std::min(0.85f, (0.05f + 0.65f * water + 0.55f * village + 0.12f * low) *
                                                        (forest > 0.4f ? 0.45f : 1.0f) * low);
                const float u = hashUnit(gi, gj, m_seed + 141u), v = hashUnit(gi, gj, m_seed + 143u);
                const std::vector<int>* pool = &m_conifers;
                float target = treeHeight, lo = 0.6f, hi = 1.15f;
                t.kind = TreeKind::Conifer;
                if (!m_broadleaf.empty() && u < broad) {
                    pool = &m_broadleaf;
                    t.kind = TreeKind::Broadleaf;
                    target = 0.75f * treeHeight;
                    lo = 1.0f;
                    hi = 2.2f;
                } else if (!m_saplings.empty() && treeHeight < 10.0f && v < 0.55f) {
                    pool = &m_saplings;
                    t.kind = TreeKind::Sapling;
                    lo = 0.7f;
                    hi = 1.25f;
                }
                // Conifers: the tallest variant for the tall trees, the others for the rest.
                std::size_t pick = std::size_t(hash3(gi, gj, m_seed + 147u) % std::uint32_t(pool->size()));
                if (t.kind == TreeKind::Conifer && pool->size() > 1) {
                    std::size_t tallest = 0;
                    for (std::size_t k = 1; k < pool->size(); ++k)
                        if (L.m_models[std::size_t((*pool)[k])].height > L.m_models[std::size_t((*pool)[tallest])].height)
                            tallest = k;
                    if (treeHeight > 13.5f) pick = tallest;
                    else if (pick == tallest) pick = (pick + 1) % pool->size();
                }
                t.model = std::int16_t((*pool)[pick]);
                const LevelModel& m = L.m_models[std::size_t(t.model)];
                t.scale = std::clamp(target / m.height, lo, hi);
                t.height = m.height * t.scale;
                const float lean = std::hypot(m.trunkTop.x, m.trunkTop.z) * t.scale * t.width;
                if (roadClearance(x, z) < kRoadMargin + m.trunkRadius * t.scale * t.width + lean + kGridSlack) continue;
            }
            rows[std::size_t(gj)].push_back(t);
        }
    });
    for (auto& r : rows) L.m_trees.insert(L.m_trees.end(), r.begin(), r.end());
}

// Render-only canopy weight in the splat alpha: 1 under the crowns of the placed trees, fading out past their edge
// (the shader darkens and tints the forest floor with it).
void LevelBaker::bakeCanopy() {
    const int cn = m_n - 1;
    std::vector<float> c(std::size_t(cn) * std::size_t(cn), 0.0f);
    for (const LevelTree& t : L.m_trees) {
        const float r = std::clamp(0.2f * t.height * t.width, 1.5f, 6.0f), reach = r + 2.0f;
        const int i0 = std::max(0, int((t.x - reach - m_origin) / m_cell)), i1 = std::min(cn - 1, int((t.x + reach - m_origin) / m_cell));
        const int j0 = std::max(0, int((t.z - reach - m_origin) / m_cell)), j1 = std::min(cn - 1, int((t.z + reach - m_origin) / m_cell));
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                const float d = std::hypot(X(i) + 0.5f * m_cell - t.x, X(j) + 0.5f * m_cell - t.z);
                float& v = c[std::size_t(j) * std::size_t(cn) + std::size_t(i)];
                v = std::max(v, 1.0f - smoothstep(0.4f * r, reach, d));
            }
    }
    for (std::size_t k = 0; k < c.size(); ++k) {
        const std::uint32_t v = L.m_splat[k];
        L.m_splat[k] = packRgba(channel(v, 0), channel(v, 1), channel(v, 2), c[k]);
    }
}

void LevelBaker::bakeRocks() {
    const float spacing = 12.0f;
    const float play = L.m_playHalf + 40.0f;
    const int g = int(2.0f * play / spacing);
    const int cn = m_n - 1;
    for (int gj = 0; gj < g; ++gj)
        for (int gi = 0; gi < g; ++gi) {
            const float x = -play + (float(gi) + hashUnit(gi, gj, m_seed + 201u)) * spacing;
            const float z = -play + (float(gj) + hashUnit(gi, gj, m_seed + 203u)) * spacing;
            const int ci = std::clamp(int((x - m_origin) / m_cell), 0, cn - 1), cj = std::clamp(int((z - m_origin) / m_cell), 0, cn - 1);
            const std::size_t c = std::size_t(cj) * std::size_t(cn) + std::size_t(ci);
            const SurfaceId s = L.m_surface[c];
            if (roadClearance(x, z) < 2.0f) continue;
            const std::size_t k = idx(ci, cj);
            bool clear = true;
            for (const Flat& f : m_flats)
                if (f.surface == SurfaceId::Count && std::hypot(x - f.x, z - f.z) < f.r + 6.0f) clear = false;
            for (const LevelPoi& p : L.m_pois)
                if (std::hypot(x - p.atX, z - p.atZ) < 16.0f) clear = false;
            if (!clear) continue;
            const bool wetBank = m_riverD[k] < m_riverReach[k] + 6.0f && m_riverD[k] > m_riverReach[k] - 2.0f;
            float p = 0.006f;
            if (s == SurfaceId::Rock) p = m_flatSurface[k] == SurfaceId::Rock ? 0.22f : 0.10f;
            else if (wetBank) p = 0.16f;
            else if (s == SurfaceId::Sand) p = 0.02f;
            else if (s == SurfaceId::Snow) p = 0.03f;
            else if (s == SurfaceId::PackedDirt) p = 0.018f;
            if (L.m_water[k] > Level::kNoWater && L.m_water[k] - m_h[k] > 0.6f) continue;
            if (hashUnit(gi, gj, m_seed + 207u) >= p) continue;
            const float big = s == SurfaceId::Rock ? 1.0f : 0.6f;
            const float r = (0.45f + 1.6f * big * hashUnit(gi, gj, m_seed + 211u) * hashUnit(gi, gj, m_seed + 223u)) ;
            const Vec3 radii(r * (1.0f + 0.5f * hashUnit(gi, gj, 5u)), r * (0.55f + 0.3f * hashUnit(gi, gj, 7u)),
                             r * (0.9f + 0.5f * hashUnit(gi, gj, 11u)));
            if (!siteClear({P2{x, z}}, kRoadMargin + std::max(radii.GetX(), radii.GetZ()), false, false)) continue;
            LevelRock rock;
            rock.x = x;
            rock.z = z;
            rock.rx = radii.GetX();
            rock.ry = radii.GetY();
            rock.rz = radii.GetZ();
            rock.y = L.heightAt(x, z) - 0.3f * rock.ry;
            rock.yaw = 2.0f * kPi * hashUnit(gi, gj, 13u);
            rock.seed = hash3(gi, gj, m_seed + 227u);
            rock.style = s == SurfaceId::Snow ? PropStyle::Stone : PropStyle::DarkStone;
            if (!m_rockModels.empty()) {
                // A rock model scaled to about the drawn extents (uniform across, at most +-40 % in height).
                const int id = m_rockModels[hash3(gi, gj, m_seed + 229u) % std::uint32_t(m_rockModels.size())];
                const LevelModel& m = L.m_models[std::size_t(id)];
                const float ex = 0.5f * (m.boundsMax.x - m.boundsMin.x), ey = 0.5f * (m.boundsMax.y - m.boundsMin.y),
                            ez = 0.5f * (m.boundsMax.z - m.boundsMin.z);
                const float sc = 0.5f * (rock.rx / ex + rock.rz / ez);
                rock.model = std::int16_t(id);
                rock.sx = rock.sz = sc;
                rock.sy = std::clamp(rock.ry / ey, 0.6f * sc, 1.4f * sc);
                rock.rx = ex * sc;
                rock.ry = ey * rock.sy;
                rock.rz = ez * sc;
                rock.y = L.heightAt(x, z) - 0.3f * rock.ry;
                if (!siteClear({P2{x, z}}, kRoadMargin + std::max(rock.rx, rock.rz), false, false)) continue;
            }
            L.m_rocks.push_back(rock);
        }
}

// Logs and stumps (collision: the convex hull of the model) on the forest floor, off the roads and the sites.
void LevelBaker::bakeLogs() {
    if (m_logs.empty()) return;
    const float spacing = 14.0f;
    const float play = L.m_playHalf;
    const int g = int(2.0f * play / spacing);
    const int cn = m_n - 1;
    for (int gj = 0; gj < g; ++gj)
        for (int gi = 0; gi < g; ++gi) {
            const float x = -play + (float(gi) + hashUnit(gi, gj, m_seed + 401u)) * spacing;
            const float z = -play + (float(gj) + hashUnit(gi, gj, m_seed + 403u)) * spacing;
            const int ci = std::clamp(int((x - m_origin) / m_cell), 0, cn - 1), cj = std::clamp(int((z - m_origin) / m_cell), 0, cn - 1);
            const std::size_t c = std::size_t(cj) * std::size_t(cn) + std::size_t(ci);
            const SurfaceId s = L.m_surface[c];
            if (s != SurfaceId::Grass && s != SurfaceId::PackedDirt) continue;
            const float forest = m_forest[c];
            if (hashUnit(gi, gj, m_seed + 407u) >= 0.10f * forest) continue;
            const std::size_t k = idx(ci, cj);
            if (m_nearWater[k] > 0 || L.waterAt(x, z)) continue;
            const float gx = m_h[idx(ci + 1, cj)] - m_h[k], gz = m_h[idx(ci, cj + 1)] - m_h[k];
            if (std::hypot(gx, gz) > 0.45f) continue;
            const int id = m_logs[hash3(gi, gj, m_seed + 409u) % std::uint32_t(m_logs.size())];
            const LevelModel& m = L.m_models[std::size_t(id)];
            const float sc = 0.8f + 0.5f * hashUnit(gi, gj, m_seed + 411u);
            LevelRock rock;
            rock.x = x;
            rock.z = z;
            rock.model = std::int16_t(id);
            rock.sx = rock.sy = rock.sz = sc;
            rock.rx = 0.5f * (m.boundsMax.x - m.boundsMin.x) * sc;
            rock.ry = 0.5f * (m.boundsMax.y - m.boundsMin.y) * sc;
            rock.rz = 0.5f * (m.boundsMax.z - m.boundsMin.z) * sc;
            // Logs lie on the ground (their origin is the base), sunk by 15 % of the height into the slope.
            rock.y = L.heightAt(x, z) - 0.15f * 2.0f * rock.ry;
            rock.yaw = 2.0f * kPi * hashUnit(gi, gj, m_seed + 413u);
            rock.seed = hash3(gi, gj, m_seed + 415u);
            const float reach = std::max(rock.rx, rock.rz);
            if (roadClearance(x, z) < kRoadMargin + reach + kGridSlack) continue;
            bool clear = true;
            for (const Flat& f : m_flats)
                if (std::hypot(x - f.x, z - f.z) < f.r + 4.0f + reach) clear = false;
            for (const LevelPoi& p : L.m_pois)
                if (std::hypot(x - p.atX, z - p.atZ) < 14.0f + reach) clear = false;
            if (!clear || !siteClear({P2{x, z}}, kRoadMargin + reach, false, false)) continue;
            L.m_rocks.push_back(rock);
        }
}

// Render-only ground cover (bushes, ferns, dry branches): forest floor, forest edges and meadows, off roads, water,
// bridges, building sites and steep ground.
// Cliff faces (collision: the model's collision mesh) cladding steep walls (canyons, gorges, mountain faces): slope
// >= 46 deg with at least 8 m of drop across 16 m. The face lies in the wall plane (pitch = slope - 90 deg about the
// model x axis, yaw toward the downhill direction), centred on the ground so its relief stands out of the wall.
void LevelBaker::bakeCliffs() {
    if (m_cliffModels.empty()) return;
    const float spacing = 7.0f;
    const float play = L.m_playHalf + 40.0f;
    const int g = int(2.0f * play / spacing);
    for (int gj = 0; gj < g; ++gj)
        for (int gi = 0; gi < g; ++gi) {
            const float x = -play + (float(gi) + 0.2f + 0.6f * hashUnit(gi, gj, m_seed + 501u)) * spacing;
            const float z = -play + (float(gj) + 0.2f + 0.6f * hashUnit(gi, gj, m_seed + 503u)) * spacing;
            if (std::abs(x) > m_half - 20.0f || std::abs(z) > m_half - 20.0f) continue;
            const float e = 1.5f;
            const float gx = (L.heightAt(x + e, z) - L.heightAt(x - e, z)) / (2.0f * e);
            const float gz = (L.heightAt(x, z + e) - L.heightAt(x, z - e)) / (2.0f * e);
            const float grad = std::hypot(gx, gz);
            const float slope = std::atan(grad);
            if (slope < 46.0f * kPi / 180.0f) continue;
            const float dx = -gx / grad, dz = -gz / grad; // downhill
            const float drop = L.heightAt(x - dx * 8.0f, z - dz * 8.0f) - L.heightAt(x + dx * 8.0f, z + dz * 8.0f);
            if (drop < 8.0f) continue;
            const float h = L.heightAt(x, z);
            if (const std::optional<float> w = L.waterAt(x, z); w && *w > h - 0.5f) continue;
            if (const std::optional<float> w = L.waterAt(x + dx * 4.0f, z + dz * 4.0f); w && *w > h - 4.0f) continue;
            if (hashUnit(gi, gj, m_seed + 507u) >= 0.85f) continue;
            bool clear = true;
            for (const Flat& f : m_flats)
                if (std::hypot(x - f.x, z - f.z) < f.r + 6.0f) clear = false;
            for (const LevelPoi& p : L.m_pois)
                if (std::hypot(x - p.atX, z - p.atZ) < 16.0f) clear = false;
            if (!clear) continue;
            const int id = m_cliffModels[hash3(gi, gj, m_seed + 509u) % std::uint32_t(m_cliffModels.size())];
            const LevelModel& m = L.m_models[std::size_t(id)];
            const float ex = 0.5f * (m.boundsMax.x - m.boundsMin.x), ey = 0.5f * (m.boundsMax.y - m.boundsMin.y),
                        ez = 0.5f * (m.boundsMax.z - m.boundsMin.z);
            const float width = 9.0f + 7.0f * hashUnit(gi, gj, m_seed + 511u);
            const float sc = width / (2.0f * ex);
            LevelRock rock;
            rock.model = std::int16_t(id);
            rock.sx = rock.sy = sc;
            rock.sz = sc * (0.8f + 0.4f * hashUnit(gi, gj, m_seed + 513u));
            rock.rx = ex * rock.sx;
            rock.ry = ey * rock.sy;
            rock.rz = ez * rock.sz;
            if (!siteClear({P2{x, z}}, kRoadMargin + std::max(rock.rx, rock.ry), false, false)) continue;
            rock.yaw = std::atan2(dx, dz) + 0.3f * (hashUnit(gi, gj, m_seed + 515u) - 0.5f);
            rock.pitch = slope - 0.5f * kPi;
            // Face frame (the shader transform of +X, +Y, +Z); the slab sits a third of its depth proud of the slope
            // and every edge point must keep the back buried (the models are open shells: a back seen from above a
            // rim reads as loose shards) without bridging a gully.
            const float cy = std::cos(rock.yaw), sy = std::sin(rock.yaw), cp = std::cos(rock.pitch), sp = std::sin(rock.pitch);
            const float rgt[3] = {cy, 0.0f, -sy}, up[3] = {sp * sy, cp, sp * cy}, nrm[3] = {cp * sy, -sp, cp * cy};
            const float out = 0.33f * rock.rz;
            rock.x = x + nrm[0] * out;
            rock.y = h + nrm[1] * out;
            rock.z = z + nrm[2] * out;
            bool flush = true;
            for (int a = -2; a <= 2 && flush; ++a)
                for (int b = -2; b <= 2 && flush; ++b) {
                    if (a == 0 && b == 0) continue;
                    const float u = 0.5f * float(a) * rock.rx, v = 0.5f * float(b) * rock.ry;
                    const float px = rock.x + rgt[0] * u + up[0] * v, py = rock.y + up[1] * v, pz = rock.z + rgt[2] * u + up[2] * v;
                    const float gy = L.heightAt(px, pz);
                    const float d = (py - gy) * std::cos(slope);
                    if (d > 0.9f * rock.rz || d < -0.5f * rock.rz - 1.0f) flush = false;
                }
            if (!flush) continue;
            // A slab reaching below the surface of water next to it leaves only slivers above the surface (dark
            // shards on the Falls Pool): within its reach of any water its lowest point stays above the water.
            const float reach = std::max(rock.rx, rock.ry);
            bool dry = true;
            for (int a = 0; a < 8 && dry; ++a) {
                const float ax = rock.x + reach * std::cos(0.25f * kPi * float(a)), az = rock.z + reach * std::sin(0.25f * kPi * float(a));
                if (const std::optional<float> w = L.waterAt(ax, az); w && rock.y - rock.ry < *w + 0.3f) dry = false;
            }
            if (!dry) continue;
            rock.seed = hash3(gi, gj, m_seed + 517u);
            rock.style = PropStyle::DarkStone;
            L.m_rocks.push_back(rock);
        }
}

// Scree (collision: the rock model's collision mesh, as bakeRocks): small clusters of boulders on the gentler ground
// at the foot of walls, where the ground within 6 m rises by more than 6 m.
void LevelBaker::bakeTalus() {
    if (m_rockModels.empty()) return;
    const float spacing = 4.0f;
    const float play = L.m_playHalf;
    const int g = int(2.0f * play / spacing);
    static constexpr float kDirs[8][2] = {{1, 0}, {0.7071f, 0.7071f}, {0, 1}, {-0.7071f, 0.7071f},
                                          {-1, 0}, {-0.7071f, -0.7071f}, {0, -1}, {0.7071f, -0.7071f}};
    for (int gj = 0; gj < g; ++gj)
        for (int gi = 0; gi < g; ++gi) {
            if (hashUnit(gi, gj, m_seed + 601u) >= 0.45f) continue;
            const float x = -play + (float(gi) + hashUnit(gi, gj, m_seed + 603u)) * spacing;
            const float z = -play + (float(gj) + hashUnit(gi, gj, m_seed + 605u)) * spacing;
            const float h = L.heightAt(x, z);
            const float e = 1.0f;
            const float gx = (L.heightAt(x + e, z) - L.heightAt(x - e, z)) / (2.0f * e);
            const float gz = (L.heightAt(x, z + e) - L.heightAt(x, z - e)) / (2.0f * e);
            if (std::hypot(gx, gz) > 0.75f) continue; // ~37 deg: the apron, not the wall
            float rise = 0.0f;
            for (const auto& d : kDirs) rise = std::max(rise, L.heightAt(x + d[0] * 6.0f, z + d[1] * 6.0f) - h);
            if (rise < 6.0f) continue;
            if (const std::optional<float> w = L.waterAt(x, z); w && *w > h - 0.3f) continue;
            if (roadClearance(x, z) < kRoadMargin + 3.0f) continue;
            bool clear = true;
            for (const Flat& f : m_flats)
                if (std::hypot(x - f.x, z - f.z) < f.r + 6.0f) clear = false;
            for (const LevelPoi& p : L.m_pois)
                if (std::hypot(x - p.atX, z - p.atZ) < 16.0f) clear = false;
            if (!clear) continue;
            const int count = 1 + int(3.0f * hashUnit(gi, gj, m_seed + 607u));
            for (int c = 0; c < count; ++c) {
                const std::uint32_t sd = m_seed + 611u + std::uint32_t(c) * 17u;
                const float rx = x + 2.2f * (hashUnit(gi, gj, sd) - 0.5f), rz = z + 2.2f * (hashUnit(gi, gj, sd + 1u) - 0.5f);
                const int id = m_rockModels[hash3(gi, gj, sd + 2u) % std::uint32_t(m_rockModels.size())];
                const LevelModel& m = L.m_models[std::size_t(id)];
                const float ex = 0.5f * (m.boundsMax.x - m.boundsMin.x), ey = 0.5f * (m.boundsMax.y - m.boundsMin.y),
                            ez = 0.5f * (m.boundsMax.z - m.boundsMin.z);
                const float u = hashUnit(gi, gj, sd + 3u);
                const float r = 0.3f + 1.2f * u * u;
                const float sc = r / std::max(ex, ez);
                LevelRock rock;
                rock.model = std::int16_t(id);
                rock.sx = rock.sz = sc;
                rock.sy = sc * (0.7f + 0.5f * hashUnit(gi, gj, sd + 4u));
                rock.rx = ex * sc;
                rock.ry = ey * rock.sy;
                rock.rz = ez * sc;
                rock.x = rx;
                rock.z = rz;
                rock.y = L.heightAt(rx, rz) - 0.3f * rock.ry;
                rock.yaw = 2.0f * kPi * hashUnit(gi, gj, sd + 5u);
                rock.seed = hash3(gi, gj, sd + 6u);
                rock.style = PropStyle::DarkStone;
                if (!siteClear({P2{rx, rz}}, kRoadMargin + std::max(rock.rx, rock.rz), false, false)) continue;
                L.m_rocks.push_back(rock);
            }
        }
}

void LevelBaker::bakeScatter() {
    if (m_bushes.empty() && m_ferns.empty() && m_debris.empty()) return;
    // Occupied 4 m cells: footprints of the placed props.
    const float occ = 4.0f;
    const int on = int(2.0f * m_half / occ) + 1;
    std::vector<std::uint8_t> used(std::size_t(on) * std::size_t(on), 0);
    for (const std::vector<P2>& site : m_sites) {
        float x0 = 1e30f, z0 = 1e30f, x1 = -1e30f, z1 = -1e30f;
        for (const P2& p : site) x0 = std::min(x0, p.x), z0 = std::min(z0, p.z), x1 = std::max(x1, p.x), z1 = std::max(z1, p.z);
        for (int j = std::max(0, int((z0 - 2.0f - m_origin) / occ)); j <= std::min(on - 1, int((z1 + 2.0f - m_origin) / occ)); ++j)
            for (int i = std::max(0, int((x0 - 2.0f - m_origin) / occ)); i <= std::min(on - 1, int((x1 + 2.0f - m_origin) / occ)); ++i)
                used[std::size_t(j) * std::size_t(on) + std::size_t(i)] = 1;
    }
    const float spacing = 3.0f;
    const float play = L.m_playHalf + 40.0f;
    const int g = int(2.0f * play / spacing);
    const int cn = m_n - 1;
    std::vector<std::vector<LevelScatter>> rows(static_cast<std::size_t>(g));
    parallelFor(g, m_threads, [&](int gj) {
        for (int gi = 0; gi < g; ++gi) {
            const float x = -play + (float(gi) + hashUnit(gi, gj, m_seed + 501u)) * spacing;
            const float z = -play + (float(gj) + hashUnit(gi, gj, m_seed + 503u)) * spacing;
            const int ci = std::clamp(int((x - m_origin) / m_cell), 0, cn - 1), cj = std::clamp(int((z - m_origin) / m_cell), 0, cn - 1);
            const std::size_t c = std::size_t(cj) * std::size_t(cn) + std::size_t(ci);
            const SurfaceId s = L.m_surface[c];
            if (s != SurfaceId::Grass && s != SurfaceId::PackedDirt) continue;
            const std::size_t k = idx(ci, cj);
            if (L.m_water[k] > Level::kNoWater || channel(L.m_mask[c], 3) > 0.0f) continue; // water, wheat
            const float gx = m_h[idx(ci + 1, cj)] - m_h[k], gz = m_h[idx(ci, cj + 1)] - m_h[k];
            const float slope = std::hypot(gx, gz);
            if (slope > 0.8f) continue;
            const int oi = std::clamp(int((x - m_origin) / occ), 0, on - 1), oj = std::clamp(int((z - m_origin) / occ), 0, on - 1);
            if (used[std::size_t(oj) * std::size_t(on) + std::size_t(oi)]) continue;
            const float forest = m_forest[c];
            const float h = m_h[k];
            const float snowFade = 1.0f - smoothstep(snowLineAt(x, z) - 60.0f, snowLineAt(x, z) - 15.0f, h);
            const float water = 1.0f - smoothstep(4.0f, 40.0f, waterDistance(x, z));
            const float cluster = smoothstep(0.35f, 0.7f, fbm(x / 23.0f, z / 23.0f, 3, m_seed + 509u));
            // Bushes along the forest edges and in clusters on the meadows, ferns on the forest floor and by the
            // water, dry branches under the trees.
            const float edge = forest * (1.0f - forest) * 4.0f;
            const float pBush = snowFade * (0.05f + 0.30f * edge + 0.18f * water + 0.22f * cluster * (1.0f - forest)) * (0.4f + 0.6f * cluster);
            const float pFern = snowFade * (0.40f * forest + 0.25f * water) * (0.3f + 0.7f * cluster);
            const float pDebris = snowFade * 0.10f * forest;
            const float u = hashUnit(gi, gj, m_seed + 511u);
            const std::vector<int>* pool = nullptr;
            float size = 1.0f;
            if (u < pBush) pool = &m_bushes, size = 0.7f + 0.6f * hashUnit(gi, gj, m_seed + 513u);
            else if (u < pBush + pFern) pool = &m_ferns, size = 1.6f + 1.2f * hashUnit(gi, gj, m_seed + 515u);
            else if (u < pBush + pFern + pDebris) pool = &m_debris, size = 1.2f + 1.3f * hashUnit(gi, gj, m_seed + 517u);
            if (!pool || pool->empty()) continue;
            const int id = (*pool)[hash3(gi, gj, m_seed + 519u) % std::uint32_t(pool->size())];
            const float reach = L.m_models[std::size_t(id)].radius * size;
            if (roadClearance(x, z) < 0.8f + reach + kGridSlack) continue;
            // The whole footprint on dry, even ground: a fern or bush whose fronds reach over water or past a wall
            // edge floats there as loose shards (the Falls Pool shore under its walls).
            bool clear = true;
            float lo = 1e30f, hi = -1e30f;
            for (int a = 0; a < 8 && clear; ++a) {
                const float ax = x + reach * std::cos(0.25f * kPi * float(a)), az = z + reach * std::sin(0.25f * kPi * float(a));
                if (L.waterAt(ax, az)) clear = false;
                const float ah = L.heightAt(ax, az);
                lo = std::min(lo, ah), hi = std::max(hi, ah);
            }
            if (!clear || hi - lo > 0.8f * reach + 0.3f) continue;
            for (const LevelPoi& p : L.m_pois)
                if (std::hypot(x - p.atX, z - p.atZ) < 8.0f + reach) clear = false;
            if (!clear) continue;
            LevelScatter sc;
            sc.x = x;
            sc.z = z;
            sc.y = L.heightAt(x, z) - 0.05f;
            sc.yaw = 2.0f * kPi * hashUnit(gi, gj, m_seed + 521u);
            sc.scale = size;
            sc.model = std::int16_t(id);
            rows[std::size_t(gj)].push_back(sc);
        }
    });
    for (auto& r : rows) L.m_scatter.insert(L.m_scatter.end(), r.begin(), r.end());
}

// ---------------------------------------------------------------------------------------------- props (blockouts)
void LevelBaker::addProp(const std::function<void(TrackGeometry&)>& build, PropStyle style) {
    std::optional<Site> site;
    if (m_role != PropRole::Bridge) site.emplace(*this);
    L.m_props.pieceStarts.push_back(std::uint32_t(L.m_props.geometry.triangleCount()));
    L.m_props.pieceRoles.push_back(m_role);
    build(L.m_props.geometry);
    L.m_props.styles.resize(L.m_props.geometry.triangleCount(), style);
}

namespace {

std::vector<P2> convexHull(std::vector<P2> p) {
    std::sort(p.begin(), p.end(), [](const P2& a, const P2& b) { return a.x < b.x || (a.x == b.x && a.z < b.z); });
    p.erase(std::unique(p.begin(), p.end(), [](const P2& a, const P2& b) { return a.x == b.x && a.z == b.z; }), p.end());
    if (p.size() < 3) return p;
    const auto cross = [](const P2& o, const P2& a, const P2& b) {
        return (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
    };
    std::vector<P2> h(2 * p.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0.0f) --k;
        h[k++] = p[i];
    }
    for (std::size_t i = p.size() - 1, t = k + 1; i > 0; --i) {
        while (k >= t && cross(h[k - 2], h[k - 1], p[i - 1]) <= 0.0f) --k;
        h[k++] = p[i - 1];
    }
    h.resize(k - 1);
    return h;
}

// Distance from a point to a convex polygon (counter-clockwise; a point or a segment also work), 0 inside.
float polygonDistance(const std::vector<P2>& poly, float x, float z) {
    if (poly.size() == 1) return std::hypot(x - poly[0].x, z - poly[0].z);
    bool inside = poly.size() >= 3;
    float best = 1e30f;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const P2 a = poly[i], b = poly[(i + 1) % poly.size()];
        const float ex = b.x - a.x, ez = b.z - a.z, px = x - a.x, pz = z - a.z;
        if (ex * pz - ez * px < 0.0f) inside = false;
        const float t = std::clamp((px * ex + pz * ez) / std::max(1e-9f, ex * ex + ez * ez), 0.0f, 1.0f);
        best = std::min(best, std::hypot(px - ex * t, pz - ez * t));
    }
    return inside ? 0.0f : best;
}

// Distance between two convex polygons (0 when they overlap).
float polygonGap(const std::vector<P2>& a, const std::vector<P2>& b) {
    float best = 1e30f;
    for (const P2& p : a) best = std::min(best, polygonDistance(b, p.x, p.z));
    for (const P2& p : b) best = std::min(best, polygonDistance(a, p.x, p.z));
    return best;
}

} // namespace

void LevelBaker::buildRoadPoints() {
    m_bucketSide = int(std::ceil(2.0f * m_half / 32.0f));
    m_roadBuckets.assign(std::size_t(m_bucketSide) * std::size_t(m_bucketSide), {});
    for (const LevelRoad& r : L.m_roads)
        for (std::size_t i = 0; i + 1 < r.samples.size(); ++i) {
            const RoadSample &a = r.samples[i], &b = r.samples[i + 1];
            const int n = std::max(1, int(std::hypot(b.x - a.x, b.z - a.z) / 0.5f));
            for (int k = 0; k < n; ++k) {
                const float t = float(k) / float(n);
                const RoadPoint p{lerp(a.x, b.x, t), lerp(a.z, b.z, t), 0.5f * r.width};
                const int bi = std::clamp(int((p.x - m_origin) / 32.0f), 0, m_bucketSide - 1);
                const int bj = std::clamp(int((p.z - m_origin) / 32.0f), 0, m_bucketSide - 1);
                m_roadBuckets[std::size_t(bj) * std::size_t(m_bucketSide) + std::size_t(bi)].push_back(p);
            }
        }
    m_features = L.features();
}

bool LevelBaker::siteClear(const std::vector<P2>& hull, float margin, bool building, bool moved) const {
    float x0 = 1e30f, z0 = 1e30f, x1 = -1e30f, z1 = -1e30f;
    for (const P2& p : hull) {
        x0 = std::min(x0, p.x), z0 = std::min(z0, p.z), x1 = std::max(x1, p.x), z1 = std::max(z1, p.z);
    }
    if (moved && std::max({std::abs(x0), std::abs(x1), std::abs(z0), std::abs(z1)}) > L.m_playHalf) return false;
    const float reach = margin + 8.0f; // road half widths are at most ~4 m
    const int bi0 = std::max(0, int((x0 - reach - m_origin) / 32.0f)), bi1 = std::min(m_bucketSide - 1, int((x1 + reach - m_origin) / 32.0f));
    const int bj0 = std::max(0, int((z0 - reach - m_origin) / 32.0f)), bj1 = std::min(m_bucketSide - 1, int((z1 + reach - m_origin) / 32.0f));
    for (int bj = bj0; bj <= bj1; ++bj)
        for (int bi = bi0; bi <= bi1; ++bi)
            for (const RoadPoint& p : m_roadBuckets[std::size_t(bj) * std::size_t(m_bucketSide) + std::size_t(bi)]) {
                if (p.x < x0 - reach || p.x > x1 + reach || p.z < z0 - reach || p.z > z1 + reach) continue;
                if (polygonDistance(hull, p.x, p.z) - p.hw < margin) return false;
            }
    for (const TrackFeature& f : m_features)
        if (polygonDistance(hull, f.position.GetX(), f.position.GetZ()) < (building ? 6.0f : 3.0f)) return false;
    if (building) {
        for (const P2& p : hull)
            if (L.waterAt(p.x, p.z)) return false;
        if (L.waterAt(0.5f * (x0 + x1), 0.5f * (z0 + z1))) return false;
    }
    if (moved)
        for (const std::vector<P2>& other : m_sites)
            if (polygonGap(hull, other) < 1.0f) return false;
    return true;
}

void LevelBaker::settle(std::size_t firstTri, std::size_t firstPiece, std::size_t firstBuilding) {
    TrackGeometry& g = L.m_props.geometry;
    const std::size_t lastTri = g.triangleCount();
    if (lastTri <= firstTri) return;
    std::vector<P2> pts;
    pts.reserve(3 * (lastTri - firstTri));
    for (std::size_t v = 3 * firstTri; v < 3 * lastTri; ++v) pts.push_back({g.positions[v].x, g.positions[v].z});
    const std::vector<P2> hull = convexHull(pts);
    if (m_baked) { // placed where the layout puts it; checkBakedLayout reports rule violations
        m_sites.push_back(hull);
        return;
    }
    float margin = 0.0f;
    bool building = false;
    for (std::size_t k = firstPiece; k < L.m_props.pieceRoles.size(); ++k) {
        const PropRole r = L.m_props.pieceRoles[k];
        building = building || r == PropRole::Building;
        margin = std::max(margin, r == PropRole::Building ? kBuildingMargin : r == PropRole::Rock ? kRoadMargin : kPropMargin);
    }
    float cx = 0.0f, cz = 0.0f;
    for (const P2& p : hull) cx += p.x, cz += p.z;
    cx /= float(hull.size());
    cz /= float(hull.size());
    const auto shifted = [&](float dx, float dz) {
        std::vector<P2> h = hull;
        for (P2& p : h) p.x += dx, p.z += dz;
        return h;
    };
    if (siteClear(hull, margin, building, false)) {
        if (building) m_structureSites.push_back(m_sites.size());
        m_sites.push_back(hull);
        return;
    }
    const float start = 2.0f * kPi * hashUnit(int(std::floor(cx)), int(std::floor(cz)), m_seed + 401u);
    for (float r = 2.0f; r <= 40.0f; r += 2.0f)
        for (int a = 0; a < 16; ++a) {
            const float ang = start + 2.0f * kPi * float(a) / 16.0f;
            const float dx = r * std::sin(ang), dz = r * std::cos(ang);
            std::vector<P2> h = shifted(dx, dz);
            if (!siteClear(h, margin, building, true)) continue;
            bool level = true; // the plinths of moved structures must still reach the ground
            for (std::size_t k = firstBuilding; k < L.m_buildings.size() && level; ++k) {
                const LevelBuilding& b = L.m_buildings[k];
                const BuildingKind kind = L.m_buildingTypes[b.type].spec.kind;
                if (!m_onGround[k] || !buildingIsStructure(kind)) continue;
                float spread = 0.0f;
                groundFloor(b.type, b.x + dx, b.z + dz, b.yaw, &spread);
                level = spread < buildingPlinthDepth(kind) - 0.5f;
            }
            if (!level) continue;
            const float dy = L.heightAt(cx + dx, cz + dz) - L.heightAt(cx, cz);
            for (std::size_t v = 3 * firstTri; v < 3 * lastTri; ++v) {
                g.positions[v].x += dx;
                g.positions[v].y += dy;
                g.positions[v].z += dz;
            }
            // Buildings on the ground get their floor rule again at the new place (the slope may differ).
            for (std::size_t k = firstBuilding; k < L.m_buildings.size(); ++k) {
                LevelBuilding& b = L.m_buildings[k];
                b.x += dx;
                b.z += dz;
                const float by = m_onGround[k] ? groundFloor(b.type, b.x, b.z, b.yaw) - b.y : dy;
                b.y += by;
                const std::size_t t1 = k + 1 < L.m_buildings.size() ? m_buildingTri[k + 1] : lastTri;
                for (std::size_t v = 3 * m_buildingTri[k]; v < 3 * t1; ++v) g.positions[v].y += by - dy;
            }
            if (building) m_structureSites.push_back(m_sites.size());
            m_sites.push_back(std::move(h));
            ++L.m_stats.propsMoved;
            return;
        }
    g.positions.resize(3 * firstTri);
    g.surfaces.resize(firstTri);
    L.m_props.styles.resize(firstTri);
    L.m_props.pieceStarts.resize(firstPiece);
    L.m_props.pieceRoles.resize(firstPiece);
    L.m_buildings.resize(firstBuilding);
    m_onGround.resize(firstBuilding);
    m_buildingTri.resize(firstBuilding);
    ++L.m_stats.propsRemoved;
}

void LevelBaker::box(float x, float y, float z, float hx, float hy, float hz, float yaw, PropStyle style, SurfaceId s) {
    addProp([&](TrackGeometry& g) { g.addBox(Vec3(x, y, z), Vec3(hx, hy, hz), yaw, s); }, style);
}

float LevelBaker::footprintMin(float x, float z, float hx, float hz, float yaw) const {
    const float c = std::cos(yaw), s = std::sin(yaw);
    float lo = L.heightAt(x, z);
    for (float u : {-1.0f, 1.0f})
        for (float v : {-1.0f, 1.0f}) {
            const float lx = u * hx, lz = v * hz;
            lo = std::min(lo, L.heightAt(x + c * lx + s * lz, z - s * lx + c * lz));
        }
    return lo;
}

void LevelBaker::groundedBox(float x, float z, float hx, float hy, float hz, float yaw, PropStyle style, SurfaceId s,
                             float lift) {
    const float base = footprintMin(x, z, hx, hz, yaw) - 0.4f + lift;
    box(x, base + hy, z, hx, hy, hz, yaw, style, s);
}

void LevelBaker::slab(Vec3 a, Vec3 b, float hw, float th, PropStyle style, SurfaceId s) {
    addProp([&](TrackGeometry& g) {
        Vec3 dir = b - a;
        dir.SetY(0.0f);
        dir = dir.NormalizedOr(Vec3::sAxisZ());
        const Vec3 left(dir.GetZ(), 0.0f, -dir.GetX());
        const Vec3 down(0.0f, -th, 0.0f);
        const Vec3 al = a + left * hw, ar = a - left * hw, bl = b + left * hw, br = b - left * hw;
        // Counter-clockwise seen from outside: Jolt's wheel casts ignore back faces.
        g.addQuad(ar, br, bl, al, s);                             // top
        g.addQuad(al + down, bl + down, br + down, ar + down, s); // bottom
        g.addQuad(al, bl, bl + down, al + down, s);               // left side
        g.addQuad(br, ar, ar + down, br + down, s);               // right side
        g.addQuad(ar, al, al + down, ar + down, s);               // start
        g.addQuad(bl, br, br + down, bl + down, s);               // end
    }, style);
}

void LevelBaker::rock(Vec3 center, Vec3 radii, float yaw, std::uint32_t seed, PropStyle style) {
    const PropRole role = std::exchange(m_role, PropRole::Rock);
    addProp([&](TrackGeometry& g) { g.addRock(center, radii, yaw, seed, SurfaceId::Rock); }, style);
    m_role = role;
}

void LevelBaker::bakeProps() {
    // Bridges: deck slab on the road profile, railings, piers down to the ground.
    m_role = PropRole::Bridge;
    for (const LevelBridge& b : L.m_bridges) {
        const float len = std::hypot(b.x1 - b.x0, b.z1 - b.z0);
        const float dx = (b.x1 - b.x0) / len, dz = (b.z1 - b.z0) / len;
        const bool stone = len < 80.0f;
        const PropStyle deckStyle = stone ? PropStyle::Stone : PropStyle::Wood;
        // Ramps into the banks at both ends, down onto the road, so the deck has no step or gap to it.
        const float r = LevelBridge::kRamp;
        const auto P = [&](float s) { return Vec3(b.x0 + dx * s, b.deckY(s), b.z0 + dz * s); };
        const Vec3 a = P(-r), e = P(len + r);
        slab(a, P(0.0f), b.halfWidth, 0.8f, deckStyle, SurfaceId::Concrete);
        slab(P(0.0f), P(len), b.halfWidth, 0.8f, deckStyle, SurfaceId::Concrete);
        slab(P(len), e, b.halfWidth, 0.8f, deckStyle, SurfaceId::Concrete);
        const float total = len + 2.0f * r;
        const int segs = std::max(2, int(total / 6.0f));
        for (int k = 0; k <= segs; ++k) {
            const Vec3 p = P(-r + total * float(k) / float(segs));
            for (float side : {-1.0f, 1.0f}) {
                const float rx = p.GetX() + dz * side * (b.halfWidth - 0.15f), rz = p.GetZ() - dx * side * (b.halfWidth - 0.15f);
                box(rx, p.GetY() + 0.55f, rz, 0.12f, 0.55f, 0.12f, std::atan2(dx, dz), stone ? PropStyle::Stone : PropStyle::DarkWood);
            }
        }
        for (float side : {-1.0f, 1.0f}) {
            const Vec3 off(dz * side * (b.halfWidth - 0.15f), 1.05f, -dx * side * (b.halfWidth - 0.15f));
            slab(a + off, e + off, 0.1f, 0.18f, stone ? PropStyle::Stone : PropStyle::DarkWood, SurfaceId::Concrete);
        }
        // Piers every ~20 m where the ground is more than 2 m below the deck.
        const int piers = std::max(1, int(len / 20.0f));
        for (int k = 1; k < piers + 1; ++k) {
            const float t = float(k) / float(piers + 1);
            const float px = lerp(b.x0, b.x1, t), pz = lerp(b.z0, b.z1, t), py = lerp(b.y0, b.y1, t) - 0.8f;
            const float ground = L.heightAt(px, pz) - 1.0f;
            if (py - ground < 2.0f) continue;
            box(px, 0.5f * (py + ground), pz, b.halfWidth * 0.8f, 0.5f * (py - ground), 1.2f, std::atan2(dx, dz),
                stone ? PropStyle::DarkStone : PropStyle::Stone);
        }
    }

    m_role = PropRole::Prop;
    using BK = BuildingKind;
    constexpr int kShutGreen = 0 << kHouseShutterShift, kShutRed = 1 << kHouseShutterShift,
                  kShutWood = 2 << kHouseShutterShift, kShutNone = 3 << kHouseShutterShift;
    // Village house archetypes (few distinct models, drawn instanced).
    const BuildingSpec villageHouses[6] = {
        {BK::House, 9.0f, 7.0f, 0.0f, 2, kHouseStoneBase | kHouseBalcony | kShutGreen},
        {BK::House, 8.0f, 6.5f, 0.0f, 2, kHouseUpperPlaster | kHouseShingles | kShutRed},
        {BK::House, 10.0f, 7.5f, 0.0f, 3, kHouseStoneBase | kHouseUpperPlaster | kHouseOchre | kHouseBalcony | kHouseSideBalcony | kShutWood},
        {BK::House, 7.0f, 6.0f, 0.0f, 1, kHouseTimber | kHouseShingles | kShutNone},
        {BK::House, 8.5f, 7.0f, 0.0f, 2, kHouseUpperPlaster | kHouseBalcony | kShutWood},
        {BK::House, 9.0f, 6.5f, 0.0f, 2, kHouseStoneBase | kHouseShingles | kHouseSideBalcony | kShutGreen},
    };
    const BuildingSpec cabin{BK::House, 6.0f, 4.5f, 0.0f, 1, kHouseTimber | kHouseShingles | kShutWood};
    const BuildingSpec hut{BK::House, 5.0f, 4.0f, 0.0f, 1, kHouseStoneBase | kHouseShingles | kShutNone};
    const BuildingSpec keeper{BK::House, 6.5f, 5.0f, 0.0f, 1, kHouseUpperPlaster | kShutRed};
    const BuildingSpec boathouse{BK::House, 6.0f, 5.0f, 0.0f, 1, kHouseTimber | kShutGreen};
    const BuildingSpec shack{BK::House, 4.5f, 4.0f, 0.0f, 1, kHouseTimber | kHouseShingles | kShutNone};
    const auto faceTo = [](float fx, float fz, float tx, float tz) { return std::atan2(tx - fx, tz - fz); };
    for (const LevelPoi& p : L.m_pois) {
        const float x = p.x, z = p.z;
        if (p.kind == "peak") {
            building({BK::Cairn, 0, 0, 0, 0, 0}, x + 4.0f, z - 3.0f, 0.3f);
        } else if (p.kind == "pass") {
            building({BK::Cairn, 0, 0, 0, 0, 1}, x - 9.0f, z + 6.0f, 1.1f);
            {
                Site site(*this);
                building(hut, x + 12.0f, z + 9.0f, 0.3f);
                building({BK::Woodpile, 3.0f, 0, 0, 0, 0}, x + 12.0f + 3.6f * std::cos(0.3f), z + 9.0f - 3.6f * std::sin(0.3f), 0.3f + 0.5f * kPi);
            }
            building({BK::Signpost, 0, 0, 0, 0, 0}, x - 6.0f, z - 7.0f, 0.4f);
        } else if (p.kind == "quarry") {
            {
                // Crusher, its discharge belt and a silo.
                Site site(*this);
                const float cx = x + 20.0f, cz = z - 18.0f, yaw = 0.4f;
                building({BK::Crusher, 0, 0, 9.0f, 0, 0}, cx, cz, yaw);
                const float c = std::cos(yaw), sn = std::sin(yaw);
                const float bx = cx - sn * 6.5f, bz = cz - c * 6.5f;
                float lo = 1e30f;
                for (int k = 0; k <= 4; ++k) lo = std::min(lo, L.heightAt(bx - sn * 3.5f * float(k), bz - c * 3.5f * float(k)));
                building({BK::Conveyor, 14.0f, 0, 4.5f, 0, 0}, bx, bz, yaw + kPi, lo);
                building({BK::Silo, 4.0f, 0, 12.0f, 0, 0}, cx + c * 7.5f, cz - sn * 7.5f, 0.0f);
            }
            building({BK::Garage, 10.0f, 7.0f, 4.2f, 0, 1}, x - 22.0f, z - 20.0f, 0.2f);
            for (int k = 0; k < 4; ++k) {
                const float hx = x - 10.0f + 9.0f * float(k), hz = z + 22.0f;
                rock(Vec3(hx, L.heightAt(hx, hz) - 0.6f, hz), Vec3(3.4f, 1.8f, 3.0f), 0.7f * float(k), 900u + k, PropStyle::Stone);
            }
        } else if (p.kind == "forest") {
            building(cabin, x + 10.0f, z + 8.0f, 0.6f);
            building({BK::Woodpile, 5.0f, 0, 0, 0, 0}, x - 9.0f, z + 7.0f, 0.0f);
        } else if (p.kind == "lake") {
            const float sx = p.atX, sz = p.atZ;
            const float dx = p.x - sx, dz = p.z - sz, dl = std::hypot(dx, dz);
            if (const auto d = shoreDock(sx + 8.0f, sz - 4.0f, dx, dz, 18.0f, 2.4f)) {
                // Beside the dock, back from the bank until the ground under it is level enough for the plinth.
                const std::uint16_t type = buildingType(boathouse);
                for (float back = 4.0f; back <= 30.0f; back += 2.0f) {
                    const float bx = d->x - dz / dl * 9.0f - dx / dl * back, bz = d->z + dx / dl * 9.0f - dz / dl * back;
                    const float yaw = faceTo(bx, bz, p.x, p.z);
                    float spread = 0.0f;
                    groundFloor(type, bx, bz, yaw, &spread);
                    if (spread > buildingPlinthDepth(BK::House) - 0.5f || L.waterAt(bx, bz)) continue;
                    building(boathouse, bx, bz, yaw);
                    break;
                }
            }
            building({BK::Bench, 1.8f, 0, 0, 0, 0}, sx - 8.0f, sz - 3.0f, 2.9f);
        } else if (p.kind == "waterfall") {
            building({BK::Fence, 9.0f, 0, 0, 0, 1}, p.atX + 2.0f, p.atZ - 9.0f, -0.2f);
            building({BK::Bench, 1.8f, 0, 0, 0, 0}, p.atX - 6.0f, p.atZ + 2.0f, 0.3f);
        } else if (p.kind == "radio_tower") {
            {
                // Mast inside a metal fence.
                Site site(*this);
                building({BK::RadioMast, 0, 0, 34.0f, 0, 0}, x, z, 0.0f);
                building({BK::Fence, 9.0f, 0, 0, 0, 2}, x, z + 4.5f, 0.0f);
                building({BK::Fence, 9.0f, 0, 0, 0, 2}, x, z - 4.5f, 0.0f);
                building({BK::Fence, 9.0f, 0, 0, 0, 2}, x + 4.5f, z, 0.5f * kPi);
                building({BK::Fence, 9.0f, 0, 0, 0, 2}, x - 4.5f, z, 0.5f * kPi);
            }
            building({BK::RadioHut, 4.0f, 3.0f, 0.0f, 0, 0}, x - 8.0f, z + 7.0f, faceTo(x - 8.0f, z + 7.0f, x, z));
        } else if (p.kind == "village") {
            // Houses around the square, fronts toward it, gardens behind some of them.
            const float ring[][2] = {{-40, -30}, {-20, -55}, {25, -50}, {50, -20}, {55, 25}, {30, 55}, {-10, 60},
                                     {-45, 40}, {-70, 5}, {75, -45}, {-60, -60}, {10, -80}, {80, 50}, {-30, 85}};
            int k = 0;
            for (const auto& r : ring) {
                const float hx = x + r[0], hz = z + r[1];
                const float yaw = faceTo(hx, hz, x, z) + 0.15f * float(k % 3);
                const BuildingSpec& spec = villageHouses[(k * 5) % 6];
                Site site(*this);
                building(spec, hx, hz, yaw);
                if (k % 3 != 1) {
                    const float back = 0.5f * spec.d + 4.0f, gx = hx - std::sin(yaw) * back, gz = hz - std::cos(yaw) * back;
                    building({BK::Garden, 7.0f, 5.0f, 0, 0, k % 4}, gx, gz, yaw);
                }
                ++k;
            }
            // Square: well and benches.
            building({BK::Well, 0, 0, 0, 0, 0}, x + 5.0f, z + 5.0f, 0.4f);
            for (int b = 0; b < 4; ++b) {
                const float a = 0.5f * kPi * float(b) + 0.3f, bx = x + 13.0f * std::sin(a), bz = z + 13.0f * std::cos(a);
                building({BK::Bench, 1.8f, 0, 0, 0, 0}, bx, bz, faceTo(bx, bz, x, z));
            }
            // Fuel station by the main road: canopy with two pump islands, shop, price pylon.
            const float fx = x + 22.0f, fz = z - 8.0f;
            {
                Site site(*this);
                building({BK::FuelCanopy, 10.0f, 8.0f, 4.8f, 0, 0}, fx, fz, 0.0f);
                building({BK::FuelShop, 8.0f, 6.0f, 0.0f, 0, 0}, fx + 13.0f, fz + 1.0f, -0.5f * kPi);
                building({BK::Pylon, 0, 0, 6.5f, 0, 0}, fx - 8.5f, fz + 6.5f, 0.0f);
            }
            // Workshop with a woodpile.
            const float gx = x - 25.0f, gz = z + 12.0f;
            {
                Site site(*this);
                building({BK::Garage, 12.0f, 9.0f, 4.4f, 0, 0}, gx, gz, 0.1f + 0.5f * kPi);
                building({BK::Woodpile, 4.0f, 0, 0, 0, 0}, gx - 2.0f, gz + 6.5f, 0.1f);
            }
            // Power line along the nearest road, past the village.
            const LevelRoad* road = nullptr;
            std::size_t near = 0;
            float best = 1e30f;
            for (const LevelRoad& rd : L.m_roads)
                for (std::size_t i = 0; i < rd.samples.size(); ++i) {
                    const float d = std::hypot(rd.samples[i].x - x, rd.samples[i].z - z);
                    if (d < best) best = d, road = &rd, near = i;
                }
            if (road) {
                std::vector<P2> poles;
                const float off = 0.5f * road->width + 6.0f;
                for (int step = -6; step <= 6; ++step) {
                    const long j = long(near) + long(step) * 18; // samples are ~2 m apart: ~36 m spans
                    if (j < 0 || j >= long(road->samples.size())) continue;
                    const RoadSample& q = road->samples[std::size_t(j)];
                    poles.push_back({q.x + q.dz * off, q.z - q.dx * off});
                }
                powerLine(poles, 8.0f);
            }
        } else if (p.kind == "ruins") {
            // Curtain walls (center, half extents: the long side is the wall), two towers, the gate in the south gap.
            Site site(*this);
            const float walls[][5] = {{-16, -14, 16, 1.2f, 6.0f}, {-16, 14, 9, 1.2f, 3.5f}, {12, 14, 4, 1.2f, 5.0f},
                                      {-17, 0, 1.2f, 13, 4.5f}, {17, -6, 1.2f, 8, 7.0f}};
            int k = 0;
            for (const auto& w : walls) {
                const bool alongX = w[2] > w[3];
                building({BK::RuinWall, 2.0f * std::max(w[2], w[3]), 0, w[4], 0, k++}, x + w[0], z + w[1], alongX ? 0.0f : 0.5f * kPi);
            }
            building({BK::RuinTower, 6.4f, 0, 10.0f, 0, 1}, x - 17.0f, z - 14.0f, 0.3f);
            building({BK::RuinTower, 6.0f, 0, 6.5f, 0, 2}, x + 17.0f, z + 14.0f, 2.1f);
            building({BK::RuinGate, 4.0f, 0, 6.0f, 0, 0}, x + 0.5f, z + 14.0f, 0.0f);
            for (int r = 0; r < 6; ++r) {
                const float rx = x - 8.0f + 3.2f * float(r), rz = z + 10.0f - 2.0f * float(r % 2);
                rock(Vec3(rx, L.heightAt(rx, rz) - 0.2f, rz), Vec3(0.7f, 0.45f, 0.6f), float(r), 950u + r, PropStyle::Stone);
            }
        } else if (p.kind == "beach_camp") {
            {
                Site site(*this);
                building({BK::Tent, 0, 0, 0, 0, 0}, x + 6.0f, z - 4.0f, 0.4f);
                building({BK::Tent, 0, 0, 0, 0, 1}, x - 6.0f, z - 2.0f, -0.3f);
            }
            building({BK::Shelter, 5.0f, 4.0f, 0, 0, 0}, x + 14.0f, z - 12.0f, 0.5f);
            building({BK::Woodpile, 2.0f, 0, 0, 0, 0}, x + 4.5f, z + 8.5f, 0.9f);
            for (int r = 0; r < 7; ++r) {
                const float a = 2.0f * kPi * float(r) / 7.0f;
                const float rx = x + 1.0f + std::cos(a) * 1.0f, rz = z + 6.0f + std::sin(a) * 1.0f;
                rock(Vec3(rx, L.heightAt(rx, rz) - 0.05f, rz), Vec3(0.22f, 0.16f, 0.2f), a, 970u + r, PropStyle::DarkStone);
            }
            shoreDock(x - 18.0f, z + 26.0f, -8.0f, 54.0f, 22.0f, 3.0f);
        } else if (p.kind == "island_camp") {
            {
                Site site(*this);
                building({BK::Tent, 0, 0, 0, 0, 1}, x + 5.0f, z + 6.0f, 1.2f);
                building({BK::Tent, 0, 0, 0, 0, 0}, x - 4.0f, z + 8.0f, 0.2f);
            }
            building(shack, x + 12.0f, z + 14.0f, 0.9f);
            building({BK::Signpost, 0, 0, 0, 0, 1}, x - 8.0f, z - 4.0f, 0.0f);
        } else if (p.kind == "lighthouse") {
            building({BK::Lighthouse, 0, 0, 18.0f, 0, 0}, x, z, 0.0f);
            building(keeper, x - 9.5f, z - 2.0f, faceTo(x - 9.5f, z - 2.0f, p.atX, p.atZ));
        } else if (p.kind == "bridge") {
            building({BK::Signpost, 0, 0, 0, 0, 2}, p.atX + 4.0f, p.atZ + 5.0f, 1.4f);
        }
    }
    L.m_stats.buildings = L.m_buildings.size();

    // Trees and rocks baked before the buildings must not stand in them.
    const auto inStructure = [&](float px, float pz, float margin) {
        for (std::size_t si : m_structureSites)
            if (polygonDistance(m_sites[si], px, pz) < margin) return true;
        return false;
    };
    std::erase_if(L.m_trees, [&](const LevelTree& t) { return inStructure(t.x, t.z, 2.0f); });
    std::erase_if(L.m_rocks, [&](const LevelRock& r) { return inStructure(r.x, r.z, std::max(r.rx, r.rz) + 0.5f); });
}

std::uint16_t LevelBaker::buildingType(const BuildingSpec& spec) {
    for (std::size_t k = 0; k < L.m_buildingTypes.size(); ++k)
        if (L.m_buildingTypes[k].spec == spec) return std::uint16_t(k);
    BuildingGeometry g = buildBuilding(spec, false);
    LevelBuildingType t;
    t.spec = spec;
    t.boundsMin = g.boundsMin;
    t.boundsMax = g.boundsMax;
    t.footprintX = g.footprintX;
    t.footprintZ = g.footprintZ;
    L.m_buildingTypes.push_back(t);
    m_kitGeometry.push_back(std::move(g));
    return std::uint16_t(L.m_buildingTypes.size() - 1);
}

void LevelBaker::building(const BuildingSpec& spec, float x, float z, float yaw, float floorY) {
    const std::uint16_t type = buildingType(spec);
    const BuildingGeometry& g = m_kitGeometry[type];
    const float c = std::cos(yaw), sn = std::sin(yaw);
    const bool structure = buildingIsStructure(spec.kind);
    const bool onGround = floorY == kGroundFloor;
    if (onGround) floorY = groundFloor(type, x, z, yaw);
    Site site(*this);
    const std::size_t firstTri = L.m_props.geometry.triangleCount();
    const PropRole role = std::exchange(m_role, structure ? PropRole::Building : PropRole::Prop);
    const std::size_t triangles = g.collision.size() / 3;
    for (std::size_t piece = 0; piece < g.pieceStarts.size(); ++piece) {
        const std::size_t t0 = g.pieceStarts[piece];
        const std::size_t t1 = piece + 1 < g.pieceStarts.size() ? g.pieceStarts[piece + 1] : triangles;
        addProp([&](TrackGeometry& tg) {
            const auto P = [&](const Float3& q) { return Vec3(x + c * q.x + sn * q.z, floorY + q.y, z - sn * q.x + c * q.z); };
            for (std::size_t k = t0; k < t1; ++k)
                tg.addTriangle(P(g.collision[3 * k]), P(g.collision[3 * k + 1]), P(g.collision[3 * k + 2]), SurfaceId::Concrete);
        }, PropStyle::Hidden);
    }
    m_role = role;
    L.m_buildings.push_back({type, x, floorY, z, yaw});
    m_onGround.push_back(onGround);
    m_buildingTri.push_back(firstTri);
}

// Floor of a building standing on the ground, from a 3 x 3 sample grid over its bounds: structures on the highest
// sample (their plinth reaches down), tents on the lowest, small props in between.
// spread: highest minus lowest sample.
float LevelBaker::groundFloor(std::uint16_t type, float x, float z, float yaw, float* spread) const {
    const LevelBuildingType& t = L.m_buildingTypes[type];
    const float c = std::cos(yaw), sn = std::sin(yaw);
    float hi = -1e30f, lo = 1e30f;
    for (int i = 0; i <= 2; ++i)
        for (int j = 0; j <= 2; ++j) {
            const float lx = lerp(t.boundsMin.x, t.boundsMax.x, 0.5f * float(i));
            const float lz = lerp(t.boundsMin.z, t.boundsMax.z, 0.5f * float(j));
            const float h = L.heightAt(x + c * lx + sn * lz, z - sn * lx + c * lz);
            hi = std::max(hi, h), lo = std::min(lo, h);
        }
    const BuildingKind k = t.spec.kind;
    if (spread) *spread = hi - lo;
    return k == BuildingKind::Tent ? lo - 0.05f : buildingIsStructure(k) ? hi + 0.05f : 0.5f * (hi + lo);
}

// Plank dock from (x0, z0) toward (x1, z1), deck surface at deckY, posts down into the ground below.
void LevelBaker::dock(float x0, float z0, float x1, float z1, float deckY, float width) {
    const float len = std::hypot(x1 - x0, z1 - z0);
    float lo = deckY;
    for (int k = 0; k <= 8; ++k) {
        const float t = float(k) / 8.0f;
        lo = std::min(lo, L.heightAt(lerp(x0, x1, t), lerp(z0, z1, t)));
    }
    const float depth = std::max(1.0f, std::ceil(deckY - lo + 0.8f));
    building({BuildingKind::Dock, std::round(len), width, depth, 0, 0}, x0, z0, std::atan2(x1 - x0, z1 - z0), deckY);
}

// Dock starting 3 m before the first water met walking from (x, z) along (dirX, dirZ), reaching `reach` m past
// it, deck 0.8 m above that water. Returns the dock's shore end; nullopt (nothing placed) if no water within 120 m.
std::optional<P2> LevelBaker::shoreDock(float x, float z, float dirX, float dirZ, float reach, float width) {
    const float len = std::hypot(dirX, dirZ);
    if (len <= 0.0f) return std::nullopt;
    dirX /= len;
    dirZ /= len;
    for (float d = 0.0f; d < 120.0f; d += 0.5f) {
        const float wx = x + dirX * d, wz = z + dirZ * d;
        if (const auto w = L.waterAt(wx, wz)) {
            dock(wx - dirX * 3.0f, wz - dirZ * 3.0f, wx + dirX * reach, wz + dirZ * reach, *w + 0.8f, width);
            return P2{wx - dirX * 3.0f, wz - dirZ * 3.0f};
        }
    }
    return std::nullopt;
}

// Wooden poles at the given points (skipping those too near a road, water or a spawn point) with three wires
// between neighbors less than 50 m apart.
void LevelBaker::powerLine(const std::vector<P2>& poles, float height) {
    std::vector<P2> ok;
    std::vector<bool> joined;
    for (const P2& p : poles) {
        const bool clear = siteClear({p}, kPropMargin + 1.0f, false, false) && !L.waterAt(p.x, p.z);
        if (!clear) {
            if (!joined.empty()) joined.back() = false;
            continue;
        }
        ok.push_back(p);
        joined.push_back(true);
    }
    for (std::size_t k = 0; k < ok.size(); ++k) {
        const P2 a = ok[k];
        const bool span = k + 1 < ok.size() && joined[k] && std::hypot(ok[k + 1].x - a.x, ok[k + 1].z - a.z) < 50.0f;
        const P2 dirTo = span ? ok[k + 1] : k > 0 ? P2{2.0f * a.x - ok[k - 1].x, 2.0f * a.z - ok[k - 1].z} : P2{a.x, a.z + 1.0f};
        const float yaw = std::atan2(dirTo.x - a.x, dirTo.z - a.z);
        const float ya = L.heightAt(a.x, a.z);
        Site site(*this);
        building({BuildingKind::PowerPole, 0, 0, height, 0, int(k % 4 == 2)}, a.x, a.z, yaw, ya);
        if (span) {
            const P2 b = ok[k + 1];
            const float yb = L.heightAt(b.x, b.z);
            building({BuildingKind::Wire, std::hypot(b.x - a.x, b.z - a.z), 0, ya - yb, 0, 0}, a.x, a.z, yaw, ya + height - 0.24f);
        }
    }
}

// ------------------------------------------------------------------------------------------------------------------
void LevelBaker::bake() {
    const auto t0 = std::chrono::steady_clock::now();
    m_cell = 1.0f;
    m_origin = -m_half;
    m_n = int(std::lround(2.0f * m_half / m_cell)) + 1;
    L.m_n = m_n;
    L.m_origin = m_origin;
    L.m_cell = m_cell;
    L.m_snowLine = m_snowLine;
    const std::size_t total = std::size_t(m_n) * std::size_t(m_n);
    m_h.assign(total, 0.0f);
    m_lakeId.assign(total, 0);
    m_riverD.assign(total, 1e30f);
    m_riverY.assign(total, 0.0f);
    m_riverReach.assign(total, 0.0f);
    m_roadness.assign(total, 0.0f);
    m_roadSurface.assign(total, SurfaceId::PackedDirt);
    m_roadEdge.assign(total, 1e30f);
    m_roadClear.assign(total, 1e30f);
    m_flatSurface.assign(total, SurfaceId::Count);
    m_nearWater.assign(total, 0);
    if (m_baked) return bakeBaked(t0);

    bakeBase();
    bakeCoast();
    bakeFlats();
    const std::vector<float> uncarved = m_h;
    bakeLakes();
    bakeRivers();
    bakeBankSmoothing(uncarved);
    bakeRoads();
    bakeCutBanks();
    L.m_height = m_h;
    buildRoadPoints();
    bakeWater();
    bakeSurfaces();
    bakeWaterDistance();
    bakeTrees();
    bakeCanopy();
    bakeRocks();
    bakeProps();
    bakeLogs();
    bakeCliffs();
    bakeTalus();
    bakeScatter();
    L.m_props.styles.resize(L.m_props.geometry.triangleCount(), PropStyle::Stone);

    // Grass mask clear under the rocks and props footprint is handled by the render masks (rocks list).
    LevelBakeStats& st = L.m_stats;
    st.samples = m_n;
    st.trees = L.m_trees.size();
    st.rocks = L.m_rocks.size();
    st.scatter = L.m_scatter.size();
    st.propTriangles = L.m_props.geometry.triangleCount();
    const auto [mn, mx] = std::minmax_element(L.m_height.begin(), L.m_height.end());
    st.minHeight = *mn;
    st.maxHeight = *mx;
    st.totalMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// ------------------------------------------------------------------------------------------------- baked levels
// level.json "baked": {"heightmap", "surfaces", "layout"} names files next to level.json, written by
// tools/levels/blender/export_level.py. The layout grid must match bakeHalfExtent (1 m cells).
bool LevelBaker::parseBaked(const json::Value& doc, std::string& err) {
    if (m_baseDir.empty()) {
        err = "level: a baked level is read from its folder (Level::load)";
        return false;
    }
    m_baked = true;
    L.m_baked = true;
    m_snowLine = 1.0e9f;
    L.m_asphaltLayer = doc["asphaltLayer"].boolOr(false);
    m_routeName = doc["attractRoute"]["name"].stringOr("circuit");
    m_routeSpeed = float(doc["attractRoute"]["speedKmh"].numberOr(40.0)) / 3.6f;
    m_routeLateral = float(doc["attractRoute"]["lateralAccel"].numberOr(0.0));
    m_routeDecel = float(doc["attractRoute"]["brakeDecel"].numberOr(0.0));
    m_routeThrottle = float(doc["attractRoute"]["maxThrottle"].numberOr(0.0));
    const json::Value& files = doc["baked"];
    const std::string dir = m_baseDir + "/";
    std::ifstream lf(dir + files["layout"].stringOr("layout.json"), std::ios::binary);
    if (!lf) {
        err = "level: cannot open the baked layout " + dir + files["layout"].stringOr("layout.json");
        return false;
    }
    std::stringstream ls;
    ls << lf.rdbuf();
    json::Value lay;
    if (!json::parse(ls.str(), lay, &err)) {
        err = "level: layout: " + err;
        return false;
    }
    if (int(lay["format"].numberOr(0)) != 1) {
        err = "level: layout: unsupported format (expected 1)";
        return false;
    }
    const json::Value& grid = lay["grid"];
    const int n = int(std::lround(2.0f * m_half)) + 1;
    if (int(grid["samples"].numberOr(0)) != n || float(grid["cell"].numberOr(0)) != 1.0f ||
        float(grid["origin"].numberOr(0)) != -m_half) {
        err = "level: layout grid does not match bakeHalfExtent (" + std::to_string(n) + " samples at 1 m)";
        return false;
    }
    const float lo = float(grid["heightMin"].numberOr(0)), hi = float(grid["heightMax"].numberOr(0));
    if (!(hi > lo)) {
        err = "level: layout heightMin / heightMax";
        return false;
    }
    PgmImage hm, sm;
    if (!readPgm(dir + files["heightmap"].stringOr("heightmap.pgm"), hm, err) ||
        !readPgm(dir + files["surfaces"].stringOr("surfaces.pgm"), sm, err)) {
        err = "level: " + err;
        return false;
    }
    if (hm.width != n || hm.height != n || hm.maxValue != 65535) {
        err = "level: the heightmap must be " + std::to_string(n) + "^2 16-bit samples";
        return false;
    }
    if (sm.width != n - 1 || sm.height != n - 1 || sm.maxValue != 255) {
        err = "level: the surface map must be " + std::to_string(n - 1) + "^2 8-bit cells";
        return false;
    }
    const float step = (hi - lo) / 65535.0f;
    m_bakedHeight.resize(hm.data.size());
    for (std::size_t k = 0; k < hm.data.size(); ++k) m_bakedHeight[k] = lo + float(hm.data[k]) * step;
    m_bakedClass.resize(sm.data.size());
    for (std::size_t k = 0; k < sm.data.size(); ++k) {
        if (sm.data[k] >= std::uint16_t(LevelSurfaceClass::Count)) {
            err = "level: surface map value " + std::to_string(sm.data[k]) + " is not a surface class";
            return false;
        }
        m_bakedClass[k] = std::uint8_t(sm.data[k]);
    }

    for (const json::Value& r : lay["roads"].array) {
        BakedRoad road{r["name"].stringOr("road"), r["kind"].stringOr("road"), float(r["width"].numberOr(6)),
                       r["closed"].boolOr(false), {}, {}};
        for (const json::Value& p : r["samples"].array)
            if (p.isArray() && p.array.size() >= 3)
                road.samples.push_back(Float3(float(p.array[0].number), float(p.array[1].number), float(p.array[2].number)));
        if (road.samples.size() < 4 || road.width < 2.0f) {
            err = "level: layout road " + road.name + " needs 4 samples and width >= 2 m";
            return false;
        }
        if (const json::Value& mk = r["markings"]; mk.isObject()) {
            BakedMarkings& m = road.markings;
            m.edgeLine = float(mk["edgeLine"].numberOr(0));
            m.edgeInset = float(mk["edgeInset"].numberOr(0.3));
            m.startLine = float(mk["startLine"].numberOr(-1));
            m.gridSlots = int(mk["gridSlots"].numberOr(0));
            m.gridPitch = float(mk["gridPitch"].numberOr(8));
            for (const json::Value& c : mk["curbs"].array)
                if (c.isArray() && c.array.size() == 3)
                    m.curbs.push_back({float(c.array[0].number), float(c.array[1].number), c.array[2].number < 0 ? -1.0f : 1.0f});
        }
        if (road.kind == "circuit") m_circuitRoad = road.name;
        m_bakedRoads.push_back(std::move(road));
    }
    for (const json::Value& w : lay["water"].array) {
        Lake lake{w["name"].stringOr("water"), float(w["level"].numberOr(0)), 0.0f, readPolygon(w["polygon"]), 0.0f};
        if (lake.poly.p.size() < 3) {
            err = "level: layout water " + lake.name + " needs a polygon";
            return false;
        }
        m_lakes.push_back(std::move(lake));
    }
    for (const json::Value& p : lay["placements"].array) {
        BakedPlacement b;
        b.kind = p["kind"].stringOr("");
        b.model = p["model"].stringOr("");
        b.x = float(p["x"].numberOr(0));
        b.z = float(p["z"].numberOr(0));
        b.yaw = float(p["yaw"].numberOr(0));
        if (b.kind == "tree") {
            b.scale = float(p["scale"].numberOr(1));
            b.width = float(p["width"].numberOr(1));
        } else if (b.kind == "rock") {
            const json::Value& sc = p["scale"];
            if (sc.isArray() && sc.array.size() == 3) {
                b.sx = float(sc.array[0].number);
                b.sy = float(sc.array[1].number);
                b.sz = float(sc.array[2].number);
            }
            b.sink = float(p["sink"].numberOr(0.3));
        } else if (b.kind == "building") {
            const std::string type = p["type"].stringOr("");
            b.spec.kind = BuildingKind::Count;
            for (int k = 0; k < int(BuildingKind::Count); ++k)
                if (type == buildingKindName(BuildingKind(k))) b.spec.kind = BuildingKind(k);
            if (b.spec.kind == BuildingKind::Count) {
                err = "level: layout building type '" + type + "' is not a kit kind";
                return false;
            }
            b.spec.w = float(p["w"].numberOr(8));
            b.spec.d = float(p["d"].numberOr(8));
            b.spec.h = float(p["h"].numberOr(0));
            b.spec.floors = int(p["floors"].numberOr(1));
            b.spec.variant = int(p["variant"].numberOr(0));
        } else {
            err = "level: layout placement kind '" + b.kind + "' (tree, rock or building)";
            return false;
        }
        m_placements.push_back(std::move(b));
    }
    for (const json::Value& g : lay["checkpoints"].array) {
        const float yaw = float(g["yaw"].numberOr(0));
        m_gates.push_back({float(g["x"].numberOr(0)), float(g["z"].numberOr(0)), std::sin(yaw), std::cos(yaw),
                           float(g["halfWidth"].numberOr(8)), g["sector"].boolOr(false)});
    }
    if (!m_gates.empty() && m_circuitRoad.empty()) {
        err = "level: layout checkpoints need a road of kind circuit";
        return false;
    }
    for (const json::Value& p : lay["pois"].array) {
        LevelPoi poi;
        poi.id = int(p["id"].numberOr(0));
        poi.name = p["name"].stringOr("POI");
        poi.kind = p["kind"].stringOr("");
        poi.x = float(p["x"].numberOr(0));
        poi.z = float(p["z"].numberOr(0));
        if (!readPoint(p["at"], poi.atX, poi.atZ)) {
            poi.atX = poi.x;
            poi.atZ = poi.z;
        }
        L.m_pois.push_back(poi);
    }
    const json::Value& sp = lay["spawn"];
    L.m_spawnX = float(sp["x"].numberOr(0));
    L.m_spawnZ = float(sp["z"].numberOr(0));
    m_spawnYaw = float(sp["yaw"].numberOr(0));
    L.m_spawnTowardX = L.m_spawnX + std::sin(m_spawnYaw);
    L.m_spawnTowardZ = L.m_spawnZ + std::cos(m_spawnYaw);
    if (std::abs(L.m_spawnX) > L.m_playHalf || std::abs(L.m_spawnZ) > L.m_playHalf) {
        err = "level: spawn outside the play area";
        return false;
    }
    return loadModels(doc["vegetation"], err);
}

void LevelBaker::bakeBaked(std::chrono::steady_clock::time_point t0) {
    m_h = m_bakedHeight;
    // Roads from the exported center line samples (every ~2 m; a closed road repeats its first sample at the end).
    for (const BakedRoad& def : m_bakedRoads) {
        LevelRoad road;
        road.name = def.name;
        road.trail = def.kind == "trail";
        road.width = def.width;
        std::vector<Float3> pts = def.samples;
        if (def.closed) pts.push_back(pts.front());
        const std::size_t n = pts.size();
        road.samples.resize(n);
        float s = 0.0f;
        for (std::size_t i = 0; i < n; ++i) {
            RoadSample& r = road.samples[i];
            r.x = pts[i].x;
            r.z = pts[i].y;
            r.bank = pts[i].z;
            if (i > 0) s += std::hypot(r.x - pts[i - 1].x, r.z - pts[i - 1].y);
            r.s = s;
            std::size_t a = i > 0 ? i - 1 : (def.closed ? n - 2 : i), b = i + 1 < n ? i + 1 : (def.closed ? 1 : i);
            const float dx = pts[b].x - pts[a].x, dz = pts[b].y - pts[a].y, len = std::max(1e-4f, std::hypot(dx, dz));
            r.dx = dx / len;
            r.dz = dz / len;
            r.y = sampleGrid(m_h, r.x, r.z);
        }
        if (def.name == m_circuitRoad) L.m_circuit.length = s;
        L.m_roads.push_back(std::move(road));
    }
    bakeRoads(); // corridor distances only: no road definitions to profile, no carving
    for (std::size_t li = 0; li < m_lakes.size(); ++li) {
        const Polygon& poly = m_lakes[li].poly;
        for (int j = std::max(0, int((poly.z0 - m_origin) / m_cell)); j <= std::min(m_n - 1, int((poly.z1 - m_origin) / m_cell) + 1); ++j)
            for (int i = std::max(0, int((poly.x0 - m_origin) / m_cell)); i <= std::min(m_n - 1, int((poly.x1 - m_origin) / m_cell) + 1); ++i)
                if (poly.inside(X(i), X(j))) m_lakeId[idx(i, j)] = std::uint8_t(li + 1);
    }
    L.m_height = m_h;
    buildRoadPoints();
    bakeWater();
    bakeBakedSurfaces();
    bakeWaterDistance();
    placeBaked();
    bakeCanopy();
    // Forest floor for the ground cover: the density of the placed trees (about 1 within 9 m of four trees).
    {
        const int cn = m_n - 1;
        m_forest.assign(std::size_t(cn) * std::size_t(cn), 0.0f);
        for (const LevelTree& t : L.m_trees) {
            const float r = 9.0f;
            const int i0 = std::max(0, int((t.x - r - m_origin) / m_cell)), i1 = std::min(cn - 1, int((t.x + r - m_origin) / m_cell));
            const int j0 = std::max(0, int((t.z - r - m_origin) / m_cell)), j1 = std::min(cn - 1, int((t.z + r - m_origin) / m_cell));
            for (int j = j0; j <= j1; ++j)
                for (int i = i0; i <= i1; ++i) {
                    const float d = std::hypot(X(i) + 0.5f - t.x, X(j) + 0.5f - t.z);
                    float& f = m_forest[std::size_t(j) * std::size_t(cn) + std::size_t(i)];
                    f = std::min(1.0f, f + 0.25f * (1.0f - smoothstep(0.5f * r, r, d)));
                }
        }
    }
    bakeScatter();
    L.m_props.styles.resize(L.m_props.geometry.triangleCount(), PropStyle::Stone);
    for (std::size_t k = 0; k < m_bakedRoads.size(); ++k) bakeMarkings(L.m_roads[k], m_bakedRoads[k].markings);
    L.m_surfaceClass = m_bakedClass;

    // Circuit gates and the attract route: the circuit center line as a closed loop.
    L.m_circuit.road = m_circuitRoad;
    L.m_circuit.gates = m_gates;
    for (const LevelRoad& r : L.m_roads)
        if (r.name == m_circuitRoad) {
            DriveRoute route;
            route.name = m_routeName;
            route.cruiseSpeed = m_routeSpeed;
            route.lateralAccel = m_routeLateral;
            route.brakeDecel = m_routeDecel;
            route.maxThrottle = m_routeThrottle;
            for (const RoadSample& p : r.samples) route.points.push_back({p.x, p.z});
            if (route.points.size() > 1) route.points.pop_back(); // the repeated first sample
            // Start at the sample nearest to the spawn (the autopilot recovers to the route start).
            std::size_t best = 0;
            float bd = 1e30f;
            for (std::size_t k = 0; k < route.points.size(); ++k) {
                const float d = std::hypot(route.points[k].x - L.m_spawnX, route.points[k].z - L.m_spawnZ);
                if (d < bd) bd = d, best = k;
            }
            std::rotate(route.points.begin(), route.points.begin() + std::ptrdiff_t(best), route.points.end());
            L.m_attractRoute = std::move(route);
        }
    checkBakedLayout();

    LevelBakeStats& st = L.m_stats;
    st.samples = m_n;
    st.trees = L.m_trees.size();
    st.rocks = L.m_rocks.size();
    st.scatter = L.m_scatter.size();
    st.buildings = L.m_buildings.size();
    st.propTriangles = L.m_props.geometry.triangleCount();
    const auto [mn, mx] = std::minmax_element(L.m_height.begin(), L.m_height.end());
    st.minHeight = *mn;
    st.maxHeight = *mx;
    st.totalMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// Physics surface and render masks from the exported surface classes. Gravel is the Gravel surface drawn with the
// road gravel layer; asphalt is the Asphalt surface (drawn through the splat snow weight, asphaltLayer).
void LevelBaker::bakeBakedSurfaces() {
    const int cn = m_n - 1;
    const std::size_t cells = std::size_t(cn) * std::size_t(cn);
    L.m_surface.assign(cells, SurfaceId::Grass);
    m_forest.assign(cells, 0.0f);
    std::vector<std::uint32_t> mask(cells), splat(cells);
    std::vector<float> keep(cells, 1.0f);
    parallelFor(cn, m_threads, [&](int j) {
        for (int i = 0; i < cn; ++i) {
            const std::size_t c = std::size_t(j) * std::size_t(cn) + std::size_t(i);
            const float x = X(i) + 0.5f * m_cell, z = X(j) + 0.5f * m_cell;
            const std::size_t a = idx(i, j), b = idx(i + 1, j), cc = idx(i, j + 1), d = idx(i + 1, j + 1);
            const LevelSurfaceClass cls = LevelSurfaceClass(m_bakedClass[c]);
            SurfaceId s = SurfaceId::Grass;
            switch (cls) {
            case LevelSurfaceClass::Asphalt: s = SurfaceId::Asphalt; break;
            case LevelSurfaceClass::Gravel: s = SurfaceId::Gravel; break;
            case LevelSurfaceClass::Dirt: s = SurfaceId::PackedDirt; break;
            case LevelSurfaceClass::Mud: s = SurfaceId::Mud; break;
            case LevelSurfaceClass::Sand: s = SurfaceId::Sand; break;
            case LevelSurfaceClass::Rock: s = SurfaceId::Rock; break;
            case LevelSurfaceClass::Snow: s = SurfaceId::Snow; break;
            default: break;
            }
            L.m_surface[c] = s;
            float wd = 0.0f;
            for (std::size_t q : {a, b, cc, d})
                if (L.m_water[q] > Level::kNoWater) wd += 0.25f * std::max(0.0f, L.m_water[q] - m_h[q]);
            const float near = float(std::max({m_nearWater[a], m_nearWater[b], m_nearWater[cc], m_nearWater[d]})) / 255.0f;
            const float meadow = fbm(x / 40.0f, z / 40.0f, 3, m_seed + 67u);
            const float patch = fbm(x / 16.0f, z / 16.0f, 3, m_seed + 61u);
            const float clear = std::min({m_roadClear[a], m_roadClear[b], m_roadClear[cc], m_roadClear[d]});
            keep[c] = smoothstep(1.5f, 2.7f, clear);
            float grass = 0.0f;
            if (cls == LevelSurfaceClass::Grass) grass = 0.35f + 0.65f * smoothstep(0.3f, 0.7f, meadow);
            else if (cls == LevelSurfaceClass::Dirt && clear > 1.0f) grass = 0.12f * smoothstep(0.4f, 0.7f, patch);
            const float paint = cls == LevelSurfaceClass::Gravel ? 1.0f : 0.0f;
            const float rock = cls == LevelSurfaceClass::Rock ? 1.0f : 0.0f;
            const float sand = cls == LevelSurfaceClass::Sand ? 1.0f : 0.0f;
            const float dark = cls == LevelSurfaceClass::Snow || (cls == LevelSurfaceClass::Asphalt && L.m_asphaltLayer) ? 1.0f : 0.0f;
            const float mud = cls == LevelSurfaceClass::Mud ? 1.0f : 0.0f;
            const float wet = wd > 0.0f ? 1.0f : std::max(0.55f * near, 0.35f * mud);
            mask[c] = packRgba(grass, mud, wet, 0.0f);
            splat[c] = packRgba(paint, rock, 0.5f + 0.5f * (sand - dark), 0.0f);
        }
    });
    auto blur = [&](const std::vector<std::uint32_t>& src, std::vector<std::uint32_t>& dst) {
        dst.resize(src.size());
        parallelFor(cn, m_threads, [&](int j) {
            for (int i = 0; i < cn; ++i) {
                float acc[4] = {0, 0, 0, 0};
                for (int dj = -1; dj <= 1; ++dj)
                    for (int di = -1; di <= 1; ++di) {
                        const std::uint32_t v = src[std::size_t(std::clamp(j + dj, 0, cn - 1)) * std::size_t(cn) +
                                                    std::size_t(std::clamp(i + di, 0, cn - 1))];
                        for (int k = 0; k < 4; ++k) acc[k] += channel(v, k);
                    }
                dst[std::size_t(j) * std::size_t(cn) + std::size_t(i)] =
                    packRgba(acc[0] / 9.0f, acc[1] / 9.0f, acc[2] / 9.0f, acc[3] / 9.0f);
            }
        });
    };
    blur(mask, L.m_mask);
    blur(splat, L.m_splat);
    for (std::size_t c = 0; c < cells; ++c)
        if (keep[c] < 1.0f) {
            const std::uint32_t v = L.m_mask[c];
            L.m_mask[c] = packRgba(channel(v, 0) * keep[c], channel(v, 1), channel(v, 2), channel(v, 3) * keep[c]);
        }
}

// Trees, rocks and kit buildings where the layout puts them, on the ground: trees 0.3 m into it, rocks by their
// sink fraction of the height, buildings by the kit floor rule (groundFloor).
void LevelBaker::placeBaked() {
    const bool library = !L.m_models.empty();
    auto model = [&](const std::string& ref, const char* kind) -> int {
        const auto it = m_modelByRef.find(ref);
        if (!library || it == m_modelByRef.end()) return -1;
        return L.m_models[std::size_t(it->second)].kind == kind ? it->second : -1;
    };
    for (const BakedPlacement& p : m_placements) {
        if (p.kind == "tree") {
            LevelTree t;
            t.x = p.x;
            t.z = p.z;
            t.y = L.heightAt(p.x, p.z) - 0.3f;
            t.yaw = p.yaw;
            t.width = p.width;
            t.hue = hashUnit(int(std::floor(p.x * 4.0f)), int(std::floor(p.z * 4.0f)), m_seed + 131u);
            const int id = model(p.model, "tree");
            if (id >= 0) {
                const LevelModel& m = L.m_models[std::size_t(id)];
                t.model = std::int16_t(id);
                t.scale = p.scale;
                t.height = m.height * p.scale;
                t.kind = std::find(m_broadleaf.begin(), m_broadleaf.end(), id) != m_broadleaf.end() ? TreeKind::Broadleaf
                         : std::find(m_saplings.begin(), m_saplings.end(), id) != m_saplings.end() ? TreeKind::Sapling
                                                                                                    : TreeKind::Conifer;
            } else {
                if (library) L.m_layoutIssues.push_back("tree model '" + p.model + "' is not in the vegetation library");
                t.height = 12.0f * p.scale; // procedural pine
            }
            L.m_trees.push_back(t);
        } else if (p.kind == "rock") {
            LevelRock r;
            r.x = p.x;
            r.z = p.z;
            r.yaw = p.yaw;
            r.seed = hash3(int(std::floor(p.x * 4.0f)), int(std::floor(p.z * 4.0f)), m_seed + 227u);
            const int id = model(p.model, "rock") >= 0 ? model(p.model, "rock") : model(p.model, "log");
            if (id >= 0) {
                const LevelModel& m = L.m_models[std::size_t(id)];
                r.model = std::int16_t(id);
                r.sx = p.sx;
                r.sy = p.sy;
                r.sz = p.sz;
                r.rx = 0.5f * (m.boundsMax.x - m.boundsMin.x) * p.sx;
                r.ry = 0.5f * (m.boundsMax.y - m.boundsMin.y) * p.sy;
                r.rz = 0.5f * (m.boundsMax.z - m.boundsMin.z) * p.sz;
            } else {
                if (library) L.m_layoutIssues.push_back("rock model '" + p.model + "' is not in the vegetation library");
                r.rx = p.sx, r.ry = p.sy, r.rz = p.sz; // procedural rock of about the model size
            }
            r.y = L.heightAt(p.x, p.z) - p.sink * r.ry;
            L.m_rocks.push_back(r);
        } else {
            building(p.spec, p.x, p.z, p.yaw);
        }
    }
}

// Curbs and painted lines on the baked terrain (props: collision == render). Every piece follows the heightfield
// (vertices at heightAt): paint is a 4 mm slab, a curb a 1 m long prism 1.15 m wide and 4.5 cm high whose inner
// edge starts 0.15 m inside the road edge, red and white per metre.
void LevelBaker::bakeMarkings(const LevelRoad& road, const BakedMarkings& mk) {
    if (road.samples.size() < 2 || (mk.edgeLine <= 0.0f && mk.startLine < 0.0f && mk.curbs.empty())) return;
    const std::vector<RoadSample>& sm = road.samples;
    const float length = sm.back().s, hw = 0.5f * road.width;
    struct Frame { float x, z, dx, dz; };
    auto at = [&](float s) {
        s = std::fmod(s, length);
        if (s < 0.0f) s += length;
        const auto it = std::upper_bound(sm.begin(), sm.end(), s, [](float v, const RoadSample& r) { return v < r.s; });
        const std::size_t b = std::min<std::size_t>(std::size_t(it - sm.begin()), sm.size() - 1), a = b > 0 ? b - 1 : 0;
        const float t = sm[b].s > sm[a].s ? (s - sm[a].s) / (sm[b].s - sm[a].s) : 0.0f;
        const float dx = sm[a].dx + (sm[b].dx - sm[a].dx) * t, dz = sm[a].dz + (sm[b].dz - sm[a].dz) * t;
        const float l = std::max(1e-6f, std::hypot(dx, dz));
        return Frame{sm[a].x + (sm[b].x - sm[a].x) * t, sm[a].z + (sm[b].z - sm[a].z) * t, dx / l, dz / l};
    };
    // Point u m left of the center line at s, h m above the ground.
    auto P = [&](float s, float u, float h) {
        const Frame f = at(s);
        const float x = f.x + f.dz * u, z = f.z - f.dx * u;
        return Vec3(x, L.heightAt(x, z) + h, z);
    };
    TrackGeometry& g = L.m_props.geometry;
    auto piece = [&](PropStyle style) {
        L.m_props.pieceStarts.push_back(std::uint32_t(g.triangleCount()));
        L.m_props.pieceRoles.push_back(PropRole::Prop);
        (void)style;
    };
    auto done = [&](PropStyle style) { L.m_props.styles.resize(g.triangleCount(), style); };
    constexpr float kPaint = 0.004f;
    // Paint quad between (s0, u0) and (s1, u1): top face only (it lies on the asphalt).
    auto paint = [&](float s0, float s1, float u0, float u1) {
        piece(PropStyle::LinePaint);
        const Vec3 a = P(s0, u0, kPaint), b = P(s0, u1, kPaint), c = P(s1, u1, kPaint), d = P(s1, u0, kPaint);
        // +u is left: (s0, u1) -> (s1, u1) -> (s1, u0) is counter-clockwise seen from above.
        g.addQuad(a, d, c, b, SurfaceId::Asphalt);
        done(PropStyle::LinePaint);
    };
    if (mk.edgeLine > 0.0f) {
        const float u1 = hw - mk.edgeInset, u0 = u1 - mk.edgeLine;
        const int steps = std::max(1, int(std::ceil(length)));
        for (int k = 0; k < steps; ++k) {
            const float s0 = length * float(k) / float(steps), s1 = length * float(k + 1) / float(steps);
            paint(s0, s1, u0, u1);
            paint(s0, s1, -u1, -u0);
        }
    }
    if (mk.startLine >= 0.0f) {
        const float u1 = hw - mk.edgeInset - mk.edgeLine;
        for (float u = -u1; u < u1 - 1e-3f; u += 1.0f) paint(mk.startLine - 0.3f, mk.startLine + 0.3f, u, std::min(u1, u + 1.0f));
        // Staggered grid boxes: an open bracket (front bar and two side legs) per slot, pole on the left.
        for (int k = 0; k < mk.gridSlots; ++k) {
            const float s = mk.startLine - 6.0f - float(k) * 0.5f * mk.gridPitch;
            const float c = (k % 2 == 0 ? 1.0f : -1.0f) * 0.25f * road.width;
            paint(s - 0.2f, s, c - 1.1f, c + 1.1f);
            paint(s - 1.5f, s - 0.2f, c - 1.1f, c - 0.95f);
            paint(s - 1.5f, s - 0.2f, c + 0.95f, c + 1.1f);
        }
    }
    // Curb cross-section: distance outward from 0.15 m inside the edge and height above the ground.
    static constexpr float kCurbU[4] = {0.0f, 0.25f, 0.95f, 1.15f};
    static constexpr float kCurbH[4] = {0.002f, 0.045f, 0.045f, 0.0f};
    for (const BakedMarkings::Curb& c : mk.curbs) {
        const float s1 = c.s1 >= c.s0 ? c.s1 : c.s1 + length;
        const int n = std::max(1, int(std::lround(s1 - c.s0)));
        for (int k = 0; k < n; ++k) {
            const float a = c.s0 + (s1 - c.s0) * float(k) / float(n), b = c.s0 + (s1 - c.s0) * float(k + 1) / float(n);
            const PropStyle style = ((int(std::floor(a)) + int(c.side > 0)) % 2) ? PropStyle::CurbRed : PropStyle::CurbWhite;
            piece(style);
            Vec3 top[2][4], bot[2][4];
            for (int e = 0; e < 2; ++e)
                for (int q = 0; q < 4; ++q) {
                    const float u = c.side * (hw - 0.15f + kCurbU[q]);
                    top[e][q] = P(e ? b : a, u, kCurbH[q]);
                    bot[e][q] = P(e ? b : a, u, -0.05f);
                }
            const Vec3 centre = 0.25f * (top[0][1] + top[0][2] + top[1][1] + top[1][2]) - Vec3(0, 0.03f, 0);
            auto tri = [&](Vec3 p0, Vec3 p1, Vec3 p2) {
                if ((p1 - p0).Cross(p2 - p0).Dot((p0 + p1 + p2) / 3.0f - centre) < 0.0f) std::swap(p1, p2);
                g.addTriangle(p0, p1, p2, SurfaceId::Asphalt);
            };
            auto quad = [&](const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3) { tri(p0, p1, p2), tri(p0, p2, p3); };
            for (int q = 0; q < 3; ++q) quad(top[0][q], top[0][q + 1], top[1][q + 1], top[1][q]); // ramp, top, ramp
            quad(top[0][0], top[1][0], bot[1][0], bot[0][0]);                                       // inner side
            quad(top[0][3], top[1][3], bot[1][3], bot[0][3]);                                       // outer side
            for (int e = 0; e < 2; ++e) {                                                           // end caps
                quad(bot[e][0], top[e][0], top[e][1], bot[e][1]);
                quad(bot[e][1], top[e][1], top[e][2], bot[e][2]);
                quad(bot[e][2], top[e][2], top[e][3], bot[e][3]);
            }
            done(style);
        }
    }
}

float LevelBaker::roadGap(const std::vector<P2>& hull) const {
    float x0 = 1e30f, z0 = 1e30f, x1 = -1e30f, z1 = -1e30f;
    for (const P2& p : hull) x0 = std::min(x0, p.x), z0 = std::min(z0, p.z), x1 = std::max(x1, p.x), z1 = std::max(z1, p.z);
    const float reach = 40.0f;
    float best = 1e30f;
    const int bi0 = std::max(0, int((x0 - reach - m_origin) / 32.0f)), bi1 = std::min(m_bucketSide - 1, int((x1 + reach - m_origin) / 32.0f));
    const int bj0 = std::max(0, int((z0 - reach - m_origin) / 32.0f)), bj1 = std::min(m_bucketSide - 1, int((z1 + reach - m_origin) / 32.0f));
    for (int bj = bj0; bj <= bj1; ++bj)
        for (int bi = bi0; bi <= bi1; ++bi)
            for (const RoadPoint& p : m_roadBuckets[std::size_t(bj) * std::size_t(m_bucketSide) + std::size_t(bi)])
                best = std::min(best, polygonDistance(hull, p.x, p.z) - p.hw);
    return best;
}

// Rules for authored layouts (the same as export_level.py validates): nothing on a road (trees and rocks 1 m from
// the edge, kit props 0.3 m, structures 3 m), structures out of the water, everything in the play area, the spawn
// and the gates on the circuit, gates spanning the road, the water on the circuit at most 0.45 m deep.
void LevelBaker::checkBakedLayout() {
    std::vector<std::string>& out = L.m_layoutIssues;
    char buf[256];
    auto inPlay = [&](float x, float z, const char* what) {
        if (L.insidePlayArea(x, z)) return;
        std::snprintf(buf, sizeof(buf), "%s at (%.1f, %.1f) is outside the play area", what, double(x), double(z));
        out.push_back(buf);
    };
    for (const LevelTree& t : L.m_trees) {
        inPlay(t.x, t.z, "tree");
        float r = 0.3f;
        if (t.model >= 0) r = L.m_models[std::size_t(t.model)].trunkRadius * t.scale * t.width;
        const float gap = roadGap({P2{t.x, t.z}}) - r;
        if (gap < 1.0f) {
            std::snprintf(buf, sizeof(buf), "tree at (%.1f, %.1f) is %.2f m from a road edge (min 1 m)", double(t.x), double(t.z), double(gap));
            out.push_back(buf);
        }
    }
    for (const LevelRock& r : L.m_rocks) {
        inPlay(r.x, r.z, "rock");
        const float gap = roadGap({P2{r.x, r.z}}) - std::max(r.rx, r.rz);
        if (gap < 1.0f) {
            std::snprintf(buf, sizeof(buf), "rock at (%.1f, %.1f) is %.2f m from a road edge (min 1 m)", double(r.x), double(r.z), double(gap));
            out.push_back(buf);
        }
    }
    for (const LevelBuilding& b : L.m_buildings) {
        const LevelBuildingType& t = L.m_buildingTypes[b.type];
        const float c = std::cos(b.yaw), sn = std::sin(b.yaw);
        std::vector<P2> hull;
        for (float u : {t.boundsMin.x, t.boundsMax.x})
            for (float v : {t.boundsMin.z, t.boundsMax.z}) hull.push_back({b.x + c * u + sn * v, b.z - sn * u + c * v});
        hull = convexHull(hull);
        const bool structure = buildingIsStructure(t.spec.kind);
        const float need = structure ? kBuildingMargin : 0.3f;
        float gap = roadGap(hull);
        if (t.spec.kind == BuildingKind::StartGantry) { // spans the road: only its pillars stand beside it
            gap = 1e30f;
            for (float sx : {-1.0f, 1.0f}) {
                std::vector<P2> pillar;
                const float u0 = sx * (0.5f * t.spec.w + 0.25f);
                for (float u : {u0 - 0.45f, u0 + 0.45f})
                    for (float v : {-0.45f, 0.45f}) pillar.push_back({b.x + c * u + sn * v, b.z - sn * u + c * v});
                gap = std::min(gap, roadGap(convexHull(pillar)));
            }
        }
        inPlay(b.x, b.z, buildingKindName(t.spec.kind));
        if (gap < need) {
            std::snprintf(buf, sizeof(buf), "%s at (%.1f, %.1f) is %.2f m from a road edge (min %.1f m)",
                          buildingKindName(t.spec.kind), double(b.x), double(b.z), double(gap), double(need));
            out.push_back(buf);
        }
        if (structure)
            for (const P2& p : hull)
                if (L.waterAt(p.x, p.z)) {
                    std::snprintf(buf, sizeof(buf), "%s at (%.1f, %.1f) stands in water", buildingKindName(t.spec.kind), double(b.x), double(b.z));
                    out.push_back(buf);
                    break;
                }
    }
    const LevelRoad* circuit = nullptr;
    for (const LevelRoad& r : L.m_roads)
        if (r.name == m_circuitRoad) circuit = &r;
    auto onCircuit = [&](float x, float z) {
        float best = 1e30f;
        if (circuit)
            for (const RoadSample& s : circuit->samples) best = std::min(best, std::hypot(s.x - x, s.z - z));
        return best;
    };
    if (circuit) {
        for (std::size_t g = 0; g < m_gates.size(); ++g) {
            const LevelGate& gate = m_gates[g];
            const float d = onCircuit(gate.x, gate.z);
            if (d > 2.0f || gate.halfWidth < 0.5f * circuit->width) {
                std::snprintf(buf, sizeof(buf), "checkpoint %zu is %.1f m off the circuit or narrower than the road", g, double(d));
                out.push_back(buf);
            }
        }
        if (onCircuit(L.m_spawnX, L.m_spawnZ) > 3.0f) out.push_back("spawn is not on the circuit");
        for (const RoadSample& s : circuit->samples) {
            const std::optional<float> w = L.waterAt(s.x, s.z);
            if (w && *w - L.heightAt(s.x, s.z) > 0.45f) {
                std::snprintf(buf, sizeof(buf), "circuit at (%.1f, %.1f) is %.2f m under water (max 0.45 m)", double(s.x),
                              double(s.z), double(*w - L.heightAt(s.x, s.z)));
                out.push_back(buf);
                break;
            }
        }
    }
    if (L.waterAt(L.m_spawnX, L.m_spawnZ)) out.push_back("spawn is in water");
}

float LevelBridge::length() const { return std::max(1e-3f, std::hypot(x1 - x0, z1 - z0)); }

float LevelBridge::deckY(float s) const {
    const float len = length();
    if (s < 0.0f) return lerp(y0, ya, std::min(1.0f, -s / kRamp));
    if (s > len) return lerp(y1, ye, std::min(1.0f, (s - len) / kRamp));
    return lerp(y0, y1, s / len);
}

// ------------------------------------------------------------------------------------------------------------------
SurfaceId levelModelSurface(const LevelModel& m) { return m.kind == "log" ? SurfaceId::Concrete : SurfaceId::Rock; }

void addLevelRock(TrackGeometry& g, const LevelRock& r, const std::vector<LevelModel>& models) {
    if (r.model < 0 || std::size_t(r.model) >= models.size()) {
        g.addRock(Vec3(r.x, r.y, r.z), Vec3(r.rx, r.ry, r.rz), r.yaw, r.seed, SurfaceId::Rock);
        return;
    }
    // Model space -> world: scale per axis, pitch about the model x axis (y' = c y - s z, z' = s y + c z), yaw about
    // +Y (x' = c x + s z, z' = -s x + c z), then the position.
    const LevelModel& m = models[std::size_t(r.model)];
    const float c = std::cos(r.yaw), s = std::sin(r.yaw), cp = std::cos(r.pitch), sp = std::sin(r.pitch);
    const SurfaceId surface = levelModelSurface(m);
    auto P = [&](const Float3& p) {
        const float x = p.x * r.sx, y0 = p.y * r.sy, z0 = p.z * r.sz;
        const float y = cp * y0 - sp * z0, z = sp * y0 + cp * z0;
        return Vec3(r.x + c * x + s * z, r.y + y, r.z - s * x + c * z);
    };
    for (std::size_t i = 0; i + 2 < m.collision.size(); i += 3)
        g.addTriangle(P(m.collision[i]), P(m.collision[i + 1]), P(m.collision[i + 2]), surface);
}

void addLevelTrunk(TrackGeometry& g, const LevelTree& t, const std::vector<LevelModel>& models) {
    const float c = std::cos(t.yaw), s = std::sin(t.yaw);
    if (t.model >= 0 && std::size_t(t.model) < models.size()) {
        // Oblique hexagonal prism from the base along the model's trunk axis (convex), rings horizontal.
        const LevelModel& m = models[std::size_t(t.model)];
        const float xz = t.scale * t.width;
        const float r = m.trunkRadius * xz;
        const float tx = m.trunkTop.x * xz, tz = m.trunkTop.z * xz;
        const Vec3 b0(t.x, t.y - 0.1f, t.z);
        const Vec3 b1(t.x + c * tx + s * tz, t.y + std::max(m.trunkTop.y * t.scale, 1.5f), t.z - s * tx + c * tz);
        auto P = [&](const Vec3& center, float a) { return center + Vec3(std::cos(a) * r, 0.0f, -std::sin(a) * r); };
        for (int k = 0; k < 6; ++k) {
            const float a0 = 2.0f * kPi * float(k) / 6.0f, a1 = 2.0f * kPi * float(k + 1) / 6.0f;
            g.addQuad(P(b0, a0), P(b0, a1), P(b1, a1), P(b1, a0), SurfaceId::Concrete);
        }
        return;
    }
    const float r = 0.022f, y0 = -0.02f, y1 = 0.32f;
    auto P = [&](float a, float y) {
        // Local ring point of the render cylinder (u = -Z, v = -X), then the per-tree yaw, width and height.
        const float lx = -std::sin(a) * r, lz = -std::cos(a) * r;
        return Vec3(t.x + (c * lx - s * lz) * t.width * t.height, t.y + y * t.height,
                    t.z + (s * lx + c * lz) * t.width * t.height);
    };
    for (int k = 0; k < 6; ++k) {
        const float a0 = 2.0f * kPi * float(k) / 6.0f, a1 = 2.0f * kPi * float(k + 1) / 6.0f;
        g.addQuad(P(a0, y0), P(a1, y0), P(a1, y1), P(a0, y1), SurfaceId::Concrete);
    }
}

std::shared_ptr<const Level> Level::fromJsonText(const std::string& text, std::string* error, int threads,
                                                 const std::string& baseDir) {
    json::Value doc;
    std::string err;
    if (!json::parse(text, doc, &err)) {
        if (error) *error = "level: " + err;
        return nullptr;
    }
    std::shared_ptr<Level> level(new Level());
    LevelBaker baker(*level, threads, baseDir);
    if (!baker.parse(doc, err)) {
        if (error) *error = err;
        return nullptr;
    }
    baker.bake();
    if (!baker.buildAttractRoute(err)) {
        if (error) *error = err;
        return nullptr;
    }
    return level;
}

// Each leg runs along one road's baked center line between the samples nearest to its end points (in either
// direction); consecutive legs share their junction point.
bool LevelBaker::buildAttractRoute(std::string& err) {
    if (m_baked) return true; // the circuit loop, set by bakeBaked
    DriveRoute route;
    route.name = m_routeName;
    route.cruiseSpeed = m_routeSpeed;
    for (const RouteLeg& leg : m_routeLegs) {
        const LevelRoad* road = nullptr;
        for (const LevelRoad& r : L.m_roads)
            if (r.name == leg.road) road = &r;
        if (!road || road->samples.size() < 2) {
            err = "level: attractRoute road '" + leg.road + "' not found";
            return false;
        }
        auto nearest = [&](float x, float z) {
            std::size_t best = 0;
            float bestD = 1e30f;
            for (std::size_t k = 0; k < road->samples.size(); ++k) {
                const float d = std::hypot(road->samples[k].x - x, road->samples[k].z - z);
                if (d < bestD) {
                    bestD = d;
                    best = k;
                }
            }
            return std::pair<std::size_t, float>(best, bestD);
        };
        const auto [a, da] = nearest(leg.x0, leg.z0);
        const auto [b, db] = nearest(leg.x1, leg.z1);
        if (da > 15.0f || db > 15.0f || a == b) {
            err = "level: attractRoute leg end points are not on road '" + leg.road + "'";
            return false;
        }
        const int step = b > a ? 1 : -1;
        for (int k = int(a); k != int(b); k += step) {
            if (!route.points.empty() && k == int(a)) continue; // the junction is the previous leg's last point
            route.points.push_back({road->samples[std::size_t(k)].x, road->samples[std::size_t(k)].z});
        }
        route.points.push_back({road->samples[b].x, road->samples[b].z});
    }
    if (!route.points.empty()) {
        const RoutePoint& f = route.points.front();
        const RoutePoint& l = route.points.back();
        if (std::hypot(f.x - l.x, f.z - l.z) < 3.0f) route.points.pop_back(); // closed loop: no duplicate start
        if (!route.valid()) {
            err = "level: attractRoute is too short";
            return false;
        }
        // Junctions meet at a corner: a few passes of a 5-point moving average round them to a drivable line
        // (about 2 m inside a 90 degree corner, within the road width; straight stretches stay on the center line).
        const std::size_t n = route.points.size();
        for (int pass = 0; pass < 4; ++pass) {
            std::vector<RoutePoint> s(n);
            for (std::size_t i = 0; i < n; ++i) {
                for (std::size_t k = 0; k < 5; ++k) {
                    const RoutePoint& q = route.points[(i + n + k - 2) % n];
                    s[i].x += 0.2f * q.x;
                    s[i].z += 0.2f * q.z;
                }
            }
            route.points = std::move(s);
        }
    }
    L.m_attractRoute = std::move(route);
    return true;
}

std::shared_ptr<const Level> Level::load(const std::string& path, std::string* error, int threads) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        if (error) *error = "level: cannot open " + path;
        return nullptr;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    const std::size_t slash = path.find_last_of("/\\");
    return fromJsonText(ss.str(), error, threads, slash == std::string::npos ? std::string(".") : path.substr(0, slash));
}

float Level::heightAt(float x, float z) const {
    // The cell index is clamped as an integer: float(m_n - 1) - 1e-4 rounds back to m_n - 1 for large grids.
    const float fx = std::clamp((x - m_origin) / m_cell, 0.0f, float(m_n - 1));
    const float fz = std::clamp((z - m_origin) / m_cell, 0.0f, float(m_n - 1));
    const int i = std::min(int(fx), m_n - 2), j = std::min(int(fz), m_n - 2);
    const float u = fx - float(i), v = fz - float(j);
    const float h00 = heightSample(i, j), h10 = heightSample(i + 1, j), h01 = heightSample(i, j + 1),
                h11 = heightSample(i + 1, j + 1);
    if (v >= u) return h00 + u * (h11 - h01) + v * (h01 - h00); // triangle 00, 01, 11
    return h00 + u * (h10 - h00) + v * (h11 - h10);             // triangle 00, 11, 10
}

std::optional<float> Level::waterAt(float x, float z) const {
    const int i = std::clamp(int(std::lround((x - m_origin) / m_cell)), 0, m_n - 1);
    const int j = std::clamp(int(std::lround((z - m_origin) / m_cell)), 0, m_n - 1);
    const float w = waterSample(i, j);
    if (w <= kNoWater || w <= heightAt(x, z)) return std::nullopt;
    return w;
}

WaterKind Level::waterKindAt(float x, float z) const {
    const int i = std::clamp(int(std::lround((x - m_origin) / m_cell)), 0, m_n - 1);
    const int j = std::clamp(int(std::lround((z - m_origin) / m_cell)), 0, m_n - 1);
    return m_waterKind[std::size_t(j) * m_n + i];
}

SurfaceId Level::surfaceAt(float x, float z) const {
    const int i = std::clamp(int(std::floor((x - m_origin) / m_cell)), 0, m_n - 2);
    const int j = std::clamp(int(std::floor((z - m_origin) / m_cell)), 0, m_n - 2);
    return cellSurface(i, j);
}

TrackFeature Level::spawn() const {
    TrackFeature f;
    f.name = "Spawn: " + m_name;
    f.position = Vec3(m_spawnX, heightAt(m_spawnX, m_spawnZ), m_spawnZ);
    f.yaw = std::atan2(m_spawnTowardX - m_spawnX, m_spawnTowardZ - m_spawnZ);
    return f;
}

std::vector<TrackFeature> Level::features() const {
    std::vector<TrackFeature> out;
    out.push_back(spawn());
    for (const LevelPoi& p : m_pois) {
        TrackFeature f;
        f.name = std::to_string(p.id) + ". " + p.name;
        f.position = Vec3(p.atX, heightAt(p.atX, p.atZ), p.atZ);
        // Face the marker unless the teleport point is on it.
        const float dx = p.x - p.atX, dz = p.z - p.atZ;
        f.yaw = std::hypot(dx, dz) > 1.0f ? std::atan2(dx, dz) : 0.0f;
        // A teleport point next to a road moves onto it (the shoulders can be steep cuts), heading along the road
        // in the direction closest to the marker.
        const TrackFeature road = nearestRoadPoint(p.atX, p.atZ);
        if (std::hypot(road.position.GetX() - p.atX, road.position.GetZ() - p.atZ) < 12.0f) {
            f.position = road.position;
            float yaw = road.yaw;
            if (std::hypot(dx, dz) > 1.0f && std::sin(yaw) * dx + std::cos(yaw) * dz < 0.0f) yaw += kPi;
            f.yaw = yaw;
        }
        out.push_back(f);
    }
    return out;
}

TrackFeature Level::nearestRoadPoint(float x, float z) const {
    TrackFeature f;
    float best = 1e30f;
    for (const LevelRoad& r : m_roads)
        for (const RoadSample& s : r.samples) {
            if (s.bridge || s.ford) continue;
            if (std::abs(s.x) > m_playHalf - 5.0f || std::abs(s.z) > m_playHalf - 5.0f) continue;
            const float d = (s.x - x) * (s.x - x) + (s.z - z) * (s.z - z);
            if (d < best) {
                best = d;
                f.name = r.name;
                f.position = Vec3(s.x, heightAt(s.x, s.z), s.z);
                f.yaw = std::atan2(s.dx, s.dz);
            }
        }
    if (best > 1e29f) return spawn();
    return f;
}

bool Level::insidePlayArea(float x, float z, float margin) const {
    return std::abs(x) <= m_playHalf - margin && std::abs(z) <= m_playHalf - margin;
}

} // namespace tf
