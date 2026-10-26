#include "track/Level.h"

#include "core/Json.h"
#include "core/ModelAsset.h"


#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
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

SurfaceId surfaceFromName(const std::string& s) {
    if (s == "rock") return SurfaceId::Rock;
    if (s == "dirt") return SurfaceId::PackedDirt;
    if (s == "sand") return SurfaceId::Sand;
    if (s == "grass") return SurfaceId::Grass;
    if (s == "mud") return SurfaceId::Mud;
    if (s == "snow") return SurfaceId::Snow;
    return SurfaceId::Count;
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
    struct Peak { float x, z, h, r; };
    struct Beach { float x, z, r; };
    struct Lake { std::string name; float level, depth; Polygon poly; float shore = 0.0f; };
    struct RiverDef { std::string name; float width, depth, bank; std::vector<Float3> points; };
    struct BridgeDef { float x0, z0, x1, z1; };
    struct RoadDef { std::string name; bool trail; float width, grade; std::vector<Float3> points; std::vector<BridgeDef> bridges; };
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
    void gableRoof(float x, float y, float z, float hx, float hz, float rise, float yaw, PropStyle style);
    void prism(float x, float y0, float z, float radius, float y1, int sides, PropStyle style, float topRadius = -1.0f);
    void slab(Vec3 a, Vec3 b, float halfWidth, float thickness, PropStyle style, SurfaceId s);
    void house(float x, float z, float w, float d, float h, float yaw, PropStyle walls, PropStyle roof);
    void tent(float x, float z, float yaw);
    void rock(Vec3 center, Vec3 radii, float yaw, std::uint32_t seed, PropStyle style);
    void pier(float x0, float z0, float x1, float z1, float deckY, float halfWidth);
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
              outer(!baker.m_inSite) {
            b.m_inSite = true;
        }
        ~Site() {
            if (!outer) return;
            b.m_inSite = false;
            b.settle(tri, piece);
        }
        Site(const Site&) = delete;
        Site& operator=(const Site&) = delete;
        LevelBaker& b;
        std::size_t tri, piece;
        bool outer;
    };
    void settle(std::size_t firstTriangle, std::size_t firstPiece);
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
    std::vector<float> m_roadClear;            // distance to the nearest road edge within 48 m of a road
    std::vector<SurfaceId> m_flatSurface;      // per sample: forced surface of a flat (Count = none)
    std::vector<std::uint8_t> m_nearWater;     // 0..255: water within ~3 m
    std::vector<float> m_forest;               // per cell forest density
    std::string m_baseDir;
    std::vector<int> m_conifers, m_saplings, m_broadleaf, m_bushes, m_ferns, m_debris, m_rockModels, m_logs, m_cliffModels;
    std::vector<float> m_waterDist;            // 8 m grid
    int m_wn = 0;
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
                  float(r["maxGrade"].numberOr(0.12)), {}, {}};
        for (const json::Value& p : r["points"].array) {
            float x, z;
            if (readPoint(p, x, z)) d.points.push_back(Float3(x, kNaN, z));
        }
        for (const json::Value& b : r["bridges"].array) {
            BridgeDef bd{};
            if (readPoint(b["from"], bd.x0, bd.z0) && readPoint(b["to"], bd.x1, bd.z1)) d.bridges.push_back(bd);
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
                surfaceFromName(f["surface"].stringOr(""))};
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
    const std::string dir = m_baseDir + "/" + veg["dir"].stringOr("../../models") + "/";
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
            L.m_models.push_back(std::move(m));
        }
        if (!warning.empty()) break;
    }
    if (!warning.empty() || m_conifers.empty()) {
        // Missing library (for example Git LFS objects not pulled): the procedural trees and rocks stand in.
        L.m_modelError = warning.empty() ? "vegetation: no conifers" : warning;
        L.m_models.clear();
        for (const auto& [key, list] : roles) list->clear();
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
                h = std::clamp(h, 0.4f + 0.012f * d + m_seaLevel, m_seaLevel + 0.4f + d * rise);
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
        const float lip = 12.0f, pad = std::max(lip, lake.shore);
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
                    h = std::min(h, bed);
                    m_lakeId[idx(i, j)] = std::uint8_t(k + 1);
                } else if (-d < pad) {
                    // Shore band: higher ground eases down to the waterline (about 7 deg at the water) instead
                    // of meeting the lake in a cut; then the low lip that keeps the water in.
                    if (-d < lake.shore) {
                        const float lowered = std::min(h, lake.level + 0.3f - d * 0.12f);
                        h = lowered + (h - lowered) * smoothstep(0.0f, lake.shore, -d);
                    }
                    if (-d < lip) h = std::max(h, lake.level + 0.3f + d * 0.06f);
                }
            }
        });
    }
}

void LevelBaker::bakeRivers() {
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

        // Carve: channel bed, then banks rising at bankSlope until they meet the terrain.
        const float reach = std::clamp(32.0f / def.bank, 10.0f, 60.0f);
        const float waterReach = hw + 0.25f / def.bank + 1.0f;
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
                        float rise = (d - hw) * def.bank;
                        if (def.bank >= 1.5f && rise > 0.0f) {
                            // Steep banks are terraced: a ledge about every 16 m of height, steeper rock between.
                            const float band = 16.0f;
                            const float phase = fbm(X(i) / 45.0f, X(j) / 45.0f, 2, m_seed + 71u) * 1.7f;
                            const auto steps = [](float q) {
                                const float f = q - std::floor(q);
                                return std::floor(q) + 0.12f * f + 0.88f * smoothstep(0.55f, 0.97f, f);
                            };
                            rise = band * (steps(rise / band + phase) - steps(phase));
                        }
                        const float carve = d < hw ? ys - 0.25f - (def.depth - 0.25f) * (1.0f - (d / hw) * (d / hw))
                                                   : ys - 0.25f + rise;
                        const std::size_t k = idx(i, j);
                        m_h[k] = std::min(m_h[k], carve);
                        if (d < m_riverD[k]) {
                            m_riverD[k] = d;
                            m_riverY[k] = ys;
                            m_riverReach[k] = waterReach;
                        }
                    }
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
                    m_roadness[g] = std::max(m_roadness[g], 1.0f - smoothstep(hw - 0.6f, hw + 0.5f, d));
                    m_roadEdge[g] = std::min(m_roadEdge[g], d - hw);
                }
        }
        for (int j = j0; j <= j1; ++j)
            for (int i = 0; i < m_n; ++i) {
                const std::size_t k = std::size_t(j - j0) * rowN + std::size_t(i);
                if (sumW[k] <= 0.0f) continue;
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
                if (h > snowAt) s = SurfaceId::Snow;
                else if (wd > 0.02f) s = SurfaceId::Rock; // gravel ford bar
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
            const float dirt = s == SurfaceId::PackedDirt || s == SurfaceId::Grass ? roadPaint : 0.0f;
            float grass = 0.0f;
            if (s == SurfaceId::Grass) grass = (0.35f + 0.65f * smoothstep(0.3f, 0.7f, meadow)) * (1.0f - 0.85f * dirt);
            else if (s == SurfaceId::PackedDirt && road < 0.2f) grass = 0.12f * smoothstep(0.4f, 0.7f, patch);
            const float wet = wd > 0.0f ? 1.0f : 0.55f * near;
            mask[std::size_t(j) * std::size_t(cn) + std::size_t(i)] = packRgba(grass, mud, wet, wheat ? 1.0f : 0.0f);
            splat[std::size_t(j) * std::size_t(cn) + std::size_t(i)] = packRgba(dirt, rock, sand, snow);
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
            const float density = treeDensity(x, z, forest) * snowFade;
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
// >= 50 deg with at least 8 m of drop across 16 m. The face lies in the wall plane (pitch = slope - 90 deg about the
// model x axis, yaw toward the downhill direction), centred on the ground so its relief stands out of the wall.
void LevelBaker::bakeCliffs() {
    if (m_cliffModels.empty()) return;
    const float spacing = 9.0f;
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
            if (slope < 50.0f * kPi / 180.0f) continue;
            const float dx = -gx / grad, dz = -gz / grad; // downhill
            const float drop = L.heightAt(x - dx * 8.0f, z - dz * 8.0f) - L.heightAt(x + dx * 8.0f, z + dz * 8.0f);
            if (drop < 8.0f) continue;
            const float h = L.heightAt(x, z);
            if (const std::optional<float> w = L.waterAt(x, z); w && *w > h - 0.5f) continue;
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
            // and every edge point must stay within the slab depth of the ground, so faces neither stick up past a
            // rim nor bridge a gully.
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
                    if (d > 1.5f * rock.rz + 0.5f || d < -0.5f * rock.rz - 1.0f) flush = false;
                }
            if (!flush) continue;
            rock.seed = hash3(gi, gj, m_seed + 517u);
            rock.style = PropStyle::DarkStone;
            L.m_rocks.push_back(rock);
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
            bool clear = true;
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

void LevelBaker::settle(std::size_t firstTri, std::size_t firstPiece) {
    TrackGeometry& g = L.m_props.geometry;
    const std::size_t lastTri = g.triangleCount();
    if (lastTri <= firstTri) return;
    std::vector<P2> pts;
    pts.reserve(3 * (lastTri - firstTri));
    for (std::size_t v = 3 * firstTri; v < 3 * lastTri; ++v) pts.push_back({g.positions[v].x, g.positions[v].z});
    const std::vector<P2> hull = convexHull(pts);
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
            const float dy = L.heightAt(cx + dx, cz + dz) - L.heightAt(cx, cz);
            for (std::size_t v = 3 * firstTri; v < 3 * lastTri; ++v) {
                g.positions[v].x += dx;
                g.positions[v].y += dy;
                g.positions[v].z += dz;
            }
            m_sites.push_back(std::move(h));
            ++L.m_stats.propsMoved;
            return;
        }
    g.positions.resize(3 * firstTri);
    g.surfaces.resize(firstTri);
    L.m_props.styles.resize(firstTri);
    L.m_props.pieceStarts.resize(firstPiece);
    L.m_props.pieceRoles.resize(firstPiece);
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

void LevelBaker::gableRoof(float x, float y, float z, float hx, float hz, float rise, float yaw, PropStyle style) {
    addProp([&](TrackGeometry& g) {
        const JPH::Quat q = JPH::Quat::sRotation(Vec3::sAxisY(), yaw);
        auto P = [&](float a, float b, float c) { return Vec3(x, y, z) + q * Vec3(a, b, c); };
        const float o = 0.35f; // eaves overhang
        // Two roof planes (ridge along local z) and the gable triangles.
        g.addQuad(P(-hx - o, 0, -hz - o), P(-hx - o, 0, hz + o), P(0, rise, hz + o), P(0, rise, -hz - o), SurfaceId::Concrete);
        g.addQuad(P(hx + o, 0, hz + o), P(hx + o, 0, -hz - o), P(0, rise, -hz - o), P(0, rise, hz + o), SurfaceId::Concrete);
        g.addTriangle(P(-hx, 0, hz), P(hx, 0, hz), P(0, rise, hz), SurfaceId::Concrete);
        g.addTriangle(P(hx, 0, -hz), P(-hx, 0, -hz), P(0, rise, -hz), SurfaceId::Concrete);
        // Underside of the eaves.
        g.addQuad(P(-hx - o, 0, hz + o), P(-hx - o, 0, -hz - o), P(hx + o, 0, -hz - o), P(hx + o, 0, hz + o), SurfaceId::Concrete);
    }, style);
}

void LevelBaker::prism(float x, float y0, float z, float radius, float y1, int sides, PropStyle style, float topRadius) {
    if (topRadius < 0.0f) topRadius = radius;
    addProp([&](TrackGeometry& g) {
        for (int k = 0; k < sides; ++k) {
            const float a0 = 2.0f * kPi * float(k) / float(sides), a1 = 2.0f * kPi * float(k + 1) / float(sides);
            const Vec3 b0(x + radius * std::cos(a0), y0, z + radius * std::sin(a0));
            const Vec3 b1(x + radius * std::cos(a1), y0, z + radius * std::sin(a1));
            const Vec3 t0(x + topRadius * std::cos(a0), y1, z + topRadius * std::sin(a0));
            const Vec3 t1(x + topRadius * std::cos(a1), y1, z + topRadius * std::sin(a1));
            g.addQuad(b0, t0, t1, b1, SurfaceId::Concrete);
            g.addTriangle(Vec3(x, y1, z), t1, t0, SurfaceId::Concrete);
        }
    }, style);
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

void LevelBaker::house(float x, float z, float w, float d, float h, float yaw, PropStyle walls, PropStyle roof) {
    Site site(*this);
    const PropRole role = std::exchange(m_role, PropRole::Building);
    const float base = footprintMin(x, z, 0.5f * w, 0.5f * d, yaw) - 0.4f;
    const float top = std::max(base + 0.4f + h, L.heightAt(x, z) + h);
    box(x, 0.5f * (base + top), z, 0.5f * w, 0.5f * (top - base), 0.5f * d, yaw, walls);
    gableRoof(x, top, z, 0.5f * w, 0.5f * d, 0.45f * w, yaw, roof);
    m_role = role;
}

void LevelBaker::rock(Vec3 center, Vec3 radii, float yaw, std::uint32_t seed, PropStyle style) {
    const PropRole role = std::exchange(m_role, PropRole::Rock);
    addProp([&](TrackGeometry& g) { g.addRock(center, radii, yaw, seed, SurfaceId::Rock); }, style);
    m_role = role;
}

void LevelBaker::tent(float x, float z, float yaw) {
    const float y = L.heightAt(x, z) - 0.1f;
    gableRoof(x, y, z, 1.3f, 1.6f, 1.7f, yaw, PropStyle::Canvas);
}

void LevelBaker::pier(float x0, float z0, float x1, float z1, float deckY, float hw) {
    Site site(*this);
    slab(Vec3(x0, deckY, z0), Vec3(x1, deckY, z1), hw, 0.25f, PropStyle::Wood, SurfaceId::Concrete);
    const float len = std::hypot(x1 - x0, z1 - z0);
    const int posts = std::max(2, int(len / 5.0f) + 1);
    const float dx = (x1 - x0) / len, dz = (z1 - z0) / len;
    for (int k = 0; k < posts; ++k) {
        const float t = float(k) / float(posts - 1);
        for (float side : {-1.0f, 1.0f}) {
            const float px = lerp(x0, x1, t) + dz * side * (hw - 0.15f), pz = lerp(z0, z1, t) - dx * side * (hw - 0.15f);
            const float bottom = L.heightAt(px, pz) - 0.5f;
            if (deckY - 0.25f - bottom < 0.6f) continue; // the deck rests on the ground here
            box(px, 0.5f * (bottom + deckY - 0.25f), pz, 0.14f, 0.5f * (deckY - 0.25f - bottom), 0.14f, 0.0f, PropStyle::DarkWood);
        }
    }
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
    for (const LevelPoi& p : L.m_pois) {
        const float x = p.x, z = p.z;
        const float gy = L.heightAt(x, z);
        if (p.kind == "peak") {
            {
                Site site(*this);
                prism(x + 4.0f, gy - 0.5f, z - 3.0f, 1.4f, gy + 1.6f, 6, PropStyle::DarkStone, 0.5f); // cairn
                box(x + 4.0f, gy + 3.0f, z - 3.0f, 0.06f, 1.6f, 0.06f, 0.0f, PropStyle::Metal);
                box(x + 4.5f, gy + 4.2f, z - 3.0f, 0.5f, 0.3f, 0.02f, 0.0f, PropStyle::Marker);
            }
        } else if (p.kind == "pass") {
            prism(x - 9.0f, L.heightAt(x - 9.0f, z + 6.0f) - 0.5f, z + 6.0f, 1.2f, L.heightAt(x - 9.0f, z + 6.0f) + 1.4f, 6,
                  PropStyle::DarkStone, 0.4f);
            house(x + 12.0f, z + 9.0f, 5.0f, 4.0f, 2.6f, 0.3f, PropStyle::DarkWood, PropStyle::RoofDark);
            {
                Site site(*this);
                groundedBox(x - 6.0f, z - 7.0f, 0.08f, 1.3f, 0.08f, 0.0f, PropStyle::DarkWood);
                box(x - 6.0f, L.heightAt(x - 6.0f, z - 7.0f) + 2.2f, z - 7.0f, 0.9f, 0.25f, 0.04f, 0.4f, PropStyle::Wood);
            }
        } else if (p.kind == "quarry") {
            {
                Site site(*this);
                groundedBox(x + 20.0f, z - 18.0f, 4.0f, 4.0f, 3.0f, 0.4f, PropStyle::Rust);              // crusher
                groundedBox(x + 20.0f, z - 18.0f, 2.0f, 6.5f, 2.0f, 0.4f, PropStyle::Rust);
                slab(Vec3(x + 24.0f, L.heightAt(x + 24.0f, z - 12.0f) + 9.0f, z - 12.0f),
                     Vec3(x + 34.0f, L.heightAt(x + 34.0f, z + 4.0f) + 1.5f, z + 4.0f), 0.8f, 0.4f, PropStyle::Metal,
                     SurfaceId::Concrete); // conveyor
            }
            house(x - 22.0f, z - 20.0f, 9.0f, 6.0f, 3.2f, 0.2f, PropStyle::Metal, PropStyle::Rust);
            for (int k = 0; k < 4; ++k) {
                const float hx = x - 10.0f + 9.0f * float(k), hz = z + 22.0f;
                rock(Vec3(hx, L.heightAt(hx, hz) - 0.6f, hz), Vec3(3.4f, 1.8f, 3.0f), 0.7f * float(k), 900u + k, PropStyle::Stone);
            }
        } else if (p.kind == "forest") {
            house(x + 10.0f, z + 8.0f, 6.0f, 4.5f, 2.6f, 0.6f, PropStyle::DarkWood, PropStyle::RoofDark);
            {
                Site site(*this);
                for (int k = 0; k < 3; ++k)
                    groundedBox(x - 9.0f, z + 6.0f + 1.0f * float(k), 2.5f, 0.3f, 0.3f, 0.0f, PropStyle::Wood, SurfaceId::Concrete, 0.3f);
            }
        } else if (p.kind == "lake") {
            const float sx = p.atX, sz = p.atZ;
            pier(sx + 8.0f, sz - 4.0f, sx + 14.0f, sz - 30.0f, 70.8f, 1.2f);
            house(sx - 12.0f, sz + 4.0f, 6.0f, 5.0f, 2.8f, 0.2f, PropStyle::Wood, PropStyle::RoofRed);
        } else if (p.kind == "waterfall") {
            const float vx = p.atX + 2.0f, vz = p.atZ - 9.0f;
            {
                Site site(*this);
                for (int k = -3; k <= 3; ++k)
                    groundedBox(vx + 1.5f * float(k), vz + 0.3f * float(k), 0.06f, 0.6f, 0.06f, 0.0f, PropStyle::DarkWood, SurfaceId::Concrete, 0.4f);
                box(vx, L.heightAt(vx, vz) + 1.0f, vz, 4.6f, 0.05f, 0.05f, -0.2f, PropStyle::Wood);
            }
            groundedBox(p.atX - 6.0f, p.atZ + 2.0f, 0.9f, 0.25f, 0.3f, 0.3f, PropStyle::Wood, SurfaceId::Concrete, 0.2f);
        } else if (p.kind == "radio_tower") {
            {
                Site site(*this);
                const float top = gy + 34.0f;
                for (float sx : {-1.0f, 1.0f})
                    for (float sz : {-1.0f, 1.0f}) {
                        // Tapered leg as a thin slanted quad prism (approximated by two boxes).
                        box(x + sx * 1.6f, gy + 8.0f, z + sz * 1.6f, 0.12f, 8.5f, 0.12f, 0.0f, PropStyle::Rust);
                        box(x + sx * 0.9f, gy + 25.0f, z + sz * 0.9f, 0.1f, 9.0f, 0.1f, 0.0f, PropStyle::Rust);
                    }
                for (int k = 1; k <= 6; ++k) {
                    const float yy = gy + 5.0f * float(k);
                    const float r = k < 4 ? 1.6f : 0.9f;
                    box(x, yy, z + r, r, 0.08f, 0.08f, 0.0f, PropStyle::Metal);
                    box(x, yy, z - r, r, 0.08f, 0.08f, 0.0f, PropStyle::Metal);
                    box(x + r, yy, z, 0.08f, 0.08f, r, 0.0f, PropStyle::Metal);
                    box(x - r, yy, z, 0.08f, 0.08f, r, 0.0f, PropStyle::Metal);
                }
                box(x, top + 1.0f, z, 0.08f, 2.0f, 0.08f, 0.0f, PropStyle::Metal);
                box(x, top + 3.1f, z, 0.25f, 0.25f, 0.25f, 0.0f, PropStyle::Beacon);
            }
            house(x - 7.0f, z + 6.0f, 4.0f, 3.0f, 2.4f, 0.0f, PropStyle::Plaster, PropStyle::RoofDark);
        } else if (p.kind == "village") {
            // Houses around the square (house() keeps them off the roads).
            const float ring[][4] = {{-40, -30, 9, 7}, {-20, -55, 8, 6}, {25, -50, 10, 7}, {50, -20, 8, 7},
                                     {55, 25, 9, 6}, {30, 55, 8, 6}, {-10, 60, 9, 7}, {-45, 40, 8, 6},
                                     {-70, 5, 7, 6}, {75, -45, 7, 6}, {-60, -60, 8, 6}, {10, -80, 9, 6},
                                     {80, 50, 8, 7}, {-30, 85, 7, 6}};
            int k = 0;
            for (const auto& r : ring) {
                const float hx = x + r[0], hz = z + r[1];
                const float yaw = std::atan2(-r[0], -r[1]) + 0.15f * float(k % 3);
                house(hx, hz, r[2], r[3], 3.0f + 0.4f * float(k % 3), yaw, PropStyle::Plaster,
                      k % 3 == 0 ? PropStyle::RoofDark : PropStyle::RoofRed);
                ++k;
            }
            // Fuel station by the main road: canopy on four posts, two pumps, kiosk.
            const float fx = x + 22.0f, fz = z - 8.0f;
            const float fy = L.heightAt(fx, fz);
            {
                Site site(*this);
                for (float sx : {-1.0f, 1.0f})
                    for (float sz : {-1.0f, 1.0f}) groundedBox(fx + sx * 4.0f, fz + sz * 3.0f, 0.15f, 2.6f, 0.15f, 0.0f, PropStyle::Metal);
                box(fx, fy + 5.1f, fz, 5.0f, 0.25f, 4.0f, 0.0f, PropStyle::Marker);
                groundedBox(fx - 1.5f, fz, 0.35f, 0.8f, 0.5f, 0.0f, PropStyle::Marker, SurfaceId::Concrete, 0.2f);
                groundedBox(fx + 1.5f, fz, 0.35f, 0.8f, 0.5f, 0.0f, PropStyle::Marker, SurfaceId::Concrete, 0.2f);
                house(fx + 9.0f, fz + 1.0f, 4.0f, 5.0f, 2.8f, 0.0f, PropStyle::Plaster, PropStyle::RoofDark);
            }
            // Garage: wide hall with a dark door.
            const float gx = x - 25.0f, gz = z + 12.0f;
            {
                Site site(*this);
                house(gx, gz, 11.0f, 8.0f, 4.0f, 0.1f, PropStyle::Metal, PropStyle::RoofDark);
                box(gx + 5.6f * std::cos(0.1f), L.heightAt(gx, gz) + 1.8f, gz - 5.6f * std::sin(0.1f), 0.05f, 1.6f, 2.4f, 0.1f, PropStyle::DarkWood);
            }
        } else if (p.kind == "ruins") {
            const float wy = gy;
            const float walls[][5] = {{-16, -14, 16, 1.2f, 6.0f}, {-16, 14, 9, 1.2f, 3.5f}, {12, 14, 4, 1.2f, 5.0f},
                                      {-17, 0, 1.2f, 13, 4.5f}, {17, -6, 1.2f, 8, 7.0f}};
            for (const auto& w : walls) {
                const float base = footprintMin(x + w[0], z + w[1], w[2], w[3], 0.0f) - 0.6f;
                box(x + w[0], 0.5f * (base + wy + w[4]), z + w[1], w[2], 0.5f * (wy + w[4] - base), w[3], 0.0f, PropStyle::Stone);
            }
            prism(x - 17.0f, wy - 1.0f, z - 14.0f, 3.2f, wy + 10.0f, 8, PropStyle::Stone);
            prism(x + 17.0f, wy - 1.0f, z + 14.0f, 3.0f, wy + 6.5f, 8, PropStyle::DarkStone, 2.8f);
            for (int k = 0; k < 6; ++k) {
                const float rx = x - 8.0f + 3.2f * float(k), rz = z + 10.0f - 2.0f * float(k % 2);
                rock(Vec3(rx, L.heightAt(rx, rz) - 0.2f, rz), Vec3(0.7f, 0.45f, 0.6f), float(k), 950u + k, PropStyle::Stone);
            }
        } else if (p.kind == "beach_camp") {
            tent(x + 6.0f, z - 4.0f, 0.4f);
            tent(x - 6.0f, z - 2.0f, -0.3f);
            house(x + 14.0f, z - 12.0f, 5.0f, 4.0f, 2.6f, 0.5f, PropStyle::Wood, PropStyle::RoofRed);
            for (int k = 0; k < 7; ++k) {
                const float a = 2.0f * kPi * float(k) / 7.0f;
                const float rx = x + 1.0f + std::cos(a) * 1.0f, rz = z + 6.0f + std::sin(a) * 1.0f;
                rock(Vec3(rx, L.heightAt(rx, rz) - 0.05f, rz), Vec3(0.22f, 0.16f, 0.2f), a, 970u + k, PropStyle::DarkStone);
            }
            pier(x - 18.0f, z + 26.0f, x - 26.0f, z + 80.0f, m_seaLevel + 1.1f, 1.5f);
        } else if (p.kind == "island_camp") {
            tent(x + 5.0f, z + 6.0f, 1.2f);
            tent(x - 4.0f, z + 8.0f, 0.2f);
            house(x + 12.0f, z + 14.0f, 4.0f, 3.5f, 2.4f, 0.9f, PropStyle::Wood, PropStyle::RoofDark);
            {
                Site site(*this);
                groundedBox(x - 8.0f, z - 4.0f, 0.06f, 3.5f, 0.06f, 0.0f, PropStyle::Metal);
                box(x - 7.4f, L.heightAt(x - 8.0f, z - 4.0f) + 5.6f, z - 4.0f, 0.6f, 0.35f, 0.02f, 0.0f, PropStyle::Marker);
            }
        } else if (p.kind == "lighthouse") {
            {
                Site site(*this);
                const float base = gy - 0.8f;
                for (int k = 0; k < 4; ++k)
                    prism(x, base + 4.4f * float(k), z, 2.6f - 0.2f * float(k), base + 4.4f * float(k + 1), 10,
                          k % 2 ? PropStyle::RoofRed : PropStyle::Plaster, 2.4f - 0.2f * float(k));
                prism(x, base + 17.6f, z, 2.2f, base + 18.0f, 10, PropStyle::DarkStone, 2.4f);
                prism(x, base + 18.0f, z, 1.4f, base + 20.0f, 10, PropStyle::Beacon);
                prism(x, base + 20.0f, z, 1.6f, base + 21.4f, 10, PropStyle::RoofRed, 0.15f);
            }
            house(x - 9.0f, z - 2.0f, 6.0f, 5.0f, 2.8f, 0.2f, PropStyle::Plaster, PropStyle::RoofRed);
        } else if (p.kind == "bridge") {
            {
                Site site(*this);
                groundedBox(p.atX + 4.0f, p.atZ + 5.0f, 0.08f, 1.2f, 0.08f, 0.0f, PropStyle::DarkWood);
                box(p.atX + 4.0f, L.heightAt(p.atX + 4.0f, p.atZ + 5.0f) + 2.0f, p.atZ + 5.0f, 0.8f, 0.25f, 0.04f, 1.4f, PropStyle::Wood);
            }
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
    m_roadEdge.assign(total, 1e30f);
    m_roadClear.assign(total, 1e30f);
    m_flatSurface.assign(total, SurfaceId::Count);
    m_nearWater.assign(total, 0);

    bakeBase();
    bakeCoast();
    bakeFlats();
    const std::vector<float> uncarved = m_h;
    bakeLakes();
    bakeRivers();
    bakeBankSmoothing(uncarved);
    bakeRoads();
    L.m_height = m_h;
    buildRoadPoints();
    bakeWater();
    bakeSurfaces();
    bakeWaterDistance();
    bakeTrees();
    bakeRocks();
    bakeProps();
    bakeLogs();
    bakeCliffs();
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
