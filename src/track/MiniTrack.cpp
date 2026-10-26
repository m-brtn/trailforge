#include "track/MiniTrack.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace tf {

using JPH::Vec3;

namespace {

constexpr float kPi = 3.14159265358979f;
// Walled area south of the proving ground (its south wall is at z = -50..-51).
constexpr float kX0 = -185.0f, kX1 = 185.0f, kZ0 = -232.0f, kZ1 = -51.0f;
constexpr float kGrid = 1.0f;       // ground grid step, m
constexpr float kHalfWidth = 5.0f;  // asphalt half width, m
constexpr float kEdge = 6.2f;       // ribbon half width: asphalt + 1.2 m curb (corners) or asphalt shoulder
constexpr float kLift = 0.03f;      // asphalt top above the ground, m
constexpr float kCurbHeight = 0.05f; // curb outer edge above the asphalt (a 2.4 degree ramp), m
constexpr float kGravelIn = kEdge + 1.0f, kGravelOut = kEdge + 17.0f;
constexpr float kTyreOffset = kEdge + 19.0f;
constexpr float kWallTop = 6.8f, kWallBottom = -1.0f;

float smoothstep(float e0, float e1, float x) {
    const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Gentle slope: the west end (sweeper, last corner, start of the main straight) is 1.8 m higher, max grade ~3 %.
float elevation(float x, float /*z*/) { return 1.8f * smoothstep(-50.0f, -140.0f, x); }

// Corners of the base polygon in driving order; each is rounded with a circular arc of radius r.
struct Corner {
    float x, z, r;
    bool gravel; // gravel trap and tyre wall on the outside
};
// 0: last corner onto the main straight; 1+2: hairpin (R 13); 3+4: the link after the chicane; 5+6: the 180 degree
// constant-radius corner (R 32); 7: fast sweeper (R 80). The chicane is on edge 2 -> 3, the slalom on edge 6 -> 7.
constexpr Corner kCorners[] = {{-160.0f, -75.0f, 25.0f, true},  {150.0f, -75.0f, 13.0f, true},
                               {150.0f, -101.0f, 13.0f, true},  {35.0f, -101.0f, 18.0f, false},
                               {35.0f, -145.0f, 18.0f, false},  {160.0f, -145.0f, 32.0f, true},
                               {160.0f, -209.0f, 32.0f, true},  {-160.0f, -209.0f, 80.0f, true}};
constexpr int kCornerCount = int(sizeof(kCorners) / sizeof(kCorners[0]));
constexpr int kChicaneEdge = 2, kSlalomEdge = 6;

enum class Piece : std::uint8_t { Straight, Corner, Chicane, Slalom };

struct Layout {
    MiniTrackInfo info;
    std::vector<Piece> piece;     // per center point
    std::vector<int> corner;      // corner index for Piece::Corner, else -1
    std::vector<float> curb;      // 0..1 curb weight per point
    std::vector<std::uint8_t> gravel; // gravel trap on the outside of this point
    std::vector<float> leftX, leftZ;  // unit left vector (vehicle +X) per point
};

// Lateral offset (toward the left) along a straight edge of length L at distance u from its start.
float edgeOffset(int edge, float u, float L, Piece& kind) {
    kind = Piece::Straight;
    if (edge == kChicaneEdge) {
        // 8 m jog away from the main straight over 44 m: two quick direction changes each way (R ~13 m).
        const float len = 44.0f, u0 = 0.5f * (L - len);
        if (u > u0 && u < u0 + len) {
            kind = Piece::Chicane;
            const float s = std::sin(kPi * (u - u0) / len);
            return -8.0f * s * s;
        }
    } else if (edge == kSlalomEdge) {
        // Three full waves of +-2.5 m over 150 m (R ~25 m at the apexes), faded in and out.
        const float len = 150.0f, u0 = 10.0f;
        if (u > u0 && u < u0 + len) {
            kind = Piece::Slalom;
            const float t = (u - u0) / len;
            return 2.5f * std::sin(2.0f * kPi * 3.0f * t) * std::sin(kPi * t);
        }
    }
    return 0.0f;
}

Layout buildLayout() {
    Layout L;
    MiniTrackInfo& info = L.info;
    info.halfWidth = kHalfWidth;
    info.area = {kX0, kX1, kZ0, kZ1};

    // Tangent points and arc centers of every rounded corner.
    struct Fillet {
        float inX, inZ, outX, outZ, cx, cz, a0, sweep, r;
    };
    Fillet f[kCornerCount];
    for (int i = 0; i < kCornerCount; ++i) {
        const Corner& p = kCorners[(i + kCornerCount - 1) % kCornerCount];
        const Corner& c = kCorners[i];
        const Corner& n = kCorners[(i + 1) % kCornerCount];
        float d0x = c.x - p.x, d0z = c.z - p.z, d1x = n.x - c.x, d1z = n.z - c.z;
        const float l0 = std::hypot(d0x, d0z), l1 = std::hypot(d1x, d1z);
        d0x /= l0, d0z /= l0, d1x /= l1, d1z /= l1;
        const float turn = std::atan2(d0x * d1z - d0z * d1x, d0x * d1x + d0z * d1z);
        const float t = c.r * std::tan(0.5f * std::abs(turn));
        Fillet& F = f[i];
        F.r = c.r;
        F.inX = c.x - d0x * t, F.inZ = c.z - d0z * t;
        F.outX = c.x + d1x * t, F.outZ = c.z + d1z * t;
        float bx = d1x - d0x, bz = d1z - d0z;
        const float bl = std::hypot(bx, bz);
        bx /= bl, bz /= bl;
        const float dc = std::sqrt(t * t + c.r * c.r);
        F.cx = c.x + bx * dc, F.cz = c.z + bz * dc;
        F.a0 = std::atan2(F.inZ - F.cz, F.inX - F.cx);
        F.sweep = turn; // the radius vector turns like the heading
    }

    auto push = [&](float x, float z, Piece kind, int corner) {
        info.center.push_back({x, z});
        L.piece.push_back(kind);
        L.corner.push_back(corner);
    };
    for (int i = 0; i < kCornerCount; ++i) {
        const Fillet& F = f[i];
        const int na = std::max(1, int(std::lround(std::abs(F.sweep) * F.r / 1.0f)));
        for (int k = 0; k < na; ++k) {
            const float a = F.a0 + F.sweep * float(k) / float(na);
            push(F.cx + F.r * std::cos(a), F.cz + F.r * std::sin(a), Piece::Corner, i);
        }
        const Fillet& G = f[(i + 1) % kCornerCount];
        const float ex = G.inX - F.outX, ez = G.inZ - F.outZ, len = std::hypot(ex, ez);
        if (len < 1e-3f) continue;
        const float dx = ex / len, dz = ez / len, lx = dz, lz = -dx;
        const int ns = std::max(1, int(std::lround(len / 1.0f)));
        for (int k = 0; k < ns; ++k) {
            const float u = len * float(k) / float(ns);
            Piece kind;
            const float o = edgeOffset(i, u, len, kind);
            push(F.outX + dx * u + lx * o, F.outZ + dz * u + lz * o, kind, -1);
        }
    }

    // Start/finish line on the main straight at x = -40: rotate so that point 0 is there.
    {
        std::size_t best = 0;
        float bestD = 1e30f;
        for (std::size_t i = 0; i < info.center.size(); ++i) {
            const float d = std::hypot(info.center[i].x + 40.0f, info.center[i].z + 75.0f);
            if (d < bestD) bestD = d, best = i;
        }
        std::rotate(info.center.begin(), info.center.begin() + std::ptrdiff_t(best), info.center.end());
        std::rotate(L.piece.begin(), L.piece.begin() + std::ptrdiff_t(best), L.piece.end());
        std::rotate(L.corner.begin(), L.corner.begin() + std::ptrdiff_t(best), L.corner.end());
    }

    const std::size_t n = info.center.size();
    info.arc.assign(n, 0.0f);
    info.curvature.assign(n, 0.0f);
    L.leftX.assign(n, 0.0f);
    L.leftZ.assign(n, 1.0f);
    for (std::size_t i = 0; i < n; ++i) {
        info.arc[i] = info.length;
        const RoutePoint& a = info.center[i];
        const RoutePoint& b = info.center[(i + 1) % n];
        info.length += std::hypot(b.x - a.x, b.z - a.z);
    }
    for (std::size_t i = 0; i < n; ++i) {
        const RoutePoint& p0 = info.center[(i + n - 1) % n];
        const RoutePoint& p1 = info.center[(i + 1) % n];
        float dx = p1.x - p0.x, dz = p1.z - p0.z;
        const float l = std::hypot(dx, dz);
        dx /= l, dz /= l;
        L.leftX[i] = dz, L.leftZ[i] = -dx;
        // Signed curvature from the heading change over +-3 points; + toward the left.
        const RoutePoint& q0 = info.center[(i + n - 3) % n];
        const RoutePoint& q1 = info.center[i];
        const RoutePoint& q2 = info.center[(i + 3) % n];
        const float ax = q1.x - q0.x, az = q1.z - q0.z, bx = q2.x - q1.x, bz = q2.z - q1.z;
        const float turn = std::atan2(ax * bz - az * bx, ax * bx + az * bz);
        info.curvature[i] = -turn / (0.5f * (std::hypot(ax, az) + std::hypot(bx, bz)));
    }

    // Curbs on corners and the chicane, starting ~5 m before and ending ~5 m after, ramped over 2 m.
    std::vector<float> raw(n, 0.0f);
    for (std::size_t i = 0; i < n; ++i) raw[i] = L.piece[i] == Piece::Corner || L.piece[i] == Piece::Chicane ? 1.0f : 0.0f;
    L.curb.assign(n, 0.0f);
    L.gravel.assign(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
        float w = 0.0f;
        for (int k = -7; k <= 7; ++k) {
            const float v = raw[(i + n + std::size_t(k + int(n))) % n];
            w = std::max(w, v * std::clamp((7.0f - float(std::abs(k))) / 2.0f, 0.0f, 1.0f));
        }
        L.curb[i] = w;
        for (int k = -12; k <= 12 && !L.gravel[i]; ++k) {
            const std::size_t j = (i + n + std::size_t(k + int(n))) % n;
            if (L.piece[j] == Piece::Corner && kCorners[L.corner[j]].gravel) L.gravel[i] = 1;
        }
    }

    // Timing gates: the start/finish line, after the chicane (sector 1) and after the constant-radius corner
    // (sector 2).
    auto nearestIndex = [&](float x, float z) {
        std::size_t best = 0;
        float bestD = 1e30f;
        for (std::size_t i = 0; i < n; ++i) {
            const float d = std::hypot(info.center[i].x - x, info.center[i].z - z);
            if (d < bestD) bestD = d, best = i;
        }
        return best;
    };
    const std::size_t gateIdx[3] = {0, nearestIndex(60.0f, -101.0f), nearestIndex(122.0f, -209.0f)};
    for (int g = 0; g < 3; ++g) {
        const std::size_t i = gateIdx[g];
        info.gates[g] = {info.center[i].x, info.center[i].z, -L.leftZ[i], L.leftX[i], 12.0f};
        info.gateArc[g] = info.arc[i];
    }

    // Grid spot 12 m behind the line.
    {
        const std::size_t i = (n - 12) % n;
        const RoutePoint& p = info.center[i];
        info.start.name = "Mini handling track";
        info.start.position = Vec3(p.x, miniTrackGround(p.x, p.z) + kLift, p.z);
        info.start.yaw = std::atan2(-L.leftZ[i], L.leftX[i]); // forward = (-leftZ, leftX)
    }

    // The middle two thirds of the constant-radius corner (corners 5 and 6 share the arc center).
    {
        float s0 = 1e30f, s1 = -1e30f;
        for (std::size_t i = 0; i < n; ++i)
            if (L.piece[i] == Piece::Corner && (L.corner[i] == 5 || L.corner[i] == 6)) {
                s0 = std::min(s0, info.arc[i]);
                s1 = std::max(s1, info.arc[i]);
            }
        info.steadyArc0 = s0 + (s1 - s0) / 6.0f;
        info.steadyArc1 = s1 - (s1 - s0) / 6.0f;
        info.steadyRadius = kCorners[5].r;
    }
    return L;
}

const Layout& layout() {
    static const Layout L = buildLayout();
    return L;
}

// Distance from (x, z) to the center line segment i -> i+1 and the side (+1 left of it, -1 right).
float segmentDistance(const MiniTrackInfo& info, std::size_t i, float x, float z, float& side) {
    const std::size_t n = info.center.size();
    const RoutePoint& a = info.center[i];
    const RoutePoint& b = info.center[(i + 1) % n];
    const float ex = b.x - a.x, ez = b.z - a.z;
    const float l2 = ex * ex + ez * ez;
    const float t = l2 > 1e-8f ? std::clamp(((x - a.x) * ex + (z - a.z) * ez) / l2, 0.0f, 1.0f) : 0.0f;
    const float px = x - (a.x + ex * t), pz = z - (a.z + ez * t);
    side = (px * ez - pz * ex) >= 0.0f ? 1.0f : -1.0f; // left = (ez, -ex) / |e|
    return std::sqrt(px * px + pz * pz);
}

struct Nearest {
    float distance = 1e30f;
    float side = 1.0f;
    std::uint32_t index = 0;
};

Nearest nearestPoint(float x, float z) {
    const MiniTrackInfo& info = layout().info;
    Nearest r;
    for (std::size_t i = 0; i < info.center.size(); ++i) {
        float side = 1.0f;
        const float d = segmentDistance(info, i, x, z, side);
        if (d < r.distance) r = {d, side, std::uint32_t(i)};
    }
    return r;
}

// Nearest center line segment for every point of a regular grid, by stamping each segment into a band around it.
std::vector<Nearest> nearestField(float x0, float z0, float step, int nx, int nz, float band) {
    const MiniTrackInfo& info = layout().info;
    std::vector<Nearest> field(std::size_t(nx) * nz);
    const std::size_t n = info.center.size();
    for (std::size_t i = 0; i < n; ++i) {
        const RoutePoint& a = info.center[i];
        const RoutePoint& b = info.center[(i + 1) % n];
        const int i0 = std::max(0, int(std::floor((std::min(a.x, b.x) - band - x0) / step)));
        const int i1 = std::min(nx - 1, int(std::ceil((std::max(a.x, b.x) + band - x0) / step)));
        const int j0 = std::max(0, int(std::floor((std::min(a.z, b.z) - band - z0) / step)));
        const int j1 = std::min(nz - 1, int(std::ceil((std::max(a.z, b.z) + band - z0) / step)));
        for (int j = j0; j <= j1; ++j)
            for (int k = i0; k <= i1; ++k) {
                float side = 1.0f;
                const float d = segmentDistance(info, i, x0 + float(k) * step, z0 + float(j) * step, side);
                Nearest& f = field[std::size_t(j) * nx + k];
                if (d < f.distance) f = {d, side, std::uint32_t(i)};
            }
    }
    return field;
}

// Gravel trap: the outside of a gravel corner, between kGravelIn and kGravelOut, with a ragged outer edge.
bool gravelAt(const Nearest& q, float x, float z) {
    const Layout& L = layout();
    if (!L.gravel[q.index]) return false;
    const float k = L.info.curvature[q.index];
    if (std::abs(k) < 1e-4f || q.side * k > 0.0f) return false; // inside of the turn
    const float wobble = 1.5f * std::sin(0.31f * x + 0.7f) * std::cos(0.23f * z - 0.4f);
    return q.distance > kGravelIn && q.distance < kGravelOut + wobble;
}

SurfaceId groundSurface(const Nearest& q, float x, float z) {
    return gravelAt(q, x, z) ? SurfaceId::Gravel : SurfaceId::Grass;
}

// Octagonal stack of tyres standing on the ground.
void addTyreStack(TrackGeometry& g, float x, float z, float radius, float height) {
    const std::size_t first = g.triangleCount();
    const float y0 = miniTrackGround(x, z) - 0.15f, y1 = y0 + 0.15f + height;
    constexpr int kSides = 8;
    for (int k = 0; k < kSides; ++k) {
        const float a0 = 2.0f * kPi * float(k) / kSides, a1 = 2.0f * kPi * float(k + 1) / kSides;
        const Vec3 b0(x + radius * std::cos(a0), y0, z + radius * std::sin(a0));
        const Vec3 b1(x + radius * std::cos(a1), y0, z + radius * std::sin(a1));
        const Vec3 t0(b0.GetX(), y1, b0.GetZ()), t1(b1.GetX(), y1, b1.GetZ());
        g.addQuad(b0, t0, t1, b1, SurfaceId::Concrete);
        g.addTriangle(Vec3(x, y1, z), t1, t0, SurfaceId::Concrete);
    }
    g.paintFrom(first, Paint::Tyre);
}

// Straight barrier from (ax, az) to (bx, bz) on the ground.
void addBarrier(TrackGeometry& g, float ax, float az, float bx, float bz, float height, float thick, Paint paint) {
    const std::size_t first = g.triangleCount();
    const float cx = 0.5f * (ax + bx), cz = 0.5f * (az + bz), len = std::hypot(bx - ax, bz - az);
    const float y0 = std::min(miniTrackGround(ax, az), miniTrackGround(bx, bz)) - 0.2f;
    const float y1 = std::max(miniTrackGround(ax, az), miniTrackGround(bx, bz)) + height;
    g.addBox(Vec3(cx, 0.5f * (y0 + y1), cz), Vec3(0.5f * thick, 0.5f * (y1 - y0), 0.5f * len), std::atan2(bx - ax, bz - az),
             SurfaceId::Concrete);
    g.paintFrom(first, paint);
}

} // namespace

DriveRoute MiniTrackInfo::route(float cruiseSpeed) const {
    DriveRoute r;
    r.name = "Mini handling track";
    r.cruiseSpeed = cruiseSpeed;
    for (std::size_t i = 0; i < center.size(); i += 2) r.points.push_back(center[i]);
    return r;
}

float MiniTrackInfo::nearestArc(float x, float z, float* distance) const {
    const Nearest q = nearestPoint(x, z);
    if (distance) *distance = q.distance;
    return arc[q.index];
}

const MiniTrackInfo& miniTrack() { return layout().info; }

bool insideMiniTrackArea(float x, float z) { return x >= kX0 && x <= kX1 && z >= kZ0 && z <= kZ1; }

DriftConfig miniTrackDrift() {
    const Layout& L = layout();
    const MiniTrackInfo& info = L.info;
    const std::size_t n = info.center.size();
    DriftConfig c;
    c.enabled = true;
    c.area = {{kX0, kZ0}, {kX1, kZ0}, {kX1, kZ1}, {kX0, kZ1}};
    std::vector<std::size_t> steady;
    for (std::size_t i = 0; i < n; ++i)
        if (info.arc[i] >= info.steadyArc0 && info.arc[i] <= info.steadyArc1) steady.push_back(i);
    if (steady.size() < 2) return c;
    // The corner turns toward the side of its curvature; the outside is the other one.
    const float out = info.curvature[steady[steady.size() / 2]] > 0.0f ? -1.0f : 1.0f;
    auto at = [&](std::size_t i, float u) {
        return DriftPoint{info.center[i].x + L.leftX[i] * u, info.center[i].z + L.leftZ[i] * u};
    };
    DriftZone outer;
    outer.name = "Outer zone";
    outer.multiplier = 2.0f;
    for (std::size_t i : steady) outer.polygon.push_back(at(i, out * (kHalfWidth - 2.5f)));
    for (auto it = steady.rbegin(); it != steady.rend(); ++it) outer.polygon.push_back(at(*it, out * kEdge));
    c.zones.push_back(std::move(outer));
    // Clipping point: the inner edge over the middle third of the corner.
    DriftZone apex;
    apex.name = "Apex";
    apex.multiplier = 1.5f;
    apex.a = at(steady[steady.size() / 3], -out * kHalfWidth);
    apex.b = at(steady[2 * steady.size() / 3], -out * kHalfWidth);
    apex.reach = 2.5f;
    c.zones.push_back(std::move(apex));
    return c;
}

float miniTrackGround(float x, float z) {
    float h = elevation(x, z);
    // Berm along the area walls (buries the wall bottoms), up to 1.5 m at the edge.
    const float d = std::min(std::min(x - kX0, kX1 - x), std::min(z - kZ0, kZ1 - z));
    if (d < 6.0f) h += 1.5f * (1.0f - std::max(d, 0.0f) / 6.0f) * (1.0f - std::max(d, 0.0f) / 6.0f);
    return h;
}

SurfaceId miniTrackSurface(float x, float z) {
    const Nearest q = nearestPoint(x, z);
    if (q.distance < kEdge) return SurfaceId::Asphalt;
    return groundSurface(q, x, z);
}

void buildMiniTrack(TrackGeometry& g) {
    const Layout& L = layout();
    const MiniTrackInfo& info = L.info;
    const std::size_t n = info.center.size();

    // ---- ground grid (grass, gravel traps), lowered just under the ribbon and left out where the ribbon covers it.
    {
        const int nx = int(std::lround((kX1 - kX0) / kGrid)), nz = int(std::lround((kZ1 - kZ0) / kGrid));
        const std::vector<Nearest> vtx = nearestField(kX0, kZ0, kGrid, nx + 1, nz + 1, kEdge + 2.0f);
        const std::vector<Nearest> cell = nearestField(kX0 + 0.5f * kGrid, kZ0 + 0.5f * kGrid, kGrid, nx, nz, kGravelOut + 3.0f);
        std::vector<float> h(std::size_t(nx + 1) * (nz + 1));
        for (int j = 0; j <= nz; ++j)
            for (int i = 0; i <= nx; ++i) {
                const float x = kX0 + float(i) * kGrid, z = kZ0 + float(j) * kGrid;
                const float d = vtx[std::size_t(j) * (nx + 1) + i].distance;
                h[std::size_t(j) * (nx + 1) + i] = miniTrackGround(x, z) - 0.3f * std::clamp((kEdge - d) / 1.0f, 0.0f, 1.0f);
            }
        auto P = [&](int i, int j) { return Vec3(kX0 + float(i) * kGrid, h[std::size_t(j) * (nx + 1) + i], kZ0 + float(j) * kGrid); };
        auto covered = [&](int i, int j) { return vtx[std::size_t(j) * (nx + 1) + i].distance < kEdge - 1.2f; };
        for (int j = 0; j < nz; ++j)
            for (int i = 0; i < nx; ++i) {
                if (covered(i, j) && covered(i + 1, j) && covered(i, j + 1) && covered(i + 1, j + 1)) continue;
                const float cx = kX0 + (float(i) + 0.5f) * kGrid, cz = kZ0 + (float(j) + 0.5f) * kGrid;
                const SurfaceId s = groundSurface(cell[std::size_t(j) * nx + i], cx, cz);
                const std::size_t first = g.triangleCount();
                g.addTriangle(P(i, j), P(i, j + 1), P(i + 1, j), s);
                g.addTriangle(P(i + 1, j), P(i, j + 1), P(i + 1, j + 1), s);
                if (s == SurfaceId::Gravel) g.paintFrom(first, Paint::Gravel);
            }
    }

    // ---- asphalt ribbon: shoulders / curbs, white edge lines, a checkered start/finish line; skirts at the edges.
    {
        // Column offsets toward the left (vehicle +X), from the left edge to the right edge.
        constexpr float off[] = {kEdge,  kHalfWidth,  4.85f,  4.65f,  3.5f,  2.33f, 1.17f, 0.0f,
                                 -1.17f, -2.33f,      -3.5f,  -4.65f, -4.85f, -kHalfWidth, -kEdge};
        constexpr int kCols = int(sizeof(off) / sizeof(off[0]));
        auto V = [&](std::size_t i, int c) {
            i %= n;
            const float x = info.center[i].x + L.leftX[i] * off[c], z = info.center[i].z + L.leftZ[i] * off[c];
            const float curb = (c == 0 || c == kCols - 1) ? kCurbHeight * L.curb[i] : 0.0f;
            return Vec3(x, miniTrackGround(x, z) + kLift + curb, z);
        };
        for (std::size_t i = 0; i < n; ++i) {
            const bool curbRow = std::max(L.curb[i], L.curb[(i + 1) % n]) > 0.05f;
            const bool redStripe = int(std::floor(info.arc[i] / 2.0f)) % 2 == 0;
            for (int c = 0; c + 1 < kCols; ++c) {
                Paint p = Paint::Asphalt;
                if (c == 0 || c == kCols - 2) p = curbRow ? (redStripe ? Paint::CurbRed : Paint::CurbWhite) : Paint::Asphalt;
                else if (c == 2 || c == kCols - 4) p = Paint::Line;
                else if (i == 0 && c >= 3 && c <= kCols - 5) p = (c % 2 == 0) ? Paint::Line : Paint::Asphalt;
                const std::size_t first = g.triangleCount();
                g.addQuad(V(i, c), V(i, c + 1), V(i + 1, c + 1), V(i + 1, c), SurfaceId::Asphalt);
                g.paintFrom(first, p);
            }
            // Edge skirts down into the ground (no gaps under the raised edge).
            const std::size_t first = g.triangleCount();
            const Vec3 l0 = V(i, 0), l1 = V(i + 1, 0), r0 = V(i, kCols - 1), r1 = V(i + 1, kCols - 1);
            const Vec3 down(0.0f, -0.4f - kCurbHeight, 0.0f);
            g.addQuad(l0 + down, l0, l1, l1 + down, SurfaceId::Asphalt);
            g.addQuad(r1 + down, r1, r0, r0 + down, SurfaceId::Asphalt);
            g.paintFrom(first, curbRow ? (redStripe ? Paint::CurbRed : Paint::CurbWhite) : Paint::Asphalt);
        }
    }

    // ---- tyre walls behind the gravel traps (one row of 0.8 m stacks, 0.7 m apart), kept clear of the other
    // parts of the circuit and of the area walls.
    {
        constexpr float kSpacing = 0.7f;
        float need = 0.0f, px = 0.0f, pz = 0.0f;
        bool havePrev = false;
        auto place = [&](float x, float z) {
            if (x < kX0 + 4.0f || x > kX1 - 4.0f || z < kZ0 + 4.0f || z > kZ1 - 4.0f) return;
            if (nearestPoint(x, z).distance < kTyreOffset - 1.0f) return;
            addTyreStack(g, x, z, 0.36f, 0.8f);
        };
        for (std::size_t i = 0; i < n; ++i) {
            const float k = info.curvature[i];
            if (L.piece[i] != Piece::Corner || !kCorners[L.corner[i]].gravel || std::abs(k) < 1e-4f) {
                havePrev = false;
                continue;
            }
            const float side = k > 0.0f ? -1.0f : 1.0f; // outside of the turn
            const float x = info.center[i].x + L.leftX[i] * side * kTyreOffset;
            const float z = info.center[i].z + L.leftZ[i] * side * kTyreOffset;
            if (!havePrev) {
                place(x, z);
                need = kSpacing;
            } else {
                // The offset curve is longer than the center line outside a turn: several stacks per step.
                const float segLen = std::hypot(x - px, z - pz);
                float pos = 0.0f;
                while (need <= segLen - pos) {
                    pos += need;
                    const float t = pos / std::max(segLen, 1e-4f);
                    place(px + (x - px) * t, pz + (z - pz) * t);
                    need = kSpacing;
                }
                need -= segLen - pos;
            }
            px = x, pz = z;
            havePrev = true;
        }
    }

    // ---- pit wall between the main straight and the parallel section behind it (red / white 4 m elements).
    for (int k = 0; k < 56; ++k) {
        const float x0 = -100.0f + 4.0f * float(k);
        addBarrier(g, x0, -88.0f, x0 + 4.0f, -88.0f, 0.9f, 0.5f, k % 2 == 0 ? Paint::BarrierRed : Paint::BarrierWhite);
    }

    // ---- area walls (the proving ground's south wall closes the north side between x = -51 and 51).
    {
        const float wy = 0.5f * (kWallTop + kWallBottom), whh = 0.5f * (kWallTop - kWallBottom), t = 0.5f;
        const float cx = 0.5f * (kX0 + kX1), hx = 0.5f * (kX1 - kX0) + 2.0f * t;
        const float zN = -50.0f; // north face line of the proving ground wall
        const float cz = 0.5f * (kZ0 + zN), hz = 0.5f * (zN - kZ0) + 2.0f * t;
        g.addBox(Vec3(kX1 + t, wy, cz), Vec3(t, whh, hz), 0.0f, SurfaceId::Concrete);
        g.addBox(Vec3(kX0 - t, wy, cz), Vec3(t, whh, hz), 0.0f, SurfaceId::Concrete);
        g.addBox(Vec3(cx, wy, kZ0 - t), Vec3(hx, whh, t), 0.0f, SurfaceId::Concrete);
        const float wx0 = 0.5f * (kX0 - 1.0f + -51.0f), wx1 = 0.5f * (51.0f + kX1 + 1.0f), whx = 0.5f * (-51.0f - (kX0 - 1.0f));
        g.addBox(Vec3(wx0, wy, zN - t), Vec3(whx, whh, t), 0.0f, SurfaceId::Concrete);
        g.addBox(Vec3(wx1, wy, zN - t), Vec3(whx, whh, t), 0.0f, SurfaceId::Concrete);
    }
}

} // namespace tf
