#pragma once

#include "track/SurfaceProperties.h"
#include "vehicle/WheelContact.h"

#include <Jolt/Jolt.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace tf {

// One piece of a tire mark ribbon, from p0 to p1. Consecutive segments of a ribbon share the end point and the side
// vector, so the ribbon is continuous. Plain floats in the layout of the `TireMarkSegment` struct in
// shaders/scene.hlsl (5 x float4); the renderer uploads the ring buffer as is.
struct TireMarkSegment {
    float p0[3];    // world, m: patch center at the start
    float u0;       // m: distance along the ribbon at p0 (tread pattern coordinate)
    float p1[3];
    float u1;
    float side0[3]; // world: half-width vector at p0, perpendicular to the travel direction in the contact plane
    float birth;    // s: simulation time when the segment was written
    float side1[3];
    float surface;  // packed: SurfaceId + 8 * soil code (packMarkSurface), an integer stored exactly in the float
    float imprint0, imprint1; // 0..1: rolling imprint (soft ground) or soil print from the tread (hard ground)
    float slip0, slip1;       // 0..1: rubber (hard surfaces) or churn (soft surfaces) from the patch sliding speed
};
static_assert(sizeof(TireMarkSegment) == 80, "TireMarkSegment must match the shader layout");

// Segment surface word: the SurfaceId in the low 3 bits, above it the colour (3 x 5 bits, sqrt-encoded linear
// albedo up to 0.5) and the wetness (3 bits) of the soil the tread carried. 21 bits, exact in a float.
float packMarkSurface(SurfaceId surface, const float soilColor[3], float soilWetness);
SurfaceId markSurface(const TireMarkSegment& s);
void markSoil(const TireMarkSegment& s, float color[3], float& wetness);

struct TireMarkSettings {
    bool enabled = true;
    int maxSegments = 16384;      // ring buffer capacity (hard cap); the oldest segment is overwritten
    float fadeTime = 240.0f;      // s: segments fade out over the last 40 % of this age and are then dropped
    float segmentLength = 0.15f;  // m: a new segment is written every time the patch has moved this far
    float referenceLoad = 4900.0f; // N: static load of one wheel; imprint and slip scale with load / this
};

// How a surface takes a mark. Visual response only: marks never change friction or ground height.
struct SurfaceMarkResponse {
    float imprint;    // gain of the rolling imprint (0 = hard surface, no imprint while rolling)
    float slip;       // gain of the slip mark (rubber on hard surfaces, churn on soft ones)
    float widthScale; // mark width / swept tire width (mud ruts get raised edges beyond the tire)
};
SurfaceMarkResponse surfaceMarkResponse(SurfaceId id);

// How much a patch on hard ground really slides, 0..1, for the rubber marks (the tire squeal uses the stricter
// squealSlip in audio/AudioMapping). Longitudinal (wheelspin, lock-up) and lateral sliding are gated separately, each
// by its sliding speed (m/s) and its ratio to max(patch speed, rolling speed, 1 m/s). Dead zones from
// `trailforge_probe skid` on concrete: a full-lock turn at 30 km/h scrubs sideways at 3.7 m/s / 0.39, a turn at
// 50 km/h at 3.8 m/s / 0.26, traction slip in 2nd gear stays below 2 m/s / 0.15 (all unmarked); the inner wheels of a
// full-throttle full-lock launch spin at 7-10 m/s / 0.6-0.75, lock-up and handbrake reach 11-13 m/s at 1.0 (marks).
float rubberSlide(const WheelContactSample& c);

// The driven-wheelspin part of rubberSlide (the tread turns faster than the ground passes; lock-up and sideways
// sliding excluded), 0..1. Ratios from WheelContactSample::longSlipRatio, as for the squeal gate.
float wheelspinSlide(const WheelContactSample& c);

// Rubber laid by wheelspin at a tread heat of `heat` (0..1, TireMarks::spinHeat), 0..1, and the share of the tire
// width it covers. Wheelspin lays rubber only once the tread is hot: a brief or slow spin leaves a faint, narrow
// streak, several seconds at a high sliding speed (a burnout) a dark mark of the full width.
float spinLaydown(float heat);
float spinLaydownWidth(float heat);

// Sliding that lays rubber marks, 0..1: rubberSlide() plus a locked wheel down to ~1 m/s (a locked tire keeps
// sliding until the stop, while the rubberSlide gate fades out below ~5 m/s).
float markSlide(const WheelContactSample& c);

// Loose soil of a surface as the tread picks it up (visual and sound only, never friction). Adhesion follows the
// soil: wet clay/mud sticks heavily, packed dirt and the earth under the grass moderately, dry sand barely (a light
// dusting that falls off within a meter or two).
struct SoilType {
    float pickup = 0.0f;       // amount the tread settles at on this ground (0 hard surfaces, 1 mud)
    float pickupLength = 1.0f; // m of tread travel to pick up ~63 % of it
    float wetness = 0.0f;      // 0 dry dust .. 1 wet mud
    float color[3] = {0, 0, 0}; // linear albedo of the soil
    float stick = 0.0f;        // adhesion 0..1 (sand 0.1, dirt 0.55, mud 1): shedding distance, body coat
    float flecks = 0.0f;       // fraction of torn grass blades stuck in it (grass only), drawn as sparse specks
};
SoilType soilType(SurfaceId id);
float soilPickup(SurfaceId id); // soilType(id).pickup

// Soil carried in the tread: picked up on loose ground (fast on mud, slower on dirt and grass, very little on sand),
// the colour mixes with the ground it rolls on; shed by distance on hard ground (e-folding ~1.2 m for dry sand,
// ~2.5 m for dirt, ~6 m for wet mud, faster while spinning or sliding); a puddle rinses it to a thin wet film within
// ~1 m. A dirty tread lays no rubber and does not squeal; it never changes grip.
struct TreadSoil {
    float amount = 0.0f;   // 0 clean .. 1 caked with mud
    float wetness = 0.0f;  // 0 dry .. 1 wet
    float color[3] = {0, 0, 0};
    float stick = 0.0f;    // adhesion of what the tread carries (SoilType::stick)
    float flecks = 0.0f;   // grass blades stuck in the tread, 0..1
    void update(const WheelContactSample& c, float dt, bool inWater = false);
};

// Mark intensities of one contact sample (before the per-surface render multipliers).
struct MarkIntensity {
    float imprint = 0.0f;   // soft ground: rolling imprint; hard ground: soil print of a dirty tread
    float slip = 0.0f;
    float halfWidth = 0.0f; // m
};
// `slide` is the (heat-limited) rubber slide of a hard-ground patch, < 0: markSlide(c) without build-up;
// `widthFactor` narrows the mark (wheelspin on a cool tread).
MarkIntensity markIntensity(const WheelContactSample& c, float referenceLoad, const TreadSoil& soil = {},
                            float slide = -1.0f, float widthFactor = 1.0f);

// Persistent tire marks: per-wheel ribbons written from the per-tick contact samples into a fixed ring buffer.
// Visual only and independent of the renderer (runs headless; the renderer reads the ring).
class TireMarks {
public:
    explicit TireMarks(const TireMarkSettings& settings = {});

    const TireMarkSettings& settings() const { return m_settings; }
    void setSettings(const TireMarkSettings& settings); // a capacity change clears the marks

    // Once per physics tick, after the step. `time` is the simulation time in seconds.
    void update(std::span<const WheelContactSample> wheels, double time);
    // The next contact starts a new ribbon (after a reset or teleport).
    void breakRibbons();
    void clear();
    void push(const TireMarkSegment& s);

    std::size_t capacity() const { return m_ring.size(); }
    std::size_t size() const { return m_count; }
    std::size_t firstIndex() const { return (m_head + m_ring.size() - m_count) % m_ring.size(); }
    std::size_t headIndex() const { return m_head; }
    const TireMarkSegment* data() const { return m_ring.data(); }
    const TireMarkSegment& oldest(std::size_t i) const { return m_ring[(firstIndex() + i) % m_ring.size()]; }
    const TreadSoil& soil(std::size_t wheel) const { return m_trails[std::min<std::size_t>(wheel, 3)].soil; }
    float patchHeat(std::size_t wheel) const { return m_trails[std::min<std::size_t>(wheel, 3)].heat; }
    float spinHeat(std::size_t wheel) const { return m_trails[std::min<std::size_t>(wheel, 3)].spinHeat; }
    std::uint64_t written() const { return m_written; }       // total segments ever written
    std::uint64_t generation() const { return m_generation; } // changes on clear / capacity change

private:
    struct Trail {
        bool active = false;
        JPH::Vec3 lastPos = JPH::Vec3::sZero();
        JPH::Vec3 lastSide = JPH::Vec3::sZero();
        float lastU = 0.0f;
        float lastImprint = 0.0f, lastSlip = 0.0f;
        double lastEmit = 0.0;
        float accImprint = 0.0f, accSlip = 0.0f, accHalfWidth = 0.0f;
        int accCount = 0;
        SurfaceId surface = SurfaceId::Concrete; // of the current ribbon
        float heat = 0.0f; // 0..1: lock-up / sideways rubber builds up over ~0.35 s of sliding (patch heating)
        // 0..1: tread heat from wheelspin, rises with the sliding distance (spinning speed x time) and cools in
        // a few seconds; drives spinLaydown.
        float spinHeat = 0.0f;
        float stampCover = 0.0f; // composited cover of the burnout stamps on the current spot
        TreadSoil soil;
    };
    void start(Trail& t, const WheelContactSample& c, const MarkIntensity& m, double time) const;
    // Writes the segment from the trail's last point to the patch and advances the trail.
    void emit(Trail& t, const WheelContactSample& c, float imprint, float slip, float halfWidth, float surfaceWord,
              double time);
    void expire(double time);

    TireMarkSettings m_settings;
    std::vector<TireMarkSegment> m_ring;
    std::size_t m_head = 0, m_count = 0;
    std::uint64_t m_written = 0, m_generation = 0;
    double m_lastUpdate = -1.0;
    std::array<Trail, 4> m_trails{};
};

} // namespace tf
