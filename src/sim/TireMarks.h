#pragma once

#include "track/SurfaceProperties.h"
#include "vehicle/WheelContact.h"

#include <Jolt/Jolt.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace tf {

// One piece of a tire mark ribbon, from p0 to p1. Consecutive segments of a ribbon share the end point and the side
// vector, so the ribbon is continuous. Plain floats in the layout of the `TireMarkSegment` struct in
// shaders/scene.metal (5 x float4); the renderer uploads the ring buffer as is.
struct TireMarkSegment {
    float p0[3];    // world, m: patch center at the start
    float u0;       // m: distance along the ribbon at p0 (tread pattern coordinate)
    float p1[3];
    float u1;
    float side0[3]; // world: half-width vector at p0, perpendicular to the travel direction in the contact plane
    float birth;    // s: simulation time when the segment was written
    float side1[3];
    float surface;  // SurfaceId as a float
    float imprint0, imprint1; // 0..1: rolling imprint (load-driven) at p0/p1
    float slip0, slip1;       // 0..1: rubber (hard surfaces) or churn (soft surfaces) from the patch sliding speed
};
static_assert(sizeof(TireMarkSegment) == 80, "TireMarkSegment must match the shader layout");

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

// Mark intensities of one contact sample (before the per-surface render multipliers).
struct MarkIntensity {
    float imprint = 0.0f;
    float slip = 0.0f;
    float halfWidth = 0.0f; // m
};
MarkIntensity markIntensity(const WheelContactSample& c, float referenceLoad);

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
    };
    void start(Trail& t, const WheelContactSample& c, const MarkIntensity& m, double time) const;
    void expire(double time);

    TireMarkSettings m_settings;
    std::vector<TireMarkSegment> m_ring;
    std::size_t m_head = 0, m_count = 0;
    std::uint64_t m_written = 0, m_generation = 0;
    std::array<Trail, 4> m_trails{};
};

} // namespace tf
