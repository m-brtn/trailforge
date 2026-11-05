#pragma once

#include <string>
#include <vector>

namespace quench::json {
struct Value;
}

namespace tf {

namespace json = quench::json;

// Drift scoring: a small judge fed with the body motion of every physics tick. Pure logic, no physics; it only reads
// simulation data (body heading and velocity, wheel contact, body impacts) and never feeds back.
//
// Body slip angle = the angle between the body heading and its horizontal velocity. A tick counts as drifting when
// |slip| >= minAngleDeg at >= minSpeedKmh with at least two wheels on the ground (inside the course area, if one is
// given). The first drifting tick starts a run; every drifting tick adds
//     pointsScale * min(|slip|, maxScoredAngleDeg) [deg] * speed [km/h] * dt [s] * zone multiplier.
// A run ends:
//   - completed: not drifting for linkSeconds (drifts closer together are linked into one run), or leaving the area;
//     its score is the run result (session best, stored best per level and vehicle by the application);
//   - failed: a spin (|slip| > spinAngleDeg above 3 m/s) or a body impact approaching at >= wallSpeed (a wall, a
//     barrier, a rollover); the run scores 0;
//   - aborted: a reset, teleport or rebuild (abort()); not counted.
struct DriftPoint {
    float x = 0.0f, z = 0.0f;
};

// Clipping zone: a polygon (x/z, any winding), or a line (a wall face, from a to b) with a reach in m on both sides.
// A zone counts while the body origin or the rear bumper point is inside it; overlapping zones: the largest
// multiplier.
struct DriftZone {
    std::string name;
    float multiplier = 2.0f;
    std::vector<DriftPoint> polygon; // >= 3 points; empty: a line zone
    DriftPoint a, b;                 // line zone
    float reach = 2.0f;              // m, line zone
    bool contains(float x, float z) const;
};

// level.json "drift": {"enabled": true, "minSpeedKmh": 20, "minAngleDeg": 12, "spinAngleDeg": 100,
//   "maxScoredAngleDeg": 70, "linkSeconds": 1.5, "wallSpeed": 1.5, "pointsScale": 0.1,
//   "area": [[x, z], ...],                                   (optional course polygon; empty: everywhere)
//   "zones": [{"name": "Outer wall T1", "multiplier": 2, "line": [[x0, z0], [x1, z1]], "reach": 2.5},
//             {"name": "Clip 1", "multiplier": 1.5, "polygon": [[x, z], [x, z], [x, z], ...]}]}
// Every key is optional (defaults below); a "drift" object without "enabled" is enabled.
struct DriftConfig {
    bool enabled = false;
    float minSpeedKmh = 20.0f;
    float minAngleDeg = 12.0f;
    float spinAngleDeg = 100.0f;
    float maxScoredAngleDeg = 70.0f;
    float linkSeconds = 1.5f;
    float wallSpeed = 1.5f; // m/s body impact approach speed that ends the run
    float pointsScale = 0.1f;
    std::vector<DriftPoint> area;
    std::vector<DriftZone> zones;
};

// Reads a level.json "drift" value into out (a null value leaves the defaults, disabled). false + err on bad data.
bool parseDriftConfig(const json::Value& v, DriftConfig& out, std::string& err);

// One physics tick of body motion.
struct DriftInput {
    float dt = 0.0f;
    float x = 0.0f, z = 0.0f;         // body origin
    float rearX = 0.0f, rearZ = 0.0f; // rear bumper center
    float fx = 0.0f, fz = 1.0f;       // body forward (+Z) projected on the ground, unit
    float vx = 0.0f, vz = 0.0f;       // body velocity, m/s (horizontal part used)
    int wheelsOnGround = 4;
    float impactSpeed = 0.0f;         // fastest body impact approach of the tick, m/s (0: none)
};

class DriftScorer {
public:
    enum class Event { None, Started, Completed, Failed };
    enum class End { None, Completed, LeftArea, Spin, Wall, Aborted };

    // Replaces the config; the running run is dropped and the session best is cleared (another course).
    void setConfig(DriftConfig config);
    const DriftConfig& config() const { return m_config; }
    bool active() const { return m_config.enabled; }
    // Drops the running run (reset, teleport, rebuild): not counted, no score.
    void abort();
    // Forgets the session (another vehicle): run, last and best.
    void clearSession();

    Event update(const DriftInput& in);

    // Live values of the last tick (also while no run is running).
    float slipAngleDeg() const { return m_slip; }   // signed, + = velocity to the left of the heading
    float speedKmh() const { return m_speed; }
    bool drifting() const { return m_drifting; }    // this tick counted
    bool inArea() const { return m_inArea; }        // inside the course area (always true without one)
    int zone() const { return m_zone; }             // config().zones index the vehicle is in, -1: none
    float multiplier() const { return m_mult; }
    // Running run (zeroed when the next one starts).
    bool running() const { return m_running; }
    double score() const { return m_score; }
    double runTime() const { return m_runTime; }     // s since the run started
    double driftTime() const { return m_driftTime; } // s of drifting ticks in the run
    double zoneTime() const { return m_zoneTime; }   // s of drifting ticks in a clipping zone
    float maxAngleDeg() const { return m_maxAngle; } // largest |slip| of a drifting tick in the run
    // Results.
    int runs() const { return m_runs; }               // completed runs (failed ones not included)
    int failedRuns() const { return m_failed; }
    double lastScore() const { return m_last; }       // last completed or failed run (0 after a failure)
    double lastLost() const { return m_lost; }        // score of the last failed run before it failed
    double bestScore() const { return m_best; }       // session best of the completed runs
    End lastEnd() const { return m_lastEnd; }
    static const char* endName(End e);

private:
    Event finish(End why);

    DriftConfig m_config;
    float m_slip = 0.0f, m_speed = 0.0f, m_mult = 1.0f, m_maxAngle = 0.0f;
    bool m_drifting = false, m_running = false, m_inArea = false;
    int m_zone = -1;
    double m_score = 0.0, m_runTime = 0.0, m_driftTime = 0.0, m_zoneTime = 0.0, m_idle = 0.0;
    int m_runs = 0, m_failed = 0;
    double m_last = 0.0, m_lost = 0.0, m_best = 0.0;
    End m_lastEnd = End::None;
};

} // namespace tf
