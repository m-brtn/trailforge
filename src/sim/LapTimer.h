#pragma once

#include "track/Level.h"

#include <algorithm>
#include <array>
#include <vector>

namespace tf {

// Lap timing over the gates of a circuit (Level::circuit(), the proving ground's mini handling track): gate 0 is the
// start / finish line, the others are checkpoints in driving order. Fed with the body position before and after every physics tick; a gate counts when
// the segment between them crosses its line forward within its half width, at the interpolated crossing time.
// Crossing the start line starts a lap; a lap counts only if every checkpoint was passed in order, otherwise it
// starts over at the line ("lap not counted"). Pure logic, no physics.
class LapTimer {
public:
    enum class Event { None, Started, Checkpoint, Lap, Invalid };

    void setGates(std::vector<LevelGate> gates);
    // Clears the running lap and the session's laps and best (another vehicle).
    void clearAll() { setGates(std::vector<LevelGate>(m_gates)); }
    const std::vector<LevelGate>& gates() const { return m_gates; }
    bool active() const { return !m_gates.empty(); }
    bool hasGates() const { return active(); }
    // Forgets the running lap (a respawn or teleport); completed laps and the best lap of the session are kept.
    void reset();

    // Body origin moved from (x0, z0) at simulation time t0 to (x1, z1) at t1 (one physics tick).
    Event update(float x0, float z0, float x1, float z1, double t0, double t1);

    bool running() const { return m_running; }
    int nextGate() const { return m_next; }      // index of the gate expected next (0: the finish line)
    int lastGate() const { return m_last; }      // last gate passed in this lap (-1: none)
    int laps() const { return m_laps; }          // valid laps completed
    double lapTime(double now) const { return m_running ? now - m_lapStart : 0.0; }
    double currentLap(double now) const { return lapTime(now); }
    double lastLap() const { return m_lastLap; } // s, 0: none yet
    double bestLap() const { return m_bestLap; } // s, 0: none yet (this session)
    double lastSplit() const { return m_split; } // lap time at the last checkpoint
    // Timing sectors: the lap split at the gates marked sector (and the finish), at most kMaxSectors.
    static constexpr int kMaxSectors = 4;
    int sectorCount() const { return m_sectorCount; }
    // Sector k of the running lap (s; 0: not finished yet) and of the last valid lap.
    double sectorTime(int k) const { return k >= 0 && k < kMaxSectors ? m_sector[std::size_t(k)] : 0.0; }
    double lastSectorTime(int k) const { return k >= 0 && k < kMaxSectors ? m_lastSector[std::size_t(k)] : 0.0; }
    double bestSectorTime(int k) const { return k >= 0 && k < kMaxSectors ? m_bestSector[std::size_t(k)] : 0.0; }
    // Finished sectors of the running lap, and all sectors of the last valid lap (empty before the first lap).
    std::vector<double> currentSectors() const { return {m_sector.begin(), m_sector.begin() + std::min(m_sectorIndex, m_sectorCount)}; }
    std::vector<double> lastSectors() const { return m_laps ? std::vector<double>(m_lastSector.begin(), m_lastSector.begin() + m_sectorCount) : std::vector<double>{}; }
    int checkpointsPassed() const { return m_running ? (m_next == 0 ? int(m_gates.size()) - 1 : m_next - 1) : 0; }

    // Signed distance of (x, z) in front of gate g (m, + past it along its direction) and the lateral offset.
    static float along(const LevelGate& g, float x, float z) { return (x - g.x) * g.dx + (z - g.z) * g.dz; }
    static float across(const LevelGate& g, float x, float z) { return (x - g.x) * g.dz - (z - g.z) * g.dx; }

private:
    bool crossed(const LevelGate& g, float x0, float z0, float x1, float z1, float& frac) const;

    std::vector<LevelGate> m_gates;
    bool m_running = false;
    int m_next = 0, m_last = -1, m_laps = 0;
    double m_lapStart = 0.0, m_lastLap = 0.0, m_bestLap = 0.0, m_split = 0.0;
    int m_sectorCount = 1, m_sectorIndex = 0;
    double m_sectorStart = 0.0;
    std::array<double, kMaxSectors> m_sector{}, m_lastSector{}, m_bestSector{};
    void endSector(double tc);
};

} // namespace tf
