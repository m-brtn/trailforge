#pragma once

#include "track/MiniTrack.h"

#include <vector>

namespace tf {

// Lap and sector timing over an ordered list of gates (gate 0 = start/finish, then one gate per sector end). Pure
// logic: fed with the vehicle position once per physics tick, crossing times are interpolated inside the tick.
// The first start/finish crossing starts the clock; a lap counts only when every sector gate was crossed in order
// since then (a cut or a reverse pass voids the running lap, which restarts at the next start/finish crossing).
class LapTimer {
public:
    void setGates(std::vector<TimingGate> gates);
    bool hasGates() const { return !m_gates.empty(); }
    const std::vector<TimingGate>& gates() const { return m_gates; }
    // Forget the running lap (respawn, teleport); completed laps and the best lap stay.
    void reset();
    void clearAll();

    // Vehicle position at the previous and the current tick (x/z) and their simulation times.
    void update(float x0, float z0, float x1, float z1, double t0, double t1);

    bool running() const { return m_running; }
    double currentLap(double now) const { return m_running ? now - m_lapStart : 0.0; }
    int laps() const { return m_laps; }
    double lastLap() const { return m_lastLap; } // s, < 0 before the first lap
    double bestLap() const { return m_bestLap; } // s, < 0 before the first lap
    int nextGate() const { return m_next; }       // gate expected next (0 = the finish line)
    // Sector times of the running lap so far and of the last completed lap, s.
    const std::vector<double>& currentSectors() const { return m_sectors; }
    const std::vector<double>& lastSectors() const { return m_lastSectors; }

private:
    // Crossing time of gate g between the two positions, or < 0 when it was not crossed forward.
    double crossing(const TimingGate& g, float x0, float z0, float x1, float z1, double t0, double t1) const;

    std::vector<TimingGate> m_gates;
    bool m_running = false;
    int m_next = 0;
    double m_lapStart = 0.0, m_splitStart = 0.0;
    int m_laps = 0;
    double m_lastLap = -1.0, m_bestLap = -1.0;
    std::vector<double> m_sectors, m_lastSectors;
};

} // namespace tf
