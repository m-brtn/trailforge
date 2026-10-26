#include "sim/LapTimer.h"

#include <cmath>

namespace tf {

void LapTimer::setGates(std::vector<TimingGate> gates) {
    m_gates = std::move(gates);
    clearAll();
}

void LapTimer::reset() {
    m_running = false;
    m_next = 0;
    m_sectors.clear();
}

void LapTimer::clearAll() {
    reset();
    m_laps = 0;
    m_lastLap = m_bestLap = -1.0;
    m_lastSectors.clear();
}

double LapTimer::crossing(const TimingGate& g, float x0, float z0, float x1, float z1, double t0, double t1) const {
    const float s0 = (x0 - g.x) * g.dirX + (z0 - g.z) * g.dirZ;
    const float s1 = (x1 - g.x) * g.dirX + (z1 - g.z) * g.dirZ;
    if (!(s0 < 0.0f && s1 >= 0.0f)) return -1.0;
    const float f = -s0 / (s1 - s0);
    const float x = x0 + (x1 - x0) * f, z = z0 + (z1 - z0) * f;
    const float lateral = (x - g.x) * g.dirZ - (z - g.z) * g.dirX;
    if (std::abs(lateral) > g.halfWidth) return -1.0;
    return t0 + (t1 - t0) * double(f);
}

void LapTimer::update(float x0, float z0, float x1, float z1, double t0, double t1) {
    if (m_gates.empty()) return;
    const int count = int(m_gates.size());
    for (int k = 0; k < count; ++k) {
        const double t = crossing(m_gates[std::size_t(k)], x0, z0, x1, z1, t0, t1);
        if (t < 0.0) continue;
        if (k == 0) {
            if (m_running && m_next == 0) {
                // A complete lap: the last sector ends here.
                m_sectors.push_back(t - m_splitStart);
                const double lap = t - m_lapStart;
                ++m_laps;
                m_lastLap = lap;
                m_lastSectors = m_sectors;
                if (m_bestLap < 0.0 || lap < m_bestLap) m_bestLap = lap;
            }
            // Start (or restart after a voided lap) the clock.
            m_running = true;
            m_lapStart = m_splitStart = t;
            m_next = count > 1 ? 1 : 0;
            m_sectors.clear();
        } else if (m_running && k == m_next) {
            m_sectors.push_back(t - m_splitStart);
            m_splitStart = t;
            m_next = (k + 1) % count;
        } else if (m_running) {
            // A sector gate out of order: the lap is void until the next start/finish crossing.
            reset();
        }
    }
}

} // namespace tf
