#include "sim/LapTimer.h"

#include <cmath>

namespace tf {

void LapTimer::setGates(std::vector<LevelGate> gates) {
    m_gates = std::move(gates);
    m_bestLap = m_lastLap = 0.0;
    m_laps = 0;
    m_sectorCount = 1;
    for (std::size_t k = 1; k < m_gates.size(); ++k)
        if (m_gates[k].sector && m_sectorCount < kMaxSectors) ++m_sectorCount;
    m_lastSector.fill(0.0);
    m_bestSector.fill(0.0);
    reset();
}

void LapTimer::endSector(double tc) {
    if (m_sectorIndex < kMaxSectors) m_sector[std::size_t(m_sectorIndex)] = tc - m_sectorStart;
    ++m_sectorIndex;
    m_sectorStart = tc;
}

void LapTimer::reset() {
    m_running = false;
    m_next = 0;
    m_last = -1;
    m_split = 0.0;
    m_sectorIndex = 0;
    m_sector.fill(0.0);
}

bool LapTimer::crossed(const LevelGate& g, float x0, float z0, float x1, float z1, float& frac) const {
    const float a0 = along(g, x0, z0), a1 = along(g, x1, z1);
    if (!(a0 < 0.0f && a1 >= 0.0f)) return false;
    frac = a0 / (a0 - a1);
    const float x = x0 + (x1 - x0) * frac, z = z0 + (z1 - z0) * frac;
    return std::abs(across(g, x, z)) <= g.halfWidth;
}

LapTimer::Event LapTimer::update(float x0, float z0, float x1, float z1, double t0, double t1) {
    const double t = t1, dt = t1 - t0;
    if (m_gates.empty()) return Event::None;
    float frac = 0.0f;
    const LevelGate& finish = m_gates[0];
    if (crossed(finish, x0, z0, x1, z1, frac)) {
        const double tc = t - dt + dt * double(frac);
        Event e = Event::Started;
        if (m_running) {
            if (m_next == 0) {
                m_lastLap = tc - m_lapStart;
                if (m_bestLap <= 0.0 || m_lastLap < m_bestLap) m_bestLap = m_lastLap;
                endSector(tc);
                m_lastSector = m_sector;
                for (int k = 0; k < m_sectorCount; ++k)
                    if (m_bestSector[std::size_t(k)] <= 0.0 || m_sector[std::size_t(k)] < m_bestSector[std::size_t(k)])
                        m_bestSector[std::size_t(k)] = m_sector[std::size_t(k)];
                ++m_laps;
                e = Event::Lap;
            } else {
                e = Event::Invalid;
            }
        }
        m_running = true;
        m_lapStart = tc;
        m_next = m_gates.size() > 1 ? 1 : 0;
        m_last = 0;
        m_split = 0.0;
        m_sectorIndex = 0;
        m_sector.fill(0.0);
        m_sectorStart = tc;
        return e;
    }
    if (!m_running || m_next == 0) return Event::None;
    const LevelGate& g = m_gates[std::size_t(m_next)];
    if (!crossed(g, x0, z0, x1, z1, frac)) return Event::None;
    m_split = t - dt + dt * double(frac) - m_lapStart;
    if (g.sector && m_sectorIndex + 1 < m_sectorCount) endSector(m_lapStart + m_split);
    m_last = m_next;
    m_next = m_next + 1 < int(m_gates.size()) ? m_next + 1 : 0;
    return Event::Checkpoint;
}

} // namespace tf
