#include "app/BestLaps.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace tf {
namespace {

bool validId(const std::string& s) {
    if (s.empty() || s.size() > 64) return false;
    for (char c : s)
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
    return true;
}

} // namespace

double BestLaps::best(const std::string& level, const std::string& vehicle) const {
    const auto it = m_best.find({level, vehicle});
    return it == m_best.end() ? 0.0 : it->second;
}

bool BestLaps::submit(const std::string& level, const std::string& vehicle, double seconds) {
    if (!validId(level) || !validId(vehicle) || !(seconds > 0.0) || !std::isfinite(seconds)) return false;
    auto [it, inserted] = m_best.try_emplace({level, vehicle}, seconds);
    if (inserted) return true;
    if (m_order == Order::Lower ? seconds >= it->second : seconds <= it->second) return false;
    it->second = seconds;
    return true;
}

std::string BestLaps::serialize() const {
    std::string out = m_order == Order::Lower ? "# TrailForge best laps: level vehicle seconds\n"
                                              : "# TrailForge best drift scores: level vehicle points\n";
    char line[192];
    for (const auto& [key, t] : m_best) {
        std::snprintf(line, sizeof(line), "%s %s %.3f\n", key.first.c_str(), key.second.c_str(), t);
        out += line;
    }
    return out;
}

void BestLaps::parse(const std::string& text, int* skipped) {
    m_best.clear();
    int bad = 0;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        if (const auto hash = line.find('#'); hash != std::string::npos) line.resize(hash);
        std::istringstream ls(line);
        std::string level, vehicle, rest;
        double t = 0.0;
        if (!(ls >> level)) continue;
        if (!(ls >> vehicle >> t) || (ls >> rest) || !submit(level, vehicle, t)) ++bad;
    }
    if (skipped) *skipped = bad;
}

bool BestLaps::save(const std::string& path, std::string* error) const {
    const std::string tmp = path + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) {
            if (error) *error = "cannot write " + tmp;
            return false;
        }
        out << serialize();
        if (!out.flush()) {
            if (error) *error = "write failed: " + tmp;
            return false;
        }
    }
    std::remove(path.c_str()); // rename() does not replace an existing file on Windows
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        if (error) *error = "cannot rename " + tmp + " -> " + path;
        return false;
    }
    return true;
}

bool BestLaps::load(const std::string& path, int* skipped) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        m_best.clear();
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    parse(ss.str(), skipped);
    return true;
}

} // namespace tf
