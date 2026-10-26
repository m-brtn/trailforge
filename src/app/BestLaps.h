#pragma once

#include <map>
#include <string>
#include <utility>

namespace tf {

// Best lap time per level and vehicle, stored as `best_laps.cfg` in the user data directory: one
// `<level> <vehicle> <seconds>` line per pair ('#' starts a comment; ids are [a-z0-9_]). No SDL, testable headless.
class BestLaps {
public:
    // 0 if there is none.
    double best(const std::string& level, const std::string& vehicle) const;
    // Stores the time if it beats the stored one (or there is none); true if it did.
    bool submit(const std::string& level, const std::string& vehicle, double seconds);
    void clear() { m_best.clear(); }
    std::size_t size() const { return m_best.size(); }

    std::string serialize() const;
    // Malformed lines are skipped (counted in *skipped).
    void parse(const std::string& text, int* skipped = nullptr);
    // Through a temporary file and a rename. load() returns false if the file does not exist (the table is cleared).
    bool save(const std::string& path, std::string* error = nullptr) const;
    bool load(const std::string& path, int* skipped = nullptr);

private:
    std::map<std::pair<std::string, std::string>, double> m_best;
};

} // namespace tf
