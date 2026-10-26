#include "app/Scenario.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>

namespace tf {
namespace {

const std::set<std::string> kKnown = {"wait", "until", "expect", "down", "up", "tap", "text", "click", "inputclick",
                                      "scrollto", "teleport", "resize", "capture", "reset_stats", "log", "quit",
                                      "mark", "orbit", "wheel", "wheels", "quality", "set", "camera", "perf_begin",
                                      "perf_end", "time", "menu", "garage", "transfer", "controls", "pedal", "vehicle",
                                      "soc", "level", "place", "map", "gamepad", "mapclick", "mapaim", "mapdrag",
                                      "marker", "post_bench"};

} // namespace

std::optional<Scenario> Scenario::load(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "scenario: cannot open %s\n", path.c_str());
        return std::nullopt;
    }
    Scenario s;
    s.m_path = path;
    std::string line;
    for (int n = 1; std::getline(in, line); ++n) {
        if (const auto hash = line.find('#'); hash != std::string::npos) line.erase(hash);
        std::istringstream ss(line);
        ScenarioCommand c;
        c.line = n;
        if (!(ss >> c.op)) continue;
        if (!kKnown.count(c.op)) {
            std::fprintf(stderr, "scenario %s:%d: unknown command '%s'\n", path.c_str(), n, c.op.c_str());
            return std::nullopt;
        }
        if (c.op == "text" || c.op == "log" || c.op == "down" || c.op == "up" || c.op == "tap") {
            // Rest of the line as a whole: key names can be two words ("Left Ctrl").
            std::string rest;
            std::getline(ss >> std::ws, rest);
            while (!rest.empty() && (rest.back() == ' ' || rest.back() == '\r')) rest.pop_back();
            c.args.push_back(rest);
        } else {
            for (std::string a; ss >> a;) c.args.push_back(a);
        }
        s.m_cmds.push_back(std::move(c));
    }
    return s;
}

void Scenario::fail(const ScenarioCommand& c, const std::string& what) {
    ++m_failed;
    std::printf("SCENARIO FAIL line %d: %s\n", c.line, what.c_str());
}

bool Scenario::compare(const ScenarioCommand& c, const ValueFn& value, bool report, std::string* detail) {
    if (c.args.size() < 3) {
        if (report) fail(c, "expected: VAR OP VALUE");
        return false;
    }
    std::optional<double> v;
    if (c.args[0].rfind("d_", 0) == 0) {
        const auto it = m_marks.find(c.args[0].substr(2));
        const auto cur = value(c.args[0].substr(2));
        if (it != m_marks.end() && cur) v = *cur - it->second;
    } else {
        v = value(c.args[0]);
    }
    if (!v) {
        if (report) fail(c, "unknown variable '" + c.args[0] + "'");
        return false;
    }
    const std::string& op = c.args[1];
    const double rhs = std::atof(c.args[2].c_str());
    bool ok = false;
    if (op == "<") ok = *v < rhs;
    else if (op == "<=") ok = *v <= rhs;
    else if (op == ">") ok = *v > rhs;
    else if (op == ">=") ok = *v >= rhs;
    else if (op == "==") ok = std::abs(*v - rhs) <= 1e-3 * std::max(1.0, std::abs(rhs));
    else if (op == "!=") ok = std::abs(*v - rhs) > 1e-3 * std::max(1.0, std::abs(rhs));
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s %s %s (actual %.4g)", c.args[0].c_str(), op.c_str(), c.args[2].c_str(), *v);
    if (detail) *detail = buf;
    return ok;
}

bool Scenario::update(double now, const std::function<bool()>& inputBusy, const ValueFn& value, const ExecFn& exec) {
    while (m_pc < m_cmds.size()) {
        if (inputBusy()) return true;
        const ScenarioCommand& c = m_cmds[m_pc];
        if (c.op == "wait") {
            if (m_resumeAt < 0.0) m_resumeAt = now + (c.args.empty() ? 0.0 : std::atof(c.args[0].c_str()));
            if (now < m_resumeAt) return true;
            m_resumeAt = -1.0;
        } else if (c.op == "until") {
            if (m_deadline < 0.0) m_deadline = now + (c.args.size() > 3 ? std::atof(c.args[3].c_str()) : 10.0);
            std::string detail;
            const bool ok = compare(c, value, false, &detail);
            if (!ok && now < m_deadline) return true;
            m_deadline = -1.0;
            if (ok) {
                ++m_passed;
                std::printf("SCENARIO ok   line %d: until %s\n", c.line, detail.c_str());
            } else {
                fail(c, "timeout: " + detail);
            }
        } else if (c.op == "expect") {
            std::string detail;
            if (compare(c, value, true, &detail)) {
                ++m_passed;
                std::printf("SCENARIO ok   line %d: %s\n", c.line, detail.c_str());
            } else if (!detail.empty()) {
                fail(c, detail);
            }
        } else if (c.op == "mark") {
            const auto v = c.args.empty() ? std::nullopt : value(c.args[0]);
            if (v) m_marks[c.args[0]] = *v;
            else fail(c, "mark: unknown variable");
        } else if (c.op == "quit") {
            ++m_pc;
            return false;
        } else {
            const Exec r = exec(c);
            if (r == Exec::Busy) return true;
            if (r == Exec::Failed) fail(c, "command '" + c.op + "' failed");
        }
        ++m_pc;
    }
    return false;
}

} // namespace tf
