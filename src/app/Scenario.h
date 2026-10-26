#pragma once

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace tf {

// Application verification scenario: a sequence of commands from a text file.
// Input is fed as real SDL events (SDL_PushEvent) and takes the same path
// as the live keyboard/mouse: ImGui -> Input -> Simulation on physics ticks.
//
// Format: one command per line, '#' starts a comment.
//   wait S                       wait S seconds (application frame time)
//   until VAR OP VALUE TIMEOUT   wait for a condition; on timeout FAIL and continue
//   expect VAR OP VALUE          check (OP: < <= > >= == !=; == with 1e-3 tolerance)
//   down KEY | up KEY | tap KEY  key (SDL name: W, Space, Escape, F1, Return, Left Ctrl...)
//   text STRING                  SDL_EVENT_TEXT_INPUT (typing into an ImGui field)
//   click ITEM                   left click on a DebugUI widget (vehicle_header, rebuild...)
//   inputclick ITEM              Cmd+click (macOS; Ctrl+click on other OSes): slider -> number input field
//   scrollto ITEM                scroll the Tuning window with the wheel until the widget is visible
//   teleport INDEX               same as the teleport button in the UI
//   autopilot drive|off          the autopilot drives the world's attract route while driving (lap checks)
//   reset                        same as R (on a circuit: back to the last gate of the running lap)
//   resize W H                   SDL_SetWindowSize (points)
//   capture PATH                 save a frame (BMP/PNG by extension)
//   mark VAR                     remember a value; then d_VAR = current - remembered
//   orbit DX DY                  RMB + mouse motion at the window center (camera orbit)
//   wheel DY                     mouse wheel at the window center (camera zoom)
//   reset_stats                  reset accumulated min/max (camera clearance, wheel lift)
//   log TEXT                     print a telemetry line
//   quit                         finish (exit code 1 if there were FAILs)
struct ScenarioCommand {
    int line = 0;
    std::string op;
    std::vector<std::string> args;
};

class Scenario {
public:
    // Returns nullopt and writes an error to stderr if the file cannot be read or has unknown commands.
    static std::optional<Scenario> load(const std::string& path);

    using ValueFn = std::function<std::optional<double>(const std::string&)>;
    enum class Exec { Done, Busy, Failed }; // Busy: retry on the next frame
    using ExecFn = std::function<Exec(const ScenarioCommand&)>;

    // Execute everything that is ready at time now. inputBusy: synthetic events have not yet
    // been processed by the application (then the scenario waits a frame). Returns false when the scenario is finished.
    bool update(double now, const std::function<bool()>& inputBusy, const ValueFn& value, const ExecFn& exec);

    int passed() const { return m_passed; }
    int failed() const { return m_failed; }
    void fail(const ScenarioCommand& c, const std::string& what);

private:
    bool compare(const ScenarioCommand& c, const ValueFn& value, bool report, std::string* detail);

    std::string m_path;
    std::vector<ScenarioCommand> m_cmds;
    std::map<std::string, double> m_marks;
    std::size_t m_pc = 0;
    double m_resumeAt = -1.0;  // wait
    double m_deadline = -1.0;  // until
    int m_passed = 0, m_failed = 0;
};

} // namespace tf
