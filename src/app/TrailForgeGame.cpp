// TrailForge on the engine's app shell: quench_main owns main (SDL main callbacks), the window, the GPU device, ImGui,
// the frame loop, the scenario runner and captures; this file connects them to tf::Application.

#include "app/Application.h"
#include "app/Paths.h"
#include "sim/Simulation.h"

#include <quench/app/App.h>
#include <quench/app/Scenario.h>

#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace {

namespace qapp = quench::app;

void printUsage() {
    std::printf("TrailForge options:\n"
                "  --demo             scripted throttle/steering without user input\n"
                "  --no-ui            start with the F1 diagnostics hidden\n"
                "  --no-intro         skip the intro and open the start menu directly\n"
                "  --vehicle ID       start with catalog vehicle ID (assets/vehicles/ID), e.g. trailforge_ev; remembered\n"
                "                     like a choice on the vehicle page (not in scenario mode)\n"
                "  --level ID         start on level ID: exploration_peaks (assets/levels/ID) or proving_ground;\n"
                "                     remembered (scenarios and --demo start on the proving ground unless --level is given)\n"
                "  --no-audio         do not open an audio device or load sounds\n"
                "  --audio-capture F  render the mix offline to WAV file F (no device; one physics tick of audio per\n"
                "                     tick) and the mix parameters to F.csv\n"
                "  --paths            print the asset lookup directories and the user data directory, then exit\n"
                "A --script scenario starts in gameplay without the intro (`menu ...` commands open the menus).\n");
}

void printPaths() {
    for (const std::string& d : tf::assetDirs("audio")) std::printf("assets: %s\n", d.c_str());
    for (const char* sub : {"music", "fonts", "images", "vehicles"})
        for (const std::string& d : tf::assetDirs(sub)) std::printf("%s: %s\n", sub, d.c_str());
    std::printf("user data: %s\n", tf::userDataDir().c_str());
}

// No exception crosses the SDL callbacks: a failure in a frame ends the app with exit code 1.
class TrailForgeGame final : public qapp::Game {
public:
    explicit TrailForgeGame(const tf::AppOptions& options) : m_options(options) {}

    quench::Result<void> init(qapp::App& app) override {
        try {
            m_game = std::make_unique<tf::Application>(app, m_options);
        } catch (const std::exception& e) {
            return quench::Error{std::string("TrailForge: ") + e.what()};
        }
        return {};
    }

    void update(qapp::App& app, double frameDt) override {
        guard(app, [&] { m_game->update(frameDt); });
    }
    void ui(qapp::App& app) override {
        guard(app, [&] { m_game->ui(); });
    }
    void render(qapp::App& app, qapp::Frame& frame) override {
        guard(app, [&] { m_game->render(frame); });
    }
    void event(qapp::App& app, const SDL_Event& event) override {
        guard(app, [&] { m_game->event(event); });
    }

    // Before the engine destroys the GPU device: every GPU resource of the game goes here.
    void shutdown(qapp::App&) override {
        if (m_game) m_game->printExitSummary();
        m_game.reset();
    }

    std::optional<double> scenarioValue(std::string_view name) const override {
        if (!m_game || m_failed) return std::nullopt;
        try {
            return m_game->scenarioValue(std::string(name));
        } catch (const std::exception& e) {
            std::fprintf(stderr, "TrailForge: scenario value %.*s: %s\n", int(name.size()), name.data(), e.what());
            return std::nullopt;
        }
    }

    qapp::ScenarioExec scenarioCommand(qapp::App&, const qapp::ScenarioCommand& command) override {
        if (!m_game || m_failed) return qapp::ScenarioExec::Failed;
        try {
            return m_game->scenarioCommand(command);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "TrailForge: scenario line %d: %s\n", command.line, e.what());
            return qapp::ScenarioExec::Failed;
        }
    }

    std::string perfDetails(qapp::App&) override { return m_game && !m_failed ? m_game->perfDetails() : std::string(); }

private:
    template <typename F> void guard(qapp::App& app, F&& f) {
        if (!m_game || m_failed) return;
        try {
            f();
        } catch (const std::exception& e) {
            std::fprintf(stderr, "TrailForge fatal error: %s\n", e.what());
            m_failed = true;
            app.quit(1);
        }
    }

    tf::AppOptions m_options;
    std::unique_ptr<tf::Application> m_game;
    bool m_failed = false;
};

} // namespace

namespace quench::app {

Result<std::unique_ptr<Game>> createGame(AppDesc& desc, std::span<const std::string_view> args) {
    desc.name = "TrailForge";
    desc.organization = ""; // the user directory stays SDL_GetPrefPath(nullptr, "TrailForge")
    desc.version = TRAILFORGE_VERSION;
    desc.width = 1440;
    desc.height = 900;
    desc.fixedStep = tf::SimulationSettings{}.fixedStep;

    for (const std::string_view a : args) {
        if (a == "--help") {
            printUsage();
            return std::unique_ptr<Game>();
        }
    }
    tf::AppOptions options;
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string_view a = args[i];
        const auto value = [&](std::string& out) {
            if (i + 1 >= args.size()) return false;
            out = std::string(args[++i]);
            return true;
        };
        bool ok = true;
        if (a == "--demo") options.demoDrive = true;
        else if (a == "--no-ui") options.startWithUi = false;
        else if (a == "--no-intro") options.intro = false;
        else if (a == "--no-audio") options.audio = false;
        else if (a == "--vehicle") ok = value(options.vehicleId);
        else if (a == "--level") ok = value(options.levelId);
        else if (a == "--audio-capture") ok = value(options.audioCapturePath);
        else if (a == "--paths") {
            printPaths();
            return std::unique_ptr<Game>();
        } else if (a.starts_with("-psn_")) {
            continue; // process serial number from older Finder launches
        } else {
            return Error{"unknown option '" + std::string(a) + "' (--help lists the options)"};
        }
        if (!ok) return Error{std::string(a) + " needs a value"};
    }
    return std::unique_ptr<Game>(std::make_unique<TrailForgeGame>(options));
}

} // namespace quench::app
