#include "app/Application.h"
#include "app/Paths.h"

#include <SDL3/SDL_main.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>

namespace {

void printUsage() {
    std::printf("Usage: trailforge [--quit-after <seconds>] [--demo] [--no-ui] [--no-intro] [--capture <file.bmp>] [--capture-at <seconds>]\n"
                "                  [--script <file>] [--vehicle <id>] [--no-audio] [--audio-capture <file.wav>] [--paths] [--version]\n"
                "  --quit-after N  exit automatically after N seconds (smoke test)\n"
                "  --demo          scripted throttle/steering without user input\n"
                "  --no-ui         start with the F1 diagnostics hidden\n"
                "  --no-intro      skip the intro and open the start menu directly\n"
                "  --capture F     save one rendered frame to BMP file F (default at 3 s)\n"
                "  --capture-at N  time of the capture in seconds\n"
                "  --script F      run a test scenario (input via SDL events), exit code 1 on failures; starts in\n"
                "                  gameplay without the intro (`menu ...` commands open the menus)\n"
                "  --vehicle ID    start with catalog vehicle ID (assets/vehicles/ID), e.g. trailforge_ev; remembered\n"
                "                  like a choice on the vehicle page (not in scenario mode)\n"
                "  --no-audio      do not open an audio device or load sounds\n"
                "  --audio-capture F  render the mix offline to WAV file F (no device; one physics tick of\n"
                "                  audio per tick) and the mix parameters to F.csv\n"
                "  --paths         print the asset lookup directories and the user data directory, then exit\n"
                "  --version       print the version and exit\n");
}

} // namespace

int main(int argc, char** argv) {
    tf::AppOptions options;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--quit-after") == 0 && i + 1 < argc) options.quitAfterSeconds = std::atof(argv[++i]);
        else if (std::strcmp(argv[i], "--capture") == 0 && i + 1 < argc) options.capturePath = argv[++i];
        else if (std::strcmp(argv[i], "--capture-at") == 0 && i + 1 < argc) options.captureAtSeconds = std::atof(argv[++i]);
        else if (std::strcmp(argv[i], "--script") == 0 && i + 1 < argc) options.scriptPath = argv[++i];
        else if (std::strcmp(argv[i], "--vehicle") == 0 && i + 1 < argc) options.vehicleId = argv[++i];
        else if (std::strcmp(argv[i], "--demo") == 0) options.demoDrive = true;
        else if (std::strcmp(argv[i], "--no-ui") == 0) options.startWithUi = false;
        else if (std::strcmp(argv[i], "--no-intro") == 0) options.intro = false;
        else if (std::strcmp(argv[i], "--no-audio") == 0) options.audio = false;
        else if (std::strcmp(argv[i], "--audio-capture") == 0 && i + 1 < argc) options.audioCapturePath = argv[++i];
        else if (std::strcmp(argv[i], "--version") == 0) { std::printf("TrailForge %s\n", TRAILFORGE_VERSION); return 0; }
        else if (std::strcmp(argv[i], "--paths") == 0) {
            for (const std::string& d : tf::assetDirs("audio")) std::printf("assets: %s\n", d.c_str());
            for (const char* sub : {"music", "fonts", "images", "vehicles"})
                for (const std::string& d : tf::assetDirs(sub)) std::printf("%s: %s\n", sub, d.c_str());
            std::printf("user data: %s\n", tf::userDataDir().c_str());
            return 0;
        }
        else if (std::strncmp(argv[i], "-psn_", 5) == 0) continue; // process serial number from older Finder launches
        else { printUsage(); return std::strcmp(argv[i], "--help") == 0 ? 0 : 2; }
    }
    try {
        tf::Application app(options);
        return app.run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "TrailForge fatal error: %s\n", e.what());
        return 1;
    }
}
