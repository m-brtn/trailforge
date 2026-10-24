#include "app/Application.h"

#include <SDL3/SDL_main.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>

namespace {

void printUsage() {
    std::printf("Usage: trailforge [--quit-after <seconds>] [--demo] [--no-ui] [--capture <file.bmp>] [--capture-at <seconds>] [--script <file>]\n"
                "  --quit-after N  exit automatically after N seconds (smoke test)\n"
                "  --demo          scripted throttle/steering without user input\n"
                "  --no-ui         start with the F1 diagnostics hidden\n"
                "  --capture F     save one rendered frame to BMP file F (default at 3 s)\n"
                "  --capture-at N  time of the capture in seconds\n"
                "  --script F      run a test scenario (input via SDL events), exit code 1 on failures\n");
}

} // namespace

int main(int argc, char** argv) {
    tf::AppOptions options;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--quit-after") == 0 && i + 1 < argc) options.quitAfterSeconds = std::atof(argv[++i]);
        else if (std::strcmp(argv[i], "--capture") == 0 && i + 1 < argc) options.capturePath = argv[++i];
        else if (std::strcmp(argv[i], "--capture-at") == 0 && i + 1 < argc) options.captureAtSeconds = std::atof(argv[++i]);
        else if (std::strcmp(argv[i], "--script") == 0 && i + 1 < argc) options.scriptPath = argv[++i];
        else if (std::strcmp(argv[i], "--demo") == 0) options.demoDrive = true;
        else if (std::strcmp(argv[i], "--no-ui") == 0) options.startWithUi = false;
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
