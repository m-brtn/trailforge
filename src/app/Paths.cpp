#include "app/Paths.h"

#include <SDL3/SDL_filesystem.h>
#include <SDL3/SDL_stdinc.h>

namespace tf {

std::vector<std::string> assetDirs(const char* subdir) {
    std::vector<std::string> dirs;
    const std::string rel = std::string("assets/") + subdir;
    if (const char* base = SDL_GetBasePath()) {
        dirs.push_back(std::string(base) + rel);
#if !defined(TRAILFORGE_PACKAGED)
        dirs.push_back(std::string(base) + "../../" + rel); // build/<preset>/trailforge
#endif
    }
#if !defined(TRAILFORGE_PACKAGED)
    dirs.push_back(std::string(TRAILFORGE_SOURCE_DIR) + "/" + rel);
#endif
    return dirs;
}

std::string userDataDir() {
    char* path = SDL_GetPrefPath(nullptr, "TrailForge");
    if (!path) return {};
    std::string out = path;
    SDL_free(path);
    return out;
}

} // namespace tf
