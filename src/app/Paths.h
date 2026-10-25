#pragma once

#include <string>
#include <vector>

namespace tf {

// Directories that may hold assets/<subdir>, in lookup order. SDL_GetBasePath() is Contents/Resources/ inside
// TrailForge.app and the executable's directory elsewhere (Windows/Linux packages ship assets/ next to the
// executable). Development builds also look in the source tree.
std::vector<std::string> assetDirs(const char* subdir);

// Per-user writable directory (SDL_GetPrefPath: ~/Library/Application Support/TrailForge on macOS,
// %APPDATA%\TrailForge on Windows, ~/.local/share/TrailForge on Linux). Created on first use; empty if unavailable.
// Nothing is stored there yet; it is the place for future settings and saves.
std::string userDataDir();

} // namespace tf
