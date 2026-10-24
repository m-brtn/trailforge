#pragma once

struct SDL_Window;

namespace tf {

// Sets the embedded Icon.png (see cmake/AppIcon.cmake) as the window icon; on macOS this is the Dock icon.
// Returns false when no icon is embedded on this platform or SDL refuses it.
bool setAppIcon(SDL_Window* window);

} // namespace tf
