#include "app/AppIcon.h"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#if defined(TRAILFORGE_HAS_ICON)
extern const std::uint8_t g_app_icon_bmp[];
extern const std::size_t g_app_icon_bmp_size;
#endif

namespace tf {

#if defined(TRAILFORGE_HAS_ICON)
namespace {

// macOS icon grid: the artwork is a rounded square of 824/1024 of the canvas with a 185.4/1024 corner radius,
// the rest is transparent. Applied as alpha with a one-pixel antialiased edge.
void applyRoundedMask(SDL_Surface* s) {
    const float size = float(std::min(s->w, s->h));
    const float half = size * (824.0f / 1024.0f) * 0.5f;
    const float radius = size * (185.4f / 1024.0f);
    const float cx = s->w * 0.5f, cy = s->h * 0.5f;
    for (int y = 0; y < s->h; ++y) {
        auto* row = static_cast<std::uint8_t*>(s->pixels) + std::ptrdiff_t(y) * s->pitch;
        for (int x = 0; x < s->w; ++x) {
            const float px = std::fabs(x + 0.5f - cx) - (half - radius);
            const float py = std::fabs(y + 0.5f - cy) - (half - radius);
            const float qx = std::max(px, 0.0f), qy = std::max(py, 0.0f);
            const float dist = std::sqrt(qx * qx + qy * qy) + std::min(std::max(px, py), 0.0f) - radius;
            const float coverage = std::clamp(0.5f - dist, 0.0f, 1.0f);
            row[x * 4 + 3] = std::uint8_t(row[x * 4 + 3] * coverage + 0.5f);
        }
    }
}

} // namespace
#endif

bool setAppIcon(SDL_Window* window) {
#if defined(TRAILFORGE_HAS_ICON)
    SDL_IOStream* io = SDL_IOFromConstMem(g_app_icon_bmp, g_app_icon_bmp_size);
    if (!io) return false;
    SDL_Surface* bmp = SDL_LoadBMP_IO(io, true);
    if (!bmp) {
        SDL_Log("App icon: %s", SDL_GetError());
        return false;
    }
    SDL_Surface* rgba = SDL_ConvertSurface(bmp, SDL_PIXELFORMAT_RGBA32);
    SDL_DestroySurface(bmp);
    if (!rgba) return false;
    applyRoundedMask(rgba);
    const bool ok = SDL_SetWindowIcon(window, rgba);
    if (!ok) SDL_Log("SDL_SetWindowIcon: %s", SDL_GetError());
    SDL_DestroySurface(rgba);
    return ok;
#else
    (void)window;
    return false;
#endif
}

} // namespace tf
