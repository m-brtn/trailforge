#pragma once

#include "app/GameFlow.h"
#include "app/GameSettings.h"

#include <array>
#include <string>
#include <vector>

struct ImFont;
struct SDL_GPUTexture;

namespace tf {

class Renderer;

// What the menus show besides the flow state.
struct MenuView {
    bool gamepad = false;         // button hints for a gamepad instead of keys
    const char* version = "";
    float pauseBlur = 0.0f;       // 0..1, already applied to the 3D frame by the renderer
};

// Draws the intro, the start menu, the pause menu, Settings and Controls with ImGui's draw lists (behind every
// ImGui window, so the F1-F3 diagnostics stay on top). Owns the menu fonts and the logo texture.
class MenuUI {
public:
    MenuUI() = default;
    ~MenuUI();
    MenuUI(const MenuUI&) = delete;
    MenuUI& operator=(const MenuUI&) = delete;

    // After the ImGui context exists and before the first NewFrame. Missing files are logged; the menus then fall
    // back to ImGui's default font / no logo.
    void load(Renderer& renderer, const std::vector<std::string>& fontDirs, const std::vector<std::string>& imageDirs);
    void release();

    void draw(const GameFlow& flow, const GameSettings& settings, const MenuView& view);

    // Row under a point (ImGui coordinates) of the menu drawn last; dir: -1 / +1 on a value arrow, else 0.
    int hitTest(float x, float y, int* dir) const;
    // Screen rectangle (ImGui coordinates) of a row drawn in the last frame; false if it is not on screen.
    bool itemRect(MenuItem item, float& x0, float& y0, float& x1, float& y1) const;
    bool fontsLoaded() const { return m_body != nullptr; }
    bool logoLoaded() const { return m_logo != nullptr; }

private:
    struct Row {
        MenuItem item;
        float x0, y0, x1, y1;
        float arrowL0 = 0, arrowL1 = 0, arrowR0 = 0, arrowR1 = 0; // x ranges of the value arrows (0 = none)
    };
    void drawLogo(float cx, float cy, float width, float alpha);
    void drawMenu(const GameFlow& flow, const GameSettings& settings, const MenuView& view, float alpha);
    void drawControls(float x, float y, float w, float rowH, float alpha);
    void drawHints(const GameFlow& flow, const MenuView& view, float alpha);

    Renderer* m_renderer = nullptr;
    ImFont* m_body = nullptr;    // Rajdhani Medium
    ImFont* m_heading = nullptr; // Rajdhani SemiBold
    SDL_GPUTexture* m_logo = nullptr;
    float m_logoAspect = 3.0f;          // content width / height
    std::array<float, 4> m_logoUv{0, 0, 1, 1}; // content box in the image (transparent margins cut away)
    std::vector<Row> m_rows;
    float m_selectAnim = 0.0f;          // highlight bar position (row index, eased)
    int m_lastPage = -1;
};

} // namespace tf
