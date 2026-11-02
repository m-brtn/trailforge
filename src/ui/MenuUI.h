#pragma once

#include "app/GameFlow.h"
#include "app/Garage.h"
#include "app/GameSettings.h"
#include "input/ControlLayout.h"

#include <array>
#include <string>
#include <vector>

struct ImDrawList;
struct ImFont;
struct SDL_GPUTexture;

namespace tf {

class Renderer;
struct VehicleTelemetry;

// One vehicle of the vehicle page (catalog order).
struct VehicleCard {
    std::string id, name, description;
    std::vector<VehicleSpec> specs;
    bool electric = false;
};

// What the menus show besides the flow state.
struct MenuView {
    bool gamepad = false;         // button hints for a gamepad instead of keys
    const char* version = "";
    float pauseBlur = 0.0f;       // 0..1, already applied to the 3D frame by the renderer
    DriveKeys driveKeys = DriveKeys::Wasd;
    KeyboardStyle keyboard = KeyboardStyle::Pc; // key cap wording of the running platform
    PadType padType = PadType::None;            // connected gamepad
    PadGlyphs padGlyphs = PadGlyphs::Xbox;      // resolved from the setting and padType
    const std::vector<VehicleCard>* vehicles = nullptr; // vehicle page
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
    // Garage screen: setup panel on the left (scrolls with the cursor), live numbers bottom right.
    void drawGarage(const Garage& garage, const VehicleTelemetry& telemetry, const MenuView& view);
    // Garage row under a point; dir: -1 / +1 on a value arrow; fraction: 0..1 on a slider bar, else -1.
    int garageHitTest(float x, float y, int* dir, float* fraction) const;
    bool garageRowRect(int row, float& x0, float& y0, float& x1, float& y1) const;

    // Modal dialog (drawn over everything, screen dimmed). Reused by the garage for rename, delete and reset.
    struct ModalSpec {
        const char* title = "";
        std::string message;
        bool textField = false;
        std::string text;
        bool selected = false;    // the whole text is selected (typing replaces it)
        std::string error;
        std::string hint;
        const char* confirm = "CONFIRM";
        int focus = 0;            // 0 confirm, 1 cancel
    };
    void drawModal(const ModalSpec& spec);
    // Modal button under a point of the last drawn modal: 0 confirm, 1 cancel, -1 none.
    int modalHitTest(float x, float y) const;
    bool modalButtonRect(int button, float& x0, float& y0, float& x1, float& y1) const;

    // Row under a point (ImGui coordinates) of the menu drawn last; dir: -1 / +1 on a value arrow, else 0.
    int hitTest(float x, float y, int* dir) const;
    // Screen rectangle (ImGui coordinates) of a row drawn in the last frame; false if it is not on screen.
    bool itemRect(MenuItem item, float& x0, float& y0, float& x1, float& y1) const;
    // Vehicle page: card under a point (catalog index) or -1; rectangle of a card drawn in the last frame.
    int vehicleCardHit(float x, float y) const;
    bool vehicleCardRect(int index, float& x0, float& y0, float& x1, float& y1) const;
    int vehicleCardCount() const { return int(m_cards.size()); }
    // Mouse wheel / trackpad steps (+ = up) for the list drawn next (menu page or garage). Keyboard / gamepad cursor
    // moves still scroll the selected row into view; a cursor set by the pointer (notePointerCursor) does not.
    void scroll(float steps) { m_wheel += steps; }
    // Finger drag (points, + = down) for the list drawn next; held: the finger is still down. The list follows the
    // finger and coasts briefly after it lifts.
    void drag(float dy, bool held) {
        m_dragPx += dy;
        m_dragHeld = held;
    }
    void notePointerCursor() { m_pointerCursor = true; }
    float menuScroll() const { return m_menuScrollTarget; }   // px, list offset of the menu page
    float garageScroll() const { return m_garageScrollTarget; } // px, list offset of the garage
    bool fontsLoaded() const { return m_body != nullptr; }
    ImFont* bodyFont() const { return m_body; }
    ImFont* headingFont() const { return m_heading; }
    // Controls page of the last frame: callouts drawn and overlapping pairs (callout / callout or callout / key cap /
    // controller outline), measured with the real font.
    int controlsCallouts() const { return m_controlsCallouts; }
    int controlsOverlaps() const { return m_controlsOverlaps; }
    bool logoLoaded() const { return m_logo != nullptr; }

private:
    struct Row {
        MenuItem item;
        int index; // position on the page (rows scrolled out of view are not in m_rows)
        float x0, y0, x1, y1;
        float arrowL0 = 0, arrowL1 = 0, arrowR0 = 0, arrowR1 = 0; // x ranges of the value arrows (0 = none)
    };
    void drawLogo(float cx, float cy, float width, float alpha);
    void drawMenu(const GameFlow& flow, const GameSettings& settings, const MenuView& view, float alpha);
    struct Box {
        float x0, y0, x1, y1;
    };
    void drawBackButton(ImDrawList* dl, float x0, float titleRuleY, bool selected, float alpha);
    float backButtonHeight() const;
    Box m_backBox{};
    // Footer: version bottom left, key cap hints bottom right; the lists end footerHeight() above the bottom.
    struct Hint {
        std::string key, label;
    };
    void drawFooter(const std::vector<Hint>& hints, const char* version, float alpha);
    float footerHeight() const;
    // Controls page diagrams; both add their callout boxes and obstacles (key caps, controller outline) for the
    // overlap count.
    void drawKeyboard(ImDrawList* dl, float x, float y, float u, const MenuView& view, float alpha);
    void drawGamepad(ImDrawList* dl, float cx, float y, float g, const MenuView& view, float alpha);
    float labelSize(float u) const;
    // Vehicle page block under the VEHICLE row: one card per vehicle, then the selected vehicle's description and specs.
    float vehicleBlockHeight(const MenuView& view, float w) const;
    void drawVehicleBlock(ImDrawList* dl, float x, float y, float w, int selected, const MenuView& view, float alpha);
    // Measures (dl == nullptr) or draws the block and records the card boxes; returns its height.
    float vehicleBlock(ImDrawList* dl, float x, float y, float w, int selected, const MenuView& view, float alpha,
                       std::vector<Box>* cardBoxes) const;
    void drawHints(const GameFlow& flow, const MenuView& view, float alpha);
    void updateScroll(float& scroll, float& target, float cursorY, float cursorH, float rowH, float contentH,
                      float viewH, bool follow);
    void drawScrollMarks(float x0, float x1, float top, float bottom, float scroll, float maxScroll, float alpha);

    Renderer* m_renderer = nullptr;
    ImFont* m_body = nullptr;    // Rajdhani Medium
    ImFont* m_heading = nullptr; // Rajdhani SemiBold
    SDL_GPUTexture* m_logo = nullptr;
    float m_logoAspect = 3.0f;          // content width / height
    std::array<float, 4> m_logoUv{0, 0, 1, 1}; // content box in the image (transparent margins cut away)
    std::vector<Row> m_rows;
    struct GarageHit {
        int row;
        float x0, y0, x1, y1;
        float arrowL0 = 0, arrowL1 = 0, arrowR0 = 0, arrowR1 = 0;
        float bar0 = 0, bar1 = 0; // slider bar x range (0 = none)
    };
    std::vector<GarageHit> m_garageRows;
    std::vector<std::array<float, 4>> m_modalButtons; // x0, y0, x1, y1 of the last drawn modal's buttons
    float m_garageScroll = 0.0f;      // eased list offset (px)
    float m_garageScrollTarget = 0.0f;
    float m_garageSelectAnim = 0.0f;  // eased highlight position (px)
    int m_garageMsgSerial = 0;
    double m_garageMsgTime = -100.0;  // ImGui time when the shown message appeared
    int m_garageLastCursor = -1;
    float m_selectAnim = 0.0f;          // highlight bar position (row index, eased)
    int m_lastPage = -1;
    int m_lastCursor = -1;
    float m_menuScroll = 0.0f, m_menuScrollTarget = 0.0f; // menu page list offset (px), eased / target
    float m_wheel = 0.0f;               // pending wheel steps
    float m_dragPx = 0.0f;              // pending finger drag (px)
    bool m_dragHeld = false, m_wasDragging = false;
    float m_dragVelocity = 0.0f;        // px/s, smoothed while the finger moves
    float m_fling = 0.0f;               // px/s after the finger lifted, decaying
    bool m_pointerCursor = false;       // the cursor of this frame was set by the mouse
    std::vector<Box> m_callouts, m_obstacles;
    std::vector<Box> m_cards; // vehicle page cards of the last frame (catalog order)
    int m_controlsCallouts = 0, m_controlsOverlaps = 0;
};

} // namespace tf
