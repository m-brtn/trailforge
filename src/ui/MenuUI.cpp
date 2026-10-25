#include "ui/MenuUI.h"

#include "render/Renderer.h"
#include "vehicle/Vehicle.h"

#include <SDL3/SDL_surface.h>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>

namespace tf {
namespace {

// Palette from the key art: warm white of TRAIL, orange of FORGE, near-black.
constexpr ImVec4 kWhite(0.97f, 0.94f, 0.89f, 1.0f);
constexpr ImVec4 kOrange(0.99f, 0.42f, 0.04f, 1.0f);
constexpr ImVec4 kDark(0.03f, 0.03f, 0.035f, 1.0f);

ImU32 col(const ImVec4& c, float alpha) { return ImGui::ColorConvertFloat4ToU32(ImVec4(c.x, c.y, c.z, c.w * alpha)); }

float smooth(float e0, float e1, float x) {
    const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float easeInOut(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t < 0.5f ? 4.0f * t * t * t : 1.0f - std::pow(-2.0f * t + 2.0f, 3.0f) * 0.5f;
}

ImFont* orDefault(ImFont* f) { return f ? f : ImGui::GetFont(); }

// Text with extra letter spacing (tracking, in ems): spaced uppercase headings.
float textWidth(ImFont* font, float size, const char* s, float tracking = 0.0f) {
    font = orDefault(font);
    if (tracking == 0.0f) return font->CalcTextSizeA(size, FLT_MAX, 0.0f, s).x;
    float w = 0.0f;
    int n = 0;
    for (const char* p = s; *p; ++p, ++n) w += font->CalcTextSizeA(size, FLT_MAX, 0.0f, p, p + 1).x;
    return w + tracking * size * float(std::max(0, n - 1));
}

void text(ImDrawList* dl, ImFont* font, float size, ImVec2 pos, ImU32 c, const char* s, float tracking = 0.0f) {
    font = orDefault(font);
    if (tracking == 0.0f) {
        dl->AddText(font, size, pos, c, s);
        return;
    }
    for (const char* p = s; *p; ++p) {
        dl->AddText(font, size, pos, c, p, p + 1);
        pos.x += font->CalcTextSizeA(size, FLT_MAX, 0.0f, p, p + 1).x + tracking * size;
    }
}

std::string findFile(const std::vector<std::string>& dirs, const char* name) {
    for (const std::string& d : dirs) {
        std::error_code ec;
        const std::filesystem::path p = std::filesystem::path(d) / name;
        if (std::filesystem::exists(p, ec)) return p.string();
    }
    return {};
}

// ------------------------------------------------------------------ Controls page geometry
// Keyboard diagram, in units of u (key cap pitch): the keys keep their ANSI positions, spread horizontally a little
// so that the callouts under them have room.
constexpr float kPitchX = 1.1f;    // horizontal key pitch / u
constexpr float kCap = 0.96f;      // key cap height / u
constexpr float kKbPitches = 21.6f; // diagram width in horizontal pitches (keys, arrows, mouse with its callout)
constexpr float kMouseX = 20.1f;   // mouse centre, in horizontal pitches

float kbRowPitch(float u, float lf) { return u * (kCap + 0.30f) + 2.0f * 1.12f * lf; }
float kbHeight(float u, float lf) { return 6.0f * kbRowPitch(u, lf); }
// Gamepad diagram, in units of g, origin at the top centre of the body: callouts above (to y0 - 1.3 g), triggers,
// body down to 4.75 g, callouts below.
constexpr float kPadTop = 2.3f;     // block top to the body origin, in g
float padHeight(float g, float lf) { return g * (kPadTop + 4.95f) + 2.0f * 1.12f * lf + 0.3f * g; }

template <typename B>
bool overlaps(const B& a, const B& b) {
    return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
}

void arrowGlyph(ImDrawList* dl, ImVec2 c, ImVec2 dir, float s, ImU32 colour, float stroke) {
    const ImVec2 tip(c.x + dir.x * s, c.y + dir.y * s), tail(c.x - dir.x * s, c.y - dir.y * s);
    const ImVec2 n(-dir.y, dir.x);
    const float h = 0.55f * s;
    dl->AddLine(tail, tip, colour, stroke);
    dl->AddLine(tip, ImVec2(tip.x - dir.x * h + n.x * h, tip.y - dir.y * h + n.y * h), colour, stroke);
    dl->AddLine(tip, ImVec2(tip.x - dir.x * h - n.x * h, tip.y - dir.y * h - n.y * h), colour, stroke);
}

ImVec2 keyArrowDir(Key k) {
    switch (k) {
    case Key::Up: return {0, -1};
    case Key::Down: return {0, 1};
    case Key::Left: return {-1, 0};
    case Key::Right: return {1, 0};
    default: return {0, 0};
    }
}

// Gamepad control positions (g units): Xbox has the left stick above the D-pad, PlayStation the other way round,
// with a touchpad in the middle and Create / Options beside it.
struct PadLayout {
    ImVec2 leftStick, rightStick, dpad, face, back, start, guide;
    bool touchpad;
};
PadLayout padLayout(PadGlyphs g) {
    if (g == PadGlyphs::PlayStation)
        return {{-1.15f, 2.2f}, {1.15f, 2.2f}, {-2.25f, 1.25f}, {2.25f, 1.25f}, {-1.3f, 0.5f}, {1.3f, 0.5f}, {0.0f, 2.05f}, true};
    return {{-2.25f, 1.25f}, {1.15f, 2.2f}, {-1.15f, 2.2f}, {2.25f, 1.25f}, {-0.6f, 1.05f}, {0.6f, 1.05f}, {0.0f, 0.5f}, false};
}
constexpr float kStickR = 0.52f, kFaceR = 0.23f, kFaceOff = 0.5f, kDpadArm = 0.48f, kDpadHalf = 0.16f;

ImVec2 faceOffset(PadControl c) {
    switch (c) {
    case PadControl::South: return {0.0f, kFaceOff};
    case PadControl::East: return {kFaceOff, 0.0f};
    case PadControl::West: return {-kFaceOff, 0.0f};
    case PadControl::North: return {0.0f, -kFaceOff};
    default: return {0, 0};
    }
}
bool isFace(PadControl c) {
    return c == PadControl::South || c == PadControl::East || c == PadControl::West || c == PadControl::North;
}

// Face button glyph: a ring with the Xbox letter or the PlayStation shape.
void faceGlyph(ImDrawList* dl, ImFont* font, ImVec2 c, float r, PadControl b, PadGlyphs g, ImU32 colour, float stroke) {
    dl->AddCircle(c, r, colour, 0, stroke);
    if (g == PadGlyphs::Xbox) {
        const char* s = padControlName(b, g);
        const float size = 1.55f * r;
        const float w = textWidth(font, size, s);
        text(dl, font, size, ImVec2(c.x - 0.5f * w, c.y - 0.52f * size), colour, s);
        return;
    }
    const float s = 0.46f * r;
    switch (b) {
    case PadControl::South:
        dl->AddLine(ImVec2(c.x - s, c.y - s), ImVec2(c.x + s, c.y + s), colour, stroke);
        dl->AddLine(ImVec2(c.x - s, c.y + s), ImVec2(c.x + s, c.y - s), colour, stroke);
        break;
    case PadControl::East: dl->AddCircle(c, 1.1f * s, colour, 0, stroke); break;
    case PadControl::West: dl->AddRect(ImVec2(c.x - s, c.y - s), ImVec2(c.x + s, c.y + s), colour, 0.0f, 0, stroke); break;
    case PadControl::North:
        dl->AddTriangle(ImVec2(c.x, c.y - 1.15f * s), ImVec2(c.x + 1.1f * s, c.y + 0.8f * s), ImVec2(c.x - 1.1f * s, c.y + 0.8f * s),
                        colour, stroke);
        break;
    default: break;
    }
}

} // namespace

MenuUI::~MenuUI() { release(); }

void MenuUI::release() {
    if (m_renderer && m_logo) m_renderer->releaseImage(m_logo);
    m_logo = nullptr;
}

void MenuUI::load(Renderer& renderer, const std::vector<std::string>& fontDirs,
                  const std::vector<std::string>& imageDirs) {
    m_renderer = &renderer;
    ImGuiIO& io = ImGui::GetIO();
    if (io.Fonts->Fonts.empty()) io.Fonts->AddFontDefault(); // the diagnostics keep ImGui's default font
    const std::string body = findFile(fontDirs, "Rajdhani-Medium.ttf");
    const std::string heading = findFile(fontDirs, "Rajdhani-SemiBold.ttf");
    if (!body.empty()) m_body = io.Fonts->AddFontFromFileTTF(body.c_str(), 32.0f);
    if (!heading.empty()) m_heading = io.Fonts->AddFontFromFileTTF(heading.c_str(), 32.0f);
    if (!m_heading) m_heading = m_body;
    std::printf("Menu: font %s\n", m_body ? body.c_str() : "not found (ImGui default font)");

    const std::string logo = findFile(imageDirs, "logo.png");
    SDL_Surface* src = logo.empty() ? nullptr : SDL_LoadPNG(logo.c_str());
    SDL_Surface* rgba = src ? SDL_ConvertSurface(src, SDL_PIXELFORMAT_RGBA32) : nullptr;
    if (src) SDL_DestroySurface(src);
    if (!rgba) {
        std::printf("Menu: logo %s\n", logo.empty() ? "not found" : SDL_GetError());
        return;
    }
    // Content box: the PNG has wide transparent margins.
    int x0 = rgba->w, y0 = rgba->h, x1 = -1, y1 = -1;
    for (int y = 0; y < rgba->h; ++y) {
        const auto* row = static_cast<const std::uint8_t*>(rgba->pixels) + std::size_t(y) * std::size_t(rgba->pitch);
        for (int x = 0; x < rgba->w; ++x) {
            if (row[x * 4 + 3] < 8) continue;
            x0 = std::min(x0, x);
            x1 = std::max(x1, x);
            y0 = std::min(y0, y);
            y1 = std::max(y1, y);
        }
    }
    if (x1 < x0) x0 = y0 = 0, x1 = rgba->w - 1, y1 = rgba->h - 1;
    std::vector<std::uint8_t> packed(std::size_t(rgba->w) * std::size_t(rgba->h) * 4);
    for (int y = 0; y < rgba->h; ++y)
        std::copy_n(static_cast<const std::uint8_t*>(rgba->pixels) + std::size_t(y) * std::size_t(rgba->pitch),
                    std::size_t(rgba->w) * 4, packed.data() + std::size_t(y) * std::size_t(rgba->w) * 4);
    m_logo = renderer.createImage(packed.data(), std::uint32_t(rgba->w), std::uint32_t(rgba->h));
    m_logoUv = {float(x0) / float(rgba->w), float(y0) / float(rgba->h), float(x1 + 1) / float(rgba->w),
                float(y1 + 1) / float(rgba->h)};
    m_logoAspect = float(x1 + 1 - x0) / float(y1 + 1 - y0);
    std::printf("Menu: logo %s (%dx%d, content %dx%d)\n", logo.c_str(), rgba->w, rgba->h, x1 + 1 - x0, y1 + 1 - y0);
    SDL_DestroySurface(rgba);
}

void MenuUI::drawLogo(float cx, float cy, float width, float alpha) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const float h = width / m_logoAspect;
    const ImVec2 a(cx - width * 0.5f, cy - h * 0.5f), b(cx + width * 0.5f, cy + h * 0.5f);
    if (m_logo) {
        dl->AddImage(ImTextureRef(ImTextureID(reinterpret_cast<std::intptr_t>(m_logo))), a, b,
                     ImVec2(m_logoUv[0], m_logoUv[1]), ImVec2(m_logoUv[2], m_logoUv[3]), col(ImVec4(1, 1, 1, 1), alpha));
        return;
    }
    // Text fallback in the logo colours.
    const float size = h * 0.9f;
    const float wt = textWidth(m_heading, size, "TRAIL", 0.04f), wf = textWidth(m_heading, size, "FORGE", 0.04f);
    const float x = cx - (wt + wf) * 0.5f;
    text(dl, m_heading, size, ImVec2(x, cy - size * 0.5f), col(kWhite, alpha), "TRAIL", 0.04f);
    text(dl, m_heading, size, ImVec2(x + wt, cy - size * 0.5f), col(kOrange, alpha), "FORGE", 0.04f);
}

void MenuUI::draw(const GameFlow& flow, const GameSettings& settings, const MenuView& view) {
    m_rows.clear();
    m_cards.clear();
    if (flow.screen() != Screen::Garage) {
        m_garageRows.clear();
        m_modalButtons.clear();
    }
    if (!flow.menuActive() || flow.screen() == Screen::Garage) {
        m_lastPage = -1;
        return;
    }
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const ImVec2 ds = ImGui::GetIO().DisplaySize;
    const float W = ds.x, H = ds.y;
    const float x0 = std::max(0.07f * W, 48.0f);
    const float menuLogoW = std::min(0.27f * W, 0.75f * H);
    const float menuLogoH = menuLogoW / m_logoAspect;
    const ImVec2 menuLogoC(x0 + menuLogoW * 0.5f, 0.08f * H + menuLogoH * 0.5f);

    if (flow.screen() == Screen::Intro || flow.menuFade() < 1.0f) {
        // Intro: logo over black; then the black lifts and the logo glides to its menu place.
        const float t = float(flow.introTime());
        const float e = easeInOut(flow.menuFade());
        if (flow.screen() == Screen::Intro || e < 1.0f)
            dl->AddRectFilled(ImVec2(0, 0), ds, col(kDark, 1.0f - e));
        const float introW = std::min(0.52f * W, 1.6f * H);
        const float push = flow.screen() == Screen::Intro ? 0.94f + 0.06f * smooth(0.0f, float(GameFlow::kIntroLength), t)
                                                           : 1.0f;
        const float logoA = flow.screen() == Screen::Intro ? smooth(0.3f, 0.3f + float(GameFlow::kLogoFadeIn), t) : 1.0f;
        const ImVec2 c(W * 0.5f + (menuLogoC.x - W * 0.5f) * e, H * 0.47f + (menuLogoC.y - H * 0.47f) * e);
        drawLogo(c.x, c.y, (introW * push) + (menuLogoW - introW * push) * e, logoA);
        if (flow.screen() == Screen::Intro) {
            // A thin orange rule grows under the logo, then a spaced tagline.
            const float lineW = introW * 0.5f * smooth(0.9f, 2.4f, t);
            const float ly = c.y + introW / m_logoAspect * 0.62f;
            dl->AddRectFilled(ImVec2(W * 0.5f - lineW * 0.5f, ly), ImVec2(W * 0.5f + lineW * 0.5f, ly + 2.0f),
                              col(kOrange, logoA));
            const char* tag = "OFF-ROAD PROVING GROUND";
            const float ts = 0.024f * H;
            text(dl, m_body, ts, ImVec2(W * 0.5f - textWidth(m_body, ts, tag, 0.35f) * 0.5f, ly + 0.025f * H),
                 col(kWhite, 0.75f * smooth(1.6f, 2.8f, t)), tag, 0.35f);
            return;
        }
        drawMenu(flow, settings, view, smooth(0.35f, 1.0f, flow.menuFade()));
        return;
    }
    // The Controls diagrams and the vehicle cards need the room.
    if (flow.page() != MenuPage::Controls && flow.page() != MenuPage::Vehicle) drawLogo(menuLogoC.x, menuLogoC.y, menuLogoW, 1.0f);
    drawMenu(flow, settings, view, 1.0f);
}

void MenuUI::drawMenu(const GameFlow& flow, const GameSettings& settings, const MenuView& view, float alpha) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const ImVec2 ds = ImGui::GetIO().DisplaySize;
    const float W = ds.x, H = ds.y;
    const float x0 = std::max(0.07f * W, 48.0f);
    const bool paused = flow.screen() == Screen::Paused;
    const MenuPage page = flow.page();

    // Readability: a dark gradient behind the left column (the frame itself is blurred in the pause menu).
    const bool controls = page == MenuPage::Controls;
    const float shade = paused ? 0.45f : 0.62f;
    const float gradW = 0.62f * W;
    dl->AddRectFilledMultiColor(ImVec2(0, 0), ImVec2(gradW, H), col(kDark, shade * alpha), 0, 0, col(kDark, shade * alpha));
    dl->AddRectFilledMultiColor(ImVec2(0, H * 0.82f), ImVec2(W, H), 0, 0, col(kDark, 0.5f * alpha), col(kDark, 0.5f * alpha));
    if (controls) dl->AddRectFilled(ImVec2(0, 0), ds, col(kDark, 0.5f * alpha)); // the diagrams span the whole width

    const float logoW = std::min(0.27f * W, 0.75f * H);
    const float logoBottom = 0.08f * H + logoW / m_logoAspect;
    const bool compact = page == MenuPage::Settings || controls;
    const bool vehiclePage = page == MenuPage::Vehicle;
    float y = controls || vehiclePage ? 0.05f * H : logoBottom + 0.05f * H;

    // Page title: thin rule, large spaced uppercase heading.
    if (const char* title = menuPageTitle(page); title[0]) {
        const float ts = 0.048f * H;
        dl->AddRectFilled(ImVec2(x0, y), ImVec2(x0 + 0.34f * W, y + 1.0f), col(kWhite, 0.35f * alpha));
        y += 0.018f * H;
        text(dl, m_heading, ts, ImVec2(x0, y), col(kWhite, alpha), title, 0.22f);
        y += ts + 0.03f * H;
    } else {
        y += 0.06f * H;
    }

    const std::vector<MenuItem>& items = flow.items();
    const int cursor = flow.cursor();
    const bool newPage = int(page) != m_lastPage;
    if (newPage) {
        m_selectAnim = float(cursor);
        m_lastPage = int(page);
        m_menuScroll = m_menuScrollTarget = 0.0f;
    }
    m_selectAnim += (float(cursor) - m_selectAnim) * std::min(1.0f, ImGui::GetIO().DeltaTime * 18.0f);

    const float textSize = compact ? 0.027f * H : 0.036f * H;
    const float rowH = compact ? 0.040f * H : 0.064f * H;
    const float barW = page == MenuPage::Settings || controls ? 0.46f * W : 0.34f * W;
    const float valueX = x0 + 0.36f * W; // centre of the value column (Settings, Controls)
    const float arrowDx = 0.075f * W;

    // Controls: the keyboard diagram under DRIVE KEYS, the gamepad diagram under GAMEPAD BUTTONS.
    const float contentW = W - 2.0f * x0;
    const float u = std::min(0.056f * H, contentW / (kKbPitches * kPitchX));
    const float g = 1.1f * u;
    const float lf = labelSize(u);
    const float kbBlock = 0.02f * H + kbHeight(u, lf), padBlock = 0.01f * H + padHeight(g, lf);

    // Everything below the title scrolls when it does not fit above the footer: the rows (section headings take
    // space) and the Controls diagrams. Positions are relative to the top of the list.
    const float listTop = y, listBottom = 0.88f * H;
    float cy = 0.0f;
    std::vector<float> rowY(items.size()), blockH(items.size(), 0.0f);
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (page == MenuPage::Settings && menuSection(items[i])) cy += 0.020f * H + (i > 0 ? 0.014f * H : 0.0f);
        if (items[i] == MenuItem::Controls) cy += 0.016f * H;
        rowY[i] = cy;
        cy += rowH;
        if (controls && items[i] == MenuItem::DriveKeys) blockH[i] = kbBlock;
        if (controls && items[i] == MenuItem::PadGlyphs) blockH[i] = padBlock;
        if (vehiclePage && items[i] == MenuItem::VehicleChoice) blockH[i] = vehicleBlockHeight(view);
        cy += blockH[i];
    }
    const float viewH = listBottom - listTop;
    const bool follow = (newPage || cursor != m_lastCursor) && !m_pointerCursor;
    // The first heading of Settings sits above its row: keep it in view with the row. A Controls row brings its
    // diagram into view with it.
    const float cursorTop = rowY[std::size_t(cursor)] - (cursor == 0 ? rowY[0] : 0.0f);
    const float cursorH = std::min(rowY[std::size_t(cursor)] + rowH + blockH[std::size_t(cursor)] - cursorTop, viewH);
    updateScroll(m_menuScroll, m_menuScrollTarget, cursorTop, cursorH, rowH, cy, viewH, follow);
    if (newPage) m_menuScroll = m_menuScrollTarget;
    m_lastCursor = cursor;
    for (float& r : rowY) r += listTop - m_menuScroll;
    dl->PushClipRect(ImVec2(0, listTop), ImVec2(W, listBottom), true);

    if (controls) {
        m_callouts.clear();
        m_obstacles.clear();
        for (std::size_t i = 0; i < items.size(); ++i) {
            if (items[i] == MenuItem::DriveKeys) drawKeyboard(dl, x0, rowY[i] + rowH + 0.02f * H, u, view, alpha);
            if (items[i] == MenuItem::PadGlyphs)
                drawGamepad(dl, x0 + 0.5f * kKbPitches * kPitchX * u, rowY[i] + rowH + 0.01f * H, g, view, alpha);
        }
        int n = 0;
        for (std::size_t a = 0; a < m_callouts.size(); ++a) {
            for (std::size_t b = a + 1; b < m_callouts.size(); ++b) n += overlaps(m_callouts[a], m_callouts[b]);
            for (const Box& o : m_obstacles) n += overlaps(m_callouts[a], o);
        }
        if (n != m_controlsOverlaps) std::printf("Menu: controls page callouts %zu, overlaps %d\n", m_callouts.size(), n);
        m_controlsCallouts = int(m_callouts.size());
        m_controlsOverlaps = n;
    }
    if (vehiclePage)
        for (std::size_t i = 0; i < items.size(); ++i)
            if (items[i] == MenuItem::VehicleChoice)
                drawVehicleBlock(dl, x0 - 0.012f * W, rowY[i] + rowH, barW + 0.012f * W, flow.vehicleIndex(settings), view, alpha);
    // Highlight bar eased between rows: soft fill fading to the right, orange edge on the left.
    {
        const float f = std::clamp(m_selectAnim, 0.0f, float(items.size() - 1));
        const int i0 = int(std::floor(f));
        const int i1 = std::min(i0 + 1, int(items.size()) - 1);
        const float by = rowY[std::size_t(i0)] + (rowY[std::size_t(i1)] - rowY[std::size_t(i0)]) * (f - float(i0));
        const ImVec2 a(x0 - 0.012f * W, by), b(x0 + barW, by + rowH * 0.92f);
        dl->AddRectFilledMultiColor(a, b, col(kWhite, 0.16f * alpha), col(kWhite, 0.0f), col(kWhite, 0.0f),
                                    col(kWhite, 0.16f * alpha));
        dl->AddLine(ImVec2(a.x, a.y), ImVec2(b.x * 0.7f + a.x * 0.3f, a.y), col(kWhite, 0.22f * alpha), 1.0f);
        dl->AddLine(ImVec2(a.x, b.y), ImVec2(b.x * 0.7f + a.x * 0.3f, b.y), col(kWhite, 0.22f * alpha), 1.0f);
        dl->AddRectFilled(a, ImVec2(a.x + 3.0f, b.y), col(kOrange, alpha));
    }

    for (std::size_t i = 0; i < items.size(); ++i) {
        const MenuItem item = items[i];
        const bool sel = int(i) == cursor;
        const float ry = rowY[i];
        if (page == MenuPage::Settings) {
            if (const char* section = menuSection(item)) {
                const float ss = 0.018f * H;
                const float sy = ry - 0.020f * H - 0.004f * H;
                text(dl, m_heading, ss, ImVec2(x0, sy), col(kOrange, alpha), section, 0.3f);
                const float sw = textWidth(m_heading, ss, section, 0.3f);
                dl->AddRectFilled(ImVec2(x0 + sw + 0.01f * W, sy + ss * 0.55f), ImVec2(x0 + barW, sy + ss * 0.55f + 1.0f),
                                  col(kWhite, 0.18f * alpha));
            }
        }
        const float ty = ry + (rowH * 0.92f - textSize) * 0.5f;
        const ImU32 c = col(kWhite, (sel ? 1.0f : 0.62f) * alpha);
        text(dl, m_body, textSize, ImVec2(x0 + 0.006f * W, ty), c, menuLabel(item), 0.08f);
        Row row{item, x0 - 0.012f * W, ry, x0 + barW, ry + rowH * 0.92f};
        if (menuAdjustable(item)) {
            std::string v = menuValue(item, settings, flow.levels().empty() ? std::string() : flow.levels().front());
            if (item == MenuItem::VehicleChoice && view.vehicles && !view.vehicles->empty()) {
                const int vi = std::clamp(flow.vehicleIndex(settings), 0, int(view.vehicles->size()) - 1);
                v = (*view.vehicles)[std::size_t(vi)].name;
                for (char& ch : v) ch = char(std::toupper(static_cast<unsigned char>(ch)));
            }
            if (item == MenuItem::PadGlyphs && settings.padGlyphs == int(PadGlyphSetting::Auto))
                v += std::string(" (") + padGlyphsName(view.padGlyphs) + ")";
            const float vw = textWidth(m_body, textSize, v.c_str(), 0.06f);
            text(dl, m_body, textSize, ImVec2(valueX - vw * 0.5f, ty), col(kWhite, (sel ? 1.0f : 0.75f) * alpha),
                 v.c_str(), 0.06f);
            // Value arrows (clickable); bright on the selected row.
            const float ah = textSize * 0.32f, acy = ty + textSize * 0.55f;
            const ImU32 ac = sel ? col(kOrange, alpha) : col(kWhite, 0.25f * alpha);
            const float lx = valueX - arrowDx, rx = valueX + arrowDx;
            dl->AddTriangleFilled(ImVec2(lx - ah * 0.6f, acy), ImVec2(lx + ah * 0.6f, acy - ah), ImVec2(lx + ah * 0.6f, acy + ah), ac);
            dl->AddTriangleFilled(ImVec2(rx + ah * 0.6f, acy), ImVec2(rx - ah * 0.6f, acy - ah), ImVec2(rx - ah * 0.6f, acy + ah), ac);
            row.arrowL0 = lx - textSize;
            row.arrowL1 = lx + textSize;
            row.arrowR0 = rx - textSize;
            row.arrowR1 = rx + textSize;
        }
        // Only the visible part of a row can be hit.
        row.y0 = std::max(row.y0, listTop);
        row.y1 = std::min(row.y1, listBottom);
        if (row.y1 > row.y0) m_rows.push_back(row);
    }
    dl->PopClipRect();
    drawScrollMarks(x0 - 0.012f * W, x0 + barW, listTop, listBottom, m_menuScroll, cy - viewH, alpha);

    // Footer: version bottom-left, input hints bottom-right.
    const float fs = 0.018f * H;
    char version[64];
    std::snprintf(version, sizeof(version), "TRAILFORGE %s", view.version);
    text(dl, m_body, fs, ImVec2(x0, H - 0.05f * H), col(kWhite, 0.4f * alpha), version, 0.2f);
    drawHints(flow, view, alpha);
}

float MenuUI::labelSize(float u) const { return std::min(0.0165f * ImGui::GetIO().DisplaySize.y, 0.31f * u); }

void MenuUI::drawKeyboard(ImDrawList* dl, float ox, float oy, float u, const MenuView& view, float alpha) {
    const float lf = labelSize(u), lineH = 1.12f * lf;
    const float pitchX = kPitchX * u, cap = kCap * u, gapX = pitchX - cap;
    const float rowPitch = kbRowPitch(u, lf);
    const float thin = std::max(1.0f, 0.028f * u), bold = std::max(2.0f, 0.06f * u);
    const float rounding = 0.16f * u, tick = 0.22f * u;
    const ImU32 leader = col(kWhite, 0.4f * alpha);
    const auto rowTop = [&](int row) { return oy + float(row) * rowPitch; };

    struct Label {
        float want = 0.0f, width = 0.0f, capBottom = 0.0f;
        std::vector<std::string> lines;
        ImFont* font = nullptr;
        ImU32 colour = 0;
    };
    std::array<std::vector<Label>, 6> rows;
    for (const KeyBinding& b : keyboardBindings(view.driveKeys)) {
        const KeyRect r = keyRect(b.key);
        const bool hi = b.group == BindingGroup::Drive;
        const ImVec2 a(ox + r.x * pitchX, rowTop(r.row));
        const ImVec2 z(a.x + r.w * pitchX - gapX, a.y + cap);
        const ImVec2 c(0.5f * (a.x + z.x), 0.5f * (a.y + z.y));
        const ImU32 kc = col(kWhite, (hi ? 1.0f : 0.72f) * alpha);
        if (hi) dl->AddRectFilled(a, z, col(kWhite, 0.10f * alpha), rounding);
        dl->AddRect(a, z, kc, rounding, 0, hi ? bold : thin);
        m_obstacles.push_back({a.x, a.y, z.x, z.y});
        const ImVec2 dir = keyArrowDir(b.key);
        if (dir.x != 0.0f || dir.y != 0.0f) {
            arrowGlyph(dl, c, dir, 0.2f * u, kc, hi ? bold : 1.3f * thin);
        } else {
            std::vector<std::string> lines;
            const std::string face = keyLabel(b.key, view.keyboard);
            for (std::size_t p = 0, q; p <= face.size(); p = q + 1) {
                q = face.find('\n', p);
                if (q == std::string::npos) q = face.size();
                lines.push_back(face.substr(p, q - p));
            }
            float size = face.size() <= 2 ? 0.40f * u : face.size() <= 3 ? 0.30f * u : 0.28f * u;
            for (const std::string& l : lines)
                size = std::min(size, (z.x - a.x - 0.16f * u) / std::max(1.0f, textWidth(m_heading, 1.0f, l.c_str(), 0.04f)));
            float ty = c.y - 0.5f * float(lines.size()) * 1.05f * size;
            for (const std::string& l : lines) {
                text(dl, m_heading, size, ImVec2(c.x - 0.5f * textWidth(m_heading, size, l.c_str(), 0.04f), ty), kc,
                     l.c_str(), 0.04f);
                ty += 1.05f * size;
            }
        }
        Label l;
        l.font = hi ? m_heading : m_body;
        l.colour = hi ? col(kOrange, alpha) : col(kWhite, 0.8f * alpha);
        const auto width = [&](const std::string& s) { return textWidth(l.font, lf, s.c_str(), 0.05f); };
        l.lines = wrapLabel(b.label, std::max(z.x - a.x, pitchX), width);
        for (const std::string& s : l.lines) l.width = std::max(l.width, width(s));
        l.want = c.x;
        l.capBottom = z.y;
        rows[std::size_t(r.row)].push_back(l);
    }

    // Callouts under each row, pushed apart where neighbours would overlap (rows 0-3 stop short of the mouse).
    for (int row = 0; row < 6; ++row) {
        std::vector<Label>& ls = rows[std::size_t(row)];
        std::sort(ls.begin(), ls.end(), [](const Label& p, const Label& q) { return p.want < q.want; });
        std::vector<float> want, size;
        for (const Label& l : ls) {
            want.push_back(l.want);
            size.push_back(l.width);
        }
        const float hi = ox + (row <= 3 ? kMouseX - 1.0f : kKbPitches) * pitchX;
        const std::vector<float> at = spreadLabels(want, size, 0.6f * lf, ox, hi);
        for (std::size_t i = 0; i < ls.size(); ++i) {
            const Label& l = ls[i];
            const float top = l.capBottom + tick;
            dl->AddLine(ImVec2(l.want, l.capBottom + 0.05f * u), ImVec2(at[i], top - 0.05f * u), leader, thin);
            float ty = top;
            for (const std::string& s : l.lines) {
                text(dl, l.font, lf, ImVec2(at[i] - 0.5f * textWidth(l.font, lf, s.c_str(), 0.05f), ty), l.colour, s.c_str(),
                     0.05f);
                ty += lineH;
            }
            m_callouts.push_back({at[i] - 0.5f * l.width, top, at[i] + 0.5f * l.width, ty});
        }
    }

    // Mouse: right button held + move orbits the camera, the wheel zooms.
    const float mw = 1.4f * u, mh = 2.1f * u, mcx = ox + kMouseX * pitchX;
    const float mTop = rowTop(1) + 0.5f * (rowTop(3) + cap - rowTop(1) - mh);
    const ImVec2 ma(mcx - 0.5f * mw, mTop), mz(mcx + 0.5f * mw, mTop + mh);
    const float split = mTop + 0.42f * mh;
    const ImU32 mc = col(kWhite, 0.72f * alpha);
    dl->AddRectFilled(ImVec2(mcx, mTop + 0.5f * thin), ImVec2(mz.x - 0.5f * thin, split), col(kWhite, 0.18f * alpha),
                      0.5f * mw, ImDrawFlags_RoundCornersTopRight);
    dl->AddRect(ma, mz, mc, 0.5f * mw, 0, thin);
    dl->AddLine(ImVec2(ma.x, split), ImVec2(mz.x, split), mc, thin);
    dl->AddLine(ImVec2(mcx, mTop), ImVec2(mcx, split), mc, thin);
    const ImVec2 wa(mcx - 0.08f * u, mTop + 0.15f * mh), wz(mcx + 0.08f * u, mTop + 0.31f * mh);
    dl->AddRectFilled(wa, wz, col(kDark, alpha), 0.08f * u);
    dl->AddRect(wa, wz, mc, 0.08f * u, 0, thin);
    m_obstacles.push_back({ma.x, ma.y, mz.x, mz.y});
    const char* mouseLines[] = {"ORBIT: RIGHT BUTTON + MOVE", "ZOOM: WHEEL"};
    float mlw = 0.0f;
    for (const char* s : mouseLines) mlw = std::max(mlw, textWidth(m_body, lf, s, 0.05f));
    const float mlx = std::min(mcx, ox + kKbPitches * pitchX - 0.5f * mlw);
    float ty = mz.y + tick;
    dl->AddLine(ImVec2(mcx, mz.y + 0.05f * u), ImVec2(mlx, ty - 0.05f * u), leader, thin);
    for (const char* s : mouseLines) {
        text(dl, m_body, lf, ImVec2(mlx - 0.5f * textWidth(m_body, lf, s, 0.05f), ty), col(kWhite, 0.8f * alpha), s, 0.05f);
        ty += lineH;
    }
    m_callouts.push_back({mlx - 0.5f * mlw, mz.y + tick, mlx + 0.5f * mlw, ty});
}

void MenuUI::drawGamepad(ImDrawList* dl, float cx, float oy, float g, const MenuView& view, float alpha) {
    const PadGlyphs glyphs = view.padGlyphs;
    const PadLayout L = padLayout(glyphs);
    const float u = g / 1.1f;
    const float lf = labelSize(u), lineH = 1.12f * lf;
    const float thin = std::max(1.0f, 0.028f * u), bold = std::max(2.0f, 0.06f * u);
    const float y0 = oy + kPadTop * g;
    const auto P = [&](float x, float y) { return ImVec2(cx + x * g, y0 + y * g); };
    const auto Pv = [&](ImVec2 v) { return P(v.x, v.y); };
    const ImU32 outline = col(kWhite, 0.78f * alpha), dim = col(kWhite, 0.32f * alpha), bright = col(kWhite, alpha);
    const ImU32 leader = col(kWhite, 0.4f * alpha);

    std::array<const PadBinding*, std::size_t(PadControl::Count)> bind{};
    const std::vector<PadBinding> bindings = padBindings();
    for (const PadBinding& b : bindings) bind[std::size_t(b.control)] = &b;
    const auto drive = [&](PadControl c) { return bind[std::size_t(c)] && bind[std::size_t(c)]->group == BindingGroup::Drive; };
    const auto colourOf = [&](PadControl c) { return !bind[std::size_t(c)] ? dim : drive(c) ? bright : outline; };
    const auto strokeOf = [&](PadControl c) { return drive(c) ? bold : thin; };

    // Body: right half as cubic segments from the top centre to the bottom centre, the left half mirrored.
    struct Seg {
        ImVec2 c1, c2, p;
    };
    const ImVec2 start(0.0f, -0.05f);
    static const Seg kRight[] = {{{0.9f, -0.12f}, {1.7f, -0.08f}, {2.3f, 0.05f}},  {{2.95f, 0.15f}, {3.35f, 0.5f}, {3.45f, 1.0f}},
                                 {{3.6f, 1.8f}, {3.85f, 3.2f}, {3.55f, 3.9f}},     {{3.4f, 4.5f}, {2.95f, 4.75f}, {2.55f, 4.55f}},
                                 {{2.15f, 4.3f}, {1.95f, 3.25f}, {1.55f, 3.05f}},  {{1.15f, 2.88f}, {0.6f, 2.85f}, {0.0f, 2.85f}}};
    dl->PathLineTo(Pv(start));
    for (const Seg& s : kRight) dl->PathBezierCubicCurveTo(Pv(s.c1), Pv(s.c2), Pv(s.p));
    for (int i = int(std::size(kRight)) - 1; i >= 0; --i) {
        const Seg& s = kRight[std::size_t(i)];
        const ImVec2 from = i > 0 ? kRight[std::size_t(i) - 1].p : start;
        dl->PathBezierCubicCurveTo(P(-s.c2.x, s.c2.y), P(-s.c1.x, s.c1.y), P(-from.x, from.y));
    }
    dl->PathStroke(outline, ImDrawFlags_Closed, 1.3f * thin);
    m_obstacles.push_back({P(-3.65f, -1.1f).x, P(-3.65f, -1.1f).y, P(3.65f, 4.8f).x, P(3.65f, 4.8f).y});

    // Shoulders: bumpers on the top edge, triggers behind them.
    for (int side = -1; side <= 1; side += 2) {
        const float s = float(side);
        const PadControl bumper = side < 0 ? PadControl::LeftShoulder : PadControl::RightShoulder;
        const PadControl trigger = side < 0 ? PadControl::LeftTrigger : PadControl::RightTrigger;
        const ImVec2 ba = P(side < 0 ? -3.05f : 1.95f, -0.24f), bz = P(side < 0 ? -1.95f : 3.05f, 0.04f);
        dl->AddRect(ba, bz, colourOf(bumper), 0.14f * g, 0, strokeOf(bumper));
        const ImVec2 ta = P(s < 0 ? -2.9f : 2.1f, -0.92f), tz = P(s < 0 ? -2.1f : 2.9f, -0.34f);
        if (drive(trigger)) dl->AddRectFilled(ta, tz, col(kWhite, 0.12f * alpha), 0.22f * g, ImDrawFlags_RoundCornersTop);
        dl->AddRect(ta, tz, colourOf(trigger), 0.22f * g, ImDrawFlags_RoundCornersTop, strokeOf(trigger));
    }
    // Sticks.
    for (const auto& [c, pos] : {std::pair{PadControl::LeftStick, L.leftStick}, std::pair{PadControl::RightStick, L.rightStick}}) {
        const ImVec2 p = Pv(pos);
        if (drive(c)) dl->AddCircleFilled(p, 0.62f * kStickR * g, col(kWhite, 0.12f * alpha));
        dl->AddCircle(p, kStickR * g, colourOf(c), 0, strokeOf(c));
        dl->AddCircle(p, 0.62f * kStickR * g, colourOf(c), 0, thin);
    }
    // D-pad: a plus outline.
    {
        const float a = kDpadArm, h = kDpadHalf;
        const ImVec2 d = L.dpad;
        const ImVec2 pts[12] = {P(d.x - h, d.y - a), P(d.x + h, d.y - a), P(d.x + h, d.y - h), P(d.x + a, d.y - h),
                                P(d.x + a, d.y + h), P(d.x + h, d.y + h), P(d.x + h, d.y + a), P(d.x - h, d.y + a),
                                P(d.x - h, d.y + h), P(d.x - a, d.y + h), P(d.x - a, d.y - h), P(d.x - h, d.y - h)};
        dl->AddPolyline(pts, 12, colourOf(PadControl::DpadUp), ImDrawFlags_Closed, thin);
        for (const PadControl c : {PadControl::DpadUp, PadControl::DpadDown, PadControl::DpadLeft, PadControl::DpadRight}) {
            const ImVec2 dir = c == PadControl::DpadUp ? ImVec2(0, -1) : c == PadControl::DpadDown ? ImVec2(0, 1)
                               : c == PadControl::DpadLeft ? ImVec2(-1, 0) : ImVec2(1, 0);
            const ImVec2 t = P(d.x + dir.x * (a - 0.13f), d.y + dir.y * (a - 0.13f));
            const ImVec2 n(-dir.y, dir.x);
            const float s = 0.07f * g;
            dl->AddTriangleFilled(ImVec2(t.x + dir.x * s, t.y + dir.y * s), ImVec2(t.x - dir.x * s + n.x * s, t.y - dir.y * s + n.y * s),
                                  ImVec2(t.x - dir.x * s - n.x * s, t.y - dir.y * s - n.y * s), colourOf(c));
        }
    }
    // Face buttons.
    for (const PadControl c : {PadControl::South, PadControl::East, PadControl::West, PadControl::North}) {
        const ImVec2 o = faceOffset(c);
        faceGlyph(dl, m_heading, P(L.face.x + o.x, L.face.y + o.y), kFaceR * g, c, glyphs, colourOf(c), strokeOf(c));
    }
    // View / Menu (Create / Options), guide, touchpad.
    for (const auto& [c, pos] : {std::pair{PadControl::Back, L.back}, std::pair{PadControl::Start, L.start}}) {
        const ImVec2 hs = L.touchpad ? ImVec2(0.07f, 0.14f) : ImVec2(0.16f, 0.11f);
        dl->AddRect(P(pos.x - hs.x, pos.y - hs.y), P(pos.x + hs.x, pos.y + hs.y), colourOf(c), 0.07f * g, 0, thin);
    }
    dl->AddCircle(Pv(L.guide), (L.touchpad ? 0.17f : 0.26f) * g, dim, 0, thin);
    if (L.touchpad) dl->AddRect(P(-0.85f, 0.15f), P(0.85f, 1.35f), dim, 0.18f * g, 0, thin);

    // Callouts: columns left and right of the body, rows above and below it for the controls in the middle.
    enum class Side { Left, Right, Top, Bottom };
    struct Line {
        PadControl icon = PadControl::Count; // face button glyph instead of the name
        ImVec2 arrow{0, 0};                  // D-pad direction marker instead of the name
        std::string name, action;
        bool drive = false;
    };
    struct Item {
        Side side;
        ImVec2 anchor; // g units
        std::vector<Line> lines;
        float w = 0.0f, h = 0.0f, at = 0.0f;
        // Side columns: the leader drops straight down from the anchor to this row (g units) and runs across to the
        // column there, so it passes below the buttons between the anchor and the column instead of across them.
        std::optional<float> dropRow = std::nullopt;
    };
    const float gapName = 0.45f * lf;
    const auto lineWidth = [&](const Line& l) {
        float w = l.icon != PadControl::Count ? lineH : l.arrow.x != 0.0f || l.arrow.y != 0.0f ? 0.7f * lineH
                                                                                              : textWidth(m_heading, lf, l.name.c_str(), 0.05f);
        if (!l.action.empty()) w += (w > 0.0f ? gapName : 0.0f) + textWidth(l.drive ? m_heading : m_body, lf, l.action.c_str(), 0.05f);
        return w;
    };
    std::vector<Item> items;
    Item dpad{Side::Left, ImVec2(L.dpad.x - kDpadArm, L.dpad.y), {Line{PadControl::Count, {0, 0}, "D-PAD", "", false}}};
    for (const PadBinding& b : bindings) {
        const PadControl c = b.control;
        const bool face = isFace(c);
        const bool isDrive = b.group == BindingGroup::Drive;
        Line name{face ? c : PadControl::Count, {0, 0}, face ? "" : padControlName(c, glyphs), "", isDrive};
        if (c == PadControl::DpadUp || c == PadControl::DpadDown || c == PadControl::DpadLeft || c == PadControl::DpadRight) {
            const ImVec2 dir = c == PadControl::DpadUp ? ImVec2(0, -1) : c == PadControl::DpadDown ? ImVec2(0, 1)
                               : c == PadControl::DpadLeft ? ImVec2(-1, 0) : ImVec2(1, 0);
            dpad.lines.push_back(Line{PadControl::Count, dir, "", b.label, isDrive});
            continue;
        }
        Item it{Side::Left, {0, 0}, {}};
        const ImVec2 fo = faceOffset(c);
        const ImVec2 fc(L.face.x + fo.x, L.face.y + fo.y);
        switch (c) {
        case PadControl::LeftTrigger: it = {Side::Left, {-2.9f, -0.63f}, {}}; break;
        case PadControl::RightTrigger: it = {Side::Right, {2.9f, -0.63f}, {}}; break;
        case PadControl::LeftShoulder: it = {Side::Left, {-3.05f, -0.1f}, {}}; break;
        case PadControl::RightShoulder: it = {Side::Right, {3.05f, -0.1f}, {}}; break;
        case PadControl::LeftStick: it = {Side::Left, {L.leftStick.x - kStickR, L.leftStick.y}, {}}; break;
        case PadControl::RightStick: it = {Side::Right, {L.rightStick.x + kStickR, L.rightStick.y}, {}}; break;
        case PadControl::West: {
            const ImVec2 so = faceOffset(PadControl::South);
            // Down from the lower right of the button, midway between the right stick and the bottom button.
            const float dx = 0.1f;
            it = {Side::Right, {fc.x + dx, fc.y + std::sqrt(kFaceR * kFaceR - dx * dx)}, {}};
            it.dropRow = L.face.y + so.y + kFaceR + 0.42f;
            break;
        }
        case PadControl::South: case PadControl::East: case PadControl::North:
            it = {Side::Right, {fc.x + kFaceR, fc.y}, {}};
            break;
        case PadControl::Back: case PadControl::Start: {
            const ImVec2 p = c == PadControl::Back ? L.back : L.start;
            it = L.touchpad ? Item{Side::Top, {p.x, p.y - 0.14f}, {}} : Item{Side::Bottom, {p.x, p.y + 0.11f}, {}};
            break;
        }
        default: continue;
        }
        if (it.side == Side::Left || it.side == Side::Right) {
            name.action = b.label;
            it.lines = {name};
        } else {
            it.lines = {name, Line{PadControl::Count, {0, 0}, "", b.label, isDrive}};
        }
        items.push_back(it);
    }
    if (dpad.lines.size() > 1) items.push_back(dpad);
    for (Item& it : items) {
        for (const Line& l : it.lines) it.w = std::max(it.w, lineWidth(l));
        it.h = float(it.lines.size()) * lineH;
    }

    const float edge = 4.15f * g, elbow = 3.9f * g;
    const auto drawLine = [&](const Line& l, float x, float y) { // x: left edge
        const float mid = y + 0.5f * lineH;
        if (l.icon != PadControl::Count) {
            faceGlyph(dl, m_heading, ImVec2(x + 0.5f * lineH, mid), 0.55f * lineH, l.icon, glyphs,
                      l.drive ? bright : outline, l.drive ? 0.75f * bold : thin);
            x += lineH + gapName;
        } else if (l.arrow.x != 0.0f || l.arrow.y != 0.0f) {
            const ImVec2 c(x + 0.35f * lineH, mid), d = l.arrow, n(-d.y, d.x);
            const float s = 0.22f * lineH;
            dl->AddTriangleFilled(ImVec2(c.x + d.x * s, c.y + d.y * s), ImVec2(c.x - d.x * s + n.x * s, c.y - d.y * s + n.y * s),
                                  ImVec2(c.x - d.x * s - n.x * s, c.y - d.y * s - n.y * s), outline);
            x += 0.7f * lineH + gapName;
        } else if (!l.name.empty()) {
            text(dl, m_heading, lf, ImVec2(x, y), bright, l.name.c_str(), 0.05f);
            x += textWidth(m_heading, lf, l.name.c_str(), 0.05f) + gapName;
        }
        if (!l.action.empty())
            text(dl, l.drive ? m_heading : m_body, lf, ImVec2(x, y), l.drive ? col(kOrange, alpha) : col(kWhite, 0.8f * alpha),
                 l.action.c_str(), 0.05f);
    };
    for (const Side side : {Side::Left, Side::Right, Side::Top, Side::Bottom}) {
        std::vector<Item*> col_;
        for (Item& it : items)
            if (it.side == side) col_.push_back(&it);
        const bool vertical = side == Side::Left || side == Side::Right;
        const auto rowOf = [](const Item* it) { return it->dropRow.value_or(it->anchor.y); };
        std::sort(col_.begin(), col_.end(), [&](const Item* a, const Item* b) {
            return vertical ? rowOf(a) < rowOf(b) : a->anchor.x < b->anchor.x;
        });
        std::vector<float> want, size;
        for (const Item* it : col_) {
            // Side labels: the first line (the name) at the anchor height.
            want.push_back(vertical ? y0 + rowOf(it) * g + 0.5f * (it->h - lineH) : cx + it->anchor.x * g);
            size.push_back(vertical ? it->h : it->w);
        }
        const std::vector<float> at = vertical ? spreadLabels(want, size, 0.35f * lf, y0 - 1.2f * g, y0 + 5.0f * g)
                                               : spreadLabels(want, size, 1.2f * lf, cx - edge, cx + edge);
        for (std::size_t i = 0; i < col_.size(); ++i) {
            const Item& it = *col_[i];
            const ImVec2 a = Pv(it.anchor);
            float bx0, by0;
            if (vertical) {
                const float s = side == Side::Left ? -1.0f : 1.0f;
                by0 = at[i] - 0.5f * it.h;
                const float ly = by0 + 0.5f * lineH;
                const ImVec2 pts[3] = {ImVec2(cx + s * (edge - 0.12f * g), ly),
                                       it.dropRow ? ImVec2(a.x, ly) : ImVec2(cx + s * elbow, ly), a};
                dl->AddPolyline(pts, 3, leader, 0, thin);
                bx0 = side == Side::Left ? cx - edge - it.w : cx + edge;
                float y = by0;
                for (const Line& l : it.lines) {
                    drawLine(l, side == Side::Left ? cx - edge - lineWidth(l) : bx0, y);
                    y += lineH;
                }
            } else {
                const bool top = side == Side::Top;
                by0 = top ? y0 - 1.38f * g - it.h : y0 + 4.98f * g;
                const float ey = top ? y0 - 1.2f * g : y0 + 4.82f * g;
                const ImVec2 pts[3] = {a, ImVec2(a.x, ey), ImVec2(at[i], top ? by0 + it.h + 0.04f * g : by0 - 0.04f * g)};
                dl->AddPolyline(pts, 3, leader, 0, thin);
                bx0 = at[i] - 0.5f * it.w;
                float y = by0;
                for (const Line& l : it.lines) {
                    drawLine(l, at[i] - 0.5f * lineWidth(l), y);
                    y += lineH;
                }
            }
            dl->AddCircleFilled(a, std::max(1.5f, 0.05f * g), leader);
            m_callouts.push_back({bx0, by0, bx0 + it.w, by0 + it.h});
        }
    }

    // Which pad the names follow.
    std::string status = view.padType == PadType::None ? "NO GAMEPAD CONNECTED" : std::string("CONNECTED: ") + padTypeName(view.padType);
    for (char& ch : status) ch = char(std::toupper(static_cast<unsigned char>(ch)));
    const float sw = textWidth(m_body, lf, status.c_str(), 0.08f);
    text(dl, m_body, lf, ImVec2(cx - edge - sw, oy + 0.1f * g), col(kWhite, 0.55f * alpha), status.c_str(), 0.08f);
    m_callouts.push_back({cx - edge - sw, oy + 0.1f * g, cx - edge, oy + 0.1f * g + lineH});
}

float MenuUI::vehicleBlockHeight(const MenuView& view) const {
    const float H = ImGui::GetIO().DisplaySize.y;
    const std::size_t specs = view.vehicles && !view.vehicles->empty() ? view.vehicles->front().specs.size() + 1 : 6;
    return 0.012f * H + 0.075f * H + 0.02f * H + 0.075f * H + float(specs) * 0.032f * H + 0.02f * H;
}

void MenuUI::drawVehicleBlock(ImDrawList* dl, float x, float y, float w, int selected, const MenuView& view, float alpha) {
    if (!view.vehicles || view.vehicles->empty()) return;
    const std::vector<VehicleCard>& cards = *view.vehicles;
    const float H = ImGui::GetIO().DisplaySize.y;
    selected = std::clamp(selected, 0, int(cards.size()) - 1);
    // Card strip: name and drivetrain type; the selected card is lit with an orange edge.
    y += 0.012f * H;
    const float gap = 0.008f * H, cardH = 0.075f * H;
    const float cardW = (w - gap * float(cards.size() - 1)) / float(cards.size());
    for (std::size_t i = 0; i < cards.size(); ++i) {
        const bool sel = int(i) == selected;
        const ImVec2 a(x + float(i) * (cardW + gap), y), b(a.x + cardW, y + cardH);
        dl->AddRectFilled(a, b, col(kDark, (sel ? 0.55f : 0.35f) * alpha), 4.0f);
        dl->AddRect(a, b, col(kWhite, (sel ? 0.55f : 0.18f) * alpha), 4.0f, 0, 1.0f);
        if (sel) dl->AddRectFilled(ImVec2(a.x, b.y - 3.0f), ImVec2(b.x, b.y), col(kOrange, alpha));
        const float ns = 0.026f * H, ks = 0.017f * H;
        std::string name = cards[i].name;
        for (char& ch : name) ch = char(std::toupper(static_cast<unsigned char>(ch)));
        text(dl, m_heading, ns, ImVec2(a.x + 0.012f * H, a.y + 0.012f * H), col(kWhite, (sel ? 1.0f : 0.6f) * alpha),
             name.c_str(), 0.06f);
        const char* kind = cards[i].electric ? "ELECTRIC" : "GASOLINE";
        text(dl, m_body, ks, ImVec2(a.x + 0.012f * H, a.y + 0.012f * H + ns * 1.1f),
             col(cards[i].electric ? ImVec4(0.45f, 0.72f, 1.0f, 1.0f) : kOrange, (sel ? 0.95f : 0.55f) * alpha), kind, 0.25f);
        m_cards.push_back({a.x, a.y, b.x, b.y});
    }
    y += cardH + 0.02f * H;
    // Description and specs of the selected vehicle.
    const VehicleCard& c = cards[std::size_t(selected)];
    const float ds = 0.021f * H;
    dl->AddText(orDefault(m_body), ds, ImVec2(x + 0.004f * H, y), col(kWhite, 0.78f * alpha), c.description.c_str(), nullptr,
                w - 0.008f * H);
    y += 0.075f * H;
    const float ls = 0.019f * H, vs = 0.023f * H, rowH = 0.032f * H;
    for (const VehicleSpec& sp : c.specs) {
        text(dl, m_heading, ls, ImVec2(x + 0.004f * H, y + (vs - ls) * 0.6f), col(kOrange, 0.9f * alpha), sp.label.c_str(), 0.25f);
        text(dl, m_body, vs, ImVec2(x + 0.17f * H, y), col(kWhite, 0.92f * alpha), sp.value.c_str(), 0.03f);
        dl->AddRectFilled(ImVec2(x, y + rowH - 0.004f * H), ImVec2(x + w, y + rowH - 0.004f * H + 1.0f),
                          col(kWhite, 0.08f * alpha));
        y += rowH;
    }
}

int MenuUI::vehicleCardHit(float x, float y) const {
    for (std::size_t i = 0; i < m_cards.size(); ++i)
        if (x >= m_cards[i].x0 && x < m_cards[i].x1 && y >= m_cards[i].y0 && y < m_cards[i].y1) return int(i);
    return -1;
}

bool MenuUI::vehicleCardRect(int index, float& x0, float& y0, float& x1, float& y1) const {
    if (index < 0 || index >= int(m_cards.size())) return false;
    const Box& b = m_cards[std::size_t(index)];
    x0 = b.x0, y0 = b.y0, x1 = b.x1, y1 = b.y1;
    return true;
}

void MenuUI::drawHints(const GameFlow& flow, const MenuView& view, float alpha) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const ImVec2 ds = ImGui::GetIO().DisplaySize;
    const float H = ds.y;
    const float ts = 0.018f * H, pad = 0.006f * H, gap = 0.022f * H;
    struct Hint {
        const char* key;
        const char* label;
    };
    std::vector<Hint> hints;
    const bool adjustable = menuAdjustable(flow.current());
    if (view.gamepad) {
        if (adjustable) hints.push_back({"D-PAD", "CHANGE"});
        hints.push_back({padControlName(PadControl::South, view.padGlyphs), "SELECT"});
        if (flow.depth() > 1 || flow.screen() == Screen::Paused)
            hints.push_back({padControlName(PadControl::East, view.padGlyphs), "BACK"});
    } else {
        if (adjustable) hints.push_back({view.driveKeys == DriveKeys::Arrows ? "LEFT / RIGHT" : "A / D", "CHANGE"});
        hints.push_back({keyName(Key::Return, view.keyboard), "SELECT"});
        if (flow.depth() > 1 || flow.screen() == Screen::Paused) hints.push_back({keyName(Key::Escape, view.keyboard), "BACK"});
    }
    float x = ds.x - std::max(0.05f * ds.x, 40.0f);
    const float y = H - 0.055f * H;
    for (auto it = hints.rbegin(); it != hints.rend(); ++it) {
        const float lw = textWidth(m_body, ts, it->label, 0.15f);
        x -= lw;
        text(dl, m_body, ts, ImVec2(x, y), col(kWhite, 0.75f * alpha), it->label, 0.15f);
        const float kw = textWidth(m_heading, ts, it->key, 0.05f) + 2.0f * pad;
        x -= kw + 0.008f * H;
        dl->AddRect(ImVec2(x, y - pad * 0.6f), ImVec2(x + kw, y + ts + pad * 0.3f), col(kWhite, 0.6f * alpha),
                    3.0f, 0, 1.0f);
        text(dl, m_heading, ts, ImVec2(x + pad, y), col(kWhite, 0.9f * alpha), it->key, 0.05f);
        x -= gap;
    }
}

void MenuUI::updateScroll(float& scroll, float& target, float cursorY, float cursorH, float rowH, float contentH,
                          float viewH, bool follow) {
    // Keyboard / gamepad moves keep the cursor a few rows away from the edges; a row picked with the pointer is on
    // screen already and never scrolls the list away from it. The wheel moves the list freely.
    if (follow) {
        const float margin = std::min(2.5f * rowH, std::max(0.0f, 0.5f * (viewH - cursorH)));
        if (cursorY - margin < target) target = cursorY - margin;
        if (cursorY + cursorH + margin > target + viewH) target = cursorY + cursorH + margin - viewH;
    }
    target -= m_wheel * 2.0f * rowH;
    m_wheel = 0.0f;
    m_pointerCursor = false;
    target = std::clamp(target, 0.0f, std::max(0.0f, contentH - viewH));
    scroll += (target - scroll) * std::min(1.0f, ImGui::GetIO().DeltaTime * 14.0f);
    if (std::abs(target - scroll) < 0.5f) scroll = target;
}

void MenuUI::drawScrollMarks(float x0, float x1, float top, float bottom, float scroll, float maxScroll, float alpha) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    if (scroll > 1.0f)
        dl->AddRectFilledMultiColor(ImVec2(x0, top), ImVec2(x1, top + 10.0f), col(kOrange, 0.5f * alpha),
                                    col(kOrange, 0.5f * alpha), 0, 0);
    if (scroll < maxScroll - 1.0f)
        dl->AddRectFilledMultiColor(ImVec2(x0, bottom - 10.0f), ImVec2(x1, bottom), 0, 0, col(kOrange, 0.5f * alpha),
                                    col(kOrange, 0.5f * alpha));
}

int MenuUI::hitTest(float x, float y, int* dir) const {
    if (dir) *dir = 0;
    for (std::size_t i = 0; i < m_rows.size(); ++i) {
        const Row& r = m_rows[i];
        if (y < r.y0 || y >= r.y1) continue;
        const bool onLeft = r.arrowL1 > r.arrowL0 && x >= r.arrowL0 && x < r.arrowL1;
        const bool onRight = r.arrowR1 > r.arrowR0 && x >= r.arrowR0 && x < r.arrowR1;
        if (onLeft || onRight) {
            if (dir) *dir = onLeft ? -1 : 1;
            return int(i);
        }
        if (x >= r.x0 && x < r.x1) return int(i);
    }
    return -1;
}

bool MenuUI::itemRect(MenuItem item, float& x0, float& y0, float& x1, float& y1) const {
    for (const Row& r : m_rows) {
        if (r.item != item) continue;
        x0 = r.x0;
        y0 = r.y0;
        x1 = r.x1;
        y1 = r.y1;
        return true;
    }
    return false;
}


void MenuUI::drawGarage(const Garage& g, const VehicleTelemetry& t, const MenuView& view) {
    m_garageRows.clear();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const ImVec2 ds = ImGui::GetIO().DisplaySize;
    const float W = ds.x, H = ds.y;
    const float x0 = std::max(0.045f * W, 36.0f);
    const float panelW = std::max(0.44f * W, 560.0f);
    const float dt = ImGui::GetIO().DeltaTime;

    dl->AddRectFilledMultiColor(ImVec2(0, 0), ImVec2(x0 + panelW * 1.35f, H), col(kDark, 0.78f), 0, 0, col(kDark, 0.78f));
    dl->AddRectFilledMultiColor(ImVec2(0, H * 0.86f), ImVec2(W, H), 0, 0, col(kDark, 0.55f), col(kDark, 0.55f));

    // Title.
    float y = 0.045f * H;
    const float ts = 0.046f * H;
    text(dl, m_heading, ts, ImVec2(x0, y), col(kWhite, 1.0f), "GARAGE", 0.22f);
    const float tw = textWidth(m_heading, ts, "GARAGE", 0.22f);
    const std::string sub = g.vehicle().name;
    text(dl, m_body, 0.022f * H, ImVec2(x0 + tw + 0.02f * W, y + ts - 0.03f * H), col(kOrange, 1.0f), sub.c_str(), 0.12f);
    y += ts + 0.012f * H;
    dl->AddRectFilled(ImVec2(x0, y), ImVec2(x0 + panelW, y + 1.0f), col(kWhite, 0.35f));

    // List geometry: rows and section headings, then a scroll offset that keeps the cursor in view.
    const float listTop = y + 0.012f * H, listBottom = 0.80f * H;
    const float textSize = 0.0235f * H, rowH = 0.034f * H, headH = 0.032f * H;
    const auto& rows = g.rows();
    std::vector<float> rowY(rows.size());
    float cy = 0.0f;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        if (g.groupTitle(int(i))) cy += headH;
        rowY[i] = cy;
        cy += rowH;
    }
    const float contentH = cy, viewH = listBottom - listTop;
    const float cursorY = rowY[std::size_t(g.cursor())];
    const bool follow = g.cursor() != m_garageLastCursor && !m_pointerCursor;
    m_garageLastCursor = g.cursor();
    updateScroll(m_garageScroll, m_garageScrollTarget, cursorY, rowH, rowH, contentH, viewH, follow);
    m_garageSelectAnim += (cursorY - m_garageSelectAnim) * std::min(1.0f, dt * 18.0f);

    const float labelX = x0 + 0.006f * W;
    const float barX0 = x0 + panelW * 0.50f, barX1 = x0 + panelW * 0.74f;
    const float valueX = x0 + panelW * 0.77f;          // number values (left aligned after the bar)
    const float choiceX = x0 + panelW * 0.66f;         // centre of choice values
    const float arrowDx = panelW * 0.20f;
    dl->PushClipRect(ImVec2(0, listTop), ImVec2(W, listBottom), true);
    {
        const float by = listTop + m_garageSelectAnim - m_garageScroll;
        const ImVec2 a(x0 - 0.008f * W, by), b(x0 + panelW, by + rowH * 0.92f);
        dl->AddRectFilledMultiColor(a, b, col(kWhite, 0.16f), col(kWhite, 0.02f), col(kWhite, 0.02f), col(kWhite, 0.16f));
        dl->AddRectFilled(a, ImVec2(a.x + 3.0f, b.y), col(kOrange, 1.0f));
    }
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const float ry = listTop + rowY[i] - m_garageScroll;
        if (ry + rowH < listTop - headH || ry > listBottom) continue;
        const GarageRow& r = rows[i];
        const bool sel = int(i) == g.cursor();
        if (const char* section = g.groupTitle(int(i))) {
            const float ss = 0.017f * H;
            const float sy = ry - headH + 0.008f * H;
            text(dl, m_heading, ss, ImVec2(x0, sy), col(kOrange, 1.0f), section, 0.3f);
            const float sw = textWidth(m_heading, ss, section, 0.3f);
            dl->AddRectFilled(ImVec2(x0 + sw + 0.01f * W, sy + ss * 0.55f), ImVec2(x0 + panelW, sy + ss * 0.55f + 1.0f),
                              col(kWhite, 0.18f));
        }
        const float ty = ry + (rowH * 0.92f - textSize) * 0.5f;
        const float alpha = sel ? 1.0f : 0.68f;
        std::string label;
        switch (r.kind) {
        case GarageRowKind::Vehicle: label = "VEHICLE"; break;
        case GarageRowKind::Setup: label = "SETUP"; break;
        case GarageRowKind::Kit: label = "KIT"; break;
        case GarageRowKind::Param: label = r.param->label; break;
        case GarageRowKind::Action: label = Garage::actionLabel(r.action); break;
        }
        if (r.kind == GarageRowKind::Param)
            for (char& c : label) c = char(std::toupper(static_cast<unsigned char>(c)));
        // Long labels shrink to fit before the slider (the RESPAWN tag of rebuild parameters included).
        const bool rebuildTag = r.kind == GarageRowKind::Param && r.param->rebuild;
        const float tagW = rebuildTag ? textWidth(m_body, textSize * 0.6f, "RESPAWN", 0.1f) + 8.0f : 0.0f;
        const float room = barX0 - labelX - 14.0f - tagW;
        const float lw = textWidth(m_body, textSize, label.c_str(), 0.05f);
        const float ls = lw > room ? textSize * room / lw : textSize;
        text(dl, m_body, ls, ImVec2(labelX, ty + (textSize - ls) * 0.6f), col(kWhite, alpha), label.c_str(), 0.05f);
        if (rebuildTag)
            text(dl, m_body, textSize * 0.6f, ImVec2(labelX + std::min(lw, room) + 8.0f, ty + textSize * 0.3f),
                 col(kOrange, 0.85f * alpha), "RESPAWN", 0.1f);
        GarageHit hit{int(i), x0 - 0.008f * W, ry, x0 + panelW, ry + rowH * 0.92f};
        const bool number = r.kind == GarageRowKind::Param && r.param->kind == SetupParamKind::Number;
        if (number) {
            const float f = g.fraction(*r.param);
            const float bh = 4.0f, byc = ty + textSize * 0.55f;
            dl->AddRectFilled(ImVec2(barX0, byc - bh * 0.5f), ImVec2(barX1, byc + bh * 0.5f), col(kWhite, 0.18f), 2.0f);
            dl->AddRectFilled(ImVec2(barX0, byc - bh * 0.5f), ImVec2(barX0 + (barX1 - barX0) * f, byc + bh * 0.5f),
                              col(sel ? kOrange : kWhite, sel ? 1.0f : 0.5f), 2.0f);
            // Base value tick: where the definition (and kit) puts it.
            const ParamRange rg = g.range(*r.param);
            if (rg.max > rg.min) {
                const float bv = r.param->get(g.kitBase(), g.kitBase());
                const float bx = barX0 + (barX1 - barX0) * std::clamp((bv - rg.min) / (rg.max - rg.min), 0.0f, 1.0f);
                dl->AddRectFilled(ImVec2(bx - 1.0f, byc - 6.0f), ImVec2(bx + 1.0f, byc + 6.0f), col(kWhite, 0.55f));
            }
            dl->AddCircleFilled(ImVec2(barX0 + (barX1 - barX0) * f, byc), sel ? 6.0f : 4.5f, col(kWhite, alpha));
            const std::string v = g.valueText(r);
            text(dl, m_body, textSize, ImVec2(valueX, ty), col(kWhite, alpha), v.c_str(), 0.04f);
            if (const std::string note = g.valueNote(r); !note.empty()) {
                const float ns = textSize * 0.72f;
                text(dl, m_body, ns, ImVec2(valueX + textWidth(m_body, textSize, v.c_str(), 0.04f) + 0.4f * textSize,
                                            ty + (textSize - ns) * 0.7f),
                     col(kOrange, 0.85f * alpha), note.c_str(), 0.04f);
            }
            hit.bar0 = barX0 - 8.0f;
            hit.bar1 = barX1 + 8.0f;
        } else if (r.kind != GarageRowKind::Action) {
            const std::string v = g.valueText(r);
            const float vw = textWidth(m_body, textSize, v.c_str(), 0.04f);
            text(dl, m_body, textSize, ImVec2(choiceX - vw * 0.5f, ty), col(kWhite, (sel ? 1.0f : 0.8f)), v.c_str(), 0.04f);
            {
                const float ah = textSize * 0.30f, acy = ty + textSize * 0.55f;
                const ImU32 ac = sel ? col(kOrange, 1.0f) : col(kWhite, 0.25f);
                const float lx = choiceX - arrowDx, rx = choiceX + arrowDx;
                dl->AddTriangleFilled(ImVec2(lx - ah * 0.6f, acy), ImVec2(lx + ah * 0.6f, acy - ah), ImVec2(lx + ah * 0.6f, acy + ah), ac);
                dl->AddTriangleFilled(ImVec2(rx + ah * 0.6f, acy), ImVec2(rx - ah * 0.6f, acy - ah), ImVec2(rx - ah * 0.6f, acy + ah), ac);
                hit.arrowL0 = lx - textSize;
                hit.arrowL1 = lx + textSize;
                hit.arrowR0 = rx - textSize;
                hit.arrowR1 = rx + textSize;
            }
        }
        // Only the visible part of a row can be hit.
        hit.y0 = std::max(hit.y0, listTop);
        hit.y1 = std::min(hit.y1, listBottom);
        if (hit.y1 > hit.y0) m_garageRows.push_back(hit);
    }
    dl->PopClipRect();
    drawScrollMarks(x0, x0 + panelW, listTop, listBottom, m_garageScroll, contentH - viewH, 1.0f);

    // Description of the selected row: range and how it is applied.
    {
        const GarageRow& r = rows[std::size_t(g.cursor())];
        std::string d;
        if (r.kind == GarageRowKind::Param) {
            const std::string range = g.rangeText(*r.param);
            if (!range.empty()) d = "RANGE " + range + (r.param->unit[0] ? std::string(" ") + r.param->unit : "") + "   ";
            d += r.param->rebuild ? "APPLIED BY REBUILDING THE VEHICLE IN PLACE" : "APPLIED LIVE";
        } else if (r.kind == GarageRowKind::Kit) {
            d = g.vehicle().kits.empty() ? "KITS: DATA STRUCTURE READY, NO KIT CONTENT FOR THIS VEHICLE YET" : "KIT: BASE -> KIT -> FINE TUNE";
        } else if (r.kind == GarageRowKind::Setup) {
            d = g.dirty() ? "UNSAVED CHANGES (*)" : "SETUPS ARE SAVED PER VEHICLE";
        }
        if (!d.empty()) text(dl, m_body, 0.018f * H, ImVec2(x0, listBottom + 0.012f * H), col(kWhite, 0.6f), d.c_str(), 0.08f);
        if (const std::string help = g.helpText(r); !help.empty())
            text(dl, m_body, 0.017f * H, ImVec2(x0, listBottom + 0.036f * H), col(kOrange, 0.8f), help.c_str(), 0.06f);
    }
    // Last message (save, rename, refusals) for a few seconds.
    if (g.messageSerial() != m_garageMsgSerial) {
        m_garageMsgSerial = g.messageSerial();
        m_garageMsgTime = ImGui::GetTime();
    }
    if (!g.message().empty() && ImGui::GetTime() - m_garageMsgTime < 4.0) {
        const float a = float(std::clamp(4.0 - (ImGui::GetTime() - m_garageMsgTime), 0.0, 1.0));
        text(dl, m_body, 0.021f * H, ImVec2(x0, listBottom + 0.064f * H), col(kOrange, a), g.message().c_str(), 0.06f);
    }

    // Live numbers, bottom right: configuration and what the running vehicle does.
    {
        const VehicleConfig& c = g.config();
        const float bs = 0.019f * H;
        const float bx = W - std::max(0.05f * W, 40.0f) - 0.25f * W;
        const float boxH = bs * 1.5f + 6.0f * bs * 1.3f;
        float by = 0.83f * H - boxH;
        dl->AddRectFilled(ImVec2(bx - 0.012f * W, by - 0.012f * H),
                          ImVec2(W - std::max(0.05f * W, 40.0f) + 0.012f * W, by + boxH + 0.006f * H), col(kDark, 0.55f), 4.0f);
        text(dl, m_heading, bs * 0.95f, ImVec2(bx, by), col(kOrange, 1.0f), "LIVE", 0.3f);
        by += bs * 1.5f;
        char line[160];
        auto row = [&](const char* k, const char* v) {
            text(dl, m_body, bs, ImVec2(bx, by), col(kWhite, 0.6f), k, 0.06f);
            text(dl, m_body, bs, ImVec2(bx + 0.11f * W, by), col(kWhite, 0.95f), v, 0.04f);
            by += bs * 1.3f;
        };
        const float front = 0.5f + c.comForward / c.wheelBase;
        std::snprintf(line, sizeof(line), "%.0f kg, %.0f %% front", double(c.mass), double(100.0f * front));
        row("MASS", line);
        std::snprintf(line, sizeof(line), "%s%s%s", driveLayoutName(c.layout), c.layout == DriveLayout::PartTime ? "  " : "",
                      c.layout == DriveLayout::PartTime ? transferModeName(t.transfer) : "");
        row("DRIVETRAIN", line);
        if (c.isElectric()) {
            std::snprintf(line, sizeof(line), "%.0f / %.0f N*m, regen %.0f N*m", double(c.electric.motors[0].maxTorque),
                          double(c.electric.motors[1].maxTorque), double(c.electric.motors[0].regenTorque));
            row("MOTORS F / R", line);
        } else {
            std::snprintf(line, sizeof(line), "%.0f N*m, final %.2f", double(c.tuning.engineMaxTorque), double(c.tuning.finalDrive));
            row("ENGINE", line);
        }
        std::snprintf(line, sizeof(line), "%.0f / %.0f N*m per wheel", double(c.tuning.axles[0].brakeTorque),
                      double(c.tuning.axles[1].brakeTorque));
        row("BRAKES F / R", line);
        std::snprintf(line, sizeof(line), "FL %.0f  FR %.0f  RL %.0f  RR %.0f %%", double(100.0f * t.wheels[0].compressionRatio),
                      double(100.0f * t.wheels[1].compressionRatio), double(100.0f * t.wheels[2].compressionRatio),
                      double(100.0f * t.wheels[3].compressionRatio));
        row("COMPRESSION", line);
        if (c.isElectric()) {
            std::snprintf(line, sizeof(line), "%.0f rpm, %+.0f kW, SoC %.0f %%", double(std::max(t.ev.motorRPM[0], t.ev.motorRPM[1])),
                          double(t.ev.motorPowerKW), double(100.0f * t.ev.soc));
            row("MOTORS NOW", line);
        } else {
            std::snprintf(line, sizeof(line), "%.0f rpm, %.1f km/h", double(t.engineRPM), double(std::abs(t.forwardSpeed) * 3.6f));
            row("ENGINE NOW", line);
        }
    }

    // Hints.
    const float fs = 0.018f * H;
    const bool modal = g.modal() != GarageModal::None;
    const std::string south = padControlName(PadControl::South, view.padGlyphs);
    const std::string east = padControlName(PadControl::East, view.padGlyphs);
    const std::string ret = keyName(Key::Return, view.keyboard), esc = keyName(Key::Escape, view.keyboard);
    const std::string hintText = modal ? (view.gamepad ? south + " CONFIRM   " + east + " CANCEL" : ret + " CONFIRM   " + esc + " CANCEL")
                                 : view.gamepad ? "D-PAD CHANGE   " + south + " SELECT   " + east + " BACK"
                                                : "ARROWS / W S A D CHANGE   " + ret + " SELECT   MOUSE CLICK / DRAG   " + esc + " BACK";
    const char* hint = hintText.c_str();
    text(dl, m_body, fs, ImVec2(W - std::max(0.05f * W, 40.0f) - textWidth(m_body, fs, hint, 0.15f), H - 0.055f * H),
         col(kWhite, 0.75f), hint, 0.15f);
    char version[64];
    std::snprintf(version, sizeof(version), "TRAILFORGE %s", view.version);
    text(dl, m_body, fs, ImVec2(x0, H - 0.05f * H), col(kWhite, 0.4f), version, 0.2f);

    m_modalButtons.clear();
    if (!modal) return;
    ModalSpec s;
    s.focus = g.modalFocus();
    const std::string name = g.activeName();
    switch (g.modal()) {
    case GarageModal::Rename:
        s.title = "RENAME SETUP";
        s.message = "NEW NAME FOR '" + name + "'";
        s.textField = true;
        s.text = g.renameText();
        s.selected = g.renameSelected();
        s.error = g.modalError();
        s.hint = view.gamepad ? "TYPE WITH A KEYBOARD.  " + south + " KEEPS THE CURRENT NAME, " + east + " CANCELS"
                              : "LETTERS, DIGITS, SPACE AND - _ . ( ) +, UP TO 32.  " + ret + " CONFIRM, " + esc + " CANCEL";
        s.confirm = "CONFIRM";
        break;
    case GarageModal::ConfirmDelete:
        s.title = "DELETE SETUP?";
        s.message = "'" + name + "' WILL BE REMOVED FROM THE SETUP FILE. THIS CANNOT BE UNDONE.";
        s.confirm = "DELETE";
        break;
    case GarageModal::ConfirmReset:
        s.title = "RESET TO FACTORY?";
        s.message = g.factoryActive() ? "ALL VALUES GO BACK TO THE VEHICLE DEFINITION."
                                     : "ALL VALUES GO BACK TO THE DEFINITION. '" + name + "' CHANGES ONLY WHEN SAVED.";
        s.confirm = "RESET";
        break;
    case GarageModal::None: break;
    }
    drawModal(s);
}

void MenuUI::drawModal(const ModalSpec& s) {
    // Over a dimmed screen: dark panel, thin lines, spaced uppercase title with an orange accent, optional text field
    // with selection and caret, an error line, Confirm / Cancel buttons (the focused one filled).
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const ImVec2 ds = ImGui::GetIO().DisplaySize;
    const float W = ds.x, H = ds.y;
    dl->AddRectFilled(ImVec2(0, 0), ds, col(kDark, 0.62f));
    const float pw = std::clamp(0.40f * W, std::min(560.0f, 0.92f * W), 0.92f * W), pad = 0.028f * H;
    const float ts = 0.034f * H, bs = 0.020f * H, fs = 0.027f * H, btnH = 0.050f * H;
    float ph = pad + ts + 0.02f * H + bs * 1.6f + (s.textField ? fs * 1.9f + 0.012f * H : 0.0f) + bs * 1.5f + btnH +
               (s.hint.empty() ? 0.0f : bs * 1.6f) + pad;
    const ImVec2 a(0.5f * (W - pw), 0.5f * (H - ph)), b(a.x + pw, a.y + ph);
    dl->AddRectFilled(a, b, col(kDark, 0.96f), 4.0f);
    dl->AddRect(a, b, col(kWhite, 0.22f), 4.0f, 0, 1.0f);
    dl->AddRectFilled(a, ImVec2(a.x + 0.22f * pw, a.y + 3.0f), col(kOrange, 1.0f));
    const float x = a.x + pad;
    float y = a.y + pad;
    text(dl, m_heading, ts, ImVec2(x, y), col(kWhite, 1.0f), s.title, 0.22f);
    y += ts + 0.012f * H;
    dl->AddRectFilled(ImVec2(x, y), ImVec2(b.x - pad, y + 1.0f), col(kWhite, 0.25f));
    y += 0.008f * H;
    text(dl, m_body, bs, ImVec2(x, y + 0.2f * bs), col(kWhite, 0.75f), s.message.c_str(), 0.06f);
    y += bs * 1.6f;
    if (s.textField) {
        y += 0.012f * H;
        const ImVec2 fa(x, y), fb(b.x - pad, y + fs * 1.6f);
        dl->AddRectFilled(fa, fb, col(ImVec4(0.0f, 0.0f, 0.0f, 1.0f), 0.55f), 3.0f);
        dl->AddRect(fa, fb, col(kOrange, 1.0f), 3.0f, 0, 1.5f);
        const float tx = fa.x + 0.4f * fs, ty = fa.y + 0.3f * fs;
        const float tw = textWidth(m_body, fs, s.text.c_str(), 0.04f);
        if (s.selected && !s.text.empty())
            dl->AddRectFilled(ImVec2(tx - 2.0f, ty - 1.0f), ImVec2(tx + tw + 2.0f, ty + fs + 1.0f), col(kOrange, 0.45f), 2.0f);
        text(dl, m_body, fs, ImVec2(tx, ty), col(kWhite, 1.0f), s.text.c_str(), 0.04f);
        if (std::fmod(ImGui::GetTime(), 1.0) < 0.6)
            dl->AddRectFilled(ImVec2(tx + tw + 3.0f, ty), ImVec2(tx + tw + 5.0f, ty + fs), col(kWhite, 1.0f));
        y = fb.y;
    }
    if (!s.error.empty())
        text(dl, m_body, bs, ImVec2(x, y + 0.25f * bs), col(ImVec4(1.0f, 0.36f, 0.26f, 1.0f), 1.0f), s.error.c_str(), 0.06f);
    y += bs * 1.5f;
    // Buttons, right aligned: Confirm then Cancel.
    const char* labels[2] = {s.confirm, "CANCEL"};
    const float bw = std::max(0.16f * pw, textWidth(m_heading, bs, s.confirm, 0.2f) + 2.0f * pad);
    float bx = b.x - pad - 2.0f * bw - 0.015f * W;
    for (int i = 0; i < 2; ++i) {
        const ImVec2 ba(bx, y), bb(bx + bw, y + btnH);
        const bool f = s.focus == i;
        if (f) dl->AddRectFilled(ba, bb, col(kOrange, 1.0f), 3.0f);
        else dl->AddRect(ba, bb, col(kWhite, 0.45f), 3.0f, 0, 1.0f);
        const float lw = textWidth(m_heading, bs, labels[i], 0.2f);
        text(dl, m_heading, bs, ImVec2(bx + 0.5f * (bw - lw), y + 0.5f * (btnH - bs)), f ? col(kDark, 1.0f) : col(kWhite, 0.9f),
             labels[i], 0.2f);
        m_modalButtons.push_back({ba.x, ba.y, bb.x, bb.y});
        bx += bw + 0.015f * W;
    }
    y += btnH;
    if (!s.hint.empty()) text(dl, m_body, bs * 0.85f, ImVec2(x, y + 0.6f * bs), col(kWhite, 0.5f), s.hint.c_str(), 0.08f);
}

int MenuUI::modalHitTest(float x, float y) const {
    for (std::size_t i = 0; i < m_modalButtons.size(); ++i) {
        const auto& r = m_modalButtons[i];
        if (x >= r[0] && x < r[2] && y >= r[1] && y < r[3]) return int(i);
    }
    return -1;
}

bool MenuUI::modalButtonRect(int button, float& x0, float& y0, float& x1, float& y1) const {
    if (button < 0 || button >= int(m_modalButtons.size())) return false;
    const auto& r = m_modalButtons[std::size_t(button)];
    x0 = r[0];
    y0 = r[1];
    x1 = r[2];
    y1 = r[3];
    return true;
}

int MenuUI::garageHitTest(float x, float y, int* dir, float* fraction) const {
    if (dir) *dir = 0;
    if (fraction) *fraction = -1.0f;
    for (const GarageHit& r : m_garageRows) {
        if (y < r.y0 || y >= r.y1) continue;
        if (r.bar1 > r.bar0 && x >= r.bar0 && x < r.bar1) {
            if (fraction) *fraction = std::clamp((x - (r.bar0 + 8.0f)) / (r.bar1 - r.bar0 - 16.0f), 0.0f, 1.0f);
            return r.row;
        }
        const bool onLeft = r.arrowL1 > r.arrowL0 && x >= r.arrowL0 && x < r.arrowL1;
        const bool onRight = r.arrowR1 > r.arrowR0 && x >= r.arrowR0 && x < r.arrowR1;
        if (onLeft || onRight) {
            if (dir) *dir = onLeft ? -1 : 1;
            return r.row;
        }
        if (x >= r.x0 && x < r.x1) return r.row;
    }
    return -1;
}

bool MenuUI::garageRowRect(int row, float& x0, float& y0, float& x1, float& y1) const {
    for (const GarageHit& r : m_garageRows) {
        if (r.row != row) continue;
        x0 = r.x0;
        y0 = r.y0;
        x1 = r.x1;
        y1 = r.y1;
        return true;
    }
    return false;
}

} // namespace tf
