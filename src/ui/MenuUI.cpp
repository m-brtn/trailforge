#include "ui/MenuUI.h"

#include "render/Renderer.h"

#include <SDL3/SDL_surface.h>
#include <imgui.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>

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

struct ControlRow {
    const char* action;
    const char* keys;
    const char* pad;
};

constexpr ControlRow kControls[] = {
    {"THROTTLE / BRAKE", "W / S", "RT / LT"},
    {"STEER", "A / D", "LEFT STICK"},
    {"HANDBRAKE", "SPACE", "A"},
    {"FORWARD / REVERSE", "Q", "X"},
    {"LOW RANGE", "L", "Y"},
    {"ABS / TRACTION / DIFF LOCKS", "B / T / X", "D-PAD UP / RIGHT / DOWN"},
    {"CAMERA", "C, RIGHT MOUSE, WHEEL", "RB"},
    {"HEADLIGHTS", "H", "-"},
    {"RESET VEHICLE", "R", "BACK / VIEW"},
    {"PAUSE MENU", "ESC", "START / MENU"},
    {"MENUS", "ARROWS / W S A D, ENTER, ESC", "D-PAD / STICK, A, B"},
    {"DIAGNOSTICS (F1 / F2 / F3)", "F1 / F2 / F3", "-"},
    {"DEBUG FREEZE / SINGLE STEP", "P / N", "-"},
};

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
    if (!flow.menuActive()) {
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
    drawLogo(menuLogoC.x, menuLogoC.y, menuLogoW, 1.0f);
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
    const float shade = paused ? 0.45f : 0.62f;
    const float gradW = 0.62f * W;
    dl->AddRectFilledMultiColor(ImVec2(0, 0), ImVec2(gradW, H), col(kDark, shade * alpha), 0, 0, col(kDark, shade * alpha));
    dl->AddRectFilledMultiColor(ImVec2(0, H * 0.82f), ImVec2(W, H), 0, 0, col(kDark, 0.5f * alpha), col(kDark, 0.5f * alpha));

    const float logoW = std::min(0.27f * W, 0.75f * H);
    const float logoBottom = 0.08f * H + logoW / m_logoAspect;
    const bool compact = page == MenuPage::Settings || page == MenuPage::Controls;
    float y = logoBottom + 0.05f * H;

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
    if (int(page) != m_lastPage) {
        m_selectAnim = float(cursor);
        m_lastPage = int(page);
    }
    m_selectAnim += (float(cursor) - m_selectAnim) * std::min(1.0f, ImGui::GetIO().DeltaTime * 18.0f);

    const float textSize = compact ? 0.027f * H : 0.036f * H;
    const float rowH = compact ? 0.040f * H : 0.064f * H;
    const float barW = page == MenuPage::Settings ? 0.46f * W : 0.34f * W;
    const float valueX = x0 + 0.36f * W; // centre of the value column (Settings)
    const float arrowDx = 0.075f * W;

    if (page == MenuPage::Controls) {
        drawControls(x0, y, barW, 0.034f * H, alpha);
        y += 0.034f * H * float(std::size(kControls) + 1) + 0.02f * H;
    }

    // Row positions first (section headings take space), then the highlight bar under them.
    std::vector<float> rowY(items.size());
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (page == MenuPage::Settings && menuSection(items[i])) y += 0.020f * H + (i > 0 ? 0.014f * H : 0.0f);
        if (items[i] == MenuItem::Controls) y += 0.016f * H;
        rowY[i] = y;
        y += rowH;
    }
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
            const std::string v = menuValue(item, settings);
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
        m_rows.push_back(row);
    }

    // Footer: version bottom-left, input hints bottom-right.
    const float fs = 0.018f * H;
    char version[64];
    std::snprintf(version, sizeof(version), "TRAILFORGE %s", view.version);
    text(dl, m_body, fs, ImVec2(x0, H - 0.05f * H), col(kWhite, 0.4f * alpha), version, 0.2f);
    drawHints(flow, view, alpha);
}

void MenuUI::drawControls(float x, float y, float w, float rowH, float alpha) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const float H = ImGui::GetIO().DisplaySize.y;
    const float ts = 0.020f * H;
    float wAction = 0.0f, wKeys = textWidth(m_heading, ts * 0.9f, "KEYBOARD / MOUSE", 0.3f), wPad = 0.0f;
    for (const ControlRow& r : kControls) {
        wAction = std::max(wAction, textWidth(m_body, ts, r.action, 0.05f));
        wKeys = std::max(wKeys, textWidth(m_body, ts, r.keys, 0.05f));
        wPad = std::max(wPad, textWidth(m_body, ts, r.pad, 0.05f));
    }
    const float gap = 0.03f * H;
    const float cKeys = x + wAction + gap, cPad = cKeys + wKeys + gap;
    w = std::max(w, cPad + wPad - x) / 1.12f;
    text(dl, m_heading, ts * 0.9f, ImVec2(x, y), col(kOrange, alpha), "ACTION", 0.3f);
    text(dl, m_heading, ts * 0.9f, ImVec2(cKeys, y), col(kOrange, alpha), "KEYBOARD / MOUSE", 0.3f);
    text(dl, m_heading, ts * 0.9f, ImVec2(cPad, y), col(kOrange, alpha), "GAMEPAD", 0.3f);
    y += rowH;
    for (const ControlRow& r : kControls) {
        dl->AddRectFilled(ImVec2(x, y - 0.004f * H), ImVec2(x + w * 1.12f, y - 0.004f * H + 1.0f), col(kWhite, 0.08f * alpha));
        text(dl, m_body, ts, ImVec2(x, y), col(kWhite, 0.85f * alpha), r.action, 0.05f);
        text(dl, m_body, ts, ImVec2(cKeys, y), col(kWhite, 0.6f * alpha), r.keys, 0.05f);
        text(dl, m_body, ts, ImVec2(cPad, y), col(kWhite, 0.6f * alpha), r.pad, 0.05f);
        y += rowH;
    }
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
        hints.push_back({"A", "SELECT"});
        if (flow.depth() > 1 || flow.screen() == Screen::Paused) hints.push_back({"B", "BACK"});
    } else {
        if (adjustable) hints.push_back({"A / D", "CHANGE"});
        hints.push_back({"ENTER", "SELECT"});
        if (flow.depth() > 1 || flow.screen() == Screen::Paused) hints.push_back({"ESC", "BACK"});
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

} // namespace tf
