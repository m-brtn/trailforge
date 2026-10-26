#include "ui/MapUI.h"

#include "render/Renderer.h"

#include <imgui.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <vector>

namespace tf {
namespace {

constexpr float kDegToRad = 0.0174532925f;

constexpr ImU32 kNavy = IM_COL32(13, 21, 35, 255);
constexpr ImU32 kPanel = IM_COL32(18, 29, 47, 255);
constexpr ImU32 kFrame = IM_COL32(74, 96, 128, 255);
constexpr ImU32 kOutsideFill = IM_COL32(20, 31, 48, 255);
constexpr ImU32 kTitle = IM_COL32(236, 240, 246, 255);
constexpr ImU32 kDim = IM_COL32(150, 165, 188, 255);
constexpr ImU32 kAccent = IM_COL32(232, 120, 40, 255);
constexpr ImU32 kPoiRed = IM_COL32(196, 48, 43, 255);
constexpr ImU32 kMarker = IM_COL32(255, 196, 40, 255);
constexpr ImU32 kRoadFill = IM_COL32(238, 214, 164, 255);
constexpr ImU32 kRoadCasing = IM_COL32(64, 46, 30, 230);
constexpr ImU32 kTrailFill = IM_COL32(196, 150, 98, 255);
constexpr ImU32 kTrailCasing = IM_COL32(60, 44, 28, 200);
constexpr ImU32 kRiver = IM_COL32(74, 150, 196, 255);
constexpr ImU32 kBridgeFill = IM_COL32(214, 214, 208, 255);
constexpr ImU32 kBridgeCasing = IM_COL32(30, 30, 34, 255);

ImU32 withAlpha(ImU32 c, float a) {
    const ImU32 alpha = ImU32(std::clamp(float((c >> IM_COL32_A_SHIFT) & 0xFF) * a, 0.0f, 255.0f));
    return (c & ~IM_COL32_A_MASK) | (alpha << IM_COL32_A_SHIFT);
}

ImFont* orDefault(ImFont* f) { return f ? f : ImGui::GetFont(); }

float textWidth(ImFont* font, float size, const char* s) { return orDefault(font)->CalcTextSizeA(size, FLT_MAX, 0.0f, s).x; }

// align: 0 left, 0.5 centered, 1 right (x is that point); y is the top. shadow: a dark copy 1 point down-right.
void text(ImDrawList* dl, ImFont* font, float size, float x, float y, ImU32 col, const char* s, float align = 0.0f,
          bool shadow = false) {
    ImFont* f = orDefault(font);
    const float w = align != 0.0f ? textWidth(f, size, s) : 0.0f;
    const ImVec2 p(std::round(x - align * w), std::round(y));
    if (shadow) dl->AddText(f, size, ImVec2(p.x + 1.0f, p.y + 1.0f), withAlpha(IM_COL32(0, 0, 0, 255), 0.75f * float((col >> IM_COL32_A_SHIFT) & 0xFF) / 255.0f), s);
    dl->AddText(f, size, p, col, s);
}

ImVec2 rotated(float x, float y, float deg) {
    const float a = deg * kDegToRad, c = std::cos(a), s = std::sin(a);
    return {x * c - y * s, x * s + y * c};
}

// Part [t0, t1] of the segment a-b inside the circle (c, r); false if it misses.
bool clipToCircle(ImVec2 a, ImVec2 b, ImVec2 c, float r, float& t0, float& t1) {
    const float dx = b.x - a.x, dy = b.y - a.y, fx = a.x - c.x, fy = a.y - c.y;
    const float A = dx * dx + dy * dy, B = 2.0f * (fx * dx + fy * dy), C = fx * fx + fy * fy - r * r;
    if (A < 1e-9f) {
        t0 = 0.0f;
        t1 = 1.0f;
        return C <= 0.0f;
    }
    const float disc = B * B - 4.0f * A * C;
    if (disc <= 0.0f) return false;
    const float q = std::sqrt(disc);
    t0 = std::max(0.0f, (-B - q) / (2.0f * A));
    t1 = std::min(1.0f, (-B + q) / (2.0f * A));
    return t0 < t1;
}

ImVec2 lerp(ImVec2 a, ImVec2 b, float t) { return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t}; }

// Polyline through screen points; with a circle, only the parts inside it (split into runs).
void polyline(ImDrawList* dl, const std::vector<ImVec2>& pts, ImU32 col, float width, const float* circle) {
    if (pts.size() < 2) return;
    if (!circle) {
        dl->AddPolyline(pts.data(), int(pts.size()), col, ImDrawFlags_None, width);
        return;
    }
    const ImVec2 c(circle[0], circle[1]);
    std::vector<ImVec2> run;
    auto flush = [&] {
        if (run.size() >= 2) dl->AddPolyline(run.data(), int(run.size()), col, ImDrawFlags_None, width);
        run.clear();
    };
    for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
        float t0 = 0.0f, t1 = 1.0f;
        if (!clipToCircle(pts[k], pts[k + 1], c, circle[2], t0, t1)) {
            flush();
            continue;
        }
        if (t0 > 0.0f || run.empty()) {
            flush();
            run.push_back(lerp(pts[k], pts[k + 1], t0));
        }
        run.push_back(lerp(pts[k], pts[k + 1], t1));
        if (t1 < 1.0f) flush();
    }
    flush();
}

void dashedLine(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col, float width, float dash) {
    const float len = std::hypot(b.x - a.x, b.y - a.y);
    if (len < 1.0f) return;
    for (float s = 0.0f; s < len; s += 2.0f * dash)
        dl->AddLine(lerp(a, b, s / len), lerp(a, b, std::min(s + dash, len) / len), col, width);
}

} // namespace

MapUI::~MapUI() { release(); }

void MapUI::release() {
    if (m_renderer && m_texture) m_renderer->releaseImage(m_texture);
    m_texture = nullptr;
}

void MapUI::setMap(Renderer& renderer, MapData data) {
    release();
    m_renderer = &renderer;
    m_data = std::move(data);
    if (m_data.raster.size > 0)
        m_texture = renderer.createImage(m_data.raster.rgba.data(), std::uint32_t(m_data.raster.size),
                                         std::uint32_t(m_data.raster.size));
    m_data.raster.rgba.clear();
    m_data.raster.rgba.shrink_to_fit(); // the GPU copy is the one that is drawn
}

MapUI::Layout MapUI::layout(float w, float h) {
    const float m = std::clamp(0.025f * h, 14.0f, 32.0f);
    const float side = std::clamp(0.27f * w, 280.0f, 440.0f);
    const float footer = std::clamp(0.05f * h, 30.0f, 48.0f);
    Layout l;
    l.sideX1 = w - m;
    l.sideX0 = l.sideX1 - side;
    l.mapX0 = m;
    l.mapY0 = m;
    l.mapX1 = l.sideX0 - m;
    l.mapY1 = h - m - footer;
    return l;
}

MapView MapUI::fitView(const Layout& l) const {
    MapView v;
    v.cx = 0.5f * (m_data.playX0 + m_data.playX1);
    v.cz = 0.5f * (m_data.playZ0 + m_data.playZ1);
    v.sx = 0.5f * (l.mapX0 + l.mapX1);
    v.sy = 0.5f * (l.mapY0 + l.mapY1);
    const float w = std::max(m_data.playX1 - m_data.playX0, 1.0f), d = std::max(m_data.playZ1 - m_data.playZ0, 1.0f);
    v.scale = 0.97f * std::min((l.mapX1 - l.mapX0) / w, (l.mapY1 - l.mapY0) / d);
    return v;
}

float MapUI::minScale(const Layout& l) const { return 0.6f * fitView(l).scale; }

float MapUI::maxScale() const { return 6.5f / std::max(m_data.raster.metersPerPixel, 0.01f); }

void MapUI::drawRaster(ImDrawList* dl, const MapView& v, float x0, float y0, float x1, float y1) const {
    if (!m_texture) return;
    const MapRaster& r = m_data.raster;
    const float S = float(r.size) * r.metersPerPixel;
    const MapPoint a = v.toScreen(r.x0, r.z0), b = v.toScreen(r.x0 + S, r.z0 + S);
    const float ix0 = std::max(x0, a.x), iy0 = std::max(y0, a.y), ix1 = std::min(x1, b.x), iy1 = std::min(y1, b.y);
    if (ix0 >= ix1 || iy0 >= iy1) return;
    const MapPoint w0 = v.toWorld(ix0, iy0), w1 = v.toWorld(ix1, iy1);
    dl->AddImage(ImTextureRef(ImTextureID(reinterpret_cast<std::intptr_t>(m_texture))), ImVec2(ix0, iy0), ImVec2(ix1, iy1),
                 ImVec2((w0.x - r.x0) / S, (w0.y - r.z0) / S), ImVec2((w1.x - r.x0) / S, (w1.y - r.z0) / S));
}

void MapUI::drawLines(ImDrawList* dl, const MapView& v, float lineScale, const float* circle) const {
    std::vector<ImVec2> pts;
    // Points far outside the circle are skipped (a long road outside the minimap costs nothing).
    const float reach = circle ? circle[2] * 1.5f + 60.0f : 0.0f;
    auto project = [&](const MapLine& line) {
        pts.clear();
        for (const RoutePoint& p : line.points) {
            const MapPoint s = v.toScreen(p.x, p.z);
            if (circle && std::hypot(s.x - circle[0], s.y - circle[1]) > reach) {
                if (!pts.empty() && pts.back().x != FLT_MAX) pts.push_back(ImVec2(FLT_MAX, FLT_MAX));
                continue;
            }
            pts.push_back(ImVec2(s.x, s.y));
        }
    };
    auto stroke = [&](ImU32 col, float width) {
        // FLT_MAX entries split the line where it left the reach.
        std::vector<ImVec2> part;
        for (const ImVec2& p : pts) {
            if (p.x == FLT_MAX) {
                polyline(dl, part, col, width, circle);
                part.clear();
            } else {
                part.push_back(p);
            }
        }
        polyline(dl, part, col, width, circle);
    };
    for (const MapLine& line : m_data.lines)
        if (line.kind == MapLineKind::River) {
            project(line);
            stroke(kRiver, std::max(2.0f, line.width * lineScale));
        }
    for (int pass = 0; pass < 2; ++pass)
        for (const MapLine& line : m_data.lines) {
            if (line.kind != MapLineKind::Road && line.kind != MapLineKind::Trail) continue;
            const bool trail = line.kind == MapLineKind::Trail;
            const float w = std::max(trail ? 1.6f : 2.4f, (trail ? 0.8f : 1.0f) * line.width * lineScale);
            project(line);
            if (pass == 0) stroke(trail ? kTrailCasing : kRoadCasing, w + 2.0f);
            else stroke(trail ? kTrailFill : kRoadFill, w);
        }
    for (const MapLine& line : m_data.lines)
        if (line.kind == MapLineKind::Bridge) {
            project(line);
            const float w = std::max(3.0f, line.width * lineScale);
            stroke(kBridgeCasing, w + 4.0f);
            stroke(kBridgeFill, w);
        }
}

void MapUI::drawPoi(ImDrawList* dl, float x, float y, int id, float r, bool hover) const {
    if (hover) dl->AddCircleFilled(ImVec2(x, y), r + 4.0f, withAlpha(kAccent, 0.85f));
    dl->AddCircleFilled(ImVec2(x, y), r + 1.5f, IM_COL32(255, 255, 255, 235));
    dl->AddCircleFilled(ImVec2(x, y), r, kPoiRed);
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%d", id);
    const float size = std::round(r * (id >= 10 ? 1.15f : 1.35f));
    text(dl, m_heading, size, x, y - 0.55f * size, IM_COL32(255, 255, 255, 255), buf, 0.5f);
}

void MapUI::drawMarkerPin(ImDrawList* dl, float x, float y, float s) const {
    const ImVec2 head(x, y - 1.7f * s);
    dl->AddTriangleFilled(ImVec2(x - 0.62f * s, head.y + 0.45f * s), ImVec2(x + 0.62f * s, head.y + 0.45f * s), ImVec2(x, y),
                          IM_COL32(40, 30, 10, 255));
    dl->AddCircleFilled(head, s + 1.2f, IM_COL32(40, 30, 10, 255));
    dl->AddTriangleFilled(ImVec2(x - 0.5f * s, head.y + 0.4f * s), ImVec2(x + 0.5f * s, head.y + 0.4f * s), ImVec2(x, y - 1.5f),
                          kMarker);
    dl->AddCircleFilled(head, s, kMarker);
    dl->AddCircleFilled(head, 0.38f * s, IM_COL32(40, 30, 10, 255));
}

void MapUI::drawPlayerArrow(ImDrawList* dl, float x, float y, float angleDeg, float s) const {
    const ImVec2 p[4] = {rotated(0.0f, -1.25f * s, angleDeg), rotated(0.8f * s, 0.9f * s, angleDeg),
                         rotated(0.0f, 0.4f * s, angleDeg), rotated(-0.8f * s, 0.9f * s, angleDeg)};
    ImVec2 q[4];
    for (int i = 0; i < 4; ++i) q[i] = ImVec2(x + p[i].x, y + p[i].y);
    dl->AddCircleFilled(ImVec2(x, y), 1.6f * s, IM_COL32(255, 255, 255, 40));
    dl->AddTriangleFilled(q[0], q[1], q[2], IM_COL32(255, 255, 255, 255));
    dl->AddTriangleFilled(q[0], q[2], q[3], IM_COL32(255, 255, 255, 255));
    dl->AddPolyline(q, 4, IM_COL32(20, 24, 30, 255), ImDrawFlags_Closed, 1.8f);
}

int MapUI::poiAt(const MapView& v, float x, float y, float radius) const {
    int best = -1;
    float bestD = radius;
    for (std::size_t k = 0; k < m_data.pois.size(); ++k) {
        const MapPoint s = v.toScreen(m_data.pois[k].x, m_data.pois[k].z);
        const float d = std::hypot(s.x - x, s.y - y);
        if (d <= bestD) {
            bestD = d;
            best = int(k);
        }
    }
    return best;
}

int MapUI::listRowAt(float x, float y) const {
    for (const ListRow& r : m_listRows)
        if (x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1) return r.poi;
    return -1;
}

void MapUI::drawFullMap(const MapView& v, const MapMarker& marker, const MapPlayer& player, const MapHints& hints,
                        int hoverPoi, bool crosshair) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const ImVec2 D = ImGui::GetIO().DisplaySize;
    const Layout l = layout(D.x, D.y);
    const float H = D.y;
    dl->AddRectFilled(ImVec2(0, 0), D, kNavy);

    // ---- Map panel
    dl->AddRectFilled(ImVec2(l.mapX0, l.mapY0), ImVec2(l.mapX1, l.mapY1), kOutsideFill);
    dl->PushClipRect(ImVec2(l.mapX0, l.mapY0), ImVec2(l.mapX1, l.mapY1), true);
    drawRaster(dl, v, l.mapX0, l.mapY0, l.mapX1, l.mapY1);
    drawLines(dl, v, v.scale, nullptr);
    // Play area boundary.
    {
        const MapPoint a = v.toScreen(m_data.playX0, m_data.playZ0), b = v.toScreen(m_data.playX1, m_data.playZ1);
        dl->AddRect(ImVec2(a.x, a.y), ImVec2(b.x, b.y), IM_COL32(255, 255, 255, 70), 0.0f, 0, 1.5f);
    }
    const MapPoint me = v.toScreen(player.x, player.z);
    if (marker.set) {
        const MapPoint mk = v.toScreen(marker.x, marker.z);
        dashedLine(dl, ImVec2(me.x, me.y), ImVec2(mk.x, mk.y), withAlpha(kMarker, 0.85f), 2.0f, 7.0f);
    }
    const float poiR = std::clamp(0.0135f * H, 9.0f, 15.0f);
    const float label = std::clamp(0.017f * H, 12.0f, 19.0f);
    // Labels go right of their circle, else left; a label that would cover another label or circle is left out
    // (the number and the side list still name it; the hovered one always shows).
    std::vector<ImVec4> taken;
    for (const MapPoi& p : m_data.pois) {
        const MapPoint s = v.toScreen(p.x, p.z);
        taken.emplace_back(s.x - poiR, s.y - poiR, s.x + poiR, s.y + poiR);
    }
    auto overlaps = [&](const ImVec4& r, std::size_t self) {
        for (std::size_t i = 0; i < taken.size(); ++i)
            if (i != self && r.x < taken[i].z && r.z > taken[i].x && r.y < taken[i].w && r.w > taken[i].y) return true;
        return false;
    };
    for (std::size_t k = 0; k < m_data.pois.size(); ++k) {
        const MapPoi& p = m_data.pois[k];
        const MapPoint s = v.toScreen(p.x, p.z);
        drawPoi(dl, s.x, s.y, p.id, poiR, int(k) == hoverPoi);
        const float w = textWidth(m_body, label, p.name.c_str()), y0 = s.y - 0.55f * label, y1 = y0 + 1.1f * label;
        const ImVec4 right(s.x + poiR + 5.0f, y0, s.x + poiR + 5.0f + w, y1), left(s.x - poiR - 5.0f - w, y0, s.x - poiR - 5.0f, y1);
        const bool hovered = int(k) == hoverPoi;
        const ImVec4* at = !overlaps(right, k) || hovered ? &right : !overlaps(left, k) ? &left : nullptr;
        if (!at) continue;
        taken.push_back(*at);
        text(dl, m_body, label, at->x, y0, IM_COL32(255, 255, 255, 240), p.name.c_str(), 0.0f, true);
    }
    if (marker.set) {
        const MapPoint mk = v.toScreen(marker.x, marker.z);
        drawMarkerPin(dl, mk.x, mk.y, std::clamp(0.011f * H, 8.0f, 13.0f));
        const std::string d = formatDistance(std::hypot(marker.x - player.x, marker.z - player.z));
        text(dl, m_heading, label, mk.x, mk.y + 4.0f, kMarker, d.c_str(), 0.5f, true);
    }
    drawPlayerArrow(dl, me.x, me.y, player.headingDeg - v.rotationDeg, std::clamp(0.012f * H, 8.0f, 14.0f));
    if (crosshair) {
        const float cx = 0.5f * (l.mapX0 + l.mapX1), cy = 0.5f * (l.mapY0 + l.mapY1), s = 0.012f * H;
        for (const ImU32 c : {IM_COL32(0, 0, 0, 160), IM_COL32(255, 255, 255, 230)}) {
            const float w = c == IM_COL32(0, 0, 0, 160) ? 3.5f : 1.5f;
            dl->AddLine(ImVec2(cx - 2.0f * s, cy), ImVec2(cx - 0.5f * s, cy), c, w);
            dl->AddLine(ImVec2(cx + 0.5f * s, cy), ImVec2(cx + 2.0f * s, cy), c, w);
            dl->AddLine(ImVec2(cx, cy - 2.0f * s), ImVec2(cx, cy - 0.5f * s), c, w);
            dl->AddLine(ImVec2(cx, cy + 0.5f * s), ImVec2(cx, cy + 2.0f * s), c, w);
        }
    }
    // Scale bar (bottom left) and north arrow (top right).
    {
        const float pad = 0.02f * H;
        const float meters = scaleBarMeters(v.scale, 0.16f * (l.mapX1 - l.mapX0));
        const float len = meters * v.scale;
        const float x = l.mapX0 + pad, y = l.mapY1 - pad;
        dl->AddRectFilled(ImVec2(x - 8.0f, y - 2.2f * label - 6.0f), ImVec2(x + len + 8.0f, y + 6.0f), IM_COL32(10, 16, 28, 170), 4.0f);
        dl->AddLine(ImVec2(x, y), ImVec2(x + len, y), IM_COL32(255, 255, 255, 255), 3.0f);
        dl->AddLine(ImVec2(x, y - 6.0f), ImVec2(x, y + 1.5f), IM_COL32(255, 255, 255, 255), 2.0f);
        dl->AddLine(ImVec2(x + len, y - 6.0f), ImVec2(x + len, y + 1.5f), IM_COL32(255, 255, 255, 255), 2.0f);
        text(dl, m_body, label, x + 0.5f * len, y - 1.6f * label, IM_COL32(255, 255, 255, 255), formatDistance(meters).c_str(), 0.5f);
        const float r = 0.026f * H, nx = l.mapX1 - pad - r, ny = l.mapY0 + pad + r;
        dl->AddCircleFilled(ImVec2(nx, ny), r, IM_COL32(10, 16, 28, 190));
        dl->AddCircle(ImVec2(nx, ny), r, withAlpha(kFrame, 1.0f), 0, 1.5f);
        const ImVec2 tip = rotated(0.0f, -0.8f * r, -v.rotationDeg), l0 = rotated(-0.3f * r, 0.05f * r, -v.rotationDeg),
                     r0 = rotated(0.3f * r, 0.05f * r, -v.rotationDeg);
        dl->AddTriangleFilled(ImVec2(nx + tip.x, ny + tip.y), ImVec2(nx + l0.x, ny + l0.y), ImVec2(nx + r0.x, ny + r0.y), kAccent);
        text(dl, m_heading, 0.85f * r, nx, ny + 0.02f * r, kTitle, "N", 0.5f);
    }
    dl->PopClipRect();
    dl->AddRect(ImVec2(l.mapX0, l.mapY0), ImVec2(l.mapX1, l.mapY1), kFrame, 0.0f, 0, 1.5f);

    // ---- Side panel: title, points of interest, legend
    dl->AddRectFilled(ImVec2(l.sideX0, l.mapY0), ImVec2(l.sideX1, l.mapY1), kPanel, 6.0f);
    dl->AddRect(ImVec2(l.sideX0, l.mapY0), ImVec2(l.sideX1, l.mapY1), kFrame, 6.0f, 0, 1.0f);
    const float sp = 0.022f * H, x0 = l.sideX0 + sp, x1 = l.sideX1 - sp;
    float y = l.mapY0 + sp;
    std::string title = m_data.title;
    for (char& c : title) c = char(std::toupper(static_cast<unsigned char>(c)));
    const float titleSize = std::clamp(0.042f * H, 22.0f, 44.0f);
    text(dl, m_heading, titleSize, 0.5f * (x0 + x1), y, kTitle, title.c_str(), 0.5f);
    y += 1.05f * titleSize;
    text(dl, m_body, label, 0.5f * (x0 + x1), y, kDim, m_data.subtitle.c_str(), 0.5f);
    y += 1.6f * label;
    dl->AddLine(ImVec2(x0, y), ImVec2(x1, y), kFrame, 1.0f);
    y += 0.6f * label;
    text(dl, m_heading, label, x0, y, kAccent, "POINTS OF INTEREST");
    y += 1.5f * label;
    const float legendH = 4.3f * label;
    const float listBottom = l.mapY1 - sp - legendH;
    const std::size_t n = m_data.pois.size();
    const float rowH = n ? std::min(2.35f * label, (listBottom - y) / float(n)) : 0.0f;
    m_listRows.clear();
    for (std::size_t k = 0; k < n && rowH > 0.8f * label; ++k) {
        const MapPoi& p = m_data.pois[k];
        const float ry = y + float(k) * rowH;
        const bool hover = int(k) == hoverPoi;
        if (hover) dl->AddRectFilled(ImVec2(x0 - 6.0f, ry), ImVec2(x1 + 6.0f, ry + rowH - 2.0f), IM_COL32(255, 255, 255, 22), 4.0f);
        const float r = std::min(0.36f * rowH, poiR);
        drawPoi(dl, x0 + r, ry + 0.5f * rowH, p.id, r, false);
        const bool twoLines = rowH > 1.9f * label && !p.kind.empty();
        const float nameY = twoLines ? ry + 0.5f * rowH - 1.0f * label : ry + 0.5f * rowH - 0.55f * label;
        text(dl, m_body, label, x0 + 2.0f * r + 10.0f, nameY, hover ? kAccent : kTitle, p.name.c_str());
        if (twoLines) text(dl, m_body, 0.82f * label, x0 + 2.0f * r + 10.0f, nameY + 1.05f * label, kDim, p.kind.c_str());
        const std::string d = formatDistance(std::hypot(p.x - player.x, p.z - player.z));
        text(dl, m_body, 0.9f * label, x1, ry + 0.5f * rowH - 0.5f * label, kDim, d.c_str(), 1.0f);
        m_listRows.push_back({x0 - 6.0f, ry, x1 + 6.0f, ry + rowH, int(k)});
    }
    // Legend: two columns of line / symbol samples.
    {
        float ly = l.mapY1 - sp - legendH + 0.4f * label;
        dl->AddLine(ImVec2(x0, ly - 0.4f * label), ImVec2(x1, ly - 0.4f * label), kFrame, 1.0f);
        const float colW = 0.5f * (x1 - x0), sw = 1.8f * label, ls = 0.85f * label;
        auto entry = [&](int i, const char* name, auto&& sample) {
            const float ex = x0 + float(i % 2) * colW, ey = ly + float(i / 2) * 1.3f * label;
            sample(ex, ey + 0.55f * label);
            text(dl, m_body, ls, ex + sw + 8.0f, ey, kDim, name);
        };
        entry(0, "Road", [&](float x, float yy) {
            dl->AddLine(ImVec2(x, yy), ImVec2(x + sw, yy), kRoadCasing, 6.0f);
            dl->AddLine(ImVec2(x, yy), ImVec2(x + sw, yy), kRoadFill, 4.0f);
        });
        entry(1, "Trail", [&](float x, float yy) {
            dl->AddLine(ImVec2(x, yy), ImVec2(x + sw, yy), kTrailCasing, 4.5f);
            dl->AddLine(ImVec2(x, yy), ImVec2(x + sw, yy), kTrailFill, 2.5f);
        });
        entry(2, "River / lake", [&](float x, float yy) { dl->AddLine(ImVec2(x, yy), ImVec2(x + sw, yy), kRiver, 4.0f); });
        entry(3, "Bridge", [&](float x, float yy) {
            dl->AddLine(ImVec2(x, yy), ImVec2(x + sw, yy), kBridgeCasing, 8.0f);
            dl->AddLine(ImVec2(x, yy), ImVec2(x + sw, yy), kBridgeFill, 4.0f);
        });
        entry(4, "Waypoint", [&](float x, float yy) { drawMarkerPin(dl, x + 0.5f * sw, yy + 0.45f * label, 0.36f * label); });
        entry(5, "You", [&](float x, float yy) { drawPlayerArrow(dl, x + 0.5f * sw, yy, 90.0f, 0.42f * label); });
    }

    // ---- Footer: the controls of the device in use
    {
        const float fy = l.mapY1 + 0.5f * (H - l.mapY1) - 0.5f * label;
        const std::pair<const std::string*, const char*> items[] = {{&hints.zoom, "ZOOM"},
                                                                   {&hints.pan, "PAN"},
                                                                   {&hints.marker, "SET / CLEAR WAYPOINT"},
                                                                   {&hints.center, "CENTER ON VEHICLE"},
                                                                   {&hints.close, "CLOSE"}};
        const float ks = 0.85f * label, gap = 1.6f * label;
        float total = 0.0f;
        for (const auto& it : items) total += textWidth(m_heading, ks, it.first->c_str()) + 14.0f + textWidth(m_body, label, it.second) + gap;
        float x = 0.5f * (D.x - total + gap);
        for (const auto& it : items) {
            const float kw = textWidth(m_heading, ks, it.first->c_str()) + 10.0f;
            dl->AddRect(ImVec2(x, fy - 3.0f), ImVec2(x + kw, fy + label + 1.0f), IM_COL32(200, 210, 225, 200), 3.0f, 0, 1.2f);
            text(dl, m_heading, ks, x + 5.0f, fy + 0.1f * label, kTitle, it.first->c_str());
            x += kw + 6.0f;
            text(dl, m_body, label, x, fy - 1.0f, kDim, it.second);
            x += textWidth(m_body, label, it.second) + gap;
        }
    }
}

void MapUI::drawMinimap(float cx, float cy, float radius, float rangeM, bool northUp, const MapMarker& marker,
                        const MapPlayer& player) {
    if (!m_texture) return;
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    MapView v;
    v.cx = player.x;
    v.cz = player.z;
    v.sx = cx;
    v.sy = cy;
    v.scale = radius / rangeM;
    v.rotationDeg = northUp ? 0.0f : player.headingDeg;
    dl->AddCircleFilled(ImVec2(cx, cy), radius + 3.0f, IM_COL32(8, 12, 20, 200));
    // Textured disc: a triangle fan whose UVs come from the inverse view (exact: the transform is affine).
    {
        const MapRaster& r = m_data.raster;
        const float S = float(r.size) * r.metersPerPixel;
        auto uv = [&](float x, float y) {
            const MapPoint w = v.toWorld(x, y);
            return ImVec2((w.x - r.x0) / S, (w.y - r.z0) / S);
        };
        constexpr int kSeg = 72;
        dl->PushTexture(ImTextureRef(ImTextureID(reinterpret_cast<std::intptr_t>(m_texture))));
        dl->PrimReserve(kSeg * 3, kSeg + 1);
        const ImDrawIdx base = ImDrawIdx(dl->_VtxCurrentIdx);
        dl->PrimWriteVtx(ImVec2(cx, cy), uv(cx, cy), IM_COL32_WHITE);
        for (int i = 0; i < kSeg; ++i) {
            const float a = 2.0f * 3.14159265f * float(i) / float(kSeg);
            const ImVec2 p(cx + radius * std::cos(a), cy + radius * std::sin(a));
            dl->PrimWriteVtx(p, uv(p.x, p.y), IM_COL32_WHITE);
        }
        for (int i = 0; i < kSeg; ++i) {
            dl->PrimWriteIdx(base);
            dl->PrimWriteIdx(ImDrawIdx(base + 1 + i));
            dl->PrimWriteIdx(ImDrawIdx(base + 1 + (i + 1) % kSeg));
        }
        dl->PopTexture();
    }
    const float circle[3] = {cx, cy, radius};
    drawLines(dl, v, std::min(v.scale, 0.9f), circle);
    // Points of interest inside the disc.
    const float poiR = std::clamp(0.075f * radius, 6.0f, 10.0f);
    for (const MapPoi& p : m_data.pois) {
        const MapPoint s = v.toScreen(p.x, p.z);
        if (std::hypot(s.x - cx, s.y - cy) < radius - poiR) drawPoi(dl, s.x, s.y, p.id, poiR, false);
    }
    // Marker: inside the disc, or on the rim in its direction; a guidance line from the vehicle.
    if (marker.set) {
        MapPoint s = v.toScreen(marker.x, marker.z);
        const float dx = s.x - cx, dy = s.y - cy, d = std::hypot(dx, dy);
        const bool inside = d < radius - 6.0f;
        if (!inside && d > 1e-3f) s = {cx + dx / d * (radius - 6.0f), cy + dy / d * (radius - 6.0f)};
        dashedLine(dl, ImVec2(cx, cy), ImVec2(s.x, s.y), withAlpha(kMarker, 0.9f), 2.0f, 5.0f);
        if (inside) {
            drawMarkerPin(dl, s.x, s.y, 0.065f * radius + 3.0f);
        } else {
            const float a = std::atan2(dy, dx);
            const ImVec2 tip(cx + std::cos(a) * (radius + 2.0f), cy + std::sin(a) * (radius + 2.0f));
            const ImVec2 b0(cx + std::cos(a + 0.09f) * (radius - 12.0f), cy + std::sin(a + 0.09f) * (radius - 12.0f));
            const ImVec2 b1(cx + std::cos(a - 0.09f) * (radius - 12.0f), cy + std::sin(a - 0.09f) * (radius - 12.0f));
            dl->AddTriangleFilled(tip, b0, b1, kMarker);
            dl->AddTriangle(tip, b0, b1, IM_COL32(40, 30, 10, 255), 1.5f);
        }
    }
    dl->AddCircle(ImVec2(cx, cy), radius, IM_COL32(255, 255, 255, 200), 72, 2.0f);
    // North on the rim.
    {
        const ImVec2 n = rotated(0.0f, -radius, -v.rotationDeg);
        const float s = std::clamp(0.13f * radius, 11.0f, 17.0f);
        dl->AddCircleFilled(ImVec2(cx + n.x, cy + n.y), 0.75f * s, IM_COL32(10, 16, 28, 235));
        text(dl, m_heading, s, cx + n.x, cy + n.y - 0.52f * s, kAccent, "N", 0.5f);
    }
    drawPlayerArrow(dl, cx, cy, player.headingDeg - v.rotationDeg, std::clamp(0.075f * radius, 6.0f, 11.0f));
    if (marker.set) {
        const std::string d = formatDistance(std::hypot(marker.x - player.x, marker.z - player.z));
        const float s = std::clamp(0.14f * radius, 13.0f, 19.0f);
        text(dl, m_heading, s, cx, cy + radius + 6.0f, kMarker, d.c_str(), 0.5f, true);
    }
}

void MapUI::drawCompass(float cx, float y, float width, float poiRangeM, const MapMarker& marker, const MapPlayer& player) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const float h = std::clamp(0.075f * width, 30.0f, 42.0f);
    const float x0 = cx - 0.5f * width, x1 = cx + 0.5f * width;
    constexpr float kHalfSpan = 90.0f;
    dl->AddRectFilled(ImVec2(x0, y), ImVec2(x1, y + h), IM_COL32(8, 12, 20, 150), 6.0f);
    dl->PushClipRect(ImVec2(x0, y), ImVec2(x1, y + h + 40.0f), true);
    const float labelSize = 0.42f * h;
    for (int b = 0; b < 360; b += 15) {
        const auto off = compassOffset(float(b), player.headingDeg, kHalfSpan);
        if (!off) continue;
        const float x = cx + *off * 0.5f * width;
        const float fade = 1.0f - 0.7f * std::abs(*off);
        if (b % 45 == 0) {
            const char* name = compassPointName(float(b));
            text(dl, m_heading, b % 90 == 0 ? labelSize * 1.1f : labelSize * 0.9f, x, y + 0.08f * h,
                 withAlpha(b == 0 ? kAccent : IM_COL32(255, 255, 255, 255), fade), name, 0.5f);
            dl->AddLine(ImVec2(x, y + 0.72f * h), ImVec2(x, y + h - 2.0f), withAlpha(IM_COL32(255, 255, 255, 230), fade), 2.0f);
        } else {
            dl->AddLine(ImVec2(x, y + 0.8f * h), ImVec2(x, y + h - 2.0f), withAlpha(IM_COL32(255, 255, 255, 170), fade), 1.2f);
        }
    }
    // Nearby points of interest under the tape.
    const float poiR = 0.19f * h;
    for (const MapPoi& p : m_data.pois) {
        const float d = std::hypot(p.x - player.x, p.z - player.z);
        if (d > poiRangeM || d < 1.0f) continue;
        const auto off = compassOffset(bearingDeg(player.x, player.z, p.x, p.z), player.headingDeg, kHalfSpan);
        if (off) drawPoi(dl, cx + *off * 0.5f * width, y + 0.6f * h, p.id, poiR, false);
    }
    dl->PopClipRect();
    // Heading pointer.
    dl->AddTriangleFilled(ImVec2(cx - 6.0f, y + h + 7.0f), ImVec2(cx + 6.0f, y + h + 7.0f), ImVec2(cx, y + h - 1.0f),
                          IM_COL32(255, 255, 255, 230));
    // The marker: on the tape, or clamped to the nearer end with an arrow; its distance below.
    if (marker.set) {
        const float bearing = bearingDeg(player.x, player.z, marker.x, marker.z);
        const float rel = relativeBearingDeg(bearing, player.headingDeg);
        const float off = std::clamp(rel / kHalfSpan, -1.0f, 1.0f);
        const float x = cx + off * (0.5f * width - 10.0f);
        const float s = 0.2f * h;
        drawMarkerPin(dl, x, y + 0.86f * h, s);
        if (std::abs(rel) > kHalfSpan) {
            const float dir = rel > 0.0f ? 1.0f : -1.0f;
            const float ax = x + dir * (s + 10.0f), ay = y + 0.5f * h;
            dl->AddTriangleFilled(ImVec2(ax + dir * 7.0f, ay), ImVec2(ax - dir * 3.0f, ay - 7.0f), ImVec2(ax - dir * 3.0f, ay + 7.0f), kMarker);
        }
        const std::string d = formatDistance(std::hypot(marker.x - player.x, marker.z - player.z));
        text(dl, m_heading, 0.48f * h, x, y + h + 8.0f, kMarker, d.c_str(), 0.5f, true);
    }
}

} // namespace tf
