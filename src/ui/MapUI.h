#pragma once

#include "app/MapData.h"
#include "app/MapView.h"

#include <string>

struct ImFont;
struct ImDrawList;
struct SDL_GPUTexture;

namespace tf {

class Renderer;

// The vehicle on the map: position and heading (bearing, degrees clockwise from north).
struct MapPlayer {
    float x = 0.0f, z = 0.0f;
    float headingDeg = 0.0f;
};

// Footer of the full-screen map: the controls of the device in use.
struct MapHints {
    bool gamepad = false;
    std::string zoom, pan, marker, center, close; // key / button names
};

// Draws the full-screen map, the corner minimap and the compass bar with ImGui draw lists from a MapData (raster
// uploaded once as a texture; roads, rivers, bridges, points of interest, the marker and the vehicle as vectors at
// screen resolution).
class MapUI {
public:
    MapUI() = default;
    ~MapUI();
    MapUI(const MapUI&) = delete;
    MapUI& operator=(const MapUI&) = delete;

    void setFonts(ImFont* body, ImFont* heading) {
        m_body = body;
        m_heading = heading;
    }
    // Uploads the raster and keeps the vector data; replaces the previous map.
    void setMap(Renderer& renderer, MapData data);
    void release();
    bool hasMap() const { return m_texture != nullptr; }
    const MapData& data() const { return m_data; }

    // Full-screen layout for a display size: the map panel (left) and the side panel (right), in points.
    struct Layout {
        float mapX0, mapY0, mapX1, mapY1;
        float sideX0, sideX1;
    };
    static Layout layout(float width, float height);
    // View that shows the whole drivable area in the map panel (north up), and the zoom limits.
    MapView fitView(const Layout& l) const;
    float minScale(const Layout& l) const;
    float maxScale() const;

    // hoverPoi: index into data().pois of the point under the cursor (highlighted), or -1. crosshair: draw the
    // aiming cross in the panel middle (keyboard / gamepad marker placement).
    void drawFullMap(const MapView& view, const MapMarker& marker, const MapPlayer& player, const MapHints& hints,
                     int hoverPoi, bool crosshair);
    // Point of interest within `radius` points of a screen point in the full map, or -1.
    int poiAt(const MapView& view, float x, float y, float radius) const;
    // Side-panel list row under a point (index into data().pois) of the last full map, or -1.
    int listRowAt(float x, float y) const;

    // Round minimap with its center at (cx, cy); rangeM: meters from the center to the rim.
    void drawMinimap(float cx, float cy, float radius, float rangeM, bool northUp, const MapMarker& marker,
                     const MapPlayer& player);
    // Compass bar centered at cx, top at y: heading tape, nearby points of interest, the marker with its distance.
    void drawCompass(float cx, float y, float width, float poiRangeM, const MapMarker& marker, const MapPlayer& player);

private:
    void drawRaster(ImDrawList* dl, const MapView& view, float x0, float y0, float x1, float y1) const;
    void drawLines(ImDrawList* dl, const MapView& view, float lineScale, const float* circle) const;
    void drawPoi(ImDrawList* dl, float x, float y, int id, float r, bool hover) const;
    void drawMarkerPin(ImDrawList* dl, float x, float y, float s) const;
    void drawPlayerArrow(ImDrawList* dl, float x, float y, float angleDeg, float s) const;

    MapData m_data;
    Renderer* m_renderer = nullptr;
    SDL_GPUTexture* m_texture = nullptr;
    ImFont* m_body = nullptr;
    ImFont* m_heading = nullptr;
    struct ListRow {
        float x0, y0, x1, y1;
        int poi;
    };
    std::vector<ListRow> m_listRows;
};

} // namespace tf
