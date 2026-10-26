#pragma once

#include "render/Mesh.h"

#include <glm/glm.hpp>

#include <functional>
#include <string>
#include <vector>

// Shared procedural building blocks for the vehicle models: materials, panels, lights, wheels and
// accessories. Everything is visual only; the body collision comes from the blueprint (vehicle/VehicleBlueprint.h).
// Body space: +X left, +Y up, +Z forward (meters).
namespace tf::style {

// ---- Materials
// Colors are given in sRGB (as picked from a reference) and stored linear.
glm::vec3 srgb(float r, float g, float b);
// Clear-coat paint: flat color (no metallic flakes), glossy coat. `mud` = receptivity 0..1.
Material paint(const glm::vec3& srgbColor, float roughness = 0.32f, float mud = 1.0f);
Material plastic(const glm::vec3& srgbColor, float roughness = 0.75f, float mud = 1.0f); // matte trim
Material metal(const glm::vec3& srgbColor, float roughness, float mud = 0.6f);
Material rubber(float mud = 1.0f);
Material glass();
// Emissive LED: the shader scales it with the headlight switch / brake / reverse / indicator state.
Material led(MatId id, const glm::vec3& linearColor);

// Palette of one vehicle livery: base paint, accent paint (two-tone panels, stripes), roof, trims.
struct Livery {
    Material body, accent, roof, stripe;
    Material trim;      // matte black plastic (flares, cladding, mirrors)
    Material steel;     // black powder-coated steel (bumpers, rack, sliders)
    Material liner;     // wheel-arch liner, recesses
    Material underbody;
    Material gunmetal;  // grille bars, winch drum
    Material alu;       // skid plate, slider caps
    Material hooks;     // anodized tow hooks / shackles
    Material lens;      // clear headlight lens housing
    Material glass;
};
Livery oliveExpedition(); // olive green, orange accents, white roof (Cars.png car #1)
Livery arcticElectric();  // white / light grey, blue accents, black cladding (Cars.png car #2)

// ---- Panels
// Flat quad facing `n` (the winding is chosen from `n`).
void face(MeshData& m, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d,
          const glm::vec3& n, const Material& mat);
// Planar patch spanned by its 4 corners (p00 = (s0,t0), p10, p11, p01), split at the normalized breaks
// `s` and `t` (0..1, sorted, including 0 and 1). `cell(i, j)` gives the material of cell i (along s), j (along t).
template <class CellFn>
void grid(MeshData& m, const glm::vec3& p00, const glm::vec3& p10, const glm::vec3& p11, const glm::vec3& p01,
          const std::vector<float>& s, const std::vector<float>& t, const glm::vec3& n, CellFn cell) {
    auto at = [&](float u, float v) {
        return (1 - u) * (1 - v) * p00 + u * (1 - v) * p10 + u * v * p11 + (1 - u) * v * p01;
    };
    for (std::size_t i = 0; i + 1 < s.size(); ++i)
        for (std::size_t j = 0; j + 1 < t.size(); ++j)
            face(m, at(s[i], t[j]), at(s[i + 1], t[j]), at(s[i + 1], t[j + 1]), at(s[i], t[j + 1]), n,
                 cell(int(i), int(j)));
}
// ---- Lofted surfaces
// Surface through rows of points (all rows have the same count): quad (i, k) joins rows i, i+1 and columns
// k, k+1 (k wraps when `closedRows`). Each quad is oriented along `hint(i, k, quadCenter)`; vertex normals are
// averaged over the neighboring quads within `smoothDeg`, so shallow curves shade smooth and chamfers stay hard.
// `cell(i, k)` picks the target mesh and material (nullptr mesh = skip the quad).
struct SkinCell {
    MeshData* mesh = nullptr;
    const Material* mat = nullptr;
};
void skin(const std::vector<std::vector<glm::vec3>>& rows, bool closedRows,
          const std::function<SkinCell(int, int)>& cell,
          const std::function<glm::vec3(int, int, const glm::vec3&)>& hint, float smoothDeg = 38.0f);

// Sweeps a cross-section (d along the path normal, y relative to the path point; ordered over the top to the
// bottom with d growing outward) along a path with horizontal normals, and caps both ends.
void sweepProfile(MeshData& m, const std::vector<glm::vec3>& path, const std::vector<glm::vec3>& normals,
                  const std::vector<glm::vec2>& profile, const Material& mat);

// Rounded-rectangle footprint at height y (x in [-hx, hx], z in [zRear, zFront], corner radius r). Points run
// front straight (-X to +X), front-left corner, left side (front to rear), rear-left corner, rear straight
// (+X to -X), rear-right corner, right side (rear to front), front-right corner (closed loop).
// `sideZ`: extra stations on the left side, front to rear (the right side uses the same z values; per ring
// they may differ, the count must match); `endX`: stations on the front and rear straights as fractions -1..1.
struct Footprint {
    enum Seg { Front, CornerFL, Left, CornerRL, Rear, CornerRR, Right, CornerFR };
    std::vector<glm::vec3> p;      // y = height
    std::vector<glm::vec3> n;      // outward horizontal normal
    std::vector<Seg> seg;
};
Footprint roundedRect(float hx, float zRear, float zFront, float r, float y, int cornerSegs,
                      const std::vector<float>& sideZ, const std::vector<float>& endX);

// Panel gap: a thin dark strip standing `lift` off a surface, from a to b, `width` wide, facing n.
void panelGap(MeshData& m, const glm::vec3& a, const glm::vec3& b, const glm::vec3& n, float width,
              const Material& mat, float lift = 0.0015f);
// Closed polygonal prism: outline points (u, v) in the plane spanned by axes u and v, extruded along w
// from w0 to w1 (outline must be convex or star-shaped around its centroid).
void prism(MeshData& m, const std::vector<glm::vec2>& outline, const glm::vec3& origin, const glm::vec3& u,
           const glm::vec3& v, const glm::vec3& w, float w0, float w1, const Material& sides, const Material& cap0,
           const Material& cap1);

// ---- Lights
// Rounded-rectangle LED outline (DRL ring) on a plane: center, outward normal n, `right` and `up` axes.
void ledRing(MeshData& m, const glm::vec3& center, const glm::vec3& n, const glm::vec3& right, float halfW, float halfH,
             float radius, float stripW, const Material& mat);
// Straight LED strip, `width` wide, raised `lift` off the plane facing n.
void ledStrip(MeshData& m, const glm::vec3& a, const glm::vec3& b, const glm::vec3& n, float width, const Material& mat,
              float lift = 0.002f);
// Round lamp: bezel cylinder plus lens disc along `axis` (pointing out of the lamp).
void roundLamp(MeshData& m, const glm::vec3& center, const glm::vec3& axis, float radius, float depth,
               const Material& bezel, const Material& lens);

// ---- Accessories
// Low-profile roof rails along the roof edges (x = +-halfW, z0..z1) on three feet, with crossbars at `crossZ`.
// Returns the height of the crossbar tops (where cargo rests).
float roofRails(MeshData& m, float halfW, float z0, float z1, float yBase, const std::vector<float>& crossZ,
                const Material& rail, const Material& feet);
// Ratchet strap over cargo: a thin band from one side of the load down to the crossbar on the other side.
void cargoStrap(MeshData& m, const glm::vec3& a, const glm::vec3& top0, const glm::vec3& top1, const glm::vec3& b,
                const Material& strap);
void lightBar(MeshData& m, const glm::vec3& center, float halfLen, const Material& housing, const Material& ledMat);
// Winch: drum along X centered at `center`, fairlead plate in front (+Z).
void winch(MeshData& m, const glm::vec3& center, const Material& drum, const Material& fairlead, const Material& dark);
// Bow shackle (tow hook): ring in the plane of `forward` and Y, hanging from `mount`.
void shackle(MeshData& m, const glm::vec3& mount, const glm::vec3& forward, const Material& mat);
// Snorkel on side sx: intake pipe from the fender up the A-pillar to a ram head at roof height.
void snorkel(MeshData& m, float sx, const glm::vec3& base, const glm::vec3& bend, const glm::vec3& top,
             const Material& body, const Material& dark);
// Slim fender flare following an arch outline (z, y points from the rear end to the front end, in body space):
// a band `band` wide along the arch with a rounded lip and chamfered profile, `proud` above the body side at
// x = xBody on side sx; it ends flat on the arch end height.
void fenderFlare(MeshData& m, float sx, const std::vector<glm::vec2>& archZY, float band, float xBody, float proud,
                 const Material& mat, const Material& liner);
void jerryCan(MeshData& m, const glm::vec3& center, const glm::vec3& outward, const Material& body,
              const Material& dark);
void cargoCase(MeshData& m, const glm::vec3& center, const glm::vec3& half, const Material& body,
               const Material& latch);
void ladder(MeshData& m, const glm::vec3& bottom, const glm::vec3& top, float halfW, const glm::vec3& side,
            const Material& mat);
void rockSlider(MeshData& m, float sx, float x, float y, float z0, float z1, const Material& tube,
                const Material& caps, const Material& brackets);
void mirror(MeshData& m, float sx, const glm::vec3& base, const Material& housing, const Material& glassMat);
// Spare wheel carrier plate with a hub stud, facing `outward`.
void spareCarrier(MeshData& m, const glm::vec3& center, const glm::vec3& outward, float radius, const Material& steel);

// ---- Wheels
enum class Tread { AllTerrain, MudTerrain };
struct WheelStyle {
    int spokes = 8;
    float rimFraction = 0.58f;  // rim radius / wheel radius (vehicle.json `rimRatio`)
    bool beadlock = false;
    Tread tread = Tread::MudTerrain;
    Material rim = metal(glm::vec3(0.18f), 0.40f);
    Material ring = metal(glm::vec3(0.55f), 0.30f, 0.5f);
    Material tire = rubber();
};
// Wheel with tread, sidewall, rim and brake disc. Axle along local X, outer face at +X. The mesh fits exactly
// in the physics wheel: max distance from the axle == radius, axial extent == +-width / 2.
MeshData buildWheel(float radius, float width, const WheelStyle& style);

} // namespace tf::style
