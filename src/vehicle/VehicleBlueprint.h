#pragma once

#include <array>
#include <string>
#include <vector>

// Vehicle blueprints: the body traced from the concept sheet (external_assets/images/Cars.png) by
// tools/cars/trace_blueprint.py into assets/vehicles/<id>/blueprint.json, embedded into the binary at build time
// (cmake/EmbedBlueprints.cmake). Each of the four orthographic views keeps the traced body outline (as a signed
// distance field) and the colour class of every pixel; the body is their visual hull, placed on the physics wheels.
// Used by the render model (render/VehicleModel) and by the collision shape (Vehicle.cpp), so both come from the same
// data. No SDL, no rendering.
//
// Concept frame: z forward from the middle of the concept wheelbase, y up from the concept axle line, x left; meters
// at the scale where the concept tire has the physics tire diameter.

namespace tf {

struct VehicleConfig;

// One view on the pixel grid of its sheet cell: cell (row i, column j) lies at u = u0 + du * j, v = v0 + dv * i.
// Side: (u, v) = (z, y); top: (z, x); front and rear: (x, y).
struct BlueprintView {
    float u0 = 0.0f, du = 1.0f, v0 = 0.0f, dv = 1.0f;
    int width = 0, height = 0;
    std::vector<char> cls;  // colour class per cell ('p' paint, 'a' accent, 'w' white, 'r' red, 'd' dark), '.' outside
    std::vector<float> sd;  // signed distance to the body outline, m (negative inside)

    char classAt(float u, float v) const; // nearest cell; '.' outside the grid
    // Class with the largest weight in a [1 2 1] blur of the cells, bilinear between cell centres: region borders
    // run smoothly between the cells instead of following the pixel staircase. '.' outside the grid.
    char smoothClassAt(float u, float v) const;
    float distance(float u, float v) const; // bilinear; grows linearly outside the grid
    bool inside(float u, float v) const { return distance(u, v) < 0.0f; }
};

struct BlueprintLight {
    std::string view;            // "front" | "rear"
    std::string kind;            // "head", "bar" (front), "tail", "brake" (rear)
    std::array<float, 4> box{};  // u0, v0, u1, v1 in the view (m, concept frame)
};

// Where an accessory sits on the sheet (hand-placed box, converted like the lights): "mirror", "spare", "winch", ...
struct BlueprintAnchor {
    std::string name;
    std::string view;
    std::array<float, 4> box{};  // u0, v0, u1, v1 in the view (m, concept frame)
};

struct VehicleBlueprint {
    std::string id;
    float tireDiameter = 0.0f;      // m, physics tire the concept was scaled to
    float conceptWheelBase = 0.0f;  // m
    float restHubY = -0.35f;        // body-space hub height at rest on flat ground: where the concept axle line goes
    BlueprintView side, top, front, rear;
    BlueprintView sideInner;         // side view with the wheel openings closed down to the sill (inboard body)
    std::array<float, 4> belt{};     // side window bottom line: z0, y0, z1, y1
    std::array<float, 4> glassTop{}; // side window top line
    std::vector<float> doorCuts;     // z of the door shut lines, front to rear
    std::array<float, 2> archZ{};    // concept axle z (front, rear)
    std::vector<BlueprintLight> lights;
    std::vector<BlueprintAnchor> anchors;

    // Anchor boxes of `name` in `view`, in sheet order.
    std::vector<const BlueprintAnchor*> find(const std::string& name, const std::string& view) const;
};

// The embedded blueprint of a visual model, parsed on first use; nullptr when there is none.
const VehicleBlueprint* findVehicleBlueprint(const std::string& visualModel);
bool parseVehicleBlueprint(const std::string& text, VehicleBlueprint& out, std::string* error = nullptr);

// A blueprint placed on a vehicle configuration, in body space (+X left, +Y up, +Z forward). The concept axles map
// onto the physics axles (z scales between the axles, the overhangs keep their length) and the concept axle line
// onto restHubY. The body is the visual hull of the four views minus the wheelhouses (the volume the tires sweep
// over the suspension travel, from the tub wall out past the tire), plus thin fender shells around the wheelhouses
// where the side view has body, so a tire that moves up tucks under the fender instead of showing through it.
class BlueprintBody {
public:
    BlueprintBody(const VehicleBlueprint& bp, const VehicleConfig& config);

    const VehicleBlueprint& blueprint() const { return m_bp; }
    float conceptZ(float z) const;
    float bodyZ(float zc) const;
    float conceptY(float y) const { return y - m_bp.restHubY; }
    float bodyY(float yc) const { return yc + m_bp.restHubY; }

    float hull(float x, float y, float z) const;        // visual hull, approximate distance (m, < 0 inside)
    float wheelhouse(float x, float y, float z) const;  // < 0 inside a wheelhouse
    float fender(float x, float y, float z) const;      // < 0 inside a fender shell
    float field(float x, float y, float z) const;       // the body
    // Where the ray origin + t * dir (unit dir) first enters the body, t in [0, maxT]; returns maxT when it does not.
    float raySurface(const std::array<float, 3>& origin, const std::array<float, 3>& dir, float maxT) const;

    // Concept-frame outline of the side view at concept z: top and bottom of the body (y), NaN where none.
    float sideTop(float zc) const;
    float beltY(float zc) const;    // concept y of the belt line (extended past its ends)
    float glassTopY(float zc) const;

    float innerX() const { return m_innerX; }   // wheelhouse inner wall (tub side), |x|
    float fenderOuterX(int axle) const;         // fender shell outer end, |x|
    float axleZ(int axle) const { return m_axleZ[axle]; }
    float bodyFront() const { return m_front; } // body-space z extents of the hull
    float bodyRear() const { return m_rear; }
    float bodyTop() const { return m_top; }
    float bodyBottom() const { return m_bottom; }

    // Rock sliders and the belly plate between them (anchor "slider" of the side view), body space; size 0 when the
    // blueprint has none. Drawn and collided as the same box.
    struct Box {
        float halfX = 0.0f, y0 = 0.0f, y1 = 0.0f, z0 = 0.0f, z1 = 0.0f;
    };
    Box sliderBox() const;

    // Collision: point sets whose convex hulls approximate the body (tub between the wheels, the full-width band
    // above the wheelhouses up to the belt, the greenhouse, the slider box), body space.
    std::vector<std::vector<std::array<float, 3>>> collisionParts() const;

private:
    struct Wheel {
        float sx, x, z, halfWidth, radius, hubLow, hubHigh; // radius: tire reach in (z, y) over the steering range
    };
    const VehicleBlueprint& m_bp;
    Wheel m_wheels[4];
    float m_axleZ[2];
    float m_conceptAxleZ;
    float m_innerX;
    float m_capX[2];
    float m_front, m_rear, m_top, m_bottom;
};

// Margins of the body fit around the wheels (m).
inline constexpr float kWheelhouseMargin = 0.025f; // tire sweep to wheelhouse surface
inline constexpr float kFenderSideMargin = 0.012f; // tire outer face to the widest fender lip
inline constexpr float kFenderShell = 0.016f;      // fender shell thickness

} // namespace tf
