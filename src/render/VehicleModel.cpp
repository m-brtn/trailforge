#include "render/VehicleModel.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace tf {
namespace {

constexpr float kPi = 3.14159265358979f;

// ---- Palette (linear albedo)
const Material kPaint{MatId::Paint, {0.105f, 0.155f, 0.080f}, 0.30f, 0.0f, 1.0f};   // olive green clear coat
const Material kRoof{MatId::Paint, {0.78f, 0.78f, 0.75f}, 0.34f, 0.0f, 0.35f};      // white roof
const Material kBlackPlastic{MatId::Generic, {0.026f, 0.026f, 0.027f}, 0.72f, 0.0f, 1.0f};
const Material kBlackSteel{MatId::Generic, {0.032f, 0.032f, 0.034f}, 0.45f, 0.35f, 1.0f};
const Material kUnderbody{MatId::Generic, {0.045f, 0.042f, 0.040f}, 0.85f, 0.0f, 1.0f};
const Material kLiner{MatId::Generic, {0.022f, 0.022f, 0.022f}, 0.90f, 0.0f, 1.0f};
const Material kChrome{MatId::Generic, {0.92f, 0.92f, 0.92f}, 0.10f, 1.0f, 0.2f};
const Material kAlu{MatId::Generic, {0.80f, 0.80f, 0.82f}, 0.32f, 1.0f, 0.6f};
const Material kGlass{MatId::Glass, {0.012f, 0.016f, 0.020f}, 0.04f, 0.0f, 0.15f};
const Material kMirror{MatId::Generic, {0.90f, 0.90f, 0.92f}, 0.04f, 1.0f, 0.0f};
const Material kHeadlight{MatId::Headlight, {1.0f, 0.93f, 0.82f}, 0.2f, 0.0f, 0.0f};
const Material kLedBar{MatId::Headlight, {0.85f, 0.92f, 1.0f}, 0.2f, 0.0f, 0.0f};
const Material kBrake{MatId::BrakeLight, {1.0f, 0.04f, 0.02f}, 0.25f, 0.0f, 0.2f};
const Material kReverse{MatId::ReverseLight, {1.0f, 1.0f, 1.0f}, 0.25f, 0.0f, 0.2f};
const Material kAmber{MatId::Amber, {1.0f, 0.42f, 0.04f}, 0.3f, 0.0f, 0.2f};
const Material kTrim{MatId::Generic, {0.018f, 0.018f, 0.018f}, 0.55f, 0.0f, 0.5f};
const Material kBoards{MatId::Generic, {0.80f, 0.22f, 0.02f}, 0.55f, 0.0f, 0.3f};
const Material kRed{MatId::Generic, {0.55f, 0.03f, 0.02f}, 0.40f, 0.0f, 0.5f};
const Material kOlive{MatId::Generic, {0.07f, 0.09f, 0.035f}, 0.55f, 0.0f, 0.5f};
const Material kIron{MatId::Generic, {0.06f, 0.06f, 0.06f}, 0.50f, 0.5f, 1.0f};

// Quad with per-vertex normals; the winding follows the normals.
void quadN(MeshData& m, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d,
           const glm::vec3& na, const glm::vec3& nb, const glm::vec3& nc, const glm::vec3& nd, const Material& mat) {
    if (glm::dot(glm::cross(b - a, c - a), na + nb + nc + nd) >= 0.0f) {
        m.addTri(a, b, c, na, nb, nc, mat);
        m.addTri(a, c, d, na, nc, nd, mat);
    } else {
        m.addTri(a, c, b, na, nc, nb, mat);
        m.addTri(a, d, c, na, nd, nc, mat);
    }
}

// ---- Side panel (sill + fenders) with wheel arches cut out, for side sx = +1 (left) or -1 (right).
constexpr float kArchCy = -0.20f;   // arch circle center height (body space)
constexpr float kArchR = 0.545f;    // arch radius
constexpr float kAxleZ = 1.40f;     // |z| of the axles (wheelBase / 2)

void addSidePanel(MeshData& m, float sx) {
    const float xi = 0.50f, xo = 0.88f, top = 0.56f, ch = 0.03f, z0 = -2.22f, z1 = 2.20f, sill = -0.10f;
    auto bottom = [&](float z) {
        float b = sill;
        for (float zc : {kAxleZ, -kAxleZ}) {
            const float dz = z - zc;
            if (std::abs(dz) < kArchR) b = std::max(b, kArchCy + std::sqrt(kArchR * kArchR - dz * dz));
        }
        return b;
    };
    std::vector<float> zs;
    for (float z = z0; z < z1; z += 0.04f) zs.push_back(z);
    zs.push_back(z1);
    const float edge = std::sqrt(kArchR * kArchR - (sill - kArchCy) * (sill - kArchCy));
    for (float zc : {kAxleZ, -kAxleZ}) {
        zs.push_back(zc - edge);
        zs.push_back(zc + edge);
    }
    std::sort(zs.begin(), zs.end());
    auto X = [&](float x) { return sx * x; };
    for (std::size_t i = 0; i + 1 < zs.size(); ++i) {
        const float za = zs[i], zb = zs[i + 1];
        if (zb - za < 1e-4f) continue;
        const float ba = bottom(za), bb = bottom(zb);
        const glm::vec3 inside(X(0.5f * (xi + xo)), 0.5f * (std::max(ba, bb) + top), 0.5f * (za + zb));
        const bool arch = std::max(ba, bb) > sill + 1e-3f;
        m.addQuadOutward({X(xo), ba, za}, {X(xo), bb, zb}, {X(xo), top - ch, zb}, {X(xo), top - ch, za}, inside, kPaint);
        m.addQuadOutward({X(xo), top - ch, za}, {X(xo), top - ch, zb}, {X(xo - ch), top, zb}, {X(xo - ch), top, za},
                         inside, kPaint);
        m.addQuadOutward({X(xo - ch), top, za}, {X(xo - ch), top, zb}, {X(xi), top, zb}, {X(xi), top, za}, inside, kPaint);
        m.addQuadOutward({X(xi), ba, za}, {X(xi), bb, zb}, {X(xi), top, zb}, {X(xi), top, za}, inside, kLiner);
        const glm::vec3 above(inside.x, std::max(ba, bb) + 0.05f, inside.z);
        m.addQuadOutward({X(xi), ba, za}, {X(xo), ba, za}, {X(xo), bb, zb}, {X(xi), bb, zb}, above,
                         arch ? kLiner : kUnderbody);
    }
    // End caps.
    for (float z : {z0, z1}) {
        const float b = bottom(z);
        const glm::vec3 inside(X(0.5f * (xi + xo)), 0.5f * (b + top), z - 0.01f * (z > 0 ? 1.0f : -1.0f));
        m.addQuadOutward({X(xi), b, z}, {X(xo), b, z}, {X(xo), top - ch, z}, {X(xi), top, z}, inside, kPaint);
        m.addTriangleOutward({X(xo), top - ch, z}, {X(xo - ch), top, z}, {X(xi), top, z}, inside, kPaint);
    }
    // Door seams (thin dark strips on the outer skin) and handles.
    for (float z : {0.74f, -0.06f, -0.80f}) {
        const float x = X(xo + 0.0015f);
        const glm::vec3 inside(X(xo - 0.1f), 0.2f, z);
        m.addQuadOutward({x, -0.06f, z - 0.004f}, {x, -0.06f, z + 0.004f}, {x, top - ch, z + 0.004f},
                         {x, top - ch, z - 0.004f}, inside, kTrim);
    }
}

// Black plastic fender flare over the arch at axle z = zc.
void addFlare(MeshData& m, float sx, float zc) {
    const float r0 = kArchR, r1 = 0.625f, xa = 0.85f, xb = 0.985f, lip = 0.02f;
    const float th0 = std::asin((-0.10f - kArchCy) / r1), th1 = kPi - th0;
    constexpr int kSeg = 28;
    auto P = [&](float th, float r, float x) { return glm::vec3(sx * x, kArchCy + r * std::sin(th), zc + r * std::cos(th)); };
    auto R = [&](float th) { return glm::vec3(0.0f, std::sin(th), std::cos(th)); };
    const glm::vec3 side(sx, 0, 0);
    for (int i = 0; i < kSeg; ++i) {
        const float ta = th0 + (th1 - th0) * float(i) / kSeg, tb = th0 + (th1 - th0) * float(i + 1) / kSeg;
        // Outer arc surface, lip chamfer, outer side face, inner arc surface.
        quadN(m, P(ta, r1, xa), P(tb, r1, xa), P(tb, r1, xb - lip), P(ta, r1, xb - lip), R(ta), R(tb), R(tb), R(ta),
              kBlackPlastic);
        const glm::vec3 ca = glm::normalize(R(ta) + side), cb = glm::normalize(R(tb) + side);
        quadN(m, P(ta, r1, xb - lip), P(tb, r1, xb - lip), P(tb, r1 - lip, xb), P(ta, r1 - lip, xb), ca, cb, cb, ca,
              kBlackPlastic);
        quadN(m, P(ta, r1 - lip, xb), P(tb, r1 - lip, xb), P(tb, r0, xb), P(ta, r0, xb), side, side, side, side,
              kBlackPlastic);
        quadN(m, P(ta, r0, xb), P(tb, r0, xb), P(tb, r0, xa), P(ta, r0, xa), -R(ta), -R(tb), -R(tb), -R(ta), kLiner);
    }
    for (float th : {th0, th1}) {
        const glm::vec3 n = glm::vec3(0, std::cos(th), -std::sin(th)) * (th == th0 ? -1.0f : 1.0f);
        quadN(m, P(th, r0, xa), P(th, r1, xa), P(th, r1, xb), P(th, r0, xb), n, n, n, n, kBlackPlastic);
    }
}

void addHeadlight(MeshData& m, float x, float y, float z) {
    m.addCylinder({x, y, z - 0.02f}, {x, y, z + 0.022f}, 0.125f, 24, kChrome);
    m.addCylinder({x, y, z + 0.022f}, {x, y, z + 0.030f}, 0.104f, 24, kHeadlight);
}

} // namespace

MeshData buildVehicleBodyMesh() {
    MeshData m;
    // ---- Ladder frame, tub, differentials
    for (float sx : {1.0f, -1.0f}) m.addBeveledBox({sx * 0.40f, -0.14f, 0.0f}, {0.045f, 0.07f, 2.12f}, 0.01f, kBlackSteel);
    for (float z : {-2.0f, -0.7f, 0.5f, 2.0f}) m.addBox({0.0f, -0.14f, z}, {0.40f, 0.04f, 0.04f}, kBlackSteel);
    m.addBox({0.0f, 0.175f, 0.0f}, {0.50f, 0.225f, 2.17f}, kUnderbody);
    for (float zc : {kAxleZ, -kAxleZ}) {
        m.addCylinder({0.0f, -0.24f, zc - 0.10f}, {0.0f, -0.24f, zc + 0.07f}, 0.12f, 18, kIron);
        m.addCylinder({0.0f, -0.24f, zc + 0.07f}, {0.0f, -0.24f, zc + 0.10f}, 0.085f, 18, kIron);
        m.addBox({0.0f, -0.22f, zc}, {0.16f, 0.05f, 0.05f}, kIron);
    }
    m.addBar({0.0f, -0.04f, 2.12f}, {0.0f, -0.21f, 1.58f}, 0.42f, 0.008f, {0, 1, 0}, kAlu, 0.004f); // skid plate
    m.addCylinder({-0.45f, -0.03f, -1.55f}, {-0.45f, -0.03f, -2.30f}, 0.032f, 12, kAlu);         // exhaust

    // ---- Lower body: sills/fenders with arches, flares, hood
    for (float sx : {1.0f, -1.0f}) {
        addSidePanel(m, sx);
        for (float zc : {kAxleZ, -kAxleZ}) addFlare(m, sx, zc);
        // Rock sliders.
        m.addCylinder({sx * 0.93f, -0.06f, -0.84f}, {sx * 0.93f, -0.06f, 0.84f}, 0.04f, 14, kBlackSteel);
        for (float z : {-0.62f, 0.0f, 0.62f}) m.addBar({sx * 0.80f, -0.06f, z}, {sx * 0.92f, -0.06f, z}, 0.02f, 0.03f, {0, 1, 0}, kBlackSteel);
        // Side marker and door handles.
        m.addBox({sx * 0.884f, 0.42f, 2.06f}, {0.004f, 0.025f, 0.05f}, kAmber);
        for (float z : {0.55f, -0.34f}) m.addBeveledBox({sx * 0.875f, 0.80f, z}, {0.012f, 0.016f, 0.075f}, 0.006f, kTrim);
    }
    m.addBeveledBox({0.0f, 0.605f, 1.49f}, {0.86f, 0.055f, 0.715f}, 0.035f, kPaint);  // hood
    m.addBeveledBox({0.0f, 0.672f, 1.45f}, {0.42f, 0.02f, 0.55f}, 0.015f, kPaint);    // hood bulge
    for (int k = 0; k < 5; ++k) m.addBox({0.0f, 0.693f, 1.06f + 0.05f * k}, {0.30f, 0.004f, 0.012f}, kBlackPlastic);

    // ---- Front: grille, headlights, bumper, bull bar
    m.addBox({0.0f, 0.33f, 2.205f}, {0.50f, 0.20f, 0.012f}, kBlackPlastic);
    for (int k = 0; k < 5; ++k)
        m.addBeveledBox({0.0f, 0.19f + 0.065f * k, 2.226f}, {0.46f, 0.014f, 0.011f}, 0.005f,
                        Material{MatId::Generic, {0.07f, 0.07f, 0.075f}, 0.35f, 0.7f, 0.8f});
    for (float sx : {1.0f, -1.0f}) {
        addHeadlight(m, sx * 0.67f, 0.40f, 2.205f);
        m.addBeveledBox({sx * 0.67f, 0.22f, 2.214f}, {0.06f, 0.028f, 0.012f}, 0.006f, kAmber);
    }
    m.addBeveledBox({0.0f, 0.12f, 2.27f}, {0.96f, 0.10f, 0.075f}, 0.025f, kBlackSteel);
    m.addBox({0.0f, 0.14f, 2.347f}, {0.10f, 0.035f, 0.003f}, kAlu); // winch fairlead
    m.addBox({0.0f, 0.14f, 2.350f}, {0.06f, 0.008f, 0.002f}, kLiner);
    for (float sx : {1.0f, -1.0f}) {
        m.addBeveledBox({sx * 0.55f, 0.12f, 2.352f}, {0.022f, 0.05f, 0.012f}, 0.008f, kRed); // shackles
        m.addCylinder({sx * 0.38f, 0.20f, 2.30f}, {sx * 0.38f, 0.50f, 2.30f}, 0.022f, 12, kBlackSteel);
    }
    m.addCylinder({-0.38f, 0.50f, 2.30f}, {0.38f, 0.50f, 2.30f}, 0.022f, 12, kBlackSteel);
    m.addCylinder({0.86f, 0.60f, 1.90f}, {0.86f, 1.75f, 1.86f}, 0.004f, 6, kTrim); // antenna

    // ---- Cab
    m.addBeveledBox({0.0f, 0.74f, -0.36f}, {0.86f, 0.18f, 1.14f}, 0.03f, kPaint);
    m.addFrustum(0.84f, 0.915f, -1.48f, 0.76f, 0.77f, 1.245f, -1.43f, 0.50f, kGlass, kGlass, kGlass, kPaint);
    for (float sx : {1.0f, -1.0f}) {
        m.addBar({sx * 0.835f, 0.91f, 0.755f}, {sx * 0.765f, 1.25f, 0.495f}, 0.035f, 0.035f, {sx, 0, 0}, kPaint, 0.01f); // A
        m.addBar({sx * 0.842f, 0.91f, -0.30f}, {sx * 0.772f, 1.25f, -0.285f}, 0.014f, 0.05f, {0, 0, 1}, kBlackPlastic); // B
        m.addBar({sx * 0.835f, 0.91f, -1.475f}, {sx * 0.765f, 1.25f, -1.425f}, 0.04f, 0.04f, {sx, 0, 0}, kPaint, 0.01f); // C
        m.addBar({sx * 0.852f, 0.925f, -1.48f}, {sx * 0.852f, 0.925f, 0.76f}, 0.012f, 0.012f, {0, 1, 0}, kTrim); // belt trim
        // Mirrors.
        m.addBar({sx * 0.84f, 0.97f, 0.66f}, {sx * 0.98f, 1.02f, 0.62f}, 0.012f, 0.012f, {0, 1, 0}, kBlackPlastic);
        m.addBeveledBox({sx * 1.01f, 1.05f, 0.60f}, {0.035f, 0.075f, 0.05f}, 0.015f, kBlackPlastic);
        m.addBox({sx * 1.01f, 1.05f, 0.548f}, {0.027f, 0.064f, 0.003f}, kMirror);
        // Drip rails.
        m.addBar({sx * 0.80f, 1.245f, -1.46f}, {sx * 0.80f, 1.245f, 0.51f}, 0.012f, 0.012f, {0, 1, 0}, kTrim);
    }
    m.addBar({-0.77f, 1.242f, 0.50f}, {0.77f, 1.242f, 0.50f}, 0.03f, 0.03f, {0, 1, 0}, kPaint, 0.01f);
    m.addBar({-0.84f, 0.92f, 0.765f}, {0.84f, 0.92f, 0.765f}, 0.02f, 0.012f, {0, 1, 0}, kTrim);
    m.addBeveledBox({0.0f, 1.28f, -0.475f}, {0.80f, 0.04f, 0.99f}, 0.03f, kRoof);

    // ---- Snorkel (right side)
    m.addCylinder({-0.93f, 0.48f, 0.70f}, {-0.93f, 1.06f, 0.70f}, 0.045f, 14, kBlackPlastic);
    m.addCylinder({-0.93f, 1.06f, 0.70f}, {-0.885f, 1.30f, 0.52f}, 0.045f, 14, kBlackPlastic);
    m.addBeveledBox({-0.885f, 1.35f, 0.53f}, {0.06f, 0.065f, 0.095f}, 0.02f, kBlackPlastic);
    m.addBox({-0.885f, 1.35f, 0.626f}, {0.045f, 0.045f, 0.002f}, kLiner);

    // ---- Bed with cargo, taillights, rear bumper
    m.addBox({0.0f, 0.50f, -1.87f}, {0.50f, 0.06f, 0.35f}, kLiner);
    for (float sx : {1.0f, -1.0f}) {
        m.addBeveledBox({sx * 0.845f, 0.72f, -1.865f}, {0.035f, 0.16f, 0.355f}, 0.012f, kPaint);
        m.addBar({sx * 0.845f, 0.885f, -2.22f}, {sx * 0.845f, 0.885f, -1.51f}, 0.04f, 0.008f, {0, 1, 0}, kTrim);
        m.addBox({sx * 0.76f, 0.38f, -2.222f}, {0.075f, 0.155f, 0.004f}, kTrim);
        m.addBeveledBox({sx * 0.76f, 0.43f, -2.226f}, {0.062f, 0.095f, 0.006f}, 0.004f, kBrake);
        m.addBeveledBox({sx * 0.76f, 0.275f, -2.226f}, {0.062f, 0.03f, 0.006f}, 0.004f, kReverse);
        m.addBeveledBox({sx * 0.30f, 0.74f, -1.95f}, {0.08f, 0.17f, 0.12f}, 0.02f, sx > 0 ? kOlive : kRed); // jerry cans
    }
    m.addBeveledBox({0.0f, 0.72f, -2.195f}, {0.88f, 0.16f, 0.025f}, 0.012f, kPaint); // tailgate
    m.addBox({0.0f, 0.80f, -2.222f}, {0.20f, 0.012f, 0.003f}, kTrim);                 // tailgate handle
    m.addBeveledBox({0.0f, 0.68f, -1.62f}, {0.45f, 0.10f, 0.09f}, 0.012f, kAlu);       // toolbox
    m.addBeveledBox({0.0f, 0.12f, -2.28f}, {0.92f, 0.09f, 0.065f}, 0.025f, kBlackSteel);
    m.addBox({0.0f, 0.07f, -2.336f}, {0.04f, 0.04f, 0.012f}, kLiner);

    // ---- Roof rack, light bar, recovery boards (the spare wheel is drawn separately)
    const float ry = 1.39f;
    for (float sx : {1.0f, -1.0f}) {
        m.addCylinder({sx * 0.72f, ry, -1.38f}, {sx * 0.72f, ry, 0.45f}, 0.018f, 10, kBlackSteel);
        for (float z : {-1.30f, -0.45f, 0.38f})
            m.addBar({sx * 0.72f, 1.32f, z}, {sx * 0.72f, ry, z}, 0.02f, 0.03f, {0, 0, 1}, kBlackSteel);
        m.addBeveledBox({sx * 0.58f, 1.42f, -0.72f}, {0.13f, 0.018f, 0.55f}, 0.01f, kBoards);
    }
    for (float z : {-1.38f, 0.45f}) m.addCylinder({-0.72f, ry, z}, {0.72f, ry, z}, 0.018f, 10, kBlackSteel);
    for (float z = -1.20f; z < 0.40f; z += 0.26f) m.addBox({0.0f, ry + 0.005f, z}, {0.71f, 0.008f, 0.02f}, kBlackSteel);
    m.addBeveledBox({0.0f, 1.44f, 0.50f}, {0.56f, 0.04f, 0.035f}, 0.012f, kBlackPlastic);
    for (int k = 0; k < 6; ++k) m.addBox({-0.45f + 0.18f * k, 1.44f, 0.537f}, {0.07f, 0.024f, 0.004f}, kLedBar);
    return m;
}

MeshData buildWheelMesh(float radius, float width) {
    MeshData m;
    const float hw = 0.5f * width;
    const float rr = 0.59f * radius;      // rim radius
    const float rt = radius - 0.016f;     // tread base (lugs reach `radius`)
    const Material tire{MatId::Rubber, {0.032f, 0.032f, 0.034f}, 0.85f, 0.0f, 1.0f};
    const Material rim{MatId::Generic, {0.16f, 0.16f, 0.17f}, 0.38f, 1.0f, 0.6f};
    const Material ring{MatId::Generic, {0.72f, 0.72f, 0.74f}, 0.28f, 1.0f, 0.5f};
    const Material dark{MatId::Generic, {0.05f, 0.05f, 0.05f}, 0.6f, 0.6f, 0.6f};
    const Material disc{MatId::Generic, {0.35f, 0.34f, 0.33f}, 0.45f, 1.0f, 0.3f};

    // ---- Carcass (clockwise profile in (x, r): left bead -> over the tread -> right bead)
    const float rm = 0.5f * (rr + rt);
    m.addLathe({{-hw * 0.86f, rr}, {-hw * 0.97f, rr + 0.03f}, {-hw * 1.02f, rm}, {-hw * 0.99f, rt - 0.03f},
                {-hw * 0.90f, rt - 0.006f}, {-hw * 0.80f, rt}, {hw * 0.80f, rt}, {hw * 0.90f, rt - 0.006f},
                {hw * 0.99f, rt - 0.03f}, {hw * 1.02f, rm}, {hw * 0.97f, rr + 0.03f}, {hw * 0.86f, rr}},
               48, tire);
    // Sidewall rib (raised ring), both sides.
    for (float s : {-1.0f, 1.0f})
        m.addLathe(s > 0 ? std::vector<glm::vec2>{{hw * 1.01f, rm + 0.012f}, {hw * 1.035f, rm + 0.006f},
                                                  {hw * 1.035f, rm - 0.006f}, {hw * 1.01f, rm - 0.012f}}
                         : std::vector<glm::vec2>{{-hw * 1.01f, rm - 0.012f}, {-hw * 1.035f, rm - 0.006f},
                                                  {-hw * 1.035f, rm + 0.006f}, {-hw * 1.01f, rm + 0.012f}},
                   48, tire);

    // ---- Tread lugs: staggered blocks with a chevron angle plus shoulder lugs on alternate pitches.
    constexpr int kPitches = 26;
    const float pitchLen = 2.0f * kPi * radius / float(kPitches);
    const glm::mat4 base = m.xf;
    for (int p = 0; p < kPitches; ++p) {
        for (int side = 0; side < 2; ++side) {
            const float s = side == 0 ? -1.0f : 1.0f;
            const float th = 2.0f * kPi * (float(p) + 0.5f * float(side)) / float(kPitches);
            glm::mat4 r = glm::rotate(glm::mat4(1.0f), th, glm::vec3(1, 0, 0)); // around the axle
            // Local frame after rotation: +Y radial, +Z tangential, +X axial.
            glm::mat4 blk = glm::translate(r, glm::vec3(s * hw * 0.45f, 0.5f * (radius + rt) - 0.001f, 0.0f));
            blk = glm::rotate(blk, s * 0.21f, glm::vec3(0, 1, 0)); // chevron
            m.xf = base * blk;
            m.addBeveledBox(glm::vec3(0), {hw * 0.36f, 0.5f * (radius - rt) + 0.002f, pitchLen * 0.30f}, 0.004f, tire);
            if ((p + side) % 2 == 0) {
                glm::mat4 sh = glm::translate(r, glm::vec3(s * hw * 0.985f, rt - 0.022f, 0.0f));
                m.xf = base * sh;
                m.addBeveledBox(glm::vec3(0), {hw * 0.05f, 0.026f, pitchLen * 0.26f}, 0.004f, tire);
            }
        }
    }
    m.xf = base;

    // ---- Rim: barrel, beadlock ring with bolts, spokes, hub, lug nuts, center cap
    m.addLathe({{hw * 0.80f, rr - 0.03f}, {-hw * 0.85f, rr - 0.03f}}, 32, dark); // barrel (faces the axle)
    m.addLathe({{hw * 0.80f, rr - 0.03f}, {hw * 0.80f, rr + 0.012f}, {hw * 0.80f, rr + 0.012f},
                {hw * 1.03f, rr + 0.012f}, {hw * 1.03f, rr + 0.012f}, {hw * 1.03f, rr - 0.03f},
                {hw * 1.03f, rr - 0.03f}, {hw * 0.80f, rr - 0.03f}},
               40, ring);
    constexpr int kBolts = 18;
    for (int i = 0; i < kBolts; ++i) {
        const float a = 2.0f * kPi * float(i) / float(kBolts);
        const glm::vec3 p(0.0f, (rr - 0.009f) * std::cos(a), (rr - 0.009f) * std::sin(a));
        m.addCylinder(p + glm::vec3(hw * 1.03f, 0, 0), p + glm::vec3(hw * 1.06f, 0, 0), 0.0055f, 6, dark);
    }
    constexpr int kSpokes = 6;
    for (int i = 0; i < kSpokes; ++i) {
        const float a = 2.0f * kPi * float(i) / float(kSpokes);
        const glm::vec3 dir(0.0f, std::cos(a), std::sin(a));
        m.addBar(glm::vec3(hw * 0.48f, 0, 0) + dir * 0.06f, glm::vec3(hw * 0.80f, 0, 0) + dir * (rr - 0.035f), 0.022f,
                 0.018f, glm::vec3(1, 0, 0), rim, 0.006f);
    }
    m.addCylinder({hw * 0.30f, 0, 0}, {hw * 0.52f, 0, 0}, 0.088f, 24, rim);
    for (int i = 0; i < 6; ++i) {
        const float a = 2.0f * kPi * (float(i) + 0.5f) / 6.0f;
        const glm::vec3 p(0.0f, 0.062f * std::cos(a), 0.062f * std::sin(a));
        m.addCylinder(p + glm::vec3(hw * 0.52f, 0, 0), p + glm::vec3(hw * 0.62f, 0, 0), 0.011f, 6, ring);
    }
    m.addCylinder({hw * 0.52f, 0, 0}, {hw * 0.68f, 0, 0}, 0.040f, 16, dark);
    // Brake disc (spins with the wheel).
    m.addCylinder({-hw * 0.30f, 0, 0}, {-hw * 0.12f, 0, 0}, 0.165f, 28, disc);
    return m;
}

MeshData buildUnitSpringMesh() {
    MeshData m;
    const Material coil{MatId::Generic, {0.55f, 0.06f, 0.02f}, 0.38f, 0.1f, 0.8f};
    const float rc = 0.052f, rw = 0.0095f, turns = 7.5f, lref = 0.62f;
    constexpr int kPerTurn = 14, kSides = 6;
    const int n = int(turns * kPerTurn);
    auto center = [&](float t) {
        const float a = 2.0f * kPi * turns * t;
        return glm::vec3(rc * std::cos(a), rc * std::sin(a), t);
    };
    for (int i = 0; i < n; ++i) {
        const float t0 = float(i) / float(n), t1 = float(i + 1) / float(n);
        auto ringAt = [&](float t, int k) {
            const float a = 2.0f * kPi * turns * t;
            const glm::vec3 radial(std::cos(a), std::sin(a), 0.0f);
            const float b = 2.0f * kPi * float(k) / float(kSides);
            // Wire cross-section in the radial/axial plane; axial extent pre-divided by the typical stretch.
            const glm::vec3 off = radial * (rw * std::cos(b)) + glm::vec3(0, 0, rw * std::sin(b) / lref);
            const glm::vec3 nrm = glm::normalize(radial * std::cos(b) + glm::vec3(0, 0, std::sin(b)));
            return std::pair<glm::vec3, glm::vec3>(center(t) + off, nrm);
        };
        for (int k = 0; k < kSides; ++k) {
            const auto a0 = ringAt(t0, k), a1 = ringAt(t0, k + 1), b0 = ringAt(t1, k), b1 = ringAt(t1, k + 1);
            quadN(m, a0.first, b0.first, b1.first, a1.first, a0.second, b0.second, b1.second, a1.second, coil);
        }
    }
    return m;
}

MeshData buildUnitTubeMesh(float radius) {
    MeshData m;
    const Material steel{MatId::Generic, {0.035f, 0.035f, 0.037f}, 0.45f, 0.4f, 1.0f};
    m.addCylinder({0, 0, 0}, {0, 0, 1}, radius, 10, steel);
    return m;
}

MeshData buildShockBodyMesh() {
    MeshData m;
    const Material gold{MatId::Generic, {0.78f, 0.52f, 0.13f}, 0.30f, 1.0f, 0.8f};
    const Material black{MatId::Generic, {0.03f, 0.03f, 0.03f}, 0.5f, 0.3f, 0.8f};
    m.addCylinder({0, 0, 0}, {0, 0, kShockBodyLength}, 0.034f, 14, gold);
    m.addCylinder({0, 0, kSpringPerch - 0.02f}, {0, 0, kSpringPerch}, 0.066f, 18, black); // lower spring perch
    m.addCylinder({0.0f, 0.045f, 0.03f}, {0.0f, 0.045f, 0.20f}, 0.022f, 10, gold); // reservoir
    return m;
}

MeshData buildShockShaftMesh() {
    MeshData m;
    const Material chrome{MatId::Generic, {0.92f, 0.92f, 0.92f}, 0.10f, 1.0f, 0.3f};
    const Material black{MatId::Generic, {0.03f, 0.03f, 0.03f}, 0.5f, 0.3f, 0.5f};
    m.addCylinder({0, 0, 0}, {0, 0, kShockShaftLength}, 0.014f, 10, chrome);
    m.addCylinder({0, 0, 0}, {0, 0, 0.025f}, 0.064f, 18, black); // top hat / upper perch
    return m;
}

SuspensionPose suspensionPose(const glm::vec3& hub, float attachZ) {
    SuspensionPose s;
    const float sx = hub.x >= 0.0f ? 1.0f : -1.0f;
    const float zc = attachZ;
    s.shockTop = {sx * 0.585f, 0.29f, zc};
    s.armPivotA = {sx * 0.22f, -0.27f, zc + 0.24f};
    s.armPivotB = {sx * 0.22f, -0.27f, zc - 0.24f};
    s.armOuter = {hub.x - sx * 0.11f, hub.y - 0.11f, hub.z};
    const glm::vec3 pivotMid(sx * 0.22f, -0.27f, zc);
    const float span = std::abs(s.armOuter.x) - 0.22f;
    const float t = span > 1e-3f ? std::clamp((0.585f - 0.22f) / span, 0.0f, 1.0f) : 1.0f;
    s.shockLow = glm::mix(pivotMid, s.armOuter, t) + glm::vec3(0.0f, 0.03f, 0.0f);
    s.shaftInner = {sx * 0.13f, -0.24f, zc};
    s.shaftOuter = {hub.x - sx * 0.09f, hub.y, hub.z};
    return s;
}

glm::mat4 segmentMatrix(const glm::vec3& a, const glm::vec3& b) {
    const glm::vec3 d = b - a;
    const float len = glm::length(d);
    const glm::vec3 z = len > 1e-6f ? d / len : glm::vec3(0, 0, 1);
    glm::vec3 x = std::abs(z.y) < 0.9f ? glm::cross(glm::vec3(0, 1, 0), z) : glm::cross(glm::vec3(1, 0, 0), z);
    x = glm::normalize(x);
    const glm::vec3 y = glm::cross(z, x);
    glm::mat4 m(1.0f);
    m[0] = glm::vec4(x, 0);
    m[1] = glm::vec4(y, 0);
    m[2] = glm::vec4(d, 0);
    m[3] = glm::vec4(a, 1);
    return m;
}

glm::mat4 aimMatrix(const glm::vec3& a, const glm::vec3& b) {
    glm::mat4 m = segmentMatrix(a, b);
    const float len = glm::length(b - a);
    m[2] = len > 1e-6f ? m[2] / len : glm::vec4(0, 0, 1, 0);
    return m;
}

glm::mat4 spareWheelMatrix(float radius, float width) {
    (void)radius;
    glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.41f + 0.5f * width, -0.72f));
    return glm::rotate(m, 0.5f * kPi, glm::vec3(0, 0, 1));
}

} // namespace tf
