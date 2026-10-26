#pragma once

#include "track/SurfaceProperties.h"

#include <array>
#include <cstdint>
#include <string_view>

// Tire sets (compounds) and the tire thermal model. No Jolt, no SDL: the vehicle applies the compound to its tire
// curves and steps one TireThermal per wheel every physics tick (Vehicle.cpp); the tests drive the model directly.

namespace tf {

// The tire set fitted to all four wheels. The vehicle's own tire (vehicle.json "tires") is the off-road set; the
// other sets are expressed relative to it, so every vehicle keeps its own base curve.
enum class TireSet : std::uint8_t { Offroad = 0, Road, Slick, Count };
inline constexpr std::size_t kTireSetCount = static_cast<std::size_t>(TireSet::Count);

const char* tireSetKey(TireSet s);  // "offroad", "road", "slick" (vehicle.json, scenario `tires`)
const char* tireSetName(TireSet s); // "OFF-ROAD", "ROAD", "SLICK" (garage, HUD)
TireSet tireSetFromKey(std::string_view key); // TireSet::Count for an unknown key

// Grip and heat properties of a tire set, relative to the vehicle's off-road tire.
struct TireCompound {
    // Multiplier on the vehicle's tire curve per surface (longitudinal and lateral), at the compound's optimum
    // temperature. Applied on top of the surface table and vehicle.json tires.surfaceGrip.
    std::array<float, kSurfaceCount> surfaceGrip{};
    float peakSlipScale = 1.0f;      // x the vehicle's peak slip ratio (stiffer tread: smaller)
    float peakSlipAngleScale = 1.0f; // x the vehicle's peak slip angle (stiffer construction: smaller)
    float slideRatioScale = 1.0f;    // x the vehicle's sliding / peak mu ratio (peakier compound: smaller)
    // Grip versus tread surface temperature: 1 within [optimumLowC, optimumHighC]; below it the grip falls by up to
    // coldLoss over coldWidthC, above it by up to hotLoss over hotWidthC (smoothstep, then constant).
    float optimumLowC = 0.0f, optimumHighC = 100.0f;
    float coldLoss = 0.0f, coldWidthC = 30.0f;
    float hotLoss = 0.0f, hotWidthC = 50.0f;
    // Rolling resistance coefficient: the hysteresis heat of a rolling tire (Crr x load x speed). Heat only: the
    // paved-road rolling resistance force itself is not applied (README "Physics simplifications").
    float rollingResistance = 0.012f;
};

const TireCompound& tireCompound(TireSet s);
// Grip multiplier of the compound at a tread surface temperature (exactly 1 inside the optimum window).
float compoundGripFactor(const TireCompound& c, float surfaceTempC);
// -1 (cold end of the window) .. 0 (inside) .. +1 (hot end): colour of the HUD tire widget.
float compoundTemperatureState(const TireCompound& c, float surfaceTempC);

// Two-node lumped thermal model of one tire: the tread surface layer (what touches the road and sets the grip) and
// the core (rest of the tread, belts, carcass, sidewalls). Heat flows:
//   surface: + share of the contact patch sliding power + share of the rolling hysteresis
//            - conduction to the core - convection to the air - conduction into the ground in the patch
//   core:    + rest of the rolling hysteresis - conduction to the surface (sign) - convection from the sidewalls
//            - conduction through the rim (rim at ambient)
// Explicit Euler per physics tick (the smallest time constant is tens of seconds, the step 1/120 s).
struct TireThermalParams {
    float surfaceCapacity = 0.0f;  // J/K, tread surface layer
    float coreCapacity = 0.0f;     // J/K
    float surfaceCoreConductance = 0.0f; // W/K
    float treadArea = 0.0f;        // m^2, outer tread band (convection of the surface node)
    float sidewallArea = 0.0f;     // m^2, both sidewalls (convection of the core)
    float rimConductance = 0.0f;   // W/K, core -> rim -> ambient
    float diameter = 0.0f;         // m, for the convection correlation
    float slidingHeatShare = 0.0f; // share of the patch sliding power that heats the tire (rest heats the road)
    float hysteresisSurfaceShare = 0.0f; // share of the rolling hysteresis heat in the surface layer
    float rollingResistance = 0.0f;
    float coldPressureKPa = 0.0f;  // gauge, set at the start temperature
    float naturalConvection = 0.0f; // W/(m^2 K), still-air floor added to the forced convection
    bool convection = true;         // tests: false = no air cooling
};

// Thermal parameters of a tire of this size and set (outer radius, width, wheel + tire mass of the vehicle config).
TireThermalParams tireThermalParams(float radius, float width, float wheelMass, const TireCompound& compound);

// What the tire sees during one tick.
struct TireThermalInput {
    bool contact = false;
    float load = 0.0f;           // N
    float slidingPower = 0.0f;   // W, |Fx * slip speed x| + |Fy * slip speed y| in the patch
    float rollingSpeed = 0.0f;   // m/s, |w * r|
    float airSpeed = 0.0f;       // m/s, vehicle speed through the air (still air)
    float ambientC = 20.0f;      // air temperature
    float groundC = 20.0f;       // ground temperature under the patch
    float groundEffusivity = 0.0f; // W s^0.5 / (m^2 K), sqrt(k rho c) of the ground
    float slidingShare = 1.0f;   // scales slidingHeatShare on this ground (surfaceSlidingHeatFactor)
};

struct TireThermalState {
    float surfaceC = 20.0f;
    float coreC = 20.0f;
    float startC = 20.0f; // temperature at which the pressure was set
    // Diagnostics of the last step, W (+ = into the tire).
    float heatSliding = 0.0f, heatRolling = 0.0f, toAir = 0.0f, toGround = 0.0f, toRim = 0.0f, surfaceToCore = 0.0f;
    float patchArea = 0.0f; // m^2

    void reset(float tempC) { *this = TireThermalState{tempC, tempC, tempC}; }
    float pressureKPa(const TireThermalParams& p) const; // ideal gas at constant volume, from the core temperature
};

// Forced convection coefficient of a cylinder of this diameter in cross flow at this air speed (Churchill-Bernstein),
// W/(m^2 K), without the natural-convection floor.
float tireForcedConvection(float diameter, float airSpeed);
// Heat transfer coefficient of the contact patch: two semi-infinite bodies brought into contact for the patch
// passage time (Carslaw & Jaeger), W/(m^2 K).
float patchContactConductance(float tireEffusivity, float groundEffusivity, float contactTime);

void stepTireThermal(const TireThermalParams& p, const TireThermalInput& in, float dt, TireThermalState& s);

// Thermal properties of the ground surfaces: effusivity sqrt(k rho c) and whether the ground is capped at 0 C (snow).
float surfaceEffusivity(SurfaceId id);
float surfaceGroundTemperature(SurfaceId id, float ambientC);
// Share of the tire's sliding heat that stays in the tire on this ground: 1 on paved surfaces, less where the slip
// happens inside loose or wet ground.
float surfaceSlidingHeatFactor(SurfaceId id);

inline constexpr float kTireRubberEffusivity = 720.0f; // sqrt(0.25 W/mK * 1150 kg/m^3 * 1800 J/kgK)
inline constexpr float kDefaultAmbientC = 20.0f;

} // namespace tf
