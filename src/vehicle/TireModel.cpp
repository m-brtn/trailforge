#include "vehicle/TireModel.h"

#include <algorithm>
#include <cmath>

namespace tf {
namespace {

constexpr float kPi = 3.14159265358979f;

constexpr const char* kKeys[kTireSetCount] = {"offroad", "road", "slick"};
constexpr const char* kNames[kTireSetCount] = {"OFF-ROAD", "ROAD", "SLICK"};

// Surface order: concrete, packed_dirt, rock, low_grip, mud, grass, sand, snow, asphalt, gravel.
//
// Off-road (all-terrain) set: the vehicle's own tire, unchanged. All-terrain / M+S compounds are made for a broad
// temperature range; the window (-10..100 C, at most -10 % below, -20 % above) is hand-tuned, not measured.
//
// Road set (summer performance tire): peak mu about 1.1-1.2 on dry asphalt / concrete against about 1.0 for an
// all-terrain tread (Milliken & Milliken, Race Car Vehicle Dynamics, ch. 2: street tires 1.0-1.2; skidpad results of
// SUVs on all-terrain tires about 0.8 g against 0.9-1.0 g on street tires): x1.15 on paved surfaces. On loose and
// soft ground a closed street tread has no lugs to shear the soil and clogs (Wong, Theory of Ground Vehicles, ch. 2:
// tractive effort of lugged vs. smooth tires); the multipliers there (mud 0.55, snow 0.60, grass 0.75, gravel 0.85)
// are hand-tuned. Stiffer tread blocks and construction: the peak at 0.8x the slip ratio and 0.75x the slip angle
// (passenger tires peak at about 0.08-0.12 slip and 4-7 deg, Pacejka, Tire and Vehicle Dynamics, ch. 1), a slightly
// larger drop beyond the peak. Summer compounds harden below about 7 C (the tire makers' summer / winter guidance);
// window 40..95 C with up to -20 % cold and -15 % hot, hand-tuned.
//
// Slick set (racing slick): warm racing slicks reach mu 1.4-1.8 at moderate load (Milliken, ch. 2); on a 2 t SUV with
// 5-6 kN per tire load sensitivity puts it at the low end: x1.40 on asphalt, x1.35 on concrete. No tread: no grip on
// loose, wet or soft ground (mud 0.35, snow 0.40, grass 0.55, gravel 0.70, hand-tuned). Racing tires peak at small
// slip (about 0.06-0.08, 3-5 deg) and fall off more sharply. Their compounds work in a narrow window (about 80-110 C
// for road racing slicks, e.g. published tyre-maker operating windows of 85-115 C); rubber friction rises steeply
// toward the window as the WLF temperature shift of the viscoelastic loss peak moves it to the sliding frequencies
// (Grosch 1963, Proc. R. Soc. A 274), so a cold slick is noticeably slow: up to -40 % 60 K below the window,
// -25 % 40 K above it (hand-tuned shape).
const TireCompound kCompounds[kTireSetCount] = {
    {.surfaceGrip = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
     .peakSlipScale = 1.0f, .peakSlipAngleScale = 1.0f, .slideRatioScale = 1.0f,
     .optimumLowC = -10.0f, .optimumHighC = 100.0f, .coldLoss = 0.10f, .coldWidthC = 30.0f,
     .hotLoss = 0.20f, .hotWidthC = 60.0f,
     // Light-truck all-terrain tires on pavement: 0.012-0.015 (Wong ch. 1; Gillespie, Fundamentals of Vehicle
     // Dynamics, ch. 4).
     .rollingResistance = 0.013f},
    {.surfaceGrip = {1.15f, 0.90f, 0.95f, 1.0f, 0.55f, 0.75f, 0.80f, 0.60f, 1.15f, 0.85f},
     .peakSlipScale = 0.8f, .peakSlipAngleScale = 0.75f, .slideRatioScale = 0.95f,
     .optimumLowC = 40.0f, .optimumHighC = 95.0f, .coldLoss = 0.20f, .coldWidthC = 45.0f,
     .hotLoss = 0.15f, .hotWidthC = 40.0f,
     // Passenger car radials: 0.008-0.012 (Gillespie ch. 4; TRB Special Report 286, 2006).
     .rollingResistance = 0.010f},
    {.surfaceGrip = {1.35f, 0.75f, 0.85f, 0.90f, 0.35f, 0.55f, 0.65f, 0.40f, 1.40f, 0.70f},
     .peakSlipScale = 0.7f, .peakSlipAngleScale = 0.65f, .slideRatioScale = 0.90f,
     .optimumLowC = 80.0f, .optimumHighC = 110.0f, .coldLoss = 0.40f, .coldWidthC = 60.0f,
     .hotLoss = 0.25f, .hotWidthC = 40.0f,
     // High-hysteresis racing compound: hand-tuned above the street tire.
     .rollingResistance = 0.014f},
};

float smoothStep01(float x) {
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

} // namespace

const char* tireSetKey(TireSet s) { return std::size_t(s) < kTireSetCount ? kKeys[std::size_t(s)] : ""; }
const char* tireSetName(TireSet s) { return std::size_t(s) < kTireSetCount ? kNames[std::size_t(s)] : ""; }

TireSet tireSetFromKey(std::string_view key) {
    for (std::size_t i = 0; i < kTireSetCount; ++i)
        if (key == kKeys[i]) return TireSet(i);
    return TireSet::Count;
}

const TireCompound& tireCompound(TireSet s) {
    return kCompounds[std::min(std::size_t(s), kTireSetCount - 1)];
}

float compoundGripFactor(const TireCompound& c, float t) {
    if (t >= c.optimumLowC && t <= c.optimumHighC) return 1.0f;
    if (t < c.optimumLowC) return 1.0f - c.coldLoss * smoothStep01((c.optimumLowC - t) / c.coldWidthC);
    return 1.0f - c.hotLoss * smoothStep01((t - c.optimumHighC) / c.hotWidthC);
}

float compoundTemperatureState(const TireCompound& c, float t) {
    if (t < c.optimumLowC) return -std::clamp((c.optimumLowC - t) / c.coldWidthC, 0.0f, 1.0f);
    if (t > c.optimumHighC) return std::clamp((t - c.optimumHighC) / c.hotWidthC, 0.0f, 1.0f);
    return 0.0f;
}

// Material values: filled tread rubber k 0.2-0.3 W/(m K), rho 1100-1200 kg/m^3, c 1.7-2.0 kJ/(kg K) (Gent & Walter,
// The Pneumatic Tire, NHTSA 2006, ch. 7 / 14); the carcass with steel belts and cords has a lower mean specific heat
// (steel 0.49 kJ/(kg K)). Geometry of the nodes: a 2 mm surface layer over the tread band (its diffusion time
// d^2 / alpha ~ 35 s is the time scale on which it follows the core), the rest of the
// tire as the core; tire mass about 60 % of the wheel + tire mass of the vehicle config (an LT 285/70 R17 all-terrain
// weighs ~22 kg on a ~14 kg rim). The node distance for the conduction is 6 mm, the rim path 5 W/K (rim, hub and
// inner air at ambient: brake heat is not modeled). Hand-tuned: the 2 mm layer, the 60 % mass share, the rim path,
// the 6 mm distance, the sliding and hysteresis shares below.
TireThermalParams tireThermalParams(float radius, float width, float wheelMass, const TireCompound& compound) {
    TireThermalParams p;
    constexpr float kRho = 1150.0f, kTreadC = 1800.0f, kCoreC = 1500.0f, kRubberK = 0.25f;
    constexpr float kLayer = 0.002f, kNodeDistance = 0.006f;
    p.treadArea = 2.0f * kPi * radius * width;
    const float surfaceMass = p.treadArea * kLayer * kRho;
    const float tireMass = std::max(0.6f * wheelMass, 3.0f * surfaceMass);
    p.surfaceCapacity = surfaceMass * kTreadC;
    p.coreCapacity = (tireMass - surfaceMass) * kCoreC;
    p.surfaceCoreConductance = kRubberK * p.treadArea / kNodeDistance;
    const float rim = 0.6f * radius; // sidewall height ~40 % of the radius
    p.sidewallArea = 2.0f * kPi * (radius * radius - rim * rim);
    p.rimConductance = 5.0f;
    p.diameter = 2.0f * radius;
    // Sliding friction heat at a sharp interface divides between the bodies by their effusivities (Blok's partition:
    // about 0.35-0.45 into rubber on asphalt / concrete), but on rough road surfaces rubber friction is mostly
    // hysteresis: the asperities deform the rubber and that energy is dissipated inside the tread (Persson, Theory of
    // rubber friction and contact mechanics, J. Chem. Phys. 115, 2001). 0.85 into the tire is hand-tuned on that.
    p.slidingHeatShare = 0.85f;
    // Rolling loss: the tread carries 60-70 % of it (TRB Special Report 286, 2006, ch. 2), the 2 mm surface layer a
    // fraction of that.
    p.hysteresisSurfaceShare = 0.2f;
    p.rollingResistance = compound.rollingResistance;
    p.coldPressureKPa = 240.0f; // typical SUV placard pressure, gauge
    p.naturalConvection = 5.0f; // free convection 2-25 W/(m^2 K) (Incropera, Fundamentals of Heat and Mass Transfer, Table 1.1)
    return p;
}

float TireThermalState::pressureKPa(const TireThermalParams& p) const {
    constexpr float kAtm = 101.325f;
    return (p.coldPressureKPa + kAtm) * (coreC + 273.15f) / (startC + 273.15f) - kAtm;
}

// Churchill & Bernstein (1977) for a cylinder in cross flow, air at ~300 K (nu 1.6e-5 m^2/s, k 0.026 W/(m K),
// Pr 0.71). The rotating tire inside a wheel arch is not a free cylinder; this is an estimate of the right order
// (tire thermal models use 20-60 W/(m^2 K) at road speeds).
float tireForcedConvection(float diameter, float airSpeed) {
    constexpr float kNu = 1.6e-5f, kK = 0.026f, kPr = 0.71f;
    const float re = std::max(airSpeed, 0.0f) * diameter / kNu;
    if (re <= 0.0f || diameter <= 0.0f) return 0.0f;
    const float pr13 = std::cbrt(kPr);
    const float nu = 0.3f + 0.62f * std::sqrt(re) * pr13 / std::pow(1.0f + std::pow(0.4f / kPr, 2.0f / 3.0f), 0.25f) *
                                std::pow(1.0f + std::pow(re / 282000.0f, 5.0f / 8.0f), 4.0f / 5.0f);
    return nu * kK / diameter;
}

// Two semi-infinite bodies at different temperatures put in contact: the interface flux is e_eff dT / sqrt(pi t) with
// e_eff = e1 e2 / (e1 + e2); averaged over the contact time t_c the coefficient is 2 e_eff / sqrt(pi t_c) (Carslaw &
// Jaeger, Conduction of Heat in Solids, 2.4).
float patchContactConductance(float tireEffusivity, float groundEffusivity, float contactTime) {
    if (groundEffusivity <= 0.0f || contactTime <= 0.0f) return 0.0f;
    const float e = tireEffusivity * groundEffusivity / (tireEffusivity + groundEffusivity);
    return 2.0f * e / std::sqrt(kPi * contactTime);
}

void stepTireThermal(const TireThermalParams& p, const TireThermalInput& in, float dt, TireThermalState& s) {
    // Heat sources.
    s.heatSliding = in.contact ? p.slidingHeatShare * std::clamp(in.slidingShare, 0.0f, 1.0f) * std::max(in.slidingPower, 0.0f) : 0.0f;
    s.heatRolling = in.contact ? p.rollingResistance * std::max(in.load, 0.0f) * std::abs(in.rollingSpeed) : 0.0f;

    // Convection: tread band from the surface node, sidewalls from the core.
    const float h = p.convection ? p.naturalConvection + tireForcedConvection(p.diameter, in.airSpeed) : 0.0f;
    const float toAirSurface = h * p.treadArea * (s.surfaceC - in.ambientC);
    const float toAirCore = h * p.sidewallArea * (s.coreC - in.ambientC);

    // Ground: the patch (area ~ load / inflation pressure, Gent & Walter ch. 7) is in contact for the passage time
    // L / v (patch length L = area / width, v floored at 0.5 m/s: a standing tire stays at the cooled patch). In series
    // with conduction from the middle of the surface layer to the skin over the whole band (the patch sweeps it).
    s.toGround = 0.0f;
    s.patchArea = 0.0f;
    if (in.contact && in.load > 0.0f && in.groundEffusivity > 0.0f) {
        const float pressure = std::max(s.pressureKPa(p), 50.0f) * 1000.0f;
        s.patchArea = in.load / pressure;
        const float width = p.treadArea / (kPi * p.diameter);
        const float length = s.patchArea / std::max(width, 0.05f);
        const float contactTime = length / std::max(std::abs(in.rollingSpeed), 0.5f);
        // Rubber touches the asperities of a road only on part of the nominal patch (Persson 2001; the real contact
        // area is a fraction of the footprint): 30 %, hand-tuned.
        constexpr float kRealContact = 0.3f;
        const float patch = patchContactConductance(kTireRubberEffusivity, in.groundEffusivity, contactTime) * s.patchArea * kRealContact;
        const float layer = 0.25f * p.treadArea / 0.001f; // rubber k over half the 2 mm layer
        const float g = patch * layer / (patch + layer);
        s.toGround = g * (s.surfaceC - in.groundC);
    }
    s.surfaceToCore = p.surfaceCoreConductance * (s.surfaceC - s.coreC);
    s.toRim = p.rimConductance * (s.coreC - in.ambientC);
    s.toAir = toAirSurface + toAirCore;

    const float qSurface = s.heatSliding + p.hysteresisSurfaceShare * s.heatRolling - s.surfaceToCore - toAirSurface - s.toGround;
    const float qCore = (1.0f - p.hysteresisSurfaceShare) * s.heatRolling + s.surfaceToCore - toAirCore - s.toRim;
    s.surfaceC += qSurface * dt / p.surfaceCapacity;
    s.coreC += qCore * dt / p.coreCapacity;
}

// Effusivity sqrt(k rho c), W s^0.5 / (m^2 K), from typical properties (Incropera Tables A.3 / A.7): concrete
// (1.4, 2300, 880), asphalt concrete (0.75, 2300, 920), granite (2.8, 2630, 775), dry soil (0.52, 2050, 1840),
// gravel (0.4, 1800, 840, loose with voids), sand (0.27, 1515, 800), snow (0.19, 500, 2100), wet soil / mud
// (1.5, 1900, 1800); grass on soil and the hard low-grip patch are hand-picked between those.
float surfaceEffusivity(SurfaceId id) {
    switch (id) {
    case SurfaceId::Concrete: return 1680.0f;
    case SurfaceId::PackedDirt: return 1400.0f;
    case SurfaceId::Rock: return 2390.0f;
    case SurfaceId::LowGrip: return 1680.0f;
    case SurfaceId::Mud: return 2270.0f;
    case SurfaceId::Grass: return 1000.0f;
    case SurfaceId::Sand: return 570.0f;
    case SurfaceId::Snow: return 450.0f;
    case SurfaceId::Asphalt: return 1260.0f;
    case SurfaceId::Gravel: return 780.0f;
    case SurfaceId::Count: break;
    }
    return 1400.0f;
}

// The ground is at the air temperature (no sun heating, no night cooling); snow cannot be above 0 C.
float surfaceGroundTemperature(SurfaceId id, float ambientC) {
    return id == SurfaceId::Snow ? std::min(ambientC, 0.0f) : ambientC;
}

// On loose or wet ground a large part of the slip is shear inside the soil, between particles or in a water film,
// and that work heats the ground, not the rubber (Wong, Theory of Ground Vehicles, ch. 2: the tractive slip of a
// tire on soft soil is mostly soil shear deformation). Hand-tuned values: paved 1, wet smooth patch and packed dirt
// 0.6, gravel and grass 0.4, sand 0.3, mud and snow 0.2.
float surfaceSlidingHeatFactor(SurfaceId id) {
    switch (id) {
    case SurfaceId::Concrete:
    case SurfaceId::Rock:
    case SurfaceId::Asphalt: return 1.0f;
    case SurfaceId::LowGrip:
    case SurfaceId::PackedDirt: return 0.6f;
    case SurfaceId::Gravel:
    case SurfaceId::Grass: return 0.4f;
    case SurfaceId::Sand: return 0.3f;
    case SurfaceId::Mud:
    case SurfaceId::Snow: return 0.2f;
    case SurfaceId::Count: break;
    }
    return 1.0f;
}

} // namespace tf
