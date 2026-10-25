#pragma once

#include "audio/AudioTypes.h"
#include "track/SurfaceProperties.h"

#include <array>

namespace tf {

// Pure parameter mapping from game state to mix parameters. No device, no state: unit-tested headless.

// {cos, sin} of x*pi/2 for x in 0..1: the squares sum to 1 (constant power).
std::array<float, 2> equalPower(float x);

// Engine: three rpm bands (idle/low, mid, high) x two loads (off, full) x two mics.
// Crossfades are equal-power on log(source rpm), where source rpm = engine rpm x pitch, so a band is played back
// close to the rpm it was recorded at. Defaults follow the sourcing notes: idle->mid 1400..2000, mid->high
// 3200..4800 source rpm (both contain the geometric mean of the neighbouring measured rpm).
struct EngineMixSettings {
    float pitch = 0.80f;
    float idleMidLo = 1400.0f, idleMidHi = 2000.0f;
    float midHighLo = 3200.0f, midHighHi = 4800.0f;
};

// Weights of the low/mid/high band, sum of squares = 1.
std::array<float, 3> engineBandWeights(float sourceRpm, const EngineMixSettings& s);

struct EngineLayerWeights {
    std::array<float, 3> off{}; // idle, mid, high recorded off-load
    std::array<float, 3> on{};  // low, mid, high recorded at full load
};
// load 0..1 blends off-load -> full-load (equal power). Sum of squares of all six weights = 1.
EngineLayerWeights engineLayerWeights(float engineRpm, float load, const EngineMixSettings& s);

// Playback rate of a loop recorded at rpmMeasured so that it sounds at engine rpm x pitch.
float enginePlaybackRate(float engineRpm, float pitch, float rpmMeasured);

// The loops are loudness-normalized; their recorded levels (sourceDb, relative to the loudest take) span ~40 dB.
// Map [minDb, maxDb] linearly onto [-spreadDb, 0] so quiet takes stay quieter, but only by spreadDb.
float compressedLevelDb(float sourceDb, float minDb, float maxDb, float spreadDb);

// Tires: loose-ground loops shared by all wheels and aggregated per layer; the rubber squeal (Skid) is per wheel.
enum class TireLayer : std::uint8_t { Skid, Gravel, Dirt, Mud, Grass, Count };
inline constexpr int kTireLayerCount = int(TireLayer::Count);

// How a surface drives each loose-ground layer: `roll` scales the rolling intensity, `slip` the sliding intensity.
// Skid is never driven from here: only squealSurface() and the per-wheel squeal state decide about rubber.
struct TireSurfaceResponse {
    std::array<float, kTireLayerCount> roll{};
    std::array<float, kTireLayerCount> slip{};
};
TireSurfaceResponse tireSurfaceResponse(SurfaceId surface);

// Rubber squeal gain of a surface: concrete 1, rock and the low-grip patch weaker, every loose surface 0.
float squealSurface(SurfaceId surface);

// Rolling and sliding intensity of one wheel, 0..~1.2. Rolling grows with patch speed; sliding with the sliding
// speed gated by the slip ratio. Both scale with the load relative to the static wheel load.
float wheelRollIntensity(const WheelAudio& w, float referenceLoad);
float wheelSlipIntensity(const WheelAudio& w, float referenceLoad);
// How much one wheel slides for the rubber squeal, 0..1, from the combined (longitudinal + lateral) patch slip.
// Proportional between onset and full: the slip ratios combined on an ellipse (onset at longitudinal 0.55 or lateral
// 0.40, full at 1.7 x that) times the combined sliding speed 2.5..9 m/s, gated by the patch ground speed
// (1..2.5 m/s) and the wheel load (0.15..0.4 of the static load). Stricter than the rubber-mark gate (rubberSlide):
// the SUV's full-throttle 2H launch on concrete spins its rear wheels at ~5 m/s and ratio ~0.5 for ~1.5 s, a 30 km/h
// full-lock turn scrubs at 4 m/s / 0.45, ABS stops stay near ratio 0.3; all stay silent. Lock-up, handbrake and power
// slides (ratio ~1, 8-15 m/s) reach 0.9-1; a full-lock plow at 6.6 m/s sideways / 0.5 squeals partly.
float squealSlip(const WheelAudio& w, float referenceLoad);
// Instantaneous squeal drive of one wheel, 0..~1.4: squealSurface x squealSlip x load x clean tread. Smoothing and
// hysteresis are applied per tick in VehicleAudioFeed.
float wheelSquealDrive(const WheelAudio& w, float referenceLoad);

struct TireLayerMix {
    std::array<float, kTireLayerCount> gain{};     // 0..~1.2 (RMS of the four wheel intensities); Skid from `squeal`
    std::array<AudioVec3, kTireLayerCount> position{}; // intensity-weighted wheel position
};
TireLayerMix tireLayerMix(const std::array<WheelAudio, kAudioWheels>& wheels, float referenceLoad);

// Electric drive sound, synthesized per motor (no recordings): the electromagnetic order (2 x pole pairs) and its
// second harmonic, the stator slot order, the two gear meshes of the reduction and the inverter switching tone.
// Torque sets the level (magnetic forces and gear mesh forces grow with it; a small no-load share remains), the
// shaft speed sets the pitch. Regeneration shifts weight from the electromagnetic order to the slot order (a thinner,
// higher whine, as reported for many EVs) and gear meshes load their coast flanks with the same torque. The inverter
// tone is a fixed carrier with a sideband at carrier + 2 x electrical frequency, loudest at low speed and fading out
// above inverterFadeSpeed (variable carrier schemes move it out of the audible band). Tones roll off above ~5 kHz.
struct EvSoundSettings {
    float polePairs = 4.0f;
    float slots = 48.0f;
    float stage1Teeth = 21.0f;   // pinion on the motor shaft
    float stage1Ratio = 3.6f;    // the second mesh runs on the intermediate shaft (motor speed / stage1Ratio)
    float stage2Teeth = 17.0f;
    float inverterHz = 3200.0f;
    float inverterFadeSpeed = 12.0f; // m/s
    float level = 0.022f;        // linear gain of the electromagnetic order at full torque
};
inline constexpr int kEvTones = 6; // em, em x2, slot, gear 1, gear 2, inverter (+ sideband in the mixer)
struct EvTones {
    std::array<float, kEvTones> hz{}, gain{};
};
EvTones evMotorTones(const MotorAudio& m, float vehicleSpeed, const EvSoundSettings& s = {});

// Rolling road hum of one wheel, 0..~1.2: hard surfaces only (squealSurface > 0), rises with the rolling speed
// (from 2 m/s, full at 25 m/s) and the load.
float roadHum(const WheelAudio& w, float referenceLoad);

// Ambience: equal-power day/night weights from dayAmount 0..1.
std::array<float, 2> dayNightWeights(float dayAmount);

// Wind loop gain from vehicle speed (m/s): silent below ~3 m/s, quadratic rise, 1 at 30 m/s.
float windGain(float speed);

// Mono source -> stereo gains for a listener: equal-power pan from the direction relative to the listener's
// right axis (limited to +-0.8 so nothing is hard-panned), inverse-distance attenuation beyond refDistance and a
// fade to silence toward maxDistance, slightly quieter behind the listener.
struct StereoGain {
    float l = 0.0f, r = 0.0f;
};
StereoGain spatialize(AudioVec3 source, AudioVec3 listenerPos, AudioVec3 listenerRight, AudioVec3 listenerForward,
                      float refDistance = 5.0f, float maxDistance = 200.0f);

inline float dbToGain(float db) { return std::pow(10.0f, db / 20.0f); }
inline float gainToDb(float g) { return g > 1e-6f ? 20.0f * std::log10(g) : -120.0f; }

} // namespace tf
