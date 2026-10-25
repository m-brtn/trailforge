// Headless audio checks: parameter mapping, manifest, and the feed + mixer driven by the real simulation with a
// synthetic sound bank (no audio device). Usage: trailforge_audio_tests <test_name>

#include "audio/AudioManifest.h"
#include "audio/AudioMapping.h"
#include "audio/AudioMixer.h"
#include "audio/VehicleAudioFeed.h"
#include "sim/Simulation.h"
#include "sim/TireMarks.h"
#include "vehicle/VehicleDefinition.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

using namespace tf;

namespace {

int g_failures = 0;

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("  FAIL %s:%d: %s — ", __FILE__, __LINE__, #cond);      \
            std::printf(__VA_ARGS__);                                           \
            std::printf("\n");                                                  \
            ++g_failures;                                                       \
        }                                                                       \
    } while (0)

const std::string kManifest = std::string(TRAILFORGE_SOURCE_DIR) + "/assets/audio/manifest.json";

float sumSquares(const EngineLayerWeights& w) {
    float s = 0.0f;
    for (int i = 0; i < 3; ++i) s += w.off[std::size_t(i)] * w.off[std::size_t(i)] + w.on[std::size_t(i)] * w.on[std::size_t(i)];
    return s;
}

// ---------------------------------------------------------------------------

void engineLayers() {
    EngineMixSettings s; // pitch 0.8, crossfades 1400..2000 and 3200..4800 source rpm
    float worst = 0.0f;
    for (float rpm = 300.0f; rpm <= 7000.0f; rpm += 25.0f)
        for (float load = 0.0f; load <= 1.0f; load += 0.05f)
            worst = std::max(worst, std::abs(sumSquares(engineLayerWeights(rpm, load, s)) - 1.0f));
    std::printf("    max |sum w^2 - 1| over rpm 300..7000, load 0..1: %.2e\n", worst);
    CHECK(worst < 1e-5f, "equal-power sum deviates by %.2e", worst);

    // Pure bands well inside each range (source rpm = rpm x 0.8).
    const auto idle = engineBandWeights(1000.0f, s), mid = engineBandWeights(2500.0f, s), high = engineBandWeights(5200.0f, s);
    CHECK(idle[0] == 1.0f && idle[1] == 0.0f && idle[2] == 0.0f, "1000 src rpm: %.3f %.3f %.3f", idle[0], idle[1], idle[2]);
    CHECK(std::abs(mid[1] - 1.0f) < 1e-6f && mid[0] < 1e-6f && mid[2] < 1e-6f, "2500 src rpm: %.3f %.3f %.3f", mid[0], mid[1], mid[2]);
    CHECK(std::abs(high[2] - 1.0f) < 1e-6f, "5200 src rpm: high %.3f", high[2]);
    // Log-rpm midpoint of a crossfade: both at 1/sqrt(2).
    const float gm = std::sqrt(1400.0f * 2000.0f);
    const auto x = engineBandWeights(gm, s);
    std::printf("    at %.0f src rpm: low %.4f mid %.4f\n", gm, x[0], x[1]);
    CHECK(std::abs(x[0] - 0.70710678f) < 1e-4f && std::abs(x[1] - 0.70710678f) < 1e-4f, "midpoint %.4f %.4f", x[0], x[1]);
    // Monotonic: the low band only fades out, the high band only fades in, as rpm rises.
    float prevLow = 2.0f, prevHigh = -1.0f;
    for (float r = 500.0f; r < 6000.0f; r += 10.0f) {
        const auto b = engineBandWeights(r, s);
        CHECK(b[0] <= prevLow + 1e-6f && b[2] >= prevHigh - 1e-6f, "not monotonic at %.0f", r);
        prevLow = b[0];
        prevHigh = b[2];
    }
    // Load: off -> on.
    const auto off = engineLayerWeights(3000.0f, 0.0f, s), on = engineLayerWeights(3000.0f, 1.0f, s);
    CHECK(on.off[1] < 1e-6f && off.on[1] < 1e-6f && off.off[1] > 0.99f && on.on[1] > 0.99f, "load blend");

    // Playback rate = rpm x pitch / measured rpm.
    const float r1 = enginePlaybackRate(3000.0f, 0.8f, 2430.0f);
    const float r2 = enginePlaybackRate(800.0f, 0.8f, 1010.0f);
    std::printf("    rate(3000 rpm, pitch 0.8, 2430) = %.4f, rate(800, 0.8, 1010) = %.4f\n", r1, r2);
    CHECK(std::abs(r1 - 0.98765f) < 1e-4f && std::abs(r2 - 0.63366f) < 1e-4f, "rates %.4f %.4f", r1, r2);
    CHECK(std::abs(enginePlaybackRate(2430.0f, 1.0f, 2430.0f) - 1.0f) < 1e-6f, "unity rate");

    // Level spread: [-36.4, 0.6] dB -> [-12, 0] dB.
    CHECK(std::abs(compressedLevelDb(-36.4f, -36.4f, 0.6f, 12.0f) + 12.0f) < 1e-4f, "min level");
    CHECK(std::abs(compressedLevelDb(0.6f, -36.4f, 0.6f, 12.0f)) < 1e-4f, "max level");
    CHECK(std::abs(compressedLevelDb(-17.9f, -36.4f, 0.6f, 12.0f) + 6.0f) < 1e-3f, "mid level %.3f",
          compressedLevelDb(-17.9f, -36.4f, 0.6f, 12.0f));
}

WheelAudio wheel(SurfaceId s, float roll, float slip, float ratio, float load = 4905.0f) {
    WheelAudio w;
    w.contact = true;
    w.surface = std::uint8_t(s);
    w.rollSpeed = roll;
    w.slipSpeed = slip;
    w.slipRatio = ratio;
    w.longRatio = ratio; // longitudinal unless set otherwise
    w.load = load;
    return w;
}
WheelAudio sideways(SurfaceId s, float roll, float slip, float longRatio, float latRatio) {
    WheelAudio w = wheel(s, roll, slip, std::sqrt(longRatio * longRatio + latRatio * latRatio));
    w.longRatio = longRatio;
    w.latRatio = latRatio;
    return w;
}

void printMix(const char* what, const TireLayerMix& m) {
    std::printf("    %-28s skid %.2f gravel %.2f dirt %.2f mud %.2f grass %.2f\n", what, m.gain[0], m.gain[1],
                m.gain[2], m.gain[3], m.gain[4]);
}

void tireLayers() {
    const float ref = 4905.0f;
    auto all = [](const WheelAudio& w) { return std::array<WheelAudio, 4>{w, w, w, w}; };
    using L = TireLayer;
    auto g = [](const TireLayerMix& m, L l) { return m.gain[std::size_t(l)]; };

    // The skid layer only follows the per-wheel squeal the feed computed (gated, smoothed); here it is set directly.
    auto squealing = [&](WheelAudio w) {
        w.squeal = wheelSquealDrive(w, ref);
        return w;
    };
    const TireLayerMix concreteRoll = tireLayerMix(all(wheel(SurfaceId::Concrete, 10, 0.3f, 0.03f)), ref);
    const TireLayerMix concreteLock = tireLayerMix(all(squealing(wheel(SurfaceId::Concrete, 8, 8, 1.0f))), ref);
    const TireLayerMix tractionSlip = tireLayerMix(all(squealing(wheel(SurfaceId::Concrete, 8, 1.7f, 0.12f))), ref);
    const TireLayerMix dirtRoll = tireLayerMix(all(wheel(SurfaceId::PackedDirt, 8, 0.2f, 0.02f)), ref);
    const TireLayerMix dirtSpin = tireLayerMix(all(wheel(SurfaceId::PackedDirt, 2, 4, 0.8f)), ref);
    const TireLayerMix mudRoll = tireLayerMix(all(wheel(SurfaceId::Mud, 6, 0.3f, 0.05f)), ref);
    const TireLayerMix grassRoll = tireLayerMix(all(wheel(SurfaceId::Grass, 6, 0.3f, 0.05f)), ref);
    WheelAudio air = wheel(SurfaceId::Concrete, 8, 8, 1.0f);
    air.contact = false;
    const TireLayerMix inAir = tireLayerMix(all(air), ref);
    printMix("concrete rolling 10 m/s:", concreteRoll);
    printMix("concrete lock-up 8 m/s:", concreteLock);
    printMix("concrete 12 % traction slip:", tractionSlip);
    printMix("dirt rolling 8 m/s:", dirtRoll);
    printMix("dirt wheelspin:", dirtSpin);
    printMix("mud rolling 6 m/s:", mudRoll);
    printMix("grass rolling 6 m/s:", grassRoll);
    printMix("wheels in the air:", inAir);

    for (int l = 0; l < kTireLayerCount; ++l) {
        CHECK(concreteRoll.gain[std::size_t(l)] == 0.0f, "concrete rolling layer %d = %.3f", l, concreteRoll.gain[std::size_t(l)]);
        CHECK(inAir.gain[std::size_t(l)] == 0.0f, "air layer %d", l);
    }
    CHECK(g(concreteLock, L::Skid) > 0.9f, "lock-up skid %.2f", g(concreteLock, L::Skid));
    CHECK(g(concreteLock, L::Dirt) == 0.0f && g(concreteLock, L::Mud) == 0.0f, "lock-up selects soft layers");
    CHECK(g(tractionSlip, L::Skid) < 0.05f, "steady traction slip squeals: %.3f", g(tractionSlip, L::Skid));
    CHECK(g(dirtRoll, L::Dirt) > 0.5f && g(dirtRoll, L::Gravel) > 0.0f && g(dirtRoll, L::Gravel) < g(dirtRoll, L::Dirt),
          "dirt roll %.2f gravel %.2f", g(dirtRoll, L::Dirt), g(dirtRoll, L::Gravel));
    CHECK(g(dirtRoll, L::Skid) == 0.0f, "dirt rolling squeals");
    CHECK(g(dirtSpin, L::Gravel) > g(dirtRoll, L::Gravel), "dirt spin should scrub gravel");
    CHECK(g(mudRoll, L::Mud) > 0.5f && g(mudRoll, L::Grass) == 0.0f && g(mudRoll, L::Skid) == 0.0f, "mud selection");
    CHECK(g(grassRoll, L::Grass) > 0.5f && g(grassRoll, L::Mud) == 0.0f && g(grassRoll, L::Dirt) == 0.0f,
          "grass selection");
    // Load: an unloaded wheel is quieter than a loaded one.
    const float light = wheelSlipIntensity(wheel(SurfaceId::PackedDirt, 8, 8, 1, 1000), ref);
    const float heavy = wheelSlipIntensity(wheel(SurfaceId::PackedDirt, 8, 8, 1, 6000), ref);
    CHECK(light < heavy, "slip x load: %.2f vs %.2f", light, heavy);
    const float lightSq = wheelSquealDrive(squealing(wheel(SurfaceId::Concrete, 8, 8, 1, 1000)), ref);
    const float heavySq = wheelSquealDrive(squealing(wheel(SurfaceId::Concrete, 8, 8, 1, 6000)), ref);
    CHECK(lightSq < heavySq, "squeal x load: %.2f vs %.2f", lightSq, heavySq);

    // Squeal surface gate: rubber on hard ground only; a full slide on soft ground never squeals.
    std::printf("    squeal drive of a full slide:");
    const SurfaceId surfaces[] = {SurfaceId::Concrete, SurfaceId::PackedDirt, SurfaceId::Rock,
                                  SurfaceId::LowGrip,  SurfaceId::Mud,        SurfaceId::Grass};
    for (SurfaceId s : surfaces) {
        const float d = wheelSquealDrive(squealing(wheel(s, 8, 8, 1.0f)), ref);
        std::printf(" %d:%.2f", int(s), d);
        const bool hard = s == SurfaceId::Concrete || s == SurfaceId::Rock || s == SurfaceId::LowGrip;
        CHECK(hard ? d > 0.3f : d == 0.0f, "surface %d squeal drive %.2f", int(s), d);
    }
    std::printf("\n");
    WheelAudio dirty = wheel(SurfaceId::Concrete, 8, 8, 1.0f);
    dirty.soil = 1.0f;
    CHECK(wheelSquealDrive(squealing(dirty), ref) == 0.0f, "a tread full of soil squeals");

    // rubberSlide dead zones (shared with the rubber marks), from measured concrete maneuvers.
    auto patch = [](float speed, float rolling, float lateral) {
        WheelContactSample c;
        c.contact = true;
        c.velocity = JPH::Vec3(lateral, 0.0f, speed);
        c.rollingSpeed = rolling;
        c.longitudinalSlipSpeed = rolling - speed;
        c.lateralSlipSpeed = lateral;
        return c;
    };
    const float traction = rubberSlide(patch(14.0f, 15.7f, 0.0f));    // 2nd-gear pull, 1.7 m/s
    const float launch = rubberSlide(patch(4.0f, 6.4f, 0.0f));        // 2.4 m/s spike at ratio 0.38
    const float fullLock = rubberSlide(patch(8.4f, 8.4f, 3.6f));      // 30 km/h full-lock scrub, 23 deg
    const float lowRange = rubberSlide(patch(5.0f, 8.6f, 0.0f));      // 3.6 m/s wheelspin at ratio 0.42
    const float lockup = rubberSlide(patch(12.0f, 0.0f, 0.0f));       // locked wheel
    const float drift = rubberSlide(patch(8.0f, 8.0f, 7.0f));         // sideways at 41 deg
    std::printf("    rubberSlide: traction %.2f launch %.2f full-lock %.2f low-range spin %.2f lock-up %.2f drift %.2f\n",
                traction, launch, fullLock, lowRange, lockup, drift);
    CHECK(traction == 0.0f && fullLock < 0.05f && launch < 0.25f, "normal driving slides: %.2f %.2f %.2f", traction,
          fullLock, launch);
    CHECK(lowRange > 0.3f && lockup > 0.99f && drift > 0.5f, "real slides: %.2f %.2f %.2f", lowRange, lockup, drift);

    // Squeal slip (stricter than the marks), from SUV maneuvers measured with `trailforge_probe squeal` on concrete:
    // patch ground speed, combined sliding speed, slip ratio.
    const float sLaunch = squealSlip(wheel(SurfaceId::Concrete, 4.0f, 5.0f, 0.50f), ref);     // 2H full throttle
    const float sLaunchSpike = squealSlip(wheel(SurfaceId::Concrete, 1.2f, 3.1f, 0.74f), ref); // first 0.2 s of it
    const float sTurn = squealSlip(sideways(SurfaceId::Concrete, 8.4f, 4.1f, 0.01f, 0.45f), ref); // 30 km/h full lock
    const float sAbs = squealSlip(wheel(SurfaceId::Concrete, 10.0f, 3.75f, 0.30f), ref);      // ABS stop
    const float sStopEnd = squealSlip(wheel(SurfaceId::Concrete, 1.5f, 1.5f, 1.0f), ref);     // locked, nearly stopped
    const float sLock = squealSlip(wheel(SurfaceId::Concrete, 12.6f, 12.6f, 1.0f), ref);      // locked at 45 km/h
    const float sDrift = squealSlip(sideways(SurfaceId::Concrete, 11.0f, 11.0f, 0.7f, 0.7f), ref); // handbrake turn
    const float sPlow = squealSlip(sideways(SurfaceId::Concrete, 12.0f, 6.7f, 0.09f, 0.53f), ref);  // full-lock plow
    const float sUnloaded = squealSlip(wheel(SurfaceId::Concrete, 12.6f, 12.6f, 1.0f, 500.0f), ref);
    std::printf("    squealSlip: launch %.3f spike %.3f full-lock turn %.3f ABS %.3f stop end %.3f | lock-up %.2f "
                "drift %.2f plow %.2f unloaded %.2f | ratio 0.6/0.7/0.8/0.9 at 8 m/s:",
                sLaunch, sLaunchSpike, sTurn, sAbs, sStopEnd, sLock, sDrift, sPlow, sUnloaded);
    float prev = 0.0f;
    bool rising = true;
    for (float r : {0.6f, 0.7f, 0.8f, 0.9f}) {
        const float s = squealSlip(wheel(SurfaceId::Concrete, 8.0f, 8.0f, r), ref);
        std::printf(" %.2f", s);
        rising = rising && s > prev;
        prev = s;
    }
    std::printf("\n");
    // Below 0.05: far under the gate of the per-tick squeal envelope (AudioFeedSettings::squealOn).
    CHECK(sLaunch < 0.05f && sTurn < 0.05f && sAbs < 0.05f && sStopEnd < 0.05f && sLaunchSpike < 0.05f,
          "everyday slip squeals: %.3f %.3f %.3f %.3f %.3f", sLaunch, sTurn, sAbs, sStopEnd, sLaunchSpike);
    CHECK(sLock > 0.95f && sDrift > 0.9f, "slides: lock-up %.2f drift %.2f", sLock, sDrift);
    CHECK(sPlow > 0.1f && sPlow < 0.7f, "plow %.2f", sPlow);
    CHECK(sUnloaded == 0.0f, "unloaded wheel %.2f", sUnloaded);
    CHECK(rising && prev < sLock, "squeal not proportional to slip ratio");

    // Tread soil: picked up within ~1 m of mud, shed over ~10 m of clean ground.
    TreadSoil soil;
    WheelContactSample onMud = patch(5.0f, 5.0f, 0.0f), onConcrete = onMud;
    onMud.surface = SurfaceId::Mud;
    onConcrete.surface = SurfaceId::Concrete;
    for (int i = 0; i < 60; ++i) soil.update(onMud, 1.0f / 120.0f); // 2.5 m
    const float muddy = soil.amount;
    for (int i = 0; i < 720; ++i) soil.update(onConcrete, 1.0f / 120.0f); // 30 m
    std::printf("    tread soil: %.2f after 2.5 m of mud, %.3f after 30 m of concrete\n", muddy, soil.amount);
    CHECK(muddy > 0.85f && soil.amount < 0.06f, "tread soil %.2f / %.3f", muddy, soil.amount);

    // Ambience and panning.
    for (float d = 0.0f; d <= 1.0f; d += 0.1f) {
        const auto w = dayNightWeights(d);
        CHECK(std::abs(w[0] * w[0] + w[1] * w[1] - 1.0f) < 1e-5f, "day/night not equal power at %.1f", d);
    }
    CHECK(dayNightWeights(1.0f)[0] > 0.999f && dayNightWeights(0.0f)[1] > 0.999f, "day/night ends");
    const AudioVec3 lp{0, 0, 0}, right{1, 0, 0}, fwd{0, 0, 1};
    const StereoGain onRight = spatialize({10, 0, 0}, lp, right, fwd);
    const StereoGain ahead5 = spatialize({0, 0, 5}, lp, right, fwd), ahead20 = spatialize({0, 0, 20}, lp, right, fwd);
    std::printf("    pan right L %.3f R %.3f; ahead 5 m %.3f, 20 m %.3f\n", onRight.l, onRight.r, ahead5.l, ahead20.l);
    CHECK(onRight.r > 2.0f * onRight.l, "right source not panned right");
    CHECK(std::abs(ahead5.l - ahead5.r) < 1e-5f && std::abs(ahead20.l / ahead5.l - 0.25f) < 0.01f, "inverse distance");
    CHECK(windGain(2.0f) == 0.0f && windGain(30.0f) == 1.0f && windGain(15.0f) < windGain(20.0f), "wind gain");
}

void manifest() {
    AudioManifest m;
    std::string error;
    const bool ok = loadAudioManifest(kManifest, m, &error);
    CHECK(ok, "%s", error.c_str());
    if (!ok) return;
    std::printf("    %s: sample rate %d, %d missing, %zu unknown\n", kManifest.c_str(), m.sampleRate, m.missingCount(),
                m.unknownIds.size());
    CHECK(m.sampleRate == 48000, "sample rate %d", m.sampleRate);
    CHECK(m.missingCount() == 0, "%d ids missing", m.missingCount());
    CHECK(m.unknownIds.empty(), "unknown ids, first '%s'", m.unknownIds.empty() ? "" : m.unknownIds[0].c_str());
    const std::filesystem::path dir = std::filesystem::path(kManifest).parent_path();
    for (std::size_t i = 0; i < kSoundCount; ++i) {
        const SoundInfo& s = m.sounds[i];
        CHECK(std::filesystem::exists(dir / s.file), "file missing: %s", s.file.c_str());
        if (i < 12) {
            const bool exhaust = i < 6, full = (i / 3) % 2 == 1;
            CHECK(s.loop && s.rpmMeasured > 500.0f, "%s: loop %d rpm %.0f", s.id.c_str(), s.loop, s.rpmMeasured);
            CHECK(s.mic == (exhaust ? EngineMic::Exhaust : EngineMic::Bay), "%s: mic", s.id.c_str());
            CHECK(s.load == (full ? EngineLoad::Full : EngineLoad::Off), "%s: load", s.id.c_str());
            std::printf("    %-34s %5.0f rpm %-4s %-7s level %+5.1f dB\n", s.id.c_str(), s.rpmMeasured,
                        full ? "full" : "off", exhaust ? "exhaust" : "bay", s.sourceLevelDb);
        }
    }
    // Bands ascend in rpm within each load/mic set.
    for (int set = 0; set < 4; ++set)
        CHECK(m.sounds[std::size_t(set * 3)].rpmMeasured < m.sounds[std::size_t(set * 3 + 1)].rpmMeasured &&
                  m.sounds[std::size_t(set * 3 + 1)].rpmMeasured < m.sounds[std::size_t(set * 3 + 2)].rpmMeasured,
              "rpm order in set %d", set);
    int cc0 = 0, ccby = 0;
    for (const SoundInfo& s : m.sounds) (s.license == "CC0 1.0" ? cc0 : ccby) += 1;
    std::printf("    licenses: %d CC0 1.0, %d other\n", cc0, ccby);
    CHECK(m[SoundId::AmbienceDayLoop].channels == 2 && m[SoundId::AmbienceDayLoop].loop, "day ambience");
}

// Synthetic bank: every sound is a sine burst (a loop for loops) so the mixer runs without the WAV files.
SoundBank syntheticBank() {
    SoundBank b;
    std::string error;
    loadAudioManifest(kManifest, b.manifest, &error);
    for (std::size_t i = 0; i < kSoundCount; ++i) {
        SoundData& d = b.sounds[i];
        const SoundInfo& info = b.manifest.sounds[i];
        d.channels = info.channels > 0 ? info.channels : 1;
        d.loop = info.loop;
        d.frames = info.loop ? 48000 : 12000;
        d.pcm.resize(d.frames * std::size_t(d.channels));
        const double f = 100.0 + 20.0 * double(i);
        for (std::size_t k = 0; k < d.frames; ++k)
            for (int c = 0; c < d.channels; ++c)
                d.pcm[k * std::size_t(d.channels) + std::size_t(c)] =
                    float(0.5 * std::sin(6.283185307 * f * double(k) / 48000.0));
    }
    return b;
}

// The shipped SUV (assets/vehicles/trailforge_suv): 2H part-time transfer case, ABS off by default.
VehicleConfig suvConfig() {
    VehicleDefinition def;
    std::vector<std::string> errors;
    const bool ok = loadVehicleDefinition(std::string(TRAILFORGE_SOURCE_DIR) + "/assets/vehicles/trailforge_suv/vehicle.json",
                                          def, errors);
    CHECK(ok, "SUV definition: %s", errors.empty() ? "?" : errors.front().c_str());
    return def.config;
}

VehicleConfig evConfig() {
    VehicleDefinition def;
    std::vector<std::string> errors;
    const bool ok = loadVehicleDefinition(std::string(TRAILFORGE_SOURCE_DIR) + "/assets/vehicles/trailforge_ev/vehicle.json",
                                          def, errors);
    CHECK(ok, "EV definition: %s", errors.empty() ? "?" : errors.front().c_str());
    return def.config;
}

struct Driver {
    explicit Driver(TrackLayout layout = TrackLayout::SurfaceLanes, double fixedStep = 1.0 / 120.0,
                    const VehicleConfig* vehicle = nullptr)
        : sim([&] {
              SimulationSettings s;
              s.layout = layout;
              s.fixedStep = fixedStep;
              if (vehicle) s.vehicle = *vehicle;
              return s;
          }()),
          framesPerTick(int(std::lround(48000.0 * fixedStep))) {}
    Simulation sim;
    int framesPerTick;
    SoundBank bank = syntheticBank();
    AudioMixer mixer{bank};
    VehicleAudioFeed feed;
    std::vector<float> buf = std::vector<float>(2 * std::size_t(framesPerTick));
    bool paused = false;
    float maxAbs = 0.0f;
    bool finite = true;
    double outPower = 0.0;
    long long outFrames = 0;
    int shifts = 0, starts = 0;
    float motorSetting = 1.0f;     // AudioSettings::motor
    std::vector<float>* tap = nullptr; // receives the mono output when set

    void tick(const DriverIntent& in) {
        sim.setIntent(in);
        if (!paused) {
            sim.tick();
            feed.onTick(sim);
        }
        AudioFrame f = feed.frame();
        f.listenerPos = f.vehiclePos - f.vehicleForward * 8.0f + AudioVec3{0, 2.6f, 0};
        f.listenerForward = f.vehicleForward;
        f.listenerRight = f.vehicleLeft * -1.0f;
        f.paused = paused;
        f.settings.motor = motorSetting;
        mixer.setFrame(f);
        for (const AudioEvent& e : feed.events()) {
            shifts += e.type == AudioEventType::GearShift;
            starts += e.type == AudioEventType::EngineStart;
            mixer.trigger(e);
        }
        feed.clearEvents();
        mixer.render(buf.data(), framesPerTick); // one physics tick at 48 kHz
        if (tap)
            for (int k = 0; k < framesPerTick; ++k)
                tap->push_back(0.5f * (buf[std::size_t(2 * k)] + buf[std::size_t(2 * k + 1)]));
        for (float x : buf) {
            finite = finite && std::isfinite(x);
            maxAbs = std::max(maxAbs, std::abs(x));
            outPower += double(x) * double(x);
        }
        outFrames += framesPerTick;
    }
    int ticks(double seconds) const { return int(std::lround(seconds / sim.fixedStep())); }
    void run(double seconds, const DriverIntent& in = {}) {
        for (int i = 0; i < ticks(seconds); ++i) tick(in);
    }
    void cruise(double seconds, float kmh, float steer = 0.0f) {
        for (int i = 0; i < ticks(seconds); ++i) {
            const float v = sim.vehicle().telemetry().speed * 3.6f;
            DriverIntent in;
            in.throttle = std::fmin(0.6f, std::fmax(0.0f, 0.25f + 0.05f * (kmh - v)));
            in.steer = steer;
            tick(in);
        }
    }
    // Runs and returns the peak of the mixed skid gain (and of the mud layer).
    float peakSkid(double seconds, const DriverIntent& in, float* peakMud = nullptr) {
        float skid = 0.0f;
        for (int i = 0; i < ticks(seconds); ++i) {
            tick(in);
            skid = std::max(skid, mixer.stats().skid);
            if (peakMud) *peakMud = std::max(*peakMud, mixer.stats().mud);
        }
        return skid;
    }
    float peakSkidCruise(double seconds, float kmh, float steer) {
        float skid = 0.0f;
        for (int i = 0; i < ticks(seconds); ++i) {
            cruise(sim.fixedStep(), kmh, steer);
            skid = std::max(skid, mixer.stats().skid);
        }
        return skid;
    }
    double rms() {
        const double r = outFrames ? std::sqrt(outPower / double(2 * outFrames)) : 0.0;
        outPower = 0.0;
        outFrames = 0;
        return r;
    }
};

void feedAndMixer() {
    Driver d;
    d.run(1.0);
    CHECK(d.starts == 1, "EngineStart events at spawn: %d", d.starts);

    // Full-throttle launch on the concrete lane: rpm rises, gears are engaged.
    float maxSrc = 0.0f;
    for (int i = 0; i < 6 * 120; ++i) {
        d.tick(DriverIntent{.throttle = 1.0f});
        maxSrc = std::max(maxSrc, d.mixer.stats().sourceRpm);
    }
    std::printf("    launch: %d gear events, max source rpm %.0f, %.1f km/h\n", d.shifts, maxSrc,
                d.sim.vehicle().telemetry().speed * 3.6f);
    CHECK(d.shifts >= 2, "gear events while accelerating: %d", d.shifts);
    CHECK(maxSrc > 2500.0f, "source rpm did not rise: %.0f", maxSrc);
    CHECK(d.mixer.stats().load > 0.9f, "load at full throttle %.2f", d.mixer.stats().load);

    // Cruise vs. full brake on concrete: the skid layer rises only while the wheels slide.
    d.cruise(3.0, 30.0f);
    float cruiseSkid = 0.0f;
    for (int i = 0; i < 240; ++i) {
        d.cruise(1.0 / 120.0, 30.0f);
        cruiseSkid = std::max(cruiseSkid, d.mixer.stats().skid);
    }
    float brakeSkid = 0.0f;
    for (int i = 0; i < 300; ++i) {
        d.tick(DriverIntent{.brake = 1.0f});
        brakeSkid = std::max(brakeSkid, d.mixer.stats().skid);
    }
    std::printf("    concrete: skid while cruising %.3f, while braking %.3f\n", cruiseSkid, brakeSkid);
    CHECK(cruiseSkid < 0.1f, "skid while cruising %.3f", cruiseSkid);
    CHECK(brakeSkid > 0.3f, "skid while braking %.3f", brakeSkid);

    // Mud lane: the mud layer, not the skid or grass layer.
    d.sim.requestTeleport(2);
    d.run(1.0);
    d.cruise(4.0, 20.0f);
    const AudioStats mud = d.mixer.stats();
    std::printf("    mud lane 20 km/h: mud %.2f skid %.2f grass %.2f dirt %.2f, %d voices\n", mud.mud, mud.skid,
                mud.grass, mud.dirt, mud.activeVoices);
    CHECK(mud.mud > 0.2f && mud.skid < 0.05f && mud.grass < 0.05f, "mud lane layers");
    d.rms();
    d.run(0.5);
    const double playing = d.rms();

    // Pause: gameplay sounds fade out and freeze; only the ambience (unchanged level) keeps playing.
    d.paused = true;
    d.run(1.0);
    const double pausedRms = d.rms();
    std::printf("    rms playing %.4f, paused %.4f, gameplay gain %.3f; peak %.3f, finite %d, clipped %lld\n", playing,
                pausedRms, d.mixer.stats().gameplayGain, d.maxAbs, d.finite, d.mixer.stats().clippedSamples);
    CHECK(d.mixer.stats().gameplayGain == 0.0f, "gameplay gain while paused %.3f", d.mixer.stats().gameplayGain);
    CHECK(pausedRms < playing, "pause did not lower the output: %.4f vs %.4f", pausedRms, playing);
    CHECK(d.finite, "NaN/Inf in the output");
    CHECK(d.maxAbs <= 0.8913f, "output peak %.4f above -1 dBFS", d.maxAbs);
    d.paused = false;
    d.run(0.5);
    CHECK(d.mixer.stats().gameplayGain > 0.9f, "gameplay did not resume: %.3f", d.mixer.stats().gameplayGain);
}

// Tire squeal on the real simulation: silent for normal launches and turns on concrete, audible for lock-up,
// handbrake and power slides, never on mud; the same at 60, 120 and 240 Hz physics.
void skidGating() {
    struct Result {
        float launch, launchLow, turnTight, turnFast, handbrake, brake, powerSlide;
        int chirps;
    };
    auto pad = [](double step) {
        Driver d(TrackLayout::Flat, step);
        Result r{};
        auto spawn = [&] {
            d.sim.requestReset();
            d.run(1.0);
        };
        spawn();
        r.launch = d.peakSkid(5.0, {.throttle = 1.0f});
        spawn();
        d.sim.requestToggleLowRange();
        d.run(0.1);
        r.launchLow = d.peakSkid(4.0, {.throttle = 1.0f});
        d.sim.requestToggleLowRange();
        spawn();
        d.cruise(6.0, 30.0f);
        r.turnTight = d.peakSkidCruise(4.0, 30.0f, 1.0f);
        spawn();
        d.cruise(8.0, 50.0f);
        r.turnFast = d.peakSkidCruise(4.0, 50.0f, 0.5f);
        const int chirps0 = d.mixer.stats().skidChirps;
        spawn();
        d.cruise(7.0, 40.0f);
        r.handbrake = d.peakSkid(2.5, {.steer = 0.6f, .handBrake = 1.0f});
        spawn();
        d.cruise(8.0, 50.0f);
        r.brake = d.peakSkid(3.0, {.brake = 1.0f});
        spawn();
        r.powerSlide = d.peakSkid(5.0, {.throttle = 1.0f, .steer = 1.0f});
        r.chirps = d.mixer.stats().skidChirps - chirps0;
        return r;
    };
    const double steps[] = {1.0 / 60.0, 1.0 / 120.0, 1.0 / 240.0};
    Result rs[3];
    for (int i = 0; i < 3; ++i) {
        const Result& r = rs[i] = pad(steps[i]);
        std::printf("    %3.0f Hz concrete skid: launch %.3f, low-range launch %.3f, turn 30 km/h full lock %.3f, "
                    "turn 50 km/h half lock %.3f | handbrake %.2f, brake %.2f, power slide %.2f, %d chirps\n",
                    1.0 / steps[i], r.launch, r.launchLow, r.turnTight, r.turnFast, r.handbrake, r.brake,
                    r.powerSlide, r.chirps);
        CHECK(r.launch < 0.05f && r.launchLow < 0.05f, "%.0f Hz: launch squeals %.3f / %.3f", 1.0 / steps[i],
              r.launch, r.launchLow);
        CHECK(r.turnTight < 0.05f && r.turnFast < 0.05f, "%.0f Hz: turn squeals %.3f / %.3f", 1.0 / steps[i],
              r.turnTight, r.turnFast);
        CHECK(r.handbrake > 0.3f && r.brake > 0.3f && r.powerSlide > 0.15f, "%.0f Hz: slides %.2f %.2f %.2f",
              1.0 / steps[i], r.handbrake, r.brake, r.powerSlide);
        CHECK(r.chirps >= 2, "%.0f Hz: %d onset chirps for 3 slides", 1.0 / steps[i], r.chirps);
    }
    // The squeal envelope is step independent; the slides themselves are not entirely: at 60 Hz Jolt locks a braked
    // wheel once the brake torque exceeds |w| x inertia / dt, so the full brake locks all four wheels there.
    CHECK(std::abs(rs[0].handbrake - rs[1].handbrake) < 0.1f && std::abs(rs[2].handbrake - rs[1].handbrake) < 0.1f,
          "step dependence: handbrake %.2f / %.2f / %.2f", rs[0].handbrake, rs[1].handbrake, rs[2].handbrake);
    CHECK(std::abs(rs[2].brake - rs[1].brake) < 0.1f && std::abs(rs[2].powerSlide - rs[1].powerSlide) < 0.1f,
          "step dependence 120/240 Hz: brake %.2f / %.2f, power slide %.2f / %.2f", rs[1].brake, rs[2].brake,
          rs[1].powerSlide, rs[2].powerSlide);

    // The shipped SUV (2H, ABS off) on the flat pad: a full-throttle launch (the keyboard start: ~5 m/s rear wheelspin
    // at slip ratio ~0.5), light braking and a 0.25 s brake tap (a brief partial rear lock) stay silent; a 0.5 s tap at
    // full pedal locks the rears for ~0.3 s and gives a short squeal that ends within 0.2 s of the slide; lock-up and
    // handbrake slides squeal.
    {
        const VehicleConfig suv = suvConfig();
        Driver d(TrackLayout::Flat, 1.0 / 120.0, &suv);
        struct Run {
            float peak = 0.0f;
            double lastSlide = -1.0, lastSkid = -1.0;
            double tail() const { return lastSkid < 0.0 ? 0.0 : lastSkid - lastSlide; }
        };
        auto drive = [&](double seconds, double hold, const DriverIntent& in) {
            Run r;
            for (int i = 0; i < d.ticks(seconds); ++i) {
                const double t = i * d.sim.fixedStep();
                d.tick(t < hold ? in : DriverIntent{});
                const float skid = d.mixer.stats().skid;
                r.peak = std::max(r.peak, skid);
                if (skid > 0.01f) r.lastSkid = t;
                for (const WheelContactSample& c : d.sim.vehicle().contacts())
                    if (c.contact && c.slipSpeed() > 2.0f) r.lastSlide = t;
            }
            return r;
        };
        auto spawn = [&](float kmh) {
            d.sim.requestReset();
            d.run(1.0);
            if (kmh > 0.0f) d.cruise(8.0, kmh);
        };
        spawn(0);
        const Run launch = drive(5.0, 5.0, {.throttle = 1.0f});
        spawn(0);
        const Run launchHalf = drive(4.0, 4.0, {.throttle = 0.5f});
        spawn(50);
        const Run light = drive(5.0, 5.0, {.brake = 0.3f});
        spawn(50);
        const Run tapShort = drive(2.5, 0.25, {.brake = 1.0f});
        spawn(50);
        const Run tap = drive(2.5, 0.5, {.brake = 1.0f});
        spawn(50);
        const Run full = drive(5.0, 5.0, {.brake = 1.0f});
        spawn(40);
        const Run handbrake = drive(3.0, 3.0, {.steer = 0.6f, .handBrake = 1.0f});
        std::printf("    SUV pad skid peak: launch full %.3f (slid until %.2f s), half %.3f, light brake %.3f, 0.25 s brake "
                    "tap %.3f | 0.5 s brake tap %.2f (slid until %.2f s, tail %.2f s), full brake %.2f (tail %.2f s), "
                    "handbrake turn %.2f (tail %.2f s)\n",
                    launch.peak, launch.lastSlide, launchHalf.peak, light.peak, tapShort.peak, tap.peak, tap.lastSlide,
                    tap.tail(), full.peak, full.tail(), handbrake.peak, handbrake.tail());
        CHECK(launch.lastSlide > 0.5 && launch.peak < 0.05f && launchHalf.peak < 0.05f && light.peak < 0.05f &&
                  tapShort.peak < 0.05f,
              "SUV everyday squeal: launch %.3f (slide until %.2f s), half %.3f, light brake %.3f, short tap %.3f",
              launch.peak, launch.lastSlide, launchHalf.peak, light.peak, tapShort.peak);
        CHECK(tap.peak > 0.1f && tap.lastSlide < 0.9 && tap.tail() < 0.25, "SUV brake tap: peak %.2f, slide until %.2f s, "
              "tail %.2f s", tap.peak, tap.lastSlide, tap.tail());
        CHECK(full.peak > 0.3f && handbrake.peak > 0.3f, "SUV slides: brake %.2f handbrake %.2f", full.peak,
              handbrake.peak);
        CHECK(full.tail() < 0.2 && handbrake.tail() < 0.2, "SUV squeal tails: %.2f / %.2f s", full.tail(),
              handbrake.tail());
    }

    // Mud lane: wheelspin, lock-up and a handbrake slide drive the mud layer and never the squeal.
    Driver d;
    d.sim.requestTeleport(2);
    d.run(1.0);
    d.sim.requestToggleLowRange();
    d.run(0.1);
    float mud = 0.0f;
    float skid = d.peakSkid(1.5, {.throttle = 1.0f, .steer = 1.0f}, &mud);
    skid = std::max(skid, d.peakSkid(2.0, {.brake = 1.0f}, &mud));
    d.sim.requestToggleLowRange();
    d.sim.requestTeleport(2);
    d.run(1.0);
    d.cruise(5.0, 35.0f);
    skid = std::max(skid, d.peakSkid(2.0, {.brake = 1.0f}, &mud));
    d.cruise(5.0, 35.0f);
    skid = std::max(skid, d.peakSkid(2.0, {.steer = 0.6f, .handBrake = 1.0f}, &mud));
    std::printf("    mud lane: peak skid %.3f, peak mud %.2f\n", skid, mud);
    CHECK(skid == 0.0f && mud > 0.3f, "mud: skid %.3f mud %.2f", skid, mud);
}

// Mixer squeal voices with the real lengths of the squeal sounds (manifest durations, every other sound empty): a
// wheel squeals for 0.3 s and stops. The onset short squeal (1.7 s file) must fade out with the slide instead of
// playing to its end; the output follows the squeal level.
void squealRelease() {
    SoundBank bank;
    std::string error;
    loadAudioManifest(kManifest, bank.manifest, &error);
    for (SoundId id : {SoundId::TireSkidLoop, SoundId::TireSkidShort}) {
        SoundData& d = bank.sounds[std::size_t(id)];
        d.loop = bank.manifest[id].loop;
        d.frames = std::size_t(bank.manifest[id].durationS * 48000.0);
        d.pcm.resize(d.frames);
        for (std::size_t k = 0; k < d.frames; ++k)
            d.pcm[k] = float(0.25 * std::sin(6.283185307 * (id == SoundId::TireSkidLoop ? 1700.0 : 1900.0) *
                                             double(k) / 48000.0));
    }
    CHECK(bank[SoundId::TireSkidShort].frames > 48000, "short squeal length %zu frames",
          bank[SoundId::TireSkidShort].frames);

    // Returns the time (s) from the end of the squeal until the output stays below -60 dBFS, and the peak (dBFS).
    auto run = [&](float squeal, double seconds, float* peakDb, int* chirps) {
        AudioMixer mixer(bank);
        AudioFrame f;
        f.vehicleValid = true;
        f.settings.whine = 0.0f;
        f.listenerPos = {0.0f, 1.5f, -6.0f};
        std::vector<float> buf(2 * 400);
        double lastLoud = -1.0;
        *peakDb = -120.0f;
        for (int i = 0; i < 3 * 120; ++i) {
            const double t = i / 120.0;
            WheelAudio& w = f.wheels[2];
            w.contact = true;
            w.position = {0.8f, 0.0f, -1.4f};
            w.squeal = t >= 0.5 && t < 0.5 + seconds ? squeal : 0.0f;
            mixer.setFrame(f);
            mixer.render(buf.data(), 400);
            double p = 0.0;
            for (float x : buf) p += double(x) * double(x);
            const float db = gainToDb(float(std::sqrt(p / double(buf.size()))));
            *peakDb = std::max(*peakDb, db);
            if (db > -60.0f) lastLoud = t;
        }
        *chirps = mixer.stats().skidChirps;
        return lastLoud < 0.0 ? 0.0 : lastLoud - (0.5 + seconds);
    };
    float fullDb = 0.0f, lowDb = 0.0f;
    int fullChirps = 0, lowChirps = 0;
    const double fullTail = run(1.0f, 0.3, &fullDb, &fullChirps);
    const double lowTail = run(0.25f, 0.3, &lowDb, &lowChirps);
    std::printf("    0.3 s squeal at 1.0: peak %.1f dBFS, %d short squeal(s), output below -60 dBFS %.3f s after the "
                "slide | at 0.25: peak %.1f dBFS, %d short squeal(s), tail %.3f s\n",
                fullDb, fullChirps, fullTail, lowDb, lowChirps, lowTail);
    CHECK(fullChirps == 1 && lowChirps == 0, "short squeals %d / %d", fullChirps, lowChirps);
    CHECK(fullTail < 0.2 && lowTail < 0.2, "squeal tail %.3f / %.3f s", fullTail, lowTail);
    CHECK(fullDb - lowDb > 9.0f, "squeal level not proportional: %.1f vs %.1f dBFS", fullDb, lowDb);
}

// 16-bit PCM mono WAV (the shipped tire recordings); returns false for anything else.
bool readPcm16Mono(const std::string& path, SoundData& d) {
    std::FILE* fp = std::fopen(path.c_str(), "rb");
    if (fp == nullptr) return false;
    std::vector<unsigned char> b;
    unsigned char buf[65536];
    for (std::size_t n; (n = std::fread(buf, 1, sizeof(buf), fp)) > 0;) b.insert(b.end(), buf, buf + n);
    std::fclose(fp);
    auto u32 = [&](std::size_t o) { return std::uint32_t(b[o]) | std::uint32_t(b[o + 1]) << 8 | std::uint32_t(b[o + 2]) << 16 | std::uint32_t(b[o + 3]) << 24; };
    auto u16 = [&](std::size_t o) { return unsigned(b[o]) | unsigned(b[o + 1]) << 8; };
    if (b.size() < 12 || std::memcmp(b.data(), "RIFF", 4) != 0 || std::memcmp(b.data() + 8, "WAVE", 4) != 0) return false;
    bool fmtOk = false;
    for (std::size_t o = 12; o + 8 <= b.size();) {
        const std::uint32_t size = u32(o + 4);
        if (std::memcmp(b.data() + o, "fmt ", 4) == 0)
            fmtOk = u16(o + 8) == 1 && u16(o + 10) == 1 && u32(o + 12) == 48000 && u16(o + 22) == 16;
        if (std::memcmp(b.data() + o, "data", 4) == 0 && fmtOk) {
            d.channels = 1;
            d.frames = std::min<std::size_t>(size, b.size() - o - 8) / 2;
            d.pcm.resize(d.frames);
            for (std::size_t k = 0; k < d.frames; ++k)
                d.pcm[k] = float(std::int16_t(u16(o + 8 + 2 * k))) / 32768.0f;
            return true;
        }
        o += 8 + size + (size & 1u);
    }
    return false;
}

// Power spectrum of a mono signal: Hann-windowed 4096-point FFTs with 50 % overlap, averaged.
std::vector<double> powerSpectrum(const std::vector<float>& x) {
    constexpr std::size_t N = 4096;
    std::vector<double> acc(N / 2, 0.0);
    std::vector<double> re(N), im(N);
    int frames = 0;
    for (std::size_t start = 0; start + N <= x.size(); start += N / 2, ++frames) {
        for (std::size_t i = 0; i < N; ++i) {
            re[i] = double(x[start + i]) * (0.5 - 0.5 * std::cos(6.283185307179586 * double(i) / double(N)));
            im[i] = 0.0;
        }
        for (std::size_t i = 1, j = 0; i < N; ++i) {
            std::size_t bit = N >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) std::swap(re[i], re[j]), std::swap(im[i], im[j]);
        }
        for (std::size_t len = 2; len <= N; len <<= 1) {
            const double a = -6.283185307179586 / double(len);
            for (std::size_t i = 0; i < N; i += len)
                for (std::size_t k = 0; k < len / 2; ++k) {
                    const double wr = std::cos(a * double(k)), wi = std::sin(a * double(k));
                    const std::size_t p = i + k, q = p + len / 2;
                    const double tr = re[q] * wr - im[q] * wi, ti = re[q] * wi + im[q] * wr;
                    re[q] = re[p] - tr, im[q] = im[p] - ti;
                    re[p] += tr, im[p] += ti;
                }
        }
        for (std::size_t i = 0; i < N / 2; ++i) acc[i] += re[i] * re[i] + im[i] * im[i];
    }
    for (double& v : acc) v /= std::max(frames, 1);
    return acc;
}

// Tire sound profiles on the real squeal recordings: one rear wheel slides at a steady 6 m/s at full squeal for 2 s.
// The off-road profile (trailforge_suv) must be darker (lower spectral centroid, more energy below 1 kHz, less above
// 3 kHz) at a comparable level; its scrub must follow the squeal and stop with it; "sporty" has no scrub.
void tireProfiles() {
    SoundBank bank;
    std::string error;
    loadAudioManifest(kManifest, bank.manifest, &error);
    for (SoundId id : {SoundId::TireSkidLoop, SoundId::TireSkidShort}) {
        SoundData& d = bank.sounds[std::size_t(id)];
        const std::string path = std::string(TRAILFORGE_SOURCE_DIR) + "/assets/audio/" + bank.manifest[id].file;
        CHECK(readPcm16Mono(path, d), "cannot read %s", path.c_str());
        d.loop = bank.manifest[id].loop;
    }
    struct Result {
        double rmsDb = 0, centroid = 0, below1k = 0, above3k = 0, tail = 0;
        float scrub = 0;
    };
    auto run = [&](const TireSoundProfile& profile, float squeal) {
        AudioMixer mixer(bank, 48000, 7u);
        AudioFrame f;
        f.vehicleValid = true;
        f.settings.whine = 0.0f;
        f.tireSound = profile;
        f.listenerPos = {0.0f, 1.5f, -6.0f};
        std::vector<float> buf(2 * 400), mono;
        Result r;
        double lastLoud = -1.0;
        for (int i = 0; i < 3 * 120; ++i) {
            const double t = i / 120.0;
            WheelAudio& w = f.wheels[2];
            w.contact = true;
            w.position = {0.8f, 0.0f, -1.4f};
            const bool on = t >= 0.2 && t < 2.2;
            w.squeal = on ? squeal : 0.0f;
            w.slipSpeed = on ? 6.0f : 0.0f;
            w.load = f.referenceLoad;
            mixer.setFrame(f);
            mixer.render(buf.data(), 400);
            double p = 0.0;
            for (float x : buf) p += double(x) * double(x);
            if (gainToDb(float(std::sqrt(p / double(buf.size())))) > -60.0f) lastLoud = t;
            if (t >= 0.6 && t < 2.2) {
                for (int k = 0; k < 400; ++k) mono.push_back(0.5f * (buf[std::size_t(2 * k)] + buf[std::size_t(2 * k + 1)]));
                r.scrub = std::max(r.scrub, mixer.stats().scrub);
            }
        }
        r.tail = lastLoud < 0.0 ? 0.0 : lastLoud - 2.2;
        double e = 0.0;
        for (float x : mono) e += double(x) * double(x);
        r.rmsDb = 10.0 * std::log10(std::max(e / double(std::max<std::size_t>(mono.size(), 1)), 1e-12));
        const std::vector<double> ps = powerSpectrum(mono);
        double total = 0.0, moment = 0.0;
        for (std::size_t k = 1; k < ps.size(); ++k) {
            const double hz = double(k) * 48000.0 / 4096.0;
            total += ps[k];
            moment += ps[k] * hz;
            if (hz < 1000.0) r.below1k += ps[k];
            if (hz >= 3000.0) r.above3k += ps[k];
        }
        r.centroid = moment / std::max(total, 1e-30);
        r.below1k /= std::max(total, 1e-30);
        r.above3k /= std::max(total, 1e-30);
        return r;
    };
    const TireSoundProfile sporty = tireSoundPreset("sporty");
    const TireSoundProfile suv = tireSoundPreset("trailforge_suv");
    TireSoundProfile suvNoScrub = suv;
    suvNoScrub.scrubGain = 0.0f;
    const Result a = run(sporty, 1.0f), b = run(suv, 1.0f), c = run(suvNoScrub, 1.0f), d = run(suv, 0.3f);
    auto print = [](const char* name, const Result& r) {
        std::printf("    %-26s rms %6.1f dBFS  centroid %5.0f Hz  <1 kHz %4.1f %%  >3 kHz %4.1f %%  scrub %.2f  tail %.3f s\n",
                    name, r.rmsDb, r.centroid, 100.0 * r.below1k, 100.0 * r.above3k, r.scrub, r.tail);
    };
    print("sporty, squeal 1.0", a);
    print("trailforge_suv, squeal 1.0", b);
    print("  same without scrub", c);
    print("trailforge_suv, squeal 0.3", d);
    CHECK(b.centroid < 0.8 * a.centroid, "SUV centroid %.0f Hz vs sporty %.0f Hz", b.centroid, a.centroid);
    CHECK(b.below1k > 1.4 * a.below1k, "SUV energy below 1 kHz %.3f vs sporty %.3f", b.below1k, a.below1k);
    CHECK(b.above3k < 0.25 * a.above3k, "SUV energy above 3 kHz %.3f vs sporty %.3f", b.above3k, a.above3k);
    CHECK(std::abs(b.rmsDb - a.rmsDb) < 3.0, "SUV level %.1f vs sporty %.1f dBFS", b.rmsDb, a.rmsDb);
    CHECK(b.below1k > c.below1k + 0.05, "scrub adds low-band energy: %.3f vs %.3f", b.below1k, c.below1k);
    CHECK(a.scrub == 0.0f && b.scrub > 0.3f, "scrub level sporty %.2f / SUV %.2f", a.scrub, b.scrub);
    CHECK(b.rmsDb - d.rmsDb > 6.0, "SUV level not proportional to the squeal: %.1f vs %.1f dBFS", b.rmsDb, d.rmsDb);
    CHECK(a.tail < 0.2 && b.tail < 0.2 && d.tail < 0.2, "tails %.3f / %.3f / %.3f s", a.tail, b.tail, d.tail);
}

// Electric drive sound: the pure tone mapping, then the real EV simulation through the feed and the mixer (no engine
// start, no gear events, motor order at 2 x pole pairs x shaft speed, regen keeps the motor audible, road hum on
// concrete) and the spectrum of a steady cruise with the motor sound on and off.
void electricSound() {
    {
        const EvSoundSettings es;
        MotorAudio m;
        m.maxTorque = 450.0f;
        EvTones still = evMotorTones(m, 0.0f, es);
        float sum = 0.0f;
        for (float g : still.gain) sum += g;
        CHECK(sum == 0.0f, "standstill without torque must be silent (%.5f)", double(sum));
        m.torque = 300.0f;
        still = evMotorTones(m, 0.0f, es);
        CHECK(still.gain[5] > 0.0f && still.gain[0] == 0.0f, "standstill with torque: inverter only (%.5f / %.5f)",
              double(still.gain[5]), double(still.gain[0]));
        m.rpm = 6000.0f;
        const EvTones full = evMotorTones(m, 5.0f, es);
        m.torque = 0.0f;
        const EvTones coast = evMotorTones(m, 5.0f, es);
        m.torque = -300.0f;
        const EvTones regen = evMotorTones(m, 5.0f, es);
        std::printf("    6000 rpm: order %.0f Hz slot %.0f Hz gear %.0f / %.0f Hz; em gain full %.5f coast %.5f regen %.5f, "
                    "slot full %.5f regen %.5f\n",
                    double(full.hz[0]), double(full.hz[2]), double(full.hz[3]), double(full.hz[4]), double(full.gain[0]),
                    double(coast.gain[0]), double(regen.gain[0]), double(full.gain[2]), double(regen.gain[2]));
        CHECK(std::abs(full.hz[0] - 800.0f) < 0.1f && std::abs(full.hz[2] - 4800.0f) < 0.1f, "orders at 6000 rpm");
        CHECK(full.gain[0] > 4.0f * coast.gain[0] && coast.gain[0] > 0.0f, "torque raises the whine");
        CHECK(regen.gain[2] > full.gain[2] * 1.5f && regen.gain[0] < full.gain[0], "regen shifts to the slot order");
        m.torque = 300.0f;
        CHECK(evMotorTones(m, 20.0f, es).gain[5] == 0.0f, "inverter tone gone at 20 m/s");
    }

    VehicleConfig ev = evConfig();
    ev.tuning.aids.traction = TractionMode::Off; // the motor order must follow free wheelspin too
    Driver d(TrackLayout::SurfaceLanes, 1.0 / 120.0, &ev);
    d.run(1.0);
    CHECK(d.starts == 0, "an electric vehicle has no engine start (%d)", d.starts);
    float maxErr = 0.0f, launchLevel = 0.0f;
    for (int i = 0; i < 3 * 120; ++i) {
        d.tick(DriverIntent{.throttle = 1.0f});
        const EvTelemetry& e = d.sim.vehicle().telemetry().ev;
        const float rpm = std::max(std::abs(e.motorRPM[0]), std::abs(e.motorRPM[1]));
        if (i > 120 && rpm > 1000.0f) maxErr = std::max(maxErr, std::abs(d.mixer.stats().motorHz / (8.0f * rpm / 60.0f) - 1.0f));
        launchLevel = std::max(launchLevel, d.mixer.stats().motorLevel);
    }
    const float launchKmh = d.sim.vehicle().telemetry().speed * 3.6f;
    float regenLevel = 0.0f, regenTorque = 0.0f;
    for (int i = 0; i < 2 * 120; ++i) {
        d.tick(DriverIntent{});
        regenTorque = std::min(regenTorque, d.sim.vehicle().telemetry().ev.motorTorque[1]);
        if (i > 60) regenLevel = std::max(regenLevel, d.mixer.stats().motorLevel);
    }
    std::printf("    EV launch to %.0f km/h: %d gear events, motor order error %.1f %%, level %.4f; lift-off regen %.0f N*m "
                "level %.4f; engine stats rpm %.0f\n",
                double(launchKmh), d.shifts, 100.0 * double(maxErr), double(launchLevel), double(regenTorque),
                double(regenLevel), double(d.mixer.stats().rpm));
    CHECK(d.shifts == 0, "gear events on an electric vehicle: %d", d.shifts);
    CHECK(maxErr < 0.05f, "motor order does not follow the shaft speed (%.3f)", double(maxErr));
    CHECK(regenTorque < -50.0f && regenLevel > 0.1f * launchLevel, "regen not audible: %.0f N*m, level %.4f",
          double(regenTorque), double(regenLevel));

    // Steady 40 km/h on concrete (5-7 N*m per motor: near the no-load floor). With the road hum off and the synthetic
    // bank silenced (its test tones are sines too), the strongest spectral line is the motor order at the predicted
    // frequency and stands > 15 dB above the same run with the motor sound off (same seed and physics). With the road
    // hum on, the hum is louder than the light-load whine (reported, not a target).
    auto cruise = [&](float motor, bool roadHum, double* peakHz, float* road, float* level) {
        VehicleConfig cfg = ev;
        if (!roadHum) cfg.tireSound.roadGain = 0.0f;
        Driver c(TrackLayout::SurfaceLanes, 1.0 / 120.0, &cfg);
        for (SoundData& snd : c.bank.sounds) std::fill(snd.pcm.begin(), snd.pcm.end(), 0.0f);
        c.motorSetting = motor;
        c.run(0.5);
        c.cruise(6.0, 40.0f);
        std::vector<float> mono;
        c.tap = &mono;
        c.cruise(1.0, 40.0f);
        c.tap = nullptr;
        *peakHz = c.mixer.stats().motorHz;
        *road = c.mixer.stats().road;
        *level = c.mixer.stats().motorLevel;
        return powerSpectrum(mono);
    };
    double hzOn = 0.0, hzOff = 0.0, hzRoad = 0.0;
    float roadOn = 0.0f, roadOff = 0.0f, roadHumLevel = 0.0f, levelOn = 0.0f, levelOff = 0.0f, levelRoad = 0.0f;
    const std::vector<double> on = cruise(1.0f, false, &hzOn, &roadOn, &levelOn);
    const std::vector<double> off = cruise(0.0f, false, &hzOff, &roadOff, &levelOff);
    const std::vector<double> withRoad = cruise(1.0f, true, &hzRoad, &roadHumLevel, &levelRoad);
    const std::size_t bin = std::size_t(std::lround(hzOn * 4096.0 / 48000.0));
    double pOn = 0.0, pOff = 0.0, pRoad = 0.0, pRoadTotal = 0.0, pOnTotal = 0.0;
    for (std::size_t k = bin - 2; k <= bin + 2; ++k) pOn += on[k], pOff += off[k], pRoad += withRoad[k];
    for (std::size_t k = 1; k < on.size(); ++k) pOnTotal += on[k], pRoadTotal += withRoad[k];
    std::size_t peak = 1;
    for (std::size_t k = 1; k < on.size(); ++k)
        if (on[k] > on[peak]) peak = k;
    const double gainDb = 10.0 * std::log10(std::max(pOn, 1e-30) / std::max(pOff, 1e-30));
    std::printf("    40 km/h cruise: motor order %.0f Hz (level %.4f), strongest line %.0f Hz, +%.1f dB with the motor "
                "sound; with the road hum (%.2f) the order holds %.1f %% of the power\n",
                hzOn, double(levelOn), double(peak) * 48000.0 / 4096.0, gainDb, double(roadHumLevel),
                100.0 * pRoad / std::max(pRoadTotal, 1e-30));
    CHECK(hzOn > 250.0 && hzOn < 600.0, "motor order at 40 km/h %.0f Hz", hzOn);
    CHECK(peak + 2 >= bin && peak <= bin + 2, "strongest line %zu, motor order bin %zu", peak, bin);
    CHECK(gainDb > 15.0, "motor order only %.1f dB above the mix without it", gainDb);
    CHECK(roadHumLevel > 0.3f && roadOn == 0.0f, "road hum at 40 km/h on concrete %.2f (off: %.2f)",
          double(roadHumLevel), double(roadOn));

    Driver suv;
    suv.run(1.0);
    suv.cruise(4.0, 40.0f);
    CHECK(suv.mixer.stats().motorLevel == 0.0f && suv.mixer.stats().road == 0.0f, "SUV plays motor %.4f / road %.2f",
          double(suv.mixer.stats().motorLevel), double(suv.mixer.stats().road));
}

// Music bus with a synthetic streamed track: fades, volume, loop, restart, and no change while the game is paused.
void music() {
    struct Sine : MusicSource {
        long long pos = 0, length = 48000; // 1 s track
        int rewinds = 0, reads = 0;
        int read(float* stereo, int frames) override {
            ++reads;
            const int n = int(std::min<long long>(frames, length - pos));
            for (int i = 0; i < n; ++i) {
                const float v = 0.25f * float(std::sin(6.283185307 * 440.0 * double(pos + i) / 48000.0));
                stereo[2 * i] = stereo[2 * i + 1] = v;
            }
            pos += n;
            return n;
        }
        void rewind() override {
            pos = 0;
            ++rewinds;
        }
        int sampleRate() const override { return 48000; }
    } track;
    const SoundBank bank; // no other sounds
    AudioMixer mixer(bank);
    mixer.setMusic(&track);
    AudioFrame f;
    f.settings.master = 1.0f;
    f.settings.music = 1.0f;
    f.music = 1.0f;
    f.musicFadeSeconds = 1.0f;
    std::vector<float> buf(2 * 4800);
    auto rms = [&](double seconds) { // renders in 0.1 s chunks, returns the RMS of the last chunk
        double r = 0.0;
        for (int k = 0; k < int(std::lround(seconds * 10.0)); ++k) {
            mixer.setFrame(f);
            mixer.render(buf.data(), 4800);
            double p = 0.0;
            for (float x : buf) p += double(x) * double(x);
            r = std::sqrt(p / double(buf.size()));
        }
        return r;
    };
    const double fading = rms(0.5);
    const double full = rms(1.5);
    std::printf("    music rms: 0.5 s into a 1 s fade %.4f, full %.4f, track rewinds %d\n", fading, full,
                track.rewinds);
    CHECK(std::abs(full - 0.25 / std::sqrt(2.0)) < 0.01, "full music level %.4f", full);
    CHECK(fading > 0.3 * full && fading < 0.6 * full, "fade-in level %.4f of %.4f", fading, full);
    CHECK(track.rewinds >= 1, "the 1 s track did not loop after 2 s (rewinds %d)", track.rewinds);
    f.paused = true;
    const double paused = rms(1.0);
    CHECK(std::abs(paused - full) < 0.01 * full, "music changed while paused: %.4f vs %.4f", paused, full);
    f.paused = false;
    f.settings.music = 0.5f;
    const double half = rms(0.5);
    CHECK(std::abs(half - 0.5 * full) < 0.02 * full, "music volume 0.5: %.4f vs %.4f", half, full);
    f.music = 0.0f;
    f.musicFadeSeconds = 2.0f;
    rms(2.1);
    const int reads = track.reads;
    const double silent = rms(0.5);
    std::printf("    music after a 2 s fade-out: rms %.6f, gain %.3f, reads while silent %d\n", silent,
                mixer.stats().musicGain, track.reads - reads);
    CHECK(silent < 1e-6 && mixer.stats().musicGain == 0.0f, "music not silent after the fade-out: %.6f", silent);
    CHECK(track.reads == reads, "the track was decoded while silent");
    const int rewinds = track.rewinds;
    f.musicRestart = 1;
    f.music = 1.0f;
    rms(0.1);
    CHECK(track.rewinds == rewinds + 1 && mixer.stats().musicSeconds < 0.11, "restart: rewinds %d -> %d, t %.3f",
          rewinds, track.rewinds, mixer.stats().musicSeconds);
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"audio_music", music},
        {"audio_engine_layers", engineLayers},
        {"audio_tire_layers", tireLayers},
        {"audio_manifest", manifest},
        {"audio_feed_and_mixer", feedAndMixer},
        {"audio_skid_gating", skidGating},
        {"audio_squeal_release", squealRelease},
        {"audio_tire_profiles", tireProfiles},
        {"audio_electric", electricSound},
    };
    int ran = 0;
    for (const auto& [name, fn] : tests) {
        if (argc > 1 && std::strcmp(argv[1], name) != 0) continue;
        std::printf("[ RUN ] %s\n", name);
        const int before = g_failures;
        fn();
        std::printf("[%s] %s\n", g_failures == before ? "  OK " : "FAIL ", name);
        ++ran;
    }
    if (ran == 0) {
        std::printf("unknown test '%s'\n", argc > 1 ? argv[1] : "");
        return 2;
    }
    return g_failures == 0 ? 0 : 1;
}
