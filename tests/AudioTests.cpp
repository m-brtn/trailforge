// Headless audio checks: parameter mapping, manifest, and the feed + mixer driven by the real simulation with a
// synthetic sound bank (no audio device). Usage: trailforge_audio_tests <test_name>

#include "audio/AudioManifest.h"
#include "audio/AudioMapping.h"
#include "audio/AudioMixer.h"
#include "audio/VehicleAudioFeed.h"
#include "sim/Simulation.h"
#include "sim/TireMarks.h"

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
    w.load = load;
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
    auto squealing = [&](WheelAudio w, float slide) {
        w.slide = slide;
        w.squeal = wheelSquealDrive(w, ref);
        return w;
    };
    const TireLayerMix concreteRoll = tireLayerMix(all(wheel(SurfaceId::Concrete, 10, 0.3f, 0.03f)), ref);
    const TireLayerMix concreteLock = tireLayerMix(all(squealing(wheel(SurfaceId::Concrete, 8, 8, 1.0f), 1.0f)), ref);
    const TireLayerMix tractionSlip = tireLayerMix(all(squealing(wheel(SurfaceId::Concrete, 8, 1.7f, 0.12f), 0.0f)), ref);
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
    const float lightSq = wheelSquealDrive(squealing(wheel(SurfaceId::Concrete, 8, 8, 1, 1000), 1.0f), ref);
    const float heavySq = wheelSquealDrive(squealing(wheel(SurfaceId::Concrete, 8, 8, 1, 6000), 1.0f), ref);
    CHECK(lightSq < heavySq, "squeal x load: %.2f vs %.2f", lightSq, heavySq);

    // Squeal surface gate: rubber on hard ground only; a full slide on soft ground never squeals.
    std::printf("    squeal drive of a full slide:");
    const SurfaceId surfaces[] = {SurfaceId::Concrete, SurfaceId::PackedDirt, SurfaceId::Rock,
                                  SurfaceId::LowGrip,  SurfaceId::Mud,        SurfaceId::Grass};
    for (SurfaceId s : surfaces) {
        const float d = wheelSquealDrive(squealing(wheel(s, 8, 8, 1.0f), 1.0f), ref);
        std::printf(" %d:%.2f", int(s), d);
        const bool hard = s == SurfaceId::Concrete || s == SurfaceId::Rock || s == SurfaceId::LowGrip;
        CHECK(hard ? d > 0.3f : d == 0.0f, "surface %d squeal drive %.2f", int(s), d);
    }
    std::printf("\n");
    WheelAudio dirty = wheel(SurfaceId::Concrete, 8, 8, 1.0f);
    dirty.soil = 1.0f;
    CHECK(wheelSquealDrive(squealing(dirty, 1.0f), ref) == 0.0f, "a tread full of soil squeals");

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

struct Driver {
    explicit Driver(TrackLayout layout = TrackLayout::SurfaceLanes, double fixedStep = 1.0 / 120.0)
        : sim([&] {
              SimulationSettings s;
              s.layout = layout;
              s.fixedStep = fixedStep;
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
        mixer.setFrame(f);
        for (const AudioEvent& e : feed.events()) {
            shifts += e.type == AudioEventType::GearShift;
            starts += e.type == AudioEventType::EngineStart;
            mixer.trigger(e);
        }
        feed.clearEvents();
        mixer.render(buf.data(), framesPerTick); // one physics tick at 48 kHz
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

    // Pause: gameplay sounds fade out and freeze; only the (ducked) ambience keeps playing.
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

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"audio_engine_layers", engineLayers},
        {"audio_tire_layers", tireLayers},
        {"audio_manifest", manifest},
        {"audio_feed_and_mixer", feedAndMixer},
        {"audio_skid_gating", skidGating},
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
