#include "audio/AudioMixer.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace tf {
namespace {

constexpr float kCeiling = 0.84f;     // limiter threshold, -1.5 dBFS
constexpr float kHardCeiling = 0.89f; // safety clamp after the limiter, -1 dBFS

// Mix levels (linear) before the user bus volumes. The loops are normalized to -18 LUFS, one-shots to a true peak.
constexpr float kExhaustGain = 0.80f;
constexpr float kBayGain = 0.45f;
constexpr std::array<float, kTireLayerCount> kTireGain = {0.75f, 0.55f, 0.55f, 0.9f, 0.7f}; // skid gravel dirt mud grass
constexpr float kWindGain = 0.35f;
constexpr float kDayGain = 0.30f, kNightGain = 0.24f;
constexpr float kStarterFadeStart = 2.3f, kStarterFadeEnd = 4.3f; // s in engine_start.wav: catch, then idle
constexpr float kStarterGain = 0.65f;
constexpr float kPinionTeeth = 10.0f; // whine = propeller shaft speed x pinion teeth

float smoothstep(float a, float b, float x) {
    const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float approach(float cur, float target, float tau, float dt) {
    return cur + (target - cur) * (1.0f - std::exp(-dt / tau));
}

inline float hermite(float xm1, float x0, float x1, float x2, float t) {
    const float c1 = 0.5f * (x1 - xm1);
    const float c2 = xm1 - 2.5f * x0 + 2.0f * x1 - 0.5f * x2;
    const float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
    return ((c3 * t + c2) * t + c1) * t + x0;
}

} // namespace

AudioMixer::AudioMixer(const SoundBank& bank, int sampleRate, std::uint32_t seed)
    : m_bank(bank), m_rate(sampleRate), m_rng(seed ? seed : 1u) {
    m_acc.resize(std::size_t(kBlock) * 2);
    auto loopVoice = [&](Voice& v, SoundId id, Bus bus, bool gameplay) {
        v.data = &m_bank[id];
        v.loop = true;
        v.active = v.data->frames > 0;
        v.bus = bus;
        v.gameplay = gameplay;
        // Random start offsets decorrelate loops that play together.
        v.pos = double(rand01()) * double(v.data->frames);
    };
    for (int i = 0; i < 12; ++i) {
        loopVoice(m_engine[std::size_t(i)], SoundId(i), Bus::Engine, true);
        m_engineRpm[std::size_t(i)] = std::max(1.0f, m_bank.manifest[SoundId(i)].rpmMeasured);
    }
    const SoundId tireIds[kTireLayerCount] = {SoundId::TireSkidLoop, SoundId::TireGravelLoop, SoundId::TireDirtLoop,
                                              SoundId::TireMudLoop, SoundId::GrassRustleLoop};
    for (int i = 0; i < kTireLayerCount; ++i) loopVoice(m_tires[std::size_t(i)], tireIds[i], Bus::Tires, true);
    m_tires[std::size_t(TireLayer::Skid)].active = false;
    for (SkidVoice& s : m_skid) {
        loopVoice(s.voice, SoundId::TireSkidLoop, Bus::Tires, true);
        s.wobblePhase = 6.2831853f * rand01();
    }
    loopVoice(m_wind, SoundId::WindLoop, Bus::Ambience, true);
    loopVoice(m_day, SoundId::AmbienceDayLoop, Bus::Ambience, false);
    loopVoice(m_night, SoundId::AmbienceNightLoop, Bus::Ambience, false);
    loopVoice(m_menu, SoundId::UiMenuAmbienceLoop, Bus::Ui, false);
    m_day.positional = m_night.positional = m_menu.positional = false;
}

float AudioMixer::rand01() {
    m_rng ^= m_rng << 13;
    m_rng ^= m_rng >> 17;
    m_rng ^= m_rng << 5;
    return float(m_rng >> 8) / float(1u << 24);
}

SoundId AudioMixer::pick(SoundId first, int count) {
    return SoundId(int(first) + std::min(count - 1, int(rand01() * float(count))));
}

void AudioMixer::recomputeEngineLevels(float spreadDb) {
    m_levelSpread = spreadDb;
    for (int mic = 0; mic < 2; ++mic) {
        float lo = 1e9f, hi = -1e9f;
        for (int k = 0; k < 6; ++k) {
            const float db = m_bank.manifest[SoundId(mic * 6 + k)].sourceLevelDb;
            lo = std::min(lo, db);
            hi = std::max(hi, db);
        }
        for (int k = 0; k < 6; ++k) {
            const float db = m_bank.manifest[SoundId(mic * 6 + k)].sourceLevelDb;
            m_engineLevel[std::size_t(mic * 6 + k)] = dbToGain(compressedLevelDb(db, lo, hi, spreadDb));
        }
    }
}

StereoGain AudioMixer::place(const AudioVec3& p) const {
    return spatialize(p, m_frame.listenerPos, m_frame.listenerRight, m_frame.listenerForward);
}

void AudioMixer::setLoop(Voice& v, float gain, const StereoGain& g) {
    float bus = m_bus[std::size_t(v.bus)] * m_master;
    if (v.gameplay) bus *= m_gameplay;
    v.tl = g.l * gain * bus;
    v.tr = g.r * gain * bus;
}

AudioMixer::Voice* AudioMixer::startOneShot(SoundId id, float gain, Bus bus, const AudioVec3* position, float rate) {
    const SoundData& d = m_bank[id];
    if (d.frames == 0) return nullptr;
    Voice* slot = nullptr;
    float mostDone = -1.0f;
    for (Voice& v : m_shots) {
        if (!v.active) {
            slot = &v;
            break;
        }
        const float done = float(v.pos / double(v.data->frames));
        if (done > mostDone) {
            mostDone = done;
            slot = &v;
        }
    }
    Voice& v = *slot;
    v = Voice{};
    v.data = &d;
    v.rate = rate;
    v.bus = bus;
    v.loop = false;
    v.active = true;
    v.gameplay = bus != Bus::Ui;
    v.positional = position != nullptr;
    if (position) v.position = *position;
    v.gain = gain;
    // Start at the target gain (no fade-in: the files have their own 2 ms fade).
    StereoGain g = v.positional ? place(v.position) : StereoGain{1.0f, 1.0f};
    setLoop(v, v.gain, g);
    v.gl = v.tl;
    v.gr = v.tr;
    ++m_stats.oneShotsPlayed;
    return &v;
}

void AudioMixer::playUi(UiSound s) {
    static constexpr SoundId kUi[] = {SoundId::UiClick, SoundId::UiHover, SoundId::UiConfirm, SoundId::UiBack,
                                      SoundId::UiSliderTick, SoundId::UiMenuOpen, SoundId::UiMenuClose};
    startOneShot(kUi[std::size_t(s)], 0.8f, Bus::Ui, nullptr);
}

void AudioMixer::trigger(const AudioEvent& e) {
    const std::size_t type = std::size_t(e.type);
    if (type < m_stats.eventsByType.size()) ++m_stats.eventsByType[type];
    const float i = std::clamp(e.intensity, 0.0f, 1.0f);
    const float jitter = 0.94f + 0.12f * rand01();
    switch (e.type) {
    case AudioEventType::EngineStart:
        m_startElapsed = 0.0;
        m_starterHandover = false;
        if (m_starter) m_starter->gain = 0.0f;
        m_starter = startOneShot(SoundId::EngineStart, kStarterGain, Bus::Engine, &e.position,
                                 m_frame.settings.enginePitch);
        break;
    case AudioEventType::GearShift:
        startOneShot(pick(SoundId::GearShift1, 3), 0.35f * i, Bus::Engine, &e.position, jitter);
        break;
    case AudioEventType::BumpStop:
        startOneShot(pick(SoundId::BottomOut1, 3), 0.55f * (0.4f + 0.6f * i), Bus::Impacts, &e.position, jitter);
        break;
    case AudioEventType::SuspensionThump:
        startOneShot(SoundId::SuspensionThump, 0.5f * i, Bus::Impacts, &e.position, 0.85f + 0.3f * rand01());
        break;
    case AudioEventType::BodyImpact:
        startOneShot(i > 0.6f ? SoundId::BodyImpactClank : pick(SoundId::BodyImpactMetal1, 3),
                     0.6f * (0.3f + 0.7f * i), Bus::Impacts, &e.position, jitter);
        break;
    case AudioEventType::Splash:
        startOneShot(pick(SoundId::WaterSplash1, 3), 0.6f * (0.4f + 0.6f * i), Bus::Tires, &e.position, jitter);
        break;
    case AudioEventType::ChassisCreak:
        startOneShot(pick(SoundId::ChassisCreak1, 3), 0.3f * i, Bus::Impacts, &e.position, jitter);
        break;
    case AudioEventType::Ui:
        playUi(e.ui);
        break;
    }
}

void AudioMixer::controlUpdate(float dt) {
    const AudioFrame& f = m_frame;
    const AudioSettings& s = f.settings;
    m_master = approach(m_master, s.mute ? 0.0f : s.master, 0.05f, dt);
    const float busTarget[int(Bus::Count)] = {s.engine, s.tires, s.impacts, s.ambience, s.ui};
    for (int b = 0; b < int(Bus::Count); ++b)
        m_bus[std::size_t(b)] = approach(m_bus[std::size_t(b)], busTarget[b], 0.05f, dt);
    m_gameplay = approach(m_gameplay, f.paused ? 0.0f : 1.0f, 0.06f, dt);
    if (f.paused && m_gameplay < 1e-3f) m_gameplay = 0.0f;
    const bool frozen = f.paused && m_gameplay == 0.0f;
    if (!frozen) m_startElapsed += dt;
    if (s.levelSpreadDb != m_levelSpread) recomputeEngineLevels(s.levelSpreadDb);
    m_pitch = s.enginePitch;

    // ---- Engine
    m_rpm = m_rpm <= 0.0f ? f.rpm : approach(m_rpm, f.rpm, 0.03f, dt);
    const float loadTarget = std::clamp(f.throttle, 0.0f, 1.0f);
    m_load = approach(m_load, loadTarget, loadTarget > m_load ? 0.08f : 0.2f, dt);
    EngineMixSettings ems;
    ems.pitch = m_pitch;
    const EngineLayerWeights w = engineLayerWeights(m_rpm, m_load, ems);
    // The recorded start settles to idle; the loops take over after the catch, or at once when the throttle opens
    // (the simulated engine revs immediately, the recording would not).
    if (m_starter && (!m_starter->active || m_starter->data != &m_bank[SoundId::EngineStart])) m_starter = nullptr;
    if (f.throttle > 0.1f) m_starterHandover = true;
    float fadeTarget = 0.0f;
    if (f.vehicleValid && m_startElapsed < 1e8)
        fadeTarget = m_starterHandover ? 1.0f
                                       : smoothstep(kStarterFadeStart, kStarterFadeEnd, float(m_startElapsed) * m_pitch);
    m_engineFade = approach(m_engineFade, fadeTarget, m_starterHandover ? 0.06f : 0.05f, dt);
    if (m_starter) m_starter->gain = kStarterGain * std::sqrt(std::max(0.0f, 1.0f - m_engineFade * m_engineFade));
    const AudioVec3 exhaust = f.vehiclePos - f.vehicleForward * 2.3f - f.vehicleUp * 0.35f - f.vehicleLeft * 0.45f;
    const AudioVec3 bay = f.vehiclePos + f.vehicleForward * 1.7f + f.vehicleUp * 0.3f;
    const StereoGain gExhaust = place(exhaust), gBay = place(bay);
    for (int i = 0; i < 12; ++i) {
        const int mic = i / 6, load = (i / 3) % 2, band = i % 3;
        const float wi = load ? w.on[std::size_t(band)] : w.off[std::size_t(band)];
        Voice& v = m_engine[std::size_t(i)];
        v.rate = std::clamp(enginePlaybackRate(m_rpm, m_pitch, m_engineRpm[std::size_t(i)]), 0.25f, 4.0f);
        setLoop(v, wi * m_engineLevel[std::size_t(i)] * (mic ? kBayGain : kExhaustGain) * m_engineFade,
                mic ? gBay : gExhaust);
    }
    for (int b = 0; b < 3; ++b) {
        m_stats.offWeights[std::size_t(b)] = w.off[std::size_t(b)];
        m_stats.onWeights[std::size_t(b)] = w.on[std::size_t(b)];
        m_stats.exhaustRates[std::size_t(b)] = m_engine[std::size_t(b)].rate;
    }
    m_stats.rpm = f.rpm;
    m_stats.sourceRpm = m_rpm * m_pitch;
    m_stats.load = m_load;

    // Transmission whine: gear mesh frequency of the propeller shaft, louder under load.
    const float whineHz = f.driveshaftHz * kPinionTeeth;
    m_whineHz = approach(m_whineHz, whineHz, 0.05f, dt);
    m_whineGain = s.whine * 0.012f * smoothstep(40.0f, 140.0f, m_whineHz) * (0.35f + 0.65f * m_load) * m_engineFade;
    {
        const StereoGain g = place(f.vehiclePos - f.vehicleUp * 0.3f + f.vehicleForward * 0.3f);
        const float bus = m_bus[std::size_t(Bus::Engine)] * m_master * m_gameplay;
        m_whineTl = g.l * m_whineGain * bus;
        m_whineTr = g.r * m_whineGain * bus;
    }
    m_stats.whineHz = m_whineHz;

    // ---- Tires
    const TireLayerMix tm = tireLayerMix(f.wheels, f.referenceLoad);
    float meanRoll = 0.0f;
    for (const WheelAudio& wa : f.wheels) meanRoll += 0.25f * wa.rollSpeed;
    const float rollT = std::clamp(meanRoll / 15.0f, 0.0f, 1.0f);
    for (int l = 0; l < kTireLayerCount; ++l) {
        if (TireLayer(l) == TireLayer::Skid) continue;
        const std::size_t k = std::size_t(l);
        const float target = f.vehicleValid ? tm.gain[k] : 0.0f;
        m_tireGain[k] = approach(m_tireGain[k], target, target > m_tireGain[k] ? 0.04f : 0.12f, dt);
        Voice& v = m_tires[k];
        v.rate = 0.8f + 0.4f * rollT;
        setLoop(v, m_tireGain[k] * kTireGain[k], place(tm.position[k]));
    }

    // Rubber squeal: one voice per wheel, driven by that wheel's gated squeal (hard surfaces only, see
    // VehicleAudioFeed). Every slide starts at a random point of the loop with its own pitch; pitch follows the
    // sliding speed and load, brightness the intensity, and a slow wobble keeps a held slide from sounding static.
    // A fresh slide starts with the short squeal; the loop is ducked under it and takes over as it decays.
    float skidPower = 0.0f;
    for (int i = 0; i < kAudioWheels; ++i) {
        SkidVoice& sv = m_skid[std::size_t(i)];
        const WheelAudio& wa = f.wheels[std::size_t(i)];
        const float target = f.vehicleValid ? std::min(wa.squeal, 1.2f) : 0.0f;
        if (!frozen) {
            if (sv.gain < 1e-3f && target > 0.0f) {
                sv.voice.pos = double(rand01()) * double(std::max<std::size_t>(sv.voice.data->frames, 1));
                sv.jitter = 0.93f + 0.14f * rand01();
                sv.wobbleHz = 2.0f + 3.0f * rand01();
            }
            const bool onset = target > 0.3f && sv.prevTarget <= 0.3f;
            if (onset && m_time - sv.lastChirp > 0.8 && m_time - m_lastChirp > 0.25) {
                const AudioVec3 p = wa.position;
                startOneShot(SoundId::TireSkidShort, 0.32f * std::min(1.0f, target) * (0.85f + 0.3f * rand01()),
                             Bus::Tires, &p, sv.jitter * (0.92f + 0.16f * rand01()));
                sv.lastChirp = m_time;
                sv.sinceChirp = 0.0;
                m_lastChirp = m_time;
                ++m_stats.skidChirps;
            }
            sv.prevTarget = target;
            sv.sinceChirp += dt;
            sv.wobblePhase = std::fmod(sv.wobblePhase + 6.2831853f * sv.wobbleHz * dt, 6.2831853f * 100.0f);
        }
        sv.gain = approach(sv.gain, target, target > sv.gain ? 0.06f : 0.10f, dt);
        if (sv.gain < 1e-4f && target == 0.0f) sv.gain = 0.0f;
        const float slide = smoothstep(1.0f, 12.0f, wa.slipSpeed);
        const float load = std::clamp(wa.load / std::max(f.referenceLoad, 1.0f), 0.0f, 1.6f);
        sv.voice.rate = sv.jitter * (0.90f + 0.14f * slide) * (1.04f - 0.05f * load) *
                        (1.0f + 0.012f * std::sin(sv.wobblePhase));
        const float cutoff = 2500.0f + 9500.0f * std::min(1.0f, sv.gain);
        sv.voice.lowpass = 1.0f - std::exp(-6.2831853f * cutoff / float(m_rate));
        const float wobble = 1.0f + 0.12f * std::sin(0.73f * sv.wobblePhase + 1.3f);
        const float duck = 0.4f + 0.6f * smoothstep(0.15f, 0.8f, float(sv.sinceChirp));
        // 0.5: four uncorrelated voices add up in power like the RMS of the four wheels.
        setLoop(sv.voice, 0.5f * kTireGain[0] * sv.gain * wobble * duck, place(wa.position));
        m_stats.skidWheel[std::size_t(i)] = sv.gain;
        skidPower += sv.gain * sv.gain;
    }
    m_tireGain[std::size_t(TireLayer::Skid)] = std::sqrt(0.25f * skidPower);
    m_stats.skid = m_tireGain[0];
    m_stats.gravel = m_tireGain[1];
    m_stats.dirt = m_tireGain[2];
    m_stats.mud = m_tireGain[3];
    m_stats.grass = m_tireGain[4];
    if (!frozen) {
        const float mud = m_tireGain[std::size_t(TireLayer::Mud)];
        if (rand01() < 3.0f * mud * dt) {
            const AudioVec3 p = tm.position[std::size_t(TireLayer::Mud)];
            startOneShot(pick(SoundId::MudSquelch1, 4), 0.5f * (0.5f + 0.5f * std::min(1.0f, mud)), Bus::Tires, &p,
                         0.9f + 0.2f * rand01());
        }
        const float grass = m_tireGain[std::size_t(TireLayer::Grass)];
        if (rand01() < 0.8f * grass * dt) {
            const AudioVec3 p = tm.position[std::size_t(TireLayer::Grass)];
            startOneShot(pick(SoundId::GrassSwish1, 3), 0.3f * std::min(1.0f, grass), Bus::Tires, &p,
                         0.9f + 0.2f * rand01());
        }
    }

    // ---- Wind and ambience
    m_windGain = approach(m_windGain, f.vehicleValid ? windGain(f.speed) : 0.0f, 0.25f, dt);
    m_wind.rate = 0.85f + 0.3f * std::sqrt(m_windGain);
    setLoop(m_wind, kWindGain * m_windGain, place(f.vehiclePos + f.vehicleUp * 1.0f));
    m_dayAmount = approach(m_dayAmount, std::clamp(f.dayAmount, 0.0f, 1.0f), 1.0f, dt);
    const auto dn = dayNightWeights(m_dayAmount);
    const float duck = 0.5f + 0.5f * m_gameplay; // ambience ducks (does not stop) while paused
    setLoop(m_day, kDayGain * dn[0] * duck, {1.0f, 1.0f});
    setLoop(m_night, kNightGain * dn[1] * duck, {1.0f, 1.0f});
    setLoop(m_menu, f.menuAmbience ? 1.0f : 0.0f, {1.0f, 1.0f});
    m_stats.wind = m_windGain;
    m_stats.day = dn[0];
    m_stats.night = dn[1];
    m_stats.gameplayGain = m_gameplay;

    // One-shots follow the bus volumes and the pause (their position stays where they started).
    for (Voice& v : m_shots) {
        if (!v.active) continue;
        setLoop(v, v.gain, v.positional ? place(v.position) : StereoGain{1.0f, 1.0f});
    }
}

void AudioMixer::mixVoice(Voice& v, float* acc, int frames) {
    const SoundData& d = *v.data;
    const long long n = (long long)d.frames;
    if (n == 0) {
        v.active = false;
        return;
    }
    int start = 0;
    if (v.delay > 0) {
        start = std::min(frames, v.delay);
        v.delay -= start;
        if (start == frames) return;
    }
    if (v.tl == 0.0f && v.tr == 0.0f && v.gl == 0.0f && v.gr == 0.0f) {
        // Silent: advance only (keeps loop phase continuous).
        v.pos += double(v.rate) * double(frames - start);
        if (v.loop) v.pos = std::fmod(v.pos, double(n));
        else if (v.pos >= double(n)) v.active = false;
        return;
    }
    const float invN = 1.0f / float(frames);
    const float dl = (v.tl - v.gl) * invN, dr = (v.tr - v.gr) * invN;
    const float* pcm = d.pcm.data();
    const int ch = d.channels;
    auto at = [&](long long k, int c) -> float {
        if (v.loop) {
            k %= n;
            if (k < 0) k += n;
        } else if (k < 0 || k >= n) {
            return 0.0f;
        }
        return pcm[k * ch + c];
    };
    for (int i = start; i < frames; ++i) {
        const long long i0 = (long long)v.pos;
        if (!v.loop && i0 >= n) {
            v.active = false;
            break;
        }
        const float t = float(v.pos - double(i0));
        float l, r;
        if (i0 >= 1 && i0 + 2 < n) {
            const float* p = pcm + (i0 - 1) * ch;
            l = hermite(p[0], p[ch], p[2 * ch], p[3 * ch], t);
            r = ch > 1 ? hermite(p[1], p[ch + 1], p[2 * ch + 1], p[3 * ch + 1], t) : l;
        } else {
            l = hermite(at(i0 - 1, 0), at(i0, 0), at(i0 + 1, 0), at(i0 + 2, 0), t);
            r = ch > 1 ? hermite(at(i0 - 1, 1), at(i0, 1), at(i0 + 1, 1), at(i0 + 2, 1), t) : l;
        }
        if (v.lowpass > 0.0f) {
            v.lpL += v.lowpass * (l - v.lpL);
            v.lpR += v.lowpass * (r - v.lpR);
            l = v.lpL;
            r = v.lpR;
        }
        acc[2 * i] += l * (v.gl + dl * float(i));
        acc[2 * i + 1] += r * (v.gr + dr * float(i));
        v.pos += double(v.rate);
        if (v.loop && v.pos >= double(n)) v.pos -= double(n);
    }
}

void AudioMixer::renderBlock(float* out, int frames) {
    const float dt = float(frames) / float(m_rate);
    controlUpdate(dt);
    const bool frozen = m_frame.paused && m_gameplay == 0.0f;
    float* acc = m_acc.data();
    std::fill(acc, acc + 2 * frames, 0.0f);
    int voices = 0;
    auto run = [&](Voice& v) {
        if (!v.active || (frozen && v.gameplay)) return;
        mixVoice(v, acc, frames);
        if (v.tl > 1e-5f || v.tr > 1e-5f) ++voices;
        v.gl = v.tl;
        v.gr = v.tr;
    };
    for (Voice& v : m_engine) run(v);
    for (Voice& v : m_tires) run(v);
    for (SkidVoice& s : m_skid) run(s.voice);
    run(m_wind);
    run(m_day);
    run(m_night);
    run(m_menu);
    for (Voice& v : m_shots) run(v);
    m_stats.activeVoices = voices;

    if (!frozen && (m_whineTl > 1e-6f || m_whineL > 1e-6f)) {
        const float invN = 1.0f / float(frames);
        const double inc = double(m_whineHz) / double(m_rate) * 6.283185307179586;
        for (int i = 0; i < frames; ++i) {
            const float gl = m_whineL + (m_whineTl - m_whineL) * float(i) * invN;
            const float gr = m_whineR + (m_whineTr - m_whineR) * float(i) * invN;
            // Mesh tone with a weaker second harmonic.
            const float s = float(std::sin(m_whinePhase) + 0.35 * std::sin(2.0 * m_whinePhase));
            acc[2 * i] += s * gl;
            acc[2 * i + 1] += s * gr;
            m_whinePhase += inc;
            if (m_whinePhase > 6.283185307179586) m_whinePhase -= 6.283185307179586;
        }
    }
    m_whineL = m_whineTl;
    m_whineR = m_whineTr;

    // Look-ahead limiter: the output is delayed by kLookahead frames; the gain follows the largest peak inside the
    // window, reaching ~99 % of the needed reduction before the peak leaves the delay line.
    const float attack = 1.0f - std::exp(-5.0f / float(kLookahead));
    const float release = 1.0f - std::exp(-1.0f / (0.15f * float(m_rate)));
    // One-pole DC blocker (~10 Hz): the exhaust-mic loops carry up to -0.04 DC offset.
    const float dcR = 1.0f - 2.0f * 3.14159265f * 10.0f / float(m_rate);
    for (int i = 0; i < frames; ++i) {
        const float xl = acc[2 * i], xr = acc[2 * i + 1];
        const float l = xl - m_dcInL + dcR * m_dcOutL, r = xr - m_dcInR + dcR * m_dcOutR;
        m_dcInL = xl;
        m_dcInR = xr;
        m_dcOutL = l;
        m_dcOutR = r;
        const float dl = m_delay[std::size_t(2 * m_delayPos)], dr = m_delay[std::size_t(2 * m_delayPos + 1)];
        m_delay[std::size_t(2 * m_delayPos)] = l;
        m_delay[std::size_t(2 * m_delayPos + 1)] = r;
        m_peaks[std::size_t(m_delayPos)] = std::max(std::abs(l), std::abs(r));
        m_delayPos = (m_delayPos + 1) % kLookahead;
        float window = 0.0f;
        for (float p : m_peaks) window = std::max(window, p);
        const float target = window > kCeiling ? kCeiling / window : 1.0f;
        m_limGain += (target - m_limGain) * (target < m_limGain ? attack : release);
        float ol = dl * m_limGain, orr = dr * m_limGain;
        if (std::abs(ol) > kHardCeiling || std::abs(orr) > kHardCeiling) {
            ++m_stats.clippedSamples;
            ol = std::clamp(ol, -kHardCeiling, kHardCeiling);
            orr = std::clamp(orr, -kHardCeiling, kHardCeiling);
        }
        out[2 * i] = ol;
        out[2 * i + 1] = orr;
        m_peakHold = std::max({m_peakHold, std::abs(ol), std::abs(orr)});
    }
    m_stats.limiterDb = gainToDb(m_limGain);
    m_peakTimer += dt;
    if (m_peakTimer >= 0.5) {
        m_stats.peakDb = gainToDb(m_peakHold);
        m_peakHold = 0.0f;
        m_peakTimer = 0.0;
    }
    m_time += dt;
}

void AudioMixer::render(float* stereo, int frames) {
    const auto t0 = std::chrono::steady_clock::now();
    for (int done = 0; done < frames;) {
        const int n = std::min(kBlock, frames - done);
        renderBlock(stereo + 2 * done, n);
        done += n;
    }
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    const double budgetUs = 1e6 * double(frames) / double(m_rate);
    m_stats.callbackUs = m_stats.framesRendered == 0 ? us : m_stats.callbackUs * 0.98 + us * 0.02;
    const double decay = std::exp(-double(frames) / (2.0 * double(m_rate)));
    m_stats.callbackMaxUs = std::max(us, m_stats.callbackMaxUs * decay);
    m_stats.cpuLoad = m_stats.framesRendered == 0 ? us / budgetUs : m_stats.cpuLoad * 0.98 + (us / budgetUs) * 0.02;
    m_stats.framesRendered += frames;
    m_stats.running = true;
}

} // namespace tf
