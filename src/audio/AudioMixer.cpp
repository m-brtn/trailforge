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
// Scrub noise level at scrubGain 1 and full squeal: band-limited noise of RMS ~0.1 per wheel (four uncorrelated
// wheels add in power), comparable to one squeal loop voice.
constexpr float kScrubLevel = 0.6f;
constexpr float kPinionTeeth = 10.0f; // whine = propeller shaft speed x pinion teeth
// Road hum at roadGain 1 and full hum: band noise (70..450 Hz) of RMS ~0.03 per wheel.
constexpr float kRoadLevel = 0.9f;
constexpr float kRoadLowHz = 70.0f, kRoadHighHz = 450.0f;
// Body shake: output level of the synthesized rattle at full shake and of a full-intensity knock (resonator
// amplitude 1 per unit impulse), before the profile gains and the impacts bus.
constexpr float kRattleLevel = 0.25f;
constexpr float kKnockLevel = 0.18f;
constexpr float kShakeGate = 0.03f; // no new rattle grains below this envelope
constexpr std::array<float, 3> kKnockModeAmp = {1.0f, 0.6f, 0.35f};
constexpr std::array<float, 3> kKnockModeDecay = {1.0f, 0.7f, 0.45f};
constexpr float kBottomOutHighpassHz = 250.0f; // the recorded slams keep their body rattle, not their low thud

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
    m_musicBuf.resize(std::size_t(kBlock) * 2);
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
        s.noise = 0x9E3779B9u * std::uint32_t(&s - m_skid.data() + 1);
    }
    loopVoice(m_wind, SoundId::WindLoop, Bus::Ambience, true);
    loopVoice(m_day, SoundId::AmbienceDayLoop, Bus::Ambience, false);
    loopVoice(m_night, SoundId::AmbienceNightLoop, Bus::Ambience, false);
    loopVoice(m_menu, SoundId::UiMenuAmbienceLoop, Bus::Ui, false);
    m_day.positional = m_night.positional = m_menu.positional = false;
    for (std::size_t i = 0; i < m_road.size(); ++i) m_road[i].noise = 0x85EBCA6Bu * std::uint32_t(i + 1);
    m_roadHp = std::exp(-6.2831853f * kRoadLowHz / float(m_rate));
    m_roadLp = 1.0f - std::exp(-6.2831853f * kRoadHighHz / float(m_rate));
    retuneBody(m_bodyProfile);
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
    v.serial = ++m_shotSerial;
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
        startKnock(i, e.position);
        shake(0.3f + 0.4f * i, 0.5f * m_bodyProfile.rattleDecay);
        // A hard hit also gets a recorded car-body slam, high-passed so only its clunk and rattle remain.
        if (i > 0.5f) {
            if (Voice* v = startOneShot(pick(SoundId::BottomOut1, 3), 0.3f * i, Bus::Impacts, &e.position, jitter))
                v->highpass = std::exp(-6.2831853f * kBottomOutHighpassHz / float(m_rate));
        }
        break;
    case AudioEventType::BodyRattle:
        shake(0.25f + 0.75f * i, m_bodyProfile.rattleDecay * (0.5f + 0.5f * i));
        break;
    case AudioEventType::BodyImpact:
        startOneShot(i > 0.6f ? SoundId::BodyImpactClank : pick(SoundId::BodyImpactMetal1, 3),
                     0.45f * (0.3f + 0.7f * i), Bus::Impacts, &e.position, jitter);
        shake(0.3f + 0.7f * i, m_bodyProfile.rattleDecay);
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
    const float fx = s.effects;
    const float busTarget[int(Bus::Count)] = {s.engine * fx, s.tires * fx, s.impacts * fx, s.ambience * fx, s.ui, s.music};
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
    if (f.electric) fadeTarget = 0.0f;
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

    // Electric drive: motor tones placed at each axle (front motor ahead of the body centre, rear behind).
    m_evFade = approach(m_evFade, f.electric && f.vehicleValid ? 1.0f : 0.0f, 0.1f, dt);
    {
        const float bus = m_bus[std::size_t(Bus::Engine)] * m_master * m_gameplay;
        float power = 0.0f, inverter = 0.0f, mainHz = 0.0f;
        for (int m = 0; m < 2; ++m) {
            const EvTones tones = evMotorTones(f.motors[std::size_t(m)], f.speed, m_evSound);
            const AudioVec3 pos = f.vehiclePos + f.vehicleForward * (m == 0 ? 1.35f : -1.35f) - f.vehicleUp * 0.25f;
            const StereoGain g = place(pos);
            for (int k = 0; k < kEvTones; ++k) {
                EvOsc& o = m_ev[std::size_t(m)][std::size_t(k)];
                o.hz = o.hz <= 0.0f ? tones.hz[std::size_t(k)] : approach(o.hz, tones.hz[std::size_t(k)], 0.03f, dt);
                if (k == kEvTones - 1) o.sideHz = o.hz + m_ev[std::size_t(m)][0].hz; // carrier + 2 x electrical frequency
                const float gain = tones.gain[std::size_t(k)] * s.motor * m_evFade;
                o.tl = g.l * gain * bus;
                o.tr = g.r * gain * bus;
                power += gain * gain;
                if (k == kEvTones - 1) inverter += gain * gain;
            }
            mainHz = std::max(mainHz, m_ev[std::size_t(m)][0].hz);
        }
        m_stats.motorHz = f.electric ? mainHz : 0.0f;
        m_stats.motorLevel = std::sqrt(power);
        m_stats.inverterLevel = std::sqrt(inverter);
    }

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
    // A fresh slide starts with the short squeal; the loop is ducked under it and takes over as it decays. The short
    // squeal follows the wheel's squeal level, so it ends with the slide instead of playing its 1.7 s to the end.
    // The vehicle's TireSoundProfile sets pitch, level and brightness and the synthesized scrub (mixScrub).
    const TireSoundProfile& ts = f.tireSound;
    m_scrubHp = std::exp(-6.2831853f * ts.scrubLowHz / float(m_rate));
    m_scrubLp = 1.0f - std::exp(-6.2831853f * ts.scrubHighHz / float(m_rate));
    float skidPower = 0.0f, scrubPower = 0.0f;
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
                sv.chirpLevel = 0.32f * ts.squealGain * (0.85f + 0.3f * rand01());
                sv.chirpGain = sv.chirpLevel * std::min(1.0f, target);
                sv.chirp = startOneShot(SoundId::TireSkidShort, sv.chirpGain, Bus::Tires, &p,
                                        ts.squealPitch * sv.jitter * (0.92f + 0.16f * rand01()));
                sv.chirpSerial = sv.chirp ? sv.chirp->serial : 0;
                sv.lastChirp = m_time;
                sv.sinceChirp = 0.0;
                m_lastChirp = m_time;
                ++m_stats.skidChirps;
            }
            if (sv.chirp && (!sv.chirp->active || sv.chirp->serial != sv.chirpSerial)) sv.chirp = nullptr;
            if (sv.chirp) {
                const float chirpTarget = sv.chirpLevel * std::min(1.0f, target);
                sv.chirpGain = approach(sv.chirpGain, chirpTarget, chirpTarget > sv.chirpGain ? 0.03f : 0.04f, dt);
                sv.chirp->gain = sv.chirpGain;
                if (target == 0.0f && sv.chirpGain < 1e-4f) {
                    sv.chirp->active = false;
                    sv.chirp = nullptr;
                }
            }
            sv.prevTarget = target;
            sv.sinceChirp += dt;
            sv.wobblePhase = std::fmod(sv.wobblePhase + 6.2831853f * sv.wobbleHz * dt, 6.2831853f * 100.0f);
        }
        sv.gain = approach(sv.gain, target, target > sv.gain ? 0.03f : 0.04f, dt);
        if (sv.gain < 1e-4f && target == 0.0f) sv.gain = 0.0f;
        const float slide = smoothstep(1.0f, 12.0f, wa.slipSpeed);
        const float load = std::clamp(wa.load / std::max(f.referenceLoad, 1.0f), 0.0f, 1.6f);
        sv.voice.rate = ts.squealPitch * sv.jitter * (0.90f + 0.14f * slide) * (1.04f - 0.05f * load) *
                        (1.0f + 0.012f * std::sin(sv.wobblePhase));
        const float cutoff = ts.lowpassMinHz + (ts.lowpassMaxHz - ts.lowpassMinHz) * std::min(1.0f, sv.gain);
        sv.voice.lowpass = 1.0f - std::exp(-6.2831853f * cutoff / float(m_rate));
        const float wobble = 1.0f + 0.12f * std::sin(0.73f * sv.wobblePhase + 1.3f);
        const float duck = 0.4f + 0.6f * smoothstep(0.15f, 0.8f, float(sv.sinceChirp));
        // 0.5: four uncorrelated voices add up in power like the RMS of the four wheels.
        const StereoGain pan = place(wa.position);
        setLoop(sv.voice, 0.5f * kTireGain[0] * ts.squealGain * sv.gain * wobble * duck, pan);
        m_stats.skidWheel[std::size_t(i)] = sv.gain;
        skidPower += sv.gain * sv.gain;
        // Scrub: rises with the squeal (same gates), modulated by the tread blocks passing the patch.
        const float scrub = ts.scrubGain * sv.gain * (0.6f + 0.4f * smoothstep(1.0f, 8.0f, wa.slipSpeed));
        sv.blockHz = std::min(wa.slipSpeed / std::max(ts.treadPitch, 0.01f), 400.0f);
        const float bus = m_bus[std::size_t(Bus::Tires)] * m_master * m_gameplay;
        sv.scrubTl = pan.l * kScrubLevel * scrub * bus;
        sv.scrubTr = pan.r * kScrubLevel * scrub * bus;
        scrubPower += scrub * scrub;
    }
    m_stats.scrub = std::sqrt(0.25f * scrubPower);
    {
        // Rolling road hum (quiet vehicles): follows each wheel's rolling speed on hard ground.
        const float bus = m_bus[std::size_t(Bus::Tires)] * m_master * m_gameplay;
        float roadPower = 0.0f;
        for (int i = 0; i < kAudioWheels; ++i) {
            RoadHum& h = m_road[std::size_t(i)];
            const WheelAudio& wa = f.wheels[std::size_t(i)];
            const float target = f.vehicleValid ? ts.roadGain * roadHum(wa, f.referenceLoad) : 0.0f;
            h.gain = approach(h.gain, target, 0.08f, dt);
            h.hz = std::min(wa.rollSpeed / std::max(ts.treadPitch, 0.01f), 600.0f);
            const StereoGain pan = place(wa.position);
            h.tl = pan.l * kRoadLevel * h.gain * bus;
            h.tr = pan.r * kRoadLevel * h.gain * bus;
            roadPower += h.gain * h.gain;
        }
        m_stats.road = std::sqrt(0.25f * roadPower);
    }
    {
        // Body shake: rattle at the body, knocks at their wheels.
        if (!(f.bodySound == m_bodyProfile)) retuneBody(f.bodySound);
        const float bus = m_bus[std::size_t(Bus::Impacts)] * m_master * m_gameplay;
        const StereoGain g = place(f.vehiclePos + f.vehicleUp * 0.3f);
        const float rattle = kRattleLevel * m_bodyProfile.rattleGain * bus;
        m_rattleTl = g.l * rattle;
        m_rattleTr = g.r * rattle;
        for (Knock& k : m_knocks) {
            if (!k.active) continue;
            const StereoGain kg = place(k.position);
            k.tl = kg.l * kKnockLevel * m_bodyProfile.knockGain * bus;
            k.tr = kg.r * kKnockLevel * m_bodyProfile.knockGain * bus;
            if (!k.struck) {
                k.l = k.tl;
                k.r = k.tr;
            }
        }
        m_stats.rattle = m_shake;
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
    // The ambience bed is not driven by the simulation: it keeps playing unchanged while paused.
    setLoop(m_day, kDayGain * dn[0], {1.0f, 1.0f});
    setLoop(m_night, kNightGain * dn[1], {1.0f, 1.0f});
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
        if (v.highpass > 0.0f) {
            float* h = v.hp.data();
            for (int s = 0; s < 2; ++s) {
                float& x = s == 0 ? l : r;
                for (int stage = 0; stage < 2; ++stage) {
                    float* st = h + 4 * s + 2 * stage; // [previous input, previous output]
                    const float y = v.highpass * (st[1] + x - st[0]);
                    st[0] = x;
                    st[1] = y;
                    x = y;
                }
            }
        }
        acc[2 * i] += l * (v.gl + dl * float(i));
        acc[2 * i + 1] += r * (v.gr + dr * float(i));
        v.pos += double(v.rate);
        if (v.loop && v.pos >= double(n)) v.pos -= double(n);
    }
}

void AudioMixer::mixScrub(float* acc, int frames) {
    const float invN = 1.0f / float(frames);
    const float hp = m_scrubHp, lp = m_scrubLp;
    for (SkidVoice& sv : m_skid) {
        if (sv.scrubTl < 1e-6f && sv.scrubTr < 1e-6f && sv.scrubL < 1e-6f && sv.scrubR < 1e-6f) {
            sv.scrubL = sv.scrubR = 0.0f;
            continue;
        }
        const float inc = 6.2831853f * sv.blockHz / float(m_rate);
        for (int i = 0; i < frames; ++i) {
            // xorshift32 white noise in [-1, 1)
            sv.noise ^= sv.noise << 13;
            sv.noise ^= sv.noise >> 17;
            sv.noise ^= sv.noise << 5;
            const float x = float(sv.noise) * (2.0f / 4294967296.0f) - 1.0f;
            sv.hpOut = hp * (sv.hpOut + x - sv.hpIn);
            sv.hpIn = x;
            sv.lp1 += lp * (sv.hpOut - sv.lp1);
            sv.lp2 += lp * (sv.lp1 - sv.lp2);
            // Tread blocks: a raised-cosine pulse per block, never fully silent between blocks.
            const float block = 0.45f + 0.55f * (0.5f - 0.5f * std::cos(sv.blockPhase));
            sv.blockPhase += inc;
            if (sv.blockPhase > 6.2831853f) sv.blockPhase -= 6.2831853f;
            const float s = sv.lp2 * block;
            const float t = float(i) * invN;
            acc[2 * i] += s * (sv.scrubL + (sv.scrubTl - sv.scrubL) * t);
            acc[2 * i + 1] += s * (sv.scrubR + (sv.scrubTr - sv.scrubR) * t);
        }
        sv.scrubL = sv.scrubTl;
        sv.scrubR = sv.scrubTr;
    }
}

void AudioMixer::mixElectric(float* acc, int frames) {
    const float invN = 1.0f / float(frames);
    constexpr double kTwoPiD = 6.283185307179586;
    for (auto& motor : m_ev) {
        for (std::size_t k = 0; k < motor.size(); ++k) {
            EvOsc& o = motor[k];
            if (o.tl < 1e-7f && o.tr < 1e-7f && o.l < 1e-7f && o.r < 1e-7f) {
                o.l = o.r = 0.0f;
                continue;
            }
            const bool inverter = k == motor.size() - 1;
            const double inc = double(o.hz) / double(m_rate) * kTwoPiD;
            const double sideInc = double(o.sideHz) / double(m_rate) * kTwoPiD;
            for (int i = 0; i < frames; ++i) {
                const float t = float(i) * invN;
                float x = float(std::sin(o.phase));
                if (inverter) {
                    x = 0.7f * x + 0.5f * float(std::sin(o.sidePhase));
                    o.sidePhase += sideInc;
                    if (o.sidePhase > kTwoPiD) o.sidePhase -= kTwoPiD;
                }
                acc[2 * i] += x * (o.l + (o.tl - o.l) * t);
                acc[2 * i + 1] += x * (o.r + (o.tr - o.r) * t);
                o.phase += inc;
                if (o.phase > kTwoPiD) o.phase -= kTwoPiD;
            }
            o.l = o.tl;
            o.r = o.tr;
        }
    }
}

void AudioMixer::mixRoad(float* acc, int frames) {
    const float invN = 1.0f / float(frames);
    for (RoadHum& h : m_road) {
        if (h.tl < 1e-6f && h.tr < 1e-6f && h.l < 1e-6f && h.r < 1e-6f) {
            h.l = h.r = 0.0f;
            continue;
        }
        const float inc = 6.2831853f * h.hz / float(m_rate);
        for (int i = 0; i < frames; ++i) {
            h.noise ^= h.noise << 13;
            h.noise ^= h.noise >> 17;
            h.noise ^= h.noise << 5;
            const float x = float(h.noise) * (2.0f / 4294967296.0f) - 1.0f;
            h.hpOut = m_roadHp * (h.hpOut + x - h.hpIn);
            h.hpIn = x;
            h.lp1 += m_roadLp * (h.hpOut - h.lp1);
            h.lp2 += m_roadLp * (h.lp1 - h.lp2);
            // Tread blocks rolling through the patch: a shallow modulation (rolling, not sliding).
            const float block = 0.75f + 0.25f * std::sin(h.phase);
            h.phase += inc;
            if (h.phase > 6.2831853f) h.phase -= 6.2831853f;
            const float y = h.lp2 * block;
            const float t = float(i) * invN;
            acc[2 * i] += y * (h.l + (h.tl - h.l) * t);
            acc[2 * i + 1] += y * (h.r + (h.tr - h.r) * t);
        }
        h.l = h.tl;
        h.r = h.tr;
    }
}

void AudioMixer::Resonator::tune(float hz, float decay, float rate) {
    const float w = 6.2831853f * std::min(hz, 0.45f * rate) / rate;
    const float r = std::exp(-1.0f / (std::max(decay, 1e-4f) * rate));
    c = 2.0f * r * std::cos(w);
    rr = r * r;
    s = std::sin(w);
}

void AudioMixer::retuneBody(const BodySoundProfile& p) {
    m_bodyProfile = p;
    const float rate = float(m_rate);
    for (std::size_t j = 0; j < m_rattleModes.size(); ++j)
        m_rattleModes[j].tune(p.rattleHz[j], p.rattleRing * (1.2f - 0.15f * float(j)), rate);
    m_clickDecay = std::exp(-1.0f / (0.0015f * rate));
    m_bodyTuned = true;
}

void AudioMixer::shake(float level, float decay) {
    if (level >= m_shake) {
        m_shake = std::min(level, 1.0f);
        m_shakeTau = std::max(decay, 0.02f);
    }
}

void AudioMixer::startKnock(float intensity, const AudioVec3& position) {
    if (!m_bodyTuned) retuneBody(m_bodyProfile);
    Knock* slot = &m_knocks[0];
    for (Knock& k : m_knocks) {
        if (!k.active) {
            slot = &k;
            break;
        }
        if (k.age > slot->age) slot = &k;
    }
    Knock& k = *slot;
    const float rate = float(m_rate);
    const float pitch = 0.95f + 0.1f * rand01();
    for (std::size_t m = 0; m < k.modes.size(); ++m) {
        k.modes[m] = Resonator{};
        k.modes[m].tune(m_bodyProfile.knockHz[m] * pitch, m_bodyProfile.knockDecay * kKnockModeDecay[m], rate);
    }
    k.amp = 0.3f + 0.7f * std::clamp(intensity, 0.0f, 1.0f);
    k.noise = 0.5f * k.amp;
    k.age = 0.0f;
    k.life = 7.0f * m_bodyProfile.knockDecay;
    k.active = true;
    k.struck = false;
    k.position = position;
    // Start at the target gain (set by the next control update; the gains ramp from zero within that block).
    k.l = k.r = k.tl = k.tr = 0.0f;
    ++m_stats.knocks;
}

void AudioMixer::mixBodyShake(float* acc, int frames) {
    const float rate = float(m_rate);
    const float invN = 1.0f / float(frames);
    auto noise = [this] {
        m_shakeNoise ^= m_shakeNoise << 13;
        m_shakeNoise ^= m_shakeNoise >> 17;
        m_shakeNoise ^= m_shakeNoise << 5;
        return float(m_shakeNoise) * (2.0f / 4294967296.0f) - 1.0f;
    };
    // Rattle: grains while the envelope is up; the resonators ring out after it.
    float energy = m_click;
    for (const Resonator& r : m_rattleModes) energy += std::abs(r.y1) + std::abs(r.y2);
    const bool ringing = energy > 1e-6f;
    if (m_shake > 0.0f || ringing) {
        const float decay = std::exp(-1.0f / (m_shakeTau * rate));
        const float grainChance = m_bodyProfile.rattleRate / rate;
        for (int i = 0; i < frames; ++i) {
            std::array<float, 4> kick{};
            if (m_shake > kShakeGate && rand01() < grainChance * m_shake) {
                const std::size_t j = std::min<std::size_t>(3, std::size_t(rand01() * 4.0f));
                const float amp = m_shake * (0.35f + 0.65f * rand01());
                kick[j] = rand01() < 0.5f ? amp : -amp;
                m_click = std::max(m_click, amp * m_bodyProfile.clickGain);
            }
            float y = noise() * m_click;
            for (std::size_t j = 0; j < 4; ++j) y += m_rattleModes[j].step(kick[j]);
            m_click *= m_clickDecay;
            m_shake *= decay;
            const float t = float(i) * invN;
            acc[2 * i] += y * (m_rattleL + (m_rattleTl - m_rattleL) * t);
            acc[2 * i + 1] += y * (m_rattleR + (m_rattleTr - m_rattleR) * t);
        }
        if (m_shake <= kShakeGate) m_shake = 0.0f;
        if (!ringing && m_shake == 0.0f)
            for (Resonator& r : m_rattleModes) r.y1 = r.y2 = 0.0f;
    }
    m_rattleL = m_rattleTl;
    m_rattleR = m_rattleTr;

    const float noiseDecay = std::exp(-1.0f / (0.003f * rate));
    for (Knock& k : m_knocks) {
        if (!k.active) continue;
        for (int i = 0; i < frames; ++i) {
            const float x = k.struck ? 0.0f : k.amp;
            k.struck = true;
            float y = noise() * k.noise;
            for (std::size_t m = 0; m < k.modes.size(); ++m) y += k.modes[m].step(x * kKnockModeAmp[m]);
            k.noise *= noiseDecay;
            const float t = float(i) * invN;
            acc[2 * i] += y * (k.l + (k.tl - k.l) * t);
            acc[2 * i + 1] += y * (k.r + (k.tr - k.r) * t);
        }
        k.l = k.tl;
        k.r = k.tr;
        k.age += float(frames) / rate;
        if (k.age > k.life) k.active = false;
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
    mixMusic(acc, frames, dt);
    if (m_musicOut > 1e-5f) ++voices;
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
    if (!frozen) {
        mixScrub(acc, frames);
        mixElectric(acc, frames);
        mixRoad(acc, frames);
        mixBodyShake(acc, frames);
    }

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

void AudioMixer::mixMusic(float* acc, int frames, float dt) {
    const AudioFrame& f = m_frame;
    if (f.musicRestart != m_musicRestart) {
        m_musicRestart = f.musicRestart;
        if (m_music) m_music->rewind();
        m_musicFrames = 0;
    }
    const float target = std::clamp(f.music, 0.0f, 1.0f);
    const float rate = dt / std::max(f.musicFadeSeconds, 0.01f);
    m_musicGain = m_musicGain < target ? std::min(target, m_musicGain + rate) : std::max(target, m_musicGain - rate);
    m_stats.musicGain = m_musicGain;
    const float out = m_musicGain * m_bus[std::size_t(Bus::Music)] * m_master;
    if (!m_music || (out <= 0.0f && m_musicOut <= 0.0f)) {
        // Silent: the track holds its position (no decoding).
        m_musicOut = 0.0f;
        return;
    }
    float* buf = m_musicBuf.data();
    int got = 0;
    for (int tries = 0; got < frames && tries < 2; ++tries) {
        const int n = m_music->read(buf + 2 * got, frames - got);
        got += std::max(0, n);
        if (got < frames) m_music->rewind(); // loop
    }
    std::fill(buf + 2 * got, buf + 2 * frames, 0.0f);
    m_musicFrames += frames;
    m_stats.musicSeconds = double(m_musicFrames) / double(m_rate);
    const float invN = 1.0f / float(frames);
    for (int i = 0; i < frames; ++i) {
        const float g = m_musicOut + (out - m_musicOut) * float(i + 1) * invN;
        acc[2 * i] += buf[2 * i] * g;
        acc[2 * i + 1] += buf[2 * i + 1] * g;
    }
    m_musicOut = out;
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
