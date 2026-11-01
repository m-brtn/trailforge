#pragma once

#include "audio/AudioManifest.h"
#include "audio/AudioMapping.h"
#include "audio/AudioTypes.h"

#include <array>
#include <cstdint>
#include <vector>

namespace tf {

// Decoded PCM of one sound: interleaved float32 at the mixer sample rate.
struct SoundData {
    std::vector<float> pcm;
    int channels = 1;
    std::size_t frames = 0;
    bool loop = false;
};

struct SoundBank {
    AudioManifest manifest;
    std::array<SoundData, kSoundCount> sounds;
    const SoundData& operator[](SoundId s) const { return sounds[std::size_t(s)]; }
};

// Streamed music (the app decodes the MP3; tests use synthetic sources). Called only from the thread that renders.
class MusicSource {
public:
    virtual ~MusicSource() = default;
    // Interleaved stereo float at the mixer rate. Returns the frames written; fewer than asked at the end of the track.
    virtual int read(float* stereo, int frames) = 0;
    virtual void rewind() = 0;
    virtual int sampleRate() const = 0;
};

// The game's sound model and mixer. Plain C++ (no device): the audio thread calls setFrame/trigger with what the
// mailbox delivered and then render(); the offline capture calls the same functions on the main thread.
// All state is owned here; nothing allocates after construction.
//
// Voices: 12 engine loops (3 rpm bands x off/full load x exhaust/engine-bay mic), 4 loose-ground tire loops, one
// squeal voice per wheel, wind, day and night ambience, the menu drone, a synthesized transmission whine, a pool of
// one-shots and a streamed music track (looped; not paused with the game). An electric vehicle (AudioFrame::electric)
// silences the engine loops and plays synthesized motor tones instead (evMotorTones); a tire profile with roadGain
// adds a synthesized rolling road hum per wheel. The body rattle and the suspension / body thuds are synthesized
// from the vehicle's BodySoundProfile (mixBodyShake). Gains ramp
// linearly over each 64-frame control block; a look-ahead peak limiter guards the master output.
class AudioMixer {
public:
    explicit AudioMixer(const SoundBank& bank, int sampleRate = 48000, std::uint32_t seed = 0x7f4a7c15u);

    void setFrame(const AudioFrame& frame) { m_frame = frame; }
    void trigger(const AudioEvent& e);
    void playUi(UiSound s);
    // Before rendering starts; nullptr = no music. The source must outlive the mixer's use of it.
    void setMusic(MusicSource* source) { m_music = source; }
    // Interleaved stereo float output.
    void render(float* stereo, int frames);

    const AudioStats& stats() const { return m_stats; }
    int sampleRate() const { return m_rate; }

    static constexpr int kBlock = 64;
    static constexpr int kOneShots = 32;

private:
    enum class Bus : std::uint8_t { Engine, Tires, Impacts, Ambience, Ui, Music, Count };

    struct Voice {
        const SoundData* data = nullptr;
        double pos = 0.0;      // frames
        float rate = 1.0f;
        float gl = 0.0f, gr = 0.0f; // gains at the start of the block
        float tl = 0.0f, tr = 0.0f; // gains at the end of the block
        Bus bus = Bus::Engine;
        bool loop = false;
        bool active = false;
        bool gameplay = true;  // paused with the game
        // One-shots
        bool positional = true;
        AudioVec3 position;
        float gain = 1.0f;
        int delay = 0;         // frames before the sound starts
        std::uint32_t serial = 0; // one-shots: start count, tells a tracked shot from a later reuse of its slot
        float lowpass = 0.0f;  // one-pole low-pass coefficient per sample (0 = off)
        float lpL = 0.0f, lpR = 0.0f;
        float highpass = 0.0f; // two cascaded one-pole high-passes, pole per sample (0 = off)
        std::array<float, 8> hp{}; // [in, out] x 2 stages x 2 channels
        // One-shots: after fadeHold s the gain decays with time constant fade (0 = plays as recorded).
        float fadeHold = 0.0f, fade = 0.0f, age = 0.0f;
    };

    // Rubber squeal of one wheel: its own voice of the skid loop with per-slide variation.
    struct SkidVoice {
        Voice voice;
        float gain = 0.0f;          // smoothed squeal
        float prevTarget = 0.0f;
        float jitter = 1.0f;        // pitch factor drawn at every slide onset
        float wobblePhase = 0.0f, wobbleHz = 3.0f;
        double lastChirp = -1e9;    // s
        double sinceChirp = 1e9;    // s, the loop is ducked under a fresh chirp
        // The onset chirp follows this wheel's squeal and fades out with it instead of playing to its end.
        Voice* chirp = nullptr;
        std::uint32_t chirpSerial = 0;
        float chirpLevel = 0.0f;    // chirp gain at full squeal
        float chirpGain = 0.0f;     // smoothed
        // Synthesized scrub (TireSoundProfile::scrubGain > 0): band-limited noise, amplitude-modulated at the
        // tread block rate; gains ramp across a block like the whine.
        std::uint32_t noise = 1;
        float hpIn = 0.0f, hpOut = 0.0f, lp1 = 0.0f, lp2 = 0.0f;
        float blockPhase = 0.0f, blockHz = 0.0f;
        float scrubL = 0.0f, scrubR = 0.0f, scrubTl = 0.0f, scrubTr = 0.0f;
    };

    void controlUpdate(float dt);
    void renderBlock(float* stereo, int frames);
    void mixVoice(Voice& v, float* acc, int frames);
    Voice* startOneShot(SoundId id, float gain, Bus bus, const AudioVec3* position, float rate = 1.0f);
    StereoGain place(const AudioVec3& p) const;
    void setLoop(Voice& v, float gain, const StereoGain& g);
    void recomputeEngineLevels(float spreadDb);
    void mixMusic(float* acc, int frames, float dt);
    float rand01();
    SoundId pick(SoundId first, int count);

    const SoundBank& m_bank;
    int m_rate;
    std::uint32_t m_rng;
    AudioFrame m_frame;
    AudioStats m_stats;

    std::array<Voice, 12> m_engine{};        // [mic * 6 + load * 3 + band]
    std::array<float, 12> m_engineLevel{};   // compressed recorded level (linear)
    std::array<float, 12> m_engineRpm{};     // measured rpm per loop
    float m_levelSpread = -1.0f;
    std::array<Voice, kTireLayerCount> m_tires{}; // loose-ground layers; [Skid] is unused (see m_skid)
    std::array<SkidVoice, kAudioWheels> m_skid{};
    Voice m_wind, m_day, m_night, m_menu;
    std::array<Voice, kOneShots> m_shots{};

    // Smoothed control state
    float m_rpm = 0.0f, m_load = 0.0f, m_pitch = 0.8f;
    std::array<float, kTireLayerCount> m_tireGain{};
    float m_windGain = 0.0f, m_dayAmount = 1.0f, m_gameplay = 1.0f, m_master = 0.0f;
    std::array<float, int(Bus::Count)> m_bus{};
    float m_engineFade = 0.0f;   // 0 before the first start, ramps in after the starter catches
    double m_startElapsed = 1e9; // s since the last EngineStart
    Voice* m_starter = nullptr;  // the playing engine_start one-shot (crossfaded against the loops)
    bool m_starterHandover = false;
    double m_time = 0.0;         // rendered time, s
    double m_lastChirp = -1e9;
    std::uint32_t m_shotSerial = 0;
    float m_scrubHp = 0.0f, m_scrubLp = 0.0f; // scrub filter coefficients per sample
    void mixScrub(float* acc, int frames);

    // Electric drive: one sine oscillator per motor and tone (the inverter tone carries its moving sideband); gains
    // ramp across a block like the whine.
    struct EvOsc {
        double phase = 0.0, sidePhase = 0.0;
        float hz = 0.0f, sideHz = 0.0f;
        float l = 0.0f, r = 0.0f, tl = 0.0f, tr = 0.0f;
    };
    std::array<std::array<EvOsc, kEvTones>, 2> m_ev{};
    float m_evFade = 0.0f;
    EvSoundSettings m_evSound;
    void mixElectric(float* acc, int frames);
    // Rolling road hum per wheel (TireSoundProfile::roadGain): band noise, amplitude-modulated at the tread rate.
    struct RoadHum {
        std::uint32_t noise = 1;
        float hpIn = 0.0f, hpOut = 0.0f, lp1 = 0.0f, lp2 = 0.0f;
        float phase = 0.0f, hz = 0.0f, gain = 0.0f;
        float l = 0.0f, r = 0.0f, tl = 0.0f, tr = 0.0f;
    };
    std::array<RoadHum, kAudioWheels> m_road{};
    float m_roadHp = 0.0f, m_roadLp = 0.0f;
    void mixRoad(float* acc, int frames);

    // Body shake (synthesized from the vehicle's BodySoundProfile). Resonator: y = c*y1 - rr*y2 + s*x, a sine of
    // amplitude x decaying with the mode's time constant per unit impulse.
    struct Resonator {
        float c = 0.0f, rr = 0.0f, s = 0.0f, y1 = 0.0f, y2 = 0.0f;
        void tune(float hz, float decay, float rate);
        float step(float x) {
            const float y = c * y1 - rr * y2 + s * x;
            y2 = y1;
            y1 = y;
            return y;
        }
    };
    // Low-Q band-pass (trapezoidal state-variable filter); retuned per rattle tick, state kept.
    struct BandPass {
        float a1 = 0.0f, a2 = 0.0f, a3 = 0.0f, ic1 = 0.0f, ic2 = 0.0f;
        void tune(float hz, float q, float rate);
        float step(float x) {
            const float v3 = x - ic2;
            const float v1 = a1 * ic1 + a2 * v3;
            const float v2 = ic2 + a2 * ic1 + a3 * v3;
            ic1 = 2.0f * v1 - ic1;
            ic2 = 2.0f * v2 - ic2;
            return v1;
        }
    };
    // Suspension / body thud: one low mode that dies within a few periods and a low-passed noise hit whose
    // brightness follows the intensity, struck once.
    struct Knock {
        Resonator mode;
        float amp = 0.0f;           // strike amplitude, for the impulse on the first sample
        float noise = 0.0f, noiseDecay = 0.0f; // noise hit envelope and its per-sample decay
        float lp = 0.0f, lp1 = 0.0f, lp2 = 0.0f; // two-stage low-pass of the noise hit (coefficient, states)
        float age = 0.0f, life = 0.0f; // s
        bool active = false, struck = false;
        AudioVec3 position;
        float l = 0.0f, r = 0.0f, tl = 0.0f, tr = 0.0f;
    };
    std::array<Knock, 3> m_knocks{};
    double m_lastKnockTime = -1e9;
    float m_lastKnockAmp = 0.0f;
    // Rattle: sparse noise ticks, each band-passed around its own random frequency (no fixed resonances), rate and
    // amplitude from the shake envelope (one source at the body).
    BandPass m_tickFilter;
    float m_tick = 0.0f, m_tickDecay = 0.0f; // tick envelope and its per-sample decay
    float m_sinceTick = 1e9f;   // s
    BodySoundProfile m_bodyProfile{};
    bool m_bodyTuned = false;
    float m_shake = 0.0f, m_shakeTau = 0.3f; // envelope 0..1, its current decay time constant (s)
    float m_rattleL = 0.0f, m_rattleR = 0.0f, m_rattleTl = 0.0f, m_rattleTr = 0.0f;
    std::uint32_t m_shakeNoise = 0x2545F491u; // body synthesis random numbers (separate from rand01)
    float shakeRand01();
    float shakeNoise() { return 2.0f * shakeRand01() - 1.0f; }
    void retuneBody(const BodySoundProfile& p);
    void startKnock(float intensity, const AudioVec3& position);
    void shake(float level, float decay);
    void mixBodyShake(float* acc, int frames);
    // Recorded body slams and metal impacts: a hit within kShotGap of the last one of its kind replaces nothing
    // unless it is clearly harder.
    double m_lastSlam = -1e9, m_lastImpactShot = -1e9;
    float m_lastSlamGain = 0.0f, m_lastImpactGain = 0.0f;

    // Transmission whine (synthesized)
    double m_whinePhase = 0.0;
    float m_whineHz = 0.0f, m_whineGain = 0.0f, m_whineL = 0.0f, m_whineR = 0.0f, m_whineTl = 0.0f, m_whineTr = 0.0f;

    // Master limiter: look-ahead delay line and peak window.
    static constexpr int kLookahead = 96; // 2 ms
    std::array<float, kLookahead * 2> m_delay{};
    std::array<float, kLookahead> m_peaks{};
    int m_delayPos = 0;
    float m_limGain = 1.0f;
    float m_peakHold = 0.0f;
    double m_peakTimer = 0.0;
    float m_dcInL = 0.0f, m_dcInR = 0.0f, m_dcOutL = 0.0f, m_dcOutR = 0.0f;

    std::vector<float> m_acc; // block accumulator (stereo)

    // Music
    MusicSource* m_music = nullptr;
    std::vector<float> m_musicBuf;
    float m_musicGain = 0.0f; // ramp state, 0..1
    float m_musicOut = 0.0f;  // gain applied at the end of the previous block (with bus and master)
    int m_musicRestart = 0;
    long long m_musicFrames = 0;
};

} // namespace tf
