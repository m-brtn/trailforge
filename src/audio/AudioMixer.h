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

// The game's sound model and mixer. Plain C++ (no device): the audio thread calls setFrame/trigger with what the
// mailbox delivered and then render(); the offline capture calls the same functions on the main thread.
// All state is owned here; nothing allocates after construction.
//
// Voices: 12 engine loops (3 rpm bands x off/full load x exhaust/engine-bay mic), 4 loose-ground tire loops, one
// squeal voice per wheel, wind, day and night ambience, the menu drone, a synthesized transmission whine, and a
// pool of one-shots. Gains ramp
// linearly over each 64-frame control block; a look-ahead peak limiter guards the master output.
class AudioMixer {
public:
    explicit AudioMixer(const SoundBank& bank, int sampleRate = 48000, std::uint32_t seed = 0x7f4a7c15u);

    void setFrame(const AudioFrame& frame) { m_frame = frame; }
    void trigger(const AudioEvent& e);
    void playUi(UiSound s);
    // Interleaved stereo float output.
    void render(float* stereo, int frames);

    const AudioStats& stats() const { return m_stats; }
    int sampleRate() const { return m_rate; }

    static constexpr int kBlock = 64;
    static constexpr int kOneShots = 32;

private:
    enum class Bus : std::uint8_t { Engine, Tires, Impacts, Ambience, Ui, Count };

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
        float lowpass = 0.0f;  // one-pole low-pass coefficient per sample (0 = off)
        float lpL = 0.0f, lpR = 0.0f;
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
    };

    void controlUpdate(float dt);
    void renderBlock(float* stereo, int frames);
    void mixVoice(Voice& v, float* acc, int frames);
    Voice* startOneShot(SoundId id, float gain, Bus bus, const AudioVec3* position, float rate = 1.0f);
    StereoGain place(const AudioVec3& p) const;
    void setLoop(Voice& v, float gain, const StereoGain& g);
    void recomputeEngineLevels(float spreadDb);
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
};

} // namespace tf
