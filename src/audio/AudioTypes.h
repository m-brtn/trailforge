#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>

#include "vehicle/BodySound.h"
#include "vehicle/TireSound.h"

namespace tf {

inline constexpr int kAudioWheels = 4;

struct AudioVec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;
};
inline AudioVec3 operator+(AudioVec3 a, AudioVec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline AudioVec3 operator-(AudioVec3 a, AudioVec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline AudioVec3 operator*(AudioVec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(AudioVec3 a, AudioVec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float length(AudioVec3 a) { return std::sqrt(dot(a, a)); }

// User-facing mix settings (Audio panel). Volumes are linear gains.
struct AudioSettings {
    float master = 0.8f;
    float engine = 1.0f;
    float tires = 1.0f;
    float impacts = 1.0f;
    float ambience = 1.0f;
    float ui = 1.0f;
    float music = 0.8f;
    float effects = 1.0f;        // Settings screen: scales engine, tires, impacts and ambience together
    bool mute = false;
    float enginePitch = 0.80f;   // source rpm = engine rpm x this (the recordings are a petrol inline-4)
    float levelSpreadDb = 12.0f; // engine: spread of the recorded loop levels after compression
    float whine = 1.0f;          // synthesized transmission whine (0 = off)
    float motor = 1.0f;          // synthesized electric motor, inverter and reduction gear sound (0 = off)
};

// One electric motor as the audio side sees it (EvTelemetry of the latest tick).
struct MotorAudio {
    float rpm = 0.0f;        // shaft rpm, signed
    float torque = 0.0f;     // N*m at the shaft, + drive, - regeneration
    float maxTorque = 1.0f;  // N*m, rated
};

// One wheel as the audio side sees it (from WheelContactSample of the latest physics tick).
struct WheelAudio {
    bool contact = false;
    std::uint8_t surface = 1;   // SurfaceId
    bool inWater = false;       // contact patch below the puddle water level
    float load = 0.0f;          // N
    float slipSpeed = 0.0f;     // m/s, patch sliding speed over the ground
    float slipRatio = 0.0f;     // sliding speed / max(patch speed, rolling speed, 1 m/s)
    float longRatio = 0.0f;     // |longitudinal sliding speed| / the same reference (wheelspin, lock-up)
    float latRatio = 0.0f;      // |lateral sliding speed| / the same reference (scrub, drift)
    float rollSpeed = 0.0f;     // m/s, |patch speed over the ground|
    float soil = 0.0f;          // 0..1, soil in the tread (TreadSoil): a dirty tread does not squeal
    float squeal = 0.0f;        // 0..~1.2, rubber squeal of this wheel: smoothed, gated per tick in VehicleAudioFeed
    AudioVec3 position;         // world, contact patch
};

// Continuous game state for the audio thread; published once per frame from the latest physics tick.
struct AudioFrame {
    AudioVec3 listenerPos;
    AudioVec3 listenerRight{1, 0, 0};
    AudioVec3 listenerForward{0, 0, 1};
    AudioVec3 vehiclePos;
    AudioVec3 vehicleForward{0, 0, 1}; // body +Z
    AudioVec3 vehicleLeft{1, 0, 0};    // body +X
    AudioVec3 vehicleUp{0, 1, 0};
    float speed = 0.0f;        // m/s
    float rpm = 0.0f;          // engine rpm
    float throttle = 0.0f;     // 0..1, applied on the last tick
    float clutch = 1.0f;       // 0..1
    int gear = 0;
    float driveshaftHz = 0.0f; // propeller shaft revolutions per second (|mean wheel speed| x final drive)
    float referenceLoad = 4905.0f; // N, static load of one wheel
    std::array<WheelAudio, kAudioWheels> wheels;
    TireSoundProfile tireSound;  // the vehicle's tire sound (VehicleConfig::tireSound); default = "sporty"
    BodySoundProfile bodySound;  // the vehicle's body rattle and knock (VehicleConfig::bodySound); default = "suv"
    // Electric drive: the engine loops, starter and gear shift sounds stay silent; the motors are synthesized.
    bool electric = false;
    std::array<MotorAudio, 2> motors{}; // front, rear
    float dayAmount = 1.0f;    // 0 = night, 1 = day (from the sun elevation)
    bool paused = false;
    bool menuAmbience = false; // menu drone loop (loaded, not used by the menus: they keep the world sound)
    float music = 0.0f;        // music target level 0..1; the mixer ramps toward it
    float musicFadeSeconds = 1.5f; // duration of a full 0 <-> 1 ramp
    int musicRestart = 0;      // a changed value rewinds the track before it plays again
    bool vehicleValid = false; // false until the first tick was seen
    AudioSettings settings;
};

enum class UiSound : std::uint8_t { Click, Hover, Confirm, Back, SliderTick, MenuOpen, MenuClose };

enum class AudioEventType : std::uint8_t {
    EngineStart,     // spawn / reset
    GearShift,       // a gear was engaged
    BumpStop,        // suspension hit its compression limit fast (knock at that wheel)
    BodyRattle,      // hard vertical jolt of the body (landing, big bump): the whole vehicle shakes
    BodyImpact,      // body collision with an estimated impulse above the threshold
    Splash,          // a wheel entered a puddle
    ChassisCreak,    // large articulation
    Ui,              // UI sound (`ui`)
};

struct AudioEvent {
    AudioEventType type = AudioEventType::Ui;
    float intensity = 1.0f; // 0..1
    AudioVec3 position;     // world
    UiSound ui = UiSound::Click;
};

// Read-outs written by the audio thread (Audio panel, scenario values).
struct AudioStats {
    bool running = false;          // device or capture active
    float rpm = 0.0f;              // simulated engine rpm received with the last frame
    float sourceRpm = 0.0f;        // smoothed rpm x pitch factor: the engine speed the loops are played at
    float load = 0.0f;             // smoothed engine load used for the on/off blend
    std::array<float, 3> offWeights{}; // equal-power weights: idle/mid/high (off-load)
    std::array<float, 3> onWeights{};  // low/mid/high (full load)
    std::array<float, 3> exhaustRates{}; // playback rate of the off-load exhaust layers
    float skid = 0.0f, gravel = 0.0f, dirt = 0.0f, mud = 0.0f, grass = 0.0f; // tire layer intensities (0..~1)
    std::array<float, kAudioWheels> skidWheel{}; // per-wheel squeal voice gains (skid = RMS of these)
    int skidChirps = 0;            // short squeal one-shots started at slide onsets
    float scrub = 0.0f;            // synthesized tire scrub level (RMS over the wheels, before the volume settings)
    float wind = 0.0f;
    float day = 0.0f, night = 0.0f; // ambience weights
    float whineHz = 0.0f;
    float motorHz = 0.0f;          // electric: main electromagnetic order of the faster motor (smoothed)
    float motorLevel = 0.0f;       // electric: RMS of all synthesized motor / gear / inverter tone gains
    float inverterLevel = 0.0f;    // electric: inverter tone gain (both motors)
    float road = 0.0f;             // rolling road hum level (RMS over the wheels)
    float rattle = 0.0f;           // body shake envelope driving the synthesized rattle (0..1)
    int knocks = 0;                // synthesized suspension knocks started
    float gameplayGain = 1.0f;     // 0 while paused (gameplay sounds are paused)
    float musicGain = 0.0f;        // current music level (before the volume settings)
    double musicSeconds = 0.0;     // music played since the last restart (s, counts across loops)
    int activeVoices = 0;
    int oneShotsPlayed = 0;
    std::array<int, 8> eventsByType{}; // AudioEventType counts since start
    float peakDb = -120.0f;        // master output peak over the last ~0.5 s, dBFS
    float limiterDb = 0.0f;        // current limiter gain reduction, dB (<= 0)
    long long clippedSamples = 0;  // samples hard-clamped after the limiter (should stay 0)
    double callbackUs = 0.0;       // average audio render time per callback, microseconds
    double callbackMaxUs = 0.0;    // max over the last ~2 s
    double cpuLoad = 0.0;          // render time / buffer duration (moving average), 0..1
    long long framesRendered = 0;
    int eventsDropped = 0;
};

// Game thread -> audio thread hand-off. The game publishes once per frame (short lock: a struct copy);
// the audio thread only try_locks, so it never waits on the game, and keeps its previous frame if the lock is
// busy. Events accumulate until the audio thread takes them.
class AudioMailbox {
public:
    static constexpr int kMaxEvents = 128;

    void publish(const AudioFrame& frame, const AudioEvent* events, int count) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_frame = frame;
        m_hasFrame = true;
        for (int i = 0; i < count; ++i) {
            if (m_eventCount < kMaxEvents) m_events[std::size_t(m_eventCount++)] = events[i];
            else ++m_dropped;
        }
    }
    // Audio thread. Returns false (and leaves the outputs alone) if the game holds the lock right now.
    bool tryTake(AudioFrame& frame, std::array<AudioEvent, kMaxEvents>& events, int& count, int& dropped) {
        std::unique_lock<std::mutex> lock(m_mutex, std::try_to_lock);
        if (!lock.owns_lock()) return false;
        if (m_hasFrame) frame = m_frame;
        for (int i = 0; i < m_eventCount; ++i) events[std::size_t(i)] = m_events[std::size_t(i)];
        count = m_eventCount;
        dropped = m_dropped;
        m_eventCount = 0;
        return m_hasFrame;
    }
    // Audio thread writes stats with try_lock; the UI reads them with a short lock.
    void tryPutStats(const AudioStats& s) {
        std::unique_lock<std::mutex> lock(m_statsMutex, std::try_to_lock);
        if (lock.owns_lock()) m_stats = s;
    }
    AudioStats stats() const {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        return m_stats;
    }

private:
    std::mutex m_mutex;
    AudioFrame m_frame;
    bool m_hasFrame = false;
    std::array<AudioEvent, kMaxEvents> m_events{};
    int m_eventCount = 0;
    int m_dropped = 0;
    mutable std::mutex m_statsMutex;
    AudioStats m_stats;
};

} // namespace tf
