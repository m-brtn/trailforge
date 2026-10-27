#pragma once

#include "audio/AudioTypes.h"
#include "sim/TireMarks.h"

#include <Jolt/Jolt.h>

#include <array>
#include <vector>

namespace tf {

class Simulation;

// Thresholds that turn per-tick physics data into one-shot sound events.
struct AudioFeedSettings {
    // Bump-stop knock: the suspension reaches its compression limit while compressing at least this fast.
    float bumpStopMinSpeed = 1.2f;   // m/s, suspension compression speed on the tick it hits the bump stop
    float bumpStopFullSpeed = 3.5f;  // m/s, intensity 1
    float bumpStopInterval = 0.4f;   // s, per wheel
    float bumpStopGap = 0.1f;        // s, between knocks of different wheels (four wheels landing = one or two knocks)
    // Body rattle: felt vertical acceleration of the body (along its up axis, in g; 1 at rest, 0 in free fall),
    // low-passed so single-tick solver spikes do not count. An event fires when it rises through rattleOnG and
    // re-arms below rattleRearmG.
    float rattleSmoothing = 0.015f;  // s
    float rattleOnG = 2.3f;          // g
    float rattleFullG = 6.0f;        // g, intensity 1
    float rattleRearmG = 1.6f;       // g
    float rattleInterval = 0.25f;    // s
    float rattlePeakWait = 0.05f;    // s, longest wait for the peak of a jolt before the event fires
    float impactMinImpulse = 1000.0f; // N*s: estimated body impact impulse threshold (2000 kg at 0.5 m/s)
    float impactFullImpulse = 8000.0f;
    float splashMinSpeed = 1.0f;     // m/s
    float creakArticulation = 0.75f; // |(FL - FR) - (RL - RR)| of the compression ratios
    float impactInterval = 0.12f;    // s
    float creakInterval = 2.5f;      // s
    // Rubber squeal per wheel: the drive (wheelSquealDrive) is followed with attack/release time constants and
    // gated with hysteresis, so single-tick slip spikes stay silent. The release ends a full squeal within ~0.1 s
    // of the slide ending.
    float squealAttack = 0.03f;      // s
    float squealRelease = 0.04f;     // s, still on a hard surface
    float squealReleaseLoose = 0.03f; // s, after the wheel left the hard surface or lost contact
    float squealOn = 0.15f;          // gate opens above this smoothed drive
    float squealOff = 0.05f;         // and closes below this one
};

// Runs on the game thread after every physics tick: keeps the latest continuous state (AudioFrame vehicle part)
// and detects edges (gear engaged, fast bump-stop hits, hard body jolts, body impacts, entering a puddle,
// articulation) that
// would be missed if the audio thread only saw one state per frame.
class VehicleAudioFeed {
public:
    explicit VehicleAudioFeed(const AudioFeedSettings& settings = {}) : m_settings(settings) {}

    void onTick(const Simulation& sim);
    void push(const AudioEvent& e) { m_events.push_back(e); }

    // Vehicle part of the frame (listener, settings, time of day and pause are filled by the caller).
    const AudioFrame& frame() const { return m_frame; }
    const std::vector<AudioEvent>& events() const { return m_events; }
    void clearEvents() { m_events.clear(); }
    AudioFeedSettings& settings() { return m_settings; }

private:
    AudioFeedSettings m_settings;
    AudioFrame m_frame;
    std::vector<AudioEvent> m_events;
    double m_time = 0.0;
    bool m_seen = false;
    unsigned m_spawnCount = 0;
    int m_gear = 0;
    std::array<float, kAudioWheels> m_prevLength{};
    std::array<bool, kAudioWheels> m_prevBumpStop{};
    std::array<bool, kAudioWheels> m_prevWater{};
    std::array<double, kAudioWheels> m_lastBump{}, m_lastSplash{};
    double m_lastImpact = -1e9, m_lastCreak = -1e9, m_lastShift = -1e9, m_lastKnock = -1e9, m_lastRattle = -1e9;
    JPH::Vec3 m_prevVelocity = JPH::Vec3::sZero();
    float m_feltG = 1.0f;     // smoothed felt vertical acceleration, g
    bool m_rattleArmed = true, m_rattlePending = false;
    double m_rattleStart = 0.0;
    float m_rattlePeak = 0.0f;
    float m_prevArticulation = 0.0f;
    std::array<TreadSoil, kAudioWheels> m_soil{};
    std::array<float, kAudioWheels> m_squealEnv{};
    std::array<bool, kAudioWheels> m_squealOpen{};
};

} // namespace tf
