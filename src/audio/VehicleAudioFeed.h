#pragma once

#include "audio/AudioTypes.h"

#include <array>
#include <vector>

namespace tf {

class Simulation;

// Thresholds that turn per-tick physics data into one-shot sound events.
struct AudioFeedSettings {
    float thumpLoadStep = 9000.0f;   // N: suspension load increase within one tick that counts as a thump
    float thumpFullStep = 40000.0f;  // N: load step for intensity 1
    float bumpStopFullLoad = 80000.0f; // N: bump-stop load for intensity 1
    float impactMinImpulse = 600.0f; // N*s: estimated body impact impulse threshold (2000 kg at 0.3 m/s)
    float impactFullImpulse = 8000.0f;
    float splashMinSpeed = 1.0f;     // m/s
    float creakArticulation = 0.75f; // |(FL - FR) - (RL - RR)| of the compression ratios
    float wheelEventInterval = 0.15f; // s, per wheel and event type
    float impactInterval = 0.08f;    // s
    float creakInterval = 2.5f;      // s
};

// Runs on the game thread after every physics tick: keeps the latest continuous state (AudioFrame vehicle part)
// and detects edges (gear engaged, bump stop, load steps, body impacts, entering a puddle, articulation) that
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
    std::array<float, kAudioWheels> m_prevLoad{};
    std::array<bool, kAudioWheels> m_prevBumpStop{};
    std::array<bool, kAudioWheels> m_prevWater{};
    std::array<double, kAudioWheels> m_lastThump{}, m_lastBump{}, m_lastSplash{};
    double m_lastImpact = -1e9, m_lastCreak = -1e9, m_lastShift = -1e9;
    float m_prevArticulation = 0.0f;
};

} // namespace tf
