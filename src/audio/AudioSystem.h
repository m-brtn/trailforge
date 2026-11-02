#pragma once

#include "audio/AudioMixer.h"
#include "audio/AudioTypes.h"

#include <memory>
#include <string>
#include <vector>

namespace tf {

struct AudioOptions {
    bool device = true;              // false: --no-audio (nothing is opened or loaded)
    std::string capturePath;         // non-empty: offline render to this WAV instead of a device
    std::vector<std::string> assetDirs; // candidate directories containing manifest.json
    std::vector<std::string> musicFiles; // candidate paths of the menu music (first existing one is streamed)
};

// Owns the sound bank, the mixer and either a miniaudio playback device (its callback runs on the device's audio
// thread) or an offline WAV capture rendered on the main thread from the physics ticks. Every failure (no
// manifest, a missing file, no device) is logged and leaves the game running silently.
class AudioSystem {
public:
    explicit AudioSystem(const AudioOptions& options);
    ~AudioSystem();
    AudioSystem(const AudioSystem&) = delete;
    AudioSystem& operator=(const AudioSystem&) = delete;

    bool active() const { return m_mixer != nullptr && (m_device != nullptr || m_capture != nullptr); }
    bool capturing() const { return m_capture != nullptr; }
    const std::string& status() const { return m_status; }

    // Game thread, once per frame (device) or per physics tick (capture). Never blocks on the audio thread.
    void publish(const AudioFrame& frame, const std::vector<AudioEvent>& events);
    // UI sounds for the pause menu: queued and sent with the next publish.
    void playUi(UiSound s);
    // Stops / restarts the playback device (the app is in the background on iOS). No effect without a device.
    void setSuspended(bool suspended);
    // Capture mode: render `seconds` of audio (fractional frames carried over) and append them to the WAV.
    void captureAdvance(double seconds);
    double capturedSeconds() const;

    AudioStats stats() const { return m_mailbox.stats(); }

private:
    struct Device;
    struct Capture;
    struct Music;
    void openMusic(const std::vector<std::string>& files);
    bool loadBank(const std::vector<std::string>& dirs);
    void renderOnAudioThread(float* out, unsigned frames);

    std::string m_status = "disabled";
    SoundBank m_bank;
    std::unique_ptr<AudioMixer> m_mixer;
    std::unique_ptr<Music> m_music; // decoded on the audio thread (or the capture's main thread)
    AudioMailbox m_mailbox;
    std::vector<AudioEvent> m_pendingUi;
    // Audio-thread scratch (no allocation in the callback).
    AudioFrame m_threadFrame;
    std::array<AudioEvent, AudioMailbox::kMaxEvents> m_threadEvents{};
    std::unique_ptr<Capture> m_capture;
    std::unique_ptr<Device> m_device; // last: stopped and closed first, before the mixer and the bank go away
};

} // namespace tf
