#include "audio/AudioSystem.h"

#include <miniaudio.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace tf {

struct AudioSystem::Device {
    ma_device device{};
    bool initialized = false;
    ~Device() {
        if (initialized) ma_device_uninit(&device); // stops the device and joins its thread
    }
};

struct AudioSystem::Capture {
    ma_encoder encoder{};
    bool initialized = false;
    std::FILE* csv = nullptr;
    std::vector<float> buffer;
    double pending = 0.0; // fractional frames carried to the next call
    long long frames = 0;
    ~Capture() {
        if (initialized) ma_encoder_uninit(&encoder);
        if (csv) std::fclose(csv);
    }
};

// Streams an MP3 (or WAV) from memory: the whole file is read once, decoding happens block by block in read().
struct AudioSystem::Music final : MusicSource {
    std::vector<char> bytes;
    ma_decoder decoder{};
    bool initialized = false;
    int rate = 48000;
    ~Music() override {
        if (initialized) ma_decoder_uninit(&decoder);
    }
    int read(float* stereo, int frames) override {
        ma_uint64 got = 0;
        ma_decoder_read_pcm_frames(&decoder, stereo, ma_uint64(frames), &got);
        return int(got);
    }
    void rewind() override { ma_decoder_seek_to_pcm_frame(&decoder, 0); }
    int sampleRate() const override { return rate; }
};

namespace {

bool decodeFile(const std::string& path, int sampleRate, SoundData& out, std::string& error) {
    ma_decoder_config cfg = ma_decoder_config_init(ma_format_f32, 0, ma_uint32(sampleRate));
    ma_decoder dec;
    const ma_result r = ma_decoder_init_file(path.c_str(), &cfg, &dec);
    if (r != MA_SUCCESS) {
        error = path + ": " + ma_result_description(r);
        return false;
    }
    ma_format format;
    ma_uint32 channels = 0, rate = 0;
    ma_decoder_get_data_format(&dec, &format, &channels, &rate, nullptr, 0);
    ma_uint64 length = 0;
    ma_decoder_get_length_in_pcm_frames(&dec, &length);
    out.channels = int(channels);
    out.pcm.assign(std::size_t(length) * channels, 0.0f);
    ma_uint64 read = 0;
    ma_decoder_read_pcm_frames(&dec, out.pcm.data(), length, &read);
    ma_decoder_uninit(&dec);
    out.frames = std::size_t(read);
    out.pcm.resize(out.frames * channels);
    if (out.frames == 0) {
        error = path + ": no frames";
        return false;
    }
    return true;
}

} // namespace

AudioSystem::AudioSystem(const AudioOptions& options) {
    if (!options.device && options.capturePath.empty()) {
        std::printf("Audio: disabled (--no-audio)\n");
        return;
    }
    if (!loadBank(options.assetDirs)) return;
    m_mixer = std::make_unique<AudioMixer>(m_bank, m_bank.manifest.sampleRate);
    openMusic(options.musicFiles);

    if (!options.capturePath.empty()) {
        // Offline render: no device; the main thread drives the mixer from the physics ticks.
        auto cap = std::make_unique<Capture>();
        ma_encoder_config cfg = ma_encoder_config_init(ma_encoding_format_wav, ma_format_f32, 2,
                                                       ma_uint32(m_mixer->sampleRate()));
        const ma_result r = ma_encoder_init_file(options.capturePath.c_str(), &cfg, &cap->encoder);
        if (r != MA_SUCCESS) {
            m_status = "capture failed: " + std::string(ma_result_description(r));
            std::printf("Audio: cannot write %s (%s), audio off\n", options.capturePath.c_str(),
                        ma_result_description(r));
            m_mixer.reset();
            return;
        }
        cap->initialized = true;
        const std::string csvPath = options.capturePath + ".csv";
        cap->csv = std::fopen(csvPath.c_str(), "w");
        if (cap->csv)
            std::fprintf(cap->csv, "t,rpm,rpm_src,load,off0,off1,off2,on0,on1,on2,rate0,rate1,rate2,skid,gravel,dirt,mud,"
                                   "grass,wind,day,night,whine_hz,gameplay,voices,peak_db,limiter_db,motor_hz,motor_level,"
                                   "inverter_level,road,scrub,ev_bump,ev_rattle,ev_impact,ev_creak,rattle,knocks\n");
        m_capture = std::move(cap);
        m_status = "capture -> " + options.capturePath;
        std::printf("Audio: offline capture to %s (48 kHz stereo float, driven by physics ticks; parameters in %s)\n",
                    options.capturePath.c_str(), csvPath.c_str());
        return;
    }

    auto dev = std::make_unique<Device>();
    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format = ma_format_f32;
    cfg.playback.channels = 2;
    cfg.sampleRate = ma_uint32(m_mixer->sampleRate());
    cfg.periodSizeInMilliseconds = 10;
    cfg.noFixedSizedCallback = MA_TRUE; // the mixer handles any callback size
    cfg.pUserData = this;
    cfg.dataCallback = [](ma_device* device, void* output, const void*, ma_uint32 frames) {
        static_cast<AudioSystem*>(device->pUserData)->renderOnAudioThread(static_cast<float*>(output), frames);
    };
    ma_result r = ma_device_init(nullptr, &cfg, &dev->device);
    if (r != MA_SUCCESS) {
        m_status = "no device: " + std::string(ma_result_description(r));
        std::printf("Audio: no playback device (%s), continuing without sound\n", ma_result_description(r));
        m_mixer.reset();
        return;
    }
    dev->initialized = true;
    r = ma_device_start(&dev->device);
    if (r != MA_SUCCESS) {
        m_status = "device start failed: " + std::string(ma_result_description(r));
        std::printf("Audio: device start failed (%s), continuing without sound\n", ma_result_description(r));
        dev.reset();
        m_mixer.reset();
        return;
    }
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s, %s, %u Hz device / %d Hz mix", dev->device.playback.name,
                  ma_get_backend_name(dev->device.pContext->backend), dev->device.playback.internalSampleRate,
                  m_mixer->sampleRate());
    m_status = buf;
    std::printf("Audio: playback on %s, period %u frames\n", buf, dev->device.playback.internalPeriodSizeInFrames);
    m_device = std::move(dev);
}

AudioSystem::~AudioSystem() {
    m_device.reset();
    m_capture.reset();
}

bool AudioSystem::loadBank(const std::vector<std::string>& dirs) {
    namespace fs = std::filesystem;
    const auto t0 = std::chrono::steady_clock::now();
    std::string dir;
    for (const std::string& d : dirs) {
        std::error_code ec;
        if (fs::exists(fs::path(d) / "manifest.json", ec)) {
            dir = fs::weakly_canonical(fs::path(d), ec).string();
            if (dir.empty()) dir = d;
            break;
        }
    }
    if (dir.empty()) {
        m_status = "assets not found";
        std::printf("Audio: assets/audio/manifest.json not found, audio off\n");
        return false;
    }
    std::string error;
    if (!loadAudioManifest((fs::path(dir) / "manifest.json").string(), m_bank.manifest, &error)) {
        m_status = "manifest error";
        std::printf("Audio: %s, audio off\n", error.c_str());
        return false;
    }
    std::size_t bytes = 0;
    int loaded = 0, failed = 0;
    for (std::size_t i = 0; i < kSoundCount; ++i) {
        const SoundInfo& info = m_bank.manifest.sounds[i];
        if (info.id.empty()) {
            std::printf("Audio: manifest has no '%s' (that sound stays silent)\n", kSoundNames[i]);
            ++failed;
            continue;
        }
        SoundData& d = m_bank.sounds[i];
        if (!decodeFile((fs::path(dir) / info.file).string(), m_bank.manifest.sampleRate, d, error)) {
            std::printf("Audio: %s (that sound stays silent)\n", error.c_str());
            d = SoundData{};
            ++failed;
            continue;
        }
        d.loop = info.loop;
        bytes += d.pcm.size() * sizeof(float);
        ++loaded;
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("Audio: %d sounds loaded from %s (%.1f MB as float, %d missing), %.0f ms\n", loaded, dir.c_str(),
                double(bytes) / (1024.0 * 1024.0), failed, ms);
    return loaded > 0;
}

void AudioSystem::openMusic(const std::vector<std::string>& files) {
    for (const std::string& path : files) {
        std::ifstream in(path, std::ios::binary);
        if (!in) continue;
        auto music = std::make_unique<Music>();
        music->bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        music->rate = m_mixer->sampleRate();
        ma_decoder_config cfg = ma_decoder_config_init(ma_format_f32, 2, ma_uint32(music->rate));
        const ma_result r = ma_decoder_init_memory(music->bytes.data(), music->bytes.size(), &cfg, &music->decoder);
        if (r != MA_SUCCESS) {
            std::printf("Audio: cannot decode music %s (%s)\n", path.c_str(), ma_result_description(r));
            continue;
        }
        music->initialized = true;
        ma_uint64 length = 0;
        ma_decoder_get_length_in_pcm_frames(&music->decoder, &length);
        std::printf("Audio: music %s (%.1f MB, %.1f s, streamed)\n", path.c_str(),
                    double(music->bytes.size()) / (1024.0 * 1024.0), double(length) / double(music->rate));
        m_music = std::move(music);
        m_mixer->setMusic(m_music.get());
        return;
    }
    std::printf("Audio: no music file found (menus stay without music)\n");
}

void AudioSystem::renderOnAudioThread(float* out, unsigned frames) {
    int count = 0, dropped = 0;
    if (m_mailbox.tryTake(m_threadFrame, m_threadEvents, count, dropped)) m_mixer->setFrame(m_threadFrame);
    for (int i = 0; i < count; ++i) m_mixer->trigger(m_threadEvents[std::size_t(i)]);
    m_mixer->render(out, int(frames));
    AudioStats s = m_mixer->stats();
    s.eventsDropped = dropped;
    m_mailbox.tryPutStats(s);
}

void AudioSystem::publish(const AudioFrame& frame, const std::vector<AudioEvent>& events) {
    if (!m_mixer) {
        m_pendingUi.clear();
        return;
    }
    if (!m_pendingUi.empty()) {
        std::vector<AudioEvent> all = events;
        all.insert(all.end(), m_pendingUi.begin(), m_pendingUi.end());
        m_pendingUi.clear();
        m_mailbox.publish(frame, all.data(), int(all.size()));
    } else {
        m_mailbox.publish(frame, events.data(), int(events.size()));
    }
}

void AudioSystem::playUi(UiSound s) {
    AudioEvent e;
    e.type = AudioEventType::Ui;
    e.ui = s;
    m_pendingUi.push_back(e);
}

void AudioSystem::captureAdvance(double seconds) {
    if (!m_capture || !m_mixer || seconds <= 0.0) return;
    Capture& c = *m_capture;
    c.pending += seconds * double(m_mixer->sampleRate());
    const long long n = (long long)std::floor(c.pending);
    if (n <= 0) return;
    c.pending -= double(n);
    c.buffer.resize(std::size_t(n) * 2);
    renderOnAudioThread(c.buffer.data(), unsigned(n));
    ma_uint64 written = 0;
    ma_encoder_write_pcm_frames(&c.encoder, c.buffer.data(), ma_uint64(n), &written);
    c.frames += n;
    if (c.csv) {
        const AudioStats& s = m_mixer->stats();
        std::fprintf(c.csv,
                     "%.4f,%.1f,%.1f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                     "%.4f,%.1f,%.3f,%d,%.1f,%.2f,%.1f,%.5f,%.5f,%.4f,%.4f,%d,%d,%d,%d,%.4f,%d\n",
                     double(c.frames) / double(m_mixer->sampleRate()), s.rpm, s.sourceRpm, s.load, s.offWeights[0],
                     s.offWeights[1], s.offWeights[2], s.onWeights[0], s.onWeights[1], s.onWeights[2],
                     s.exhaustRates[0], s.exhaustRates[1], s.exhaustRates[2], s.skid, s.gravel, s.dirt, s.mud,
                     s.grass, s.wind, s.day, s.night, s.whineHz, s.gameplayGain, s.activeVoices, s.peakDb,
                     s.limiterDb, s.motorHz, s.motorLevel, s.inverterLevel, s.road, s.scrub, s.eventsByType[2],
                     s.eventsByType[3], s.eventsByType[4], s.eventsByType[6], s.rattle, s.knocks);
    }
}

double AudioSystem::capturedSeconds() const {
    return m_capture && m_mixer ? double(m_capture->frames) / double(m_mixer->sampleRate()) : 0.0;
}

} // namespace tf
