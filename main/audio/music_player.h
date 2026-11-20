#ifndef MUSIC_PLAYER_H
#define MUSIC_PLAYER_H

#include <atomic>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

class AudioService;

/**
 * MusicPlayer - plays an online mp3/aac URL through the existing speaker path.
 *
 * The xiaozhi protocol only carries Opus TTS frames from the cloud, so an
 * external MCP server (connected through the console's MCP endpoint) can never
 * push music to the device by itself: its tools can only return text.
 *
 * This class closes that gap. The cloud LLM hands the URL returned by the MCP
 * tool to the on-device tool `self.audio_speaker.play_url`; the worker below
 * pulls the stream over HTTP, decodes it with esp_audio_codec's MP3 decoder,
 * downmixes to mono, resamples to the codec output rate and feeds the PCM into
 * AudioService's playback queue -- the very same queue TTS uses. Volume,
 * amplifier power sequencing, I2S setup and the playback-drained bookkeeping
 * therefore keep working unchanged.
 *
 * Quality note: the codec output is fixed at 24 kHz mono, so this is roughly
 * "good voice quality" rather than hi-fi. Reconfiguring I2S per stream would
 * be needed for 44.1 kHz stereo and is deliberately out of scope here.
 */
class MusicPlayer {
public:
    static MusicPlayer& GetInstance();

    /** Must be called once after AudioService::Start(). */
    void Initialize(AudioService* audio_service);

    /** One stream to play. `title` is only used for the screen. */
    struct Track {
        std::string url;
        std::string title;
    };

    /** Stops the current stream (if any) and starts `url`. Non-blocking.
     *  `title` is only used for the screen ("正在播放 ..."). */
    bool Play(const std::string& url, const std::string& title = "");

    /** Replaces the queue with `tracks` and starts playing the first one.
     *  The rest follow back to back until the queue is empty or Stop() is
     *  called. Non-blocking. */
    bool PlayQueue(std::vector<Track> tracks);

    /** Number of tracks still queued (not counting the one playing). */
    size_t GetQueueLength() const;

    /** Asks the worker to stop. Returns immediately; the worker exits within
     *  a few hundred milliseconds. */
    void Stop();

    bool IsPlaying() const { return playing_.load(); }
    std::string GetCurrentUrl() const;
    std::string GetCurrentTitle() const;

private:
    MusicPlayer() = default;
    MusicPlayer(const MusicPlayer&) = delete;
    MusicPlayer& operator=(const MusicPlayer&) = delete;

    enum class StreamResult { kEndOfStream, kInterrupted, kStopped, kFatal };

    static void TaskEntry(void* arg);
    void TaskLoop();
    void Playback(const std::string& url);
    // Pops the next track; false when the queue is empty.
    bool NextTrack(Track* out);
    void ClearQueue();
    // Opens `url` at `offset`, streams and decodes until the body ends, the
    // socket dies, or Stop() is called. `offset` is advanced by every byte read.
    StreamResult StreamOnce(const std::string& url, size_t& offset);
    bool EnsureDecoder();
    void KeepRadioAwake();
    void FeedPcm(const uint8_t* data, size_t size, uint32_t sample_rate, uint8_t channels,
                 uint8_t bits);
    void ReleaseStream();

    AudioService* audio_service_ = nullptr;
    SemaphoreHandle_t request_sem_ = nullptr;
    mutable std::mutex url_mutex_;
    std::string current_url_;
    std::string current_title_;
    // Tracks waiting to be played, filled by PlayQueue()/Play() and drained by
    // the worker. Guarded by queue_mutex_ so the MCP handler never blocks.
    std::deque<Track> queue_;
    mutable std::mutex queue_mutex_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> playing_{false};
    // False when no MP3 decoder could be registered (see Initialize); Play()
    // then fails fast instead of streaming into a dead decoder.
    std::atomic<bool> decoder_ready_{false};
    // Set once per stream, when the first PCM chunk actually reaches the queue.
    bool idle_requested_ = false;
    int64_t last_power_check_us_ = 0;

    // Stream state, only touched by the worker task.
    void* decoder_ = nullptr;
    void* resampler_ = nullptr;
    uint32_t stream_rate_ = 0;
    std::vector<int16_t> mono_;
    std::vector<int16_t> resampled_;
    uint8_t* in_buf_ = nullptr;
    uint8_t* out_buf_ = nullptr;
    size_t out_buf_size_ = 0;
};

#endif
