#include "music_player.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include <esp_crt_bundle.h>
#include <esp_heap_caps.h>
#include <esp_http_client.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "audio_service.h"
#include "application.h"
#include "board.h"
#include "esp_ae_rate_cvt.h"
#include "esp_audio_dec_reg.h"
#include "esp_audio_simple_dec.h"
#include "esp_audio_simple_dec_default.h"
#include "esp_mp3_dec.h"

#define TAG "MusicPlayer"

namespace {

constexpr uint32_t kHttpBufSize = 4096;
// One mp3 frame decodes to at most 1152 samples per channel, i.e. 4608 bytes
// for 16-bit stereo. Keep headroom for the decoder's frame alignment.
constexpr uint32_t kOutBufSize = 8192;
// The decoder needs roughly 20 KB of stack and ~28 KB of heap (in PSRAM).
constexpr size_t kTaskStackBytes = 24 * 1024;
// PCM handed to the playback queue per push. Two of these fit in the queue,
// which gives ~240 ms of slack against network jitter.
constexpr size_t kChunkMs = 120;
constexpr int kMaxRedirects = 6;
// A CDN socket can be closed at any point; resume that many times before
// treating the stream as dead for good.
constexpr int kMaxReconnects = 6;
// How long the first music chunk may wait for a server reply to finish before
// the player takes the speaker back. Long enough for a normal "好啦，正在播放"
// sentence, short enough that a stuck speaking state cannot mute the song.
constexpr int64_t kMaxReplyWaitUs = 4LL * 1000 * 1000;

#define MUSIC_RATE_CVT_CFG(_src_rate, _dest_rate)                               \
    (esp_ae_rate_cvt_cfg_t) {                                                   \
        .src_rate = (uint32_t)(_src_rate), .dest_rate = (uint32_t)(_dest_rate), \
        .channel = (uint8_t)ESP_AUDIO_MONO, .bits_per_sample = ESP_AUDIO_BIT16, \
        .complexity = 2, .perf_type = ESP_AE_RATE_CVT_PERF_TYPE_SPEED,          \
    }

// Static task storage: the stack lives in PSRAM so that a 24 KB stack does not
// eat the ~45 KB of free internal DRAM left after Wi-Fi and audio are running.
StackType_t* s_task_stack = nullptr;
StaticTask_t* s_task_tcb = nullptr;

}  // namespace

MusicPlayer& MusicPlayer::GetInstance() {
    static MusicPlayer instance;
    return instance;
}

void MusicPlayer::Initialize(AudioService* audio_service) {
    if (audio_service_ != nullptr) {
        return;
    }
    audio_service_ = audio_service;
    request_sem_ = xSemaphoreCreateBinary();

    // Populates the simple decoder registry (container parsers) and, depending
    // on Kconfig, the MP3 decoder as well.
    esp_audio_simple_dec_register_default();
    // esp_audio_codec is shipped as a single prebuilt library, but its frame
    // decoders are only entered into the registry when the matching Kconfig
    // option (CONFIG_AUDIO_DECODER_MP3_SUPPORT) is set at build time. This
    // firmware builds without it, so register MP3 explicitly -- otherwise
    // esp_audio_simple_dec_open() fails with "Decoder MP3 ... not registered".
    if (esp_audio_dec_get_ops(ESP_AUDIO_TYPE_MP3) == nullptr) {
        esp_audio_err_t err = esp_mp3_dec_register();
        if (err == ESP_AUDIO_ERR_OK) {
            decoder_ready_ = true;
            ESP_LOGI(TAG, "mp3 decoder registered explicitly (Kconfig was off)");
        } else {
            ESP_LOGE(TAG, "mp3 decoder registration failed: %d", (int)err);
        }
    } else {
        decoder_ready_ = true;
        ESP_LOGI(TAG, "mp3 decoder already registered by Kconfig");
    }

    s_task_stack = static_cast<StackType_t*>(
        heap_caps_malloc(kTaskStackBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    s_task_tcb =
        static_cast<StaticTask_t*>(heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL));
    if (s_task_stack == nullptr || s_task_tcb == nullptr) {
        ESP_LOGE(TAG, "failed to allocate music player task stack in PSRAM");
        return;
    }
    xTaskCreateStatic(TaskEntry, "music_player", kTaskStackBytes / sizeof(StackType_t), this, 3,
                      s_task_stack, s_task_tcb);
    ESP_LOGI(TAG, "music player ready (stack %u bytes in PSRAM)", (unsigned)kTaskStackBytes);
}

std::string MusicPlayer::GetCurrentUrl() const {
    std::lock_guard<std::mutex> lock(url_mutex_);
    return current_url_;
}

std::string MusicPlayer::GetCurrentTitle() const {
    std::lock_guard<std::mutex> lock(url_mutex_);
    return current_title_;
}

bool MusicPlayer::Play(const std::string& url, const std::string& title) {
    return PlayQueue({Track{url, title}});
}

bool MusicPlayer::PlayQueue(std::vector<Track> tracks) {
    if (audio_service_ == nullptr || request_sem_ == nullptr || tracks.empty()) {
        ESP_LOGW(TAG, "play rejected: player not initialized or empty url");
        return false;
    }
    if (!decoder_ready_.load()) {
        ESP_LOGE(TAG, "play rejected: no mp3 decoder registered");
        return false;
    }
    for (const auto& track : tracks) {
        if (track.url.rfind("http://", 0) != 0 && track.url.rfind("https://", 0) != 0) {
            ESP_LOGE(TAG, "play rejected: bad url in queue");
            return false;
        }
    }
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.clear();
        for (auto& track : tracks) {
            queue_.push_back(std::move(track));
        }
    }
    ESP_LOGI(TAG, "queue set: %u track(s)", (unsigned)tracks.size());
    // Interrupt whatever is running; the worker drains the new queue.
    stop_requested_ = true;
    xSemaphoreGive(request_sem_);
    return true;
}

size_t MusicPlayer::GetQueueLength() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return queue_.size();
}

bool MusicPlayer::NextTrack(Track* out) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.empty()) {
        return false;
    }
    *out = std::move(queue_.front());
    queue_.pop_front();
    return true;
}

void MusicPlayer::ClearQueue() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.clear();
}

void MusicPlayer::Stop() {
    if (request_sem_ == nullptr) {
        return;
    }
    stop_requested_ = true;
    ClearQueue();
    // Drop a request that has been queued but not picked up yet.
    while (xSemaphoreTake(request_sem_, 0) == pdTRUE) {
    }
}

void MusicPlayer::TaskEntry(void* arg) { static_cast<MusicPlayer*>(arg)->TaskLoop(); }

void MusicPlayer::TaskLoop() {
    while (true) {
        if (xSemaphoreTake(request_sem_, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        // One "leave the conversation" hand-off per queue, not per track.
        idle_requested_ = false;
        while (true) {
            Track track;
            if (!NextTrack(&track)) {
                break;
            }
            // A PlayQueue() call during the previous track leaves
            // stop_requested_ set; clearing it here starts the next track.
            stop_requested_ = false;
            {
                std::lock_guard<std::mutex> lock(url_mutex_);
                current_url_ = track.url;
                current_title_ = track.title;
            }
            playing_ = true;
            // Show the new title before the first sample of the track, so a
            // multi-track queue updates the screen at every transition.
            Application::GetInstance().Schedule(
                []() { Application::GetInstance().RefreshMusicDisplay(); });
            Playback(track.url);
            playing_ = false;
            ReleaseStream();
            {
                std::lock_guard<std::mutex> lock(url_mutex_);
                current_url_.clear();
                current_title_.clear();
            }
            ESP_LOGI(TAG, "track finished, %u track(s) left", (unsigned)GetQueueLength());
            if (stop_requested_.load()) {
                // Either Stop() (queue cleared, loop exits next) or a fresh
                // queue that must keep playing. Re-reading is correct for both.
                ESP_LOGI(TAG, "playback interrupted, re-reading the queue");
            }
        }
        ClearQueue();
        {
            std::lock_guard<std::mutex> lock(url_mutex_);
            current_url_.clear();
            current_title_.clear();
        }
        // Bring the screen back to standby now that nothing is playing.
        Application::GetInstance().Schedule(
            []() { Application::GetInstance().RefreshMusicDisplay(); });
        ESP_LOGI(TAG, "playback finished");
    }
}

void MusicPlayer::ReleaseStream() {
    if (decoder_ != nullptr) {
        esp_audio_simple_dec_close(decoder_);
        decoder_ = nullptr;
    }
    if (resampler_ != nullptr) {
        esp_ae_rate_cvt_close(resampler_);
        resampler_ = nullptr;
    }
    if (in_buf_ != nullptr) {
        heap_caps_free(in_buf_);
        in_buf_ = nullptr;
    }
    if (out_buf_ != nullptr) {
        heap_caps_free(out_buf_);
        out_buf_ = nullptr;
        out_buf_size_ = 0;
    }
    stream_rate_ = 0;
    std::vector<int16_t>().swap(mono_);
    std::vector<int16_t>().swap(resampled_);
}

bool MusicPlayer::EnsureDecoder() {
    if (decoder_ != nullptr) {
        return true;
    }
    esp_audio_simple_dec_cfg_t dec_cfg = {};
    dec_cfg.dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_MP3;
    dec_cfg.use_frame_dec = false;
    esp_audio_simple_dec_handle_t decoder = nullptr;
    auto ret = esp_audio_simple_dec_open(&dec_cfg, &decoder);
    if (ret != ESP_AUDIO_ERR_OK || decoder == nullptr) {
        ESP_LOGE(TAG, "mp3 decoder open failed: %d", ret);
        return false;
    }
    decoder_ = decoder;
    return true;
}

void MusicPlayer::KeepRadioAwake() {
    // The audio channel close path drops the radio to modem sleep, and a
    // modem-sleeping station loses most of a sustained HTTP stream: the CDN
    // socket eventually dies with ENOTCONN in the middle of a song. Re-assert
    // performance mode periodically while streaming (cheap: the driver ignores
    // a no-op change).
    const int64_t now = esp_timer_get_time();
    if (now - last_power_check_us_ < 2000000) {
        return;
    }
    last_power_check_us_ = now;
    Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
}

MusicPlayer::StreamResult MusicPlayer::StreamOnce(const std::string& url, size_t& offset) {
    esp_http_client_config_t config = {};
    config.url = url.c_str();
    config.timeout_ms = 8000;
    config.buffer_size = kHttpBufSize;
    config.buffer_size_tx = 512;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.max_redirection_count = kMaxRedirects;
    config.user_agent = "Mozilla/5.0 (ESP32; Xiaozhi)";

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        ESP_LOGE(TAG, "failed to init http client");
        return StreamResult::kFatal;
    }

    // Resume where the previous connection died. NetEase's CDN serves byte
    // ranges, so a dropped socket only costs a few hundred milliseconds of music.
    const bool resuming = offset > 0;
    if (resuming) {
        ESP_LOGI(TAG, "resuming at byte %u", (unsigned)offset);
    }

    int status = 0;
    esp_err_t err = ESP_FAIL;
    for (int hop = 0; hop <= kMaxRedirects; ++hop) {
        if (resuming) {
            // Set it on every hop: the redirect target is what actually serves
            // the bytes, so the header has to survive the URL swap.
            char range[40];
            snprintf(range, sizeof(range), "bytes=%u-", (unsigned)offset);
            esp_http_client_set_header(client, "Range", range);
        }
        err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "open failed: %s", esp_err_to_name(err));
            break;
        }
        esp_http_client_fetch_headers(client);
        status = esp_http_client_get_status_code(client);
        if (status == 301 || status == 302 || status == 303 || status == 307 || status == 308) {
            if (esp_http_client_set_redirection(client) != ESP_OK) {
                ESP_LOGE(TAG, "redirect without a usable Location header");
                status = 0;
                break;
            }
            ESP_LOGI(TAG, "http %d, following redirect", status);
            esp_http_client_close(client);
            continue;
        }
        break;
    }

    if (err != ESP_OK || (status != 200 && status != 206)) {
        ESP_LOGE(TAG, "stream unavailable (http %d)", status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return StreamResult::kFatal;
    }

    if (resuming && status == 200) {
        // The server ignored the Range header and restarted the file. Skip the
        // part we already played instead of looping the song from the top.
        ESP_LOGW(TAG, "server ignored Range, skipping %u bytes", (unsigned)offset);
        size_t remaining = offset;
        while (remaining > 0 && !stop_requested_.load()) {
            int n = esp_http_client_read(
                client, reinterpret_cast<char*>(in_buf_),
                static_cast<int>(std::min(remaining, static_cast<size_t>(kHttpBufSize))));
            if (n <= 0) {
                break;
            }
            remaining -= n;
        }
        if (remaining > 0) {
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return StreamResult::kInterrupted;
        }
    }

    if (!EnsureDecoder()) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return StreamResult::kFatal;
    }

    size_t filled = 0;
    bool eos = false;
    int empty_reads = 0;
    StreamResult result = StreamResult::kInterrupted;

    while (!stop_requested_.load()) {
        KeepRadioAwake();

        if (!eos && filled < kHttpBufSize) {
            int n = esp_http_client_read(client, reinterpret_cast<char*>(in_buf_ + filled),
                                         kHttpBufSize - filled);
            if (n < 0) {
                ESP_LOGW(TAG, "read failed: %d", n);
                result = StreamResult::kInterrupted;
                break;
            }
            if (n == 0) {
                // A read timeout and the end of the body look the same here.
                // Only a fully received body is a clean end; three empty reads
                // in a row mean the connection is done for.
                if (esp_http_client_is_complete_data_received(client)) {
                    eos = true;
                    result = StreamResult::kEndOfStream;
                } else if (++empty_reads >= 3) {
                    result = StreamResult::kInterrupted;
                    break;
                }
            } else {
                empty_reads = 0;
                filled += n;
                offset += n;
            }
        }

        // Drain every complete frame the parser can produce from what we hold.
        bool progress = true;
        while (progress && !stop_requested_.load()) {
            if (filled == 0 && !eos) {
                break;
            }
            esp_audio_simple_dec_raw_t raw = {};
            raw.buffer = in_buf_;
            raw.len = filled;
            raw.eos = eos;

            esp_audio_simple_dec_out_t out = {};
            out.buffer = out_buf_;
            out.len = out_buf_size_;

            auto ret = esp_audio_simple_dec_process(decoder_, &raw, &out);
            if (ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH && out.needed_size > out_buf_size_) {
                heap_caps_free(out_buf_);
                out_buf_size_ = out.needed_size;
                out_buf_ =
                    static_cast<uint8_t*>(heap_caps_malloc(out_buf_size_, MALLOC_CAP_SPIRAM));
                if (out_buf_ == nullptr) {
                    ESP_LOGE(TAG, "cannot grow decode buffer to %u", (unsigned)out_buf_size_);
                    out_buf_size_ = 0;
                    break;
                }
                continue;
            }
            if (ret != ESP_AUDIO_ERR_OK) {
                ESP_LOGW(TAG, "decode error %d, dropping the rest of the stream", ret);
                eos = true;
                filled = 0;
                result = StreamResult::kEndOfStream;
                break;
            }

            if (raw.consumed > 0) {
                filled -= raw.consumed;
                if (filled > 0) {
                    std::memmove(in_buf_, in_buf_ + raw.consumed, filled);
                }
            }
            progress = raw.consumed > 0;

            if (out.decoded_size > 0) {
                esp_audio_simple_dec_info_t info = {};
                if (esp_audio_simple_dec_get_info(decoder_, &info) == ESP_AUDIO_ERR_OK) {
                    FeedPcm(out.buffer, out.decoded_size, info.sample_rate, info.channel,
                            info.bits_per_sample);
                }
            }
        }

        if (eos && (filled == 0 || !progress)) {
            // Either the stream ended cleanly, or the leftover tail is not a
            // frame the parser can use.
            break;
        }
        if (!progress && filled >= kHttpBufSize) {
            // Full buffer and the parser still cannot find a frame: the payload
            // is not mp3. Give up instead of spinning.
            ESP_LOGE(TAG, "no mp3 frame found in %u bytes, aborting",
                     (unsigned)kHttpBufSize);
            result = StreamResult::kFatal;
            break;
        }
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (stop_requested_.load()) {
        return StreamResult::kStopped;
    }
    if (eos && result == StreamResult::kInterrupted) {
        return StreamResult::kEndOfStream;
    }
    return result;
}

void MusicPlayer::Playback(const std::string& url) {
    ESP_LOGI(TAG, "opening %s", url.c_str());
    // Do not claim a playback session here: the session flag belongs to server
    // replies, and the music stream has to yield to them (see FeedPcm()).
    audio_service_->EnsureOutputEnabled();
    KeepRadioAwake();

    in_buf_ = static_cast<uint8_t*>(heap_caps_malloc(kHttpBufSize, MALLOC_CAP_SPIRAM));
    out_buf_ = static_cast<uint8_t*>(heap_caps_malloc(kOutBufSize, MALLOC_CAP_SPIRAM));
    if (in_buf_ == nullptr || out_buf_ == nullptr) {
        ESP_LOGE(TAG, "out of memory for stream buffers");
        return;
    }
    out_buf_size_ = kOutBufSize;
    stream_rate_ = 0;

    size_t total_bytes = 0;
    const int64_t started_us = esp_timer_get_time();
    int reconnects = 0;

    while (!stop_requested_.load()) {
        StreamResult result = StreamOnce(url, total_bytes);
        if (result != StreamResult::kInterrupted) {
            break;
        }
        if (++reconnects > kMaxReconnects) {
            ESP_LOGW(TAG, "stream keeps failing, giving up after %d retries", kMaxReconnects);
            break;
        }
        ESP_LOGW(TAG, "stream interrupted at %u bytes, reconnecting (%d/%d)",
                 (unsigned)total_bytes, reconnects, kMaxReconnects);
        // The resume point can land mid-frame; a fresh decoder resyncs on the
        // next frame header instead of dropping everything as garbage.
        if (decoder_ != nullptr) {
            esp_audio_simple_dec_close(decoder_);
            decoder_ = nullptr;
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    ESP_LOGI(TAG, "stream done: %u bytes in %lld ms (stopped=%d, retries=%d)",
             (unsigned)total_bytes, (long long)((esp_timer_get_time() - started_us) / 1000),
             (int)stop_requested_.load(), reconnects);
}

void MusicPlayer::FeedPcm(const uint8_t* data, size_t size, uint32_t sample_rate, uint8_t channels,
                          uint8_t bits) {
    if (bits != 16 || (channels != 1 && channels != 2) || size < sizeof(int16_t)) {
        return;
    }
    const int16_t* in = reinterpret_cast<const int16_t*>(data);
    const size_t samples = size / sizeof(int16_t);

    // Downmix to mono: the codec output path is mono.
    if (channels == 2) {
        const size_t frames = samples / 2;
        mono_.resize(frames);
        for (size_t i = 0; i < frames; ++i) {
            mono_[i] = static_cast<int16_t>((static_cast<int32_t>(in[i * 2]) + in[i * 2 + 1]) / 2);
        }
    } else {
        mono_.assign(in, in + samples);
    }

    auto codec = Board::GetInstance().GetAudioCodec();
    if (codec == nullptr) {
        return;
    }
    const uint32_t target_rate = static_cast<uint32_t>(codec->output_sample_rate());

    const std::vector<int16_t>* source = &mono_;
    if (sample_rate > 0 && sample_rate != target_rate) {
        if (resampler_ == nullptr || stream_rate_ != sample_rate) {
            if (resampler_ != nullptr) {
                esp_ae_rate_cvt_close(resampler_);
                resampler_ = nullptr;
            }
            esp_ae_rate_cvt_cfg_t cfg = MUSIC_RATE_CVT_CFG(sample_rate, target_rate);
            esp_ae_rate_cvt_open(&cfg, &resampler_);
            if (resampler_ == nullptr) {
                ESP_LOGE(TAG, "resampler %u -> %u Hz failed", (unsigned)sample_rate,
                         (unsigned)target_rate);
                return;
            }
            stream_rate_ = sample_rate;
            ESP_LOGI(TAG, "resampling %u -> %u Hz", (unsigned)sample_rate, (unsigned)target_rate);
        }
        uint32_t max_out = mono_.size();
        esp_ae_rate_cvt_get_max_out_sample_num(resampler_, mono_.size(), &max_out);
        resampled_.resize(max_out);
        uint32_t out_samples = max_out;
        esp_ae_rate_cvt_process(resampler_, reinterpret_cast<esp_ae_sample_t>(mono_.data()),
                                mono_.size(), reinterpret_cast<esp_ae_sample_t>(resampled_.data()),
                                &out_samples);
        resampled_.resize(out_samples);
        source = &resampled_;
    } else {
        stream_rate_ = sample_rate;
        if (resampler_ != nullptr) {
            esp_ae_rate_cvt_close(resampler_);
            resampler_ = nullptr;
        }
    }

    const size_t chunk = std::max<size_t>(1, target_rate * kChunkMs / 1000);
    size_t offset = 0;
    int64_t reply_wait_us = 0;
    while (offset < source->size() && !stop_requested_.load()) {
        // A server reply owns the speaker. Keep the decoded PCM here and stop
        // pulling from the socket until the reply is over, so the sentence is
        // not interleaved with music and the stream resumes afterwards.
        //
        // Bounded wait: some LLM turns end right after the tool call without a
        // closing sentence, and the server then leaves the session in the
        // speaking state forever. Before this timeout the music thread waited
        // for an "reply finished" flag that never came, so the screen stayed on
        // "说话中" and not a single sample reached the speaker.
        //
        // FeedPcm runs once per decoded frame, so the timeout must be latched
        // by idle_requested_: waiting again on every frame would turn a stuck
        // session into 4 s of silence per frame.
        if (audio_service_->IsPlaybackSessionActive() && !idle_requested_) {
            if (reply_wait_us < kMaxReplyWaitUs) {
                vTaskDelay(pdMS_TO_TICKS(30));
                reply_wait_us += 30000;
                continue;
            }
            idle_requested_ = true;
            ESP_LOGW(TAG, "reply session still active after %d ms, taking the speaker back",
                     (int)(reply_wait_us / 1000));
            Application::GetInstance().Schedule(
                []() { Application::GetInstance().EnterIdleForMusicPlayback(); });
        }
        const size_t n = std::min(chunk, source->size() - offset);
        std::vector<int16_t> block(source->begin() + offset, source->begin() + offset + n);
        if (!audio_service_->EnqueuePlaybackPcm(std::move(block))) {
            break;
        }
        offset += n;
        if (!idle_requested_) {
            // The reply that announced the song is over, so this is the first
            // audible chunk. Leave the server's listening session now instead
            // of letting it time out and say goodbye over the music.
            idle_requested_ = true;
            Application::GetInstance().Schedule(
                []() { Application::GetInstance().EnterIdleForMusicPlayback(); });
        }
    }
}
