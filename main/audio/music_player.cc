#include "music_player.h"

#include <algorithm>
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

// Releasing the amplifier session unconditionally would cut the power
// pre-enable that Application relies on while a TTS reply is being spoken, so
// only do it when the device is back to idle.
void EndPlaybackSession(AudioService* service) {
    if (Application::GetInstance().GetDeviceState() == kDeviceStateIdle) {
        service->SetPlaybackSessionActive(false);
    }
}

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

bool MusicPlayer::Play(const std::string& url) {
    if (audio_service_ == nullptr || request_sem_ == nullptr || url.empty()) {
        ESP_LOGW(TAG, "play rejected: player not initialized or empty url");
        return false;
    }
    if (!decoder_ready_.load()) {
        ESP_LOGE(TAG, "play rejected: no mp3 decoder registered");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(url_mutex_);
        pending_url_ = url;
    }
    // Interrupt whatever is running, then hand the new URL to the worker.
    stop_requested_ = true;
    xSemaphoreGive(request_sem_);
    return true;
}

void MusicPlayer::Stop() {
    if (request_sem_ == nullptr) {
        return;
    }
    stop_requested_ = true;
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
        std::string url;
        {
            std::lock_guard<std::mutex> lock(url_mutex_);
            url = pending_url_;
            pending_url_.clear();
            current_url_ = url;
        }
        if (url.empty()) {
            continue;
        }
        stop_requested_ = false;
        playing_ = true;
        Playback(url);
        playing_ = false;
        ReleaseStream();
        {
            std::lock_guard<std::mutex> lock(url_mutex_);
            current_url_.clear();
        }
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

void MusicPlayer::Playback(const std::string& url) {
    ESP_LOGI(TAG, "opening %s", url.c_str());
    audio_service_->SetPlaybackSessionActive(true);

    esp_http_client_config_t config = {};
    config.url = url.c_str();
    config.timeout_ms = 5000;
    config.buffer_size = kHttpBufSize;
    config.buffer_size_tx = 512;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.max_redirection_count = kMaxRedirects;
    config.user_agent = "Mozilla/5.0 (ESP32; Xiaozhi)";

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        ESP_LOGE(TAG, "failed to init http client");
        EndPlaybackSession(audio_service_);
        return;
    }

    // esp_http_client_open() does not follow redirects for the manual
    // open/read flow, so walk the Location chain ourselves. NetEase's
    // /song/media/outer/url always answers 302 to a CDN host.
    int status = 0;
    esp_err_t err = ESP_FAIL;
    for (int hop = 0; hop <= kMaxRedirects; ++hop) {
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

    if (err != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "stream unavailable (http %d)", status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        EndPlaybackSession(audio_service_);
        return;
    }

    esp_audio_simple_dec_cfg_t dec_cfg = {};
    dec_cfg.dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_MP3;
    dec_cfg.use_frame_dec = false;
    esp_audio_simple_dec_handle_t decoder = nullptr;
    auto ret = esp_audio_simple_dec_open(&dec_cfg, &decoder);
    if (ret != ESP_AUDIO_ERR_OK || decoder == nullptr) {
        ESP_LOGE(TAG, "mp3 decoder open failed: %d", ret);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        EndPlaybackSession(audio_service_);
        return;
    }
    decoder_ = decoder;

    in_buf_ = static_cast<uint8_t*>(heap_caps_malloc(kHttpBufSize, MALLOC_CAP_SPIRAM));
    out_buf_ = static_cast<uint8_t*>(heap_caps_malloc(kOutBufSize, MALLOC_CAP_SPIRAM));
    if (in_buf_ == nullptr || out_buf_ == nullptr) {
        ESP_LOGE(TAG, "out of memory for stream buffers");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        EndPlaybackSession(audio_service_);
        return;
    }
    out_buf_size_ = kOutBufSize;
    stream_rate_ = 0;

    size_t filled = 0;
    bool eos = false;
    int empty_reads = 0;
    size_t total_bytes = 0;
    const int64_t started_us = esp_timer_get_time();

    while (!stop_requested_.load()) {
        if (!eos && filled < kHttpBufSize) {
            int n = esp_http_client_read(client, reinterpret_cast<char*>(in_buf_ + filled),
                                         kHttpBufSize - filled);
            if (n < 0) {
                ESP_LOGW(TAG, "read failed: %d", n);
                break;
            }
            if (n == 0) {
                // read() also returns 0 on a socket timeout, so only treat it as
                // the end of the stream once the transport says it is complete.
                if (empty_reads >= 3 || esp_http_client_is_complete_data_received(client)) {
                    eos = true;
                } else {
                    empty_reads++;
                }
            } else {
                empty_reads = 0;
                filled += n;
                total_bytes += n;
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

            ret = esp_audio_simple_dec_process(decoder, &raw, &out);
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
                if (esp_audio_simple_dec_get_info(decoder, &info) == ESP_AUDIO_ERR_OK) {
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
            break;
        }
    }

    ESP_LOGI(TAG, "stream done: %u bytes in %lld ms (stopped=%d)", (unsigned)total_bytes,
             (long long)((esp_timer_get_time() - started_us) / 1000), (int)stop_requested_.load());

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    EndPlaybackSession(audio_service_);
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
    while (offset < source->size() && !stop_requested_.load()) {
        const size_t n = std::min(chunk, source->size() - offset);
        std::vector<int16_t> block(source->begin() + offset, source->begin() + offset + n);
        if (!audio_service_->EnqueuePlaybackPcm(std::move(block))) {
            break;
        }
        offset += n;
    }
}
