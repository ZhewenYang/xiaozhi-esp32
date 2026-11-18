#pragma once
#include "sdkconfig.h"

#include <lvgl.h>
#include <memory>
#include <thread>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "camera.h"
#include "esp_camera.h"
#include "jpg/image_to_jpeg.h"

struct JpegChunk {
    uint8_t* data;
    size_t len;
};

class Esp32Camera : public Camera {
private:
    bool streaming_on_ = false;
    bool swap_bytes_enabled_ = true;  // Swap pixel byte order for RGB565, enabled by default
    std::string explain_url_;
    std::string explain_token_;
    std::thread encoder_thread_;
    camera_fb_t* current_fb_ = nullptr;
    uint8_t* encode_buf_ = nullptr;  // Buffer for JPEG encoding (with optional byte swap)
    size_t encode_buf_size_ = 0;

    // esp32-camera#869: on ESP32-S3 the XCLK is never stopped once started.
    // Keep the driver object alive (MCP tool registration needs it) but gate
    // the 20MHz clock off while idle, and only turn it on for a capture.
    camera_config_t config_{};
    bool xclk_gated_ = false;
    void GateXclkOff();
    bool CaptureInternal();

public:
    Esp32Camera(const camera_config_t& config, bool enable_xclk_gating = false);
    ~Esp32Camera();

    virtual void SetExplainUrl(const std::string& url, const std::string& token) override;
    virtual bool Capture() override;
    virtual bool SetHMirror(bool enabled) override;
    virtual bool SetVFlip(bool enabled) override;
    virtual bool SetSwapBytes(bool enabled) override;
    virtual std::expected<std::string, std::string> Explain(const std::string& question) override;
};
