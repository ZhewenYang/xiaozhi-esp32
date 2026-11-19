#include "head_controller.h"
#include "mcp_server.h"

#include <algorithm>
#include <cstdio>
#include <esp_log.h>

#define TAG "HeadControl"

HeadController::HeadController(uart_port_t uart_num, gpio_num_t tx_pin) : uart_num_(uart_num) {
    uart_config_t config = {};
    config.baud_rate = 115200;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(uart_num_, &config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, tx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num_, 512, 0, 0, nullptr, 0));
    ESP_LOGI(TAG, "head link ready: TX=GPIO%d @115200 8N1", (int)tx_pin);

    queue_ = xQueueCreate(8, sizeof(Command));
    if (queue_ == nullptr) {
        ESP_LOGE(TAG, "failed to create command queue; head tools disabled");
        return;
    }
    xTaskCreate(TaskEntry, "head_ctrl", 3072, this, 4, nullptr);

    RegisterTools();
    // Start from a known pose so the head does not stay wherever it was left.
    Enqueue(Action::kCenter, kHomeAngle, kHomeAngle, 1);
}

void HeadController::TaskEntry(void* arg) {
    static_cast<HeadController*>(arg)->Run();
}

void HeadController::Run() {
    Command cmd = {};
    for (;;) {
        if (xQueueReceive(queue_, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        Execute(cmd);
    }
}

void HeadController::SendLine(const std::string& line) {
    std::string payload = line;
    payload.push_back('\n');
    uart_write_bytes(uart_num_, payload.data(), payload.size());
    ESP_LOGI(TAG, "-> %s", line.c_str());
}

void HeadController::SendAngles() {
    char buf[16];
    snprintf(buf, sizeof(buf), "shake=%d", pan_);
    SendLine(buf);
    snprintf(buf, sizeof(buf), "nod=%d", tilt_);
    SendLine(buf);
}

bool HeadController::Enqueue(Action action, int pan, int tilt, int count) {
    if (queue_ == nullptr) {
        return false;
    }
    Command cmd = {
        action,
        std::clamp(pan, kAngleMin, kAngleMax),
        std::clamp(tilt, kAngleMin, kAngleMax),
        std::max(1, count),
    };
    return xQueueSend(queue_, &cmd, 0) == pdTRUE;
}

void HeadController::Execute(const Command& cmd) {
    switch (cmd.action) {
        case Action::kCenter:
            pan_ = kHomeAngle;
            tilt_ = kHomeAngle;
            SendLine("center");
            break;

        case Action::kLook:
            pan_ = cmd.pan;
            tilt_ = cmd.tilt;
            SendAngles();
            break;

        case Action::kNod: {
            const int base = tilt_;
            for (int i = 0; i < cmd.count; ++i) {
                tilt_ = std::clamp(base + kSwingAmplitude, kAngleMin, kAngleMax);
                SendAngles();
                vTaskDelay(pdMS_TO_TICKS(kSwingStepMs));
                tilt_ = std::clamp(base - kSwingAmplitude, kAngleMin, kAngleMax);
                SendAngles();
                vTaskDelay(pdMS_TO_TICKS(kSwingStepMs));
            }
            tilt_ = base;
            SendAngles();
            break;
        }

        case Action::kShake: {
            const int base = pan_;
            for (int i = 0; i < cmd.count; ++i) {
                pan_ = std::clamp(base + kSwingAmplitude, kAngleMin, kAngleMax);
                SendAngles();
                vTaskDelay(pdMS_TO_TICKS(kSwingStepMs));
                pan_ = std::clamp(base - kSwingAmplitude, kAngleMin, kAngleMax);
                SendAngles();
                vTaskDelay(pdMS_TO_TICKS(kSwingStepMs));
            }
            pan_ = base;
            SendAngles();
            break;
        }
    }
}

void HeadController::RegisterTools() {
    auto& mcp_server = McpServer::GetInstance();

    mcp_server.AddTool(
        "self.head.look",
        "Turn the head to an absolute angle so it faces a direction. "
        "pan: 0-180 horizontal, 90 is straight ahead. "
        "tilt: 0-180 vertical, 90 is level, smaller looks down, larger looks up.",
        PropertyList({
            Property("pan", kPropertyTypeInteger, kHomeAngle, kAngleMin, kAngleMax),
            Property("tilt", kPropertyTypeInteger, kHomeAngle, kAngleMin, kAngleMax),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            int pan = properties["pan"].value<int>();
            int tilt = properties["tilt"].value<int>();
            if (!Enqueue(Action::kLook, pan, tilt, 1)) {
                return std::string("Head is busy, command dropped.");
            }
            ESP_LOGI(TAG, "look pan=%d tilt=%d", pan, tilt);
            return std::string("Head turned to pan=") + std::to_string(pan) + " tilt=" +
                   std::to_string(tilt) + ".";
        });

    mcp_server.AddTool(
        "self.head.center",
        "Return the head to its neutral forward-facing pose.",
        PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            if (!Enqueue(Action::kCenter, kHomeAngle, kHomeAngle, 1)) {
                return std::string("Head is busy, command dropped.");
            }
            ESP_LOGI(TAG, "center");
            return std::string("Head centered.");
        });

    mcp_server.AddTool(
        "self.head.nod",
        "Nod the head up and down, like saying yes.",
        PropertyList({
            Property("count", kPropertyTypeInteger, 1, 1, 5),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            int count = properties["count"].value<int>();
            if (!Enqueue(Action::kNod, pan_, tilt_, count)) {
                return std::string("Head is busy, command dropped.");
            }
            ESP_LOGI(TAG, "nod count=%d", count);
            return std::string("Head nodded.");
        });

    mcp_server.AddTool(
        "self.head.shake",
        "Shake the head left and right, like saying no.",
        PropertyList({
            Property("count", kPropertyTypeInteger, 1, 1, 5),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            int count = properties["count"].value<int>();
            if (!Enqueue(Action::kShake, pan_, tilt_, count)) {
                return std::string("Head is busy, command dropped.");
            }
            ESP_LOGI(TAG, "shake count=%d", count);
            return std::string("Head shook.");
        });
}
