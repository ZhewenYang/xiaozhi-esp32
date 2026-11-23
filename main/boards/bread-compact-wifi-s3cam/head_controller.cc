#include "head_controller.h"
#include "mcp_server.h"
#include "settings.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <esp_log.h>

#define TAG "ArmControl"

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
    ESP_LOGI(TAG, "arm link ready: TX=GPIO%d @115200 8N1", (int)tx_pin);

    LoadMount();
    ESP_LOGI(TAG, "mounting %s (hang=%d deg)", mount_ == kMountFront ? "front" : "back",
             ArmDownPhys());

    queue_ = xQueueCreate(8, sizeof(Command));
    if (queue_ == nullptr) {
        ESP_LOGE(TAG, "failed to create command queue; arm tools disabled");
        return;
    }
    xTaskCreate(TaskEntry, "arm_ctrl", 3072, this, 4, nullptr);

    RegisterTools();

    // Deliberately does not send an angle here. The mechanical mounting has just
    // been changed by hand and the firmware cannot know which way round the arms
    // are until self.arm.mount says so; sending the hanging pose blindly could
    // whip a long arm 180 degrees. The first gesture (or self.arm.home, which
    // always resends) establishes the pose instead.
    left_ = kArmDown;
    right_ = kArmDown;
}

void HeadController::WaveOnWake() {
    ESP_LOGI(TAG, "wave on wake");
    if (!Enqueue(Action::kWave, kArmDown, kArmDown, 3)) {
        ESP_LOGW(TAG, "wave dropped, queue full");
    }
}

void HeadController::TaskEntry(void* arg) {
    static_cast<HeadController*>(arg)->Run();
}

void HeadController::Run() {
    // No initial pose is sent: see the constructor. Just idle until a command
    // arrives, then execute it.
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
    ESP_LOGD(TAG, "-> %s", line.c_str());
}

int HeadController::ToPhysical(int lift) const {
    return std::clamp(ArmDownPhys() + LiftSign() * lift, kAngleMin, kAngleMax);
}

int HeadController::ToLift(int physical) const {
    return (physical - ArmDownPhys()) * LiftSign();  // LiftSign() is +1 or -1
}

void HeadController::LoadMount() {
    Settings settings("arm");
    const int stored = settings.GetInt("mount", kMountFront);
    mount_ = (stored == kMountBack) ? kMountBack : kMountFront;
    zero_ = std::clamp(static_cast<int>(settings.GetInt("zero", DefaultZero())), kAngleMin,
                       kAngleMax);
}

void HeadController::SaveMount(int mount) {
    Settings settings("arm", true);
    settings.SetInt("mount", mount);
    settings.SetInt("zero", zero_);
}

void HeadController::SaveZero(int zero) {
    Settings settings("arm", true);
    settings.SetInt("zero", zero);
}

void HeadController::SendAngles() {
    char buf[16];
    // left_ / right_ hold lift angles; the sub-board wants raw servo angles.
    const int left_phys = ToPhysical(left_);
    const int right_phys = ToPhysical(right_);
    snprintf(buf, sizeof(buf), "nod=%d", left_phys);     // left arm
    SendLine(buf);
    snprintf(buf, sizeof(buf), "shake=%d", right_phys);  // right arm
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

// Move both arms to the targets in small steps. The sub-board has no speed
// control, so the pacing happens here: one UART line pair every step_ms, which
// at 115200 baud costs well under a millisecond each.
void HeadController::RampTo(int left_target, int right_target, int step_deg, int step_ms) {
    // Clamp in lift space to whatever the servo travel allows in this mounting.
    const int lift_lo = std::min(ToLift(kAngleMin), ToLift(kAngleMax));
    const int lift_hi = std::max(ToLift(kAngleMin), ToLift(kAngleMax));
    left_target = std::clamp(left_target, lift_lo, lift_hi);
    right_target = std::clamp(right_target, lift_lo, lift_hi);

    const int from_left = left_;
    const int from_right = right_;
    const int delta_left = left_target - from_left;
    const int delta_right = right_target - from_right;
    const int distance = std::max(std::abs(delta_left), std::abs(delta_right));
    if (distance == 0) {
        return;
    }

    const int steps = (distance + step_deg - 1) / step_deg;
    for (int i = 1; i <= steps; ++i) {
        left_ = from_left + delta_left * i / steps;
        right_ = from_right + delta_right * i / steps;
        SendAngles();
        if (i < steps) {
            vTaskDelay(pdMS_TO_TICKS(step_ms));
        }
    }
    left_ = left_target;
    right_ = right_target;
    SendAngles();
}

// "Dance": the arms trade places, one beat per RampTo call, then both drop home.
void HeadController::Dance(int count) {
    for (int i = 0; i < count; ++i) {
        if ((i % 2) == 0) {
            RampTo(kDanceHigh, kDanceLow, kDanceStepDeg, kDanceStepMs);
        } else {
            RampTo(kDanceLow, kDanceHigh, kDanceStepDeg, kDanceStepMs);
        }
    }
    RampTo(kArmDown, kArmDown, kSlowStepDeg, kSlowStepMs);
}

// "Hello": lift the right arm, wave it count times, drop it again.
void HeadController::Wave(int count) {
    RampTo(left_, kWaveHigh, kWaveStepDeg, kWaveStepMs);
    for (int i = 0; i < count; ++i) {
        RampTo(left_, kWaveLow, kWaveStepDeg, kWaveStepMs);
        RampTo(left_, kWaveHigh, kWaveStepDeg, kWaveStepMs);
    }
    RampTo(left_, kArmDown, kWaveStepDeg, kWaveStepMs);
}

void HeadController::Execute(const Command& cmd) {
    switch (cmd.action) {
        case Action::kHome:
            RampTo(kArmDown, kArmDown, kSlowStepDeg, kSlowStepMs);
            // RampTo bails out when the arms are already home in software, which
            // is exactly the state right after a reboot. Resend anyway so "put
            // the arms down" is a reliable way to discover the real pose.
            SendAngles();
            break;

        case Action::kLook:  // pan drives the right arm, tilt the left one
            RampTo(cmd.tilt, cmd.pan, kSlowStepDeg, kSlowStepMs);
            break;

        case Action::kRaiseLeft:
            RampTo(kArmRaise, right_, kSlowStepDeg, kSlowStepMs);
            break;

        case Action::kLowerLeft:
            RampTo(kArmDown, right_, kSlowStepDeg, kSlowStepMs);
            break;

        case Action::kRaiseRight:
            RampTo(left_, kArmRaise, kSlowStepDeg, kSlowStepMs);
            break;

        case Action::kLowerRight:
            RampTo(left_, kArmDown, kSlowStepDeg, kSlowStepMs);
            break;

        case Action::kDance:
            Dance(cmd.count);
            break;

        case Action::kWave:
            Wave(cmd.count);
            break;

        case Action::kSet:  // cmd.pan/tilt carry raw servo angles
            RampTo(ToLift(cmd.tilt), ToLift(cmd.pan), kSlowStepDeg, kSlowStepMs);
            break;

        case Action::kSync:  // mounting changed: re-assert the hanging pose
            left_ = kArmDown;
            right_ = kArmDown;
            SendAngles();
            break;
    }
}

void HeadController::RegisterTools() {
    auto& mcp_server = McpServer::GetInstance();

    mcp_server.AddTool(
        "self.arm.raise_left",
        "举起左手（左臂）。Raise the robot's left arm up.",
        PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            if (!Enqueue(Action::kRaiseLeft)) {
                return std::string("Arm is busy, command dropped.");
            }
            ESP_LOGI(TAG, "raise left");
            return std::string("Left arm raised.");
        });

    mcp_server.AddTool(
        "self.arm.raise_right",
        "举起右手（右臂）。Raise the robot's right arm up.",
        PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            if (!Enqueue(Action::kRaiseRight)) {
                return std::string("Arm is busy, command dropped.");
            }
            ESP_LOGI(TAG, "raise right");
            return std::string("Right arm raised.");
        });

    mcp_server.AddTool(
        "self.arm.lower_left",
        "放下左手（左臂）。Lower the robot's left arm back down.",
        PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            if (!Enqueue(Action::kLowerLeft)) {
                return std::string("Arm is busy, command dropped.");
            }
            ESP_LOGI(TAG, "lower left");
            return std::string("Left arm lowered.");
        });

    mcp_server.AddTool(
        "self.arm.lower_right",
        "放下右手（右臂）。Lower the robot's right arm back down.",
        PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            if (!Enqueue(Action::kLowerRight)) {
                return std::string("Arm is busy, command dropped.");
            }
            ESP_LOGI(TAG, "lower right");
            return std::string("Right arm lowered.");
        });

    mcp_server.AddTool(
        "self.arm.dance",
        "跳舞：左右手交替上下挥动 count 次，然后复位。"
        "Dance: the arms alternate up and down count times, then return home.",
        PropertyList({
            Property("count", kPropertyTypeInteger, 10, 2, 20),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            int count = properties["count"].value<int>();
            if (!Enqueue(Action::kDance, kArmDown, kArmDown, count)) {
                return std::string("Arm is busy, command dropped.");
            }
            ESP_LOGI(TAG, "dance count=%d", count);
            return std::string("Dancing.");
        });

    mcp_server.AddTool(
        "self.arm.home",
        "双臂放下复位，回到自然下垂姿势。Put both arms back down to the resting pose.",
        PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            if (!Enqueue(Action::kHome)) {
                return std::string("Arm is busy, command dropped.");
            }
            ESP_LOGI(TAG, "home");
            return std::string("Arms at rest.");
        });

    // Bench calibration: drive the servos to a raw angle so the real "hanging
    // down" position can be found by eye, then compared against the mapping.
    mcp_server.AddTool(
        "self.arm.set",
        "标定/直调：把手臂调到指定舵机角度（0-180）。0 与 180 是两个极限，"
        "side 取 left/right/both。Set raw servo angle for calibration.",
        PropertyList({
            Property("angle", kPropertyTypeInteger, 90, 0, 180),
            Property("side", kPropertyTypeString, std::string("both")),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const int angle = properties["angle"].value<int>();
            const std::string side = properties["side"].value<std::string>();
            int left_phys = ToPhysical(left_);
            int right_phys = ToPhysical(right_);
            if (side == "left") {
                left_phys = angle;
            } else if (side == "right") {
                right_phys = angle;
            } else {
                left_phys = angle;
                right_phys = angle;
            }
            if (!Enqueue(Action::kSet, right_phys, left_phys, 1)) {
                return std::string("Arm is busy, command dropped.");
            }
            ESP_LOGI(TAG, "set %s=%d", side.c_str(), angle);
            return std::string("Arms set to ") + std::to_string(angle) + ".";
        });

    // Mechanical mounting direction. The servos can only sweep the back half of
    // the circle as shipped, so the arms had to be re-mounted 180 degrees; this
    // tells the firmware which of the two re-mountings was actually done.
    mcp_server.AddTool(
        "self.arm.mount",
        "切换手臂安装方向，机械改装后才需要。mode=front：舵盘或延长臂已翻转180度（下垂=舵机180度）；"
        "mode=back：整个舵机绕输出轴掉头安装（下垂=舵机0度）。设置存入 NVS，重启后保留。"
        "Switch the arm mounting after a mechanical change. front = horn turned 180, "
        "back = servo body turned 180. Saved in NVS.",
        PropertyList({
            Property("mode", kPropertyTypeString, std::string("front")),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const std::string mode = properties["mode"].value<std::string>();
            int next;
            if (mode == "front") {
                next = kMountFront;
            } else if (mode == "back") {
                next = kMountBack;
            } else {
                return std::string("mode must be front or back.");
            }
            if (next == mount_) {
                return std::string("Already mounted ") + mode + ".";
            }
            mount_ = next;
            zero_ = DefaultZero();
            SaveMount(mount_);
            Enqueue(Action::kSync);
            ESP_LOGI(TAG, "mount -> %s, zero=%d", mode.c_str(), zero_);
            return std::string("Mounting set to ") + mode + ", hang pose at " +
                   std::to_string(zero_) + " deg.";
        });

    // Fine trim of the hanging pose: the horn lands wherever its splines fall,
    // so "arm straight down" is rarely the 180 / 0 endpoint. Positive delta
    // turns the gear one way, negative the other; absolute angle also accepted.
    mcp_server.AddTool(
        "self.arm.zero",
        "微调复位位：设定手臂自然下垂时对应的舵机角度（0-180）。"
        "angle 传绝对值（不传则用 delta 相对调整，delta 正=角度增大、负=角度减小）。"
        "设置存入 NVS，重启后保留。Set the servo angle treated as arms hanging down.",
        PropertyList({
            Property("angle", kPropertyTypeInteger, -1, -1, kAngleMax),
            Property("delta", kPropertyTypeInteger, 0, -90, 90),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const int angle = properties["angle"].value<int>();
            const int delta = properties["delta"].value<int>();
            int next = zero_;
            if (angle >= 0) {
                next = angle;
            } else if (delta != 0) {
                next = zero_ + delta;
            } else {
                return std::string("zero is ") + std::to_string(zero_) + " deg.";
            }
            next = std::clamp(next, kAngleMin, kAngleMax);
            zero_ = next;
            SaveZero(zero_);
            Enqueue(Action::kSync);
            ESP_LOGI(TAG, "zero -> %d", zero_);
            return std::string("Hang pose at ") + std::to_string(zero_) + " deg.";
        });
}
