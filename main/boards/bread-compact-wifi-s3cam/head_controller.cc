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
    ESP_LOGI(TAG, "mounting %s (right hang=%d sig%+d, left hang=%d sig%+d, dir %+d)",
             mount_ == kMountFront ? "front" : "back", zero_, CombinedSign(false), left_zero_,
             CombinedSign(true), LiftSign());

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
    if (!Enqueue(Action::kWaveRight, kArmDown, kArmDown, 3)) {
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

int HeadController::ToPhysical(int lift, bool left) const {
    return std::clamp(ArmZero(left) + CombinedSign(left) * lift, kAngleMin, kAngleMax);
}

int HeadController::ToLift(int physical, bool left) const {
    return (physical - ArmZero(left)) * CombinedSign(left);  // sign is +1 or -1
}

// Lift values that are still reachable on this arm, given the hang pose and the
// per-arm polarity. The hang pose sits somewhere in 0-180, so one direction
// always runs out of travel before the other.
void HeadController::LiftRange(bool left, int* lo, int* hi) const {
    const int a = ToLift(kAngleMin, left);
    const int b = ToLift(kAngleMax, left);
    *lo = std::min(a, b);
    *hi = std::max(a, b);
}

void HeadController::LoadMount() {
    Settings settings("arm");
    const int stored = settings.GetInt("mount", kMountFront);
    mount_ = (stored == kMountBack) ? kMountBack : kMountFront;
    zero_ = std::clamp(static_cast<int>(settings.GetInt("zero", DefaultZero())), kAngleMin,
                       kAngleMax);
    // The left servo is mounted back-to-back with the right one, so its hanging
    // pose sits at the opposite end of the travel: 180 - zero. Stored separately
    // because the mirror is never perfect and it may need its own trim.
    left_zero_ = std::clamp(static_cast<int>(settings.GetInt("lzero", kAngleMax - zero_)),
                            kAngleMin, kAngleMax);
    dir_ = (settings.GetInt("dir", kDirNormal) < 0) ? kDirFlipped : kDirNormal;
    left_dir_ = (settings.GetInt("ldir", kArmDirLeft) < 0) ? kDirFlipped : kDirNormal;
    right_dir_ = (settings.GetInt("rdir", kArmDirRight) < 0) ? kDirFlipped : kDirNormal;
}

void HeadController::SaveMount(int mount) {
    Settings settings("arm", true);
    settings.SetInt("mount", mount);
    settings.SetInt("zero", zero_);
    settings.SetInt("lzero", left_zero_);
    settings.SetInt("dir", dir_);
    settings.SetInt("ldir", left_dir_);
    settings.SetInt("rdir", right_dir_);
}

void HeadController::SaveZero(int zero) {
    Settings settings("arm", true);
    settings.SetInt("zero", zero);
    settings.SetInt("lzero", left_zero_);
}

void HeadController::SaveLeftZero(int zero) {
    Settings settings("arm", true);
    settings.SetInt("lzero", zero);
}

void HeadController::SaveDir(int dir) {
    Settings settings("arm", true);
    settings.SetInt("dir", dir);
}

void HeadController::SaveArmDirs() {
    Settings settings("arm", true);
    settings.SetInt("ldir", left_dir_);
    settings.SetInt("rdir", right_dir_);
}

void HeadController::SendAngles() {
    char buf[16];
    // left_ / right_ hold lift angles; the sub-board wants raw servo angles, and
    // the two arms need opposite conversions because they are mounted mirrored.
    const int left_phys = ToPhysical(left_, true);
    const int right_phys = ToPhysical(right_, false);
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
    // Clamp in lift space to whatever the servo travel allows for each arm.
    int lift_lo = 0;
    int lift_hi = 0;
    LiftRange(true, &lift_lo, &lift_hi);
    left_target = std::clamp(left_target, lift_lo, lift_hi);
    LiftRange(false, &lift_lo, &lift_hi);
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

// "Hello": lift one arm, wave it count times, drop it again. The other arm keeps
// whatever pose it already had. Lift values are mirrored by the per-arm mapping,
// so passing the same numbers makes the left arm wave symmetrically to the right.
void HeadController::Wave(int count, bool left) {
    const int other = left ? right_ : left_;
    if (left) {
        RampTo(kWaveHigh, other, kWaveStepDeg, kWaveStepMs);
        for (int i = 0; i < count; ++i) {
            RampTo(kWaveLow, other, kWaveStepDeg, kWaveStepMs);
            RampTo(kWaveHigh, other, kWaveStepDeg, kWaveStepMs);
        }
        RampTo(kArmDown, other, kWaveStepDeg, kWaveStepMs);
    } else {
        RampTo(other, kWaveHigh, kWaveStepDeg, kWaveStepMs);
        for (int i = 0; i < count; ++i) {
            RampTo(other, kWaveLow, kWaveStepDeg, kWaveStepMs);
            RampTo(other, kWaveHigh, kWaveStepDeg, kWaveStepMs);
        }
        RampTo(other, kArmDown, kWaveStepDeg, kWaveStepMs);
    }
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

        case Action::kWaveLeft:
            Wave(cmd.count, true);
            break;

        case Action::kWaveRight:
            Wave(cmd.count, false);
            break;

        case Action::kSet:  // cmd.pan/tilt carry raw servo angles
            RampTo(ToLift(cmd.tilt, true), ToLift(cmd.pan, false), kSlowStepDeg, kSlowStepMs);
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
        "self.arm.wave_left",
        "左手（左臂）挥手打招呼：抬手 → 左右挥动 count 次 → 放下。另一只手保持不动。"
        "count 默认 3。Wave the left arm hello count times, then put it down.",
        PropertyList({
            Property("count", kPropertyTypeInteger, 3, 1, 10),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const int count = properties["count"].value<int>();
            if (!Enqueue(Action::kWaveLeft, kArmDown, kArmDown, count)) {
                return std::string("Arm is busy, command dropped.");
            }
            ESP_LOGI(TAG, "wave left count=%d", count);
            return std::string("Waving with the left arm.");
        });

    mcp_server.AddTool(
        "self.arm.wave_right",
        "右手（右臂）挥手打招呼：抬手 → 左右挥动 count 次 → 放下。另一只手保持不动。"
        "count 默认 3。Wave the right arm hello count times, then put it down.",
        PropertyList({
            Property("count", kPropertyTypeInteger, 3, 1, 10),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const int count = properties["count"].value<int>();
            if (!Enqueue(Action::kWaveRight, kArmDown, kArmDown, count)) {
                return std::string("Arm is busy, command dropped.");
            }
            ESP_LOGI(TAG, "wave right count=%d", count);
            return std::string("Waving with the right arm.");
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
            int left_phys = ToPhysical(left_, true);
            int right_phys = ToPhysical(right_, false);
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
            left_zero_ = kAngleMax - zero_;
            dir_ = kDirNormal;
            left_dir_ = kArmDirLeft;
            right_dir_ = kArmDirRight;
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
        "angle 传绝对值（不传则用 delta 相对调整）；side 默认 both（左臂自动取镜像 180-angle，"
        "单独微调左臂时传 left）。设置存入 NVS，重启后保留。"
        "Set the servo angle treated as arms hanging down.",
        PropertyList({
            Property("angle", kPropertyTypeInteger, -1, -1, kAngleMax),
            Property("delta", kPropertyTypeInteger, 0, -90, 90),
            Property("side", kPropertyTypeString, std::string("both")),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const int angle = properties["angle"].value<int>();
            const int delta = properties["delta"].value<int>();
            const std::string side = properties["side"].value<std::string>();
            const bool left_only = (side == "left");
            int base = left_only ? left_zero_ : zero_;
            int next = base;
            if (angle >= 0) {
                next = angle;
            } else if (delta != 0) {
                next = base + delta;
            } else {
                return std::string("right hang ") + std::to_string(zero_) + ", left hang " +
                       std::to_string(left_zero_) + " deg.";
            }
            next = std::clamp(next, kAngleMin, kAngleMax);
            if (left_only) {
                left_zero_ = next;
                SaveLeftZero(left_zero_);
            } else {
                zero_ = next;
                left_zero_ = kAngleMax - zero_;  // keep the pair mirrored
                SaveZero(zero_);
            }
            Enqueue(Action::kSync);
            ESP_LOGI(TAG, "zero %s -> right %d, left %d", side.c_str(), zero_, left_zero_);
            return std::string("Hang pose: right ") + std::to_string(zero_) + ", left " +
                   std::to_string(left_zero_) + " deg.";
        });

    // Which way round "lift" runs. Kept apart from the hang pose so the two can
    // be fixed independently: set zero until the arms hang right, then flip this
    // once if "raise" still swings them backwards instead of forwards.
    mcp_server.AddTool(
        "self.arm.dir",
        "反转举起方向，复位位不变。dir=1 正常、dir=-1 反向；不传则直接取反。"
        "设置存入 NVS，重启后保留。Flip which way \"raise\" swings the arms.",
        PropertyList({
            Property("dir", kPropertyTypeInteger, 0, -1, 1),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const int requested = properties["dir"].value<int>();
            dir_ = (requested == 0) ? -dir_ : (requested < 0 ? kDirFlipped : kDirNormal);
            SaveDir(dir_);
            Enqueue(Action::kSync);
            ESP_LOGI(TAG, "dir -> %+d (sign %+d)", dir_, LiftSign());
            return std::string("Lift direction ") + (dir_ < 0 ? "flipped." : "normal.");
        });

    // The two servos are mounted mirrored, so one arm can end up running the
    // opposite way from the other. This flips a single arm without touching the
    // hang pose or the other side.
    mcp_server.AddTool(
        "self.arm.flip",
        "单独反转某一只手臂的上下方向（复位位不变）。side 取 left/right/both。"
        "设置存入 NVS，重启后保留。Flip one arm's up/down direction on its own.",
        PropertyList({
            Property("side", kPropertyTypeString, std::string("left")),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            const std::string side = properties["side"].value<std::string>();
            if (side == "left") {
                left_dir_ = -left_dir_;
            } else if (side == "right") {
                right_dir_ = -right_dir_;
            } else if (side == "both") {
                left_dir_ = -left_dir_;
                right_dir_ = -right_dir_;
            } else {
                return std::string("side must be left, right or both.");
            }
            SaveArmDirs();
            Enqueue(Action::kSync);
            ESP_LOGI(TAG, "flip %s -> left %+d, right %+d", side.c_str(), CombinedSign(true),
                     CombinedSign(false));
            return std::string("Arm direction flipped: left ") + (left_dir_ < 0 ? "-" : "+") +
                   ", right " + (right_dir_ < 0 ? "-" : "+") + ".";
        });
}
