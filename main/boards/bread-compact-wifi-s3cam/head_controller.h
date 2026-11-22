#ifndef __HEAD_CONTROLLER_H__
#define __HEAD_CONTROLLER_H__

#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <string>

// Drives the two servos on the sub-board (ESP32-S3 + two SG90).
//
// The link is one-wire UART at 115200 8N1, one ASCII command per line:
//   shake=<0-180>   right arm angle
//   nod=<0-180>     left arm angle
//   center          both servos to 90 degrees (not used in arm mode)
//
// ARM DEMO MODE (temporary): the pan/tilt head has been re-mounted as a pair of
// arms for a children demo. The sub-board firmware is untouched, so the two
// channels keep their old names: "shake" drives the right arm, "nod" drives the
// left one.
//
// Angles are split in two: the driver keeps a "lift" angle where 0 means the arm
// hangs straight down and positive means lifting it, while the servo wants a raw
// 0-180 angle. ArmDownPhys() / LiftSign() bridge the two; see their comment for
// the bench measurement and the mechanical fix that makes lifting go forward.
//
// Two things matter here:
//   * nothing blocks the MCP callback: commands are queued to a dedicated task;
//   * long moves are interpolated in 1-3 degree steps, so a long arm glides
//     instead of snapping (a single jump whips the arm and stalls the gearbox).
class HeadController {
public:
    HeadController(uart_port_t uart_num, gpio_num_t tx_pin);

    // Greeting gesture fired from the wake-word path: raise the right arm, wave
    // three times, put it back down. Returns immediately; motion is queued.
    void WaveOnWake();

private:
    enum class Action {
        kHome,        // both arms back down
        kLook,        // absolute angles (pan = right arm, tilt = left arm)
        kRaiseLeft,
        kLowerLeft,
        kRaiseRight,
        kLowerRight,
        kDance,       // arms alternate up/down N times, then home
        kWave,        // raise right arm, wave N times, then home
        kSet,         // direct servo angles, for bench calibration
        kSync,        // re-send the hanging pose after a mounting change
    };

    struct Command {
        Action action;
        int pan;
        int tilt;
        int count;
    };

    static constexpr int kAngleMin = 0;
    static constexpr int kAngleMax = 180;

    // Servo zero point / mounting direction.
    //
    // Measured on the robot as shipped: commanding 0 puts the arm straight
    // down, 90 points it horizontally *backwards* and 180 lifts it straight up
    // behind the body. The reachable half of the circle is the back half, so no
    // servo angle can reach forward - the arm has to be re-mounted by 180
    // degrees first, and these servos' horns are press-fitted, so depending on
    // what could actually be turned there are two ways to get there:
    //
    //   kMountFront - the horn (or the arm extension bolted to it) was turned
    //                 180 degrees on the spline. Travel: 0 = up, 90 = forward,
    //                 180 = hanging down. Lifting walks *down* the scale.
    //   kMountBack  - the whole servo body was turned 180 degrees around its own
    //                 output shaft instead. Same sweep, reached from the other
    //                 end: 0 = hanging down, 90 = backwards, 180 = up.
    //
    // Both end up identical from the outside once the matching change is made,
    // so the mapping is a runtime switch (self.arm.mount) saved in NVS rather
    // than a rebuild, because the mechanical side is trial and error.
    static constexpr int kMountFront = 0;
    static constexpr int kMountBack = 1;

    int mount_ = kMountFront;

    int ArmDownPhys() const { return mount_ == kMountFront ? 180 : 0; }
    int LiftSign() const { return mount_ == kMountFront ? -1 : 1; }

    // Lift angles (relative to hanging down).
    static constexpr int kArmDown = 0;      // hanging down, natural pose
    static constexpr int kArmRaise = 95;    // arm raised forward, a bit above level
    static constexpr int kWaveLow = 55;     // wave swing, lower end
    static constexpr int kWaveHigh = 80;    // wave swing, upper end
    static constexpr int kDanceLow = 20;    // dance swing, lower end
    static constexpr int kDanceHigh = 110;  // dance swing, upper end

    // Speed is set by step size / step period, not by the sub-board.
    static constexpr int kSlowStepDeg = 1;      // ~55 deg/s, calm gestures
    static constexpr int kSlowStepMs = 18;
    static constexpr int kWaveStepDeg = 2;      // ~85 deg/s, hand waving
    static constexpr int kWaveStepMs = 24;
    static constexpr int kDanceStepDeg = 2;     // ~125 deg/s, dance beat
    static constexpr int kDanceStepMs = 16;

    uart_port_t uart_num_;
    QueueHandle_t queue_ = nullptr;
    int left_ = kArmDown;    // nod channel
    int right_ = kArmDown;   // shake channel

    void RegisterTools();
    bool Enqueue(Action action, int pan = kArmDown, int tilt = kArmDown, int count = 1);
    void SendLine(const std::string& line);
    void SendAngles();
    int ToPhysical(int lift) const;
    int ToLift(int physical) const;
    void LoadMount();
    void SaveMount(int mount);
    void RampTo(int left_target, int right_target, int step_deg, int step_ms);
    void Dance(int count);
    void Wave(int count);
    void Execute(const Command& cmd);
    static void TaskEntry(void* arg);
    void Run();
};

#endif  // __HEAD_CONTROLLER_H__
