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
// arms for a children demo. 0 degrees is the arm hanging down, larger angles
// lift it. The sub-board firmware is untouched, so the two channels keep their
// old names: "shake" drives the right arm, "nod" drives the left one.
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
    };

    struct Command {
        Action action;
        int pan;
        int tilt;
        int count;
    };

    static constexpr int kAngleMin = 0;
    static constexpr int kAngleMax = 180;
    static constexpr int kArmDown = 0;      // hanging down, natural pose
    static constexpr int kArmRaise = 100;   // "arm raised" pose
    static constexpr int kWaveLow = 60;     // wave swing, lower end
    static constexpr int kWaveHigh = 95;    // wave swing, upper end
    static constexpr int kDanceLow = 10;    // dance swing, lower end
    static constexpr int kDanceHigh = 80;   // dance swing, upper end

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
    void RampTo(int left_target, int right_target, int step_deg, int step_ms);
    void Dance(int count);
    void Wave(int count);
    void Execute(const Command& cmd);
    static void TaskEntry(void* arg);
    void Run();
};

#endif  // __HEAD_CONTROLLER_H__
