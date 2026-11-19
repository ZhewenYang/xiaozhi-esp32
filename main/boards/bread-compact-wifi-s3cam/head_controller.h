#ifndef __HEAD_CONTROLLER_H__
#define __HEAD_CONTROLLER_H__

#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <string>

// Drives the pan/tilt head that lives on the sub-board (ESP32-S3 + two SG90).
//
// The link is one-wire UART at 115200 8N1, one ASCII command per line:
//   shake=<0-180>   pan angle
//   nod=<0-180>     tilt angle
//   center          both servos back to 90 degrees
//
// A servo sweep takes hundreds of milliseconds, so nothing here blocks the MCP
// callback: commands are queued and executed by a dedicated task. That keeps
// the voice loop and the camera worker responsive while the head swings.
class HeadController {
public:
    HeadController(uart_port_t uart_num, gpio_num_t tx_pin);

private:
    enum class Action { kLook, kCenter, kNod, kShake };

    struct Command {
        Action action;
        int pan;
        int tilt;
        int count;
    };

    static constexpr int kAngleMin = 0;
    static constexpr int kAngleMax = 180;
    static constexpr int kHomeAngle = 90;
    static constexpr int kSwingAmplitude = 18;
    static constexpr uint32_t kSwingStepMs = 320;

    uart_port_t uart_num_;
    QueueHandle_t queue_ = nullptr;
    int pan_ = kHomeAngle;
    int tilt_ = kHomeAngle;

    void RegisterTools();
    bool Enqueue(Action action, int pan, int tilt, int count);
    void SendLine(const std::string& line);
    void SendAngles();
    void Execute(const Command& cmd);
    static void TaskEntry(void* arg);
    void Run();
};

#endif  // __HEAD_CONTROLLER_H__
