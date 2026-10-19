#pragma once

#include "LogFields.h"
#include "IRobot.h"

#include <Arduino.h>
#include <atomic>
#include <stddef.h>
#include <stdint.h>

class DataLogger {
public:
    bool begin();

    // Called only by the existing 200 Hz ControlLoop task.
    void updateTrigger(bool receiverConnected, bool triggerHigh);
    void capture(const RobotCore& robot, const ReceiverInput& input, uint32_t nowUs);
    void setSafeToSave(bool safe);

    // Called only by a low-priority task. Never from ControlLoop.
    void service();

    bool hasUnsavedData() const;
    bool isBusy() const;

private:
    enum class State : uint8_t { Idle, Recording, PendingSave, Saving };

    struct Sample {
        uint32_t t_us = 0;
#define BB_DECLARE_FIELD(name, expression) float name = 0.0f;
        BB_LOG_FIELDS(BB_DECLARE_FIELD)
#undef BB_DECLARE_FIELD
    };

    static constexpr size_t MAX_SAMPLES =
        (static_cast<size_t>(RobotConfig::LOG_DURATION_MS) * RobotConfig::TASK_CONTROL_HZ + 999) / 1000 + 4;

    static_assert(RobotConfig::LOG_DURATION_MS > 0, "Logging duration must be positive");
    static_assert(MAX_SAMPLES >= 2, "Invalid logger sample capacity");

    std::atomic<State> state_{State::Idle};
    std::atomic<bool> safeToSave_{false};

    // Sample buffer belongs to the control task until state becomes PendingSave.
    Sample* samples_ = nullptr;
    size_t count_ = 0;
    uint32_t startedUs_ = 0;
    uint32_t lastSaveAttemptMs_ = 0;
    bool triggerArmed_ = false; // Require a LOW before the first HIGH.
    bool mounted_ = false;

    // USB console state belongs exclusively to the service task.
    char command_[20]{};
    size_t commandLength_ = 0;

    bool saveCsv();
    void pollSerial();
    void handleCommand(const char* command);
    void dumpLastLog();
};
