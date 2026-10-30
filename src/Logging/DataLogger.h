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

    // ControlLoop: acquisition only, never write flash here.
    void updateTrigger(bool receiverConnected, bool triggerHigh);
    void capture(const RobotCore& robot, const ReceiverInput& input, uint32_t nowUs);
    void setSafeToSave(bool safe);

    // LoggerStorageThread only.
    void service();

    bool hasUnsavedData() const;
    bool isBusy() const;

    // ControlLoop polls this counter to trigger a one-shot LED confirmation.
    uint32_t completedSaves() const {
        return completedSaves_.load(std::memory_order_acquire);
    }

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
    std::atomic<uint32_t> completedSaves_{0};

    Sample* samples_ = nullptr;
    size_t count_ = 0;
    uint32_t startedUs_ = 0;
    uint32_t lastSaveAttemptMs_ = 0;
    bool triggerArmed_ = false;
    bool mounted_ = false;

    // Accessed only from setup() or LoggerStorageThread.
    uint32_t nextLogId_ = 1;
    uint32_t latestLogId_ = 0;

    char command_[32]{};
    size_t commandLength_ = 0;

    static void fileNameFor(uint32_t id, char* path, size_t pathSize);
    static uint32_t parseLogId(const char* name);
    uint32_t scanLogs(size_t* count = nullptr);

    bool saveCsv();
    void pollSerial();
    void handleCommand(const char* command);
    void listLogs();
    void dumpLog(uint32_t id);
    void deleteLog(uint32_t id);
};
