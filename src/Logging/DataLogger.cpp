#include "DataLogger.h"

#include <LittleFS.h>
#include <esp_heap_caps.h>
#include <cstring>

namespace {
constexpr char LOG_FILE[] = "/capture.csv";
constexpr char TEMP_FILE[] = "/capture.tmp";
}

bool DataLogger::begin() {
    const size_t bytes = sizeof(Sample) * MAX_SAMPLES;

    // Prefer PSRAM on ESP32-S3, with an internal-RAM fallback.
    samples_ = static_cast<Sample*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!samples_) {
        samples_ = static_cast<Sample*>(heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (!samples_) {
        Serial.printf("BBLOG: RAM allocation failed (%u bytes)\n", unsigned(bytes));
        return false;
    }

    // Never autoformat: if mount fails, send FORMAT over USB when disarmed.
    mounted_ = LittleFS.begin(false);
    if (!mounted_) {
        Serial.println("BBLOG: LittleFS not mounted. Send FORMAT over USB when disarmed.");
    }
    Serial.printf("BBLOG: buffer=%u bytes, max_samples=%u\n",
                  unsigned(bytes), unsigned(MAX_SAMPLES));
    return true;
}

void DataLogger::updateTrigger(bool receiverConnected, bool triggerHigh) {
    if (!receiverConnected) {
        triggerArmed_ = false;
        return;
    }
    if (!triggerHigh) {
        triggerArmed_ = true;
        return;
    }
    if (!triggerArmed_) return;
    triggerArmed_ = false;

    if (!samples_ || state_.load(std::memory_order_acquire) != State::Idle) return;

    count_ = 0;
    startedUs_ = micros();
    state_.store(State::Recording, std::memory_order_release);
}

void DataLogger::capture(const RobotCore& robot, const ReceiverInput& input, uint32_t nowUs) {
    if (state_.load(std::memory_order_acquire) != State::Recording) return;

    const uint32_t elapsedUs = nowUs - startedUs_; // Handles 32-bit micros() wrap.
    if (elapsedUs >= RobotConfig::LOG_DURATION_MS * 1000UL || count_ >= MAX_SAMPLES) {
        state_.store(State::PendingSave, std::memory_order_release);
        return;
    }

    const AccelData a1 = robot.state.accel1;
    const AccelData a2 = robot.state.accel2;

    Sample& sample = samples_[count_];
    sample.t_us = elapsedUs;
#define BB_SAMPLE_FIELD(name, expression) sample.name = static_cast<float>(expression);
    BB_LOG_FIELDS(BB_SAMPLE_FIELD)
#undef BB_SAMPLE_FIELD

    ++count_;
}

void DataLogger::setSafeToSave(bool safe) {
    safeToSave_.store(safe, std::memory_order_release);
}

bool DataLogger::hasUnsavedData() const {
    const State current = state_.load(std::memory_order_acquire);
    return current == State::PendingSave || current == State::Saving;
}

bool DataLogger::isBusy() const {
    return state_.load(std::memory_order_acquire) != State::Idle;
}

bool DataLogger::saveCsv() {
    if (!mounted_) return false;

    File file = LittleFS.open(TEMP_FILE, "w");
    if (!file) return false;

    // CSV schema generated from the same compile-time field list as the sample struct.
    file.print("t_us");
#define BB_WRITE_HEADER(name, expression) file.print("," #name);
    BB_LOG_FIELDS(BB_WRITE_HEADER)
#undef BB_WRITE_HEADER
    file.println();

    for (size_t i = 0; i < count_; ++i) {
        const Sample& sample = samples_[i];
        file.print(sample.t_us);
#define BB_WRITE_FIELD(name, expression) file.print(','); file.print(sample.name, 6);
        BB_LOG_FIELDS(BB_WRITE_FIELD)
#undef BB_WRITE_FIELD
        file.println();
    }

    file.flush();
    const bool writeOk = file.getWriteError() == 0;
    file.close();
    if (!writeOk) return false;

    // Preserve previous complete capture until the new file is fully written.
    if (LittleFS.exists(LOG_FILE) && !LittleFS.remove(LOG_FILE)) return false;
    return LittleFS.rename(TEMP_FILE, LOG_FILE);
}

void DataLogger::service() {
    if (!safeToSave_.load(std::memory_order_acquire)) return;

    if (mounted_ && state_.load(std::memory_order_acquire) == State::PendingSave &&
        (lastSaveAttemptMs_ == 0 || millis() - lastSaveAttemptMs_ >= 2000)) {
        lastSaveAttemptMs_ = millis();
        // Control task is forced into Idle while this state is pending/saving.
        state_.store(State::Saving, std::memory_order_release);
        const bool saved = saveCsv();
        if (saved) {
            Serial.printf("BBLOG: saved %u samples -> %s\n", unsigned(count_), LOG_FILE);
            state_.store(State::Idle, std::memory_order_release);
        } else {
            Serial.println("BBLOG: save failed; keeping samples in RAM (retrying when idle)");
            state_.store(State::PendingSave, std::memory_order_release);
        }
    }

    pollSerial();
}

void DataLogger::pollSerial() {
    while (Serial.available() > 0) {
        const int ch = Serial.read();
        if (ch == '\r' || ch == '\n') {
            if (commandLength_ == 0) continue;
            command_[commandLength_] = '\0';
            handleCommand(command_);
            commandLength_ = 0;
        } else if (ch >= 32 && ch < 127) {
            if (commandLength_ < sizeof(command_) - 1) {
                command_[commandLength_++] = static_cast<char>(ch);
            } else {
                commandLength_ = 0;
            }
        }
    }
}

void DataLogger::handleCommand(const char* command) {
    if (strcmp(command, "STATUS") == 0) {
        Serial.printf("BBLOG: state=%u mounted=%u buffer=%u count=%u file=%u\n",
                      unsigned(state_.load(std::memory_order_acquire)),
                      unsigned(mounted_), unsigned(samples_ != nullptr), unsigned(count_),
                      unsigned(mounted_ && LittleFS.exists(LOG_FILE)));
    } else if (strcmp(command, "DUMP") == 0) {
        if (state_.load(std::memory_order_acquire) != State::Idle) {
            Serial.println("BBLOG: finish saving first");
            return;
        }
        dumpLastLog();
    } else if (strcmp(command, "FORMAT") == 0) {
        if (state_.load(std::memory_order_acquire) != State::Idle) {
            Serial.println("BBLOG: cannot format while busy");
            return;
        }
        if (mounted_) LittleFS.end();
        const bool formatted = LittleFS.format();
        mounted_ = formatted && LittleFS.begin(false);
        Serial.println(mounted_ ? "BBLOG: formatted; filesystem ready" : "BBLOG: format failed");
    } else {
        Serial.println("BBLOG: commands: STATUS, DUMP, FORMAT (FORMAT erases all logs!)");
    }
}

void DataLogger::dumpLastLog() {
    if (!mounted_) {
        Serial.println("BBLOG: filesystem not mounted");
        return;
    }
    File file = LittleFS.open(LOG_FILE, "r");
    if (!file) {
        Serial.println("BBLOG: no saved capture");
        return;
    }

    const size_t bytes = file.size();
    Serial.printf("BBLOG_BEGIN %u\n", unsigned(bytes));
    uint8_t buffer[256];
    while (file.available()) {
        const size_t n = file.read(buffer, sizeof(buffer));
        if (n == 0) break;
        Serial.write(buffer, n);
    }
    file.close();
    Serial.print("\nBBLOG_END\n");
    Serial.flush();
}
