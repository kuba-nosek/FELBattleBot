#include "DataLogger.h"

#include <LittleFS.h>
#include <esp_heap_caps.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
constexpr char TEMP_FILE[] = "/capture.tmp";

// Numeric serial command argument, 0 means invalid/no argument.
uint32_t parsePositiveId(const char* text) {
    if (!text || !*text) return 0;
    for (const char* c = text; *c; ++c) {
        if (*c < '0' || *c > '9') return 0;
    }
    char* end = nullptr;
    const unsigned long value = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || value == 0 || value > UINT32_MAX) return 0;
    return static_cast<uint32_t>(value);
}
} // namespace

void DataLogger::fileNameFor(uint32_t id, char* path, size_t pathSize) {
    snprintf(path, pathSize, "/capture_%04lu.csv", static_cast<unsigned long>(id));
}

uint32_t DataLogger::parseLogId(const char* name) {
    if (!name) return 0;
    const char* base = strrchr(name, '/');
    base = base ? base + 1 : name;

    unsigned long id = 0;
    int consumed = 0;
    if (sscanf(base, "capture_%lu.csv%n", &id, &consumed) != 1 ||
        consumed == 0 || base[consumed] != '\0' || id == 0 || id > UINT32_MAX) {
        return 0;
    }
    return static_cast<uint32_t>(id);
}

uint32_t DataLogger::scanLogs(size_t* count) {
    if (count) *count = 0;
    if (!mounted_) return 0;

    File root = LittleFS.open("/");
    if (!root || !root.isDirectory()) return 0;

    uint32_t maxId = 0;
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            const uint32_t id = parseLogId(file.name());
            if (id != 0) {
                if (count) ++(*count);
                if (id > maxId) maxId = id;
            }
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();
    return maxId;
}

bool DataLogger::begin() {
    const size_t bytes = sizeof(Sample) * MAX_SAMPLES;
    samples_ = static_cast<Sample*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!samples_) {
        samples_ = static_cast<Sample*>(heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (!samples_) {
        Serial.printf("BBLOG: RAM allocation failed (%u bytes)\n", unsigned(bytes));
        return false;
    }

    mounted_ = LittleFS.begin(false); // Never automatically erase existing logs.
    if (mounted_) {
        latestLogId_ = scanLogs();
        nextLogId_ = latestLogId_ + 1;

        // Migrate the single-log format from previous firmware, without erasing data.
        if (LittleFS.exists("/capture.csv") && nextLogId_ != 0) {
            char newPath[40];
            fileNameFor(nextLogId_, newPath, sizeof(newPath));
            if (LittleFS.rename("/capture.csv", newPath)) {
                latestLogId_ = nextLogId_++;
                Serial.printf("BBLOG: migrated capture.csv -> %s\n", newPath);
            } else {
                Serial.println("BBLOG: warning - old /capture.csv could not be migrated");
            }
        }
    } else {
        Serial.println("BBLOG: LittleFS not mounted. Send FORMAT over USB when disarmed.");
    }
    Serial.printf("BBLOG: buffer=%u bytes, max_samples=%u, last_log=%lu\n",
                  unsigned(bytes), unsigned(MAX_SAMPLES), static_cast<unsigned long>(latestLogId_));
    return true;
}

void DataLogger::updateTrigger(bool receiverConnected, bool triggerHigh) {
    if (!receiverConnected) { triggerArmed_ = false; return; }
    if (!triggerHigh) { triggerArmed_ = true; return; }
    if (!triggerArmed_) return;
    triggerArmed_ = false;

    if (!samples_ || state_.load(std::memory_order_acquire) != State::Idle) return;
    count_ = 0;
    startedUs_ = micros();
    state_.store(State::Recording, std::memory_order_release);
}

void DataLogger::capture(const RobotCore& robot, const ReceiverInput& input, uint32_t nowUs) {
    if (state_.load(std::memory_order_acquire) != State::Recording) return;

    const uint32_t elapsedUs = nowUs - startedUs_;
    if (elapsedUs >= RobotConfig::LOG_DURATION_MS * 1000UL || count_ >= MAX_SAMPLES) {
        state_.store(State::PendingSave, std::memory_order_release);
        return;
    }

    (void)input; // Compile cleanly when LogFields does not select any RC input.
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
    if (!mounted_ || nextLogId_ == 0) return false;

    char path[40];
    fileNameFor(nextLogId_, path, sizeof(path));
    if (LittleFS.exists(path)) return false; // Never overwrite an older recording.

    File file = LittleFS.open(TEMP_FILE, "w");
    if (!file) return false;

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
    if (!writeOk || !LittleFS.rename(TEMP_FILE, path)) {
        LittleFS.remove(TEMP_FILE); // Failed/partial capture is not counted.
        return false;
    }

    latestLogId_ = nextLogId_;
    ++nextLogId_;
    return true;
}

void DataLogger::service() {
    if (!safeToSave_.load(std::memory_order_acquire)) return;

    if (mounted_ && state_.load(std::memory_order_acquire) == State::PendingSave &&
        (lastSaveAttemptMs_ == 0 || millis() - lastSaveAttemptMs_ >= 2000)) {
        lastSaveAttemptMs_ = millis();
        state_.store(State::Saving, std::memory_order_release);
        const bool saved = saveCsv();
        if (saved) {
            Serial.printf("BBLOG: saved %u samples -> capture_%04lu.csv\n",
                          unsigned(count_), static_cast<unsigned long>(latestLogId_));
            state_.store(State::Idle, std::memory_order_release);
            completedSaves_.fetch_add(1, std::memory_order_release);
        } else {
            Serial.println("BBLOG: save failed (flash full?). DELETE an old log, then retry.");
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

void DataLogger::listLogs() {
    if (!mounted_) { Serial.println("BBLOG: filesystem not mounted"); return; }

    File root = LittleFS.open("/");
    if (!root || !root.isDirectory()) {
        Serial.println("BBLOG_LIST_END");
        return;
    }
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            const uint32_t id = parseLogId(file.name());
            if (id != 0) {
                Serial.printf("BBLOG_FILE %lu %u\n",
                              static_cast<unsigned long>(id), unsigned(file.size()));
            }
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();
    Serial.println("BBLOG_LIST_END");
}

void DataLogger::dumpLog(uint32_t id) {
    if (!mounted_ || id == 0) { Serial.println("BBLOG: no saved capture"); return; }
    char path[40];
    fileNameFor(id, path, sizeof(path));
    File file = LittleFS.open(path, "r");
    if (!file) { Serial.println("BBLOG: log not found"); return; }

    Serial.printf("BBLOG_BEGIN %u\n", unsigned(file.size())); // Keeps old Python downloader compatible.
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

void DataLogger::deleteLog(uint32_t id) {
    if (!mounted_ || id == 0) { Serial.println("BBLOG: invalid id"); return; }
    char path[40];
    fileNameFor(id, path, sizeof(path));
    if (!LittleFS.exists(path)) { Serial.println("BBLOG: log not found"); return; }
    if (!LittleFS.remove(path)) { Serial.println("BBLOG: delete failed"); return; }
    latestLogId_ = scanLogs();
    Serial.printf("BBLOG: deleted capture_%04lu.csv\n", static_cast<unsigned long>(id));
}

void DataLogger::handleCommand(const char* command) {
    if (strcmp(command, "STATUS") == 0) {
        size_t logCount = 0;
        const uint32_t newest = scanLogs(&logCount);
        Serial.printf("BBLOG: state=%u mounted=%u buffer=%u count=%u files=%u latest=%lu\n",
                      unsigned(state_.load(std::memory_order_acquire)), unsigned(mounted_),
                      unsigned(samples_ != nullptr), unsigned(count_), unsigned(logCount),
                      static_cast<unsigned long>(newest));
    } else if (strcmp(command, "LIST") == 0) {
        listLogs();
    } else if (strcmp(command, "DUMP") == 0 || strncmp(command, "DUMP ", 5) == 0) {
        if (isBusy()) { Serial.println("BBLOG: finish saving first"); return; }
        const uint32_t id = strcmp(command, "DUMP") == 0 ? latestLogId_ : parsePositiveId(command + 5);
        dumpLog(id);
    } else if (strncmp(command, "DELETE ", 7) == 0) {
        // Also permitted in PendingSave so an old capture can be removed when flash is full.
        if (state_.load(std::memory_order_acquire) == State::Recording) {
            Serial.println("BBLOG: stop recording first"); return;
        }
        deleteLog(parsePositiveId(command + 7));
    } else if (strcmp(command, "FORMAT") == 0) {
        if (isBusy()) { Serial.println("BBLOG: cannot format while busy"); return; }
        if (mounted_) LittleFS.end();
        mounted_ = false;
        const bool formatted = LittleFS.format();
        mounted_ = formatted && LittleFS.begin(false);
        if (mounted_) { nextLogId_ = 1; latestLogId_ = 0; }
        Serial.println(mounted_ ? "BBLOG: formatted; filesystem ready" : "BBLOG: format failed");
    } else {
        Serial.println("BBLOG: commands: STATUS, LIST, DUMP [id], DELETE <id>, FORMAT");
    }
}
