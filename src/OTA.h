#pragma once

#include <stdint.h>

enum class OTAStatus : uint8_t {
    Starting,
    Waiting,
    Connected,
    Updating,
    Finished,
    Error,
};

struct OTACallbacks {
    void (*onStatus)(OTAStatus) = nullptr;
    bool (*shouldExit)() = nullptr;
};

[[noreturn]] void enterOtaMode(const OTACallbacks& callbacks);
