#pragma once

#include "config.h"
#include <optional>
#include <stdint.h>

typedef void (*TelemetryDisconnectCallback)();

#define DECLARE_TELEMETRY_FIELD(name, type) std::optional<type> name;

struct TelemetryData {
    TELEMETRY_FIELD_MAP(DECLARE_TELEMETRY_FIELD)
};

class ITelemetryLink {
  public:
    virtual ~ITelemetryLink() = default;

    virtual void connect() = 0;
    virtual bool isConnected() const = 0;
    virtual void onDisconnect(TelemetryDisconnectCallback callback) = 0;
    virtual bool sendTelemetry(const TelemetryData& data) = 0;
};
