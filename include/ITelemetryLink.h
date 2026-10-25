#pragma once

#include "config.h"
#include <optional>
#include <stdint.h>

#define DECLARE_TELEMETRY_FIELD(name, type) std::optional<type> name;


struct TelemetryData {
    TELEMETRY_FIELD_MAP(DECLARE_TELEMETRY_FIELD)
};


class ITelemetryLink {
  public:
    virtual ~ITelemetryLink() = default;

    virtual bool sendTelemetry(const TelemetryData& data) = 0;
};
