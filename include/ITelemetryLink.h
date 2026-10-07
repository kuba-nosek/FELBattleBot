#pragma once
#include "BattlebotTelemetry.h"
#include <stdint.h>

struct ReceiverStats {
    uint8_t linkQuality = 0;
    int16_t activeRssiDbm = 0;
};

struct TelemetryTxStats {
    uint32_t sentPackets = 0;
    uint32_t skippedWrites = 0;
    uint32_t partialWrites = 0;
};

class ITelemetryLink {
  public:
    virtual ~ITelemetryLink() = default;

    virtual ReceiverStats getStatistics() const = 0;
    virtual void sendTelemetry(const char* statusText, uint32_t currentMs) = 0;
    virtual bool sendBattlebotTelemetry(const BattlebotTelemetry::TelemetryData& telemetry) = 0;
    virtual TelemetryTxStats getTelemetryTxStats() const = 0;
};
