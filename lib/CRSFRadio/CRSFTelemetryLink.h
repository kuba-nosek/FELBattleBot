#pragma once

#include "ITelemetryLink.h"
#include <HardwareSerial.h>
#include <stddef.h>

class CRSFTelemetryLink : public ITelemetryLink {
  public:
    // Třída dostane při inicializaci referenci na UART port
    explicit CRSFTelemetryLink(HardwareSerial& serial);

    bool sendTelemetry(const TelemetryData& data) override;

  private:
    HardwareSerial& serial_;
    uint8_t sequence_ = 0;

    static uint8_t crc8DvbS2(const uint8_t* data, size_t length);
};
