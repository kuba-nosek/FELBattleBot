#pragma once

#include "ITelemetryLink.h"
#include <HardwareSerial.h>

class CRSFTelemetryLink : public ITelemetryLink {
  public:
    explicit CRSFTelemetryLink(HardwareSerial& serial);

    void connect() override;
    bool isConnected() const override;
    void onDisconnect(TelemetryDisconnectCallback callback) override;
    bool sendTelemetry(const TelemetryData& data) override;

  private:
    static uint8_t crc8DvbS2(const uint8_t* data, size_t length);

    HardwareSerial& serial_;
    uint8_t sequence_ = 0;
    TelemetryDisconnectCallback disconnectCallback_ = nullptr;
};
