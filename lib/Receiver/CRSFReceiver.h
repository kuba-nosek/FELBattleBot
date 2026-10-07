#pragma once
#include "BattlebotTelemetry.h"
#include "IReceiver.h"
#include "ITelemetryLink.h"

#include <freertos/FreeRTOS.h>
#include <stddef.h>
#include <stdint.h>

class CRSFReceiver : public IReceiver, public ITelemetryLink {
  public:
    CRSFReceiver(uint8_t rxPin, uint8_t txPin);

    void connect() override;
    void update() override;
    bool isConnected() const override;
    void onDisconnect(DisconnectCallback callback) override;
    ReceiverChannels getChannelsSnapshot() const override;
    
    ReceiverStats getStatistics() const override;
    void sendTelemetry(const char* statusText, uint32_t currentMs) override;
    bool sendBattlebotTelemetry(const BattlebotTelemetry::TelemetryData& telemetry) override;
    TelemetryTxStats getTelemetryTxStats() const override;

  private:
    uint8_t _rxPin;
    uint8_t _txPin;

    uint32_t _lastValidFrameMs;
    uint32_t _lastTelemetryMs;
    uint8_t _telemetrySequence;
    ReceiverStats _stats;
    TelemetryTxStats _telemetryTxStats;

    bool _connected;
    DisconnectCallback _disconnectCallback;

    // --- Internal CRSF parser state ---
    uint8_t _frame[64]{};
    size_t _position;
    size_t _expectedSize;
    uint16_t _channels[ReceiverChannels::COUNT];
    mutable portMUX_TYPE _channelsMux = portMUX_INITIALIZER_UNLOCKED;

    void processByte(uint8_t byte);
};
