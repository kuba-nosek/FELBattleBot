#pragma once

#include "BattlebotTelemetry.h"
#include "IReceiver.h"

#include <freertos/FreeRTOS.h>
#include <stddef.h>
#include <stdint.h>

// Statistiky přesunuty z původního rozhraní
struct ReceiverStats {
    uint8_t linkQuality = 0;
    int16_t activeRssiDbm = 0;
};

struct TelemetryTxStats {
    uint32_t sentPackets = 0;
    uint32_t skippedWrites = 0;
    uint32_t partialWrites = 0;
};

class CRSFReceiver : public IReceiver {
  public:
    CRSFReceiver(uint8_t rxPin, uint8_t txPin);

    void connect() override;
    void update() override;
    bool isConnected() const override;
    void onDisconnect(DisconnectCallback callback) override;
    ReceiverChannels getChannelsSnapshot() const override;
    
    ReceiverStats getStatistics() const;
    void sendTelemetry(const char* statusText, uint32_t currentMs);
    bool sendBattlebotTelemetry(const BattlebotTelemetry::TelemetryData& telemetry);
    TelemetryTxStats getTelemetryTxStats() const;

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
