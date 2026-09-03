#pragma once
#include <stdint.h>
#include <stddef.h>
#include <freertos/FreeRTOS.h>
#include "BattlebotTelemetry.h"

// Signal loss callback signature
typedef void (*DisconnectCallback)();

struct ReceiverStats {
    uint8_t linkQuality = 0;
    int16_t activeRssiDbm = 0;
};

struct ReceiverChannels {
    static constexpr size_t COUNT = 16;

    uint16_t channelsUs[COUNT]{};
};

struct TelemetryTxStats {
    uint32_t sentPackets = 0;
    uint32_t skippedWrites = 0;
    uint32_t partialWrites = 0;
};

class Receiver {
public:
    Receiver(uint8_t rxPin, uint8_t txPin);
    
    void connect(); 
    void update(); 
    
    bool isConnected() const;
    void onDisconnect(DisconnectCallback callback);
    
    ReceiverChannels getChannelsSnapshot() const;
    ReceiverStats getStatistics() const;
    void sendTelemetry(const char* statusText, uint32_t currentMs);
    bool sendBattlebotTelemetry(
        const BattlebotTelemetry::Snapshot& snapshot,
        uint32_t currentMs,
        uint32_t intervalMs);
    TelemetryTxStats getTelemetryTxStats() const;

private:
    uint8_t _rxPin;
    uint8_t _txPin;
    
    uint32_t _lastValidFrameMs;
    uint32_t _lastTelemetryMs;
    uint32_t _lastBattlebotTelemetryMs;
    uint8_t _telemetrySequence;
    ReceiverStats _stats;
    TelemetryTxStats _telemetryTxStats;
    
    bool _connected;
    DisconnectCallback _disconnectCallback;
    
    // --- Internal CRSF parser state ---
    uint8_t _frame[64];
    size_t _position;
    size_t _expectedSize;
    uint16_t _channels[ReceiverChannels::COUNT];
    mutable portMUX_TYPE _channelsMux = portMUX_INITIALIZER_UNLOCKED;
    
    void processByte(uint8_t byte);
};
