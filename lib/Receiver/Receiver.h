#pragma once
#include <stdint.h>
#include <stddef.h>
#include <freertos/FreeRTOS.h>

// Signal loss callback signature
typedef void (*DisconnectCallback)();

struct ReceiverStats {
    uint8_t linkQuality;
    int16_t activeRssiDbm;
};

struct ReceiverChannels {
    static constexpr size_t COUNT = 16;

    uint16_t channelsUs[COUNT]{};
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

private:
    uint8_t _rxPin;
    uint8_t _txPin;
    
    uint32_t _lastValidFrameMs;
    uint32_t _lastTelemetryMs;
    ReceiverStats _stats;
    
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
