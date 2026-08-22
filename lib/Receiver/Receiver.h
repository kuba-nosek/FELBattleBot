#pragma once
#include <stdint.h>
#include <stddef.h>

// Signal loss callback signature
typedef void (*DisconnectCallback)();

struct ReceiverStats {
    uint8_t linkQuality;
    int16_t activeRssiDbm;
};

class Receiver {
public:
    Receiver(uint8_t rxPin, uint8_t txPin);
    
    void connect(); 
    void update(uint32_t currentMs); 
    
    bool isConnected() const;
    void onDisconnect(DisconnectCallback callback);
    
    uint16_t getChannel(size_t index) const;
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
    uint16_t _channels[16];
    
    void processByte(uint8_t byte);
};