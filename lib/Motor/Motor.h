#pragma once
#include <stdint.h>
#include <driver/rmt.h>

class Motor {
public:
    Motor(uint8_t gpioPin, rmt_channel_t rmtChannel, bool reversed = false);
    
    bool init();
    bool arm(); // Provedení inicializační/armovací DShot sekvence
    bool setSpeed(int16_t speed, uint32_t currentMs, bool requestTelemetry = false);
    void stop();
    
    void setReversed(bool reversed);
    void resetDirectionGuard();

private:
    uint8_t _gpioPin;
    rmt_channel_t _rmtChannel;
    bool _reversed;
    bool _initialized;
    
    // Proměnné pro Direction Change Guard
    int8_t _lastDirection;
    bool _atZero;
    uint32_t _zeroSinceMs;

    void prepareItems(uint16_t packet, rmt_item32_t* items);
    bool writeDshotPacket(uint16_t value, bool requestTelemetry);
};