#pragma once
#include <stdint.h>
#include <driver/rmt.h>

class Motor {
public:
    Motor(uint8_t gpioPin, rmt_channel_t rmtChannel, bool reversed = false);
    
    bool init();
    
    // Executes mandatory DShot 3D initialization sequence
    bool arm(); 
    
    bool setSpeed(int16_t speed, uint32_t currentMs, bool requestTelemetry = false);
    void stop();
    
    void setReversed(bool reversed);
    void resetDirectionGuard();

private:
    uint8_t _gpioPin;
    rmt_channel_t _rmtChannel;
    bool _reversed;
    bool _initialized;
    
    // --- Direction Change Guard State ---
    int8_t _lastDirection;
    bool _atZero;
    uint32_t _zeroSinceMs;

    // --- Hardware Abstraction ---
    void prepareItems(uint16_t packet, rmt_item32_t* items);
    bool writeDshotPacket(uint16_t value, bool requestTelemetry);
};