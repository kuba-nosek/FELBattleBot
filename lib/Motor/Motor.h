#pragma once
#include <driver/rmt.h>
#include <stdint.h>

class Motor {
  public:
    Motor(uint8_t gpioPin, rmt_channel_t rmtChannel, bool reversed = false);

    bool init();

    // Executes mandatory DShot 3D initialization sequence[cite: 22]
    bool arm();

    bool setSpeed(int16_t speed, uint32_t currentMs, bool requestTelemetry = false);
    void stop();

    void setReversed(bool reversed);
    void resetDirectionGuard();

    int16_t getSpeed() const;

  private:
    uint8_t _gpioPin;
    rmt_channel_t _rmtChannel;
    bool _reversed;
    bool _initialized;

    // --- Current State ---
    int16_t _currentSpeed;

    // --- Direction Change Guard State ---[cite: 22]
    int8_t _lastDirection;
    bool _atZero;
    uint32_t _zeroSinceMs;

    // --- Hardware Abstraction ---[cite: 22]
    void prepareItems(uint16_t packet, rmt_item32_t* items);
    bool writeDshotPacket(uint16_t value, bool requestTelemetry);
};