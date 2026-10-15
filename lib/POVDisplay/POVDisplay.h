#pragma once

#include "GeneratedLEDStripImages.h"
#include "config.h"

#include <freertos/FreeRTOS.h>
#include <stdint.h>

enum class POVDisplayMode : uint8_t { Static, Spinning };

class POVDisplay {
  public:
    explicit POVDisplay(uint8_t pin);

    void init();

    void setMode(POVDisplayMode mode);
    void setAnimation(LEDStripAnimation animation);
    void setAngularVelocity(float radiansPerSecond);

    // Must be called cyclically in the dedicated LED thread.
    void update();

  private:
    uint8_t _pin;

    POVDisplayMode _mode;
    LEDStripAnimation _animation;
    float _angularVelocityRadPerSec;
    bool _modeChanged;
    bool _animationChanged;

    float _angleRadians;
    uint32_t _lastIntegrationUs;
    uint32_t _lastRefreshUs;
    uint32_t _animationStartMs;
    bool _frameSent;
    POVDisplayMode _lastTransmittedMode;
    uint32_t _pixels[RobotConfig::LED_STRIP_LED_COUNT];
    uint32_t _lastPixels[RobotConfig::LED_STRIP_LED_COUNT];

    portMUX_TYPE _stateLock = portMUX_INITIALIZER_UNLOCKED;
    portMUX_TYPE _outputLock = portMUX_INITIALIZER_UNLOCKED;

    void renderStaticStrip(LEDStripAnimation animation, uint32_t elapsedMs);
    void renderSpinningStrip(LEDStripAnimation animation);
    void applyBrightness();
    bool pixelsChanged() const;
    void rememberPixels();
    void transmitFrame(uint16_t pixelCount);
};
