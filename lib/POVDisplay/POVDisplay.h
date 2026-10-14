#pragma once

#include "GeneratedLEDStripImages.h"
#include "config.h"

#include <freertos/FreeRTOS.h>
#include <stdint.h>
#include <Adafruit_NeoPixel.h>

enum class POVDisplayMode : uint8_t { Static, Spinning };

class POVDisplay {
  public:
    explicit POVDisplay(uint8_t pin);

    void init();
    void setMode(POVDisplayMode mode);
    void setAnimation(LEDStripAnimation animation);
    void setAngularVelocity(float radiansPerSecond);
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

    Adafruit_NeoPixel _strip;
    portMUX_TYPE _stateLock = portMUX_INITIALIZER_UNLOCKED;

    void renderStaticStrip(LEDStripAnimation animation, uint32_t elapsedMs);
    void renderSpinningStrip(LEDStripAnimation animation);
    void applyBrightness();
    bool pixelsChanged() const;
    void rememberPixels();
    void transmitFrame();
};
