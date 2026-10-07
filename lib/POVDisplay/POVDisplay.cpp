#include "POVDisplay.h"

#include "LEDStripMath.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <hal/cpu_hal.h>
#include <iterator>
#include <soc/gpio_reg.h>
#include <soc/soc.h>

namespace {
uint8_t scaleColorChannel(uint8_t channel) {
    const uint16_t scaled = static_cast<uint16_t>(channel) * RobotConfig::LED_STRIP_BRIGHTNESS + 127;
    return static_cast<uint8_t>(scaled / 255);
}

uint32_t scaleColor(uint32_t color) {
    const uint8_t red = scaleColorChannel(static_cast<uint8_t>(color >> 16));
    const uint8_t green = scaleColorChannel(static_cast<uint8_t>(color >> 8));
    const uint8_t blue = scaleColorChannel(static_cast<uint8_t>(color));
    return (static_cast<uint32_t>(red) << 16) | (static_cast<uint32_t>(green) << 8) | blue;
}
} // namespace

POVDisplay::POVDisplay(uint8_t pin)
    : _pin(pin), _mode(POVDisplayMode::Static), _animation(LEDStripAnimation::Off),
      _angularVelocityRadPerSec(0.0f), _modeChanged(true), _animationChanged(true),
      _angleRadians(0.0f), _lastIntegrationUs(0), _lastRefreshUs(0), _animationStartMs(0), _frameSent(false),
      _lastTransmittedMode(POVDisplayMode::Static), _pixels{}, _lastPixels{} {}

void POVDisplay::init() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
}

void POVDisplay::setMode(POVDisplayMode mode) {
    portENTER_CRITICAL(&_stateLock);
    if(_mode != mode) {
        _mode = mode;
        _modeChanged = true;
    }
    portEXIT_CRITICAL(&_stateLock);
}

void POVDisplay::setAnimation(LEDStripAnimation animation) {
    portENTER_CRITICAL(&_stateLock);
    if(_animation != animation) {
        _animation = animation;
        _animationChanged = true;
    }
    portEXIT_CRITICAL(&_stateLock);
}

void POVDisplay::setAngularVelocity(float radiansPerSecond) {
    portENTER_CRITICAL(&_stateLock);
    _angularVelocityRadPerSec = std::isfinite(radiansPerSecond) ? radiansPerSecond : 0.0f;
    portEXIT_CRITICAL(&_stateLock);
}

void POVDisplay::update() {
    const uint32_t currentMs = millis();
    const uint32_t currentUs = micros();

    POVDisplayMode mode;
    LEDStripAnimation animation;
    float angularVelocityRadPerSec;
    bool modeChanged;
    bool animationChanged;

    portENTER_CRITICAL(&_stateLock);
    mode = _mode;
    animation = _animation;
    angularVelocityRadPerSec = _angularVelocityRadPerSec;
    modeChanged = _modeChanged;
    animationChanged = _animationChanged;
    _modeChanged = false;
    _animationChanged = false;
    portEXIT_CRITICAL(&_stateLock);

    const LEDStripMath::IntegrationState integration = LEDStripMath::updateIntegration(
        _angleRadians, _lastIntegrationUs, currentUs, angularVelocityRadPerSec, modeChanged);
    _angleRadians = integration.angleRadians;
    _lastIntegrationUs = integration.lastUpdateUs;

    if(animationChanged) _animationStartMs = currentMs;

    constexpr uint32_t REFRESH_INTERVAL_US = 1000000UL / RobotConfig::LED_STRIP_REFRESH_HZ;
    if(!LEDStripMath::shouldRefresh(currentUs, _lastRefreshUs, REFRESH_INTERVAL_US, _frameSent)) return;
    _lastRefreshUs = currentUs;

    if(mode == POVDisplayMode::Spinning) {
        renderSpinningStrip(animation);
    } else {
        renderStaticStrip(animation, currentMs - _animationStartMs);
    }

    applyBrightness();
    if(!LEDStripMath::shouldTransmit(_frameSent, pixelsChanged())) return;

    const bool spinning = mode == POVDisplayMode::Spinning;
    const bool lastTransmissionWasSpinning = _lastTransmittedMode == POVDisplayMode::Spinning;
    const uint16_t pixelCount = LEDStripMath::transmissionPixelCount(
        spinning, lastTransmissionWasSpinning, RobotConfig::LED_STRIP_LED_COUNT, RobotConfig::LED_STRIP_SPIN_LED_COUNT);

    transmitFrame(pixelCount);
    rememberPixels();
    _frameSent = true;
    _lastTransmittedMode = mode;
}

void POVDisplay::renderStaticStrip(LEDStripAnimation animation, uint32_t elapsedMs) {
    uint32_t color = 0;

    switch(animation) {
        case LEDStripAnimation::Bootup: {
            if(elapsedMs >= 2000) {
                const uint16_t idlePhase = elapsedMs % 2000;
                const uint8_t idleBrightness =
                    idlePhase < 1000 ? idlePhase * 96 / 1000 : (2000 - idlePhase) * 96 / 1000;
                color = static_cast<uint32_t>(idleBrightness) << 8 | idleBrightness;
                break;
            }

            std::fill(std::begin(_pixels), std::end(_pixels), 0);
            const uint16_t activePixel = (elapsedMs / 80) % RobotConfig::LED_STRIP_LED_COUNT;
            _pixels[activePixel] = 0x0080FF;
            return;
        }
        case LEDStripAnimation::Idle: {
            const uint16_t phase = elapsedMs % 2000;
            const uint8_t brightness = phase < 1000 ? phase * 96 / 1000 : (2000 - phase) * 96 / 1000;
            color = static_cast<uint32_t>(brightness) << 8 | brightness;
            break;
        }
        case LEDStripAnimation::Forward:
            color = 0x00FF00;
            break;
        case LEDStripAnimation::Failsafe:
            color = elapsedMs % 500 < 250 ? 0xFF0000 : 0;
            break;
        case LEDStripAnimation::LowBattery:
            color = elapsedMs % 1000 < 100 ? 0xFF6000 : 0;
            break;
        case LEDStripAnimation::HardwareError:
            color = elapsedMs % 60 < 30 ? 0xFF0000 : 0;
            break;
        default:
            color = 0;
            break;
    }

    std::fill(std::begin(_pixels), std::end(_pixels), color);
}

void POVDisplay::renderSpinningStrip(LEDStripAnimation animation) {
    const uint16_t sector = LEDStripMath::angleToSector(_angleRadians, RobotConfig::LED_STRIP_SECTOR_COUNT);
    const uint32_t* sectorPixels = LEDStripImages::getSectorPixels(animation, sector);

    std::fill(std::begin(_pixels), std::end(_pixels), 0);

    if(sectorPixels == nullptr) return;

    std::copy_n(sectorPixels, RobotConfig::LED_STRIP_SPIN_LED_COUNT, _pixels);
}

void POVDisplay::applyBrightness() {
    for(uint32_t& pixel : _pixels) {
        pixel = scaleColor(pixel);
    }
}

bool POVDisplay::pixelsChanged() const {
    return std::memcmp(_pixels, _lastPixels, sizeof(_pixels)) != 0;
}

void POVDisplay::rememberPixels() {
    std::memcpy(_lastPixels, _pixels, sizeof(_pixels));
}

void IRAM_ATTR POVDisplay::transmitFrame(uint16_t pixelCount) {
    static_assert(RobotConfig::PIN_LED_STRIP < 32, "Software strip output only supports GPIOs 0..31");

    const uint32_t pinMask = 1UL << _pin;
    const uint32_t cyclesPerMicrosecond = getCpuFrequencyMhz();
    const uint32_t zeroHighCycles = cyclesPerMicrosecond * 35 / 100;
    const uint32_t oneHighCycles = cyclesPerMicrosecond * 70 / 100;
    const uint32_t bitCycles = cyclesPerMicrosecond * 125 / 100;

    portENTER_CRITICAL(&_outputLock);
    for(uint16_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
        const uint32_t pixel = _pixels[pixelIndex];
        const uint32_t red = (pixel >> 16) & 0xFF;
        const uint32_t green = (pixel >> 8) & 0xFF;
        const uint32_t blue = pixel & 0xFF;
        const uint32_t grb = (green << 16) | (red << 8) | blue;

        for(int8_t bit = 23; bit >= 0; --bit) {
            const uint32_t highCycles = grb & (1UL << bit) ? oneHighCycles : zeroHighCycles;
            const uint32_t startCycle = cpu_hal_get_cycle_count();
            REG_WRITE(GPIO_OUT_W1TS_REG, pinMask);
            while(cpu_hal_get_cycle_count() - startCycle < highCycles) {}
            REG_WRITE(GPIO_OUT_W1TC_REG, pinMask);
            while(cpu_hal_get_cycle_count() - startCycle < bitCycles) {}
        }
    }
    portEXIT_CRITICAL(&_outputLock);
}
