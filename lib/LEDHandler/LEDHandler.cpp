#include "LEDHandler.h"

#include "LEDStripMath.h"
#include "config.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <hal/cpu_hal.h>
#include <iterator>
#include <soc/gpio_reg.h>
#include <soc/soc.h>

namespace {
bool hasReached(uint32_t currentUs, uint32_t deadlineUs) {
    return static_cast<int32_t>(currentUs - deadlineUs) >= 0;
}

bool isEarlier(uint32_t firstUs, uint32_t secondUs) {
    return static_cast<int32_t>(firstUs - secondUs) < 0;
}

bool isFlashValid(uint32_t startTimeFromNowUs, uint32_t flashDurationUs, uint32_t minimumTimeBetweenFlashesUs) {
    if(flashDurationUs == 0) return false;
    if(startTimeFromNowUs > INT32_MAX) return false;
    if(flashDurationUs > INT32_MAX) return false;

    const uint32_t maximumGapUs = static_cast<uint32_t>(INT32_MAX) - flashDurationUs;
    return minimumTimeBetweenFlashesUs <= maximumGapUs;
}

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

LEDHandler::LEDHandler(uint8_t ledPin)
    : _ledPin(ledPin), _currentIndication(LEDIndication::Off), _currentAnimation(LEDAnimation::None),
      _animationActive(false), _animationStartMs(0), _animationDurationMs(0), _lastToggleMs(0), _ledState(false),
      _stepCounter(0), _flashRunning(false), _flashEndUs(0), _flashPending(false), _pendingFlashStartUs(0),
      _pendingFlashDurationUs(0), _pendingMinimumGapUs(0), _hasLastFlashEnd(false), _lastFlashEndUs(0),
      _stripMode(LEDStripMode::Static), _stripAnimation(LEDStripAnimation::Off), _stripAngularVelocityRadPerSec(0.0f),
      _stripModeChanged(true), _stripAnimationChanged(true), _stripAngleRadians(0.0f), _lastStripIntegrationUs(0),
      _lastStripRefreshUs(0), _stripAnimationStartMs(0), _stripFrameSent(false), _stripPixels{}, _lastStripPixels{} {}

void LEDHandler::init() {
    pinMode(_ledPin, OUTPUT);
    digitalWrite(_ledPin, LOW);

    if(RobotConfig::USE_LED_STRIP) {
        pinMode(RobotConfig::PIN_LED_STRIP, OUTPUT);
        digitalWrite(RobotConfig::PIN_LED_STRIP, LOW);
    }
}

void LEDHandler::setIndication(LEDIndication mode) {
    if(_currentIndication == mode) return;

    _currentIndication = mode;

    portENTER_CRITICAL(&_flashStateLock);
    _flashRunning = false;
    _flashPending = false;
    _hasLastFlashEnd = false;
    portEXIT_CRITICAL(&_flashStateLock);
}

void LEDHandler::playAnimation(LEDAnimation mode) {
    _currentAnimation = mode;
    _animationActive = true;
    _animationStartMs = millis();

    switch(mode) {
        case LEDAnimation::Bootup:
            _animationDurationMs = 2000;
            break;
        case LEDAnimation::ModeChanged:
            _animationDurationMs = 500;
            break;
        case LEDAnimation::ErrorAlert:
            _animationDurationMs = 3000;
            break;
        case LEDAnimation::TelemetrySent:
            _animationDurationMs = 100;
            break;
        default:
            _animationDurationMs = 1000;
            break;
    }
}

void LEDHandler::scheduleFlash(uint32_t startTimeFromNowUs, uint32_t flashDurationUs,
                               uint32_t minimumTimeBetweenFlashesUs) {
    if(!isFlashValid(startTimeFromNowUs, flashDurationUs, minimumTimeBetweenFlashesUs)) return;

    const uint32_t currentUs = micros();
    const uint32_t flashStartUs = currentUs + startTimeFromNowUs;
    bool shouldSchedule = true;

    portENTER_CRITICAL(&_flashStateLock);
    if(_flashRunning) {
        const uint32_t earliestStartUs = _flashEndUs + minimumTimeBetweenFlashesUs;

        shouldSchedule = !isEarlier(flashStartUs, earliestStartUs);
    } else if(_hasLastFlashEnd) {
        const uint32_t earliestStartUs = _lastFlashEndUs + minimumTimeBetweenFlashesUs;

        if(!hasReached(currentUs, earliestStartUs) && isEarlier(flashStartUs, earliestStartUs)) {
            shouldSchedule = false;
        }
    }

    if(shouldSchedule && _flashPending && !isEarlier(flashStartUs, _pendingFlashStartUs)) {
        shouldSchedule = false;
    }

    if(shouldSchedule) {
        _pendingFlashStartUs = flashStartUs;
        _pendingFlashDurationUs = flashDurationUs;
        _pendingMinimumGapUs = minimumTimeBetweenFlashesUs;
        _flashPending = true;
    }
    portEXIT_CRITICAL(&_flashStateLock);
}

void LEDHandler::cancelScheduledFlash() {
    portENTER_CRITICAL(&_flashStateLock);
    _flashPending = false;
    portEXIT_CRITICAL(&_flashStateLock);
}

void LEDHandler::setStripMode(LEDStripMode mode) {
    if(!RobotConfig::USE_LED_STRIP) return;

    portENTER_CRITICAL(&_stripStateLock);
    if(_stripMode != mode) {
        _stripMode = mode;
        _stripModeChanged = true;
    }
    portEXIT_CRITICAL(&_stripStateLock);
}

void LEDHandler::setAnimation(LEDStripAnimation animation) {
    if(!RobotConfig::USE_LED_STRIP) return;

    portENTER_CRITICAL(&_stripStateLock);
    if(_stripAnimation != animation) {
        _stripAnimation = animation;
        _stripAnimationChanged = true;
    }
    portEXIT_CRITICAL(&_stripStateLock);
}

void LEDHandler::setAngularVelocity(float radiansPerSecond) {
    if(!RobotConfig::USE_LED_STRIP) return;

    portENTER_CRITICAL(&_stripStateLock);
    _stripAngularVelocityRadPerSec = std::isfinite(radiansPerSecond) ? radiansPerSecond : 0.0f;
    portEXIT_CRITICAL(&_stripStateLock);
}

void LEDHandler::update() {
    uint32_t currentMs = millis();
    uint32_t currentUs = micros();
    bool shouldLight = false;

    portENTER_CRITICAL(&_flashStateLock);
    if(_flashRunning && hasReached(currentUs, _flashEndUs)) {
        _flashRunning = false;
        _hasLastFlashEnd = true;
        _lastFlashEndUs = currentUs;
    }

    if(!_flashRunning && _flashPending) {
        uint32_t earliestStartUs = _pendingFlashStartUs;

        if(_hasLastFlashEnd) {
            const uint32_t endOfMinimumGapUs = _lastFlashEndUs + _pendingMinimumGapUs;

            if(!hasReached(currentUs, endOfMinimumGapUs) && isEarlier(earliestStartUs, endOfMinimumGapUs)) {
                earliestStartUs = endOfMinimumGapUs;
            }
        }

        if(hasReached(currentUs, earliestStartUs)) {
            _flashPending = false;
            _flashRunning = true;
            _flashEndUs = currentUs + _pendingFlashDurationUs;
        }
    }
    const bool flashRunning = _flashRunning;
    portEXIT_CRITICAL(&_flashStateLock);

    if(_animationActive) {
        if(currentMs - _animationStartMs >= _animationDurationMs) {
            _animationActive = false;
            _currentAnimation = LEDAnimation::None;
        }
    }

    if(_animationActive) {
        // --- ANIMATION (Priority) ---
        uint32_t elapsed = currentMs - _animationStartMs;

        switch(_currentAnimation) {
            case LEDAnimation::Bootup:
                // 4 Hz blinking
                shouldLight = (elapsed % 250) < 125;
                break;

            case LEDAnimation::ModeChanged:
                // 10 Hz blinking
                shouldLight = (elapsed % 100) < 50;
                break;

            case LEDAnimation::ErrorAlert:
                // 5 Hz rapid blinking
                shouldLight = (elapsed % 200) < 100;
                break;

            case LEDAnimation::TelemetrySent:
                // Single blink
                shouldLight = (elapsed < 50);
                break;

            default:
                shouldLight = true;
                break;
        }
    } else {
        // --- INDICATION ---
        switch(_currentIndication) {
            case LEDIndication::Off:
                shouldLight = false;
                break;

            case LEDIndication::Idle: {
                // Heart beat effect
                uint32_t cycle = currentMs % 2000;
                shouldLight = (cycle < 100) || (cycle > 200 && cycle < 300);
                break;
            }

            case LEDIndication::Forward:
                // solid light
                shouldLight = true;
                break;

            case LEDIndication::Spin:
                shouldLight = flashRunning;
                break;

            case LEDIndication::Failsafe:
                // 2 Hz, 250 ms on 250 ms off
                shouldLight = (currentMs % 500) < 250;
                break;

            case LEDIndication::HardwareError:
                // 15 Hz rapid blinking
                shouldLight = (currentMs % 60) < 30;
                break;

            case LEDIndication::LowBattery:
                // 100 ms on 900 ms off
                shouldLight = (currentMs % 1000) < 100;
                break;

            default:
                break;
        }
    }

    if(_ledState != shouldLight) {
        _ledState = shouldLight;
        digitalWrite(_ledPin, _ledState ? HIGH : LOW);
    }

    updateStrip(currentMs, currentUs);
}

void LEDHandler::updateStrip(uint32_t currentMs, uint32_t currentUs) {
    if(!RobotConfig::USE_LED_STRIP) return;

    LEDStripMode mode;
    LEDStripAnimation animation;
    float angularVelocityRadPerSec;
    bool modeChanged;
    bool animationChanged;

    portENTER_CRITICAL(&_stripStateLock);
    mode = _stripMode;
    animation = _stripAnimation;
    angularVelocityRadPerSec = _stripAngularVelocityRadPerSec;
    modeChanged = _stripModeChanged;
    animationChanged = _stripAnimationChanged;
    _stripModeChanged = false;
    _stripAnimationChanged = false;
    portEXIT_CRITICAL(&_stripStateLock);

    const LEDStripMath::IntegrationState integration = LEDStripMath::updateIntegration(
        _stripAngleRadians, _lastStripIntegrationUs, currentUs, angularVelocityRadPerSec, modeChanged);
    _stripAngleRadians = integration.angleRadians;
    _lastStripIntegrationUs = integration.lastUpdateUs;

    if(animationChanged) _stripAnimationStartMs = currentMs;

    constexpr uint32_t REFRESH_INTERVAL_US = 1000000UL / RobotConfig::LED_STRIP_REFRESH_HZ;
    if(!LEDStripMath::shouldRefresh(currentUs, _lastStripRefreshUs, REFRESH_INTERVAL_US, _stripFrameSent)) return;
    _lastStripRefreshUs = currentUs;

    if(mode == LEDStripMode::Spinning) {
        renderSpinningStrip(animation);
    } else {
        renderStaticStrip(animation, currentMs - _stripAnimationStartMs);
    }

    applyStripBrightness();
    if(!LEDStripMath::shouldTransmit(_stripFrameSent, stripPixelsChanged())) return;

    transmitStripFrame();
    rememberStripPixels();
    _stripFrameSent = true;
}

void LEDHandler::renderStaticStrip(LEDStripAnimation animation, uint32_t elapsedMs) {
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

            std::fill(std::begin(_stripPixels), std::end(_stripPixels), 0);
            const uint16_t activePixel = (elapsedMs / 80) % RobotConfig::LED_STRIP_LED_COUNT;
            _stripPixels[activePixel] = 0x0080FF;
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

    std::fill(std::begin(_stripPixels), std::end(_stripPixels), color);
}

void LEDHandler::renderSpinningStrip(LEDStripAnimation animation) {
    const uint16_t sector = LEDStripMath::angleToSector(_stripAngleRadians, RobotConfig::LED_STRIP_SECTOR_COUNT);
    const uint32_t* sectorPixels = LEDStripImages::getSectorPixels(animation, sector);

    if(sectorPixels == nullptr) {
        std::fill(std::begin(_stripPixels), std::end(_stripPixels), 0);
        return;
    }

    std::copy_n(sectorPixels, RobotConfig::LED_STRIP_LED_COUNT, _stripPixels);
}

void LEDHandler::applyStripBrightness() {
    for(uint32_t& pixel : _stripPixels) {
        pixel = scaleColor(pixel);
    }
}

bool LEDHandler::stripPixelsChanged() const {
    return std::memcmp(_stripPixels, _lastStripPixels, sizeof(_stripPixels)) != 0;
}

void LEDHandler::rememberStripPixels() {
    std::memcpy(_lastStripPixels, _stripPixels, sizeof(_stripPixels));
}

void IRAM_ATTR LEDHandler::transmitStripFrame() {
    static_assert(RobotConfig::PIN_LED_STRIP < 32, "Software strip output only supports GPIOs 0..31");

    const uint32_t pinMask = 1UL << RobotConfig::PIN_LED_STRIP;
    const uint32_t cyclesPerMicrosecond = getCpuFrequencyMhz();
    const uint32_t zeroHighCycles = cyclesPerMicrosecond * 35 / 100;
    const uint32_t oneHighCycles = cyclesPerMicrosecond * 70 / 100;
    const uint32_t bitCycles = cyclesPerMicrosecond * 125 / 100;

    portENTER_CRITICAL(&_stripOutputLock);
    for(const uint32_t pixel : _stripPixels) {
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
    portEXIT_CRITICAL(&_stripOutputLock);
}
