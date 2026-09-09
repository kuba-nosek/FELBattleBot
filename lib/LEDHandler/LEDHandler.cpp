#include "LEDHandler.h"
#include <Arduino.h>

namespace
{
    bool hasReached(uint32_t currentUs, uint32_t deadlineUs)
    {
        return static_cast<int32_t>(currentUs - deadlineUs) >= 0;
    }

    bool isEarlier(uint32_t firstUs, uint32_t secondUs)
    {
        return static_cast<int32_t>(firstUs - secondUs) < 0;
    }
}

LEDHandler::LEDHandler(uint8_t ledPin) 
    : _ledPin(ledPin), _currentIndication(LEDIndication::Off), 
      _currentAnimation(LEDAnimation::None), _animationActive(false),
      _animationStartMs(0), _animationDurationMs(0), _lastToggleMs(0), 
      _ledState(false), _stepCounter(0),
      _flashRunning(false), _flashEndUs(0),
      _flashPending(false), _pendingFlashStartUs(0),
      _pendingFlashDurationUs(0), _pendingMinimumGapUs(0),
      _hasLastFlashEnd(false), _lastFlashEndUs(0) {
}

void LEDHandler::init() {
    pinMode(_ledPin, OUTPUT);
    digitalWrite(_ledPin, LOW);
}

void LEDHandler::setIndication(LEDIndication mode) {
    if (_currentIndication == mode) return;

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
    
    switch (mode) {
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

void LEDHandler::scheduleFlash(
    uint32_t startTimeFromNowUs,
    uint32_t flashDurationUs,
    uint32_t minimumTimeBetweenFlashesUs) {
    if (flashDurationUs == 0 ||
        startTimeFromNowUs > INT32_MAX ||
        flashDurationUs > INT32_MAX ||
        minimumTimeBetweenFlashesUs >
            static_cast<uint32_t>(INT32_MAX) - flashDurationUs) return;

    const uint32_t currentUs = micros();
    const uint32_t flashStartUs = currentUs + startTimeFromNowUs;
    bool shouldSchedule = true;

    portENTER_CRITICAL(&_flashStateLock);
    if (_flashRunning) {
        const uint32_t earliestStartUs =
            _flashEndUs + minimumTimeBetweenFlashesUs;

        shouldSchedule = !isEarlier(flashStartUs, earliestStartUs);
    } else if (_hasLastFlashEnd) {
        const uint32_t earliestStartUs =
            _lastFlashEndUs + minimumTimeBetweenFlashesUs;

        if (!hasReached(currentUs, earliestStartUs) &&
            isEarlier(flashStartUs, earliestStartUs)) {
            shouldSchedule = false;
        }
    }

    if (shouldSchedule && _flashPending &&
        !isEarlier(flashStartUs, _pendingFlashStartUs)) {
        shouldSchedule = false;
    }

    if (shouldSchedule) {
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

void LEDHandler::update() {
    uint32_t currentMs = millis();
    uint32_t currentUs = micros();
    bool shouldLight = false;

    portENTER_CRITICAL(&_flashStateLock);
    if (_flashRunning && hasReached(currentUs, _flashEndUs)) {
        _flashRunning = false;
        _hasLastFlashEnd = true;
        _lastFlashEndUs = currentUs;
    }

    if (!_flashRunning && _flashPending) {
        uint32_t earliestStartUs = _pendingFlashStartUs;

        if (_hasLastFlashEnd) {
            const uint32_t endOfMinimumGapUs =
                _lastFlashEndUs + _pendingMinimumGapUs;

            if (!hasReached(currentUs, endOfMinimumGapUs) &&
                isEarlier(earliestStartUs, endOfMinimumGapUs)) {
                earliestStartUs = endOfMinimumGapUs;
            }
        }

        if (hasReached(currentUs, earliestStartUs)) {
            _flashPending = false;
            _flashRunning = true;
            _flashEndUs = currentUs + _pendingFlashDurationUs;
        }
    }
    const bool flashRunning = _flashRunning;
    portEXIT_CRITICAL(&_flashStateLock);

    if (_animationActive) {
        if (currentMs - _animationStartMs >= _animationDurationMs) {
            _animationActive = false;
            _currentAnimation = LEDAnimation::None;
        }
    }

    if (_animationActive) {
        // --- ANIMATION (Priority) ---
        uint32_t elapsed = currentMs - _animationStartMs;
        
        switch (_currentAnimation) {
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
        switch (_currentIndication) {
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

    if (_ledState != shouldLight) {
        _ledState = shouldLight;
        digitalWrite(_ledPin, _ledState ? HIGH : LOW);
    }
}
