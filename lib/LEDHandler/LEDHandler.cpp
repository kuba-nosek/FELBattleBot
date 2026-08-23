#include "LEDHandler.h"
#include <Arduino.h>

LEDHandler::LEDHandler(uint8_t ledPin) 
    : _ledPin(ledPin), _currentIndication(LEDIndication::Off), 
      _currentAnimation(LEDAnimation::None), _animationActive(false),
      _animationStartMs(0), _animationDurationMs(0), _lastToggleMs(0), 
      _ledState(false), _stepCounter(0),
      _meltyPeriodUs(0), _meltyPhaseOffsetUs(0), _meltyFlashDurationUs(0) {
}

void LEDHandler::init() {
    pinMode(_ledPin, OUTPUT);
    digitalWrite(_ledPin, LOW);
}

void LEDHandler::setIndication(LEDIndication mode) {
    if (_currentIndication == mode) return;
    _currentIndication = mode;
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

void LEDHandler::setMeltySync(uint32_t periodUs, uint32_t phaseOffsetUs, uint32_t flashDurationUs) {
    _meltyPeriodUs = periodUs;
    _meltyPhaseOffsetUs = phaseOffsetUs;
    _meltyFlashDurationUs = flashDurationUs;
    setIndication(LEDIndication::MeltySync);
}

bool LEDHandler::isMeltySyncActive() const {
    return (_currentIndication == LEDIndication::MeltySync && !_animationActive);
}

void LEDHandler::update() {
    uint32_t currentMs = millis();
    bool shouldLight = false;

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
            case LEDIndication::Spin:
                // solid light
                shouldLight = true;
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

            case LEDIndication::MeltySync:
                // MeltySpin indication
                if (_meltyPeriodUs > 0) {
                    uint32_t currentUs = micros(); 
                    uint32_t positionInRotation = (currentUs - _meltyPhaseOffsetUs) % _meltyPeriodUs;
                    shouldLight = (positionInRotation < _meltyFlashDurationUs);
                }
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