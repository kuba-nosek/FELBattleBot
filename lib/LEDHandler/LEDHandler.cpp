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
    
    // Reset sequence steps if no overriding animation is active
    if (!_animationActive) {
        _stepCounter = 0;
        _ledState = false;
        digitalWrite(_ledPin, LOW);
    }
}

void LEDHandler::playAnimation(LEDAnimation mode) {
    _currentAnimation = mode;
    _animationActive = true;
    _animationStartMs = millis();
    _stepCounter = 0;
    
    _ledState = true; // Animations typically start with LED ON
    digitalWrite(_ledPin, HIGH);
    
    // Set durations for specific animation blocks
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
        default:
            _animationDurationMs = 1000;
            break;
    }
}

void LEDHandler::setMeltySync(uint32_t periodUs, uint32_t phaseOffsetUs, uint32_t flashDurationUs) {
    _meltyPeriodUs = periodUs;
    _meltyPhaseOffsetUs = phaseOffsetUs;
    _meltyFlashDurationUs = flashDurationUs;
    
    // Automatically transition to the MeltySync base indication
    setIndication(LEDIndication::MeltySync);
}

bool LEDHandler::isMeltySyncActive() const {
    // Returns true if MeltySync is the active indication AND no priority animation is playing
    return (_currentIndication == LEDIndication::MeltySync && !_animationActive);
}

void LEDHandler::update() {
    uint32_t currentMs = millis();

    // 1. Check animation expiration
    if (_animationActive) {
        if (currentMs - _animationStartMs >= _animationDurationMs) {
            _animationActive = false;
            _currentAnimation = LEDAnimation::None;
            _stepCounter = 0; // Prepare return to base indication
        }
    }

    // 2. Evaluate blinking pattern
    if (_animationActive) {
        // --- ANIMATION LOGIC ---
        switch (_currentAnimation) {
            case LEDAnimation::ModeChanged:
                // Fast toggle (50ms ON / 50ms OFF)
                if (currentMs - _lastToggleMs >= 50) {
                    _ledState = !_ledState;
                    digitalWrite(_ledPin, _ledState);
                    _lastToggleMs = currentMs;
                }
                break;
                
            case LEDAnimation::ErrorAlert:
                // Rapid 100ms strobe for critical errors
                if (currentMs - _lastToggleMs >= 100) {
                    _ledState = !_ledState;
                    digitalWrite(_ledPin, _ledState);
                    _lastToggleMs = currentMs;
                }
                break;
                
            default:
                break;
        }
    } else {
        // --- INDICATION LOGIC ---
        switch (_currentIndication) {
            case LEDIndication::Off:
                if (_ledState) {
                    _ledState = false;
                    digitalWrite(_ledPin, LOW);
                }
                break;

            case LEDIndication::Idle:
                // Slow pulse (500ms ON / 500ms OFF)
                if (currentMs - _lastToggleMs >= 500) {
                    _ledState = !_ledState;
                    digitalWrite(_ledPin, _ledState);
                    _lastToggleMs = currentMs;
                }
                break;

            case LEDIndication::Forward:
            case LEDIndication::Spin:
                // Solid ON for standard drive modes
                if (!_ledState) {
                    _ledState = true;
                    digitalWrite(_ledPin, HIGH);
                }
                break;

            case LEDIndication::MeltySync:
                {
                    // Microsecond precision logic for Melty Brain tracking
                    uint32_t currentUs = micros(); 
                    
                    if (_meltyPeriodUs > 0) {
                        // Find elapsed time within the current rotation (Modulo math)
                        uint32_t positionInRotation = (currentUs - _meltyPhaseOffsetUs) % _meltyPeriodUs;
                        
                        // Determine if we are inside the designated flash window
                        bool shouldFlash = (positionInRotation < _meltyFlashDurationUs);
                        
                        if (_ledState != shouldFlash) {
                            _ledState = shouldFlash;
                            digitalWrite(_ledPin, _ledState);
                        }
                    }
                }
                break;

            case LEDIndication::Failsafe:
                // Failsafe pattern (e.g., 200ms ON/OFF)
                if (currentMs - _lastToggleMs >= 200) {
                    _ledState = !_ledState;
                    digitalWrite(_ledPin, _ledState);
                    _lastToggleMs = currentMs;
                }
                break;

            case LEDIndication::HardwareError:
                // Terminal error - rapid blinking (30ms ON / 30ms OFF)
                if (currentMs - _lastToggleMs >= 30) {
                    _ledState = !_ledState;
                    digitalWrite(_ledPin, _ledState);
                    _lastToggleMs = currentMs;
                }
                break;

            default:
                break;
        }
    }
}