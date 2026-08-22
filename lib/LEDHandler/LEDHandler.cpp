#include "LEDHandler.h"
#include <Arduino.h>

LEDHandler::LEDHandler(uint8_t ledPin) 
    : _ledPin(ledPin), _currentIndication(LEDIndication::Off), 
      _currentAnimation(LEDAnimation::None), _animationActive(false),
      _animationStartMs(0), _animationDurationMs(0), _lastToggleMs(0), 
      _ledState(false), _stepCounter(0) {
}

void LEDHandler::init() {
    pinMode(_ledPin, OUTPUT);
    digitalWrite(_ledPin, LOW);
}

void LEDHandler::setIndication(LEDIndication mode) {
    if (_currentIndication == mode) return;
    
    _currentIndication = mode;
    // Reset kroků pro nový vzor, pokud nehraje animace
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
    _ledState = true; // Obvykle animace začíná svícením
    
    // Zde definujete, jak dlouho má konkrétní animace blokovat indikaci
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

void LEDHandler::update(uint32_t currentMs) {
    // 1. Kontrola expirace animace
    if (_animationActive) {
        if (currentMs - _animationStartMs >= _animationDurationMs) {
            _animationActive = false;
            _currentAnimation = LEDAnimation::None;
            _stepCounter = 0; // Příprava na návrat k indikaci
        }
    }

    // 2. Vyhodnocení vzoru blikání (buď animace nebo indikace)
    if (_animationActive) {
        // --- LOGIKA PRO ANIMACE ---
        switch (_currentAnimation) {
            case LEDAnimation::ModeChanged:
                // Příklad: Rychlé probliknutí (50 ms on, 50 ms off)
                if (currentMs - _lastToggleMs >= 50) {
                    _ledState = !_ledState;
                    digitalWrite(_ledPin, _ledState);
                    _lastToggleMs = currentMs;
                }
                break;
                
            // Zde doplníte další vzory pro animace...
            default:
                break;
        }
    } else {
        // --- LOGIKA PRO INDIKACI ---
        switch (_currentIndication) {
            case LEDIndication::Off:
                if (_ledState) {
                    _ledState = false;
                    digitalWrite(_ledPin, LOW);
                }
                break;

            case LEDIndication::Idle:
                // Příklad: Pomalé pulzování / blikání (500 ms on, 500 ms off)
                if (currentMs - _lastToggleMs >= 500) {
                    _ledState = !_ledState;
                    digitalWrite(_ledPin, _ledState);
                    _lastToggleMs = currentMs;
                }
                break;

            case LEDIndication::Forward:
                // Příklad: Trvalé svícení
                if (!_ledState) {
                    _ledState = true;
                    digitalWrite(_ledPin, HIGH);
                }
                break;

            // Zde doplníte vzory pro Idle, Spin, Failsafe...
            default:
                break;
        }
    }
}