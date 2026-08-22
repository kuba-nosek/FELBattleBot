#pragma once
#include <stdint.h>

// Definice stavů pro dlouhodobou indikaci (např. vázané na stav robota)
enum class LEDIndication : uint8_t {
    Off = 0,
    Idle,
    Forward,
    Spin,
    Failsafe,
    LowBattery
    // možnost přidat další do kapacity 15+
};

// Definice stavů pro krátkodobé prioritní animace
enum class LEDAnimation : uint8_t {
    None = 0,
    Bootup,
    ModeChanged,
    ErrorAlert,
    TelemetrySent
    // možnost přidat další
};

class LEDHandler {
public:
    LEDHandler(uint8_t ledPin);
    
    void init();
    
    // Nastaví trvalý mód indikace (voláno např. při přepnutí režimu ModeHandlerem)
    void setIndication(LEDIndication mode);
    
    // Spustí krátkodobou animaci, která dočasně překryje indikaci
    void playAnimation(LEDAnimation mode);
    
    // Nutno volat cyklicky ve 3. vlákně (např. každých 10-20 ms)
    void update(uint32_t currentMs);


private:
    uint8_t _ledPin;
    
    // Uložení aktuálních stavů
    LEDIndication _currentIndication;
    LEDAnimation _currentAnimation;
    
    // Stavové proměnné pro animace
    bool _animationActive;
    uint32_t _animationStartMs;
    uint32_t _animationDurationMs;
    
    // Interní proměnné pro blikání/časování uvnitř metody update()
    uint32_t _lastToggleMs;
    bool _ledState;
    uint8_t _stepCounter;
};