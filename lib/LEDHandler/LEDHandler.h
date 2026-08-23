#pragma once
#include <stdint.h>

// Persistent indication states (linked to active robot drive modes)
enum class LEDIndication : uint8_t {
    Off = 0,
    Idle,
    Forward,
    Spin,
    MeltySync,   // High-precision microsecond rotational sync
    Failsafe,
    LowBattery,
    HardwareError
};

// Short-term priority animation states
enum class LEDAnimation : uint8_t {
    None = 0,
    Bootup,
    ModeChanged,
    ErrorAlert,
    TelemetrySent
};

class LEDHandler {
public:
    LEDHandler(uint8_t ledPin);
    
    void init();
    
    // Set persistent mode (e.g., called upon ModeHandler state change)
    void setIndication(LEDIndication mode);
    
    // Play a priority animation (temporarily overrides base indication)
    void playAnimation(LEDAnimation mode);
    
    // Setup high-precision parameters for Melty Brain visual tracking
    void setMeltySync(uint32_t periodUs, uint32_t phaseOffsetUs, uint32_t flashDurationUs);
    
    // Returns true if MeltySync is active and not blocked by an animation
    bool isMeltySyncActive() const;
    
    // Must be called cyclically in the dedicated LED thread
    void update();

private:
    uint8_t _ledPin;
    
    // Active states
    LEDIndication _currentIndication;
    LEDAnimation _currentAnimation;
    
    // Animation timers
    bool _animationActive;
    uint32_t _animationStartMs;
    uint32_t _animationDurationMs;
    
    // Internal blinking variables
    uint32_t _lastToggleMs;
    bool _ledState;
    uint8_t _stepCounter;

    // Melty Brain microsecond timing variables
    uint32_t _meltyPeriodUs;
    uint32_t _meltyPhaseOffsetUs;
    uint32_t _meltyFlashDurationUs;
};