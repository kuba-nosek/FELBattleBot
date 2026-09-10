#pragma once
#include <freertos/FreeRTOS.h>
#include <stdint.h>

// Persistent indication states (linked to active robot drive modes)
enum class LEDIndication : uint8_t { Off = 0, Idle, Forward, Spin, Failsafe, LowBattery, HardwareError };

// Short-term priority animation states
enum class LEDAnimation : uint8_t { None = 0, Bootup, ModeChanged, ErrorAlert, TelemetrySent };

class LEDHandler {
  public:
    explicit LEDHandler(uint8_t ledPin);

    void init();

    // Set persistent mode (e.g., called upon ModeHandler state change)
    void setIndication(LEDIndication mode);

    // Play a priority animation (temporarily overrides base indication)
    void playAnimation(LEDAnimation mode);

    // Schedule one flash relative to the current time.
    void scheduleFlash(uint32_t startTimeFromNowUs, uint32_t flashDurationUs, uint32_t minimumTimeBetweenFlashesUs);

    // Cancel only the pending flash. A running flash is allowed to finish.
    void cancelScheduledFlash();

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

    // One running flash and at most one future flash.
    bool _flashRunning;
    uint32_t _flashEndUs;

    bool _flashPending;
    uint32_t _pendingFlashStartUs;
    uint32_t _pendingFlashDurationUs;
    uint32_t _pendingMinimumGapUs;

    bool _hasLastFlashEnd;
    uint32_t _lastFlashEndUs;

    portMUX_TYPE _flashStateLock = portMUX_INITIALIZER_UNLOCKED;
};
