#pragma once
#include <stdint.h>

// ==========================================
// DEBUG AP & TELEMETRY
// ==========================================
namespace RobotConfig {
// ==========================================
// PINOUT
// ==========================================
constexpr uint8_t PIN_CRSF_RX = 0;
constexpr uint8_t PIN_CRSF_TX = 1;

constexpr uint8_t PIN_MOTOR_L = 3;
constexpr uint8_t PIN_MOTOR_R = 2;

constexpr bool MOTOR_LEFT_REVERSED = false;
constexpr bool MOTOR_RIGHT_REVERSED = false;

constexpr uint8_t PIN_LED = 9;

// POV LED strip
constexpr bool USE_LED_STRIP = true;
constexpr uint8_t PIN_LED_STRIP = 21;
constexpr uint16_t LED_STRIP_LED_COUNT = 14;
constexpr uint16_t LED_STRIP_SPIN_LED_COUNT = LED_STRIP_LED_COUNT / 2;
constexpr uint8_t LED_STRIP_SECTORS_PER_LED = 3;
constexpr uint16_t LED_STRIP_SECTOR_COUNT = LED_STRIP_LED_COUNT * LED_STRIP_SECTORS_PER_LED;
constexpr uint16_t LED_STRIP_REFRESH_HZ = 500;
constexpr uint8_t LED_STRIP_BRIGHTNESS = 32;

static_assert(LED_STRIP_LED_COUNT > 1, "The POV strip needs at least two LEDs");
static_assert(LED_STRIP_LED_COUNT % 2 == 0, "The half-strip POV layout requires an even LED count");
static_assert(LED_STRIP_SECTORS_PER_LED > 0, "The POV strip needs at least one sector per LED");
static_assert(LED_STRIP_REFRESH_HZ > 0, "The POV strip refresh rate must be positive");
static_assert(LED_STRIP_REFRESH_HZ <= 1000000, "The POV strip refresh interval must be at least one microsecond");

// SPI Bus
constexpr uint8_t PIN_SPI_SCK = 8;
constexpr uint8_t PIN_SPI_MISO = 6;
constexpr uint8_t PIN_SPI_MOSI = 7;
constexpr uint8_t PIN_SPI_CS1 = 4;
constexpr uint8_t PIN_SPI_CS2 = 5;

// ==========================================
// IMU calibration [g]
// ==========================================
constexpr float IMU1_OFFSET_X_G = 0.08f;
constexpr float IMU1_OFFSET_Y_G = 0.57f;
constexpr float IMU1_OFFSET_Z_G = 2.85f;

constexpr float IMU2_OFFSET_X_G = -0.30f;
constexpr float IMU2_OFFSET_Y_G = 0.15f;
constexpr float IMU2_OFFSET_Z_G = 1.94f;

// ==========================================
// RTOS thread timing
// ==========================================
constexpr uint32_t TASK_CONTROL_HZ = 200;
constexpr uint32_t TASK_RECEIVER_HZ = 500;
constexpr uint32_t TASK_LED_HZ = 1000;

// ======================
// MOTOR SETTINGS
// ======================
constexpr uint8_t MOTOR_POLE_PAIRS = 7;

// ==========================================
// Custom CRSF telemetry (EdgeTX BBOT.lua)
// ==========================================
// This uses a private payload (0xF3) inside CRSF frame type 0x80.
// Do not enable it together with ArduPilot/Yaapu passthrough telemetry.
constexpr bool TELEMETRY_ENABLED = true;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 500;

// ==========================================
// RC & Signal Processing
// ==========================================
constexpr uint16_t RC_CHANNEL_MIN = 988;
constexpr uint16_t RC_CHANNEL_MAX = 2012;
constexpr uint16_t RC_CHANNEL_CENTER = 1500;
// Deadband to prevent motor drift when sticks are centered
constexpr uint16_t RC_DEADBAND = 20;

// Normalized internal scale (-1000 to 1000)
constexpr int32_t RC_OUTPUT_SCALE = 1000;

// ==========================================
// Forward mode
// ==========================================
constexpr int32_t FORWARD_MAX_MOVING_TURN_PERCENT = 40;
constexpr int32_t FORWARD_MAX_STANDING_TURN_POWER = 50;

static_assert(FORWARD_MAX_MOVING_TURN_PERCENT >= 0 && FORWARD_MAX_MOVING_TURN_PERCENT <= 100,
              "Forward moving turn percentage must be in the range 0..100");
static_assert(FORWARD_MAX_STANDING_TURN_POWER >= 0 && FORWARD_MAX_STANDING_TURN_POWER <= RC_OUTPUT_SCALE,
              "Forward standing turn power must be in the range 0..1000");
// Generic thresholds used for mapped switches.
constexpr uint16_t RC_SWITCH_LOW_THRESHOLD = 1300;
constexpr uint16_t RC_SWITCH_HIGH_THRESHOLD = 1700;
} // namespace RobotConfig

// Compile-time receiver input configuration.
// Format: X(fieldName, inputType, oneBasedChannel)
// Supported input types: Stick, Potentiometer, TwoStateSwitch,
// ThreeStateSwitch, SixStateSwitch.
#define RECEIVER_INPUT_MAP(X)                                                                                          \
    X(rightStickHorizontal, Stick, 1)                                                                                  \
    X(rightStickVertical, Stick, 2)                                                                                    \
    X(leftStickVertical, Stick, 3)                                                                                     \
    X(leftStickHorizontal, Stick, 4)                                                                                   \
    X(leftSwitch, TwoStateSwitch, 5)                                                                                   \
    X(left3StateSwitch, ThreeStateSwitch, 6)                                                                           \
    X(right3StateSwitch, ThreeStateSwitch, 7)                                                                          \
    X(rightSwitch, TwoStateSwitch, 8)                                                                                  \
    X(leftPot, Potentiometer, 9)                                                                                      \
    X(rightPot, Potentiometer, 10)                                                                                     \
    X(sixStateSwitch, SixStateSwitch, 11)

// Compile-time telemetry packet configuration.
// Format: X(fieldName, fieldType)
// Supported field types: int8_t, uint8_t, int16_t, uint16_t, int32_t,
// uint32_t, float.
#define TELEMETRY_FIELD_MAP(X)                                                                                         \
    X(mode, uint8_t)                                                                                                   \
    X(rpm, int16_t)                                                                                                    \
    X(accel1X, float)                                                                                                  \
    X(accel1Y, float)                                                                                                  \
    X(accel1Z, float)                                                                                                  \
    X(accel2X, float)                                                                                                  \
    X(accel2Y, float)                                                                                                  \
    X(accel2Z, float)                                                                                                  \
    X(escLeftRpm, int32_t)                                                                                             \
    X(escLeftVolts, float)                                                                                             \
    X(escRightRpm, int32_t)                                                                                             \
    X(escRightVolts, float)                                                                                             