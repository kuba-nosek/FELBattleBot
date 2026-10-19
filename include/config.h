#pragma once
#include <stdint.h>

namespace RobotConfig {

// ==========================================
// 1. PINOUT & HARDWARE
// ==========================================
constexpr uint8_t PIN_CRSF_RX = 1;
constexpr uint8_t PIN_CRSF_TX = 2;

constexpr uint8_t PIN_MOTOR_L = 41;
constexpr uint8_t PIN_MOTOR_R = 42;
constexpr uint8_t PIN_MOTOR_L_TLM = 15;
constexpr uint8_t PIN_MOTOR_R_TLM = 16;

constexpr uint8_t PIN_LED = 14;
constexpr uint8_t PIN_LED_STRIP = 17;

constexpr uint8_t PIN_SPI_SCK = 12;
constexpr uint8_t PIN_SPI_MISO = 13;
constexpr uint8_t PIN_SPI_MOSI = 11;
constexpr uint8_t PIN_SPI_CS1 = 9;
constexpr uint8_t PIN_SPI_CS2 = 10;

// ==========================================
// 2. OTA (OVER-THE-AIR) SETTINGS
// ==========================================
constexpr char OTA_AP_SSID[] = "MELTY-OTA";
constexpr char OTA_AP_PASSWORD[] = "heslo_pro_ota";
constexpr char OTA_HOSTNAME[] = "meltybrain-ota";

// ==========================================
// 3. RTOS THREAD TIMING
// ==========================================
constexpr uint32_t TASK_CONTROL_HZ = 200;
constexpr uint32_t TASK_RECEIVER_HZ = 500;
constexpr uint32_t TASK_LED_HZ = 1000;

// ==========================================
// 4. MOTOR & DRIVE SETTINGS
// ==========================================
constexpr bool MOTOR_LEFT_REVERSED = false;
constexpr bool MOTOR_RIGHT_REVERSED = true;
constexpr uint8_t MOTOR_POLE_PAIRS = 7;

// ==========================================
// 6. RC & SIGNAL PROCESSING
// ==========================================
constexpr uint16_t RC_CHANNEL_MIN = 988;
constexpr uint16_t RC_CHANNEL_MAX = 2012;
constexpr uint16_t RC_CHANNEL_CENTER = 1500;

// Deadband to prevent motor drift when sticks are centered
constexpr uint16_t RC_DEADBAND = 20;

// Normalized internal scale (-1000 to 1000)
constexpr int32_t RC_OUTPUT_SCALE = 1000;

// Generic thresholds used for mapped switches
constexpr uint16_t RC_SWITCH_LOW_THRESHOLD = 1300;
constexpr uint16_t RC_SWITCH_HIGH_THRESHOLD = 1700;

} // namespace RobotConfig

// ==========================================
// COMPILE-TIME MAPPINGS
// ==========================================

// Receiver input configuration.
// Format: X(fieldName, inputType, oneBasedChannel)
// Supported input types: Stick, Potentiometer, TwoStateSwitch, ThreeStateSwitch, SixStateSwitch.
#define RECEIVER_INPUT_MAP(X)                                                                                          \
    X(rightStickHorizontal, Stick, 1)                                                                                  \
    X(rightStickVertical, Stick, 2)                                                                                    \
    X(leftStickVertical, Stick, 3)                                                                                     \
    X(leftStickHorizontal, Stick, 4)                                                                                   \
    X(leftSwitch, TwoStateSwitch, 5)                                                                                   \
    X(left3StateSwitch, ThreeStateSwitch, 6)                                                                           \
    X(right3StateSwitch, ThreeStateSwitch, 7)                                                                          \
    X(rightSwitch, TwoStateSwitch, 8)                                                                                  \
    X(leftPot, Potentiometer, 9)                                                                                       \
    X(rightPot, Potentiometer, 10)                                                                                     \
    X(sixStateSwitch, SixStateSwitch, 11)                                                                              \
    X(leftShoulderSwitch, TwoStateSwitch, 12)
