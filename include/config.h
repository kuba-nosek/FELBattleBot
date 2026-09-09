#pragma once
#include <stdint.h>

// ==========================================
// DEBUG AP & TELEMETRY
// ==========================================
#ifndef ENABLE_DEBUG_AP
    #define ENABLE_DEBUG_AP false
#endif

#if ENABLE_DEBUG_AP
    constexpr char AP_SSID[] = "FELBattleBot";
    constexpr char AP_PASS[] = "melty123";
    constexpr char UDP_BROADCAST_IP[] = "192.168.4.255"; 
    constexpr uint16_t UDP_PORT = 4444;
#endif

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
    
    constexpr uint8_t PIN_LED     = 9; 

    // SPI Bus
    constexpr uint8_t PIN_SPI_SCK  = 8;  
    constexpr uint8_t PIN_SPI_MISO = 6;  
    constexpr uint8_t PIN_SPI_MOSI = 7;  
    constexpr uint8_t PIN_SPI_CS1  = 4;  
    constexpr uint8_t PIN_SPI_CS2  = 5;  

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
    constexpr uint32_t TASK_CONTROL_HZ = 200; // Main thread frequency (200 Hz = 5 ms)
    constexpr uint32_t TASK_RECEIVER_MS = 2;  // UART readout interval
    constexpr uint32_t TASK_LED_MS = 40;      // LED update interval

    // ==========================================
    // Custom CRSF telemetry (EdgeTX BBOT.lua)
    // ==========================================
    // This uses a private payload (0xF3) inside CRSF frame type 0x80.
    // Do not enable it together with ArduPilot/Yaapu passthrough telemetry.
    constexpr bool TELEMETRY_ENABLED = true;
    constexpr uint32_t TELEMETRY_INTERVAL_MS = 100; // 10 packets/second

    constexpr bool TELEMETRY_SEND_RPM = true;
    constexpr bool TELEMETRY_SEND_ACCEL1_X = true;
    constexpr bool TELEMETRY_SEND_ACCEL1_Y = true;
    constexpr bool TELEMETRY_SEND_ACCEL1_Z = true;
    constexpr bool TELEMETRY_SEND_ACCEL2_X = true;
    constexpr bool TELEMETRY_SEND_ACCEL2_Y = true;
    constexpr bool TELEMETRY_SEND_ACCEL2_Z = true;

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

    static_assert(
        FORWARD_MAX_MOVING_TURN_PERCENT >= 0 &&
        FORWARD_MAX_MOVING_TURN_PERCENT <= 100,
        "Forward moving turn percentage must be in the range 0..100");
    static_assert(
        FORWARD_MAX_STANDING_TURN_POWER >= 0 &&
        FORWARD_MAX_STANDING_TURN_POWER <= RC_OUTPUT_SCALE,
        "Forward standing turn power must be in the range 0..1000");
    // Generic thresholds used for mapped switches.
    constexpr uint16_t RC_SWITCH_LOW_THRESHOLD = 1300;
    constexpr uint16_t RC_SWITCH_HIGH_THRESHOLD = 1700;
}

// Compile-time receiver input configuration.
// Format: X(fieldName, inputType, oneBasedChannel)
// Supported input types: Stick, Potentiometer, TwoStateSwitch,
// ThreeStateSwitch, SixStateSwitch.
#define RECEIVER_INPUT_MAP(X) \
    X(rightStickHorizontal,     Stick,            1)  \
    X(rightStickVertical,       Stick,            2)  \
    X(leftStickVertical,       Stick,            3)  \
    X(leftStickHorizontal,     Stick,            4)  \
    X(leftSwitch,               TwoStateSwitch,   5)  \
    X(rightSwitch,               TwoStateSwitch,   8)  \
    X(left3StateSwitch,    ThreeStateSwitch, 6)  \
    X(right3StateSwitch,              ThreeStateSwitch, 7)  \
    X(leftPot,         Potentiometer,     11)  \
    X(rightPot,         Potentiometer,     15)  \
    X(sixStateSwitch,          SixStateSwitch,   14)
