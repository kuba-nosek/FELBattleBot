#pragma once
#include <stdint.h>

namespace RobotConfig {
    // ==========================================
    // PINOUT
    // ==========================================
    constexpr uint8_t PIN_CRSF_RX = 20; 
    constexpr uint8_t PIN_CRSF_TX = 21; 
    
    constexpr uint8_t PIN_MOTOR_L = 2;  
    constexpr uint8_t PIN_MOTOR_R = 3;  
    
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
    constexpr float IMU1_OFFSET_X = 0.08f;
    constexpr float IMU1_OFFSET_Y = 0.57f;
    constexpr float IMU1_OFFSET_Z = 2.85f;

    constexpr float IMU2_OFFSET_X = -0.30f;
    constexpr float IMU2_OFFSET_Y = 0.15f;
    constexpr float IMU2_OFFSET_Z = 1.94f;

    // ==========================================
    // RTOS thread timing
    // ==========================================
    constexpr uint32_t TASK_CONTROL_HZ = 100; // Main thread frequency (100 Hz = 10 ms)
    constexpr uint32_t TASK_RECEIVER_MS = 2;  // UART readout interval
    constexpr uint32_t TASK_LED_MS = 20;      // LED update interval

    // ==========================================
    // RC & Signal Processing
    // ==========================================
    constexpr uint16_t RC_CHANNEL_MIN = 988;
    constexpr uint16_t RC_CHANNEL_MAX = 2012;
    constexpr uint16_t RC_CHANNEL_CENTER = 1500;
    
    // Half-range (1500 to 2000 roughly equals 500)
    constexpr uint16_t RC_CHANNEL_HALF_RANGE = 500; 
    
    // Deadband to prevent motor drift when sticks are centered
    constexpr uint16_t RC_DEADBAND = 20;
    
    // Normalized internal scale (-1000 to 1000)
    constexpr int32_t RC_OUTPUT_SCALE = 1000;

    // Thresholds for the 3-position mode switch
    constexpr uint16_t RC_MODE_SPIN_THRESHOLD = 1300;
    constexpr uint16_t RC_MODE_FORWARD_THRESHOLD = 1700;
}