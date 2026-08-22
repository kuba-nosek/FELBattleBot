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
    
    constexpr uint8_t PIN_LED     = 10; 

    // SPI Sběrnice[cite: 11]
    constexpr uint8_t PIN_SPI_SCK  = 8;  
    constexpr uint8_t PIN_SPI_MISO = 6;  
    constexpr uint8_t PIN_SPI_MOSI = 7;  
    constexpr uint8_t PIN_SPI_CS1  = 4;  
    constexpr uint8_t PIN_SPI_CS2  = 5;  

    // ==========================================
    // KALIBRACE SENZORŮ (v jednotkách g)[cite: 11]
    // ==========================================
    constexpr float IMU1_OFFSET_X = 0.08f;
    constexpr float IMU1_OFFSET_Y = 0.57f;
    constexpr float IMU1_OFFSET_Z = 2.85f;

    constexpr float IMU2_OFFSET_X = -0.30f;
    constexpr float IMU2_OFFSET_Y = 0.15f;
    constexpr float IMU2_OFFSET_Z = 1.94f;

    // ==========================================
    // ČASOVÁNÍ RTOS VLÁKEN
    // ==========================================
    constexpr uint32_t TASK_CONTROL_HZ = 100; // Frekvence řídicí smyčky (100 Hz = 10 ms)
    constexpr uint32_t TASK_RECEIVER_MS = 2;  // Interval čtení UARTu
    constexpr uint32_t TASK_LED_MS = 20;      // Interval aktualizace LED
}