#include "IMU.h"
#include <Arduino.h>
#include <SPI.h>

namespace {
    // H3LIS331DL register map
    constexpr uint8_t WHO_AM_I_REG = 0x0F;
    constexpr uint8_t CTRL_REG1    = 0x20;
    constexpr uint8_t CTRL_REG2    = 0x21;
    constexpr uint8_t CTRL_REG3    = 0x22;
    constexpr uint8_t CTRL_REG4    = 0x23;
    constexpr uint8_t CTRL_REG5    = 0x24;
    constexpr uint8_t OUT_X_L      = 0x28;
    constexpr uint8_t EXPECTED_ID  = 0x32;
    constexpr float STANDARD_GRAVITY_MPS2 = 9.80665f;
}

IMU::IMU(
    uint8_t csPin,
    float offsetXG,
    float offsetYG,
    float offsetZG)
    : _csPin(csPin),
      _offsetXG(offsetXG),
      _offsetYG(offsetYG),
      _offsetZG(offsetZG),
      _sensitivityGPerLsb(0.049f),
      _isAvailable(false) {
}

bool IMU::init() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    uint8_t id = readRegister(WHO_AM_I_REG);
    
    if (id == EXPECTED_ID) {
        // Default configuration
        writeConfig(CTRL_REG1, 0x27); // 50 Hz, XYZ axes enabled
        writeConfig(CTRL_REG2, 0x00);
        writeConfig(CTRL_REG3, 0x00);
        writeConfig(CTRL_REG4, 0x80); // Block Data Update (BDU) active, +-100g
        writeConfig(CTRL_REG5, 0x00);
        
        _isAvailable = true;
        return true;
    }
    
    _isAvailable = false;
    return false;
}

bool IMU::isAvailable() const {
    return _isAvailable;
}

void IMU::writeConfig(uint8_t reg, uint8_t value) {
    uint8_t instruction = reg & 0x3F; 
    digitalWrite(_csPin, LOW);
    SPI.transfer(instruction);
    SPI.transfer(value);
    digitalWrite(_csPin, HIGH);

    // Update sensitivity if scale (CTRL_REG4) is modified dynamically.
    if (reg == CTRL_REG4) {
        _sensitivityGPerLsb = getSensitivityGPerLsb(value);
    }
}

uint8_t IMU::readRegister(uint8_t reg) {
    uint8_t instruction = 0x80 | (reg & 0x3F); 
    digitalWrite(_csPin, LOW);
    SPI.transfer(instruction);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(_csPin, HIGH);
    return val;
}

void IMU::readMultipleRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    uint8_t instruction = 0xC0 | (reg & 0x3F); 
    digitalWrite(_csPin, LOW);
    SPI.transfer(instruction);
    for(uint8_t i = 0; i < len; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(_csPin, HIGH);
}

float IMU::getSensitivityGPerLsb(uint8_t ctrlReg4) {
    // Extract FS1 and FS0 bits to determine scale
    uint8_t fs_bits = (ctrlReg4 >> 4) & 0x03; 
    if (fs_bits == 0x00) return 0.049f; // +-100g
    if (fs_bits == 0x01) return 0.098f; // +-200g
    if (fs_bits == 0x03) return 0.195f; // +-400g
    return 0.049f;
}

bool IMU::readData(IMUData& dataOut) {
    if (!_isAvailable) return false;

    uint8_t data[6];
    readMultipleRegisters(OUT_X_L, data, 6);
    
    // Combine 12-bit data and remove alignment
    int16_t x_12bit = (int16_t)(data[0] | (data[1] << 8)) >> 4;
    int16_t y_12bit = (int16_t)(data[2] | (data[3] << 8)) >> 4;
    int16_t z_12bit = (int16_t)(data[4] | (data[5] << 8)) >> 4;

    dataOut.xMps2 =
        ((x_12bit * _sensitivityGPerLsb) - _offsetXG) * STANDARD_GRAVITY_MPS2;
    dataOut.yMps2 =
        ((y_12bit * _sensitivityGPerLsb) - _offsetYG) * STANDARD_GRAVITY_MPS2;
    dataOut.zMps2 =
        ((z_12bit * _sensitivityGPerLsb) - _offsetZG) * STANDARD_GRAVITY_MPS2;

    return true;
}
