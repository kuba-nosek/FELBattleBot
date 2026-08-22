#pragma once
#include <stdint.h>

struct IMUData {
    float x;
    float y;
    float z;
};

class IMU {
public:
    IMU(uint8_t csPin, float offsetX, float offsetY, float offsetZ);
    
    bool init();
    bool isAvailable() const;
    bool readData(IMUData& dataOut);
    
    // Zápis do konfiguračních registrů (např. pro změnu rozsahu g) za běhu
    void writeConfig(uint8_t reg, uint8_t value);

private:
    uint8_t _csPin;
    float _offsetX;
    float _offsetY;
    float _offsetZ;
    float _sensitivityMultiplier;
    bool _isAvailable;

    uint8_t readRegister(uint8_t reg);
    void readMultipleRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);
    float getSensitivityMultiplier(uint8_t ctrlReg4);
};