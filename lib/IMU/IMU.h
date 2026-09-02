#pragma once
#include <stdint.h>

struct IMUData {
    float xG = 0.0f;
    float yG = 0.0f;
    float zG = 0.0f;
};

class IMU {
public:
    IMU(
        uint8_t csPin,
        float offsetXG,
        float offsetYG,
        float offsetZG);
    
    bool init();
    bool isAvailable() const;
    bool readData(IMUData& dataOut);
    
    // Write to config registers (e.g., dynamically changing g-range limits)
    void writeConfig(uint8_t reg, uint8_t value);

private:
    uint8_t _csPin;
    float _offsetXG;
    float _offsetYG;
    float _offsetZG;
    float _sensitivityGPerLsb;
    bool _isAvailable;

    uint8_t readRegister(uint8_t reg);
    void readMultipleRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);
    float getSensitivityGPerLsb(uint8_t ctrlReg4);
};
