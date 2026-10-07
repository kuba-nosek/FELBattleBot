#pragma once
#include "IAccel.h"
#include <stdint.h>

class H3LIS331DL : public IAccel {
  public:
    H3LIS331DL(uint8_t csPin, float offsetXG, float offsetYG, float offsetZG);

    bool init() override;
    bool isAvailable() const override;
    bool readData(AccelData& dataOut) override;

    void writeConfig(uint8_t reg, uint8_t value);

  private:
    uint8_t _csPin;
    float _offsetXG;
    float _offsetYG;
    float _offsetZG;
    float _sensitivityGPerLsb;
    bool _isAvailable;

    uint8_t readRegister(uint8_t reg);
    void readMultipleRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);
    float getSensitivityGPerLsb(uint8_t ctrlReg4);
};
