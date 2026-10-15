#pragma once

struct AccelData {
    float xMps2 = 0.0f;
    float yMps2 = 0.0f;
    float zMps2 = 0.0f;
};

class IAccel {
  public:
    virtual ~IAccel() = default;

    virtual bool init() = 0;
    virtual bool isAvailable() const = 0;
    virtual bool readData(AccelData& dataOut) = 0;
};
