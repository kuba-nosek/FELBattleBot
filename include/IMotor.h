#pragma once
#include <stdint.h>

class IMotor {
public:
    virtual ~IMotor() = default;

    virtual bool init() = 0;
    virtual void setReversed(bool reversed) = 0;
    virtual void resetDirectionGuard() = 0;
    virtual void stop() = 0;
    
    virtual void setSpeed(int16_t speed, uint32_t currentMs) = 0;
};
