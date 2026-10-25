#pragma once
#include <stdint.h>

class IEscFeedback {
  public:
    virtual ~IEscFeedback() = default;
    
    virtual bool isFeedbackValid() const = 0;
    virtual int32_t getRpm() const = 0;
    virtual float getVoltage() const = 0;
    virtual float getCurrent() const = 0;
};

