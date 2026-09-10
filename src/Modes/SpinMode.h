#pragma once

#include "IRobotMode.h"

class SpinMode : public IRobotMode {
  public:
    void init(RobotCore& robot) override;
    void execute(RobotCore& robot, const ReceiverInput& input) override;

  private:
    float headingRadians_ = 0.0f;
    float constantHeadingOffsetRadians_ = 0.0f;
    float variableHeadingOffsetRadians_ = 0.0f;
    uint32_t lastUpdateUs_ = 0;
    int8_t spinDirection_ = 1;

    void updateHeading(float deltaSeconds, float angularSpeedRadPerSec);
};
