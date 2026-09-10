#pragma once
#include "IRobotMode.h"

class ForwardMode : public IRobotMode {
  public:
    void init(RobotCore& robot) override;
    void execute(RobotCore& robot, const ReceiverInput& input) override;
};
