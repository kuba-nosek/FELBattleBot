#pragma once
#include "IRobotMode.h"

class SpinMode : public IRobotMode {
public:
    void init(RobotCore& robot) override;
    void execute(RobotCore& robot) override;
};