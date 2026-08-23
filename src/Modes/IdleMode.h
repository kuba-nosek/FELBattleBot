#pragma once
#include "IRobotMode.h"

class IdleMode : public IRobotMode {
public:
    void init(RobotCore& robot) override;
    void execute(RobotCore& robot) override;
};