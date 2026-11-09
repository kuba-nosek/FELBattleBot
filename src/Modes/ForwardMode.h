#pragma once
#include "IRobot.h"

class ForwardMode : public IRobotMode {
  public:
    void init(RobotCore& robot) override;
    void execute(RobotCore& robot, const ReceiverInput& input) override;
    void fillTelemetryData(const RobotCore& robot, TelemetryData& data) const override;
};
