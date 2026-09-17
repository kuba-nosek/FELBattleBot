#pragma once
#include "IRobotMode.h"

class IdleMode : public IRobotMode {
  public:
    void init(RobotCore& robot) override;
    void execute(RobotCore& robot, const ReceiverInput& input) override;
    void sendTelemetry(const RobotCore& robot, BattlebotTelemetry::TelemetryData& telemetry) const override;
};
