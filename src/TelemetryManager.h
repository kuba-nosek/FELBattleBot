#pragma once

#include "Modes/IRobotMode.h"

class TelemetryManager {
  public:
    bool shouldSendTelemetry(const RobotCore& robot, bool modeChanged);
    void getRpm(RobotCore& robot);
    void sendTelemetry(RobotCore& robot, const IRobotMode& activeMode);

  private:
    uint32_t lastSentMs_ = 0;
    bool immediateSendPending_ = false;
};
