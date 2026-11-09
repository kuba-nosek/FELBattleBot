#pragma once

#include "IRobot.h"
#include "ITelemetryLink.h"

class TelemetryManager {
  public:
    bool shouldSendTelemetry(const RobotCore& robot, bool modeChanged);
    void sendTelemetry(const RobotCore& robot, const IRobotMode& activeMode);

  private:
    uint32_t lastSentMs_ = 0;
    bool immediateSendPending_ = false;
    
    TelemetryData currentPayload_{};
};
