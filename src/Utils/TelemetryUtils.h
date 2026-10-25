#pragma once
#include "IRobot.h"
#include "ITelemetryLink.h"

namespace TelemetryUtils {

    inline void fillCommonTelemetryData(const RobotCore& robot, TelemetryData& data) {
        const DriveModeType currentMode = robot.state.requestedMode;

        data.mode = static_cast<uint8_t>(currentMode);
        
    }

} // namespace TelemetryUtils
