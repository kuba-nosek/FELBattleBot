#pragma once
#include "IRobot.h"
#include "ITelemetryLink.h"

namespace TelemetryUtils {

    inline void fillAccelerometerData(const RobotCore& robot, TelemetryData& data) {
        const AccelData a1 = robot.state.accel1;
        const AccelData a2 = robot.state.accel2;

        data.accel1X = a1.xMps2;
        data.accel1Y = a1.yMps2;
        data.accel1Z = a1.zMps2;
        data.accel2X = a2.xMps2;
        data.accel2Y = a2.yMps2;
        data.accel2Z = a2.zMps2;
    }

} // namespace TelemetryUtils
