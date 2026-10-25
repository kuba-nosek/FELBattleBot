#pragma once
#include "IRobot.h"

namespace HardwareMonitor {

    inline void updateEscFeedback(RobotCore& robot) {
        if (!robot.state.isConnected) return;

        if (robot.hw.leftEscFeedback && robot.hw.leftEscFeedback->isFeedbackValid()) {
            robot.state.escLeftRpm = robot.hw.leftEscFeedback->getRpm();
            robot.state.escLeftVolts = robot.hw.leftEscFeedback->getVoltage();
            // robot.state.escLeftAmps = robot.hw.leftEscFeedback->getCurrent();
        }
        if (robot.hw.rightEscFeedback && robot.hw.rightEscFeedback->isFeedbackValid()) {
            robot.state.escRightRpm = robot.hw.rightEscFeedback->getRpm();
            robot.state.escRightVolts = robot.hw.rightEscFeedback->getVoltage();
            // robot.state.escRightAmps = robot.hw.rightEscFeedback->getCurrent();
        }
    }



    inline void updateAccelerometers(RobotCore& robot) {
        if (robot.hw.accel1 && robot.hw.accel1->isAvailable()) {
            AccelData temp;
            robot.hw.accel1->readData(temp);
            robot.state.accel1 = temp;
        }
        if (robot.hw.accel2 && robot.hw.accel2->isAvailable()) {
            AccelData temp;
            robot.hw.accel2->readData(temp);
            robot.state.accel2 = temp;
        }
    }




    inline void pollAllHardwareData(RobotCore& robot) {
        updateEscFeedback(robot);
        updateAccelerometers(robot);
    }

} // namespace HardwareMonitor
