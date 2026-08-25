#include "IdleMode.h"

void IdleMode::init(RobotCore& robot) {
    // Failsafe or neutral initialization
}

void IdleMode::execute(RobotCore& robot) {
    robot.hw.leftMotor->stop();
    robot.hw.rightMotor->stop();

    robot.hw.led->setIndication(LEDIndication::Idle);
}