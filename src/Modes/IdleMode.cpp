#include "IdleMode.h"

void IdleMode::init(RobotCore& robot) {
    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
    robot.hw.led->setIndication(LEDIndication::Idle);
}

void IdleMode::execute(RobotCore& robot) {
    robot.hw.leftMotor->stop();
    robot.hw.rightMotor->stop();
}