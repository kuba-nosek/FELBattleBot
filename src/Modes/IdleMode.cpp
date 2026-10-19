#include "IdleMode.h"

void IdleMode::init(RobotCore& robot) {
    robot.hw.indicator->playAnimation(LEDAnimation::ModeChanged);
    robot.hw.indicator->setIndication(LEDIndication::Idle);
}

void IdleMode::execute(RobotCore& robot, const ReceiverInput&) {
    robot.hw.leftMotor->stop();
    robot.hw.rightMotor->stop();
}
