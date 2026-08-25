#include "SpinMode.h"

void SpinMode::init(RobotCore& robot) {
    // Mode independently triggers its visual feedback
    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
}

void SpinMode::execute(RobotCore& robot) {
    // Tank spin output
    robot.hw.leftMotor->setSpeed(robot.state.receiver.throttle, robot.state.currentMs);
    robot.hw.rightMotor->setSpeed(-robot.state.receiver.throttle, robot.state.currentMs);
    robot.hw.led->setIndication(LEDIndication::Spin);
}