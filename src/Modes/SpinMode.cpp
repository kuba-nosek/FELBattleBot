#include "SpinMode.h"

void SpinMode::init(RobotCore& robot) {
    // Mode independently triggers its visual feedback
    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
    robot.hw.led->setIndication(LEDIndication::Spin);
}

void SpinMode::execute(RobotCore& robot) {
    // Tank spin output
    robot.hw.leftMotor->setSpeed(robot.state.receiver.throttle, robot.state.currentMs);
    robot.hw.rightMotor->setSpeed(-robot.state.receiver.throttle, robot.state.currentMs);
    
}