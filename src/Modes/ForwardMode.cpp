#include "ForwardMode.h"

void ForwardMode::init(RobotCore& robot) {
    // Mode independently triggers its visual feedback
    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
}

void ForwardMode::execute(RobotCore& robot) {
    int32_t left = robot.state.throttle;
    int32_t right = robot.state.throttle;

    // Differential steering mix
    if (robot.state.steering < 0) {
        left = left * (1000 + robot.state.steering) / 1000;
    } else if (robot.state.steering > 0) {
        right = right * (1000 - robot.state.steering) / 1000;
    }

    // Direct hardware output
    robot.hw.leftMotor->setSpeed(static_cast<int16_t>(left), robot.state.currentMs);
    robot.hw.rightMotor->setSpeed(static_cast<int16_t>(right), robot.state.currentMs);
    robot.hw.led->setIndication(LEDIndication::Forward);
}