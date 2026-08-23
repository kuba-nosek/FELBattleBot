#include "SpinMode.h"

void SpinMode::init(RobotCore& robot) {
    // Mode independently triggers its visual feedback
    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
}

void SpinMode::execute(RobotCore& robot) {
    // Tank spin output
    robot.hw.leftMotor->setSpeed(robot.state.throttle, robot.state.currentMs);
    robot.hw.rightMotor->setSpeed(-robot.state.throttle, robot.state.currentMs);
    robot.hw.led->setIndication(LEDIndication::Spin);
    
    // TODO: Zde budeme přidávat výpočty pro úhlovou rychlost a volání MeltySync
}