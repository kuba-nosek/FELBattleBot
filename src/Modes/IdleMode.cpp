#include "IdleMode.h"
#include "Utils/TelemetryUtils.h"

void IdleMode::init(RobotCore& robot) {
    robot.hw.indicator->playAnimation(LEDAnimation::ModeChanged);
    robot.hw.indicator->setIndication(LEDIndication::Idle);
}

void IdleMode::execute(RobotCore& robot, const ReceiverInput&) {
    robot.hw.leftMotor->stop();
    robot.hw.rightMotor->stop();
}

void IdleMode::fillTelemetryData(const RobotCore& robot, TelemetryData& data) const {
    const DriveModeType currentMode = robot.state.currentMode;
    data.mode = static_cast<uint8_t>(currentMode);
    TelemetryUtils::fillAccelerometerData(robot, data);
}
