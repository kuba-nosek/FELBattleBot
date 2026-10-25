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
 
    const AccelData a1 = robot.state.accel1;
    const AccelData a2 = robot.state.accel2;

    // fill common data
    TelemetryUtils::fillCommonTelemetryData(robot, data);

    // fill mode specific data
    data.accel1X = a1.xMps2;
    data.accel1Y = a1.yMps2;
    data.accel1Z = a1.zMps2;

    data.accel2X = a2.xMps2;
    data.accel2Y = a2.yMps2;
    data.accel2Z = a2.zMps2;
}
