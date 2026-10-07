#include "IdleMode.h"

void IdleMode::init(RobotCore& robot) {
    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
    robot.hw.led->setIndication(LEDIndication::Idle);
    robot.hw.led->setStripMode(LEDStripMode::Static);
    robot.hw.led->setAnimation(LEDStripAnimation::Idle);
}

void IdleMode::execute(RobotCore& robot, const ReceiverInput&) {
    robot.hw.leftMotor->stop();
    robot.hw.rightMotor->stop();
}

void IdleMode::sendTelemetry(const RobotCore& robot, BattlebotTelemetry::TelemetryData& telemetry) const {
    telemetry.mode = DriveModeType::Idle;
    telemetry.accel1X = robot.state.accel1.xMps2;
    telemetry.accel1Y = robot.state.accel1.yMps2;
    telemetry.accel1Z = robot.state.accel1.zMps2;
    telemetry.accel2X = robot.state.accel2.xMps2;
    telemetry.accel2Y = robot.state.accel2.yMps2;
    telemetry.accel2Z = robot.state.accel2.zMps2;
}
