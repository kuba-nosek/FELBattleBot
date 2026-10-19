#include "IdleMode.h"

void IdleMode::init(RobotCore& robot) {
    robot.hw.indicator->playAnimation(LEDAnimation::ModeChanged);
    robot.hw.indicator->setIndication(LEDIndication::Idle);
}

void IdleMode::execute(RobotCore& robot, const ReceiverInput&) {
    robot.hw.leftMotor->stop();
    robot.hw.rightMotor->stop();
}

void IdleMode::sendTelemetry(const RobotCore& robot, BattlebotTelemetry::TelemetryData& telemetry) const {
    telemetry.mode = DriveModeType::Idle;

    AccelData a1 = robot.state.accel1;
    AccelData a2 = robot.state.accel2;
    
    telemetry.accel1X = a1.xMps2;
    telemetry.accel1Y = a1.yMps2;
    telemetry.accel1Z = a1.zMps2;
    
    telemetry.accel2X = a2.xMps2;
    telemetry.accel2Y = a2.yMps2;
    telemetry.accel2Z = a2.zMps2;
}

