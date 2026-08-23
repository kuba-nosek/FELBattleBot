#include "ModeHandler.h"

// ====================================================================
// IDLE MODE
// ====================================================================
void IdleMode::init(RobotCore& robot) {
    // Failsafe or neutral initialization
}

void IdleMode::execute(RobotCore& robot) {
    robot.hw.leftMotor->stop();
    robot.hw.rightMotor->stop();

    if (!robot.state.isConnected) {
        robot.hw.led->setIndication(LEDIndication::Failsafe);
    } else {
        robot.hw.led->setIndication(LEDIndication::Idle);
    }
}

// ====================================================================
// FORWARD MODE
// ====================================================================
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

// ====================================================================
// SPIN MODE
// ====================================================================
void SpinMode::init(RobotCore& robot) {
    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
}

void SpinMode::execute(RobotCore& robot) {
    // Tank spin output
    robot.hw.leftMotor->setSpeed(robot.state.throttle, robot.state.currentMs);
    robot.hw.rightMotor->setSpeed(-robot.state.throttle, robot.state.currentMs);
    robot.hw.led->setIndication(LEDIndication::Spin);
}

// ====================================================================
// MODE HANDLER CORE
// ====================================================================
ModeHandler::ModeHandler() 
    : _currentMode(&_idleMode), _currentModeType(DriveModeType::Idle) {
}

void ModeHandler::update(RobotCore& robot) {
    // 1. Check for requested mode transitions
    if (robot.state.requestedMode != _currentModeType) {
        changeMode(robot.state.requestedMode, robot);
    }
    
    // 2. Execute active mode logic
    _currentMode->execute(robot);
}

void ModeHandler::changeMode(DriveModeType newMode, RobotCore& robot) {
    _currentModeType = newMode;

    switch (newMode) {
        case DriveModeType::Forward:
            _currentMode = &_forwardMode;
            break;
        case DriveModeType::Spin:
            _currentMode = &_spinMode;
            break;
        default:
            _currentMode = &_idleMode;
            _currentModeType = DriveModeType::Idle;
            break;
    }

    // Trigger initialization routine of the new mode
    _currentMode->init(robot);
}

DriveModeType ModeHandler::getCurrentModeType() const {
    return _currentModeType;
}