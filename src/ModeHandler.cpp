#include "ModeHandler.h"

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