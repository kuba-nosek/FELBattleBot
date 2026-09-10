#include "ModeHandler.h"

ModeHandler::ModeHandler() : _currentMode(&_idleMode), _currentModeType(DriveModeType::Idle) {}

void ModeHandler::update(RobotCore& robot) {
    if(robot.state.requestedMode != _currentModeType) {
        changeMode(robot.state.requestedMode, robot);
    }
}

DriveModeType ModeHandler::decodeMode(bool leftSwitch, bool rightSwitch) const {
    if(!leftSwitch) {
        return DriveModeType::Idle;
    }

    return rightSwitch ? DriveModeType::Forward : DriveModeType::Spin;
}

void ModeHandler::changeMode(DriveModeType newMode, RobotCore& robot) {
    _currentModeType = newMode;

    switch(newMode) {
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

DriveModeType ModeHandler::getCurrentModeID() const {
    return _currentModeType;
}

IRobotMode* ModeHandler::getCurrentMode() const {
    return _currentMode;
}
