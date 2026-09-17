#include "ModeHandler.h"

ModeHandler::ModeHandler() : _currentMode(&_idleMode), _currentModeType(DriveModeType::Idle) {}

bool ModeHandler::update(RobotCore& robot) {
    if(robot.state.requestedMode == _currentModeType) {
        return false;
    }

    changeMode(robot.state.requestedMode, robot);
    return true;
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
