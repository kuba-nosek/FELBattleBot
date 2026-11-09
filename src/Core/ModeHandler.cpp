#include "Core/ModeHandler.h"

ModeHandler::ModeHandler() : _currentMode(&_idleMode) {}

bool ModeHandler::update(RobotCore& robot) {
    const DriveModeType requestedMode = robot.state.requestedMode;
    const DriveModeType currentMode = robot.state.currentMode;
    if(requestedMode == currentMode) {
        return false;
    }

    changeMode(requestedMode, robot);
    return true;
}

DriveModeType ModeHandler::decodeMode(bool leftSwitch, bool rightSwitch) const {
    if(!leftSwitch) {
        return DriveModeType::Idle;
    }

    return rightSwitch ? DriveModeType::Forward : DriveModeType::Spin;
}

void ModeHandler::changeMode(DriveModeType newMode, RobotCore& robot) {
    switch(newMode) {
        case DriveModeType::Forward:
            _currentMode = &_forwardMode;
            break;
        case DriveModeType::Spin:
            _currentMode = &_spinMode;
            break;
        default:
            _currentMode = &_idleMode;
            newMode = DriveModeType::Idle;
            break;
    }

    robot.state.currentMode = newMode;

    // Trigger initialization routine of the new mode
    _currentMode->init(robot);
}

IRobotMode* ModeHandler::getCurrentMode() const {
    return _currentMode;
}
