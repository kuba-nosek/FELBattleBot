#pragma once
#include "Modes/IRobotMode.h"
#include "Modes/IdleMode.h"
#include "Modes/ForwardMode.h"
#include "Modes/SpinMode.h"

class ModeHandler {
public:
    ModeHandler();
    
    void update(RobotCore& robot);
    DriveModeType decodeMode(bool leftSwitch, bool rightSwitch) const;
    DriveModeType getCurrentModeID() const;
    IRobotMode* getCurrentMode() const;

private:
    IRobotMode* _currentMode;
    DriveModeType _currentModeType;
    
    IdleMode _idleMode;
    ForwardMode _forwardMode;
    SpinMode _spinMode;
    
    void changeMode(DriveModeType newMode, RobotCore& robot);
};
