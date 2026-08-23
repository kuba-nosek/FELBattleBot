#pragma once
#include "Modes/IRobotMode.h"
#include "Modes/IdleMode.h"
#include "Modes/ForwardMode.h"
#include "Modes/SpinMode.h"

class ModeHandler {
public:
    ModeHandler();
    
    // Main update loop - handles mode switching and execution internally
    void update(RobotCore& robot);
    DriveModeType getCurrentModeType() const;

private:
    IRobotMode* _currentMode;
    DriveModeType _currentModeType;
    
    IdleMode _idleMode;
    ForwardMode _forwardMode;
    SpinMode _spinMode;
    
    void changeMode(DriveModeType newMode, RobotCore& robot);
};