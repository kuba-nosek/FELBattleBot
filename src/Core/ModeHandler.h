#pragma once
#include "Modes/IdleMode.h"
#include "IRobot.h"

class ModeHandler {
  public:
    ModeHandler();

    bool update(RobotCore& robot);
    DriveModeType decodeMode(bool leftSwitch, bool rightSwitch) const;
    DriveModeType getCurrentModeID() const;
    IRobotMode* getCurrentMode() const;

  private:
    IRobotMode* _currentMode;
    DriveModeType _currentModeType;

    IdleMode _idleMode;

    void changeMode(DriveModeType newMode, RobotCore& robot);
};
