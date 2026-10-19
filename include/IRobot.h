#pragma once
#include "IAccel.h"
#include "IReceiver.h"
#include "IndicatorLED.h"
#include "DriveModeType.h"
#include "IMotor.h"
#include "ReceiverInput.h"
#include "ThreadSafe.h"

#include <stdint.h>

struct RobotState {
    ThreadSafe<bool> isConnected{false};
    ThreadSafe<DriveModeType> requestedMode{DriveModeType::Idle};
};

struct RobotHardware {
    IMotor* leftMotor = nullptr;
    IMotor* rightMotor = nullptr;
    IReceiver* rx = nullptr;
    IndicatorLED* indicator = nullptr;
};

struct RobotCore {
    RobotState state;
    RobotHardware hw;
};

class IRobotMode {
  public:
    virtual ~IRobotMode() = default;

    virtual void init(RobotCore& robot) = 0;

    virtual void execute(RobotCore& robot, const ReceiverInput& input) = 0;
};
