#pragma once
#include <stdint.h>
#include "IMU.h"
#include "LEDHandler.h"
#include "Motor.h"
#include "Receiver.h"

enum class DriveModeType {
    Invalid,
    Idle,
    Forward,
    Spin
};

// 1. Data and sensor state (What the robot feels)
struct RobotState {
    uint32_t currentMs;
    bool isConnected;
    int16_t throttle;
    int16_t steering;
    DriveModeType requestedMode;
    IMUData imu1;
    IMUData imu2;
};

// 2. Hardware abstraction (What the robot controls)
struct RobotHardware {
    Motor* leftMotor;
    Motor* rightMotor;
    LEDHandler* led;
    Receiver* rx;
};

// 3. Central context container
struct RobotCore {
    RobotState state;
    RobotHardware hw;
};

// Base interface for drive modes
class IRobotMode {
public:
    virtual ~IRobotMode() = default;
    
    // Zde je předán odkaz na celého robota
    virtual void init(RobotCore& robot) = 0;
    virtual void execute(RobotCore& robot) = 0; 
};