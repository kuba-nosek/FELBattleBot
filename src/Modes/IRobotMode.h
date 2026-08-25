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

struct ReceiverInput {
    int16_t throttle;
    int16_t steering;
};

struct RobotState {
    uint32_t currentMs;
    bool isConnected;
    ReceiverInput receiver;
    DriveModeType requestedMode;
    IMUData imu1;
    IMUData imu2;
};

struct RobotHardware {
    Motor* leftMotor;
    Motor* rightMotor;
    LEDHandler* led;
    Receiver* rx;
};

struct RobotCore {
    RobotState state;
    RobotHardware hw;
};

// ==============================================================================
// DRIVE MODES INTERFACE
// ==============================================================================
// This is a "contract" that every drive mode in this robot must fulfill.
// It contains no logic on its own, but guarantees to the ModeHandler (which switches modes)
// that regardless of which mode is active, these methods will reliably exist.
// This allows ModeHandler to operate generically using Polymorphism.
class IRobotMode {
public:
    // Virtual destructor (Safety measure)
    // Ensures that deleting a generic IRobotMode pointer correctly frees 
    // the memory of the derived concrete class (e.g., SpinMode), preventing memory leaks.
    virtual ~IRobotMode() = default;
    
    // Pure virtual methods (marked with "= 0")
    // The "= 0" syntax specifies that these functions have no default implementation.
    // Any derived class MUST implement them; otherwise, the code will fail to compile.
    
    // 1. Runs once upon switching to this mode (e.g., to update LED states)
    virtual void init(RobotCore& robot) = 0;
    
    // 2. Called continuously (100 times per second) within the main control loop
    virtual void execute(RobotCore& robot) = 0; 
};