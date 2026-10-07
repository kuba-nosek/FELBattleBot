#pragma once
#include "BattlebotTelemetry.h"
#include "DriveModeType.h"
#include "IAccel.h"
#include "LEDHandler.h"
#include "Motor.h"
#include "Receiver.h"
#include "ReceiverInput.h"

#include <stdint.h>

struct RobotState {
    uint32_t currentMs = 0;
    bool isConnected = false;
    DriveModeType requestedMode = DriveModeType::Idle;
    AccelData accel1{};
    AccelData accel2{};
    // --- ESC Telemetry ---
    int32_t escLeftRpm = 0;
    float escLeftVolts = 0.0f;
    int32_t escRightRpm = 0;
    float escRightVolts = 0.0f;
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


class IRobotMode {
  public:
    virtual ~IRobotMode() = default;

    virtual void init(RobotCore& robot) = 0;

    virtual void execute(RobotCore& robot, const ReceiverInput& input) = 0;

    virtual void sendTelemetry(const RobotCore& robot, BattlebotTelemetry::TelemetryData& telemetry) const = 0;
};
