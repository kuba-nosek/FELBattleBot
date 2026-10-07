#pragma once
#include "IAccel.h"
#include "IReceiver.h"
#include "ITelemetryLink.h"
#include "IndicatorLED.h"
#include "POVDisplay.h"
#include "BattlebotTelemetry.h"
#include "DriveModeType.h"
#include "Motor.h"
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
    Motor* leftMotor = nullptr;
    Motor* rightMotor = nullptr;
    IReceiver* rx = nullptr;
    ITelemetryLink* telemetryLink = nullptr;
    IndicatorLED* indicator = nullptr;
    POVDisplay* povDisplay = nullptr;
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
