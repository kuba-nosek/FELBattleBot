#pragma once

#include "IAccel.h"
#include "IReceiver.h"
#include "ITelemetryLink.h"
#include "IMotor.h"
#include "IEscFeedback.h"

#include "IndicatorLED.h"
#include "POVDisplay.h"
#include "DriveModeType.h"
#include "ReceiverInput.h"
#include "ThreadSafe.h"

#include <stdint.h>

struct RobotState {
    ThreadSafe<uint32_t> currentMs{0};
    ThreadSafe<bool> isConnected{false};
    ThreadSafe<DriveModeType> requestedMode{DriveModeType::Idle};
    
    ThreadSafe<AccelData> accel1{};
    ThreadSafe<AccelData> accel2{};
    
    // --- ESC Telemetry ---
    ThreadSafe<int32_t> escLeftRpm{0};
    ThreadSafe<float> escLeftVolts{0.0f};
    ThreadSafe<int32_t> escRightRpm{0};
    ThreadSafe<float> escRightVolts{0.0f};
};

struct RobotHardware {
    IMotor* leftMotor = nullptr;
    IMotor* rightMotor = nullptr;
    IEscFeedback* leftEscFeedback = nullptr;
    IEscFeedback* rightEscFeedback = nullptr;
    IReceiver* rx = nullptr;
    ITelemetryLink* telemetryLink = nullptr;
    IAccel* accel1 = nullptr;
    IAccel* accel2 = nullptr;
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

    virtual void fillTelemetryData(const RobotCore& robot, TelemetryData& data) const = 0;

};
