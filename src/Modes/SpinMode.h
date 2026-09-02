#pragma once

#include "IRobotMode.h"

class SpinMode : public IRobotMode {
public:
    void init(RobotCore& robot) override;
    void execute(RobotCore& robot, const ReceiverInput& input) override;

private:
    float centripetalAccelerationMps2_ = 0.0f;
    float angularSpeedRadPerSec_ = 0.0f;
    float headingRadians_ = 0.0f;
    uint32_t lastUpdateUs_ = 0;
    int8_t spinDirection_ = 1;

    int32_t calculatePower(const ReceiverInput& input) const;
    int32_t calculateAmplitude(const ReceiverInput& input) const;
    float calculateOffsetRadians(const RobotCore& robot) const;
    float calculateSensorRadiusMeters(const ReceiverInput& input) const;

    void updateCentripetalAcceleration(const IMUData& imu);
    void calculateAngularSpeed(float sensorRadiusMeters);
    void updateSpinDirection(int32_t power);
    void updateHeading(uint32_t currentUs);
    void commandSpinThrottle(
        RobotCore& robot,
        int32_t power,
        int32_t amplitude,
        float offsetRadians);
    void updateLightIndication(RobotCore& robot, uint32_t currentUs);
};
