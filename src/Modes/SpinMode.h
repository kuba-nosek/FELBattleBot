#pragma once

#include "IRobotMode.h"

class SpinMode : public IRobotMode {
public:
    void init(RobotCore& robot) override;
    void execute(RobotCore& robot) override;

private:
    int32_t rotationalAccelerationMg_ = 0;
    uint32_t translationalAccelerationMg_ = 0;
    uint32_t angularSpeedMilliRadPerSec_ = 0;
    uint64_t angularSpeedPhasePerSec_ = 0;
    uint32_t headingPhase_ = 0;
    uint32_t lastUpdateUs_ = 0;
    int8_t spinDirection_ = 1;

    int32_t calculatePower(const RobotCore& robot) const;
    int32_t calculateAmplitude(const RobotCore& robot) const;
    uint32_t calculateOffsetPhase(const RobotCore& robot) const;

    void calculateAccelerations(const IMUData& imu1, const IMUData& imu2);
    void calculateAngularSpeed();
    void updateSpinDirection(int32_t power);
    void updateHeading(uint32_t currentUs);
    void applyMeltyMix(
        RobotCore& robot,
        int32_t power,
        int32_t amplitude,
        uint32_t offsetPhase);
    void updateMeltySync(RobotCore& robot, uint32_t currentUs);
};
