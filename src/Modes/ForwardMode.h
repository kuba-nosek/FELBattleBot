#pragma once
#include "IRobotMode.h"

#pragma once

class ForwardMode : public IRobotMode {
public:
    void init(RobotCore& robot) override;
    void execute(RobotCore& robot) override;

private:
    int32_t limitedThrottle_ = 0;
    int32_t limitedSteering_ = 0;
    uint32_t lastUpdateMs_ = 0;

    void applyPowerExpo(int32_t& throttle);
    void applySteeringExpo(int32_t& steering);
    void applySpeedDependentSteering(int32_t throttle, int32_t& steering);
    void applyDifferentialMix(int32_t throttle, int32_t steering, int32_t& left, int32_t& right);
    void applyMixNormalization(int32_t& left, int32_t& right);
    void applyPowerSlew(int32_t& throttle, uint32_t deltaMs);
    void applySteeringSlew(int32_t& steering, uint32_t deltaMs);
};