#include "ForwardMode.h"

#include <algorithm>
#include <cstdlib>

namespace
{

    constexpr int32_t POWER_EXPO_AMOUNT = 1000;     // 0 = linear, 1000 = full expo
    constexpr int32_t POWER_EXPO_POWER = 5;         // 3 or 5
    constexpr int32_t STEERING_EXPO_AMOUNT = 1000;  // 0 = linear, 1000 = full expo
    constexpr int32_t STEERING_EXPO_POWER = 3;      // 3 or 5
    constexpr int32_t POWER_SLEW_RATE = 2500;       // command units / second
    constexpr int32_t STEERING_SLEW_RATE = 4000;    // command units / second
    constexpr int32_t MAX_STEERING_REDUCTION = 400; // 40% reduction at full throttle

    int64_t intPow(int64_t base, int32_t exponent)
    {
        int64_t result = 1;

        for (int32_t i = 0; i < exponent; ++i)
        {
            result *= base;
        }

        return result;
    }

    // y = (1 - a)x + a*x^expo_power
    // a = expo_amount / 1000
    // x, y in [-1000, 1000]
    // expo_amount in [0, 1000]
    int32_t applyExpo(
        int32_t x,
        int32_t expo_amount,
        int32_t expo_power)
    {

        constexpr int64_t SCALE = 1000;

        const int64_t x64 = x;
        const int64_t amount64 = expo_amount;

        const int64_t xp = intPow(x64, expo_power) / intPow(SCALE, expo_power - 1);

        return ((SCALE - amount64) * x64 + amount64 * xp) / SCALE;
    }

    int32_t applySlew(
        int32_t current,
        int32_t target,
        int32_t rate,
        uint32_t deltaMs)
    {

        const int64_t rate64 = rate;
        const int64_t deltaMs64 = deltaMs;

        const int32_t maxChange =
            (rate64 * deltaMs64) / 1000;

        const int32_t difference = target - current;

        if (difference > maxChange)
        {
            return current + maxChange;
        }

        if (difference < -maxChange)
        {
            return current - maxChange;
        }

        return target;
    }
} // namespace

void ForwardMode::init(RobotCore &robot)
{
    limitedThrottle_ = 0;
    limitedSteering_ = 0;
    lastUpdateMs_ = robot.state.currentMs;

    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
}

void ForwardMode::execute(RobotCore &robot)
{
    const uint32_t currentMs = robot.state.currentMs;
    const uint32_t deltaMs = currentMs - lastUpdateMs_;
    lastUpdateMs_ = currentMs;

    int32_t throttle = robot.state.receiver.throttle;
    int32_t steering = robot.state.receiver.steering;
    int32_t left = throttle;
    int32_t right = throttle;

    applyPowerExpo(throttle);
    applySteeringExpo(steering);
    applyPowerSlew(throttle, deltaMs);
    applySteeringSlew(steering, deltaMs);
    applySpeedDependentSteering(throttle, steering);
    // applyDifferentialMix(throttle, steering, left, right);
    // applyMixNormalization(left, right);

    robot.hw.leftMotor->setSpeed(static_cast<int16_t>(left), currentMs);
    robot.hw.rightMotor->setSpeed(static_cast<int16_t>(right), currentMs);
    robot.hw.led->setIndication(LEDIndication::Forward);
}

void ForwardMode::applyPowerExpo(int32_t &throttle)
{
    throttle = applyExpo(
        throttle,
        POWER_EXPO_AMOUNT,
        POWER_EXPO_POWER);
}

void ForwardMode::applySteeringExpo(int32_t &steering)
{
    steering = applyExpo(
        steering,
        STEERING_EXPO_AMOUNT,
        STEERING_EXPO_POWER);
}

void ForwardMode::applyPowerSlew(
    int32_t &throttle,
    uint32_t deltaMs)
{

    limitedThrottle_ = applySlew(
        limitedThrottle_,
        throttle,
        POWER_SLEW_RATE,
        deltaMs);

    throttle = limitedThrottle_;
}

void ForwardMode::applySteeringSlew(
    int32_t &steering,
    uint32_t deltaMs)
{

    limitedSteering_ = applySlew(
        limitedSteering_,
        steering,
        STEERING_SLEW_RATE,
        deltaMs);

    steering = limitedSteering_;
}

void ForwardMode::applySpeedDependentSteering(
    int32_t throttle,
    int32_t &steering)
{
    const int64_t throttle64 = throttle;
    const int64_t steering64 = steering;
    const int64_t maxReduction64 = MAX_STEERING_REDUCTION;

    const int32_t reduction =
        maxReduction64 * std::abs(throttle64) / 1000;

    steering =
        steering64 * (1000 - reduction) / 1000;
}


void ForwardMode::applyDifferentialMix(
    int32_t throttle,
    int32_t steering,
    int32_t &left,
    int32_t &right)
{
    left = throttle + steering;
    right = throttle - steering;
}


void ForwardMode::applyMixNormalization(
    int32_t &left,
    int32_t &right)
{
    const int32_t largest =
        std::max(std::abs(left), std::abs(right));

    if (largest <= 1000)
    {
        return;
    }

    const int64_t left64 = left;
    const int64_t right64 = right;

    left = left64 * 1000 / largest;
    right = right64 * 1000 / largest;
}