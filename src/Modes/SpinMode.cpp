#include "SpinMode.h"

#include <Arduino.h>
#include <cmath>

namespace
{
    // Hardware/timing placeholders to be calibrated on the finished robot.
    constexpr float MIN_SENSOR_RADIUS_METERS = 0.015f;
    constexpr float MAX_SENSOR_RADIUS_METERS = 0.05f;
    constexpr uint32_t LED_PHASE_OFFSET_US = 0;
    constexpr uint32_t LED_FLASH_DURATION_US = 2000;

    constexpr float STANDARD_GRAVITY_MPS2 = 9.80665f;
    constexpr float TWO_PI_RADIANS = 6.28318530718f;
    constexpr float MICROSECONDS_TO_SECONDS = 0.000001f;
    constexpr float SECONDS_TO_MICROSECONDS = 1000000.0f;

    static_assert(
        MIN_SENSOR_RADIUS_METERS > 0.0f &&
        MAX_SENSOR_RADIUS_METERS >= MIN_SENSOR_RADIUS_METERS,
        "Sensor radius range must be positive");

    float wrapRadians(float angleRadians)
    {
        const float wrapped = std::fmod(angleRadians, TWO_PI_RADIANS);
        return wrapped < 0.0f ? wrapped + TWO_PI_RADIANS : wrapped;
    }
}

void SpinMode::init(RobotCore& robot)
{
    centripetalAccelerationMps2_ = 0.0f;
    angularSpeedRadPerSec_ = 0.0f;
    headingRadians_ = 0.0f;
    lastUpdateUs_ = micros();
    spinDirection_ = 1;

    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
    robot.hw.led->setMeltySync(0, 0, LED_FLASH_DURATION_US);
}

void SpinMode::execute(RobotCore& robot, const ReceiverInput& input)
{
    const uint32_t currentUs = micros();
    const int32_t power = calculatePower(input);
    const int32_t amplitude = calculateAmplitude(input);
    const float offsetRadians = calculateOffsetRadians(robot);
    const float sensorRadiusMeters = calculateSensorRadiusMeters(input);

    updateCentripetalAcceleration(robot.state.imu1);
    calculateAngularSpeed(sensorRadiusMeters);
    updateSpinDirection(power);
    updateHeading(currentUs);
    commandSpinThrottle(robot, power, amplitude, offsetRadians);
    updateLightIndication(robot, currentUs);
}

int32_t SpinMode::calculatePower(const ReceiverInput& input) const
{
    return input.leftStickVertical;
}

int32_t SpinMode::calculateAmplitude(const ReceiverInput& input) const
{
    return input.leftStickHorizontal;
}

float SpinMode::calculateOffsetRadians(const RobotCore&) const
{
    return 0.0f;
}

float SpinMode::calculateSensorRadiusMeters(const ReceiverInput& input) const
{
    const float potentiometerPosition =
        (static_cast<float>(input.leftPot) + RobotConfig::RC_OUTPUT_SCALE) /
        (2.0f * RobotConfig::RC_OUTPUT_SCALE);

    return MIN_SENSOR_RADIUS_METERS + potentiometerPosition *
        (MAX_SENSOR_RADIUS_METERS - MIN_SENSOR_RADIUS_METERS);
}

void SpinMode::updateCentripetalAcceleration(const IMUData& imu)
{
    // Use the XY-plane magnitude so small mounting-angle errors do not affect
    // the radial acceleration estimate.
    const float radialAccelerationG = std::hypot(imu.xG, imu.yG);
    centripetalAccelerationMps2_ =
        radialAccelerationG * STANDARD_GRAVITY_MPS2;
}

void SpinMode::calculateAngularSpeed(float sensorRadiusMeters)
{
    if (centripetalAccelerationMps2_ <= 0.0f)
    {
        angularSpeedRadPerSec_ = 0.0f;
        return;
    }

    // Centripetal acceleration: a = omega^2 * radius.
    angularSpeedRadPerSec_ = std::sqrt(
        centripetalAccelerationMps2_ / sensorRadiusMeters);
}

void SpinMode::updateSpinDirection(int32_t power)
{
    if (power > 0)
    {
        spinDirection_ = 1;
    }
    else if (power < 0)
    {
        spinDirection_ = -1;
    }
}

void SpinMode::updateHeading(uint32_t currentUs)
{
    const uint32_t deltaUs = currentUs - lastUpdateUs_;
    lastUpdateUs_ = currentUs;

    const float deltaSeconds =
        static_cast<float>(deltaUs) * MICROSECONDS_TO_SECONDS;
    const float signedAngularSpeed =
        spinDirection_ * angularSpeedRadPerSec_;

    headingRadians_ = wrapRadians(
        headingRadians_ + signedAngularSpeed * deltaSeconds);
}

void SpinMode::commandSpinThrottle(
    RobotCore& robot,
    int32_t power,
    int32_t amplitude,
    float offsetRadians)
{
    const float modulation = std::sin(headingRadians_ + offsetRadians);
    const int32_t wave = static_cast<int32_t>(
        std::lround(static_cast<float>(amplitude) * modulation));

    const int32_t left = power + wave;
    const int32_t right = -power + wave;

    robot.hw.leftMotor->setSpeed(static_cast<int16_t>(left), robot.state.currentMs);
    robot.hw.rightMotor->setSpeed(static_cast<int16_t>(right), robot.state.currentMs);
}

void SpinMode::updateLightIndication(RobotCore& robot, uint32_t currentUs)
{
    if (angularSpeedRadPerSec_ <= 0.0f)
    {
        robot.hw.led->setMeltySync(0, 0, LED_FLASH_DURATION_US);
        return;
    }

    const float periodUsFloat =
        (TWO_PI_RADIANS / angularSpeedRadPerSec_) *
        SECONDS_TO_MICROSECONDS;

    if (periodUsFloat <= 0.0f || periodUsFloat > UINT32_MAX)
    {
        robot.hw.led->setMeltySync(0, 0, LED_FLASH_DURATION_US);
        return;
    }

    const float phaseSinceZeroRadians = spinDirection_ > 0
        ? headingRadians_
        : wrapRadians(-headingRadians_);
    const float elapsedSinceZeroUs =
        (phaseSinceZeroRadians / angularSpeedRadPerSec_) *
        SECONDS_TO_MICROSECONDS;

    const uint32_t zeroHeadingUs =
        currentUs - static_cast<uint32_t>(elapsedSinceZeroUs + 0.5f);

    robot.hw.led->setMeltySync(
        static_cast<uint32_t>(periodUsFloat + 0.5f),
        zeroHeadingUs + LED_PHASE_OFFSET_US,
        LED_FLASH_DURATION_US);
}
