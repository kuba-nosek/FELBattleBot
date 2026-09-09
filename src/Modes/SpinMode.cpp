#include "SpinMode.h"

#include <Arduino.h>
#include <cmath>

namespace
{
    struct MotorPowers
    {
        int32_t left;
        int32_t right;
    };

    // Hardware/timing placeholders to be calibrated on the finished robot.
    constexpr float MIN_SENSOR_RADIUS_METERS = 0.015f;
    constexpr float MAX_SENSOR_RADIUS_METERS = 0.05f;
    constexpr uint32_t LED_FLASH_DURATION_US = 500;
    constexpr uint32_t LED_MINIMUM_TIME_BETWEEN_FLASHES_US = 1000;

    constexpr float TWO_PI_RADIANS = 6.28318530718f;
    constexpr float PI_RADIANS = TWO_PI_RADIANS / 2.0f;
    constexpr float MAX_HEADING_CHANGE_SPEED_RAD_PER_SEC = TWO_PI_RADIANS;
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

    float calculateSensorRadiusMeters(int32_t potentiometer)
    {
        const float potentiometerPosition =
            (static_cast<float>(potentiometer) + RobotConfig::RC_OUTPUT_SCALE) /
            (2.0f * RobotConfig::RC_OUTPUT_SCALE);

        return MIN_SENSOR_RADIUS_METERS + potentiometerPosition *
                                              (MAX_SENSOR_RADIUS_METERS - MIN_SENSOR_RADIUS_METERS);
    }

    float calculateConstantHeadingOffsetRadians(int32_t potentiometer)
    {
        return static_cast<float>(potentiometer) /
               RobotConfig::RC_OUTPUT_SCALE * PI_RADIANS;
    }

    float calculateAngularSpeed(
        float centripetalAccelerationMps2,
        float sensorRadiusMeters)
    {
        // Centripetal acceleration: a = omega^2 * radius.
        return std::sqrt(centripetalAccelerationMps2 / sensorRadiusMeters);
    }

    float calculateTimeUntilHeadingOffsetUs(
        float headingRadians,
        float signedAngularSpeedRadPerSec)
    {
        const float radiansUntilHeadingOffset = signedAngularSpeedRadPerSec > 0.0f
                                                    ? (headingRadians > 0.0f
                                                           ? TWO_PI_RADIANS - headingRadians
                                                           : 0.0f)
                                                    : headingRadians;

        return (radiansUntilHeadingOffset / std::abs(signedAngularSpeedRadPerSec)) *
               SECONDS_TO_MICROSECONDS;
    }

    MotorPowers calculateMotorPowers(
        int32_t power,
        int32_t amplitude,
        float headingRadians)
    {
        const float modulation = std::sin(headingRadians);
        const int32_t wave = static_cast<int32_t>(
            std::lround(static_cast<float>(amplitude) * modulation));

        return {
            power + wave,
            -power + wave};
    }
}

void SpinMode::init(RobotCore& robot)
{
    headingRadians_ = 0.0f;
    constantHeadingOffsetRadians_ = 0.0f;
    variableHeadingOffsetRadians_ = 0.0f;
    lastUpdateUs_ = micros();
    spinDirection_ = 1;

    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
    robot.hw.led->setIndication(LEDIndication::Spin);
    robot.hw.led->cancelScheduledFlash();
}

void SpinMode::updateHeading(
    float deltaSeconds,
    float angularSpeedRadPerSec)
{


    headingRadians_ = wrapRadians(
        headingRadians_ + spinDirection_ * angularSpeedRadPerSec);
}

void SpinMode::execute(RobotCore& robot, const ReceiverInput& input)
{
    const uint32_t currentUs = micros();
    const uint32_t deltaUs = currentUs - lastUpdateUs_;
    lastUpdateUs_ = currentUs;

    const float deltaSeconds =
        static_cast<float>(deltaUs) * MICROSECONDS_TO_SECONDS;
    const int32_t power = input.leftStickVertical;
    const int32_t amplitude = input.rightStickVertical;

    const float sensorRadiusMeters =
        calculateSensorRadiusMeters(input.leftPot);

    // Use the XY-plane magnitude so small mounting-angle errors do not affect
    // the radial acceleration estimate.
    const float centripetalAccelerationMps2 = std::hypot(
        robot.state.imu1.xMps2,
        robot.state.imu1.yMps2);

    const float angularSpeedRadPerSec = calculateAngularSpeed(
        centripetalAccelerationMps2,
        sensorRadiusMeters);

    if (power != 0)
        spinDirection_ = power > 0 ? 1 : -1;

    if (std::isfinite(angularSpeedRadPerSec))
        updateHeading(deltaSeconds, angularSpeedRadPerSec);

    const float constantHeadingOffsetRadians =
        calculateConstantHeadingOffsetRadians(input.rightPot);
    const bool headingOffsetChanged =
        constantHeadingOffsetRadians != constantHeadingOffsetRadians_ ||
        input.rightStickHorizontal != 0;

    constantHeadingOffsetRadians_ = constantHeadingOffsetRadians;
    const float headingChangeSpeedRadPerSec =
        static_cast<float>(input.rightStickHorizontal) /
        RobotConfig::RC_OUTPUT_SCALE *
        MAX_HEADING_CHANGE_SPEED_RAD_PER_SEC;
    variableHeadingOffsetRadians_ = wrapRadians(
        variableHeadingOffsetRadians_ +
        headingChangeSpeedRadPerSec * deltaSeconds);

    const float correctedHeadingRadians = wrapRadians(
        headingRadians_ +
        constantHeadingOffsetRadians_ +
        variableHeadingOffsetRadians_);
    const float correctedAngularSpeedRadPerSec =
        spinDirection_ * angularSpeedRadPerSec +
        headingChangeSpeedRadPerSec;

    const MotorPowers motorPowers = calculateMotorPowers(
        power,
        amplitude,
        correctedHeadingRadians);

    robot.hw.leftMotor->setSpeed(
        static_cast<int16_t>(motorPowers.left),
        robot.state.currentMs);
    robot.hw.rightMotor->setSpeed(
        static_cast<int16_t>(motorPowers.right),
        robot.state.currentMs);

    if (angularSpeedRadPerSec <= 0.0f ||
        !std::isfinite(angularSpeedRadPerSec))
    {
        robot.hw.led->cancelScheduledFlash();
        return;
    }

    const float timeUntilHeadingOffsetUs = calculateTimeUntilHeadingOffsetUs(
        correctedHeadingRadians,
        correctedAngularSpeedRadPerSec);

    if (!std::isfinite(timeUntilHeadingOffsetUs) ||
        timeUntilHeadingOffsetUs < 0.0f ||
        timeUntilHeadingOffsetUs > INT32_MAX)
    {
        robot.hw.led->cancelScheduledFlash();
        return;
    }

    if (headingOffsetChanged)
        robot.hw.led->cancelScheduledFlash();

    robot.hw.led->scheduleFlash(
        static_cast<uint32_t>(timeUntilHeadingOffsetUs),
        LED_FLASH_DURATION_US,
        LED_MINIMUM_TIME_BETWEEN_FLASHES_US);
}
