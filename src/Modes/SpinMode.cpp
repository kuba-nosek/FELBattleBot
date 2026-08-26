#include "SpinMode.h"

#include <Arduino.h>

namespace
{
    // Hardware/timing placeholders to be calibrated on the finished robot.
    constexpr uint32_t SENSOR_RADIUS_MM = 50; // Replace with measured center-to-IMU radius.
    constexpr uint32_t LED_PHASE_OFFSET_US = 0;
    constexpr uint32_t LED_FLASH_DURATION_US = 2000;

    constexpr uint64_t MICROSECONDS_PER_SECOND = 1000000ULL;
    constexpr uint64_t PHASE_UNITS_PER_TURN = 1ULL << 32;
    constexpr uint64_t TWO_PI_MICRORAD = 6283185ULL;
    constexpr int32_t SINE_Q15_SCALE = 32767;

    // sin(0..pi/2), sampled at 64 equal intervals and scaled to Q15.
    constexpr int16_t QUARTER_SINE_Q15[65] = {
        0, 804, 1608, 2410, 3212, 4011, 4808, 5602,
        6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793,
        12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
        18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
        23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
        27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
        30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
        32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
        32767
    };

    uint32_t integerSqrt(uint64_t value)
    {
        uint64_t result = 0;
        uint64_t bit = 1ULL << 62;

        while (bit > value)
        {
            bit >>= 2;
        }

        while (bit != 0)
        {
            if (value >= result + bit)
            {
                value -= result + bit;
                result = (result >> 1) + bit;
            }
            else
            {
                result >>= 1;
            }

            bit >>= 2;
        }

        return static_cast<uint32_t>(result);
    }

    int16_t sineQ15(uint32_t phase)
    {
        const uint8_t index = static_cast<uint8_t>(phase >> 24);
        const uint8_t quadrant = index >> 6;
        const uint8_t offset = index & 0x3F;

        switch (quadrant)
        {
            case 0:
                return QUARTER_SINE_Q15[offset];
            case 1:
                return QUARTER_SINE_Q15[64 - offset];
            case 2:
                return -QUARTER_SINE_Q15[offset];
            default:
                return -QUARTER_SINE_Q15[64 - offset];
        }
    }
}

void SpinMode::init(RobotCore& robot)
{
    rotationalAccelerationMg_ = 0;
    translationalAccelerationMg_ = 0;
    angularSpeedMilliRadPerSec_ = 0;
    angularSpeedPhasePerSec_ = 0;
    headingPhase_ = 0;
    lastUpdateUs_ = micros();
    spinDirection_ = 1;

    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
    robot.hw.led->setMeltySync(0, 0, LED_FLASH_DURATION_US);
}

void SpinMode::execute(RobotCore& robot)
{
    const uint32_t currentUs = micros();
    const int32_t power = calculatePower(robot);
    const int32_t amplitude = calculateAmplitude(robot);
    const uint32_t offsetPhase = calculateOffsetPhase(robot);

    calculateAccelerations(robot.state.imu1, robot.state.imu2);
    calculateAngularSpeed();
    updateSpinDirection(power);
    updateHeading(currentUs);
    applyMeltyMix(robot, power, amplitude, offsetPhase);
    updateMeltySync(robot, currentUs);
}

int32_t SpinMode::calculatePower(const RobotCore& robot) const
{
    return robot.state.receiver.throttle;
}

int32_t SpinMode::calculateAmplitude(const RobotCore& robot) const
{
    return robot.state.receiver.steering;
}

uint32_t SpinMode::calculateOffsetPhase(const RobotCore&) const
{
    // One full turn is 2^32: 0x40000000 is 90 degrees.
    return 0;
}

void SpinMode::calculateAccelerations(const IMUData& imu1, const IMUData& imu2)
{
    rotationalAccelerationMg_ = static_cast<int32_t>(
        ((imu1.y - imu2.y) * 1000.0f) / 2.0f);

    const int32_t translationalXMg = static_cast<int32_t>(
        ((imu1.x + imu2.x) * 1000.0f) / 2.0f);
    const int32_t translationalYMg = static_cast<int32_t>(
        ((imu1.y + imu2.y) * 1000.0f) / 2.0f);
    const int64_t x = translationalXMg;
    const int64_t y = translationalYMg;

    translationalAccelerationMg_ = integerSqrt(
        static_cast<uint64_t>((x * x) + (y * y)));
}

void SpinMode::calculateAngularSpeed()
{
    static_assert(SENSOR_RADIUS_MM > 0, "Sensor radius must be non-zero");

    if (rotationalAccelerationMg_ <= 0)
    {
        angularSpeedMilliRadPerSec_ = 0;
        angularSpeedPhasePerSec_ = 0;
        return;
    }

    // omega^2 [mrad^2/s^2] = acceleration [mg] * 9,806,650 / radius [mm]
    const uint64_t angularSpeedSquared =
        static_cast<uint64_t>(rotationalAccelerationMg_) * 9806650ULL /
        SENSOR_RADIUS_MM;

    angularSpeedMilliRadPerSec_ = integerSqrt(angularSpeedSquared);

    // Convert mrad/s to unsigned 32-bit-turn phase units per second.
    angularSpeedPhasePerSec_ =
        static_cast<uint64_t>(angularSpeedMilliRadPerSec_) *
        PHASE_UNITS_PER_TURN * 1000ULL /
        TWO_PI_MICRORAD;
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

    const uint64_t phaseChange =
        angularSpeedPhasePerSec_ * deltaUs /
        MICROSECONDS_PER_SECOND;

    if (spinDirection_ > 0)
    {
        headingPhase_ += static_cast<uint32_t>(phaseChange);
    }
    else
    {
        headingPhase_ -= static_cast<uint32_t>(phaseChange);
    }
}

void SpinMode::applyMeltyMix(
    RobotCore& robot,
    int32_t power,
    int32_t amplitude,
    uint32_t offsetPhase)
{
    const int32_t sine = sineQ15(headingPhase_ + offsetPhase);
    const int32_t wave =
        static_cast<int64_t>(amplitude) * sine /
        SINE_Q15_SCALE;

    const int32_t left = power + wave;
    const int32_t right = -power + wave;

    robot.hw.leftMotor->setSpeed(static_cast<int16_t>(left), robot.state.currentMs);
    robot.hw.rightMotor->setSpeed(static_cast<int16_t>(right), robot.state.currentMs);
}

void SpinMode::updateMeltySync(RobotCore& robot, uint32_t currentUs)
{
    if (angularSpeedPhasePerSec_ == 0)
    {
        robot.hw.led->setMeltySync(0, 0, LED_FLASH_DURATION_US);
        return;
    }

    const uint64_t periodUs64 =
        PHASE_UNITS_PER_TURN * MICROSECONDS_PER_SECOND /
        angularSpeedPhasePerSec_;

    if (periodUs64 == 0 || periodUs64 > UINT32_MAX)
    {
        robot.hw.led->setMeltySync(0, 0, LED_FLASH_DURATION_US);
        return;
    }

    const uint32_t phaseSinceZero = spinDirection_ > 0
        ? headingPhase_
        : 0U - headingPhase_;

    const uint64_t elapsedSinceZeroUs =
        static_cast<uint64_t>(phaseSinceZero) * MICROSECONDS_PER_SECOND /
        angularSpeedPhasePerSec_;

    const uint32_t zeroHeadingUs =
        currentUs - static_cast<uint32_t>(elapsedSinceZeroUs);

    robot.hw.led->setMeltySync(
        static_cast<uint32_t>(periodUs64),
        zeroHeadingUs + LED_PHASE_OFFSET_US,
        LED_FLASH_DURATION_US);
}
