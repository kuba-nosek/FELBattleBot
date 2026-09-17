#include "SpinMode.h"

#include <Arduino.h>
#include <cmath>

namespace {
struct MotorPowers {
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

static_assert(MIN_SENSOR_RADIUS_METERS > 0.0f && MAX_SENSOR_RADIUS_METERS >= MIN_SENSOR_RADIUS_METERS,
              "Sensor radius range must be positive");

float wrapRadians(float angleRadians) {
    const float wrapped = std::fmod(angleRadians, TWO_PI_RADIANS);
    return wrapped < 0.0f ? wrapped + TWO_PI_RADIANS : wrapped;
}

float calculateSensorRadiusMeters(int32_t potentiometer) {
    const float shiftedPotentiometer = static_cast<float>(potentiometer) + RobotConfig::RC_OUTPUT_SCALE;
    const float fullPotentiometerRange = 2.0f * RobotConfig::RC_OUTPUT_SCALE;
    const float potentiometerPosition = shiftedPotentiometer / fullPotentiometerRange;

    return MIN_SENSOR_RADIUS_METERS + potentiometerPosition * (MAX_SENSOR_RADIUS_METERS - MIN_SENSOR_RADIUS_METERS);
}

float calculateConstantHeadingOffsetRadians(int32_t potentiometer) {
    return static_cast<float>(potentiometer) / RobotConfig::RC_OUTPUT_SCALE * PI_RADIANS;
}

float calculateAngularSpeed(float centripetalAccelerationMps2, float sensorRadiusMeters) {
    // Centripetal acceleration: a = omega^2 * radius.
    return std::sqrt(centripetalAccelerationMps2 / sensorRadiusMeters);
}

float calculateTimeUntilHeadingOffsetUs(float headingRadians, float headingSpeedRadPerSec) {
    float radiansUntilHeadingOffset = headingRadians;

    if(headingSpeedRadPerSec > 0.0f) {
        radiansUntilHeadingOffset = 0.0f;
        if(headingRadians > 0.0f) radiansUntilHeadingOffset = TWO_PI_RADIANS - headingRadians;
    }

    return (radiansUntilHeadingOffset / std::abs(headingSpeedRadPerSec)) * SECONDS_TO_MICROSECONDS;
}

bool isFlashDelayValid(float flashDelayUs) {
    return std::isfinite(flashDelayUs) && flashDelayUs >= 0.0f && flashDelayUs <= INT32_MAX;
}

MotorPowers calculateMotorPowers(int32_t power, int32_t amplitude, float headingRadians) {
    const float modulation = std::sin(headingRadians);
    const int32_t wave = static_cast<int32_t>(std::lround(static_cast<float>(amplitude) * modulation));

    return {power + wave, -power + wave};
}
} // namespace

void SpinMode::init(RobotCore& robot) {
    headingRadians_ = 0.0f;
    constantHeadingOffsetRadians_ = 0.0f;
    variableHeadingOffsetRadians_ = 0.0f;
    lastUpdateUs_ = micros();
    spinDirection_ = 1;
    angularSpeedRadPerSec_ = 0.0f;

    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
    robot.hw.led->setIndication(LEDIndication::Spin);
    robot.hw.led->cancelScheduledFlash();
}

void SpinMode::updateHeading(float deltaSeconds, float angularSpeedRadPerSec) {
    headingRadians_ = wrapRadians(headingRadians_ + spinDirection_ * angularSpeedRadPerSec * deltaSeconds);
}

void SpinMode::execute(RobotCore& robot, const ReceiverInput& input) {
    const uint32_t currentUs = micros();
    const uint32_t deltaUs = currentUs - lastUpdateUs_;
    lastUpdateUs_ = currentUs;

    const float deltaSeconds = static_cast<float>(deltaUs) * MICROSECONDS_TO_SECONDS;
    const int32_t power = input.leftStickVertical;
    const int32_t amplitude = input.rightStickVertical;

    const float sensorRadiusMeters = calculateSensorRadiusMeters(input.leftPot);

    // Use the XY-plane magnitude so small mounting-angle errors do not affect
    // the radial acceleration estimate.
    const float centripetalAccelerationMps2 = std::hypot(robot.state.imu1.xMps2, robot.state.imu1.yMps2);

    angularSpeedRadPerSec_ = calculateAngularSpeed(centripetalAccelerationMps2, sensorRadiusMeters);

    if(power != 0) spinDirection_ = power > 0 ? 1 : -1;

    if(std::isfinite(angularSpeedRadPerSec_)) updateHeading(deltaSeconds, angularSpeedRadPerSec_);

    const float constantHeadingOffsetRadians = calculateConstantHeadingOffsetRadians(input.rightPot);

    constantHeadingOffsetRadians_ = constantHeadingOffsetRadians;
    const float headingInput = static_cast<float>(input.rightStickHorizontal) / RobotConfig::RC_OUTPUT_SCALE;
    const float headingChangeSpeedRadPerSec = headingInput * MAX_HEADING_CHANGE_SPEED_RAD_PER_SEC;
    const float headingChangeRadians = headingChangeSpeedRadPerSec * deltaSeconds;
    const float updatedVariableOffsetRadians = variableHeadingOffsetRadians_ + headingChangeRadians;
    variableHeadingOffsetRadians_ = wrapRadians(updatedVariableOffsetRadians);

    const float combinedHeadingRadians =
        headingRadians_ + constantHeadingOffsetRadians_ + variableHeadingOffsetRadians_;
    const float correctedHeadingRadians = wrapRadians(combinedHeadingRadians);
    const float correctedAngularSpeedRadPerSec = spinDirection_ * angularSpeedRadPerSec_ + headingChangeSpeedRadPerSec;

    const MotorPowers motorPowers = calculateMotorPowers(power, amplitude, correctedHeadingRadians);

    robot.hw.leftMotor->setSpeed(static_cast<int16_t>(motorPowers.left), robot.state.currentMs);
    robot.hw.rightMotor->setSpeed(static_cast<int16_t>(motorPowers.right), robot.state.currentMs);

    if(angularSpeedRadPerSec_ <= 0.0f || !std::isfinite(angularSpeedRadPerSec_)) {
        robot.hw.led->cancelScheduledFlash();
        return;
    }

    const float timeUntilHeadingOffsetUs =
        calculateTimeUntilHeadingOffsetUs(correctedHeadingRadians, correctedAngularSpeedRadPerSec);

    if(!isFlashDelayValid(timeUntilHeadingOffsetUs)) {
        robot.hw.led->cancelScheduledFlash();
        return;
    }

    robot.hw.led->scheduleFlash(static_cast<uint32_t>(timeUntilHeadingOffsetUs), LED_FLASH_DURATION_US,
                                LED_MINIMUM_TIME_BETWEEN_FLASHES_US);
}

void SpinMode::sendTelemetry(const RobotCore& robot, BattlebotTelemetry::TelemetryData& telemetry) const {
    telemetry.mode = DriveModeType::Spin;
    telemetry.accel1X = robot.state.imu1.xMps2;
    telemetry.accel1Y = robot.state.imu1.yMps2;
    telemetry.accel1Z = robot.state.imu1.zMps2;
    telemetry.accel2X = robot.state.imu2.xMps2;
    telemetry.accel2Y = robot.state.imu2.yMps2;
    telemetry.accel2Z = robot.state.imu2.zMps2;

    const float rpm = spinDirection_ * angularSpeedRadPerSec_ * (60.0f / TWO_PI_RADIANS);
    telemetry.rpm = static_cast<int16_t>(rpm);
}
