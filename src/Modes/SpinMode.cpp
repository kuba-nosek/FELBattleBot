#include "SpinMode.h"
#include "config.h"
#include "Utils/Kinematics.h"

#include <Arduino.h>
#include <cmath>
#include <algorithm> // Přidáno pro použití std::clamp

using namespace RobotConfig;

namespace {
struct MotorPowers {
    int32_t left;
    int32_t right;
};

constexpr float MIN_SENSOR_RADIUS_METERS = 0.014f;
constexpr float MAX_SENSOR_RADIUS_METERS = 0.016f;
constexpr uint32_t LED_FLASH_DURATION_US = 500;
constexpr uint32_t LED_MINIMUM_TIME_BETWEEN_FLASHES_US = 1000;

constexpr float TWO_PI_RADIANS = 6.28318530718f;
constexpr float PI_RADIANS = TWO_PI_RADIANS / 2.0f;
constexpr float MAX_HEADING_CHANGE_SPEED_RAD_PER_SEC = TWO_PI_RADIANS;
constexpr float MICROSECONDS_TO_SECONDS = 0.000001f;
constexpr float SECONDS_TO_MICROSECONDS = 1000000.0f;

static_assert(MIN_SENSOR_RADIUS_METERS > 0.0f && MAX_SENSOR_RADIUS_METERS >= MIN_SENSOR_RADIUS_METERS,
              "Sensor radius range must be positive");

static_assert(SPIN_MAX_TRANSLATION_RATIO >= 0.0f && SPIN_MAX_TRANSLATION_RATIO < 1.0f,
              "Spin translation ratio must be in the range [0, 1)");

float wrapRadians(float angleRadians) {
    const float wrapped = std::fmod(angleRadians, TWO_PI_RADIANS);
    return wrapped < 0.0f ? wrapped + TWO_PI_RADIANS : wrapped;
}

float calculateSensorRadiusMeters(int32_t potentiometer) {
    const float shiftedPotentiometer = static_cast<float>(potentiometer) + RC_OUTPUT_SCALE;
    const float fullPotentiometerRange = 2.0f * RC_OUTPUT_SCALE;
    const float potentiometerPosition = shiftedPotentiometer / fullPotentiometerRange;

    return MIN_SENSOR_RADIUS_METERS + potentiometerPosition * (MAX_SENSOR_RADIUS_METERS - MIN_SENSOR_RADIUS_METERS);
}

float calculateConstantHeadingOffsetRadians(int32_t potentiometer) {
    return static_cast<float>(potentiometer) / RC_OUTPUT_SCALE * PI_RADIANS;
}

float calculateAngularSpeed(float centripetalAccelerationMps2, float sensorRadiusMeters) {
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

int32_t getPower(int32_t rawPower, int switchPosition) {
    if (switchPosition < 0) {
        return rawPower / 3;
    } else if (switchPosition == 0) {
        return (rawPower * 2) / 3;
    } else {
        return rawPower;
    }
}

void limitTranslationAmplitude(int32_t power, int32_t& forwardAmplitude, int32_t& lateralAmplitude) {
    // Limit the combined translation vector, not each axis separately.
    const float amplitude = std::hypot(static_cast<float>(forwardAmplitude),
                                       static_cast<float>(lateralAmplitude));
    const float maxAmplitude = static_cast<float>(std::abs(power)) * SPIN_MAX_TRANSLATION_RATIO;

    // Keep modulation smaller than spin power, preventing motor direction reversals.
    if (amplitude > maxAmplitude) {
        const float scale = maxAmplitude / amplitude;
        forwardAmplitude = static_cast<int32_t>(forwardAmplitude * scale);
        lateralAmplitude = static_cast<int32_t>(lateralAmplitude * scale);
    }
}

void limitTranslation(int32_t& power, int32_t& forwardAmplitude, int32_t& lateralAmplitude) {
    limitTranslationAmplitude(power, forwardAmplitude, lateralAmplitude);

    // Scale all three values together when the motor commands could exceed +/-1000.
    const float maxOutput = static_cast<float>(std::abs(power)) +
                            std::hypot(static_cast<float>(forwardAmplitude),
                                       static_cast<float>(lateralAmplitude));
    if (maxOutput > RC_OUTPUT_SCALE) {
        const float scale = static_cast<float>(RC_OUTPUT_SCALE) / maxOutput;
        power = static_cast<int32_t>(power * scale);
        forwardAmplitude = static_cast<int32_t>(forwardAmplitude * scale);
        lateralAmplitude = static_cast<int32_t>(lateralAmplitude * scale);
        // Truncating spin power can slightly reduce the safety margin.
        limitTranslationAmplitude(power, forwardAmplitude, lateralAmplitude);
    }
}

MotorPowers calculateMotorPowers(int32_t power, int32_t forwardAmplitude, int32_t lateralAmplitude,
                                 float headingRadians) {
    // Two 90-degree shifted components give independent forward and sideways translation.
    const float modulation = static_cast<float>(forwardAmplitude) * std::sin(headingRadians) +
                             static_cast<float>(lateralAmplitude) * std::cos(headingRadians);
    const int32_t wave = static_cast<int32_t>(std::lround(modulation));

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

    if (robot.hw.indicator) {
        robot.hw.indicator->playAnimation(LEDAnimation::ModeChanged);
        robot.hw.indicator->setIndication(LEDIndication::Spin);
        robot.hw.indicator->cancelScheduledFlash();
    }

    if (robot.hw.povDisplay) {
        robot.hw.povDisplay->setMode(POVDisplayMode::Spinning);
        robot.hw.povDisplay->setAnimation(LEDStripAnimation::Off);
        robot.hw.povDisplay->setAngularVelocity(0.0f);
    }
}

void SpinMode::updateHeading(float deltaSeconds, float angularSpeedRadPerSec) {
    headingRadians_ = wrapRadians(headingRadians_ + spinDirection_ * angularSpeedRadPerSec * deltaSeconds);
}

void SpinMode::execute(RobotCore& robot, const ReceiverInput& input) {
    const AccelData a1 = robot.state.accel1;
    const AccelData a2 = robot.state.accel2;
    const uint32_t currentMs = robot.state.currentMs;

    const uint32_t currentUs = micros();
    const uint32_t deltaUs = currentUs - lastUpdateUs_;
    lastUpdateUs_ = currentUs;

    const float deltaSeconds = static_cast<float>(deltaUs) * MICROSECONDS_TO_SECONDS;
    int32_t power = getPower(input.leftStickVertical, input.left3StateSwitch);
    
    //amplitude for forward and side
    int32_t forwardAmplitude = getPower(input.rightStickVertical, input.left3StateSwitch);
    int32_t lateralAmplitude = getPower(input.leftStickHorizontal, input.left3StateSwitch);

    if (Kinematics::getAverageZ(a1, a2) < 0.0f) {
        forwardAmplitude = -forwardAmplitude;
        lateralAmplitude = -lateralAmplitude;
    }

    limitTranslation(power, forwardAmplitude, lateralAmplitude);

    const float sensorRadiusMeters = calculateSensorRadiusMeters(input.leftPot);

    const float centripetalAccelerationMps2 = Kinematics::getCentripetalAcceleration(a1, a2);

    angularSpeedRadPerSec_ = calculateAngularSpeed(centripetalAccelerationMps2, sensorRadiusMeters);

    if(power != 0) spinDirection_ = power > 0 ? 1 : -1;

    const float signedAngularVelocityRadPerSec = spinDirection_ * angularSpeedRadPerSec_;
    
    if (robot.hw.povDisplay) {
        robot.hw.povDisplay->setAngularVelocity(signedAngularVelocityRadPerSec);
        robot.hw.povDisplay->setAnimation(LEDStripImages::animationForSpinSwitchPosition(input.sixStateSwitch));
    }

    if(std::isfinite(angularSpeedRadPerSec_)) updateHeading(deltaSeconds, angularSpeedRadPerSec_);

    const float constantHeadingOffsetRadians = calculateConstantHeadingOffsetRadians(input.rightPot);

    constantHeadingOffsetRadians_ = constantHeadingOffsetRadians;
    const float headingInput = static_cast<float>(input.rightStickHorizontal) / RC_OUTPUT_SCALE;
    const float headingChangeSpeedRadPerSec = headingInput * MAX_HEADING_CHANGE_SPEED_RAD_PER_SEC;
    const float headingChangeRadians = headingChangeSpeedRadPerSec * deltaSeconds;
    const float updatedVariableOffsetRadians = variableHeadingOffsetRadians_ + headingChangeRadians;
    variableHeadingOffsetRadians_ = wrapRadians(updatedVariableOffsetRadians);

    const float combinedHeadingRadians =
        headingRadians_ + constantHeadingOffsetRadians_ + variableHeadingOffsetRadians_;
    const float correctedHeadingRadians = wrapRadians(combinedHeadingRadians);
    const float correctedAngularSpeedRadPerSec = spinDirection_ * angularSpeedRadPerSec_ + headingChangeSpeedRadPerSec;

    if(power == 0) {
        forwardAmplitude = 0;
        lateralAmplitude = 0;
    }
    const MotorPowers motorPowers = calculateMotorPowers(power, forwardAmplitude, lateralAmplitude, correctedHeadingRadians);

    robot.hw.leftMotor->setSpeed(static_cast<int16_t>(motorPowers.left), currentMs);
    robot.hw.rightMotor->setSpeed(static_cast<int16_t>(motorPowers.right), currentMs);

    if(angularSpeedRadPerSec_ <= 0.0f || !std::isfinite(angularSpeedRadPerSec_)) {
        if (robot.hw.indicator) {
            robot.hw.indicator->cancelScheduledFlash();
        }
        return;
    }

    const float timeUntilHeadingOffsetUs =
        calculateTimeUntilHeadingOffsetUs(correctedHeadingRadians, correctedAngularSpeedRadPerSec);

    if(!isFlashDelayValid(timeUntilHeadingOffsetUs)) {
        if (robot.hw.indicator) {
            robot.hw.indicator->cancelScheduledFlash();
        }
        return;
    }

    if (robot.hw.indicator) {
        robot.hw.indicator->scheduleFlash(static_cast<uint32_t>(timeUntilHeadingOffsetUs), LED_FLASH_DURATION_US,
                                          LED_MINIMUM_TIME_BETWEEN_FLASHES_US);
    }
}

void SpinMode::sendTelemetry(const RobotCore& robot, BattlebotTelemetry::TelemetryData& telemetry) const {
    telemetry.mode = DriveModeType::Spin;

    AccelData a1 = robot.state.accel1;
    AccelData a2 = robot.state.accel2;
    
    telemetry.accel1X = a1.xMps2;
    telemetry.accel1Y = a1.yMps2;
    telemetry.accel1Z = a1.zMps2;
    
    telemetry.accel2X = a2.xMps2;
    telemetry.accel2Y = a2.yMps2;
    telemetry.accel2Z = a2.zMps2;

    telemetry.escLeftRpm = robot.state.escLeftRpm;
    telemetry.escLeftVolts = robot.state.escLeftVolts;
    telemetry.escRightRpm = robot.state.escRightRpm;
    telemetry.escRightVolts = robot.state.escRightVolts;

    const float rpm = spinDirection_ * angularSpeedRadPerSec_ * (60.0f / TWO_PI_RADIANS);
    telemetry.rpm = static_cast<int16_t>(rpm);
}
