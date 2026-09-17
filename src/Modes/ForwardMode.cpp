#include "ForwardMode.h"

#include "config.h"

#include <algorithm>
#include <cstdlib>

namespace {
struct MotorPowers {
    int32_t left;
    int32_t right;
};

constexpr int32_t SCALE = RobotConfig::RC_OUTPUT_SCALE;
constexpr int32_t MAX_STANDING_TURN_POWER = RobotConfig::FORWARD_MAX_STANDING_TURN_POWER;
constexpr int32_t MAX_MOVING_TURN_PERCENT = RobotConfig::FORWARD_MAX_MOVING_TURN_PERCENT;

int32_t expoPowerFromSwitch(int8_t switchPosition) {
    if(switchPosition < 0) {
        return 1;
    }

    if(switchPosition > 0) {
        return 5;
    }

    return 3;
}

int32_t unitScaleFromPotentiometer(int16_t potentiometer) {
    return (static_cast<int32_t>(potentiometer) + RobotConfig::RC_OUTPUT_SCALE) / 2;
}

int64_t integerPower(int64_t base, int32_t exponent) {
    int64_t result = 1;

    for(int32_t i = 0; i < exponent; ++i) {
        result *= base;
    }

    return result;
}

// Blend the linear and exponential curves; amount is scaled 0..1000.
int32_t applyExpo(int32_t input, int32_t amount, int32_t exponent) {
    const int64_t curvedInput = integerPower(input, exponent) / integerPower(SCALE, exponent - 1);

    return static_cast<int32_t>(((SCALE - amount) * input + amount * curvedInput) / SCALE);
}

MotorPowers mixMotorPowers(int32_t throttle, int32_t steering) {
    int32_t turn = 0;

    if(throttle == 0) {
        turn = static_cast<int32_t>(static_cast<int64_t>(steering) * MAX_STANDING_TURN_POWER / SCALE);
    } else {
        // The total difference between the motor commands is at most the
        // configured percentage of the signed throttle command.
        turn = static_cast<int32_t>(static_cast<int64_t>(throttle) * steering * MAX_MOVING_TURN_PERCENT /
                                    (2 * SCALE * 100));
    }

    MotorPowers powers{throttle + turn, throttle - turn};

    const int32_t largestMagnitude = std::max(std::abs(powers.left), std::abs(powers.right));

    if(largestMagnitude > SCALE) {
        powers.left = static_cast<int32_t>(static_cast<int64_t>(powers.left) * SCALE / largestMagnitude);
        powers.right = static_cast<int32_t>(static_cast<int64_t>(powers.right) * SCALE / largestMagnitude);
    }

    return powers;
}
} // namespace

void ForwardMode::init(RobotCore& robot) {
    robot.hw.led->playAnimation(LEDAnimation::ModeChanged);
    robot.hw.led->setIndication(LEDIndication::Forward);
    robot.hw.led->setStripMode(LEDStripMode::Static);
    robot.hw.led->setAnimation(LEDStripAnimation::Forward);
}

void ForwardMode::execute(RobotCore& robot, const ReceiverInput& input) {
    const int32_t rawThrottle = input.leftStickVertical;
    const int32_t throttleExpoAmount = unitScaleFromPotentiometer(input.leftPot);
    const int32_t throttleExponent = expoPowerFromSwitch(input.left3StateSwitch);
    const int32_t throttle = applyExpo(rawThrottle, throttleExpoAmount, throttleExponent);

    const int32_t steeringExpoAmount = SCALE;
    const int32_t steeringExponent = expoPowerFromSwitch(input.right3StateSwitch);
    const int32_t curvedSteering = applyExpo(input.rightStickHorizontal, steeringExpoAmount, steeringExponent);

    const int32_t steeringScale = unitScaleFromPotentiometer(input.rightPot);
    const int64_t scaledSteering = static_cast<int64_t>(curvedSteering) * steeringScale;
    const int32_t steering = static_cast<int32_t>(scaledSteering / SCALE);

    const MotorPowers powers = mixMotorPowers(throttle, steering);

    robot.hw.leftMotor->setSpeed(static_cast<int16_t>(powers.left), robot.state.currentMs);
    robot.hw.rightMotor->setSpeed(static_cast<int16_t>(powers.right), robot.state.currentMs);
}

void ForwardMode::sendTelemetry(const RobotCore& robot, BattlebotTelemetry::TelemetryData& telemetry) const {
    telemetry.mode = DriveModeType::Forward;
    telemetry.accel1X = robot.state.imu1.xMps2;
    telemetry.accel1Y = robot.state.imu1.yMps2;
    telemetry.accel1Z = robot.state.imu1.zMps2;
    telemetry.accel2X = robot.state.imu2.xMps2;
    telemetry.accel2Y = robot.state.imu2.yMps2;
    telemetry.accel2Z = robot.state.imu2.zMps2;
}
