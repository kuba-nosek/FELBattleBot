#pragma once

#include "config.h"
#include "Utils/Kinematics.h"

// Logging configuration: only this file needs editing to change the capture.
namespace RobotConfig {
constexpr uint32_t LOG_DURATION_MS = 5000;
constexpr uint32_t LOG_SETTLE_IDLE_MS = 400;
}

// Each entry is: X(CSV_column_name, C++ expression).
// Available inside a sample: a1, a2 (AccelData), robot (RobotCore), input (ReceiverInput).
// Units of accelerometer values are m/s^2 AFTER driver calibration offsets.
// Keep name identifiers unique. Timestamp (t_us) is included automatically.

#define BB_LOG_FIELDS(X)                                                \
    X(accel1_x_mps2, a1.xMps2)                                         \
    X(accel1_y_mps2, a1.yMps2)                                         \
    X(accel2_x_mps2, a2.xMps2)                                         \
    X(accel2_y_mps2, a2.yMps2)                                         \
    X(translation_x_mps2, Kinematics2::getTranslationalAcceleration(a1, a2).xMps2) \
    X(translation_y_mps2, Kinematics2::getTranslationalAcceleration(a1, a2).yMps2) \
    X(centripetal_old_mps2, Kinematics::getCentripetalAcceleration(a1, a2)) \
    X(centripetal_new_mps2, Kinematics2::getCentripetalAcceleration(a1, a2)) \
    X(angular_accel_rad_s2, Kinematics2::getAngularAcceleration(a1, a2))
    


// Examples: append a backslash to the line above before adding a new field.
// X(esc_left_rpm, robot.state.escLeftRpm)
// X(esc_right_rpm, robot.state.escRightRpm)
// X(rc_spin_command, input.leftStickVertical)
// X(spin_mode, robot.state.requestedMode == DriveModeType::Spin ? 1.0f : 0.0f)
