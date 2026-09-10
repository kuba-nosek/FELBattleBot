#pragma once

#include <stddef.h>
#include <stdint.h>

namespace BattlebotTelemetry {

constexpr uint8_t CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8;
constexpr uint8_t CRSF_FRAME_TYPE_ARDUPILOT = 0x80;
constexpr uint8_t PRIVATE_SUBTYPE = 0xF3;
constexpr uint8_t PROTOCOL_VERSION = 1;

constexpr size_t SENSOR_COUNT = 2;
constexpr size_t AXIS_COUNT = 3;
constexpr size_t FRAME_SIZE = 26;

enum ValidField : uint8_t {
    VALID_RPM = 1U << 0,
    VALID_ACCEL1_X = 1U << 1,
    VALID_ACCEL1_Y = 1U << 2,
    VALID_ACCEL1_Z = 1U << 3,
    VALID_ACCEL2_X = 1U << 4,
    VALID_ACCEL2_Y = 1U << 5,
    VALID_ACCEL2_Z = 1U << 6,
};

struct Snapshot {
    int16_t rpm = 0;
    int16_t accelerationCentiG[SENSOR_COUNT][AXIS_COUNT]{};
    uint16_t peakMagnitudeCentiG[SENSOR_COUNT]{};
    uint8_t validMask = 0;
};

// The private payload transports acceleration in 0.01 g units. The physical
// sensors are configured for +/-100 g, so values are limited to that range.
int16_t encodeAccelerationCentiG(float accelerationG);

// Peak vector magnitude can reach sqrt(3) * 100 g even when every individual
// axis remains inside its +/-100 g range.
uint16_t encodePeakMagnitudeCentiG(float accelerationG);

// RPM is transported as a signed value so the spin direction is retained.
int16_t encodeRpm(float rpm);

// Builds a CRSF 0x80 frame with the private 0xF3 battlebot payload.
size_t buildFrame(uint8_t* frame, size_t capacity, uint8_t sequence, const Snapshot& snapshot);

} // namespace BattlebotTelemetry
