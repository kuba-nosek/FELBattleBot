#include "BattlebotTelemetry.h"

#include <cmath>

namespace BattlebotTelemetry {
namespace {

constexpr uint8_t CRSF_LENGTH = FRAME_SIZE - 2;
constexpr float ACCELERATION_LIMIT_G = 100.0f;
constexpr float PEAK_MAGNITUDE_LIMIT_G = 173.21f;

void writeUint16BigEndian(uint8_t* destination, uint16_t value)
{
    destination[0] = static_cast<uint8_t>(value >> 8);
    destination[1] = static_cast<uint8_t>(value);
}

uint8_t crc8DvbS2(const uint8_t* data, size_t length)
{
    uint8_t crc = 0;
    for (size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80U) != 0
                ? static_cast<uint8_t>((crc << 1) ^ 0xD5U)
                : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

float finiteAndConstrained(float value, float minimum, float maximum)
{
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

}  // namespace

int16_t encodeAccelerationCentiG(float accelerationG)
{
    const float limited = finiteAndConstrained(
        accelerationG, -ACCELERATION_LIMIT_G, ACCELERATION_LIMIT_G);
    return static_cast<int16_t>(std::lround(limited * 100.0f));
}

uint16_t encodePeakMagnitudeCentiG(float accelerationG)
{
    const float limited = finiteAndConstrained(
        accelerationG, 0.0f, PEAK_MAGNITUDE_LIMIT_G);
    return static_cast<uint16_t>(std::lround(limited * 100.0f));
}

int16_t encodeRpm(float rpm)
{
    const float limited = finiteAndConstrained(rpm, -32768.0f, 32767.0f);
    return static_cast<int16_t>(std::lround(limited));
}

size_t buildFrame(uint8_t* frame, size_t capacity, uint8_t sequence,
                  const Snapshot& snapshot)
{
    if (frame == nullptr || capacity < FRAME_SIZE) {
        return 0;
    }

    frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame[1] = CRSF_LENGTH;
    frame[2] = CRSF_FRAME_TYPE_ARDUPILOT;
    frame[3] = PRIVATE_SUBTYPE;
    frame[4] = PROTOCOL_VERSION;
    frame[5] = sequence;
    frame[6] = snapshot.validMask;
    writeUint16BigEndian(&frame[7], static_cast<uint16_t>(snapshot.rpm));

    size_t offset = 9;
    for (size_t sensor = 0; sensor < SENSOR_COUNT; ++sensor) {
        for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
            writeUint16BigEndian(
                &frame[offset],
                static_cast<uint16_t>(snapshot.accelerationCentiG[sensor][axis]));
            offset += 2;
        }
    }

    for (size_t sensor = 0; sensor < SENSOR_COUNT; ++sensor) {
        writeUint16BigEndian(
            &frame[offset], snapshot.peakMagnitudeCentiG[sensor]);
        offset += 2;
    }

    // CRC covers frame type and payload, excluding address and length.
    frame[FRAME_SIZE - 1] = crc8DvbS2(&frame[2], FRAME_SIZE - 3);
    return FRAME_SIZE;
}

}  // namespace BattlebotTelemetry
