#include "BattlebotTelemetry.h"

#include <cstring>
#include <limits>

namespace BattlebotTelemetry {
namespace {

constexpr uint8_t CRSF_LENGTH = FRAME_SIZE - 2;

void writeUint16BigEndian(uint8_t* destination, uint16_t value) {
    destination[0] = static_cast<uint8_t>(value >> 8);
    destination[1] = static_cast<uint8_t>(value);
}

void writeUint32BigEndian(uint8_t* destination, uint32_t value) {
    destination[0] = static_cast<uint8_t>(value >> 24);
    destination[1] = static_cast<uint8_t>(value >> 16);
    destination[2] = static_cast<uint8_t>(value >> 8);
    destination[3] = static_cast<uint8_t>(value);
}

void writeValue(uint8_t* destination, int8_t value) {
    destination[0] = static_cast<uint8_t>(value);
}

void writeValue(uint8_t* destination, uint8_t value) {
    destination[0] = value;
}

void writeValue(uint8_t* destination, int16_t value) {
    writeUint16BigEndian(destination, static_cast<uint16_t>(value));
}

void writeValue(uint8_t* destination, uint16_t value) {
    writeUint16BigEndian(destination, value);
}

void writeValue(uint8_t* destination, int32_t value) {
    writeUint32BigEndian(destination, static_cast<uint32_t>(value));
}

void writeValue(uint8_t* destination, uint32_t value) {
    writeUint32BigEndian(destination, value);
}

void writeValue(uint8_t* destination, float value) {
    static_assert(sizeof(float) == sizeof(uint32_t), "Telemetry requires 32-bit IEEE-754 floats");
    static_assert(std::numeric_limits<float>::is_iec559, "Telemetry requires IEEE-754 floats");
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    writeUint32BigEndian(destination, bits);
}

uint8_t crc8DvbS2(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for(size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for(uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80U) != 0 ? static_cast<uint8_t>((crc << 1) ^ 0xD5U) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

} // namespace

size_t buildFrame(uint8_t* frame, size_t capacity, uint8_t sequence, const TelemetryData& telemetry) {
    if(frame == nullptr || capacity < FRAME_SIZE) {
        return 0;
    }

    frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame[1] = CRSF_LENGTH;
    frame[2] = CRSF_FRAME_TYPE_ARDUPILOT;
    frame[3] = PRIVATE_SUBTYPE;
    frame[4] = PROTOCOL_VERSION;
    frame[5] = sequence;

    uint32_t validFields = 0;
    size_t fieldIndex = 0;
#define TELEMETRY_SET_VALID_BIT(name, type)                                                                            \
    if(telemetry.name.hasValue()) {                                                                                    \
        validFields |= UINT32_C(1) << fieldIndex;                                                                      \
    }                                                                                                                  \
    ++fieldIndex;

    TELEMETRY_FIELD_MAP(TELEMETRY_SET_VALID_BIT)

#undef TELEMETRY_SET_VALID_BIT

    static_cast<void>(fieldIndex);
    writeUint32BigEndian(&frame[6], validFields);

    size_t offset = 10;
#define TELEMETRY_WRITE_FIELD(name, type)                                                                              \
    writeValue(&frame[offset], telemetry.name.hasValue() ? telemetry.name.value() : type{});                           \
    offset += sizeof(type);

    TELEMETRY_FIELD_MAP(TELEMETRY_WRITE_FIELD)

#undef TELEMETRY_WRITE_FIELD

    frame[FRAME_SIZE - 1] = crc8DvbS2(&frame[2], FRAME_SIZE - 3);
    return FRAME_SIZE;
}

} // namespace BattlebotTelemetry
