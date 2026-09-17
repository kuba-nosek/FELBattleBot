#pragma once

#include "config.h"

#include <stddef.h>
#include <stdint.h>

namespace BattlebotTelemetry {

constexpr uint8_t CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8;
constexpr uint8_t CRSF_FRAME_TYPE_ARDUPILOT = 0x80;
constexpr uint8_t PRIVATE_SUBTYPE = 0xF3;
constexpr uint8_t PROTOCOL_VERSION = 2;

template <typename T> class TelemetryField {
  public:
    TelemetryField& operator=(T value) {
        value_ = value;
        valid_ = true;
        return *this;
    }

    TelemetryField& operator=(decltype(nullptr)) {
        reset();
        return *this;
    }

    void reset() {
        value_ = T{};
        valid_ = false;
    }

    bool hasValue() const {
        return valid_;
    }

    T value() const {
        return value_;
    }

  private:
    T value_{};
    bool valid_ = false;
};

#define TELEMETRY_DECLARE_FIELD(name, type) TelemetryField<type> name{};

struct TelemetryData {
    TELEMETRY_FIELD_MAP(TELEMETRY_DECLARE_FIELD)

    void clear() {
#define TELEMETRY_RESET_FIELD(name, type) name.reset();
        TELEMETRY_FIELD_MAP(TELEMETRY_RESET_FIELD)
#undef TELEMETRY_RESET_FIELD
    }
};

#undef TELEMETRY_DECLARE_FIELD

#define TELEMETRY_COUNT_FIELD(name, type) +1
constexpr size_t FIELD_COUNT = 0 TELEMETRY_FIELD_MAP(TELEMETRY_COUNT_FIELD);
#undef TELEMETRY_COUNT_FIELD

#define TELEMETRY_TYPE_SIZE_int8_t 1
#define TELEMETRY_TYPE_SIZE_uint8_t 1
#define TELEMETRY_TYPE_SIZE_int16_t 2
#define TELEMETRY_TYPE_SIZE_uint16_t 2
#define TELEMETRY_TYPE_SIZE_int32_t 4
#define TELEMETRY_TYPE_SIZE_uint32_t 4
#define TELEMETRY_TYPE_SIZE_float 4
#define TELEMETRY_FIELD_SIZE(name, type) +TELEMETRY_TYPE_SIZE_##type

constexpr size_t SERIALIZED_FIELDS_SIZE = 0 TELEMETRY_FIELD_MAP(TELEMETRY_FIELD_SIZE);

#undef TELEMETRY_FIELD_SIZE
#undef TELEMETRY_TYPE_SIZE_float
#undef TELEMETRY_TYPE_SIZE_uint32_t
#undef TELEMETRY_TYPE_SIZE_int32_t
#undef TELEMETRY_TYPE_SIZE_uint16_t
#undef TELEMETRY_TYPE_SIZE_int16_t
#undef TELEMETRY_TYPE_SIZE_uint8_t
#undef TELEMETRY_TYPE_SIZE_int8_t

constexpr size_t VALIDITY_MASK_SIZE = sizeof(uint32_t);
constexpr size_t PRIVATE_HEADER_SIZE = 3 + VALIDITY_MASK_SIZE;
constexpr size_t PRIVATE_PAYLOAD_SIZE = PRIVATE_HEADER_SIZE + SERIALIZED_FIELDS_SIZE;
constexpr size_t FRAME_SIZE = 2 + 1 + PRIVATE_PAYLOAD_SIZE + 1;

static_assert(FIELD_COUNT <= 32, "Telemetry supports at most 32 configured fields");
static_assert(FRAME_SIZE <= 64, "Configured telemetry fields exceed the CRSF frame size limit");

size_t buildFrame(uint8_t* frame, size_t capacity, uint8_t sequence, const TelemetryData& telemetry);

} // namespace BattlebotTelemetry
