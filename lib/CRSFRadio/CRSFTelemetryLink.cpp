#include "CRSFTelemetryLink.h"
#include <cstring>
#include <limits>

namespace {
    constexpr uint8_t CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8;
    constexpr uint8_t CRSF_FRAME_TYPE_ARDUPILOT = 0x80;
    constexpr uint8_t PRIVATE_SUBTYPE = 0xF3;
    constexpr uint8_t PROTOCOL_VERSION = 1;
    constexpr size_t MAX_FRAME_SIZE = 64; // Dostatečně velký buffer pro payload

    // Pomocné funkce pro zápis Big-Endian dat
    void writeValue(uint8_t* dest, uint8_t value) { dest[0] = value; }
    
    void writeValue(uint8_t* dest, int16_t value) {
        dest[0] = static_cast<uint8_t>(value >> 8);
        dest[1] = static_cast<uint8_t>(value);
    }
    
    void writeValue(uint8_t* dest, uint16_t value) {
        dest[0] = static_cast<uint8_t>(value >> 8);
        dest[1] = static_cast<uint8_t>(value);
    }
    
    void writeValue(uint8_t* dest, int32_t value) {
        dest[0] = static_cast<uint8_t>(value >> 24);
        dest[1] = static_cast<uint8_t>(value >> 16);
        dest[2] = static_cast<uint8_t>(value >> 8);
        dest[3] = static_cast<uint8_t>(value);
    }

    void writeValue(uint8_t* dest, float value) {
        uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        writeValue(dest, static_cast<int32_t>(bits));
    }
} // namespace

CRSFTelemetryLink::CRSFTelemetryLink(HardwareSerial& serial) : serial_(serial) {}

bool CRSFTelemetryLink::sendTelemetry(const TelemetryData& data) {
    uint8_t frame[MAX_FRAME_SIZE];
    
    frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame[2] = CRSF_FRAME_TYPE_ARDUPILOT;
    frame[3] = PRIVATE_SUBTYPE;
    frame[4] = PROTOCOL_VERSION;
    frame[5] = sequence_;

    uint32_t validFields = 0;
    size_t fieldIndex = 0;

    // 1. Průchod makrem pro označení naplněných polí v bitové masce
#define TELEMETRY_SET_VALID_BIT(name, type) \
    if(data.name.has_value()) { validFields |= (1U << fieldIndex); } \
    ++fieldIndex;

    TELEMETRY_FIELD_MAP(TELEMETRY_SET_VALID_BIT)
#undef TELEMETRY_SET_VALID_BIT

    writeValue(&frame[6], static_cast<int32_t>(validFields));

    // 2. Zápis samotných dat (pokud data nejsou, vloží se nuly)
    size_t offset = 10;

#define TELEMETRY_WRITE_FIELD(name, type) \
    writeValue(&frame[offset], data.name.has_value() ? data.name.value() : type{}); \
    offset += sizeof(type);

    TELEMETRY_FIELD_MAP(TELEMETRY_WRITE_FIELD)
#undef TELEMETRY_WRITE_FIELD

    const size_t frameLength = offset + 1; // Započtení CRC
    frame[1] = frameLength - 2;            // Délka CRSF = Payload + Type + CRC

    // 3. Výpočet CRC a uložení na konec rámce
    frame[offset] = crc8DvbS2(&frame[2], frameLength - 3);

    // 4. Odeslání do portu
    if (serial_.availableForWrite() >= static_cast<int>(frameLength)) {
        serial_.write(frame, frameLength);
        ++sequence_;
        return true;
    }
    
    return false;
}

uint8_t CRSFTelemetryLink::crc8DvbS2(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for(size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for(uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80U) != 0 ? static_cast<uint8_t>((crc << 1) ^ 0xD5U) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}
