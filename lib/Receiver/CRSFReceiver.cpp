#include "CRSFReceiver.h"

#include <Arduino.h>

namespace {
HardwareSerial crsfSerial(1); // UART1 mapping for CRSF

// --- Configuration ---
constexpr uint32_t CRSF_BAUD_RATE = 420000;
constexpr uint32_t CRSF_FAILSAFE_TIMEOUT_MS = 500;

constexpr uint8_t CRSF_FLIGHT_CONTROLLER_ADDRESS = 0xC8;
constexpr uint8_t CRSF_RECEIVER_ADDRESS = 0xEC;
constexpr uint8_t CRSF_TRANSMITTER_ADDRESS = 0xEE;
constexpr uint8_t RC_CHANNELS_PACKED = 0x16;
constexpr uint8_t LINK_STATISTICS = 0x14;

bool isValidAddress(uint8_t address) {
    if(address == CRSF_FLIGHT_CONTROLLER_ADDRESS) return true;
    if(address == CRSF_RECEIVER_ADDRESS) return true;
    if(address == CRSF_TRANSMITTER_ADDRESS) return true;
    return false;
}

// Helper: DVB-S2 CRC8 checksum calculator
uint8_t crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for(size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for(uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
        }
    }
    return crc;
}

// Helper: Map raw 11-bit CRSF data to standard RC microseconds (988 - 2012 us)
uint16_t rawToMicroseconds(const uint16_t raw) {
    int32_t scaled = 988 + (static_cast<int32_t>(raw) - 172) * 1024 / 1639;
    if(scaled < 988) return 988;
    if(scaled > 2012) return 2012;
    return static_cast<uint16_t>(scaled);
}
} // namespace

CRSFReceiver::CRSFReceiver(uint8_t rxPin, uint8_t txPin)
    : _rxPin(rxPin), _txPin(txPin), _lastValidFrameMs(0),
      _connected(false), _disconnectCallback(nullptr) {
    _position = 0;
    _expectedSize = 0;
    for(size_t i = 0; i < ReceiverChannels::COUNT; ++i) {
        _channels[i] = 1500; // Initialize channels to neutral
    }
}

void CRSFReceiver::connect() {
    crsfSerial.begin(CRSF_BAUD_RATE, SERIAL_8N1, _rxPin, _txPin);
}

void CRSFReceiver::onDisconnect(DisconnectCallback callback) {
    _disconnectCallback = callback;
}

bool CRSFReceiver::isConnected() const {
    return _connected;
}

ReceiverChannels CRSFReceiver::getChannelsSnapshot() const {
    ReceiverChannels snapshot{};

    portENTER_CRITICAL(&_channelsMux);
    for(size_t channel = 0; channel < ReceiverChannels::COUNT; ++channel) {
        snapshot.channelsUs[channel] = _channels[channel];
    }
    portEXIT_CRITICAL(&_channelsMux);

    return snapshot;
}

ReceiverStats CRSFReceiver::getStatistics() const {
    return _stats;
}

// ====================================================================
// CONTINUOUS POLLING & FAILSAFE
// ====================================================================

void CRSFReceiver::update() {
    // Process incoming serial buffer
    size_t bytesProcessed = 0;
    while(crsfSerial.available() > 0 && bytesProcessed < 256) {
        processByte(crsfSerial.read());
        bytesProcessed++;
    }

    // Trigger failsafe if no valid frames received within timeout
    if(_connected && (millis() - _lastValidFrameMs > CRSF_FAILSAFE_TIMEOUT_MS)) {
        _connected = false;
        if(_disconnectCallback != nullptr) {
            _disconnectCallback(); // Escalate to higher logic layer
        }
    }
}

// ====================================================================
// CRSF FRAME PARSER STATE MACHINE
// ====================================================================

void CRSFReceiver::processByte(uint8_t byte) {
    // 1. Sync & Device Address
    if(_position == 0) {
        if(isValidAddress(byte)) {
            _frame[_position++] = byte;
        }
        return;
    }

    // 2. Frame Length
    if(_position == 1) {
        if(byte < 2 || byte > 62) {
            _position = 0; // Invalid size, reset parser
            processByte(byte);
            return;
        }
        _frame[_position++] = byte;
        _expectedSize = byte + 2;
        return;
    }

    // 3. Accumulate Payload
    _frame[_position++] = byte;
    if(_position < _expectedSize) {
        return;
    }

    // 4. Validate CRC and Decode
    size_t crcIndex = _expectedSize - 1;
    if(crc8(&_frame[2], crcIndex - 2) == _frame[crcIndex]) {
        uint8_t frameType = _frame[2];

        // Decode RC Joystick/Switch Data
        if(frameType == RC_CHANNELS_PACKED) {
            ReceiverChannels decodedChannels{};
            const uint8_t* payload = &_frame[3];
            uint32_t bitBuffer = 0;
            uint8_t bitsAvailable = 0;
            size_t payloadIndex = 0;

            for(size_t channelIndex = 0; channelIndex < ReceiverChannels::COUNT; ++channelIndex) {
                while(bitsAvailable < 11) {
                    bitBuffer |= static_cast<uint32_t>(payload[payloadIndex++]) << bitsAvailable;
                    bitsAvailable += 8;
                }
                decodedChannels.channelsUs[channelIndex] = rawToMicroseconds(bitBuffer & 0x07FF);
                bitBuffer >>= 11;
                bitsAvailable -= 11;
            }

            portENTER_CRITICAL(&_channelsMux);
            for(size_t channel = 0; channel < ReceiverChannels::COUNT; ++channel) {
                _channels[channel] = decodedChannels.channelsUs[channel];
            }
            portEXIT_CRITICAL(&_channelsMux);

            _lastValidFrameMs = millis();
            _connected = true;
        }
        // Decode RF Link Statistics
        else if(frameType == LINK_STATISTICS) {
            const uint8_t* payload = &_frame[3];
            _stats.activeRssiDbm = -static_cast<int16_t>(payload[4] == 0 ? payload[0] : payload[1]);
            _stats.linkQuality = payload[2];
        }
    }

    _position = 0; // Reset parser for next frame
}