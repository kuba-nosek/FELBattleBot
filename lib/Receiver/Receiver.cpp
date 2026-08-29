#include "Receiver.h"
#include <Arduino.h>

namespace {
    HardwareSerial crsfSerial(1); // UART1 mapping for CRSF
    
    // --- Configuration ---
    constexpr uint32_t CRSF_BAUD_RATE = 420000;
    constexpr uint32_t CRSF_FAILSAFE_TIMEOUT_MS = 500;
    
    // --- CRSF Protocol Constants ---[cite: 1, 2, 7, 8]
    constexpr uint8_t CRSF_FLIGHT_CONTROLLER_ADDRESS = 0xC8;
    constexpr uint8_t CRSF_RECEIVER_ADDRESS = 0xEC;
    constexpr uint8_t CRSF_TRANSMITTER_ADDRESS = 0xEE;
    constexpr uint8_t RC_CHANNELS_PACKED = 0x16;
    constexpr uint8_t LINK_STATISTICS = 0x14;
    constexpr uint8_t FLIGHT_MODE_FRAME_TYPE = 0x21;

    // Helper: DVB-S2 CRC8 checksum calculator[cite: 1]
    uint8_t crc8(const uint8_t *data, size_t length) {
        uint8_t crc = 0;
        for (size_t i = 0; i < length; ++i) {
            crc ^= data[i];
            for (uint8_t bit = 0; bit < 8; ++bit) {
                crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
            }
        }
        return crc;
    }

    // Helper: Map raw 11-bit CRSF data to standard RC microseconds (988 - 2012 us)[cite: 1]
    uint16_t rawToMicroseconds(const uint16_t raw) {
        int32_t scaled = 988 + (static_cast<int32_t>(raw) - 172) * 1024 / 1639;
        if (scaled < 988) return 988;
        if (scaled > 2012) return 2012;
        return static_cast<uint16_t>(scaled);
    }
}

Receiver::Receiver(uint8_t rxPin, uint8_t txPin) 
    : _rxPin(rxPin), _txPin(txPin), _lastValidFrameMs(0), _lastTelemetryMs(0), 
      _connected(false), _disconnectCallback(nullptr) {
    _position = 0;
    _expectedSize = 0;
    for (size_t i = 0; i < ReceiverChannels::COUNT; ++i) {
        _channels[i] = 1500; // Initialize channels to neutral
    }
}

void Receiver::connect() {
    crsfSerial.begin(CRSF_BAUD_RATE, SERIAL_8N1, _rxPin, _txPin);
}

void Receiver::onDisconnect(DisconnectCallback callback) {
    _disconnectCallback = callback;
}

bool Receiver::isConnected() const {
    return _connected;
}

ReceiverChannels Receiver::getChannelsSnapshot() const {
    ReceiverChannels snapshot{};

    portENTER_CRITICAL(&_channelsMux);
    for (size_t channel = 0; channel < ReceiverChannels::COUNT; ++channel) {
        snapshot.channelsUs[channel] = _channels[channel];
    }
    portEXIT_CRITICAL(&_channelsMux);

    return snapshot;
}

ReceiverStats Receiver::getStatistics() const {
    return _stats;
}

// ====================================================================
// CONTINUOUS POLLING & FAILSAFE
// ====================================================================

void Receiver::update() {
    // Process incoming serial buffer
    size_t bytesProcessed = 0;
    while (crsfSerial.available() > 0 && bytesProcessed < 256) {
        processByte(crsfSerial.read());
        bytesProcessed++;
    }

    // Trigger failsafe if no valid frames received within timeout[cite: 5]
    if (_connected && (millis() - _lastValidFrameMs > CRSF_FAILSAFE_TIMEOUT_MS)) {
        _connected = false;
        if (_disconnectCallback != nullptr) {
            _disconnectCallback(); // Escalate to higher logic layer
        }
    }
}

// ====================================================================
// CRSF FRAME PARSER STATE MACHINE
// ====================================================================

void Receiver::processByte(uint8_t byte) {
    // 1. Sync & Device Address[cite: 1]
    if (_position == 0) {
        if (byte == CRSF_FLIGHT_CONTROLLER_ADDRESS || 
            byte == CRSF_RECEIVER_ADDRESS || 
            byte == CRSF_TRANSMITTER_ADDRESS) {
            _frame[_position++] = byte;
        }
        return;
    }

    // 2. Frame Length[cite: 1]
    if (_position == 1) {
        if (byte < 2 || byte > 62) {
            _position = 0; // Invalid size, reset parser
            processByte(byte);
            return;
        }
        _frame[_position++] = byte;
        _expectedSize = byte + 2;
        return;
    }

    // 3. Accumulate Payload[cite: 1]
    _frame[_position++] = byte;
    if (_position < _expectedSize) {
        return;
    }

    // 4. Validate CRC and Decode[cite: 1]
    size_t crcIndex = _expectedSize - 1;
    if (crc8(&_frame[2], crcIndex - 2) == _frame[crcIndex]) {
        uint8_t frameType = _frame[2];
        
        // Decode RC Joystick/Switch Data[cite: 1]
        if (frameType == RC_CHANNELS_PACKED) {
            ReceiverChannels decodedChannels{};
            const uint8_t *payload = &_frame[3];
            uint32_t bitBuffer = 0;
            uint8_t bitsAvailable = 0;
            size_t payloadIndex = 0;

            for (size_t channelIndex = 0;
                 channelIndex < ReceiverChannels::COUNT;
                 ++channelIndex) {
                while (bitsAvailable < 11) {
                    bitBuffer |= static_cast<uint32_t>(payload[payloadIndex++]) << bitsAvailable;
                    bitsAvailable += 8;
                }
                decodedChannels.channelsUs[channelIndex] =
                    rawToMicroseconds(bitBuffer & 0x07FF);
                bitBuffer >>= 11;
                bitsAvailable -= 11;
            }

            portENTER_CRITICAL(&_channelsMux);
            for (size_t channel = 0; channel < ReceiverChannels::COUNT; ++channel) {
                _channels[channel] = decodedChannels.channelsUs[channel];
            }
            portEXIT_CRITICAL(&_channelsMux);
            
            _lastValidFrameMs = millis();
            _connected = true;
        }
        // Decode RF Link Statistics[cite: 1]
        else if (frameType == LINK_STATISTICS) {
            const uint8_t *payload = &_frame[3];
            _stats.activeRssiDbm = -static_cast<int16_t>(payload[4] == 0 ? payload[0] : payload[1]);
            _stats.linkQuality = payload[2];
        }
    }
    
    _position = 0; // Reset parser for next frame
}

// ====================================================================
// TELEMETRY TRANSMISSION
// ====================================================================

void Receiver::sendTelemetry(const char* statusText, uint32_t currentMs) {
    // Rate limit to 2 Hz to prevent bandwidth saturation
    if (currentMs - _lastTelemetryMs < 500) return; 

    if (statusText == nullptr) return;

    size_t textLength = 0;
    while (textLength < 20 && statusText[textLength] != '\0') textLength++;

    // Build standard Flight Mode text frame
    // Future: Expand to send Battery Voltage or Melty Brain RPM data back to the radio
    uint8_t frame[32];
    frame[0] = CRSF_FLIGHT_CONTROLLER_ADDRESS;
    frame[1] = textLength + 3; // Type (1) + Text (X) + NUL (1) + CRC (1)[cite: 2]
    frame[2] = FLIGHT_MODE_FRAME_TYPE;

    for (size_t i = 0; i < textLength; ++i) {
        frame[3 + i] = statusText[i];
    }
    frame[3 + textLength] = '\0';
    frame[4 + textLength] = crc8(&frame[2], textLength + 2);

    size_t frameSize = textLength + 5;
    if (crsfSerial.availableForWrite() >= frameSize) {
        crsfSerial.write(frame, frameSize);
        _lastTelemetryMs = currentMs;
    }
}
