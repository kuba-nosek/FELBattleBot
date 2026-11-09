#include "CRSFReceiver.h"
#include <Arduino.h>

namespace {
// Smazáno: HardwareSerial crsfSerial(1);

constexpr uint32_t CRSF_BAUD_RATE = 420000;
constexpr uint32_t CRSF_FAILSAFE_TIMEOUT_MS = 500;

constexpr uint8_t CRSF_FLIGHT_CONTROLLER_ADDRESS = 0xC8;
constexpr uint8_t CRSF_RECEIVER_ADDRESS = 0xEC;
constexpr uint8_t CRSF_TRANSMITTER_ADDRESS = 0xEE;
constexpr uint8_t RC_CHANNELS_PACKED = 0x16;
constexpr uint8_t LINK_STATISTICS = 0x14;
constexpr uint8_t FLIGHT_MODE_FRAME_TYPE = 0x21;

bool isValidAddress(uint8_t address) {
    if(address == CRSF_FLIGHT_CONTROLLER_ADDRESS) return true;
    if(address == CRSF_RECEIVER_ADDRESS) return true;
    if(address == CRSF_TRANSMITTER_ADDRESS) return true;
    return false;
}

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

uint16_t rawToMicroseconds(const uint16_t raw) {
    int32_t scaled = 988 + (static_cast<int32_t>(raw) - 172) * 1024 / 1639;
    if(scaled < 988) return 988;
    if(scaled > 2012) return 2012;
    return static_cast<uint16_t>(scaled);
}
} // namespace

CRSFReceiver::CRSFReceiver(HardwareSerial& serial, uint8_t rxPin, uint8_t txPin)
    : _serial(serial), _rxPin(rxPin), _txPin(txPin), _lastValidFrameMs(0),
      _connected(false), _disconnectCallback(nullptr) {
    _position = 0;
    _expectedSize = 0;
    for(size_t i = 0; i < ReceiverChannels::COUNT; ++i) {
        _channels[i] = 1500; 
    }
}

void CRSFReceiver::connect() {
    _serial.begin(CRSF_BAUD_RATE, SERIAL_8N1, _rxPin, _txPin);
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

void CRSFReceiver::update() {
    size_t bytesProcessed = 0;
    while(_serial.available() > 0 && bytesProcessed < 256) {
        processByte(_serial.read());
        bytesProcessed++;
    }

    if(_connected && (millis() - _lastValidFrameMs > CRSF_FAILSAFE_TIMEOUT_MS)) {
        _connected = false;
        if(_disconnectCallback != nullptr) {
            _disconnectCallback(); 
        }
    }
}

void CRSFReceiver::processByte(uint8_t byte) {
    if(_position == 0) {
        if(isValidAddress(byte)) {
            _frame[_position++] = byte;
        }
        return;
    }

    if(_position == 1) {
        if(byte < 2 || byte > 62) {
            _position = 0; 
            processByte(byte);
            return;
        }
        _frame[_position++] = byte;
        _expectedSize = byte + 2;
        return;
    }

    _frame[_position++] = byte;
    if(_position < _expectedSize) {
        return;
    }

    size_t crcIndex = _expectedSize - 1;
    if(crc8(&_frame[2], crcIndex - 2) == _frame[crcIndex]) {
        uint8_t frameType = _frame[2];

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
        else if(frameType == LINK_STATISTICS) {
            const uint8_t* payload = &_frame[3];
            _stats.activeRssiDbm = -static_cast<int16_t>(payload[4] == 0 ? payload[0] : payload[1]);
            _stats.linkQuality = payload[2];
        }
    }

    _position = 0;
}
