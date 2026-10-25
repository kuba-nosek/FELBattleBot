#pragma once

#include "IReceiver.h"
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <stddef.h>
#include <stdint.h>

struct ReceiverStats {
    uint8_t linkQuality = 0;
    int16_t activeRssiDbm = 0;
};

class CRSFReceiver : public IReceiver {
  public:
    // Konstruktor nyní přijímá referenci na sériovou linku
    CRSFReceiver(HardwareSerial& serial, uint8_t rxPin, uint8_t txPin);

    void connect() override;
    void update() override;
    bool isConnected() const override;
    void onDisconnect(DisconnectCallback callback) override;
    ReceiverChannels getChannelsSnapshot() const override;
    
    ReceiverStats getStatistics() const;

  private:
    HardwareSerial& _serial;
    uint8_t _rxPin;
    uint8_t _txPin;

    uint32_t _lastValidFrameMs;
    ReceiverStats _stats;

    bool _connected;
    DisconnectCallback _disconnectCallback;

    uint8_t _frame[64]{};
    size_t _position;
    size_t _expectedSize;
    uint16_t _channels[ReceiverChannels::COUNT];
    mutable portMUX_TYPE _channelsMux = portMUX_INITIALIZER_UNLOCKED;

    void processByte(uint8_t byte);
};
