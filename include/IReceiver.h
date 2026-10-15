#pragma once
#include <stddef.h>
#include <stdint.h>

typedef void (*DisconnectCallback)();

struct ReceiverChannels {
    static constexpr size_t COUNT = 16;
    uint16_t channelsUs[COUNT]{};
};

class IReceiver {
  public:
    virtual ~IReceiver() = default;

    virtual void connect() = 0;
    virtual void update() = 0;

    virtual bool isConnected() const = 0;
    virtual void onDisconnect(DisconnectCallback callback) = 0;

    virtual ReceiverChannels getChannelsSnapshot() const = 0;
};
