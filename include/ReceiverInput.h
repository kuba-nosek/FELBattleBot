#pragma once

#include "Receiver.h"
#include "config.h"

#include <stdint.h>

#define RECEIVER_FIELD_TYPE_Stick int16_t
#define RECEIVER_FIELD_TYPE_Potentiometer int16_t
#define RECEIVER_FIELD_TYPE_TwoStateSwitch bool
#define RECEIVER_FIELD_TYPE_ThreeStateSwitch int8_t
#define RECEIVER_FIELD_TYPE_SixStateSwitch uint8_t
#define RECEIVER_DECLARE_FIELD(name, type, channel) RECEIVER_FIELD_TYPE_##type name{};

struct ReceiverInput {
    RECEIVER_INPUT_MAP(RECEIVER_DECLARE_FIELD)
};

#undef RECEIVER_DECLARE_FIELD
#undef RECEIVER_FIELD_TYPE_SixStateSwitch
#undef RECEIVER_FIELD_TYPE_ThreeStateSwitch
#undef RECEIVER_FIELD_TYPE_TwoStateSwitch
#undef RECEIVER_FIELD_TYPE_Potentiometer
#undef RECEIVER_FIELD_TYPE_Stick

#define RECEIVER_VALIDATE_CHANNEL(name, type, channel)                                                                 \
    static_assert((channel) >= 1 && (channel) <= ReceiverChannels::COUNT,                                              \
                  "Receiver channel for " #name " must be in the range 1..16");

RECEIVER_INPUT_MAP(RECEIVER_VALIDATE_CHANNEL)

#undef RECEIVER_VALIDATE_CHANNEL
