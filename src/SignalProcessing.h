#pragma once
#include <stdint.h>
#include "ModeHandler.h"

namespace SignalProcessing {
    int16_t normalizeChannel(uint16_t channelUs);
    
    DriveModeType decodeMode(uint16_t armChannelUs, uint16_t modeChannelUs);
}