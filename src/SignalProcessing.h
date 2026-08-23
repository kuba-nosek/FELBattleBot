#pragma once
#include <stdint.h>
#include "ModeHandler.h"

namespace SignalProcessing {
    // Převod surových mikrosekund (988-2012) na plyn/zatáčení (-1000 až 1000)
    int16_t normalizeChannel(uint16_t channelUs);
    
    // Dekódování polohy třípolohového přepínače na konkrétní režim
    DriveModeType decodeMode(uint16_t channelUs);
}