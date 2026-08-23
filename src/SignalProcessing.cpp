#include "SignalProcessing.h"
#include <stdlib.h>

namespace SignalProcessing {

    int16_t normalizeChannel(uint16_t channelUs) {
        if (channelUs < 988) channelUs = 988;
        if (channelUs > 2012) channelUs = 2012;
        
        int32_t delta = static_cast<int32_t>(channelUs) - 1500;
        
        // Mrtvá zóna (deadband)[cite: 6]
        if (abs(delta) <= 20) {
            return 0;
        }
        
        int32_t normalized = (abs(delta) - 20) * 1000 / (500 - 20);
        return static_cast<int16_t>(delta < 0 ? -normalized : normalized);
    }

    DriveModeType decodeMode(uint16_t channelUs) {
        if (channelUs <= 1300) {
            return DriveModeType::Spin;
        }
        if (channelUs >= 1700) {
            return DriveModeType::Forward;
        }
        return DriveModeType::Idle;
    }

}