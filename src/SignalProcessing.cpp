#include "SignalProcessing.h"
#include "config.h"
#include <stdlib.h>

namespace SignalProcessing {

    int16_t normalizeChannel(uint16_t channelUs) {
        // Zastřešení extrémních hodnot mimo stanovený limit
        if (channelUs < RobotConfig::RC_CHANNEL_MIN) channelUs = RobotConfig::RC_CHANNEL_MIN;
        if (channelUs > RobotConfig::RC_CHANNEL_MAX) channelUs = RobotConfig::RC_CHANNEL_MAX;
        
        // Zjištění odchylky od středu páčky
        int32_t delta = static_cast<int32_t>(channelUs) - RobotConfig::RC_CHANNEL_CENTER;
        
        // Mrtvá zóna (deadband) kolem středu
        if (abs(delta) <= RobotConfig::RC_DEADBAND) {
            return 0;
        }
        
        // Přepočet do výstupní škály s kompenzací mrtvé zóny
        int32_t normalized = (abs(delta) - RobotConfig::RC_DEADBAND) * RobotConfig::RC_OUTPUT_SCALE / 
                             (RobotConfig::RC_CHANNEL_HALF_RANGE - RobotConfig::RC_DEADBAND);
                             
        return static_cast<int16_t>(delta < 0 ? -normalized : normalized);
    }

    DriveModeType decodeMode(uint16_t channelUs) {
        // Dekódování pozice 3polohového přepínače
        if (channelUs <= RobotConfig::RC_MODE_SPIN_THRESHOLD) {
            return DriveModeType::Spin;
        }
        if (channelUs >= RobotConfig::RC_MODE_FORWARD_THRESHOLD) {
            return DriveModeType::Forward;
        }
        return DriveModeType::Idle;
    }

}