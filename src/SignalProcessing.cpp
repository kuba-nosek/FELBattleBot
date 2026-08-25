#include "SignalProcessing.h"
#include "config.h"
#include <stdlib.h>

namespace SignalProcessing {

    int16_t normalizeChannel(uint16_t channelUs) {
        if (channelUs < RobotConfig::RC_CHANNEL_MIN) channelUs = RobotConfig::RC_CHANNEL_MIN;
        if (channelUs > RobotConfig::RC_CHANNEL_MAX) channelUs = RobotConfig::RC_CHANNEL_MAX;
        
        int32_t delta = static_cast<int32_t>(channelUs) - RobotConfig::RC_CHANNEL_CENTER;
        
        if (abs(delta) <= RobotConfig::RC_DEADBAND) {
            return 0;
        }
        
        int32_t normalized = (abs(delta) - RobotConfig::RC_DEADBAND) * RobotConfig::RC_OUTPUT_SCALE / 
                             (RobotConfig::RC_CHANNEL_HALF_RANGE - RobotConfig::RC_DEADBAND);
                             
        return static_cast<int16_t>(delta < 0 ? -normalized : normalized);
    }

    DriveModeType decodeMode(uint16_t armChannelUs, uint16_t modeChannelUs) {
        // 1. BEZPEČNOSTNÍ ZÁMEK (CH5)
        // Pokud je přepínač v horní polovině (např. 1000 us), robot musí být bezpečně v Idle.
        if (armChannelUs < RobotConfig::RC_CHANNEL_CENTER) {
            return DriveModeType::Idle;
        }

        // 2. VÝBĚR MÓDU (CH8) - Vyhodnotí se, pouze pokud je Arm přepínač dole (nad 1500 us)
        if (modeChannelUs <= RobotConfig::RC_MODE_SPIN_THRESHOLD) {
            return DriveModeType::Spin;
        }
        if (modeChannelUs >= RobotConfig::RC_MODE_FORWARD_THRESHOLD) {
            return DriveModeType::Forward;
        }
        
        // Výchozí pojistka: pokud je 3polohový přepínač ve středu, přejdi raději do Idle
        return DriveModeType::Idle; 
    }
}