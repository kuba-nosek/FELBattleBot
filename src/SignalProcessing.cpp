#include "SignalProcessing.h"
#include "config.h"

namespace
{
    uint16_t clampChannel(uint16_t channelUs)
    {
        if (channelUs < RobotConfig::RC_CHANNEL_MIN)
        {
            return RobotConfig::RC_CHANNEL_MIN;
        }

        if (channelUs > RobotConfig::RC_CHANNEL_MAX)
        {
            return RobotConfig::RC_CHANNEL_MAX;
        }

        return channelUs;
    }

    int16_t mapRange(
        uint16_t value,
        uint16_t inputMin,
        uint16_t inputMax,
        int16_t outputMin,
        int16_t outputMax)
    {
        const int32_t inputPosition =
            static_cast<int32_t>(value) - inputMin;
        const int32_t inputRange =
            static_cast<int32_t>(inputMax) - inputMin;
        const int32_t outputRange =
            static_cast<int32_t>(outputMax) - outputMin;

        return static_cast<int16_t>(
            outputMin + (inputPosition * outputRange) / inputRange);
    }

    int16_t processStick(uint16_t channelUs)
    {
        const uint16_t value = clampChannel(channelUs);
        const uint16_t deadbandLow =
            RobotConfig::RC_CHANNEL_CENTER - RobotConfig::RC_DEADBAND;
        const uint16_t deadbandHigh =
            RobotConfig::RC_CHANNEL_CENTER + RobotConfig::RC_DEADBAND;

        if (value < deadbandLow)
        {
            return mapRange(
                value,
                RobotConfig::RC_CHANNEL_MIN,
                deadbandLow,
                -RobotConfig::RC_OUTPUT_SCALE,
                0);
        }

        if (value > deadbandHigh)
        {
            return mapRange(
                value,
                deadbandHigh,
                RobotConfig::RC_CHANNEL_MAX,
                0,
                RobotConfig::RC_OUTPUT_SCALE);
        }

        return 0;
    }

    int16_t processPotentiometer(uint16_t channelUs)
    {
        const uint16_t value = clampChannel(channelUs);

        if (value <= RobotConfig::RC_CHANNEL_CENTER)
        {
            return mapRange(
                value,
                RobotConfig::RC_CHANNEL_MIN,
                RobotConfig::RC_CHANNEL_CENTER,
                -RobotConfig::RC_OUTPUT_SCALE,
                0);
        }

        return mapRange(
            value,
            RobotConfig::RC_CHANNEL_CENTER,
            RobotConfig::RC_CHANNEL_MAX,
            0,
            RobotConfig::RC_OUTPUT_SCALE);
    }

    bool processTwoStateSwitch(uint16_t channelUs)
    {
        return clampChannel(channelUs) >= RobotConfig::RC_CHANNEL_CENTER;
    }

    int8_t processThreeStateSwitch(uint16_t channelUs)
    {
        const uint16_t value = clampChannel(channelUs);

        if (value <= RobotConfig::RC_SWITCH_LOW_THRESHOLD)
        {
            return -1;
        }

        if (value >= RobotConfig::RC_SWITCH_HIGH_THRESHOLD)
        {
            return 1;
        }

        return 0;
    }

    uint8_t processSixStateSwitch(uint16_t channelUs)
    {
        const uint32_t value = clampChannel(channelUs);
        const uint32_t inputRange =
            RobotConfig::RC_CHANNEL_MAX - RobotConfig::RC_CHANNEL_MIN;
        const uint32_t position =
            (value - RobotConfig::RC_CHANNEL_MIN) * 5U;

        // Round to the nearest of the six equally spaced switch detents.
        return static_cast<uint8_t>(
            (position + inputRange / 2U) / inputRange);
    }
}

namespace SignalProcessing
{
    ReceiverInput processReceiverInput(const ReceiverInputRaw& rawInput)
    {
        ReceiverInput input{};

#define RECEIVER_PROCESS_FIELD(name, type, channel) \
        input.name = process##type(rawInput.channelsUs[(channel) - 1]);

        RECEIVER_INPUT_MAP(RECEIVER_PROCESS_FIELD)

#undef RECEIVER_PROCESS_FIELD

        return input;
    }
}
