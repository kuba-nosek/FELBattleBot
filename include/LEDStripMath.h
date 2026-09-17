#pragma once

#include <cmath>
#include <stdint.h>

namespace LEDStripMath {
constexpr float TWO_PI_RADIANS = 6.28318530718f;
constexpr float MICROSECONDS_TO_SECONDS = 0.000001f;

struct IntegrationState {
    float angleRadians;
    uint32_t lastUpdateUs;
};

inline float wrapRadians(float angleRadians) {
    const float wrapped = std::fmod(angleRadians, TWO_PI_RADIANS);
    return wrapped < 0.0f ? wrapped + TWO_PI_RADIANS : wrapped;
}

inline float integrateAngle(float angleRadians, float radiansPerSecond, uint32_t elapsedUs) {
    if(!std::isfinite(radiansPerSecond)) return angleRadians;

    const float elapsedSeconds = static_cast<float>(elapsedUs) * MICROSECONDS_TO_SECONDS;
    return wrapRadians(angleRadians + radiansPerSecond * elapsedSeconds);
}

inline IntegrationState updateIntegration(float angleRadians, uint32_t lastUpdateUs, uint32_t currentUs,
                                          float radiansPerSecond, bool reset) {
    if(reset) return {0.0f, currentUs};

    return {integrateAngle(angleRadians, radiansPerSecond, currentUs - lastUpdateUs), currentUs};
}

inline uint16_t angleToSector(float angleRadians, uint16_t sectorCount) {
    if(sectorCount == 0 || !std::isfinite(angleRadians)) return 0;

    const float wrappedAngle = wrapRadians(angleRadians);
    const float sector = wrappedAngle * static_cast<float>(sectorCount) / TWO_PI_RADIANS;
    const uint16_t sectorIndex = static_cast<uint16_t>(sector);
    return sectorIndex < sectorCount ? sectorIndex : 0;
}

inline bool shouldRefresh(uint32_t currentUs, uint32_t lastRefreshUs, uint32_t intervalUs, bool frameSent) {
    return !frameSent || currentUs - lastRefreshUs >= intervalUs;
}

inline bool shouldTransmit(bool frameSent, bool pixelsChanged) {
    return !frameSent || pixelsChanged;
}
} // namespace LEDStripMath
