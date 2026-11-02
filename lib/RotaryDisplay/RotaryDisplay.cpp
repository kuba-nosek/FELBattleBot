#include "RotaryDisplay.h"
#include "LEDStripMath.h"
#include "config.h"

#include <Arduino.h>

using namespace RobotConfig;

// --- Configuration Guards ---
static_assert(LED_STRIP_LED_COUNT > 1, "The rotary display needs at least two LEDs");
static_assert(LED_STRIP_SECTOR_COUNT > 0, "The rotary display needs at least one sector");
static_assert(LED_STRIP_REFRESH_HZ > 0, "The rotary display refresh rate must be positive");
static_assert(LED_STRIP_REFRESH_HZ <= 1000000, "The rotary display refresh interval must be at least one microsecond");

namespace {
constexpr uint32_t LED_RMT_RESOLUTION_HZ = 20000000;

uint32_t scaleColor(uint32_t color, uint8_t brightness) {
    uint8_t r = ((color >> 16) & 0xFF) * brightness / 255;
    uint8_t g = ((color >> 8) & 0xFF) * brightness / 255;
    uint8_t b = (color & 0xFF) * brightness / 255;
    return (r << 16) | (g << 8) | b;
}
} // namespace

// --- Initialization ---

RotaryDisplay::RotaryDisplay(uint8_t pin)
    : _pin(pin), _mode(RotaryDisplayMode::Static), _animation(LEDStripAnimation::Off),
      _angularVelocityRadPerSec(0.0f), _modeChanged(true), _animationChanged(true),
      _angleRadians(0.0f), _lastIntegrationUs(0), _lastRefreshUs(0), _animationStartMs(0), _frameSent(false),
      _pixels{}, _lastPixels{} {}

RotaryDisplay::~RotaryDisplay() {
    if(_tx) {
        rmt_tx_wait_all_done(_tx, portMAX_DELAY);
        rmt_disable(_tx);
    }
    if(_encoder) rmt_del_encoder(_encoder);
    if(_tx) rmt_del_channel(_tx);
}

void RotaryDisplay::init() {
    if(!USE_LED_STRIP) return;

    rmt_tx_channel_config_t txConfig = {};
    txConfig.gpio_num = static_cast<gpio_num_t>(_pin);
    txConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    txConfig.resolution_hz = LED_RMT_RESOLUTION_HZ;
    txConfig.mem_block_symbols = 48;
    txConfig.trans_queue_depth = 1;
    if(rmt_new_tx_channel(&txConfig, &_tx) != ESP_OK) return;

    rmt_bytes_encoder_config_t encoderConfig = {};
    encoderConfig.bit0 = {8, 1, 17, 0};
    encoderConfig.bit1 = {16, 1, 9, 0};
    encoderConfig.flags.msb_first = 1;
    if(rmt_new_bytes_encoder(&encoderConfig, &_encoder) != ESP_OK) return;
    if(rmt_enable(_tx) != ESP_OK) return;

    // Clear all physical pixels before the first rendered frame.
    rmt_transmit_config_t transmitConfig = {};
    rmt_transmit(_tx, _encoder, _txBytes, sizeof(_txBytes), &transmitConfig);
}

// --- Setters ---

void RotaryDisplay::setMode(RotaryDisplayMode mode) {
    if(_mode != mode) {
        _mode = mode;
        _modeChanged = true;
    }
}

void RotaryDisplay::setAnimation(LEDStripAnimation animation) {
    if(_animation != animation) {
        _animation = animation;
        _animationChanged = true;
    }
}

void RotaryDisplay::setAngularVelocity(float radiansPerSecond) {
    _angularVelocityRadPerSec = radiansPerSecond;
}

// --- Main Update Loop ---

void RotaryDisplay::update() {
    if(!USE_LED_STRIP || !_tx || !_encoder) return;

    const uint32_t currentMs = millis();
    const uint32_t currentUs = micros();

    const bool resetIntegration = _modeChanged || _animationChanged;
    
    if(resetIntegration) {
        _animationStartMs = currentMs;
        _modeChanged = false;
        _animationChanged = false;
    }

    const uint32_t refreshIntervalUs = 1000000 / LED_STRIP_REFRESH_HZ;
    if(!LEDStripMath::shouldRefresh(currentUs, _lastRefreshUs, refreshIntervalUs, _frameSent)) {
        return;
    }
    
    if(_mode == RotaryDisplayMode::Spinning) {
        LEDStripMath::IntegrationState state = LEDStripMath::updateIntegration(
            _angleRadians, _lastIntegrationUs, currentUs, _angularVelocityRadPerSec, resetIntegration);
        
        _angleRadians = state.angleRadians;
        _lastIntegrationUs = state.lastUpdateUs;

        renderSpinningStrip(_animation);
    } else {
        renderStaticStrip(_animation, currentMs - _animationStartMs);
    }

    applyBrightness();

    const uint16_t changedPixelCount =
        LEDStripMath::changedPrefixLength(_pixels, _lastPixels, LED_STRIP_LED_COUNT);
    if(_frameSent && changedPixelCount == 0) return;

    const uint16_t transmittedPixelCount = _frameSent ? changedPixelCount : LED_STRIP_LED_COUNT;
    if(!transmitFrame(transmittedPixelCount)) return;

    rememberPixels(transmittedPixelCount);
    _frameSent = true;
    _lastRefreshUs = currentUs;
}

// --- Rendering Logic ---

void RotaryDisplay::renderStaticStrip(LEDStripAnimation animation, uint32_t elapsedMs) {
    uint32_t color = 0x000000;

    switch(animation) {
        case LEDStripAnimation::Bootup:
            if((elapsedMs % 250) < 125) color = 0x0000FF; 
            break;
        case LEDStripAnimation::Idle:
            if((elapsedMs % 2000) < 100 || ((elapsedMs % 2000) > 200 && (elapsedMs % 2000) < 300)) {
                color = 0x00FF00;
            }
            break;
        case LEDStripAnimation::Forward:
            color = 0x00FF00;
            break;
        case LEDStripAnimation::Failsafe:
            if((elapsedMs % 500) < 250) color = 0xFF0000;
            break;
        case LEDStripAnimation::HardwareError:
            if((elapsedMs % 60) < 30) color = 0xFF0000;
            break;
        case LEDStripAnimation::LowBattery:
            if((elapsedMs % 1000) < 100) color = 0xFF9900;
            break;
        default:
            break;
    }

    for(uint16_t i = 0; i < LED_STRIP_LED_COUNT; ++i) {
        _pixels[i] = color;
    }
}

void RotaryDisplay::renderSpinningStrip(LEDStripAnimation animation) {
    for(uint16_t i = 0; i < LED_STRIP_LED_COUNT; ++i) {
        _pixels[i] = 0x000000;
    }

    if(LEDStripImages::hasImage(animation)) {
        const uint16_t sector = LEDStripMath::angleToSector(_angleRadians, LED_STRIP_SECTOR_COUNT);
        const uint32_t* sectorData = LEDStripImages::getSectorPixels(animation, sector);
        
        if(sectorData != nullptr) {
            for(uint16_t i = 0; i < LED_STRIP_LED_COUNT; ++i) {
                _pixels[i] = sectorData[i];
            }
        }
    }
}

// --- Hardware & Utility ---

void RotaryDisplay::applyBrightness() {
    for(uint16_t i = 0; i < LED_STRIP_LED_COUNT; ++i) {
        _pixels[i] = scaleColor(_pixels[i], LED_STRIP_BRIGHTNESS);
    }
}

void RotaryDisplay::rememberPixels(uint16_t pixelCount) {
    for(uint16_t i = 0; i < pixelCount; ++i) {
        _lastPixels[i] = _pixels[i];
    }
}

bool RotaryDisplay::transmitFrame(uint16_t pixelCount) {
    if(rmt_tx_wait_all_done(_tx, pdMS_TO_TICKS(5)) != ESP_OK) return false;

    for(uint16_t i = 0; i < pixelCount; ++i) {
        const uint32_t color = _pixels[i];
        _txBytes[3 * i] = static_cast<uint8_t>(color >> 8);  // Green
        _txBytes[3 * i + 1] = static_cast<uint8_t>(color >> 16); // Red
        _txBytes[3 * i + 2] = static_cast<uint8_t>(color); // Blue
    }

    rmt_transmit_config_t transmitConfig = {};
    return rmt_transmit(_tx, _encoder, _txBytes, pixelCount * 3, &transmitConfig) == ESP_OK;
}
