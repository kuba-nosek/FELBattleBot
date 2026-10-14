#include "POVDisplay.h"
#include "LEDStripMath.h"
#include "config.h"

#include <Arduino.h>

using namespace RobotConfig;

// --- Configuration Guards ---
static_assert(LED_STRIP_LED_COUNT > 1, "The POV strip needs at least two LEDs");
static_assert(LED_STRIP_LED_COUNT % 2 == 0, "The half-strip POV layout requires an even LED count");
static_assert(LED_STRIP_SECTORS_PER_LED > 0, "The POV strip needs at least one sector per LED");
static_assert(LED_STRIP_REFRESH_HZ > 0, "The POV strip refresh rate must be positive");
static_assert(LED_STRIP_REFRESH_HZ <= 1000000, "The POV strip refresh interval must be at least one microsecond");

namespace {
uint32_t scaleColor(uint32_t color, uint8_t brightness) {
    uint8_t r = ((color >> 16) & 0xFF) * brightness / 255;
    uint8_t g = ((color >> 8) & 0xFF) * brightness / 255;
    uint8_t b = (color & 0xFF) * brightness / 255;
    return (r << 16) | (g << 8) | b;
}
} // namespace

// --- Initialization ---

POVDisplay::POVDisplay(uint8_t pin)
    : _pin(pin), _mode(POVDisplayMode::Static), _animation(LEDStripAnimation::Off),
      _angularVelocityRadPerSec(0.0f), _modeChanged(true), _animationChanged(true),
      _angleRadians(0.0f), _lastIntegrationUs(0), _lastRefreshUs(0), _animationStartMs(0), _frameSent(false),
      _lastTransmittedMode(POVDisplayMode::Static), _pixels{}, _lastPixels{},
      _strip(LED_STRIP_LED_COUNT, pin, NEO_GRB + NEO_KHZ800) {}

void POVDisplay::init() {
    _strip.begin();
    _strip.show();
}

// --- Setters ---

void POVDisplay::setMode(POVDisplayMode mode) {
    if(_mode != mode) {
        _mode = mode;
        _modeChanged = true;
    }
}

void POVDisplay::setAnimation(LEDStripAnimation animation) {
    if(_animation != animation) {
        _animation = animation;
        _animationChanged = true;
    }
}

void POVDisplay::setAngularVelocity(float radiansPerSecond) {
    _angularVelocityRadPerSec = radiansPerSecond;
}

// --- Main Update Loop ---

void POVDisplay::update() {
    if(!USE_LED_STRIP) return;

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
    
    if(_mode == POVDisplayMode::Spinning) {
        LEDStripMath::IntegrationState state = LEDStripMath::updateIntegration(
            _angleRadians, _lastIntegrationUs, currentUs, _angularVelocityRadPerSec, resetIntegration);
        
        _angleRadians = state.angleRadians;
        _lastIntegrationUs = state.lastUpdateUs;

        renderSpinningStrip(_animation);
    } else {
        renderStaticStrip(_animation, currentMs - _animationStartMs);
    }

    applyBrightness();

    if(!LEDStripMath::shouldTransmit(_frameSent, pixelsChanged()) && _lastTransmittedMode == _mode) {
        return;
    }

    transmitFrame();
    
    rememberPixels();
    _frameSent = true;
    _lastTransmittedMode = _mode;
    _lastRefreshUs = currentUs;
}

// --- Rendering Logic ---

void POVDisplay::renderStaticStrip(LEDStripAnimation animation, uint32_t elapsedMs) {
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

void POVDisplay::renderSpinningStrip(LEDStripAnimation animation) {
    for(uint16_t i = 0; i < LED_STRIP_LED_COUNT; ++i) {
        _pixels[i] = 0x000000;
    }

    if(LEDStripImages::hasImage(animation)) {
        const uint16_t sector = LEDStripMath::angleToSector(_angleRadians, LED_STRIP_SECTOR_COUNT);
        const uint32_t* sectorData = LEDStripImages::getSectorPixels(animation, sector);
        
        if(sectorData != nullptr) {
            for(uint16_t i = 0; i < LED_STRIP_SPIN_LED_COUNT; ++i) {
                _pixels[i] = sectorData[i];
            }
        }
    }
}

// --- Hardware & Utility ---

void POVDisplay::applyBrightness() {
    for(uint16_t i = 0; i < LED_STRIP_LED_COUNT; ++i) {
        _pixels[i] = scaleColor(_pixels[i], LED_STRIP_BRIGHTNESS);
    }
}

bool POVDisplay::pixelsChanged() const {
    for(uint16_t i = 0; i < LED_STRIP_LED_COUNT; ++i) {
        if(_pixels[i] != _lastPixels[i]) {
            return true;
        }
    }
    return false;
}

void POVDisplay::rememberPixels() {
    for(uint16_t i = 0; i < LED_STRIP_LED_COUNT; ++i) {
        _lastPixels[i] = _pixels[i];
    }
}

void POVDisplay::transmitFrame() {
    for(uint16_t pixelIndex = 0; pixelIndex < LED_STRIP_LED_COUNT; ++pixelIndex) {
        _strip.setPixelColor(pixelIndex, _pixels[pixelIndex]);
    }
    
    _strip.show(); 
}
