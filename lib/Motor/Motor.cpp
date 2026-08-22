#include "Motor.h"
#include <Arduino.h>

namespace {
    // Časování RMT pro DShot300 na ESP32 (80MHz APB)[cite: 3]
    constexpr uint8_t RMT_CLOCK_DIVIDER = 2;
    constexpr uint16_t BIT_TOTAL_TICKS = 133;
    constexpr uint16_t ZERO_HIGH_TICKS = 50;
    constexpr uint16_t ONE_HIGH_TICKS = 100;

    // Hodnoty protokolu DShot[cite: 10]
    constexpr uint16_t DSHOT_MOTOR_STOP = 0;
    constexpr uint16_t DSHOT_3D_MODE_ON = 10;
    constexpr uint16_t DSHOT_REVERSE_MIN = 48;
    constexpr uint16_t DSHOT_FORWARD_MIN = 1048;
    constexpr int16_t SPEED_SCALE = 1000;

    // Bezpečnostní prodleva pro Direction Change Guard
    constexpr uint32_t DIRECTION_CHANGE_STOP_MS = 150; 
}

Motor::Motor(uint8_t gpioPin, rmt_channel_t rmtChannel, bool reversed)
    : _gpioPin(gpioPin), _rmtChannel(rmtChannel), _reversed(reversed),
      _initialized(false), _lastDirection(0), _atZero(true), _zeroSinceMs(0) {
}

bool Motor::init() {
    pinMode(_gpioPin, OUTPUT);
    digitalWrite(_gpioPin, LOW);

    rmt_config_t config{};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = _rmtChannel;
    config.gpio_num = static_cast<gpio_num_t>(_gpioPin);
    config.clk_div = RMT_CLOCK_DIVIDER;
    config.mem_block_num = 1;
    config.flags = 0;
    config.tx_config.carrier_freq_hz = 38000;
    config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.carrier_duty_percent = 33;
    config.tx_config.carrier_en = false;
    config.tx_config.loop_en = false;
    config.tx_config.idle_output_en = true;

    if (rmt_config(&config) != ESP_OK) return false;
    if (rmt_driver_install(_rmtChannel, 0, 0) != ESP_OK) return false;

    _initialized = true;
    return writeDshotPacket(DSHOT_MOTOR_STOP, false);
}

bool Motor::arm() {
    if (!_initialized) return false;
    
    // Udržení nuly pro detekci ESC
    for(int i = 0; i < 500; i++) {
        writeDshotPacket(DSHOT_MOTOR_STOP, false);
        delay(1);
    }

    // Odeslání příkazu pro 3D režim (musí mít nastavený requestTelemetry bit)
    for (uint8_t repeat = 0; repeat < 10; ++repeat) {
        writeDshotPacket(DSHOT_3D_MODE_ON, true);
        delay(1);
    }

    for(int i = 0; i < 50; i++) {
        writeDshotPacket(DSHOT_MOTOR_STOP, false);
        delay(1);
    }
    return true;
}

void Motor::setReversed(bool reversed) {
    _reversed = reversed;
}

void Motor::resetDirectionGuard() {
    _lastDirection = 0;
    _atZero = true;
    _zeroSinceMs = 0;
}

void Motor::stop() {
    setSpeed(0, millis(), false);
}

bool Motor::setSpeed(int16_t speed, uint32_t currentMs, bool requestTelemetry) {
    if (!_initialized) return false;

    // 1. Aplikace polarity
    int16_t targetSpeed = _reversed ? -speed : speed;

    // 2. Direction Change Guard (ochrana převodovky)
    if (targetSpeed == 0) {
        if (!_atZero) {
            _atZero = true;
            _zeroSinceMs = currentMs;
        }
    } else {
        int8_t requestedDirection = targetSpeed > 0 ? 1 : -1;

        if (_lastDirection == 0) {
            _lastDirection = requestedDirection;
            _atZero = false;
        } else if (requestedDirection == _lastDirection) {
            _atZero = false;
        } else {
            if (!_atZero) {
                _atZero = true;
                _zeroSinceMs = currentMs;
                targetSpeed = 0;
            } else if (currentMs - _zeroSinceMs < DIRECTION_CHANGE_STOP_MS) {
                targetSpeed = 0;
            } else {
                _lastDirection = requestedDirection;
                _atZero = false;
            }
        }
    }

    // 3. Omezení na maximální limity
    if (targetSpeed > SPEED_SCALE) targetSpeed = SPEED_SCALE;
    if (targetSpeed < -SPEED_SCALE) targetSpeed = -SPEED_SCALE;

    // 4. Převod na DShot hodnotu[cite: 4]
    uint16_t dshotValue = DSHOT_MOTOR_STOP;
    if (targetSpeed != 0) {
        uint16_t magnitude = targetSpeed < 0 ? -targetSpeed : targetSpeed;
        dshotValue = targetSpeed > 0 ? (DSHOT_FORWARD_MIN + magnitude - 1) 
                                     : (DSHOT_REVERSE_MIN + magnitude - 1);
    }

    return writeDshotPacket(dshotValue, requestTelemetry);
}

bool Motor::writeDshotPacket(uint16_t value, bool requestTelemetry) {
    // Vytvoření DShot paketu včetně CRC[cite: 4]
    uint16_t payload = (value << 1) | (requestTelemetry ? 1U : 0U);
    uint16_t checksumData = payload;
    uint16_t checksum = 0;

    for (uint8_t nibble = 0; nibble < 3; ++nibble) {
        checksum ^= checksumData;
        checksumData >>= 4;
    }
    
    uint16_t packet = (payload << 4) | (checksum & 0x0F);

    rmt_item32_t items[16];
    prepareItems(packet, items);
    
    return rmt_write_items(_rmtChannel, items, 16, true) == ESP_OK;
}

void Motor::prepareItems(uint16_t packet, rmt_item32_t* items) {
    for (size_t index = 0; index < 16; ++index) {
        bool one = (packet & (0x8000U >> index)) != 0;
        uint16_t highTicks = one ? ONE_HIGH_TICKS : ZERO_HIGH_TICKS;

        items[index].level0 = 1;
        items[index].duration0 = highTicks;
        items[index].level1 = 0;
        items[index].duration1 = BIT_TOTAL_TICKS - highTicks;
    }
}