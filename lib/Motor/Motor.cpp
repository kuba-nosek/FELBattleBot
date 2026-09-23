#include "Motor.h"
#include <driver/gpio.h>
#include <esp_timer.h>

namespace {
    constexpr uint32_t RMT_RES_HZ = 80000000;
    constexpr size_t RMT_MEM = 48;
    constexpr uint32_t BITRATE_DSHOT600 = 600000;
    constexpr int64_t ARM_HOLD_US = 1200000;
    constexpr uint32_t DIRECTION_CHANGE_STOP_MS = 150;

    const uint8_t kGcrToNibble[32] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
        0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F, 
        0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07, 
        0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF, 
    };
}

Motor::Motor(uint8_t gpioPin, bool reversed, bool bidirectional)
    : _gpioPin(gpioPin), _reversed(reversed), _bidirectional(bidirectional), _initialized(false),
      _lastDirection(0), _atZero(true), _zeroSinceMs(0), _currentSpeed(0) {
}

Motor::~Motor() {
    if (_rx) { rmt_disable(_rx); rmt_del_channel(_rx); }
    if (_tx) { rmt_disable(_tx); rmt_del_channel(_tx); }
    if (_enc) { rmt_del_encoder(_enc); }
}

bool Motor::init() {
    gpio_config_t pull_cfg = {};
    pull_cfg.pin_bit_mask = 1ULL << _gpioPin;
    pull_cfg.mode = GPIO_MODE_OUTPUT_OD;
    pull_cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&pull_cfg);
    gpio_set_level((gpio_num_t)_gpioPin, 1);
    delay(2500);

    _tbit = RMT_RES_HZ / BITRATE_DSHOT600;
    _t0h = (uint16_t)((uint32_t)_tbit * 3 / 8);
    _t1h = (uint16_t)((uint32_t)_tbit * 3 / 4);
    _telemQ8 = (uint32_t)(((uint64_t)RMT_RES_HZ * 4 * 256) / (5ull * BITRATE_DSHOT600));
    _gapMin = (uint16_t)((uint32_t)_tbit * 5 / 2);

    if (_bidirectional) {
        rmt_rx_channel_config_t rxc = {};
        rxc.gpio_num = (gpio_num_t)_gpioPin;
        rxc.clk_src = RMT_CLK_SRC_DEFAULT;
        rxc.resolution_hz = RMT_RES_HZ;
        rxc.mem_block_symbols = RMT_MEM;
        if (rmt_new_rx_channel(&rxc, &_rx) != ESP_OK) return false;

        rmt_rx_event_callbacks_t cbs = {};
        cbs.on_recv_done = onRxDone;
        if (rmt_rx_register_event_callbacks(_rx, &cbs, this) != ESP_OK) return false;
    }

    rmt_tx_channel_config_t txc = {};
    txc.gpio_num = (gpio_num_t)_gpioPin;
    txc.clk_src = RMT_CLK_SRC_DEFAULT;
    txc.resolution_hz = RMT_RES_HZ;
    txc.mem_block_symbols = RMT_MEM;
    txc.trans_queue_depth = 2;
    txc.flags.io_loop_back = _bidirectional;  
    txc.flags.io_od_mode = _bidirectional; 
    if (rmt_new_tx_channel(&txc, &_tx) != ESP_OK) return false;

    rmt_copy_encoder_config_t enc = {};
    if (rmt_new_copy_encoder(&enc, &_enc) != ESP_OK) return false;

    if (_bidirectional) {
        gpio_pullup_en((gpio_num_t)_gpioPin);
        if (rmt_enable(_rx) != ESP_OK) return false;
    }

    if (rmt_enable(_tx) != ESP_OK) return false;

    _beginUs = esp_timer_get_time();
    _escArmed = false;
    _edtEnabled = false;
    _initialized = true;

    sendRaw(DSHOT_CMD_MOTOR_STOP);
    return true;
}

void Motor::setReversed(bool reversed) { _reversed = reversed; }

void Motor::resetDirectionGuard() {
    _lastDirection = 0;
    _atZero = true;
    _zeroSinceMs = 0;
}

void Motor::stop() {
    setSpeed(0, millis(), false);
}

bool Motor::setSpeed(int16_t speed, uint32_t currentMs, bool requestTelemetry) {
    if(!_initialized) return false;

    int16_t targetSpeed = _reversed ? -speed : speed;

    if(targetSpeed == 0) {
        if(!_atZero) {
            _atZero = true;
            _zeroSinceMs = currentMs;
        }
    } else {
        int8_t requestedDirection = targetSpeed > 0 ? 1 : -1;

        if(_lastDirection == 0) {
            _lastDirection = requestedDirection;
            _atZero = false;
        } else if(requestedDirection == _lastDirection) {
            _atZero = false;
        } else {
            if(!_atZero) {
                _atZero = true;
                _zeroSinceMs = currentMs;
                targetSpeed = 0;
            } else if(currentMs - _zeroSinceMs < DIRECTION_CHANGE_STOP_MS) {
                targetSpeed = 0;
            } else {
                _lastDirection = requestedDirection;
                _atZero = false;
            }
        }
    }

    if(targetSpeed > SPEED_SCALE) targetSpeed = SPEED_SCALE;
    if(targetSpeed < -SPEED_SCALE) targetSpeed = -SPEED_SCALE;

    uint16_t dshotValue = DSHOT_CMD_MOTOR_STOP;
    if(targetSpeed != 0) {
        uint16_t magnitude = targetSpeed < 0 ? -targetSpeed : targetSpeed;
        dshotValue = targetSpeed > 0 ? (DSHOT_FORWARD_MIN + magnitude - 1) : (DSHOT_REVERSE_MIN + magnitude - 1);
    }

    bool hasTelemetry = sendRaw(dshotValue);
    if(hasTelemetry || dshotValue == DSHOT_CMD_MOTOR_STOP) {
        _currentSpeed = _reversed ? -targetSpeed : targetSpeed;
    }

    return hasTelemetry;
}

int16_t Motor::getSpeed() const { return _currentSpeed; }

void Motor::sendCommand(uint16_t cmd, uint8_t repeat) {
    _cmd = cmd;
    _cmdRepeat = repeat ? repeat : 1;
}

bool Motor::onRxDone(rmt_channel_handle_t, const rmt_rx_done_event_data_t *ev, void *ctx) {
    Motor *self = (Motor *)ctx;
    self->_rxCount = ev->num_symbols;
    self->_rxDone = true;
    return false;
}

void Motor::armRx() {
    rmt_receive_config_t cfg = {};
    cfg.signal_range_min_ns = 100;
    cfg.signal_range_max_ns = _rxIdleNs;
    _rxDone = false;
    _rxArmed = (rmt_receive(_rx, _rxBuf, sizeof(_rxBuf), &cfg) == ESP_OK);
}

void Motor::buildFrame(uint16_t value) {
    uint16_t packet = (value & 0x07FF) << 1;
    
    // Příkazy (1-47) vyžadují zapnutý telemetrický bit, jinak je ESC ignoruje
    if (value > 0 && value < 48) {
        packet |= 1;
    }

    uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    if (_bidirectional) {
        crc = (~crc) & 0x0F; // Pro obousměrný provoz invertujeme CRC jako výzvu
    }
    _frame = (packet << 4) | crc;

    // OPRAVA POLARITY: AM32 VŽDY očekává invertovaný DShot (idles HIGH).
    // Push-Pull výstup pro pravý motor to natvrdo vyžene nahoru i bez rezistoru.
    for (int i = 0; i < 16; ++i) {
        uint16_t hi = (_frame & (0x8000 >> i)) ? _t1h : _t0h;
        _txSym[i].level0 = 0;
        _txSym[i].duration0 = hi;
        _txSym[i].level1 = 1;
        _txSym[i].duration1 = _tbit - hi;
    }
}

bool Motor::sendRaw(uint16_t value) {
    bool fresh = false;

    if (_bidirectional && _rxArmed) {
        const uint32_t budget = ((uint32_t)_tbit * 16 + ((_telemQ8 >> 8) * 21)) / (RMT_RES_HZ / 1000000) + 40 + _rxIdleNs / 1000 + 30;
        const int64_t deadline = esp_timer_get_time() + budget;
        while (!_rxDone && esp_timer_get_time() < deadline) {}

        if (_rxDone) {
            _rxDone = false;
            _rxArmed = false;
            _status = decode(_rxCount);
        } else {
            rmt_disable(_rx);
            rmt_enable(_rx);
            _rxArmed = false;
            _echoPulses = 0;
            _status = DSHOT_RX_NO_REPLY;
        }
        fresh = (_status == DSHOT_RX_OK);
    }

    // Automatický asynchronní arming - krmí správně dlouhé nuly bez blokování
    if (!_escArmed) {
        if (esp_timer_get_time() - _beginUs < ARM_HOLD_US) {
            value = DSHOT_CMD_MOTOR_STOP; 
        } else {
            _escArmed = true; 
            sendCommand(DSHOT_CMD_3D_MODE_ON, 10);
            value = _cmd;
            _cmdRepeat--;
        }
    } else if (_cmdRepeat > 0) {
        value = _cmd;
        _cmdRepeat--;
    } else if (_bidirectional && !_edtEnabled) {
        _edtEnabled = true;
        sendCommand(DSHOT_CMD_EDT_ENABLE, 10);
        value = _cmd;
        _cmdRepeat--;
    }

    if (_bidirectional) armRx();
    buildFrame(value);

    rmt_transmit_config_t txc = {};
    // OPRAVA POLARITY: EOT (End of Transmission) musí VŽDY skončit v HIGH
    txc.flags.eot_level = 1; 
    rmt_transmit(_tx, _enc, _txSym, sizeof(_txSym), &txc);

    return fresh;
}

DShotRxStatus Motor::decode(size_t nsym) {
    const size_t np = nsym * 2;
    if (np == 0) return DSHOT_RX_NO_REPLY;

    #define PULSE_LVL(i) ((i) & 1 ? _rxBuf[(i) >> 1].level1 : _rxBuf[(i) >> 1].level0)
    #define PULSE_DUR(i) ((i) & 1 ? _rxBuf[(i) >> 1].duration1 : _rxBuf[(i) >> 1].duration0)

    size_t p = 0;
    bool found = false;
    for (; p < np; ++p) {
        uint16_t d = PULSE_DUR(p);
        if (d == 0) break;
        if (PULSE_LVL(p) && d >= _gapMin) {
            found = true;
            ++p;
            break;
        }
    }
    
    _echoPulses = found ? (uint16_t)(p - 1) : (uint16_t)p;
    if (!found || p >= np) return DSHOT_RX_NO_REPLY;

    uint32_t v = 0;
    int bits = 0;
    for (; p < np && bits < 21; ++p) {
        uint16_t d = PULSE_DUR(p);
        if (d == 0) break;
        if (bits == 0 && PULSE_LVL(p)) return DSHOT_RX_FRAMING;

        uint32_t n = ((uint32_t)d * 256u + (_telemQ8 >> 1)) / _telemQ8;
        if (n == 0) n = 1;
        if (bits + (int)n > 21) n = 21 - bits;

        v = (v << n) | (PULSE_LVL(p) ? 0u : ((1u << n) - 1u));
        bits += n;
    }
    if (bits == 0) return DSHOT_RX_NO_REPLY;
    if (bits < 21) v <<= (21 - bits);

    #undef PULSE_LVL
    #undef PULSE_DUR

    const uint32_t gcr = (v ^ (v >> 1)) & 0xFFFFFu;
    const uint8_t n3 = kGcrToNibble[(gcr >> 15) & 0x1F];
    const uint8_t n2 = kGcrToNibble[(gcr >> 10) & 0x1F];
    const uint8_t n1 = kGcrToNibble[(gcr >> 5) & 0x1F];
    const uint8_t n0 = kGcrToNibble[gcr & 0x1F];
    if ((n3 | n2 | n1 | n0) & 0xF0) return DSHOT_RX_BAD_GCR;

    const uint16_t d16 = (n3 << 12) | (n2 << 8) | (n1 << 4) | n0;
    uint16_t csum = d16 ^ (d16 >> 8);
    csum ^= csum >> 4;
    if ((csum & 0x0F) != 0x0F) return DSHOT_RX_BAD_CRC;

    apply(d16 >> 4);
    return DSHOT_RX_OK;
}

void Motor::apply(uint16_t data12) {
    const uint8_t type = data12 >> 8;
    if (type != 0 && (type & 1) == 0) {
        const uint8_t val = data12 & 0xFF;
        _edtSeen = true;
        switch (type) {
            case 0x02: _edtTemp = val; break;
            case 0x04: _edtVolts = val * 0.25f; break;
            case 0x06: _edtAmps = val * 0.5f; break;
            default: break;
        }
        return;
    }

    if (data12 == 0x0FFF) {
        _periodUs = 0;
        _erpm = 0;
    } else {
        _periodUs = (uint32_t)(data12 & 0x1FF) << (data12 >> 9);
        _erpm = _periodUs ? (60000000u / _periodUs) : 0;
    }
    _lastOkUs = esp_timer_get_time();
}