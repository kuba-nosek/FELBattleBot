#pragma once

#include <Arduino.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>

// --- DShot Protocol Constants ---
constexpr uint16_t DSHOT_CMD_MOTOR_STOP = 0;
constexpr uint16_t DSHOT_CMD_3D_MODE_ON = 10;
constexpr uint16_t DSHOT_CMD_EDT_ENABLE = 13;

constexpr uint16_t DSHOT_REVERSE_MIN = 48;
constexpr uint16_t DSHOT_FORWARD_MIN = 1048;
constexpr int16_t SPEED_SCALE = 1000;

enum DShotRxStatus : uint8_t {
    DSHOT_RX_OK,
    DSHOT_RX_IDLE,
    DSHOT_RX_NO_REPLY,
    DSHOT_RX_FRAMING,
    DSHOT_RX_BAD_GCR,
    DSHOT_RX_BAD_CRC
};

class Motor {
public:
    // Přidán parametr bidirectional pro volbu jednosměrného / obousměrného režimu
    Motor(uint8_t gpioPin, bool reversed = false, bool bidirectional = true);
    ~Motor();

    bool init();
    
    // Namísto blokujícího arm() budeme frontovat příkazy synchronně
    void sendCommand(uint16_t cmd, uint8_t repeat = 10);

    void setReversed(bool reversed);
    void resetDirectionGuard();
    void stop();
    bool setSpeed(int16_t speed, uint32_t currentMs, bool requestTelemetry = true);
    
    int16_t getSpeed() const;

    // --- Telemetrie ---
    int32_t getErpm() const { return _erpm; }
    float getVoltage() const { return _edtVolts; }
    float getCurrent() const { return _edtAmps; }
    bool isTelemetryValid() const { return _status == DSHOT_RX_OK; }

private:
    uint8_t _gpioPin;
    bool _reversed;
    bool _bidirectional;
    bool _initialized;

    bool _edtEnabled = false;

    int8_t _lastDirection;
    bool _atZero;
    uint32_t _zeroSinceMs;
    int16_t _currentSpeed;

    rmt_channel_handle_t _tx = nullptr;
    rmt_channel_handle_t _rx = nullptr;
    rmt_encoder_handle_t _enc = nullptr;

    uint16_t _tbit = 0, _t0h = 0, _t1h = 0;
    uint32_t _telemQ8 = 0;
    uint16_t _gapMin = 0;
    uint32_t _rxIdleNs = 60000;

    rmt_symbol_word_t _txSym[16];
    rmt_symbol_word_t _rxBuf[64];
    volatile size_t _rxCount = 0;
    volatile bool _rxDone = false;
    bool _rxArmed = false;

    uint16_t _frame = 0;
    uint16_t _cmd = 0;
    uint8_t _cmdRepeat = 0;
    uint16_t _echoPulses = 0;
    int64_t _beginUs = 0;
    bool _escArmed = false;

    DShotRxStatus _status = DSHOT_RX_IDLE;
    uint32_t _erpm = 0;
    uint32_t _periodUs = 0;
    int64_t _lastOkUs = 0;

    bool _edtSeen = false;
    float _edtTemp = NAN, _edtVolts = NAN, _edtAmps = NAN;

    bool sendRaw(uint16_t value);
    void buildFrame(uint16_t value);
    void armRx();
    DShotRxStatus decode(size_t nsym);
    void apply(uint16_t data12);
    
    static bool IRAM_ATTR onRxDone(rmt_channel_handle_t ch, const rmt_rx_done_event_data_t *ev, void *ctx);
};