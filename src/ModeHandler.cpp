#include "ModeHandler.h"

// ====================================================================
// IDLE MODE
// ====================================================================
void IdleMode::init() {
    // Kód vykonaný jednorázově při přepnutí do režimu Idle
}

void IdleMode::calculateResponse(RobotContext& ctx) {
    // V nečinnosti (nebo při ztrátě signálu) motory stojí
    ctx.leftMotorSpeed = 0;
    ctx.rightMotorSpeed = 0;

    // Nastavení LED indikace podle stavu spojení
    if (!ctx.isConnected) {
        ctx.ledIndication = LEDIndication::Failsafe;
    } else {
        ctx.ledIndication = LEDIndication::Idle;
    }
}

// ====================================================================
// FORWARD MODE
// ====================================================================
void ForwardMode::init() {
    // Kód vykonaný jednorázově při přepnutí do režimu Forward
}

void ForwardMode::calculateResponse(RobotContext& ctx) {
    int32_t left = ctx.throttle;
    int32_t right = ctx.throttle;

    // Aplikace diferenciálního řízení (mix plynu a zatáčení)
    if (ctx.steering < 0) {
        left = left * (1000 + ctx.steering) / 1000;
    } else if (ctx.steering > 0) {
        right = right * (1000 - ctx.steering) / 1000;
    }

    ctx.leftMotorSpeed = static_cast<int16_t>(left);
    ctx.rightMotorSpeed = static_cast<int16_t>(right);
    
    // Nastavení specifické indikace pro jízdu
    ctx.ledIndication = LEDIndication::Forward;
}

// ====================================================================
// SPIN MODE
// ====================================================================
void SpinMode::init() {
    // Kód vykonaný jednorázově při přepnutí do režimu Spin
}

void SpinMode::calculateResponse(RobotContext& ctx) {
    // V režimu Spin se motory točí stejnou rychlostí proti sobě
    ctx.leftMotorSpeed = ctx.throttle;
    ctx.rightMotorSpeed = -ctx.throttle;

    // Nastavení specifické indikace pro rotaci
    ctx.ledIndication = LEDIndication::Spin;
}

// ====================================================================
// MODE HANDLER (Správce režimů)
// ====================================================================
ModeHandler::ModeHandler() 
    : _currentMode(&_idleMode), _currentModeType(DriveModeType::Idle) {
}

void ModeHandler::setMode(DriveModeType newMode) {
    if (_currentModeType == newMode) {
        return;
    }

    _currentModeType = newMode;

    switch (newMode) {
        case DriveModeType::Forward:
            _currentMode = &_forwardMode;
            break;
        case DriveModeType::Spin:
            _currentMode = &_spinMode;
            break;
        default:
            _currentMode = &_idleMode;
            _currentModeType = DriveModeType::Idle;
            break;
    }

    // Provedení inicializační rutiny nově zvoleného režimu. Operátor -> pro ukazatel.
    _currentMode->init();
}

DriveModeType ModeHandler::getCurrentModeType() const {
    return _currentModeType;
}

IRobotMode* ModeHandler::getCurrentMode() const {
    return _currentMode;
}