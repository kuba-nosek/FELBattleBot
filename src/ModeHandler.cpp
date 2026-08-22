#include "ModeHandler.h"

// ====================================================================
// IDLE MODE
// ====================================================================
void IdleMode::init() {
    // One-time initialization for Idle mode
}

void IdleMode::calculateResponse(RobotContext& ctx) {
    // Stop motors during idle or signal loss
    ctx.leftMotorSpeed = 0;
    ctx.rightMotorSpeed = 0;

    // Set LED indication based on connection
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
    // One-time initialization for Forward mode
}

void ForwardMode::calculateResponse(RobotContext& ctx) {
    int32_t left = ctx.throttle;
    int32_t right = ctx.throttle;

    // Apply differential steering (throttle/steering mix)
    if (ctx.steering < 0) {
        left = left * (1000 + ctx.steering) / 1000;
    } else if (ctx.steering > 0) {
        right = right * (1000 - ctx.steering) / 1000;
    }

    ctx.leftMotorSpeed = static_cast<int16_t>(left);
    ctx.rightMotorSpeed = static_cast<int16_t>(right);
    
    // Forward LED indication
    ctx.ledIndication = LEDIndication::Forward;
}

// ====================================================================
// SPIN MODE
// ====================================================================
void SpinMode::init() {
    // One-time initialization for Spin mode
}

void SpinMode::calculateResponse(RobotContext& ctx) {
    // Tank spin: motors rotate in opposite directions (Future: Melty translation here)
    ctx.leftMotorSpeed = ctx.throttle;
    ctx.rightMotorSpeed = -ctx.throttle;

    // Spin LED indication (Future: Melty Brain strobe sync)
    ctx.ledIndication = LEDIndication::Spin;
}

// ====================================================================
// MODE HANDLER (State Manager)
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

    // Trigger initialization of the new active mode
    _currentMode->init();
}

DriveModeType ModeHandler::getCurrentModeType() const {
    return _currentModeType;
}

IRobotMode* ModeHandler::getCurrentMode() const {
    return _currentMode;
}