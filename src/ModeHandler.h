#pragma once
#include <stdint.h>
#include "IMU.h"
#include "LEDHandler.h"

enum class DriveModeType {
    Invalid,
    Idle,
    Forward,
    Spin
};

// Struktura pro kompletní předávání dat
struct RobotContext {
    // --- VSTUPY ---
    bool isConnected;
    int16_t throttle;
    int16_t steering;
    DriveModeType requestedMode;
    IMUData imu1;
    IMUData imu2;

    // --- VÝSTUPY ---
    int16_t leftMotorSpeed;
    int16_t rightMotorSpeed;
    LEDIndication ledIndication;
};

// Rozhraní pro všechny režimy
class IRobotMode {
public:
    virtual ~IRobotMode() = default;
    virtual void init() = 0;
    virtual void calculateResponse(RobotContext& ctx) = 0; 
};

// Deklarace konkrétních režimů
class IdleMode : public IRobotMode {
public:
    void init() override;
    void calculateResponse(RobotContext& ctx) override;
};

class ForwardMode : public IRobotMode {
public:
    void init() override;
    void calculateResponse(RobotContext& ctx) override;
};

class SpinMode : public IRobotMode {
public:
    void init() override;
    void calculateResponse(RobotContext& ctx) override;
};

// Správce režimů
class ModeHandler {
public:
    ModeHandler();
    void setMode(DriveModeType newMode);
    DriveModeType getCurrentModeType() const;
    
    IRobotMode* getCurrentMode() const; 

private:
    IRobotMode* _currentMode;
    DriveModeType _currentModeType;
    
    IdleMode _idleMode;
    ForwardMode _forwardMode;
    SpinMode _spinMode;
};