#include "TelemetryManager.h"

#include "config.h"

bool TelemetryManager::shouldSendTelemetry(const RobotCore& robot, bool modeChanged) {
    if(modeChanged) immediateSendPending_ = true;
    if(!RobotConfig::TELEMETRY_ENABLED || !robot.state.isConnected) return false;

    const uint32_t timeSinceLastSendMs = robot.state.currentMs - lastSentMs_;
    return immediateSendPending_ || timeSinceLastSendMs >= RobotConfig::TELEMETRY_INTERVAL_MS;
}

void TelemetryManager::sendTelemetry(RobotCore& robot, const IRobotMode& activeMode) {
    BattlebotTelemetry::TelemetryData telemetry{};

    // 1. Předvyplnění univerzálních hardwarových dat
    telemetry.escLeftRpm = robot.state.escLeftRpm;
    telemetry.escLeftVolts = robot.state.escLeftVolts;

    // (Tip: Stejným způsobem sem z tvých módů přesunout i přiřazování akcelerometrů,
    // např. telemetry.accel1X = robot.state.imu1.xMps2, ušetříš si tak duplicitní kód v módech.)

    // 2. Přidání specifických logických dat z aktuálně aktivního módu
    activeMode.sendTelemetry(robot, telemetry);

    if(robot.hw.rx->sendBattlebotTelemetry(telemetry)) {
        lastSentMs_ = robot.state.currentMs;
        immediateSendPending_ = false;
    }
}