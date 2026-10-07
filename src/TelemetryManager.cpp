#include "TelemetryManager.h"

#include "config.h"

bool TelemetryManager::shouldSendTelemetry(const RobotCore& robot, bool modeChanged) {
    if(modeChanged) immediateSendPending_ = true;
    if(!RobotConfig::TELEMETRY_ENABLED || !robot.state.isConnected || robot.hw.telemetryLink == nullptr) return false;

    const uint32_t timeSinceLastSendMs = robot.state.currentMs - lastSentMs_;
    return immediateSendPending_ || timeSinceLastSendMs >= RobotConfig::TELEMETRY_INTERVAL_MS;
}

void TelemetryManager::sendTelemetry(RobotCore& robot, const IRobotMode& activeMode) {
    BattlebotTelemetry::TelemetryData telemetry{};

    activeMode.sendTelemetry(robot, telemetry);

    if(robot.hw.telemetryLink->sendBattlebotTelemetry(telemetry)) {
        lastSentMs_ = robot.state.currentMs;
        immediateSendPending_ = false;
    }
}