#include "TelemetryManager.h"
#include "config.h"

bool TelemetryManager::shouldSendTelemetry(const RobotCore& robot, bool modeChanged) {
    if(modeChanged) immediateSendPending_ = true;
    if(!RobotConfig::TELEMETRY_ENABLED || !robot.state.isConnected) return false;

    const uint32_t timeSinceLastSendMs = robot.state.currentMs - lastSentMs_;
    return immediateSendPending_ || timeSinceLastSendMs >= RobotConfig::TELEMETRY_INTERVAL_MS;
}

void TelemetryManager::sendTelemetry(const RobotCore& robot, const IRobotMode& activeMode) {
    if (!robot.hw.telemetryLink) return;

    currentPayload_ = TelemetryData{};
    activeMode.fillTelemetryData(robot, currentPayload_);

    if (robot.hw.telemetryLink->sendTelemetry(currentPayload_)) {
        lastSentMs_ = robot.state.currentMs;
        immediateSendPending_ = false;
    }
}
