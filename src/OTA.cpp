#include "OTA.h"

#include "config.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

namespace {
OTACallbacks activeCallbacks;
bool isUpdating = false;
bool hasReportedStatus = false;
OTAStatus lastReportedStatus = OTAStatus::Starting;

void reportStatus(OTAStatus status) {
    if(hasReportedStatus && status == lastReportedStatus) return;

    hasReportedStatus = true;
    lastReportedStatus = status;
    if(activeCallbacks.onStatus) activeCallbacks.onStatus(status);
}
} // namespace

[[noreturn]] void enterOtaMode(const OTACallbacks& callbacks) {
    activeCallbacks = callbacks;
    isUpdating = false;
    hasReportedStatus = false;
    reportStatus(OTAStatus::Starting);

    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    delay(100);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    if(!WiFi.softAP(RobotConfig::OTA_AP_SSID, RobotConfig::OTA_AP_PASSWORD)) {
        reportStatus(OTAStatus::Error);
    }

    ArduinoOTA.onStart([]() {
        isUpdating = true;
        reportStatus(OTAStatus::Updating);
    });
    ArduinoOTA.onEnd([]() {
        isUpdating = false;
        reportStatus(OTAStatus::Finished);
    });
    ArduinoOTA.onError([](ota_error_t) {
        isUpdating = false;
        reportStatus(OTAStatus::Error);
    });

    ArduinoOTA.setHostname(RobotConfig::OTA_HOSTNAME);
    ArduinoOTA.begin();

    while(true) {
        ArduinoOTA.handle();

        if(!isUpdating) {
            reportStatus(WiFi.softAPgetStationNum() > 0 ? OTAStatus::Connected : OTAStatus::Waiting);
        }

        if(activeCallbacks.shouldExit && activeCallbacks.shouldExit()) ESP.restart();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
