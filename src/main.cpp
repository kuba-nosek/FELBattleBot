#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "CRSFReceiver.h"
#include "IndicatorLED.h"
#include "Core/ModeHandler.h"
#include "Utils/SignalProcessing.h"
#include "config.h"

using namespace RobotConfig;

// --- Global hardware instances ---
CRSFReceiver receiver(PIN_CRSF_RX, PIN_CRSF_TX);

IndicatorLED indicator(PIN_LED);

ModeHandler modeHandler;

RobotCore robot;

// --- FreeRTOS task prototypes ---
void mainThread(void* pvParameters);
void ReceiverThread(void* pvParameters);
void LEDThread(void* pvParameters);

// --- Safety failsafe callback ---
void onFailsafe() {
    robot.state.requestedMode = DriveModeType::Idle;
    
    if (robot.hw.indicator) {
        robot.hw.indicator->setIndication(LEDIndication::Failsafe);
    }
}

// OTA update
void enterOTAMode() {
    robot.state.requestedMode = DriveModeType::Idle;

    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    delay(100); 
    WiFi.softAP(RobotConfig::OTA_AP_SSID, RobotConfig::OTA_AP_PASSWORD);
    
    static bool isUpdating = false;

    ArduinoOTA.onStart([]() { 
        isUpdating = true; 
        if (robot.hw.indicator) {
            robot.hw.indicator->setIndication(LEDIndication::OTA_Updating);
        }
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        if (robot.hw.indicator) {
            robot.hw.indicator->update();
        }
    });

    ArduinoOTA.onEnd([]() { isUpdating = false; });
    ArduinoOTA.onError([](ota_error_t error) { isUpdating = false; });

    ArduinoOTA.setHostname(RobotConfig::OTA_HOSTNAME);
    ArduinoOTA.begin();

    while(true) {
        ArduinoOTA.handle();
        
        if (robot.hw.indicator) {
            if (!isUpdating) {
                if (WiFi.softAPgetStationNum() > 0) {
                    robot.hw.indicator->setIndication(LEDIndication::OTA_Connected);
                } else {
                    robot.hw.indicator->setIndication(LEDIndication::OTA_Waiting);
                }
            }
            robot.hw.indicator->update();
        }

        if (receiver.isConnected()) {
            const ReceiverChannels channels = receiver.getChannelsSnapshot();
            const ReceiverInput input = SignalProcessing::processReceiverInput(channels);
            
            if (!input.leftShoulderSwitch) {
                ESP.restart();
            }
        } else {
            ESP.restart();
        }   
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    WiFi.mode(WIFI_OFF);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));

    indicator.init();
    
    indicator.setIndication(LEDIndication::Idle);
    indicator.playAnimation(LEDAnimation::Bootup);

    receiver.connect();
    receiver.onDisconnect(onFailsafe);

    // --- HARDWARE VERIFICATION ---
    bool hardwareOk = true;

    if(!hardwareOk) {
        indicator.setIndication(LEDIndication::HardwareError);

        while(true) {
            indicator.update();
            delay(10);
        }
    }

    robot.hw.rx = &receiver;

    xTaskCreatePinnedToCore(mainThread, "ControlLoop", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(ReceiverThread, "CRSF_RX", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(LEDThread, "LED_Control", 2048, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}

// ====================================================================
// TASK IMPLEMENTATIONS
// ====================================================================

void mainThread(void* pvParameters) {
    const TickType_t taskPeriod = pdMS_TO_TICKS(1000 / TASK_CONTROL_HZ);

    modeHandler.update(robot);
    bool wasConnected = false;

    while(true) {
        robot.state.isConnected = receiver.isConnected();

        const ReceiverChannels channels = receiver.getChannelsSnapshot();
        const ReceiverInput input = SignalProcessing::processReceiverInput(channels);

        if(robot.state.isConnected) {
            DriveModeType decodedMode = modeHandler.decodeMode(input.leftSwitch, input.rightSwitch);
            robot.state.requestedMode = decodedMode;
            
            if(decodedMode == DriveModeType::Idle && input.leftShoulderSwitch) {
                enterOTAMode();
            }
        }

        const bool modeChanged = modeHandler.update(robot);
        IRobotMode* currentMode = modeHandler.getCurrentMode();
        currentMode->execute(robot, input);
        
        if(!robot.state.isConnected) {
            if (robot.hw.indicator) {
                robot.hw.indicator->setIndication(LEDIndication::Failsafe);
            }
        } else if(!wasConnected && !modeChanged) {
            currentMode->init(robot);
        }
        wasConnected = robot.state.isConnected;

        vTaskDelay(taskPeriod);
    }
}

// ====================================================================
// RECEIVER THREAD
// ====================================================================
void ReceiverThread(void* pvParameters) {
    const TickType_t taskPeriod = pdMS_TO_TICKS(1000 / TASK_RECEIVER_HZ);

    while(true) {
        receiver.update();
        vTaskDelay(taskPeriod);
    }
}

// ====================================================================
// LED THREAD
// ====================================================================
void LEDThread(void* pvParameters) {
    const TickType_t taskPeriod = pdMS_TO_TICKS(1000 / TASK_LED_HZ);

    while(true) {
        if (robot.hw.indicator) robot.hw.indicator->update();
        vTaskDelay(taskPeriod);
    }
}
