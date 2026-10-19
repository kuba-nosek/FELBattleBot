#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "H3LIS331DL.h"
#include "CRSFReceiver.h"
#include "IndicatorLED.h"
#include "POVDisplay.h"
#include "Core/ModeHandler.h"
#include "AM32Motor.h"
#include "Utils/SignalProcessing.h"
#include "Core/TelemetryManager.h"
#include "config.h"

using namespace RobotConfig;

// --- Global hardware instances ---
AM32Motor motorLeft(PIN_MOTOR_L, MOTOR_LEFT_REVERSED, MOTOR_POLE_PAIRS);
AM32Motor motorRight(PIN_MOTOR_R, MOTOR_RIGHT_REVERSED, MOTOR_POLE_PAIRS);

H3LIS331DL accel1(PIN_SPI_CS1, ACCEL1_OFFSET_X_G, ACCEL1_OFFSET_Y_G, ACCEL1_OFFSET_Z_G);
H3LIS331DL accel2(PIN_SPI_CS2, ACCEL2_OFFSET_X_G, ACCEL2_OFFSET_Y_G, ACCEL2_OFFSET_Z_G);

CRSFReceiver receiver(PIN_CRSF_RX, PIN_CRSF_TX);

IndicatorLED indicator(PIN_LED);
POVDisplay povDisplay(PIN_LED_STRIP);

ModeHandler modeHandler;
TelemetryManager telemetryManager;

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
    if (robot.hw.povDisplay) {
        robot.hw.povDisplay->setMode(POVDisplayMode::Static);
        robot.hw.povDisplay->setAnimation(LEDStripAnimation::Failsafe);
    }
}

// OTA update
void enterOTAMode() {
    robot.state.requestedMode = DriveModeType::Idle;

    if (robot.hw.povDisplay) {
        robot.hw.povDisplay->setMode(POVDisplayMode::Static);
        robot.hw.povDisplay->setAnimation(LEDStripAnimation::Bootup);
    }

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
    povDisplay.init();
    
    indicator.setIndication(LEDIndication::Idle);
    indicator.playAnimation(LEDAnimation::Bootup);
    
    povDisplay.setMode(POVDisplayMode::Static);
    povDisplay.setAnimation(LEDStripAnimation::Bootup);

    receiver.connect();
    receiver.onDisconnect(onFailsafe);

    // --- HARDWARE VERIFICATION ---
    bool hardwareOk = true;

    hardwareOk &= motorLeft.init();
    hardwareOk &= motorRight.init();

    bool accel1ok = accel1.init();
    bool accel2ok = accel2.init();

    hardwareOk &= (accel1ok);

    if(!hardwareOk) {
        indicator.setIndication(LEDIndication::HardwareError);
        povDisplay.setMode(POVDisplayMode::Static);
        povDisplay.setAnimation(LEDStripAnimation::HardwareError);

        while(true) {
            indicator.update();
            povDisplay.update();
            delay(10);
        }
    }

    robot.hw.leftMotor = &motorLeft;
    robot.hw.rightMotor = &motorRight;
    robot.hw.rx = &receiver;
    robot.hw.indicator = &indicator;
    robot.hw.povDisplay = &povDisplay;

    // Spawn FreeRTOS tasks
    xTaskCreate(mainThread, "ControlLoop", 8192, NULL, 3, NULL);
    xTaskCreate(ReceiverThread, "CRSF_RX", 4096, NULL, 2, NULL);
    xTaskCreate(LEDThread, "LED_Control", 4096, NULL, 3, NULL);
    // Spawn FreeRTOS tasks pinned to respective cores
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
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t taskPeriod = pdMS_TO_TICKS(1000 / TASK_CONTROL_HZ);

    modeHandler.update(robot);
    bool wasConnected = false;

    while(true) {
        robot.state.currentMs = millis();
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

        if(accel1.isAvailable()) {
            AccelData temp;
            accel1.readData(temp);
            robot.state.accel1 = temp;
        }
        if(accel2.isAvailable()) {
            AccelData temp;
            accel2.readData(temp);
            robot.state.accel2 = temp;
        }

        const bool modeChanged = modeHandler.update(robot);
        IRobotMode* currentMode = modeHandler.getCurrentMode();
        currentMode->execute(robot, input);

        if(robot.state.isConnected) {
            if(motorLeft.isTelemetryValid()) {
                robot.state.escLeftRpm = motorLeft.getRpm();
                robot.state.escLeftVolts = motorLeft.getVoltage();
            }
            if(motorRight.isTelemetryValid()) {
                robot.state.escRightRpm = motorRight.getRpm();
                robot.state.escRightVolts = motorRight.getVoltage();
            }
        }
        
        if(!robot.state.isConnected) {
            if (robot.hw.indicator) {
                robot.hw.indicator->setIndication(LEDIndication::Failsafe);
            }
        } else if(!wasConnected && !modeChanged) {
            currentMode->init(robot);
        }
        wasConnected = robot.state.isConnected;

        if(telemetryManager.shouldSendTelemetry(robot, modeChanged)) {
            telemetryManager.sendTelemetry(robot, *currentMode, receiver); 
        }

        vTaskDelayUntil(&xLastWakeTime, taskPeriod);
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
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskPeriod = pdMS_TO_TICKS(1000 / TASK_LED_HZ);

    while(true) {
        if (robot.hw.indicator) robot.hw.indicator->update();
        if (robot.hw.povDisplay) robot.hw.povDisplay->update();
        vTaskDelay(taskPeriod);
    }
}
