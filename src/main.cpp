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
#include "Logging/DataLogger.h"

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
DataLogger dataLogger; //shared between controlloop and loggingstorage

RobotCore robot;

// --- FreeRTOS task prototypes ---
void mainThread(void* pvParameters);
void ReceiverThread(void* pvParameters);
void LEDThread(void* pvParameters);
void LoggerStorageThread(void* pvParameters);
void accelThread(void* pvParameters);

// Globální handle pro notifikace
TaskHandle_t accelTaskHandle = nullptr;
TaskHandle_t mainTaskHandle = nullptr;

// ISR funkce musí být v IRAM (nesmí se stránkovat z flash paměti)
void IRAM_ATTR accel1_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(accelTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void IRAM_ATTR accel2_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(accelTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

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

    // Allocate RAM buffer, mount LittleFS (no automatic erase).
    dataLogger.begin();

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

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

    hardwareOk &= (accel1ok && accel2ok);

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

    pinMode(PIN_ACCEL1_INT1, INPUT);
    pinMode(PIN_ACCEL2_INT1, INPUT);

    robot.hw.leftMotor = &motorLeft;
    robot.hw.rightMotor = &motorRight;
    robot.hw.rx = &receiver;
    robot.hw.indicator = &indicator;
    robot.hw.povDisplay = &povDisplay;

    // --- Jádro 0 (PRO_CPU): Rozhodování, pohyb a komunikace ---
    xTaskCreatePinnedToCore(mainThread, "ControlLoop", 8192, NULL, 3, &mainTaskHandle, 0);
    xTaskCreatePinnedToCore(ReceiverThread, "CRSF_RX", 4096, NULL, 2, NULL, 0);

    // --- Jádro 1 (APP_CPU): Senzory, logger a vizuální periferie (včetně ISR) ---
    xTaskCreatePinnedToCore(accelThread, "IMU_Task", 4096, nullptr, 4, &accelTaskHandle, 1);
    xTaskCreatePinnedToCore(LoggerStorageThread, "LogStorage", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(LEDThread, "LED_Control", 2048, NULL, 1, NULL, 1);

    attachInterrupt(digitalPinToInterrupt(PIN_ACCEL1_INT1), accel1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ACCEL2_INT1), accel2_isr, RISING);

    xTaskNotifyGive(accelTaskHandle);
}

void loop() {
    vTaskDelete(NULL);
}

// ====================================================================
// TASK IMPLEMENTATIONS
// ====================================================================

void mainThread(void* pvParameters) {
    modeHandler.update(robot);
    bool wasConnected = false;
    uint32_t idleSinceMs = 0; //after enough time in idle mode - starts flashing

    while(true) {
        // Blokování smyčky čekáním na data od akcelerometru
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        robot.state.currentMs = millis();
        robot.state.isConnected = receiver.isConnected();

        const ReceiverChannels channels = receiver.getChannelsSnapshot();
        const ReceiverInput input = SignalProcessing::processReceiverInput(channels);

        if(robot.state.isConnected) {
            DriveModeType decodedMode = modeHandler.decodeMode(input.leftSwitch, input.rightSwitch);

            // A pending flash save locks re-arming once the robot reaches Idle.
            // While still spinning, let the operator disarm normally.
            if(dataLogger.hasUnsavedData() && modeHandler.getCurrentModeID() == DriveModeType::Idle) {
               decodedMode = DriveModeType::Idle;
            }

            robot.state.requestedMode = decodedMode;
            
            if(decodedMode == DriveModeType::Idle && input.leftShoulderSwitch && !dataLogger.isBusy()) {
                enterOTAMode();
            }
        }

        // CH7: right 3-position switch, trigger only on a LOW -> HIGH transition.
        dataLogger.updateTrigger(robot.state.isConnected, input.right3StateSwitch == 1);

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

        // Fast RAM-only sampling. No Serial or filesystem calls here.
        dataLogger.capture(robot, input, micros());
        
        // Flash operations are permitted only after the motors have remained
        // in Idle for a short settling interval.
        const uint32_t nowMs = millis();
        if(modeHandler.getCurrentModeID() == DriveModeType::Idle) {
            if(idleSinceMs == 0) idleSinceMs = nowMs;
            dataLogger.setSafeToSave(nowMs - idleSinceMs >= LOG_SETTLE_IDLE_MS);
        } else {
            idleSinceMs = 0;
            dataLogger.setSafeToSave(false);
        }
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
        if (robot.hw.povDisplay) robot.hw.povDisplay->update();
        vTaskDelay(taskPeriod);
    }
}

// ====================================================================
// LOW PRIORITY STORAGE / USB CONTROL (CORE 1)
// ====================================================================
void LoggerStorageThread(void* pvParameters) {
    while (true) {
        dataLogger.service();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// accelerometer data readout
void accelThread(void* pvParameters) {
    while(true) {
        // Vlákno zde spí a nekonzumuje žádný CPU čas. 
        // Probudí se okamžitě, jakmile jakýkoliv ISR zavolá vTaskNotifyGiveFromISR.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        AccelData temp1, temp2;
        
        accel1.readData(temp1); 
        robot.state.accel1 = temp1; // Zápis přes bezpečný ThreadSafe wrapper

        accel2.readData(temp2);
        robot.state.accel2 = temp2;

        if (mainTaskHandle != nullptr) {
            xTaskNotifyGive(mainTaskHandle);
        }
    }
}
