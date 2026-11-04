#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>

#include "H3LIS331DL.h"
#include "CRSFReceiver.h"
#include "IndicatorLED.h"
#include "RotaryDisplay.h"
#include "Core/ModeHandler.h"
#include "AM32Motor.h"
#include "Utils/SignalProcessing.h"
#include "Core/TelemetryManager.h"
#include "config.h"
#include "OTA.h"

using namespace RobotConfig;

// --- Global hardware instances ---
AM32Motor motorLeft(PIN_MOTOR_L, MOTOR_LEFT_REVERSED, MOTOR_POLE_PAIRS);
AM32Motor motorRight(PIN_MOTOR_R, MOTOR_RIGHT_REVERSED, MOTOR_POLE_PAIRS);
H3LIS331DL accel1(PIN_SPI_CS1, ACCEL1_OFFSET_X_G, ACCEL1_OFFSET_Y_G, ACCEL1_OFFSET_Z_G);
H3LIS331DL accel2(PIN_SPI_CS2, ACCEL2_OFFSET_X_G, ACCEL2_OFFSET_Y_G, ACCEL2_OFFSET_Z_G);
CRSFReceiver receiver(PIN_CRSF_RX, PIN_CRSF_TX);
IndicatorLED indicator(PIN_LED);
RotaryDisplay rotaryDisplay(PIN_LED_STRIP);

RobotCore robot;
ModeHandler modeHandler;
TelemetryManager telemetryManager;

void mainThread(void* pvParameters);
void ReceiverThread(void* pvParameters);
void LEDThread(void* pvParameters);
void accelThread(void* pvParameters);
void accel1_isr();
void accel2_isr();
void onFailsafe();
void onOTAStatus(OTAStatus status);
bool shouldExitOtaMode();

TaskHandle_t accelTaskHandle = nullptr;
TaskHandle_t mainTaskHandle = nullptr;

void setup() {
    Serial.begin(115200);
    delay(1000);

    WiFi.mode(WIFI_OFF);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

    indicator.init();
    rotaryDisplay.init();
    
    indicator.setIndication(LEDIndication::Idle);
    indicator.playAnimation(LEDAnimation::Bootup);
    
    rotaryDisplay.setMode(RotaryDisplayMode::Static);
    rotaryDisplay.setAnimation(LEDStripAnimation::Bootup);

    receiver.connect();
    receiver.onDisconnect(onFailsafe);

    bool hardwareOk = true;

    hardwareOk &= motorLeft.init();
    hardwareOk &= motorRight.init();

    bool accel1ok = accel1.init();
    bool accel2ok = accel2.init();

    hardwareOk &= (accel1ok && accel2ok);

    if(!hardwareOk) {
        indicator.setIndication(LEDIndication::HardwareError);
        rotaryDisplay.setMode(RotaryDisplayMode::Static);
        rotaryDisplay.setAnimation(LEDStripAnimation::HardwareError);

        while(true) {
            indicator.update();
            rotaryDisplay.update();
            delay(10);
        }
    }

    pinMode(PIN_ACCEL1_INT1, INPUT);
    pinMode(PIN_ACCEL2_INT1, INPUT);

    robot.hw.leftMotor = &motorLeft;
    robot.hw.rightMotor = &motorRight;
    robot.hw.rx = &receiver;
    robot.hw.indicator = &indicator;
    robot.hw.rotaryDisplay = &rotaryDisplay;

    xTaskCreatePinnedToCore(mainThread, "ControlLoop", 8192, NULL, 3, &mainTaskHandle, 0);
    xTaskCreatePinnedToCore(ReceiverThread, "CRSF_RX", 4096, NULL, 2, NULL, 0);

    xTaskCreatePinnedToCore(LEDThread, "LED_Control", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(accelThread, "Accel_Task", 4096, nullptr, 4, &accelTaskHandle, 1);

    attachInterrupt(digitalPinToInterrupt(PIN_ACCEL1_INT1), accel1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ACCEL2_INT1), accel2_isr, RISING);

    xTaskNotifyGive(accelTaskHandle);
}

// ====================================================================
// TASK IMPLEMENTATIONS
// ====================================================================

void mainThread(void* pvParameters) {
    
    modeHandler.update(robot);
    bool wasConnected = false;

    while(true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        robot.state.currentMs = millis();
        robot.state.isConnected = receiver.isConnected();

        const ReceiverChannels channels = receiver.getChannelsSnapshot();
        const ReceiverInput input = SignalProcessing::processReceiverInput(channels);

        if(robot.state.isConnected) {
            DriveModeType decodedMode = modeHandler.decodeMode(input.leftSwitch, input.rightSwitch);
            robot.state.requestedMode = decodedMode;
            
            if(decodedMode == DriveModeType::Idle && input.leftShoulderSwitch) {
                enterOtaMode({onOTAStatus, shouldExitOtaMode});
            }
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
        if (robot.hw.rotaryDisplay) robot.hw.rotaryDisplay->update();
        vTaskDelay(taskPeriod);
    }
}

// ====================================================================
// ACCEL THREAD
// ====================================================================
void accelThread(void* pvParameters) {
    uint32_t lastReadUs = 0;
    // TODO dat do configu a sychronizovat s nastavenim akcelerometru
    const uint32_t RATE_LIMIT_US = 800; // Spodní hranice ochrany proti 2000Hz double-readům
    
    while(true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        uint32_t nowUs = micros();

        if (nowUs - lastReadUs >= RATE_LIMIT_US) {
            lastReadUs = nowUs;

            AccelData temp1, temp2;
            
            accel1.readData(temp1); 
            robot.state.accel1 = temp1;

            accel2.readData(temp2);
            robot.state.accel2 = temp2;

            if (mainTaskHandle != nullptr) {
                xTaskNotifyGive(mainTaskHandle);
            }
        }
    }
}

// ====================================================================
// ARDUINO IDE LOOP DELETE
// ====================================================================
void loop() {
    vTaskDelete(NULL);
}

// ====================================================================
// SUPPORT FUNCTIONS
// ====================================================================
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
    if (robot.hw.rotaryDisplay) {
        robot.hw.rotaryDisplay->setMode(RotaryDisplayMode::Static);
        robot.hw.rotaryDisplay->setAnimation(LEDStripAnimation::Failsafe);
    }
}

void onOTAStatus(OTAStatus status) {
    switch(status) {
        case OTAStatus::Starting:
            robot.state.requestedMode = DriveModeType::Idle;
            if(robot.hw.rotaryDisplay) {
                robot.hw.rotaryDisplay->setMode(RotaryDisplayMode::Static);
                robot.hw.rotaryDisplay->setAnimation(LEDStripAnimation::Bootup);
            }
            break;
        case OTAStatus::Waiting:
            indicator.setIndication(LEDIndication::OTA_Waiting);
            break;
        case OTAStatus::Connected:
            indicator.setIndication(LEDIndication::OTA_Connected);
            break;
        case OTAStatus::Updating:
            indicator.setIndication(LEDIndication::OTA_Updating);
            break;
        case OTAStatus::Error:
            indicator.playAnimation(LEDAnimation::ErrorAlert);
            break;
        case OTAStatus::Finished:
            break;
    }
}

bool shouldExitOtaMode() {
    if(!receiver.isConnected()) return true;

    const ReceiverChannels channels = receiver.getChannelsSnapshot();
    const ReceiverInput input = SignalProcessing::processReceiverInput(channels);
    return !input.leftShoulderSwitch;
}
