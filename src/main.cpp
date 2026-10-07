#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// Custom libraries
#include "H3LIS331DL.h"
#include "CRSFReceiver.h"
#include "LEDHandler.h"
#include "ModeHandler.h"
#include "Motor.h"
#include "SignalProcessing.h"
#include "TelemetryManager.h"
#include "config.h"

using namespace RobotConfig;

// --- Global hardware instances ---
Motor motorLeft(PIN_MOTOR_L, MOTOR_LEFT_REVERSED, MOTOR_POLE_PAIRS);
Motor motorRight(PIN_MOTOR_R, MOTOR_RIGHT_REVERSED, MOTOR_POLE_PAIRS);

H3LIS331DL accel1(PIN_SPI_CS1, ACCEL1_OFFSET_X_G, ACCEL1_OFFSET_Y_G, ACCEL1_OFFSET_Z_G);
H3LIS331DL accel2(PIN_SPI_CS2, ACCEL2_OFFSET_X_G, ACCEL2_OFFSET_Y_G, ACCEL2_OFFSET_Z_G);

CRSFReceiver receiver(PIN_CRSF_RX, PIN_CRSF_TX);
LEDHandler ledHandler(PIN_LED);
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
    robot.hw.led->setIndication(LEDIndication::Failsafe);
    robot.hw.led->setStripMode(LEDStripMode::Static);
    robot.hw.led->setAnimation(LEDStripAnimation::Failsafe);
}

void enterOTAMode() {

    robot.state.requestedMode = DriveModeType::Idle;

    WiFi.mode(WIFI_AP);
    WiFi.softAP("MELTY-OTA", "heslo_pro_ota");
    
    ArduinoOTA.setHostname("meltybrain-ota");
    ArduinoOTA.begin();

    while(true) {
        ArduinoOTA.handle();
        ledHandler.update(); // Zajištění nepřetržitého chodu blikání LED
        
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    // Initialize SPI for IMUs
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));

    // Initialize core peripherals
    ledHandler.init();
    ledHandler.setIndication(LEDIndication::Idle);
    ledHandler.playAnimation(LEDAnimation::Bootup);
    ledHandler.setStripMode(LEDStripMode::Static);
    ledHandler.setAnimation(LEDStripAnimation::Bootup);
    receiver.connect();
    receiver.onDisconnect(onFailsafe);

    // --- HARDWARE VERIFICATION ---
    bool hardwareOk = true;

    // Verify motor ESCs
    hardwareOk &= motorLeft.init();
    hardwareOk &= motorRight.init();

    // Verify IMU sensors
    bool accel1ok = accel1.init();
    bool accel2ok = accel2.init();

    hardwareOk &= (accel1ok);

    if(!hardwareOk) {
        ledHandler.setIndication(LEDIndication::HardwareError);
        ledHandler.setStripMode(LEDStripMode::Static);
        ledHandler.setAnimation(LEDStripAnimation::HardwareError);

        while(true) {
            ledHandler.update();
            delay(10);
        }
    }


    // Spawn FreeRTOS tasks
    xTaskCreate(mainThread, "ControlLoop", 4096, NULL, 3, NULL);
    xTaskCreate(ReceiverThread, "CRSF_RX", 4096, NULL, 2, NULL);
    xTaskCreate(LEDThread, "LED_Control", 2048, NULL, 1, NULL);
}

void loop() {
    vTaskDelete(NULL);
}

// ====================================================================
// TASK IMPLEMENTATIONS
// ====================================================================

void mainThread(void* pvParameters) {
    robot.hw.leftMotor = &motorLeft;
    robot.hw.rightMotor = &motorRight;
    robot.hw.led = &ledHandler;
    robot.hw.rx = &receiver;
    robot.hw.telemetryLink = &receiver;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t taskPeriod = pdMS_TO_TICKS(1000 / TASK_CONTROL_HZ);

    // light indication sync
    modeHandler.update(robot);
    bool wasConnected = false;

    while(true) {
        robot.state.currentMs = millis();

        robot.state.isConnected = receiver.isConnected();

        // Capture one coherent frame so processing cannot mix channel values
        // while the receiver task publishes the next CRSF frame.
        const ReceiverChannels channels = receiver.getChannelsSnapshot();

        const ReceiverInput input = SignalProcessing::processReceiverInput(channels);

        // TODO aby fungovalo (neni zatim definovany otabutton nikde)
        /*
        if (input.otaButton) {
            enterOTAMode();
        }
        */

        if(robot.state.isConnected) {
            robot.state.requestedMode = modeHandler.decodeMode(input.leftSwitch, input.rightSwitch);
        }

        if(accel1.isAvailable()) accel1.readData(robot.state.accel1);
        if(accel2.isAvailable()) accel2.readData(robot.state.accel2);

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
            ledHandler.setIndication(LEDIndication::Failsafe);
            ledHandler.setStripMode(LEDStripMode::Static);
            ledHandler.setAnimation(LEDStripAnimation::Failsafe);
        } else if(!wasConnected && !modeChanged) {
            currentMode->init(robot);
        }
        wasConnected = robot.state.isConnected;

        if(telemetryManager.shouldSendTelemetry(robot, modeChanged)) {
            telemetryManager.sendTelemetry(robot, *currentMode);
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
    const TickType_t taskPeriod = pdMS_TO_TICKS(1000 / TASK_LED_HZ);

    while(true) {
        ledHandler.update();
        vTaskDelay(taskPeriod);
    }
}
