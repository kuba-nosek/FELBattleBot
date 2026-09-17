#include <Arduino.h>
#include <SPI.h>

// Custom libraries
#include "IMU.h"
#include "LEDHandler.h"
#include "ModeHandler.h"
#include "Motor.h"
#include "Receiver.h"
#include "SignalProcessing.h"
#include "TelemetryManager.h"
#include "config.h"

using namespace RobotConfig;

// --- Global hardware instances ---
Motor motorLeft(PIN_MOTOR_L, RMT_CHANNEL_0, MOTOR_LEFT_REVERSED);
Motor motorRight(PIN_MOTOR_R, RMT_CHANNEL_1, MOTOR_RIGHT_REVERSED);

IMU imu1(PIN_SPI_CS1, IMU1_OFFSET_X_G, IMU1_OFFSET_Y_G, IMU1_OFFSET_Z_G);
IMU imu2(PIN_SPI_CS2, IMU2_OFFSET_X_G, IMU2_OFFSET_Y_G, IMU2_OFFSET_Z_G);

Receiver receiver(PIN_CRSF_RX, PIN_CRSF_TX);
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
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    // Initialize SPI for IMUs
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));

    // Initialize core peripherals
    ledHandler.init();
    receiver.connect();
    receiver.onDisconnect(onFailsafe);

    // --- HARDWARE VERIFICATION ---
    bool hardwareOk = true;

    // Verify motor ESCs
    hardwareOk &= motorLeft.init();
    hardwareOk &= motorRight.init();

    // Verify IMU sensors
    bool imu1Ok = imu1.init();
    bool imu2Ok = imu2.init();

    hardwareOk &= (imu1Ok && imu2Ok);

    // if(!hardwareOk) {
    //     ledHandler.setIndication(LEDIndication::HardwareError);

    //     while(true) {
    //         ledHandler.update();
    //         delay(10);
    //     }
    // }

    // Arm motors if hardware is OK
    motorLeft.arm();
    motorRight.arm();

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

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / TASK_CONTROL_HZ);

    // light indication sync
    modeHandler.update(robot);

    while(true) {
        robot.state.currentMs = millis();

        robot.state.isConnected = receiver.isConnected();

        // Capture one coherent frame so processing cannot mix channel values
        // while the receiver task publishes the next CRSF frame.
        const ReceiverChannels channels = receiver.getChannelsSnapshot();

        const ReceiverInput input = SignalProcessing::processReceiverInput(channels);

        if(robot.state.isConnected) {
            robot.state.requestedMode = modeHandler.decodeMode(input.leftSwitch, input.rightSwitch);
        }

        if(imu1.isAvailable()) imu1.readData(robot.state.imu1);
        if(imu2.isAvailable()) imu2.readData(robot.state.imu2);

        const bool modeChanged = modeHandler.update(robot);
        IRobotMode* currentMode = modeHandler.getCurrentMode();
        currentMode->execute(robot, input);

        if(telemetryManager.shouldSendTelemetry(robot, modeChanged)) {
            telemetryManager.sendTelemetry(robot, *currentMode);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ====================================================================
// RECEIVER THREAD
// ====================================================================
void ReceiverThread(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_RECEIVER_MS);

    while(true) {
        receiver.update();
        vTaskDelay(xFrequency);
    }
}

// ====================================================================
// LED THREAD
// ====================================================================
void LEDThread(void* pvParameters) {
    while(true) {
        ledHandler.update();
        vTaskDelay(1);
    }
}
