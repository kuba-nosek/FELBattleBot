#include <Arduino.h>
#include <SPI.h>

// Custom libraries
#include "config.h"
#include "Motor.h"
#include "IMU.h"
#include "Receiver.h"
#include "LEDHandler.h"
#include "ModeHandler.h"

#include "SignalProcessing.h"

using namespace RobotConfig;

// --- Global hardware instances ---
Motor motorLeft(PIN_MOTOR_L, RMT_CHANNEL_0, false);
Motor motorRight(PIN_MOTOR_R, RMT_CHANNEL_1, true);

IMU imu1(PIN_SPI_CS1, IMU1_OFFSET_X, IMU1_OFFSET_Y, IMU1_OFFSET_Z);
IMU imu2(PIN_SPI_CS2, IMU2_OFFSET_X, IMU2_OFFSET_Y, IMU2_OFFSET_Z);

Receiver receiver(PIN_CRSF_RX, PIN_CRSF_TX);
LEDHandler ledHandler(PIN_LED);
ModeHandler modeHandler;

// --- FreeRTOS task prototypes ---
void mainThread(void *pvParameters);
void ReceiverThread(void *pvParameters);
void LEDThread(void *pvParameters);

// --- Safety failsafe callback ---
void onFailsafe() {
    ledHandler.playAnimation(LEDAnimation::ErrorAlert);
    modeHandler.setMode(DriveModeType::Idle);
    motorLeft.stop();
    motorRight.stop();
}

void setup() {
    Serial.begin(115200);

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

    if (!hardwareOk) {
        Serial.println("CRITICAL ERROR: Hardware initialization failed!");
        
        while (true) {
            // Infinite loop on hardware failure with LED alert
            ledHandler.playAnimation(LEDAnimation::ErrorAlert);
            ledHandler.update(millis());
            delay(10);
        }
    }

    // Arm motors if hardware is OK
    motorLeft.arm();
    motorRight.arm();

    // Spawn FreeRTOS tasks
    xTaskCreate(mainThread, "ControlLoop", 4096, NULL, 3, NULL);
    xTaskCreate(ReceiverThread, "CRSF_RX", 4096, NULL, 2, NULL);
    xTaskCreate(LEDThread, "LED_Control", 2048, NULL, 1, NULL);
}

void loop() {
    // Delete default Arduino task to free memory
    vTaskDelete(NULL);
}

// ====================================================================
// TASK IMPLEMENTATIONS
// ====================================================================

void mainThread(void *pvParameters) {
  RobotContext ctx;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / TASK_CONTROL_HZ);

  while (true) {
    uint32_t currentMs = millis();

    // 1. READ INPUTS
    ctx.isConnected = receiver.isConnected();
    if (ctx.isConnected) {
      ctx.throttle = SignalProcessing::normalizeChannel(receiver.getChannel(1));
      ctx.steering = SignalProcessing::normalizeChannel(receiver.getChannel(0));
      ctx.requestedMode = SignalProcessing::decodeMode(receiver.getChannel(7));
    } else {
      // Force Idle mode on signal loss
      ctx.requestedMode = DriveModeType::Idle;
    }

    // Read accelerometer data (Future: Melty Brain math basis)
    if (imu1.isAvailable()) {
      imu1.readData(ctx.imu1);
    }
    if (imu2.isAvailable()) {
      imu2.readData(ctx.imu2);
    }

    // 2. MODE SWITCH CHECK
    if (ctx.requestedMode != modeHandler.getCurrentModeType()) {
      modeHandler.setMode(ctx.requestedMode);
      ledHandler.playAnimation(LEDAnimation::ModeChanged);
    }

    // 3. CALCULATE RESPONSE
    // Execute active mode logic via RobotContext
    modeHandler.getCurrentMode()->calculateResponse(ctx);

    // 4. HARDWARE OUTPUT
    // Apply motor speeds
    motorLeft.setSpeed(ctx.leftMotorSpeed, currentMs);
    motorRight.setSpeed(ctx.rightMotorSpeed, currentMs);
    
    // Update LED state (Animations are handled dynamically by LEDThread)
    ledHandler.setIndication(ctx.ledIndication);

    // Maintain strict loop frequency
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void ReceiverThread(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_RECEIVER_MS);

    while (true) {
        receiver.update(millis());
        vTaskDelay(xFrequency);
    }
}

void LEDThread(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_LED_MS);

    while (true) {
        ledHandler.update(millis());

        DriveModeType currentMode = modeHandler.getCurrentModeType();
        if (!receiver.isConnected()) {
            ledHandler.setIndication(LEDIndication::Failsafe);
        } else if (currentMode == DriveModeType::Forward) {
            ledHandler.setIndication(LEDIndication::Forward);
        } else if (currentMode == DriveModeType::Spin) {
            ledHandler.setIndication(LEDIndication::Spin);
        } else {
            ledHandler.setIndication(LEDIndication::Idle);
        }

        vTaskDelay(xFrequency);
    }
}