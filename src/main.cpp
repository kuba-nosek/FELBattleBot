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

#if ENABLE_DEBUG_AP
    #include <WiFi.h>
    #include <WiFiUdp.h>
    WiFiUDP udp;
#endif

using namespace RobotConfig;

// --- Global hardware instances ---
Motor motorLeft(PIN_MOTOR_L, RMT_CHANNEL_0, false);
Motor motorRight(PIN_MOTOR_R, RMT_CHANNEL_1, true);

IMU imu1(PIN_SPI_CS1, IMU1_OFFSET_X_G, IMU1_OFFSET_Y_G, IMU1_OFFSET_Z_G);
IMU imu2(PIN_SPI_CS2, IMU2_OFFSET_X_G, IMU2_OFFSET_Y_G, IMU2_OFFSET_Z_G);

Receiver receiver(PIN_CRSF_RX, PIN_CRSF_TX);
LEDHandler ledHandler(PIN_LED);
ModeHandler modeHandler;

RobotCore robot;

// --- FreeRTOS task prototypes ---
void mainThread(void *pvParameters);
void ReceiverThread(void *pvParameters);
void LEDThread(void *pvParameters);

// --- Safety failsafe callback ---
void onFailsafe() {
  robot.state.requestedMode = DriveModeType::Idle;
  robot.hw.led->setIndication(LEDIndication::Failsafe);
}

void sendSerialTelemetry(ReceiverInput input)
{
    static uint32_t lastPrintMs = 0;

    if (robot.state.currentMs - lastPrintMs < 100)
    {
        return;
    }

    // Serial.printf(
    //     "IMU1[g]  X:%+8.3f  Y:%+8.3f  Z:%+8.3f | "
    //     "IMU2[g]  X:%+8.3f  Y:%+8.3f  Z:%+8.3f\n",
    //     robot.state.imu1.xG,
    //     robot.state.imu1.yG,
    //     robot.state.imu1.zG,
    //     robot.state.imu2.xG,
    //     robot.state.imu2.yG,
    //     robot.state.imu2.zG);

    Serial.printf(
        "stick:%8.3d\n",
        input.leftStickVertical);

    lastPrintMs = robot.state.currentMs;
}

#if ENABLE_DEBUG_AP 
void sendWiFiTelemetry() {

        static uint32_t lastUdpMs = 0;
        if (robot.state.currentMs - lastUdpMs > 50) {
            char payload[128];
            snprintf(payload, sizeof(payload), 
                "IMU1[g]: X=%.3f Y=%.3f Z=%.3f | IMU2[g]: X=%.3f Y=%.3f Z=%.3f",
                robot.state.imu1.xG,
                robot.state.imu1.yG,
                robot.state.imu1.zG,
                robot.state.imu2.xG,
                robot.state.imu2.yG,
                robot.state.imu2.zG);
            
            udp.beginPacket(UDP_BROADCAST_IP, UDP_PORT);
            udp.print(payload);
            udp.endPacket();
            
            lastUdpMs = robot.state.currentMs;
        }
    }
#endif

void setup() {
    Serial.begin(115200);
    delay(2000);

    #if ENABLE_DEBUG_AP
        Serial.println("Startuji Wi-Fi AP...");
        WiFi.softAP(AP_SSID, AP_PASS);
        Serial.print("AP IP adresa: ");
        Serial.println(WiFi.softAPIP());
    #endif

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

        ledHandler.setIndication(LEDIndication::HardwareError);
        
        while (true) {
            ledHandler.update();
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
    vTaskDelete(NULL);
}

// ====================================================================
// TASK IMPLEMENTATIONS
// ====================================================================

void mainThread(void *pvParameters) {
    robot.hw.leftMotor = &motorLeft;
    robot.hw.rightMotor = &motorRight;
    robot.hw.led = &ledHandler;
    robot.hw.rx = &receiver;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / TASK_CONTROL_HZ);

    // light indication sync
    modeHandler.update(robot);

    while (true) {
        // NOTE: why?
        robot.state.currentMs = millis();

        robot.state.isConnected = receiver.isConnected();

        // Capture one coherent frame so processing cannot mix channel values
        // while the receiver task publishes the next CRSF frame.
        const ReceiverChannels channels = receiver.getChannelsSnapshot();

        const ReceiverInput input =
            SignalProcessing::processReceiverInput(channels);

        if (robot.state.isConnected) {
            robot.state.requestedMode = modeHandler.decodeMode(
                input.leftSwitch,
                input.rightSwitch);
        }

        if (imu1.isAvailable()) imu1.readData(robot.state.imu1);
        if (imu2.isAvailable()) imu2.readData(robot.state.imu2);

        sendSerialTelemetry(input);

        #if ENABLE_DEBUG_AP 
            sendWiFiTelemetry();
        #endif

        modeHandler.update(robot);
    
        modeHandler.getCurrentMode()->execute(robot, input);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ====================================================================
// RECEIVER THREAD
// ====================================================================
void ReceiverThread(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_RECEIVER_MS); 

    while (true) {
        receiver.update();
        vTaskDelay(xFrequency);
    }
}

// ====================================================================
// LED THREAD
// ====================================================================
void LEDThread(void *pvParameters) {
    while (true) {
        ledHandler.update();

        // precision for meltySync mode
        if (ledHandler.isMeltySyncActive()) {
            vTaskDelay(1); 
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}
