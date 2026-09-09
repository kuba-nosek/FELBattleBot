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
#include "BattlebotTelemetry.h"

#include <cmath>

#if ENABLE_DEBUG_AP
    #include <WiFi.h>
    #include <WiFiUdp.h>
    WiFiUDP udp;
#endif

using namespace RobotConfig;

// --- Global hardware instances ---
Motor motorLeft(PIN_MOTOR_L, RMT_CHANNEL_0, MOTOR_LEFT_REVERSED);
Motor motorRight(PIN_MOTOR_R, RMT_CHANNEL_1, MOTOR_RIGHT_REVERSED);

IMU imu1(PIN_SPI_CS1, IMU1_OFFSET_X_G, IMU1_OFFSET_Y_G, IMU1_OFFSET_Z_G);
IMU imu2(PIN_SPI_CS2, IMU2_OFFSET_X_G, IMU2_OFFSET_Y_G, IMU2_OFFSET_Z_G);

Receiver receiver(PIN_CRSF_RX, PIN_CRSF_TX);
LEDHandler ledHandler(PIN_LED);
ModeHandler modeHandler;

RobotCore robot;

namespace {

struct AccelerationTelemetryAccumulator {
    float sums[BattlebotTelemetry::SENSOR_COUNT]
              [BattlebotTelemetry::AXIS_COUNT]{};
    float peakMagnitudeG[BattlebotTelemetry::SENSOR_COUNT]{};
    uint16_t sampleCount[BattlebotTelemetry::SENSOR_COUNT]{};

    void reset()
    {
        for (size_t sensor = 0;
             sensor < BattlebotTelemetry::SENSOR_COUNT;
             ++sensor) {
            sampleCount[sensor] = 0;
            peakMagnitudeG[sensor] = 0.0f;
            for (size_t axis = 0;
                 axis < BattlebotTelemetry::AXIS_COUNT;
                 ++axis) {
                sums[sensor][axis] = 0.0f;
            }
        }
    }

    void add(size_t sensor, const IMUData& data)
    {
        if (sensor >= BattlebotTelemetry::SENSOR_COUNT ||
            sampleCount[sensor] == UINT16_MAX) {
            return;
        }

        sums[sensor][0] += data.xG;
        sums[sensor][1] += data.yG;
        sums[sensor][2] += data.zG;
        ++sampleCount[sensor];

        const float magnitude = std::sqrt(
            data.xG * data.xG +
            data.yG * data.yG +
            data.zG * data.zG);
        if (std::isfinite(magnitude) &&
            magnitude > peakMagnitudeG[sensor]) {
            peakMagnitudeG[sensor] = magnitude;
        }
    }
};

AccelerationTelemetryAccumulator telemetryAccumulator;

void setAccelerationTelemetry(
    BattlebotTelemetry::Snapshot& snapshot,
    size_t sensor,
    const bool enabled[BattlebotTelemetry::AXIS_COUNT],
    const uint8_t validBits[BattlebotTelemetry::AXIS_COUNT])
{
    const uint16_t count = telemetryAccumulator.sampleCount[sensor];
    if (count == 0) {
        return;
    }

    for (size_t axis = 0;
         axis < BattlebotTelemetry::AXIS_COUNT;
         ++axis) {
        if (!enabled[axis]) {
            continue;
        }
        snapshot.accelerationCentiG[sensor][axis] =
            BattlebotTelemetry::encodeAccelerationCentiG(
                telemetryAccumulator.sums[sensor][axis] /
                static_cast<float>(count));
        snapshot.validMask |= validBits[axis];
    }

    snapshot.peakMagnitudeCentiG[sensor] =
        BattlebotTelemetry::encodePeakMagnitudeCentiG(
            telemetryAccumulator.peakMagnitudeG[sensor]);
}

void sendBattlebotTelemetry()
{
    if (!TELEMETRY_ENABLED || !robot.state.isConnected) {
        telemetryAccumulator.reset();
        return;
    }

    BattlebotTelemetry::Snapshot snapshot{};
    if (TELEMETRY_SEND_RPM && robot.state.rpmValid) {
        snapshot.rpm = BattlebotTelemetry::encodeRpm(robot.state.rpm);
        snapshot.validMask |= BattlebotTelemetry::VALID_RPM;
    }

    const bool imu1Enabled[BattlebotTelemetry::AXIS_COUNT] = {
        TELEMETRY_SEND_ACCEL1_X,
        TELEMETRY_SEND_ACCEL1_Y,
        TELEMETRY_SEND_ACCEL1_Z,
    };
    const bool imu2Enabled[BattlebotTelemetry::AXIS_COUNT] = {
        TELEMETRY_SEND_ACCEL2_X,
        TELEMETRY_SEND_ACCEL2_Y,
        TELEMETRY_SEND_ACCEL2_Z,
    };
    const uint8_t imu1ValidBits[BattlebotTelemetry::AXIS_COUNT] = {
        BattlebotTelemetry::VALID_ACCEL1_X,
        BattlebotTelemetry::VALID_ACCEL1_Y,
        BattlebotTelemetry::VALID_ACCEL1_Z,
    };
    const uint8_t imu2ValidBits[BattlebotTelemetry::AXIS_COUNT] = {
        BattlebotTelemetry::VALID_ACCEL2_X,
        BattlebotTelemetry::VALID_ACCEL2_Y,
        BattlebotTelemetry::VALID_ACCEL2_Z,
    };

    setAccelerationTelemetry(snapshot, 0, imu1Enabled, imu1ValidBits);
    setAccelerationTelemetry(snapshot, 1, imu2Enabled, imu2ValidBits);

    if (receiver.sendBattlebotTelemetry(
            snapshot,
            robot.state.currentMs,
            TELEMETRY_INTERVAL_MS)) {
        telemetryAccumulator.reset();
    }
}

}  // namespace

// --- FreeRTOS task prototypes ---
void mainThread(void *pvParameters);
void ReceiverThread(void *pvParameters);
void LEDThread(void *pvParameters);

// --- Safety failsafe callback ---
void onFailsafe() {
  robot.state.requestedMode = DriveModeType::Idle;
  robot.hw.led->setIndication(LEDIndication::Failsafe);
}

void sendSerialTelemetry(const ReceiverInput& input)
{
    static uint32_t lastPrintMs = 0;

    if (robot.state.currentMs - lastPrintMs < 100)
    {
        return;
    }

    // Serial.printf(
    //     "IMU1[m/s^2]  X:%+8.3f  Y:%+8.3f  Z:%+8.3f | "
    //     "IMU2[m/s^2]  X:%+8.3f  Y:%+8.3f  Z:%+8.3f\n",
    //     robot.state.imu1.xMps2,
    //     robot.state.imu1.yMps2,
    //     robot.state.imu1.zMps2,
    //     robot.state.imu2.xMps2,
    //     robot.state.imu2.yMps2,
    //     robot.state.imu2.zMps2);

    // Serial.print("leftVertical: ");
    // Serial.print(input.leftStickVertical);
    // Serial.print("  leftHorizontal: ");
    // Serial.print(input.leftStickHorizontal);
    // Serial.print("  rightVertical: ");
    // Serial.print(input.rightStickVertical);
    // Serial.print("  rightHorizontal: ");
    // Serial.println(input.rightStickHorizontal);

    lastPrintMs = robot.state.currentMs;
}

#if ENABLE_DEBUG_AP 
void sendWiFiTelemetry() {

        static uint32_t lastUdpMs = 0;
        if (robot.state.currentMs - lastUdpMs > 50) {
            char payload[128];
            snprintf(payload, sizeof(payload), 
                "IMU1[m/s^2]: X=%.3f Y=%.3f Z=%.3f | IMU2[m/s^2]: X=%.3f Y=%.3f Z=%.3f",
                robot.state.imu1.xMps2,
                robot.state.imu1.yMps2,
                robot.state.imu1.zMps2,
                robot.state.imu2.xMps2,
                robot.state.imu2.yMps2,
                robot.state.imu2.zMps2);
            
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

        if (imu1.isAvailable() && imu1.readData(robot.state.imu1)) {
            telemetryAccumulator.add(0, robot.state.imu1);
        }
        if (imu2.isAvailable() && imu2.readData(robot.state.imu2)) {
            telemetryAccumulator.add(1, robot.state.imu2);
        }

        // sendSerialTelemetry(input);

        #if ENABLE_DEBUG_AP 
            sendWiFiTelemetry();
        #endif

        modeHandler.update(robot);

        // Only SpinMode makes the calculated RPM valid for this cycle.
        robot.state.rpmValid = false;
        modeHandler.getCurrentMode()->execute(robot, input);

        sendBattlebotTelemetry();

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
        vTaskDelay(1);
    }
}
