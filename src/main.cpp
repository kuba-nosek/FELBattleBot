#include <Arduino.h>
#include <SPI.h>

// Vložení vlastních knihoven
#include "config.h"
#include "Motor.h"
#include "IMU.h"
#include "Receiver.h"
#include "LEDHandler.h"
#include "ModeHandler.h"

#include "SignalProcessing.h"

using namespace RobotConfig;

// --- Inicializace globálních objektů ---
Motor motorLeft(PIN_MOTOR_L, RMT_CHANNEL_0, false);
Motor motorRight(PIN_MOTOR_R, RMT_CHANNEL_1, true);

IMU imu1(PIN_SPI_CS1, IMU1_OFFSET_X, IMU1_OFFSET_Y, IMU1_OFFSET_Z);
IMU imu2(PIN_SPI_CS2, IMU2_OFFSET_X, IMU2_OFFSET_Y, IMU2_OFFSET_Z);

Receiver receiver(PIN_CRSF_RX, PIN_CRSF_TX);
LEDHandler ledHandler(PIN_LED);
ModeHandler modeHandler;

// --- Prototypy úloh FreeRTOS ---
void mainThread(void *pvParameters);
void ReceiverThread(void *pvParameters);
void LEDThread(void *pvParameters);

// --- Bezpečnostní callback ---
void onFailsafe() {
    ledHandler.playAnimation(LEDAnimation::ErrorAlert);
    modeHandler.setMode(DriveModeType::Idle);
    motorLeft.stop();
    motorRight.stop();
}

void setup() {
    Serial.begin(115200);

    // Inicializace SPI pro IMU
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));

    // Inicializace základního hardwaru
    ledHandler.init();
    receiver.connect();
    receiver.onDisconnect(onFailsafe);

    // --- KONTROLA INICIALIZACE ---
    bool hardwareOk = true;

    // Pokud selže inicializace motorů, hardwareOk bude false
    hardwareOk &= motorLeft.init();
    hardwareOk &= motorRight.init();

    // Pokusíme se inicializovat senzory
    bool imu1Ok = imu1.init();
    bool imu2Ok = imu2.init();
    
    hardwareOk &= (imu1Ok && imu2Ok);

    if (!hardwareOk) {
        Serial.println("KRITICKÁ CHYBA: Hardware se nepodařilo inicializovat!");
        
        while (true) {
            // Rozblikáme LED jako indikaci hardwarové chyby (např. 100ms pulzy)
            ledHandler.playAnimation(LEDAnimation::ErrorAlert);
            ledHandler.update(millis());
            delay(10);
        }
    }

    // Pokud je vše v pořádku, můžeme motory odjistit
    motorLeft.arm();
    motorRight.arm();

    // Vytvoření RTOS vláken
    xTaskCreate(mainThread, "ControlLoop", 4096, NULL, 3, NULL);
    xTaskCreate(ReceiverThread, "CRSF_RX", 4096, NULL, 2, NULL);
    xTaskCreate(LEDThread, "LED_Control", 2048, NULL, 1, NULL);
}

void loop() {
    // Smazání výchozí úlohy, uvolnění paměti.
    vTaskDelete(NULL);
}

// ====================================================================
// IMPLEMENTACE VLÁKEN
// ====================================================================

void mainThread(void *pvParameters) {
  RobotContext ctx;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / TASK_CONTROL_HZ);

  while (true) {
    uint32_t currentMs = millis();

    // Načtení dat z přijímače
    ctx.isConnected = receiver.isConnected();
    if (ctx.isConnected) {
      ctx.throttle = SignalProcessing::normalizeChannel(receiver.getChannel(1));
      ctx.steering = SignalProcessing::normalizeChannel(receiver.getChannel(0));
      ctx.requestedMode = SignalProcessing::decodeMode(receiver.getChannel(7));
    } else {
      // Vynucení idle módu při ztrátě signálu
      ctx.requestedMode = DriveModeType::Idle;
    }

    // Načtení dat z akcelerometrů
    if (imu1.isAvailable()) {
      imu1.readData(ctx.imu1);
    }
    if (imu2.isAvailable()) {
      imu2.readData(ctx.imu2);
    }

    // 2. KONTROLA ZMĚNY MÓDU
    if (ctx.requestedMode != modeHandler.getCurrentModeType()) {
      modeHandler.setMode(ctx.requestedMode);
      ledHandler.playAnimation(LEDAnimation::ModeChanged);
    }

    // 3. VÝPOČET ODEZVY
    // Přesně podle vašeho návrhu: zavolání metody aktuálního módu s předáním struktury
    modeHandler.getCurrentMode()->calculateResponse(ctx);

    // 4. ZÁPIS DO HARDWARU
    // Zápis rychlostí do motorů
    motorLeft.setSpeed(ctx.leftMotorSpeed, currentMs);
    motorRight.setSpeed(ctx.rightMotorSpeed, currentMs);
    
    // Zápis požadovaného stavu do LED handleru
    // (Případné animace řeší LEDThread automaticky přes tento nastavený základní stav)
    ledHandler.setIndication(ctx.ledIndication);

    // Opakování ve fixním intervalu
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