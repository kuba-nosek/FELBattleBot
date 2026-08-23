# ESP32-C3 Modular Combat Robot Firmware

This repository contains the modular, FreeRTOS-based firmware for a differential-drive combat robot, built on the ESP32-C3 microcontroller. The system is designed for high-performance real-time execution, integrating custom libraries for DShot motor control, CRSF radio communication, and SPI accelerometer handling. 

The architecture is explicitly prepared for advanced kinetic operations, including "Melty Brain" rotational translation.

## Table of Contents
- [System Architecture](#system-architecture)
- [Custom Libraries and Modules](#custom-libraries-and-modules)
- [FreeRTOS Threading Structure](#freertos-threading-structure)
- [Safety and Failsafes](#safety-and-failsafes)

---

## System Architecture

The firmware operates on a clean **State-Driven Object-Oriented** model within a multi-threaded FreeRTOS environment. 

The core data structure is `RobotCore`, which strictly separates data from physical actions:
1.  **`RobotState` (What the robot feels/knows):** Contains continuous dynamic data such as current time (ms), receiver inputs, accelerometer data, and connection status.
2.  **`RobotHardware` (What the robot controls):** Contains pointers directly to the hardware abstraction objects (Motors, LED, Receiver).
3.  **Process & Output:** A centralized state machine (`ModeHandler`) delegates execution to the currently active drive mode (Idle, Forward, Spin). The active mode reads from `RobotState`, processes the logic, and issues commands directly to the `RobotHardware` components.

---

## Custom Libraries and Modules

Hardware peripherals and logical units are isolated into dedicated libraries (located in `lib/`), ensuring a clean separation of concerns.

### 1. `Motor` (DShot ESC Controller)
Handles bidirectional motor control using the digital DShot300 protocol via the ESP32 RMT (Remote Control) peripheral.
*   **`init()` & `arm()`**: Configures the RMT channel and transmits the 3D Mode initialization sequence.
*   **`setSpeed(speed, currentMs)`**: Converts a target speed (-1000 to 1000) into a valid DShot packet. Natively implements a **Direction Change Guard**: sudden directional reversals force the output to zero for a safety threshold (e.g., 150 ms) to prevent mechanical gearbox failure.
*   **`getSpeed()`**: Returns the actual currently applied speed (respecting the Direction Guard constraints).
*   **`stop()`**: Immediately commands zero speed.

### 2. `IMU` (H3LIS331DL Accelerometer)
Manages high-g accelerometer data acquisition over the SPI bus. Each instance stores its own calibration offsets.
*   **`init()`**: Verifies the sensor ID and writes initial configurations to the control registers.
*   **`readData(IMUData& dataOut)`**: Reads all 6 coordinate registers in a single SPI transaction, applies sensitivity multipliers, subtracts offsets, and outputs standard `g` values.
*   **`writeConfig(reg, value)`**: Allows dynamic modification of sensor parameters during runtime.

### 3. `Receiver` (CRSF Protocol)
An isolated, lock-free driver for parsing the ExpressLRS / Crossfire serial protocol via UART.
*   **`connect()` & `update(currentMs)`**: Initializes the UART at 420000 baud, reads the buffer, identifies frame boundaries, validates CRC8, and decodes RC channels/link statistics natively.
*   **`getChannel(index)`**: Safely returns the latest pulse width (988–2012 µs) for the requested channel.
*   **`sendTelemetry(text, currentMs)`**: Packages and transmits standard CRSF telemetry frames back to the radio transmitter.
*   **`onDisconnect(callback)`**: Registers a safety callback executed immediately upon signal loss.

### 4. `LEDHandler` (Status and Sync UI)
Manages the visual feedback of the robot using a priority-based indication system.
*   **`setIndication(LEDIndication)`**: Sets the persistent status of the robot (e.g., `Idle`, `Forward`, `Failsafe`).
*   **`playAnimation(LEDAnimation)`**: Triggers a high-priority visual sequence (e.g., `ModeChanged` or `ErrorAlert`) that temporarily overrides the base indication.
*   **`setMeltySync(periodUs, phaseOffsetUs, flashDurationUs)`**: Establishes microsecond-level LED timing required for Melty Brain rotational tracking, enabling direct IMU-to-LED synchronization.
*   **`isMeltySyncActive()`**: Allows the thread scheduler to adjust loop timing dynamically for maximum precision.

### 5. `ModeHandler` and `IRobotMode` (State Machine)
The logical core of the robot utilizing polymorphism to manage diverse driving behaviors without conditional clutter.
*   **`IRobotMode::execute(RobotCore& robot)`**: An abstract method implemented by specific modes (`IdleMode`, `ForwardMode`, `SpinMode`). Each mode reads inputs from `robot.state` and drives the components via `robot.hw`.
*   **`IRobotMode::init(RobotCore& robot)`**: Executed once upon entering a new mode (e.g., triggering a UI animation or resetting IMU filters).
*   **`ModeHandler::update(RobotCore& robot)`**: Safely manages state transitions and invokes the active mode's logic.

### 6. `SignalProcessing` (Application Math)
A local namespace to keep the main thread clean.
*   **`normalizeChannel(channelUs)`**: Converts raw RC values into a symmetric internal scale (-1000 to 1000) while applying a safety deadband.
*   **`decodeMode(channelUs)`**: Maps the state of the 3-position auxiliary switch to specific `DriveModeType` enumerators.

---

## FreeRTOS Threading Structure

The firmware distributes the workload across three independent tasks scheduled by the FreeRTOS kernel on the ESP32-C3:

1. **`ControlLoop` (Main Thread)**
    *   **Priority:** High (3)
    *   **Frequency:** 100 Hz (10 ms period)
    *   **Role:** Acts as the data aggregator. Reads inputs (`Receiver`, `IMU`), updates the `RobotState`, and invokes `modeHandler.update(robot)`. The entire hardware execution is delegated internally to the active mode. Uses `vTaskDelayUntil` for strict loop timing.

2. **`CRSF_RX` (Communication Thread)**
    *   **Priority:** Medium (2)
    *   **Frequency:** 500 Hz (2 ms period)
    *   **Role:** Continuously polls the UART buffer. Isolating this process prevents serial buffer overflows at 420k baud and ensures the main control loop is never blocked by communication processing.

3. **`LED_Control` (UI & Sync Thread)**
    *   **Priority:** Low (1)
    *   **Frequency:** Dynamic (50 Hz standard; 1-tick delay for Melty mode)
    *   **Role:** Evaluates animations and blinks the LED. During standard operation, it sleeps for 20 ms. When `isMeltySyncActive()` returns true, it ramps up the execution speed to allow microsecond-accurate strobe flashes based on the IMU phase offset parameters.

---

## Safety and Failsafes

Safety is strictly integrated into the boot sequence and the operational loop:

*   **Startup Verification:** During `setup()`, the system evaluates the boolean return values of all hardware `init()` routines. If critical hardware fails to initialize, the robot enters an infinite lock-state, triggering a continuous LED error sequence.
*   **Hardware Failsafe:** The CRSF receiver continuously monitors the incoming telemetry frames. Upon a timeout (signal loss), a designated callback is executed, forcing the `ModeHandler` into `IdleMode` and setting motor speeds to a strict zero.