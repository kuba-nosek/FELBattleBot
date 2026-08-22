TODO - ai for now

# ESP32-C3 Modular Combat Robot Firmware

This repository contains the modular, FreeRTOS-based firmware for a differential-drive combat robot, built on the ESP32-C3 microcontroller. The system is designed for high-performance real-time execution, integrating custom libraries for DShot motor control, CRSF radio communication, and SPI accelerometer handling. 

The architecture is also prepared for advanced kinetic operations, including "Melty Brain" rotational translation.

## Table of Contents
- [System Architecture](#system-architecture)
- [Custom Libraries and Modules](#custom-libraries-and-modules)
- [FreeRTOS Threading Structure](#freertos-threading-structure)
- [Safety and Failsafes](#safety-and-failsafes)

---

## System Architecture

The firmware operates on a strict **Input-Process-Output (IPO)** model driven by a multi-threaded FreeRTOS environment. 

1. **Input:** Data is gathered from the CRSF receiver (RC commands, link quality) and the dual H3LIS331DL accelerometers via the SPI bus.
2. **Process:** A centralized state machine (`ModeHandler`) evaluates the active drive mode (Idle, Forward, Spin). The active mode reads the inputs from a shared `RobotContext` structure, executes control algorithms, and writes the desired motor speeds and LED states back into the context.
3. **Output:** The computed values are dispatched to the ESCs via the DShot300 protocol, and the visual UI (LEDs) is updated.

---

## Custom Libraries and Modules

Hardware peripherals and logical units are isolated into dedicated libraries (located in `lib/`), ensuring a clean separation of concerns.

### 1. `Motor` (DShot ESC Controller)
Handles bidirectional motor control using the digital DShot300 protocol via the ESP32 RMT (Remote Control) peripheral.
*   **`init()`**: Configures the RMT channel and GPIO pin.
*   **`arm()`**: Transmits the initialization sequence and the 3D Mode command to arm the ESC for bidirectional operation.
*   **`setSpeed(speed, currentMs)`**: Converts a target speed (-1000 to 1000) into a valid DShot packet (including CRC and telemetry bit). Implements a **Direction Change Guard**: sudden directional reversals force the output to zero for a safety threshold (e.g., 150 ms) to prevent mechanical gearbox failure.
*   **`stop()`**: Immediately commands zero speed.

### 2. `IMU` (H3LIS331DL Accelerometer)
Manages high-g accelerometer data acquisition over the SPI bus. Each instance stores its own calibration offsets.
*   **`init()`**: Verifies the sensor ID (`WHO_AM_I`) and writes initial configurations to the control registers (data rate, block data update, active axes).
*   **`readData(IMUData& dataOut)`**: Reads all 6 coordinate registers in a single SPI transaction, applies the configured sensitivity multiplier (e.g., for ±100g), subtracts calibration offsets, and outputs standard `g` values.
*   **`writeConfig(reg, value)`**: Allows dynamic modification of sensor parameters (e.g., changing the measurement scale) during runtime.

### 3. `Receiver` (CRSF Protocol)
An isolated driver for parsing the ExpressLRS / Crossfire serial protocol via UART.
*   **`connect()`**: Initializes the hardware serial port at 420000 baud.
*   **`update(currentMs)`**: Reads the serial buffer, identifies frame boundaries, validates the CRC8 checksum, and decodes RC channels and link statistics.
*   **`getChannel(index)`**: Returns the latest raw pulse width (988–2012 µs) for the requested channel.
*   **`onDisconnect(callback)`**: Registers a safety callback executed immediately upon signal loss.

### 4. `LEDHandler` (Status and Sync UI)
Manages the visual feedback of the robot using a priority-based indication system.
*   **`setIndication(LEDIndication)`**: Sets the persistent status of the robot (e.g., `Idle`, `Forward`, `Failsafe`).
*   **`playAnimation(LEDAnimation)`**: Triggers a high-priority visual sequence (e.g., `ModeChanged` or `ErrorAlert`) that temporarily overrides the base indication.

### 5. `ModeHandler` and `IRobotMode` (State Machine)
The logical core of the robot. It utilizes polymorphism to manage driving behaviors.
*   **`RobotContext`**: A data struct containing all inputs (throttle, steering, IMU data, connection status) and outputs (motor speeds, LED state). Passed by reference to avoid copying overhead.
*   **`IRobotMode::calculateResponse(RobotContext& ctx)`**: An abstract method implemented by specific modes (`IdleMode`, `ForwardMode`, `SpinMode`). Each mode reads the inputs, performs specific math, and writes directly back to the context.
*   **`ModeHandler::setMode(DriveModeType)`**: Safely transitions between states and triggers the `init()` method of the newly selected mode.

### 6. `SignalProcessing` (Application Math)
A local namespace located in `src/` to keep the main thread clean.
*   **`normalizeChannel(channelUs)`**: Converts raw RC values into a symmetric internal scale (-1000 to 1000) while applying a safety deadband.
*   **`decodeMode(channelUs)`**: Maps the state of the 3-position auxiliary switch to specific `DriveModeType` enumerators.

---

## FreeRTOS Threading Structure

The firmware distributes the workload across three independent tasks scheduled by the FreeRTOS kernel on the ESP32-C3:

1. **`ControlLoop` (Main Thread)**
    *   **Priority:** High (3)
    *   **Frequency:** 100 Hz (10 ms period)
    *   **Role:** Reads `Receiver` channels and `IMU` values, populates the `RobotContext`, invokes the active mode's `calculateResponse(ctx)`, and applies the resulting speeds to the `Motor` objects. Uses `vTaskDelayUntil` for strict loop timing.

2. **`CRSF_RX` (Communication Thread)**
    *   **Priority:** Medium (2)
    *   **Frequency:** 500 Hz (2 ms period)
    *   **Role:** Continuously polls the UART buffer to parse incoming CRSF packets and maintain the link state. Isolating this prevents serial buffer overflows and ensures the control loop is never blocked by communication overhead.

3. **`LED_Control` (UI & Sync Thread)**
    *   **Priority:** Low (1)
    *   **Frequency:** Dynamic (50 Hz standard, yielding for Melty mode)
    *   **Role:** Evaluates active animations and indications. During standard operation, it updates every 20 ms. In Melty Brain mode, the delay yields to allow microsecond-accurate strobe flashes based on the IMU rotational data.

---

## Safety and Failsafes

Safety is strictly integrated into the boot sequence and the operational loop:

*   **Startup Verification:** During `setup()`, the system evaluates the boolean return values of all hardware `init()` routines (Motors and IMUs). If critical hardware fails to initialize, the robot enters an infinite lock-state, keeping motors completely unpowered and triggering a continuous LED error sequence.
*   **Hardware Failsafe:** The CRSF receiver continuously monitors the incoming telemetry frames. Upon a timeout (signal loss), a designated callback is executed, forcing the `ModeHandler` into `IdleMode` and setting motor speeds to a strict zero.