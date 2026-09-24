# ESP32-C3 Modular Combat Robot Firmware

This repository contains the modular, FreeRTOS-based firmware for a differential-drive combat robot, built on the ESP32-C3 microcontroller. The system is designed for high-performance real-time execution, integrating custom libraries for DShot motor control, CRSF radio communication, and SPI accelerometer handling. 

The architecture is explicitly prepared for advanced kinetic operations, including "Melty Brain" rotational translation.

## Table of Contents
- [System Architecture](#system-architecture)
- [Custom Libraries (`lib/`)](#custom-libraries-lib)
- [Core Application Logic (`src/`)](#core-application-logic-src)
- [EdgeTX Telemetry](#edgetx-telemetry)
- [Visual Feedback (LED UI)](#visual-feedback-led-ui)
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

## Custom Libraries (`lib/`)

Hardware peripherals are isolated into dedicated, reusable libraries, ensuring a clean separation of concerns from the main application logic.

### 1. `Motor` (DShot ESC Controller)
Handles bidirectional motor control using the digital DShot300 protocol via the ESP32 RMT (Remote Control) peripheral.
*   **`init()` & `arm()`**: Configures the RMT channel and transmits the 3D Mode initialization sequence.
*   **`setSpeed(speed, currentMs)`**: Converts a target speed (-1000 to 1000) into a valid DShot packet. Natively implements a **Direction Change Guard**: sudden directional reversals force the output to zero for a safety threshold (e.g., 150 ms) to prevent mechanical gearbox failure.
*   **`getSpeed()`**: Returns the actual currently applied speed (respecting the Direction Guard constraints).
*   **`stop()`**: Immediately commands zero speed.

### 2. `IMU` (H3LIS331DL Accelerometer)
Manages high-g accelerometer data acquisition over the SPI bus. Each instance stores its own calibration offsets.
*   **`init()`**: Verifies the sensor ID and writes initial configurations to the control registers.
*   **`readData(IMUData& dataOut)`**: Reads all 6 coordinate registers in a single SPI transaction, applies calibration, and outputs floating-point acceleration in m/s².
*   **`writeConfig(reg, value)`**: Allows dynamic modification of sensor parameters during runtime.

### 3. `Receiver` (CRSF Protocol)
An isolated driver for parsing the ExpressLRS / Crossfire serial protocol via UART.
*   **`connect()` & `update()`**: Initializes the UART at 420000 baud, reads the buffer, identifies frame boundaries, validates CRC8, and decodes RC channels/link statistics natively.
*   **`getChannelsSnapshot()`**: Returns one coherent snapshot of all channel pulse widths (988–2012 µs) without mixing values from different CRSF frames.
*   **`sendTelemetry(text, currentMs)`**: Packages and transmits a standard CRSF flight-mode text frame.
*   **`sendBattlebotTelemetry(telemetry)`**: Sends the schema-driven private `0x80/0xF3` packet decoded by the Boxer Lua screen.
*   **`onDisconnect(callback)`**: Registers a safety callback executed immediately upon signal loss.

### 4. `LEDHandler` (Status, Flash, and POV UI)
Manages the visual feedback of the robot using a priority-based status LED and an independent WS2812B strip.
*   **`setIndication(LEDIndication)`**: Sets the persistent status of the robot (e.g., `Idle`, `Forward`, `HardwareError`).
*   **`playAnimation(LEDAnimation)`**: Triggers a high-priority visual sequence (e.g., `ModeChanged` or `InitializationFail`) that temporarily overrides the base indication.
*   **`scheduleFlash(startTimeFromNowUs, flashDurationUs, minimumTimeBetweenFlashesUs)`**: Schedules one future flash while enforcing a minimum dark interval between flashes.
*   **`cancelScheduledFlash()`**: Cancels a pending flash without shortening a flash that is already running.
*   **`setStripMode(LEDStripMode)`**: Selects static effects or angle-integrated spinning output.
*   **`setAnimation(LEDStripAnimation)`**: Selects a predefined static effect or a generated POV image.
*   **`setAngularVelocity(radiansPerSecond)`**: Supplies the last signed angular velocity used by the spinning display.

### Receiver Input Mapping

Receiver channels are declared in `include/config.h` through `RECEIVER_INPUT_MAP`.
Sticks and potentiometers are normalized to `-1000..1000`, while three-position
switches produce `-1 / 0 / 1` and six-position switches produce `0..5`. Sticks
apply the configured center deadzone; potentiometers do not. Six-position switch
values are selected by the midpoint between six equally spaced RC positions.

| Channel | Input | Use |
| :--- | :--- | :--- |
| CH1 | Horizontal sticks | Current left/right placeholder mapping |
| CH2 | Vertical sticks | Current left/right placeholder mapping |
| CH5 | Two-position switches | Current left/right placeholder mapping; left arms |
| CH6 | Potentiometers | Current left/right placeholder mapping (`-1000..1000`) |
| CH7 | Left three-position switch | Power expo selection (`1 / 3 / 5`) |
| CH8 | Right three-position switch | Mode and steering expo selection |
| CH14 | Six-position switch | Spin-mode POV selection (`Off`, test pattern, or user images 1–4) |

The processed `ReceiverInput` is passed explicitly to each drive mode's `execute`
method. Forward mode converts its signed potentiometer inputs to `0..1000` expo
amounts locally.

In Spin mode, the left vertical stick controls spin power and the left
potentiometer selects the IMU radius. The right vertical stick controls
translation amplitude. The right potentiometer sets a constant heading offset,
mapping its full range to `-π..+π`. Holding the right horizontal stick continuously
changes a separate variable heading offset at up to one revolution per second;
that offset remains fixed when the stick is released. The combined offset drives
both motor modulation and the directional LED flash.

---

## Core Application Logic (`src/`)

The application-specific logic is encapsulated in the `src/` directory, maintaining a clutter-free `main.cpp`.

### 1. `ModeHandler` and `Modes/` (State Machine)
The logical core of the robot utilizing polymorphism to manage diverse driving behaviors without conditional clutter.
*   **`IRobotMode::execute(RobotCore& robot, const ReceiverInput& input)`**: An abstract interface implemented by specific classes (`IdleMode`, `ForwardMode`, `SpinMode`) located in the `src/Modes/` directory. Each mode receives the current controls explicitly and drives the components via `robot.hw`.
*   **`IRobotMode::init(RobotCore& robot)`**: Executed once upon entering a new mode (e.g., triggering a UI animation or resetting IMU filters).
*   **`IRobotMode::sendTelemetry(const RobotCore&, TelemetryData&)`**: Populates the configured nullable telemetry fields owned by the active mode.
*   **`ModeHandler::update(RobotCore& robot)`**: Safely manages state transitions and invokes the active mode's logic.
*   **`ModeHandler::decodeMode(leftSwitch, rightSwitch)`**: Maps the processed mode switches to a `DriveModeType`.
*   **`TelemetryManager::shouldSendTelemetry(...)`**: Checks the 500 ms schedule and records immediate mode-change requests.
*   **`TelemetryManager::sendTelemetry(...)`**: Populates and transmits one packet while retaining failed sends for retry.

### 2. `SignalProcessing` (Application Math)
A local namespace dedicated to math operations specific to this robot's RC configuration.
*   **`processReceiverInput(rawInput)`**: Applies the compile-time mapping and converts all configured channels into a strongly typed `ReceiverInput`.

---

## EdgeTX Telemetry

Telemetry fields and their wire types are declared once through
`TELEMETRY_FIELD_MAP` in `include/config.h`. The current schema sends the active
mode, signed Spin-mode RPM, and the latest XYZ output from both high-g
accelerometers every 500 ms. Acceleration values are transmitted directly as
floating-point m/s². A mode change forces an immediate packet, and fields not
populated by the active mode are marked invalid.

Every PlatformIO build regenerates self-contained graph, formatted-values, and
raw-table EdgeTX screen scripts from the same field map.
It can also be generated manually with
`python3 scripts/generate_telemetry_lua.py`.

Install and model-setup instructions for the RadioMaster Boxer are in
[`edgetx/README.md`](edgetx/README.md). The generated screen scripts are
`BBGRPH.lua`, `BBVALS.lua`, and `BBRAW.lua`.

---

## Visual Feedback (LED UI)

The robot utilizes a time-modulo based visual language to communicate its current state and urgent events. The system distinguishes between persistent indications and temporary priority animations.

### Persistent Indications (Base States)
These states represent the current operating mode or health of the robot.

| State | Visual Pattern | Description |
| :--- | :--- | :--- |
| **Idle** | Heartbeat | A repeating 2000 ms cycle featuring a double-blink "heartbeat" effect. |
| **Forward** | Solid Light | Continuous solid light indicating forward drive mode. |
| **Spin** | Directional Flash | Flashes when the estimated heading reaches zero. |
| **Failsafe** | Slow Blink (2 Hz) | 250 ms ON, 250 ms OFF loop warning of RC link loss. |
| **Low Battery** | Short Pulse | A power-saving warning flash (100 ms ON, 900 ms OFF). |
| **Hardware Error** | Rapid Strobe (15 Hz) | Fast terminal error warning (30 ms ON, 30 ms OFF). |
| **Off** | Dark | LED is completely deactivated. |

### POV LED strip

The optional 14-pixel WS2812B strip is configured in `include/config.h` and uses
GPIO 21 by default. Set `USE_LED_STRIP` to `false` to leave that GPIO untouched.
The first implementation sends GRB data with software timing because both ESP32-C3
RMT TX channels are occupied by the motors. Static effects use all 14 LEDs. Spin
mode uses pixels 0–6 from the outer edge toward the rotation axis and keeps pixels
7–13 dark. Its first frame clears all 14 pixels; later frames transmit only seven,
reducing interrupt-disabled time from approximately 420 microseconds to 210
microseconds. The refresh rate is capped at 500 Hz and unchanged frames are not
retransmitted.

Source images live in `assets/led-strip/`. Each PlatformIO build runs
`python3 scripts/generate_led_strip_images.py`, which reads the LED count and
sector density from `config.h` and generates compile-time RGB arrays. The default
14 LEDs at three sectors per LED produce 42 angular columns, with seven radial
pixels in each column. A built-in test pattern is always generated, and up to four
user images are assigned alphabetically to six-position switch positions 3–6.
Missing positions display black. See `assets/led-strip/README.md` for the
standalone generation and check commands.

Each default 7×42 image uses 1,176 bytes of flash for its 32-bit RGB values,
plus a small amount of generated selection code. Images are selected at runtime
but are changed by regenerating and reflashing the firmware.

The strip needs a separate adequate 5 V supply, common ground, and suitable
3.3 V-to-5 V data-level conditioning. A future FastLED clockless-SPI version may
reduce interrupt blocking, but it must coordinate with the two 50 Hz IMUs sharing
the SPI peripheral.

### Priority Animations (Events)
These sequences temporarily override the base indication to alert the user of specific events.

| Animation | Visual Pattern | Duration |
| :--- | :--- | :--- |
| **Bootup** | 4 Hz Blinking | 2000 ms sequence during initialization. |
| **Mode Changed** | 10 Hz Fast Blink | 500 ms visual confirmation of switch toggling. |
| **Error Alert** | 5 Hz Rapid Blink | 3000 ms sequence for non-terminal faults. |
| **Telemetry Sent** | Single Flash | A brief 100 ms override displaying a 50 ms flash. |

---

## FreeRTOS Threading Structure

The firmware distributes the workload across three independent tasks scheduled by the FreeRTOS kernel on the ESP32-C3:

1. **`ControlLoop` (Main Thread)**
    *   **Priority:** High (3)
    *   **Frequency:** 200 Hz (5 ms period)
    *   **Role:** Reads inputs (`Receiver`, `IMU`), updates `RobotState`, invokes the active mode, and schedules schema-driven telemetry. Uses `vTaskDelayUntil` for strict loop timing.

2. **`CRSF_RX` (Communication Thread)**
    *   **Priority:** Medium (2)
    *   **Frequency:** 500 Hz (2 ms period)
    *   **Role:** Continuously polls the UART buffer. Isolating this process prevents serial buffer overflows at 420k baud and ensures the main control loop is never blocked by communication processing.

3. **`LED_Control` (UI Thread)**
    *   **Priority:** Low (1)
    *   **Frequency:** One scheduler tick between updates.
    *   **Role:** Evaluates animations, status indications, and one-shot directional flashes requested by Spin mode.

---

## Safety and Failsafes

Safety is strictly integrated into the boot sequence and the operational loop:

*   **Startup Verification:** During `setup()`, the system evaluates the boolean return values of all hardware `init()` routines. If critical hardware fails to initialize, the robot enters an infinite lock-state, triggering a continuous LED error sequence (`HardwareError`).
*   **Hardware Failsafe:** The CRSF receiver continuously monitors the incoming telemetry frames. Upon a timeout (signal loss), a designated callback is executed, forcing the `ModeHandler` into `IdleMode` and setting motor speeds to a strict zero.
