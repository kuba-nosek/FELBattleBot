# ESP32-S3 Modular Combat Robot Firmware

This repository contains the modular, FreeRTOS-based firmware for a differential-drive combat robot, built on the dual-core **ESP32-S3 WROOM-1** microcontroller. The system is designed for high-performance real-time execution, integrating custom libraries for DShot motor control, CRSF radio communication, and SPI accelerometer handling. 

The architecture is explicitly prepared for advanced kinetic operations, including "Melty Brain" rotational translation.

## Table of Contents
- [System Architecture](#system-architecture)
- [Thread Safety & State Management](#thread-safety--state-management)
- [Custom Libraries (`lib/`)](#custom-libraries-lib)
- [Core Application Logic (`src/`)](#core-application-logic-src)
- [EdgeTX Telemetry](#edgetx-telemetry)
- [Visual Feedback (LED UI & POV)](#visual-feedback-led-ui--pov)
- [FreeRTOS Threading Structure](#freertos-threading-structure)
- [Safety and Failsafes](#safety-and-failsafes)

---

## System Architecture

The firmware operates on a clean **State-Driven Object-Oriented** model within a multi-threaded FreeRTOS environment. 

The core data structure is `RobotCore`, which strictly separates data from physical actions:
1.  **`RobotState` (What the robot feels/knows):** Contains continuous dynamic data such as current time (ms), receiver inputs, accelerometer data, and connection status. This structure is strictly thread-safe.
2.  **`RobotHardware` (What the robot controls):** Contains pointers directly to the hardware abstraction objects (Motors, LED, Receiver).
3.  **Process & Output:** A centralized state machine (`ModeHandler`) delegates execution to the currently active drive mode (Idle, Forward, Spin). The active mode reads from `RobotState`, processes the logic, and issues commands directly to the `RobotHardware` components.

---

## Thread Safety & State Management

Building a highly modular framework requires strict guarantees that shared data can be accessed concurrently across multiple CPU cores without data races or tearing. 

### The `ThreadSafe<T>` Template
To ensure data integrity without cluttering application logic with manual mutex locks, the `RobotState` utilizes a custom `ThreadSafe<T>` template wrapper. This wrapper provides:
*   **Automatic Spinlocks:** Each variable within the state is independently protected by a hardware-level `portMUX_TYPE` spinlock.
*   **Operator Overloading:** The template overloads the assignment (`=`) and casting operators, making the wrapped variables behave like standard C++ types in the application code.
*   **Atomic Transactions:** Complex structures (like `AccelData`) are written and read in a single locked transaction, preventing the extraction of torn or inconsistent states.

When developing new libraries or extending the `RobotState`, developers must ensure that any new shared properties are wrapped in `ThreadSafe<T>`. However, inside the `main.cpp` or specific Drive Modes, developers interact with these variables naturally, as the locking mechanism is entirely abstracted away.

---

## Custom Libraries (`lib/`)

Hardware peripherals are isolated into dedicated, reusable libraries, ensuring a clean separation of concerns from the main application logic.

### 1. `Motor` (DShot ESC Controller)
Handles bidirectional motor control using the digital DShot protocol via the ESP32 RMT (Remote Control) peripheral.
*   **`init()` & `arm()`**: Configures the RMT channel and transmits the 3D Mode initialization sequence.
*   **`setSpeed(speed, currentMs)`**: Converts a target speed into a valid DShot packet. Natively implements a **Direction Change Guard**: sudden directional reversals force the output to zero for a safety threshold to prevent mechanical gearbox failure.

### 2. `IMU` (H3LIS331DL Accelerometer)
Manages high-g accelerometer data acquisition over the SPI bus. Each instance stores its own calibration offsets.
*   **`init()`**: Verifies the sensor ID and writes initial configurations to the control registers.
*   **`readData(IMUData& dataOut)`**: Reads all 6 coordinate registers in a single SPI transaction, applies calibration, and outputs floating-point acceleration in m/s².

### 3. `Receiver` (CRSF Protocol)
An isolated driver for parsing the ExpressLRS / Crossfire serial protocol via UART.
*   **`connect()` & `update()`**: Initializes the UART, reads the buffer, identifies frame boundaries, validates CRC8, and decodes RC channels/link statistics natively.
*   **`getChannelsSnapshot()`**: Returns one coherent, thread-safe snapshot of all channel pulse widths (988–2012 µs) without mixing values from different CRSF frames.
*   **`onDisconnect(callback)`**: Registers a safety callback executed immediately upon signal loss.

### 4. `LEDHandler` (Status, Flash, and POV UI)
Manages the visual feedback of the robot using a priority-based status LED and an independent WS2812B strip.
*   **`setIndication(LEDIndication)`**: Sets the persistent status of the robot (e.g., `Idle`, `Forward`, `HardwareError`).
*   **`setStripMode(LEDStripMode)`**: Selects static effects or angle-integrated spinning output.
*   **`setAngularVelocity(radiansPerSecond)`**: Supplies the last signed angular velocity used by the spinning display.

### Receiver Input Mapping

Receiver channels are declared in `include/config.h` through `RECEIVER_INPUT_MAP`[span_0](start_span)[span_0](end_span).
Sticks and potentiometers are normalized to `-1000..1000`, while three-position
switches produce `-1 / 0 / 1` and six-position switches produce `0..5`[span_1](start_span)[span_1](end_span). Sticks
apply the configured center deadzone; potentiometers do not[span_2](start_span)[span_2](end_span). Six-position switch
values are selected by the midpoint between six equally spaced RC positions[span_3](start_span)[span_3](end_span).

| Channel | Input | Use |
| :--- | :--- | :--- |
| CH1 | Horizontal sticks | Current left/right placeholder mapping[span_4](start_span)[span_4](end_span) |
| CH2 | Vertical sticks | Current left/right placeholder mapping[span_5](start_span)[span_5](end_span) |
| CH5 | Two-position switches | Current left/right placeholder mapping; left arms[span_6](start_span)[span_6](end_span) |
| CH6 | Potentiometers | Current left/right placeholder mapping (`-1000..1000`)[span_7](start_span)[span_7](end_span) |
| CH7 | Left three-position switch | Power expo selection (`1 / 3 / 5`)[span_8](start_span)[span_8](end_span) |
| CH8 | Right three-position switch | Mode and steering expo selection[span_9](start_span)[span_9](end_span) |
| CH14 | Six-position switch | Spin-mode POV selection (`Off`, test pattern, or user images 1–4)[span_10](start_span)[span_10](end_span) |

The processed `ReceiverInput` is passed explicitly to each drive mode's `execute`
method[span_11](start_span)[span_11](end_span). Forward mode converts its signed potentiometer inputs to `0..1000` expo
amounts locally[span_12](start_span)[span_12](end_span).

In Spin mode, the left vertical stick controls spin power and the left
potentiometer selects the IMU radius[span_13](start_span)[span_13](end_span). The right vertical stick controls
translation amplitude[span_14](start_span)[span_14](end_span). The right potentiometer sets a constant heading offset,
mapping its full range to `-π..+π`[span_15](start_span)[span_15](end_span). Holding the right horizontal stick continuously
changes a separate variable heading offset at up to one revolution per second;
that offset remains fixed when the stick is released[span_16](start_span)[span_16](end_span). The combined offset drives
both motor modulation and the directional LED flash[span_17](start_span)[span_17](end_span).

---

## Core Application Logic (`src/`)

The application-specific logic is encapsulated in the `src/` directory, maintaining a clutter-free `main.cpp`.

### 1. `ModeHandler` and `Modes/` (State Machine)
The logical core of the robot utilizing polymorphism to manage diverse driving behaviors without conditional clutter.
*   **`IRobotMode::execute(RobotCore& robot, const ReceiverInput& input)`**: An abstract interface implemented by specific classes (`IdleMode`, `ForwardMode`, `SpinMode`) located in the `src/Modes/` directory. Each mode receives the current controls explicitly and drives the components via `robot.hw`.
*   **`ModeHandler::update(RobotCore& robot)`**: Safely manages state transitions and invokes the active mode's logic.
*   **`TelemetryManager::shouldSendTelemetry(...)`**: Checks the 500 ms schedule and records immediate mode-change requests.

### 2. `SignalProcessing` (Application Math)
A local namespace dedicated to math operations specific to this robot's RC configuration.
*   **`processReceiverInput(rawInput)`**: Applies the compile-time mapping and converts all configured channels into a strongly typed `ReceiverInput`.

---

## EdgeTX Telemetry

Telemetry fields and their wire types are declared once through
`TELEMETRY_FIELD_MAP` in `include/config.h`[span_18](start_span)[span_18](end_span). The current schema sends the active
mode, signed Spin-mode RPM, and the latest XYZ output from both high-g
accelerometers every 500 ms[span_19](start_span)[span_19](end_span). Acceleration values are transmitted directly as
floating-point m/s²[span_20](start_span)[span_20](end_span). A mode change forces an immediate packet, and fields not
populated by the active mode are marked invalid[span_21](start_span)[span_21](end_span).

Every PlatformIO build regenerates self-contained graph, formatted-values, and
raw-table EdgeTX screen scripts from the same field map[span_22](start_span)[span_22](end_span).
It can also be generated manually with
`python3 scripts/generate_telemetry_lua.py`[span_23](start_span)[span_23](end_span).

Install and model-setup instructions for the RadioMaster Boxer are in
[`edgetx/README.md`](edgetx/README.md)[span_24](start_span)[span_24](end_span). The generated screen scripts are
`BBGRPH.lua`, `BBVALS.lua`, and `BBRAW.lua`[span_25](start_span)[span_25](end_span).

---

## Logging systen - 5-second RAM -> LittleFS 

### Record
- Lower CH7 right3StateSwitch, then switch to HIGH (+1). The rising edge starts a 5-second recording.
- Logging samples are written to RAM at each existing 200 Hz control iteration; IMU hardware ODR is still 50 Hz (repeats are expected).
- After 5 s, recording stops but motors continue operating normally until the operator disarms.
- Once the robot reaches Idle, it stays disarmed until the buffered sample data is saved to `/capture.csv`.
- New recording replaces the previous capture only once the new CSV is completed.

### Download
Disconnect power from motors or otherwise make the robot safe first. With the robot disarmed and USB attached:

`python3 -m pip install pyserial`

`python3 scripts/download_log.py /dev/ttyACM0 capture.csv`

Other serial commands (115200 baud, newline terminated): `STATUS`, `DUMP`, `FORMAT`.
`FORMAT` erases logs; do not send unless initializing or repairing the filesystem.

### Select fields
Edit only `src/Logging/LogFields.h`, adding/removing `X(name, expression)` lines in the `BB_LOG_FIELDS` macro. Existing `t_us` (time since trigger in microseconds) is always present. File's first line is the CSV header, automatically updated when changing fields.

Default fields: six **post-driver-offset, m/s^2** acceleration channels (not untouched sensor register counts). RC/ESC fields can be enabled using examples in LogFields.h. ESP32-S3 PSRAM is preferred, internal RAM fallback.

### Limitations
- One saved capture `/capture.csv` at a time; next capture overwrites it after successful save. Download before next run if you want to keep multiple logs.
- `LittleFS.begin(false)` does not silently erase data on mount failure; explicit FORMAT is required for a blank filesystem.
- Flash writes and USB dumping are only allowed after entering Idle, and the robot cannot re-arm until the save finishes. Flash operations can stall real-time code even from another core.
- Capture timestamps are taken once per control iteration after existing accelerometer reads; they are not accelerometer DRDY timestamps, and do not prove both sensors were sampled simultaneously.
- If supply power is cst before the robot disarms and finishes saving, this capture (in RAM) is lost.
- Manual tests and on-target PlatformIO compilation still required.

## Visual Feedback (LED UI & POV)

The robot utilizes a time-modulo based visual language to communicate its current state and urgent events, coupled with an advanced POV display.

### Persistent Indications (Base States)
These states represent the current operating mode or health of the robot.

| State | Visual Pattern | Description |
| :--- | :--- | :--- |
| **Idle** | Heartbeat | A repeating 2000 ms cycle featuring a double-blink "heartbeat" effect[span_26](start_span)[span_26](end_span). |
| **Forward** | Solid Light | Continuous solid light indicating forward drive mode[span_27](start_span)[span_27](end_span). |
| **Spin** | Directional Flash | Flashes when the estimated heading reaches zero[span_28](start_span)[span_28](end_span). |
| **Failsafe** | Slow Blink (2 Hz) | 250 ms ON, 250 ms OFF loop warning of RC link loss[span_29](start_span)[span_29](end_span). |
| **Hardware Error** | Rapid Strobe (15 Hz) | Fast terminal error warning (30 ms ON, 30 ms OFF)[span_30](start_span)[span_30](end_span). |

### POV LED Strip (Hardware RMT)

The optional 14-pixel WS2812B strip is configured in `include/config.h`[span_31](start_span)[span_31](end_span). Thanks to the ESP32-S3 architecture, the strip is driven entirely by the **hardware RMT peripheral**, utilizing DMA transfers. This removes the need for CPU-blocking bit-banging, freeing the processor for high-frequency sensor fusion and motor control.

Spin mode uses pixels 0–6 from the outer edge toward the rotation axis and keeps pixels 7–13 dark[span_32](start_span)[span_32](end_span). Its first frame clears all 14 pixels; later frames transmit only seven[span_33](start_span)[span_33](end_span). The refresh rate is capped at 500 Hz and unchanged frames are not retransmitted[span_34](start_span)[span_34](end_span).

Source images live in `assets/led-strip/`[span_35](start_span)[span_35](end_span). Each PlatformIO build runs `python3 scripts/generate_led_strip_images.py`, generating compile-time RGB arrays for the POV sectors[span_36](start_span)[span_36](end_span). A built-in test pattern is always generated, and up to four user images are assigned alphabetically to six-position switch positions 3–6[span_37](start_span)[span_37](end_span).

---

## FreeRTOS Threading Structure

The firmware distributes the workload across three independent tasks, heavily utilizing the asymmetric dual-core architecture of the **ESP32-S3**:

1. **`ControlLoop` (Core 0)**
    *   **Priority:** High (3)
    *   **Frequency:** 200 Hz (5 ms period)
    *   **Role:** Reads inputs (`Receiver`, `IMU`), updates the thread-safe `RobotState`, invokes the active mode, and generates motor output. It is pinned to Core 0 (PRO_CPU) alongside the Wi-Fi stack.

2. **`CRSF_RX` (Core 0)**
    *   **Priority:** Medium (2)
    *   **Frequency:** 500 Hz (2 ms period)
    *   **Role:** Continuously polls the UART buffer to parse incoming CRSF frames.

3. **`LED_Control` (Core 1)**
    *   **Priority:** Low (1)
    *   **Frequency:** Variable (up to 500 Hz)
    *   **Role:** Evaluates animations, status indications, and one-shot directional flashes. Pinned to Core 1 (APP_CPU), completely isolating it from the critical drive mechanics.

---

## Safety and Failsafes

Safety is strictly integrated into the boot sequence and the operational loop:

*   **Startup Verification:** During `setup()`, the system evaluates the boolean return values of all hardware `init()` routines. If critical hardware fails to initialize, the robot enters an infinite lock-state, triggering a continuous LED error sequence (`HardwareError`).
*   **Hardware Failsafe:** The CRSF receiver continuously monitors the incoming telemetry frames. Upon a timeout (signal loss), a thread-safe callback is executed, forcing the `ModeHandler` into `IdleMode` and setting motor speeds to a strict zero.


