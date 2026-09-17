# BattleBot EdgeTX telemetry

This directory contains the RadioMaster Boxer decoder and display for the
firmware's private CRSF telemetry packet. It targets EdgeTX 2.8 and ExpressLRS
3.5.1.

## Generate and install

`include/config.h` is the single source of truth for packet fields:

```cpp
#define TELEMETRY_FIELD_MAP(X) \
    X(mode, uint8_t)            \
    X(rpm, int16_t)             \
    X(accel1X, float)
```

Supported types are `int8_t`, `uint8_t`, `int16_t`, `uint16_t`, `int32_t`,
`uint32_t`, and `float`. Field order in the map is field order on the wire.
There can be at most 32 fields, and the resulting CRSF frame must not exceed 64
bytes.

Every PlatformIO build regenerates three self-contained telemetry screens. To
regenerate them without building the firmware, run:

```sh
python3 scripts/generate_telemetry_lua.py
```

Copy the generated files to the same paths on the radio SD card:

- `edgetx/SCRIPTS/TELEMETRY/BBGRPH.lua` to `/SCRIPTS/TELEMETRY/BBGRPH.lua`
- `edgetx/SCRIPTS/TELEMETRY/BBVALS.lua` to `/SCRIPTS/TELEMETRY/BBVALS.lua`
- `edgetx/SCRIPTS/TELEMETRY/BBRAW.lua` to `/SCRIPTS/TELEMETRY/BBRAW.lua`

Remove the old combined `BBOT.lua` and `BBOT.luac` files if they are present.
In the model settings, assign `BBGRPH`, `BBVALS`, and `BBRAW` to three telemetry
Script screens. The generated files must match the firmware's field map.

When replacing an existing installation, also delete the old `/SCRIPTS/BBOT`
directory and `BBGRPH.luac`, `BBVALS.luac`, and `BBRAW.luac`. EdgeTX will
compile fresh copies from the updated `.lua` files.

## Controls and views

Press TELE to open the configured telemetry screens. PAGE> and PAGE< use the
normal EdgeTX navigation to switch between:

- `BBGRPH`: the RPM/acceleration graph.
- `BBVALS`: formatted mode, RPM, and accelerometer values.
- `BBRAW`: a generated raw table containing every configured field.

Turn the rotary control to select the plotted acceleration axis or move through
raw-table pages. An invalid field is displayed as `--`.

RPM and the six recognized accelerometer names are also published as normal
EdgeTX telemetry sensors. Acceleration is transmitted directly as float m/s²
and converted to centi-g when published as an EdgeTX `UNIT_G` sensor.

## Packet layout

The packet uses CRSF frame type `0x80`, private subtype `0xF3`, and protocol
version 2. It must not be used together with ArduPilot/Yaapu passthrough
telemetry because they share the same reserved CRSF frame namespace.

The payload passed to Lua contains:

| Offset | Size | Meaning |
| --- | ---: | --- |
| 0 | 1 | Private subtype `0xF3` |
| 1 | 1 | Protocol version `2` |
| 2 | 1 | Sequence number |
| 3 | 4 | Big-endian validity mask; one bit per configured field |
| 7 | Variable | Configured fields in `TELEMETRY_FIELD_MAP` order |

Invalid fields remain in the fixed packet layout with zero data and a cleared
validity bit. Multibyte integer and IEEE-754 float values are big-endian.
