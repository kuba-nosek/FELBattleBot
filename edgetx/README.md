# BattleBot EdgeTX telemetry

This directory contains the Boxer-side decoder and display for the firmware's
private CRSF telemetry packet. It is intended for EdgeTX 2.8 and ExpressLRS
3.5.1.

## Install on the RadioMaster Boxer

1. Copy `SCRIPTS/TELEMETRY/BBOT.lua` to the same path on the radio SD card.
2. In the model, keep the internal ExpressLRS RF module enabled normally. The
   Lua screen does not replace the mixer or channel transmission.
3. Open `Model settings` -> `Telemetry screens`, choose a `Script` screen, and
   select `BBOT`.
4. Open that telemetry screen. Short-press Enter to select the plotted
   accelerometer axis. Long-press Enter to switch between graphs and values.
5. To expose RPM, all six axes, and both peak magnitudes as normal EdgeTX
   telemetry sources, run `Discover new sensors` while `BBOT` is active and
   telemetry is linked.

The firmware sends averages for each acceleration axis and the maximum vector
magnitude observed in each 100 ms packet window. RPM is calculated and marked
valid only while Spin mode is active. A graph packet marked `STALE` has not
arrived for at least one second.

## Firmware switches

Edit the `Custom CRSF telemetry` section of `include/config.h`, then rebuild the
ESP32 firmware. `TELEMETRY_ENABLED` controls the complete packet. RPM and every
accelerometer axis have separate `TELEMETRY_SEND_*` switches. Disabled fields
remain in the fixed-size packet but have their validity bit cleared, so the Lua
screen ignores them.

## Compatibility note

The packet is CRSF frame type `0x80` (ArduPilot reserved passthrough), followed
by private subtype `0xF3`. ExpressLRS 3.5.1 forwards type `0x80`, and EdgeTX
passes unknown CRSF frame types to Lua. The subtype prevents this script from
interpreting normal ArduPilot payloads, but this private scheme must not be used
at the same time as ArduPilot/Yaapu passthrough telemetry because both share the
same reserved frame namespace and Lua receive queue.

Packet bytes received by Lua are:

| Offset | Size | Meaning |
| --- | ---: | --- |
| 0 | 1 | Private subtype `0xF3` |
| 1 | 1 | Protocol version `1` |
| 2 | 1 | Sequence number |
| 3 | 1 | Validity mask: RPM, then A1 XYZ, then A2 XYZ |
| 4 | 2 | Signed RPM, big-endian |
| 6 | 12 | Six signed acceleration averages, `0.01 g`, big-endian |
| 18 | 4 | Two unsigned peak magnitudes, `0.01 g`, big-endian |

The complete on-wire CRSF frame is 26 bytes including address, length, frame
type, and CRC8-DVB-S2.
