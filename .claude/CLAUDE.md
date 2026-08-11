# PURSUIT-1 — Person-Following PCB-Frame Micro Quadcopter

## What this is
The PCB is the airframe. 2-layer FR4 X-frame (100mm wheelbase, 91.91×91.91mm). Complete flight electronics on the board. Person detection runs off-board (laptop YOLO/MediaPipe) and sends attitude/throttle setpoints over UDP. On-board loop: Madgwick + cascaded PID at 1kHz + altitude hold on a fused
height (30Hz ToF below 3m, 50Hz BMP390 above, blended across the handover).

## Hardware on board
- **MCU**: Seeed XIAO ESP32S3 Sense (USB-C front edge, on 2.54mm headers)
- **Camera**: OV2640/OV3660 (on XIAO module)
- **IMU**: ICM-42688-P (I2C 400kHz, addr 0x68, INT1 → D9)
- **Motor drivers**: 4× AO3400A low-side (32kHz PWM) + SS34 flyback diodes
- **Motors**: 8520 coreless, clamped into frame rings
- **Altitude**: VL53L1X ToF (Pololu 3415) on rear belly, 0x29 — good to ~3.5m
- **Barometer**: BMP390 breakout at 0x77, piggybacked on the J2 I2C header —
  NOT a PCB part, no board change. Lifts the ceiling to 9.5m (~30ft). Its port
  must be covered with open-cell foam, not tape.
- **Power**: 1S LiPo, JST-PH + pigtail pads, battery telemetry divider
- **Bypass**: 4× 100nF ceramic at each motor flyback

## Repository layout
```
hardware/          KiCad 10 project (pursuit.kicad_pcb/sch, pursuit.pretty/)
gerbers/fab/       JLCPCB-ready gerbers + Excellon drill (Protel names)
gerbers/           BOM.csv, schematic PDF, renders, PURSUIT-1_JLCPCB.zip
scripts/           design.py         canonical netlist (source of truth)
                   gen_pcb.py        board generator (pcbnew API)
                   gen_sch.py        schematic generator
                   routes.py         explicit routing tables
                   footprints.py     custom footprint emitter
                   check_place.py    placement collision checker
                   preflight.py      order gate over release artefacts
                   gen_jlcpcb.py     JLCPCB BOM/CPL/orientation sheet
                   release.py        gerbers/drill/renders/PDF/zip + MANIFEST
docs/              DESIGN.md, VERIFICATION.md, assembly SVGs
sim/               LTspice simulation files
```

## Status
- KiCad design COMPLETE as of 2026-08-04
- Gerbers exported and zipped for JLCPCB
- Next: order PCB + components, flash firmware, bringup

## Key conventions
- Single source of truth: `scripts/design.py` — edit netlist here, regenerate board/schematic
- Use kicad-mcp for any schematic/layout inspection or edits
- Use ltspice MCP for analog sim (motor driver switching, power rail noise)
- Firmware will be PlatformIO (not yet scaffolded) — use `pio_build`/`pio_upload` when ready
- JLCPCB DRC rules: ~/hw/drc/JLCPCB.kicad_dru
- Release gate: run `scripts/preflight.py` before ordering

## Off-board pipeline (laptop)
- YOLO / MediaPipe person detection + face-embedding re-ID
- Sends attitude + throttle setpoints over UDP to drone
- Drone returns camera frames over WiFi
