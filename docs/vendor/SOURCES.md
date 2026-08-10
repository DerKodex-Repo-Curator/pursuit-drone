# Vendor documentation — where to get it

The datasheets and reference schematics this design was drawn against are not
redistributed here. They are third-party copyrighted material, and a link is
both more honest and more useful than a stale copy: manufacturers revise these,
and a PDF frozen in a git history in 2026 will quietly disagree with the part
you actually receive.

Download these into this directory (it is gitignored) if you want them locally.

| Document | Part / board | Where |
|---|---|---|
| ICM-42688-P datasheet | InvenSense/TDK 6-axis IMU, rev 1.7 | [invensense.tdk.com](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/) |
| SGM6029 datasheet | SGM Micro boost regulator | [sgmicro.com](https://www.sgmicro.com/product/SGM6029) |
| XIAO ESP32S3 core schematic v1.2 | Seeed Studio | [wiki.seeedstudio.com](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) |
| XIAO ESP32S3 Sense expansion schematic v1.0 | Seeed Studio | [wiki.seeedstudio.com](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) |
| VL53L1X datasheet | STMicroelectronics ToF | [st.com](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html) |
| Carrier board 3415 | Pololu VL53L1X breakout | [pololu.com/product/3415](https://www.pololu.com/product/3415) |
| AO3400A datasheet | Alpha & Omega N-channel MOSFET | [aosmd.com](http://www.aosmd.com/products/mosfets) |
| SS34 datasheet | Schottky rectifier, 3A 40V | vendor-specific; the JLCPCB part is C8678 |

## What was taken from where

Facts extracted from these documents and encoded in this repository, so the
provenance of every non-obvious constant is traceable:

- **Register map, `firmware/src/icm42688.cpp`** — bank-0 addresses, the
  `WHO_AM_I` value `0x47`, the big-endian word order, the 1 ms post-reset delay
  and the ~45 ms gyro settling time all come from the ICM-42688-P datasheet
  rev 1.7, section 14. The big-endian detail is the one that matters: read it
  little-endian and you get values that look like plausible noise.
- **`C6`/`C7` marked DNF, `scripts/design.py`** — the SGM6029 datasheet's
  maximum output capacitance is the reason those two pads are populated in the
  footprint but deliberately not fitted.
- **Camera pin assignments, `firmware/src/camera.cpp`** — from the XIAO
  ESP32S3 Sense expansion schematic. These are on the module's internal
  connector, which is why they do not appear in `scripts/design.py`.
- **The microSD pin conflict** — the Sense expansion wires its card socket to
  D8/D9/D10, which this board uses for M4 PWM, the IMU interrupt and the LED.
  That conflict is why the blackbox logs to PSRAM instead of a card.
