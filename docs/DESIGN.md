# PURSUIT-1 — Person-Following PCB-Frame Micro Quad

The PCB **is** the airframe: a 4-layer 1.6mm FR4 X-frame carrying a Seeed XIAO
ESP32S3 Sense (flight controller + camera + WiFi), an ICM-42688-P IMU, four
brushed-motor MOSFET stages, and a belly header for a VL53L1X time-of-flight
altitude sensor. Person recognition/re-ID runs off-board on a laptop over WiFi;
the board runs the fast stability loop.

## Architecture

```
1S LiPo (JST-PH) ──┬── VBAT rail (top pour) ──► 4x [8520 motor ── AO3400A low-side + SS34 flyback]
                   ├── 100k/100k divider ──► XIAO D2/A2 (battery telemetry)
                   └── jumper wires ──► XIAO BAT pads (USB-C charging via XIAO)

XIAO 3V3 ──► ICM-42688-P (I2C @400k, INT1) + VL53L1X breakout (I2C, shared bus) + I2C pullups
XIAO PWM x4 ──► gate resistors ──► AO3400A gates (10k pulldowns keep motors off at boot)
```

## Electrical design decisions (rationale)

- **Low-side N-MOSFET brushed drive** — no ESCs. AO3400A is the de-facto
  whoop-clone part, logic-level at 2.5V, SOT-23 hand-solderable.
- **Gate pulldowns (10k)** are mandatory: motor pins must stay low through
  ESP32-S3 boot/reset so props never spin on connect.
- **I2C for the IMU** (not SPI) — saves 2 pins on the XIAO's tight 11-GPIO
  budget; 400kHz reads gyro+accel comfortably at a 1kHz loop.
- **Flyback SS34 across each motor** (cathode to VBAT), plus bulk capacitance
  across the battery rail to ride through brushed-motor sag.
- **Battery sense** via 100k/100k divider (4.2V → 2.1V, inside ADC range,
  21µA standing drain — negligible).
- **XIAO BAT pads** wired to carrier pads under the module: enables LiPo
  charging through the XIAO's USB-C. (Slow — external 1S charger recommended
  for regular use.)

## Pin budget (XIAO ESP32S3 Sense, 11 usable GPIO)

| Pin | Net | Purpose |
|-----|-----|---------|
| D0  | M1_PWM | Motor 1 gate (GPIO1) |
| D1  | M2_PWM | Motor 2 gate (GPIO2) |
| D2  | VBAT_SENSE | ADC battery telemetry (GPIO3/ADC1_CH2 — strap pin, high-Z divider only) |
| D3  | M3_PWM | Motor 3 gate (GPIO4) |
| D4  | SDA | I2C bus (IMU + ToF) |
| D5  | SCL | I2C bus |
| D6  | (spare/UART TX) | debug via USB-C; free on castellations |
| D7  | (spare/UART RX) | spare |
| D8  | M4_PWM | Motor 4 gate |
| D9  | IMU_INT | ICM-42688-P INT1 |
| D10 | LED | orientation LED |

*(final D→GPIO mapping locked against the Seeed wiki — see research log)*

## Mechanical

- Wheelbase ~100mm (motor-to-motor diagonal) → 70.7mm adjacent spacing,
  ~5mm tip clearance for 65mm props.
- Motors press-fit in routed rings + zip-tie backup slots.
- Camera flex bent 90° at front edge; mounting slot for a foam/printed wedge.
- Zip-tie slots under center plate for the 1S 450mAh pack.
- ToF breakout on belly header, facing down.

## Verification log

Every fab-critical number (footprints, pinouts, fab rules) is
research-verified against datasheets before layout; ERC/DRC/netlist-diff and
Gerber re-import checks run before release. See VERIFICATION.md.

## Standing constraints (adversarial review findings)

- **Never insert a microSD card**: the Sense expansion's SD slot is
  hard-wired to GPIO7/8/9 = our M4_PWM / IMU_INT / LED_CTRL. Slot empty =
  lines open = fine. A card inserted would fight the IMU interrupt line.
  Optionally cut the expansion board's SD-enable jumper (Seeed wiki).
- **Never burn the STRAP_JTAG_SEL eFuse** on this module: GPIO3 carries the
  mid-rail battery divider; the strap is only sampled if that eFuse is set
  (default unset → harmless).
