# Diagrams and figures

Two kinds of thing live here, and the distinction matters:

- **Generated** figures are produced by a script in this repository and are
  reproducible from source. If the code changes, re-run the script.
- **Authored** figures are drawn by hand in Figma. The PNG here is an export;
  the Figma file is the master. If the code changes, these go stale **silently**
  — there is no check that catches it. Re-export after any change to a port, a
  gain, a threshold or a test count.

That second category is a duplication of facts, which is exactly the problem
`vision/gen_protocol_header.py` exists to avoid for the wire format. It is
accepted here because a diagram cannot be generated from the source in any
useful way, but it is worth knowing which figures can lie to you.

---

## Generated — reproducible from source

| File | Produced by | Shows |
|---|---|---|
| `lag_stability.png` | `python sim/flight/lag_sweep.py` | Transport-delay stability of the pitch cascade, with and without attitude compensation. The result the whole control design rests on. |
| `verification_roc.png` | `python vision/tools/verify_eval.py` | Face-verification score separation and error rates at the shipped threshold. Needs an enrolled gallery, which is gitignored — so this one cannot be regenerated from a clean clone. |

## Authored — Figma is the master

### Design file — `PURSUIT-1 — System Architecture`

https://www.figma.com/design/VfClitWLj7aM0wbpyvOWAK

Dark theme, semantic colour (blue = data/link, purple = ML, teal = control,
amber = degraded-but-flying, red = would have crashed it, green = verified fix),
Inter for prose and Geist Mono for anything that is a measured value. Colour
tokens live in the file's `PURSUIT tokens` variable collection.

| File | Frame | Notes |
|---|---|---|
| `pursuit_mindmap.png` | mind map | **README hero.** Radial node graph, four branches, curved links. 6680x3716. |
| `pursuit_architecture.png` | poster | Full detail sheet, 4400x5278. Too wide to read on screen — use the sections. |
| `arch_01_vision.png` | 01 | ML pipeline: detect, identify, decide |
| `arch_02_link.png` | 02 | Wire protocol and every failure guard |
| `arch_03_firmware.png` | 03 | On-board cascade: sense, stabilise, actuate |
| `arch_04_operate.png` | 04 | Connecting to a Mac, and the three flight modes |

### FigJam boards — rougher, easier to edit

Generated from Mermaid, so quicker to restructure than the design file. Kept
because they are the better format for dropping into a document or editing
during a conversation.

| File | Board |
|---|---|
| `figjam_01_vision_pipeline.png` | https://www.figma.com/board/bQcnI6pURh39yQSHu7r4LI |
| `figjam_02_firmware_loop.png` | https://www.figma.com/board/hDKIXj14JIKriPhL40V91c |
| `figjam_03_connect.png` | https://www.figma.com/board/q8O62Io5zOx0vvt3tfbU8w |
| `figjam_04_control_modes.png` | https://www.figma.com/board/Yqo1LX4an1RkqDv2Kq5rGm |

---

## Facts these figures assert

If any of these change, the authored figures above are wrong until re-exported.
Listed so the check is a diff of this table against the source, not a hunt.

| Fact | Value | Source of truth |
|---|---|---|
| Command port | 14550 | `firmware/include/pursuit_link.h` |
| Telemetry port | 14551 | `firmware/include/pursuit_link.h` |
| Command / telemetry size | 30 B / 36 B | `vision/pursuit/protocol.py` |
| SoftAP address | 192.168.4.1 | `vision/pursuit/config.py` |
| Camera stream / blackbox | port 81 / port 82 | `firmware/src/camera.cpp`, `blackbox.cpp` |
| Attitude loop rate | 1 kHz | `firmware/include/config.h` `LOOP_HZ` |
| Angle loop gain | 6.0 | `firmware/src/main.cpp` `ANGLE_KP` |
| Vision pitch gain | 34.0 | `vision/pursuit/config.py` |
| Image feedback DC gain | 34 / 26 = 1.31 | `sim/flight/lag_sweep.py` |
| Face match / veto threshold | 0.36 / 0.15 | `vision/pursuit/config.py` |
| Detector confidence | 0.15 | `vision/pursuit/config.py` |
| Watchdog timeouts | 400 ms, 3 s | `firmware/src/flight_state.cpp` |
| Test counts | 207 Python + 85 native | `make all` |
