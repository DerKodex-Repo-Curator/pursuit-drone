#!/usr/bin/env python3
"""Run the follow-me pipeline.

    python run.py --target alex              # webcam, link off — the test rig
    python run.py --target alex --link       # send to the drone
    python run.py --source clip.mp4 --target alex
    python run.py --headless --target alex   # no window, prints setpoints

With `--link` omitted nothing is transmitted: the pipeline runs, the HUD shows
exactly the setpoints it *would* send, and you can verify the whole loop on a
laptop with no aircraft in the room. That is the intended way to tune it.

Keys:  SPACE arm/disarm   r reset lock   t toggle other tracks   q quit

Arming is deliberately a held state you have to set every session and it starts
SAFE. The flag rides in every packet, so a firmware that respects it will not
spin a motor because a Python process was left running.
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pursuit.arbiter import Arbiter, FlightMode                 # noqa: E402
from pursuit.config import Config                              # noqa: E402
from pursuit.hud import Hud                                     # noqa: E402
from pursuit.link import CommandPump, DroneLink                 # noqa: E402
from pursuit.lock import LockState                              # noqa: E402
from pursuit.pilot import KeyboardPilot, make_pilot             # noqa: E402
from pursuit.pipeline import Pipeline                           # noqa: E402


def screen_aspect() -> float | None:
    """Width/height of the display, for padding a full-screen canvas.

    OpenCV cannot answer this and asking it is actively misleading:
    `getWindowImageRect` reports the IMAGE it was last given, not the window.
    Measured on this machine, a 1952x1268 image in a full-screen window
    reports back exactly (0, 0, 1952, 1268) — so deriving the screen aspect
    from it is circular, hands back the aspect you already had, and the
    padding silently does nothing. That bug shipped once already.
    """
    import re
    import subprocess
    if sys.platform == "darwin":
        try:
            out = subprocess.run(["system_profiler", "SPDisplaysDataType"],
                                 capture_output=True, text=True,
                                 timeout=10).stdout
            # First match is the built-in panel on a laptop. Multi-monitor
            # setups should pass --aspect rather than hope.
            m = re.search(r"Resolution:\s*(\d+)\s*x\s*(\d+)", out)
            if m and int(m.group(2)):
                return int(m.group(1)) / int(m.group(2))
        except Exception:
            pass
    try:
        import tkinter
        root = tkinter.Tk()
        root.withdraw()
        a = root.winfo_screenwidth() / max(root.winfo_screenheight(), 1)
        root.destroy()
        return a
    except Exception:
        return None


def open_source(cfg: Config, source: str | None):
    # In flight the frames arrive as an MJPEG stream from the drone's own
    # camera, not from a local device, so a configured URL wins over the
    # webcam default. The command-line --source still overrides both.
    if source is None and cfg.source.url:
        source = cfg.source.url

    if source is None:
        cap = cv2.VideoCapture(cfg.source.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.source.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.source.height)
        cap.set(cv2.CAP_PROP_FPS, cfg.source.fps)
        # A camera silently gives you the nearest mode it has rather than
        # refusing, so ASK is not GOT. Reporting both means a request for
        # 1080p that quietly landed on 720p is visible here instead of being
        # discovered later as "why does this look soft".
        got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        label = f"camera {cfg.source.camera}  {got_w}x{got_h}"
        if (got_w, got_h) != (cfg.source.width, cfg.source.height):
            label += (f"  (asked {cfg.source.width}x{cfg.source.height} — "
                      f"camera has no such mode)")
        live = True
    elif source.startswith(("http://", "https://", "rtsp://", "udp://")):
        cap = cv2.VideoCapture(source)
        # A network stream buffers by default, which on a control loop means
        # steering on frames that are already a second old. Keep the queue at
        # one frame and drop the rest — stale video is worse than less video.
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        label, live = source, True
    elif source.isdigit():
        cap = cv2.VideoCapture(int(source))
        label, live = f"camera {source}", True
    else:
        cap = cv2.VideoCapture(source)
        label, live = source, False

    if not cap.isOpened():
        raise SystemExit(f"could not open {label}")
    return cap, label, live


def main() -> int:
    ap = argparse.ArgumentParser(description="PURSUIT-1 follow-me vision")
    ap.add_argument("--target", help="enrolled identity to follow")
    ap.add_argument("--source", help="camera index, video file, or stream URL")
    ap.add_argument("--config", help="YAML config override")
    ap.add_argument("--link", action="store_true",
                    help="actually transmit to the drone")
    ap.add_argument("--host", help="drone IP (default %(default)s)",
                    default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--headless", action="store_true", help="no preview window")
    ap.add_argument("--res", metavar="WxH",
                    help="capture resolution, e.g. 1920x1080 (default "
                         "1280x720). The detector letterboxes to 640 either "
                         "way, so this buys a sharper PICTURE, not better "
                         "detection")
    ap.add_argument("--fullscreen", action="store_true",
                    help="open the preview full screen ([f] toggles)")
    ap.add_argument("--aspect", metavar="W:H",
                    help="force the full-screen canvas aspect, e.g. 16:10. "
                         "Only needed when the screen is auto-detected wrong "
                         "— a second monitor, or a scaled resolution")
    ap.add_argument("--record", metavar="PATH", help="write annotated video")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="stop after N frames (for scripted checks)")
    ap.add_argument("--no-pilot", action="store_true",
                    help="autonomous only — no manual control layer")
    ap.add_argument("--keyboard", action="store_true",
                    help="force the keyboard pilot even if a pad is present")
    args = ap.parse_args()

    cfg = Config.load(args.config)
    if args.host:
        cfg.link.host = args.host
    if args.port:
        cfg.link.port = args.port
    cfg.link.enabled = args.link
    if args.res:
        try:
            w_s, h_s = args.res.lower().split("x")
            cfg.source.width, cfg.source.height = int(w_s), int(h_s)
        except ValueError:
            raise SystemExit(f"--res wants WxH, e.g. 1920x1080 (got {args.res})")

    cap, label, live = open_source(cfg, args.source)
    pipe = Pipeline(cfg, target_name=args.target)

    link = DroneLink(cfg.link)
    pump = CommandPump(link, cfg.link.send_hz).start() if link.enabled else None

    # Manual control sits BETWEEN vision and the link: the arbiter is the only
    # thing that authors what goes on the wire, so no code path can arm the
    # aircraft without going past the deadman check.
    pilot = None if args.no_pilot else make_pilot(
        prefer_gamepad=not args.keyboard)
    arbiter = Arbiter(cfg.control) if pilot is not None else None
    # The HUD is also what gets written to the recording, so it must be
    # rendered whenever we display OR record — not only when there is a window.
    hud = (Hud(aim_bias=cfg.control.aim_y_bias)
           if (not args.headless or args.record) else None)

    WINDOW = "PURSUIT-1"
    if hud is not None and not args.headless:
        # WINDOW_NORMAL, not the default AUTOSIZE: autosize locks the window
        # to the pixel size of the image, so it can be neither resized nor
        # made full screen.
        #
        # Deliberately WITHOUT KEEPRATIO. Keeping the ratio makes the window
        # manager letterbox a full-screen window with its own grey, which is
        # a band of somebody else's colour down the side of the instrument.
        # Instead the screen is measured once and the HUD pads its own canvas
        # out to that aspect in its own chrome — so the window is filled
        # exactly, with no stretch, and the padding belongs to the design.
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        if args.fullscreen:
            cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN,
                                  cv2.WINDOW_FULLSCREEN)

    fullscreen = bool(args.fullscreen)
    if args.aspect:
        try:
            a_w, a_h = args.aspect.replace("/", ":").split(":")
            fill = float(a_w) / float(a_h)
        except (ValueError, ZeroDivisionError):
            raise SystemExit(f"--aspect wants W:H, e.g. 16:10 (got {args.aspect})")
    else:
        fill = screen_aspect() if fullscreen else None
    if hud is not None and fullscreen:
        hud.fill_aspect = fill
        print(f"screen:  {'%.4f' % fill if fill else 'unknown'} aspect"
              f"{'' if fill else ' — pass --aspect W:H if a band shows'}")
    writer = None
    armed = False
    paused = False
    running = True
    frames = 0
    acc = {"n": 0, "total": 0.0, "face_ms": 0.0, "face_n": 0, "noface_ms": 0.0}
    t_start = time.monotonic()
    last_poll = t_start
    last_arb = t_start
    pilot_in = None
    outgoing = None

    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)

    print(f"source: {label}")
    print(f"target: {args.target or 'ANY enrolled face'}")
    print(f"link:   {'ON -> %s:%d' % (cfg.link.host, cfg.link.port) if link.enabled else 'OFF (test mode, nothing transmitted)'}")
    print(f"{'headless' if args.headless else 'press q to quit'}\n")

    try:
        while running:
            ok, frame = cap.read()
            if not ok:
                if live:
                    continue
                break
            if live and cfg.source.flip:
                frame = cv2.flip(frame, 1)

            if not paused:
                # Feed the airframe's own attitude back in, so the control
                # loop can subtract its own tilt from the target's apparent
                # motion. Without a live link this is None and the loop runs
                # uncorrected — fine on a bench, coupled in the air.
                attitude = None
                if link.enabled and link.link_up and link.telemetry is not None:
                    attitude = (link.telemetry.roll, link.telemetry.pitch)
                    # Replace the configured lead-time guess with the measured
                    # round-trip as soon as the link provides one.
                    if link.stats.rtt_ms > 0.0:
                        pipe.controller.set_measured_latency(link.stats.rtt_ms)

                result = pipe.process(frame, armed=armed, attitude=attitude)

                # Vision has produced a proposal. Whether it reaches the
                # aircraft is the arbiter's decision, not the pipeline's.
                outgoing = result.command
                if arbiter is not None:
                    now = time.monotonic()
                    pilot_in = pilot.poll(now, max(now - last_poll, 1e-3))
                    last_poll = now
                    outgoing = arbiter.update(pilot_in, result.command,
                                              now - (last_arb or now))
                    last_arb = now

                if pump is not None:
                    pump.submit(outgoing)
                frames += 1

            if hud is not None:
                view = hud.draw(result, link=link if link.enabled else None,
                                armed=armed, paused=paused,
                                arbiter=arbiter, pilot=pilot_in,
                                outgoing=outgoing)
                if args.record:
                    if writer is None:
                        h, w = view.shape[:2]
                        writer = cv2.VideoWriter(
                            args.record, cv2.VideoWriter_fourcc(*"mp4v"),
                            25.0, (w, h))
                    writer.write(view)

            if hud is not None and not args.headless:
                cv2.imshow(WINDOW, view)

                key = cv2.waitKey(1) & 0xFF
                # The keyboard pilot needs every key press, including SPACE
                # (its deadman) and 1/2/3 (mode), so feed it first and let the
                # window shortcuts take what is left.
                if isinstance(pilot, KeyboardPilot):
                    pilot.key(key)

                if key == 27 or (key == ord("q") and
                                 not isinstance(pilot, KeyboardPilot)):
                    break
                if key == ord("x"):
                    break
                if key == ord(" ") and pilot is None:
                    armed = not armed
                    print(f"  {'ARMED' if armed else 'SAFE'}")
                if key == ord("r"):
                    pipe.reset()
                    print("  lock reset")
                if key == ord("t"):
                    hud.show_all_tracks = not hud.show_all_tracks
                if key == ord("f"):
                    fullscreen = not fullscreen
                    cv2.setWindowProperty(
                        WINDOW, cv2.WND_PROP_FULLSCREEN,
                        cv2.WINDOW_FULLSCREEN if fullscreen
                        else cv2.WINDOW_NORMAL)
                    # Pad to the screen going in; drop the padding coming
                    # out, or a windowed preview keeps a screen's worth of
                    # empty chrome down both sides.
                    hud.fill_aspect = fill if fullscreen else None
                if key == ord("s"):
                    # The spotlight is the right default, but it costs you the
                    # background — and in a dark room the background is most
                    # of what there is to see.
                    hud.spotlight = not hud.spotlight
                if key == ord("p"):
                    paused = not paused

            # Averaged over every frame, not just the ones we print. Sampling
            # the printed frame alone is misleading: the face stage runs on a
            # cadence, so any print interval that shares a factor with it lands
            # exclusively on the expensive frames and overstates the cost.
            acc["n"] += 1
            acc["total"] += result.timings.total
            if result.timings.face_ran:
                acc["face_ms"] += result.timings.face
                acc["face_n"] += 1
            else:
                acc["noface_ms"] += result.timings.total
            acc[result.lock.state.value] = acc.get(result.lock.state.value, 0) + 1

            if args.headless and frames % 17 == 0:
                c, lk, t = result.command, result.lock, result.timings
                print(f"  {lk.state.value:9} {lk.source:9} "
                      f"conf {lk.confidence:4.2f}  "
                      f"yaw {c.yaw_rate:+6.1f}  pitch {c.pitch:+5.1f}  "
                      f"alt {c.alt_sp:4.2f}  |  "
                      f"this frame {t.total:5.1f}ms   "
                      f"mean {acc['total']/max(acc['n'],1):5.1f}ms  "
                      f"{t.fps:4.1f}fps")

            if args.max_frames and frames >= args.max_frames:
                break
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if not args.headless:
            cv2.destroyAllWindows()
        if pump is not None:
            pump.stop()
        link.close()

    elapsed = time.monotonic() - t_start
    print(f"\n{frames} frames in {elapsed:.1f}s "
          f"({frames/max(elapsed,1e-6):.1f} fps)")
    if acc["n"]:
        n_face, n_plain = acc["face_n"], acc["n"] - acc["face_n"]
        print(f"mean {acc['total']/acc['n']:.1f}ms/frame")
        print(f"  with face stage    {n_face:4d} frames  "
              f"({100.0*n_face/acc['n']:.0f}%)  "
              f"face itself {acc['face_ms']/max(n_face,1):.1f}ms")
        if n_plain:
            print(f"  without            {n_plain:4d} frames  "
                  f"({100.0*n_plain/acc['n']:.0f}%)  "
                  f"{acc['noface_ms']/n_plain:.1f}ms total")
        states = {k: v for k, v in acc.items()
                  if k not in ("n", "total", "face_ms", "face_n", "noface_ms")}
        total = sum(states.values()) or 1
        print("lock: " + "  ".join(f"{k} {v} ({100*v/total:.0f}%)"
                                   for k, v in sorted(states.items(),
                                                      key=lambda kv: -kv[1])))
    if link.enabled:
        s = link.stats
        print(f"link: tx {s.sent}  rx {s.received}  rejected {s.rejected}  "
              f"send errors {s.send_errors}")
        if pump is not None and pump.substituted:
            print(f"      {pump.substituted} failsafe packets substituted "
                  f"for stale commands")
    if args.record:
        print(f"recorded -> {args.record}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
