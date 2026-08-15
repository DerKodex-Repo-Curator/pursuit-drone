#!/bin/bash
# Run the vision stack against the AIRCRAFT'S camera instead of the laptop
# webcam, and give the network back afterwards.
#
#   tools/run_on_drone.sh [--target NAME] [extra run.py args...]
#
# This is the real data path: OV2640 on the module -> JPEG -> WiFi -> ONNX on
# the laptop. It exercises compression artefacts, link latency and the actual
# lens the aircraft flies with, none of which a webcam test touches.
#
# The aircraft is a SoftAP with no route to the internet, so this takes the
# machine offline for as long as it runs. The restore is on a trap, so the
# network comes back whether you quit with `x`, Ctrl-C, or it crashes.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PY="$ROOT/vision/.venv/bin/python"
[ -x "$PY" ] || { echo "no venv at $PY — create it first"; exit 1; }

DEV=$(networksetup -listallhardwareports | awk '/Wi-Fi|AirPort/{getline; print $2}' | head -1)
DEV=${DEV:-en0}
PREV=$(ipconfig getsummary "$DEV" 2>/dev/null | awk -F' : ' '/ SSID/{print $2; exit}')

SSID=$(sed -nE 's/.*define[[:space:]]+PURSUIT_AP_SSID[[:space:]]+"(.*)".*/\1/p' firmware/include/secrets.h)
PASS=$(sed -nE 's/.*define[[:space:]]+PURSUIT_AP_PASS[[:space:]]+"(.*)".*/\1/p' firmware/include/secrets.h)
[ -n "$PASS" ] || { echo "could not read PURSUIT_AP_PASS from secrets.h"; exit 1; }

restore() {
  echo ""
  echo "--- restoring your network ---"
  # Drop the drone from the preferred list, or macOS will silently auto-join a
  # SoftAP with no internet later and you will spend an hour on it.
  networksetup -removepreferredwirelessnetwork "$DEV" "$SSID" >/dev/null 2>&1
  [ -n "${PREV:-}" ] && networksetup -setairportnetwork "$DEV" "$PREV" >/dev/null 2>&1
  networksetup -setairportpower "$DEV" off >/dev/null 2>&1
  sleep 2
  networksetup -setairportpower "$DEV" on  >/dev/null 2>&1
  for _ in $(seq 1 15); do
    IP=$(ifconfig "$DEV" | awk '/inet /{print $2}')
    case "${IP:-}" in
      192.168.4.*|"") sleep 2 ;;
      *) echo "back online ($IP)"; return ;;
    esac
  done
  echo "WARNING: could not confirm the network came back — check WiFi."
}
trap restore EXIT INT TERM

echo "joining $SSID — you will be OFFLINE until this exits"
for attempt in 1 2 3; do
  networksetup -setairportnetwork "$DEV" "$SSID" "$PASS" >/dev/null 2>&1
  for _ in $(seq 1 8); do
    IP=$(ifconfig "$DEV" | awk '/inet /{print $2}')
    case "${IP:-}" in 192.168.4.*) break 2 ;; esac
    sleep 1
  done
  echo "  not associated yet, retrying ($attempt/3)"
done

IP=$(ifconfig "$DEV" | awk '/inet /{print $2}')
case "${IP:-}" in
  192.168.4.*) echo "joined, $IP" ;;
  *) echo "could not join $SSID. Is the aircraft powered?"; exit 2 ;;
esac

exec "$PY" vision/run.py --source "http://192.168.4.1:81/" "$@"
