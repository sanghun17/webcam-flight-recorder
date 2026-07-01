#!/usr/bin/env bash
# Camera control helper for the ABKO APC930 — fix autofocus hunting & tune image.
# Controls apply live, even while previewing, so you can dial focus by eye.
#
#   ./camctl.sh show                 # print current control values
#   ./camctl.sh af-off               # disable continuous autofocus (stop hunting)
#   ./camctl.sh af-on                # restore continuous autofocus
#   ./camctl.sh focus 30             # set manual focus (0..150); implies af-off
#   ./camctl.sh sweep                # step focus 0→150 slowly (watch preview, note sharp value)
#   ./camctl.sh ae-lock              # lock exposure (manual), stop brightness pumping
#   ./camctl.sh ae-auto              # restore auto exposure
#   ./camctl.sh preview              # open live preview window (q to quit)
#
# Typical calibration:
#   Terminal A:  ./camctl.sh preview
#   Terminal B:  ./camctl.sh af-off ; ./camctl.sh sweep      # watch A, note the sharpest value
#                ./camctl.sh focus <that value>
#   Then put it in the service:  Environment=CAM_FOCUS=<value>
set -e
D="${CAM_DEVICE:-/dev/video0}"
cmd="${1:-show}"; arg="${2:-}"

case "$cmd" in
  show)
    v4l2-ctl -d "$D" --list-ctrls | grep -E "focus|exposure|white_balance|power_line|sharp|bright|contrast" ;;
  af-off)
    v4l2-ctl -d "$D" -c focus_automatic_continuous=0 && echo "continuous AF OFF (focus now manual/locked)" ;;
  af-on)
    v4l2-ctl -d "$D" -c focus_automatic_continuous=1 && echo "continuous AF ON" ;;
  focus)
    [ -n "$arg" ] || { echo "usage: $0 focus <0..150>"; exit 1; }
    v4l2-ctl -d "$D" -c focus_automatic_continuous=0 -c focus_absolute="$arg" \
      && echo "focus locked at $arg" ;;
  sweep)
    v4l2-ctl -d "$D" -c focus_automatic_continuous=0 >/dev/null
    echo "sweeping focus 0→150 (Ctrl-C to stop at a sharp value, then './camctl.sh focus N')"
    for f in $(seq 0 5 150); do
      v4l2-ctl -d "$D" -c focus_absolute="$f" >/dev/null
      printf "\rfocus_absolute=%-4s" "$f"; sleep 0.8
    done; echo ;;
  ae-lock)
    v4l2-ctl -d "$D" -c auto_exposure=1 && echo "exposure LOCKED (manual). Adjust: $0 exposure <3..2047>" ;;
  ae-auto)
    v4l2-ctl -d "$D" -c auto_exposure=3 && echo "auto exposure ON" ;;
  exposure)
    [ -n "$arg" ] || { echo "usage: $0 exposure <3..2047>"; exit 1; }
    v4l2-ctl -d "$D" -c auto_exposure=1 -c exposure_time_absolute="$arg" \
      && echo "exposure locked at $arg" ;;
  wb)
    [ -n "$arg" ] || { echo "usage: $0 wb <2800..6500>  (neutral≈4600, lower=warmer)"; exit 1; }
    v4l2-ctl -d "$D" -c white_balance_automatic=0 -c white_balance_temperature="$arg" \
      && echo "white balance locked at ${arg}K" ;;
  wb-auto)
    v4l2-ctl -d "$D" -c white_balance_automatic=1 && echo "auto white balance ON" ;;
  bright)
    [ -n "$arg" ] || { echo "usage: $0 bright <-64..64>  (default 0)"; exit 1; }
    v4l2-ctl -d "$D" -c brightness="$arg" && echo "brightness=$arg" ;;
  preview)
    exec "$(dirname "$0")/preview.sh" ;;
  *)
    echo "unknown: $cmd"; sed -n '2,30p' "$0"; exit 1 ;;
esac
