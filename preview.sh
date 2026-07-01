#!/usr/bin/env bash
# Live webcam preview for framing/aiming adjustment (화면조정용).
# Opens a low-latency window on the ml PC's local display.
#
#   ./preview.sh                # 1280x720 preview
#   CAM_SIZE=1920x1080 ./preview.sh
#
# NOTE: the camera can be opened by only ONE process at a time, so stop any
# active recording before previewing (and vice versa). Press q or ESC to close.
set -e
DEVICE="${CAM_DEVICE:-/dev/video0}"
SIZE="${CAM_SIZE:-1280x720}"
FPS="${CAM_FPS:-30}"

exec ffplay \
  -hide_banner -loglevel warning \
  -f v4l2 -input_format mjpeg -video_size "$SIZE" -framerate "$FPS" \
  -fflags nobuffer -flags low_delay -framedrop \
  -window_title "webcam preview ($SIZE) — q/ESC to quit" \
  -i "$DEVICE"
