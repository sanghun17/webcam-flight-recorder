#!/usr/bin/env bash
# Install + start BOTH cameras as always-on systemd services.
#
# Uses a systemd template (webcam-recorder@.service) instantiated once per camera
# (cam1, cam2). Each instance owns one APC930 (selected by-id in env/<inst>.env),
# publishes a live ROS preview, and records to disk on <ns>/start .. <ns>/stop.
# A fan-out node (webcam-recorder-mux) re-advertises the single /recorder/start|stop
# that the onboard flight-safety recorder calls, forwarding it to both cameras.
#
# Run once:   sudo bash /home/ml/webcam_recorder/install-service.sh
set -e
HERE=/home/ml/webcam_recorder
INSTANCES="cam1 cam2"

if [ "$(id -u)" -ne 0 ]; then echo "run with sudo: sudo bash $0"; exit 1; fi

# 1. Retire the old single-camera unit if present (superseded by the template).
if systemctl list-unit-files webcam-recorder.service >/dev/null 2>&1; then
  systemctl disable --now webcam-recorder.service 2>/dev/null || true
  rm -f /etc/systemd/system/webcam-recorder.service
fi

# 2. Install the per-camera template, the fan-out unit, and udev auto-restart rules.
cp "$HERE/webcam-recorder@.service"     /etc/systemd/system/webcam-recorder@.service
cp "$HERE/webcam-recorder-mux.service"  /etc/systemd/system/webcam-recorder-mux.service
cp "$HERE/99-webcam-recorder.rules"     /etc/udev/rules.d/99-webcam-recorder.rules
udevadm control --reload-rules
# create /dev/webcam_cam1|cam2 symlinks for the already-connected cameras (no replug needed)
udevadm trigger --subsystem-match=video4linux
udevadm settle

# 3. Enable + (re)start every camera instance, then the fan-out. restart (not just
#    start) so an already-running unit picks up new settings.
systemctl daemon-reload
for i in $INSTANCES; do
  systemctl enable "webcam-recorder@${i}.service"
  systemctl restart "webcam-recorder@${i}.service"
done
systemctl enable webcam-recorder-mux.service
systemctl restart webcam-recorder-mux.service

echo "--- status ---"
for u in webcam-recorder@cam1 webcam-recorder@cam2 webcam-recorder-mux; do
  systemctl --no-pager status "${u}.service" | grep -E "Active|Main PID" | sed "s/^/  ${u}: /"
done

cat <<'EOF'

Installed both cameras + fan-out. Starts on boot, restarts on crash, and
re-grabs its camera on USB re-plug (each camera restarts only its own instance).

  logs:    journalctl -u webcam-recorder@cam1 -f
           journalctl -u webcam-recorder@cam2 -f
           journalctl -u webcam-recorder-mux -f
  check:   rosservice list | grep recorder
           rostopic hz /recorder_cam1/image_raw/compressed
           rostopic hz /recorder_cam2/image_raw/compressed
  both:    rosservice call /recorder/start ; rosservice call /recorder/stop
           (onboard ARM/DISARM path -> both cameras, paired flight_<ts>_cam1/_cam2)
  one:     rosservice call /recorder_cam1/start ; rosservice call /recorder_cam1/stop
EOF
