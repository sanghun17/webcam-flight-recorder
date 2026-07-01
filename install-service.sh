#!/usr/bin/env bash
# Install + start the unified recorder ROS node as an always-on systemd service.
# It owns the camera: always publishes a live ROS preview, and records to disk
# on /recorder/start .. /recorder/stop.
# Run once:   sudo bash /home/ml/webcam_recorder/install-service.sh
set -e
SRC=/home/ml/webcam_recorder/webcam-recorder.service
DST=/etc/systemd/system/webcam-recorder.service

if [ "$(id -u)" -ne 0 ]; then echo "run with sudo: sudo bash $0"; exit 1; fi

cp "$SRC" "$DST"
systemctl daemon-reload
systemctl enable webcam-recorder
# restart (not just start) so an already-running instance picks up new settings.
systemctl restart webcam-recorder
echo "--- status ---"
systemctl --no-pager status webcam-recorder || true
echo
echo "Installed. Starts on boot, restarts on crash."
echo "  logs:        journalctl -u webcam-recorder -f"
echo "  ros check:   rosservice list | grep recorder"
echo "               rostopic hz /recorder/image_raw/compressed"
echo "  record:      rosservice call /recorder/start ; rosservice call /recorder/stop"
