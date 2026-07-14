#!/usr/bin/env bash
# Double-click: refine THIS recording's extrinsics + time offset by clicking the
# drone (opens a GUI). Writes webcam_extrinsics_clicked.json into this folder;
# the live/record buttons then use it automatically. Optional — only if the base
# overlay looks off.
if [ -z "$OVERLAY_IN_TERM" ] && ! [ -t 1 ]; then
    export OVERLAY_IN_TERM=1; SELF="$(readlink -f "$0")"
    for TE in gnome-terminal x-terminal-emulator xterm; do
        command -v "$TE" >/dev/null 2>&1 || continue
        exec "$TE" -- bash -c "\"$SELF\"; echo; echo '--- done. close window ---'; read -r"
    done
fi
DIR="$(dirname "$(readlink -f "$0")")"
REPO=/home/ml/webcam_recorder
source /opt/ros/noetic/setup.bash   # calib_click reads the bag (needs rosbag on PYTHONPATH)
python3 "$REPO/calib_click.py" --dir "$DIR"
