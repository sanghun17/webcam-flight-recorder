#!/usr/bin/env bash
# Double-click: live rviz overlay of THIS recording (bag markers on the webcam
# video), low-res for smooth playback, in an isolated ROS master. Space=pause.
if [ -z "$OVERLAY_IN_TERM" ] && ! [ -t 1 ]; then
    export OVERLAY_IN_TERM=1; SELF="$(readlink -f "$0")"
    for TE in gnome-terminal x-terminal-emulator xterm; do
        command -v "$TE" >/dev/null 2>&1 || continue
        exec "$TE" -- bash -c "\"$SELF\"; echo; echo '--- closed. close window ---'; read -r"
    done
fi
DIR="$(dirname "$(readlink -f "$0")")"
REPO=/home/ml/webcam_recorder
bash "$REPO/rviz_overlay/run_overlay.sh" "$DIR"
