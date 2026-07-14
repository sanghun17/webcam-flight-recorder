#!/usr/bin/env bash
# Double-click AFTER 1_recalibrate: make THIS recording's clicked extrinsics the
# new BASE for this camera (cam1/cam2), so future flights of that camera overlay
# without re-clicking. Optional — only once you're happy with the overlay.
if [ -z "$OVERLAY_IN_TERM" ] && ! [ -t 1 ]; then
    export OVERLAY_IN_TERM=1; SELF="$(readlink -f "$0")"
    for TE in gnome-terminal x-terminal-emulator xterm; do
        command -v "$TE" >/dev/null 2>&1 || continue
        exec "$TE" -- bash -c "\"$SELF\"; echo; echo '--- done. close window ---'; read -r"
    done
fi
DIR="$(dirname "$(readlink -f "$0")")"
REPO=/home/ml/webcam_recorder
bash "$REPO/promote_extrinsics.sh" "$DIR"
