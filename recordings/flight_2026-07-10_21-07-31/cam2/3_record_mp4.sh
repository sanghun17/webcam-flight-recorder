#!/usr/bin/env bash
# Double-click: render THIS recording's rviz overlay to overlay_rviz.mp4, fully
# headless (nothing on screen, ~2-3 min), then open the result.
if [ -z "$OVERLAY_IN_TERM" ] && ! [ -t 1 ]; then
    export OVERLAY_IN_TERM=1; SELF="$(readlink -f "$0")"
    for TE in gnome-terminal x-terminal-emulator xterm; do
        command -v "$TE" >/dev/null 2>&1 || continue
        exec "$TE" -- bash -c "\"$SELF\"; echo; echo '--- done. close window ---'; read -r"
    done
fi
DIR="$(dirname "$(readlink -f "$0")")"
REPO=/home/ml/webcam_recorder
echo ">>> Rendering rviz overlay to mp4 (headless, ~2-3 min)..."
bash "$REPO/rviz_overlay/record_headless.sh" "$DIR"
OUT="$DIR/overlay_rviz.mp4"
[ -f "$OUT" ] && { echo ">>> Done: $OUT"; xdg-open "$OUT" >/dev/null 2>&1 || true; }
