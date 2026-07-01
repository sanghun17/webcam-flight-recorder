#!/usr/bin/env bash
# Double-click me: render the rviz overlay to an mp4, fully headless (nothing
# shows on screen), then open the result. Takes ~2-3 min. Isolated ROS master,
# so it never touches the online master / real drone.

# reopen in a terminal if double-clicked from Files, so progress is visible
if [ -z "$OVERLAY_IN_TERM" ] && ! [ -t 1 ]; then
    export OVERLAY_IN_TERM=1
    SELF="$(readlink -f "$0")"
    for TERM_EMU in gnome-terminal x-terminal-emulator xterm; do
        command -v "$TERM_EMU" >/dev/null 2>&1 || continue
        exec "$TERM_EMU" -- bash -c "\"$SELF\"; echo; echo '--- 완료. 창을 닫으세요 ---'; read -r"
    done
fi

OUT="/home/ml/webcam_recorder/recordings/safety_2026-06-30-14-17-18/overlay_rviz.mp4"
echo ">>> Rendering rviz overlay to mp4 (headless, ~2-3 min). Please wait..."
bash /home/ml/webcam_recorder/rviz_overlay/record_headless.sh

if [ -f "$OUT" ]; then
    echo ">>> Done: $OUT"
    xdg-open "$OUT" >/dev/null 2>&1 || true
else
    echo ">>> Something went wrong — no output file."
fi
