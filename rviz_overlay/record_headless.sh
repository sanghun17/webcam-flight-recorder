#!/usr/bin/env bash
# Fully headless (Xvfb) recording of the real rviz overlay -> mp4.
#   ./record_headless.sh              # full record
#   TEST_SHOT=1 ./record_headless.sh  # just set up + one screenshot (for tuning)
#
# Nothing appears on the physical screen. Plays the bag slowed so rviz renders
# every frame, grabs the virtual display, retimes to real-time 30fps.
HERE="$(cd "$(dirname "$0")" && pwd)"
REC="$(dirname "$HERE")/recordings/safety_2026-06-30-14-17-18"
BAG="$REC/safety_2026-06-30-14-17-18.bag"
CFG="$HERE/rviz_capture.rviz"
S="${SCRATCH:-/tmp}"
RAW="$REC/overlay_rviz_raw.mkv"
OUT="$REC/overlay_rviz.mp4"
RES="${XVFB_RES:-2048x1536}"          # virtual screen (4:3, no monitor limit)
RATE="${OVERLAY_REC_RATE:-0.3}"

source /opt/ros/noetic/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
export ROS_MASTER_URI="http://127.0.0.1:${ROS_OVERLAY_PORT:-11399}"
export ROS_HOSTNAME=127.0.0.1; unset ROS_IP
export OVERLAY_SCALE="${OVERLAY_SCALE:-1.0}"
TF="2.798667 2.841027 2.101022 0.367036 0.784515 -0.470636 -0.168291 odom webcam_optical"

pids=(); trap 'kill "${pids[@]}" 2>/dev/null; pkill -f "Xvfb :99"; pkill -f video_publisher; pkill -f "rosbag play"; pkill -f bin/rviz; pkill -f rosmaster' EXIT

echo ">>> starting virtual display + rviz (headless, ~25s)..."
Xvfb :99 -screen 0 "${RES}x24" +extension GLX +render -noreset & pids+=($!)
sleep 2
export DISPLAY=:99

roscore -p "${ROS_OVERLAY_PORT:-11399}" & pids+=($!)
until rostopic list >/dev/null 2>&1; do sleep 0.3; done
rosparam set /use_sim_time true
rosrun tf2_ros static_transform_publisher $TF & pids+=($!)
python3 "$HERE/video_publisher.py" & pids+=($!)
rviz -d "$CFG" & pids+=($!)
sleep 25
# grab the floating "Webcam_Overlay" camera panel, enlarge it, place at origin
PW="${PANEL_W:-1920}"; PH="${PANEL_H:-1440}"; TITLE="${TITLE_H:-26}"
WID=$(xdotool search --name "Webcam_Overlay" | tail -1)
xdotool windowsize "$WID" "$PW" "$PH"; xdotool windowmove "$WID" 0 0
sleep 3
# capture region = panel minus its Qt title bar
CX=0; CY=$TITLE; CW=$PW; CH=$((PH - TITLE))
echo "panel WID=$WID capture=${CW}x${CH}+${CX},${CY}"
if [ -n "$TEST_SHOT" ]; then
    rosbag play --clock --start 20 "$BAG" & pids+=($!)
    sleep 6
    ffmpeg -y -f x11grab -draw_mouse 0 -video_size "${CW}x${CH}" -i ":99.0+${CX},${CY}" -frames:v 1 "$S/hs_shot.png" 2>&1 | tail -1
    echo "SHOT_DONE"
    sleep 2
    exit 0
fi

DUR=$(python3 -c "import rosbag;b=rosbag.Bag('$BAG');print(round(b.get_end_time()-b.get_start_time(),1))")
echo ">>> recording overlay at ${RATE}x (bag ${DUR}s -> ~$(python3 -c "print(round($DUR/$RATE))")s capture)..."
ffmpeg -y -f x11grab -draw_mouse 0 -framerate 30 -video_size "${CW}x${CH}" -i ":99.0+${CX},${CY}" -c:v libx264 -preset ultrafast -qp 0 "$RAW" 2>/dev/null & FF=$!
sleep 1
rosbag play --clock -r "$RATE" "$BAG"
sleep 1; kill -INT "$FF" 2>/dev/null; wait "$FF" 2>/dev/null
echo ">>> retiming to real-time 30fps..."
ffmpeg -y -i "$RAW" -vf "setpts=PTS*${RATE}" -r 30 -c:v libx264 -crf 18 -pix_fmt yuv420p "$OUT" 2>/dev/null
rm -f "$RAW"
echo "DONE -> $OUT"
sleep 1
