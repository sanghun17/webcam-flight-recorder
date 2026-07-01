#!/usr/bin/env bash
# Fully headless (Xvfb) recording of a recording's rviz overlay -> mp4.
#   ./record_headless.sh [RECORDING_DIR]      (default: the sample recording)
#   TEST_SHOT=1 ./record_headless.sh [DIR]    # set up + one screenshot (tuning)
#
# Nothing shows on the physical screen. Isolated ROS master. Plays the bag slowed
# so rviz renders every frame, grabs the virtual display, retimes to real 30fps.
HERE="$(cd "$(dirname "$0")" && pwd)"
PROJ="$(dirname "$HERE")"
REC_DIR="$(readlink -f "${1:-$PROJ/recordings/safety_2026-06-30-14-17-18}")"
export REC_DIR
S="${SCRATCH:-/tmp}"
RAW="$REC_DIR/overlay_rviz_raw.mkv"
OUT="$REC_DIR/overlay_rviz.mp4"
RES="${XVFB_RES:-2048x1536}"
RATE="${OVERLAY_REC_RATE:-0.3}"

source /opt/ros/noetic/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
PORT="${ROS_OVERLAY_PORT:-11399}"
export ROS_MASTER_URI="http://127.0.0.1:$PORT" ROS_HOSTNAME=127.0.0.1; unset ROS_IP
export OVERLAY_SCALE="${OVERLAY_SCALE:-1.0}"

field() { python3 -c "import sys;sys.path.insert(0,'$PROJ');import overlay_lib as o;print(o.find_recording('$REC_DIR')['$1'] or '')"; }
BAG="$(field bag)"
for k in bag video extr; do
    [ -n "$(field "$k")" ] || { echo "!! missing '$k' in $REC_DIR"; exit 1; }
done
# Proven capture config: it floats the Webcam_Overlay Camera panel as its own
# window (so we can grab it). The rviz display setup is the same across flights;
# only tf/bag/video/start are per-recording (from overlay_lib + REC_DIR).
CAP="$HERE/rviz_capture.rviz"
TF="$(python3 "$PROJ/overlay_lib.py" tf "$REC_DIR")"

pids=(); trap 'kill "${pids[@]}" 2>/dev/null; pkill -f "Xvfb :99"; pkill -f video_publisher; pkill -f "rosbag play"; pkill -f bin/rviz; pkill -f rosmaster' EXIT

echo ">>> starting virtual display + rviz (headless, ~25s)..."
Xvfb :99 -screen 0 "${RES}x24" +extension GLX +render -noreset & pids+=($!)
sleep 2
export DISPLAY=:99
roscore -p "$PORT" & pids+=($!)
until rostopic list >/dev/null 2>&1; do sleep 0.3; done
rosparam set /use_sim_time true
rosrun tf2_ros static_transform_publisher $TF & pids+=($!)
python3 "$HERE/video_publisher.py" & pids+=($!)
rviz -d "$CAP" & pids+=($!)

PW="${PANEL_W:-1920}"; PH="${PANEL_H:-1440}"; TITLE="${TITLE_H:-26}"
echo ">>> waiting for rviz camera panel (software GL start can be slow)..."
WID=""
for _ in $(seq 1 90); do
    WID=$(xdotool search --name "Webcam_Overlay" 2>/dev/null | tail -1)
    [ -n "$WID" ] && break
    sleep 1
done
[ -n "$WID" ] || { echo "!! rviz camera panel never appeared"; exit 1; }
sleep 3
xdotool windowsize "$WID" "$PW" "$PH"; xdotool windowmove "$WID" 0 0
sleep 3
CW=$PW; CH=$((PH - TITLE))
echo "panel WID=$WID capture=${CW}x${CH}+0,${TITLE}"
if [ -n "$TEST_SHOT" ]; then
    rosbag play --clock --start 20 "$BAG" & pids+=($!)
    sleep 6
    ffmpeg -y -f x11grab -draw_mouse 0 -video_size "${CW}x${CH}" -i ":99.0+0,${TITLE}" -frames:v 1 "$S/hs_shot.png" 2>&1 | tail -1
    echo "SHOT_DONE"; sleep 2; exit 0
fi

DUR=$(python3 -c "import rosbag;b=rosbag.Bag('$BAG');print(round(b.get_end_time()-b.get_start_time(),1))")
echo ">>> recording overlay at ${RATE}x (bag ${DUR}s -> ~$(python3 -c "print(round($DUR/$RATE))")s)..."
ffmpeg -y -f x11grab -draw_mouse 0 -framerate 30 -video_size "${CW}x${CH}" -i ":99.0+0,${TITLE}" -c:v libx264 -preset ultrafast -qp 0 "$RAW" 2>/dev/null & FF=$!
sleep 1
rosbag play --clock -r "$RATE" "$BAG"
sleep 1; kill -INT "$FF" 2>/dev/null; wait "$FF" 2>/dev/null
echo ">>> retiming to real-time 30fps..."
ffmpeg -y -i "$RAW" -vf "setpts=PTS*${RATE}" -r 30 -c:v libx264 -crf 18 -pix_fmt yuv420p "$OUT" 2>/dev/null
rm -f "$RAW"
echo "DONE -> $OUT"
sleep 1
