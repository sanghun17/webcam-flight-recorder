#!/usr/bin/env bash
# Record the REAL rviz overlay to an mp4 (exact rviz look).
#
#   ./record_rviz_overlay.sh
#
# How it works: launches the isolated overlay at full image quality, plays the
# bag SLOWED (so rviz renders every 5MP frame cleanly), screen-grabs the rviz
# view with ffmpeg, then retimes the capture back to real-time 30fps.
#
# NOTE: screen capture is limited to the on-screen size (this display is
# 1920x1080), so the mp4 is <=1080p, not full 5MP. Projection stays exact.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REC="$(dirname "$HERE")/recordings/safety_2026-06-30-14-17-18"
BAG="$REC/safety_2026-06-30-14-17-18.bag"
RVIZ="$HERE/rviz_overlay.rviz"
RAW="$REC/overlay_rviz_raw.mkv"
OUT="$REC/overlay_rviz.mp4"
RATE="${OVERLAY_REC_RATE:-0.3}"          # bag playback speed while recording

source /opt/ros/noetic/setup.bash
export OVERLAY_SCALE="${OVERLAY_SCALE:-1.0}"       # full image quality for capture
export ROS_MASTER_URI="http://127.0.0.1:${ROS_OVERLAY_PORT:-11399}"
export ROS_HOSTNAME=127.0.0.1; unset ROS_IP

TF_ARGS="2.798667 2.841027 2.101022 0.367036 0.784515 -0.470636 -0.168291 odom webcam_optical"
DUR=$(python3 -c "import rosbag;b=rosbag.Bag('$BAG');print(round(b.get_end_time()-b.get_start_time(),1));b.close()")
REC_SECS=$(python3 -c "print(round($DUR/$RATE + 3, 1))")

pids=(); cleanup(){ kill "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

roscore -p "${ROS_OVERLAY_PORT:-11399}" & pids+=($!)
until rostopic list >/dev/null 2>&1; do sleep 0.3; done
rosparam set /use_sim_time true
rosrun tf2_ros static_transform_publisher $TF_ARGS & pids+=($!)
python3 "$HERE/video_publisher.py" & pids+=($!)
rviz -d "$RVIZ" & pids+=($!)

echo
echo ">>> rviz is starting. In rviz: make the 'Webcam_Overlay' Camera panel"
echo "    fill the window (double-click its tab to maximize; hide side docks"
echo "    via the Panels menu if you like)."
echo ">>> Then come back here and press ENTER to pick the capture region."
read -r
echo ">>> CLICK the rviz overlay area to select it for capture..."
eval "$(xdotool selectwindow getwindowgeometry --shell)"   # sets X,Y,WIDTH,HEIGHT
W=$((WIDTH - WIDTH % 2)); H=$((HEIGHT - HEIGHT % 2))
echo ">>> capturing region ${W}x${H} at +${X},${Y}  (bag ${DUR}s @ ${RATE}x -> ~${REC_SECS}s)"

# start screen capture
ffmpeg -y -f x11grab -framerate 30 -video_size "${W}x${H}" -i ":0.0+${X},${Y}" \
       -c:v libx264 -preset ultrafast -qp 0 "$RAW" >/dev/null 2>&1 & FF=$!
sleep 1

echo ">>> playing bag (slowed) for capture..."
rosbag play --clock -r "$RATE" "$BAG"
sleep 1
kill -INT "$FF" 2>/dev/null || true
wait "$FF" 2>/dev/null || true

echo ">>> retiming capture to real-time 30fps -> $OUT"
ffmpeg -y -i "$RAW" -vf "setpts=PTS*${RATE}" -r 30 -c:v libx264 -crf 18 -pix_fmt yuv420p "$OUT" >/dev/null 2>&1
rm -f "$RAW"
echo ">>> done: $OUT"
