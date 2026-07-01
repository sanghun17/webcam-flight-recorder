#!/usr/bin/env bash
# Live rviz overlay of a recording's bag markers on its webcam video.
#   ./run_overlay.sh [RECORDING_DIR]      (default: the sample recording)
#
# ISOLATED: private roscore on loopback, so rosbag play never reaches the online
# master / real drone. Space = pause the bag.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PROJ="$(dirname "$HERE")"
REC_DIR="$(readlink -f "${1:-$PROJ/recordings/safety_2026-06-30-14-17-18}")"
export REC_DIR

source /opt/ros/noetic/setup.bash
PORT="${ROS_OVERLAY_PORT:-11399}"
export ROS_MASTER_URI="http://127.0.0.1:$PORT" ROS_HOSTNAME=127.0.0.1; unset ROS_IP
export OVERLAY_SCALE="${OVERLAY_SCALE:-0.5}"

field() { python3 -c "import sys;sys.path.insert(0,'$PROJ');import overlay_lib as o;print(o.find_recording('$REC_DIR')['$1'] or '')"; }
BAG="$(field bag)"
for k in bag video extr; do
    [ -n "$(field "$k")" ] || { echo "!! missing '$k' in $REC_DIR — waiting for the remote to send bag/extrinsics/rviz?"; exit 1; }
done
OVER="$HERE/rviz_overlay.rviz"          # standard display setup (same across flights)
TF_ARGS="$(python3 "$PROJ/overlay_lib.py" tf "$REC_DIR")"
echo ">>> $REC_DIR  (isolated master $ROS_MASTER_URI)"

pids=(); cleanup(){ kill "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
roscore -p "$PORT" & pids+=($!)
until rostopic list >/dev/null 2>&1; do sleep 0.3; done
rosparam set /use_sim_time true
rosrun tf2_ros static_transform_publisher $TF_ARGS & pids+=($!)
python3 "$HERE/video_publisher.py" & pids+=($!)
rviz -d "$OVER" & pids+=($!)
sleep 3
echo ">>> playing bag (space=pause, ctrl-c=quit). looping."
rosbag play --clock --loop "$BAG"
