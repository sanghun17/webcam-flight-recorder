#!/usr/bin/env bash
# Live rviz overlay of the bag's markers on the recorded webcam video.
#
#   ./run_overlay.sh
#
# ISOLATED: runs its own private roscore on a separate port bound to loopback,
# so `rosbag play` (which republishes /mavros, /tf, ... ) can NEVER reach the
# online master / real drone. Nothing here touches your live ROS network.
#
# Space = pause the bag.  Ctrl-C = quit (kills the private master + all nodes).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REC="$(dirname "$HERE")/recordings/safety_2026-06-30-14-17-18"
BAG="$REC/safety_2026-06-30-14-17-18.bag"
RVIZ="$HERE/rviz_overlay.rviz"

source /opt/ros/noetic/setup.bash

# --- isolation: private master on loopback, overriding any online ROS_MASTER_URI ---
PORT="${ROS_OVERLAY_PORT:-11399}"
export ROS_MASTER_URI="http://127.0.0.1:$PORT"
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
echo ">>> isolated master: $ROS_MASTER_URI (online master untouched)"

# extrinsic-derived static transform odom -> webcam_optical
TF_ARGS="2.798667 2.841027 2.101022 0.367036 0.784515 -0.470636 -0.168291 odom webcam_optical"

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# always start our OWN roscore on the private port (never reuse an existing master)
roscore -p "$PORT" & pids+=($!)
until rostopic list >/dev/null 2>&1; do sleep 0.3; done
echo ">>> private roscore up"

rosparam set /use_sim_time true
rosrun tf2_ros static_transform_publisher $TF_ARGS & pids+=($!)
python3 "$HERE/video_publisher.py" & pids+=($!)
rviz -d "$RVIZ" & pids+=($!)
sleep 3

echo ">>> playing bag (space=pause, ctrl-c=quit). looping."
rosbag play --clock --loop "$BAG"
