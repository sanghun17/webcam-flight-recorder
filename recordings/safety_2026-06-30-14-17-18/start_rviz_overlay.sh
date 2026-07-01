#!/usr/bin/env bash
# Double-click / run me: launches the webcam<-bag rviz overlay in a FULLY
# ISOLATED ROS master (private port, loopback) so it can never touch the online
# master / real drone. Space=pause the bag, Ctrl-C or close terminal=quit.
#
# Considers everything we solved: clicked extrinsic (via static tf), the +0.732s
# video/bag time offset, and lens distortion (frames are undistorted).

# If launched without a terminal (double-clicked in Files), reopen ourselves in
# a terminal so playback output is visible and Ctrl-C works.
if [ -z "$OVERLAY_IN_TERM" ] && ! [ -t 1 ]; then
    export OVERLAY_IN_TERM=1
    SELF="$(readlink -f "$0")"
    for TERM_EMU in gnome-terminal x-terminal-emulator xterm; do
        command -v "$TERM_EMU" >/dev/null 2>&1 || continue
        exec "$TERM_EMU" -- bash -c "\"$SELF\"; echo; echo '--- 종료됨. 창을 닫으세요 ---'; read -r"
    done
fi
set -e

OVR="/home/ml/webcam_recorder/rviz_overlay"           # publisher + rviz config
REC="/home/ml/webcam_recorder/recordings/safety_2026-06-30-14-17-18"
BAG="$REC/safety_2026-06-30-14-17-18.bag"
RVIZ="$OVR/rviz_overlay.rviz"

source /opt/ros/noetic/setup.bash

# Published image scale. Full 5MP is too heavy for rviz to texture at 30Hz (the
# image lags the markers during fast motion). 0.5 keeps it live; drop to 0.4 if
# still laggy, or 1.0 for full res. Overlay stays exact (K scales too).
export OVERLAY_SCALE="${OVERLAY_SCALE:-0.5}"

# --- isolation: private master on loopback, override any online ROS_MASTER_URI ---
PORT="${ROS_OVERLAY_PORT:-11399}"
export ROS_MASTER_URI="http://127.0.0.1:$PORT"
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
echo ">>> isolated master $ROS_MASTER_URI  (online master untouched)"

# extrinsic-derived static transform odom -> webcam_optical (webcam_extrinsics_clicked.json)
TF_ARGS="2.798667 2.841027 2.101022 0.367036 0.784515 -0.470636 -0.168291 odom webcam_optical"

pids=()
cleanup() { echo; echo ">>> shutting down..."; kill "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

roscore -p "$PORT" & pids+=($!)
until rostopic list >/dev/null 2>&1; do sleep 0.3; done
echo ">>> private roscore up"

rosparam set /use_sim_time true
rosrun tf2_ros static_transform_publisher $TF_ARGS & pids+=($!)
python3 "$OVR/video_publisher.py" & pids+=($!)
rviz -d "$RVIZ" & pids+=($!)
sleep 3

echo ">>> playing bag (space=pause, ctrl-c=quit). looping."
rosbag play --clock --loop "$BAG"
