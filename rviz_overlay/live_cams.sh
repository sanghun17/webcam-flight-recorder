#!/usr/bin/env bash
# Live view of BOTH webcam feeds in rviz (the compressed preview topics that the
# recorder nodes always publish). Two Image panels: Cam1 and Cam2.
#   bash rviz_overlay/live_cams.sh
#
# The recorder nodes live on the JETSON master (192.168.50.36) — that's where the
# drone stack triggers them. Your ~/.bashrc points shells at the LOCAL ml master
# (192.168.50.12), so a normal rviz sees nothing. Force the Jetson master here
# (hard-set, not :- fallback, so it overrides the .bashrc default).
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.36:11311   # Jetson (recorder's master)
export ROS_IP=192.168.50.12                        # this ml PC (where rviz + recorder run)
exec rviz -d "$(dirname "$(readlink -f "$0")")/live_cams.rviz"
