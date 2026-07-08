#!/usr/bin/env bash
# Pre-flight: verify the webcam trigger path is FULLY live — the mux AND both
# cameras registered on the Jetson master. Polls ~15s (services take a few seconds
# to (re)register after a restart). A Jetson roscore restart orphans these nodes;
# the fix restarts all three so they re-register.
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.50.36:11311 ROS_IP=192.168.50.12
need="/recorder/start /recorder_cam1/start /recorder_cam2/start"
for i in $(seq 1 15); do
  have=$(rosservice list 2>/dev/null)
  miss=""
  for s in $need; do echo "$have" | grep -qE "^$s$" || miss="$miss $s"; done
  [ -z "$miss" ] && { echo "✅ 웹캠 트리거 정상 (mux + cam1 + cam2 모두 등록) — 비행 OK"; exit 0; }
  sleep 1
done
echo "❌ 마스터에 없음:$miss"
echo "   → 웹캠 녹화 안 됩니다 (보통 드론 스택/roscore 재시작으로 orphan)"
echo "   고치기: sudo systemctl restart webcam-recorder@cam1 webcam-recorder@cam2 webcam-recorder-mux"
echo "   (10~15초 뒤 다시 실행해 확인)"
exit 1
