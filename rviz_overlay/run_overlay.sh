#!/usr/bin/env bash
# Live rviz overlay of a recording's bag markers on its webcam video.
#   ./run_overlay.sh [RECORDING_DIR] [UNCERTAINTY_SCALE]
#   UNCERTAINTY_VIZ_SCALE=2 ./run_overlay.sh [RECORDING_DIR]
# The scale changes only the replayed uncertainty ellipsoid marker sizes.  It
# never changes planner costs, safety masks, or any data stored in the bag.
#
# ISOLATED: private roscore on loopback, so rosbag play never reaches the online
# master / real drone. Space = pause the bag.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PROJ="$(dirname "$HERE")"
REC_DIR="$(readlink -f "${1:-/home/ml/drone-data/shared/results/webcam-recordings/safety_2026-06-30-14-17-18}")"
export REC_DIR

source /opt/ros/noetic/setup.bash
PORT="${ROS_OVERLAY_PORT:-11399}"
export ROS_MASTER_URI="http://127.0.0.1:$PORT" ROS_HOSTNAME=127.0.0.1; unset ROS_IP
exec 9>"/tmp/webcam-rviz-overlay-${PORT}.lock"
flock -n 9 || { echo "!! another overlay job is already using ROS port $PORT"; exit 1; }
export OVERLAY_SCALE="${OVERLAY_SCALE:-0.5}"
UNCERTAINTY_VIZ_SCALE="$(python3 - "${UNCERTAINTY_VIZ_SCALE:-${2:-1}}" <<'PY'
import math
import sys

try:
    value = float(sys.argv[1])
except ValueError:
    raise SystemExit(f"!! invalid UNCERTAINTY_VIZ_SCALE={sys.argv[1]!r}: expected a number > 0")
if not math.isfinite(value) or value <= 0:
    raise SystemExit(f"!! invalid UNCERTAINTY_VIZ_SCALE={value!r}: expected a finite number > 0")
print(f"{value:.12g}")
PY
)"

field() { python3 -c "import sys;sys.path.insert(0,'$PROJ');import overlay_lib as o;print(o.find_recording('$REC_DIR')['$1'] or '')"; }
BAG="$(field bag)"
for k in bag video extr; do
    [ -n "$(field "$k")" ] || { echo "!! missing '$k' in $REC_DIR — waiting for the remote to send bag/extrinsics/rviz?"; exit 1; }
done
# this flight's own rviz.rviz (from the jetson); repo copy only for old recordings
OVER="$(python3 -c "import sys;sys.path.insert(0,'$PROJ');import overlay_lib as o;print(o.build_configs('$REC_DIR')[0] or '')")"
[ -f "$OVER" ] || { OVER="$HERE/rviz_overlay.rviz"; echo "!! no rviz.rviz in recording; using repo snapshot (topics may be stale)"; }
TF_ARGS="$(python3 "$PROJ/overlay_lib.py" tf "$REC_DIR")"
echo ">>> $REC_DIR  (isolated master $ROS_MASTER_URI)"
echo ">>> uncertainty ellipsoid visualization scale: x$UNCERTAINTY_VIZ_SCALE"

pids=()
process_tree() {
    local parent="$1" child
    for child in $(pgrep -P "$parent" 2>/dev/null || true); do process_tree "$child"; done
    echo "$parent"
}
cleanup() {
    local targets=() p q
    for p in "${pids[@]:-}"; do
        while read -r q; do targets+=("$q"); done < <(process_tree "$p")
    done
    [ "${#targets[@]}" -eq 0 ] || kill -INT "${targets[@]}" 2>/dev/null || true
    sleep 0.3
    [ "${#targets[@]}" -eq 0 ] || kill -TERM "${targets[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
roscore -p "$PORT" & pids+=($!)
until rostopic list >/dev/null 2>&1; do sleep 0.3; done
rosparam set /use_sim_time true
rosrun tf2_ros static_transform_publisher $TF_ARGS & pids+=($!)
python3 "$HERE/video_publisher.py" & pids+=($!)
rviz -d "$OVER" & pids+=($!)
bag_remaps=()
if [ "$UNCERTAINTY_VIZ_SCALE" != "1" ]; then
    python3 "$HERE/marker_scale_relay.py" \
        _scale:="$UNCERTAINTY_VIZ_SCALE" \
        _input_topic:=/local_planner/uncertainty_risk_raw \
        _output_topic:=/local_planner/uncertainty_risk & pids+=($!)
    bag_remaps+=("/local_planner/uncertainty_risk:=/local_planner/uncertainty_risk_raw")
fi
sleep 3
echo ">>> playing bag (space=pause, ctrl-c=quit). looping."
rosbag play --clock --loop "$BAG" "${bag_remaps[@]}"
