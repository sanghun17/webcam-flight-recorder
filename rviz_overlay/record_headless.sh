#!/usr/bin/env bash
# Set OVERLAY_BAG=/path/to/augmented.bag to render an alternate bag while
# retaining this recording's video, extrinsics, and RViz configuration.
# Fully headless (Xvfb) recording of a recording's rviz overlay -> mp4.
#   ./record_headless.sh [RECORDING_DIR]      (default: the sample recording)
#   UNCERTAINTY_VIZ_SCALE=0.5 ./record_headless.sh [RECORDING_DIR]
#   FASTLIVO_HIDE_PREALIGN=1 ./record_headless.sh [DIR]  # visualization only
#   FASTLIVO_HISTORY=1 ./record_headless.sh [DIR]         # aligned brown trail
#   OVERLAY_OUT=/path/new.mp4 ./record_headless.sh [DIR]  # preserve default mp4
#   TEST_SHOT=1 ./record_headless.sh [DIR]    # set up + one screenshot (tuning)
#
# Nothing shows on the physical screen. Isolated ROS master. Plays the bag slowed
# so rviz renders every frame, grabs the virtual display, retimes to real 30fps.
#
# Wall time is set by one thing: how many DISTINCT frames rviz renders per wall
# second. The bag decode, the publisher and x264 all have headroom. Xvfb has no
# hardware GLX, so plain rviz falls back to llvmpipe (~14 fps at 1920x1414, bag rate
# 0.5x). With VirtualGL installed we render on the GPU via EGL and blit into the Xvfb
# window -- still nothing on screen, but several times the frame rate.
#
# The bag rate follows from that: the output is 30fps of bag time, so rviz must
# render 30*RATE frames per wall second. RATE>1 is legal -- setpts slows it back.
#
# Why not the GPU (there are four idle RTX 2080 Ti here):
#   * OVERLAY_GL=zink runs GL on top of NVIDIA's Vulkan and *does* create a 4.6
#     context on Xvfb -- but Xvfb exports Present without DRI3, so zink cannot share
#     a buffer with the X drawable and every frame comes out BLACK. Do not enable it
#     without checking a TEST_SHOT. Fixing this needs a virtual display with DRI3
#     (a headless Wayland compositor + Xwayland), none of which is installed.
#   * Rendering on the real :0 does work (~55 fps, bag rate ~1.8x, ~30s total) but
#     rviz is then visible: this box's four monitors tile the framebuffer with no
#     offscreen space, and Mutter ignores `xrandr --fb`, so there is nowhere to hide
#     the window. Ogre also stops rendering a window that is not visible, so parking
#     it on another virtual desktop just freezes the capture.
# The clean fix is VirtualGL (renders via EGL on any GPU, blits into Xvfb), which is
# not packaged for Ubuntu 20.04 and needs a one-time root install.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PROJ="$(dirname "$HERE")"
REC_DIR="$(readlink -f "${1:-/home/ml/drone-data/shared/results/webcam-recordings/safety_2026-06-30-14-17-18}")"
export REC_DIR
S="${SCRATCH:-/tmp}"
OUT="${OVERLAY_OUT:-$REC_DIR/overlay_rviz.mp4}"
RES="${XVFB_RES:-2048x1536}"

FASTLIVO_HIDE_PREALIGN="${FASTLIVO_HIDE_PREALIGN:-0}"
FASTLIVO_HISTORY="${FASTLIVO_HISTORY:-0}"
case "$FASTLIVO_HIDE_PREALIGN:$FASTLIVO_HISTORY" in
    0:0|0:1|1:0|1:1) ;;
    *) echo "!! FASTLIVO_HIDE_PREALIGN and FASTLIVO_HISTORY must be 0 or 1"; exit 1 ;;
esac
[ -d "$(dirname "$OUT")" ] || { echo "!! output directory does not exist: $(dirname "$OUT")"; exit 1; }

source /opt/ros/noetic/setup.bash
PORT="${ROS_OVERLAY_PORT:-11399}"
export ROS_MASTER_URI="http://127.0.0.1:$PORT" ROS_HOSTNAME=127.0.0.1; unset ROS_IP
exec 9>"/tmp/webcam-rviz-overlay-${PORT}.lock"
flock -n 9 || { echo "!! another overlay job is already using ROS port $PORT"; exit 1; }
# 0.75 produces an 8.50MB raw ROS image (about 255MB/s at 30Hz) and amplified the
# old publisher's stale-frame scheduling bug.  The fixed publisher also passes
# at 0.75, but 0.5 (3.78MB/frame, about 113MB/s) leaves substantial ROS/RViz
# texture-upload headroom against regressions.  The captured panel is still
# 1920px wide; RViz performs the final display scale.
export OVERLAY_SCALE="${OVERLAY_SCALE:-0.5}"
UNCERTAINTY_VIZ_SCALE="$(python3 - "${UNCERTAINTY_VIZ_SCALE:-1}" <<'PY'
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
BAG="${OVERLAY_BAG:-$(field bag)}"
[ -f "$BAG" ] || { echo "!! missing bag: $BAG"; exit 1; }
for k in video extr; do
    [ -n "$(field "$k")" ] || { echo "!! missing '$k' in $REC_DIR"; exit 1; }
done
# Capture config is derived from THIS flight's rviz.rviz (shipped from the jetson),
# so displays and topic names track the drone instead of drifting from a snapshot.
# Old recordings that predate rviz.rviz fall back to the repo copy. Either way the
# Webcam_Overlay Camera panel floats as its own window, so we can grab it by name.
# Override with OVERLAY_RVIZ_CFG=/path/to/some.rviz to render a different look.
CAP="${OVERLAY_RVIZ_CFG:-}"
if [ -z "$CAP" ]; then
    CAP="$(python3 -c "import sys;sys.path.insert(0,'$PROJ');import overlay_lib as o;print(o.build_configs('$REC_DIR', fastlivo_history=('$FASTLIVO_HISTORY' == '1'))[1] or '')")"
    [ -f "$CAP" ] || { CAP="$HERE/rviz_capture.rviz"; echo "!! no rviz.rviz in recording; using repo snapshot (topics may be stale)"; }
fi
echo ">>> rviz config: $CAP"
TF="$(python3 "$PROJ/overlay_lib.py" tf "$REC_DIR")"

# Kill only what we started: a bare `pkill -f rviz/rosmaster` also kills any ROS
# session the user has open elsewhere on this machine.
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

echo ">>> starting virtual display..."
Xvfb :99 -screen 0 "${RES}x24" +extension GLX +render -noreset & pids+=($!)
sleep 2
export DISPLAY=:99

# Pick the GL stack on the virtual display, then set the bag rate to match what it
# can render. Probe vglrun for real (an EGL device can exist but fail to render);
# OVERLAY_GL=vgl|llvmpipe|zink forces a choice.
GL="${OVERLAY_GL:-auto}"
VGL_DEV="${VGL_DEVICE:-egl0}"
if [ "$GL" = auto ]; then
    if command -v vglrun >/dev/null 2>&1 &&
       vglrun -d "$VGL_DEV" glxinfo -B 2>/dev/null | grep -qi "OpenGL vendor string: NVIDIA"; then
        GL=vgl
    else
        GL=llvmpipe
    fi
fi
case "$GL" in
vgl)   # VirtualGL renders on the GPU via EGL and blits into the Xvfb window.
       # 1.0x, not higher: the publisher now sustains 60-90Hz (threaded), but rviz
       # renders on its own ~55-60/s cadence, unsynchronized with the publisher, so
       # each rendered frame samples bag time with an error that grows with RATE.
       # Verified against the trusted 0.3x render (aligned marker IoU / PSNR):
       #   1.0x -> IoU 0.972 / 34.0dB (PASS, 49s)   1.5x -> 0.862 (FAIL, 38s)
       #   2.0x -> IoU 0.840 / 26.6dB (FAIL, 33s)
       # OVERLAY_REC_RATE=2.0 is usable as a fast draft; markers smear during fast
       # motion. Making 2x solid would need render/capture synced to the publisher.
       unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER
       RVIZ=(vglrun -d "$VGL_DEV" rviz); GLXINFO=(vglrun -d "$VGL_DEV" glxinfo -B)
       RATE="${OVERLAY_REC_RATE:-1.0}" ;;
zink)  # GL-on-Vulkan: makes a context but cannot present without DRI3 -> black frames.
       unset LIBGL_ALWAYS_SOFTWARE; export GALLIUM_DRIVER=zink
       RVIZ=(rviz); GLXINFO=(glxinfo -B)
       RATE="${OVERLAY_REC_RATE:-1.5}"
       echo "!! OVERLAY_GL=zink: no DRI3 on Xvfb, frames are very likely BLACK" ;;
*)     export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
       RVIZ=(rviz); GLXINFO=(glxinfo -B)
       RATE="${OVERLAY_REC_RATE:-0.5}" ;;
esac
echo ">>> GL: $("${GLXINFO[@]}" 2>/dev/null | sed -n 's/.*OpenGL renderer string: //p') [$GL]"

roscore -p "$PORT" & pids+=($!)
until rostopic list >/dev/null 2>&1; do sleep 0.3; done
rosparam set /use_sim_time true
rosrun tf2_ros static_transform_publisher $TF & pids+=($!)
python3 "$HERE/video_publisher.py" & pids+=($!)
"${RVIZ[@]}" -d "$CAP" & pids+=($!)
bag_remaps=()
if [ "$FASTLIVO_HIDE_PREALIGN" = "1" ] || [ "$FASTLIVO_HISTORY" = "1" ]; then
    # FAST-LIVO's high-rate body odometry is local until the OptiTrack anchor is
    # latched.  Remap the recorded stream and expose it to RViz only after the
    # first globally aligned output.  After that point the relay preserves every
    # Odometry message unchanged; the source bag and estimator output stay intact.
    relay_args=(
        "_input_topic:=/aft_mapped_to_body_imu_propagated_raw"
        "_output_topic:=/aft_mapped_to_body_imu_propagated"
        "_anchor_topic:=/aft_mapped_to_optitrack"
    )
    if [ "$FASTLIVO_HISTORY" = "1" ]; then
        relay_args+=(
            "_history_topic:=/fast_livo/odom_history"
            "_history_min_distance:=0.02"
            "_history_max_period:=0.10"
            "_history_publish_period:=0.10"
            "_history_width:=0.025"
        )
    fi
    python3 -u "$HERE/odom_after_anchor_relay.py" "${relay_args[@]}" & pids+=($!)
    bag_remaps+=("/aft_mapped_to_body_imu_propagated:=/aft_mapped_to_body_imu_propagated_raw")
    echo ">>> FAST-LIVO pre-alignment odometry hidden from visualization"
    [ "$FASTLIVO_HISTORY" != "1" ] || echo ">>> FAST-LIVO aligned history enabled (brown)"
fi
if [ "$UNCERTAINTY_VIZ_SCALE" != "1" ]; then
    python3 "$HERE/marker_scale_relay.py" \
        _scale:="$UNCERTAINTY_VIZ_SCALE" \
        _input_topic:=/local_planner/uncertainty_risk_raw \
        _output_topic:=/local_planner/uncertainty_risk & pids+=($!)
    bag_remaps+=("/local_planner/uncertainty_risk:=/local_planner/uncertainty_risk_raw")
fi
echo ">>> uncertainty ellipsoid visualization scale: x$UNCERTAINTY_VIZ_SCALE"

PW="${PANEL_W:-1920}"; PH="${PANEL_H:-1440}"; TITLE="${TITLE_H:-26}"
echo ">>> waiting for rviz camera panel..."
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

# Prime /use_sim_time and the webcam texture before recording.  Previously the
# capture began on a black Camera panel and bag playback raced RViz's first large
# image upload; depending on load this left 2--5 seconds of black/stale output.
# A single clock value at the exact bag start lets video_publisher decode the
# correct first frame without advancing any recorded topic or creating a time
# rewind.  Wait for one delivered image, then give RViz one render interval.
read -r BAG_START_SEC BAG_START_NSEC < <(python3 - "$BAG" <<'PY'
import rosbag
import sys

with rosbag.Bag(sys.argv[1]) as bag:
    total_ns = int(round(bag.get_start_time() * 1e9))
print(total_ns // 1_000_000_000, total_ns % 1_000_000_000)
PY
)
FIRST_IMAGE="$S/webcam-first-image-${PORT}.txt"
timeout 10 rostopic echo -n 1 /webcam/image_rect/header >"$FIRST_IMAGE" 2>/dev/null & FIRST_IMAGE_PID=$!
sleep 0.2
rostopic pub -1 /clock rosgraph_msgs/Clock \
    "clock: {secs: $BAG_START_SEC, nsecs: $BAG_START_NSEC}" >/dev/null
if wait "$FIRST_IMAGE_PID"; then
    echo ">>> webcam first frame ready at bag start ${BAG_START_SEC}.${BAG_START_NSEC}"
else
    echo "!! webcam first-frame handshake timed out"; exit 1
fi
sleep 0.5

if [ -n "$TEST_SHOT" ]; then
    rosbag play --clock --start 20 "$BAG" "${bag_remaps[@]}" & pids+=($!)
    sleep 6
    ffmpeg -y -f x11grab -draw_mouse 0 -video_size "${CW}x${CH}" -i ":99.0+0,${TITLE}" -frames:v 1 "$S/hs_shot.png" 2>&1 | tail -1
    echo "SHOT_DONE"; sleep 2; exit 0
fi

DUR=$(python3 -c "import rosbag;b=rosbag.Bag('$BAG');print(round(b.get_end_time()-b.get_start_time(),1))")
# The picture changes 30*RATE times per wall second. Grabbing at exactly that rate
# aliases -- x11grab and rviz free-run against each other, so -r 30 keeps landing on
# a frame it already has and ~10% of output frames come out as held duplicates.
# Oversample so every render gets seen and -r 30 can pick the nearest.
GRAB=$(python3 -c "print(min(120, max(30, int(round(30*$RATE*${OVERLAY_OVERSAMPLE:-2})))))")
GRAB="${OVERLAY_GRAB_FPS:-$GRAB}"
echo ">>> recording at ${RATE}x, grabbing ${GRAB}fps (bag ${DUR}s -> ~$(python3 -c "print(round($DUR/$RATE))")s)..."
# Single pass: retime with setpts *while* grabbing, straight to the final mp4. The
# old two-pass wrote a multi-GB lossless intermediate, then decoded it and threw away
# the frames -r 30 did not need. Only the ~30 output fps get encoded, so this is
# less CPU than the old ultrafast/qp0 capture, and the second transcode disappears.
ffmpeg -y -f x11grab -draw_mouse 0 -framerate "$GRAB" -video_size "${CW}x${CH}" -i ":99.0+0,${TITLE}" \
       -vf "setpts=PTS*${RATE}" -r 30 -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p \
       "$OUT" 2>/dev/null & FF=$!
pids+=($FF)
sleep 0.3
rosbag play --clock -r "$RATE" "$BAG" "${bag_remaps[@]}"
sleep 1; kill -INT "$FF" 2>/dev/null; wait "$FF" 2>/dev/null || true
# ffmpeg normally reports its requested SIGINT as non-zero; the finalized MP4 is valid.
echo "DONE -> $OUT"
sleep 1
