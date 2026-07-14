#!/usr/bin/env bash
# Fully headless (Xvfb) recording of a recording's rviz overlay -> mp4.
#   ./record_headless.sh [RECORDING_DIR]      (default: the sample recording)
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
HERE="$(cd "$(dirname "$0")" && pwd)"
PROJ="$(dirname "$HERE")"
REC_DIR="$(readlink -f "${1:-$PROJ/recordings/safety_2026-06-30-14-17-18}")"
export REC_DIR
S="${SCRATCH:-/tmp}"
OUT="$REC_DIR/overlay_rviz.mp4"
RES="${XVFB_RES:-2048x1536}"

source /opt/ros/noetic/setup.bash
PORT="${ROS_OVERLAY_PORT:-11399}"
export ROS_MASTER_URI="http://127.0.0.1:$PORT" ROS_HOSTNAME=127.0.0.1; unset ROS_IP
# The Camera panel is 1920 wide, so 0.75 (1944px) is the smallest image that still
# feeds it 1:1. Texture upload dominates rviz's per-frame cost, so this is the knob
# that matters -- panel size and scene complexity barely move it.
export OVERLAY_SCALE="${OVERLAY_SCALE:-0.75}"

field() { python3 -c "import sys;sys.path.insert(0,'$PROJ');import overlay_lib as o;print(o.find_recording('$REC_DIR')['$1'] or '')"; }
BAG="$(field bag)"
for k in bag video extr; do
    [ -n "$(field "$k")" ] || { echo "!! missing '$k' in $REC_DIR"; exit 1; }
done
# Capture config is derived from THIS flight's rviz.rviz (shipped from the jetson),
# so displays and topic names track the drone instead of drifting from a snapshot.
# Old recordings that predate rviz.rviz fall back to the repo copy. Either way the
# Webcam_Overlay Camera panel floats as its own window, so we can grab it by name.
# Override with OVERLAY_RVIZ_CFG=/path/to/some.rviz to render a different look.
CAP="${OVERLAY_RVIZ_CFG:-}"
if [ -z "$CAP" ]; then
    CAP="$(python3 -c "import sys;sys.path.insert(0,'$PROJ');import overlay_lib as o;print(o.build_configs('$REC_DIR')[1] or '')")"
    [ -f "$CAP" ] || { CAP="$HERE/rviz_capture.rviz"; echo "!! no rviz.rviz in recording; using repo snapshot (topics may be stale)"; }
fi
echo ">>> rviz config: $CAP"
TF="$(python3 "$PROJ/overlay_lib.py" tf "$REC_DIR")"

# Kill only what we started: a bare `pkill -f rviz/rosmaster` also kills any ROS
# session the user has open elsewhere on this machine.
pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null; done; return 0; }
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
if [ -n "$TEST_SHOT" ]; then
    rosbag play --clock --start 20 "$BAG" & pids+=($!)
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
sleep 1
rosbag play --clock -r "$RATE" "$BAG"
sleep 1; kill -INT "$FF" 2>/dev/null; wait "$FF" 2>/dev/null
echo "DONE -> $OUT"
sleep 1
