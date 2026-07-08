#!/usr/bin/env python3
"""
recorder_node — unified ROS 1 camera node for the flight testbed.

One process owns the camera (a single ffmpeg). It ALWAYS publishes a downscaled
live preview as sensor_msgs/CompressedImage (jpeg), and on a start/stop service
call it ALSO records the full-resolution stream to an H.264 MP4 on disk.

  Live image (always on, even with no subscribers):
      <CAM_IMG_TOPIC>/compressed   (default: /recorder/image_raw/compressed)

  Record control (std_srvs/Trigger):
      rosservice call /recorder/start        # begin full-res recording
      rosservice call /recorder/stop         # finalize + save

Design: heavy encoding stays in ffmpeg (C). ffmpeg emits a small MJPEG stream on
its stdout pipe; this node just splits JPEG frames and republishes them — no
decode/re-encode in Python, so full 5MP is cheap. Exactly one ffmpeg owns
/dev/video0 at a time (preview-only when idle, preview+file when recording), so
there is never a device-busy conflict.

All the recorder.py safety behavior is preserved: max-duration watchdog, disk
floor, rotate-on-restart, collision-safe filenames, and the recordings.csv
manifest.

No HTTP. stdlib + rospy + sensor_msgs/std_srvs only (no OpenCV). ffmpeg and
v4l2-ctl must be installed.
"""
import csv
import os
import shutil
import signal
import stat
import subprocess
import threading
import time
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))
_BUTTONS_SRC = os.path.join(_HERE, "rviz_overlay", "folder_buttons")


def _install_folder_buttons(recdir):
    """Drop the overlay double-click buttons into a new recording folder so, once
    the remote sends the bag/extrinsics/rviz, the user can view/recalibrate/render
    with a double-click. Best-effort."""
    if not os.path.isdir(_BUTTONS_SRC):
        return
    for fn in sorted(os.listdir(_BUTTONS_SRC)):
        if not fn.endswith(".sh"):
            continue
        dst = os.path.join(recdir, fn)
        try:
            shutil.copyfile(os.path.join(_BUTTONS_SRC, fn), dst)
            os.chmod(dst, os.stat(dst).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        except OSError as e:
            rospy.logwarn("could not install overlay button %s: %s", fn, e)

import rospy
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger, TriggerResponse

# ---- CONFIG (override with env vars) ----------------------------------------
DEVICE     = os.environ.get("CAM_DEVICE", "/dev/video0")
VIDEO_SIZE = os.environ.get("CAM_SIZE", "1920x1080")
FRAMERATE  = os.environ.get("CAM_FPS", "30")
INPUT_FMT  = os.environ.get("CAM_INPUT_FMT", "mjpeg")   # camera native pixel format
OUTDIR     = os.environ.get("CAM_OUTDIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings"))
PRESET     = os.environ.get("CAM_PRESET", "veryfast")   # x264 speed/size tradeoff
CRF        = os.environ.get("CAM_CRF", "20")            # x264 quality: lower=sharper/bigger
CODEC      = os.environ.get("CAM_CODEC", "h264").lower()  # h264 (mp4) | ffv1 (lossless, mkv)
NODE_NAME  = os.environ.get("CAM_NODE_NAME", "recorder")  # ROS node name = service/topic namespace (one per camera)
TAG        = os.environ.get("CAM_TAG", "")                # per-camera recording suffix: flight_<ts>_<TAG> (e.g. cam1)
STEM_PARAM = os.environ.get("CAM_SESSION_STEM_PARAM", "/recorder/session_stem")  # shared name pinned by recorder_mux
# ---- live preview (ROS CompressedImage) ----
IMG_TOPIC    = os.environ.get("CAM_IMG_TOPIC", "/recorder/image_raw")  # "/compressed" is appended
FRAME_ID     = os.environ.get("CAM_FRAME_ID", "camera")
PREVIEW_W    = os.environ.get("CAM_PREVIEW_W", "640")     # preview width px (height auto, aspect kept)
PREVIEW_FPS  = os.environ.get("CAM_PREVIEW_FPS", "0")     # preview frames/sec to ROS; 0/empty = camera native rate
PREVIEW_Q    = os.environ.get("CAM_PREVIEW_Q", "6")       # mjpeg quality 2..31 (lower=better/bigger)
# ---- safety nets ----
MAX_SEC      = int(os.environ.get("CAM_MAX_SEC", "1800"))      # auto-stop after N s (0=off)
MIN_FREE_MB  = int(os.environ.get("CAM_MIN_FREE_MB", "2000"))  # refuse/auto-stop below this free space (0=off)
ON_DUP       = os.environ.get("CAM_ON_DUP", "rotate").lower()  # start while recording: rotate | reject
WATCHDOG_SEC = 2
# ---- camera controls (applied via v4l2-ctl before each ffmpeg (re)launch) ----
FOCUS_AUTO    = os.environ.get("CAM_FOCUS_AUTO", "0")
FOCUS_ABS     = os.environ.get("CAM_FOCUS", "")
AUTO_EXPOSURE = os.environ.get("CAM_AUTO_EXPOSURE", "")
EXPOSURE_ABS  = os.environ.get("CAM_EXPOSURE", "")
AUTO_WB       = os.environ.get("CAM_AUTO_WB", "")
WB_TEMP       = os.environ.get("CAM_WB_TEMP", "")
BRIGHTNESS    = os.environ.get("CAM_BRIGHTNESS", "")
POWER_FREQ    = os.environ.get("CAM_POWER_FREQ", "2")
SHARPNESS     = os.environ.get("CAM_SHARPNESS", "")
EXTRA_CTRLS   = os.environ.get("CAM_CTRLS", "")
# -----------------------------------------------------------------------------

os.makedirs(OUTDIR, exist_ok=True)
MANIFEST = os.path.join(OUTDIR, "recordings.csv")
MANIFEST_COLS = ["name", "file", "start", "end", "duration_sec", "size_bytes", "size_mb", "stop_reason"]

_REC_EXT = {"h264": ".mp4", "ffv1": ".mkv"}.get(CODEC, ".mp4")


def _safe(s):
    return "".join(c for c in s if c.isalnum() or c in "-_")[:40] or "flight"


def _ts(dt):
    return dt.strftime("%Y-%m-%d %H:%M:%S")


def _tail(p, n=400):
    try:
        with open(p, "rb") as f:
            return f.read()[-n:].decode("utf-8", "replace").strip()
    except OSError:
        return "(no log)"


def _free_mb(path):
    try:
        st = os.statvfs(path)
        return int(st.f_bavail * st.f_frsize / 1e6)
    except OSError:
        return None


def _append_manifest(row):
    new = not os.path.exists(MANIFEST)
    try:
        with open(MANIFEST, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=MANIFEST_COLS)
            if new:
                w.writeheader()
            w.writerow(row)
    except OSError as e:
        rospy.logwarn("could not write manifest: %s", e)


def _apply_camera_controls():
    """Pin focus/exposure/wb via v4l2-ctl so the image stops hunting. Auto modes
    must be disabled BEFORE their absolute values, or this driver ignores the
    values — so 'first' (auto toggles) is applied before 'rest' (absolutes).

    Each control is set in its OWN v4l2-ctl call: the two APC930 hardware
    revisions expose different control sets (rev 1105 is fixed-focus with no
    focus_* controls), and a comma-joined call fails as a whole on the first
    unknown control — silently dropping exposure/wb/brightness too, leaving that
    camera fully auto and looking nothing like the other. Per-control calls keep
    both cameras matched on every control that exists. Best-effort: an unknown
    control is skipped with a log line, never fatal."""
    first = [f"focus_automatic_continuous={FOCUS_AUTO}"]
    if AUTO_EXPOSURE != "": first.append(f"auto_exposure={AUTO_EXPOSURE}")
    if AUTO_WB != "":       first.append(f"white_balance_automatic={AUTO_WB}")
    rest = []
    if FOCUS_ABS != "":    rest.append(f"focus_absolute={FOCUS_ABS}")
    if EXPOSURE_ABS != "": rest.append(f"exposure_time_absolute={EXPOSURE_ABS}")
    if WB_TEMP != "":      rest.append(f"white_balance_temperature={WB_TEMP}")
    if BRIGHTNESS != "":   rest.append(f"brightness={BRIGHTNESS}")
    if POWER_FREQ != "":   rest.append(f"power_line_frequency={POWER_FREQ}")
    if SHARPNESS != "":    rest.append(f"sharpness={SHARPNESS}")
    if EXTRA_CTRLS:        rest += [c.strip() for c in EXTRA_CTRLS.split(",") if c.strip()]
    for ctrl in first + rest:   # order preserved: auto toggles, then absolutes
        for attempt in range(4):   # device can be briefly busy right after a (re)start
            try:
                subprocess.run(["v4l2-ctl", "-d", DEVICE, "-c", ctrl],
                               check=True, capture_output=True, text=True, timeout=5)
                break
            except FileNotFoundError:
                rospy.logwarn("v4l2-ctl not installed — camera controls skipped")
                return
            except subprocess.CalledProcessError as e:
                err = e.stderr.strip()
                # "Cannot open device ... busy" during a restart race -> retry so we
                # don't leave e.g. exposure frozen at its auto value. A genuinely
                # unknown control (fixed-focus unit rejecting focus_*) won't recover
                # -> skip immediately, keep applying the rest.
                if "annot open" in err and attempt < 3:
                    time.sleep(0.5)
                    continue
                rospy.logwarn("camera control skipped (%s): %s", ctrl, err)
                break
            except subprocess.TimeoutExpired:
                rospy.logwarn("v4l2-ctl timed out on %s", ctrl)
                break


def _input_args():
    return ["-f", "v4l2", "-input_format", INPUT_FMT,
            "-video_size", VIDEO_SIZE, "-framerate", FRAMERATE, "-i", DEVICE]


def _preview_output():
    # Downscaled MJPEG to stdout; each frame is a self-contained JPEG.
    # PREVIEW_FPS 0/empty -> no fps filter, i.e. publish at the camera's native
    # rate (bounded by what the sensor delivers at CAM_SIZE: ~30fps @1080p,
    # ~19fps @5MP). A positive value caps the preview to save CPU/bandwidth.
    vf = f"scale={PREVIEW_W}:-2"
    if PREVIEW_FPS not in ("", "0"):
        vf += f",fps={PREVIEW_FPS}"
    return ["-map", "0:v",
            "-filter:v", vf,
            "-c:v", "mjpeg", "-q:v", PREVIEW_Q,
            "-f", "mjpeg", "pipe:1"]


def _record_output(path):
    if CODEC == "ffv1":
        return ["-map", "0:v", "-c:v", "ffv1", "-level", "3", path]
    return ["-map", "0:v", "-c:v", "libx264", "-preset", PRESET, "-crf", CRF,
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", path]


class CameraNode:
    """Owns one ffmpeg at a time. Publishes preview always; records on demand."""

    def __init__(self):
        self._lock = threading.RLock()
        self._proc = None
        self._logf = None
        self._recording = False
        self._path = self._name = None
        self._started_at = self._started_dt = None

        topic = IMG_TOPIC.rstrip("/") + "/compressed"
        self._pub = rospy.Publisher(topic, CompressedImage, queue_size=2)
        rospy.loginfo("publishing preview -> %s", topic)

        rospy.Service("~start", Trigger, self._srv_start)
        rospy.Service("~stop", Trigger, self._srv_stop)

        # Start the always-on preview.
        with self._lock:
            self._launch_preview_unlocked()

    # --- ffmpeg lifecycle (call with lock held) ---------------------------

    def _launch_preview_unlocked(self):
        _apply_camera_controls()
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "warning", "-nostdin"]
        cmd += _input_args() + _preview_output()
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, start_new_session=True)
        except FileNotFoundError:
            rospy.logerr("ffmpeg not installed — cannot open camera")
            return
        self._proc, self._recording = proc, False
        self._start_reader(proc)
        self._reapply_controls_after_stream()   # exposure gets reset at stream start
        pv_rate = "native" if PREVIEW_FPS in ("", "0") else PREVIEW_FPS + "fps"
        rospy.loginfo("preview running (%s @ %sfps, %s px wide, %s to ROS)",
                      VIDEO_SIZE, FRAMERATE, PREVIEW_W, pv_rate)

    def _launch_recording_unlocked(self, stem, tag, label):
        # Nested layout: recordings/<stem>/<tag>/ so both cameras of one flight share
        # the <stem> parent (the shared bag lands at <stem>/). Standalone (no tag)
        # falls back to the flat recordings/<stem>/.
        recdir = os.path.join(OUTDIR, stem, tag) if tag else os.path.join(OUTDIR, label)
        base = label   # mp4 basename + manifest name; suffixed only on a real collision
        n = 2
        while os.path.exists(os.path.join(recdir, base + _REC_EXT)):
            base = f"{label}_{n}"; n += 1
        os.makedirs(recdir, exist_ok=True)
        _install_folder_buttons(recdir)
        path = os.path.join(recdir, base + _REC_EXT)
        logpath = os.path.join(recdir, base + ".ffmpeg.log")
        _apply_camera_controls()
        logf = open(logpath, "wb")
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "warning", "-nostdin"]
        cmd += _input_args() + _record_output(path) + _preview_output()
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=logf, start_new_session=True)
        except FileNotFoundError:
            logf.close()
            self._launch_preview_unlocked()
            return False, "ffmpeg not installed"

        self._proc, self._logf, self._recording = proc, logf, True
        self._path, self._name = path, base
        self._started_at, self._started_dt = time.time(), datetime.now()
        self._start_reader(proc)
        self._reapply_controls_after_stream()   # exposure gets reset at stream start

        time.sleep(1.0)
        if proc.poll() is not None:  # died immediately -> bad args/device
            logf.flush()
            tail = _tail(logpath)
            self._recording = False
            self._logf = None
            logf.close()
            self._launch_preview_unlocked()
            return False, f"ffmpeg exited immediately: {tail}"

        note = "" if base == label else f" (requested '{label}', auto-suffixed)"
        rospy.loginfo("START name=%s%s file=%s", base, note, path)
        return True, "recording started"

    def _reapply_controls_after_stream(self):
        """exposure_time_absolute gets reset to a default (~78) when a NEW
        streaming session starts on the rev-1008 unit — wiping the value _apply
        set just before ffmpeg opened the device (focus/wb/brightness survive;
        exposure does not). Re-apply once the stream is up so the pinned exposure
        actually holds (verified: setting exposure while streaming sticks)."""
        def _later():
            time.sleep(1.5)
            _apply_camera_controls()
        threading.Thread(target=_later, daemon=True).start()

    def _kill_proc_unlocked(self, sigint):
        proc = self._proc
        self._proc = None
        if not proc:
            return
        try:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT if sigint else signal.SIGTERM)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
        except Exception as e:
            rospy.logwarn("error stopping ffmpeg: %s", e)

    def _finalize_recording_unlocked(self, reason):
        """Stop the recording ffmpeg cleanly (SIGINT writes the container
        trailer), log it to the manifest. Does NOT relaunch preview."""
        path, name = self._path, self._name
        start_dt, started_at = self._started_dt, self._started_at
        self._kill_proc_unlocked(sigint=True)  # SIGINT -> finalize file
        if self._logf:
            self._logf.close()
            self._logf = None
        self._recording = False

        end_dt = datetime.now()
        dur = round(time.time() - started_at, 1)
        size = os.path.getsize(path) if path and os.path.exists(path) else 0
        size_mb = round(size / 1e6, 1)
        _append_manifest({
            "name": name, "file": path,
            "start": _ts(start_dt), "end": _ts(end_dt),
            "duration_sec": dur, "size_bytes": size, "size_mb": size_mb,
            "stop_reason": reason,
        })
        tag = "" if reason == "request" else f"  [{reason}]"
        rospy.loginfo("STOP%s name=%s duration=%ss size=%sMB file=%s",
                      tag, name, dur, size_mb, path)
        return {"file": path, "name": name, "duration_sec": dur, "size_mb": size_mb}

    # --- preview reader ---------------------------------------------------

    def _start_reader(self, proc):
        t = threading.Thread(target=self._reader, args=(proc,), daemon=True)
        t.start()

    def _reader(self, proc):
        """Split ffmpeg's MJPEG stdout into JPEG frames and publish each. Ends at
        EOF, i.e. when this ffmpeg is stopped/replaced."""
        SOI, EOI = b"\xff\xd8", b"\xff\xd9"
        buf = b""
        stream = proc.stdout
        try:
            while True:
                # read1(): return whatever one syscall yields, instead of blocking
                # until the full buffer fills — keeps frames flowing individually
                # (a greedy read() batches them into bursts that a small queue drops).
                chunk = stream.read1(65536)
                if not chunk:
                    break
                buf += chunk
                while True:
                    s = buf.find(SOI)
                    if s < 0:
                        if len(buf) > 4_000_000:
                            buf = b""  # runaway guard: no SOI in sight
                        break
                    e = buf.find(EOI, s + 2)
                    if e < 0:
                        if s > 0:
                            buf = buf[s:]  # drop junk before the next frame
                        break
                    self._publish(buf[s:e + 2])
                    buf = buf[e + 2:]
        except Exception as e:
            rospy.logwarn("preview reader stopped: %s", e)

    def _publish(self, jpeg):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = FRAME_ID
        msg.format = "jpeg"
        msg.data = jpeg
        try:
            self._pub.publish(msg)
        except Exception:
            pass  # publishing during shutdown

    # --- services ---------------------------------------------------------

    def _srv_start(self, _req):
        with self._lock:
            free = _free_mb(OUTDIR)
            if MIN_FREE_MB and free is not None and free < MIN_FREE_MB:
                return TriggerResponse(False, f"low disk: {free}MB < {MIN_FREE_MB}MB — not starting")
            # recorder_mux pins one shared stem (this param) at ARM so every camera's
            # file for the same flight matches: flight_<stamp>_cam1, _cam2, ...
            # Empty/unset -> this node timestamps itself (manual/standalone use).
            stem = str(rospy.get_param(STEM_PARAM, "")).strip() or \
                   ("flight_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
            tag = _safe(TAG) if TAG else ""              # cam1/cam2 subfolder; "" = standalone
            label = stem + ("_" + tag if tag else "")    # mp4 basename + manifest name
            if self._recording:
                if ON_DUP != "rotate":
                    return TriggerResponse(False, f"already recording '{self._name}' (send stop first)")
                rospy.loginfo("ROTATE: finalizing '%s' before new recording", self._name)
                self._finalize_recording_unlocked(reason="rotate")
            else:
                self._kill_proc_unlocked(sigint=False)  # release preview ffmpeg
            ok, msg = self._launch_recording_unlocked(stem, tag, label)
            return TriggerResponse(ok, msg)

    def _srv_stop(self, _req):
        with self._lock:
            if not self._recording:
                return TriggerResponse(False, "not recording")
            info = self._finalize_recording_unlocked(reason="request")
            self._launch_preview_unlocked()  # resume live view
            return TriggerResponse(True, f"stopped: {info['file']} ({info['size_mb']}MB)")

    # --- watchdog ---------------------------------------------------------

    def watchdog_tick(self):
        with self._lock:
            if not self._recording:
                return
            elapsed = time.time() - self._started_at
            if MAX_SEC and elapsed >= MAX_SEC:
                rospy.loginfo("max duration %ss reached — auto-stopping '%s'", MAX_SEC, self._name)
                self._finalize_recording_unlocked(reason=f"auto:max_{MAX_SEC}s")
                self._launch_preview_unlocked()
                return
            if MIN_FREE_MB:
                free = _free_mb(OUTDIR)
                if free is not None and free < MIN_FREE_MB:
                    rospy.logwarn("low disk %sMB < %sMB — auto-stopping '%s'", free, MIN_FREE_MB, self._name)
                    self._finalize_recording_unlocked(reason=f"auto:low_disk_{free}MB")
                    self._launch_preview_unlocked()

    def shutdown(self):
        with self._lock:
            if self._recording:
                self._finalize_recording_unlocked(reason="shutdown")
            else:
                self._kill_proc_unlocked(sigint=False)


def _watchdog_loop(node):
    r = rospy.Rate(1.0 / WATCHDOG_SEC)
    while not rospy.is_shutdown():
        try:
            node.watchdog_tick()
        except Exception as e:
            rospy.logwarn("watchdog error: %s", e)
        r.sleep()


def _master_watchdog():
    """The remote ROS master (Jetson roscore) restarts whenever the drone stack is
    relaunched, which silently ORPHANS this node: rospy stays bound to the dead
    master and never re-registers, so recording breaks with no error until a manual
    restart. Detect a master restart (its PID changes) and exit — systemd
    (Restart=always) then relaunches us onto the live master."""
    m = rospy.get_master()
    base = None
    while not rospy.is_shutdown():
        try:
            pid = m.getPid()[2]
            if base is None:
                base = pid
            elif pid != base:
                rospy.logwarn("ROS master restarted (pid %s->%s) — exiting to re-register", base, pid)
                rospy.signal_shutdown("ros master restarted")
                time.sleep(2)
                os._exit(1)
        except Exception:
            pass  # master briefly unreachable; keep polling until it (re)appears
        time.sleep(5)


def main():
    rospy.init_node(NODE_NAME)
    node = CameraNode()
    rospy.on_shutdown(node.shutdown)
    threading.Thread(target=_watchdog_loop, args=(node,), daemon=True).start()
    threading.Thread(target=_master_watchdog, daemon=True).start()  # re-register if Jetson roscore restarts
    rospy.loginfo("recorder node ready (services: ~start ~stop). codec=%s outdir=%s", CODEC, OUTDIR)
    rospy.spin()


if __name__ == "__main__":
    main()
