#!/usr/bin/env python3
"""
Webcam flight recorder — HTTP-triggered.

Listens on :8088. The onboard flight PC starts/stops recording over the network:

    curl -X POST http://192.168.50.12:8088/start          # begin
    curl -X POST 'http://192.168.50.12:8088/start?name=flight7'
    curl -X POST http://192.168.50.12:8088/stop           # finalize + save
    curl http://192.168.50.12:8088/status                 # JSON state

Captures /dev/video0 (MJPEG) and encodes H.264 MP4 into ./recordings/.
No external Python deps — stdlib only. Config via env vars (see CONFIG below).
"""
import csv
import json
import os
import signal
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

# ---- CONFIG (override with env vars) ----------------------------------------
DEVICE     = os.environ.get("CAM_DEVICE", "/dev/video0")
VIDEO_SIZE = os.environ.get("CAM_SIZE", "1920x1080")
FRAMERATE  = os.environ.get("CAM_FPS", "30")
INPUT_FMT  = os.environ.get("CAM_INPUT_FMT", "mjpeg")   # camera native pixel format
OUTDIR     = os.environ.get("CAM_OUTDIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings"))
PORT       = int(os.environ.get("CAM_PORT", "8088"))
HOST       = os.environ.get("CAM_HOST", "0.0.0.0")
PRESET     = os.environ.get("CAM_PRESET", "veryfast")   # x264 speed/size tradeoff
CRF        = os.environ.get("CAM_CRF", "20")            # x264 quality: lower=sharper/bigger (18=near-lossless, 23=default)
# ---- safety nets ----
MAX_SEC      = int(os.environ.get("CAM_MAX_SEC", "1800"))      # auto-stop after N s (0=off). Guards forgotten /stop.
MIN_FREE_MB  = int(os.environ.get("CAM_MIN_FREE_MB", "2000"))  # refuse/auto-stop below this free space (0=off)
ON_DUP       = os.environ.get("CAM_ON_DUP", "rotate").lower()  # /start while recording: rotate | reject
WATCHDOG_SEC = 2                                               # how often the watchdog checks (bounds overshoot)
# ---- camera controls (applied via v4l2-ctl right before each recording) ----
# Fixes the autofocus "hunting" (blurry↔sharp) by locking focus, and pins
# exposure/white-balance/power-line-freq so the image stops drifting.
FOCUS_AUTO    = os.environ.get("CAM_FOCUS_AUTO", "0")    # 1=continuous AF on, 0=off (manual, no hunting)
FOCUS_ABS     = os.environ.get("CAM_FOCUS", "")          # manual focus 0..150 ('' = leave current)
AUTO_EXPOSURE = os.environ.get("CAM_AUTO_EXPOSURE", "")  # '' leave, 1=manual, 3=auto(aperture priority)
EXPOSURE_ABS  = os.environ.get("CAM_EXPOSURE", "")       # exposure_time_absolute 3..2047 (needs auto_exposure=1)
AUTO_WB       = os.environ.get("CAM_AUTO_WB", "")        # '' leave, 1=auto, 0=manual white balance
WB_TEMP       = os.environ.get("CAM_WB_TEMP", "")        # white_balance_temperature 2800..6500 (needs auto_wb=0)
BRIGHTNESS    = os.environ.get("CAM_BRIGHTNESS", "")     # -64..64 ('' = leave)
POWER_FREQ    = os.environ.get("CAM_POWER_FREQ", "2")    # 0=off 1=50Hz 2=60Hz (Korea=60)
SHARPNESS     = os.environ.get("CAM_SHARPNESS", "")      # 1..7 ('' = leave)
EXTRA_CTRLS   = os.environ.get("CAM_CTRLS", "")          # freeform extra, e.g. "brightness=10,contrast=40"
# -----------------------------------------------------------------------------

os.makedirs(OUTDIR, exist_ok=True)

MANIFEST = os.path.join(OUTDIR, "recordings.csv")
MANIFEST_COLS = ["name", "file", "start", "end", "duration_sec", "size_bytes", "size_mb", "stop_reason"]


def _safe(s):
    return "".join(c for c in s if c.isalnum() or c in "-_")[:40] or "flight"


def _tail(p, n=400):
    try:
        with open(p, "rb") as f:
            return f.read()[-n:].decode("utf-8", "replace").strip()
    except OSError:
        return "(no log)"


def _ts(dt):
    return dt.strftime("%Y-%m-%d %H:%M:%S")


def _free_mb(path):
    try:
        st = os.statvfs(path)
        return int(st.f_bavail * st.f_frsize / 1e6)
    except OSError:
        return None


def _apply_camera_controls():
    """Pin camera controls via v4l2-ctl so the image stops hunting. AF must be
    turned off first, otherwise focus_absolute stays inactive. Best-effort:
    a failure here is logged but never blocks recording."""
    # Stage 1: disable the auto modes FIRST, in their own ioctl. Setting an
    # absolute value in the same atomic call as its auto-toggle gets ignored by
    # this driver (e.g. exposure_time stays at the auto value), so they must be
    # separate calls — manual mode active before the value is written.
    first = [f"focus_automatic_continuous={FOCUS_AUTO}"]
    if AUTO_EXPOSURE != "": first.append(f"auto_exposure={AUTO_EXPOSURE}")
    if AUTO_WB != "":       first.append(f"white_balance_automatic={AUTO_WB}")
    # Stage 2: the dependent absolute values + independent controls.
    rest = []
    if FOCUS_ABS != "":     rest.append(f"focus_absolute={FOCUS_ABS}")
    if EXPOSURE_ABS != "":  rest.append(f"exposure_time_absolute={EXPOSURE_ABS}")
    if WB_TEMP != "":       rest.append(f"white_balance_temperature={WB_TEMP}")
    if BRIGHTNESS != "":    rest.append(f"brightness={BRIGHTNESS}")
    if POWER_FREQ != "":    rest.append(f"power_line_frequency={POWER_FREQ}")
    if SHARPNESS != "":     rest.append(f"sharpness={SHARPNESS}")
    if EXTRA_CTRLS:         rest.append(EXTRA_CTRLS)
    applied = []
    for group in (first, rest):
        if not group:
            continue
        try:
            subprocess.run(["v4l2-ctl", "-d", DEVICE, "-c", ",".join(group)],
                           check=True, capture_output=True, text=True, timeout=5)
            applied += group
        except FileNotFoundError:
            _log("WARN v4l2-ctl not installed — camera controls skipped")
            return
        except subprocess.CalledProcessError as e:
            _log(f"WARN camera control failed ({','.join(group)}): {e.stderr.strip()}")
        except subprocess.TimeoutExpired:
            _log("WARN v4l2-ctl timed out applying camera controls")
    if applied:
        _log(f"         controls: {', '.join(applied)}")


def _unique_label(label):
    """Return label, or label_2 / label_3 ... if an .mp4 already exists.
    Guarantees the new recording never overwrites an earlier one."""
    if not os.path.exists(os.path.join(OUTDIR, label + ".mp4")):
        return label
    n = 2
    while os.path.exists(os.path.join(OUTDIR, f"{label}_{n}.mp4")):
        n += 1
    return f"{label}_{n}"


def _log(msg):
    print(f"[{_ts(datetime.now())}] {msg}", flush=True)


def _append_manifest(row):
    """Append one recording's summary to recordings.csv (writes header once)."""
    new = not os.path.exists(MANIFEST)
    try:
        with open(MANIFEST, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=MANIFEST_COLS)
            if new:
                w.writeheader()
            w.writerow(row)
    except OSError as e:
        _log(f"WARN could not write manifest: {e}")


class Recorder:
    """Owns at most one ffmpeg process. Thread-safe."""

    def __init__(self):
        self._lock = threading.Lock()
        self._proc = None
        self._path = None
        self._logf = None
        self._started_at = None
        self._name = None

    def is_running(self):
        return self._proc is not None and self._proc.poll() is None

    def start(self, name=None):
        with self._lock:
            # The name sent with /start IS the filename. Fall back to a timestamp
            # only if no name is given, so a recording is never lost.
            if name:
                label = _safe(name)
            else:
                label = "flight_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

            # --- /start while already recording ---
            # A start arriving mid-recording means the previous flight never got
            # a /stop. Finalize the current file and begin a new one. The new
            # name gets an auto-suffix if it collides, so no data is lost or
            # overwritten (e.g. flight7 → flight7.mp4, again → flight7_2.mp4).
            if self.is_running():
                if ON_DUP == "rotate":
                    _log(f"↻ ROTATE on /start: finalizing '{self._name}', starting '{label}'")
                    self._stop_unlocked(reason="rotate")
                else:  # reject
                    return False, f"already recording '{self._name}' (send /stop first)", self._info_unlocked()

            # --- disk-space guard ---
            free = _free_mb(OUTDIR)
            if MIN_FREE_MB and free is not None and free < MIN_FREE_MB:
                return False, f"low disk: {free}MB free < {MIN_FREE_MB}MB min — not starting", None

            return self._start_unlocked(label)

    def _start_unlocked(self, label):
        # Auto-suffix on collision so a repeat name re-saves as name_2, name_3...
        base = _unique_label(label)
        self._path = os.path.join(OUTDIR, base + ".mp4")
        self._name = base
        self._started_at = time.time()
        self._started_dt = datetime.now()

        # Lock focus/exposure/etc. before ffmpeg opens the device.
        _apply_camera_controls()

        if True:
            cmd = [
                "ffmpeg", "-hide_banner", "-loglevel", "warning", "-nostdin",
                "-f", "v4l2",
                "-input_format", INPUT_FMT,
                "-video_size", VIDEO_SIZE,
                "-framerate", FRAMERATE,
                "-i", DEVICE,
                "-c:v", "libx264", "-preset", PRESET, "-crf", CRF, "-pix_fmt", "yuv420p",
                "-movflags", "+faststart",
                self._path,
            ]
            # -nostdin keeps ffmpeg from eating our stdin; we stop via SIGINT
            # so the MP4 trailer (moov atom) is written cleanly.
            self._logf = open(os.path.join(OUTDIR, base + ".ffmpeg.log"), "wb")
            try:
                self._proc = subprocess.Popen(
                    cmd, stdout=self._logf, stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except FileNotFoundError:
                self._logf.close()
                self._proc = None
                return False, "ffmpeg not installed", None

            # Give ffmpeg a moment; if it dies instantly the device/args are bad.
            time.sleep(1.0)
            if not self.is_running():
                self._logf.flush()
                tail = _tail(self._path.replace(".mp4", ".ffmpeg.log"))
                self._proc = None
                return False, f"ffmpeg exited immediately: {tail}", None

            note = "" if self._name == label else f" (requested '{label}', auto-suffixed)"
            _log(f"▶ START  name={self._name}{note}")
            _log(f"         file={self._path}")
            _log(f"         start={_ts(self._started_dt)}  {VIDEO_SIZE}@{FRAMERATE}fps {INPUT_FMT}")
            return True, "recording started", self._info_unlocked()

    def stop(self):
        with self._lock:
            return self._stop_unlocked(reason="request")

    def _stop_unlocked(self, reason="request"):
        if not self.is_running():
            return False, "not recording", None
        path, name = self._path, self._name
        start_dt = self._started_dt
        dur = time.time() - self._started_at
        proc = self._proc

        # SIGINT == graceful: ffmpeg finalizes the file.
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

        if self._logf:
            self._logf.close()
            self._logf = None
        self._proc = None

        end_dt = datetime.now()
        size = os.path.getsize(path) if os.path.exists(path) else 0
        size_mb = round(size / 1e6, 1)
        dur = round(dur, 1)
        info = {
            "file": path,
            "name": name,
            "start": _ts(start_dt),
            "end": _ts(end_dt),
            "duration_sec": dur,
            "size_bytes": size,
            "size_mb": size_mb,
            "stop_reason": reason,
        }
        _append_manifest({
            "name": name, "file": path,
            "start": _ts(start_dt), "end": _ts(end_dt),
            "duration_sec": dur, "size_bytes": size, "size_mb": size_mb,
            "stop_reason": reason,
        })
        tag = "" if reason == "request" else f"  [{reason}]"
        _log(f"■ STOP{tag}   name={name}  duration={dur}s  size={size_mb}MB")
        _log(f"         file={path}")
        _log(f"         start={_ts(start_dt)}  end={_ts(end_dt)}")
        _log(f"         logged → {MANIFEST}")
        return True, "recording stopped", info

    def watchdog_tick(self):
        """Called periodically by a background thread. Enforces the max-duration
        cap and disk-space floor so a missing /stop can't run forever."""
        with self._lock:
            if not self.is_running():
                return
            elapsed = time.time() - self._started_at
            if MAX_SEC and elapsed >= MAX_SEC:
                _log(f"⏱ max duration {MAX_SEC}s reached — auto-stopping '{self._name}'")
                self._stop_unlocked(reason=f"auto:max_{MAX_SEC}s")
                return
            if MIN_FREE_MB:
                free = _free_mb(OUTDIR)
                if free is not None and free < MIN_FREE_MB:
                    _log(f"⚠ low disk {free}MB < {MIN_FREE_MB}MB — auto-stopping '{self._name}'")
                    self._stop_unlocked(reason=f"auto:low_disk_{free}MB")

    def status(self):
        with self._lock:
            return self._info_unlocked()

    def _info_unlocked(self):
        running = self.is_running()
        info = {"recording": running}
        if running:
            info.update({
                "file": self._path,
                "name": self._name,
                "start": _ts(self._started_dt),
                "elapsed_sec": round(time.time() - self._started_at, 1),
            })
        return info


REC = Recorder()


class Handler(BaseHTTPRequestHandler):
    server_version = "WebcamRecorder/1.0"

    def _reply(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _route(self):
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/") or "/"
        qs = parse_qs(parsed.query)

        if path == "/start":
            name = qs.get("name", [None])[0]
            ok, msg, info = REC.start(name)
            self._reply(200 if ok else 409, {"ok": ok, "msg": msg, "info": info})
        elif path == "/stop":
            ok, msg, info = REC.stop()
            self._reply(200 if ok else 409, {"ok": ok, "msg": msg, "info": info})
        elif path == "/status":
            self._reply(200, {"ok": True, "info": REC.status()})
        elif path == "/":
            self._reply(200, {"ok": True, "msg": "webcam recorder",
                              "endpoints": ["/start[?name=]", "/stop", "/status"],
                              "device": DEVICE, "size": VIDEO_SIZE, "fps": FRAMERATE,
                              "outdir": OUTDIR})
        else:
            self._reply(404, {"ok": False, "msg": "unknown endpoint"})

    # Accept both GET and POST so plain `curl` works too.
    do_GET = _route
    do_POST = _route

    def log_message(self, fmt, *args):
        print("[%s] %s" % (self.log_date_time_string(), fmt % args), flush=True)


def _watchdog_loop():
    while True:
        time.sleep(WATCHDOG_SEC)
        try:
            REC.watchdog_tick()
        except Exception as e:  # never let the watchdog thread die
            _log(f"watchdog error: {e}")


def main():
    httpd = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"webcam recorder listening on {HOST}:{PORT}", flush=True)
    print(f"  device={DEVICE} size={VIDEO_SIZE} fps={FRAMERATE} fmt={INPUT_FMT}", flush=True)
    print(f"  outdir={OUTDIR}", flush=True)
    print(f"  encode: libx264 preset={PRESET} crf={CRF}", flush=True)
    print(f"  camera: af_continuous={FOCUS_AUTO} focus={FOCUS_ABS or 'as-is'} "
          f"auto_exposure={AUTO_EXPOSURE or 'as-is'} power_freq={POWER_FREQ or 'as-is'}", flush=True)
    print(f"  safety: max_sec={MAX_SEC or 'off'} min_free_mb={MIN_FREE_MB or 'off'} on_dup={ON_DUP}", flush=True)

    threading.Thread(target=_watchdog_loop, daemon=True).start()

    def _shutdown(*_):
        print("shutting down; stopping any active recording...", flush=True)
        REC.stop()
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    httpd.serve_forever()
    print("stopped", flush=True)


if __name__ == "__main__":
    main()
