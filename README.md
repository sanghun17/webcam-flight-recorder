# Webcam flight recorder

HTTP-triggered recorder for the ABKO APC930 webcam on the `ml` PC (192.168.50.12).
The onboard flight PC sends start/stop over the network; this PC captures
`/dev/video0` and saves an H.264 MP4 per flight into `recordings/`.

## Run (foreground, for testing)

```bash
python3 /home/ml/webcam_recorder/recorder.py
```

## Trigger from the onboard PC (or anywhere on the network)

```bash
ML=192.168.50.12        # use the interface the onboard Docker host can reach
curl -X POST http://$ML:8088/start                 # begin
curl -X POST "http://$ML:8088/start?name=flight7"  # begin, custom label
curl -X POST http://$ML:8088/stop                  # finalize + save
curl http://$ML:8088/status                        # JSON state
```

`/start` while already recording returns HTTP 409 (no double-start).
`/stop` finalizes the MP4 cleanly (SIGINT → ffmpeg writes the moov atom).

## Run as a service (auto-start on boot, restart on crash)

```bash
sudo cp /home/ml/webcam_recorder/webcam-recorder.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now webcam-recorder
systemctl status webcam-recorder
journalctl -u webcam-recorder -f      # live logs
```

## Config (env vars; set in the service file or shell)

| var            | default      | notes                                  |
|----------------|--------------|----------------------------------------|
| CAM_DEVICE     | /dev/video0  | capture node                           |
| CAM_SIZE       | 1920x1080    | also valid: 2592x1944, 1280x720, ...   |
| CAM_FPS        | 30           |                                        |
| CAM_INPUT_FMT  | mjpeg        | camera native format                   |
| CAM_PORT       | 8088         | HTTP listen port                       |
| CAM_OUTDIR     | ./recordings | where MP4s land                        |
| CAM_PRESET     | veryfast     | x264 speed/size tradeoff               |
| CAM_MAX_SEC    | 1800         | auto-stop after N s (0=off). Guards a forgotten /stop. |
| CAM_MIN_FREE_MB| 2000         | refuse start / auto-stop below this free space (0=off) |
| CAM_ON_DUP     | rotate       | /start while recording: `rotate` (finalize + start new) or `reject` |

## Safety behavior (remote-trigger robustness)

- **Forgotten /stop** → the watchdog auto-stops & saves after `CAM_MAX_SEC`
  (default 30 min). The file is finalized normally; manifest `stop_reason`
  shows `auto:max_1800s`.
- **/start while already recording** (`CAM_ON_DUP=rotate`, default) → the
  current file is finalized and a new one begins. If the new name collides it
  is auto-suffixed: `flight7.mp4`, then `flight7_2.mp4`, `flight7_3.mp4` — so a
  repeated name re-saves instead of overwriting, and nothing is lost.
  Set `CAM_ON_DUP=reject` to instead refuse with HTTP 409 until `/stop`.
- **Low disk** → start is refused (HTTP 409) and an active recording is
  auto-stopped when free space drops below `CAM_MIN_FREE_MB`.
- Every recording (including auto-stopped ones) is appended to `recordings.csv`
  with a `stop_reason` column (`request` / `rotate` / `auto:...`).

Per-recording ffmpeg logs are written next to each MP4 (`*.ffmpeg.log`).

> The deployed unit runs the ROS 1 node `recorder_node.py` (single ffmpeg owns
> `/dev/video0`: always-on `sensor_msgs/CompressedImage` preview + on-demand
> recording via `std_srvs/Trigger` `~start` / `~stop`). See
> `webcam-recorder.service`. `recorder.py` is the standalone HTTP variant above.

---

# Odometry / rviz overlay toolkit

Overlay a flight's ROS bag (drone odometry, planner markers, trajectories) onto
the recorded webcam video, projected with the camera's calibrated pose — like an
rviz "Camera" view. The webcam is fixed and calibrated against the OptiTrack
world (`webcam_extrinsics.json`: `rvec/tvec` map a world point straight into the
camera), and the drone's mocap pose lives in that same world, so 3D geometry
projects exactly, with perspective.

Per flight you need, in `recordings/<name>/`: the recording `<name>.mp4`, the
matching `<name>.bag`, and `webcam_extrinsics.json` + an rviz config.

### Extrinsic recalibration (drone as a moving PnP target)

`calib_click.py` — click the drone across a dozen well-spread frames; it solves
the camera extrinsics **and** the video↔bag time offset (the mp4 starts ~0.7 s
after the csv `start` time due to camera warm-up) via `cv2.solvePnP`, writing
`webcam_extrinsics_clicked.json` (original preserved). Re-solve without
re-clicking: `--clicks clicks_N.json`.

### Quick projection check

```bash
python3 overlay_odom.py --extr webcam_extrinsics_clicked.json   # 10 sample PNGs
```

### Live rviz overlay (isolated)

```bash
rviz_overlay/run_overlay.sh          # or double-click recordings/<name>/start_rviz_overlay.sh
```
Runs a private roscore on loopback (never touches the online master / real
drone), publishes the undistorted webcam frames + `CameraInfo` + the extrinsic
as a static tf, and plays the bag so rviz overlays every enabled display.

### Render to mp4 (headless, full quality)

```bash
rviz_overlay/record_headless.sh      # or double-click recordings/<name>/record_overlay_mp4.sh
```
Records the real rviz overlay on a virtual display (Xvfb, nothing on screen),
playing the bag slowed so every frame renders, then retimes to real-time 30fps →
`recordings/<name>/overlay_rviz.mp4`. Needs `xvfb` installed.

Paths are currently hard-coded to the `safety_2026-06-30-14-17-18` sample; adjust
for other recordings.
