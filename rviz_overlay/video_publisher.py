#!/usr/bin/env python3
"""
video_publisher — republish the recorded webcam mp4 as a ROS image + CameraInfo,
time-aligned to the bag so rviz's Camera display can overlay the bag's markers
on it.

Frames are undistorted (rviz's Camera display assumes a rectified pinhole and
ignores distortion), so the published CameraInfo carries P=[K|0], D=0 and the
overlay is geometrically exact right to the edges.

Sim time (from `rosbag play --clock`) drives which frame is shown:
    frame_i = (sim_now - VIDEO_START_EPOCH - time_offset) * fps
so playback rate / pause / seek all stay in sync automatically.

Decode + undistort (~7-12ms/frame) and message serialization used to run
serially inside one rospy.Timer callback (~22ms/frame all-in), which caps the
publish rate at ~30Hz no matter the playback rate: at bag rate 2x the node
needs ~60Hz and the webcam image visibly freezes while the bag's markers keep
moving. OpenCV's C calls release the GIL, so decoding ahead of time on its own
thread genuinely overlaps the callback instead of just interleaving with it:
    producer thread : sequential cap.read() + cv2.remap(), handed off through
                       a small bounded queue as (frame_idx, bgr8 bytes).
    consumer thread : picks the newest queued frame at or before the sim-time
                       target and publishes it directly (no cv_bridge copy);
                       never blocks on decode.
Falling behind, or a `rosbag play --loop` rewind, is closed by the producer
thread using the same grab()-skip-vs-seek rule the old single-threaded reader
used (cheap forward catch-up, real seek only for a rewind or a big jump).

The consumer polls on a plain wall-clock thread instead of rospy.Timer. That
isn't cosmetic: rospy.Timer schedules against the *simulated* clock, i.e. it
only wakes on /clock updates, and `rosbag play` republishes /clock at a fixed
~50Hz in REAL time no matter what -r RATE is -- only the sim-time jump per
update gets bigger. So a sim-time timer fires in bursts glued to that 50Hz
cadence (several logical ticks back-to-back, then nothing until the next
update) instead of evenly spaced, and an 8.5MB frame takes ~17ms to actually
clear the TCP writer thread -- bursty delivery starves that writer and
collapses throughput to ~30Hz regardless of RATE or how fast decode is.
Polling wall-clock (still reading rospy.Time.now() for *which* frame to show)
gives the writer thread a steady stream of chances instead and recovers the
full rate; confirmed by isolating a bare 8.5MB publish (no decode at all)
against the real bag: rospy.Timer capped at ~30Hz at every RATE, a wall-clock
poll loop reached ~75Hz.

Run via overlay.launch (needs use_sim_time=true).
"""
import collections
import json
import os
import sys
import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import overlay_lib


class VideoPublisher:
    # Beyond this a keyframe seek is cheaper than decoding every intermediate frame.
    MAX_SKIP = 60
    # Producer lookahead: enough to absorb a hiccup without hoarding stale frames.
    QLEN = 8
    # Wall-clock poll rate for the consumer (see module docstring for why this
    # isn't a sim-time rospy.Timer). Comfortably oversamples fps*RATE for any
    # RATE up to ~6x so it never becomes the limiting factor itself; idle
    # iterations (no new target idx yet) are just a Time read + int compare.
    POLL_HZ = 200.0

    def __init__(self):
        # recording folder comes from $REC_DIR (set by the launcher)
        recdir = os.environ.get("REC_DIR") or rospy.get_param("~rec_dir", "")
        info = overlay_lib.find_recording(recdir)
        if not info["video"] or not info["extr"]:
            rospy.logerr("REC_DIR %s missing video/extrinsics", recdir)
            raise SystemExit(1)
        self.video_path = info["video"]
        self.start_epoch = info["start_epoch"]
        if self.start_epoch is None:
            rospy.logerr("no start time in recordings.csv for %s", info["name"])
            raise SystemExit(1)
        E = json.load(open(info["extr"]))
        self.K = np.array(E["K"], np.float64)
        self.D = np.array(E["dist"], np.float64)
        self.W, self.H = int(E["width"]), int(E["height"])
        self.offset = info["time_offset"]
        self.frame_id = rospy.get_param("~frame_id", overlay_lib.OPTICAL_FRAME)

        # Full 5MP raw is ~450MB/s @30Hz — too heavy for rviz to texture at rate,
        # so the image lags the markers during fast motion. Downscale (K scales
        # with it, so the overlay stays geometrically exact) to keep it live.
        self.scale = float(os.environ.get("OVERLAY_SCALE",
                                          rospy.get_param("~scale", 0.5)))
        self.OW, self.OH = int(round(self.W * self.scale)), int(round(self.H * self.scale))
        self.newK = self.K.copy()
        self.newK[:2, :] *= self.scale        # fx,fy,cx,cy scale; bottom row stays

        self.cap = cv2.VideoCapture(self.video_path)
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.nframes = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        # undistort+resize look-up maps (dst = newK, size = scaled); once, for speed
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, None, self.newK, (self.OW, self.OH), cv2.CV_16SC2)
        self.cur = -1              # producer: highest frame index decoded so far
        self.last_pub_idx = -1     # consumer: last frame index actually published

        # Producer/consumer handoff. `_buf` holds up to QLEN (idx, bgr8-bytes)
        # tuples oldest-first; `_jump_to`, when set by the timer callback, tells
        # the producer to reposition (flush + grab-skip or seek) instead of
        # stepping to the next sequential frame.
        self._cv = threading.Condition()
        self._buf = collections.deque()
        self._jump_to = None
        self._stop = False
        self._producer = threading.Thread(target=self._produce, daemon=True)
        self._consumer = threading.Thread(target=self._consume, daemon=True)

        self.pub_img = rospy.Publisher("/webcam/image_rect", Image, queue_size=2)
        self.pub_info = rospy.Publisher("/webcam/camera_info", CameraInfo, queue_size=2)
        self.info = self._make_info()
        rospy.loginfo("video_publisher: %d frames @ %.1ffps, offset %+.3fs, scale %.2f -> %dx%d",
                      self.nframes, self.fps, self.offset, self.scale, self.OW, self.OH)
        self._producer.start()
        self._consumer.start()
        rospy.on_shutdown(self._shutdown)

    def _make_info(self):
        ci = CameraInfo()
        ci.width, ci.height = self.OW, self.OH
        ci.distortion_model = "plumb_bob"
        ci.D = [0.0] * 5
        ci.K = list(self.newK.flatten())
        ci.R = list(np.eye(3).flatten())
        P = np.zeros((3, 4)); P[:3, :3] = self.newK
        ci.P = list(P.flatten())
        return ci

    def _shutdown(self):
        with self._cv:
            self._stop = True
            self._cv.notify_all()
        self._producer.join(timeout=1.0)
        self._consumer.join(timeout=1.0)

    def _advance(self, idx):
        """Move the capture to `idx`, returning the raw (distorted) frame or None.
        Forward gaps within MAX_SKIP are closed with cheap grab()-only skips
        (~2.5ms/frame): CAP_PROP_POS_FRAMES costs ~100ms, so once decoding falls
        one frame behind, seeking every step makes it fall further behind every
        step and collapses to ~10fps regardless of playback rate. Rewinds
        (rosbag --loop) and bigger jumps get a real seek instead."""
        gap = idx - self.cur
        if 0 < gap <= self.MAX_SKIP:
            for _ in range(gap - 1):
                self.cap.grab()
            ok, img = self.cap.read()
        else:                                   # rewind (looping) or a long jump
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, img = self.cap.read()
        self.cur = idx
        return (img if ok else None)

    def _produce(self):
        """Runs on its own thread: decode + undistort ahead of the consumer
        and hand finished frames off through a small bounded queue. cv2's grab/
        read/remap release the GIL, so this genuinely overlaps the consumer
        thread instead of just interleaving with it."""
        while True:
            with self._cv:
                while not self._stop and self._jump_to is None and len(self._buf) >= self.QLEN:
                    self._cv.wait(0.5)
                if self._stop:
                    return
                if self._jump_to is not None:
                    idx = self._jump_to
                    self._jump_to = None
                    self._buf.clear()
                else:
                    idx = self.cur + 1
                    if idx >= self.nframes:
                        self._cv.wait(0.1)     # at EOF; wait for a loop-restart jump
                        continue
            img = self._advance(idx)           # heavy OpenCV work, lock released
            if img is None:
                time.sleep(0.005)
                continue
            rect = cv2.remap(img, self.map1, self.map2, cv2.INTER_LINEAR)
            data = rect.tobytes()
            with self._cv:
                self._buf.append((self.cur, data))
                self._cv.notify_all()

    def _consume(self):
        """Wall-clock poll loop (see module docstring): decides which frame is
        due and publishes it. Deliberately not a rospy.Timer -- that schedules
        off the sim clock, which under `rosbag play -r RATE` only advances in
        bursts tied to /clock's fixed ~50Hz real-time update rate, and bursty
        delivery of an 8.5MB message collapses actual throughput."""
        period = 1.0 / self.POLL_HZ
        next_t = time.time()
        while not self._stop:
            self._tick()
            next_t += period
            delay = next_t - time.time()
            if delay > 0:
                time.sleep(delay)
            else:                          # fell behind wall-clock; resync
                next_t = time.time()

    def _tick(self):
        now = rospy.Time.now().to_sec()
        if now <= 0:
            return                       # waiting for first /clock
        target = int(round((now - self.start_epoch - self.offset) * self.fps))
        if target < 0 or target >= self.nframes or target == self.last_pub_idx:
            return
        frame = None
        with self._cv:
            notify = False
            while self._buf and self._buf[0][0] <= target:
                frame = self._buf.popleft()    # keep draining; only the newest matters
                notify = True                  # freed a slot -> wake a blocked producer
            # Producer hasn't reached `target` yet, or sim time jumped backwards
            # (rosbag --loop) or far ahead (a stall) of where it's decoding --
            # tell it to reposition; _advance() itself decides grab-skip vs seek.
            if self.cur < target or target < self.cur - self.MAX_SKIP:
                self._jump_to = target
                notify = True
            if notify:
                self._cv.notify_all()
        if frame is None:
            return
        idx, data = frame
        stamp = rospy.Time.now()
        # Direct Image construction (no cv_bridge) avoids an extra frame copy per
        # publish; the producer already froze the bytes once via tobytes().
        msg = Image()
        msg.height, msg.width = self.OH, self.OW
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = self.OW * 3
        msg.data = data
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.info.header.stamp = stamp
        self.info.header.frame_id = self.frame_id
        self.pub_img.publish(msg)
        self.pub_info.publish(self.info)
        self.last_pub_idx = idx


if __name__ == "__main__":
    rospy.init_node("webcam_video_publisher")
    VideoPublisher()
    rospy.spin()
