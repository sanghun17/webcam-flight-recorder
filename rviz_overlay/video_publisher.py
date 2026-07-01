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

Run via overlay.launch (needs use_sim_time=true).
"""
import json
import os
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import overlay_lib


class VideoPublisher:
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
        self.cur = -1

        self.bridge = CvBridge()
        self.pub_img = rospy.Publisher("/webcam/image_rect", Image, queue_size=2)
        self.pub_info = rospy.Publisher("/webcam/camera_info", CameraInfo, queue_size=2)
        self.info = self._make_info()
        rospy.loginfo("video_publisher: %d frames @ %.1ffps, offset %+.3fs, scale %.2f -> %dx%d",
                      self.nframes, self.fps, self.offset, self.scale, self.OW, self.OH)
        rospy.Timer(rospy.Duration(1.0 / self.fps), self._tick)

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

    def _read(self, idx):
        if idx == self.cur + 1:
            ok, img = self.cap.read()
        else:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, img = self.cap.read()
        self.cur = idx
        return (img if ok else None)

    def _tick(self, _evt):
        now = rospy.Time.now().to_sec()
        if now <= 0:
            return                       # waiting for first /clock
        idx = int(round((now - self.start_epoch - self.offset) * self.fps))
        if idx < 0 or idx >= self.nframes or idx == self.cur:
            return
        img = self._read(idx)
        if img is None:
            return
        rect = cv2.remap(img, self.map1, self.map2, cv2.INTER_LINEAR)
        stamp = rospy.Time.now()
        msg = self.bridge.cv2_to_imgmsg(rect, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.info.header.stamp = stamp
        self.info.header.frame_id = self.frame_id
        self.pub_img.publish(msg)
        self.pub_info.publish(self.info)


if __name__ == "__main__":
    rospy.init_node("webcam_video_publisher")
    VideoPublisher()
    rospy.spin()
