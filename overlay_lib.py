#!/usr/bin/env python3
"""
overlay_lib — resolve everything the overlay tools need from a recording folder,
so the same scripts work for any recording (not just one hard-coded sample).

Given recordings/<name>/ it finds the bag, the recording video, the extrinsics
(prefers a per-recording webcam_extrinsics_clicked.json, else the base
webcam_extrinsics.json), and the rviz config; derives the video start epoch from
recordings.csv; and computes the odom->camera static transform.

CLI (used by the shell launchers):
    python3 overlay_lib.py tf   <recdir>     # -> "x y z qx qy qz qw odom webcam_optical"
    python3 overlay_lib.py show <recdir>     # -> human summary
    python3 overlay_lib.py rviz <recdir>     # generate .rviz_overlay/.rviz_capture configs
"""
import csv as _csv
import glob
import json
import os
import re
import sys
import time
from datetime import datetime

import numpy as np

DEFAULT_TIME_OFFSET = 0.73   # camera/ffmpeg warm-up; refine per-recording via calib_click
OPTICAL_FRAME = "webcam_optical"
WORLD_FRAME = "odom"
ODOM_TOPIC = "/vrpn_client_node/pure/pose"   # mocap pose, in the calibrated world

REPO = os.path.dirname(os.path.abspath(__file__))


def camera_tag(name):
    """camN from a flat label 'flight_<stamp>_cam2' OR a nested subfolder 'cam2',
    else None. Lets the tooling pick per-camera defaults (base extrinsics, bag)."""
    m = re.search(r"(?:^|_)(cam\d+)$", name)
    return m.group(1) if m else None


def load_odom(bag_path, topic=ODOM_TOPIC):
    """Return (times[N], pos[N,3], quat[N,4] xyzw) sorted by header stamp."""
    import rosbag
    ts, pos, quat = [], [], []
    with rosbag.Bag(bag_path) as b:
        for _, m, _ in b.read_messages(topics=[topic]):
            ts.append(m.header.stamp.to_sec())
            p, o = m.pose.position, m.pose.orientation
            pos.append((p.x, p.y, p.z)); quat.append((o.x, o.y, o.z, o.w))
    ts = np.array(ts); order = np.argsort(ts)
    return ts[order], np.array(pos)[order], np.array(quat)[order]


def nearest(ts, t):
    i = int(np.searchsorted(ts, t))
    if i <= 0:
        return 0
    if i >= len(ts):
        return len(ts) - 1
    return i if (ts[i] - t) < (t - ts[i - 1]) else i - 1
# outputs we must not mistake for the source recording video
_OUTPUT_VIDEOS = ("overlay_rviz.mp4", "overlay_full.mp4", "overlay_rviz_raw.mkv")


def _first(recdir, pats):
    for p in pats:
        for g in sorted(glob.glob(os.path.join(recdir, p))):
            if os.path.basename(g) not in _OUTPUT_VIDEOS:
                return g
    return None


def _sibling_bag(recdir, name):
    """The odometry bag is one-per-flight but the mp4 is one-per-camera, so a
    <flight>_cam2 folder holds only its own video. Fall back to a sibling
    <flight>_camN folder (same flight, same stamp) for the shared bag."""
    if "_cam" not in name:
        return None
    stem = name[:name.rfind("_cam")]
    for sib in sorted(glob.glob(os.path.join(os.path.dirname(recdir), stem + "_cam*"))):
        if os.path.abspath(sib) == recdir:
            continue
        b = _first(sib, [os.path.basename(sib) + ".bag", "*.bag"])
        if b:
            return b
    return None


def find_recording(recdir):
    recdir = os.path.abspath(recdir)
    dname = os.path.basename(recdir.rstrip("/"))
    video = _first(recdir, ["*.mp4", "*.mkv"])
    # The manifest keys by the full flight_<stamp>_camN label; a nested subfolder is
    # named just 'camN', so take the label from the mp4 basename when there is one.
    name = os.path.splitext(os.path.basename(video))[0] if video else dname
    tag = camera_tag(name) or camera_tag(dname)
    extr = None
    for c in ("webcam_extrinsics_clicked.json", "webcam_extrinsics.json"):
        if os.path.exists(os.path.join(recdir, c)):
            extr = os.path.join(recdir, c); break
    if extr is None and tag:   # no per-folder extrinsics yet -> this camera's base
        cand = os.path.join(REPO, "extrinsics", tag, "webcam_extrinsics.json")
        if os.path.exists(cand):
            extr = cand
    parent = os.path.dirname(recdir)
    # bag: this folder (flat) -> the parent flight folder (nested) -> a sibling cam
    bag = (_first(recdir, [name + ".bag", "*.bag"])
           or _first(parent, ["*.bag"])
           or _sibling_bag(recdir, dname))
    info = {
        "name": name,
        "dir": recdir,
        "bag": bag,
        "video": video,
        "extr": extr,
        "rviz": _first(recdir, ["rviz.rviz", "*.rviz"]) or _first(parent, ["rviz.rviz", "*.rviz"]),
    }
    info["start_epoch"] = video_start_epoch(name, recdir)
    info["time_offset"] = time_offset(extr)
    return info


def video_start_epoch(name, recdir):
    """Wall-clock epoch of video frame 0, from recordings.csv `start` (local tz).
    The manifest lives in the top recordings/ dir; walk up to find it (a nested cam
    folder sits two levels below it). Returns None if not found."""
    csvpath, d = None, recdir
    for _ in range(4):
        p = os.path.join(d, "recordings.csv")
        if os.path.exists(p):
            csvpath = p; break
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    if not csvpath:
        return None
    with open(csvpath) as f:
        for row in _csv.DictReader(f):
            if row.get("name") == name and row.get("start"):
                dt = datetime.strptime(row["start"], "%Y-%m-%d %H:%M:%S")
                return time.mktime(dt.timetuple())
    return None


def time_offset(extr_path):
    if extr_path and os.path.exists(extr_path):
        o = json.load(open(extr_path)).get("time_offset_sec")
        if o is not None:
            return float(o)
    return DEFAULT_TIME_OFFSET


def load_extrinsics(extr_path):
    import cv2
    E = json.load(open(extr_path))
    K = np.array(E["K"], np.float64)
    dist = np.array(E["dist"], np.float64)
    rvec = np.array(E["rvec"], np.float64).reshape(3, 1)
    tvec = np.array(E["tvec"], np.float64).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    return K, dist, rvec, tvec, R


def _mat2quat(M):
    t = np.trace(M)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(M[2, 1] - M[1, 2]) / s, (M[0, 2] - M[2, 0]) / s,
                         (M[1, 0] - M[0, 1]) / s, 0.25 * s])
    i = int(np.argmax([M[0, 0], M[1, 1], M[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(M[i, i] - M[j, j] - M[k, k] + 1) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (M[j, i] + M[i, j]) / s
    q[k] = (M[k, i] + M[i, k]) / s
    q[3] = (M[k, j] - M[j, k]) / s
    return q   # x,y,z,w


def static_tf(extr_path):
    """odom->webcam_optical: (x,y,z, qx,qy,qz,qw)."""
    import cv2
    _, _, rvec, tvec, R = load_extrinsics(extr_path)
    cam_pos = (-R.T @ tvec).reshape(3)
    q = _mat2quat(R.T)
    return list(cam_pos) + list(q)


def _add_camera_display(cfg):
    disp = cfg["Visualization Manager"]["Displays"]
    disp[:] = [d for d in disp if d.get("Name") != "Webcam_Overlay"]
    disp.append({"Class": "rviz/Camera", "Enabled": True,
                 "Image Topic": "/webcam/image_rect", "Name": "Webcam_Overlay",
                 "Overlay Alpha": 0.6, "Queue Size": 2, "Transport Hint": "raw",
                 "Unreliable": False, "Value": True, "Zoom Factor": 1})


def build_configs(recdir):
    """From the folder's rviz.rviz make .rviz_overlay.rviz (adds Camera display)
    and .rviz_capture.rviz (also strips side panels + empty Image docks)."""
    import yaml
    info = find_recording(recdir)
    base = info["rviz"]
    if not base:
        return None, None
    over = os.path.join(recdir, ".rviz_overlay.rviz")
    cap = os.path.join(recdir, ".rviz_capture.rviz")

    cfg = yaml.safe_load(open(base))
    _add_camera_display(cfg)
    yaml.safe_dump(cfg, open(over, "w"), default_flow_style=False, sort_keys=False)

    cfg = yaml.safe_load(open(base))
    _add_camera_display(cfg)
    cfg["Panels"] = []

    def strip(ds):
        out = []
        for d in ds:
            if d.get("Class") == "rviz/Image":
                continue
            if isinstance(d.get("Displays"), list):
                d["Displays"] = strip(d["Displays"])
            out.append(d)
        return out
    vm = cfg["Visualization Manager"]
    vm["Displays"] = strip(vm["Displays"])
    # Force the layout where the Webcam_Overlay Camera panel FLOATS as its own
    # window (so the headless recorder can grab it by name). A saved config's
    # QMainWindow State otherwise docks the new display and it has no X window.
    wg = _float_window_geometry()
    if wg:
        cfg["Window Geometry"] = wg
    yaml.safe_dump(cfg, open(cap, "w"), default_flow_style=False, sort_keys=False)
    return over, cap


def _float_window_geometry():
    """A known-good Window Geometry whose QMainWindow State floats Webcam_Overlay
    as a separate top-level window (taken from rviz_overlay/rviz_capture.rviz)."""
    import yaml
    tmpl = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "rviz_overlay", "rviz_capture.rviz")
    try:
        wg = yaml.safe_load(open(tmpl)).get("Window Geometry")
        return wg if (wg and "Webcam_Overlay" in wg) else None
    except OSError:
        return None


def _cli():
    cmd, recdir = sys.argv[1], sys.argv[2]
    info = find_recording(recdir)
    if cmd == "tf":
        if not info["extr"]:
            sys.exit("no extrinsics in " + recdir)
        v = static_tf(info["extr"])
        print(" ".join(f"{x:.6f}" for x in v) + f" {WORLD_FRAME} {OPTICAL_FRAME}")
    elif cmd == "rviz":
        over, cap = build_configs(recdir)
        print(over or "", cap or "")
    elif cmd == "show":
        for k in ("name", "bag", "video", "extr", "rviz", "start_epoch", "time_offset"):
            print(f"  {k:12s}: {info[k]}")
        miss = [k for k in ("bag", "video", "extr", "rviz") if not info[k]]
        if miss:
            print("  MISSING:", ", ".join(miss))
    else:
        sys.exit("usage: overlay_lib.py {tf|rviz|show} <recdir>")


if __name__ == "__main__":
    _cli()
