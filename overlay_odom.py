#!/usr/bin/env python3
"""
overlay_odom — project drone odometry from a ROS bag onto the static webcam
video as perspective-correct 3D arrows.

The webcam is fixed and calibrated against the OptiTrack world (see
webcam_extrinsics.json: rvec/tvec map an OptiTrack-world point straight into the
camera, so cv2.projectPoints does the whole pinhole+distortion projection). The
drone's mocap pose (/vrpn_client_node/pure/pose) lives in that same world, so we
just project a fixed-length 3D arrow (drone position -> body-forward axis) per
frame. Because the arrow has a constant 3D length, perspective makes it large
when the drone is near the camera and small when far — exactly like an rviz
camera view.

POC mode (default): sample 10 evenly-spaced frames, draw odometry only, dump PNGs
so the projection/time-sync can be eyeballed before committing to the full run.

Usage:
    python3 overlay_odom.py                 # 10-frame POC
    python3 overlay_odom.py --n 10 --offset 0.0
"""
import argparse
import json
import os

import cv2
import numpy as np
import rosbag

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.join(HERE, "recordings", "safety_2026-06-30-14-17-18")
BAG = os.path.join(REC, "safety_2026-06-30-14-17-18.bag")
VIDEO = os.path.join(REC, "safety_2026-06-30-14-17-18.mp4")
EXTR = os.path.join(HERE, "webcam_extrinsics.json")

# Video frame 0 wall-clock epoch. From recordings.csv start "2026-06-30 23:17:18"
# (local tz) == 1782829038.0; the bag starts 1.5 s later at 1782829039.50.
VIDEO_START_EPOCH = 1782829038.0

ODOM_TOPIC = "/vrpn_client_node/pure/pose"
ARROW_LEN_M = 0.4          # fixed 3D length of the heading arrow (metres)


def quat_to_R(x, y, z, w):
    """Rotation matrix from a (x,y,z,w) quaternion."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n == 0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def load_extrinsics(path):
    e = json.load(open(path))
    K = np.array(e["K"], dtype=np.float64)
    dist = np.array(e["dist"], dtype=np.float64)
    rvec = np.array(e["rvec"], dtype=np.float64).reshape(3, 1)
    tvec = np.array(e["tvec"], dtype=np.float64).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    return K, dist, rvec, tvec, R


def load_odom(bag_path, topic):
    """Return (times[N], pos[N,3], quat[N,4] xyzw) sorted by header stamp."""
    ts, pos, quat = [], [], []
    with rosbag.Bag(bag_path) as b:
        for _, m, _ in b.read_messages(topics=[topic]):
            ts.append(m.header.stamp.to_sec())
            p, o = m.pose.position, m.pose.orientation
            pos.append((p.x, p.y, p.z))
            quat.append((o.x, o.y, o.z, o.w))
    ts = np.array(ts)
    order = np.argsort(ts)
    return ts[order], np.array(pos)[order], np.array(quat)[order]


def nearest(ts, t):
    i = int(np.searchsorted(ts, t))
    if i <= 0:
        return 0
    if i >= len(ts):
        return len(ts) - 1
    return i if (ts[i] - t) < (t - ts[i - 1]) else i - 1


def draw_frame(img, K, dist, rvec, tvec, R, pos, R_body):
    """Draw the odometry marker + fixed-length body-forward arrow with
    perspective. Returns (annotated_img, depth_m) or (img, None) if off-frame."""
    forward = R_body @ np.array([1.0, 0.0, 0.0])   # body +x
    base = pos
    tip = pos + ARROW_LEN_M * forward

    depth = float((R @ base.reshape(3, 1) + tvec)[2])   # camera-frame Z (metres)
    if depth <= 0:
        return img, None   # behind the camera

    pts = np.array([base, tip], dtype=np.float64)
    proj, _ = cv2.projectPoints(pts, rvec, tvec, K, dist)
    (bx, by), (tx, ty) = proj.reshape(-1, 2)

    h, w = img.shape[:2]
    if not (0 <= bx < w and 0 <= by < h):
        return img, depth   # drone projects outside the image

    thick = int(np.clip(90.0 / depth, 3, 22))          # thicker when near
    tip_len = float(np.clip(0.18 * (thick / 6.0), 0.15, 0.5))
    cv2.arrowedLine(img, (int(bx), int(by)), (int(tx), int(ty)),
                    (0, 255, 0), thick, cv2.LINE_AA, tipLength=tip_len)
    cv2.circle(img, (int(bx), int(by)), max(thick, 6), (0, 128, 255), -1, cv2.LINE_AA)
    return img, depth


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10, help="number of frames to sample")
    ap.add_argument("--offset", type=float, default=0.0,
                    help="seconds added to video->bag time sync (tune if arrows lead/lag)")
    ap.add_argument("--outdir", default=os.path.join(REC, "overlay_poc"))
    ap.add_argument("--extr", default=EXTR, help="extrinsics json (use a calibrated one to verify)")
    args = ap.parse_args()

    K, dist, rvec, tvec, R = load_extrinsics(args.extr)
    # A calibrated extrinsics file carries the video<->bag time offset it was
    # solved with; fold it in so the odometry lines up without a manual --offset.
    args.offset += json.load(open(args.extr)).get("time_offset_sec", 0.0)
    print(f"extrinsics: {args.extr}  (time offset {args.offset:+.3f}s)")
    ts, pos, quat = load_odom(BAG, ODOM_TOPIC)
    print(f"loaded {len(ts)} odom samples, bag t=[{ts[0]:.2f},{ts[-1]:.2f}]")

    cap = cv2.VideoCapture(VIDEO)
    fps = cap.get(cv2.CAP_PROP_FPS)
    nframes = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # Only sample frames whose wall-clock falls inside the bag's time span.
    lo = int(np.ceil((ts[0] - VIDEO_START_EPOCH - args.offset) * fps))
    hi = int(np.floor((ts[-1] - VIDEO_START_EPOCH - args.offset) * fps))
    lo, hi = max(lo, 0), min(hi, nframes - 1)
    idxs = np.linspace(lo, hi, args.n).round().astype(int)
    print(f"sampling frames {list(idxs)} (covered range {lo}..{hi})")

    os.makedirs(args.outdir, exist_ok=True)
    # Frame indices shift when the time offset changes, so filenames carry the
    # frame number and old runs would otherwise linger and mix with the new
    # ones. Clear stale poc_*.png first so the folder only ever holds this run.
    import glob
    for f in glob.glob(os.path.join(args.outdir, "poc_*.png")):
        os.remove(f)

    for k, fi in enumerate(idxs):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(fi))
        ok, img = cap.read()
        if not ok:
            print(f"  frame {fi}: read failed")
            continue
        t_epoch = VIDEO_START_EPOCH + fi / fps + args.offset
        j = nearest(ts, t_epoch)
        R_body = quat_to_R(*quat[j])
        img, depth = draw_frame(img, K, dist, rvec, tvec, R, pos[j], R_body)

        dt = ts[j] - t_epoch
        label = (f"frame {fi}  t+{fi/fps:5.2f}s  drone=("
                 f"{pos[j][0]:.2f},{pos[j][1]:.2f},{pos[j][2]:.2f})  "
                 f"dist={depth:.2f}m  dsync={dt*1000:+.0f}ms")
        cv2.putText(img, label, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (0, 0, 0), 6, cv2.LINE_AA)
        cv2.putText(img, label, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (255, 255, 255), 2, cv2.LINE_AA)
        out = os.path.join(args.outdir, f"poc_{k:02d}_f{fi}.png")
        cv2.imwrite(out, img)
        print(f"  {out}  {label}")

    cap.release()
    print(f"\ndone -> {args.outdir}")


if __name__ == "__main__":
    main()
