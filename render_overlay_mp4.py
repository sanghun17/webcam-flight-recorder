#!/usr/bin/env python3
"""
render_overlay_mp4 — offline, full-resolution overlay of the bag's rviz markers
onto the webcam video, projected with the calibrated extrinsics + distortion +
time offset. This is the "full power" render (no real-time constraint), matching
what the live rviz Camera display shows but at 5MP and distortion-exact.

Which displays are drawn is read straight from the rviz config, so it stays in
sync with your visualization. Markers carry their own type/color/scale/points,
so they render as rviz shows them.

    python3 render_overlay_mp4.py --poc            # 10 sample PNGs (verify first)
    python3 render_overlay_mp4.py                  # full mp4
    python3 render_overlay_mp4.py --start 10 --dur 8   # a slice
"""
import argparse
import json
import os

import cv2
import numpy as np
import rosbag
import yaml

import overlay_odom as oo

REC = oo.REC
BAG = oo.BAG
VIDEO = oo.VIDEO
RVIZ_CFG = os.path.join(REC, "rviz.rviz")
EXTR_CLICKED = os.path.join(os.path.dirname(oo.EXTR), "webcam_extrinsics_clicked.json")

# rviz display class -> the config key holding its topic
TOPIC_KEY = {
    "rviz/MarkerArray": "Marker Topic",
    "rviz/Marker": "Marker Topic",
    "rviz/Pose": "Topic",
    "rviz/Odometry": "Topic",
    "rviz/Path": "Topic",
    "rviz/PoseArray": "Topic",
}
POSE_LIKE = {"rviz/Pose", "rviz/Odometry"}   # drawn as a single arrow


def load_extrinsics():
    E = json.load(open(EXTR_CLICKED))
    K = np.array(E["K"]); dist = np.array(E["dist"])
    rvec = np.array(E["rvec"]).reshape(3, 1); tvec = np.array(E["tvec"]).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    return K, dist, rvec, tvec, R, E.get("time_offset_sec", 0.0)


def collect_display_topics(cfg_path):
    """Walk the rviz config tree; return enabled (class, topic, name, opts)."""
    cfg = yaml.safe_load(open(cfg_path))
    out = []

    def walk(node):
        if isinstance(node, dict):
            cls = node.get("Class", "")
            if cls in TOPIC_KEY and node.get("Enabled", True):
                topic = node.get(TOPIC_KEY[cls], "")
                if topic:
                    out.append((cls, topic, node.get("Name", topic), node))
            for v in node.values():
                walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)
    walk(cfg["Visualization Manager"]["Displays"])
    return out


class Projector:
    def __init__(self, K, dist, rvec, tvec, R):
        self.K, self.dist, self.rvec, self.tvec, self.R = K, dist, rvec, tvec, R
        self.f = 0.5 * (K[0, 0] + K[1, 1])

    def project(self, P3):
        P3 = np.asarray(P3, np.float64).reshape(-1, 3)
        zc = (self.R @ P3.T + self.tvec)[2]
        uv, _ = cv2.projectPoints(P3, self.rvec, self.tvec, self.K, self.dist)
        return uv.reshape(-1, 2), zc

    def rpx(self, r_m, z):                 # metres -> pixels at depth z
        return max(1, int(round(self.f * r_m / max(z, 1e-3))))


def bgr(c):
    return (int(c.b * 255), int(c.g * 255), int(c.r * 255))


def pt_colors(mk):
    if mk.colors and len(mk.colors) == len(mk.points):
        return [bgr(c) for c in mk.colors]
    return None


def blend_roi(img, layer, alpha, pts):
    """Alpha-composite `layer` onto `img` only within the bbox of `pts`."""
    if alpha >= 0.99:
        return
    h, w = img.shape[:2]
    xs = [int(p[0]) for p in pts]; ys = [int(p[1]) for p in pts]
    x0, x1 = max(min(xs) - 5, 0), min(max(xs) + 5, w)
    y0, y1 = max(min(ys) - 5, 0), min(max(ys) + 5, h)
    if x1 <= x0 or y1 <= y0:
        return
    roi = slice(y0, y1), slice(x0, x1)
    cv2.addWeighted(layer[roi], alpha, img[roi], 1 - alpha, 0, img[roi])


CUBE_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
              (0, 4), (1, 5), (2, 6), (3, 7)]


def draw_marker(img, proj, mk):
    a = mk.color.a if mk.color.a > 0 else 1.0
    col = bgr(mk.color)
    t = mk.type
    layer = img if a >= 0.99 else img.copy()

    if t in (4, 5):                                   # LINE_STRIP / LINE_LIST
        if len(mk.points) < 2:
            return
        P = np.array([[p.x, p.y, p.z] for p in mk.points])
        uv, zc = proj.project(P)
        w = max(1, proj.rpx(mk.scale.x, np.median(zc[zc > 0]) if (zc > 0).any() else 1))
        pc = pt_colors(mk)
        segs = ([(2 * i, 2 * i + 1) for i in range(len(mk.points) // 2)] if t == 5
                else [(i, i + 1) for i in range(len(mk.points) - 1)])
        polylines = []
        for i, j in segs:
            if zc[i] <= 0 or zc[j] <= 0:
                continue
            if pc:
                cv2.line(layer, tuple(uv[i].astype(int)), tuple(uv[j].astype(int)),
                         pc[i], w, cv2.LINE_AA)
            else:
                polylines.append(uv[[i, j]].astype(np.int32))
        if polylines:
            cv2.polylines(layer, polylines, False, col, w, cv2.LINE_AA)
        blend_roi(img, layer, a, uv[zc > 0])

    elif t in (7, 8, 6):                              # SPHERE_LIST/POINTS/CUBE_LIST
        if not mk.points:
            return
        P = np.array([[p.x, p.y, p.z] for p in mk.points])
        uv, zc = proj.project(P)
        pc = pt_colors(mk)
        vis = []
        for i in range(len(P)):
            if zc[i] <= 0:
                continue
            r = proj.rpx(max(mk.scale.x, 0.02) / 2, zc[i])
            cv2.circle(layer, tuple(uv[i].astype(int)), r, pc[i] if pc else col, -1, cv2.LINE_AA)
            vis.append(uv[i])
        if vis:
            blend_roi(img, layer, a, vis)

    elif t in (1, 2, 3):                              # CUBE / SPHERE / CYLINDER
        c = mk.pose.position
        uv, zc = proj.project([[c.x, c.y, c.z]])
        if zc[0] <= 0:
            return
        if t == 1 and (mk.scale.x > 0.5 or mk.scale.y > 0.5):   # big box -> wireframe
            hx, hy, hz = mk.scale.x / 2, mk.scale.y / 2, mk.scale.z / 2
            corners = np.array([[sx * hx, sy * hy, sz * hz]
                                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            # reorder to a proper box corner sequence
            order = [0, 1, 3, 2, 4, 5, 7, 6]
            cw = np.array([c.x, c.y, c.z]) + corners[order]
            cuv, czc = proj.project(cw)
            for i, j in CUBE_EDGES:
                if czc[i] > 0 and czc[j] > 0:
                    cv2.line(layer, tuple(cuv[i].astype(int)), tuple(cuv[j].astype(int)),
                             col, 2, cv2.LINE_AA)
            blend_roi(img, layer, a, cuv[czc > 0])
        else:
            r = proj.rpx(max(mk.scale.x, 0.03) / 2, zc[0])
            cv2.circle(layer, tuple(uv[0].astype(int)), r, col, -1, cv2.LINE_AA)
            blend_roi(img, layer, a, [uv[0] + [r, r], uv[0] - [r, r]])

    elif t == 0:                                      # ARROW
        if len(mk.points) >= 2:
            s, e = mk.points[0], mk.points[1]
            P = np.array([[s.x, s.y, s.z], [e.x, e.y, e.z]])
        else:
            c, q = mk.pose.position, mk.pose.orientation
            Rq = _quat_R(q.x, q.y, q.z, q.w)
            fwd = Rq @ np.array([max(mk.scale.x, 0.2), 0, 0])
            P = np.array([[c.x, c.y, c.z], [c.x + fwd[0], c.y + fwd[1], c.z + fwd[2]]])
        uv, zc = proj.project(P)
        if (zc > 0).all():
            w = max(2, proj.rpx(max(mk.scale.y, 0.02), zc.mean()))
            cv2.arrowedLine(layer, tuple(uv[0].astype(int)), tuple(uv[1].astype(int)),
                            col, w, cv2.LINE_AA, tipLength=0.3)
            blend_roi(img, layer, a, uv)

    elif t == 9 and mk.text:                          # TEXT
        c = mk.pose.position
        uv, zc = proj.project([[c.x, c.y, c.z]])
        if zc[0] > 0:
            cv2.putText(img, mk.text, tuple(uv[0].astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, col, 2, cv2.LINE_AA)


def _quat_R(x, y, z, w):
    n = (x * x + y * y + z * z + w * w) ** 0.5 or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def draw_pose(img, proj, msg, col=(0, 0, 255)):
    p = msg.pose.position if hasattr(msg, "pose") else msg.pose.pose.position
    o = (msg.pose.orientation if hasattr(msg, "pose") and hasattr(msg.pose, "orientation")
         else msg.pose.pose.orientation)
    R = _quat_R(o.x, o.y, o.z, o.w)
    fwd = R @ np.array([0.4, 0, 0])
    P = np.array([[p.x, p.y, p.z], [p.x + fwd[0], p.y + fwd[1], p.z + fwd[2]]])
    uv, zc = proj.project(P)
    if (zc > 0).all():
        cv2.arrowedLine(img, tuple(uv[0].astype(int)), tuple(uv[1].astype(int)),
                        col, max(2, proj.rpx(0.03, zc.mean())), cv2.LINE_AA, tipLength=0.3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poc", action="store_true", help="render 10 sample PNGs instead of mp4")
    ap.add_argument("--start", type=float, default=0.0, help="start seconds into video")
    ap.add_argument("--dur", type=float, default=None, help="duration seconds (default: to end)")
    ap.add_argument("--out", default=os.path.join(REC, "overlay_full.mp4"))
    args = ap.parse_args()

    K, dist, rvec, tvec, R, offset = load_extrinsics()
    proj = Projector(K, dist, rvec, tvec, R)
    print(f"extrinsics loaded, time offset {offset:+.3f}s")

    displays = collect_display_topics(RVIZ_CFG)
    cap = cv2.VideoCapture(VIDEO)
    fps = cap.get(cv2.CAP_PROP_FPS)
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    nframes = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # load bag messages for the enabled topics that actually exist & are in odom
    bag = rosbag.Bag(BAG)
    bag_topics = set(bag.get_type_and_topic_info().topics)
    streams = {}   # topic -> (name, class, sorted [(t, msg)])
    for cls, topic, name, _ in displays:
        if topic not in bag_topics or topic in streams:
            continue
        msgs = []
        for _, m, t in bag.read_messages(topics=[topic]):
            msgs.append((t.to_sec(), m))
        if not msgs:
            continue
        # keep only odom-framed content (what our extrinsic can place)
        streams[topic] = (name, cls, msgs)
        print(f"  {name:28s} {topic}  ({len(msgs)} msgs, {cls})")
    bag.close()

    def latest(msgs, T):
        lo, hi, best = 0, len(msgs) - 1, None
        while lo <= hi:
            mid = (lo + hi) // 2
            if msgs[mid][0] <= T:
                best = msgs[mid][1]; lo = mid + 1
            else:
                hi = mid - 1
        return best

    def render_frame(fi):
        cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
        ok, img = cap.read()
        if not ok:
            return None
        T = oo.VIDEO_START_EPOCH + fi / fps + offset
        for topic, (name, cls, msgs) in streams.items():
            m = latest(msgs, T)
            if m is None:
                continue
            if cls in POSE_LIKE:
                draw_pose(img, proj, m)
            else:
                markers = m.markers if hasattr(m, "markers") else [m]
                for mk in markers:
                    if mk.header.frame_id and mk.header.frame_id != "odom":
                        continue
                    if mk.action == 2:      # DELETE
                        continue
                    draw_marker(img, proj, mk)
        return img

    lo = max(int(args.start * fps), 0)
    hi = nframes - 1 if args.dur is None else min(int((args.start + args.dur) * fps), nframes - 1)

    if args.poc:
        outdir = os.path.join(REC, "overlay_full_poc")
        os.makedirs(outdir, exist_ok=True)
        import glob
        for f in glob.glob(os.path.join(outdir, "*.png")):
            os.remove(f)
        for k, fi in enumerate(np.linspace(lo, hi, 10).round().astype(int)):
            img = render_frame(int(fi))
            if img is not None:
                cv2.imwrite(os.path.join(outdir, f"full_{k:02d}_f{fi}.png"), img)
                print(f"  wrote full_{k:02d}_f{fi}.png")
        print(f"done -> {outdir}")
    else:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        vw = cv2.VideoWriter(args.out, fourcc, fps, (W, H))
        for fi in range(lo, hi + 1):
            img = render_frame(fi)
            if img is not None:
                vw.write(img)
            if fi % 60 == 0:
                print(f"  {fi}/{hi}")
        vw.release()
        print(f"done -> {args.out}")
    cap.release()


if __name__ == "__main__":
    main()
