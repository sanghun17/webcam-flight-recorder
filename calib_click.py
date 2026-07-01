#!/usr/bin/env python3
"""
calib_click — recalibrate the webcam extrinsics by clicking the drone.

The drone flies through the OptiTrack volume, so it is a moving calibration
target: at any instant we know its 3D world position (/vrpn_client_node/pure/pose)
and, if you click where it appears in the frame, its 2D pixel position. A handful
of well-spread (3D <-> 2D) correspondences is all cv2.solvePnP needs to re-solve
the camera pose (rvec/tvec) with the intrinsics (K, dist) held fixed.

RUN THIS YOURSELF (it opens a GUI window):

    python3 calib_click.py                 # 12 auto-picked frames
    python3 calib_click.py --n 16

Per frame:
    * left-click the drone (its mocap-marker centre — the bright cluster on top
      if you can see it; be consistent frame to frame)
    * zoom/pan with the matplotlib toolbar first for precision, then click
    * red '+' = where the CURRENT extrinsics project the drone (the error you're fixing)
    * press  n = skip this frame (drone hidden/unclear)   r = redo last click
             q = stop early and solve with what we have

Writes webcam_extrinsics_clicked.json (original is left untouched) and
clicks_<n>.json so a re-solve needs no re-clicking. Verify with:

    python3 overlay_odom.py --extr webcam_extrinsics_clicked.json
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import overlay_lib
from overlay_lib import ODOM_TOPIC, load_extrinsics, load_odom, nearest

DEFAULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "recordings", "safety_2026-06-30-14-17-18")


def pick_frames(ts, pos, fps, n, offset, start_epoch, zmin=0.3):
    """Farthest-point-sample n frames to maximise 3D spread (best conditioning
    for solvePnP), reserving a couple of near-stationary ground frames as exact
    time-sync anchors. Returns list of (frame_idx, sample_idx)."""
    lo_t, hi_t = ts[0], ts[-1]
    inview = [i for i in range(0, len(ts), 5) if lo_t <= ts[i] <= hi_t]
    air = [i for i in inview if pos[i, 2] > zmin]           # airborne
    ground = [i for i in inview if pos[i, 2] <= zmin]       # on/near floor
    if len(air) < n:
        air = inview

    def fps_sample(cand, k, seed):
        P = pos[cand]
        chosen = [seed]
        while len(chosen) < min(k, len(cand)):
            d = np.min([np.linalg.norm(P - P[c], axis=1) for c in chosen], axis=0)
            chosen.append(int(np.argmax(d)))
        return [cand[c] for c in chosen]

    sel = set(fps_sample(air, n - 2, int(np.argmax(pos[air][:, 0]))))
    # add 2 ground anchors spread in time (start-ish and end-ish)
    if ground:
        sel.add(ground[len(ground) // 4])
        sel.add(ground[3 * len(ground) // 4])
    out = []
    for si in sorted(sel):
        fi = int(round((ts[si] - start_epoch - offset) * fps))
        out.append((fi, si))
    return out


def reproj_rms(objp, imgp, rvec, tvec, K, dist):
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
    return float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - imgp) ** 2, axis=1))))


def collect_clicks(frames, ts, pos, K, dist, rvec, tvec, video):
    cap = cv2.VideoCapture(video)
    corr = []   # (sample_idx, frame_idx, (u,v))
    i = 0
    while i < len(frames):
        fi, si = frames[i]
        cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
        ok, img = cap.read()
        if not ok:
            i += 1
            continue
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        proj, _ = cv2.projectPoints(pos[si].reshape(1, 3), rvec, tvec, K, dist)
        px, py = proj.reshape(2)

        state = {}
        fig, ax = plt.subplots(figsize=(16, 12))
        ax.imshow(rgb)
        ax.plot([px], [py], "r+", ms=20, mew=3)          # current-extrinsic hint
        ax.set_title(f"[{i+1}/{len(frames)}] frame {fi}  drone3D=("
                     f"{pos[si][0]:.2f},{pos[si][1]:.2f},{pos[si][2]:.2f})   "
                     f"LEFT-CLICK drone CENTRE (marker/body)  |  n=skip(unclear)  r=redo  q=finish")

        def on_click(ev):
            if fig.canvas.toolbar.mode:       # ignore clicks while zooming/panning
                return
            if ev.button == 1 and ev.inaxes is ax:
                state["pt"] = (float(ev.xdata), float(ev.ydata))
                plt.close(fig)

        def on_key(ev):
            if ev.key == "n":
                state["skip"] = True; plt.close(fig)
            elif ev.key == "q":
                state["quit"] = True; plt.close(fig)
            elif ev.key == "r":
                state["redo"] = True; plt.close(fig)

        fig.canvas.mpl_connect("button_press_event", on_click)
        fig.canvas.mpl_connect("key_press_event", on_key)
        plt.show()

        if state.get("quit"):
            break
        if state.get("redo"):
            if corr:
                corr.pop()
                i = max(0, i - 1)
            continue
        if state.get("skip") or "pt" not in state:
            i += 1
            continue
        corr.append((si, fi, state["pt"]))
        print(f"  {len(corr)}: frame {fi} -> click {state['pt'][0]:.0f},{state['pt'][1]:.0f}")
        i += 1
    cap.release()
    return corr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEFAULT_DIR, help="recording folder")
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--offset", type=float, default=0.0)
    ap.add_argument("--out", default=None, help="output extrinsics (default: <dir>/webcam_extrinsics_clicked.json)")
    ap.add_argument("--clicks", default=None, help="reuse a saved clicks json (skip GUI)")
    args = ap.parse_args()

    info = overlay_lib.find_recording(args.dir)
    REC, BAG, VIDEO = info["dir"], info["bag"], info["video"]
    EXTR = info["extr"]
    start_epoch = info["start_epoch"]
    out_path = args.out or os.path.join(REC, "webcam_extrinsics_clicked.json")
    if not (BAG and VIDEO and EXTR and start_epoch is not None):
        print(f"missing inputs in {REC}: bag={BAG} video={VIDEO} extr={EXTR} start={start_epoch}")
        return
    print(f"recording: {REC}\n  base extrinsics: {EXTR}")

    E = json.load(open(EXTR))
    K, dist, rvec0, tvec0, _ = load_extrinsics(EXTR)
    ts, pos, quat = load_odom(BAG, ODOM_TOPIC)
    cap = cv2.VideoCapture(VIDEO); fps = cap.get(cv2.CAP_PROP_FPS); cap.release()

    if args.clicks:
        saved = json.load(open(args.clicks))
        frames = np.array([c["frame"] for c in saved])
        imgp = np.array([c["uv"] for c in saved], np.float64)
    else:
        picks = pick_frames(ts, pos, fps, args.n, args.offset, start_epoch)
        print(f"picked {len(picks)} frames; opening GUI...")
        corr = collect_clicks(picks, ts, pos, K, dist, rvec0, tvec0, VIDEO)
        if len(corr) < 4:
            print(f"only {len(corr)} clicks — need >=4. aborting.")
            return
        frames = np.array([fi for _, fi, _ in corr])
        imgp = np.array([uv for _, _, uv in corr], np.float64)
        ck = os.path.join(REC, f"clicks_{len(corr)}.json")
        json.dump([{"xyz": pos[si].tolist(), "uv": list(uv), "frame": fi}
                   for si, fi, uv in corr], open(ck, "w"), indent=2)
        print(f"saved correspondences -> {ck}")

    # Because the drone is moving and the video<->bag clock offset is only known
    # to ~1s (csv second-resolution start + ffmpeg/camera warm-up), we jointly
    # estimate the time offset and the extrinsics: scan the offset, and at each
    # offset re-pair every click with the drone's interpolated 3D position at
    # that instant, then solvePnP. Pick the offset with the lowest reprojection.
    def pos_at(t):
        return np.array([np.interp(t, ts, pos[:, k]) for k in range(3)])

    def solve_at(off):
        objp = np.array([pos_at(start_epoch + f / fps + off) for f in frames],
                        np.float64)
        rv, tv = rvec0.copy(), tvec0.copy()
        cv2.solvePnP(objp, imgp, K, dist, rv, tv,
                     useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
        rv, tv = cv2.solvePnPRefineLM(objp, imgp, K, dist, rv, tv)
        return reproj_rms(objp, imgp, rv, tv, K, dist), rv, tv

    best = (1e9, None, None, 0.0)
    for off in np.arange(-0.5, 1.5, 0.02):
        rms, rv, tv = solve_at(off)
        if rms < best[0]:
            best = (rms, rv, tv, off)
    for off in np.arange(best[3] - 0.02, best[3] + 0.02, 0.002):   # refine
        rms, rv, tv = solve_at(off)
        if rms < best[0]:
            best = (rms, rv, tv, off)
    rms_after, rvec, tvec, off = best

    rms_before = reproj_rms(np.array([pos_at(start_epoch + f / fps) for f in frames]),
                            imgp, rvec0, tvec0, K, dist)
    R, _ = cv2.Rodrigues(rvec)
    cam_pos = (-R.T @ tvec).reshape(3)
    T = np.eye(4); T[:3, :3] = R.T; T[:3, 3] = cam_pos

    print(f"\ntime offset: {off:+.3f}s   (video frame time -> bag time)")
    print(f"reprojection RMS: before={rms_before:.1f}px  after={rms_after:.1f}px  (n={len(frames)})")
    print(f"camera_pos_optitrack: was {np.array(E['camera_pos_optitrack']).round(3)} "
          f"-> {cam_pos.round(3)}")

    E["rvec"] = rvec.reshape(3).tolist()
    E["tvec"] = tvec.reshape(3).tolist()
    E["T_O_W"] = T.tolist()
    E["camera_pos_optitrack"] = cam_pos.tolist()
    E["method"] = "drone-click-pnp"
    E["reproj_rms_px"] = rms_after
    E["time_offset_sec"] = off      # add to VIDEO_START_EPOCH before matching odom
    json.dump(E, open(out_path, "w"), indent=2)
    print(f"\nsaved -> {out_path}")
    print(f"verify: rviz_overlay/run_overlay.sh {REC}")


if __name__ == "__main__":
    main()
