#!/usr/bin/env python3
"""Insert a rviz/Camera display into the user's rviz config so all existing
displays get overlaid on the webcam image. Regenerate if the base config
changes:  python3 make_rviz_config.py
"""
import os
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.join(os.path.dirname(HERE), "recordings", "safety_2026-06-30-14-17-18")
BASE = os.path.join(REC, "rviz.rviz")
OUT = os.path.join(HERE, "rviz_overlay.rviz")

CAMERA = {
    "Class": "rviz/Camera",
    "Enabled": True,
    "Image Topic": "/webcam/image_rect",
    "Name": "Webcam_Overlay",
    "Overlay Alpha": 0.6,          # 0 = image only, 1 = markers only
    "Queue Size": 2,
    "Transport Hint": "raw",
    "Unreliable": False,
    "Value": True,
    "Zoom Factor": 1,
}


def main():
    cfg = yaml.safe_load(open(BASE))
    disp = cfg["Visualization Manager"]["Displays"]
    disp[:] = [d for d in disp if d.get("Name") != "Webcam_Overlay"]
    disp.append(CAMERA)
    # keep odom as fixed frame (already is); ensure sim-time friendliness
    yaml.safe_dump(cfg, open(OUT, "w"), default_flow_style=False, sort_keys=False)
    print(f"wrote {OUT}  ({len(disp)} displays, +Webcam_Overlay Camera)")


if __name__ == "__main__":
    main()
