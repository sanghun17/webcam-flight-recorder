#!/usr/bin/env python3
"""Capture-only rviz config: strip side panels and the empty Image displays so
the Webcam_Overlay Camera view dominates the window (for headless recording)."""
import os
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(HERE, "rviz_overlay.rviz")
OUT = os.path.join(HERE, "rviz_capture.rviz")

cfg = yaml.safe_load(open(BASE))
# keep only the Displays panel out; drop dockable side panels entirely
cfg["Panels"] = []

def strip(displays):
    out = []
    for d in displays:
        if d.get("Class") == "rviz/Image":     # empty "No Image" docks
            continue
        if "Displays" in d and isinstance(d["Displays"], list):
            d["Displays"] = strip(d["Displays"])
        out.append(d)
    return out

vm = cfg["Visualization Manager"]
vm["Displays"] = strip(vm["Displays"])
yaml.safe_dump(cfg, open(OUT, "w"), default_flow_style=False, sort_keys=False)
print(f"wrote {OUT}")
