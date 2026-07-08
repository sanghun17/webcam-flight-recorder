#!/usr/bin/env bash
# Promote a recording's drone-clicked extrinsics to that camera's BASE, so every
# future recording from the same camera uses it (until re-clicked).
#
#   bash promote_extrinsics.sh recordings/flight_<stamp>_cam2
#
# The camera (cam1/cam2/...) is read from the folder-name suffix; the file lands
# in extrinsics/<cam>/webcam_extrinsics.json, which overlay_lib falls back to for
# any folder of that camera lacking its own extrinsics.
set -e
REPO="$(dirname "$(readlink -f "$0")")"
DIR="$(readlink -f "${1:?usage: promote_extrinsics.sh <recording-folder>}")"
name="$(basename "$DIR")"
clicked="$DIR/webcam_extrinsics_clicked.json"

tag="$(printf '%s' "$name" | grep -oE 'cam[0-9]+$' || true)"
[ -n "$tag" ] || { echo "folder '$name' has no _camN suffix — cannot tell which camera"; exit 1; }
[ -f "$clicked" ] || { echo "no $clicked — run 1_recalibrate_extrinsic.sh (drone-click) first"; exit 1; }

dest="$REPO/extrinsics/$tag/webcam_extrinsics.json"
mkdir -p "$(dirname "$dest")"
cp "$clicked" "$dest"
echo "promoted $tag base extrinsics -> $dest"
echo "future $tag recordings without their own extrinsics now overlay with this."
