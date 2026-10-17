"""Render an episode to preview.mp4: raw bench | highlighted bench | wrist (what a --highlight sam
dataset would show). Usage: python -m pick_place_SO101.tools.preview_episode data/<session>/ep_0001 [--open]"""
import argparse
import pathlib
import subprocess

import cv2

from ..highlight import apply_highlight, frame_masks
from ..tracking import load_tracks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ep")
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--open", action="store_true")
    a = ap.parse_args()
    d = pathlib.Path(a.ep)
    rd = lambda p: (cv2.imread(str(p), 0) > 0) if p.exists() else None
    t, de = rd(d / "masks/target.png"), rd(d / "masks/dest.png")
    tracks = load_tracks(d)
    if tracks["target"] is None and tracks["dest"] is None:
        tracks = None
    files = sorted((d / "frames/bench").glob("*.jpg"))[::a.every]
    out = d / "preview.mp4"
    vw = None
    for i, f in enumerate(files):
        b = cv2.cvtColor(cv2.imread(str(f)), cv2.COLOR_BGR2RGB)
        w = cv2.imread(str(d / "frames/wrist" / f.name))
        w = cv2.cvtColor(w, cv2.COLOR_BGR2RGB) if w is not None else b * 0
        hl = apply_highlight(b, *frame_masks(i * a.every, tracks, (t, de)))
        row = cv2.cvtColor(cv2.hconcat([b, hl, w]), cv2.COLOR_RGB2BGR)
        if vw is None:
            vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), 30 / a.every, row.shape[1::-1])
        vw.write(row)
    vw.release()
    print(f"wrote {out}  ({len(files)} frames; masks: target={t is not None} dest={de is not None}, tracked={tracks is not None})")
    if a.open:
        subprocess.run(["open", str(out)])


if __name__ == "__main__":
    main()
