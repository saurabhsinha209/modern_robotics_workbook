"""List/test cameras: resolution, fps, and a snapshot per index (saved to --out).

    .venv/bin/python -m pick_place_SO101.tools.check_cameras [--out /tmp/cams]

macOS: the terminal app needs Camera permission (System Settings > Privacy &
Security > Camera) or every index reports "not opened". Indices include the
built-in camera and a Continuity-Camera iPhone, so match frames by looking at
the saved JPEGs, not by assuming order.
"""
import argparse
import pathlib
import time

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="cam_snapshots")
    ap.add_argument("--max-index", type=int, default=6)
    a = ap.parse_args()
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for i in range(a.max_index):
        cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)
        if not cap.isOpened():
            print(f"[{i}] not opened")
            continue
        ok, f = False, None
        for _ in range(15):                       # let auto-exposure settle
            ok, f = cap.read()
            time.sleep(0.03)
        if not ok:
            print(f"[{i}] opened, no frame")
            cap.release()
            continue
        h, w = f.shape[:2]
        n, t = 0, time.time()
        while time.time() - t < 1.5:
            n += bool(cap.read()[0])
        path = out / f"cam{i}.jpg"
        cv2.imwrite(str(path), f)
        print(f"[{i}] {w}x{h}  {n / 1.5:.1f} fps  mean={f.mean():.0f}  -> {path}")
        cap.release()


if __name__ == "__main__":
    main()
