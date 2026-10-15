"""Watch the USB cameras live in a browser, through the same path the teleop UI uses.

    .venv/bin/python -m pick_place_SO101.tools.view_cameras --wrist 0 --bench 1
    open  https://localhost:8443/cams.html        (or https://<mac-ip>:8443/cams.html)

Use check_cameras first to find indices. Omit a flag to skip that camera. Stop the
teleop process first (same port), or pass --port.
"""
import argparse
import threading
import time

import cv2

from ..server.server import TeleopServer


def grab(server, name, index, w, h, fps, quality):
    cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        print(f"[cam] {name}: index {index} not opened")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    n, t0 = 0, time.time()
    period = 1.0 / fps
    while True:
        ok, f = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        ok, jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            server.push_frame(name, jpg.tobytes())
        n += 1
        if n % 150 == 0:
            print(f"[cam] {name}: {n / (time.time() - t0):.1f} fps  {f.shape[1]}x{f.shape[0]}")
        time.sleep(max(0.0, period * 0.3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wrist", type=int)
    ap.add_argument("--bench", type=int)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=float, default=30)
    ap.add_argument("--quality", type=int, default=80)
    ap.add_argument("--port", type=int, default=8443)
    a = ap.parse_args()
    srv = TeleopServer(port=a.port)
    srv.start()
    for name in ("wrist", "bench"):
        idx = getattr(a, name)
        if idx is not None:
            threading.Thread(target=grab, daemon=True,
                             args=(srv, name, idx, a.width, a.height, a.fps, a.quality)).start()
    print(f"[cam] open  https://localhost:{a.port}/cams.html   (accept the cert warning)")
    print(f"[cam] or    https://{srv.ip}:{a.port}/cams.html     Ctrl-C to stop")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
