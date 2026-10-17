"""Watch the USB cameras live in a browser, through the same path the teleop UI uses.

    .venv/bin/python -m pick_place_SO101.tools.view_cameras [--cam-size 640x480] [--only wrist]
    open  https://localhost:8443/cams.html        (or https://<mac-ip>:8443/cams.html)

Uses config.CAMERA_UIDS (same cameras as run_teleop --cameras). Stop the teleop first (same port),
or pass --port.
"""
import argparse
import time

from .. import config
from ..cameras import CameraSet
from ..server.server import TeleopServer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam-size", default="%dx%d" % config.CAMERA_SIZE, help="WxH")
    ap.add_argument("--only", choices=sorted(config.CAMERA_UIDS), help="start just one camera")
    ap.add_argument("--port", type=int, default=8443)
    a = ap.parse_args()
    w, h = (int(v) for v in a.cam_size.lower().split("x"))
    uids = {k: v for k, v in config.CAMERA_UIDS.items() if a.only in (None, k)}
    srv = TeleopServer(port=a.port)
    srv.start()
    cams = CameraSet(uids, width=w, height=h).start(on_frame=srv.push_frame)
    print(f"[cam] open  https://localhost:{a.port}/cams.html   (accept the cert warning)")
    print(f"[cam] or    https://{srv.ip}:{a.port}/cams.html     Ctrl-C to stop")
    try:
        while True:
            time.sleep(5)
            print("[cam] frames", cams.counts(), "errors", cams.errors() or "-")
    except KeyboardInterrupt:
        pass
    finally:
        cams.stop()


if __name__ == "__main__":
    main()
