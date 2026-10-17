"""Live bench-camera tracking demo: highlighted feed in Chrome, mask updated by EdgeTAM at --hz.

    python -m pick_place_SO101.tools.live_track --target "pink highlighter" --dest "white tray"
    open http://localhost:8444/            (add --replay <ep_dir> to use recorded frames, --mock for colour seeds)
"""
import argparse
import asyncio
import glob
import json
import os
import threading
import time

os.environ.setdefault("HF_HUB_DISABLE_IMPLICIT_TOKEN", "1")   # user's cached HF login may be expired

import cv2
import numpy as np
from aiohttp import web

from .. import config
from ..highlight import apply_highlight
from ..live_tracker import LiveTracker
from ..coreml_live_tracker import CoreMLLiveTracker

PAGE = """<!doctype html><meta charset=utf-8><title>Live track</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px ui-monospace,monospace}
img{display:block;max-width:100%;margin:auto}#s{padding:10px 16px;white-space:pre}</style>
<img src="/live.mjpg"><div id=s>…</div>
<script>
async function tick(){try{const r=await (await fetch('/stats')).json();
document.getElementById('s').textContent=
`tracker ${r.hz.toFixed(1)} Hz (target ${r.target_hz})   ${r.ms.toFixed(0)} ms/update   updates ${r.updates}  reseeds ${r.reseeds}
state ${r.state}   video ${r.video_fps.toFixed(0)} fps
target: area ${r.area.target??'-'}${r.lost.target?'  LOST':''}     dest: area ${r.area.dest??'-'}${r.lost.dest?'  LOST':''}`}catch(e){}
setTimeout(tick,500)}tick()</script>"""


class Source:
    """Newest BGR frame, from the bench camera or a replayed episode."""
    def __init__(self, replay=None):
        self.replay, self.lock, self.bgr, self.n = replay, threading.Lock(), None, 0
        if replay:
            self.files = sorted(glob.glob(os.path.join(replay, "frames", "bench", "*.jpg")))
            threading.Thread(target=self._play, daemon=True).start()
        else:
            from ..cameras import CameraSet
            self.cams = CameraSet({"bench": config.CAMERA_UIDS["bench"]}).start()
            if self.cams.errors() or self.cams.jpeg("bench")[0] is None:
                raise SystemExit(f"bench camera unavailable: {self.cams.errors()} "
                                 "(grant Camera access to the terminal, or use --replay <ep_dir>)")
            threading.Thread(target=self._grab, daemon=True).start()

    def _set(self, bgr):
        with self.lock:
            self.bgr, self.n = bgr, self.n + 1

    def _grab(self):
        last = None
        while True:
            j, t = self.cams.jpeg("bench")
            if j is not None and t != last:
                last = t
                self._set(cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR))
            time.sleep(0.005)

    def _play(self):
        i, t0 = 0, time.perf_counter()
        while True:
            self._set(cv2.imread(self.files[i % len(self.files)]))
            i += 1
            time.sleep(max(0.0, t0 + i / 30.0 - time.perf_counter()))

    def get(self):
        with self.lock:
            return None if self.bgr is None else self.bgr.copy()

    def stop(self):
        if not self.replay:
            self.cams.stop()


def seed_masks(frame, target, dest, mock):
    from .. import segment
    ok, jpg = cv2.imencode(".jpg", frame)
    m = segment.segment(jpg.tobytes(), [target, dest], backend=segment.MockBackend() if mock else None)
    return {"target": m.get(target), "dest": m.get(dest)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", required=True)
    ap.add_argument("--dest", required=True)
    ap.add_argument("--backend", choices=("pytorch", "coreml"), default="coreml",
                    help="coreml = EdgeTAM Core ML on the Apple GPU (models/EdgeTAM-CoreML-Video)")
    ap.add_argument("--hz", type=float, default=None, help="tracker rate (default 15 coreml / 5 pytorch)")
    ap.add_argument("--port", type=int, default=8444)
    ap.add_argument("--replay", default=None, help="episode dir: replay its bench frames at 30 fps")
    ap.add_argument("--mock", action="store_true", help="colour-threshold seeds (no OWLv2/SAM2)")
    ap.add_argument("--seconds", type=float, default=0, help="auto-exit after N s (benchmarking)")
    a = ap.parse_args()

    src = Source(a.replay)
    while src.get() is None:
        time.sleep(0.05)
    print("[live] seeding masks…")
    seeds = seed_masks(src.get(), a.target, a.dest, a.mock)
    print("[live] seed areas:", {k: (None if v is None else int(v.sum())) for k, v in seeds.items()})
    if seeds["target"] is None:
        raise SystemExit(f"segmenter could not find target {a.target!r}")
    a.hz = a.hz or (15.0 if a.backend == "coreml" else 5.0)
    lt = (CoreMLLiveTracker if a.backend == "coreml" else LiveTracker)(src.get, seeds, hz=a.hz).start()

    vid = {"fps": 0.0}

    async def live(req):
        r = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=f",
                                        "Cache-Control": "no-store"})
        await r.prepare(req)
        last, t_hist = -1, []
        while True:
            if src.n == last:
                await asyncio.sleep(0.004)
                continue
            frame = src.get()
            last = src.n
            frame = cv2.resize(frame, (640, 360) if frame.shape[1] / frame.shape[0] > 1.5 else (640, 480))
            h, w = frame.shape[:2]
            m = lt.masks()
            rs = lambda x: None if x is None else cv2.resize(x.astype(np.uint8), (w, h),
                                                            interpolation=cv2.INTER_NEAREST).astype(bool)
            rgb = apply_highlight(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), rs(m.get("target")), rs(m.get("dest")))
            ok, jpg = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 80])
            try:
                await r.write(b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")
            except ConnectionResetError:
                return r
            now = time.perf_counter()
            t_hist = [t for t in t_hist + [now] if t > now - 3]
            if len(t_hist) > 1:
                vid["fps"] = (len(t_hist) - 1) / (t_hist[-1] - t_hist[0])

    async def stats(_):
        s = lt.stats()
        s.update(target_hz=a.hz, video_fps=vid["fps"])
        return web.json_response({k: (v if k != "area" else {kk: vv for kk, vv in v.items()}) for k, v in s.items()})

    app = web.Application()
    app.add_routes([web.get("/", lambda _: web.Response(text=PAGE, content_type="text/html")),
                    web.get("/live.mjpg", live), web.get("/stats", stats)])
    runner = web.AppRunner(app)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(runner.setup())
    loop.run_until_complete(web.TCPSite(runner, "0.0.0.0", a.port).start())
    print(f"[live] http://localhost:{a.port}/   (Ctrl-C to stop)")
    t_end = time.time() + a.seconds if a.seconds else None
    log = []

    async def watchdog():
        while t_end is None or time.time() < t_end:
            await asyncio.sleep(5)
            s = lt.stats()
            log.append(s)
            print(f"[live] {s['hz']:.1f} Hz  {s['ms']:.0f} ms  updates {s['updates']}  reseeds {s['reseeds']}  "
                  f"area {s['area']}  lost {s['lost']}  state {s['state']}  video {vid['fps']:.0f} fps")
    try:
        loop.run_until_complete(watchdog())
    except KeyboardInterrupt:
        pass
    finally:
        lt.stop()
        src.stop()
        loop.run_until_complete(runner.cleanup())


if __name__ == "__main__":
    main()
