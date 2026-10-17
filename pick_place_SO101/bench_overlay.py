"""Live highlighted bench feed for the teleop UI (served as /cam/bench_hl.mjpg, bench.html).

    idle                -> raw bench frames
    proposal ready /    -> Core ML EdgeTAM (GPU) follows target + destination at `hz`; the planner's frame-0
    episode recording      masks are shown until the tracker has loaded

The recorded frames stay raw (episodes log cams.jpeg("bench")); this is a view for the operator, and it
previews exactly what convert_lerobot --highlight sam will produce. Failure to load the tracker degrades
to the static / raw view, never to a crash of the control loop.
"""
import threading
import time

import cv2
import numpy as np

from .highlight import apply_highlight


class BenchOverlay:
    def __init__(self, get_jpeg, publish, peek_masks=lambda: None, hz=15.0, out_hz=20.0, backend="coreml"):
        self.get_jpeg, self.publish, self.peek_masks = get_jpeg, publish, peek_masks
        self.hz, self.out_hz, self.backend = hz, out_hz, backend
        self._tracker, self._lock, self._stop, self._t = None, threading.Lock(), threading.Event(), None
        self._last, self._seeded = None, None

    def _frame(self):
        j, t = self.get_jpeg()
        if j is None or t == self._last:
            return None, None
        self._last = t
        return j, cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR)

    def start(self):
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()
        return self

    def stop(self):
        self._stop.set()
        self.untrack()
        if self._t:
            self._t.join(timeout=5)

    def track(self, masks):
        """Begin following the masks. A no-op if these exact masks are already being tracked."""
        if masks is not None and masks is self._seeded and self._tracker is not None:
            return
        self.untrack()
        self._seeded = masks
        seeds = {k: v for k, v in (masks or {}).items() if v is not None and np.asarray(v).any()}
        if not seeds:
            return
        try:
            if self.backend == "coreml":
                from .coreml_live_tracker import CoreMLLiveTracker as T
            else:
                from .live_tracker import LiveTracker as T
            tr = T(lambda: self._latest_bgr(), seeds, hz=self.hz).start()
        except Exception as e:
            print(f"[overlay] tracker unavailable ({type(e).__name__}: {e}); showing static masks")
            return
        with self._lock:
            self._tracker = tr

    def untrack(self):
        self._seeded = None
        with self._lock:
            tr, self._tracker = self._tracker, None
        if tr is not None:
            tr.stop()

    def _latest_bgr(self):
        j, _ = self.get_jpeg()
        return None if j is None else cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR)

    def _run(self):
        period = 1.0 / self.out_hz
        while not self._stop.is_set():
            t0 = time.perf_counter()
            try:
                pm = self.peek_masks()
                if pm is not None and pm is not self._seeded:
                    self.track(pm)                    # plan ready -> start following it now
                j, bgr = self._frame()
                if j is not None:
                    with self._lock:
                        tr = self._tracker
                    m = tr.masks() if tr is not None and tr.stats()["state"] == "tracking" \
                        else self._seeded
                    if m and any(v is not None for v in m.values()):
                        rgb = apply_highlight(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), m.get("target"), m.get("dest"))
                        ok, enc = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                               [cv2.IMWRITE_JPEG_QUALITY, 80])
                        self.publish("bench_hl", enc.tobytes())
                    else:
                        self.publish("bench_hl", j)
            except Exception as e:                       # a bad frame must not kill the view
                print(f"[overlay] {type(e).__name__}: {e}")
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))
