"""CoreMLLiveTracker — same interface as live_tracker.LiveTracker, but EdgeTAM runs as Core ML
(community port AhmadYarAI/EdgeTAM-CoreML-Video) on the Apple GPU (CPU_AND_GPU).

One EdgeTamCoreML per tracked object (each re-runs the 12 ms image encoder; shared-encoder is a
later optimisation). Re-seeds from the last good mask every RESEED_EVERY updates.
"""
import pathlib
import threading
import time

import cv2
import numpy as np

from .coreml_edgetam import EdgeTamCoreML

MODEL_DIR = pathlib.Path(__file__).parent / "models" / "EdgeTAM-CoreML-Video"
RESEED_EVERY = 300
MIN_FRAC = 0.15


class CoreMLLiveTracker:
    def __init__(self, get_frame, seeds, hz=15.0, size=(320, 240), model_dir=MODEL_DIR, units="CPU_AND_GPU"):
        self.get_frame, self.hz, self.size = get_frame, hz, size
        self.model_dir, self.units = str(model_dir), units
        self.names = [k for k, v in seeds.items() if v is not None and np.asarray(v).any()]
        W, H = size
        self._seed = {k: cv2.resize(np.asarray(seeds[k]).astype(np.uint8), (W, H),
                                    interpolation=cv2.INTER_NEAREST).astype(bool) for k in self.names}
        self._area0 = {k: max(1, int(m.sum())) for k, m in self._seed.items()}
        self._cur = {k: m.copy() for k, m in self._seed.items()}
        self._lock = threading.Lock()
        self._masks = {k: m.copy() for k, m in self._seed.items()}
        self._stat = dict(hz=0.0, ms=0.0, area={k: int(m.sum()) for k, m in self._seed.items()},
                          lost={k: False for k in self.names}, updates=0, reseeds=0, state="loading")
        self._stop, self._t, self._times = threading.Event(), None, []

    def start(self):
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()
        return self

    def stop(self):
        self._stop.set()
        if self._t:
            self._t.join(timeout=10)

    def masks(self):
        with self._lock:
            return {k: (m.copy() if m is not None else None) for k, m in self._masks.items()}

    def stats(self):
        with self._lock:
            return dict(self._stat, area=dict(self._stat["area"]), lost=dict(self._stat["lost"]))

    def _small(self, bgr):
        return cv2.resize(bgr, self.size, interpolation=cv2.INTER_AREA)

    def _seed_all(self, bgr):
        self.trk = {k: EdgeTamCoreML(self.model_dir, self.units) for k in self.names} \
            if not hasattr(self, "trk") else self.trk
        for k in self.names:
            self.trk[k].seed(bgr, self._cur[k])

    def _run(self):
        try:
            bgr = None
            while bgr is None and not self._stop.is_set():
                bgr = self.get_frame()
                time.sleep(0.02)
            self._seed_all(self._small(bgr))
            with self._lock:
                self._stat["state"] = "tracking"
            n, period, nxt = 0, 1.0 / self.hz, time.perf_counter()
            while not self._stop.is_set():
                bgr = self.get_frame()
                if bgr is None:
                    time.sleep(0.01)
                    continue
                small = self._small(bgr)
                t0 = time.perf_counter()
                new, lost, area = {}, {}, {}
                for k in self.names:
                    mk, sc = self.trk[k].step(small)
                    ok = sc >= 0 and mk.sum() >= MIN_FRAC * self._area0[k]
                    lost[k], new[k], area[k] = (not ok), (mk if ok else None), (int(mk.sum()) if ok else 0)
                    if ok:
                        self._cur[k] = mk
                dt = time.perf_counter() - t0
                now = time.perf_counter()
                self._times = [t for t in self._times + [now] if t > now - 5.0]
                hz = (len(self._times) - 1) / (self._times[-1] - self._times[0]) if len(self._times) > 1 else 0.0
                with self._lock:
                    self._masks = new
                    st = self._stat
                    st.update(hz=hz, ms=dt * 1000, area=area, lost=lost, updates=st["updates"] + 1)
                n += 1
                if n >= RESEED_EVERY:
                    self._seed_all(small)
                    n = 0
                    with self._lock:
                        self._stat["reseeds"] += 1
                nxt += period
                time.sleep(max(0.0, nxt - time.perf_counter()))
                if time.perf_counter() - nxt > period:
                    nxt = time.perf_counter()
        except Exception as e:
            import traceback
            traceback.print_exc()
            with self._lock:
                self._stat["state"] = f"ERROR {type(e).__name__}: {e}"
