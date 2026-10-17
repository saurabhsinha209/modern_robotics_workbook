"""LiveTracker — EdgeTAM streaming mask tracker for the bench camera, in its own thread.

    lt = LiveTracker(get_frame, seeds={"target": mask, "dest": mask}, hz=5).start()
    lt.masks()  -> {"target": bool HxW | None, "dest": ...}   (latest, held between updates)
    lt.stats()  -> {hz, ms, area, lost, ...}

`get_frame()` returns the newest BGR frame (any size). The tracker downsizes to `size`
(default 320x240), feeds EdgeTAM one frame per update (streaming mode: no video preloaded),
and holds the last mask in between. Memory is bounded two ways: old per-frame tensors/outputs
are pruned every update, and the session is re-seeded from the last good mask every
RESEED_EVERY updates (this also limits drift of the memory bank).
"""
import os
import threading
import time

import cv2
import numpy as np

RESEED_EVERY = 150
KEEP_FRAMES = 12            # memory-bank window kept in the session
MIN_FRAC = 0.15             # mask < this fraction of seed area -> lost


def _device():
    import torch
    return "mps" if torch.backends.mps.is_available() else ("cuda" if torch.cuda.is_available() else "cpu")


class LiveTracker:
    def __init__(self, get_frame, seeds, hz=5.0, size=(320, 240), model_id="yonigozlan/EdgeTAM-hf", device=None):
        self.get_frame, self.hz, self.size = get_frame, hz, size
        self.model_id, self.dev = model_id, device or _device()
        self.names = [k for k, v in seeds.items() if v is not None and np.asarray(v).any()]
        W, H = size
        self._seed = {k: cv2.resize(np.asarray(seeds[k]).astype(np.uint8), (W, H),
                                    interpolation=cv2.INTER_NEAREST).astype(bool) for k in self.names}
        self._area0 = {k: max(1, int(self._seed[k].sum())) for k in self.names}
        self._cur = {k: m.copy() for k, m in self._seed.items()}      # last good mask per object
        self._lock = threading.Lock()
        self._masks = {k: self._seed[k].copy() for k in self.names}
        self._stat = dict(hz=0.0, ms=0.0, area={k: int(m.sum()) for k, m in self._seed.items()},
                          lost={k: False for k in self.names}, updates=0, reseeds=0, state="loading")
        self._stop = threading.Event()
        self._t = None
        self._times = []

    # ---- public ----
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

    # ---- internals ----
    def _load(self):
        from transformers import EdgeTamVideoModel, Sam2VideoProcessor
        try:
            proc = Sam2VideoProcessor.from_pretrained(self.model_id, local_files_only=True)
            model = EdgeTamVideoModel.from_pretrained(self.model_id, local_files_only=True)
        except Exception:
            proc = Sam2VideoProcessor.from_pretrained(self.model_id)
            model = EdgeTamVideoModel.from_pretrained(self.model_id)
        self.proc, self.model = proc, model.to(self.dev).eval()

    def _prep(self, bgr):
        W, H = self.size
        rgb = cv2.cvtColor(cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
        return self.proc(images=rgb, device=self.dev, return_tensors="pt").pixel_values[0]

    def _new_session(self, bgr):
        """Fresh streaming session seeded from the last good masks on this frame."""
        import torch
        sess = self.proc.init_video_session(inference_device=self.dev, processing_device=self.dev,
                                            video_storage_device="cpu", inference_state_device=self.dev,
                                            dtype=torch.float32)
        W, H = self.size
        ids = list(range(1, len(self.names) + 1))
        self.proc.add_inputs_to_inference_session(
            sess, frame_idx=0, obj_ids=ids,
            input_masks=[self._cur[k].astype(np.uint8) for k in self.names], original_size=(H, W))
        with torch.inference_mode():
            self.model(inference_session=sess, frame=self._prep(bgr))
        return sess

    def _prune(self, sess, idx):
        old = idx - KEEP_FRAMES
        if sess.processed_frames is not None:
            # NOT del: sess.num_frames == len(processed_frames) is the next frame index, so
            # deleting entries freezes the tracker on one stale frame. Keep the keys, free the pixels.
            for k in [k for k, v in sess.processed_frames.items() if k < old and v is not None and v.numel() > 0]:
                sess.processed_frames[k] = sess.processed_frames[k].new_empty(0)
        for od in sess.output_dict_per_obj.values():
            for k in [k for k in od["non_cond_frame_outputs"] if k < old]:
                del od["non_cond_frame_outputs"][k]

    def _run(self):
        import torch
        try:
            self._load()
            bgr = None
            while bgr is None and not self._stop.is_set():
                bgr = self.get_frame()
                time.sleep(0.02)
            sess = self._new_session(bgr)
            n_sess, period = 0, 1.0 / self.hz
            with self._lock:
                self._stat["state"] = "tracking"
            nxt = time.perf_counter()
            while not self._stop.is_set():
                bgr = self.get_frame()
                if bgr is None:
                    time.sleep(0.01)
                    continue
                t0 = time.perf_counter()
                W, H = self.size
                with torch.inference_mode():
                    out = self.model(inference_session=sess, frame=self._prep(bgr))
                    pm = self.proc.post_process_masks([out.pred_masks], original_sizes=[[H, W]],
                                                      binarize=True)[0]
                    sc = out.object_score_logits.reshape(len(out.object_ids), -1)[:, 0].float().cpu().numpy()
                n_sess += 1
                self._prune(sess, out.frame_idx)
                new, lost, area = {}, {}, {}
                for j, k in enumerate(self.names):
                    mk = pm[j, 0].cpu().numpy().astype(bool)
                    ok = sc[j] >= 0 and mk.sum() >= MIN_FRAC * self._area0[k]
                    lost[k] = not ok
                    new[k] = mk if ok else None
                    area[k] = int(mk.sum()) if ok else 0
                    if ok:
                        self._cur[k] = mk
                dt = time.perf_counter() - t0
                self._times.append(time.perf_counter())
                self._times = [t for t in self._times if t > self._times[-1] - 5.0]
                hz = (len(self._times) - 1) / (self._times[-1] - self._times[0]) if len(self._times) > 1 else 0.0
                with self._lock:
                    self._masks = new
                    st = self._stat
                    st.update(hz=hz, ms=dt * 1000, area=area, lost=lost, updates=st["updates"] + 1)
                if n_sess >= RESEED_EVERY:                      # bound memory + memory-bank drift
                    sess = self._new_session(bgr)
                    n_sess = 0
                    with self._lock:
                        self._stat["reseeds"] += 1
                nxt += period
                time.sleep(max(0.0, nxt - time.perf_counter()))
                if time.perf_counter() - nxt > period:          # fell behind: don't burst
                    nxt = time.perf_counter()
        except Exception as e:                                   # surface in the UI instead of dying silently
            import traceback
            traceback.print_exc()
            with self._lock:
                self._stat["state"] = f"ERROR {type(e).__name__}: {e}"
