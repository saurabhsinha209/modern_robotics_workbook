"""EpisodeLogger — raw, self-describing teleop episodes (no LeRobot dependency).

Training comes later, so this stores *everything* needed to choose the action
space and label later, per proj_so101_shopping_list.md §5.1 ("log everything,
decide at training time"). Conversion to a LeRobotDataset is a separate, later
step that reads these files.

    data/<session>/ep_0007/
        steps.npz     per-step arrays (below)
        meta.json     instruction, outcome, counts, timing, calibration id
        frames/<cam>/000123.jpg   downsized to config.FRAME_SIZE (training frames)
        frames_full/<cam>/000120.jpg   full-size, every config.FULL_FRAME_EVERY steps (debug)
        masks/target.png, dest.png     uint8 0/255 at FRAME_SIZE (optional, from start())
    data/<session>/episodes.jsonl   one line per finished episode

Per-step arrays (N steps):
    t                (N,)   s since episode start
    q                (N,6)  MEASURED joints  [5 deg, gripper %]      <- the state
    q_cmd            (N,6)  COMMANDED joints (post IK + guards)      <- joint-space action
    tcp_p / tcp_rot6d  (N,3)/(N,6)  FK(q_cmd): achieved TCP pose      <- EE-space action
    tgt_p / tgt_rot6d  (N,3)/(N,6)  what the phone asked for (pre-IK)
    phone_p / phone_q  (N,3)/(N,4)  raw WebXR pose
    clutch, orient_lock, ik_ok, held, rate_limited, resynced, vio_jump, stale  (N,) flags
    terminal         (N,)   0 everywhere, 1 on the last step
    outcome          scalar 1 = success, 0 = failure (also in meta.json)

Terminal label: the human presses SUCCESS / FAIL in the UI. That press happens
*after* the event it describes (reaction time), so `meta.json` keeps
`press_t` and the label should be treated as ending at `terminal`, not as an
exact success instant. The policy-side goal — learning to emit this termination
itself — only needs the final steps + outcome, which is why `terminal` and
`outcome` are first-class here rather than an afterthought.
"""
import json
import pathlib
import queue
import shutil
import threading
import time

import cv2
import numpy as np

from . import config

FLAGS = ("clutch", "orient_lock", "ik_ok", "held", "rate_limited", "resynced", "vio_jump", "stale", "homing")


TASK_KEYS = ("instruction", "verb", "target", "destination", "scene_id", "planner", "variant_tags")


def make_task(instruction="", task=None):
    """Normalise a task dict to the meta.json `task` block (missing fields -> defaults)."""
    t = {"instruction": instruction, "verb": "", "target": "", "destination": "",
         "scene_id": "", "planner": "manual", "variant_tags": []}
    t.update({k: v for k, v in (task or {}).items() if k in TASK_KEYS and v is not None})
    t["variant_tags"] = list(t["variant_tags"])
    if not t["instruction"]:
        t["instruction"] = instruction
    return t


def _mask_u8(m, size):
    m = np.asarray(m)
    if m.ndim == 3:
        m = m[..., 0]
    m = (m > 0).astype(np.uint8) * 255
    w, h = size
    return m if m.shape == (h, w) else cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)


def rot6d(R):
    """Continuous 6-D rotation rep: first two columns (Zhou et al.)."""
    return np.asarray(R)[:, :2].T.reshape(-1)


class EpisodeLogger:
    def __init__(self, root, session=None, robot_id=config.ROBOT_ID, hz=config.CONTROL_HZ, extra_meta=None,
                 frame_size=config.FRAME_SIZE, full_every=config.FULL_FRAME_EVERY):
        root = pathlib.Path(root)
        self.session = session or time.strftime("%Y%m%d_%H%M%S")
        self.dir = root / self.session
        self.dir.mkdir(parents=True, exist_ok=True)
        self.robot_id, self.hz = robot_id, hz
        self.extra_meta = extra_meta or {}
        self.frame_size, self.full_every = tuple(frame_size), full_every
        # JPEG decode/resize/encode runs on a writer thread so the 30 Hz loop only enqueues.
        self._q = queue.Queue(maxsize=600)
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer.start()
        self.dropped_frames = 0
        existing = [int(p.name[3:]) for p in self.dir.glob("ep_*") if p.name[3:].isdigit()]
        self.next_idx = (max(existing) + 1) if existing else 0
        self.counts = {"success": 0, "failure": 0, "discarded": 0, "aborted": 0}
        self._rows = None
        self._ep_dir = None
        self._t0 = None
        self.instruction = ""
        self.task = make_task()

    @property
    def active(self):
        return self._rows is not None

    @property
    def elapsed(self):
        return 0.0 if self._t0 is None else time.monotonic() - self._t0

    def _write_loop(self):
        while True:
            job = self._q.get()
            try:
                if job is not None:
                    self._write_frame(*job)
            except Exception as e:                     # never kill the writer
                print(f"[rec] frame write failed: {e}")
            finally:
                self._q.task_done()

    def _write_frame(self, ep_dir, cam, n, jpeg):
        w, h = self.frame_size
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        if self.full_every and n % self.full_every == 0:
            d = ep_dir / "frames_full" / cam
            d.mkdir(parents=True, exist_ok=True)
            (d / f"{n:06d}.jpg").write_bytes(jpeg)
        if img.shape[1] != w or img.shape[0] != h:
            img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            d = ep_dir / "frames" / cam
            d.mkdir(parents=True, exist_ok=True)
            (d / f"{n:06d}.jpg").write_bytes(buf.tobytes())

    def flush(self):
        self._q.join()

    def start(self, instruction, task=None, masks=None):
        """masks: {"target": HxW array, "dest": HxW array} (any size; stored at frame_size)."""
        assert not self.active
        self.instruction = instruction
        self.task = make_task(instruction, task)
        self._ep_dir = self.dir / f"ep_{self.next_idx:04d}"
        self._ep_dir.mkdir(parents=True)
        if masks:
            md = self._ep_dir / "masks"
            md.mkdir()
            for name in ("target", "dest"):
                if masks.get(name) is not None:
                    cv2.imwrite(str(md / f"{name}.png"), _mask_u8(masks[name], self.frame_size))
        self._rows = {k: [] for k in
                      ("t", "q", "q_cmd", "tcp_p", "tcp_rot6d", "tgt_p", "tgt_rot6d",
                       "phone_p", "phone_q", "phone_t", "phone_rx", *FLAGS)}
        self._t0 = time.monotonic()
        self._frame_n = 0
        return self._ep_dir

    def step(self, q_meas, q_cmd, info, msg, frames=None):
        r = self._rows
        r["t"].append(time.monotonic() - self._t0)
        r["q"].append(np.asarray(q_meas, float))
        r["q_cmd"].append(np.asarray(q_cmd, float))
        r["tcp_p"].append(info.tcp_p)
        r["tcp_rot6d"].append(rot6d(info.tcp_R))
        r["tgt_p"].append(info.tgt_p)
        r["tgt_rot6d"].append(rot6d(info.tgt_R))
        r["phone_p"].append(np.asarray(msg["p"], float) if msg else np.full(3, np.nan))
        r["phone_q"].append(np.asarray(msg["q"], float) if msg else np.full(4, np.nan))
        r["phone_t"].append(float(msg["t"]) if msg and "t" in msg else np.nan)
        r["phone_rx"].append(float(msg["_rx"]) if msg and "_rx" in msg else np.nan)
        r["clutch"].append(info.engaged)
        r["orient_lock"].append(info.orient_lock)
        r["ik_ok"].append(info.ik_ok)
        r["held"].append(info.held)
        r["rate_limited"].append(info.rate_limited)
        r["resynced"].append(info.resynced)
        r["vio_jump"].append(info.vio_jump)
        r["stale"].append(info.stale)
        r["homing"].append(info.homing)
        if frames:
            for cam, jpeg in frames.items():
                try:
                    self._q.put_nowait((self._ep_dir, cam, self._frame_n, jpeg))
                except queue.Full:
                    self.dropped_frames += 1
        self._frame_n += 1

    def end(self, outcome, extra=None):
        """outcome: 'success' | 'failure' | 'aborted'. Returns the meta dict."""
        assert self.active
        self.flush()
        r = self._rows
        n = len(r["t"])
        arrays = {k: np.asarray(v) for k, v in r.items()}
        for k in FLAGS:
            arrays[k] = arrays[k].astype(bool)
        term = np.zeros(n, np.uint8)
        if n:
            term[-1] = 1
        arrays["terminal"] = term
        arrays["outcome"] = np.array(1 if outcome == "success" else 0)
        np.savez_compressed(self._ep_dir / "steps.npz", **arrays)
        meta = {
            "episode": self.next_idx, "instruction": self.instruction,
            "outcome": outcome, "n_steps": n, "hz": self.hz,
            "duration_s": float(arrays["t"][-1]) if n else 0.0,
            "press_t": float(arrays["t"][-1]) if n else 0.0,
            "ik_fail_steps": int((~arrays["ik_ok"]).sum()) if n else 0,
            "held_steps": int(arrays["held"].sum()) if n else 0,
            "rate_limited_steps": int(arrays["rate_limited"].sum()) if n else 0,
            "resync_steps": int(arrays["resynced"].sum()) if n else 0,
            "clutch_steps": int(arrays["clutch"].sum()) if n else 0,
            "task": self.task, "frame_size": list(self.frame_size),
            "masks": sorted(p.stem for p in (self._ep_dir / "masks").glob("*.png"))
                     if (self._ep_dir / "masks").exists() else [],
            "dropped_frames": self.dropped_frames,
            "robot_id": self.robot_id, "units": "deg x5 + gripper pct",
            "cameras": sorted(p.name for p in (self._ep_dir / "frames").glob("*")) if
                       (self._ep_dir / "frames").exists() else [],
            "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
            **self.extra_meta, **(extra or {}),
        }
        (self._ep_dir / "meta.json").write_text(json.dumps(meta, indent=2))
        with open(self.dir / "episodes.jsonl", "a") as f:
            f.write(json.dumps(meta) + "\n")
        self.counts[outcome] = self.counts.get(outcome, 0) + 1
        self.next_idx += 1
        self._rows = self._ep_dir = self._t0 = None
        return meta

    def discard(self):
        assert self.active
        self.flush()
        shutil.rmtree(self._ep_dir, ignore_errors=True)
        self.counts["discarded"] += 1
        self._rows = self._ep_dir = self._t0 = None
