"""AutoTasker — the self-running data-collection loop's brain.

    READY (you rearranged the scene) -> bench frame -> VLM planner -> task
    -> segmenter masks for target/destination -> proposal shown in the UI
    -> START uses it -> teleop -> SUCCESS/FAIL -> state 'rearrange' -> READY ...

Planning runs on a background thread so the control loop never blocks. `take()`
hands the prepared proposal to START (None if still pending / failed -> the
operator's typed task is used instead).
"""
import threading
import time

from . import planner, segment


class AutoTasker:
    def __init__(self, get_jpeg, plan_backend=None, seg_backend=None, history_n=8, max_attempts=4):
        self.get_jpeg = get_jpeg
        self.plan_backend, self.seg_backend = plan_backend, seg_backend
        self.history, self.history_n, self.max_attempts = [], history_n, max_attempts
        self.state = "idle"            # idle | planning | ready | failed
        self.proposal = None           # {"task": {...}, "masks": {...}}
        self.error = ""
        self._lock = threading.Lock()
        self._scene = 0

    def prepare(self):
        """Kick off planning on the current scene (call on READY / startup)."""
        with self._lock:
            if self.state == "planning":
                return
            self.state, self.proposal, self.error = "planning", None, ""
            self._scene += 1
            scene = self._scene
        threading.Thread(target=self._work, args=(scene,), daemon=True).start()

    def _work(self, scene):
        try:
            jpeg = self.get_jpeg()
            if jpeg is None:
                raise RuntimeError("no bench frame yet")
            hist = list(self.history[-self.history_n:])
            for attempt in range(self.max_attempts):
                p = planner.plan(jpeg, history=hist, backend=self.plan_backend)
                m = segment.segment(jpeg, [p["target"], p["destination"]], backend=self.seg_backend)
                masks = {"target": m.get(p["target"]), "dest": m.get(p["destination"])}
                if all(v is not None and v.any() for v in masks.values()):
                    break
                miss = [k for k, v in masks.items() if v is None or not v.any()]
                print(f"[auto] dropped '{p['target']}' -> '{p['destination']}': segmenter found no {'/'.join(miss)} "
                      f"(attempt {attempt + 1}/{self.max_attempts})")
                hist.append({"target": p["target"], "destination": p["destination"]})   # don't propose it again
            else:
                raise RuntimeError(f"no plan the segmenter could find after {self.max_attempts} tries")
            task = {"instruction": p["instruction"], "verb": p["verb"], "target": p["target"],
                    "destination": p["destination"], "scene_id": f"scene{scene:04d}", "planner": "llm"}
            with self._lock:
                self.proposal, self.state = {"task": task, "masks": masks}, "ready"
        except Exception as e:                       # offline / no token / bad JSON: fall back to manual
            with self._lock:
                self.state, self.error = "failed", f"{type(e).__name__}: {e}"
            print(f"[auto] planning failed -> manual task. {self.error[:300]}")

    def take(self):
        """Proposal for START (consumed), or None."""
        with self._lock:
            if self.state != "ready":
                return None
            prop, self.state = self.proposal, "idle"
        self.history.append(prop["task"])
        return prop

    def peek_masks(self):
        """Masks of the ready proposal (not consumed), or None."""
        with self._lock:
            return self.proposal["masks"] if self.state == "ready" and self.proposal else None

    def status(self):
        with self._lock:
            t = self.proposal["task"] if self.proposal else {}
            return {"auto": self.state, "auto_err": self.error, "auto_target": t.get("target", ""),
                    "auto_dest": t.get("destination", ""),
                    "auto_masks": bool(self.proposal and any(v is not None for v in self.proposal["masks"].values()))}
