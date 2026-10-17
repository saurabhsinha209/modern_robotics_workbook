"""Task planner: a scene image -> {"verb","target","destination","instruction"}.

    plan(jpeg_bytes, history=(), backend=None)

Backends: HFBackend (default; HF Inference API, needs HF_TOKEN, model via
PLANNER_MODEL), MockBackend (offline/tests), ManualBackend (human types it).
"""
import base64
import json
import os
import re

DEFAULT_MODEL = "Qwen/Qwen3-VL-30B-A3B-Instruct"
FALLBACK_MODELS = ("google/gemma-4-26B-A4B-it", "Qwen/Qwen2.5-VL-72B-Instruct",
                   "meta-llama/Llama-4-Scout-17B-16E-Instruct", "google/gemma-3-27b-it")

PROMPT = """You are the task planner for a tabletop robot arm with a gripper.
Look at the image and propose ONE simple pick-and-place task using objects that
are CLEARLY VISIBLE. The target is a small graspable object; the destination is
a container, surface or marked place that is also visible and is not the target.
Use short noun phrases (e.g. "green toy boat", "blue bin").
RULES: never pick an object that is already inside a bin, tray or any container
(it has been placed already); only pick objects lying loose on the table surface.
{avoid}Reply with ONLY a JSON object:
{{"verb": "pick and place", "target": "...", "destination": "..."}}"""


def _norm(s):
    return re.sub(r"\s+", " ", str(s or "").strip().lower())


def _finish(d, history):
    t, dest = _norm(d.get("target")), _norm(d.get("destination"))
    if not t or not dest:
        raise ValueError(f"planner output missing target/destination: {d!r}")
    if t == dest:
        raise ValueError("target and destination are identical")
    verb = _norm(d.get("verb")) or "pick and place"
    last = next((h for h in reversed(list(history)) if isinstance(h, dict)), None)
    if last and _norm(last.get("target")) and (
            t in _norm(last["target"]) or _norm(last["target"]) in t):
        raise ValueError(f"'{t}' was the previous target; pick a different object")
    if (t, dest) in {(_norm(h.get("target")), _norm(h.get("destination"))) for h in history if isinstance(h, dict)}:
        raise ValueError(f"repeats an earlier task: {t} -> {dest}")
    instr = d.get("instruction") or (
        f"pick up the {t} and place it in the {dest}" if verb == "pick and place" else f"{verb} the {t} {dest}")
    return {"verb": verb, "target": t, "destination": dest, "instruction": instr}


def parse_json(text):
    """Tolerant JSON extraction: fenced blocks, prose around the object."""
    text = re.sub(r"```(?:json)?", "", text or "")
    for m in re.finditer(r"\{.*?\}", text, re.S):
        try:
            return json.loads(m.group(0))
        except json.JSONDecodeError:
            continue
    raise ValueError(f"no JSON object in reply: {text[:200]!r}")


def _avoid_text(history):
    prev = [f"{h.get('target')} -> {h.get('destination')}" for h in history if isinstance(h, dict)]
    if not prev:
        return ""
    last = next(h for h in reversed(list(history)) if isinstance(h, dict))
    return (f"The previous target was '{last.get('target')}': do NOT choose it (or anything already "
            f"in a container) again. Recent tasks: " + "; ".join(prev[-8:]) + ".\n")


class HFBackend:
    def __init__(self, model=None, token=None):
        self.model = model or os.environ.get("PLANNER_MODEL", DEFAULT_MODEL)
        self.token = token or os.environ.get("HF_TOKEN")
        if not self.token:                       # fall back to `huggingface-cli login`
            try:
                from huggingface_hub import get_token
                self.token = get_token()
            except Exception:
                pass
        self._client = None

    def __call__(self, jpeg, history, retry_note=""):
        if not self.token:
            raise RuntimeError("HF_TOKEN is not set (export HF_TOKEN=hf_...)")
        from huggingface_hub import InferenceClient
        url = "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()
        text = PROMPT.format(avoid=_avoid_text(history)) + retry_note
        msgs = [{"role": "user", "content": [
            {"type": "image_url", "image_url": {"url": url}},
            {"type": "text", "text": text}]}]
        # the router only serves models your enabled providers offer: walk the list
        models = [self.model] + ([] if os.environ.get("PLANNER_MODEL") else list(FALLBACK_MODELS))
        last = None
        for m in models:
            try:
                r = InferenceClient(model=m, token=self.token).chat_completion(
                    messages=msgs, max_tokens=150, temperature=0.7)
                if m != self.model:
                    print(f"[planner] using fallback model {m}")
                    self.model = m
                return parse_json(r.choices[0].message.content)
            except Exception as e:
                last = e
                if "not supported" not in str(e) and "model_not_found" not in str(e):
                    raise
        raise last


class MockBackend:
    """Cycles through fixed tasks, skipping any in `history`."""
    def __init__(self, tasks=(("green boat", "bin"), ("pink highlighter", "bin"), ("green boat", "tray"))):
        self.tasks, self.i = list(tasks), 0

    def __call__(self, jpeg, history, retry_note=""):
        t, d = self.tasks[self.i % len(self.tasks)]
        self.i += 1
        return {"verb": "pick and place", "target": t, "destination": d}


class ManualBackend:
    """Human typed: asks on stdin for target and destination."""
    def __init__(self, ask=input):
        self.ask = ask

    def __call__(self, jpeg, history, retry_note=""):
        return {"verb": "pick and place", "target": self.ask("target object: "),
                "destination": self.ask("destination: ")}


def plan(jpeg_bytes, history=(), backend=None, retries=3):
    backend = backend or HFBackend()
    note, last = "", None
    for _ in range(retries):
        try:
            return _finish(backend(jpeg_bytes, history, note), history)
        except ValueError as e:      # bad JSON / repeat / empty -> re-ask with the reason
            last = e
            note = f"\nYour previous answer was rejected: {e}. Try again."
    raise RuntimeError(f"planner failed after {retries} tries: {last}")
