"""Tracker (Core ML EdgeTAM, 1 object) and policy (random-weight ACT Core ML) running at the same time.
Reports each one's median latency alone vs concurrent, per compute-unit assignment.
    python -m pick_place_SO101.tools.coreml_eval.concurrent <edgetam_models_dir> <act.mlpackage> [--secs 8]"""
import argparse, glob, threading, time
import numpy as np, cv2, coremltools as ct
from ...tracking import load_tracks
from ...coreml_edgetam import EdgeTamCoreML, EP, U

S = 224


def run(tag, assignments, edir, act_path, secs, frames, seedmask, which):
    res = {}
    stop = threading.Event()
    tk = EdgeTamCoreML(edir, assignments[0]) if "tracker" in which else None
    if tk: tk.seed(frames[0], seedmask)
    act = ct.models.MLModel(act_path, compute_units=U[assignments[1]]) if "policy" in which else None
    x = {"bench": np.random.rand(1, 3, S, S).astype(np.float32), "wrist": np.random.rand(1, 3, S, S).astype(np.float32),
         "robot_q": np.random.rand(1, 6).astype(np.float32)}
    if act: act.predict(x)
    if tk: tk.step(frames[1])

    def trk():
        ts, i = [], 2
        while not stop.is_set():
            t = time.perf_counter(); tk.step(frames[i % len(frames)]); ts.append((time.perf_counter() - t) * 1e3); i += 1
        res["tracker"] = ts

    def pol():                                   # 30 Hz, like the real control loop (not a tight spin)
        ts, nxt = [], time.perf_counter()
        while not stop.is_set():
            t = time.perf_counter(); act.predict(x); ts.append((time.perf_counter() - t) * 1e3)
            nxt += 1 / 30; time.sleep(max(0.0, nxt - time.perf_counter()))
        res["policy"] = ts
    th = [threading.Thread(target=f) for f, n in ((trk, "tracker"), (pol, "policy")) if n in which]
    [t.start() for t in th]; time.sleep(secs); stop.set(); [t.join() for t in th]
    out = []
    for k in ("tracker", "policy"):
        if k in res: out.append(f"{k}: median {np.median(res[k][2:]):.1f} ms  ({len(res[k]) / secs:.0f}/s)")
    print(f"{tag:46s} " + " | ".join(out), flush=True)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("edir"); ap.add_argument("act"); ap.add_argument("--secs", type=float, default=8); a = ap.parse_args()
    fs = sorted(glob.glob(EP + "/frames/bench/*.jpg"))[840::6][:30]
    frames = [cv2.imread(f) for f in fs]; seed = load_tracks(EP)["target"][840]
    for tr_u in ("CPU_AND_GPU",):
        run(f"tracker alone [{tr_u}]", (tr_u, "CPU_AND_GPU"), a.edir, a.act, a.secs, frames, seed, ("tracker",))
    run("policy alone [CPU_AND_GPU]", ("CPU_AND_GPU", "CPU_AND_GPU"), a.edir, a.act, a.secs, frames, seed, ("policy",))
    run("policy alone [CPU_AND_NE]", ("CPU_AND_GPU", "CPU_AND_NE"), a.edir, a.act, a.secs, frames, seed, ("policy",))
    for t_u, p_u in (("CPU_AND_GPU", "CPU_AND_GPU"), ("CPU_AND_GPU", "CPU_AND_NE"), ("CPU_AND_NE", "CPU_AND_GPU"), ("CPU_AND_NE", "CPU_AND_NE")):
        run(f"CONCURRENT tracker[{t_u}] + policy[{p_u}]", (t_u, p_u), a.edir, a.act, a.secs, frames, seed, ("tracker", "policy"))


if __name__ == "__main__":
    main()
