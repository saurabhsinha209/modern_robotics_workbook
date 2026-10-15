"""End-to-end self-test: synthetic phone -> mapping -> IK -> guards -> sim arm -> episodes.

No phone, no hardware, no viewer. Exercises the things that go wrong on a real arm:
reachability, windup/re-anchor, per-tick joint jumps, stale packets, the gripper
presets, and the START / SUCCESS / FAIL / DISCARD episode lifecycle on disk.
"""
import json
import pathlib
import tempfile
import time
import types

import numpy as np

from . import config
from .backends.sim import SimBackend
from .episodes import EpisodeLogger
from .pipeline import TeleopPipeline

HZ = 30.0
DT = 1.0 / HZ


class FakeServer:
    """Stands in for TeleopServer: scripted phone message + scripted events."""
    def __init__(self):
        self.msg = None
        self.events = []
        self.status = []
        self.instruction = ""

    def latest(self):
        return None if self.msg is None else dict(self.msg)

    def take_events(self):
        ev, self.events = self.events, []
        return ev

    def set_instruction(self, t):
        self.instruction = t

    def push_status(self, **kw):
        self.status.append(kw)


class Phone:
    """A phone held still at the origin of {ar}; edit .p / .clutch / .gripper."""
    def __init__(self):
        self.p = np.zeros(3)
        self.clutch = False
        self.gripper = 50.0
        self.lock = True

    def msg(self, t):
        return {"type": "xr", "p": self.p.tolist(), "q": [0, 0, 0, 1], "clutch": self.clutch,
                "gripper": self.gripper, "orient_lock": self.lock, "tracked": True, "_rx": t}


def _make():
    sim = SimBackend(viewer=False)
    sim.connect()
    # start from the *home* pose so we are mid-workspace
    sim.data.qpos[sim.qadr] = sim._to_rad(config.HOME_Q)
    sim.data.ctrl[:6] = sim.data.qpos[sim.qadr]
    import mujoco
    mujoco.mj_forward(sim.model, sim.data)
    pipe = TeleopPipeline(hz=HZ, use_calibration_limits=False)
    pipe.reset(sim.read())
    return sim, pipe


class Runner:
    def __init__(self):
        self.sim, self.pipe = _make()
        self.phone = Phone()
        self.t = 0.0
        self.max_dq = 0.0
        self.infos = []
        self.prev_cmd = self.pipe.q_ref.copy()

    def run(self, seconds):
        for _ in range(int(seconds * HZ)):
            self.t += DT
            q_cmd, info = self.pipe.step(self.phone.msg(self.t), self.sim.read(), now=self.t)
            self.max_dq = max(self.max_dq, np.abs(q_cmd[:5] - self.prev_cmd[:5]).max())
            self.prev_cmd = q_cmd.copy()
            self.sim.command(q_cmd)
            self.sim.advance(DT)
            self.infos.append(info)
        return self.infos[-1]

    def ramp(self, to, seconds, then=0.0):
        """Move the phone smoothly (like a hand), then optionally keep running."""
        a, n = self.phone.p.copy(), max(1, int(seconds * HZ))
        for i in range(1, n + 1):
            self.phone.p = a + (np.asarray(to, float) - a) * (i / n)
            self.run(DT)
        if then:
            self.run(then)
        return self.infos[-1]

    def tcp(self):
        p, _ = self.pipe.fk_deg(self.sim.read())
        return p


def _check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}")
    return ok


def run_selftest():
    t_start = time.time()
    results = []
    cap = config.MAX_JOINT_SPEED_DPS * DT + 1e-6

    # 1 ── clutched relative mapping: forward 10 cm + up 5 cm of phone → arm +x, +z
    print("1. clutched mapping")
    r = Runner(); r.run(0.5)
    p0 = r.tcp()
    r.phone.clutch = True; r.run(0.1)
    r.ramp([0.0, 0.05, -0.10], 1.0, then=2.0)        # {ar}: up +y, forward −z
    d = r.tcp() - p0
    results.append(_check("moves +x ≈10 cm, +z ≈5 cm", abs(d[0] - 0.10) < 0.02 and abs(d[2] - 0.05) < 0.02,
                          f"Δ={np.round(d, 3)}"))
    results.append(_check("joint speed cap respected", r.max_dq <= cap, f"max Δq/tick={r.max_dq:.2f}° cap={cap:.2f}°"))

    # 2 ── clutch off: arm holds still no matter how the phone moves
    print("2. ratchet / hold")
    r.phone.clutch = False; r.run(0.5)
    p1 = r.tcp(); r.phone.p = np.array([0.3, 0.3, 0.3]); r.run(1.5)   # clutch off: free to roam
    results.append(_check("holds with clutch off", np.linalg.norm(r.tcp() - p1) < 0.002,
                          f"drift={np.linalg.norm(r.tcp() - p1) * 1000:.2f} mm"))
    r.phone.clutch = True; r.run(0.1)                # re-engage far away: reference resets, no jump
    results.append(_check("re-engage does not jump", np.linalg.norm(r.tcp() - p1) < 0.01,
                          f"jump={np.linalg.norm(r.tcp() - p1) * 1000:.1f} mm"))

    # 2b ── stall/resync under a held clutch must recover without a clutch toggle
    print("2b. resync recovery")
    r.phone.clutch = True; r.run(0.2)
    qm = r.sim.read().copy(); qm[2] += 25.0                  # pretend the elbow stalled 25° behind
    r.pipe.step(r.phone.msg(r.t), qm, now=r.t)
    pa = r.tcp(); r.ramp(r.phone.p + np.array([0.0, 0.0, -0.06]), 1.0, then=0.3)
    results.append(_check("recovers after resync, clutch still on", np.linalg.norm(r.tcp() - pa) > 0.03,
                          f"moved={np.linalg.norm(r.tcp() - pa) * 1000:.0f} mm"))

    # 2c ── bursty packet arrival: jitter buffer removes the stair-step
    print("2c. jitter buffer")
    from .smoothing import PoseJitterBuffer
    rng = np.random.default_rng(0)
    v = 0.15                                                    # 15 cm/s steady hand motion
    pk = []                                                     # 30 Hz phone frames, jittery arrival
    for k in range(120):
        tt = k / 30.0
        arr = tt + 0.01 + (rng.random() < 0.25) * rng.uniform(0.04, 0.17)
        pk.append((tt * 1000.0, arr, (v * tt, 0.0, 0.0), (0, 0, 0, 1)))
    def steps(use_buf):
        buf = PoseJitterBuffer(delay_s=0.08)
        last, out = None, []
        for n in range(30, 110):
            now = n / 30.0 + 0.004
            hist = [h for h in pk if h[1] <= now]
            if use_buf:
                x = buf.render(hist, now)[0][0]
            else:
                x = hist[-1][2][0]
            if last is not None:
                out.append(x - last)
            last = x
        d = np.array(out) * 1000
        return d.std() / d.mean()
    raw_cv, buf_cv = steps(False), steps(True)
    results.append(_check("step-size irregularity drops with buffer", buf_cv < 0.5 * raw_cv,
                          f"CV raw={raw_cv:.2f} buffered={buf_cv:.2f}"))

    # 3 ── unreachable target: smooth, flagged, and responsive on the way back
    print("3. out of reach")
    r.max_dq = 0.0
    base = r.phone.p.copy()
    info = r.ramp(base + np.array([0.0, 0.0, -0.9]), 3.0, then=1.0)   # 90 cm past the workspace
    results.append(_check("flagged not-ok / reanchored", (not info.ik_ok) or r.pipe.reanchors > 0,
                          f"ik_ok={info.ik_ok} reanchors={r.pipe.reanchors}"))
    results.append(_check("no joint jump while out of reach", r.max_dq <= cap, f"max Δq/tick={r.max_dq:.2f}°"))
    within = np.all(r.sim.read()[:5] >= r.pipe.lo - 1.0) and np.all(r.sim.read()[:5] <= r.pipe.hi + 1.0)
    results.append(_check("inside joint limits", within))
    edge = r.tcp()
    r.ramp(r.phone.p + np.array([0.0, 0.0, +0.08]), 0.5, then=0.5)   # retreat 8 cm
    moved = np.linalg.norm(r.tcp() - edge)
    results.append(_check("responds immediately on retreat (no dead zone)", moved > 0.03,
                          f"moved {moved * 100:.1f} cm"))

    # 3b ─ VIO teleport (relocalisation): absorbed, arm does not lurch
    print("3b. VIO teleport")
    r.phone.clutch = False; r.run(0.3); r.phone.clutch = True; r.run(0.2)
    pj = r.tcp(); r.max_dq = 0.0
    r.phone.p = r.phone.p + np.array([0.6, 0.4, 0.5])           # instantaneous 0.9 m jump
    r.run(0.5)
    results.append(_check("teleport absorbed", np.linalg.norm(r.tcp() - pj) < 0.01 and r.max_dq <= cap,
                          f"moved {np.linalg.norm(r.tcp() - pj) * 1000:.1f} mm"))

    # 4 ── gripper presets are continuous, not binary
    print("4. gripper")
    for pct in (0, 25, 50, 75, 100):
        r.phone.gripper = pct; r.run(1.2)
        g = r.sim.read()[5]
        results.append(_check(f"gripper {pct:3d}% → {g:5.1f}%", abs(g - pct) < 4))

    # 5 ── stale packets are treated as clutch off
    print("5. stale phone packets")
    r.phone.clutch = True; r.run(0.2)
    p2 = r.tcp()
    for _ in range(int(1.0 * HZ)):                   # phone stops updating (its _rx freezes)
        r.t += DT
        m = r.phone.msg(r.t - 1.0); m["p"] = [0.0, 0.5, 0.0]   # old packet, big pose
        q_cmd, info = r.pipe.step(m, r.sim.read(), now=r.t)
        r.sim.command(q_cmd); r.sim.advance(DT)
    results.append(_check("stale → arm stays put", np.linalg.norm(r.tcp() - p2) < 0.005,
                          f"moved {np.linalg.norm(r.tcp() - p2) * 1000:.1f} mm"))

    # 6 ── orientation: free mode follows phone pitch; lock ignores it
    print("6. orientation lock vs free")
    from mr.so3 import matrix_exp3
    def run_pitch(lock):
        rr = Runner(); rr.run(0.3); rr.phone.lock = lock; rr.phone.clutch = True; rr.run(0.1)
        R_before = rr.pipe.fk_deg(rr.sim.read())[1]
        a_before = rr.pipe.kin.approach_axis(rr.pipe.jmap.arm_to_rad(rr.sim.read()[:5]))
        # rotate the phone 0.5 rad about its {ar} x axis (a pitch), position fixed
        h, th = np.array([1.0, 0, 0]), 0.5
        Rm = matrix_exp3(h, th)
        # quaternion xyzw for Rm about x
        q = [np.sin(th / 2), 0, 0, np.cos(th / 2)]
        n = int(3 * HZ)
        for i in range(n):
            rr.t += DT
            a = th * min(1.0, (i + 1) / (HZ * 1.0)) / 2          # 1 s ramp to 0.5 rad
            m = rr.phone.msg(rr.t); m["q"] = [np.sin(a), 0, 0, np.cos(a)]
            qc, info = rr.pipe.step(m, rr.sim.read(), now=rr.t)
            rr.sim.command(qc); rr.sim.advance(DT)
        a_after = rr.pipe.kin.approach_axis(rr.pipe.jmap.arm_to_rad(rr.sim.read()[:5]))
        return np.degrees(np.arccos(np.clip(a_before @ a_after, -1, 1)))
    ang_lock, ang_free = run_pitch(True), run_pitch(False)
    results.append(_check("lock: approach axis unchanged", ang_lock < 3, f"rotated {ang_lock:.1f}°"))
    results.append(_check("free: approach axis follows phone (joint-limited)", ang_free > 8,
                          f"rotated {ang_free:.1f}°"))

    # 7 ── episode lifecycle on disk
    print("7. episode logger")
    tmp = pathlib.Path(tempfile.mkdtemp(prefix="so101_selftest_"))
    log = EpisodeLogger(tmp, session="t", hz=HZ)
    rr = Runner(); rr.phone.clutch = True
    def episode(outcome, n=40):
        log.start("pick up the red cube")
        for i in range(n):
            rr.t += DT
            rr.phone.p = np.array([0, 0.0005 * i, -0.001 * i])
            msg = rr.phone.msg(rr.t)
            qc, info = rr.pipe.step(msg, rr.sim.read(), now=rr.t)
            rr.sim.command(qc); rr.sim.advance(DT)
            log.step(rr.sim.read(), qc, info, msg)
    episode("x"); m1 = log.end("success")
    episode("x"); m2 = log.end("failure")
    episode("x"); log.discard()
    eps = sorted(p.name for p in (tmp / "t").glob("ep_*"))
    results.append(_check("success+failure kept, discard removed", eps == ["ep_0000", "ep_0001"], str(eps)))
    z = np.load(tmp / "t" / "ep_0000" / "steps.npz")
    ok_shapes = (z["q"].shape == (40, 6) and z["q_cmd"].shape == (40, 6) and z["tcp_rot6d"].shape == (40, 6)
                 and z["terminal"].sum() == 1 and z["terminal"][-1] == 1 and int(z["outcome"]) == 1)
    results.append(_check("steps.npz schema + terminal flag", ok_shapes, f"keys={len(z.files)}"))
    z2 = np.load(tmp / "t" / "ep_0001" / "steps.npz")
    results.append(_check("failure labelled 0", int(z2["outcome"]) == 0 and m2["outcome"] == "failure"))
    idx = [json.loads(l) for l in (tmp / "t" / "episodes.jsonl").read_text().splitlines()]
    results.append(_check("episodes.jsonl index", [i["outcome"] for i in idx] == ["success", "failure"]))

    # 8 ── the real control loop, driven by UI events, writes episodes to disk
    print("8. run_loop integration (events -> episodes)")
    from .run_teleop import parse_args, run_loop
    tmp2 = pathlib.Path(tempfile.mkdtemp(prefix="so101_loop_"))
    args = parse_args(["--backend", "sim", "--no-viewer", "--hz", str(HZ),
                       "--data-dir", str(tmp2), "--session", "s", "--task", "stack the cubes"])
    fs = FakeServer(); ph = Phone(); ph.clutch = True
    schedule = {5: ("start", {"instruction": "put the cup on the plate"}),
                35: ("success", {}), 40: ("start", {}), 60: ("fail", {}),
                65: ("start", {}), 75: ("discard", {})}
    class Srv(FakeServer):
        tick = 0
        def latest(self):
            self.tick += 1
            ph.p = np.array([0.0, 0.0, -0.002 * self.tick])
            return ph.msg(time.monotonic())
        def take_events(self):
            ev = schedule.get(self.tick)
            return [ev] if ev else []
    sim2 = SimBackend(viewer=False)
    run_loop(args, sim2, Srv(), max_ticks=90)
    d = tmp2 / "s"
    eps = sorted(p.name for p in d.glob("ep_*"))
    results.append(_check("loop wrote success + failure, discarded one", eps == ["ep_0000", "ep_0001"], str(eps)))
    m0 = json.loads((d / "ep_0000" / "meta.json").read_text())
    m1 = json.loads((d / "ep_0001" / "meta.json").read_text())
    results.append(_check("instruction from UI recorded; later eps reuse it",
                          m0["instruction"] == "put the cup on the plate"
                          and m1["instruction"] == "put the cup on the plate"))
    results.append(_check("labels", (m0["outcome"], m1["outcome"]) == ("success", "failure"),
                          f"{m0['n_steps']} / {m1['n_steps']} steps"))

    # 9 ── websocket server: phone messages in, events out, camera hook
    print("9. websocket server + camera hook")
    results.append(_check("server round trip", _server_roundtrip()))

    n_ok, n = sum(results), len(results)
    print(f"\n{'ALL PASS' if n_ok == n else 'FAILURES'}: {n_ok}/{n}   ({time.time() - t_start:.1f}s)")
    return 0 if n_ok == n else 1


def _server_roundtrip():
    import asyncio
    import ssl as _ssl

    import aiohttp

    from .server.server import TeleopServer
    port = 8979
    srv = TeleopServer(port=port)
    srv.start()
    time.sleep(0.8)

    async def go():
        ctx = _ssl.create_default_context()
        ctx.check_hostname, ctx.verify_mode = False, _ssl.CERT_NONE
        async with aiohttp.ClientSession(connector=aiohttp.TCPConnector(ssl=ctx)) as s:
            async with s.ws_connect(f"https://localhost:{port}/ws") as ws:
                await ws.send_json({"type": "xr", "p": [0, 1, 0], "q": [0, 0, 0, 1],
                                    "clutch": True, "gripper": 33, "orient_lock": False, "tracked": True})
                await ws.send_json({"type": "event", "name": "start", "instruction": "hi"})
                await asyncio.sleep(0.2)
                srv.push_status(state="recording", ep=3, ep_t=1.5)
                msg = await asyncio.wait_for(ws.receive_json(), 2.0)
            cams0 = await (await s.get(f"https://localhost:{port}/cams")).json()
            srv.push_frame("wrist", b"\xff\xd8fakejpeg\xff\xd9")
            cams1 = await (await s.get(f"https://localhost:{port}/cams")).json()
            page = await s.get(f"https://localhost:{port}/")
            return msg, cams0, cams1, page.status
    msg, cams0, cams1, status = asyncio.run(go())
    m = srv.latest()
    ev = srv.take_events()
    ok = (m and m["gripper"] == 33 and m["orient_lock"] is False and "_rx" in m
          and ev and ev[0][0] == "start" and ev[0][1]["instruction"] == "hi"
          and msg.get("state") == "recording"
          and cams0 == {"wrist": False, "bench": False} and cams1["wrist"] and not cams1["bench"]
          and status == 200)
    return bool(ok)
