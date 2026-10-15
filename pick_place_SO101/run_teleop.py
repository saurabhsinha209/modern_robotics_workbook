"""SO-101 phone teleop — entry point.

    # validate the phone stream first (no robot, no sim):
    .venv/bin/python -m pick_place_SO101.run_teleop --validate

    # dry run in sim (physics + viewer; macOS viewer needs mjpython):
    .venv/bin/mjpython -m pick_place_SO101.run_teleop --backend sim

    # the real arm (+ optional digital-twin viewer of the measured joints):
    .venv/bin/python   -m pick_place_SO101.run_teleop --backend real --port /dev/tty.usbmodem...
    .venv/bin/mjpython -m pick_place_SO101.run_teleop --backend real --twin

    # no-hardware, no-phone self-test of the whole pipeline:
    .venv/bin/python -m pick_place_SO101.run_teleop --selftest
"""
import argparse
import time

import numpy as np

from . import config
from .backends.base import Backend
from .episodes import EpisodeLogger
from .pipeline import TeleopPipeline


def banner(server, args):
    print("=" * 66)
    print(f"  SO-101 phone teleop · backend={args.backend}"
          f"{' + twin' if args.twin else ''} · {args.hz:.0f} Hz")
    print("-" * 66)
    print(f"  Phone (WebXR, Android Chrome): https://{server.ip}:{server.port}/")
    print("  Accept the self-signed cert once, then Start AR session.")
    print("  1. stand behind the arm, point the phone where the arm points, ⟳ SET FWD")
    print("  2. CLUTCH on → move → CLUTCH off to reposition (the ratchet)")
    print("  3. START → do the task → SUCCESS or FAIL  (DISCARD drops it)")
    print("-" * 66)
    print("  Ctrl-C: arm is parked (folded) before torque is released.")
    print("=" * 66)


def run_validate(server):
    print("[validate] waiting for phone pose… (Ctrl-C to stop)")
    while True:
        m = server.latest()
        if m:
            p = m["p"]
            print(f"p={p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f}  clutch={'ON ' if m.get('clutch') else 'off'}"
                  f"  grip={m.get('gripper', '-'):>3}  lock={m.get('orient_lock')}"
                  f"  {'6-DoF' if m.get('tracked') else '3-DoF(no pos!)'}   ", end="\r", flush=True)
        time.sleep(0.1)


def run_loop(args, backend: Backend, server, twin: Backend | None = None,
             should_stop=lambda: False, max_ticks=None):
    """The control loop. Returns the pipeline (for tests)."""
    dt = 1.0 / args.hz
    pipe = TeleopPipeline(hz=args.hz, scale=args.scale, azimuth=args.azimuth,
                           smooth_s=args.smooth_ms / 1000.0,
                          max_speed_dps=args.max_speed, robot_id=args.robot_id)
    log = None
    if not args.no_record:
        log = EpisodeLogger(args.data_dir, session=args.session, robot_id=args.robot_id,
                            hz=args.hz, extra_meta={"backend": backend.name,
                                                    "scale": args.scale, "azimuth": args.azimuth,
                                                    "smooth_ms": args.smooth_ms,
                                                    "max_speed_dps": args.max_speed})
        print(f"[rec] session {log.dir}")
    instruction = args.task
    server.set_instruction(instruction)

    backend.connect()
    if twin is not None:
        twin.connect()
    q = backend.read()
    pipe.reset(q)
    backend.command(q)                     # first command: hold exactly where we are
    print("[start] measured:", np.round(q, 1))

    fails, tick, last_status, was_stale, last_resync = 0, 0, 0.0, False, -9.0
    q_meas, msg = q, None
    nxt = time.perf_counter()
    try:
        while backend.running() and not should_stop():
            if max_ticks is not None and tick >= max_ticks:
                break
            # ---- events from the phone UI ----
            for name, payload in server.take_events():
                if name == "start" and log and not log.active:
                    if payload.get("instruction"):
                        instruction = payload["instruction"]
                        server.set_instruction(instruction)
                    d = log.start(instruction)
                    print(f"[rec] START ep {d.name}: {instruction!r}")
                elif name in ("success", "fail") and log and log.active:
                    meta = log.end("success" if name == "success" else "failure")
                    print(f"[rec] {meta['outcome'].upper()} ep {meta['episode']} "
                          f"{meta['n_steps']} steps {meta['duration_s']:.1f}s "
                          f"ik_fail={meta['ik_fail_steps']} held={meta['held_steps']}")
                elif name == "discard":
                    if log and log.active:
                        log.discard()
                        print("[rec] DISCARD")
                elif name == "home":
                    tgt = config.HOME_Q.copy()
                    if msg is not None:                # recovery must not open/close the jaw
                        tgt[5] = float(msg.get("gripper", tgt[5]))
                    pipe.goto(tgt)
                    print("[home] returning to HOME_Q (gripper unchanged)")
                elif name == "setforward":
                    print("[map] set forward:", pipe.mapper.set_forward())

            # ---- sense → map → IK → guard → act ----
            try:
                q_meas = backend.read()
                msg = server.latest()
                q_cmd, info = pipe.step(msg, q_meas)
                backend.command(q_cmd)
                fails = 0
            except Exception as e:                     # bus hiccup: hold, don't die
                fails += 1
                print(f"[warn] backend I/O error ({fails}): {e}")
                if fails >= 10:
                    raise SystemExit("too many consecutive I/O errors — stopping")
                q_cmd, info = pipe.q_ref.copy(), None
            backend.advance(dt)
            if info is not None:
                backend.set_markers(info.tgt_p, info.tcp_p)
                if twin is not None:
                    twin.command(q_meas)
                    twin.set_markers(info.tgt_p, info.tcp_p)
                    twin.sync_viewer()
                backend.sync_viewer()
                if log and log.active:
                    log.step(q_meas, q_cmd, info, msg)

                now = time.monotonic()
                if info.resynced:
                    last_resync = now
                    print("[guard] resync: arm lagged the command >15° (stall/overload); re-latched")
                if info.stale != was_stale:
                    was_stale = info.stale
                    print("[link] phone stream STALE (arm holding)" if was_stale
                          else "[link] phone stream resumed")
                if now - last_status > 0.1:
                    last_status = now
                    server.push_status(
                        state="recording" if (log and log.active) else "idle",
                        ep=(log.next_idx if log else 0),
                        ep_t=(log.elapsed if log else 0.0),
                        instruction=instruction, ik_ok=info.ik_ok, held=info.held,
                        rate_limited=info.rate_limited, homing=info.homing,
                        link_ms=(int(1000 * (now - msg["_rx"])) if msg else None),
                        tracked=(bool(msg.get("tracked", True)) if msg else False),
                        vio_jump=info.vio_jump, resynced=(now - last_resync < 1.0),
                        n_success=(log.counts["success"] if log else 0),
                        n_fail=(log.counts["failure"] if log else 0))
            tick += 1
            nxt += dt
            sleep = nxt - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                nxt = time.perf_counter()              # overran; don't spiral
    finally:
        if log and log.active:
            meta = log.end("aborted")
            print(f"[rec] aborted in-progress episode {meta['episode']} (kept, outcome=aborted)")
        _park(args, backend, pipe, dt)
        backend.disconnect()
        if twin is not None:
            twin.disconnect()
    return pipe


def _park(args, backend, pipe, dt):
    """Fold the arm to rest before torque is released, so it doesn't drop."""
    if not args.park or args.backend != "real":
        return
    try:
        q = backend.read()
        pipe.reset(q)
        tgt = q.copy()
        for i, v in enumerate(config.PARK_Q):
            if v is not None:
                tgt[i] = v
        pipe.goto(tgt)
        for _ in range(int(8.0 / dt)):
            q_cmd, info = pipe.step(None, backend.read())
            backend.command(q_cmd)
            time.sleep(dt)
            if not pipe.homing:
                break
        print("[park] folded to rest")
    except Exception as e:
        print(f"[park] skipped ({e})")


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=["sim", "real"], default="sim")
    ap.add_argument("--port", default=config.default_port(), help="serial port (real)")
    ap.add_argument("--robot-id", default=config.ROBOT_ID)
    ap.add_argument("--hz", type=float, default=config.CONTROL_HZ)
    ap.add_argument("--smooth-ms", type=float, default=config.SMOOTH_S * 1000,
                    help="phone jitter-buffer delay in ms (0 = off)")
    ap.add_argument("--scale", type=float, default=config.PHONE_SCALE, help="phone->arm travel")
    ap.add_argument("--azimuth", type=float, default=0.0,
                    help="deg; direction 'phone forward' maps to in the base frame (0 = arm +x)")
    ap.add_argument("--max-speed", type=float, default=config.MAX_JOINT_SPEED_DPS, help="deg/s per joint")
    ap.add_argument("--task", default="pick up the object and place it in the bin")
    ap.add_argument("--data-dir", default=str(config.ROOT / "data"))
    ap.add_argument("--session", default=None, help="session name (default timestamp; reuse to append)")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--twin", action="store_true", help="real backend: also show a MuJoCo digital twin")
    ap.add_argument("--no-park", dest="park", action="store_false")
    ap.add_argument("--port-https", type=int, default=8443)
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    return ap.parse_args(argv)


def main():
    args = parse_args()
    if args.selftest:
        from .selftest import run_selftest
        raise SystemExit(run_selftest())

    from .server.server import TeleopServer
    server = TeleopServer(port=args.port_https)
    server.start()
    banner(server, args)
    try:
        if args.validate:
            run_validate(server)
            return
        if args.backend == "real":
            if not args.port:
                raise SystemExit("no --port and SO101_PORT unset (ls /dev/tty.usbmodem*)")
            from .backends.real import RealBackend
            backend = RealBackend(args.port, robot_id=args.robot_id)
            twin = None
            if args.twin and not args.no_viewer:
                from .backends.sim import SimBackend
                twin = SimBackend(viewer=True, display_only=True)
        else:
            from .backends.sim import SimBackend
            backend = SimBackend(viewer=not args.no_viewer)
            twin = None
        run_loop(args, backend, server, twin)
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
