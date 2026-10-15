"""Read-only: does the real arm's joint zero agree with the kinematic model?

Torque is never enabled — the arm stays limp, you pose it by hand. Prints the
measured joints and what FK says about them; with --viewer (needs mjpython) a
digital twin mirrors the real arm so you can compare the two *visually*.

    .venv/bin/python  -m pick_place_SO101.tools.check_mapping --port /dev/tty.usbmodem...
    .venv/bin/mjpython -m pick_place_SO101.tools.check_mapping --port ... --viewer

How to use it:
  1. Pose the arm in the folded rest. TCP should read ~ (0.13, 0, 0.0), approach
     pointing forward-ish and horizontal. Gross disagreement => wrong sign.
  2. Pose it with the forearm straight up and the upper arm horizontal-forward
     (the URDF "zero"; TCP ~ (0.39, 0, 0.25), approach straight up). Every joint
     should read ~0. A joint that reads +6 deg there has an offset: put
     {"<joint>": {"offset_deg": -6}} in pick_place_SO101/joint_map.json
     (rad = radians(sign*deg + offset_deg)). Same for a flipped sign.
  3. Compare the twin to the real arm in 3-4 more poses.
"""
import argparse
import time

import numpy as np

from .. import config
from ..kinematics import SO101Kinematics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=config.default_port())
    ap.add_argument("--robot-id", default=config.ROBOT_ID)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--viewer", action="store_true")
    args = ap.parse_args()
    if not args.port:
        raise SystemExit("no --port (ls /dev/tty.usbmodem*)")

    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.robot_id, use_degrees=True))
    robot.bus.connect()                      # bus only: no configure(), torque untouched
    kin, jm = SO101Kinematics(), config.JointMap()

    twin = None
    if args.viewer:
        from ..backends.sim import SimBackend
        twin = SimBackend(viewer=True, display_only=True)
        twin.connect()

    print("joint order:", config.JOINTS)
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds and (twin is None or twin.running()):
            obs = robot.bus.sync_read("Present_Position")
            q = np.array([obs[j] for j in config.JOINTS], float)
            qr = jm.arm_to_rad(q[:5])
            p, _ = kin.fk(qr)
            el = np.degrees(np.arcsin(np.clip(kin.approach_axis(qr)[2], -1, 1)))
            print("q[deg/%] " + " ".join(f"{v:7.1f}" for v in q)
                  + f" | tcp x={p[0]:+.3f} y={p[1]:+.3f} z={p[2]:+.3f} | approach {el:+5.0f}° "
                  + ("below" if el < 0 else "above") + "     ", end="\r", flush=True)
            if twin is not None:
                twin.command(q)
                twin.sync_viewer()
            time.sleep(0.05)
    finally:
        print()
        robot.bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
