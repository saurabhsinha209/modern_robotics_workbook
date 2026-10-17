"""Every tunable in one place.

Units at the robot boundary are LeRobot's (`use_degrees=True`): the 5 arm joints
in degrees, the gripper in percent (0 closed .. 100 open). Kinematics works in
the MJCF's radians; `JointMap` converts between the two.
"""
import glob
import json
import os
import pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
REPO = ROOT.parent
MJCF = REPO / "mujoco_menagerie/robotstudio_so101/scene.xml"
JOINT_MAP_FILE = ROOT / "joint_map.json"
CALIB_DIR = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower"

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"
JOINTS = ARM + [GRIPPER]

ROBOT_ID = "so101_follower"

# ready pose: TCP ~20 cm out, ~12 cm up, approach ~55 deg below horizontal
HOME_Q = np.array([0.0, -65.0, 45.0, 75.0, 0.0, 50.0])
# folded rest, kept a few degrees shy of the hard stops; torque goes off from here
PARK_Q = np.array([None, -100.0, 95.0, 75.0, None, 0.0], dtype=object)

# gripper: the UI exposes presets, but the *recorded* quantity is the continuous %
GRIPPER_PRESETS = {"closed": 0, "quarter": 25, "half": 50, "three_quarter": 75, "open": 100}

# where the *sim* arm starts (the physical folded rest)
PARK_Q_START = np.array([0.0, -100.0, 95.0, 75.0, 0.0, 0.0])

# --- control / safety defaults (override from the CLI) ---
CONTROL_HZ = 30.0
MAX_JOINT_SPEED_DPS = 60.0      # per-joint cap on the commanded target
MAX_GRIPPER_SPEED_PCT_S = 250.0
JUMP_DEG = 20.0                 # IK solution this far from last command in one tick -> hold
RESYNC_DEG = 15.0               # commanded vs measured diverged (blocked/stalled) -> resync
LIMIT_MARGIN_DEG = 3.0          # stay this far inside the joint limits
MAX_RELATIVE_TARGET = 8.0       # LeRobot's own per-send cap (second layer)
CART_MAX_LIN = 0.35             # m/s   Cartesian target rate limit
CART_MAX_ANG = 2.0              # rad/s
STALE_S = 0.25                  # no phone packet for this long -> treat as clutch off
SMOOTH_S = 0.06                 # phone-pose jitter buffer delay (0 = off, raw latest packet)
PHONE_SCALE = 1.0               # phone travel -> arm travel


def default_port():
    p = os.environ.get("SO101_PORT")
    if p:
        return p
    found = sorted(glob.glob("/dev/tty.usbmodem*"))
    return found[0] if len(found) == 1 else None


class JointMap:
    """LeRobot units <-> MJCF radians.   rad = radians(sign * deg + offset_deg).

    Identity by default (LeRobot's middle-of-range zero matches the MJCF, which
    comes from the "new calibration" URDF). `tools/check_mapping.py` shows how to
    measure an offset if the real arm disagrees with the twin.
    """
    def __init__(self, path=JOINT_MAP_FILE):
        raw = json.loads(pathlib.Path(path).read_text()) if pathlib.Path(path).exists() else {}
        self.sign = np.array([raw.get(j, {}).get("sign", 1.0) for j in ARM])
        self.offset = np.array([raw.get(j, {}).get("offset_deg", 0.0) for j in ARM])

    def arm_to_rad(self, deg):
        return np.radians(self.sign * np.asarray(deg, float) + self.offset)

    def rad_to_arm(self, rad):
        return self.sign * (np.degrees(np.asarray(rad, float)) - self.offset)


def calibration_half_spans_deg(robot_id=ROBOT_ID):
    """Half the *recorded* sweep of each arm joint, in LeRobot degrees.

    LeRobot centres each joint's range at 0 deg, so the calibrated limits are
    +-(range_max - range_min)/2 * 360/4095. Only as wide as the sweep you did.
    """
    f = CALIB_DIR / f"{robot_id}.json"
    if not f.exists():
        return None
    c = json.loads(f.read_text())
    return np.array([(c[j]["range_max"] - c[j]["range_min"]) / 2 * 360.0 / 4095.0 for j in ARM])

# macOS camera Unique IDs (system_profiler SPCameraDataType). The UVC ID encodes the
# USB port, so these follow the *ports*; swap cables and wrist/bench swap too.
CAMERA_UIDS = {"wrist": "0x110000032e69221", "bench": "0x310000032e69221"}
CAMERA_SIZE = (1280, 720)
CAMERA_FPS = 30

# Episode frames are stored downsized (training) + a full-size debug frame every N steps.
FRAME_SIZE = (320, 240)
FULL_FRAME_EVERY = 30
