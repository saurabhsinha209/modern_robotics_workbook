"""PhoneMapper — WebXR phone pose -> desired TCP pose, clutched & relative.

Ported from pick_place/teleop_source.py with three deliberate changes:

1. **Reference is the arm's *commanded* pose, not a held target.** On every
   clutch engage the reference E0 is FK(q_commanded) — a pose the arm can reach by
   construction. A target pushed past the workspace edge therefore can't wind up:
   when you release and re-engage, you start from where the arm really is.
   (Disengaged, the output simply equals that pose, so the arm holds still.)
2. **Orientation lock.** With `orient_lock` the phone's rotation is ignored and the
   gripper keeps the orientation it had at engage (so the wrist compensates and
   the fingers keep pointing the same way as the arm moves). Free mode follows the
   phone's rotation — but the SO-101 can only realise pitch + roll of it
   (see kinematics.py), phone yaw is dropped.
3. **Stale-packet safety.** No phone packet for `stale_s` -> treated as clutch off.

Maths is unchanged (Ch 3): motion since engage is measured in the phone's
gravity-aligned AR world frame {ar}, mapped to the robot base frame by R_align:

    p_tgt = p(E0) + s · R_align · (p(t) − p0)
    R_tgt = R_align · (R(t) R0ᵀ) · R_alignᵀ · R(E0)
"""
import time
from dataclasses import dataclass

import numpy as np

from mr.so3 import matrix_log3

from . import config
from .smoothing import PoseJitterBuffer

AR_UP = np.array([0.0, 1.0, 0.0])           # WebXR {ar} is y-up
F0_DEFAULT = np.array([0.0, 0.0, -1.0])     # session-start forward ({ar} -z)


def quat_xyzw_to_mat(q):
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def build_R_align(f0, azimuth_deg):
    """Rotation taking {ar} (y-up) to the robot base frame (z-up).

    `f0` (phone-forward, horizontal in {ar}) maps to the horizontal direction at
    `azimuth_deg` in the base frame (0 deg = the arm's +x, i.e. operator standing
    behind the base looking where the arm points). {ar} up maps to +z.
    """
    a = np.radians(azimuth_deg)
    fh = np.array([np.cos(a), np.sin(a), 0.0])
    up_r = np.array([0.0, 0.0, 1.0])
    r = np.cross(fh, up_r); r /= np.linalg.norm(r)
    M_robot = np.column_stack([r, up_r, fh])

    f0 = np.asarray(f0, float); f0 = f0 / np.linalg.norm(f0)
    right_ar = np.cross(f0, AR_UP); right_ar /= np.linalg.norm(right_ar)
    M_ar = np.column_stack([right_ar, AR_UP, f0])
    return M_robot @ M_ar.T


@dataclass
class MapOut:
    pos: np.ndarray
    R: np.ndarray
    gripper: float           # 0..100 %
    engaged: bool            # clutch actually driving the target this tick
    orient_lock: bool
    tracked: bool
    stale: bool
    jumped: bool = False     # VIO discontinuity absorbed this tick


class PhoneMapper:
    def __init__(self, scale=config.PHONE_SCALE, azimuth=0.0,
                 pos_jump=0.10, ang_jump=0.5, stale_s=config.STALE_S,
                 smooth_s=config.SMOOTH_S):
        self.scale = scale
        self.azimuth = azimuth
        self.stale_s = stale_s
        self.pos_jump, self.ang_jump = pos_jump, ang_jump
        self._f0 = F0_DEFAULT.copy()
        self._Ralign = build_R_align(self._f0, azimuth)
        self.jbuf = PoseJitterBuffer(delay_s=smooth_s, pos_jump=pos_jump) if smooth_s > 0 else None
        self.gripper = 50.0
        self._reset_latch()
        self._last_R = None
        self._prev_engaged = False
        self._prev_lock = True

    def _reset_latch(self):
        self._p0 = self._R0 = self._E0p = self._E0R = None
        self._p_prev = self._R_prev = None

    def set_forward(self):
        """Phone's current forward (device -z, flattened) becomes "away from me"."""
        if self._last_R is None:
            return False
        fwd = self._last_R @ np.array([0.0, 0.0, -1.0])
        fwd[1] = 0.0
        n = np.linalg.norm(fwd)
        if n < 1e-6:
            return False
        self._f0 = fwd / n
        self._Ralign = build_R_align(self._f0, self.azimuth)
        return True

    def reanchor(self, ee_p, ee_R):
        """Slide the mapping: current phone pose <-> current (reachable) EE pose.

        Called when the target has been pushed past what the arm can reach. Like a
        mouse hitting the screen edge, the reference moves so that moving back
        takes effect immediately instead of first "unwinding" the excess.
        """
        if self._p_prev is None:
            return
        self._p0, self._R0 = self._p_prev.copy(), self._R_prev.copy()
        self._E0p, self._E0R = ee_p.copy(), ee_R.copy()

    def update(self, msg, ee_p, ee_R, now=None) -> MapOut:
        """msg: latest phone dict (or None).  ee_p/ee_R: FK of the *commanded* q."""
        now = time.monotonic() if now is None else now
        hold = lambda g, lock, tr, st: MapOut(ee_p.copy(), ee_R.copy(), g, False, lock, tr, st)

        if msg is None:
            self._prev_engaged = False
            self._reset_latch()
            return hold(self.gripper, True, False, True)

        self.gripper = float(np.clip(msg.get("gripper", self.gripper), 0.0, 100.0))
        lock = bool(msg.get("orient_lock", True))
        tracked = bool(msg.get("tracked", True))
        stale = (now - msg.get("_rx", now)) > self.stale_s
        clutch = bool(msg.get("clutch", False)) and not stale

        p = np.asarray(msg["p"], float)
        q = msg["q"]
        if self.jbuf is not None and msg.get("_hist"):
            r = self.jbuf.render(msg["_hist"], now)
            if r is not None:
                p, q = r
        R = quat_xyzw_to_mat(q)
        self._last_R = R

        # (re)latch on rising clutch edge, when the lock mode flips mid-motion, or when
        # the pipeline dropped the latch under a held clutch (resync after a stall)
        if clutch and (not self._prev_engaged or lock != self._prev_lock or self._p0 is None):
            self._p0, self._R0 = p.copy(), R.copy()
            self._E0p, self._E0R = ee_p.copy(), ee_R.copy()
            self._p_prev, self._R_prev = p.copy(), R.copy()
            self._Ralign = build_R_align(self._f0, self.azimuth)
        self._prev_engaged, self._prev_lock = clutch, lock

        if not clutch or self._p0 is None:
            if not clutch:
                self._reset_latch()
            return hold(self.gripper, lock, tracked, stale)

        # reject VIO teleports (loop closure / relocalisation): re-latch, no motion
        jump = not tracked
        if self._p_prev is not None and not jump:
            _, ang = matrix_log3(R @ self._R_prev.T)
            jump = (np.linalg.norm(p - self._p_prev) > self.pos_jump or ang > self.ang_jump)
        if jump:
            self._p0, self._R0 = p.copy(), R.copy()
            self._E0p, self._E0R = ee_p.copy(), ee_R.copy()
            self._p_prev, self._R_prev = p.copy(), R.copy()
            return MapOut(ee_p.copy(), ee_R.copy(), self.gripper, True, lock, tracked, stale, True)

        dp = p - self._p0
        tp = self._E0p + self.scale * (self._Ralign @ dp)
        if lock:
            tR = self._E0R.copy()
        else:
            dR = R @ self._R0.T
            tR = self._Ralign @ dR @ self._Ralign.T @ self._E0R
        self._p_prev, self._R_prev = p.copy(), R.copy()
        return MapOut(tp, tR, self.gripper, True, lock, tracked, stale)
