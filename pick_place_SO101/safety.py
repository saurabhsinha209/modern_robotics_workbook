"""Guards between IK and the servos — the three mitigations from
proj_so101_shopping_list.md §5.2 that matter for dataset quality:

  1. never emit a discontinuous target  -> `held` (keep last valid)
  2. rate-limit the joint target        -> `rate_limited`
  3. flag, don't silently drop          -> every event is counted per episode

(The fourth — seed IK from the current configuration — lives in the pipeline.)
Everything is in LeRobot units: degrees for the 5 arm joints, percent gripper.
"""
from dataclasses import dataclass

import numpy as np

from . import config


@dataclass
class GuardReport:
    held: bool = False          # IK solution jumped -> kept last command
    rate_limited: bool = False  # joint speed cap bit
    clipped: bool = False       # joint limit margin bit


class JointGuard:
    def __init__(self, lo_deg, hi_deg, dt,
                 max_speed_dps=config.MAX_JOINT_SPEED_DPS,
                 jump_deg=config.JUMP_DEG,
                 gripper_speed=config.MAX_GRIPPER_SPEED_PCT_S):
        self.lo, self.hi = np.asarray(lo_deg, float), np.asarray(hi_deg, float)
        self.dt = dt
        self.max_step = max_speed_dps * dt
        self.jump = jump_deg
        self.g_step = gripper_speed * dt
        self.max_hold_ticks = 3
        self._held_run = 0

    def arm(self, q_ik, q_prev):
        q_ik, q_prev = np.asarray(q_ik, float), np.asarray(q_prev, float)
        rep = GuardReport()
        delta = q_ik - q_prev
        if np.abs(delta).max() > self.jump:
            # Holding forever would deadlock (same seed -> same jumpy solution),
            # so hold for a few ticks to ride out a transient, then fall through
            # to the rate-limited step below.
            self._held_run += 1
            if self._held_run <= self.max_hold_ticks:
                rep.held = True
                return q_prev.copy(), rep
        else:
            self._held_run = 0
        step = np.clip(delta, -self.max_step, self.max_step)
        rep.rate_limited = bool(np.any(np.abs(delta) > self.max_step + 1e-9))
        q = q_prev + step
        qc = np.clip(q, self.lo, self.hi)
        rep.clipped = bool(np.any(np.abs(qc - q) > 1e-9))
        return qc, rep

    def gripper(self, g_target, g_prev):
        return float(np.clip(g_target, g_prev - self.g_step, g_prev + self.g_step)
                     .clip(0.0, 100.0))
