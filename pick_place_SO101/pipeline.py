"""TeleopPipeline — phone message -> mapped target -> IK -> guarded joint command.

One object, one `step()` per control tick. Robot-agnostic: it never touches a
backend, only consumes the measured state and returns the command, so the same
code drives the sim and the real arm (and is what the self-test exercises).

State that matters:
  q_ref   last *commanded* joint vector (deg / %).  IK is seeded from it and the
          mapper's engage-reference is FK(q_ref).  Not the measured state: the
          measured state carries gravity sag, and re-deriving the goal from it
          would creep the arm down (P-only servo, goal = measured => no holding
          torque).  Measured state is only used to *resync* if the arm stalls.
"""
from dataclasses import dataclass, field

import numpy as np

from mr.so3 import matrix_log3, matrix_exp3

from . import config
from .kinematics import SO101Kinematics
from .phone_mapping import PhoneMapper
from .safety import JointGuard


@dataclass
class StepInfo:
    engaged: bool = False
    orient_lock: bool = True
    ik_ok: bool = True
    pos_err: float = 0.0
    held: bool = False
    rate_limited: bool = False
    clipped: bool = False
    resynced: bool = False
    vio_jump: bool = False
    stale: bool = False
    homing: bool = False
    tcp_p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tcp_R: np.ndarray = field(default_factory=lambda: np.eye(3))
    tgt_p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tgt_R: np.ndarray = field(default_factory=lambda: np.eye(3))


class TeleopPipeline:
    def __init__(self, hz=config.CONTROL_HZ, scale=config.PHONE_SCALE, azimuth=0.0,
                 max_speed_dps=config.MAX_JOINT_SPEED_DPS, robot_id=config.ROBOT_ID,
                 use_calibration_limits=True, smooth_s=config.SMOOTH_S):
        self.dt = 1.0 / hz
        self.kin = SO101Kinematics()
        self.jmap = config.JointMap()
        self.mapper = PhoneMapper(scale=scale, azimuth=azimuth, smooth_s=smooth_s)

        # limits (deg): MJCF range, tightened to the swept calibration range, minus margin
        lo = self.jmap.rad_to_arm(self.kin.lo)
        hi = self.jmap.rad_to_arm(self.kin.hi)
        lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
        span = config.calibration_half_spans_deg(robot_id) if use_calibration_limits else None
        if span is not None:
            lo, hi = np.maximum(lo, -span), np.minimum(hi, span)
        self.lo = lo + config.LIMIT_MARGIN_DEG
        self.hi = hi - config.LIMIT_MARGIN_DEG
        self.kin.set_limits(self.jmap.arm_to_rad(self.lo), self.jmap.arm_to_rad(self.hi))
        self.guard = JointGuard(self.lo, self.hi, self.dt, max_speed_dps=max_speed_dps)

        self.max_lin = config.CART_MAX_LIN * self.dt
        self.max_ang = config.CART_MAX_ANG * self.dt
        self.q_ref = None             # (6,) commanded, LeRobot units
        self._cmd_p = self._cmd_R = None
        self._goto = None             # (6,) joint-space target while homing/parking
        self._bad = 0                 # consecutive engaged ticks the IK couldn't meet
        self.reanchors = 0

    # ---- helpers ------------------------------------------------------------
    def fk_deg(self, q_deg6):
        return self.kin.fk(self.jmap.arm_to_rad(q_deg6[:5]))

    def reset(self, q_meas):
        self.q_ref = np.asarray(q_meas, float).copy()
        self.q_ref[:5] = np.clip(self.q_ref[:5], self.lo, self.hi)
        self._cmd_p, self._cmd_R = self.fk_deg(self.q_ref)
        self._goto = None
        self.mapper._reset_latch()

    def goto(self, q6):
        """Joint-space move (home / park); teleop is suspended until it arrives."""
        q = np.asarray(q6, float).copy()
        q[:5] = np.clip(q[:5], self.lo, self.hi)
        self._goto = q

    @property
    def homing(self):
        return self._goto is not None

    def _cart_limit(self, p, R):
        dp = p - self._cmd_p
        n = np.linalg.norm(dp)
        if n > self.max_lin:
            dp *= self.max_lin / n
        self._cmd_p = self._cmd_p + dp
        w, th = matrix_log3(R @ self._cmd_R.T)
        th = min(th, self.max_ang)
        if th > 0:
            self._cmd_R = matrix_exp3(w, th) @ self._cmd_R

    # ---- one control tick -----------------------------------------------------
    def step(self, msg, q_meas, now=None):
        """-> (q_cmd (6,) LeRobot units, StepInfo)."""
        info = StepInfo()
        q_meas = np.asarray(q_meas, float)

        # blocked / stalled arm: don't let the command run away from reality
        err = np.abs(self.q_ref[:5] - q_meas[:5]).max()
        if err > config.RESYNC_DEG:
            self.q_ref[:5] = q_meas[:5] + np.clip(self.q_ref[:5] - q_meas[:5],
                                                  -config.RESYNC_DEG / 3, config.RESYNC_DEG / 3)
            self._cmd_p, self._cmd_R = self.fk_deg(self.q_ref)
            self.mapper._reset_latch()
            info.resynced = True

        # --- homing / parking: joint-space ramp, teleop suspended ---
        if self._goto is not None:
            info.homing = True
            q_new, rep = self.guard.arm(self._goto[:5], self.q_ref[:5])
            self.q_ref[:5] = q_new
            self.q_ref[5] = self.guard.gripper(self._goto[5], self.q_ref[5])
            info.rate_limited = rep.rate_limited
            if np.abs(self._goto - self.q_ref).max() < 0.5:
                self._goto = None
            self._cmd_p, self._cmd_R = self.fk_deg(self.q_ref)
            self.mapper._reset_latch()
            info.tcp_p, info.tcp_R = self._cmd_p, self._cmd_R
            info.tgt_p, info.tgt_R = self._cmd_p, self._cmd_R
            return self.q_ref.copy(), info

        # --- normal teleop ---
        ee_p, ee_R = self.fk_deg(self.q_ref)
        out = self.mapper.update(msg, ee_p, ee_R, now=now)
        info.engaged, info.orient_lock = out.engaged, out.orient_lock
        info.stale, info.vio_jump = out.stale, out.jumped
        if not out.engaged:                       # pin the limiter to the commanded pose
            self._cmd_p, self._cmd_R = ee_p.copy(), ee_R.copy()
        self._cart_limit(out.pos, out.R)

        res = self.kin.solve(self.jmap.arm_to_rad(self.q_ref[:5]), self._cmd_p, self._cmd_R,
                             max_travel=np.radians(self.guard.jump / 2))
        q_ik = self.jmap.rad_to_arm(res.q)
        q_new, rep = self.guard.arm(q_ik, self.q_ref[:5])
        self.q_ref[:5] = q_new
        self.q_ref[5] = self.guard.gripper(out.gripper, self.q_ref[5])

        info.ik_ok, info.pos_err = res.ok, res.pos_err
        info.held, info.rate_limited, info.clipped = rep.held, rep.rate_limited, rep.clipped
        info.tcp_p, info.tcp_R = self.fk_deg(self.q_ref)
        info.tgt_p, info.tgt_R = out.pos, out.R

        # target pushed beyond reach for a few ticks -> slide the phone mapping
        self._bad = self._bad + 1 if (out.engaged and not res.ok) else 0
        if self._bad >= 3:
            self.mapper.reanchor(info.tcp_p, info.tcp_R)
            self._cmd_p, self._cmd_R = info.tcp_p.copy(), info.tcp_R.copy()
            self._bad = 0
            self.reanchors += 1
        return self.q_ref.copy(), info
