"""SO-101 forward kinematics + a 5-DOF-aware inverse kinematics.

Why not reuse the Franka IK (`pick_place/ik.py`)? That one solves the full 6-D
pose. The SO-101 has only 5 arm joints:

    shoulder_pan            rotates the whole arm about vertical
    shoulder_lift, elbow_flex, wrist_flex   three PARALLEL pitch joints -> they
                            move the TCP inside one vertical plane
    wrist_roll              spins the gripper about its own approach axis

So the arm can reach any TCP position (3 numbers) plus a gripper *pitch* and a
gripper *roll* -- 5 numbers -- but NOT an arbitrary heading: the approach
direction's azimuth is welded to the pan angle (the "approach azimuth is tied to
shoulder-pan" point in proj_so101_shopping_list.md §5.1). Asking a 6-D solver
for a pose with the wrong heading just makes it fight itself.

So the IK error is 5-D and square:

    rows 1-3   position error                      (m)
    row  4     rotation error about the pitch axis h   (rad)   h = lift joint axis
    row  5     rotation error about the approach axis a (rad)  a = roll joint axis

The third rotation component (about the in-plane axis perpendicular to a) is
simply not controllable and is dropped -- that is the lost "yaw" DOF.
Damped least squares (DLS) keeps the 5x5 solve well-conditioned near singularities.
"""
from dataclasses import dataclass

import numpy as np
import mujoco

from mr.so3 import matrix_log3

from . import config


@dataclass
class IKResult:
    q: np.ndarray            # (5,) joint solution, radians (MJCF convention)
    pos_err: float           # m, residual position error
    pitch_err: float         # rad, residual about the pitch axis
    roll_err: float          # rad, residual about the approach axis
    iters: int
    ok: bool                 # converged to a useful residual


class SO101Kinematics:
    def __init__(self, mjcf=config.MJCF, damping=0.04, max_iters=30,
                 pos_tol=1e-3, rot_tol=1e-2, max_dq=0.3, rot_weight=0.4, min_iters=1):
        self.model = mujoco.MjModel.from_xml_path(str(mjcf))
        self.data = mujoco.MjData(self.model)
        m = self.model
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in config.ARM]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jid])
        self.dadr = np.array([m.jnt_dofadr[j] for j in jid])
        self.lo = m.jnt_range[jid, 0].copy()
        self.hi = m.jnt_range[jid, 1].copy()
        self.sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        self.bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        self.lift_jid = jid[1]
        self.damping, self.max_iters = damping, max_iters
        self.pos_tol, self.rot_tol, self.max_dq = pos_tol, rot_tol, max_dq
        self.min_iters = min_iters          # >=1: always take a step, so sub-tolerance target motion isn't swallowed
        self.rot_weight = rot_weight
        self._jacp = np.zeros((3, m.nv))
        self._jacr = np.zeros((3, m.nv))

    def set_limits(self, lo, hi):
        self.lo, self.hi = np.asarray(lo, float), np.asarray(hi, float)

    # ---- forward kinematics -------------------------------------------------
    def _set(self, q5):
        self.data.qpos[self.qadr] = q5
        mujoco.mj_kinematics(self.model, self.data)

    def fk(self, q5):
        """TCP pose (p (3,), R (3,3)) in the base frame; q5 in radians."""
        self._set(q5)
        return (self.data.site_xpos[self.sid].copy(),
                self.data.site_xmat[self.sid].reshape(3, 3).copy())

    def approach_axis(self, q5):
        """Unit vector along the fingers (= roll-joint axis, pointing at the tip)."""
        self._set(q5)
        return -self.data.xmat[self.bid].reshape(3, 3)[:, 2].copy()

    def pitch_axis(self, q5):
        self._set(q5)
        return self.data.xaxis[self.lift_jid].copy()

    # ---- inverse kinematics -------------------------------------------------
    def solve(self, q_seed, p_t, R_t, max_travel=None) -> IKResult:
        """5-D DLS IK, warm-started from `q_seed` (radians, 5). Never raises.

        `max_travel` (rad, per joint) is a trust region around the seed: the
        returned solution never lies further than this from `q_seed`. Far-away
        targets are then pursued *incrementally* over successive ticks instead of
        as one big jump that a downstream guard would have to reject.
        """
        m, d = self.model, self.data
        q_seed = np.clip(np.asarray(q_seed, float), self.lo, self.hi)
        q = q_seed.copy()
        W = np.array([1, 1, 1, self.rot_weight, self.rot_weight])
        lam2 = self.damping ** 2
        pos_err = np.inf
        e_h = e_a = 0.0
        it = 0
        for it in range(self.max_iters):
            d.qpos[self.qadr] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            p = d.site_xpos[self.sid]
            R = d.site_xmat[self.sid].reshape(3, 3)
            h = d.xaxis[self.lift_jid]
            a = -d.xmat[self.bid].reshape(3, 3)[:, 2]

            dp = p_t - p
            w_hat, th = matrix_log3(R_t @ R.T)
            omega = w_hat * th                       # world-frame rotation error
            e_h, e_a = float(h @ omega), float(a @ omega)
            pos_err = float(np.linalg.norm(dp))
            if it >= self.min_iters and pos_err < self.pos_tol and abs(e_h) < self.rot_tol and abs(e_a) < self.rot_tol:
                break

            mujoco.mj_jacSite(m, d, self._jacp, self._jacr, self.sid)
            Jp = self._jacp[:, self.dadr]
            Jr = self._jacr[:, self.dadr]
            J = np.vstack([Jp, (h @ Jr)[None], (a @ Jr)[None]])      # (5,5)
            e = np.array([*dp, e_h, e_a])
            Jw, ew = W[:, None] * J, W * e
            dq = Jw.T @ np.linalg.solve(Jw @ Jw.T + lam2 * np.eye(5), ew)
            n = np.linalg.norm(dq)
            if n > self.max_dq:
                dq *= self.max_dq / n
            q = np.clip(q + dq, self.lo, self.hi)

        if max_travel is not None:
            q = q_seed + np.clip(q - q_seed, -max_travel, max_travel)
        ok = pos_err < 0.01 and abs(e_h) < 0.15 and abs(e_a) < 0.15
        return IKResult(q, pos_err, e_h, e_a, it + 1, ok)

    def controllable_error(self, q5, p_t, R_t):
        """(pos_err_vec, e_pitch, e_roll) at q5 — for logging/diagnostics."""
        p, R = self.fk(q5)
        h, a = self.pitch_axis(q5), self.approach_axis(q5)
        w_hat, th = matrix_log3(R_t @ R.T)
        om = w_hat * th
        return p_t - p, float(h @ om), float(a @ om)
