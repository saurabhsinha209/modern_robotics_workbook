"""MuJoCo SO-101 (menagerie MJCF): position-actuated sim, or a display-only twin.

Used two ways:
  * as the backend for a full dry run — `--backend sim` — with physics, so you can
    test the phone mapping + guards with zero hardware risk;
  * as a *digital twin* of the real arm — `--twin` — fed the measured joints, only
    for looking at it on the Mac while the real arm moves.
Needs `mjpython` for the live viewer on macOS; headless works under plain python.
"""
import numpy as np
import mujoco

from .. import config
from .base import Backend


def _build_model():
    spec = mujoco.MjSpec.from_file(str(config.MJCF))
    for name, rgba in (("target_marker", [1.0, 0.25, 0.2, 0.55]),
                       ("tcp_marker", [0.2, 0.8, 1.0, 0.55])):
        b = spec.worldbody.add_body(name=name, mocap=True)
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.008, 0, 0],
                   rgba=rgba, contype=0, conaffinity=0)
    return spec.compile()


class SimBackend(Backend):
    name = "sim"

    def __init__(self, viewer=True, display_only=False):
        self.model = _build_model()
        self.data = mujoco.MjData(self.model)
        self.jmap = config.JointMap()
        self.want_viewer = viewer
        self.display_only = display_only
        self._viewer = None
        self._ctx = None
        m = self.model
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in config.JOINTS]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jid])
        self.g_lo, self.g_hi = m.jnt_range[jid[5]]
        self._mocap = {n: m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)]
                       for n in ("target_marker", "tcp_marker")}

    # --- unit conversions ---
    def _to_rad(self, q6):
        out = np.empty(6)
        out[:5] = self.jmap.arm_to_rad(q6[:5])
        out[5] = self.g_lo + np.clip(q6[5], 0, 100) / 100.0 * (self.g_hi - self.g_lo)
        return out

    def _to_deg(self, qrad):
        out = np.empty(6)
        out[:5] = self.jmap.rad_to_arm(qrad[:5])
        out[5] = (qrad[5] - self.g_lo) / (self.g_hi - self.g_lo) * 100.0
        return out

    # --- Backend API ---
    def connect(self):
        self.data.qpos[self.qadr] = self._to_rad(config.PARK_Q_START)
        self.data.ctrl[:] = self.data.qpos[self.qadr]
        mujoco.mj_forward(self.model, self.data)
        if self.want_viewer:
            from mujoco import viewer as mjviewer
            try:
                self._ctx = mjviewer.launch_passive(self.model, self.data)
            except RuntimeError as e:
                raise SystemExit(f"{e}\nOn macOS run the viewer under mjpython: "
                                 ".venv/bin/mjpython -m pick_place_SO101.run_teleop ...")
            self._viewer = self._ctx
            self._viewer.cam.azimuth = 0.0          # operator-behind-the-base view
            self._viewer.cam.elevation = -25.0
            self._viewer.cam.distance = 0.9
            self._viewer.cam.lookat[:] = [0.15, 0.0, 0.08]

    def read(self):
        return self._to_deg(self.data.qpos[self.qadr])

    def command(self, q6):
        qr = self._to_rad(np.asarray(q6, float))
        if self.display_only:
            self.data.qpos[self.qadr] = qr
            mujoco.mj_forward(self.model, self.data)
        else:
            self.data.ctrl[:6] = qr

    def advance(self, dt):
        if self.display_only:
            return
        for _ in range(max(1, round(dt / self.model.opt.timestep))):
            mujoco.mj_step(self.model, self.data)

    def set_markers(self, target_p=None, tcp_p=None):
        if target_p is not None:
            self.data.mocap_pos[self._mocap["target_marker"]] = target_p
        if tcp_p is not None:
            self.data.mocap_pos[self._mocap["tcp_marker"]] = tcp_p

    def sync_viewer(self):
        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()

    def viewer_azimuth(self):
        return None if self._viewer is None else float(self._viewer.cam.azimuth)

    def running(self):
        return self._viewer is None or self._viewer.is_running()

    def disconnect(self):
        if self._ctx is not None:
            self._ctx.close()
