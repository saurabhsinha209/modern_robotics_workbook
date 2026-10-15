"""Jitter buffer for the phone pose.

Measured on logged teleop runs: ~25 % of 30 Hz control ticks saw no fresh phone
packet and some gaps ran 3-6 ticks (100-200 ms), then two packets landed in one
tick. Sampling "latest packet" at a fixed tick turns a smooth hand motion into
stair-steps (zero step, then double step) -- that is the jitter.

Fix: keep recent packets, place each on the phone's own timeline (its XR frame
timestamp `t`), and render the pose `delay` seconds in the past by interpolating
between the two packets that bracket that instant. Arrival-time jitter vanishes;
the price is `delay` of latency.

A history entry is (t_ms, rx_s, p(3), q_xyzw(4)).
"""
import numpy as np


def _slerp(q0, q1, u):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(q0 @ q1)
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
    else:
        th = np.arccos(d)
        q = (np.sin((1 - u) * th) * q0 + np.sin(u * th) * q1) / np.sin(th)
    return q / np.linalg.norm(q)


class PoseJitterBuffer:
    def __init__(self, delay_s=0.06, window_s=2.0, pos_jump=0.10, extrap_s=0.05):
        self.delay_s, self.window_s = delay_s, window_s
        self.pos_jump, self.extrap_s = pos_jump, extrap_s
        self._off = None            # mac_time ~= phone_time + off  (min-delay estimate)
        self._t_last = None

    def reset(self):
        self._off, self._t_last = None, None

    def _update_offset(self, hist, now):
        cand = min(rx - t / 1000.0 for t, rx, _, _ in hist)
        if self._off is None or cand < self._off:
            self._off = cand
        else:   # let the estimate rise slowly (clock drift) instead of jumping
            dt = 0.0 if self._t_last is None else max(0.0, now - self._t_last)
            self._off += min(cand - self._off, 0.05 * dt)
        self._t_last = now

    def render(self, hist, now):
        """-> (p, q_xyzw) at (now - delay) on the phone timeline, or None."""
        if len(hist) < 2:
            return None
        hist = [h for h in hist if now - h[1] <= self.window_s] or hist[-2:]
        if len(hist) < 2:
            return None
        self._update_offset(hist, now)
        T = [t / 1000.0 + self._off for t, _, _, _ in hist]     # packet times, mac clock
        tr = now - self.delay_s
        if tr <= T[0]:
            return np.asarray(hist[0][2], float), np.asarray(hist[0][3], float)
        if tr >= T[-1]:
            # buffer ran dry: hold, or extrapolate a little along the last velocity
            p1, p0 = np.asarray(hist[-1][2], float), np.asarray(hist[-2][2], float)
            span = max(T[-1] - T[-2], 1e-3)
            if np.linalg.norm(p1 - p0) > self.pos_jump:
                return p1, np.asarray(hist[-1][3], float)
            ahead = min(tr - T[-1], self.extrap_s)
            return p1 + (p1 - p0) / span * ahead, np.asarray(hist[-1][3], float)
        i = int(np.searchsorted(T, tr)) 
        i = min(max(i, 1), len(T) - 1)
        p0, p1 = np.asarray(hist[i - 1][2], float), np.asarray(hist[i][2], float)
        if np.linalg.norm(p1 - p0) > self.pos_jump:               # VIO teleport: snap, don't blend
            return p1, np.asarray(hist[i][3], float)
        u = (tr - T[i - 1]) / max(T[i] - T[i - 1], 1e-6)
        return p0 + u * (p1 - p0), _slerp(hist[i - 1][3], hist[i][3], u)
