"""Figure: limit cycles, orbital stability, and the Poincare return map.

The Van der Pol oscillator  qddot + mu(q^2 - 1) qdot + q = 0  is a spring-mass
with NONLINEAR damping: it dissipates energy when |q|>1 and ADDS energy when
|q|<1. So every trajectory (except the origin) spirals onto one closed orbit --
a stable LIMIT CYCLE.

(a) Phase portrait: trajectories from inside and outside both converge to the
    same closed orbit. The orbit is the attractor, not a point.

(b) Time domain: the reason we need a NEW notion of stability. Two trajectories
    converge to the same waveform but stay PHASE-SHIFTED -- |x(t) - x*(t)| does
    NOT go to zero. Stability of a *trajectory in time* is the wrong idea;
    stability of the *orbit* (distance to the nearest point on the cycle) is
    right.

(c) Poincare map: pick a surface of section S (here qdot-axis crossings with
    q=0, going up). Record qdot each crossing: qdot[n] -> qdot[n+1] = P(qdot[n]).
    A closed orbit = a FIXED POINT of P. The staircase shows iterates walking
    into it. Cycle stability (hard, continuous) has become fixed-point stability
    of a 1-D map (easy) -- and |slope| < 1 at the fixed point = stable.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})
BLUE, ORANGE, GREY, RED, GREEN = "#2b6cb0", "#dd6b20", "#4a5568", "#c53030", "#2f855a"

MU = 1.0


def vdp(state, mu=MU):
    q, qd = state
    return np.array([qd, -mu * (q ** 2 - 1) * qd - q])


def sim(s0, T=40.0, dt=0.004):
    s = np.array(s0, float)
    S, TS = [s.copy()], [0.0]
    for i in range(int(T / dt)):
        k1 = vdp(s); k2 = vdp(s + 0.5 * dt * k1)
        k3 = vdp(s + 0.5 * dt * k2); k4 = vdp(s + dt * k3)
        s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        S.append(s.copy()); TS.append((i + 1) * dt)
    return np.array(TS), np.array(S)

fig, axes = plt.subplots(1, 3, figsize=(15, 4.7))

# ---------------- (a) phase portrait ----------------
ax = axes[0]
# vector field
xs = np.linspace(-3.2, 3.2, 20)
X, Y = np.meshgrid(xs, np.linspace(-3.6, 3.6, 20))
U = Y
V = -MU * (X ** 2 - 1) * Y - X
ax.quiver(X, Y, U, V, color="#cbd5e0", alpha=0.9, width=0.004, zorder=1)
# the limit cycle itself (long sim, take the tail)
_, Sc = sim([2.0, 0.0], T=60)
cyc = Sc[-1500:]
ax.plot(cyc[:, 0], cyc[:, 1], color=RED, lw=2.8, zorder=4, label="limit cycle")
# trajectory from OUTSIDE
_, So = sim([3.0, 3.0], T=18)
ax.plot(So[:, 0], So[:, 1], color=BLUE, lw=1.5, zorder=3, alpha=0.9)
ax.plot(*So[0], "o", color=BLUE, ms=6, zorder=5)
# trajectory from INSIDE
_, Si = sim([0.1, 0.1], T=18)
ax.plot(Si[:, 0], Si[:, 1], color=GREEN, lw=1.5, zorder=3, alpha=0.9)
ax.plot(*Si[0], "o", color=GREEN, ms=6, zorder=5)
ax.text(2.7, 3.15, "from outside", color=BLUE, fontsize=9, weight="bold")
ax.text(-0.3, 0.55, "from\ninside", color=GREEN, fontsize=9, weight="bold")
ax.set_xlim(-3.2, 3.2); ax.set_ylim(-3.8, 3.8)
ax.set_xlabel(r"$q$"); ax.set_ylabel(r"$\dot q$")
ax.set_title("(a) all trajectories spiral onto\none closed orbit (the attractor)",
             fontsize=10.5)
ax.legend(loc="lower left", fontsize=8.5, framealpha=0.9)
ax.grid(alpha=0.2, lw=0.5)

# ---------------- (b) time domain: orbital not trajectory ----------------
# start B in ~antiphase so the persistent phase offset is visible: both lock to
# the same waveform, but stay shifted in time forever (phase is neutrally stable).
ax = axes[1]
t1, S1 = sim([2.0, 0.0], T=26)
t2, S2 = sim([-2.0, 0.0], T=26)
ax.plot(t1, S1[:, 0], color=BLUE, lw=1.8, label="trajectory A")
ax.plot(t2, S2[:, 0], color=GREEN, lw=1.8, label="trajectory B")
ax.set_xlim(0, 26); ax.set_ylim(-3, 3)
ax.set_xlabel("time  $t$"); ax.set_ylabel(r"$q(t)$")
ax.set_title("(b) same waveform, different PHASE\n"
             r"$|x(t)-x^*(t)|\not\to 0$ — need orbital stability", fontsize=10.5)
ax.legend(loc="upper right", fontsize=8.5, framealpha=0.9)
ax.grid(alpha=0.25, lw=0.5)

# ---------------- (c) Poincare return map + staircase ----------------
ax = axes[2]
# build the return map by sampling: cross q=0 with qdot>0, record qdot
def return_map_samples():
    pts = []
    for qd0 in np.linspace(0.2, 3.6, 40):
        s = np.array([0.0, qd0]); dt = 0.002; prev = s.copy()
        crossings = []
        for _ in range(int(30 / dt)):
            k1 = vdp(s); k2 = vdp(s + 0.5 * dt * k1)
            k3 = vdp(s + 0.5 * dt * k2); k4 = vdp(s + dt * k3)
            sn = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            if prev[0] < 0 <= sn[0] and sn[1] > 0:      # upward crossing of q=0
                frac = -prev[0] / (sn[0] - prev[0])
                crossings.append(prev[1] + frac * (sn[1] - prev[1]))
                if len(crossings) >= 2:
                    break
            prev = sn; s = sn
        if crossings:
            pts.append((qd0, crossings[0]))
    return np.array(pts)

P = return_map_samples()
ax.plot(P[:, 0], P[:, 1], color=BLUE, lw=2.2, zorder=4, label=r"return map $P$")
lim = 3.6
ax.plot([0, lim], [0, lim], color=RED, lw=1.6, ls="--", zorder=3,
        label="slope-1 line")
# fixed point = intersection
from numpy import interp
grid = np.linspace(P[:, 0].min(), P[:, 0].max(), 400)
Pg = interp(grid, P[:, 0], P[:, 1])
istar = np.argmin(np.abs(Pg - grid))
qstar = grid[istar]
ax.plot(qstar, qstar, "o", color=RED, ms=10, zorder=6, mec="white", mew=1.5)
ax.annotate("stable fixed point\n= the limit cycle", xy=(qstar, qstar),
            xytext=(1.15, 3.05), fontsize=9, color=RED, weight="bold",
            arrowprops=dict(arrowstyle="-|>", color=RED, lw=1.5))
# staircase from qd0 = 3.4
x = 3.4
for _ in range(12):
    y = interp(x, P[:, 0], P[:, 1])
    ax.plot([x, x], [x, y], color=GREY, lw=1.0, zorder=5)   # vertical
    ax.plot([x, y], [y, y], color=GREY, lw=1.0, zorder=5)   # horizontal
    x = y
ax.set_xlim(0, lim); ax.set_ylim(0, lim); ax.set_aspect("equal")
ax.set_xlabel(r"$\dot q_p[n]$"); ax.set_ylabel(r"$\dot q_p[n+1]$")
ax.set_title("(c) Poincaré map: cycle-stability\nbecomes fixed-point stability",
             fontsize=10.5)
ax.legend(loc="lower right", fontsize=8.5, framealpha=0.9)
ax.grid(alpha=0.25, lw=0.5)

fig.suptitle("Limit cycles & the Poincaré map — the stability language for gaits "
             "(Van der Pol oscillator)", fontsize=12.5, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.93])
out = __file__.replace("gen_04a_limit_cycle.py", "04a_limit_cycle.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out, " fixed point qdot* =", round(qstar, 3))
