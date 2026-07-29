"""Figure: phase portraits of the simple pendulum, undamped vs damped.

A second-order system thetaddot = f(theta, thetadot) is rewritten as a pair of
first-order equations

    x1 = theta,      x1dot = x2
    x2 = thetadot,   x2dot = f(x1, x2)

so the state is a POINT in the (theta, thetadot) plane and the dynamics is a
VECTOR FIELD on that plane. That picture is the phase portrait.

With m = l = g = 1:   thetaddot = -sin(theta) - b*thetadot

Left  (b = 0):  conservative. Total energy E = 0.5*thetadot^2 - cos(theta) is
                constant along trajectories, so trajectories ARE the level sets
                of E. Nothing converges; the downright fixed points are stable
                i.s.L. but NOT asymptotically stable. The level set E = mgl = 1
                is the homoclinic orbit -- the trajectory that leaves the upright
                equilibrium and comes back to it, taking infinite time at each
                end. It is the separatrix between swinging and spinning.

Right (b > 0):  damping breaks conservation; E decreases, trajectories spiral
                into the downright fixed points, which become asymptotically
                (in fact exponentially) stable.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})

BLUE, ORANGE, GREY, RED = "#2b6cb0", "#dd6b20", "#4a5568", "#c53030"

TH_LIM, TD_LIM = 2 * np.pi, 3.2


def energy(th, td):
    """E = 0.5*m*l^2*thetadot^2 - m*g*l*cos(theta), with m = l = g = 1."""
    return 0.5 * td**2 - np.cos(th)


def rhs(th, td, b):
    return td, -np.sin(th) - b * td


def rk4(th, td, b, dt, n):
    ths, tds = [th], [td]
    for _ in range(n):
        k1 = rhs(th, td, b)
        k2 = rhs(th + 0.5 * dt * k1[0], td + 0.5 * dt * k1[1], b)
        k3 = rhs(th + 0.5 * dt * k2[0], td + 0.5 * dt * k2[1], b)
        k4 = rhs(th + dt * k3[0], td + dt * k3[1], b)
        th = th + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        td = td + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        ths.append(th)
        tds.append(td)
    return np.array(ths), np.array(tds)


def setup(ax, title):
    ax.set_xlim(-TH_LIM, TH_LIM)
    ax.set_ylim(-TD_LIM, TD_LIM)
    ax.set_xticks([k * np.pi for k in range(-2, 3)])
    ax.set_xticklabels([r"$-2\pi$", r"$-\pi$", "0", r"$\pi$", r"$2\pi$"])
    ax.set_xlabel(r"$\theta$   (0 = hanging down,  $\pm\pi$ = upright)")
    ax.set_ylabel(r"$\dot{\theta}$")
    ax.set_title(title, fontsize=11)
    ax.grid(alpha=0.2, lw=0.5)


def vector_field(ax, b):
    TH, TD = np.meshgrid(np.linspace(-TH_LIM, TH_LIM, 26),
                         np.linspace(-TD_LIM, TD_LIM, 18))
    U, V = TD, -np.sin(TH) - b * TD
    ax.quiver(TH, TD, U, V, color="#a0aec0", alpha=0.85, width=0.0032,
              zorder=1)


def fixed_points(ax):
    """Even multiples of pi = downright (stable). Odd = upright (saddle)."""
    for k in range(-2, 3):
        th = k * np.pi
        upright = k % 2 != 0
        ax.plot(th, 0, "o", ms=10, zorder=8,
                mfc=("white" if upright else GREY), mec=GREY, mew=2.0)


fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.4))

# ---------------- undamped ----------------
ax = axes[0]
setup(ax, "(a) undamped  ($b = 0$)  —  conservative\n"
          "trajectories are level sets of the energy $E$")
vector_field(ax, 0.0)

th = np.linspace(-TH_LIM, TH_LIM, 700)
td = np.linspace(-TD_LIM, TD_LIM, 700)
TH, TD = np.meshgrid(th, td)
E = energy(TH, TD)
ax.contour(TH, TD, E, levels=np.linspace(-1, 6, 17), colors="#4a5568",
           linewidths=0.7, alpha=0.55, zorder=2, linestyles="solid")
# the homoclinic orbit: E = mgl = 1
ax.contour(TH, TD, E, levels=[1.0], colors=[RED], linewidths=2.6, zorder=5)
fixed_points(ax)

# INSIDE the homoclinic orbit -> closed orbits -> swinging back and forth.
# OUTSIDE it (either sign of thetadot) -> the pendulum goes over the top forever.
lbl = dict(ha="center", fontsize=9, color=GREY, zorder=7,
           bbox=dict(boxstyle="round,pad=0.22", fc="white", ec="none", alpha=0.88))
ax.text(0, 1.05, "swinging (libration)", **lbl)
ax.text(0, 2.95, "spinning (rotation)", **lbl)
ax.text(0, -2.9, "spinning, other direction", **lbl)
ax.annotate("homoclinic orbit\n$E = mgl$", xy=(-np.pi / 2, 1.62),
            xytext=(-5.6, 2.35), color=RED, fontsize=9.5, weight="bold",
            arrowprops=dict(arrowstyle="->", color=RED, lw=1.4))

# ---------------- damped ----------------
ax = axes[1]
setup(ax, "(b) damped  ($b = 0.35$)  —  dissipative\n"
          "downright becomes asymptotically stable")
vector_field(ax, 0.35)

for th0, td0 in [(-5.6, 0.5), (-3.0, 2.2), (0.6, 2.6), (2.4, 1.4),
                 (4.2, -2.4), (-1.2, -2.7), (5.9, 0.9)]:
    ths, tds = rk4(th0, td0, 0.35, 0.01, 4200)
    keep = np.abs(tds) < TD_LIM * 1.4
    ax.plot(ths[keep], tds[keep], color=BLUE, lw=1.5, alpha=0.9, zorder=4)
    ax.plot(th0, td0, "o", color=BLUE, ms=4, zorder=5)
fixed_points(ax)
ax.text(0, -2.85, "energy drains away → spirals in", ha="center", fontsize=9,
        color=BLUE, weight="bold",
        bbox=dict(boxstyle="round,pad=0.22", fc="white", ec="none", alpha=0.85))

fig.suptitle("Phase portraits of the simple pendulum   "
             r"($\ddot{\theta} = -\sin\theta - b\,\dot{\theta}$)",
             fontsize=12.5, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.95])
out = __file__.replace("gen_02_phase_portraits.py", "02_phase_portraits.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
