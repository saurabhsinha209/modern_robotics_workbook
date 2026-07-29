"""Figure: the set of instantaneously reachable accelerations.

The whole of Chapter 1 in one picture. For a control-affine system

    qddot = f1(q, qdot, t) + f2(q, qdot, t) u

the accelerations you can command *right now* form the set

    { f1 + f2 u  :  u admissible }

which is the column space of f2, shifted to sit at the drift term f1.

  (a) fully actuated   -> f2 has rank 2, columns span R^2  -> the whole plane
  (b) underactuated    -> f2 has rank 1, columns span a line -> a line
  (c) + torque limits  -> the same line, clipped to a segment

Panel (b) also makes the key non-obvious point: losing the shoulder motor does
NOT mean "qddot_1 is stuck at its drift value". Because M^-1 is not diagonal,
the reachable line is *tilted* — elbow torque still accelerates the shoulder
through inertial coupling. That coupling is what swing-up control exploits.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10, "axes.titlesize": 11})

# Drift: the acceleration you get for free, with u = 0 (here, gravity).
f1 = np.array([-2.0, 1.0])

# Underactuated case: B = [0, 1]^T (elbow motor only), and f2 = M^-1 B.
# Numbers match the worked example in 01_underactuation.md: the m=l=1 double
# pendulum at theta_2 = 90 deg, where M = [[3,1],[1,1]] and M^-1 = [[.5,-.5],[-.5,1.5]].
# M^-1 is symmetric positive definite but NOT diagonal, so f2 comes out tilted.
Minv = np.array([[0.5, -0.5], [-0.5, 1.5]])
f2_under = Minv @ np.array([0.0, 1.0])  # -> [-0.5, 1.5]

LIM = 6.0
BLUE, ORANGE, GREY = "#2b6cb0", "#dd6b20", "#4a5568"


def setup(ax, title):
    ax.set_xlim(-LIM, LIM)
    ax.set_ylim(-LIM, LIM)
    ax.set_aspect("equal")
    ax.axhline(0, color="#cbd5e0", lw=0.8, zorder=0)
    ax.axvline(0, color="#cbd5e0", lw=0.8, zorder=0)
    ax.set_xlabel(r"$\ddot{q}_1$  (shoulder accel)")
    ax.set_ylabel(r"$\ddot{q}_2$  (elbow accel)")
    ax.set_title(title)
    ax.grid(alpha=0.25, lw=0.5)


def draw_drift(ax):
    """The u = 0 point: where you drift to if you command nothing."""
    ax.plot(*f1, "o", color=GREY, ms=7, zorder=5)
    ax.annotate(
        r"$f_1$ (drift, $u=0$)",
        f1,
        textcoords="offset points",
        xytext=(8, -14),
        color=GREY,
        fontsize=9,
    )


fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.8))

# ---- (a) fully actuated: rank(f2) = 2 --------------------------------------
ax = axes[0]
setup(ax, "(a) fully actuated\n" + r"rank$(f_2) = 2 = \dim(q)$")
ax.add_patch(
    plt.Rectangle((-LIM, -LIM), 2 * LIM, 2 * LIM, color=BLUE, alpha=0.16, zorder=1)
)
draw_drift(ax)
# a few sample reachable accelerations, to stress "anywhere"
rng = np.random.default_rng(0)
pts = rng.uniform(-LIM * 0.85, LIM * 0.85, size=(14, 2))
ax.plot(pts[:, 0], pts[:, 1], "x", color=BLUE, ms=5, mew=1.2, zorder=4)
ax.text(
    0, -LIM * 0.86, "every $\\ddot{q}$ is reachable", ha="center",
    color=BLUE, fontsize=9.5, weight="bold",
)

# ---- (b) underactuated: rank(f2) = 1 ---------------------------------------
ax = axes[1]
setup(ax, "(b) underactuated (no shoulder motor)\n" + r"rank$(f_2) = 1 < \dim(q)$")
t = np.linspace(-LIM * 2, LIM * 2, 2)
line = f1[:, None] + f2_under[:, None] * t
ax.plot(line[0], line[1], color=ORANGE, lw=2.6, zorder=3)
# the direction vector itself, drawn from f1
ax.annotate(
    "", xy=f1 + f2_under * 2.2, xytext=f1,
    arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=2.2),
)
ax.text(
    *(f1 + f2_under * 2.2 + np.array([-0.9, 0.1])),
    r"$f_2$", color=ORANGE, fontsize=11, weight="bold",
)
draw_drift(ax)
# the unreachable directions
ax.plot(3.0, -3.0, "x", color="#c53030", ms=9, mew=2.4, zorder=5)
ax.text(3.0, -4.0, "unreachable", ha="center", color="#c53030", fontsize=9)
ax.text(
    0, -LIM * 0.86, "only a 1-D line of $\\ddot{q}$", ha="center",
    color=ORANGE, fontsize=9.5, weight="bold", zorder=6,
    bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.9),
)

# ---- (c) underactuated + input limits --------------------------------------
ax = axes[2]
setup(ax, "(c) underactuated + torque limit\n" + r"$|u| \leq 1$")
ax.plot(line[0], line[1], color=ORANGE, lw=1.1, ls=":", zorder=2)
seg = f1[:, None] + f2_under[:, None] * np.array([-1.0, 1.0])
ax.plot(seg[0], seg[1], color=ORANGE, lw=3.4, zorder=3,
        solid_capstyle="round")
ax.plot(seg[0], seg[1], "|", color=ORANGE, ms=12, mew=2.4, zorder=4)
draw_drift(ax)
ax.text(
    0, -LIM * 0.86, "shrinks to a segment", ha="center",
    color=ORANGE, fontsize=9.5, weight="bold",
)

fig.suptitle(
    "Reachable accelerations  $\\{\\,f_1 + f_2 u\\,\\}$ — what 'underactuated' actually means",
    fontsize=12.5, y=0.99,
)
fig.tight_layout(rect=[0, 0, 1, 0.94])
out = __file__.replace("gen_01_reachable_accelerations.py", "01_reachable_accelerations.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
