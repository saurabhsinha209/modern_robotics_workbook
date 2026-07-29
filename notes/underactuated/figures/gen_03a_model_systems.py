"""Figure: the three canonical underactuated model systems of Chapter 3.

All three are "trivially underactuated" -- fewer actuators than DOF -- but they
fail in three different ways, which is why the book uses all three:

  Acrobot          q = [th1, th2],  u = elbow torque.   B = [0, 1]^T
                   The unactuated joint is the one attached to the WORLD.

  Cart-Pole        q = [x, th],     u = force on cart.  B = [1, 0]^T
                   The actuated joint is the one attached to the world; the
                   *pole* is passive. Mirror image of the acrobot.

  Planar Quadrotor q = [x, y, th],  u = [u1, u2].       B(q) depends on th!
                   3 DOF, 2 inputs, and B is CONFIGURATION-DEPENDENT -- which is
                   why its linearization keeps a term the other two drop.

Filled joint = actuated. Open joint = passive.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})

BLUE, ORANGE, GREY, RED = "#2b6cb0", "#dd6b20", "#4a5568", "#c53030"
LINK = "#2d3748"


def joint(ax, p, actuated):
    ax.plot(*p, "o", ms=11, zorder=10,
            mfc=(ORANGE if actuated else "white"), mec=LINK, mew=2.0)


fig, axes = plt.subplots(1, 3, figsize=(14, 5.2))

# ------------------------------------------------------------------ Acrobot
ax = axes[0]
l1 = l2 = 1.0
t1, t2 = 0.7, -1.1
p0 = np.array([0.0, 0.0])
p1 = p0 + np.array([l1 * np.sin(t1), -l1 * np.cos(t1)])
p2 = p1 + np.array([l2 * np.sin(t1 + t2), -l2 * np.cos(t1 + t2)])

ax.plot([-0.55, 0.55], [0, 0], color=LINK, lw=3)           # the bar
ax.plot(*zip(p0, p1), color=LINK, lw=4, solid_capstyle="round")
ax.plot(*zip(p1, p2), color=LINK, lw=4, solid_capstyle="round")
joint(ax, p0, actuated=False)
joint(ax, p1, actuated=True)
ax.plot(*p2, "o", ms=8, color=LINK, zorder=10)

# torque arrow at the elbow
arc = np.linspace(-0.6, 2.0, 40)
ax.plot(p1[0] + 0.30 * np.cos(arc), p1[1] + 0.30 * np.sin(arc),
        color=ORANGE, lw=2.2, zorder=9)
ax.annotate("", xy=(p1[0] + 0.30 * np.cos(2.0), p1[1] + 0.30 * np.sin(2.0)),
            xytext=(p1[0] + 0.30 * np.cos(1.75), p1[1] + 0.30 * np.sin(1.75)),
            arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=2.2))
ax.text(p1[0] + 0.46, p1[1] + 0.30, r"$\tau$", color=ORANGE, fontsize=13,
        weight="bold")

ax.text(p0[0] - 0.60, p0[1] + 0.16, "shoulder\n(passive)", fontsize=8.5,
        color=RED, ha="center")
ax.text(0.34, -0.42, r"$\theta_1$", fontsize=11, color=GREY)
ax.text(p1[0] - 0.34, p1[1] - 0.26, r"$\theta_2$", fontsize=11, color=GREY)
ax.set_title("Acrobot", fontsize=12.5, weight="bold")
ax.text(0.5, -0.115,
        r"$q=[\theta_1,\theta_2]^T$,   $u=\tau$   (elbow)"
        "\n"
        r"$\dim q=2$,  $\dim u=1$,   $B=[0,\,1]^T$"
        "\nthe joint touching the world is passive",
        transform=ax.transAxes, ha="center", va="top", fontsize=9.5)
ax.set_xlim(-2.0, 2.0)
ax.set_ylim(-2.40, 0.50)   # 4.0 x 2.9 data box

# ---------------------------------------------------------------- Cart-Pole
ax = axes[1]
# theta = 0 is hanging straight down; the balancing task is theta = pi, so draw
# it near-upright with a small lean -- that is the configuration we care about.
xc, lp = 0.0, 1.30
th = np.pi - 0.42
pivot = np.array([xc, 0.14])
tip = pivot + lp * np.array([np.sin(th), -np.cos(th)])

ax.plot([-2.0, 2.0], [-0.28, -0.28], color=GREY, lw=2, zorder=1)  # rail
ax.add_patch(plt.Rectangle((xc - 0.42, -0.28), 0.84, 0.42,
                           fc="#e2e8f0", ec=LINK, lw=2.2, zorder=4))
for dx in (-0.24, 0.24):                                            # wheels
    ax.plot(xc + dx, -0.32, "o", ms=7, mfc=GREY, mec=GREY, zorder=5)
ax.plot(*zip(pivot, tip), color=LINK, lw=4, solid_capstyle="round", zorder=6)
joint(ax, pivot, actuated=False)
ax.plot(*tip, "o", ms=9, color=LINK, zorder=10)

# vertical reference through the pivot, to make theta readable
ax.plot([xc, xc], [pivot[1], pivot[1] + 1.35], color=GREY, lw=0.9, ls=":",
        zorder=2)
ax.annotate("", xy=(xc + 1.15, -0.07), xytext=(xc + 0.50, -0.07),
            arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=2.8))
ax.text(xc + 1.22, -0.11, r"$f_x$", color=ORANGE, fontsize=13, weight="bold")
ax.text(xc + 0.13, 1.02, r"$\theta$", fontsize=11, color=GREY)
ax.text(xc - 1.15, 0.55, "pole pivot\n(passive)", fontsize=8.5, color=RED,
        ha="center")

ax.set_title("Cart-Pole", fontsize=12.5, weight="bold")
ax.text(0.5, -0.115,
        r"$q=[x,\theta]^T$,   $u=f_x$   (force on cart)"
        "\n"
        r"$\dim q=2$,  $\dim u=1$,   $B=[1,\,0]^T$"
        "\nmirror image: the world-joint is the actuated one",
        transform=ax.transAxes, ha="center", va="top", fontsize=9.5)
ax.set_xlim(-2.0, 2.0)
ax.set_ylim(-0.90, 2.00)   # 4.0 x 2.9 data box

# --------------------------------------------------------- Planar Quadrotor
ax = axes[2]
xq, yq, thq, r = 0.0, 0.0, 0.32, 0.95
d = np.array([np.cos(thq), np.sin(thq)])       # along the body
n = np.array([-np.sin(thq), np.cos(thq)])      # body "up"
cL = np.array([xq, yq]) - r * d
cR = np.array([xq, yq]) + r * d

ax.plot(*zip(cL, cR), color=LINK, lw=5, solid_capstyle="round", zorder=4)
for c, lab in ((cL, r"$u_1$"), (cR, r"$u_2$")):
    ax.annotate("", xy=c + 0.85 * n, xytext=c,
                arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=2.8))
    ax.text(*(c + 1.0 * n + np.array([0.10, 0.05])), lab, color=ORANGE,
            fontsize=13, weight="bold")
    ax.plot(*c, "o", ms=9, mfc=ORANGE, mec=LINK, mew=2.0, zorder=6)

ax.plot(xq, yq, "o", ms=8, color=LINK, zorder=6)
ax.annotate("", xy=(xq, yq - 0.95), xytext=(xq, yq),
            arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=2.4))
ax.text(xq + 0.10, yq - 0.72, r"$mg$", color=BLUE, fontsize=12, weight="bold")
ax.plot([xq - 0.75, xq + 0.75], [yq, yq], color=GREY, lw=0.9, ls=":", zorder=2)
ax.text(xq + 0.52, yq - 0.26, r"$\theta$", fontsize=11, color=GREY, zorder=9)

ax.set_title("Planar Quadrotor", fontsize=12.5, weight="bold")
ax.text(0.5, -0.115,
        r"$q=[x,y,\theta]^T$,   $u=[u_1,u_2]^T$   (thrusts)"
        "\n"
        r"$\dim q=3$,  $\dim u=2$,   $B(q)$ depends on $\theta$"
        "\nthrust is always along body-up → $B$ rotates with the robot",
        transform=ax.transAxes, ha="center", va="top", fontsize=9.5)
ax.set_xlim(-2.0, 2.0)
ax.set_ylim(-1.35, 1.55)   # 4.0 x 2.9 data box

for ax in axes:
    ax.set_aspect("equal")
    ax.axis("off")

fig.suptitle("The three model systems of Chapter 3 — all underactuated, in three different ways",
             fontsize=13, y=0.99)
fig.tight_layout(rect=[0, 0.04, 1, 0.95])
out = __file__.replace("gen_03a_model_systems.py", "03a_model_systems.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
