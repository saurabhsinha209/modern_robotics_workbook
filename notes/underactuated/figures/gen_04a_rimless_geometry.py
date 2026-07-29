"""Figure: the rimless wheel's angle conventions, drawn.

(a) Definitions on the ramp:
      gamma = ramp slope (from horizontal)
      alpha = half-angle between adjacent spokes (2*alpha apart)
      theta = stance-leg angle from the TRUE vertical (gravity), the state
      key configs: post-support theta = gamma - alpha  (leg behind vertical)
                   top          theta = 0              (mass above foot)
                   touchdown    theta = gamma + alpha  (leg ahead, next spoke hits)
      one step rotates the wheel by exactly 2*alpha.

(b) The collision that costs cos(2*alpha):
      at touchdown the mass is rotating about the OLD foot, so its velocity v-
      is perpendicular to the old leg. Support transfers to the NEW leg (2*alpha
      away); angular momentum about the new contact is conserved, which keeps
      only the component of v- perpendicular to the NEW leg:  v+ = v- cos(2 alpha).
      The velocity along the new leg is killed by the impact.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})
BLUE, ORANGE, GREY, RED, GREEN, LINK = (
    "#2b6cb0", "#dd6b20", "#4a5568", "#c53030", "#2f855a", "#2d3748")

alpha = np.pi / 8          # 22.5 deg  -> 8 spokes
gamma = 0.32               # exaggerated ramp for visibility
l = 1.0
N = 8


def rot(v, a):
    c, s = np.cos(a), np.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def leg_dir(theta):
    """foot->hub unit vector for stance angle theta from vertical (up), + = tilt right."""
    return np.array([np.sin(theta), np.cos(theta)])


fig, axes = plt.subplots(1, 2, figsize=(13.5, 6.0))

# =========================================================== (a) definitions
ax = axes[0]
# ramp descending to the right at angle gamma below horizontal
downhill = np.array([np.cos(-gamma), np.sin(-gamma)])
ramp_n = np.array([np.sin(gamma), np.cos(gamma)])     # upward normal
F = np.array([0.0, 0.0])                              # stance foot at origin
ramp_a = F - downhill * 2.4
ramp_b = F + downhill * 2.4
ax.plot([ramp_a[0], ramp_b[0]], [ramp_a[1], ramp_b[1]], color=LINK, lw=3, zorder=3)
# ground hatching
for t in np.linspace(-2.2, 2.2, 24):
    p = F + downhill * t
    ax.plot([p[0], p[0] - 0.12], [p[1], p[1] - 0.16], color=GREY, lw=0.7,
            alpha=0.6, zorder=2)
# horizontal reference through foot + gamma arc
ax.plot([-2.4, 2.4], [0, 0], color=GREY, lw=0.8, ls=":", zorder=2)
arc = np.linspace(-gamma, 0, 30)
ax.plot(0.9 * np.cos(arc), 0.9 * np.sin(arc), color=ORANGE, lw=1.8, zorder=4)
ax.text(1.05, -0.16, r"$\gamma$", color=ORANGE, fontsize=14, weight="bold")
ax.text(1.55, -0.62, "ramp slope", color=ORANGE, fontsize=8.5)

# wheel: stance leg at theta = gamma (mid-ish, leg near ramp normal) for a clean look
theta = gamma
H = F + l * leg_dir(theta)          # hub
# all spokes
stance_dir_from_hub = -leg_dir(theta)     # hub->foot
for k in range(N):
    d = rot(stance_dir_from_hub, k * 2 * alpha)
    tip = H + l * d
    is_stance = (k == 0)
    ax.plot([H[0], tip[0]], [H[1], tip[1]],
            color=(BLUE if is_stance else "#94a3b8"),
            lw=(3.4 if is_stance else 1.4), zorder=5,
            solid_capstyle="round")
    if not is_stance:
        ax.plot(*tip, "o", ms=3, color="#94a3b8", zorder=5)
# hub mass
ax.plot(*H, "o", ms=13, color=LINK, zorder=8)
ax.text(H[0] + 0.12, H[1] + 0.08, "mass $m$", fontsize=9, color=LINK)
ax.plot(*F, "o", ms=6, color=BLUE, zorder=8)

# vertical reference through foot + theta arc
ax.plot([F[0], F[0]], [F[1], F[1] + 1.4], color=GREY, lw=0.9, ls="--", zorder=4)
ax.text(F[0] - 0.02, 1.45, "vertical", color=GREY, fontsize=8, ha="center")
arc = np.linspace(0, theta, 30)
ax.plot(0.55 * np.sin(arc), 0.55 * np.cos(arc), color=RED, lw=1.8, zorder=6)
ax.text(0.16, 0.66, r"$\theta$", color=RED, fontsize=14, weight="bold")

# alpha arc between stance spoke and the next, at the hub
d0 = stance_dir_from_hub
d1 = rot(stance_dir_from_hub, 2 * alpha)
aa = np.linspace(0, 2 * alpha, 30)
arcpts = np.array([rot(d0, a) for a in aa]) * 0.5 + H
ax.plot(arcpts[:, 0], arcpts[:, 1], color=GREEN, lw=1.8, zorder=7)
mid = rot(d0, alpha)
ax.text(*(H + mid * 0.66), r"$2\alpha$", color=GREEN, fontsize=13, weight="bold",
        ha="center", va="center")

# gravity
ax.annotate("", xy=(H[0] - 1.15, H[1] - 0.55), xytext=(H[0] - 1.15, H[1] + 0.05),
            arrowprops=dict(arrowstyle="-|>", color=GREY, lw=2))
ax.text(H[0] - 1.35, H[1] - 0.35, "$g$", color=GREY, fontsize=12)

ax.set_title("(a) angle definitions", fontsize=12, weight="bold")
ax.set_xlim(-2.5, 2.2)
ax.set_ylim(-1.0, 1.9)
ax.set_aspect("equal")
ax.axis("off")

# key-config caption
ax.text(-2.45, -0.75,
        r"one step turns the wheel by $2\alpha$;   theta runs:"
        "\n"
        r"$\theta=\gamma-\alpha$ (post-support) $\to$ $\theta=0$ (top) $\to$ "
        r"$\theta=\gamma+\alpha$ (touchdown)",
        fontsize=8.8, va="top", color=LINK)

# =========================================================== (b) the collision
ax = axes[1]
H2 = np.array([0.0, 0.0])                        # hub at origin for clarity
# at touchdown: old stance leg at theta = gamma + alpha (ahead)
th_td = gamma + alpha
old_leg = -leg_dir(th_td)                        # hub->old foot
new_leg = rot(old_leg, 2 * alpha)                # hub->new foot, 2a clockwise-> forward
old_foot = H2 + l * old_leg
new_foot = H2 + l * new_leg

ax.plot([H2[0], old_foot[0]], [H2[1], old_foot[1]], color=GREY, lw=3,
        solid_capstyle="round", zorder=4)
ax.plot([H2[0], new_foot[0]], [H2[1], new_foot[1]], color=BLUE, lw=3.4,
        solid_capstyle="round", zorder=5)
ax.plot(*old_foot, "o", ms=6, color=GREY, zorder=6)
ax.plot(*new_foot, "o", ms=7, color=BLUE, zorder=6)
ax.plot(*H2, "o", ms=13, color=LINK, zorder=8)
ax.text(*(old_foot + np.array([-0.05, -0.18])), "old foot", color=GREY,
        fontsize=9, ha="center")
ax.text(*(new_foot + np.array([0.0, -0.20])), "new foot", color=BLUE,
        fontsize=9, ha="center")

# 2 alpha between legs at hub
aa = np.linspace(0, 2 * alpha, 30)
arcpts = np.array([rot(old_leg, a) for a in aa]) * 0.42 + H2
ax.plot(arcpts[:, 0], arcpts[:, 1], color=GREEN, lw=1.8, zorder=7)
ax.text(*(H2 + rot(old_leg, alpha) * 0.60), r"$2\alpha$", color=GREEN,
        fontsize=13, weight="bold", ha="center", va="center")

# velocity before impact: perpendicular to OLD leg, magnitude v
vmag = 1.05
v_before = rot(old_leg, -np.pi / 2)              # perp to old leg (motion sense)
v_before = v_before / np.linalg.norm(v_before)
ax.annotate("", xy=H2 + v_before * vmag, xytext=H2,
            arrowprops=dict(arrowstyle="-|>", color=RED, lw=2.6))
ax.text(*(H2 + v_before * vmag + np.array([0.05, 0.08])),
        r"$v^- = l\,\dot\theta^-$", color=RED, fontsize=11, weight="bold")

# velocity after: perpendicular to NEW leg, magnitude v cos(2a)
v_after = rot(new_leg, -np.pi / 2)
v_after = v_after / np.linalg.norm(v_after)
ax.annotate("", xy=H2 + v_after * vmag * np.cos(2 * alpha), xytext=H2,
            arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=2.6))
ax.text(*(H2 + v_after * vmag * np.cos(2 * alpha) + np.array([0.02, -0.22])),
        r"$v^+ = v^-\cos 2\alpha$", color=GREEN, fontsize=11, weight="bold")

# projection guide (drop from tip of v- onto v+ direction)
tip = H2 + v_before * vmag
proj_len = np.dot(v_before * vmag, v_after)
proj = H2 + v_after * proj_len
ax.plot([tip[0], proj[0]], [tip[1], proj[1]], color=GREY, lw=1.0, ls=":", zorder=4)

ax.set_title("(b) the collision: $v^+ = v^-\\cos 2\\alpha$  (energy lost)",
             fontsize=12, weight="bold")
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.35)
ax.set_aspect("equal")
ax.axis("off")
ax.text(0, -1.42,
        "momentum about the new contact is conserved → only the velocity\n"
        r"component $\perp$ the new leg survives; the along-leg part is killed.",
        fontsize=8.8, ha="center", va="bottom", color=LINK)

fig.suptitle("The rimless wheel — angle conventions and the collision geometry",
             fontsize=13, y=0.99)
fig.tight_layout(rect=[0, 0, 1, 0.95])
out = __file__.replace("gen_04a_rimless_geometry.py", "04a_rimless_geometry.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
