"""Figure: moment of inertia -- the 'rotational mass' -- for non-point masses.

The one equation:  L = I * omega   (angular momentum = moment of inertia x
angular velocity), the rotational twin of  p = m v.

I answers 'how hard is this to spin about THIS axis', and it depends on how far
the mass sits from the axis:   I = sum_i m_i r_i^2 = integral r^2 dm.

(a) I = m r^2 for a single point mass -- move it out, I grows as r^2.
(b) SAME total mass M, different distribution -> different I: a point mass at L,
    a rod about its end, a rod about its centre. Concentrated-far = big I;
    spread-in = small I.
(c) Parallel-axis theorem: I about any point = I about the centre of mass + M d^2.
    (How you move the reference point -- needed for the collision-about-contact.)
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})
BLUE, ORANGE, GREY, RED, GREEN, LINK = (
    "#2b6cb0", "#dd6b20", "#4a5568", "#c53030", "#2f855a", "#2d3748")

fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.9))


def pivot(ax, p):
    ax.plot(*p, "^", ms=13, color=LINK, zorder=8)
    ax.plot(*p, "o", ms=5, color="white", zorder=9)


# ============================================= (a) I = m r^2
ax = axes[0]
P = np.array([0.0, 0.0])
pivot(ax, P)
for r, yoff in [(0.9, 0), (1.9, 0)]:
    H = P + np.array([r, 1.1])
    ax.plot([P[0], H[0]], [P[1], H[1]], color=GREY, lw=1.6, zorder=3)
    ax.plot(*H, "o", ms=15, color=BLUE, zorder=6)
    ax.annotate("", xy=(H[0], P[1] - 0.15), xytext=(P[0], P[1] - 0.15),
                arrowprops=dict(arrowstyle="<|-|>", color=RED, lw=1.4))
    ax.text((P[0] + H[0]) / 2, P[1] - 0.42, f"$r={r}$", color=RED, fontsize=10,
            ha="center")
# rotation arrow
aa = np.linspace(0.3, 1.5, 30)
ax.plot(0.5 * np.cos(aa) + P[0], 0.5 * np.sin(aa) + P[1] + 0.1, color=ORANGE, lw=2)
ax.annotate("", xy=(0.5 * np.cos(1.5) + P[0], 0.5 * np.sin(1.5) + P[1] + 0.1),
            xytext=(0.5 * np.cos(1.4) + P[0], 0.5 * np.sin(1.4) + P[1] + 0.1),
            arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=2))
ax.text(P[0] - 0.15, P[1] + 0.75, r"$\omega$", color=ORANGE, fontsize=13)
ax.set_title(r"(a) point mass:  $I = m\,r^2$", fontsize=11.5)
ax.text(0.5, 0.02, r"double $r$ $\to$ $4\times$ the inertia",
        transform=ax.transAxes, ha="center", fontsize=9.5, color=BLUE,
        weight="bold")
ax.set_xlim(-1.0, 2.6); ax.set_ylim(-0.9, 2.0)
ax.set_aspect("equal"); ax.axis("off")

# ============================================= (b) same M, different I
ax = axes[1]
L = 1.8
# point mass at end
x0 = -0.2
pivot(ax, (x0, 1.6))
ax.plot([x0, x0], [1.6, 1.6 - L], color=GREY, lw=1.2, ls=":", zorder=2)
ax.plot(x0, 1.6 - L, "o", ms=16, color=BLUE, zorder=6)
ax.text(x0, 1.6 - L - 0.28, r"$I=ML^2$", ha="center", fontsize=9.5, color=BLUE,
        weight="bold")
ax.text(x0, 1.78, "point\nat $L$", ha="center", fontsize=8.5, color=GREY)
# rod about end
x1 = 1.3
pivot(ax, (x1, 1.6))
ax.plot([x1, x1], [1.6, 1.6 - L], color=ORANGE, lw=7, solid_capstyle="round",
        zorder=5, alpha=0.85)
ax.text(x1, 1.6 - L - 0.28, r"$I=\frac{1}{3}ML^2$", ha="center", fontsize=9.5,
        color=ORANGE, weight="bold")
ax.text(x1, 1.78, "rod about\nend", ha="center", fontsize=8.5, color=GREY)
# rod about centre
x2 = 2.8
pivot(ax, (x2, 1.6 - L / 2))
ax.plot([x2, x2], [1.6, 1.6 - L], color=GREEN, lw=7, solid_capstyle="round",
        zorder=5, alpha=0.85)
ax.text(x2, 1.6 - L - 0.28, r"$I=\frac{1}{12}ML^2$", ha="center", fontsize=9.5,
        color=GREEN, weight="bold")
ax.text(x2 + 0.5, 1.6 - L / 2, "rod about\ncentre", ha="left", fontsize=8.5,
        color=GREY, va="center")
ax.set_title("(b) same mass $M$, different distribution", fontsize=11.5)
ax.text(0.5, 0.02, "concentrated-far = big $I$   ·   spread-in = small $I$",
        transform=ax.transAxes, ha="center", fontsize=9.3, color=LINK,
        weight="bold")
ax.set_xlim(-1.0, 4.2); ax.set_ylim(-0.9, 2.1)
ax.set_aspect("equal"); ax.axis("off")

# ============================================= (c) parallel axis
ax = axes[2]
cm = np.array([1.4, 1.0])
Ppt = np.array([0.0, 1.0])
# body: a disk at cm
disk = plt.Circle(cm, 0.55, fc="#dbeafe", ec=BLUE, lw=2, zorder=3)
ax.add_patch(disk)
ax.plot(*cm, "o", ms=7, color=BLUE, zorder=5)
ax.text(cm[0], cm[1] + 0.72, "centre of mass", ha="center", fontsize=8.5,
        color=BLUE)
ax.text(cm[0], cm[1] - 0.05, r"$I_{cm}$", ha="center", fontsize=11, color=BLUE)
pivot(ax, Ppt)
ax.text(Ppt[0], Ppt[1] + 0.28, "pivot $P$", ha="center", fontsize=8.5, color=LINK)
# distance d
ax.annotate("", xy=cm, xytext=Ppt,
            arrowprops=dict(arrowstyle="<|-|>", color=RED, lw=1.5))
ax.text((cm[0] + Ppt[0]) / 2, cm[1] + 0.14, "$d$", color=RED, fontsize=12,
        ha="center", weight="bold")
ax.set_title("(c) parallel-axis theorem", fontsize=11.5)
ax.text(0.5, 0.06, r"$I_P = I_{cm} + M d^2$", transform=ax.transAxes,
        ha="center", fontsize=13, color=RED, weight="bold")
ax.text(0.5, -0.04, "move the reference point out by $d$", transform=ax.transAxes,
        ha="center", fontsize=9, color=GREY)
ax.set_xlim(-0.8, 2.5); ax.set_ylim(-0.1, 2.0)
ax.set_aspect("equal"); ax.axis("off")

fig.suptitle(r"Moment of inertia $I$ — the 'rotational mass' in $L = I\,\omega$   "
             r"(point mass is just the special case $I=m r^2$)",
             fontsize=12.5, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.93])
out = __file__.replace("gen_04a_moment_of_inertia.py", "04a_moment_of_inertia.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
