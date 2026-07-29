"""Figure: graphical analysis of a first-order system (the overdamped pendulum).

When damping dominates, ml^2 thetaddot is negligible and the pendulum becomes
effectively FIRST order:

    b xdot = u0 - m g l sin x        (m = l = g = b = 1, u0 = 0)
    xdot   = -sin x

The whole long-term behaviour can be read off one plot of xdot vs x:
  - zero crossings           -> fixed points
  - sign of xdot             -> direction of flow (right if +, left if -)
  - slope at the crossing    -> stable (negative slope) or unstable (positive)
  - shaded region            -> basin of attraction
  - unstable points          -> the separatrix between basins

A 1-D flow can only monotonically approach a fixed point or run off to infinity.
Oscillation is impossible. That is why the second-order pendulum (next figure)
needs a 2-D phase portrait instead.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})

BLUE, ORANGE, GREY = "#2b6cb0", "#dd6b20", "#4a5568"

x = np.linspace(-2 * np.pi, 2 * np.pi, 800)
xdot = -np.sin(x)

fig, ax = plt.subplots(figsize=(11, 4.6))

# basin of attraction of x* = 0 is the open interval (-pi, pi)
ax.axvspan(-np.pi, np.pi, color=BLUE, alpha=0.10, zorder=0)
ax.text(0, 1.32, "basin of attraction of $x^*=0$", ha="center",
        color=BLUE, fontsize=9.5, weight="bold")

ax.plot(x, xdot, color=BLUE, lw=2.4, zorder=3)
ax.axhline(0, color="#4a5568", lw=1.0, zorder=2)

# fixed points: xdot = 0 at multiples of pi.
# slope of -sin x is -cos x  ->  negative (stable) at even multiples of pi.
for k in range(-2, 3):
    xs = k * np.pi
    stable = np.cos(xs) > 0  # slope -cos(xs) < 0
    ax.plot(xs, 0, "o", ms=11, zorder=6,
            mfc=(GREY if stable else "white"), mec=GREY, mew=2.0)

# flow arrows along the x-axis: right where xdot > 0, left where xdot < 0
for a, b in [(-2 * np.pi, -np.pi), (-np.pi, 0), (0, np.pi), (np.pi, 2 * np.pi)]:
    mid = 0.5 * (a + b)
    direction = np.sign(-np.sin(mid))
    ax.annotate("", xy=(mid + 0.5 * direction, 0), xytext=(mid - 0.5 * direction, 0),
                arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=2.4), zorder=5)

# separatrix markers
for xs in (-np.pi, np.pi):
    ax.axvline(xs, color="#c53030", lw=1.0, ls="--", alpha=0.7, zorder=1)
ax.text(np.pi + 0.12, -1.28, "separatrix", color="#c53030", fontsize=9)

ax.annotate("stable\n(filled)", xy=(0, 0), xytext=(0.35, -0.62),
            color=GREY, fontsize=9,
            arrowprops=dict(arrowstyle="->", color=GREY, lw=1.2))
ax.annotate("unstable\n(open)", xy=(np.pi, 0), xytext=(np.pi + 0.9, 0.55),
            color=GREY, fontsize=9,
            arrowprops=dict(arrowstyle="->", color=GREY, lw=1.2))

ax.set_xlim(-2 * np.pi, 2 * np.pi)
ax.set_ylim(-1.5, 1.5)
ax.set_xticks([k * np.pi for k in range(-2, 3)])
ax.set_xticklabels([r"$-2\pi$", r"$-\pi$", "0", r"$\pi$", r"$2\pi$"])
ax.set_xlabel(r"$x$   (pendulum angle, 0 = hanging down)")
ax.set_ylabel(r"$\dot{x}$")
ax.set_title(
    r"Graphical analysis of the overdamped pendulum:  $\dot{x} = -\sin x$"
    "\nthe entire long-term behaviour, read off one curve",
    fontsize=11.5,
)
ax.grid(alpha=0.25, lw=0.5)

fig.tight_layout()
out = __file__.replace("gen_02_first_order_flow.py", "02_first_order_flow.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
