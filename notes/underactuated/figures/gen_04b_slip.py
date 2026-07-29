"""Figure: the SLIP runner -- energy-CONSERVING, and its apex-to-apex map.

Spring-Loaded Inverted Pendulum: a point mass on a massless SPRINGY leg. Two
phases, stitched into a hybrid system:
  FLIGHT  (ballistic parabola): xddot=0, zddot=-g; leg angle set for free (massless)
  STANCE  (spring, foot pinned): compresses then rebounds, NO energy lost
     (unlike the rimless wheel, whose collisions dissipated -- that dissipation
      was what gave it stability).

Because energy is conserved, at the hop APEX (zdot=0) height z and speed xdot are
linked (E = 1/2 xdot^2 + g z), so the Poincare section collapses to ONE variable:
apex height. The apex-to-apex map z[n] -> z[n+1] is 1-D and staircase-able.

(a) the model + the two phases.
(b) apex-to-apex return map (real simulation, k~=10.7, E~=1.61, td angle 30deg):
    a STABLE fixed point (~0.877 m) and an UNSTABLE one (~0.916 m). The map is
    undefined below ~0.87 (apex lower than touchdown height). Note: energy
    conservation means this can only certify PARTIAL (i.s.L.) stability -- it
    says nothing about disturbances that change total energy.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})
BLUE, ORANGE, GREY, RED, GREEN, LINK = (
    "#2b6cb0", "#dd6b20", "#4a5568", "#c53030", "#2f855a", "#2d3748")

m, g, l0 = 1.0, 1.0, 1.0
k, E, th_td = 10.7, 1.61, np.radians(30)


def stance_rhs(s):
    r, th, rd, thd = s
    rdd = r * thd ** 2 - g * np.cos(th) + (k / m) * (l0 - r)
    thdd = (g * np.sin(th)) / r - 2 * rd * thd / r
    return np.array([rd, thd, rdd, thdd])


def apex_to_apex(z_apex):
    val = 2 * (E - g * z_apex)
    if val <= 0:
        return None
    xd = np.sqrt(val)
    z_td = l0 * np.cos(th_td)
    if z_apex < z_td:
        return None
    t = np.sqrt(2 * (z_apex - z_td) / g)
    zd = -g * t
    r, th = l0, th_td
    rd = -np.sin(th) * xd + np.cos(th) * zd
    thd = (1.0 / r) * (-np.cos(th) * xd - np.sin(th) * zd)
    s = np.array([r, th, rd, thd]); dt = 1e-4
    for _ in range(200000):
        k1 = stance_rhs(s); k2 = stance_rhs(s + .5 * dt * k1)
        k3 = stance_rhs(s + .5 * dt * k2); k4 = stance_rhs(s + dt * k3)
        s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if s[0] >= l0 and s[2] > 0:
            break
    else:
        return None
    r, th, rd, thd = s
    zd2 = np.cos(th) * rd - r * np.sin(th) * thd
    z2 = r * np.cos(th)
    return z2 + zd2 ** 2 / (2 * g)


fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.4))

# ============================================= (a) model + phases
ax = axes[0]
ax.axis("off")
ax.set_xlim(0, 10); ax.set_ylim(0, 10)
# ground
ax.plot([0.3, 9.7], [1.2, 1.2], color=LINK, lw=2.5, zorder=2)
for x in np.linspace(0.5, 9.5, 30):
    ax.plot([x, x - 0.18], [1.2, 1.0], color=GREY, lw=0.7, alpha=0.6)

# flight parabola (apex in the middle)
xt = np.linspace(1.0, 5.4, 100)
zt = 8.5 - 0.55 * (xt - 3.2) ** 2
ax.plot(xt, zt, color=BLUE, lw=1.6, ls="--", zorder=3)
ax.text(3.2, 8.9, "apex  ($\\dot z=0$)", ha="center", fontsize=9, color=BLUE,
        weight="bold")
ax.annotate("", xy=(1.4, zt[3] - 0.9), xytext=(1.4, zt[3]),
            arrowprops=dict(arrowstyle="-|>", color=GREY, lw=1.6))
ax.text(0.55, zt[3] - 0.6, "$g$", fontsize=12, color=GREY)
ax.text(2.0, 6.6, "FLIGHT\nballistic\n(leg set for free)", fontsize=8.8,
        color=BLUE, ha="center")

# mass in flight
mf = np.array([4.7, zt[np.argmin(np.abs(xt - 4.7))]])
foot_f = mf + l0 * 1.7 * np.array([np.sin(th_td), -np.cos(th_td)])
ax.plot([mf[0], foot_f[0]], [mf[1], foot_f[1]], color=GREY, lw=2, zorder=4)
ax.plot(*mf, "o", ms=15, color=BLUE, zorder=6)

# stance: compressed spring, mass lower, foot on ground
stance_foot = np.array([7.4, 1.2])
mass_st = stance_foot + l0 * 1.35 * np.array([-np.sin(th_td * 0.4), np.cos(th_td * 0.4)])
# draw a zigzag spring
def spring(p0, p1, n=7, w=0.16):
    p0, p1 = np.array(p0), np.array(p1)
    d = p1 - p0; L = np.linalg.norm(d); u = d / L
    perp = np.array([-u[1], u[0]])
    pts = [p0]
    for i in range(1, n):
        s = i / n
        off = w * (1 if i % 2 else -1)
        pts.append(p0 + d * s + perp * off)
    pts.append(p1)
    return np.array(pts)
sp = spring(stance_foot, mass_st)
ax.plot(sp[:, 0], sp[:, 1], color=ORANGE, lw=2.2, zorder=4)
ax.plot(*mass_st, "o", ms=15, color=BLUE, zorder=6)
ax.plot(*stance_foot, "o", ms=6, color=LINK, zorder=6)
ax.text(8.7, 4.0, "STANCE\nspring: compress\n+ rebound\n"
        "$\\bf{no\\ energy\\ lost}$", fontsize=8.8, color=ORANGE, ha="center")

# leg-angle label
ax.annotate(r"leg $l$, angle $\theta$, stiffness $k$", xy=(mf[0], mf[1]),
            xytext=(4.4, 9.2), fontsize=8.8, color=LINK,
            arrowprops=dict(arrowstyle="-", color=GREY, lw=0.8))
ax.set_title("(a) SLIP: point mass on a massless springy leg",
             fontsize=11.5, weight="bold")

# ============================================= (b) apex-to-apex map
ax = axes[1]
zs = np.linspace(0.869, 0.99, 120)
zn = np.array([apex_to_apex(z) for z in zs], dtype=float)
good = np.isfinite(zn)
ax.plot(zs[good], zn[good], color=BLUE, lw=2.4, zorder=4, label="apex-to-apex $P$")
lo, hi = 0.85, 1.02
ax.plot([lo, hi], [lo, hi], color=RED, lw=1.5, ls="--", zorder=3, label="slope-1")

# find fixed points (residual sign changes)
res = zn[good] - zs[good]
zz = zs[good]
fps = []
for i in range(len(res) - 1):
    if res[i] == 0 or res[i] * res[i + 1] < 0:
        zf = zz[i] - res[i] * (zz[i + 1] - zz[i]) / (res[i + 1] - res[i])
        slope_neg = res[i + 1] < res[i]     # residual decreasing -> stable
        fps.append((zf, slope_neg))
for zf, stable in fps:
    ax.plot(zf, zf, "o", ms=11, mec="white", mew=1.5, zorder=6,
            color=(GREEN if stable else RED))
    if stable:
        ax.annotate(f"STABLE\n$z^*\\approx{zf:.3f}$", xy=(zf, zf),
                    xytext=(zf + 0.012, zf + 0.006), fontsize=9, color=GREEN,
                    weight="bold", ha="left",
                    arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=1.3))
    else:
        ax.annotate(f"unstable\n$z^*\\approx{zf:.3f}$", xy=(zf, zf),
                    xytext=(zf + 0.004, zf - 0.02), fontsize=9, color=RED,
                    weight="bold", ha="left")
# staircase converging to the stable fp from 0.90
x = 0.905
for _ in range(14):
    y = apex_to_apex(x)
    if y is None or not lo < y < hi:
        break
    ax.plot([x, x], [x, y], color=GREY, lw=0.9, zorder=5)
    ax.plot([x, y], [y, y], color=GREY, lw=0.9, zorder=5)
    x = y
ax.text(0.862, 1.0, "map undefined below\nthe stable point\n(apex < touchdown height)",
        fontsize=8.2, color=GREY, va="top")
ax.set_xlim(lo, hi); ax.set_ylim(lo, hi); ax.set_aspect("equal")
ax.set_xlabel("apex height  $z[n]$  (m)")
ax.set_ylabel("$z[n+1]$")
ax.set_title("(b) apex-to-apex map: stable + unstable hop heights",
             fontsize=11.5, weight="bold")
ax.legend(loc="lower right", fontsize=8.5, framealpha=0.9)
ax.grid(alpha=0.25, lw=0.5)

fig.suptitle("The SLIP runner — energy-conserving, so stability is only PARTIAL "
             "(i.s.L.), certified by a 1-D apex-to-apex map",
             fontsize=12.5, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.94])
out = __file__.replace("gen_04b_slip.py", "04b_slip.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out, " fixed points:", [(round(z, 3), s) for z, s in fps])
