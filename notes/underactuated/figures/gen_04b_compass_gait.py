"""Figure: the compass gait -- two legs, a hip, and the Acrobot connection.

The rimless wheel assumed a spoke was always waiting at the collision. Remove all
but two spokes and pin them at a hip: now the SWING leg has its own dynamics and
must actually swing forward in time to catch the next step. That swing is the new
content.

(a) The model: stance leg (planted, pin at foot), swing leg (free), point masses
    m on each leg and m_h at the hip. State q = [theta_st, theta_sw].

(b) The key connection: with a hip torque, the swing dynamics are EXACTLY an
    Acrobot (3c) in disguise -- a two-link pendulum with the actuated joint at the
    'elbow' (hip) and the passive joint at the 'shoulder' (stance foot). Passive
    walking is u=0; add hip torque and every 3c tool (PFL, energy shaping) applies.

The trade for realism: the basin of attraction is a NARROW band around the limit
cycle -- much more sensitive than the rimless wheel (whose extra spokes were free
insurance). This is why passive walkers are famously hard to start.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})
BLUE, ORANGE, GREY, RED, GREEN, LINK = (
    "#2b6cb0", "#dd6b20", "#4a5568", "#c53030", "#2f855a", "#2d3748")

fig, axes = plt.subplots(1, 2, figsize=(13, 5.6))

# ============================================= (a) the model
ax = axes[0]
ax.axis("off")
gamma = 0.16
downhill = np.array([np.cos(-gamma), np.sin(-gamma)])
# ground/ramp
F = np.array([0.0, 0.0])
a = F - downhill * 3.0; b = F + downhill * 2.4
ax.plot([a[0], b[0]], [a[1], b[1]], color=LINK, lw=2.5, zorder=2)
for t in np.linspace(-2.8, 2.2, 26):
    p = F + downhill * t
    ax.plot([p[0], p[0] - 0.13], [p[1], p[1] - 0.17], color=GREY, lw=0.7,
            alpha=0.6)

l = 2.2
th_st = -0.22    # stance leg tilted back (uphill) from vertical, planted at F
hip = F + l * np.array([np.sin(th_st), np.cos(th_st)])
# swing leg hangs from hip and is swung FORWARD (downhill, +x), foot in the air
th_sw = 0.42     # swing-leg angle forward of vertical
swing_foot = hip + l * np.array([np.sin(th_sw), -np.cos(th_sw)])

ax.plot([F[0], hip[0]], [F[1], hip[1]], color=BLUE, lw=4.5,
        solid_capstyle="round", zorder=5)
ax.plot([hip[0], swing_foot[0]], [hip[1], swing_foot[1]], color=ORANGE, lw=4.5,
        solid_capstyle="round", zorder=5)
# masses at leg midpoints
mst = F + (hip - F) * 0.5
msw = hip + (swing_foot - hip) * 0.5
ax.plot(*mst, "o", ms=13, color=BLUE, zorder=7)
ax.plot(*msw, "o", ms=13, color=ORANGE, zorder=7)
ax.plot(*hip, "o", ms=16, color=LINK, zorder=8)
ax.plot(*F, "o", ms=6, color=BLUE, zorder=8)
ax.plot(*swing_foot, "o", ms=6, color=ORANGE, zorder=8)
ax.text(hip[0] - 0.05, hip[1] + 0.16, "$m_h$ (hip)", fontsize=9, color=LINK,
        ha="center")
ax.text(mst[0] - 0.28, mst[1], "$m$", fontsize=9, color=BLUE, ha="center")
ax.text(msw[0] + 0.26, msw[1], "$m$", fontsize=9, color=ORANGE, ha="center")
ax.text(F[0] - 0.75, F[1] + 0.55, "stance leg\n(planted)", fontsize=8.5,
        color=BLUE, ha="center")
ax.text(swing_foot[0] + 0.05, swing_foot[1] - 0.35, "swing leg\n(free, mid-air)",
        fontsize=8.5, color=ORANGE, ha="center")
# vertical ref
ax.plot([hip[0], hip[0]], [hip[1] - 2.6, hip[1] + 0.4], color=GREY, lw=0.8,
        ls="--", zorder=1)
ax.text(-1.95, 2.55, r"state $q=[\theta_{st},\,\theta_{sw}]$", fontsize=10,
        color=LINK)
ax.set_title("(a) compass gait: stance leg + a swing leg that must\n"
             "swing forward in time to catch the step", fontsize=10.8,
             weight="bold")
ax.set_xlim(-2.6, 2.6); ax.set_ylim(-0.7, 3.0)
ax.set_aspect("equal")

# ============================================= (b) the Acrobot connection
ax = axes[1]
ax.axis("off")
ax.set_xlim(0, 10); ax.set_ylim(0, 10)
ax.text(5, 9.4, "the swing dynamics ARE an Acrobot (3c)", ha="center",
        fontsize=12, weight="bold", color=LINK)

# draw the compass gait as a 2-link pendulum hanging from the stance foot
pivot = np.array([3.0, 7.6])
th1 = 0.5
knee = pivot + 2.6 * np.array([np.sin(th1), -np.cos(th1)])
th2 = -0.7
foot = knee + 2.6 * np.array([np.sin(th1 + th2), -np.cos(th1 + th2)])
ax.plot([pivot[0], knee[0]], [pivot[1], knee[1]], color=BLUE, lw=4,
        solid_capstyle="round", zorder=4)
ax.plot([knee[0], foot[0]], [knee[1], foot[1]], color=ORANGE, lw=4,
        solid_capstyle="round", zorder=4)
ax.plot(*pivot, "o", ms=12, mfc="white", mec=RED, mew=2.5, zorder=6)
ax.plot(*knee, "o", ms=12, mfc=ORANGE, mec=LINK, mew=2, zorder=6)
ax.plot(*foot, "o", ms=7, color=LINK, zorder=6)
# hip torque
arc = np.linspace(-0.5, 1.8, 30)
ax.plot(knee[0] + 0.42 * np.cos(arc), knee[1] + 0.42 * np.sin(arc), color=GREEN,
        lw=2.2, zorder=5)
ax.annotate("", xy=(knee[0] + 0.42 * np.cos(1.8), knee[1] + 0.42 * np.sin(1.8)),
            xytext=(knee[0] + 0.42 * np.cos(1.55), knee[1] + 0.42 * np.sin(1.55)),
            arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=2.2))

ax.annotate("stance foot =\nPASSIVE joint\n('shoulder')", xy=pivot,
            xytext=(0.4, 6.3), fontsize=9, color=RED, weight="bold",
            arrowprops=dict(arrowstyle="-|>", color=RED, lw=1.4))
ax.annotate("hip = ACTUATED\njoint ('elbow'),\ntorque $\\tau$", xy=knee,
            xytext=(6.0, 6.9), fontsize=9, color=GREEN, weight="bold",
            arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=1.4))
ax.text(5, 2.3,
        "passive walking = $u=0$;   add hip torque and\n"
        "every 3c tool applies — PFL, energy shaping, swing-up.\n"
        r"$B=[-1,\,1]^T$: one actuator, two DOF → underactuated.",
        ha="center", fontsize=9.3, color=LINK)
ax.text(5, 0.7,
        "cost of realism: the stable limit cycle has a NARROW basin\n"
        "(passive walkers are famously hard to start)",
        ha="center", fontsize=9, color=RED, style="italic")
ax.set_title("(b) it's the swing-up system from Ch 3, walking",
             fontsize=10.8, weight="bold")

fig.suptitle("The compass gait — the rimless wheel with real swing dynamics "
             "(which turn out to be an Acrobot)", fontsize=12.5, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.94])
out = __file__.replace("gen_04b_compass_gait.py", "04b_compass_gait.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
