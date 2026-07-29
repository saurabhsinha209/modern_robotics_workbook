"""Figure: the rimless wheel -- the simplest walker, and a HYBRID system.

A wheel with rigid spokes and a point mass at the hub, rolling down a ramp of
slope gamma; each spoke pair makes half-angle alpha. Two ingredients:

  STANCE (continuous):  the mass vaults over the planted foot exactly like the
     UNDAMPED INVERTED PENDULUM,   thetaddot = (g/l) sin(theta).
     -> you already know this phase portrait (note 02), centred on the unstable
        upright fixed point.

  COLLISION (discrete):  when the next spoke hits the ground, angular momentum
     about the new contact is conserved and velocity jumps DOWN by a factor:
        thetadot+ = cos(2 alpha) * thetadot-.
     A 'reset map' -- energy is lost instantaneously. This is the contact event
     that makes locomotion a HYBRID system (flow + jumps).

Gravity feeds energy in on the downslope; each collision takes energy out. When
they balance, the wheel settles into a STABLE ROLLING LIMIT CYCLE -- from a huge
range of initial speeds.

(a) phase portrait: pendulum arcs punctuated by downward velocity jumps at the
    collision boundary theta = gamma + alpha; the loop is the limit cycle.
(b) the step-to-step return map thetadot+[n] -> thetadot+[n+1] with the staircase
    walking into the rolling fixed point.  |slope| < 1 -> stable.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})
BLUE, ORANGE, GREY, RED, GREEN = "#2b6cb0", "#dd6b20", "#4a5568", "#c53030", "#2f855a"

g, l = 9.8, 1.0
alpha = np.pi / 8      # half-angle between spokes
gamma = 0.15           # ramp slope

# stance: theta measured so that theta=0 is the mass directly above the foot
# (top). thetaddot = (g/l) sin(theta). Foot strike when theta = gamma + alpha,
# leaving the new stance angle at theta = gamma - alpha.
TH_TD = gamma + alpha       # touchdown angle
TH_0 = gamma - alpha        # post-support angle


def stance_step(thd0):
    """Integrate one stance phase from theta=TH_0, thetadot=thd0 until the next
    foot strike at theta=TH_TD; return (trajectory, thetadot just before strike).
    Requires thd0 > omega1 so the mass vaults over the top (theta=0)."""
    th, thd = TH_0, thd0
    dt = 0.0005
    traj = [(th, thd)]
    for _ in range(20000):
        def rhs(a, w):
            return w, (g / l) * np.sin(a)      # thetaddot = (g/l) sin(theta)
        k1 = rhs(th, thd)
        k2 = rhs(th + 0.5 * dt * k1[0], thd + 0.5 * dt * k1[1])
        k3 = rhs(th + 0.5 * dt * k2[0], thd + 0.5 * dt * k2[1])
        k4 = rhs(th + dt * k3[0], thd + dt * k3[1])
        th = th + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        thd = thd + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        traj.append((th, thd))
        if th >= TH_TD:
            break
    return np.array(traj), thd


def collision(thd_minus):
    return np.cos(2 * alpha) * thd_minus


# analytic return map (book eq): thd+[n+1] = cos(2a) sqrt(thd^2 + 4 g/l sin a sin g)
def return_map(thd):
    return np.cos(2 * alpha) * np.sqrt(thd ** 2 + 4 * g / l * np.sin(alpha) * np.sin(gamma))


omega_roll = 1.0 / np.tan(2 * alpha) * np.sqrt(4 * g / l * np.sin(alpha) * np.sin(gamma))

fig, axes = plt.subplots(1, 2, figsize=(13, 5.4))

# ---------------- (a) hybrid phase portrait ----------------
ax = axes[0]
thd = 2.6          # start FAST (above threshold); watch it converge DOWN
for step in range(7):
    traj, thd_minus = stance_step(thd)
    ax.plot(traj[:, 0], traj[:, 1], color=BLUE, lw=1.3, alpha=0.85, zorder=3)
    thd_plus = collision(thd_minus)
    # the collision: vertical drop in thetadot at theta = TH_TD
    ax.plot([TH_TD, TH_TD], [thd_minus, thd_plus], color=RED, lw=1.8, zorder=4)
    # instantaneous support transfer: theta resets from TH_TD back to TH_0
    ax.plot([TH_TD, TH_0], [thd_plus, thd_plus], color=GREY, lw=0.8, ls=":",
            alpha=0.6, zorder=2)
    thd = thd_plus

# the limit cycle (many steps in)
for _ in range(60):
    thd = return_map(thd)
traj, thd_minus = stance_step(thd)
ax.plot(traj[:, 0], traj[:, 1], color=GREEN, lw=2.8, zorder=6,
        label="rolling limit cycle")
ax.plot([TH_TD, TH_TD], [thd_minus, collision(thd_minus)], color=GREEN, lw=2.8,
        zorder=6)
ax.plot([TH_TD, TH_0], [collision(thd_minus), collision(thd_minus)], color=GREEN,
        lw=2.8, ls=":", zorder=6)

ax.axvline(0, color=GREY, lw=0.8, alpha=0.6)
ax.text(0.02, 0.35, "top\n$\\theta=0$", fontsize=8, color=GREY)
ax.axvline(TH_TD, color=GREY, lw=1.0, ls="--", alpha=0.7)
ax.annotate("collision:\n$\\dot\\theta^+ = \\cos(2\\alpha)\\,\\dot\\theta^-$\n"
            "(energy lost)", xy=(TH_TD, (thd_minus + collision(thd_minus)) / 2),
            xytext=(TH_TD + 0.06, 1.9), fontsize=8.8, color=RED, weight="bold",
            arrowprops=dict(arrowstyle="-|>", color=RED, lw=1.5))
ax.annotate("support transfer\nresets $\\theta$ to $\\gamma-\\alpha$",
            xy=((TH_TD + TH_0) / 2, collision(thd_minus)),
            xytext=(TH_0 - 0.15, 3.05), fontsize=8.2, color=GREY,
            arrowprops=dict(arrowstyle="-|>", color=GREY, lw=1.2))
ax.text(TH_0 - 0.02, 3.5, "each stance arc = inverted pendulum "
        r"($\ddot\theta=\frac{g}{l}\sin\theta$)", fontsize=8.3, color=BLUE)
ax.set_xlabel(r"stance leg angle  $\theta$")
ax.set_ylabel(r"$\dot\theta$")
ax.set_ylim(0, 3.9)
ax.set_title("(a) hybrid dynamics: pendulum arcs + collision jumps\n"
             "→ a stable rolling limit cycle", fontsize=10.5)
ax.legend(loc="lower right", fontsize=9, framealpha=0.9)
ax.grid(alpha=0.25, lw=0.5)

# ---------------- (b) return map + staircase ----------------
ax = axes[1]
w = np.linspace(0.0, 3.6, 300)
ax.plot(w, return_map(w), color=BLUE, lw=2.4, zorder=4, label=r"return map $P$")
ax.plot([0, 3.6], [0, 3.6], color=RED, lw=1.6, ls="--", zorder=3, label="slope-1")
ax.plot(omega_roll, omega_roll, "o", color=GREEN, ms=11, zorder=6, mec="white",
        mew=1.5)
ax.annotate(r"rolling fixed point $\omega^*_{roll}$" "\n(gravity in = collision out)",
            xy=(omega_roll, omega_roll), xytext=(1.35, 0.85), fontsize=9,
            color=GREEN, weight="bold",
            arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=1.5))
# staircase from two ICs
for x0, col in [(0.5, GREY), (3.3, "#a0aec0")]:
    x = x0
    for _ in range(20):
        y = return_map(x)
        ax.plot([x, x], [x, y], color=col, lw=1.0, zorder=5)
        ax.plot([x, y], [y, y], color=col, lw=1.0, zorder=5)
        x = y
ax.text(0.35, 3.1, "converges to the rolling cycle\nfrom slow AND fast starts",
        fontsize=8.8, color=GREY)
ax.set_xlim(0, 3.6); ax.set_ylim(0, 3.6); ax.set_aspect("equal")
ax.set_xlabel(r"$\dot\theta^+[n]$  (speed after step $n$)")
ax.set_ylabel(r"$\dot\theta^+[n+1]$")
ax.set_title("(b) return map: the gait is a stable fixed point of $P$",
             fontsize=10.5)
ax.legend(loc="lower right", fontsize=9, framealpha=0.9)
ax.grid(alpha=0.25, lw=0.5)

fig.suptitle("The rimless wheel — simplest walker: inverted-pendulum stance + collision reset "
             r"($\alpha=\pi/8$, $\gamma=0.15$)", fontsize=12.5, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.93])
out = __file__.replace("gen_04a_rimless_wheel.py", "04a_rimless_wheel.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out, " omega_roll =", round(omega_roll, 3),
      " P(omega_roll) =", round(return_map(omega_roll), 3))
