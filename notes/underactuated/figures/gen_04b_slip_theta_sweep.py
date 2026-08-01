"""Figure: SLIP touchdown-angle sweep at a FIXED incoming apex.

Answers a discussion question: at a fixed incoming apex height, what does the
touchdown angle theta do to the NEXT apex height z[n+1] and next horizontal
speed xdot[n+1]?  Same physics/integrator as gen_04b_slip.py (k~=10.7, E~=1.61),
only now theta_td is the swept variable instead of a hard-coded 30 deg.

Surprise finding (overturns the naive "pogo" intuition):
  - z[n+1] and xdot[n+1] always move OPPOSITELY (energy conservation) -- expected.
  - Over the main branch (theta ~ 22..40 deg) a MORE-FORWARD plant (larger theta)
    gives a HIGHER, SLOWER next apex, not lower/faster. A forward-tilted leg
    CATCHES more horizontal velocity as spring compression -> pumps it into
    height. A near-vertical leg just SKIMS over the foot -> stays fast and low.
  - Past ~40 deg the map FOLDS back down (that fold is the non-monotonicity that
    gives the two fixed points in gen_04b_slip.py). Both ends run into "fall".
"""
import matplotlib.pyplot as plt
import numpy as np

m, g, l0 = 1.0, 1.0, 1.0
k, E = 10.7, 1.61


def stance_rhs(s):
    r, th, rd, thd = s
    rdd = r * thd ** 2 - g * np.cos(th) + (k / m) * (l0 - r)
    thdd = (g * np.sin(th)) / r - 2 * rd * thd / r
    return np.array([rd, thd, rdd, thdd])


def apex_map(z_apex, th_td):
    """Return (z_next, xdot_next) for a given incoming apex height and td angle."""
    val = 2 * (E - g * z_apex)
    if val <= 0:
        return None, None
    xd = np.sqrt(val)                     # horizontal speed (const in flight)
    z_td = l0 * np.cos(th_td)             # touchdown height
    if z_apex < z_td:                     # apex below touchdown -> can't reach
        return None, None
    t = np.sqrt(2 * (z_apex - z_td) / g)  # fall time apex -> touchdown
    zd = -g * t
    r, th = l0, th_td
    rd = -np.sin(th) * xd + np.cos(th) * zd
    thd = (1.0 / r) * (-np.cos(th) * xd - np.sin(th) * zd)
    s = np.array([r, th, rd, thd]); dt = 1e-4
    for _ in range(400000):
        k1 = stance_rhs(s); k2 = stance_rhs(s + .5 * dt * k1)
        k3 = stance_rhs(s + .5 * dt * k2); k4 = stance_rhs(s + dt * k3)
        s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if s[0] >= l0 and s[2] > 0:       # leg back to rest length, extending = takeoff
            break
    else:
        return None, None
    r, th, rd, thd = s
    zd2 = np.cos(th) * rd - r * np.sin(th) * thd
    z2 = r * np.cos(th)
    if zd2 <= 0:                          # not moving up at takeoff -> no apex
        return None, None
    z_next = z2 + zd2 ** 2 / (2 * g)
    xd_next = np.sqrt(max(2 * (E - g * z_next), 0.0))
    return z_next, xd_next


Z_IN = 1.00                               # fixed incoming apex height (m)
thetas = np.linspace(5, 50, 91)
z_next = np.full_like(thetas, np.nan)
xd_next = np.full_like(thetas, np.nan)
for i, th_deg in enumerate(thetas):
    zn, xn = apex_map(Z_IN, np.radians(th_deg))
    if zn is not None:
        z_next[i], xd_next[i] = zn, xn

fig, ax1 = plt.subplots(figsize=(8.2, 5.2))
good = np.isfinite(z_next)
ax1.plot(thetas[good], z_next[good], color="#2b6cb0", lw=2.4, marker="o", ms=3,
         label="next apex height  $z[n{+}1]$")
ax1.axhline(Z_IN, color="#4a5568", ls=":", lw=1.2)
ax1.text(5.3, Z_IN + 0.01, "incoming apex $z[n]=1.0$", fontsize=8, color="#4a5568")
ax1.set_xlabel(r"touchdown angle from vertical  $\theta$  (deg)")
ax1.set_ylabel("next apex height  $z[n{+}1]$  (m)", color="#2b6cb0")
ax1.tick_params(axis="y", labelcolor="#2b6cb0")
ax1.grid(alpha=0.25, lw=0.5)

ax2 = ax1.twinx()
ax2.plot(thetas[good], xd_next[good], color="#dd6b20", lw=2.4, marker="s", ms=3,
         label="next horiz. speed  $\\dot x[n{+}1]$")
ax2.set_ylabel("next horizontal speed  $\\dot x[n{+}1]$  (m/s)", color="#dd6b20")
ax2.tick_params(axis="y", labelcolor="#dd6b20")

# mark 30 deg reference and the fold
ax1.axvline(30, color="#c53030", ls="--", lw=1.0, alpha=0.7)
ax1.text(30.4, 0.42, r"book $\theta=30°$", color="#c53030",
         fontsize=8, rotation=90, va="bottom")
th_fold = thetas[good][np.argmax(z_next[good])]
ax1.annotate("fold: apex peaks\n(≈ all energy → height)",
             xy=(th_fold, np.nanmax(z_next)), xytext=(th_fold - 12, 1.5),
             fontsize=8, color="#2b6cb0",
             arrowprops=dict(arrowstyle="-|>", color="#2b6cb0", lw=1.1))

lines = ax1.get_lines()[:1] + ax2.get_lines()[:1]
ax1.legend(lines, [l.get_label() for l in lines], loc="upper left", fontsize=9)
ax1.set_title("SLIP: sweep touchdown angle at fixed incoming apex\n"
              "forward-tilted leg CATCHES velocity → higher, slower apex "
              "(until it folds)", fontsize=10.5, weight="bold")
fig.tight_layout()
out = __file__.replace("gen_04b_slip_theta_sweep.py", "04b_slip_theta_sweep.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
