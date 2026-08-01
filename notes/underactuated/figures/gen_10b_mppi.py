"""Figure: MPPI -- optimize by SAMPLING, not by differentiating.

Torque-limited pendulum swing-up (|u| <= 2, but m g l = 9.81, so a single push
cannot lift it -- it must pump, exactly the 02/03c swing-up problem). MPPI never
touches a gradient: it perturbs a nominal plan K times, rolls all K through the
dynamics, and takes a softmax-weighted average of the perturbations.

(a) ONE TICK.  The sample fan: K=300 rollouts drawn around the nominal plan,
    coloured by their MPPI weight w^k = exp(-(1/lambda)(S^k - S_min)) / sum.
    Nearly all the weight lands on a thin bundle of good samples; the updated
    nominal is their weighted mean. That reweighting IS the optimizer.

(b) CLOSED LOOP.  Running that every tick (receding horizon) swings the pendulum
    up: pump left, pump right, catch. No solver, no Jacobian, no convexity.
"""
import numpy as np
import matplotlib.pyplot as plt

BLUE, ORANGE, GREEN, GREY, RED, INK = (
    "#2b6cb0", "#dd6b20", "#2f855a", "#4a5568", "#c53030", "#1a202c")

rng = np.random.default_rng(0)

# ---- pendulum: theta=0 hanging down, theta=pi up. x = [theta, thetadot] -----
g, l, m, b = 9.81, 1.0, 1.0, 0.10
dt = 0.05
U_MAX = 2.0                      # << m g l = 9.81  -> must pump


def step(x, u):
    """Vectorised one-step dynamics. x: (...,2), u: (...,). Returns (...,2)."""
    th, thd = x[..., 0], x[..., 1]
    thdd = (u - b * thd - m * g * l * np.sin(th)) / (m * l * l)
    # clip the velocity: forward Euler can blow up on wild samples, and an
    # exploded rollout would poison the softmax with inf/nan costs
    return np.stack([th + dt * thd, np.clip(thd + dt * thdd, -20.0, 20.0)], axis=-1)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def cost(x, u):
    return 5.0 * wrap(x[..., 0] - np.pi) ** 2 + 0.20 * x[..., 1] ** 2 + 0.02 * u ** 2


# ---- MPPI ------------------------------------------------------------------
# lambda is the TEMPERATURE. Too small -> all the weight collapses onto the single
# best sample (that degenerate limit is "predictive sampling"); too large -> every
# sample weighted equally and the update vanishes. Costs here are O(100s), so a
# lambda of ~20 gives a genuine spread of weights.
K, N, LAM, SIG = 300, 45, 20.0, 1.6


def mppi(x0, U, K=K, rng=rng):
    """One MPPI update. Returns (new U, sampled state rollouts, weights)."""
    eps = rng.standard_normal((K, N)) * SIG
    # correlated (smoothed) noise -- white noise per step barely moves the plant
    ker = np.ones(7) / 7.0
    eps = np.apply_along_axis(lambda r: np.convolve(r, ker, mode="same"), 1, eps)

    V = np.clip(U[None, :] + eps, -U_MAX, U_MAX)
    x = np.repeat(x0[None, :], K, axis=0)
    S = np.zeros(K)
    traj = np.zeros((K, N + 1, 2))
    traj[:, 0] = x
    for n in range(N):
        S += cost(x, V[:, n])
        x = step(x, V[:, n])
        traj[:, n + 1] = x
    S += 20.0 * (5.0 * wrap(x[:, 0] - np.pi) ** 2 + 0.20 * x[:, 1] ** 2)   # terminal

    S = np.nan_to_num(S, nan=1e12, posinf=1e12)
    w = np.exp(-(S - S.min()) / LAM)
    w /= w.sum()
    du = (w[:, None] * eps).sum(axis=0)          # literally  sum_k w^k eps^k
    return np.clip(U + du, -U_MAX, U_MAX), traj, w, S


fig, axd = plt.subplot_mosaic([["a", "b1"], ["a", "b2"]], figsize=(13.2, 5.6),
                              gridspec_kw=dict(height_ratios=[1.35, 1.0]))
axa, axb, axu = axd["a"], axd["b1"], axd["b2"]

# ---------------- (a) one tick: the sample fan ------------------------------
x0 = np.array([0.35, 0.0])
U = np.zeros(N)
for _ in range(3):                       # a few warm-up iterations at this state
    U, traj, w, S = mppi(x0, U)
U_new, traj, w, S = mppi(x0, U)

tt = np.arange(N + 1) * dt
order = np.argsort(w)                    # draw bad ones first, good ones on top
cmap = plt.get_cmap("viridis")
wn = w / w.max()
for k in order:
    axa.plot(tt, traj[k, :, 0], "-", lw=0.7 + 2.0 * wn[k],
             color=cmap(0.15 + 0.85 * wn[k]), alpha=0.25 + 0.7 * wn[k], zorder=2)

# the updated nominal, rolled out
xr, xs = x0.copy(), [x0.copy()]
for n in range(N):
    xr = step(xr, U_new[n]); xs.append(xr.copy())
xs = np.array(xs)
axa.plot(tt, xs[:, 0], "-", color=RED, lw=3.0, zorder=5,
         label="updated nominal = $\\sum_k w^k \\epsilon^k$ (weighted mean)")
axa.axhline(np.pi, color=GREEN, lw=1.4, ls=":", zorder=1)
axa.text(tt[-1] * 0.02, np.pi + 0.12, "upright  $\\theta=\\pi$", color=GREEN, fontsize=10)

eff = 1.0 / np.sum(w ** 2)
axa.annotate(f"K = {K} samples\ntop weight = {w.max():.2f}\n"
             f"effective samples = {eff:.0f}",
             xy=(0.02, 0.72), xycoords="axes fraction", fontsize=10,
             family="monospace", color=INK,
             bbox=dict(boxstyle="round,pad=0.4", fc="#f7fafc", ec=GREY, lw=0.8))
axa.set_title("(a) One MPPI tick: 300 rollouts, coloured by softmax weight",
              fontsize=12, color=INK)
axa.set_xlabel("time into the horizon  (s)"); axa.set_ylabel("angle  $\\theta$  (rad)")
axa.legend(loc="lower right", fontsize=9, framealpha=0.95)
axa.grid(alpha=0.25)

sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, 1))
cb = fig.colorbar(sm, ax=axa, pad=0.02, fraction=0.045)
cb.set_label("relative weight  $w^k$", fontsize=9)

# ---------------- (b) closed-loop MPPI swing-up ----------------------------
x = np.array([0.0, 0.0])
U = np.zeros(N)
TH, TD, UU = [x[0]], [x[1]], []
rng2 = np.random.default_rng(7)
for k in range(160):
    U, *_ = mppi(x, U, K=300, rng=rng2)
    u0 = U[0]
    UU.append(u0)
    x = step(x, u0)
    TH.append(x[0]); TD.append(x[1])
    U = np.concatenate([U[1:], [0.0]])          # shift the horizon (receding)
TH, UU = np.array(TH), np.array(UU)
t2 = np.arange(len(TH)) * dt

axb.plot(t2, TH, "-", color=BLUE, lw=2.6)
for s in (1, -1):
    axb.axhline(s * np.pi, color=GREEN, lw=1.4, ls=":")
axb.text(0.1, np.pi + 0.35, "upright  $\\theta=\\pm\\pi$  (same configuration)",
         color=GREEN, fontsize=9.5)
axb.annotate("torque limit 2 N$\\cdot$m $\\ll$ $mgl=9.81$ $\\Rightarrow$ it must PUMP,\n"
             "and MPPI finds that without ever computing a derivative",
             xy=(0.26, 0.78), xycoords="axes fraction", fontsize=9.5, color=INK,
             bbox=dict(boxstyle="round,pad=0.35", fc="#f0fff4", ec=GREEN, lw=0.9))
axb.set_title("(b) Closed loop: re-solve every tick, execute $u[0]$",
              fontsize=12, color=INK)
axb.set_ylabel("angle  $\\theta$  (rad)")
axb.set_ylim(-4.6, 6.4)
axb.grid(alpha=0.25)

axu.plot(t2[:-1], UU, "-", color=ORANGE, lw=1.4)
axu.axhline(U_MAX, color=GREY, lw=0.9, ls="--")
axu.axhline(-U_MAX, color=GREY, lw=0.9, ls="--")
axu.set_title("chatter once balanced -- real stacks hand off to LQR near the goal",
              fontsize=9, color=GREY, loc="left")
axu.set_ylabel("torque $u$")
axu.set_xlabel("time  (s)")
axu.set_ylim(-3.4, 3.4)
axu.grid(alpha=0.25)

fig.suptitle("MPPI: zero-order, embarrassingly parallel, indifferent to what $f$ is",
             fontsize=13, color=INK)
fig.tight_layout(rect=(0, 0, 1, 0.94))
fig.savefig("10b_mppi.png", dpi=150)
print("final theta =", round(TH[-1], 3), " (pi =", round(np.pi, 3), ")")
print("max |u| used =", round(np.abs(UU).max(), 3))
print("wrote 10b_mppi.png")
