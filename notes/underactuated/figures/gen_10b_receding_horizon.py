"""Figure: MPC = trajectory optimization turned into a feedback policy.

Double integrator, dt = 0.1, finite-horizon LQR used as the inner "solver" so
everything here is exact numpy (no QP package needed). Horizon N = 20 steps.

(a) THE RECEDING HORIZON.  At each tick we solve a fresh N-step plan from the
    MEASURED state (dashed, fading), execute only its FIRST action, let the world
    move, and throw the rest away. The horizon keeps sliding forward -- it
    "recedes". With process noise the plans visibly re-aim each tick: that
    re-aiming IS the feedback.

(b) WHY THE FEEDBACK MATTERS.  A single plan computed at t=0 and played back
    open-loop (10a gotcha #3) never recovers from a disturbance -- it is a
    schedule, not a controller. MPC re-solves from where the robot ACTUALLY is,
    so the kick is rejected.
"""
import numpy as np
import matplotlib.pyplot as plt

BLUE, ORANGE, GREEN, GREY, RED, INK = (
    "#2b6cb0", "#dd6b20", "#2f855a", "#4a5568", "#c53030", "#1a202c")

rng = np.random.default_rng(4)

dt, N = 0.1, 20
A = np.array([[1.0, dt], [0.0, 1.0]])
B = np.array([[0.5 * dt * dt], [dt]])
Q = np.diag([1.0, 0.1])
R = np.array([[0.5]])
Qf = np.diag([20.0, 20.0])


def finite_horizon_gains(N):
    """Backward Riccati sweep -> time-varying gains K[0..N-1]  (08, discrete form)."""
    S = Qf.copy()
    Ks = [None] * N
    for n in range(N - 1, -1, -1):
        K = np.linalg.solve(R + B.T @ S @ B, B.T @ S @ A)     # (R+B'SB)^-1 B'SA
        Ks[n] = K
        S = Q + A.T @ S @ (A - B @ K)
    return Ks


Ks = finite_horizon_gains(N)


def plan_from(x):
    """The inner trajectory optimization: N-step optimal plan from state x."""
    xs, us = [x.copy()], []
    for n in range(N):
        u = float((-Ks[n] @ xs[-1])[0])
        us.append(u)
        xs.append(A @ xs[-1] + B.flatten() * u)
    return np.array(xs), np.array(us)


fig, (axa, axb) = plt.subplots(1, 2, figsize=(13.2, 5.2))

# ---------------- (a) the receding horizon ---------------------------------
T = 26
x = np.array([1.0, 0.0])
traj, plans = [x.copy()], []
for k in range(T):
    xs, us = plan_from(x)                       # (1) solve from the MEASURED state
    plans.append((k, xs))
    x = A @ x + B.flatten() * us[0]             # (2) execute ONLY u[0]
    x = x + np.array([0.0, 0.45]) * rng.standard_normal()   # (3) the world happens
    traj.append(x.copy())
traj = np.array(traj)

show = [0, 6, 12, 18]
for i, k in enumerate(show):
    kk, xs = plans[k]
    tt = (kk + np.arange(N + 1)) * dt
    al = 0.30 + 0.20 * i
    axa.plot(tt, xs[:, 0], "--", color=ORANGE, lw=1.8, alpha=al, zorder=2,
             label="plan solved at each tick (N=20 ahead)" if i == 0 else None)
    axa.plot(tt[0], xs[0, 0], "o", color=ORANGE, ms=6, alpha=al, zorder=3)
    axa.annotate("", xy=(tt[1], xs[1, 0]), xytext=(tt[0], xs[0, 0]),
                 arrowprops=dict(arrowstyle="-|>", color=RED, lw=3.2,
                                 mutation_scale=18), zorder=6)

axa.plot(np.arange(T + 1) * dt, traj[:, 0], "-", color=BLUE, lw=2.8, zorder=4,
         label="what actually happens (closed loop)")
axa.plot([], [], "-", color=RED, lw=2.6, label="the only part executed:  $u[0]$")
axa.axhline(0.0, color=GREEN, lw=1.0, ls=":")
axa.annotate("each tick: measure $x$ $\\rightarrow$ re-solve $\\rightarrow$ "
             "apply $u[0]$ $\\rightarrow$ discard the rest",
             xy=(0.03, 0.06), xycoords="axes fraction", fontsize=10, color=INK,
             bbox=dict(boxstyle="round,pad=0.4", fc="#f7fafc", ec=GREY, lw=0.8))
axa.set_title("(a) Receding horizon: 99% of every solve is thrown away",
              fontsize=12, color=INK)
axa.set_xlabel("time  t"); axa.set_ylabel("position  p")
axa.legend(loc="upper right", fontsize=9, framealpha=0.95)
axa.grid(alpha=0.25)

# ---------------- (b) open-loop playback vs MPC, with a kick ---------------
T2 = 40
kick_step, kick = 12, np.array([0.0, -2.2])

xs0, us0 = plan_from(np.array([1.0, 0.0]))      # ONE plan, computed once at t=0

x = np.array([1.0, 0.0]); ol = [x.copy()]
for k in range(T2):
    u = us0[k] if k < N else 0.0                # blindly replay the schedule
    x = A @ x + B.flatten() * u
    if k == kick_step:
        x = x + kick
    ol.append(x.copy())
ol = np.array(ol)

x = np.array([1.0, 0.0]); mpc = [x.copy()]
for k in range(T2):
    _, us = plan_from(x)                        # re-solve from where we REALLY are
    x = A @ x + B.flatten() * us[0]
    if k == kick_step:
        x = x + kick
    mpc.append(x.copy())
mpc = np.array(mpc)

t2 = np.arange(T2 + 1) * dt
axb.plot(t2, ol[:, 0], "--", color=RED, lw=2.4, label="open-loop playback of the t=0 plan")
axb.plot(t2, mpc[:, 0], "-", color=BLUE, lw=2.8, label="MPC (re-solve every tick)")
axb.axhline(0.0, color=GREEN, lw=1.0, ls=":")
axb.axvline((kick_step + 1) * dt, color=GREY, lw=1.2, ls="-.")
axb.annotate("disturbance\n(velocity kick)", xy=((kick_step + 1) * dt, -0.55),
             xytext=((kick_step + 1) * dt + 0.35, -1.15), color=GREY, fontsize=10,
             arrowprops=dict(arrowstyle="->", color=GREY, lw=1.2))
axb.annotate("a feasible trajectory is a SCHEDULE;\nre-solving makes it a CONTROLLER",
             xy=(0.30, 0.06), xycoords="axes fraction", fontsize=10, color=INK,
             bbox=dict(boxstyle="round,pad=0.4", fc="#fff5f5", ec=RED, lw=0.8))
axb.set_title("(b) The same plan, open-loop vs re-solved", fontsize=12, color=INK)
axb.set_ylim(-3.2, 1.45)
axb.set_xlabel("time  t"); axb.set_ylabel("position  p")
axb.legend(loc="upper right", fontsize=9, framealpha=0.95)
axb.grid(alpha=0.25)

fig.suptitle("Model-Predictive Control: run the trajectory optimization inside the "
             "feedback loop", fontsize=13, color=INK)
fig.tight_layout(rect=(0, 0, 1, 0.94))
fig.savefig("10b_receding_horizon.png", dpi=150)
print("open-loop final p =", round(ol[-1, 0], 3), " MPC final p =", round(mpc[-1, 0], 3))
print("wrote 10b_receding_horizon.png")
