"""Figure: iLQR -- trajectory optimization by repeated LQR.

Same pendulum, same swing-up, but now solved with gradients instead of samples.
The loop is exactly three lines of Ch 8 + Ch 10:

    1. LINEARIZE the dynamics and QUADRATIZE the cost about the current guess
    2. BACKWARD PASS: a Riccati sweep on that local LQ problem -> feedforward k[n]
       and feedback K[n]  (this IS 08's finite-horizon LQR)
    3. FORWARD PASS: roll the real nonlinear dynamics using u + alpha*k + K*dx,
       with a line search on alpha. Repeat.

So iLQR = "keep re-solving 08 in the neighbourhood of where you currently are."
Note what it does NOT have: any way to enforce a hard torque limit. That's the
real limitation of the Riccati route (10b s5) -- constraints have to come back in
via penalties/augmented Lagrangian or by swapping the Riccati solve for a QP.

(a) the trajectory across iterations;  (b) cost vs iteration.
"""
import numpy as np
import matplotlib.pyplot as plt

BLUE, ORANGE, GREEN, GREY, RED, INK = (
    "#2b6cb0", "#dd6b20", "#2f855a", "#4a5568", "#c53030", "#1a202c")

g, l, m, b = 9.81, 1.0, 1.0, 0.10
dt, N = 0.05, 80
xg = np.array([np.pi, 0.0])
Q = np.diag([0.5, 0.05])
R = np.array([[0.08]])
Qf = np.diag([500.0, 50.0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def f(x, u):
    th, thd = x
    thdd = (u - b * thd - m * g * l * np.sin(th)) / (m * l * l)
    return np.array([th + dt * thd, thd + dt * thdd])


def fx(x, u):
    """Analytic Jacobians of the discrete dynamics (no autodiff needed here)."""
    th = x[0]
    A = np.array([[1.0, dt],
                  [-dt * g / l * np.cos(th), 1.0 - dt * b / (m * l * l)]])
    B = np.array([[0.0], [dt / (m * l * l)]])
    return A, B


def err(x):
    return np.array([wrap(x[0] - xg[0]), x[1] - xg[1]])


def rollout(x0, U):
    X = np.zeros((N + 1, 2)); X[0] = x0
    for n in range(N):
        X[n + 1] = f(X[n], U[n])
    return X


def total_cost(X, U):
    J = 0.5 * err(X[N]) @ Qf @ err(X[N])
    for n in range(N):
        e = err(X[n])
        J += 0.5 * e @ Q @ e + 0.5 * R[0, 0] * U[n] ** 2
    return J


def ilqr(x0, U, iters=40):
    X = rollout(x0, U)
    J = total_cost(X, U)
    hist, snaps = [J], {0: (X.copy(), U.copy())}
    mu = 1e-6
    for it in range(1, iters + 1):
        # ---------- backward pass (the Riccati sweep of 08) ------------------
        e = err(X[N])
        Vx, Vxx = Qf @ e, Qf.copy()
        ks, Ks = np.zeros(N), np.zeros((N, 2))
        for n in range(N - 1, -1, -1):
            A, B = fx(X[n], U[n])
            en = err(X[n])
            lx, lu = Q @ en, R[0, 0] * U[n]
            Qx = lx + A.T @ Vx
            Qu = lu + (B.T @ Vx)[0]
            Qxx = Q + A.T @ Vxx @ A
            Vreg = Vxx + mu * np.eye(2)
            Quu = R[0, 0] + (B.T @ Vreg @ B)[0, 0]
            Qux = (B.T @ Vreg @ A)[0]
            k, Kg = -Qu / Quu, -Qux / Quu
            ks[n], Ks[n] = k, Kg
            Vx = Qx + Kg * (Quu * k) + Kg * Qu + Qux * k
            Vxx = Qxx + np.outer(Kg, Quu * Kg) + np.outer(Kg, Qux) + np.outer(Qux, Kg)
            Vxx = 0.5 * (Vxx + Vxx.T)
        # ---------- forward pass with a line search on alpha -----------------
        improved = False
        for alpha in [1.0, 0.5, 0.25, 0.1, 0.05, 0.01]:
            Xn = np.zeros((N + 1, 2)); Xn[0] = x0
            Un = np.zeros(N)
            for n in range(N):
                dx = np.array([wrap(Xn[n, 0] - X[n, 0]), Xn[n, 1] - X[n, 1]])
                Un[n] = U[n] + alpha * ks[n] + Ks[n] @ dx
                Xn[n + 1] = f(Xn[n], Un[n])
            Jn = total_cost(Xn, Un)
            if Jn < J:
                X, U, J, improved = Xn, Un, Jn, True
                mu = max(mu * 0.5, 1e-8)
                break
        if not improved:
            mu *= 10.0                       # regularize and retry
            if mu > 1e6:
                break
        hist.append(J)
        if it in (1, 2, 4, 8, 40):
            snaps[it] = (X.copy(), U.copy())
    return X, U, hist, snaps


rng = np.random.default_rng(1)
U0 = 0.15 * rng.standard_normal(N)           # tiny noise to break the symmetry
X, U, hist, snaps = ilqr(np.array([0.0, 0.0]), U0)

print("iterations =", len(hist) - 1)
print("J: %.1f -> %.4f" % (hist[0], hist[-1]))
print("final theta = %.4f (pi = %.4f), final thetadot = %.4f"
      % (X[-1, 0], np.pi, X[-1, 1]))
print("peak |u| =", round(np.abs(U).max(), 2))

fig, (axa, axb) = plt.subplots(1, 2, figsize=(13.2, 5.0),
                               gridspec_kw=dict(width_ratios=[1.55, 1.0]))
tt = np.arange(N + 1) * dt

# ---------------- (a) the trajectory across iterations ---------------------
keys = sorted(snaps)
cmap = plt.get_cmap("plasma")
for i, it in enumerate(keys):
    Xi, _ = snaps[it]
    frac = i / max(len(keys) - 1, 1)
    axa.plot(tt, Xi[:, 0], "-", color=cmap(0.05 + 0.75 * frac),
             lw=1.6 + 1.6 * frac, alpha=0.55 + 0.45 * frac,
             label=f"iteration {it}" + ("  (converged)" if it == keys[-1] else ""))
for sgn in (1, -1):
    axa.axhline(sgn * np.pi, color=GREEN, lw=1.4, ls=":")
axa.text(0.05, np.pi + 0.15, "upright  $\\theta=\\pm\\pi$", color=GREEN, fontsize=9.5)
axa.annotate("iteration 0 = the initial guess: $u\\approx0$, so it just hangs",
             xy=(2.0, 0.03), xytext=(0.15, -4.15), fontsize=9.5, color=GREY,
             arrowprops=dict(arrowstyle="->", color=GREY, lw=1.2))
axa.set_ylim(-4.7, 6.1)
axa.set_title("(a) Each iteration = linearize $\\rightarrow$ Riccati backward "
              "$\\rightarrow$ nonlinear forward roll", fontsize=11.5, color=INK)
axa.set_xlabel("time  (s)"); axa.set_ylabel("angle  $\\theta$  (rad)")
axa.legend(loc="upper center", fontsize=8.5, framealpha=0.95, ncol=3)
axa.grid(alpha=0.25)

# ---------------- (b) cost vs iteration ------------------------------------
axb.semilogy(np.arange(len(hist)), hist, "o-", color=BLUE, ms=4, lw=2.0)
axb.set_title("(b) Cost per iteration", fontsize=12, color=INK)
axb.set_xlabel("iteration"); axb.set_ylabel("total cost  $J$  (log scale)")
axb.grid(alpha=0.25, which="both")
axb.annotate("one Riccati sweep updates the WHOLE\ntrajectory, not one small gradient step\n"
             "$\\Rightarrow$ most of the progress in ~5 iterations,\nthen a long polishing tail",
             xy=(0.22, 0.62), xycoords="axes fraction", fontsize=9.5, color=INK,
             bbox=dict(boxstyle="round,pad=0.4", fc="#f7fafc", ec=GREY, lw=0.8))

fig.suptitle("iLQR: trajectory optimization as repeated LQR "
             "(DDP = the same thing with 2nd-order dynamics terms)",
             fontsize=13, color=INK)
fig.tight_layout(rect=(0, 0, 1, 0.93))
fig.savefig("10b_ilqr.png", dpi=150)
print("wrote 10b_ilqr.png")
