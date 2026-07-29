"""Figure: LQR balancing the cart-pole, and what the cost weights Q, R buy you.

Using the 3a linearization (constants = 1):

    A = [[0,0,1,0],[0,0,0,1],[0,1,0,0],[0,2,0,0]],   B = [0,0,1,1]^T

LQR picks u = -K x to minimise  J = integral( x^T Q x + u^T R u ) dt. We compute
K by integrating the Riccati ODE to steady state (no scipy needed), then simulate
the TRUE nonlinear cart-pole from a small initial lean under u = -K x_bar.

Panel (a): the pole angle theta(t) driven back to upright (pi) for two designs.
Panel (b): the control effort u(t) for each.

Two designs contrast the fundamental Q-vs-R knob:
  - 'cheap control' (small R): aggressive, fast recovery, big forces.
  - 'expensive control' (large R): gentle, slow recovery, small forces.

The non-minimum-phase wink (Tedrake): to bring the pole left, the cart first
darts the 'wrong' way -- watch x(t) move opposite to its final destination early
on. We show it in panel (c).
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})
BLUE, ORANGE, GREY, RED = "#2b6cb0", "#dd6b20", "#4a5568", "#c53030"

A = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, 1, 0, 0], [0, 2, 0, 0]], float)
B = np.array([[0], [0], [1], [1]], float)
XSTAR = np.array([0.0, np.pi, 0.0, 0.0])


def lqr_gain(Q, R, iters=300000, dt=1e-4):
    P = np.eye(4)
    Rinv = np.linalg.inv(R)
    for _ in range(iters):
        P = P + dt * (A.T @ P + P @ A - P @ B @ Rinv @ B.T @ P + Q)
    return Rinv @ B.T @ P


def cartpole_f(s, u):
    """Exact nonlinear cart-pole, constants = 1 (Tedrake eq 16-17)."""
    x, th, xd, thd = s
    den = 1 + np.sin(th) ** 2
    xdd = (u + np.sin(th) * (thd ** 2 + np.cos(th))) / den
    thdd = (-u * np.cos(th) - thd ** 2 * np.cos(th) * np.sin(th)
            - 2 * np.sin(th)) / den
    return np.array([xd, thd, xdd, thdd])


def rollout(K, s0, T=8.0, dt=0.002):
    s = s0.copy()
    S, U, TT = [s.copy()], [float(-K @ (s - XSTAR))], [0.0]
    for i in range(int(T / dt)):
        u = float(-K @ (s - XSTAR))
        k1 = cartpole_f(s, u)
        k2 = cartpole_f(s + 0.5 * dt * k1, u)
        k3 = cartpole_f(s + 0.5 * dt * k2, u)
        k4 = cartpole_f(s + dt * k3, u)
        s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        S.append(s.copy())
        U.append(u)
        TT.append((i + 1) * dt)
    return np.array(TT), np.array(S), np.array(U)


# two designs
K_cheap = lqr_gain(np.diag([1, 10, 1, 1.0]), np.array([[0.3]]))     # small R
K_exp = lqr_gain(np.diag([1, 10, 1, 1.0]), np.array([[8.0]]))       # large R

s0 = np.array([0.0, np.pi - 0.5, 0.0, 0.0])   # start leaning 0.5 rad off upright
t_c, S_c, U_c = rollout(K_cheap, s0)
t_e, S_e, U_e = rollout(K_exp, s0)

fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.6))

# (a) pole angle
ax = axes[0]
ax.axhline(np.pi, color=GREY, lw=1.2, ls="--")
ax.text(t_c[-1] * 0.98, np.pi + 0.02, "upright $=\\pi$", ha="right",
        color=GREY, fontsize=9)
ax.plot(t_c, S_c[:, 1], color=BLUE, lw=2.2, label="cheap control (small $R$)")
ax.plot(t_e, S_e[:, 1], color=ORANGE, lw=2.2, label="expensive control (large $R$)")
ax.set_xlim(0, t_c[-1])
ax.set_xlabel("time  $t$")
ax.set_ylabel(r"pole angle  $\theta(t)$")
ax.set_title("(a) both recover the upright", fontsize=11)
ax.legend(fontsize=8.5, loc="lower right", framealpha=0.92)
ax.grid(alpha=0.25, lw=0.5)

# (b) control effort
ax = axes[1]
ax.axhline(0, color=GREY, lw=0.8)
ax.plot(t_c, U_c, color=BLUE, lw=2.0, label="cheap: fast, big force")
ax.plot(t_e, U_e, color=ORANGE, lw=2.0, label="expensive: gentle, small force")
ax.set_xlim(0, t_c[-1])
ax.set_xlabel("time  $t$")
ax.set_ylabel(r"control force  $u(t)$")
ax.set_title(r"(b) $R$ is the price of control effort", fontsize=11)
ax.legend(fontsize=8.5, loc="upper right", framealpha=0.92)
ax.grid(alpha=0.25, lw=0.5)

fig.suptitle("LQR balancing the cart-pole:  $u = -K\\,\\bar x$  minimising  "
             r"$\int (\bar x^T Q\bar x + u^T R u)\,dt$"
             "\none linearized controller, run on the true nonlinear system",
             fontsize=12.5, y=1.03)
fig.tight_layout(rect=[0, 0, 1, 0.90])
out = __file__.replace("gen_03b_lqr_balance.py", "03b_lqr_balance.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
print("K_cheap =", K_cheap.round(3), " K_exp =", K_exp.round(3))
