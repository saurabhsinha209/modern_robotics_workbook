"""Figure: WHY you might prefer direct transcription -- conditioning & sparsity.

(a) Shooting sensitivity.  In shooting, x[N] = A^N x[0] + sum_k A^(N-1-k) B u[k],
    so the gradient of the terminal state w.r.t. an early input is A^(N-1-k) B.
    On an UNSTABLE system (linearized inverted pendulum, eigenvalues +-3.13),
    A^n grows like e^(3.13 n h): over a 3 s horizon u[0] comes out ~2800x more
    powerful than u[N-1]. One optimization problem, coefficients spanning three
    and a half orders of magnitude -> ill-conditioned.  This is literally the exploding-gradient problem of deep
    nets: a rollout IS a deep network with the same weight matrix every layer.

(b) Transcription sparsity.  Direct transcription's constraint Jacobian is
    BANDED -- each dynamics constraint touches only x[n], u[n], x[n+1]. The bad
    A^n coefficients never get formed; they are spread over N well-scaled rows.
    This is the same banded structure the Riccati recursion exploits.
"""
import numpy as np
import matplotlib.pyplot as plt

BLUE, ORANGE, GREEN, GREY, RED, INK = (
    "#2b6cb0", "#dd6b20", "#2f855a", "#4a5568", "#c53030", "#1a202c")

# ---- linearized inverted pendulum about the top: xdot = [[0,1],[g/l,0]] x + [0,1/ml^2] u
g, l, m = 9.81, 1.0, 1.0
Ac = np.array([[0.0, 1.0], [g / l, 0.0]])
Bc = np.array([[0.0], [1.0 / (m * l * l)]])
h, N = 0.05, 60
Ad = np.eye(2) + h * Ac          # forward Euler, enough to make the point
Bd = h * Bc
print("cont. eigs =", np.linalg.eigvals(Ac), " discrete eigs =", np.linalg.eigvals(Ad))

sens = []
for k in range(N):
    M = np.linalg.matrix_power(Ad, N - 1 - k) @ Bd
    sens.append(np.linalg.norm(M))
sens = np.array(sens)

fig, (axc, axs) = plt.subplots(1, 2, figsize=(13.0, 5.0))

# ---------------- (a) conditioning -----------------------------------------
axc.semilogy(np.arange(N), sens, "o-", color=RED, ms=4, lw=2.0)
axc.set_title("(a) Shooting: $\\|\\partial x[N]/\\partial u[k]\\| = \\|A^{N-1-k}B\\|$",
              fontsize=12, color=INK)
axc.set_xlabel("input index  k   (early  $\\rightarrow$  late)")
axc.set_ylabel("sensitivity of the final state  (log scale)")
axc.grid(alpha=0.25, which="both")
ratio = sens[0] / sens[-1]
axc.annotate(f"u[0] is {ratio:,.0f}$\\times$ more\ninfluential than u[N-1]",
             xy=(2, sens[0]), xytext=(14, sens[0] * 0.55), color=RED, fontsize=10,
             arrowprops=dict(arrowstyle="->", color=RED, lw=1.2))
axc.annotate("\"tail wagging the dog\"\n= exploding gradients\nin a deep net",
             xy=(0.04, 0.06), xycoords="axes fraction", fontsize=10, color=INK,
             bbox=dict(boxstyle="round,pad=0.4", fc="#fff5f5", ec=RED, lw=0.8))

# ---------------- (b) sparsity ---------------------------------------------
# variables z = [x0(2), u0(1), x1(2), u1(1), ... , xM(2)] for a short horizon
M = 8
nx, nu = 2, 1
nvar = (M + 1) * nx + M * nu
ncon = nx + M * nx                      # x[0]=x0  plus  M dynamics constraints
J = np.zeros((ncon, nvar))


def ix(n):     # slice of x[n] in z
    return slice(n * (nx + nu), n * (nx + nu) + nx)


def iu(n):     # slice of u[n] in z
    return slice(n * (nx + nu) + nx, n * (nx + nu) + nx + nu)


J[0:nx, ix(0)] = np.eye(nx)
for n in range(M):
    r = slice(nx + n * nx, nx + (n + 1) * nx)
    J[r, ix(n)] = -Ad
    J[r, iu(n)] = -Bd
    J[r, ix(n + 1)] = np.eye(nx)

axs.imshow(np.abs(J) > 1e-12, cmap="Blues", aspect="auto", interpolation="nearest")
axs.set_title("(b) Transcription: constraint Jacobian is BANDED",
              fontsize=12, color=INK)
axs.set_xlabel("decision variables  $z=[x_0,u_0,x_1,u_1,\\dots,x_M]$")
axs.set_ylabel("equality constraints")
axs.set_xticks(np.arange(0, nvar, 3))
axs.grid(color="white", lw=0.6, alpha=0.6)
axs.annotate("each row touches only\n$x[n],\\,u[n],\\,x[n\\!+\\!1]$\n$\\Rightarrow$ no $A^n$ ever formed",
             xy=(0.50, 0.80), xycoords="axes fraction", fontsize=10, color=INK,
             bbox=dict(boxstyle="round,pad=0.4", fc="#f7fafc", ec=BLUE, lw=0.8))

fig.suptitle("Direct transcription trades more variables for far better numerics",
             fontsize=13, color=INK)
fig.tight_layout(rect=(0, 0, 1, 0.94))
fig.savefig("10a_conditioning.png", dpi=150)
print("sensitivity ratio u[0]/u[N-1] =", ratio)
print("wrote 10a_conditioning.png")
