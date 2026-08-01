"""Figure: LQR = DP's closed-form case, on the double integrator.

(a) The cost-to-go J*(x) = x^T S x is a quadratic BOWL (Ch 7's landscape, now
    smooth). Contours are ellipses; S's eigenvectors are the bowl's principal
    axes. Closed-loop trajectories flow DOWNHILL across the contours to the origin
    -- the LQR controller is "descend the cost-to-go," exactly like Ch 7.

(b) The finite-horizon Riccati sweep run BACKWARD in time is value iteration in
    closed form: start from terminal S=0 and integrate; ||S|| settles to the
    infinite-horizon (algebraic Riccati) value -- the steady S that gives the
    3b balancing gain. No scipy: we integrate the Riccati ODE ourselves.
"""
import numpy as np
import matplotlib.pyplot as plt

BLUE, ORANGE, GREEN, GREY, RED, INK = (
    "#2b6cb0", "#dd6b20", "#2f855a", "#4a5568", "#c53030", "#1a202c")

# double integrator:  x=[pos, vel],  xdot = A x + B u
A = np.array([[0.0, 1.0], [0.0, 0.0]])
B = np.array([[0.0], [1.0]])
Q = np.eye(2)
R = np.array([[1.0]])
Rinv = np.linalg.inv(R)

# ---- solve the Riccati by integrating BACKWARD from S(T)=0 (the value-iteration sweep)
#   dS/dtau = A^T S + S A - S B Rinv B^T S + Q,   S(0)=0,   tau = T - t
dt = 1e-3
S = np.zeros((2, 2))
norms, taus = [], []
for i in range(20000):
    dS = A.T @ S + S @ A - S @ B @ Rinv @ B.T @ S + Q
    S = S + dt * dS
    norms.append(np.linalg.norm(S)); taus.append(i * dt)
    if i > 0 and abs(norms[-1] - norms[-2]) < 1e-12:
        break
K = Rinv @ B.T @ S                      # optimal gain u = -K x
Acl = A - B @ K                         # closed-loop dynamics
evals, evecs = np.linalg.eigh(S)        # S is symmetric PD -> real eigenpairs
print("S =\n", np.round(S, 4))
print("K =", np.round(K, 4))
print("eig(S) =", np.round(evals, 4), " eig(Acl) =", np.round(np.linalg.eigvals(Acl), 4))

fig, (axb, axr) = plt.subplots(1, 2, figsize=(12.6, 5.2))

# ============ (a) cost-to-go bowl + closed-loop trajectories
g = np.linspace(-3, 3, 300)
X, Y = np.meshgrid(g, g)
J = S[0, 0] * X**2 + 2 * S[0, 1] * X * Y + S[1, 1] * Y**2
cs = axb.contour(X, Y, J, levels=np.linspace(1, 40, 10), colors=[BLUE], linewidths=0.9,
                 alpha=0.55)
axb.clabel(cs, inline=True, fontsize=6, fmt="%.0f")

# closed-loop trajectories descending the bowl
for x0 in [(-2.6, 1.8), (2.6, -1.8), (-2.2, -2.2), (2.2, 2.2), (0.2, 2.8), (-0.2, -2.8)]:
    x = np.array(x0, float); traj = [x.copy()]
    for _ in range(4000):
        x = x + 0.004 * (Acl @ x)
        traj.append(x.copy())
    traj = np.array(traj)
    axb.plot(traj[:, 0], traj[:, 1], color=ORANGE, lw=1.8, zorder=4)
    axb.plot(*x0, "o", color=ORANGE, ms=5, zorder=5)

# eigenvector axes of S (principal axes of the bowl)
for lam, v in zip(evals, evecs.T):
    L = 2.4 / np.sqrt(lam) * np.sqrt(evals.mean())   # scale for visibility
    axb.annotate("", xy=(v[0]*L, v[1]*L), xytext=(-v[0]*L, -v[1]*L),
                 arrowprops=dict(arrowstyle="-", color=GREEN, lw=2.2, alpha=0.9))
axb.text(0, 0, "goal", color=RED, fontsize=8, ha="center", va="center", weight="bold")
axb.plot(0, 0, "*", color=RED, ms=15, zorder=6)
axb.set_xlim(-3, 3); axb.set_ylim(-3, 3); axb.set_aspect("equal")
axb.set_xlabel("position $x_1$"); axb.set_ylabel("velocity $x_2$")
axb.set_title("(a) cost-to-go  $J^*=x^\\top S x$ : a quadratic bowl",
              fontsize=10.5, weight="bold")
axb.text(-2.9, 2.55, "green = eigenvectors of $S$\n(bowl's principal axes)",
         fontsize=7.5, color=GREEN)
axb.text(-2.9, -2.55, "orange = closed-loop\ntrajectories flow downhill",
         fontsize=7.5, color=ORANGE)

# ============ (b) backward Riccati sweep converging to the ARE
axr.plot(taus, norms, color=BLUE, lw=2.4)
axr.axhline(norms[-1], color=RED, ls="--", lw=1.3)
axr.text(taus[-1]*0.62, norms[-1]*0.9,
         f"settles to $\\|S_\\infty\\|={norms[-1]:.3f}$\n(the algebraic Riccati soln =\n"
         f"the 3b balancing gain)", fontsize=8, color=RED)
axr.set_xlim(0, taus[len(taus)//3]);
axr.set_xlabel("backward time  $\\tau = T - t$  (how long a horizon)")
axr.set_ylabel("$\\|S(\\tau)\\|$")
axr.set_title("(b) Riccati swept backward from $S(T)=0$\n"
              "= value iteration, closed form → infinite-horizon $S$",
              fontsize=10.5, weight="bold")
axr.grid(alpha=0.25, lw=0.5)

fig.suptitle("LQR = dynamic programming's one closed-form case: quadratic cost-to-go, "
             "solved by the Riccati backward sweep", fontsize=12, y=1.01, weight="bold")
fig.tight_layout()
out = __file__.replace("gen_08_lqr_costtogo.py", "08_lqr_costtogo.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
