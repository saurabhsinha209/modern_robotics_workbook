"""Figure: change of basis, and what the eigenvalues of A mean for a flow.

(a) CHANGE OF BASIS. One fixed arrow x = (3,3). Two ways to describe it:
    - standard axes:      coordinates (3, 3)
    - eigenvector axes v1,v2:  coordinates (2, 1)  [x = 2*v1 + 1*v2]
    The arrow never moves. Only the *list of numbers* describing it changes,
    because we changed the ruler we measure with. r = V^-1 x answers
    'how much of each new axis do I need to build x'.

(b) EIGENVALUES AS RATES for xdot = A x. Along an eigenvector the flow only
    scales, at rate lambda:  that mode evolves as e^{lambda t}.
      lambda < 0  -> that direction CONTRACTS (stable)
      lambda > 0  -> that direction EXPANDS  (unstable)
    A saddle (one of each) is exactly the pendulum's upright equilibrium from
    note 02. The eigen-lines are the only straight-line trajectories; every
    other trajectory is a blend of the two exponentials.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})
BLUE, ORANGE, GREY, RED, GREEN, PURPLE = (
    "#2b6cb0", "#dd6b20", "#4a5568", "#c53030", "#2f855a", "#6b46c1")

# eigenvectors (columns of V)
v1 = np.array([2.0, 1.0])
v2 = np.array([-1.0, 1.0])
V = np.column_stack([v1, v2])
Vinv = np.linalg.inv(V)

fig, axes = plt.subplots(1, 2, figsize=(13, 5.6))

# =================================================================== (a)
ax = axes[0]
x = np.array([3.0, 3.0])
r = Vinv @ x            # (2, 1)

# faint standard grid
for k in range(-1, 6):
    ax.axhline(k, color="#e2e8f0", lw=0.8, zorder=0)
    ax.axvline(k, color="#e2e8f0", lw=0.8, zorder=0)

# eigenvector axes (skewed grid) through origin
for a in np.arange(-3, 6):
    ax.plot(*zip(a * v1 - 3 * v2, a * v1 + 4 * v2), color=ORANGE, lw=0.6,
            alpha=0.45, zorder=1)
    ax.plot(*zip(a * v2 - 3 * v1, a * v2 + 4 * v1), color=ORANGE, lw=0.6,
            alpha=0.45, zorder=1)

# the two eigenvector basis arrows
for v, lab, col in ((v1, r"$v_1$", ORANGE), (v2, r"$v_2$", ORANGE)):
    ax.annotate("", xy=v, xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color=col, lw=2.6))
    ax.text(*(v * 1.12), lab, color=col, fontsize=13, weight="bold")

# standard basis arrows
for e, lab in ((np.array([1, 0]), r"$e_1$"), (np.array([0, 1]), r"$e_2$")):
    ax.annotate("", xy=e, xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color=GREY, lw=2.0))
    ax.text(*(e * 1.35 + np.array([0.05, -0.12])), lab, color=GREY, fontsize=11)

# the vector itself
ax.annotate("", xy=x, xytext=(0, 0),
            arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=3.2))
ax.plot(*x, "o", color=BLUE, ms=7, zorder=6)

# show the decomposition x = 2 v1 + 1 v2
ax.annotate("", xy=2 * v1, xytext=(0, 0),
            arrowprops=dict(arrowstyle="-", color=BLUE, lw=1.3, ls=":"))
ax.annotate("", xy=x, xytext=2 * v1,
            arrowprops=dict(arrowstyle="-", color=BLUE, lw=1.3, ls=":"))

ax.text(3.15, 2.75, r"$x$", color=BLUE, fontsize=14, weight="bold")
ax.text(0.15, 5.3,
        "same arrow, two descriptions:\n"
        r"standard axes:  $x=(3,3)$" "\n"
        r"eigen axes:  $x = 2v_1 + 1v_2 \Rightarrow (2,1)$",
        fontsize=10, va="top",
        bbox=dict(boxstyle="round,pad=0.4", fc="white", ec=GREY, alpha=0.95))

ax.set_xlim(-2.2, 5.6)
ax.set_ylim(-1.6, 5.8)
ax.set_aspect("equal")
ax.set_title("(a) change of basis — the arrow is fixed,\nthe coordinates depend on the ruler",
             fontsize=11.5)
ax.set_xticks([]); ax.set_yticks([])

# =================================================================== (b)
ax = axes[1]
lam = np.array([-0.8, 0.8])          # one stable, one unstable -> saddle
A = V @ np.diag(lam) @ Vinv

# vector field
xs = np.linspace(-3, 3, 22)
X, Y = np.meshgrid(xs, xs)
U = A[0, 0] * X + A[0, 1] * Y
Wv = A[1, 0] * X + A[1, 1] * Y
ax.quiver(X, Y, U, Wv, color="#a0aec0", alpha=0.8, width=0.004, zorder=1)

# eigen-lines
t = np.linspace(-3.4, 3.4, 2)
# stable eigenvector v1 (lambda<0): arrows point INWARD
ln = np.outer(t, v1 / np.linalg.norm(v1)) * 1.4
ax.plot(ln[:, 0], ln[:, 1], color=GREEN, lw=2.6, zorder=3)
# unstable eigenvector v2 (lambda>0): arrows point OUTWARD
ln2 = np.outer(t, v2 / np.linalg.norm(v2)) * 1.4
ax.plot(ln2[:, 0], ln2[:, 1], color=RED, lw=2.6, zorder=3)

# integrate a few generic trajectories to show the blend
def flow(p, T=2.6, dt=0.01):
    pts = [p.copy()]
    for _ in range(int(T / dt)):
        p = p + dt * (A @ p)
        pts.append(p.copy())
    return np.array(pts)

for p0 in [np.array([0.35, -0.15]), np.array([-0.35, 0.15]),
           np.array([0.2, 0.35]), np.array([-0.2, -0.35])]:
    tr = flow(p0)
    keep = (np.abs(tr[:, 0]) < 3.2) & (np.abs(tr[:, 1]) < 3.2)
    ax.plot(tr[keep, 0], tr[keep, 1], color=BLUE, lw=1.6, alpha=0.9, zorder=4)

# direction arrows on eigen-lines
for s in (1, -1):
    ax.annotate("", xy=(0.9 * s * v1[0] / np.hypot(*v1), 0.9 * s * v1[1] / np.hypot(*v1)),
                xytext=(1.7 * s * v1[0] / np.hypot(*v1), 1.7 * s * v1[1] / np.hypot(*v1)),
                arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=2.0))
    ax.annotate("", xy=(1.7 * s * v2[0] / np.hypot(*v2), 1.7 * s * v2[1] / np.hypot(*v2)),
                xytext=(0.9 * s * v2[0] / np.hypot(*v2), 0.9 * s * v2[1] / np.hypot(*v2)),
                arrowprops=dict(arrowstyle="-|>", color=RED, lw=2.0))

ax.plot(0, 0, "o", ms=10, mfc="white", mec=GREY, mew=2.0, zorder=8)
ax.text(2.0, 1.35, r"$v_1$: $\lambda=-0.8$" "\ncontracts (stable)", color=GREEN,
        fontsize=9.5, weight="bold", ha="center")
ax.text(-1.65, 1.9, r"$v_2$: $\lambda=+0.8$" "\nexpands (unstable)", color=RED,
        fontsize=9.5, weight="bold", ha="center")

ax.set_xlim(-3.2, 3.2)
ax.set_ylim(-3.2, 3.2)
ax.set_aspect("equal")
ax.set_title("(b) eigenvalues of $A$ = rates of the flow $\\dot x = Ax$\n"
             "(a saddle — this IS the pendulum's upright)", fontsize=11.5)
ax.set_xticks([]); ax.set_yticks([])

fig.suptitle("Change of basis  &  what the eigenvalues of $A$ mean",
             fontsize=13, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.94])
out = __file__.replace("gen_03b_basis_eigen.py", "03b_basis_eigen.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
print("A =", A.round(3).tolist(), " eigs =", np.linalg.eigvals(A).round(3))
print("x=(3,3) in eigen coords r = V^-1 x =", (Vinv @ np.array([3.0, 3.0])).round(3))
