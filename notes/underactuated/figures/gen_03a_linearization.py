"""Figure: what linearization does, and why it is only LOCAL.

Take the pendulum near its UPRIGHT equilibrium. Put phi = theta - pi so that
phi = 0 is upright. With m = l = g = 1 the true dynamics are

    phiddot = sin(phi)          (nonlinear, exact)

and the linearization about phi = 0 replaces sin(phi) by its tangent line:

    phiddot = phi               (linear approximation)

Note the sign: phiddot = +phi, not -phi. Positive feedback -- lean further and
gravity pulls you further. Eigenvalues +1 and -1: a SADDLE, i.e. unstable, which
is exactly what "balancing a pole" feels like.

  (a) the nonlinearity sin(phi) against its tangent line phi.
  (b) small initial lean -- the two agree to plotting accuracy.
  (c) large initial lean -- they visibly part company.

The moral: linearization keeps the dynamics honest in a NEIGHBOURHOOD of the
fixed point, which is all a balancing controller needs -- and is exactly why
LQR (3b) balances but cannot swing up (3c).
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})

BLUE, ORANGE, GREY, RED = "#2b6cb0", "#dd6b20", "#4a5568", "#c53030"


def simulate(phi0, nonlinear, T=2.6, dt=0.002):
    """phiddot = sin(phi) (true) or phiddot = phi (linearized)."""
    f = np.sin if nonlinear else (lambda z: z)
    phi, phid = phi0, 0.0
    P, TT = [phi], [0.0]
    for i in range(int(T / dt)):
        # RK4 on (phi, phid)
        def rhs(p, v):
            return v, f(p)
        k1 = rhs(phi, phid)
        k2 = rhs(phi + 0.5 * dt * k1[0], phid + 0.5 * dt * k1[1])
        k3 = rhs(phi + 0.5 * dt * k2[0], phid + 0.5 * dt * k2[1])
        k4 = rhs(phi + dt * k3[0], phid + dt * k3[1])
        phi = phi + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        phid = phid + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        P.append(phi)
        TT.append((i + 1) * dt)
    return np.array(TT), np.array(P)


fig, axes = plt.subplots(1, 3, figsize=(14, 4.4))

# ---------------- (a) the nonlinearity and its tangent ----------------
ax = axes[0]
phi = np.linspace(-np.pi, np.pi, 500)
ax.plot(phi, np.sin(phi), color=BLUE, lw=2.6, label=r"true:  $\sin\phi$", zorder=4)
ax.plot(phi, phi, color=ORANGE, lw=2.2, ls="--",
        label=r"linearized:  $\phi$", zorder=5)
ax.axhline(0, color=GREY, lw=0.8)
ax.axvline(0, color=GREY, lw=0.8)
ax.axvspan(-0.45, 0.45, color=BLUE, alpha=0.12, zorder=0)
ax.text(0, -2.55, "they agree\nhere", ha="center", fontsize=9, color=BLUE,
        weight="bold")
ax.plot(0, 0, "o", ms=9, mfc="white", mec=GREY, mew=2.0, zorder=8)
ax.set_xlim(-np.pi, np.pi)
ax.set_ylim(-3.0, 3.0)
ax.set_xticks([-np.pi, -np.pi / 2, 0, np.pi / 2, np.pi])
ax.set_xticklabels([r"$-\pi$", r"$-\pi/2$", "0", r"$\pi/2$", r"$\pi$"])
ax.set_xlabel(r"$\phi = \theta - \pi$   (0 = upright)")
ax.set_ylabel(r"$\ddot{\phi}$")
ax.set_title("(a) replace the curve by its tangent", fontsize=11)
ax.legend(loc="upper left", fontsize=9, framealpha=0.92)
ax.grid(alpha=0.25, lw=0.5)

# ---------------- (b), (c) trajectories ----------------
for ax, phi0, tag in ((axes[1], 0.10, "small"), (axes[2], 1.10, "large")):
    t_n, p_n = simulate(phi0, nonlinear=True)
    t_l, p_l = simulate(phi0, nonlinear=False)
    ax.plot(t_n, p_n, color=BLUE, lw=2.4, label="true nonlinear", zorder=4)
    ax.plot(t_l, p_l, color=ORANGE, lw=2.2, ls="--", label="linearized", zorder=5)
    ax.axhline(np.pi, color=GREY, lw=1.0, ls=":")
    ax.text(0.06, np.pi + 0.12, "hanging down", fontsize=8.5, color=GREY)
    ax.set_xlim(0, t_n[-1])
    ax.set_ylim(0, 4.4)
    ax.set_xlabel("time  $t$")
    ax.set_ylabel(r"$\phi(t)$")
    ax.set_title(f"({'b' if tag == 'small' else 'c'}) {tag} initial lean, "
                 rf"$\phi(0)={phi0}$", fontsize=11)
    ax.legend(loc="upper left", fontsize=9, framealpha=0.92)
    ax.grid(alpha=0.25, lw=0.5)
    err = np.abs(p_n - p_l).max()
    ax.text(0.97, 0.06, f"max gap = {err:.2f} rad", transform=ax.transAxes,
            ha="right", fontsize=9, color=RED, weight="bold",
            bbox=dict(boxstyle="round,pad=0.28", fc="white", ec=RED, alpha=0.9))

fig.suptitle("Linearizing about the upright:  $\\ddot{\\phi}=\\sin\\phi \\;\\to\\; "
             "\\ddot{\\phi}=\\phi$  — exact at the fixed point, good nearby, wrong far away",
             fontsize=12.5, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.93])
out = __file__.replace("gen_03a_linearization.py", "03a_linearization.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
