"""Figure: the two ways to turn a trajectory into a finite list of numbers.

Double integrator, x = [p, v], h = 0.1, N = 20. Task: go from p=0 to p=1, at rest.

(a) DIRECT SHOOTING -- decision variables are u[0..N-1] only. x[.] comes from
    rolling the dynamics forward, so EVERY candidate is a physically real
    trajectory. The optimizer's job is to steer that rollout onto the target.
    Three different u-guesses -> three real arcs, only one lands.

(b) DIRECT TRANSCRIPTION -- decision variables are x[0..N] AND u[0..N-1].
    The nodes start out NOT obeying the dynamics: the red bars are the
    "defects", x[n+1] - f(x[n],u[n]). They are equality constraints the solver
    drives to zero. Until it converges, the "trajectory" is physically fake --
    but the endpoint condition is satisfied from iteration zero.
"""
import numpy as np
import matplotlib.pyplot as plt

BLUE, ORANGE, GREEN, GREY, RED, INK = (
    "#2b6cb0", "#dd6b20", "#2f855a", "#4a5568", "#c53030", "#1a202c")

h, N = 0.1, 20
A = np.array([[1.0, h], [0.0, 1.0]])
B = np.array([[0.5 * h * h], [h]])
x0 = np.array([0.0, 0.0])
xg = np.array([1.0, 0.0])
t = np.arange(N + 1) * h


def rollout(u):
    x = np.zeros((N + 1, 2))
    x[0] = x0
    for n in range(N):
        x[n + 1] = A @ x[n] + B.flatten() * u[n]
    return x


# --- three candidate input sequences for the shooting panel -----------------
# bang-bang that actually lands at p=1 with v=0: accelerate a, then brake a,
# with distance = a*T^2/4  =>  a = 4*d/T^2.
T = N * h
a_star = 4.0 * 1.0 / T**2
u_opt = np.concatenate([np.full(N // 2, a_star), np.full(N // 2, -a_star)])
u_lazy = 0.5 * u_opt
u_hot = 1.45 * u_opt

fig, (axs, axt) = plt.subplots(1, 2, figsize=(13.0, 5.0))

# ---------------- (a) direct shooting --------------------------------------
for u, c, ls, lab in [(u_lazy, GREY, "--", "guess 1  (u too small)"),
                      (u_hot, ORANGE, "--", "guess 2  (u too large)"),
                      (u_opt, BLUE, "-", "converged u*")]:
    x = rollout(u)
    axs.plot(t, x[:, 0], ls, color=c, lw=2.4 if ls == "-" else 1.8, label=lab)
    axs.plot(t[-1], x[-1, 0], "o", color=c, ms=7,
             mfc=c if ls == "-" else "white", zorder=5)

axs.axhline(1.0, color=GREEN, lw=1.2, ls=":")
axs.plot(T, 1.0, "*", color=GREEN, ms=20, zorder=6)
axs.annotate("target p(T)=1", xy=(T, 1.0), xytext=(T - 0.95, 1.28),
             color=GREEN, fontsize=10,
             arrowprops=dict(arrowstyle="->", color=GREEN, lw=1.2))
axs.annotate("every curve is a REAL trajectory\n(dynamics satisfied by construction)",
             xy=(0.55, 0.18), fontsize=10, color=INK,
             bbox=dict(boxstyle="round,pad=0.4", fc="#f7fafc", ec=GREY, lw=0.8))
axs.set_title("(a) Direct shooting:  decision vars = $u[0..N\\!-\\!1]$ only",
              fontsize=12, color=INK)
axs.set_xlabel("time  t"); axs.set_ylabel("position  p")
axs.legend(loc="upper left", fontsize=9, framealpha=0.95)
axs.set_ylim(-0.15, 1.75); axs.grid(alpha=0.25)

# ---------------- (b) direct transcription ---------------------------------
# a typical INITIAL GUESS: straight line in p, v = 0, u = 0. Nothing obeys f.
p_guess = np.linspace(0.0, 1.0, N + 1)
v_guess = np.zeros(N + 1)
u_guess = np.zeros(N)
X = np.stack([p_guess, v_guess], axis=1)

# defect[n] = f(x[n],u[n]) - x[n+1]   (what the solver must drive to zero)
pred = np.array([A @ X[n] + B.flatten() * u_guess[n] for n in range(N)])
defect_p = pred[:, 0] - X[1:, 0]

x_conv = rollout(u_opt)

axt.plot(t, x_conv[:, 0], "-", color=BLUE, lw=2.4, zorder=2,
         label="converged (defects = 0)")
axt.plot(t, p_guess, "o--", color=GREY, lw=1.4, ms=6, mfc="white",
         label="initial guess: straight line", zorder=3)

for n in range(N):
    axt.plot([t[n + 1], t[n + 1]], [X[n + 1, 0], pred[n, 0]],
             "-", color=RED, lw=2.6, solid_capstyle="butt", zorder=4)
axt.plot([], [], "-", color=RED, lw=2.6,
         label="defect  $x[n\\!+\\!1]-f(x[n],u[n])$")

axt.annotate("nodes are FREE variables --\nthe dynamics are just equality\nconstraints, violated until convergence",
             xy=(0.05, 1.18), fontsize=10, color=INK,
             bbox=dict(boxstyle="round,pad=0.4", fc="#fff5f5", ec=RED, lw=0.8))
axt.plot(T, 1.0, "*", color=GREEN, ms=20, zorder=6)
axt.set_title("(b) Direct transcription:  decision vars = $x[0..N]$  AND  $u[0..N\\!-\\!1]$",
              fontsize=12, color=INK)
axt.set_xlabel("time  t"); axt.set_ylabel("position  p")
axt.legend(loc="lower right", fontsize=9, framealpha=0.95)
axt.set_ylim(-0.15, 1.75); axt.grid(alpha=0.25)

fig.suptitle("Two transcriptions of the same problem: who owns the dynamics, "
             "the simulator or the solver?", fontsize=13, color=INK)
fig.tight_layout(rect=(0, 0, 1, 0.94))
fig.savefig("10a_shooting_vs_transcription.png", dpi=150)
print("max |defect_p| in the guess =", np.abs(defect_p).max())
print("wrote 10a_shooting_vs_transcription.png")
