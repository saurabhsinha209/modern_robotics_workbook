"""Figure: controllability via modal decomposition — 'can the input touch every mode?'

Tedrake's cleanest intuition (the non-repeated-eigenvalue case). Change to modal
coordinates r = V^-1 x, where V holds the eigenvectors of A. In those
coordinates the dynamics DIAGONALIZE:

    rdot_i = lambda_i r_i + (V^-1 B)_i u

so the n coupled equations become n INDEPENDENT scalar equations -- one per
eigenmode. Input j can touch mode i iff (V^-1 B)_ij != 0. The system is
controllable iff EVERY mode has at least one nonzero input coupling.

Left  : a controllable system -- every modal input gain beta_i is nonzero, so the
        single actuator (an underactuated system!) still reaches every mode.
Right : an uncontrollable system -- one mode has beta = 0; that coordinate drifts
        on its own, forever untouchable, and if it is unstable the system is lost.

The bars are |beta_i| = |(V^-1 B)_i|, the 'grip' the input has on each mode.
Numbers are illustrative (a controllable 4-state single-input system vs. a
doctored copy with one decoupled mode).
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})
BLUE, ORANGE, GREY, RED, GREEN = "#2b6cb0", "#dd6b20", "#4a5568", "#c53030", "#2f855a"

modes = [r"$r_1$", r"$r_2$", r"$r_3$", r"$r_4$"]
beta_ctrl = np.array([0.82, 1.10, 0.55, 0.93])
beta_unco = np.array([0.82, 0.00, 0.55, 0.93])   # mode 2 decoupled

fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.6))


def panel(ax, beta, title, ok):
    colors = [GREEN if b > 1e-9 else RED for b in beta]
    ax.bar(range(4), beta, color=colors, width=0.6, zorder=3, edgecolor="white")
    for i, b in enumerate(beta):
        if b < 1e-9:
            ax.annotate("input can't\ntouch this mode\n"
                        r"$\beta_2 = 0$",
                        xy=(i, 0.02), xytext=(i, 0.62), ha="center",
                        fontsize=9, color=RED, weight="bold",
                        arrowprops=dict(arrowstyle="-|>", color=RED, lw=1.6))
        else:
            ax.text(i, b + 0.03, f"{b:.2f}", ha="center", fontsize=9, color=GREY)
    ax.set_xticks(range(4))
    ax.set_xticklabels(modes, fontsize=12)
    ax.set_ylim(0, 1.35)
    ax.set_xlabel("eigenmode  (modal coordinate)")
    ax.set_ylabel(r"input grip  $|\beta_i| = |(V^{-1}B)_i|$")
    ax.set_title(title, fontsize=11.5)
    ax.grid(axis="y", alpha=0.25, lw=0.5)
    verdict = ("CONTROLLABLE\nevery mode reachable"
               if ok else "UNCONTROLLABLE\none mode is untouchable")
    ax.text(0.5, 0.90, verdict, transform=ax.transAxes, ha="center",
            fontsize=10.5, weight="bold", color=(GREEN if ok else RED),
            bbox=dict(boxstyle="round,pad=0.35",
                      fc="white", ec=(GREEN if ok else RED), lw=1.5))


panel(axes[0], beta_ctrl, "(a) controllable — e.g. the cart-pole (1 actuator!)", True)
panel(axes[1], beta_unco, "(b) uncontrollable — a decoupled mode", False)

fig.suptitle("Controllability = the input can reach every eigenmode   "
             r"($\dot r_i = \lambda_i r_i + \beta_i u$)",
             fontsize=12.5, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.94])
out = __file__.replace("gen_03b_controllability.py", "03b_controllability.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
