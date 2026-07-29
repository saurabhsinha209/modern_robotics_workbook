"""Figure: partial feedback linearization — you can't linearize everything, but
you can pick WHICH part to linearize.

Chapter 1 said: an underactuated system is NOT feedback-equivalent to qddot = u --
you cannot cancel the whole dynamics. PFL is the consolation prize: linearize a
CHOSEN SUBSET of the coordinates exactly, globally, and reason about the rest.

Reorganise coordinates into passive q1 (l of them) and actuated q2 (m of them):

    M11 q1dd + M12 q2dd = tau1            (passive rows -- no input here)
    M21 q1dd + M22 q2dd = tau2 + u        (actuated rows -- input u lives here)

COLLOCATED PFL   : command u so the ACTUATED joints obey q2dd = v (whatever you
                   ask). Always possible (M22 - M21 M11^-1 M12 is invertible).
                   The passive joints are left as residual dynamics you steer
                   indirectly. This is what swing-up uses.

NON-COLLOCATED   : command u so the PASSIVE joints obey q1dd = v. Only possible
   PFL             when M12 has full rank (rank l) -- 'Strong Inertial Coupling'.
                   For the cart-pole M12 = cos(theta), which DROPS RANK at
                   theta = +-pi/2: horizontal, no cart force can torque the pole,
                   and the controller demands infinite force. Blows up.

The panels contrast which block of coordinates comes out clean, and where
non-collocated dies.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})
BLUE, ORANGE, GREY, RED, GREEN = "#2b6cb0", "#dd6b20", "#4a5568", "#c53030", "#2f855a"

fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.0))

# =============================================== (a) the two choices, schematic
ax = axes[0]
ax.axis("off")
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)

ax.text(5, 9.5, "which coordinates do you make behave like\n"
        r"a trivial double integrator  $\ddot q = v$ ?",
        ha="center", va="top", fontsize=11, weight="bold")

# collocated box
ax.add_patch(plt.Rectangle((0.4, 4.7), 9.2, 3.5, fc="#ebf8ff", ec=BLUE, lw=2))
ax.text(5, 7.85, "COLLOCATED  PFL", ha="center", fontsize=11, weight="bold",
        color=BLUE)
ax.text(5, 7.05, r"linearize the ACTUATED joints:  $\ddot q_2 = v$",
        ha="center", fontsize=10)
ax.text(5, 6.3, "always works — no rank condition", ha="center", fontsize=9.5,
        color=GREEN, weight="bold")
ax.text(5, 5.35, "passive joints left as residual dynamics\n(steer them indirectly"
        " — this is swing-up)", ha="center", va="center", fontsize=8.8, color=GREY)

# non-collocated box
ax.add_patch(plt.Rectangle((0.4, 0.6), 9.2, 3.5, fc="#fff5f5", ec=RED, lw=2))
ax.text(5, 3.75, "NON-COLLOCATED  PFL", ha="center", fontsize=11, weight="bold",
        color=RED)
ax.text(5, 2.95, r"linearize the PASSIVE joints:  $\ddot q_1 = v$",
        ha="center", fontsize=10)
ax.text(5, 2.2, r"only if $M_{12}$ has full rank"
        "  (Strong Inertial Coupling)", ha="center", fontsize=9.5, color=RED,
        weight="bold")
ax.text(5, 1.35, "blows up where the coupling vanishes", ha="center",
        fontsize=8.8, color=GREY)

# =============================================== (b) where non-collocated dies
ax = axes[1]
th = np.linspace(-np.pi, np.pi, 400)
coupling = np.cos(th)          # M12 for the cart-pole
ax.plot(th, coupling, color=BLUE, lw=2.6, zorder=4,
        label=r"coupling  $M_{12} = \cos\theta$")
ax.axhline(0, color=GREY, lw=0.9)

# mark the rank-drop points
for tc in (-np.pi / 2, np.pi / 2):
    ax.plot(tc, 0, "o", ms=11, mfc=RED, mec="white", mew=1.5, zorder=6)
ax.axvspan(np.pi / 2 - 0.14, np.pi / 2 + 0.14, color=RED, alpha=0.12)
ax.axvspan(-np.pi / 2 - 0.14, -np.pi / 2 + 0.14, color=RED, alpha=0.12)
ax.annotate("pole horizontal:\n$\\cos\\theta=0$, coupling gone,\n"
            "non-collocated demands\n$\\infty$ force",
            xy=(np.pi / 2, 0), xytext=(np.pi / 2 - 0.15, -0.72),
            ha="center", fontsize=8.8, color=RED, weight="bold",
            arrowprops=dict(arrowstyle="-|>", color=RED, lw=1.6))

# region where collocated is happy (everywhere)
ax.text(0, 0.72, "collocated PFL: fine everywhere\n"
        "(avoids inverting $\\cos\\theta$)", ha="center", fontsize=9,
        color=GREEN, weight="bold")

ax.set_xlim(-np.pi, np.pi)
ax.set_ylim(-1.15, 1.15)
ax.set_xticks([-np.pi, -np.pi / 2, 0, np.pi / 2, np.pi])
ax.set_xticklabels([r"$-\pi$", r"$-\pi/2$", "0", r"$\pi/2$", r"$\pi$"])
ax.set_xlabel(r"pole angle  $\theta$")
ax.set_ylabel(r"inertial coupling  $M_{12}$")
ax.set_title("(b) cart-pole: non-collocated PFL fails when the pole is horizontal",
             fontsize=11)
ax.legend(loc="lower center", fontsize=9, framealpha=0.9)
ax.grid(alpha=0.25, lw=0.5)

fig.suptitle("Partial feedback linearization: linearize a CHOSEN subset of coordinates, exactly",
             fontsize=13, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.94])
out = __file__.replace("gen_03c_pfl.py", "03c_pfl.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
