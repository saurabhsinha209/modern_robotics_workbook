"""Figure: the RL family tree, rooted at Ch 7's value iteration / Bellman backup.

Two lineages branch from tabular value iteration:
  VALUE line  : value iteration -> Q-learning -> DQN
  POLICY line : REINFORCE -> actor-critic -> PPO / SAC
They rejoin at actor-critic (the critic IS the Ch 7 value function, coaching the
actor). Each edge is annotated with the ONE thing that step adds. The whole point:
every box is the same Bellman backup with an organ swapped (mesh -> net, model ->
samples).
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

BLUE, ORANGE, GREEN, GREY, RED, INK = (
    "#2b6cb0", "#dd6b20", "#2f855a", "#4a5568", "#c53030", "#1a202c")

fig, ax = plt.subplots(figsize=(12.5, 7.4))
ax.set_xlim(0, 100); ax.set_ylim(0, 100); ax.axis("off")


def box(x, y, w, h, title, sub, color, tcol="white"):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                 boxstyle="round,pad=0.6,rounding_size=2.2",
                 fc=color, ec="white", lw=1.5, zorder=3))
    ax.text(x, y + h * 0.16, title, ha="center", va="center",
            fontsize=11, weight="bold", color=tcol, zorder=4)
    ax.text(x, y - h * 0.24, sub, ha="center", va="center",
            fontsize=7.6, color=tcol, zorder=4)


def arrow(x0, y0, x1, y1, label, lx=None, ly=None, col=GREY):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>",
                 mutation_scale=18, lw=2, color=col, zorder=2,
                 shrinkA=6, shrinkB=6))
    if label:
        ax.text(lx if lx is not None else (x0 + x1) / 2,
                ly if ly is not None else (y0 + y1) / 2,
                label, ha="center", va="center", fontsize=7.8,
                style="italic", color=col,
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none",
                          alpha=0.9), zorder=5)


# ---- root
box(50, 90, 34, 12, "VALUE ITERATION  (Ch 7)",
    "tabular · needs model $f$ · Bellman backup", INK)
ax.text(50, 98.7, "the Bellman backup:   value(here) ← reward + value(next state)",
        ha="center", fontsize=9.5, weight="bold", color=INK)

# ---- value (left) lineage
box(22, 66, 30, 12, "Q-LEARNING",
    "learn $Q(x,u)$ → act by $\\arg\\max_u$\n(model-FREE, off-policy)", BLUE)
box(22, 42, 30, 12, "DQN  (2015)",
    "$Q$ = neural net + replay\n+ target net · discrete $u$", BLUE)

arrow(43, 84, 27, 72, "drop the model:\nsample $Q$, no $f$ needed", 25, 79, BLUE)
arrow(22, 60, 22, 48, "table → deep net\n(+ stabilizers)", 22, 54, BLUE)

# ---- policy (right) lineage
box(78, 66, 30, 12, "REINFORCE  (book stops)",
    "parameterize $\\pi_\\theta$ directly\ncontinuous $u$ · high variance", ORANGE)
arrow(57, 84, 73, 72, "skip the value fn:\nascend expected return", 76, 79, ORANGE)

# ---- rejoin: actor-critic
box(66, 42, 32, 12, "ACTOR–CRITIC",
    "actor $\\pi_\\theta$ + critic $V/Q$\ncritic = Ch 7 value fn, coaches actor", GREEN)
arrow(78, 60, 70, 48, "add a critic to\ncut the variance", 84, 54, GREEN)
# value line feeds the critic idea (horizontal cross-link, no overlap)
arrow(37, 42, 50, 42, "critic = a\nvalue fn", 43.5, 46, GREEN)

# ---- workhorses
box(52, 14, 30, 13, "PPO   (on-policy)",
    "clipped step · robust\nSIM locomotion (Isaac ×1000s)", RED)
box(84, 14, 30, 13, "SAC   (off-policy)",
    "replay + max-entropy · efficient\nREAL-robot / sample-limited", RED)
arrow(60, 36, 54, 21, "", col=GREEN)
arrow(72, 36, 82, 21, "", col=GREEN)

ax.text(50, 3.5, "every box = the same Bellman backup, one organ swapped: "
        "mesh → net (“deep”),  model → samples (“reinforcement”)",
        ha="center", fontsize=9, style="italic", color=INK)

ax.text(6, 54, "VALUE\nlineage", ha="center", fontsize=9.5, weight="bold",
        color=BLUE, alpha=0.85)
ax.text(96, 78, "POLICY\nlineage", ha="center", fontsize=9.5, weight="bold",
        color=ORANGE, alpha=0.85)

fig.tight_layout()
out = __file__.replace("gen_07b_rl_family_tree.py", "07b_rl_family_tree.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
