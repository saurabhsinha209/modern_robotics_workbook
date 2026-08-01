"""Figure: value iteration on a 4x4 grid -> cost-to-go landscape + greedy policy.

Each move costs 1, one wall is impassable, goal at top-right. Running cost g=1
per step => total cost = number of steps => J*(x) = shortest-path length to goal.

Left panels: J filling in, one ring per sweep, rippling out from the goal
             (the essence of value iteration).
Right panel: the converged cost-to-go landscape (heatmap + numbers) with the
             GREEDY policy arrows -- step to your lowest-J neighbor -- which flow
             downhill and route AROUND the wall, a behavior nobody hand-coded.
"""
import matplotlib.pyplot as plt
import numpy as np

N = 4
GOAL = (3, 3)                      # (row, col), row 0 = bottom
WALLS = {(1, 1), (2, 1)}
INF = np.inf


def neighbors(r, c):
    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        rr, cc = r + dr, c + dc
        if 0 <= rr < N and 0 <= cc < N and (rr, cc) not in WALLS:
            yield rr, cc


def sweep(J):
    Jn = J.copy()
    for r in range(N):
        for c in range(N):
            if (r, c) in WALLS or (r, c) == GOAL:
                continue
            best = min((1 + J[rr, cc] for rr, cc in neighbors(r, c)), default=INF)
            Jn[r, c] = best
    return Jn


J = np.full((N, N), INF)
J[GOAL] = 0.0
history = [J.copy()]
for _ in range(8):
    J = sweep(J)
    history.append(J.copy())
    if np.array_equal(history[-1], history[-2]):   # inf==inf is True; exact converge
        break
Jstar = history[-1]


def draw_grid(ax, Jm, title, arrows=False, cmap_on=True):
    disp = Jm.copy()
    finite = np.isfinite(disp)
    vmax = np.nanmax(disp[finite]) if finite.any() else 1
    # image: row 0 at bottom
    img = np.full((N, N), np.nan)
    for r in range(N):
        for c in range(N):
            if (r, c) in WALLS:
                img[r, c] = np.nan
            elif np.isfinite(Jm[r, c]):
                img[r, c] = Jm[r, c]
    if cmap_on:
        ax.imshow(img, origin="lower", cmap="YlGnBu_r", vmin=0, vmax=vmax,
                  extent=(-.5, N - .5, -.5, N - .5))
    for (wr, wc) in WALLS:
        ax.add_patch(plt.Rectangle((wc - .5, wr - .5), 1, 1, color="#2d3748"))
    for r in range(N):
        for c in range(N):
            if (r, c) in WALLS:
                continue
            v = Jm[r, c]
            txt = "∞" if not np.isfinite(v) else f"{int(round(v))}"
            col = "white" if (np.isfinite(v) and v > 0.55 * vmax) else "#1a202c"
            if (r, c) == GOAL:
                txt = "G"; col = "#c53030"
            ax.text(c, r, txt, ha="center", va="center",
                    fontsize=12, weight="bold", color=col)
    if arrows:
        for r in range(N):
            for c in range(N):
                if (r, c) in WALLS or (r, c) == GOAL or not np.isfinite(Jm[r, c]):
                    continue
                nb = list(neighbors(r, c))
                rr, cc = min(nb, key=lambda p: Jm[p])
                dr, dc = rr - r, cc - c
                oy = -0.28                       # park arrows below the number
                ax.annotate("", xy=(c + 0.22 * dc, r + oy + 0.22 * dr),
                            xytext=(c - 0.22 * dc, r + oy - 0.22 * dr),
                            arrowprops=dict(arrowstyle="-|>", color="#c53030", lw=2.2))
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_xlim(-.5, N - .5); ax.set_ylim(-.5, N - .5)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10.5, weight="bold")


fig = plt.figure(figsize=(12.5, 4.6))
gs = fig.add_gridspec(1, 4, width_ratios=[1, 1, 1, 1.35], wspace=0.28)

# three fill-in snapshots
snaps = [1, 2, 3]
for i, k in enumerate(snaps):
    ax = fig.add_subplot(gs[0, i])
    draw_grid(ax, history[k], f"after sweep {k}", arrows=False)

# final + policy
axf = fig.add_subplot(gs[0, 3])
draw_grid(axf, Jstar, "converged  $J^*$  +  greedy policy", arrows=True)

fig.suptitle("Value iteration: cost-to-go ripples out from the goal, and the greedy "
             "policy routes AROUND the wall — discovered, not hand-coded",
             fontsize=12, y=1.02, weight="bold")
out = __file__.replace("gen_07_value_iteration.py", "07_value_iteration.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out, "| converged in", len(history) - 1, "sweeps")
