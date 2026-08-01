"""Figure: direct collocation -- the cubic-spline "sweet spot".

Scalar system  xdot = f(x,u) = u - x, one spline segment [t_k, t_k+h], h = 1.

Decision variables are ONLY the break-point values x[k], x[k+1], u[k], u[k+1].
That is enough to pin down the whole cubic, because the *dynamics* supply the
two slopes:  xdot[k] = f(x[k],u[k]),  xdot[k+1] = f(x[k+1],u[k+1]).
So: 4 numbers -> a unique cubic Hermite segment.

The collocation constraint is imposed at the MIDPOINT t_c:
     xdot(t_c)  ==  f( x(t_c), u(t_c) )
i.e. "the slope the spline actually has" must equal "the slope physics demands."

(a) a candidate iterate: the two slopes disagree -> collocation defect.
(b) after the solver adjusts x[k+1]: they agree -> the segment is (3rd-order)
    accurate integration of the ODE, achieved by CONSTRAINT rather than by
    marching a simulator forward.
"""
import numpy as np
import matplotlib.pyplot as plt

BLUE, ORANGE, GREEN, GREY, RED, INK = (
    "#2b6cb0", "#dd6b20", "#2f855a", "#4a5568", "#c53030", "#1a202c")

h = 1.0
x0, u0, u1 = 0.2, 1.0, 1.2


def f(x, u):
    return u - x


def hermite(x0, xd0, x1, xd1, s):
    """cubic through (0,x0) slope xd0 and (h,x1) slope xd1;  s in [0,1]."""
    h00 = 2 * s**3 - 3 * s**2 + 1
    h10 = s**3 - 2 * s**2 + s
    h01 = -2 * s**3 + 3 * s**2
    h11 = s**3 - s**2
    return h00 * x0 + h10 * h * xd0 + h01 * x1 + h11 * h * xd1


def midpoint_quantities(x1):
    xd0, xd1 = f(x0, u0), f(x1, u1)
    uc = 0.5 * (u0 + u1)
    xc = 0.5 * (x0 + x1) + h / 8.0 * (xd0 - xd1)
    xdc_spline = -1.5 / h * (x0 - x1) - 0.25 * (xd0 + xd1)
    return xd0, xd1, uc, xc, xdc_spline, f(xc, uc)


def defect(x1):
    *_, xdc_spline, xdc_dyn = midpoint_quantities(x1)
    return xdc_spline - xdc_dyn


# bisection for the x1 that zeroes the collocation defect (no scipy in the venv)
lo, hi = 0.2, 3.0
for _ in range(200):
    mid = 0.5 * (lo + hi)
    if defect(lo) * defect(mid) <= 0:
        hi = mid
    else:
        lo = mid
x1_star = 0.5 * (lo + hi)
print("candidate x[k+1]=0.90 -> defect =", round(defect(0.90), 4))
print("solved    x[k+1]=", round(x1_star, 4), "-> defect =", round(defect(x1_star), 8))

fig, axes = plt.subplots(1, 2, figsize=(13.0, 5.0), sharey=True)
ts = np.linspace(0, h, 300)

for ax, x1, title, ok in [
        (axes[0], 0.90, "(a) candidate iterate:  collocation defect $\\neq 0$", False),
        (axes[1], x1_star, "(b) constraint satisfied:  defect $= 0$", True)]:

    xd0, xd1, uc, xc, xdc_spline, xdc_dyn = midpoint_quantities(x1)
    curve = hermite(x0, xd0, x1, xd1, ts / h)
    ax.plot(ts, curve, "-", color=BLUE, lw=2.6, zorder=3,
            label="cubic $x(t)$ (from the 4 break-point numbers)")

    # break points + their dynamics-given slopes
    for tb, xb, xdb in [(0.0, x0, xd0), (h, x1, xd1)]:
        ax.plot(tb, xb, "o", color=BLUE, ms=10, mfc="white", mew=2.2, zorder=5)
        d = 0.18
        ax.annotate("", xy=(tb + d, xb + d * xdb), xytext=(tb - d, xb - d * xdb),
                    arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=2.0))
    ax.plot([], [], "-", color=GREEN, lw=2.0,
            label="slope $\\dot x = f(x[k],u[k])$ at break points")

    # collocation point: spline slope vs dynamics slope
    ax.plot(h / 2, xc, "s", color=ORANGE, ms=9, zorder=6,
            label="collocation point $t_c$ (midpoint)")
    d = 0.30
    ax.annotate("", xy=(h / 2 + d, xc + d * xdc_spline),
                xytext=(h / 2 - d, xc - d * xdc_spline),
                arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=2.4))
    ax.annotate("", xy=(h / 2 + d, xc + d * xdc_dyn),
                xytext=(h / 2 - d, xc - d * xdc_dyn),
                arrowprops=dict(arrowstyle="-|>", color=RED, lw=2.4,
                                linestyle="--" if not ok else "-"))

    gap = xdc_spline - xdc_dyn
    col = RED if not ok else GREEN
    ax.annotate(f"spline slope   {xdc_spline:.3f}\n"
                f"physics slope  {xdc_dyn:.3f}\n"
                f"defect         {gap:+.3f}",
                xy=(0.03, 0.72), xycoords="axes fraction", fontsize=10,
                family="monospace", color=INK,
                bbox=dict(boxstyle="round,pad=0.4",
                          fc="#fff5f5" if not ok else "#f0fff4", ec=col, lw=1.0))

    ax.set_title(title, fontsize=12, color=INK)
    ax.set_xlabel("time within the segment")
    ax.set_xlim(-0.30, 1.30)
    ax.set_ylim(0.0, 1.02)
    ax.set_xticks([0, h / 2, h])
    ax.set_xticklabels(["$t_k$", "$t_{c,k}$", "$t_{k+1}$"])
    ax.grid(alpha=0.25)

axes[0].set_ylabel("state  x")
axes[0].plot([], [], "-", color=BLUE, lw=2.4, label="$\\dot x(t_c)$ from the spline")
axes[0].plot([], [], "--", color=RED, lw=2.4, label="$f(x(t_c),u(t_c))$ from physics")
axes[0].legend(loc="lower right", fontsize=8.5, framealpha=0.95)

fig.suptitle("Direct collocation: integrate the ODE by SATISFYING A CONSTRAINT, "
             "not by marching a simulator", fontsize=13, color=INK)
fig.tight_layout(rect=(0, 0, 1, 0.93))
fig.savefig("10a_collocation.png", dpi=150)
print("wrote 10a_collocation.png")
