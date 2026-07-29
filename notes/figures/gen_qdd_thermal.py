"""Figures for notes/proj_qdd_joint_fundamentals.md

Two panels:
  (a) P = (tau/Km)^2  -- heat is quadratic in torque, independent of Kv
  (b) I = tau/Kt      -- current (and therefore your driver) is set by Kv

Run:  .venv/bin/python notes/figures/gen_qdd_thermal.py
"""
import numpy as np
import matplotlib.pyplot as plt

OUT = "notes/figures/qdd_thermal_and_current.png"

# --- elbow joint case constants (see the note for where these come from) ---
KM = 0.08          # Nm/sqrt(W), motor constant of a 5010-class outrunner (ESTIMATE)
N = 6              # belt reduction
ETA = 0.95         # belt efficiency
RTH_BARE = 5.0     # degC/W, bare motor on a printed mount, still air (ESTIMATE)
RTH_ALU = 2.5      # degC/W, aluminium mount + light airflow (ESTIMATE)
T_AMB = 25.0

# operating points: (label, joint torque Nm)
POINTS = [
    ("no payload",      0.735),
    ("+300 g payload",  1.700),
    ("payload + spring", 0.965),
]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 5.0))

# ---------------- panel (a): heat vs torque ----------------
tau_j = np.linspace(0, 2.2, 400)          # joint torque
tau_m = tau_j / (N * ETA)                 # motor torque
P = (tau_m / KM) ** 2                     # winding dissipation

# thermal bands, expressed as motor power, for the two mounting cases
def p_for_dT(dT, rth):
    return dT / rth

ax1.axhspan(0, p_for_dT(55, RTH_BARE), color="#d8f0d8", zorder=0)
ax1.axhspan(p_for_dT(55, RTH_BARE), p_for_dT(55, RTH_ALU), color="#fdf2cc", zorder=0)
ax1.axhspan(p_for_dT(55, RTH_ALU), 40, color="#fadbd8", zorder=0)

ax1.text(2.15, p_for_dT(55, RTH_BARE) / 2, "safe on a printed mount",
         fontsize=8.5, va="center", ha="right", color="#2d6a2d")
ax1.text(2.15, (p_for_dT(55, RTH_BARE) + p_for_dT(55, RTH_ALU)) / 2,
         "needs aluminium mount / airflow", fontsize=8.5, va="center",
         ha="right", color="#8a6d1f")
ax1.text(2.15, 30, "magnets cook (>80 °C)  —  duty-cycle only",
         fontsize=8.5, va="center", ha="right", color="#a03028")

ax1.plot(tau_j, P, color="#1f3a93", lw=2.4, zorder=3)

# (dx, dy, ha) chosen per point so the callouts don't collide
OFFSETS = [(-26, 72, "right"), (34, 58, "left"), (-16, 34, "right")]
for (label, tj), (dx, dy, ha) in zip(POINTS, OFFSETS):
    p = ((tj / (N * ETA)) / KM) ** 2
    ax1.plot([tj], [p], "o", ms=8, color="#1f3a93", zorder=4,
             mec="white", mew=1.4)
    ax1.annotate(f"{label}\n{tj:.2f} N·m → {p:.1f} W",
                 xy=(tj, p), xytext=(dx, dy), textcoords="offset points",
                 fontsize=8.5, ha=ha, zorder=5,
                 arrowprops=dict(arrowstyle="-", color="#1f3a93", lw=1.0),
                 bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#1f3a93", alpha=0.95))

ax1.set_xlim(0, 2.2)
ax1.set_ylim(0, 40)
ax1.set_xlabel("joint torque  τ_joint  [N·m]")
ax1.set_ylabel("winding dissipation  P  [W]")
ax1.set_title("(a)  Heat is quadratic in torque\nP = (τ_motor / K_m)²   —   independent of Kv",
              fontsize=10.5)
ax1.grid(alpha=0.25, zorder=1)

# ---------------- panel (b): current vs Kv ----------------
kv = np.linspace(60, 420, 400)
kt = 8.3 / kv
tau_m_hold = 1.700 / (N * ETA)     # motor torque for the loaded-hold case
I = tau_m_hold / kt

ax2.plot(kv, I, color="#c0392b", lw=2.4, zorder=3)
ax2.axhline(2.5, ls="--", color="#7f8c8d", lw=1.6)
ax2.text(410, 2.9, "DRV8313-class board  (~2.5 A cont.)",
         fontsize=8.5, ha="right", color="#7f8c8d")
ax2.axhline(10.0, ls="--", color="#7f8c8d", lw=1.6)
ax2.text(68, 10.4, "B-G431B-ESC1 w/ heatsink  (~10 A cont.)",
         fontsize=8.5, ha="left", color="#7f8c8d")

for kvp, tag, (dx, dy) in [(100, "gimbal\n100 KV", (18, 34)),
                           (360, "drone\n360 KV", (-20, 30))]:
    ip = tau_m_hold / (8.3 / kvp)
    ax2.plot([kvp], [ip], "o", ms=8, color="#c0392b", zorder=4, mec="white", mew=1.4)
    ax2.annotate(f"{tag}\n{ip:.1f} A", xy=(kvp, ip), xytext=(dx, dy),
                 textcoords="offset points", fontsize=8.5,
                 ha="right" if dx < 0 else "left", zorder=5,
                 arrowprops=dict(arrowstyle="-", color="#c0392b", lw=1.0),
                 bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#c0392b", alpha=0.95))

ax2.set_xlim(60, 420)
ax2.set_ylim(0, 19)
ax2.set_xlabel("motor Kv  [rpm/V]")
ax2.set_ylabel("phase current to hold 1.70 N·m at the joint  [A]")
ax2.set_title("(b)  Kv picks your driver, not your torque\nI = τ_motor / K_t,   K_t ≈ 8.3 / Kv",
              fontsize=10.5)
ax2.grid(alpha=0.25, zorder=1)

fig.suptitle("QDD elbow joint — the two equations that size everything",
             fontsize=12.5, y=0.99)
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(OUT, dpi=150)
print(f"wrote {OUT}")
