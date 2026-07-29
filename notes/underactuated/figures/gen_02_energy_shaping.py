"""Figure: energy-shaping swing-up of the torque-limited pendulum.

The pendulum is FULLY actuated until you add a torque limit. With |u| <= u_max
and u_max < mgl, torque alone cannot hold the pendulum horizontal, let alone
drive it to the top -- so no feedback-cancellation controller exists. The system
is underactuated by Chapter 1's definition (input constraint), and we must
instead PUMP ENERGY over several swings.

The trick (Tedrake 2.3.1): don't control position, control *energy*.

    E    = 0.5*m*l^2*thetadot^2 - m*g*l*cos(theta)
    Edot = u * thetadot                    <- torque is the ONLY way energy enters

Take the desired energy to be that of the homoclinic orbit, E_d = mgl -- the
orbit that asymptotically reaches the upright. With Etilde = E - E_d and

    u = -k * thetadot * Etilde

the error dynamics collapse to

    Etildedot = -k * thetadot^2 * Etilde

which drives Etilde -> 0 exponentially for any thetadot != 0. Push in the
direction of motion when short on energy, brake when carrying too much.

Panels:
  (a) phase portrait with the swing-up trajectory converging onto the homoclinic
      orbit (red). Note it takes several passes -- that is the "pumping".
  (b) energy vs time converging to E_d = mgl.
  (c) the commanded torque, showing it saturating against the limit.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})

BLUE, ORANGE, GREY, RED = "#2b6cb0", "#dd6b20", "#4a5568", "#c53030"

# m = l = g = 1  ->  E_d = mgl = 1, and the torque needed to hold horizontal is mgl = 1
M = L = G = 1.0
E_D = M * G * L
U_MAX = 0.28          # well under mgl -> cannot muscle it up, must pump
K = 0.6               # energy-shaping gain
DT, N = 0.005, 12000


def energy(th, td):
    return 0.5 * M * L**2 * td**2 - M * G * L * np.cos(th)


def control(th, td):
    """u = -k*thetadot*(E - E_d), saturated at the torque limit."""
    return np.clip(-K * td * (energy(th, td) - E_D), -U_MAX, U_MAX)


def step(th, td, dt):
    def rhs(th, td):
        u = control(th, td)
        return td, (u - M * G * L * np.sin(th)) / (M * L**2)

    k1 = rhs(th, td)
    k2 = rhs(th + 0.5 * dt * k1[0], td + 0.5 * dt * k1[1])
    k3 = rhs(th + 0.5 * dt * k2[0], td + 0.5 * dt * k2[1])
    k4 = rhs(th + dt * k3[0], td + dt * k3[1])
    return (th + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
            td + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]))


# start hanging almost straight down, with a tiny nudge (at exactly thetadot = 0
# and theta = 0 the controller does nothing -- see the note in the text)
th, td = 0.15, 0.0
THS, TDS, US, TS = [th], [td], [control(th, td)], [0.0]
for i in range(N):
    th, td = step(th, td, DT)
    THS.append(th)
    TDS.append(td)
    US.append(control(th, td))
    TS.append((i + 1) * DT)
THS, TDS, US, TS = map(np.array, (THS, TDS, US, TS))
ES = energy(THS, TDS)

fig = plt.figure(figsize=(13.5, 5.6))
gs = fig.add_gridspec(2, 2, width_ratios=[1.35, 1], hspace=0.42, wspace=0.24)

# ---------------- (a) phase portrait + trajectory ----------------
ax = fig.add_subplot(gs[:, 0])
TH_LIM, TD_LIM = 1.15 * np.pi, 2.6
TH, TD = np.meshgrid(np.linspace(-TH_LIM, TH_LIM, 600),
                     np.linspace(-TD_LIM, TD_LIM, 600))
ax.contour(TH, TD, energy(TH, TD), levels=np.linspace(-1, 4, 13),
           colors="#4a5568", linewidths=0.6, alpha=0.4, zorder=1,
           linestyles="solid")
ax.contour(TH, TD, energy(TH, TD), levels=[E_D], colors=[RED],
           linewidths=2.6, zorder=4)

ax.plot(THS, TDS, color=BLUE, lw=1.4, alpha=0.9, zorder=5)
ax.plot(THS[0], TDS[0], "o", color=BLUE, ms=7, zorder=6)
ax.annotate("start\n(hanging down)", xy=(THS[0], TDS[0]), xytext=(0.55, -1.55),
            fontsize=9, color=BLUE,
            arrowprops=dict(arrowstyle="->", color=BLUE, lw=1.2))

for th_fp, filled in [(0.0, True), (np.pi, False), (-np.pi, False)]:
    ax.plot(th_fp, 0, "o", ms=10, zorder=8,
            mfc=(GREY if filled else "white"), mec=GREY, mew=2.0)
ax.annotate("goal: upright", xy=(np.pi, 0), xytext=(1.75, 1.75),
            fontsize=9, color=GREY,
            arrowprops=dict(arrowstyle="->", color=GREY, lw=1.2))
ax.text(0, 2.32, "homoclinic orbit  $E = mgl$", ha="center", color=RED,
        fontsize=9.5, weight="bold", zorder=9,
        bbox=dict(boxstyle="round,pad=0.22", fc="white", ec="none", alpha=0.9))

ax.set_xlim(-TH_LIM, TH_LIM)
ax.set_ylim(-TD_LIM, TD_LIM)
ax.set_xticks([-np.pi, -np.pi / 2, 0, np.pi / 2, np.pi])
ax.set_xticklabels([r"$-\pi$", r"$-\pi/2$", "0", r"$\pi/2$", r"$\pi$"])
ax.set_xlabel(r"$\theta$")
ax.set_ylabel(r"$\dot{\theta}$")
ax.set_title(f"(a) swing-up trajectory,  $u_{{max}} = {U_MAX}$  "
             f"$< mgl = {E_D:.0f}$", fontsize=11)
ax.grid(alpha=0.2, lw=0.5)

# ---------------- (b) energy ----------------
ax = fig.add_subplot(gs[0, 1])
ax.plot(TS, ES, color=BLUE, lw=1.6)
ax.axhline(E_D, color=RED, lw=2.0, ls="--")
ax.text(TS[-1] * 0.985, E_D + 0.1, "$E_d = mgl$", ha="right", color=RED,
        fontsize=9.5, weight="bold")
ax.set_xlim(0, TS[-1])
ax.set_ylabel("energy  $E$")
ax.set_title("(b) energy pumps up to the homoclinic level", fontsize=11)
ax.grid(alpha=0.25, lw=0.5)

# ---------------- (c) torque ----------------
ax = fig.add_subplot(gs[1, 1])
ax.plot(TS, US, color=ORANGE, lw=1.2)
for s in (+U_MAX, -U_MAX):
    ax.axhline(s, color=GREY, lw=1.2, ls=":")
ax.text(TS[-1] * 0.985, U_MAX * 1.12, "$\\pm u_{max}$", ha="right",
        color=GREY, fontsize=9)
ax.set_xlim(0, TS[-1])
ax.set_ylim(-U_MAX * 1.55, U_MAX * 1.55)
ax.set_xlabel("time  $t$")
ax.set_ylabel("torque  $u$")
ax.set_title("(c) torque saturates — it cannot brute-force the lift",
             fontsize=11)
ax.grid(alpha=0.25, lw=0.5)

fig.suptitle("Energy-shaping swing-up:  $u = -k\\,\\dot{\\theta}\\,\\tilde{E}$   "
             "— control the energy, not the position",
             fontsize=12.5, y=0.99)
fig.tight_layout(rect=[0, 0, 1, 0.945])
out = __file__.replace("gen_02_energy_shaping.py", "02_energy_shaping.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
print(f"final energy {ES[-1]:.4f}  (target {E_D})   max|theta| reached {np.abs(THS).max():.3f}")
