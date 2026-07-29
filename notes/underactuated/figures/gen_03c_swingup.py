"""Figure: full cart-pole swing-up = collocated PFL + energy shaping + LQR catch.

Everything from Chapter 3, one trajectory. Constants = 1, cart-pole:

    2 xddot + thetaddot cos th - thetadot^2 sin th = f_x        (18)
    xddot cos th + thetaddot + sin th = 0                       (19)

COLLOCATED PFL (3.5.1): choose f_x so the CART obeys a commanded accel xddot = u:
    f_x = (2 - c^2) u - s c - thetadot^2 s
    => the pole reduces to a decoupled 'pendulum with modulated torque':
       thetaddot = -u cos th - sin th                            (46-47)

ENERGY SHAPING (3.6.2): with pendulum energy E = 0.5 thetadot^2 - cos th and
Ed = 1 (the homoclinic / upright energy), Etilde = E - Ed,
    Edot_tilde = -u thetadot cos th,
so commanding
    u = kE * thetadot * cos th * Etilde   drives Etilde -> 0 (pump/brake energy)
plus a PD term on the cart to keep it near the origin:
    u = kE thetadot cos th Etilde - kp x - kd xdot.

LQR CATCH (3b): once the full state enters the basin near [0, pi, 0, 0], switch to
the linear balancing controller u = -K xbar.
"""

import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"font.size": 10})
BLUE, ORANGE, GREY, RED, GREEN = "#2b6cb0", "#dd6b20", "#4a5568", "#c53030", "#2f855a"

XSTAR = np.array([0.0, np.pi, 0.0, 0.0])

# --- LQR gain for the cart-pole linearization (from 3b), via Riccati iteration
A = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, 1, 0, 0], [0, 2, 0, 0]], float)
B = np.array([[0], [0], [1], [1]], float)


def lqr_gain(Q, R, iters=400000, dt=1e-4):
    P = np.eye(4)
    Ri = np.linalg.inv(R)
    for _ in range(iters):
        P = P + dt * (A.T @ P + P @ A - P @ B @ Ri @ B.T @ P + Q)
    return Ri @ B.T @ P


K = lqr_gain(np.diag([1.0, 20.0, 1.0, 2.0]), np.array([[1.0]]))

# --- energy-shaping gains
kE, kp, kd = 0.8, 0.10, 0.30


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def energy(th, thd):
    return 0.5 * thd ** 2 - np.cos(th)


def force_from_cart_accel(u, th, thd):
    """Collocated PFL: solve (18) for f_x that yields cart accel = u."""
    c, s = np.cos(th), np.sin(th)
    return (2 - c ** 2) * u - s * c - thd ** 2 * s


def controller(state):
    x, th, xd, thd = state
    near = abs(wrap(th - np.pi)) < 0.5 and abs(thd) < 2.0
    if near:
        u_force = float(-K @ (np.array([x, th, xd, thd]) - XSTAR))
        return u_force, True
    # energy shaping (commanded cart accel) + cart PD, then map to force via PFL
    Et = energy(th, thd) - 1.0
    u_cmd = kE * thd * np.cos(th) * Et - kp * x - kd * xd
    return force_from_cart_accel(u_cmd, th, thd), False


def cartpole_f(s, fx):
    x, th, xd, thd = s
    c, sn = np.cos(th), np.sin(th)
    den = 2 - c ** 2
    xdd = (fx + sn * c + thd ** 2 * sn) / den          # from (18),(19), const=1
    thdd = -(xdd * c + sn)
    return np.array([xd, thd, xdd, thdd])


# --- simulate from hanging down
s = np.array([0.0, 0.05, 0.0, 0.0])
T, dt = 24.0, 0.002
S, U, MODE, TS = [s.copy()], [], [], [0.0]
for i in range(int(T / dt)):
    fx, is_lqr = controller(s)
    k1 = cartpole_f(s, fx)
    k2 = cartpole_f(s + 0.5 * dt * k1, fx)
    k3 = cartpole_f(s + 0.5 * dt * k2, fx)
    k4 = cartpole_f(s + dt * k3, fx)
    s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    S.append(s.copy())
    U.append(fx)
    MODE.append(is_lqr)
    TS.append((i + 1) * dt)
S = np.array(S)
TS = np.array(TS)
MODE = np.array(MODE + [MODE[-1]])
ES = energy(S[:, 1], S[:, 3])
tswitch = TS[np.argmax(MODE)] if MODE.any() else None

fig = plt.figure(figsize=(14, 5.2))
gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1], hspace=0.42, wspace=0.22)

# ---------------- (a) pendulum phase portrait ----------------
ax = fig.add_subplot(gs[:, 0])
TH_LIM, TD_LIM = 1.2 * np.pi, 3.2
TH, TD = np.meshgrid(np.linspace(-TH_LIM, TH_LIM, 400),
                     np.linspace(-TD_LIM, TD_LIM, 400))
ax.contour(TH, TD, energy(TH, TD), levels=np.linspace(-1, 5, 13),
           colors="#cbd5e0", linewidths=0.6, zorder=1, linestyles="solid")
ax.contour(TH, TD, energy(TH, TD), levels=[1.0], colors=[RED], linewidths=2.4,
           zorder=3)

th_plot = wrap(S[:, 1])
# break the line where wrap jumps, and colour by mode
seg = np.where(np.abs(np.diff(th_plot)) > 1.0)[0]
start = 0
for end in list(seg) + [len(th_plot) - 1]:
    idx = slice(start, end + 1)
    ax.plot(th_plot[idx], S[idx, 3], color=BLUE, lw=1.3, alpha=0.85, zorder=4)
    start = end + 1
# mark switch point
isw = np.argmax(MODE)
ax.plot(wrap(S[isw, 1]), S[isw, 3], "o", color=GREEN, ms=10, zorder=7,
        mec="white", mew=1.5)
ax.annotate("switch to LQR\n(enter the basin)", xy=(wrap(S[isw, 1]), S[isw, 3]),
            xytext=(-2.95, -1.9), fontsize=9, color=GREEN, weight="bold",
            ha="center",
            arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=1.6))
ax.plot(0, 0, "o", ms=8, mfc=GREY, mec=GREY, zorder=6)
for xf in (np.pi, -np.pi):
    ax.plot(xf, 0, "o", ms=10, mfc="white", mec=GREY, mew=2.0, zorder=6)
ax.text(0, 2.3, "homoclinic orbit  $E=mgl$", ha="center", color=RED,
        fontsize=9.5, weight="bold", zorder=8,
        bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.9))
ax.set_xlim(-TH_LIM, TH_LIM)
ax.set_ylim(-TD_LIM, TD_LIM)
ax.set_xticks([-np.pi, -np.pi / 2, 0, np.pi / 2, np.pi])
ax.set_xticklabels([r"$-\pi$", r"$-\pi/2$", "0", r"$\pi/2$", r"$\pi$"])
ax.set_xlabel(r"pole angle  $\theta$  (wrapped)")
ax.set_ylabel(r"$\dot\theta$")
ax.set_title("(a) pendulum subsystem: pump onto the homoclinic orbit, then LQR",
             fontsize=11)
ax.grid(alpha=0.2, lw=0.5)

# ---------------- (b) energy ----------------
ax = fig.add_subplot(gs[0, 1])
ax.plot(TS, ES, color=BLUE, lw=1.6)
ax.axhline(1.0, color=RED, lw=1.8, ls="--")
ax.text(TS[-1] * 0.98, 1.08, "$E_d = mgl$", ha="right", color=RED, fontsize=9.5,
        weight="bold")
if tswitch:
    ax.axvline(tswitch, color=GREEN, lw=1.4, ls=":")
ax.set_xlim(0, TS[-1])
ax.set_ylabel("pendulum energy $E$")
ax.set_title("(b) energy pumps up to the homoclinic level", fontsize=11)
ax.grid(alpha=0.25, lw=0.5)

# ---------------- (c) cart position ----------------
ax = fig.add_subplot(gs[1, 1])
ax.plot(TS, S[:, 0], color=ORANGE, lw=1.5)
ax.axhline(0, color=GREY, lw=0.9)
if tswitch:
    ax.axvline(tswitch, color=GREEN, lw=1.4, ls=":")
    ax.text(tswitch, ax.get_ylim()[1] * 0.7, " LQR", color=GREEN, fontsize=9,
            weight="bold")
ax.set_xlim(0, TS[-1])
ax.set_xlabel("time  $t$")
ax.set_ylabel("cart position $x$")
ax.set_title("(c) cart stays bounded in swing-up, then LQR recenters it",
             fontsize=11)
ax.grid(alpha=0.25, lw=0.5)

fig.suptitle("Cart-pole swing-up:  collocated PFL  +  energy shaping  →  LQR catch",
             fontsize=13, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.94])
out = __file__.replace("gen_03c_swingup.py", "03c_swingup.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("wrote", out)
print(f"switch at t={tswitch:.2f}s; final state = {S[-1].round(3)}  "
      f"(target {XSTAR}); final |theta-pi|={abs(wrap(S[-1,1]-np.pi)):.4f}")
