# Underactuated Robotics — course overview & plan

The next book/course after *Modern Robotics*: **MIT 6.832 "Underactuated
Robotics"** by **Russ Tedrake**. Free online text + lectures:
<https://underactuated.csail.mit.edu/index.html> (uses **Drake** / `pydrake` for examples).
Local PDF: `UnderactuatedRobotics.pdf` (2024 edition, 20 chapters + 5 appendices).

This note is the launchpad; per-topic notes live alongside it in this folder
(`notes/underactuated/`), each intuition-first with a `figures/` subfolder, same
workflow as the MR notes.

---

## What "underactuated" means — and why it's the natural next step

*Modern Robotics* almost always treated the robot as something you **command
directly**: a fully-actuated, kinematic / quasi-static object. Want the arm at a
pose? Solve IK, send joint targets, a controller cancels the dynamics (computed
torque, Ch 11) and you get there. The dynamics were an *obstacle to cancel*.

An **underactuated** system has **fewer independent actuators than configuration
DOF** — or, more precisely, at some states it **cannot command an arbitrary
acceleration**. You physically *can't* cancel the dynamics; you have to **exploit**
them. A passive-ankle walker, a quadrotor, an acrobot, a robot throwing or catching,
a legged robot mid-stride — none can be driven pose-to-pose. You must plan and
control *through* the natural dynamics.

This is exactly the boundary we hit at the end of MR:
- **Ch 13 quadruped tangent** — a legged base is force/contact-constrained and
  underactuated; the clean `J_e†` gives way to dynamics-aware, constrained control.
- **Ch 11 controls** — computed torque *cancels* dynamics; underactuated systems
  are where that stops being possible, and optimal control takes over.

### What the book actually is

Despite the name, this is **not primarily a legged-locomotion book**. It is a book
on **nonlinear dynamics and computational optimal control**, with walking/flying as
the motivating examples. From the preface: *"the primary focus of these notes is on
computational approaches to control design, especially using optimization and
machine learning."*

| | Modern Robotics | Underactuated |
|---|---|---|
| Robot is | a thing you command pose-to-pose | a thing with dynamics you can't override |
| Dynamics are | an obstacle to cancel (computed torque) | the substrate you exploit |
| Core math | SE(3), Jacobians, linear algebra | optimization, value functions, Riccati / Lyapunov |
| Answer shape | closed-form / IK solve | solve an optimization problem |

**Ch 7–11 are the actual theoretical foundation of RL.** Dynamic programming → HJB →
value iteration → LQR → policy search is the classical spine that "reward, value
function, actor-critic" sits on top of. That block alone justifies the course for the
north star.

---

## Table of contents (real, from the 2024 PDF)

**Model Systems** — 1 Fully-actuated vs Underactuated · 2 The Simple Pendulum ·
3 Acrobots, Cart-Poles, and Quadrotors · 4 Simple Models of Walking and Running ·
5 Highly-articulated Legged Robots · 6 Model Systems with Stochasticity

**Nonlinear Planning and Control** — 7 Dynamic Programming · 8 Linear Quadratic
Regulators · 9 Lyapunov Analysis · 10 Trajectory Optimization · 11 Policy Search ·
12 Motion Planning as Search · 13 Feedback Motion Planning · 14 Robust and Stochastic
Control · 15 Output Feedback (aka Pixels-to-Torques) · 16 Algorithms for Limit Cycles ·
17 Planning and Control through Contact

**Estimation and Learning** — 18 System Identification · 19 State Estimation ·
20 Model-Free Policy Search

**Appendices** — A Drake · B Multi-Body Dynamics · C Optimization and Mathematical
Programming · D An Optimization Playbook · E Miscellaneous

### ⚠️ The text is a working draft — coverage is very uneven

These are *"working notes used for a course being taught at MIT,"* updated per
semester. Measured page counts in the 2024 PDF:

| Substantial | pp | | Thin | pp |
|---|---|---|---|---|
| 9 Lyapunov / SOS | 43 | | 11 Policy Search | 9 |
| 10 Trajectory Optimization | 28 | | 16 Limit Cycles | 9 |
| 4 Walking and Running | 24 | | 17 Contact | 8 |
| 18 System Identification | 24 | | 20 Model-Free Policy Search | 8 |
| 7 Dynamic Programming | 23 | | 12 Motion Planning as Search | 7 |
| **B** Multi-Body Dynamics | 22 | | **15 Pixels-to-Torques** | **6** |
| 3 Acrobot / Cart-Pole / Quadrotor | 21 | | **13 Feedback Motion Planning** | **1** |
| 1 Fully- vs Underactuated | 15 | | **19 State Estimation** | **1** |
| 8 LQR | 15 | | | |

**Ch 13 is a title page with no body. Ch 19 is three section headings with no body.**
Both are excluded from the tiering below — get that material elsewhere.

Several sections are placeholders rather than content: §15.5.4 (teacher-student),
§15.6.1 (visuomotor policies — three sentence fragments and two citations),
§14.7 (domain randomization), plus explicit "Coming soon..." markers in §14.5.3
and §15.5.3.

**Rule of thumb:** the book is deep where Tedrake does research (SOS/Lyapunov,
trajectory optimization, GCS, contact, system ID) and a bibliography where he does
not (perception, imitation learning, modern deep RL). See
`00b_sota_augmentation.md`.

---

## Chapter tiering

> **Weighting decision (2026-07-18):** tiering assumes **roadmap M4 (humanoid
> loco-manipulation) is a real target**, so the legged chapters are pulled up to
> Tier 1. This is a *reading-plan* decision for this book only — `CLAUDE.md` and
> `notes/00_roadmap.md` are unchanged, and the near-term hands-on track is still the
> MuJoCo pick-place line (first MVP shipped; physical-model improvements later).

### Tier 1 — go deep

**The control spine** (durable regardless of embodiment):

- **Ch 1 Fully-actuated vs Underactuated** — short framing chapter. Read properly once;
  the ASIMO-vs-passive-walker and birds-vs-aircraft motivations set up everything.
- **Ch 2 The Simple Pendulum** — *the atom.* Phase portraits, orbits, energy-shaping
  control. Every later system is this one with more dimensions. Small chapter, do it fully.
- **Ch 3 Acrobots, Cart-Poles, Quadrotors** — dense; **split into 3a/3b/3c**:
  (a) the systems + linearizing the manipulator equations, (b) controllability +
  **LQR balancing**, (c) partial feedback linearization + swing-up / energy shaping.
  Heavy on the LA weak spot (eigenvalues, controllability rank, Riccati) → slow pass.
  A humanoid is a cart-pole with more coordinates; this is the honest miniature of balance.
- **Ch 7 Dynamic Programming** — HJB, value iteration, *neural fitted value iteration*.
  This **is** RL, derived from dynamics instead of handed over as a black box.
- **Ch 8 Linear Quadratic Regulators** — the single most reused object in robot control.
  Time-varying LQR along a trajectory is the bridge into trajectory optimization.
- **Ch 10 Trajectory Optimization** — direct transcription/shooting, **direct collocation**,
  **iLQR/DDP**, **MPC**. Same family as cuRobo; also what generates demos and warm-starts
  for imitation learning. The most directly buildable chapter in the book.

**The legged block** (Tier 1 because of M4):

- **Ch 4 Simple Models of Walking and Running** — rimless wheel → compass gait → kneed
  walker → **SLIP**, plus **limit cycles and Poincaré maps**. The durable reframing:
  *a gait is a stable limit cycle, not a trajectory.*
- **Ch 5 Highly-articulated Legged Robots** — **centroidal dynamics**, impact dynamics,
  ZMP, whole-body control, footstep planning. See the caveat below on ZMP.
- **Ch 17 Planning and Control through Contact** — hybrid systems *are* legged robots;
  each footfall is a mode switch. Contact-implicit trajopt is the model-based frontier.
  Also directly relevant to contact-rich manipulation.
- **Appendix B Multi-Body Dynamics** — specifically **floating-base coordinates** and
  **impact/collision dynamics**; the roadmap already flagged this (*"go deeper on Ch 3
  and Ch 8 toward floating-base dynamics"*). Featherstone spatial-vector algebra is the
  natural supplement. Bonus: the contact-solver section (complementarity, Anitescu /
  Todorov convex relaxations, SAP) is **what MuJoCo is doing under us**.

### Tier 2 — concepts yes, machinery no

- **Ch 16 Algorithms for Limit Cycles** — transverse linearization, orbital stabilization.
  How you actually *stabilize* a gait rather than merely find one. Pairs with Ch 4.
- **Ch 14 Robust and Stochastic Control** — read the framing and the stochastic-LQR
  material; skip H∞ / LPV / LEQG. **Note:** §14.7 domain randomization is a heading with
  no body — that content comes from `00b_sota_augmentation.md`.
- **Ch 9 Lyapunov Analysis** — take stability, LaSalle, and region-of-attraction
  *intuitively*. **Skip essentially all the sums-of-squares / SDP machinery** — it is
  roughly half the chapter (43pp, the longest in the book) and it is Tedrake's research
  program, not our path.
- **Ch 11 Policy Search** — short, and the honest theoretical answer to "why does policy
  gradient work" (LQR as a nonconvex-but-benign policy-optimization problem).
- **Ch 6 Model Systems with Stochasticity** — its worked example is the rimless wheel on
  rough terrain, so it doubles as Ch 4 practice.
- **Ch 12 Motion Planning as Search** — RRT/RRT* already covered in MR Ch 10; read here
  for **RRT with dynamics** and **GCS (graphs of convex sets)**, which is genuinely new
  and current (2023-era).
- **Ch 18 System Identification** — skim overall, but **§18.2.5 "residual physics models"**
  and "learning models for control" are substantive and directly serve the pick-place
  "improve with physical models" thread.

### Tier 3 — read the framing only, then leave the book

- **Ch 15 Output Feedback ("Pixels-to-Torques")** — *demoted.* §15.1 is an excellent
  3-page framing of why feedback-from-pixels is hard, and contains Tedrake's candid
  admission that model-based control currently has little to offer here. But §15.6.1
  "Visuomotor policies" is three sentence fragments plus citations to Diffusion Policy
  and ACT. **Read §15.1, then go to the papers** — see `00b_sota_augmentation.md`.
- **Ch 20 Model-Free Policy Search** — REINFORCE / weight perturbation only. Correct but
  ~2010-era; no PPO / SAC / TRPO / DQN anywhere in the book. Read for grounding in *why*
  policy gradients work, learn modern RL elsewhere.
- **Appendix C Optimization / Appendix D Optimization Playbook** — consult as needed when
  Ch 8/10 lean on convexity, LP/QP/SOCP/SDP. Not a front-to-back read.
- **Appendix A Drake** — setup, read when installing.

### Excluded — the chapter is empty

- **Ch 13 Feedback Motion Planning** (1 page, title only) — funnels / LQR-trees. Get it
  from Tedrake's papers (Majumdar & Tedrake, "Funnel Libraries for Real-Time Robust
  Feedback Motion Planning") if wanted; Ch 16 covers the adjacent orbital-stabilization
  idea with actual content.
- **Ch 19 State Estimation** (1 page, headings only) — Kalman / recursive Bayes /
  smoothing. Source this from *Probabilistic Robotics* (Thrun/Burgard/Fox) or the
  existing SLAM-app thread (`notes/modern_robotics/13d_sota_slam_navigation.md`).

---

## Caveat: Ch 5's ZMP material is largely historical

ZMP is the ASIMO-era stack — Tedrake opens Ch 1 by contrasting exactly that against
passive dynamic walkers. Modern humanoid locomotion is overwhelmingly **massively-parallel
RL in Isaac Lab + sim-to-real** (Unitree / Booster controllers, GR00T, Helix), with
**MPC on centroidal dynamics** as the model-based alternative.

So: learn Ch 5 for **vocabulary and the centroidal-dynamics idea** (which *is* still
current), and treat ZMP itself as context rather than target. The chapters that actually
feed modern humanoid RL are **7, 8, 10, 11, 14**.

Two practical consequences if M4 is pursued for real:
- **Isaac Lab becomes more important than MuJoCo** — parallel envs are non-negotiable
  for locomotion RL.
- **Hardware is a much larger jump** — a fall is destructive in a way a wheeled base's
  isn't.

---

## Reading order

The book's own order is motivated by walking. Given the tiering above:

```
1 → 2 → 3a/3b/3c → 4 → 7 → 8 → 10 → 16/17 → 5 → back-fill Tier 2 → augmentation notes
```

Rationale: **Ch 4 early** (right after the cart-pole) because limit cycles and Poincaré
maps are cheap to learn and reframe everything legged; then the **7 → 8 → 10 optimal-control
block** front-loaded, since it is the RL foundation and the most reusable; then **16/17**
to close the loop on stabilizing and contact; **Ch 5** after that, so ZMP arrives already
framed as history rather than as the method.

The **SOTA augmentation notes** (`00b_sota_augmentation.md`) are interleaved rather than
saved for the end — each attaches to the chapter it patches.

---

## Tooling

- The course uses **Drake** (`pip install drake` / `pydrake`). Its trajectory-optimization
  and LQR tooling is best-in-class — the Ch 8/10 examples are near-trivial there and
  painful to reimplement. **Install it and use it for Ch 7–10**, where the solvers *are*
  the lesson.
- **MuJoCo** is already in `.venv` with a working render/sim setup, and the whole
  pick-place stack sits on it. Use it for quick toy sims (pendulum, cart-pole, acrobot)
  and anything that needs to talk to existing work.
- **Isaac Lab** — bring in later, if/when the legged RL milestone becomes hands-on.

---

## How we'll work it

Same as MR: **theory note → discussion → practice** (by-hand for the math that builds
intuition — LQR, value iteration, a collocation setup — and code/sim where a system needs
to be *seen* moving).

First up: **Ch 1 (fully vs underactuated) + Ch 2 (the simple pendulum)**, where the whole
philosophy — "shape energy, don't fight it" — first bites.
