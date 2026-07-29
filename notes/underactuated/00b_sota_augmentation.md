# 00b — SOTA augmentation plan for Underactuated Robotics

Companion to `00_overview.md`. Mirrors what `09_10_learned_sota.md`,
`12b_learned_grasping.md`, and `13d_sota_slam_navigation.md` did for *Modern
Robotics*: the book gives the classical/model-based spine, these notes give the
learned methods that have displaced or extended it.

---

## Why the gap here is different from MR's

**MR (2017) is a *finished* book that predates the deep-learning wave.** Its gaps
are uniform and predictable — it covers classical trajectory generation, planning,
and grasping completely and competently, then simply stops before 2018. Easy to
patch: one SOTA note per stale chapter.

**Underactuated (2024) is a *living draft* that is fully aware of the modern work
and cites it — but delivers unevenly.** It is deep where Tedrake does research and
a bibliography where he does not. So the augmentation is **surgical, not
wholesale**: most chapters need nothing, a few need everything.

Tedrake states the boundary himself in §15.1.2:

> *"As of this writing, conventional wisdom is that model-based control does not
> have a lot to offer to this problem — to design control from cameras, we are
> often limited to either imitation learning or black-box reinforcement learning.
> (I personally think that we have thrown the baby out with the bathwater, and
> consider a highly important research area to close this gap.)"*

Treat that as the map: **everything on the camera/learning side of that line is
ours to supply.**

### What does NOT need augmenting

Ch 2, 3, 7, 8, 10, 16, 17 and Appendix B are **durable**. Pendulum dynamics, LQR,
the HJB equation, direct collocation, transverse linearization, and contact
complementarity do not go stale — they are the substrate the learned methods run
on. This is the reason to do the book at all. Resist the urge to "modernize" them.

---

## The augmentation notes

Numbered to attach to the chapter they patch; written *after* that chapter's
theory pass, so the classical version is in hand first.

### `07b_rl_bridge.md` — from value iteration to modern deep RL
**Patches:** Ch 7 (Dynamic Programming), Ch 11 (Policy Search), Ch 20 (Model-Free
Policy Search — REINFORCE-era, ~2010 vintage).

**The gap:** the book has no PPO, SAC, TRPO, DQN, GAE, or replay buffers anywhere
in 337 pages. It stops at REINFORCE and weight perturbation.

**Content:** Ch 7's value iteration → Q-learning → DQN → the actor-critic family;
what "deep" actually buys (function approximation over a mesh, §7.4.4 taken
seriously); **PPO** and **SAC** as the two workhorses; why on-policy PPO dominates
sim locomotion while off-policy SAC dominates real-robot sample efficiency. Anchor
everything to the Bellman backup from Ch 7 so RL never becomes a black box.

**Payoff:** this is the single highest-value augmentation note — it converts Ch 7/8
from "classical control history" into "the thing PPO is approximating."

---

### `10b_learned_and_gpu_planning.md` — trajopt at scale
**Patches:** Ch 10 (Trajectory Optimization), Ch 12 (Motion Planning as Search).

**The gap:** modest — Ch 10 is strong (28pp) and Ch 12's GCS material is genuinely
current. Mostly about *scale* and *reuse*.

**Content:** **cuRobo** (GPU-parallel trajopt, already met in the MR Ch 9/10 SOTA
note — cross-link rather than repeat); **MPC as a policy** vs MPC as a planner;
learned warm-starts and amortized optimization (MπNets); **trajopt as a demo
generator** for imitation learning, which is exactly how it enters the north-star
pipeline. Short note; largely a bridge back to existing MR material.

---

### `14b_sim_to_real.md` — the missing chapter
**Patches:** Ch 14 §14.7 (domain randomization — *a heading with no body*), Ch 18
(System ID), Ch 15 §15.5.4 (teacher-student — *heading only*).

**The gap:** total. `sim-to-real` appears **zero** times in the book;
`domain randomization` appears twice, both times as a table-of-contents entry.

**Content:** the reality gap and where it actually comes from (contact params,
friction, actuator dynamics, latency); **domain randomization** — dynamics vs
visual, and why over-randomizing yields a conservative useless policy;
**privileged-teacher → student distillation** (the standard legged recipe: train
with ground-truth state, distill to proprioception-only); **system ID + residual
physics** as the alternative to randomizing (§18.2.5 *is* substantive — build on
it, cite TossingBot); actuator network identification.

**Payoff:** load-bearing for M4, and directly serves the pick-place
"improve the MVP with physical models" thread.

---

### `15b_visuomotor_policies.md` — pixels to actions
**Patches:** Ch 15 §15.6 (*three sentence fragments and two citations*).

**The gap:** total, and it is the north star itself.

**Content:** **behavior cloning** and why naive BC fails (compounding error,
covariate shift → DAgger); **action chunking** (ACT) and **Diffusion Policy** —
the two citations Tedrake leaves dangling, expanded properly; flow matching
(already implemented in `pick_place/`, so this is partly a write-up of work
done); **VLAs** (OpenVLA, π0, Gemini Robotics); the observation that **the action
space is still SE(3)/joint deltas through a low-level controller** — i.e. MR and
the Underactuated control spine are what these policies output *into*.

**Cross-link heavily** to the existing pick-place notes and
`notes/00_roadmap.md`; do not re-derive what is already written there.

---

### `05b_modern_locomotion.md` — how humanoids are actually controlled now
**Patches:** Ch 5 (ZMP / whole-body control, 11pp, pre-2015 in flavor).

**The gap:** large, and it is exactly where the M4 priority now sits. `Isaac`
appears zero times in the book.

**Content:** why the field moved from ZMP → **massively-parallel RL** (Isaac Lab /
Isaac Gym, thousands of parallel envs, minutes-to-hours training); the standard
legged-RL recipe (PPO + domain randomization + privileged-teacher distillation +
reward shaping for gait quality); **MPC on centroidal dynamics** as the surviving
model-based branch — and note that *centroidal dynamics itself, from Ch 5, is very
much still current*; humanoid foundation models (GR00T, Helix) and where
loco-manipulation is heading; hardware reality (Unitree G1/H1, Booster) and the
fact that a fall is destructive in a way a wheeled base's isn't.

**Depends on** `07b_rl_bridge.md` and `14b_sim_to_real.md` — write it after both.

---

### Sourced entirely outside the book

- **State estimation** (Ch 19 is empty) — Kalman / EKF / recursive Bayes /
  factor-graph smoothing. Source: *Probabilistic Robotics* (Thrun, Burgard, Fox)
  + the existing `13d_sota_slam_navigation.md`. Only write a note if the SLAM app
  needs it; otherwise the MR note already covers the practical side.
- **Feedback motion planning** (Ch 13 is empty) — funnels / LQR-trees, from
  Majumdar & Tedrake's papers. **Low priority**: Ch 16's transverse linearization
  covers the adjacent idea with actual content, and the funnel program has not
  become mainstream practice.
- **Perception** — unchanged from the MR-era assessment; neither book covers it.
  Already tracked in `notes/00_roadmap.md`.

---

## Priority order

Assuming M4 weighting and the north star:

1. **`07b_rl_bridge.md`** — unlocks reading everything else as approximation of DP.
2. **`14b_sim_to_real.md`** — load-bearing for M4 *and* the pick-place physical-model work.
3. **`15b_visuomotor_policies.md`** — the north star; partly a write-up of work already done.
4. **`05b_modern_locomotion.md`** — the M4 payoff note; depends on 1 and 2.
5. **`10b_learned_and_gpu_planning.md`** — mostly cross-links to existing MR notes.

Ch 13 / Ch 19 replacements: **only on demand.**

---

## Working rule

For each of these: **read the book's classical version first**, then write the
augmentation note as *"here is what changed and why."* The failure mode to avoid
is skipping straight to the modern method — the whole reason to read Tedrake is
that PPO, diffusion policies, and MPC are all far less mysterious once dynamic
programming, the HJB equation, and Riccati are in hand.
