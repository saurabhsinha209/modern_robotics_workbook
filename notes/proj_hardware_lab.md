# Project note — Building the hardware lab

> **What this is.** The plan for moving from sim to real hardware: what to build
> first, in what order, and which decisions are expensive to reverse. Written
> 2026-08-01, at the point of starting the physical lab. The arm is settled
> (SO-101); the open question was *how to get to legged and wheeled-legged
> without burning money on the wrong hardware*.
>
> Companion notes: [`proj_qdd_joint_fundamentals.md`](proj_qdd_joint_fundamentals.md)
> (motor constants, torque/heat budget), [`proj_pick_place_plan.md`](proj_pick_place_plan.md),
> [`proj_iteration_loop.md`](proj_iteration_loop.md) (the diagnose-driven loop —
> applies unchanged on hardware, with a longer cycle time).

---

## 0. TL;DR

```
   Project 1   SO-101 learned pick-place              ← the sim work, made real
   Step A      two-wheel balancer, QDD motors         ← FIRST legged-adjacent build
   Step B      + legs  →  Ascento-class               ← this IS wheeled-legged Phase 1
   Step C      single leg on a boom  (optional)       ← contact, impact, SLIP
   Step D      quadruped                              ← only if B left something wanted
```

**Do not buy a quadruped second.** And **commit to one QDD actuator ecosystem
early** — it's the only genuinely irreversible decision on this list.

---

## 1. Project 1 — SO-101 learned pick-place

The policy side is already understood from sim. Three things that are *different*
on hardware, in descending order of how much time they'll cost:

### Use absolute joint-position actions. Not EE deltas.

The sim failure was measured-pose vs. running-reference EE-delta integration
(see the deploy-loop debugging in [`proj_iteration_loop.md`](proj_iteration_loop.md)).
SO-101 is position-controlled at the servo, so **record and predict absolute joint
targets** and that entire failure class stops existing.

Don't port the delta action space over out of continuity. It was never the right
fit; hardware is a clean excuse to drop it.

### Data collection *is* the project

| goal | rough demo count |
|---|---|
| fixed object pose, fixed start | ~50 |
| object position variation | 200+ |
| multi-object / language-conditioned | many more, and a different problem |

Budget more time for demo collection and camera mounting than for training.
**Camera placement (wrist + one static) will move success rate more than swapping
ACT for a diffusion policy will.** Leader-follower teleop makes the SO-101 pair
the whole rig.

### Nothing resets itself

Build the reset fixture *before* the pipeline — a marked object start pose and a
bin. This sounds trivial and is the difference between collecting 200 demos and
giving up at 40.

**Stack:** LeRobot. ACT, diffusion policy, SmolVLA, π0 are all in-repo and SO-101
is natively supported, so the sim work transfers almost directly.

---

## 2. Why not go straight to a quadruped

The obvious next step after an arm is a quadruped. It's the wrong one.

A quadruped hands you **four coupled legs + contact scheduling + floating-base
state estimation + a safety problem** simultaneously, on hardware where a bad
controller costs a gearbox. And the consumer tiers of the obvious platforms don't
expose low-level joint torque control at all — which makes them useless for
everything in `10b`.

The ladder below reaches the actual target (wheeled-legged) *sooner*, on cheaper
hardware, while paying the sim2real tax on parts that can be replaced for pocket
money.

---

## 3. Step A — two-wheel balancing robot

**The single best first legged-adjacent build**, for four independent reasons:

1. **It is the Underactuated syllabus made physical.** Cart-pole / Acrobot family.
   LQR from `08` literally works on it, and you can *feel* the difference between
   a good and a bad `Q` — which no amount of sim gives you.
2. **It's the direct precursor to Phase 1** of the wheeled-legged plan (the
   Ascento-class 2-wheel balancing legged robot).
3. **It's the natural extension of the one-joint QDD bench** already planned in
   [`proj_qdd_joint_fundamentals.md`](proj_qdd_joint_fundamentals.md) — same
   motors, same driver, same calibration, one more joint.
4. **Cheap and hard to destroy.**

**Bill of materials:** two BLDC motors, an FOC driver, an IMU, a bench supply.

**Where the time actually goes:** state estimation (fusing IMU + wheel encoders
into a usable tilt estimate) and actuator characterization. That's precisely the
tax you want to pay on hardware you can replace.

### 3.1 What the system actually is

A **wheeled inverted pendulum (WIP)** — and structurally it *is* cart-pole:
4 states (wheel angle, body pitch, and their rates), 1 input, degree of
underactuation 1. The only real difference is that the input is a **torque between
wheel and body** rather than a force on a cart. Ground contact converts wheel
torque into forward force; the reaction torque tilts the body. Action–reaction
does the work.

### 3.2 Target *motion*, not balance — because of the RHP zero

> **To accelerate forward you must first lean forward. To lean forward you must
> first drive the wheels backward.** The robot moves *away* from the commanded
> direction before it comes back.

That's a **right-half-plane zero** — the system is **non-minimum phase**. It is
structural, not a tuning failure; no controller removes it. (Same phenomenon as
crouching before a jump, and every Segway lurch.)

**A pure balancing controller never shows you this.** A tracking controller hits it
immediately, and it changes the design problem: you cannot demand fast position
tracking, because the initial undershoot fights you. This is the main argument for
making the goal "follow a trajectory" rather than "stay up."

### 3.3 Milestone ladder

1. **Balance at rest.** Note this is already the interesting version: regulating
   *only* pitch lets the robot drift away indefinitely. Pitch **and** position must
   be regulated simultaneously with one input — that's the underactuation, and
   there's nowhere to hide from it.
2. **Velocity tracking** — hold 0.5 m/s. Exposes the equilibrium lean (lean during
   acceleration; near-upright at constant speed, minus rolling resistance).
3. **Position trajectory following** — where the RHP zero bites.
4. **Disturbance rejection** — push it, measure recovery. Now "what's the largest
   recoverable tilt?" becomes answerable, and it's a **reachability / funnel**
   question, not a tuning question.
5. **Yaw.** Two independently driven wheels ⇒ heading is also controllable. Turn
   while balancing, then track a curved path — MR **Ch 13** diff-drive
   nonholonomic content, physically.

### 3.4 Controller ladder, mapped to the notes

| controller | note | what it teaches |
|---|---|---|
| **LQR about upright** | `08` | Works, and works well. Get this first — a `Q`/`R` playground you can *feel*. |
| **LQR + integral action** | — | Steady-state velocity tracking without droop. |
| **TVLQR on a planned trajectory** | `10b §1` | The plan-then-stabilize two-step. |
| **Constrained MPC** | `10b §2` | **Where MPC earns its keep** — see below. |
| **iLQR** | `10b §5` | Aggressive maneuvers, large recoveries. |

**Why this is an unusually good MPC testbed:** a hard **torque limit**, a **state
constraint** (tilt beyond some angle is unrecoverable), *and* an **RHP zero** that
makes naive aggressive tracking saturate the motors. That combination is precisely
where LQR is genuinely inadequate and constrained MPC visibly wins. On most toy
systems MPC is overkill; here the difference is demonstrable on real hardware.

Also: the WIP is **differentially flat**, so `10b §8` applies directly — plan in
output space, recover the torque profile algebraically, RHP zero handled by
construction rather than fought.

### 3.5 Two gotchas that will bite

**1. The accelerometer lies while accelerating.** An accelerometer measures
*specific force*, not tilt — horizontal acceleration is indistinguishable from a
rotation of the gravity vector. So the standard complementary filter (accel for
low-frequency tilt, gyro for high-frequency) works beautifully **at rest** and
degrades **exactly when you start driving**, which is the regime that matters.
Compensate using the known wheel acceleration, or estimate pitch in a filter that
models the dynamics.

> This is the single most common reason balancing robots work on the bench and
> fall over the moment they're commanded to move. It is a **state-estimation**
> bug that presents as a control bug — the §9 lesson again.

**2. Make it tall.** Higher CoM ⇒ *slower* dynamics ⇒ **easier** to balance (a
broom balances on your palm; a pencil doesn't). Counterintuitive, and the right
call for build #1: height buys time constant, which buys margin for a mediocre
first estimator. Lower it later to make the problem harder *deliberately*.

**Sizing note:** the **torque limit sets the maximum recoverable tilt angle** —
the number that decides whether the robot survives a shove. Compute it from the
motor constants in
[`proj_qdd_joint_fundamentals.md`](proj_qdd_joint_fundamentals.md) **before**
buying, not after.

---

## 4. Step B — add legs → Ascento-class

Hip + knee per side, wheel at the foot. **This is already wheeled-legged Phase 1.**

The milestone stays what was set in the project plan: **body-pose control
decoupled from wheel motion**, not "drive straight."

Crucially, you reach this **without ever solving contact scheduling** — rolling
contact is persistent, which was the entire reason wheeled-legged was chosen over
a quadruped (`10b §12`, "why wheels change the calculus").

**Continuity from Step A:** Ascento is the WIP of §3 with the **CoM height made
controllable** by adding a leg. Every controller written in Step A survives; the
leg adds a second, slower authority over the same balance problem (and eventually
the ability to hop a step, which the WIP can never do).

> **Expect the centroidal model to hit a wall here**, and read the wall as
> motivation rather than a bug: the wheel's nonholonomic rolling constraint is
> tied to wheel *orientation*, hence to joint angles, so a centroidal model that
> abstracts legs into point force vectors literally cannot express it. That's the
> argument for whole-body MPC — see `notes/underactuated/10b_mpc_ilqr_mppi.md §12`.

---

## 5. Step C — single leg on a boom *(optional)*

If you want contact experience before committing to four legs. Teaches impact,
ground-reaction-force estimation, and makes the SLIP model of
`notes/underactuated/04b_compass_gait_slip.md` physical.

High value, genuinely optional. Skip it if Step B is going well.

---

## 6. Step D — quadruped

By this point you know your actuator ecosystem, your sim2real gap, and your state
estimator. Two routes:

| | pros | cons |
|---|---|---|
| **Pupper-class** (Stanford Pupper v3 etc.) | cheap, fully open, hackable, MJX sim2real pipelines exist, real course material | small, limited payload, less capable |
| **Unitree Go2 EDU** | genuinely capable, well-supported SDK | expensive; **tier trap below** |

> ⚠️ **Tier trap.** Consumer Go2 tiers do **not** expose low-level joint torque
> control — only the EDU / developer tier does, and that's the only thing that
> makes it usable for `10b`-style work. **Verify current SDK access before
> buying**; this detail has changed between hardware revisions.

**Note that Step B already lands at wheeled-legged.** Step D may turn out to be
unnecessary — decide after B, not before A.

---

## 7. The one decision that's hard to reverse: actuators

Everything else on this list can be swapped. This one can't, so make it
deliberately.

| | verdict |
|---|---|
| **Dynamixel** (smart servos) | Fine for **arms** — it's what SO-101 uses. **Wrong for legged:** poor torque control, not backdrivable, geartrains don't survive impacts. |
| **QDD / BLDC + FOC** | **What legged needs, non-negotiable.** Backdrivable, real torque control, impact-tolerant. |

Ecosystems worth comparing: **ODrive**, **mjbots moteus**, **CubeMars AK series**,
**MyActuator RMD**, **Unitree standalone motors**.

**Pick one and buy into it.** Mixing ecosystems means maintaining two CAN stacks,
two calibration procedures, and two sets of firmware quirks — a permanent tax on
every project after this one. The sizing math for making the choice is in
[`proj_qdd_joint_fundamentals.md`](proj_qdd_joint_fundamentals.md).

---

## 8. Lab infrastructure, in purchase order

| | why |
|---|---|
| **E-stop + boom / gantry** | Before any legged hardware powers on, not after. |
| **Ground-truth pose** | You cannot debug a state estimator without one. Vive trackers are the cheap route; ArUco + a couple of cameras works for slow motion. |
| **Compute** | x86 mini-PC if **MPC** is the target (OCS2 / Crocoddyl / Pinocchio build painlessly). Jetson if **learned policies** are. |
| **Bench PSU with current limit** | Catches a bad controller *before* it destroys a gearbox. |

---

## 9. The habit that will save the most time

> **For every piece of hardware: build its MuJoCo model first, and validate the
> actuator command path before writing any controller.** Command 1 Nm, measure
> what actually arrives.

The `body_gravcomp` finding is the template — the bug lived in the layer everyone
assumes is correct. On legged hardware that layer is **torque command → applied
torque**, and getting it wrong looks *exactly* like a bad controller. You will
spend days tuning gains that were never the problem.

Same lesson as [`proj_iteration_loop.md`](proj_iteration_loop.md): the rollout is
ground truth, and the instrumentation comes before the tuning.

---

## 10. Concrete first purchase

**Two QDD motors from whichever ecosystem is chosen, a driver, an IMU, a bench
supply.**

That's Step A; it's simultaneously the one-joint bench already planned; and it
makes the single irreversible decision (§7) with the smallest possible
commitment.

---

## 11. Theory status — stop reading, build

**Assessment (2026-08-01): theory is sufficient.** MR complete, Underactuated
through Ch 10, a full sim build with three policy families trained and diagnosed.
There is more theory banked here than can currently be cashed out; more reading
now would be a way of *not building*.

**The WIP (§3) is the intro to Underactuated-on-hardware**, taken after the SO-101
project — the smallest system that forces the underactuation, estimation, and
delay lessons all at once.

### Three real gaps, in order of cost

1. **The embedded/mechanical layer — the largest gap, and not in any book.**
   CAN wiring and termination, motor calibration and encoder offsets, firmware
   flashing, power distribution, connectors that vibrate loose, mechanical
   stiffness, debugging a silent bus. The first WIP will take longer than
   expected and **almost none of the extra time will be controls.** Budget for it.
2. **State estimation — the one genuine theory gap, learned just-in-time.**
   Barely in MR or Underactuated, and it blocks every project here: WIP pitch
   under acceleration (§3.5), floating-base estimation for Ascento. The SLAM
   deep-dive covered the *exogenous* half; what's missing is **proprioceptive**
   estimation — IMU + wheel/leg odometry fusion, contact-aided EKF, gyro bias.
   **Don't read a book first.** Learn it on the WIP, where the state is 4-D and
   everything is plottable.
3. **Discretization, loop delay, sensor noise, actuator bandwidth.**
   Underactuated is state-space-first and skips classical control. That's fine
   until hardware. An LQR designed in continuous time, run at 200 Hz, with a few
   ms of sensor delay and a motor current loop of its own, **is a different
   system than the one designed for** — and on a balancer, delay directly eats
   the phase margin that is keeping it upright.

   *Symptom:* stable in sim, oscillates on hardware, **worse as gains go up.**
   Without the concept you'll blame the model; with it you check loop rate and
   delay budget first. Worth a **short targeted read — hours, not a course**:
   phase margin, how delay consumes it, why sampling rate matters.

### The defer list — and the trigger for each

Nothing below is needed now. Each is worth reading **when the project demands
it**, because you'll arrive with a concrete failure it explains.

| deferred | trigger — read it when… |
|---|---|
| **Underactuated Ch 16** — limit-cycle stabilization | **you make it to Ascento-class** and it needs to walk, hop, or hold a gait. Stabilizing an *orbit* rather than a *trajectory* is the whole chapter, and Ascento is where it stops being abstract. |
| **Underactuated Ch 17** — planning/control through contact | **Phase 2: the curb.** The moment a wheel must leave the ground, the mode sequence returns and this is the chapter about it. |
| **Underactuated Ch 11+** — policy search / RL | **MPC visibly hits its ceiling** — i.e. Phase 2, or contact modes that must be *discovered* rather than scheduled (`10b §12`). |

**Rule:** a chapter becomes worth reading the moment you can name the hardware
failure it would have prevented.
