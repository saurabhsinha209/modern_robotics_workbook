# Shopping list — SO-101 build (Project 1)

> **Scope.** Everything needed to go from nothing to *collecting teleop demos with
> an SO-101 follower arm, driven by phone teleop*. Written 2026-08-01,
> revised 2026-08-02 (dropped the leader arm — see §5).
>
> Deliberately **excludes** all legged/QDD hardware — see
> [`proj_hardware_lab.md`](proj_hardware_lab.md) §10 for that, and §13 below for
> what's being deferred and why.
>
> Prices are USD from the official BOM (TheRobotStudio/SO-ARM100). Sourcing to
> India adds shipping + customs — see §12.

---

## 0. TL;DR — three orders

| # | order | approx. |
|---|---|---|
| **1** | Printer + filament | ~$585 |
| **2** | SO-101 **follower** (motors, board, PSU, fasteners) | ~$122 |
| **2b** | *Leader motors — order now, build later* (§5.4) | +$83 |
| **3** | Cameras + workspace | ~$100 |

**Order 1 and 2 the same day.** The printer arrives and you print for ~1–2 days
while the motors are in transit; they converge.

---

## 1. 3D printer

| ✅ | item | qty | spec | why |
|---|---|---|---|---|
| ☐ | **Bambu Lab P1S — AMS Combo** | 1 | enclosed CoreXY, 256³ mm build volume | Enclosure is what later unlocks PETG/ASA/PA-CF for the legged builds. Build volume comfortably fits every SO-101 part. |
| ☐ | **PLA+ filament** | 1 kg | any reputable brand | The BOM specifies **PLA+**. Follower-only needs roughly half of what a pair would; 1 kg leaves headroom for failed prints, camera mounts and fixtures. |

**Notes**
- The **P1S was superseded by the P2S** (+$150). P1S dropped to **$399 solo /
  $549 AMS combo** in Feb 2026. P1S is the value pick; P2S buys stock
  abrasive-readiness, quick-swap nozzle, and a 110 °C bed — none of which
  SO-101 needs.
- **AMS is not needed for SO-101** (single-colour functional parts). It's on the
  list for the multi-colour toy use case. Drop to P1S solo at $399 if you'd
  rather defer.
- ⚠️ **PLA in an enclosed printer can heat-creep and clog.** On the P1S, print
  PLA with the **front door open and glass top removed**. Known P1S quirk (one of
  the things the P2S fixed). Costs nothing to work around, but will waste a print
  if you don't know it.

### Print settings (from the official BOM — don't improvise)

```
material   PLA+
nozzle     0.4 mm @ 0.2 mm layer   (or 0.6 mm @ 0.4 mm)
infill     15%
supports   everywhere, ignore slopes >45° from horizontal
```

---

## 2. Motors

### Buying now — follower arm

**6 × Feetech STS3215, 7.4 V, 1/345 gear (C001) @ $13.89 = $83.34**

All six follower joints use the **same** motor. Uniform, no sorting.

### Order alongside — leader motors (see §5.4 for the argument)

| ✅ | gear ratio | code | qty | leader joint |
|---|---|---|---|---|
| ☐ | 1 / 191 | C044 | 2 | shoulder pan, elbow flex |
| ☐ | 1 / 345 | C001 | 1 | shoulder lift |
| ☐ | 1 / 147 | C046 | 3 | wrist flex, wrist roll, gripper |

**+$83.34.** The leader deliberately mixes ratios so it can hold its own weight
while staying backdrivable by hand — that's what makes it light to teleop with.

> ⚠️ **If you order both, check the ratio split on the listing.** Vendors sell
> "12× STS3215" bundles that are all 1/345 — that's the SO-100 spec. A uniform
> set assembles fine and then the leader is heavy and unpleasant, which quietly
> degrades every demo you collect.

> ⚠️ **7.4 V version, not 12 V.** Both exist. Voltage is printed on the motor
> body. The PSU in §3 assumes 7.4 V.

---

## 3. Electronics

| ✅ | item | qty | spec | notes |
|---|---|---|---|---|
| ☐ | **Motor control board** (Waveshare/Feetech serial bus servo driver, ST3215-compatible) | **1** | $10.60 | One per arm. If Waveshare: **set both jumpers to the `B` channel (USB)** or `lerobot-setup-motors` won't see the bus. |
| ☐ | **Power supply** | **1** | 5 V barrel jack, ≥4 A | ⚠️ Confirm voltage/amperage against the vendor listing for the motor variant you order. |
| ☐ | **USB-C cable** | 1 | $7 for a 2-pack | Board → computer. |
| ☐ | **3-pin servo cables** | 6+ | usually bundled with motors | ⚠️ Verify they're included. Buy 2–3 spares — most-handled part of the build. |

*Second board + PSU only if/when the leader gets built (+$21).*

---

## 4. Fasteners & tools

| ✅ | item | qty | notes |
|---|---|---|---|
| ☐ | **M2 × 6 mm screws** | kit | Motor mounting — 4 per motor, plus wrist/gripper. |
| ☐ | **M3 × 6 mm screws** | kit | Everything structural + all motor-horn screws. |
| ☐ | **Screwdriver set** | 1 | $6 — **Phillips #0 and #1** specifically. |
| ☐ | **Table clamps** | 2 | ~$4.50 — non-optional. An unclamped follower will walk itself off the desk. |

> **M2×6 and M3×6 are the only two sizes in the entire assembly.** The official
> BOM does *not* publish screw counts, so **buy a pre-counted SO-101 hardware kit
> from the same vendor as the motors** (get the full-pair kit — the delta is
> pennies and it covers the leader later). Running out of one screw size
> mid-assembly is the most common way this build stalls for a week.

---

## 5. Teleop — phone, not leader

Phone teleop is already built and proven in sim, and LeRobot supports the pattern
natively: **controller pose → EE target → IK → joint commands**, with a
**clutch/deadman** so the arm only moves while grip is engaged. Nothing to buy.

### 5.1 The rule that must not be broken

> **Record absolute targets. Never integrate.**

The teleop *interface* and the recorded *action space* are separable. The phone
is just a target generator; what lands in the dataset is a self-contained target.

⚠️ **Narrower than [`proj_hardware_lab.md`](proj_hardware_lab.md) §1 states.**
That note says "absolute joint positions, not EE deltas," which reads as *joint
space vs Cartesian*. The actual failure was **integration** — a running reference
diverging from measured pose. **Absolute EE pose in the base frame has no
integration step** and does not have that failure mode. Cartesian was never the
problem; the accumulator was.

So both of these are legitimate action spaces:

| action space | note |
|---|---|
| **absolute joint targets** `q_target` | Directly executable on a position-controlled servo. No IK at inference. No rotation-representation choice. |
| **absolute EE pose in base frame** | Transferable across embodiments; matches VLA convention (π0, Octo, RT-X). Needs IK at inference. |

**What is *not* legitimate:** anything the policy has to integrate — EE deltas,
joint deltas, velocities — because the reference the deltas accumulate onto
drifts from the measured state.

#### If recording EE pose, two things are load-bearing

**1. Record `FK(q_achieved)`, not the commanded phone pose.**
SO-101 is **5-DOF + gripper** (shoulder pan, shoulder lift, elbow, wrist flex,
wrist roll) — *deficient*, not redundant. Full 6-DOF EE pose is unreachable; the
gripper's approach azimuth is structurally tied to the shoulder-pan angle. A
6-DOF EE action space is therefore over-parameterized on this arm: only a 5-D
manifold is achievable. **FK of the measured joints lies on that manifold by
construction.** The commanded phone pose does not — it is off-manifold by exactly
the amount the IK projected away.

**2. Use a 6D rotation representation.** Not Euler (gimbal discontinuity), not
raw quaternions (`q` and `−q` are the same rotation, so the regression target is
ambiguous and the loss jumps across the sign flip). Predict two 3-vectors and
Gram-Schmidt them into a rotation matrix. This is the Ch 3 SO(3) material cashing
out directly.

#### The decision you don't have to make yet

> **Log everything, decide the action space at training time.**

Every step, record: `q`, `q_target`, `FK(q)`, and the raw commanded phone pose.

FK is exact and free, so **joint positions are the more complete record** — EE
derives from them losslessly, while the reverse needs IK and is ambiguous. Logging
all four makes the action space an A/B you can run without re-collecting a single
demo, and it tells you whether the phone, the IK, or the servo was at fault when a
demo looks wrong later.

**Default for task 1: train on joint targets.** Simplest executable stack, no IK
in the inference loop. Keep the EE option alive for cross-embodiment / pretrained
VLA fine-tuning, which is on the roadmap.

### 5.2 What the leader arm gave you for free, and now doesn't

A leader arm is a **kinematic twin** — any pose you can physically put it in is
by construction reachable, singularity-free and within joint limits. Phone + IK
gives that guarantee up. Expect, and handle explicitly:

| failure | what it looks like in the data |
|---|---|
| **IK has no solution** (out of reach) | commanded pose diverges from achieved; arm stalls at the boundary |
| **Joint limit hit** | one joint saturates while the EE keeps tracking — silent, and the arm is now in a different configuration than you think |
| **Near-singularity** (arm near-straight, wrist aligned) | tiny phone motion → huge joint velocity. The arm snaps. |
| **Elbow flip** | IK jumps between solutions; a discontinuous step appears in `q_target` |

> **These produce discontinuous action jumps mid-demo, and that is the main
> dataset-quality risk of going leader-less.** A policy trained on data
> containing teleport-steps will reproduce them.

> **Note the interaction with §5.1.** This risk applies to a **joint-space**
> action record — an elbow flip lands directly in `q_target`. If the recorded
> action is **EE pose**, the flip is an execution-side artifact and never enters
> the dataset; the problem moves to inference time, where the guards below apply
> in one place, identically at collection and rollout. That's a real point in
> favour of EE pose here, and another reason to log both and decide later.

**Mitigations, in order of value:**

1. **Never emit a discontinuous `q_target`.** If IK fails or jumps, **hold the
   last valid target** rather than commanding the new one. Freezing is
   recoverable; a jump is not.
2. **Seed IK from the current joint configuration** and pick the nearest
   solution. Kills most elbow flips outright.
3. **Rate-limit `q_target`** in joint space. A hard cap on per-step joint delta
   is the cheapest possible guard against singularity snap.
4. **Flag, don't silently drop.** Mark demos where IK failed and audit them —
   same discipline as `audit_dataset.py`. Discontinuities are far easier to catch
   at collection time than to debug from a failed rollout.

### 5.3 Interaction details to settle before the first session

- **Clutch / deadman.** Mandatory, and does double duty: safety, *and* workspace
  re-indexing — release, move the phone back to a comfortable pose, re-engage.
  Exactly like lifting a mouse. Without it you run out of either arm workspace or
  arm reach within seconds.
- **Motion scale.** Phone travel → arm travel. Start ~1:1 and tune down; too
  large makes fine approach impossible, too small makes demos slow.
- **Gripper control.** No trigger on a phone. On-screen button or volume key.
  Decide binary open/close vs continuous **before** collecting — it's an action
  dimension and changing it invalidates the dataset.
- **Orientation is the hard part.** Position via phone translation is intuitive;
  wrist orientation via phone rotation is much less so. If demos are slow, this
  is why. Consider locking orientation for the first task.

### 5.4 Order the leader motors anyway — build them only if needed

**$83 of motors, on an order you're already placing.**

The argument is sourcing latency, not doubt about phone teleop. These are
imported, long-lead items, and a second international order means shipping +
customs + weeks all over again. If phone teleop works well, you've spent $83 you
didn't need on a ~$900 build. If demo throughput or the discontinuity problem in
§5.2 turns out to bite, you print the leader over a weekend instead of losing a
month.

**Data collection is the project** — that's §1 of the lab note. This is cheap
insurance on the one thing that gates everything downstream.

*(Skip the second board + PSU for now — those are locally sourceable and only
$21.)*

---

## 6. Cameras

| ✅ | item | qty | notes |
|---|---|---|---|
| ☐ | **Static/scene camera** | 1 | Standard USB webcam, 1080p. |
| ☐ | **Wrist camera** | 1 | Small USB module (Arducam/ELP-class) that can bolt to a printed wrist mount. |
| ☐ | **Powered USB hub** | 1 | See bandwidth note below. |

**Three things that matter more than camera quality:**

1. **Disable autofocus and auto-exposure. Lock them.** If the camera refocuses or
   re-exposes between demo collection and policy rollout, the policy sees a
   different image distribution than it trained on and fails for reasons that
   look nothing like a camera problem. Prefer **fixed-focus** modules, or lock
   focus/exposure in software and never touch it again.
2. **USB bandwidth.** Two uncompressed 1080p streams will saturate a single USB
   controller — dropped frames, or a camera that refuses to open. Use **MJPEG**,
   and use separate controllers or a powered hub.
3. **Mount rigidly, then don't move it.** A static camera that shifts between
   sessions silently splits your dataset in two. Mark its position on the desk.

> ⚠️ **Phone teleop makes camera discipline more important, not less.** With a
> leader arm your hands are on the rig and your eyes are on the workspace. With a
> phone, you may be tempted to watch the *phone screen*. Don't rely on a camera
> feed you haven't also verified is being recorded correctly — and never move a
> camera to get a better view mid-session.

---

## 7. Workspace (print these, don't buy them)

| ✅ | item | notes |
|---|---|---|
| ☐ | **Reset fixture** | Marked object start pose + a bin. `proj_hardware_lab.md` §1: build this *before* the pipeline. The difference between 200 demos and quitting at 40. |
| ☐ | **Camera mounts** | Wrist mount + static mount. Print 3–4 variants. |
| ☐ | **Phone stand / cradle** | Optional, but a resting position you can return the phone to makes clutch re-indexing repeatable. |
| ☐ | **Objects to grasp** | Start with one rigid, matte, high-contrast object. Print it so it's replaceable and dimensionally known. |
| ☐ | **Non-slip mat** | The one actual purchase — keeps the workspace registered between sessions. |

---

## 8. Software — free, set up while parts ship

| ✅ | item |
|---|---|
| ☐ | **LeRobot** + Feetech SDK: `pip install -e ".[feetech]"` |
| ☐ | **Port the phone teleop layer** — swap the sim target for the SO-101 IK path (§5.1) |
| ☐ | **Bambu Studio** (slicer) |
| ☐ | **STL files** — TheRobotStudio/SO-ARM100 repo |

---

## 9. Cost summary

| block | USD |
|---|---|
| Follower motors (6 × 1/345) | 83.34 |
| Control board | 10.60 |
| Power supply | 10.00 |
| USB-C cable | 7.00 |
| Table clamps (2) | ~4.50 |
| Screwdriver set | 6.00 |
| **Follower subtotal** | **~122** |
| *Leader motors (deferred build)* | *83.34* |
| Printer (P1S AMS combo) | 549.00 |
| PLA+ filament (1 kg) | ~25.00 |
| Cameras + hub | ~100.00 |
| **Total, follower only** | **~795** |
| **Total, incl. leader motors** | **~880** |

*Dropping the leader saves ~$110 all-in against the paired build.*

---

## 10. Order of operations

1. **Order printer + parts same day.**
2. Printer arrives first → print all follower parts (~1 day). Clean supports with
   a small screwdriver.
3. Motors arrive → **set motor IDs and baudrates *before* assembly**, one motor
   at a time via `lerobot-setup-motors`. Motors ship with ID=1; each needs a
   unique bus ID.
4. **Insert a 3-pin cable into each motor before mounting it.** Once assembled
   there is not enough room to seat a connector. Flagged in the official docs and
   every build log.
5. Assemble follower → calibrate.
6. **Validate the command path before any teleop** — command a known joint
   target, measure what actually arrives. `proj_hardware_lab.md` §9; the
   `body_gravcomp` lesson applies here too.
7. Bring up phone teleop → IK → joint targets. Verify §5.2 guards *before*
   collecting data, not after.
8. Cameras, mounts, reset fixture.
9. First teleop session.

---

## 11. Verify before paying

- [ ] Follower motors are **6 × 1/345**, 7.4 V
- [ ] If ordering leader motors: **2 × 1/191, 1 × 1/345, 3 × 1/147**
- [ ] **3-pin cables included** (or ordered separately)
- [ ] PSU voltage/amperage **matches the motor variant**
- [ ] Control board is **ST3215-compatible**
- [ ] Screw kit covers **both** M2×6 and M3×6 — get the full-pair kit

---

## 12. India sourcing

The BOM's linked vendors are US/CN. Practical routes:

- **Seeed Studio / Waveshare / OpenELAB** — sell SO-101 kits and ship
  internationally. Simplest path; the kit removes the fastener-counting problem
  entirely. ⚠️ Most kits are sold as **pairs** — check whether a follower-only SKU
  exists, or whether the pair kit is cheap enough that §5.4 decides itself.
- **Local (Robu.in, ThinkRobotics, etc.)** — worth checking for the generic parts
  (PSU, screws, clamps, cameras, USB hub). Unlikely to stock the correct STS3215
  gear-ratio mix.

**Budget shipping + customs duty on top of the USD figures** — for imported
electronics this is not a rounding error. Ask whether the vendor ships **DDP**
(duty paid) before ordering; often cheaper than being billed on delivery.

**This import latency is the entire argument for §5.4.**

The **printer** is the one item worth buying through Indian distribution —
warranty and support on a machine you'll depend on for years beats saving on
import.

---

## 13. Deliberately *not* buying yet

| item | trigger to revisit |
|---|---|
| **Leader arm electronics** (2nd board + PSU, $21) | Only if the leader gets built. Locally sourceable, no import latency — hence deferred while the motors aren't. |
| **Filament dryer** | First **PETG / ASA / PA-CF** print. PLA+ doesn't need it. ~$60–100, 70 °C+, PTFE feed-through port. |
| **Hardened nozzle + extruder gears** | Same trigger — first carbon-fibre-filled filament. ~$25–40. |
| **QDD motors, FOC driver, IMU, bench PSU** | Step A (two-wheel balancer). See [`proj_hardware_lab.md`](proj_hardware_lab.md) §10. Separate decision, separate order — and §7 there is the irreversible one. |
| **Local GPU / Jetson** | Cloud GPU stays fine through training. Revisit when *inference latency on the robot* is the bottleneck, not before. |
