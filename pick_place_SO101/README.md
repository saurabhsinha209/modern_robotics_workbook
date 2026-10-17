# pick_place_SO101 — phone teleop → IK → SO-101

Phone (WebXR) pose → clutched target → 5-DOF IK → guarded joint targets →
SO-101 (MuJoCo sim **or** the real arm through LeRobot). Collects raw teleop
episodes with a human-triggered SUCCESS / FAIL terminal label. No training code
yet — that comes after the camera pipeline.

```
phone ──ws──▶ PhoneMapper ──▶ Cartesian ──▶ SO101Kinematics ──▶ JointGuard ──▶ Backend
 pose,clutch   relative map     rate limit   5-DOF DLS IK        hold/rate/    sim | real
 gripper %     ref = FK(q_cmd)                 (trust region)      limits        (LeRobot)
                                                                                   │
phone UI ◀─status── EpisodeLogger ◀─────────── q, q_cmd, FK, phone pose ◀──────────┘
 START / SUCCESS / FAIL / DISCARD
```

## Run

```bash
cd modern_robotics                                   # repo root

# 0. no phone, no hardware: full pipeline self-test (25 checks, ~5 s)
.venv/bin/python -m pick_place_SO101.run_teleop --selftest

# 1. validate the phone stream (no robot)
.venv/bin/python -m pick_place_SO101.run_teleop --validate

# 2. dry run in sim with the viewer (macOS viewer needs mjpython)
.venv/bin/mjpython -m pick_place_SO101.run_teleop --backend sim

# 3. real arm; --twin mirrors the *measured* joints in a MuJoCo viewer
.venv/bin/python   -m pick_place_SO101.run_teleop --backend real --port /dev/tty.usbmodem…
.venv/bin/mjpython -m pick_place_SO101.run_teleop --backend real --twin
```

Phone (Android Chrome, same Wi-Fi): open `https://<mac-ip>:8443/`, accept the
self-signed cert once, **Start AR session**.

## Phone UI

| control | what it does |
|---|---|
| **CLUTCH** | toggle. On: phone motion since engage drives the arm. Off: arm holds, reposition the phone (ratchet). |
| **ORIENT LOCK / FREE** | Lock: phone rotation ignored, gripper keeps its orientation (wrist compensates). Free: phone pitch/roll are followed. Default **lock** — orientation is the hard part with a phone. |
| **GRIPPER** | slider 0–100 % + presets CLOSED · ¼ · ½ · ¾ · OPEN. Recorded as the continuous %, presets are only UI sugar, so the action dimension never changes. |
| **START** | begins an episode; the text box is the language instruction, sent with it (recorded in `meta.json`). |
| **SUCCESS / FAIL** | ends the episode with that label. Only enabled while recording. |
| **DISCARD** | drops the episode (deletes it). |
| **HOME** | joint-space ramp to the ready pose; teleop suspended until it arrives. |
| **SET FWD** | point the phone where the arm points, tap. Fixes the phone→robot heading. |
| IK chip | `IK ok` · `speed-limited` · `OUT OF REACH` · `IK HOLD` · `HOMING` |

No haptics (removed on purpose); status is visual only.

## Why a new IK (read this once)

The SO-101 has **5** arm joints. Pan + three parallel pitch joints + wrist roll
give a reachable set of: TCP position (3) + gripper pitch + gripper roll. The
approach direction's *azimuth is welded to pan*. A 6-D pose solver (the Franka
one in `../pick_place/ik.py`) fights itself on such an arm. `kinematics.py`
solves a **square 5-D** error instead — position, rotation about the pitch axis,
rotation about the approach axis — and drops the uncontrollable component.
Phone **yaw is therefore ignored** (unless the gripper points straight down,
where yaw *is* roll). This is the "deficient, not redundant" point from
`notes/proj_so101_shopping_list.md` §5.1/§5.2.

## Guards (§5.2 of the shopping-list note, implemented)

| failure | defence | counted per episode as |
|---|---|---|
| target past the workspace | mapping **re-anchors** (mouse-at-screen-edge) — no dead zone on retreat | `ik_fail_steps` |
| IK solution far from last command | solve is trust-region-bounded; if still jumpy, **hold** ≤ 3 ticks then rate-limited pursuit (never deadlocks) | `held_steps` |
| joint speed | per-joint cap (60 °/s default) | `rate_limited_steps` |
| joint limits | MJCF range ∩ your calibration sweep, minus 3° | — |
| bursty phone packets (stair-step) | **jitter buffer**: pose rendered `--smooth-ms` (60) in the past on the phone's own timeline; `0` = off | — |
| phone VIO teleport | re-latch, arm does not move | `vio_jump` |
| phone stops sending | >0.25 s stale ⇒ clutch off | — |
| arm blocked / stalled | command resyncs to measured | `resync_steps` |
| bus I/O hiccup | hold; >10 in a row ⇒ stop | — |
| exit | **park** (fold) before torque off, so it doesn't drop | — |

Key design choice: the engage reference is `FK(q_commanded)`, **not** the measured
pose. A P-only servo under gravity sags 1–3°; re-deriving the goal from the
measured pose would creep the arm down. Measured state is only used to resync.

## Data

```
data/<session>/ep_0007/steps.npz     per-step arrays
data/<session>/ep_0007/meta.json     instruction, outcome, counts, calibration id
data/<session>/ep_0007/frames/<cam>/ (only when the camera hook passes frames)
data/<session>/episodes.jsonl
```

`steps.npz` keys: `t` · `q` (**measured**, 5 deg + gripper %) · `q_cmd`
(**commanded** joint targets) · `tcp_p`, `tcp_rot6d` (**FK of the command**, the
EE-space action; 6-D rotation, not Euler/quat) · `tgt_p`, `tgt_rot6d` (what the
phone asked for, pre-IK) · `phone_p`, `phone_q` · flags `clutch orient_lock ik_ok
held rate_limited resynced vio_jump` · `terminal` (1 on the last step) ·
`outcome` (1 success / 0 failure).

Everything is logged so the **action space is a training-time choice** (joint
targets default; EE pose via `tcp_*`). Outcome `aborted` = process stopped
mid-episode; kept but should be excluded.

**About the terminal label.** The press comes *after* the event it labels (human
reaction time, ~0.3–1 s). Treat the label as "this episode ended in X", with
the last steps carrying it — which is exactly the signal a termination/success
head needs. Don't treat `terminal` as the exact success instant.

## Cameras

```bash
.venv/bin/python -m pick_place_SO101.run_teleop --backend real --port ... --cameras
```

- **wrist** -> phone UI (visible after Start AR). **bench** -> `https://localhost:8443/bench.html`
  on the Mac (`/cams.html` is the same page, bench only).
- Captured via AVFoundation, selected by macOS **Unique ID** (`config.CAMERA_UIDS`),
  not OpenCV index (the two openaicams share a name and indices move). A UVC Unique ID
  encodes the USB **port**: swap the cables between ports and wrist/bench swap too.
- Frames of both cameras are written with every recorded step
  (`frames/<cam>/NNNNNN.jpg`, 1280x720 JPEG ~115 KB, ~7 MB/s). `--cam-size 640x480` to shrink.
- Not done: exposure/focus lock (AVFoundation controls exist, not wired), per-frame capture
  timestamps (frames are the latest at each 30 Hz tick, up to ~33 ms stale).
- Tools: `tools/check_cameras.py` (OpenCV indices), `tools/view_cameras.py` (OpenCV preview).

## First bring-up on the real arm (do in this order)

1. `--selftest` passes.
2. `tools/check_mapping.py --port …` (read-only, arm limp): confirm the model's
   zero matches the real arm (see its docstring). If a joint reads an offset,
   put it in `joint_map.json`. **This is the one assumption I could not verify
   without the arm** — a few degrees of offset on `shoulder_pan` is harmless,
   on `wrist_flex`/`elbow_flex` it will make heights wrong.
3. `--backend sim`: run the phone → arm path with zero hardware risk; settle
   SET FWD / scale / lock-vs-free here.
4. `--backend real --max-speed 30`, **hand on the 12 V plug**, clutch on, small
   moves, then HOME.
5. Only then START an episode.

## Known limits / assumptions

- LeRobot degrees ≙ MJCF radians with identity sign/offset (the menagerie MJCF
  derives from the "new calibration" URDF). Verified for sign using your folded
  rest pose; **offset not verified** (step 2 above).
- Joint limits come from your calibration *sweep*. If you didn't reach the
  mechanical stops the workspace is a bit narrower than the arm's true one
  (re-run `lerobot-calibrate` with full sweeps to widen it).
- Gripper %→jaw angle in the twin is a linear map over the MJCF range; fine for
  visuals, don't read it as a calibrated jaw width.
- Servo P-gain stays at LeRobot's 16, so steady-state error grows with load
  (≈2° at the elbow measured). That is why `q` (measured) is logged alongside
  `q_cmd`.
- 30 Hz loop; the serial bus read+write budget is ~5–8 ms, plenty of headroom.

## Task planner and segmentation (`planner.py`, `segment.py`)

- `planner.plan(jpeg, history)` asks a VLM for `{verb,target,destination,instruction}`.
  Default backend is the HF Inference API: `export HF_TOKEN=hf_...`, optional
  `PLANNER_MODEL` (default `Qwen/Qwen3-VL-30B-A3B-Instruct`). `MockBackend` and
  `ManualBackend` work offline.
- `segment.segment(jpeg, names)` returns boolean masks (`None` if not found) using
  OWLv2 + SAM2 through `transformers` on MPS. Weights download on first use
  (~1 GB total); `MockBackend` is colour-threshold only.
- Offline tests: `.venv/bin/python -m pick_place_SO101.test_planner_segment`.

## Live highlighted bench view

`run_teleop --cameras --auto-tasks` also serves `https://<mac>:8443/bench.html` (`/cam/bench_hl.mjpg`): raw bench
frames when idle, then as soon as a task is planned Core ML EdgeTAM (Apple GPU,
~13 Hz for two objects, `bench_overlay.py` + `coreml_live_tracker.py`) following the target and destination.
Recorded frames stay raw. `--no-overlay` turns it off, `--track-backend pytorch` uses the slower MPS tracker.
Weights live in `models/EdgeTAM-CoreML-Video/` (gitignored). Standalone demo: `python -m pick_place_SO101.tools.live_track
--target "..." --dest "..."`. Rejected experiments (Apple Vision subject lift, hybrid box tracker, ACT→Core ML) are in `tools/coreml_eval/`.
