"""Raw teleop episodes -> LeRobotDataset (v3, lerobot 0.6.0).

    python -m pick_place_SO101.convert_lerobot --data data/<session> [data/<session2> ...] \
        --out <dir> --action {joint,ee_body,ee_space} --highlight {none,sam} --size 320x240 \
        [--success-only] [--fps 30]

Observation: observation.state = MEASURED q (5 deg + gripper %), observation.images.<cam>.
Action (all from COMMANDED quantities, never measured q — see pickplace deploy-loop bug):
  joint     a[t] = q_cmd[t]  (absolute joints; the command issued on tick t, paired with obs q[t])
  ee_body   a[t] = [dp_body(3), rotvec(3), gripper_cmd]  where the delta is tcp[t-1] -> tcp[t]
            of the commanded TCP pose, expressed in the PREVIOUS TCP body frame
            (dp = R_prev^T (p_t - p_prev), rotvec = log(R_prev^T R_t))
  ee_space  same, expressed in the world/base frame (dp = p_t - p_prev, rotvec = log(R_t R_prev^T))
Delta actions are zero on the first tick of any kept run. Dropped ticks: `stale` and the tick
right after it (its delta would span the gap). `homing` ticks are KEPT as normal frames/actions
(HOME is intended recovery behaviour; deltas of the commanded tcp_* are fine across the joint-space
ramp) and count as non-idle. Idle head/tail (no clutch, no command motion, no homing) is trimmed
with a small pad. `--highlight sam` draws
the masks onto the BENCH frames (highlight.apply_highlight): per-frame tracked masks
<ep>/masks/track_{target,dest}.npz if present (run tools/track_episode.py), else the static
frame-0 <ep>/masks/{target,dest}.png. Episodes without masks are left unhighlighted.
"""
import argparse
import json
import pathlib
import shutil

import numpy as np
from PIL import Image

from . import config
from .highlight import apply_highlight, frame_masks

PAD = 5          # ticks kept around the active span
MOVE_EPS = 0.05  # deg/tick of q_cmd change counted as motion


def rot6d_to_R(r6):
    """Inverse of episodes.rot6d (first two columns, Gram-Schmidt)."""
    r6 = np.asarray(r6, float)
    a, b = r6[..., :3], r6[..., 3:6]
    a = a / np.linalg.norm(a, axis=-1, keepdims=True)
    b = b - (a * b).sum(-1, keepdims=True) * a
    b = b / np.linalg.norm(b, axis=-1, keepdims=True)
    return np.stack([a, b, np.cross(a, b)], axis=-1)  # columns a, b, c


def rotvec(R):
    """log of rotation matrices (N,3,3) -> (N,3)."""
    tr = np.clip((np.trace(R, axis1=-2, axis2=-1) - 1) / 2, -1, 1)
    ang = np.arccos(tr)
    v = np.stack([R[..., 2, 1] - R[..., 1, 2], R[..., 0, 2] - R[..., 2, 0],
                  R[..., 1, 0] - R[..., 0, 1]], -1) / 2.0  # = sin(ang) * axis
    s = np.sin(ang)
    k = np.where(s > 1e-6, ang / np.where(s > 1e-6, s, 1.0), 1.0)
    return v * k[..., None]


def make_actions(z, kind):
    q_cmd = z["q_cmd"].astype(np.float64)
    if kind == "joint":
        return q_cmd.copy()
    p = z["tcp_p"].astype(np.float64)
    R = rot6d_to_R(z["tcp_rot6d"])
    n = len(p)
    dp, rv = np.zeros((n, 3)), np.zeros((n, 3))
    if n > 1:
        Rp, Rc = R[:-1], R[1:]
        d = p[1:] - p[:-1]
        if kind == "ee_body":
            dp[1:] = np.einsum("nji,nj->ni", Rp, d)                       # Rp^T d
            rv[1:] = rotvec(np.einsum("nji,njk->nik", Rp, Rc))           # Rp^T Rc
        elif kind == "ee_space":
            dp[1:] = d
            rv[1:] = rotvec(np.einsum("nij,nkj->nik", Rc, Rp))           # Rc Rp^T
        else:
            raise ValueError(kind)
    return np.concatenate([dp, rv, q_cmd[:, 5:6]], 1)


def keep_mask(z):
    n = len(z["t"])
    drop = np.zeros(n, bool)
    if "stale" in z:
        drop |= z["stale"].astype(bool)
    drop[1:] |= drop[:-1].copy()  # tick after a dropped tick spans the gap
    q = z["q_cmd"].astype(float)
    mv = np.zeros(n, bool)
    mv[1:] = np.abs(np.diff(q, axis=0)).max(1) > MOVE_EPS
    mv |= z["clutch"].astype(bool)
    if "homing" in z:
        mv |= z["homing"].astype(bool)        # homing is real motion, never "idle"
    mv &= ~drop
    keep = np.zeros(n, bool)
    idx = np.flatnonzero(mv)
    if len(idx):
        keep[max(idx[0] - PAD, 0): idx[-1] + PAD + 1] = True
    return keep & ~drop


def _load_img(path, size):
    im = Image.open(path).convert("RGB")
    if im.size != size:
        im = im.resize(size, Image.BILINEAR)
    return np.asarray(im)


def _load_mask(path, size):
    if not path.exists():
        return None
    return np.asarray(Image.open(path).convert("L").resize(size, Image.NEAREST))


def find_episodes(paths):
    eps = []
    for p in map(pathlib.Path, paths):
        if (p / "steps.npz").exists():
            eps.append(p)
        else:
            eps += sorted(d for d in p.glob("ep_*") if (d / "steps.npz").exists())
    return eps


def convert(data, out, action="joint", highlight="none", size=(320, 240), success_only=False,
            fps=int(config.CONTROL_HZ), repo_id="local/so101", verbose=True):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    eps = find_episodes(data)
    metas = [json.loads((e / "meta.json").read_text()) for e in eps]
    sel = [(e, m) for e, m in zip(eps, metas)
           if m.get("cameras") and (not success_only or m["outcome"] == "success")
           and m["outcome"] != "aborted"]
    if not sel:
        raise SystemExit("no usable episodes (need frames; check --success-only)")
    cams = sorted(set.intersection(*(set(m["cameras"]) for _, m in sel)))
    if not cams:
        raise SystemExit("episodes share no common camera")
    w, h = size
    adim = 6 if action == "joint" else 7
    names = (["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
             if action == "joint" else ["dx", "dy", "dz", "rx", "ry", "rz", "gripper"])
    feats = {
        "observation.state": {"dtype": "float32", "shape": (6,), "names": names[:6] if action == "joint"
                              else ["q0", "q1", "q2", "q3", "q4", "gripper"]},
        "action": {"dtype": "float32", "shape": (adim,), "names": names},
    }
    for c in cams:
        feats[f"observation.images.{c}"] = {"dtype": "video", "shape": (h, w, 3),
                                            "names": ["height", "width", "channels"]}
    out = pathlib.Path(out)
    if out.exists():
        shutil.rmtree(out)
    ds = LeRobotDataset.create(repo_id, fps=fps, features=feats, root=out, robot_type="so101_follower",
                               use_videos=True)
    stats = dict(episodes=0, frames=0, highlighted=0, skipped=0, cams=cams)
    for e, m in sel:
        z = np.load(e / "steps.npz")
        keep = keep_mask(z)
        act = make_actions(z, action)
        idx = np.flatnonzero(keep)
        if len(idx) < 2:
            stats["skipped"] += 1
            continue
        # delta actions: zero at the first tick of each kept run
        if action != "joint":
            starts = np.flatnonzero(np.diff(np.r_[False, keep].astype(int)) == 1)
            act[starts, :6] = 0.0
        masks, tracks = (None, None), None
        if highlight == "sam":
            from .tracking import load_tracks
            masks = (_load_mask(e / "masks" / "target.png", size), _load_mask(e / "masks" / "dest.png", size))
            tracks = load_tracks(e)
            if tracks["target"] is None and tracks["dest"] is None:
                tracks = None
            if masks[0] is None and masks[1] is None and tracks is None:
                masks = None
            else:
                stats["highlighted"] += 1
                stats["tracked"] = stats.get("tracked", 0) + (tracks is not None)
        task = (m.get("task") or {}).get("instruction") or m.get("instruction") or ""
        for i in idx:
            fr = {"observation.state": z["q"][i].astype(np.float32), "action": act[i].astype(np.float32),
                  "task": task}
            for c in cams:
                img = _load_img(e / "frames" / c / f"{i:06d}.jpg", size)
                if masks and c == "bench":      # masks live in bench-camera pixels
                    img = apply_highlight(img, *frame_masks(i, tracks, masks))
                fr[f"observation.images.{c}"] = img
            ds.add_frame(fr)
        ds.save_episode(parallel_encoding=False)
        stats["episodes"] += 1
        stats["frames"] += len(idx)
        if verbose:
            print(f"[convert] {e.parent.name}/{e.name}: {m['n_steps']} -> {len(idx)} frames ({m['outcome']})")
    ds.finalize()
    if verbose:
        print(f"[convert] done: {stats}  action_dim={adim}  img={h}x{w}  out={out}")
    return stats


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--data", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--action", choices=["joint", "ee_body", "ee_space"], default="joint")
    ap.add_argument("--highlight", choices=["none", "sam"], default="none")
    ap.add_argument("--size", default="320x240")
    ap.add_argument("--success-only", action="store_true")
    ap.add_argument("--fps", type=int, default=int(config.CONTROL_HZ))
    a = ap.parse_args(argv)
    w, h = map(int, a.size.lower().split("x"))
    convert(a.data, a.out, a.action, a.highlight, (w, h), a.success_only, a.fps)


if __name__ == "__main__":
    main()
