"""Selftest: .venv/bin/python -m pick_place_SO101.test_convert"""
import json
import pathlib
import tempfile

import numpy as np
from PIL import Image

from .convert_lerobot import convert, make_actions, rot6d_to_R, rotvec
from .episodes import rot6d
from .highlight import apply_highlight

ok = True


def check(name, cond):
    global ok
    ok &= bool(cond)
    print(("PASS " if cond else "FAIL ") + name)


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def synth(root, n=60, cams=("wrist", "bench"), full=(320, 240), masks=True):
    ep = pathlib.Path(root) / "s" / "ep_0000"
    ep.mkdir(parents=True)
    t = np.arange(n) / 30.0
    p = np.stack([0.2 + 0.001 * np.arange(n), np.zeros(n), 0.1 * np.ones(n)], 1)
    R = [Rz(0.01 * i) for i in range(n)]
    q_cmd = np.stack([np.arange(n) * 0.5] * 5 + [np.full(n, 30.0)], 1)
    homing = np.zeros(n, bool)
    homing[20:23] = True
    clutch = np.ones(n, bool)
    clutch[:8] = False
    clutch[-8:] = False
    q_cmd[:8] = q_cmd[0]
    q_cmd[-8:] = q_cmd[-9]
    z = dict(t=t, q=q_cmd + 0.5, q_cmd=q_cmd, tcp_p=p, tcp_rot6d=np.stack([rot6d(r) for r in R]),
             clutch=clutch, homing=homing, stale=np.zeros(n, bool))
    np.savez_compressed(ep / "steps.npz", **z)
    json.dump({"episode": 0, "instruction": "pick the cube", "outcome": "success", "n_steps": n,
               "cameras": list(cams), "task": {"instruction": "pick the cube"}}, open(ep / "meta.json", "w"))
    rng = np.random.default_rng(0)
    for c in cams:
        (ep / "frames" / c).mkdir(parents=True)
        for i in range(n):
            Image.fromarray(rng.integers(0, 255, (full[1], full[0], 3), dtype=np.uint8)).save(
                ep / "frames" / c / f"{i:06d}.jpg")
    if masks:
        (ep / "masks").mkdir()
        m = np.zeros((full[1], full[0]), np.uint8)
        m[:full[1] // 2, :full[0] // 2] = 255
        Image.fromarray(m).save(ep / "masks" / "target.png")
        Image.fromarray(m[::-1, ::-1].copy()).save(ep / "masks" / "dest.png")
    return ep.parent, z


# math
R = Rz(0.3)
check("rot6d roundtrip", np.allclose(rot6d_to_R(rot6d(R)), R))
check("rotvec", np.allclose(rotvec(Rz(0.4)[None])[0], [0, 0, 0.4]) and np.allclose(rotvec(np.eye(3)[None]), 0))

# highlight
img = np.full((20, 30, 3), 100, np.uint8)
m = np.zeros((20, 30), np.uint8)
m[:10] = 255
check("highlight no masks = identity", apply_highlight(img, None, None) is img)
o = apply_highlight(img, m, None)
check("highlight target brighter / bg dimmer", o[0, 0].mean() > 100 > o[-1, 0].mean())
check("highlight deterministic", np.array_equal(o, apply_highlight(img, m, None)))

with tempfile.TemporaryDirectory() as d:
    sess, z = synth(d)
    zz = {k: np.asarray(v) for k, v in z.items()}
    a_body = make_actions(zz, "ee_body")
    a_sp = make_actions(zz, "ee_space")
    check("ee_body dp is +x in body, rot z", np.allclose(a_body[5, :3], Rz(-0.04) @ [0.001, 0, 0], atol=1e-9)
          and np.allclose(a_body[5, 3:6], [0, 0, 0.01], atol=1e-9))
    check("ee_space rot z, gripper abs", np.allclose(a_sp[5, 5], 0.01) and np.allclose(a_sp[:, 6], 30.0))
    from pick_place_SO101.convert_lerobot import keep_mask
    zh = dict(zz); zh["clutch"] = np.zeros(len(zz["t"]), bool); zh["homing"] = np.zeros(len(zz["t"]), bool)
    zh["homing"][30:34] = True; zh["q_cmd"] = np.zeros_like(zz["q_cmd"])
    kh = keep_mask(zh)
    check("homing ticks kept + count as non-idle", kh[30:34].all() and not kh[:10].any() and kh[30 - 5] and not kh[30 - 8])
    zs = dict(zh); zs["stale"] = np.zeros(len(zz["t"]), bool); zs["stale"][32] = True
    ks = keep_mask(zs)
    check("stale tick and the one after dropped", not ks[32] and not ks[33] and ks[31])
    for kind, dim in (("joint", 6), ("ee_body", 7), ("ee_space", 7)):
        out = pathlib.Path(d) / f"out_{kind}"
        st = convert([sess], out, kind, "sam", (64, 48), verbose=False)
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        ds = LeRobotDataset("local/so101", root=out)
        item = ds[0]
        check(f"{kind}: frames/shape", st["episodes"] == 1 and ds.num_frames == st["frames"] < 60
              and tuple(item["action"].shape) == (dim,) and tuple(item["observation.state"].shape) == (6,)
              and tuple(item["observation.images.wrist"].shape) == (3, 48, 64))
        print("   ", kind, st)
print("ALL PASS" if ok else "FAILURES")
raise SystemExit(0 if ok else 1)
