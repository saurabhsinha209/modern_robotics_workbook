"""Per-frame highlight masks: SAM2 video tracking seeded from the frame-0 masks.

    track_episode(ep_dir) -> writes masks/track_target.npz, masks/track_dest.npz
    load_tracks(ep_dir)   -> {"target": (N,H,W) bool | None, "dest": ...}

The seed masks (masks/target.png, dest.png) are tracked through the stored bench frames
(frames/bench/*.jpg) as separate SAM2 objects. Frames where SAM2 says the object is absent
(object score < 0) or the mask collapses (< MIN_FRAC of the seed area) are stored EMPTY
rather than as garbage; if the object reappears it is accepted again.

npz layout: bits = np.packbits(masks.reshape(N,-1), axis=1), shape = (N,H,W).
"""
import os
import pathlib
import time

os.environ.setdefault("HF_HUB_DISABLE_IMPLICIT_TOKEN", "1")   # a stale cached login would 401 public downloads

import cv2
import numpy as np

CHUNK = 100         # tracked frames per session (1024^2 fp32 frames: ~12.6 MB each)
MODEL = "yonigozlan/EdgeTAM-hf"   # fast default; "facebook/sam2.1-hiera-small" also works
STRIDE = 3           # run the tracker on every Nth frame, hold the mask between
MIN_FRAC = 0.15      # mask area below this fraction of the seed area == lost


def _device():
    import torch
    return "mps" if torch.backends.mps.is_available() else ("cuda" if torch.cuda.is_available() else "cpu")


def pack(masks):
    n = masks.shape[0]
    return dict(bits=np.packbits(masks.reshape(n, -1), axis=1), shape=np.array(masks.shape))


def unpack(z):
    shape = tuple(int(s) for s in z["shape"])
    n = shape[0]
    return np.unpackbits(z["bits"], axis=1)[:, :shape[1] * shape[2]].reshape(shape).astype(bool)


def load_tracks(ep_dir):
    md = pathlib.Path(ep_dir) / "masks"
    out = {}
    for k in ("target", "dest"):
        f = md / f"track_{k}.npz"
        out[k] = unpack(np.load(f)) if f.exists() else None
    return out


def clean_seed(m):
    """Largest connected component + small morphological close (fills SAM speckle)."""
    m = (np.asarray(m) > 0).astype(np.uint8)
    if m.sum() == 0:
        return m.astype(bool)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
    if n > 2:
        m = (lab == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))).astype(np.uint8)
    return m.astype(bool)


def _load(model_id, dev):
    """EdgeTAM and SAM2 share the video API; cached weights first (no network)."""
    from transformers import AutoProcessor, EdgeTamVideoModel, Sam2VideoModel
    cls = EdgeTamVideoModel if "edgetam" in model_id.lower() else Sam2VideoModel
    try:
        proc = AutoProcessor.from_pretrained(model_id, local_files_only=True)
        model = cls.from_pretrained(model_id, local_files_only=True)
    except Exception:
        proc = AutoProcessor.from_pretrained(model_id)
        model = cls.from_pretrained(model_id)
    return proc, model.to(dev).eval()


def track_episode(ep_dir, cam="bench", model_id=MODEL, device=None, stride=STRIDE,
                  chunk=None, profile=False, log=print):
    """Track the seed masks through the episode. Only every `stride`-th frame is run through the
    model; each tracked mask is held until the next tracked frame."""
    import torch
    from PIL import Image

    ep = pathlib.Path(ep_dir)
    md = ep / "masks"
    files = sorted((ep / "frames" / cam).glob("*.jpg"))
    if not files:
        raise FileNotFoundError(f"no frames in {ep / 'frames' / cam}")
    seeds = {}
    for oid, k in ((1, "target"), (2, "dest")):
        f = md / f"{k}.png"
        if f.exists():
            m = clean_seed(cv2.imread(str(f), 0))
            if m.any():
                seeds[oid] = (k, m)
    if not seeds:
        raise FileNotFoundError(f"no seed masks in {md}")
    t0 = time.time()
    dev = device or _device()
    proc, model = _load(model_id, dev)
    n = len(files)
    H, W = cv2.imread(str(files[0]), 0).shape
    tidx = list(range(0, n, max(1, stride)))              # frames actually run through the model
    if tidx[-1] != n - 1:
        tidx.append(n - 1)
    out = {k: np.zeros((n, H, W), bool) for k, _ in seeds.values()}
    area0 = {oid: int(m.sum()) for oid, (k, m) in seeds.items()}
    lost = {k: 0 for k, _ in seeds.values()}
    cur = {oid: m for oid, (k, m) in seeds.items()}      # seed for the next chunk (last good mask)
    chunk = chunk or CHUNK                                # in TRACKED frames
    prof = {"init": 0.0, "model": 0.0, "post": 0.0}
    last_ok = {}
    for c0 in range(0, len(tidx), chunk):
        ids_c = tidx[c0:c0 + chunk + 1]                   # 1 frame overlap with the next chunk
        t = time.time()
        sess = proc.init_video_session(video=[Image.open(files[i]).convert("RGB") for i in ids_c],
                                       inference_device=dev, processing_device="cpu",
                                       video_storage_device="cpu", inference_state_device="cpu",
                                       dtype=torch.float32)
        ids = sorted(cur)           # all objects in ONE call (separate calls break the memory bank)
        proc.add_inputs_to_inference_session(sess, frame_idx=0, obj_ids=ids,
                                             input_masks=[cur[i].astype(np.uint8) for i in ids],
                                             original_size=(H, W))
        prof["init"] += time.time() - t
        t = time.time()
        with torch.inference_mode():
            model(inference_session=sess, frame_idx=0)    # runs the memory encoder on the seed masks
            it = model.propagate_in_video_iterator(sess, start_frame_idx=0)
            while True:
                t = time.time()
                o = next(it, None)
                if o is None:
                    break
                if dev == "mps":
                    torch.mps.synchronize()           # MPS is async: otherwise the cost lands in "post"
                prof["model"] += time.time() - t
                t = time.time()
                ri = ids_c[o.frame_idx]
                nxt = tidx[c0 + o.frame_idx + 1] if c0 + o.frame_idx + 1 < len(tidx) else n
                hold = slice(ri, nxt) if o.frame_idx < len(ids_c) - 1 or ids_c[-1] == n - 1 else slice(ri, ri)
                last_of_chunk = o.frame_idx == len(ids_c) - 1 and len(ids_c) > chunk
                if last_of_chunk:
                    hold = slice(ri, ri)                  # belongs to the next chunk
                masks = proc.post_process_masks([o.pred_masks], original_sizes=[[H, W]], binarize=True)[0]
                sc = o.object_score_logits
                sc = (sc.reshape(len(o.object_ids), -1)[:, 0].float().cpu().numpy()
                      if sc is not None else np.ones(len(o.object_ids)))
                for j, oid in enumerate(o.object_ids):
                    k = seeds[oid][0]
                    mk = masks[j, 0].cpu().numpy().astype(bool)
                    if sc[j] < 0 or mk.sum() < MIN_FRAC * area0[oid]:
                        lost[k] += hold.stop - hold.start
                        continue
                    out[k][hold] = mk
                    cur[oid] = mk
                prof["post"] += time.time() - t
        del sess
        log(f"[track] {min(tidx[min(c0 + chunk, len(tidx) - 1)] + 1, n)}/{n}  {time.time() - t0:.0f}s")
    for k, arr in out.items():
        np.savez_compressed(md / f"track_{k}.npz", **pack(arr))
    dt = time.time() - t0
    log(f"[track] {ep.name}: {n} frames ({len(tidx)} tracked, stride {stride}) in {dt:.0f}s; "
        f"empty frames {lost}; time init {prof['init']:.0f}s model {prof['model']:.0f}s post {prof['post']:.0f}s")
    return {"seconds": dt, "frames": n, "lost": lost, "profile": prof}
