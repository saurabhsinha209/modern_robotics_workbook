"""Prompted re-segmentation tracker on Apple's Core ML SAM2.1-tiny (image-only model):
each update = image encoder + box prompt (from the previous mask's bbox, expanded) + decoder.
No memory bank, so it cannot follow an object through occlusion by memory -- that is the point
of the comparison. Evaluated against the offline EdgeTAM track on a recorded episode.

    python -m pick_place_SO101.tools.coreml_eval.hybrid_box_tracker <sam2_coreml_dir> [--stride 6] [--units CPU_AND_GPU]
"""
import argparse, glob, time
import numpy as np, cv2, coremltools as ct
from PIL import Image
from ...tracking import load_tracks

EP = "pick_place_SO101/data/20261016_222619/ep_0001"
U = {"ALL": ct.ComputeUnit.ALL, "CPU_AND_GPU": ct.ComputeUnit.CPU_AND_GPU, "CPU_AND_NE": ct.ComputeUnit.CPU_AND_NE}


class HybridBoxTracker:
    def __init__(self, d, units="CPU_AND_GPU", expand=0.2, mode="point"):
        self.mode = mode
        cu = U[units]
        ld = lambda n: ct.models.MLModel(f"{d}/SAM2_1Tiny{n}FLOAT16.mlpackage", compute_units=cu)
        self.enc, self.pe, self.dec = ld("ImageEncoder"), ld("PromptEncoder"), ld("MaskDecoder")
        self.expand = expand
        self.ms = {"enc": [], "prompt": [], "dec": []}

    def _box(self, mask):                      # mask HxW bool in frame coords -> box in 1024 space
        ys, xs = np.nonzero(mask)
        H, W = mask.shape
        x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
        w, h = max(x1 - x0, 6), max(y1 - y0, 6)
        x0, x1 = x0 - self.expand * w, x1 + self.expand * w
        y0, y1 = y0 - self.expand * h, y1 + self.expand * h
        return np.array([[x0 / W * 1024, y0 / H * 1024], [x1 / W * 1024, y1 / H * 1024]], np.float16)

    def step(self, bgr, prev_mask):
        H, W = bgr.shape[:2]
        img = Image.fromarray(cv2.cvtColor(cv2.resize(bgr, (1024, 1024)), cv2.COLOR_BGR2RGB))
        t0 = time.perf_counter(); e = self.enc.predict({"image": img}); t1 = time.perf_counter()
        if self.mode == "box":
            pts, lab = self._box(prev_mask)[None], np.array([[2, 3]], np.float16)
        else:                                   # centroid point, positive label
            ys, xs = np.nonzero(prev_mask)
            pts = np.array([[[xs.mean() / W * 1024, ys.mean() / H * 1024]]], np.float16)
            lab = np.array([[1]], np.float16)
        p = self.pe.predict({"points": pts, "labels": lab}); t2 = time.perf_counter()
        o = self.dec.predict({"image_embedding": e["image_embedding"], "sparse_embedding": p["sparse_embeddings"],
                              "dense_embedding": p["dense_embeddings"], "feats_s0": e["feats_s0"], "feats_s1": e["feats_s1"]})
        t3 = time.perf_counter()
        self.ms["enc"].append((t1 - t0) * 1e3); self.ms["prompt"].append((t2 - t1) * 1e3); self.ms["dec"].append((t3 - t2) * 1e3)
        sc = np.asarray(o["scores"]).reshape(-1)
        ms = [cv2.resize(np.asarray(o["low_res_masks"])[0, k], (W, H)) > 0 for k in range(3)]
        a0 = max(1, int(prev_mask.sum()))
        ok = [k for k in range(3) if 0.4 * a0 <= ms[k].sum() <= 2.5 * a0]    # reject whole-scene / collapsed masks
        k = max(ok, key=lambda j: sc[j]) if ok else 1
        return ms[k], float(sc[k])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("--stride", type=int, default=6)
    ap.add_argument("--units", default="CPU_AND_GPU"); ap.add_argument("--mode", default="point"); ap.add_argument("--start", type=int, default=840); a = ap.parse_args()
    fs = sorted(glob.glob(EP + "/frames/bench/*.jpg")); tr = load_tracks(EP)["target"]
    H, W = tr.shape[1:]
    tk = HybridBoxTracker(a.dir, a.units, mode=a.mode)
    cur = tr[a.start].copy(); ious, cds, rows = [], [], []
    tk.step(cv2.imread(fs[a.start]), cur)       # warm-up
    for k in range(a.start + a.stride, len(fs), a.stride):
        m, s = tk.step(cv2.imread(fs[k]), cur)
        if m.sum() > 10: cur = m   # (never accept an empty mask as the next prompt)
        ref = tr[k]
        inter = (m & ref).sum(); iou = inter / max(1, (m | ref).sum()); ious.append(iou)
        c = lambda x: np.array(np.nonzero(x)[::-1]).mean(1) if x.any() else np.array([np.nan, np.nan])
        cds.append(np.linalg.norm(c(m) - c(ref)))
        if (k - a.start) % 60 == 0: rows.append(f"f{k}: IoU {iou:.2f} centroid-err {cds[-1]:.0f}px score {s:.2f} area {int(m.sum())}/{int(ref.sum())}")
    print("\n".join(rows))
    ms = {k: np.median(v) for k, v in tk.ms.items()}
    print(f"[{a.units} {a.mode}] median ms enc/prompt/dec = {ms['enc']:.1f}/{ms['prompt']:.1f}/{ms['dec']:.1f}  total {sum(ms.values()):.1f} ms/update  "
          f"| vs offline EdgeTAM: mean IoU {np.nanmean(ious):.2f}, frames IoU<0.3: {int((np.array(ious)<0.3).sum())}/{len(ious)}, "
          f"median centroid err {np.nanmedian(cds):.1f}px, final err {cds[-1]:.0f}px")


if __name__ == "__main__":
    main()
