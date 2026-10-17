"""Core ML EdgeTAM video tracker (community port AhmadYarAI/EdgeTAM-CoreML-Video, Apache-2.0).

The port exposes 4 models and leaves the memory bank to the app. Wiring below is inferred
from the model graphs (input shapes + ops), NOT from documentation -- validated by IoU against
the PyTorch EdgeTAM offline track:
  spatial_bank       (1,7,512,64)  memory_features of up to 7 frames: [cond, 6 most recent]
  spatial_positions  (1,7,512,64)  memory_positions + temporal_positions[slot]   (tpos from the mem encoder)
  pointer_bank       (1,16,256)    object pointers (cond + recent), zero padded
  attention_bias     (1,1,1,3648)  0 = attend, -1e4 = padding   (7*512 spatial + 16*4 pointer tokens)
  rotary_weight      (1,1792)      per 256 RoPE tokens x 7 slots: 1 = apply RoPE (valid slot), 0 = pad

    python -m pick_place_SO101.coreml_edgetam <models_dir> [--stride 6] [--units CPU_AND_GPU]
"""
import argparse, glob, time
import numpy as np, cv2, coremltools as ct
from PIL import Image
from .tracking import load_tracks

EP = "pick_place_SO101/data/20261016_222619/ep_0001"
U = {"ALL": ct.ComputeUnit.ALL, "CPU_AND_GPU": ct.ComputeUnit.CPU_AND_GPU, "CPU_AND_NE": ct.ComputeUnit.CPU_AND_NE,
     "CPU_ONLY": ct.ComputeUnit.CPU_ONLY}
NEG = -1e4


class EdgeTamCoreML:
    def __init__(self, d, units="CPU_AND_GPU"):
        cu = U[units]
        ld = lambda n: ct.models.MLModel(f"{d}/EdgeTAMVideo{n}.mlpackage", compute_units=cu)
        self.enc, self.init, self.menc, self.prop = ld("ImageEncoder"), ld("Initializer"), ld("MemoryEncoder"), ld("Propagator")
        self.t = {"enc": [], "prop": [], "total": []}

    def _enc(self, bgr):
        img = Image.fromarray(cv2.cvtColor(cv2.resize(bgr, (1024, 1024)), cv2.COLOR_BGR2RGB))
        return self.enc.predict({"image": img})

    def seed(self, bgr, mask):
        """Condition on a first-frame mask (centroid click for the pointer; the mask itself for memory)."""
        H, W = mask.shape
        f = self._enc(bgr)
        ys, xs = np.nonzero(mask)
        pt = np.array([[[xs.mean() / W * 1024, ys.mean() / H * 1024]]], np.float16)
        i = self.init.predict({"initial_vision_features": f["initial_vision_features"], "high_res_feature_0": f["high_res_feature_0"],
                               "high_res_feature_1": f["high_res_feature_1"], "point_coords": pt, "point_labels": np.array([[1]], np.int32)})
        hr = cv2.resize(mask.astype(np.float32), (1024, 1024), interpolation=cv2.INTER_NEAREST)
        hr = (np.where(hr > 0, 10.0, -10.0)[None, None]).astype(np.float16)     # seed mask as logits
        m = self.menc.predict({"raw_vision_features": f["raw_vision_features"], "high_res_mask": hr,
                               "object_score": np.array([[10.0]], np.float16)})
        self.tpos = np.asarray(m["temporal_positions"])                          # (7,64)
        self.cond = (np.asarray(m["memory_features"])[0], np.asarray(m["memory_positions"])[0])
        self.cond_ptr = np.asarray(i["object_pointer"])[0]
        self.recent, self.ptrs = [], []                                         # newest last

    def _bank(self):
        sb = np.zeros((1, 7, 512, 64), np.float16); sp = np.zeros_like(sb)
        bias = np.full((1, 1, 1, 3648), NEG, np.float16); rw = np.zeros((1, 1792), np.float16)
        slots = [(self.cond, 6)] + [(r, 6 - d) for d, r in zip(range(len(self.recent), 0, -1), self.recent)]
        for k, ((mf, mp), ti) in enumerate(slots[:7]):
            sb[0, k] = mf; sp[0, k] = mp + self.tpos[ti][None]
            bias[0, 0, 0, k * 512:(k + 1) * 512] = 0; rw[0, k * 256:(k + 1) * 256] = 1
        pb = np.zeros((1, 16, 256), np.float16)
        ptrs = [self.cond_ptr] + self.ptrs[-15:]
        for k, p in enumerate(ptrs):
            pb[0, k] = p; bias[0, 0, 0, 3584 + k * 4:3584 + (k + 1) * 4] = 0
        return sb, sp, pb, bias, rw

    def step(self, bgr):
        H, W = bgr.shape[:2]
        t0 = time.perf_counter(); f = self._enc(bgr); t1 = time.perf_counter()
        sb, sp, pb, bias, rw = self._bank()
        o = self.prop.predict({"raw_vision_features": f["raw_vision_features"], "high_res_feature_0": f["high_res_feature_0"],
                               "high_res_feature_1": f["high_res_feature_1"], "spatial_bank": sb, "spatial_positions": sp,
                               "pointer_bank": pb, "attention_bias": bias, "rotary_weight": rw})
        t2 = time.perf_counter()
        score = float(np.asarray(o["object_score"]).reshape(-1)[0])
        self.recent.append((np.asarray(o["memory_features"])[0], np.asarray(o["memory_positions"])[0])); self.recent = self.recent[-6:]
        self.ptrs.append(np.asarray(o["object_pointer"])[0])
        mask = cv2.resize(np.asarray(o["low_res_mask"])[0, 0], (W, H), interpolation=cv2.INTER_LINEAR) > 0
        self.t["enc"].append((t1 - t0) * 1e3); self.t["prop"].append((t2 - t1) * 1e3); self.t["total"].append((time.perf_counter() - t0) * 1e3)
        return mask, score


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("--stride", type=int, default=6)
    ap.add_argument("--units", default="CPU_AND_GPU"); ap.add_argument("--start", type=int, default=840); a = ap.parse_args()
    fs = sorted(glob.glob(EP + "/frames/bench/*.jpg")); tr = load_tracks(EP)["target"]
    tk = EdgeTamCoreML(a.dir, a.units); tk.seed(cv2.imread(fs[a.start]), tr[a.start])
    ious, cds, rows = [], [], []
    c = lambda x: np.array(np.nonzero(x)[::-1]).mean(1) if x.any() else np.array([np.nan, np.nan])
    for k in range(a.start + a.stride, len(fs), a.stride):
        m, s = tk.step(cv2.imread(fs[k])); ref = tr[k]
        iou = (m & ref).sum() / max(1, (m | ref).sum()); ious.append(iou); cds.append(np.linalg.norm(c(m) - c(ref)))
        if (k - a.start) % 60 == 0: rows.append(f"f{k}: IoU {iou:.2f} centroid-err {cds[-1]:.0f}px score {s:.1f} area {int(m.sum())}/{int(ref.sum())}")
    print("\n".join(rows))
    md = {k: np.median(v[1:]) for k, v in tk.t.items()}
    print(f"[{a.units}] median ms/update: encoder {md['enc']:.1f}  propagator {md['prop']:.1f}  total {md['total']:.1f} (1 object, excl. cv2 decode)"
          f" | vs PyTorch-EdgeTAM offline: mean IoU {np.nanmean(ious):.2f}, IoU<0.3 on {int((np.array(ious)<0.3).sum())}/{len(ious)}, "
          f"median centroid err {np.nanmedian(cds):.1f}px, final {cds[-1]:.0f}px")


if __name__ == "__main__":
    main()
