"""Open-vocabulary segmentation: names -> boolean masks.

    segment(jpeg_bytes, names, backend=None) -> {name: bool HxW ndarray | None}

Default backend: OWLv2 (text-prompted boxes) + SAM2 (box -> mask), both via
`transformers`, run locally on MPS/CUDA/CPU. Models are imported and loaded
lazily and cached. The first call downloads weights (~0.6 GB OWLv2 + ~0.2-0.4 GB
SAM2-small). Override ids with SEG_DETECTOR / SEG_SAM env vars.
MockBackend (colour-name box/threshold) needs no models or network.
"""
import io
import os

import numpy as np


def decode(jpeg):
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB"))


def masks_to_png(mask):
    from PIL import Image
    buf = io.BytesIO()
    Image.fromarray((np.asarray(mask) > 0).astype(np.uint8) * 255, "L").save(buf, "PNG")
    return buf.getvalue()


def _device():
    import torch
    if torch.backends.mps.is_available():
        return "mps"
    return "cuda" if torch.cuda.is_available() else "cpu"


class OwlSamBackend:
    def __init__(self, detector=None, sam=None, score_thresh=0.15):
        self.det_id = detector or os.environ.get("SEG_DETECTOR", "google/owlv2-base-patch16-ensemble")
        self.sam_id = sam or os.environ.get("SEG_SAM", "facebook/sam2.1-hiera-small")
        self.thresh, self._m = score_thresh, None

    def _load(self):
        if self._m is None:
            import torch
            from transformers import (AutoProcessor, Owlv2ForObjectDetection,
                                      Sam2Model, Sam2Processor)
            dev = _device()
            self._m = dict(
                dev=dev,
                dp=AutoProcessor.from_pretrained(self.det_id),
                dm=Owlv2ForObjectDetection.from_pretrained(self.det_id).to(dev).eval(),
                sp=Sam2Processor.from_pretrained(self.sam_id),
                sm=Sam2Model.from_pretrained(self.sam_id).to(dev).eval())
        return self._m

    def __call__(self, jpeg, names):
        import torch
        from PIL import Image
        m = self._load()
        img = Image.open(io.BytesIO(jpeg)).convert("RGB")
        W, H = img.size
        side = max(W, H)     # OWLv2 pads to a square: boxes are relative to that
        with torch.no_grad():
            inp = m["dp"](text=[[f"a photo of a {n}" for n in names]], images=img,
                          return_tensors="pt").to(m["dev"])
            out = m["dm"](**inp)
        res = m["dp"].post_process_grounded_object_detection(
            out, threshold=self.thresh, target_sizes=[(side, side)])[0]
        result = {}
        for i, n in enumerate(names):
            sel = (res["labels"] == i).nonzero().flatten()
            if len(sel) == 0:
                result[n] = None
                continue
            j = sel[res["scores"][sel].argmax()]
            box = res["boxes"][j].clamp(min=0).tolist()
            box = [box[0], box[1], min(box[2], W), min(box[3], H)]
            si = m["sp"](images=img, input_boxes=[[box]], return_tensors="pt").to(m["dev"])
            with torch.no_grad():
                so = m["sm"](**si, multimask_output=False)
            mk = m["sp"].post_process_masks(so.pred_masks.cpu(), si["original_sizes"].cpu())[0]
            result[n] = mk[0, 0].numpy().astype(bool)
        return result


COLORS = {"red": (200, 40, 40), "green": (40, 160, 70), "blue": (40, 70, 200),
          "pink": (230, 100, 170), "yellow": (230, 210, 40), "orange": (240, 140, 30)}


class MockBackend:
    """Colour-word threshold: 'green boat' -> pixels near green. None if too few."""
    def __init__(self, tol=70, min_px=20):
        self.tol, self.min_px = tol, min_px

    def __call__(self, jpeg, names):
        img = decode(jpeg).astype(int)
        out = {}
        for n in names:
            col = next((c for w, c in COLORS.items() if w in n.lower().split()), None)
            if col is None:
                out[n] = None
                continue
            mk = np.linalg.norm(img - np.array(col), axis=2) < self.tol
            out[n] = mk if mk.sum() >= self.min_px else None
        return out


_default = None


def segment(jpeg_bytes, names, backend=None):
    global _default
    if backend is None:
        _default = _default or OwlSamBackend()
        backend = _default
    return backend(jpeg_bytes, list(names))
