"""Mask highlight ("illumination trick") — pure, deterministic, usable at train AND inference.

    apply_highlight(img_rgb, target_mask, dest_mask) -> img_rgb (uint8)

Target gets brightened, destination gets a blue-green tint, everything else is dimmed so
the two task-relevant regions pop out. Masks are optional (None -> that region is skipped;
both None -> image returned unchanged). Masks are (h, w) arrays, >0 / True = inside; if
their size differs from the image they are nearest-resized.
"""
import numpy as np

DIM = 0.2           # background brightness factor
TARGET_GAIN = 1.35  # target brightening
DEST_TINT = np.array([0.0, 200.0, 255.0])
DEST_ALPHA = 0.35


def _fit(mask, h, w):
    m = np.asarray(mask)
    if m.ndim == 3:
        m = m[..., 0]
    if m.shape != (h, w):
        ys = (np.arange(h) * m.shape[0] // h).clip(0, m.shape[0] - 1)
        xs = (np.arange(w) * m.shape[1] // w).clip(0, m.shape[1] - 1)
        m = m[ys][:, xs]
    return m > 0


def apply_highlight(img_rgb, target_mask=None, dest_mask=None):
    if target_mask is None and dest_mask is None:
        return img_rgb
    img = np.asarray(img_rgb)
    h, w = img.shape[:2]
    out = img.astype(np.float32)
    t = _fit(target_mask, h, w) if target_mask is not None else np.zeros((h, w), bool)
    d = _fit(dest_mask, h, w) if dest_mask is not None else np.zeros((h, w), bool)
    d = d & ~t
    bg = ~(t | d)
    out[bg] *= DIM
    out[t] = out[t] * TARGET_GAIN
    out[d] = out[d] * (1 - DEST_ALPHA) + DEST_TINT * DEST_ALPHA
    return np.clip(out, 0, 255).astype(np.uint8)


def frame_masks(i, tracks=None, static=(None, None)):
    """(target, dest) masks for step i: tracked per-frame masks when present, else the static
    frame-0 masks. A tracked-but-lost (all-False) mask is passed as None so it isn't drawn."""
    out = []
    for k, st in zip(("target", "dest"), static):
        tr = None if tracks is None else tracks.get(k)
        if tr is not None:
            m = tr[min(i, len(tr) - 1)]
            out.append(m if m.any() else None)
        else:
            out.append(st)
    return tuple(out)
