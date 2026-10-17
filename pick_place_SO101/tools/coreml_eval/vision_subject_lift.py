"""Apple Vision foreground-instance masks (the 'Subject Lifting' API, macOS 14+): latency and whether it
isolates the target on a recorded bench frame. Class-agnostic and prompt-free: it returns every salient
instance; we pick the instance under the previous centroid (instanceAtPoint-style via the label map).
    python -m pick_place_SO101.tools.coreml_eval.vision_subject_lift"""
import glob, time
import numpy as np, cv2, objc
import Vision, Quartz
from Foundation import NSData
from ...tracking import load_tracks

EP = "pick_place_SO101/data/20261016_222619/ep_0001"


def instances(bgr):
    ok, buf = cv2.imencode(".jpg", bgr)
    data = NSData.dataWithBytes_length_(buf.tobytes(), len(buf))
    h = Vision.VNImageRequestHandler.alloc().initWithData_options_(data, {})
    req = Vision.VNGenerateForegroundInstanceMaskRequest.alloc().init()
    ok, err = h.performRequests_error_([req], None)
    if not ok or not req.results(): return None
    obs = req.results()[0]
    idx = obs.allInstances()
    pb, err = obs.generateScaledMaskForImageForInstances_fromRequestHandler_error_(idx, h, None)   # union soft mask
    # label map (instance id per pixel) at the mask resolution
    lab = obs.instanceMask()
    Quartz.CVPixelBufferLockBaseAddress(lab, 1)
    w, hh, bpr = Quartz.CVPixelBufferGetWidth(lab), Quartz.CVPixelBufferGetHeight(lab), Quartz.CVPixelBufferGetBytesPerRow(lab)
    base = Quartz.CVPixelBufferGetBaseAddress(lab)
    arr = np.frombuffer(base.as_buffer(bpr * hh), np.uint8).reshape(hh, bpr)[:, :w].copy()
    Quartz.CVPixelBufferUnlockBaseAddress(lab, 1)
    return arr, sorted(int(i) for i in idx)


def main():
    fs = sorted(glob.glob(EP + "/frames/bench/*.jpg")); tr = load_tracks(EP)["target"]
    H, W = tr.shape[1:]
    ts, hits, ious, nin = [], 0, [], []
    for k in range(840, 1228, 12):
        bgr = cv2.imread(fs[k]); t = time.perf_counter(); r = instances(bgr); ts.append((time.perf_counter() - t) * 1e3)
        if r is None: nin.append(0); continue
        lab, ids = r; nin.append(len(ids))
        lab = cv2.resize(lab, (W, H), interpolation=cv2.INTER_NEAREST)
        ref = tr[k]
        if not ref.any(): continue
        best = max(ids, key=lambda i: ((lab == i) & ref).sum() / max(1, ((lab == i) | ref).sum()), default=None)
        if best is not None:
            m = lab == best; iou = (m & ref).sum() / max(1, (m | ref).sum()); ious.append(iou); hits += iou > 0.5
        if k % 120 == 0: print(f"f{k}: instances {ids}  best-instance IoU vs target track {ious[-1]:.2f}")
    print(f"Vision subject lifting @{W}x{H} jpeg: median {np.median(ts[1:]):.0f} ms/frame (incl. jpeg encode); "
          f"instances/frame {np.mean(nin):.1f}; target isolated as ONE instance (IoU>0.5) on {hits}/{len(ious)} frames; "
          f"best-instance IoU mean {np.mean(ious):.2f}")


if __name__ == "__main__":
    main()
