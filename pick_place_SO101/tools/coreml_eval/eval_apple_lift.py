"""Evaluate Apple Subject Lifting as a bench-camera mask tracker.
   python -m pick_place_SO101.tools.coreml_eval.eval_apple_lift {latency,sheet,track} ..."""
import argparse, glob, os, resource, time
import cv2, numpy as np
from .apple_lift import AppleLift
from ...tracking import load_tracks

D = "pick_place_SO101/data"
EP_A = f"{D}/20261016_222619/ep_0001"     # 320x240, tracked
EP_B = f"{D}/20261016_233015/ep_0000"     # 448x336, frames_full 1280x720
OUT = "/private/tmp/claude-502/-Users-Saurabh-Desktop-studies-books-modern-robotics/eb1dd9fa-23fd-4a38-8183-6c4d6d590230/scratchpad"
PAL = [(230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200), (245, 130, 48), (145, 30, 180),
       (70, 240, 240), (240, 50, 230), (210, 245, 60), (250, 190, 212), (0, 128, 128), (170, 110, 40)]


def frames(ep, sub="frames/bench"):
    return sorted(glob.glob(f"{ep}/{sub}/*.jpg"))


def cpu():
    r = resource.getrusage(resource.RUSAGE_SELF)
    return r.ru_utime + r.ru_stime


def latency(n=60):
    L = AppleLift()
    cfgs = []
    fa, fb, ff = frames(EP_A), frames(EP_B), frames(EP_B, "frames_full/bench")
    cfgs.append(("320x240 stored", [open(f, "rb").read() for f in fa[100:100 + n]]))
    cfgs.append(("448x336 stored", [open(f, "rb").read() for f in fb[100:100 + n]]))
    up = [cv2.imencode(".jpg", cv2.resize(cv2.imread(f), (640, 480)))[1].tobytes() for f in fb[100:100 + n]]
    cfgs.append(("640x480 upscaled", up))
    cfgs.append(("1280x720 full", [open(f, "rb").read() for f in ff[:n]]))
    for name, data in cfgs:
        for d in data[:5]:
            L(d)
        t, c0 = [], cpu()
        for d in data:
            t0 = time.perf_counter(); m = L(d); t.append(time.perf_counter() - t0)
        wall = sum(t)
        print(f"{name:18s} {np.median(t)*1000:6.1f} ms median  p95 {np.percentile(t,95)*1000:6.1f}  "
              f"max {max(t)*1000:6.1f}  -> {1/np.median(t):5.0f} fps  | CPU/wall {100*(cpu()-c0)/wall:3.0f}% of one core  "
              f"| instances last frame {len(m)}")


def overlay(img, masks):
    o = img.copy()
    for i, m in enumerate(masks):
        o[m] = (0.45 * o[m] + 0.55 * np.array(PAL[i % len(PAL)][::-1])).astype(np.uint8)
        c = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        cv2.drawContours(o, c, -1, PAL[i % len(PAL)][::-1], 1)
    return o


def sheet():
    L = AppleLift()
    for tag, ep, idxs in (("A", EP_A, [0, 300, 600, 840, 900, 960, 1020, 1100]),
                          ("B", EP_B, [0, 150, 300, 450, 600, 750, 900, 1000])):
        fs = frames(ep); tiles = []
        for i in idxs:
            im = cv2.imread(fs[min(i, len(fs) - 1)]); im = cv2.resize(im, (448, 336)); ms = L(im)
            o = overlay(im, ms)
            cv2.putText(o, f"f{i} n={len(ms)}", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            tiles.append(o)
        rows = [np.hstack(tiles[k:k + 4]) for k in range(0, 8, 4)]
        cv2.imwrite(f"{OUT}/apple_sheet_{tag}.png", np.vstack(rows)); print("wrote", f"{OUT}/apple_sheet_{tag}.png")
    # raw vs instances at full res for small-object question
    ff = frames(EP_B, "frames_full/bench")
    im = cv2.imread(ff[0]); ms = L(im); o = overlay(im, ms)
    cv2.imwrite(f"{OUT}/apple_full_B0.png", np.hstack([cv2.resize(im, (640, 360)), cv2.resize(o, (640, 360))]))
    print("full-res instances:", [int(m.sum()) for m in ms])


def iou(a, b):
    u = (a | b).sum(); return float((a & b).sum() / u) if u else 0.0


def cen(m):
    ys, xs = np.nonzero(m); return (xs.mean(), ys.mean()) if len(xs) else (np.nan, np.nan)


def track(start=840, end=1228, min_iou=0.05):
    L = AppleLift()
    fs = frames(EP_A); gt = load_tracks(EP_A)["target"]        # EdgeTAM offline (stored size)
    H, W = gt.shape[1:]
    seed = gt[start].copy(); prev = seed
    rows = []; lost_at = None; t_ms = []
    for i in range(start, min(end, len(fs))):
        im = cv2.imread(fs[i]); t0 = time.perf_counter(); inst = L(im); t_ms.append(time.perf_counter() - t0)
        # association: best IoU with prev mask; fallback nearest centroid within radius
        best, bi = 0.0, -1
        for j, m in enumerate(inst):
            v = iou(m, prev)
            if v > best: best, bi = v, j
        cur = None
        if bi >= 0 and best >= min_iou:
            cur = inst[bi]
        else:        # centroid gate
            pc = np.array(cen(prev)); d = [np.linalg.norm(np.array(cen(m)) - pc) if m.any() else 1e9 for m in inst]
            if d and min(d) < 25:
                cur = inst[int(np.argmin(d))]
        if cur is None:
            if lost_at is None: lost_at = i
            cur = np.zeros_like(prev)
        else:
            prev = cur
        g = gt[i]; rows.append((i, iou(cur, g), cen(cur), cen(g), int(cur.sum()), int(g.sum()), len(inst)))
    print(f"vision {1000*np.median(t_ms):.1f} ms/frame; first lost frame: {lost_at}")
    print("frame  IoU(vs EdgeTAM)  vision_cen   edgetam_cen   areas(v/e)  n_inst")
    for r in rows[::24]:
        print(f"f{r[0]:4d}  {r[1]:.2f}  ({r[2][0]:5.0f},{r[2][1]:5.0f})  ({r[3][0]:5.0f},{r[3][1]:5.0f})  {r[4]:4d}/{r[5]:4d}  {r[6]}")
    ious = np.array([r[1] for r in rows])
    print(f"mean IoU {ious.mean():.2f}; frames IoU>0.5: {100*(ious>0.5).mean():.0f}%; IoU>0.3: {100*(ious>0.3).mean():.0f}%")
    # sheet of tracked-vs-gt
    tiles = []
    for i in range(start, end, 55):
        if i >= len(fs): break
        im = cv2.imread(fs[i]); k = i - start
        v = rows[min(k, len(rows) - 1)]
        # recompute vision mask for display
        inst = L(im); o = overlay(im, inst)
        cv2.circle(o, (int(v[3][0]), int(v[3][1])), 6, (0, 255, 255), 2)    # EdgeTAM centroid (yellow)
        if not np.isnan(v[2][0]): cv2.circle(o, (int(v[2][0]), int(v[2][1])), 3, (0, 0, 255), -1)   # vision track (red)
        cv2.putText(o, f"f{i} IoU{v[1]:.2f}", (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        tiles.append(cv2.resize(o, (400, 300)))
    while len(tiles) % 4: tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(f"{OUT}/apple_track.png", np.vstack([np.hstack(tiles[k:k + 4]) for k in range(0, len(tiles), 4)]))
    print("wrote", f"{OUT}/apple_track.png")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("what", choices=["latency", "sheet", "track"])
    {"latency": latency, "sheet": sheet, "track": track}[ap.parse_args().what]()
