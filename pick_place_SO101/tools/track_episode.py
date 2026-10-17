"""Track the highlight masks through an episode (SAM2 video).
   python -m pick_place_SO101.tools.track_episode data/<session>/ep_0000 [more eps | session dirs]"""
import argparse
import pathlib

from ..tracking import MODEL, STRIDE, track_episode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--cam", default="bench")
    ap.add_argument("--model", default=MODEL, help="yonigozlan/EdgeTAM-hf (default) or facebook/sam2.1-hiera-small")
    ap.add_argument("--stride", type=int, default=STRIDE, help="track every Nth frame, hold between")
    a = ap.parse_args()
    eps = []
    for p in map(pathlib.Path, a.paths):
        eps += sorted(p.glob("ep_*")) if not p.name.startswith("ep_") else [p]
    for e in eps:
        if (e / "masks/target.png").exists() or (e / "masks/dest.png").exists():
            track_episode(e, cam=a.cam, model_id=a.model, stride=a.stride)
        else:
            print(f"[track] skip {e.name}: no seed masks")


if __name__ == "__main__":
    main()
