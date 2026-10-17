"""Offline tests (no network/models):  .venv/bin/python -m pick_place_SO101.test_planner_segment"""
import io

import numpy as np
from PIL import Image

from . import planner, segment


def _jpeg(arr):
    b = io.BytesIO()
    Image.fromarray(arr).save(b, "JPEG", quality=95)
    return b.getvalue()


def test_planner():
    img = _jpeg(np.zeros((8, 8, 3), np.uint8))
    p = planner.plan(img, backend=planner.MockBackend())
    assert p["target"] == "green boat" and "green boat" in p["instruction"], p
    hist = [p]
    p2 = planner.plan(img, hist, backend=planner.MockBackend([("green boat", "bin"), ("pen", "tray")]))
    assert p2["target"] == "pen", p2          # repeat rejected, retried
    assert planner.parse_json('Sure!\n```json\n{"target":"a","destination":"b"}\n```')["target"] == "a"
    try:
        planner.plan(img, backend=planner.MockBackend([("x", "x")]), retries=2)
        raise AssertionError("expected failure")
    except RuntimeError:
        pass
    asked = iter(["cup", "tray"])
    assert planner.plan(img, backend=planner.ManualBackend(lambda _: next(asked)))["destination"] == "tray"


def test_segment():
    a = np.full((60, 80, 3), 128, np.uint8)
    a[10:30, 10:30] = (40, 160, 70)           # green
    a[40:55, 50:70] = (230, 100, 170)         # pink
    out = segment.segment(_jpeg(a), ["green boat", "pink highlighter", "purple cup"],
                          backend=segment.MockBackend())
    g, k = out["green boat"], out["pink highlighter"]
    assert g.shape == (60, 80) and g.dtype == bool
    assert g[20, 20] and not g[50, 60] and k[50, 60] and out["purple cup"] is None
    png = segment.masks_to_png(g)
    assert np.asarray(Image.open(io.BytesIO(png)))[20, 20] == 255


if __name__ == "__main__":
    test_planner()
    test_segment()
    print("planner/segment: all offline tests passed")
