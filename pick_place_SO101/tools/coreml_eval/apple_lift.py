"""Apple Vision Subject Lifting (VNGenerateForegroundInstanceMaskRequest) -> per-instance bool masks.

    lifter = AppleLift()
    masks = lifter(jpeg_or_bgr)      # list[bool HxW] at the INPUT image size, one per lifted instance

Not text-promptable: it only separates salient 'foreground instances'. To use it as a tracker
it needs a seed + association (see tools/coreml_eval/eval_apple_lift.py). Compute unit (ANE/GPU/CPU) is
chosen by Vision; we can't pin it.
"""
import cv2
import numpy as np
import objc
import Quartz
import Vision


class AppleLift:
    def __init__(self):
        self._req = None

    def __call__(self, img):
        if isinstance(img, (bytes, bytearray)):
            data = bytes(img)
            h, w = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR).shape[:2]
        else:
            ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])
            data, (h, w) = enc.tobytes(), img.shape[:2]
        with objc.autorelease_pool():
            req = Vision.VNGenerateForegroundInstanceMaskRequest.alloc().init()
            handler = Vision.VNImageRequestHandler.alloc().initWithData_options_(data, {})
            ok, err = handler.performRequests_error_([req], None)
            if not ok:
                raise RuntimeError(f"Vision failed: {err}")
            res = req.results()
            if not res:
                return []
            r = res[0]
            ids = sorted(int(i) for i in r.allInstances())
            if not ids:
                return []
            pb = r.instanceMask()
            Quartz.CVPixelBufferLockBaseAddress(pb, 1)
            try:
                pw, ph = Quartz.CVPixelBufferGetWidth(pb), Quartz.CVPixelBufferGetHeight(pb)
                bpr = Quartz.CVPixelBufferGetBytesPerRow(pb)
                buf = Quartz.CVPixelBufferGetBaseAddress(pb).as_buffer(bpr * ph)
                lab = np.frombuffer(buf, np.uint8).reshape(ph, bpr)[:, :pw].copy()
            finally:
                Quartz.CVPixelBufferUnlockBaseAddress(pb, 1)
        if lab.shape != (h, w):
            lab = cv2.resize(lab, (w, h), interpolation=cv2.INTER_NEAREST)
        return [lab == i for i in ids]
