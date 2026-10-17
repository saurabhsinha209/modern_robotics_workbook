"""USB cameras -> JPEG frames via AVFoundation, selected by macOS Unique ID.

    cams = CameraSet(config.CAMERA_UIDS).start(on_frame=server.push_frame)
    cams.jpeg("wrist") -> (jpeg_bytes, monotonic_capture_time)

Why not OpenCV indices: both openaicams share a name, and OpenCV's index order
differs from system_profiler's and moves when the iPhone / another camera
appears. Unique ID is the only stable handle. A UVC Unique ID encodes the USB
*port* (0x11... vs 0x31...), so wrist/bench follow the ports they sit in.
"""
import threading
import time

import cv2
import numpy as np
import objc
import AVFoundation as AV
import CoreMedia as CM
import dispatch
import Quartz
from Foundation import NSObject

from . import config

# Resolve every CoreVideo/CoreMedia symbol on the importing (main) thread. pyobjc
# loads them lazily, and two camera threads doing that at once can raise
# KeyError('CVPixelBufferUnlockBaseAddress') -- which, raised inside an
# Objective-C callback, aborts the whole process (arm left torque-on).
_get_image_buffer = CM.CMSampleBufferGetImageBuffer
_lock_base = Quartz.CVPixelBufferLockBaseAddress
_unlock_base = Quartz.CVPixelBufferUnlockBaseAddress
_pb_width = Quartz.CVPixelBufferGetWidth
_pb_height = Quartz.CVPixelBufferGetHeight
_pb_bpr = Quartz.CVPixelBufferGetBytesPerRow
_pb_base = Quartz.CVPixelBufferGetBaseAddress


def list_devices():
    """[(name, unique_id)] as AVFoundation sees them (order is not meaningful)."""
    return [(str(d.localizedName()), str(d.uniqueID()))
            for d in AV.AVCaptureDevice.devicesWithMediaType_(AV.AVMediaTypeVideo)]


class _Delegate(NSObject):
    def initWithCallback_(self, cb):
        self = objc.super(_Delegate, self).init()
        self.cb = cb
        return self

    def captureOutput_didOutputSampleBuffer_fromConnection_(self, out, sb, conn):
        try:                                   # an exception here would abort the process
            pb = _get_image_buffer(sb)
            if pb is None:
                return
            _lock_base(pb, 0)
            try:
                w, h = _pb_width(pb), _pb_height(pb)
                bpr = _pb_bpr(pb)
                buf = _pb_base(pb).as_buffer(bpr * h)
                bgr = np.frombuffer(buf, np.uint8).reshape(h, bpr // 4, 4)[:, :w, :3].copy()
            finally:
                _unlock_base(pb, 0)
            self.cb(bgr)
        except Exception as e:                 # drop the frame, keep the robot process alive
            self.errors = getattr(self, "errors", 0) + 1
            if self.errors in (1, 10, 100):
                print(f"[cam] frame dropped ({self.errors}): {type(e).__name__}: {e}")


class _Cam:
    def __init__(self, name, uid, width, height, fps, quality, on_frame):
        self.name, self.uid = name, uid
        self.width, self.height, self.fps, self.quality = width, height, fps, quality
        self.on_frame = on_frame
        self.jpeg, self.t, self.n = None, 0.0, 0
        self.shape = None
        self.lock = threading.Lock()
        self.error = None

    def open(self):
        dev = AV.AVCaptureDevice.deviceWithUniqueID_(self.uid)
        if dev is None:
            self.error = f"device {self.uid} not found"
            return
        self._pick_format(dev)
        inp, err = AV.AVCaptureDeviceInput.deviceInputWithDevice_error_(dev, None)
        if inp is None:
            self.error = f"cannot open: {err} (Camera permission?)"
            return
        self.session = AV.AVCaptureSession.alloc().init()
        self.session.addInput_(inp)
        out = AV.AVCaptureVideoDataOutput.alloc().init()
        out.setVideoSettings_({Quartz.kCVPixelBufferPixelFormatTypeKey:
                               Quartz.kCVPixelFormatType_32BGRA})
        out.setAlwaysDiscardsLateVideoFrames_(True)
        self.delegate = _Delegate.alloc().initWithCallback_(self._frame)
        self.queue = dispatch.dispatch_queue_create(f"cam.{self.name}".encode(), None)
        out.setSampleBufferDelegate_queue_(self.delegate, self.queue)
        self.session.addOutput_(out)
        self.session.startRunning()

    def _pick_format(self, dev):
        """Smallest native format >= requested size, at the requested fps if offered."""
        best, best_key = None, None
        for f in dev.formats():
            d = CM.CMVideoFormatDescriptionGetDimensions(f.formatDescription())
            if d.width < self.width or d.height < self.height:
                continue
            rates = [r.maxFrameRate() for r in f.videoSupportedFrameRateRanges()]
            ok_fps = any(r >= self.fps - 0.5 for r in rates)
            key = (not ok_fps, d.width * d.height)
            if best_key is None or key < best_key:
                best, best_key = f, key
        if best is None:
            return
        ok, _ = dev.lockForConfiguration_(None)
        if ok:
            dev.setActiveFormat_(best)
            try:
                rngs = [r for r in best.videoSupportedFrameRateRanges()
                        if r.maxFrameRate() >= self.fps - 0.5]
                if rngs:                      # must be the range's own exact duration
                    r = min(rngs, key=lambda r: abs(r.maxFrameRate() - self.fps))
                    dev.setActiveVideoMinFrameDuration_(r.minFrameDuration())
                    dev.setActiveVideoMaxFrameDuration_(r.minFrameDuration())
            except Exception as e:           # keep the format, take whatever fps results
                print(f"[cam] {self.name}: frame-rate not set ({e})")
            dev.unlockForConfiguration()

    def _frame(self, bgr):
        if (bgr.shape[1], bgr.shape[0]) != (self.width, self.height) and \
                bgr.shape[1] > self.width:
            bgr = cv2.resize(bgr, (self.width, self.height), interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        b = jpg.tobytes()
        with self.lock:
            self.jpeg, self.t, self.n, self.shape = b, time.monotonic(), self.n + 1, bgr.shape
        if self.on_frame:
            self.on_frame(self.name, b)

    def stop(self):
        try:
            self.session.stopRunning()
        except Exception:
            pass


class CameraSet:
    def __init__(self, uids=None, width=None, height=None, fps=None, quality=85):
        self.uids = dict(config.CAMERA_UIDS if uids is None else uids)
        w, h = config.CAMERA_SIZE
        self.width, self.height = width or w, height or h
        self.fps = fps or config.CAMERA_FPS
        self.quality = quality
        self._cams = {}

    @property
    def names(self):
        return list(self.uids)

    def start(self, on_frame=None):
        present = {u for _, u in list_devices()}
        for name, uid in self.uids.items():
            c = _Cam(name, uid, self.width, self.height, self.fps, self.quality, on_frame)
            if uid not in present:
                c.error = (f"Unique ID {uid} not present (have: "
                           f"{sorted(present)}). Replug / check port.")
            else:
                c.open()
            self._cams[name] = c
        t0 = time.time()
        while time.time() - t0 < 6.0:
            if all(c.jpeg is not None or c.error for c in self._cams.values()):
                break
            time.sleep(0.05)
        for n, c in self._cams.items():
            print(f"[cam] {n}: " + (f"ERROR {c.error}" if c.error or c.jpeg is None
                                    else f"{c.shape[1]}x{c.shape[0]} live"))
        return self

    def jpeg(self, name):
        c = self._cams[name]
        with c.lock:
            return c.jpeg, c.t

    def counts(self):
        return {n: c.n for n, c in self._cams.items()}

    def errors(self):
        return {n: c.error for n, c in self._cams.items() if c.error}

    def stop(self):
        for c in self._cams.values():
            c.stop()
