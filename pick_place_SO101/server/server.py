"""TeleopServer — aiohttp WSS in a daemon thread (the robot process IS the server).

phone -> robot : {type:"xr", p, q, clutch, gripper(0..100), orient_lock, tracked}
                 {type:"event", name: start|success|fail|discard|home|setforward,
                  instruction?: str}
robot -> phone : {type:"status", ...}     (UI only; no haptics by design)

Camera hook (the pipeline you're building plugs in here):
    server.push_frame("wrist", jpeg_bytes)
    server.push_frame("bench", jpeg_bytes)
Frames are re-served as MJPEG at /cam/<name>.mjpg and listed at /cams; the phone
shows "coming soon" placeholders until a stream appears.
"""
import asyncio
import collections
import json
import pathlib
import queue
import ssl
import threading
import time

from aiohttp import WSMsgType, web

from . import gen_cert

HERE = pathlib.Path(__file__).resolve().parent
STATIC = HERE / "static"
CAMERAS = ("wrist", "bench")


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self._msg = None
        self._events = collections.deque()
        self._instruction = ""
        self._hist = collections.deque(maxlen=120)   # (t_ms, rx_s, p, q) of fresh xr packets

    def update(self, msg):
        msg["_rx"] = time.monotonic()
        with self._lock:
            self._msg = msg
            try:
                if not self._hist or self._hist[-1][0] != msg["t"]:
                    self._hist.append((float(msg["t"]), msg["_rx"], tuple(msg["p"]), tuple(msg["q"])))
            except (KeyError, TypeError, ValueError):
                pass

    def latest(self):
        with self._lock:
            if not self._msg:
                return None
            m = dict(self._msg)
            m["_hist"] = list(self._hist)
            return m

    def push_event(self, name, payload):
        with self._lock:
            self._events.append((name, payload))

    def take_events(self):
        with self._lock:
            ev = list(self._events)
            self._events.clear()
            return ev

    def set_instruction(self, text):
        with self._lock:
            self._instruction = text

    def get_instruction(self):
        with self._lock:
            return self._instruction


class TeleopServer:
    def __init__(self, port=8443):
        self.port = port
        self.state = SharedState()
        self._out = queue.Queue()
        self._clients = set()
        self.ip = gen_cert.lan_ip()
        self._frames = {c: None for c in CAMERAS}
        self._frame_t = {c: 0.0 for c in CAMERAS}
        self._frame_lock = threading.Lock()

    # ---- robot-thread API ----
    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def latest(self):
        return self.state.latest()

    def take_events(self):
        return self.state.take_events()

    def set_instruction(self, text):
        self.state.set_instruction(text)

    def push_status(self, **kw):
        self._out.put({"type": "status", **kw})

    def push_frame(self, name, jpeg):
        if name not in self._frames:
            raise KeyError(f"unknown camera {name!r}; expected one of {CAMERAS}")
        with self._frame_lock:
            self._frames[name] = jpeg
            self._frame_t[name] = time.monotonic()

    def _frame(self, name):
        with self._frame_lock:
            return self._frames.get(name), self._frame_t.get(name, 0.0)

    # ---- loop thread ----
    def _run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        app = web.Application()
        app.add_routes([
            web.get("/", self._index),
            web.get("/ws", self._ws),
            web.get("/cams", self._cams),
            web.get("/cam/{name}.mjpg", self._mjpg),
            web.get("/{asset:.+}", self._asset),
        ])
        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())
        cert, key = gen_cert.generate()
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(cert, key)
        loop.run_until_complete(web.TCPSite(runner, "0.0.0.0", self.port, ssl_context=ctx).start())
        loop.create_task(self._pump())
        loop.run_forever()

    async def _index(self, request):
        return web.FileResponse(STATIC / "teleop.html", headers={"Cache-Control": "no-cache"})

    async def _asset(self, request):
        path = (STATIC / request.match_info["asset"]).resolve()
        if STATIC not in path.parents or not path.is_file():
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers={"Cache-Control": "no-cache"})

    async def _cams(self, request):
        now = time.monotonic()
        live = {}
        for c in CAMERAS:
            frame, t = self._frame(c)
            live[c] = frame is not None and now - t < 2.0
        return web.json_response(live)

    async def _mjpg(self, request):
        name = request.match_info["name"]
        if name not in self._frames:
            raise web.HTTPNotFound()
        resp = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-cache"})
        await resp.prepare(request)
        last_t = 0.0
        try:
            while True:
                frame, t = self._frame(name)
                if frame is not None and t != last_t:
                    last_t = t
                    await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                await asyncio.sleep(1 / 60)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp

    async def _ws(self, request):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self._clients.add(ws)
        print(f"[teleop] phone connected ({request.remote})")
        instr = self.state.get_instruction()
        if instr:
            await ws.send_str(json.dumps({"type": "status", "instruction": instr}))
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    kind = data.get("type")
                    if kind == "xr":
                        self.state.update(data)
                    elif kind == "event":
                        self.state.push_event(data.get("name"), data)
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            self._clients.discard(ws)
            print(f"[teleop] phone disconnected ({request.remote})")
        return ws

    async def _pump(self):
        while True:
            await asyncio.sleep(0.02)
            while True:
                try:
                    event = self._out.get_nowait()
                except queue.Empty:
                    break
                payload = json.dumps(event)
                for ws in list(self._clients):
                    try:
                        await ws.send_str(payload)
                    except ConnectionError:
                        self._clients.discard(ws)
