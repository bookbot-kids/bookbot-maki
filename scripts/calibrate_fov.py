#!/usr/bin/env python3
"""Measure the camera's real field of view by rotating the head.

Why this exists: ``vision.tracking.camera_hfov_deg`` / ``camera_vfov_deg`` are
what convert a face's pixel offset into a body-centric angle. Get them wrong
and the head consistently over- or under-shoots every face, and no amount of
gain tuning fixes it — the geometry is wrong before the controller sees it.
The values inherited from the ROS stack were commented as being for an
IMX219, but this robot has an IMX179, so they were never verified here.

Method: point the camera at a static, textured scene, rotate the head by a
known angle, and measure how far the scene translated in pixels. For a pinhole
camera a small rotation dtheta shifts the image by ``dpx = f * dtheta``, so::

    f_px  = dpx / dtheta
    FOV   = 2 * atan((size_px / 2) / f_px)

Rotation comes from the gateway (which owns the servos); the shift is measured
with OpenCV phase correlation, which is robust to lighting change and needs no
feature matching.

Requirements:
  * The gateway must be running WITHOUT vision, so the camera is free:
        MAKI_PUPPET_CONFIG=/tmp/no_vision.yaml scripts/run_puppet.sh
  * A static scene with some texture. A blank wall will produce a low
    correlation response and the script will tell you so rather than
    silently returning nonsense.

Usage:
    python scripts/calibrate_fov.py --host 127.0.0.1
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import statistics
import sys
import time

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - tool script
    sys.exit("this tool needs opencv: pip install -e '.[vision]'")

import websockets

# Correlation response below this means the scene had too little texture (or
# moved) for the measurement to mean anything.
MIN_RESPONSE = 0.05

# Settle time after commanding a pose, so the S-curve has actually arrived.
SETTLE_S = 1.5


class Gateway:
    """Minimal MPP/1 client: move the head, read back where it really is.

    A background reader drains the socket and answers the gateway's heartbeat
    pings. Without it the connection is dropped mid-run (1001, heartbeat
    timeout): this tool spends most of its time sleeping while the head
    settles, and a client that only reads when it wants a reply misses the
    pings that arrive in between.
    """

    def __init__(self, ws):
        self.ws = ws
        self._n = 0
        self._inbox: asyncio.Queue = asyncio.Queue()
        self._reader = asyncio.ensure_future(self._read_loop())

    async def _read_loop(self):
        try:
            async for raw in self.ws:
                msg = json.loads(raw)
                if msg["type"] == "ping":
                    await self.ws.send(json.dumps({
                        "type": "pong", "id": f"p-{msg['id']}",
                        "ts": int(time.time() * 1000),
                        "payload": {"ref": msg["id"]},
                    }))
                    continue
                await self._inbox.put(msg)
        except Exception:
            pass

    async def close(self):
        self._reader.cancel()

    async def _send(self, type_, payload):
        self._n += 1
        mid = f"cal-{self._n}"
        await self.ws.send(json.dumps(
            {"type": type_, "id": mid, "ts": int(time.time() * 1000), "payload": payload}
        ))
        return mid

    async def _await_ref(self, mid, type_, terminal=None):
        while True:
            msg = await asyncio.wait_for(self._inbox.get(), timeout=30)
            if msg["payload"].get("ref") != mid or msg["type"] != type_:
                continue
            if terminal is None or msg["payload"].get("status") in terminal:
                return msg["payload"]

    async def hello(self):
        mid = await self._send("hello", {
            "protocol": 1,
            "client": {"name": "fov-calibration", "kind": "python", "version": "1"},
            "priority": 70,
        })
        return await self._await_ref(mid, "welcome")

    async def pose(self, joints, duration_ms=900):
        """Command a pose and wait for it to complete, then let it settle."""
        mid = await self._send("act", {
            "do": {"kind": "pose", "joints": joints, "duration_ms": duration_ms},
            "priority": 70,
        })
        await self._await_ref(
            mid, "ack", terminal={"completed", "cancelled", "superseded", "dropped", "error"}
        )
        await asyncio.sleep(SETTLE_S)

    async def pose_rad(self):
        """Actual head angles in radians, from the gateway's own feedback."""
        mid = await self._send("state.get", {"fields": ["pose"]})
        payload = await self._await_ref(mid, "state")
        from maki_puppet.motion import joints as j

        pose = payload["pose"]
        return {
            name: j.norm_to_rad(name, pose[name])
            for name in ("head_pan", "head_tilt")
            if name in pose
        }


def grab(cap, warmup=5):
    """Read a frame, discarding buffered ones so it reflects the pose NOW."""
    frame = None
    for _ in range(warmup):
        ok, f = cap.read()
        if ok:
            frame = f
    if frame is None:
        raise RuntimeError("camera read failed")
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)


def shift_px(a, b):
    """(dx, dy, response) translation from image a to image b."""
    window = cv2.createHanningWindow((a.shape[1], a.shape[0]), cv2.CV_32F)
    (dx, dy), response = cv2.phaseCorrelate(a, b, window)
    return dx, dy, response


async def run(args):
    cap = cv2.VideoCapture(args.device if not str(args.device).isdigit() else int(args.device))
    if not cap.isOpened():
        sys.exit(f"cannot open camera {args.device!r} — is the gateway still holding it?")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera {args.device}: {w}x{h}")

    focals: list = []   # (focal_px, rotation_rad) from every usable step
    async with websockets.connect(f"ws://{args.host}:{args.port}/ws") as ws:
        gw = Gateway(ws)
        await gw.hello()
        print("connected to gateway\n")

        for axis, joint, sweep, size in (
            ("pan", "head_pan", args.pan_sweep, w),
            ("tilt", "head_tilt", args.tilt_sweep, h),
        ):
            print(f"--- {axis} ---")
            for lo, hi in zip(sweep, sweep[1:]):
                await gw.pose({joint: lo}, duration_ms=900)
                before = grab(cap)
                rad_before = (await gw.pose_rad())[joint]

                await gw.pose({joint: hi}, duration_ms=900)
                after = grab(cap)
                rad_after = (await gw.pose_rad())[joint]

                dtheta = rad_after - rad_before
                dx, dy, response = shift_px(before, after)
                dpx = dx if axis == "pan" else dy
                if abs(dtheta) < 1e-3:
                    print(f"  {lo:+.2f}->{hi:+.2f}: head did not move, skipped")
                    continue
                if response < MIN_RESPONSE:
                    print(f"  {lo:+.2f}->{hi:+.2f}: response {response:.3f} too low, skipped"
                          " (scene has too little texture, or something moved)")
                    continue
                f_px = abs(dpx / dtheta)
                fov = math.degrees(2 * math.atan((size / 2) / f_px))
                focals.append((f_px, dtheta))
                print(f"  {lo:+.2f}->{hi:+.2f} rad {dtheta:+.4f}  shift {dpx:+7.1f}px  "
                      f"resp {response:.2f}  ->  f={f_px:.0f}px  FOV={fov:.1f} deg")

        await gw.pose({"head_pan": 0.0, "head_tilt": 0.0}, duration_ms=900)
        await gw.close()

    cap.release()
    print("\n================ RESULT ================")
    # One lens has ONE focal length, so pool every sample from both axes and
    # derive both FOVs from it. Estimating the axes independently throws away
    # half the data and lets a single noisy sample skew one axis on its own.
    # Small rotations are dropped: their pixel shift is a handful of pixels, so
    # quantisation error dominates the ratio.
    good = [f for f, dt in focals if abs(dt) >= args.min_rotation_rad]
    dropped = len(focals) - len(good)
    if not good:
        print("NO USABLE MEASUREMENTS — point the camera at a textured, static scene")
        print("========================================")
        return

    f_px = statistics.median(good)
    spread = max(good) - min(good)
    print(f"focal length: {f_px:.0f} px   (n={len(good)}"
          f"{f', {dropped} small-rotation samples dropped' if dropped else ''}"
          f", spread {spread:.0f} px)")
    if spread > 0.1 * f_px:
        print("  NOTE: >10% spread — rerun against a scene with more texture")
    print()
    print(f"camera_hfov_deg: {math.degrees(2 * math.atan((w / 2) / f_px)):.1f}")
    print(f"camera_vfov_deg: {math.degrees(2 * math.atan((h / 2) / f_px)):.1f}")
    print("========================================")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--device", default="/dev/video0")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    # Normalized wire units. Kept modest so the scene stays in frame between
    # steps — phase correlation needs the two images to overlap substantially.
    p.add_argument("--pan-sweep", type=float, nargs="+",
                   default=[-0.30, -0.15, 0.0, 0.15, 0.30])
    p.add_argument("--tilt-sweep", type=float, nargs="+",
                   default=[-0.30, -0.15, 0.0, 0.15, 0.30])
    # Below this the shift is only a few pixels and quantisation dominates.
    p.add_argument("--min-rotation-rad", type=float, default=0.03)
    asyncio.run(run(p.parse_args()))


if __name__ == "__main__":
    main()
