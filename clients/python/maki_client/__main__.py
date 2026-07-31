"""CLI over the maki_client SDK.

Usage::

    python -m maki_client --host <robot-ip> [--port 8765] <command> [args...]

Commands:
    emit NAME [k=v ...]   send a semantic event (values parsed as JSON, else string)
    blink                 one blink
    look PAN TILT         orient head+eyes (normalized [-1, 1])
    led ANIMATION         switch the LED ring animation
    gesture NAME          play a named gesture
    say TEXT...           text-to-speech (expect tts_unavailable while deferred)
    state                 fetch and print the robot state
    estop [on|off]        engage (default) or release the e-stop
    cancel                cancel all queued/running work
    demo                  blink → look 4 corners → nod → shake → LED cycle → neutral

Exit codes: 0 ok, 1 action error, 2 connection error.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from typing import Any, Callable, Optional

from . import (
    Maki,
    MakiActionError,
    MakiCancelled,
    MakiConnectionError,
    MakiError,
    RobotState,
)

Reporter = Callable[[str, bool, str], None]


# ── Demo sequence (shared with scripts/smoke.py) ───────────────────────────


async def run_demo(
    m: Maki,
    report: Optional[Reporter] = None,
    *,
    timeout: float = 30.0,
    led_dwell_s: float = 0.8,
) -> bool:
    """Bench demo: blink → look at 4 corners → nod → head_shake → LED cycle →
    neutral. Reports each sub-step via ``report(label, ok, detail)``; returns
    True only if every sub-step completed."""
    ok_all = True

    def _report(label: str, ok: bool, detail: str = "") -> None:
        if report is not None:
            report(label, ok, detail)

    async def step(label: str, handle_coro: Any) -> None:
        nonlocal ok_all
        try:
            handle = await handle_coro
            payload = await handle.wait(timeout)
            duration = payload.get("duration_ms")
            _report(label, True, f"{duration} ms" if duration is not None else "")
        except MakiError as exc:
            ok_all = False
            _report(label, False, str(exc))

    await step("blink", m.blink())
    for pan, tilt in ((-0.6, -0.4), (0.6, -0.4), (0.6, 0.4), (-0.6, 0.4)):
        await step(f"look({pan:+.1f}, {tilt:+.1f})", m.look(pan, tilt))
    await step("look(+0.0, +0.0)", m.look(0.0, 0.0))

    for gesture in ("nod", "head_shake"):
        if gesture in m.gestures:
            await step(f"gesture {gesture}", m.gesture(gesture))
        else:
            ok_all = False
            _report(f"gesture {gesture}", False, "not in server gesture catalog")

    cycle = [a for a in ("attention_sweep", "glow_gold", "warm_pulse", "rainbow_swirl")
             if a in m.animations]
    if not cycle:
        ok_all = False
        _report("led cycle", False, "no known animations in server catalog")
    for animation in cycle:
        await step(f"led {animation}", m.led(animation))
        await asyncio.sleep(led_dwell_s)
    if "breathing_cyan" in m.animations:
        await step("led breathing_cyan", m.led("breathing_cyan"))

    await step("neutral", m.neutral())
    return ok_all


# ── Helpers ────────────────────────────────────────────────────────────────


def parse_params(pairs: list[str]) -> dict[str, Any]:
    """Parse ``k=v`` CLI arguments; values go through JSON, falling back to str."""
    params: dict[str, Any] = {}
    for pair in pairs:
        key, sep, value = pair.partition("=")
        if not sep or not key:
            raise SystemExit(f"bad event param {pair!r}: expected key=value")
        try:
            params[key] = json.loads(value)
        except ValueError:
            params[key] = value
    return params


def format_state(s: RobotState) -> str:
    lines = ["pose:"]
    for joint, value in sorted(s.pose.items()):
        lines.append(f"  {joint:<10} {value:+.3f}")
    if "color" in s.led:
        c = s.led["color"]
        lines.append(f"led:     color=({c.get('r')}, {c.get('g')}, {c.get('b')})")
    else:
        lines.append(f"led:     animation={s.led.get('animation')}")
    q = s.queue
    lines.append(f"queue:   depth={q.get('depth')} active_id={q.get('active_id')} "
                 f"active_client={q.get('active_client')}")
    lines.append(f"estop:   {s.estop}")
    lines.append(f"lock:    held_by={s.lock.get('held_by')} scopes={s.lock.get('scopes')}")
    clients = ", ".join(f"{c.get('name')} ({c.get('kind')})" for c in s.clients) or "none"
    lines.append(f"clients: {clients}")
    h = s.health
    lines.append(f"health:  servo={h.get('servo')} led={h.get('led')} "
                 f"loop_hz={h.get('loop_hz')}")
    return "\n".join(lines)


def _print_done(label: str, payload: dict) -> None:
    duration = payload.get("duration_ms")
    extra = f" ({duration} ms)" if duration is not None else ""
    choreography = payload.get("choreography")
    if choreography:
        extra = f" [choreography={choreography}]" + extra
    print(f"{label}: {payload.get('status', 'completed')}{extra}")


# ── Command dispatch ───────────────────────────────────────────────────────


async def _dispatch(m: Maki, args: argparse.Namespace) -> int:
    timeout = args.timeout
    cmd = args.command

    if cmd == "emit":
        handle = await m.emit(args.name, **parse_params(args.params))
        _print_done(f"event '{args.name}'", await handle.wait(timeout))
    elif cmd == "blink":
        _print_done("blink", await (await m.blink()).wait(timeout))
    elif cmd == "look":
        handle = await m.look(args.pan, args.tilt)
        _print_done(f"look({args.pan}, {args.tilt})", await handle.wait(timeout))
    elif cmd == "led":
        _print_done(f"led '{args.animation}'",
                    await (await m.led(args.animation)).wait(timeout))
    elif cmd == "gesture":
        _print_done(f"gesture '{args.name}'",
                    await (await m.gesture(args.name)).wait(timeout))
    elif cmd == "say":
        text = " ".join(args.text)
        _print_done(f"say {text!r}", await (await m.say(text)).wait(timeout))
    elif cmd == "state":
        print(format_state(await m.state(fresh=True)))
    elif cmd == "estop":
        engage = args.mode != "off"
        await m.estop(engage, reason="cli")
        print("estop engaged" if engage else "estop released")
    elif cmd == "cancel":
        ack = await m.cancel("all")
        detail = ack.get("detail", "")
        print(f"cancel all: {ack.get('status')}" + (f" ({detail})" if detail else ""))
    elif cmd == "demo":
        def report(label: str, ok: bool, detail: str) -> None:
            mark = "ok  " if ok else "FAIL"
            print(f"  [{mark}] {label}" + (f" — {detail}" if detail else ""))

        print("demo: blink → look corners → nod → shake → LED cycle → neutral")
        if not await run_demo(m, report, timeout=timeout):
            return 1
        print("demo complete")
    else:  # pragma: no cover — argparse enforces the choices
        raise SystemExit(f"unknown command {cmd!r}")
    return 0


async def _run(args: argparse.Namespace) -> int:
    url = args.url or f"ws://{args.host}:{args.port}/ws"
    try:
        m = await Maki.connect(url, name=args.name, reconnect=False)
    except MakiError as exc:
        print(f"connection error: {exc}", file=sys.stderr)
        return 2
    try:
        return await _dispatch(m, args)
    except MakiActionError as exc:
        print(f"action error [{exc.code}]: {exc.detail or exc}", file=sys.stderr)
        return 1
    except MakiCancelled as exc:
        print(f"action did not complete: {exc.status}", file=sys.stderr)
        return 1
    except MakiConnectionError as exc:
        print(f"connection error: {exc}", file=sys.stderr)
        return 2
    finally:
        await m.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="maki_client",
        description="Command-line puppeteer for the MAKI robot (MPP/1).",
    )
    parser.add_argument("--host", default="localhost", help="robot host (default: %(default)s)")
    parser.add_argument("--port", type=int, default=8765, help="gateway port (default: %(default)s)")
    parser.add_argument("--url", default=None, help="full ws:// URL (overrides --host/--port)")
    parser.add_argument("--name", default="maki-cli", help="client name shown to the server")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="seconds to wait for command completion (default: %(default)s)")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("emit", help="send a semantic event")
    p.add_argument("name")
    p.add_argument("params", nargs="*", metavar="k=v")
    sub.add_parser("blink", help="one blink")
    p = sub.add_parser("look", help="orient head+eyes")
    p.add_argument("pan", type=float)
    p.add_argument("tilt", type=float)
    p = sub.add_parser("led", help="switch LED animation")
    p.add_argument("animation")
    p = sub.add_parser("gesture", help="play a named gesture")
    p.add_argument("name")
    p = sub.add_parser("say", help="speak text (deferred: expect tts_unavailable)")
    p.add_argument("text", nargs="+")
    sub.add_parser("state", help="fetch and print robot state")
    p = sub.add_parser("estop", help="engage/release the e-stop")
    p.add_argument("mode", nargs="?", choices=("on", "off"), default="on")
    sub.add_parser("cancel", help="cancel all queued/running work")
    sub.add_parser("demo", help="run the bench demo sequence")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return asyncio.run(_run(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
