#!/usr/bin/env python3
"""Bench acceptance smoke test for the maki_puppet gateway (MPP/1).

Run from a laptop against the robot (or a --sim gateway)::

    python maki_puppet/scripts/smoke.py --host <robot-ip>

Steps (each reported PASS/FAIL):
  1. connect + print the welcome catalogs
  2. demo sequence: blink → look 4 corners → nod → shake → LED cycle → neutral
  3. emit celebrate (awaited to completion)
  4. word_read flood: 20 rapid fire-and-forget events — verifies the
     `on_busy: drop` tuning sheds load instead of building a queue
  5. e-stop cycle: engage → act rejected `estopped` → disengage → act works

Exit codes: 0 all steps passed, 1 one or more failed, 2 could not connect.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import sys
import time
from pathlib import Path

# Make the SDK importable straight from the repo checkout (no install needed).
_SDK_DIR = Path(__file__).resolve().parents[1] / "clients" / "python"
if str(_SDK_DIR) not in sys.path:
    sys.path.insert(0, str(_SDK_DIR))

from maki_client import (  # noqa: E402
    Maki,
    MakiActionError,
    MakiError,
)
from maki_client.__main__ import run_demo  # noqa: E402


class Report:
    def __init__(self) -> None:
        self.results: list[tuple[str, bool, str]] = []

    def record(self, step: str, ok: bool, detail: str = "") -> None:
        self.results.append((step, ok, detail))
        mark = "PASS" if ok else "FAIL"
        print(f"[{mark}] {step}" + (f" — {detail}" if detail else ""))

    def summary(self) -> bool:
        passed = sum(1 for _, ok, _ in self.results if ok)
        print("\n==== SMOKE SUMMARY ====")
        for step, ok, detail in self.results:
            print(f"  {'PASS' if ok else 'FAIL'}  {step}" + (f" — {detail}" if detail else ""))
        print(f"{passed}/{len(self.results)} steps passed")
        return passed == len(self.results)


def print_catalogs(m: Maki) -> None:
    print(f"  server session : {m.session}")
    print(f"  capabilities   : {', '.join(m.capabilities)}")
    print(f"  joints         : {', '.join(m.joints)}")
    print(f"  animations ({len(m.animations)}): {', '.join(m.animations)}")
    print(f"  gestures   ({len(m.gestures)}): {', '.join(m.gestures)}")
    print(f"  events     ({len(m.events)}): {', '.join(m.events)}")


async def step_demo(m: Maki, report: Report, timeout: float) -> None:
    failures: list[str] = []

    def sub(label: str, ok: bool, detail: str) -> None:
        print(f"    [{'ok  ' if ok else 'FAIL'}] {label}" + (f" — {detail}" if detail else ""))
        if not ok:
            failures.append(label)

    ok = await run_demo(m, sub, timeout=timeout)
    report.record("demo sequence", ok,
                  "" if ok else f"failed sub-steps: {', '.join(failures)}")


async def step_celebrate(m: Maki, report: Report, timeout: float) -> None:
    try:
        handle = await m.emit("celebrate")
        payload = await handle.wait(timeout)
        duration = payload.get("duration_ms")
        report.record("emit celebrate", True,
                      f"completed in {duration} ms" if duration is not None else "completed")
    except MakiError as exc:
        report.record("emit celebrate", False, str(exc))


async def step_word_read_flood(m: Maki, report: Report, timeout: float) -> None:
    """20 rapid fire-and-forget word_read events. The choreography is tuned
    `priority: low, on_busy: drop`, so all but the running one must be shed —
    never queued (a queue here means per-word lag during a reading session)."""
    try:
        t0 = time.monotonic()
        handles = [await m.emit("word_read", word=f"word-{i}") for i in range(20)]
        send_ms = (time.monotonic() - t0) * 1000
        state = await m.state(fresh=True)
        depth = int(state.queue.get("depth") or 0)
        # Wait for every handle to reach a terminal state (drops raise
        # MakiCancelled — swallowed by return_exceptions).
        await asyncio.gather(*(h.wait(timeout) for h in handles),
                             return_exceptions=True)
        statuses = [h.status for h in handles]
        completed = statuses.count("completed")
        dropped = statuses.count("dropped")
        queued_ever = sum(1 for h in handles
                          if any(a.get("status") == "queued" for a in h.acks))
        unresolved = sum(1 for h in handles if not h.done)
        detail = (f"sent 20 in {send_ms:.0f} ms: completed={completed} "
                  f"dropped={dropped} queued={queued_ever} "
                  f"queue_depth_after_burst={depth}")
        ok = (completed >= 1 and dropped >= 10 and queued_ever == 0
              and depth <= 2 and unresolved == 0)
        report.record("word_read flood (20x fire-and-forget)", ok, detail)
    except MakiError as exc:
        report.record("word_read flood (20x fire-and-forget)", False, str(exc))


async def step_estop_cycle(m: Maki, report: Report, timeout: float) -> None:
    try:
        await m.estop(True, reason="smoke test")
        state = await m.state(fresh=True)
        engaged = state.estop is True
        led_alarm = state.led.get("animation") == "alarm_red"

        rejected = False
        try:
            handle = await m.blink()
            await handle.wait(timeout)
        except MakiActionError as exc:
            rejected = exc.code == "estopped"
        except MakiError:
            rejected = False

        await m.estop(False, reason="smoke test done")
        state = await m.state(fresh=True)
        disengaged = state.estop is False

        blink_after = False
        try:
            handle = await m.blink()
            await handle.wait(timeout)
            blink_after = True
        except MakiError:
            pass

        ok = engaged and rejected and disengaged and blink_after
        report.record(
            "estop cycle", ok,
            f"engaged={engaged} led_alarm={led_alarm} act_rejected_estopped={rejected} "
            f"disengaged={disengaged} blink_after={blink_after}")
    except MakiError as exc:
        report.record("estop cycle", False, str(exc))
        # Never leave the bench robot frozen.
        with contextlib.suppress(MakiError):
            await m.estop(False, reason="smoke cleanup")


async def main_async(args: argparse.Namespace) -> int:
    url = args.url or f"ws://{args.host}:{args.port}/ws"
    report = Report()
    timeout = args.timeout

    print(f"connecting to {url} ...")
    try:
        m = await Maki.connect(url, name="smoke-test", reconnect=False)
    except MakiError as exc:
        report.record("connect", False, str(exc))
        report.summary()
        return 2

    report.record("connect", True, f"session={m.session}")
    print_catalogs(m)

    try:
        await step_demo(m, report, timeout)
        await step_celebrate(m, report, timeout)
        await step_word_read_flood(m, report, timeout)
        await step_estop_cycle(m, report, timeout)

        # Leave the robot tidy regardless of outcomes.
        with contextlib.suppress(MakiError):
            await (await m.neutral()).wait(timeout)
    finally:
        await m.close()

    return 0 if report.summary() else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Bench acceptance smoke test for the maki_puppet gateway.")
    parser.add_argument("--host", default="localhost",
                        help="robot host (default: %(default)s)")
    parser.add_argument("--port", type=int, default=8765,
                        help="gateway port (default: %(default)s)")
    parser.add_argument("--url", default=None,
                        help="full ws:// URL (overrides --host/--port)")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="per-action completion timeout in seconds")
    args = parser.parse_args(argv)
    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
