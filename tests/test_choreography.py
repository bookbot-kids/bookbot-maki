"""Choreographer tests: real-file loading, templating, validation, ignored
events and hot-reload atomic-swap semantics."""

from __future__ import annotations

import os
from pathlib import Path

import pytest
import yaml

from maki_puppet.choreography import Choreographer, apply_templates
from maki_puppet.protocol import Action, ProtocolError, Seq

CONFIG = Path(__file__).parent.parent / "config"

with open(CONFIG / "animations.yaml") as fh:
    ANIMATIONS = list(yaml.safe_load(fh)["animations"].keys())


@pytest.fixture
def real():
    c = Choreographer(CONFIG / "choreographies.yaml", ANIMATIONS)
    c.load()
    return c


MINIMAL = """\
version: 1
defaults: {priority: normal, on_busy: replace}
ignored_events: [noop_event]
choreographies:
  hello_event:
    priority: high
    on_busy: queue
    steps:
      - {kind: led, animation: glow_gold}
      - {kind: say, text: "{{text|Great job!}}"}
      - {kind: wait, duration_ms: "{{pause_ms|250}}"}
gestures:
  bob:
    keyframes:
      - {t_ms: 0, joints: {head_tilt: 0.0}}
      - {t_ms: 100, joints: {head_tilt: -0.5}}
"""


@pytest.fixture
def tmp_choreo(tmp_path):
    path = tmp_path / "choreographies.yaml"
    path.write_text(MINIMAL)
    c = Choreographer(path, ANIMATIONS)
    c.load()
    return c, path


# ── Real-file loading ───────────────────────────────────────────────────────


def test_real_file_catalogs(real):
    assert set(real.gesture_names) == {
        "nod", "head_shake", "happy_wiggle", "curious_tilt", "wake_up", "sleepy",
    }
    assert "celebrate" in real.event_names
    assert "sentence_read" in real.event_names       # ignored but accepted
    assert real.ignored_events == ["sentence_read"]


def test_real_resolve_celebrate(real):
    resolved = real.resolve("celebrate", {})
    assert resolved.priority == 70        # high
    assert resolved.on_busy == "replace"
    assert resolved.name == "celebrate"
    assert isinstance(resolved.step, Seq)


def test_real_resolve_word_read_is_low_drop(real):
    resolved = real.resolve("word_read", {"word": "cat"})
    assert resolved.priority == 20        # low — per-word events must be cheap
    assert resolved.on_busy == "drop"


def test_ignored_event_resolves_to_none(real):
    assert real.resolve("sentence_read", {}) is None


def test_unknown_event(real):
    with pytest.raises(ProtocolError) as ei:
        real.resolve("moonwalk", {})
    assert ei.value.code == "unknown_event"


def test_gesture_lookup(real):
    frames = real.get_gesture("nod")
    assert frames[0]["t_ms"] == 0
    assert "head_tilt" in frames[0]["joints"]
    with pytest.raises(KeyError):
        real.get_gesture("moonwalk")


# ── Templating ──────────────────────────────────────────────────────────────


def test_template_substitution_string():
    assert apply_templates("Say {{word|hi}} now", {"word": "cat"}) == "Say cat now"
    assert apply_templates("Say {{word|hi}} now", {}) == "Say hi now"
    assert apply_templates("Say {{word}} now", {}) == "Say  now"


def test_whole_value_template_keeps_json_type():
    assert apply_templates("{{streak}}", {"streak": 5}) == 5
    assert apply_templates("{{flag}}", {"flag": True}) is True
    assert apply_templates("{{ms|250}}", {}) == 250          # typed default
    assert apply_templates("{{name|maki}}", {}) == "maki"    # string default
    assert apply_templates("{{missing}}", {}) == ""


def test_template_in_resolved_steps(tmp_choreo):
    c, _ = tmp_choreo
    resolved = c.resolve("hello_event", {"text": "Nice!", "pause_ms": 100})
    say = resolved.step.children[1]
    wait = resolved.step.children[2]
    assert say.args["text"] == "Nice!"
    assert wait.args["duration_ms"] == 100
    # defaults apply when params are absent
    resolved = c.resolve("hello_event", {})
    assert resolved.step.children[1].args["text"] == "Great job!"
    assert resolved.step.children[2].args["duration_ms"] == 250
    assert resolved.priority == 70 and resolved.on_busy == "queue"


def test_single_step_choreography_is_bare_action(tmp_path):
    path = tmp_path / "c.yaml"
    path.write_text(
        "choreographies:\n  solo:\n    steps:\n      - {kind: blink}\n"
    )
    c = Choreographer(path, ANIMATIONS)
    c.load()
    assert isinstance(c.resolve("solo", {}).step, Action)


# ── Validation failures ─────────────────────────────────────────────────────


def _expect_invalid(tmp_path, text, match):
    path = tmp_path / "bad.yaml"
    path.write_text(text)
    c = Choreographer(path, ANIMATIONS)
    with pytest.raises(ValueError, match=match):
        c.load()


def test_bad_animation_rejected(tmp_path):
    _expect_invalid(tmp_path, (
        "choreographies:\n  e:\n    steps:\n"
        "      - {kind: led, animation: nonexistent_glow}\n"
    ), "nonexistent_glow")


def test_bad_gesture_ref_rejected(tmp_path):
    _expect_invalid(tmp_path, (
        "choreographies:\n  e:\n    steps:\n"
        "      - {kind: gesture, name: moonwalk}\n"
    ), "moonwalk")


def test_bad_keyframe_joint_rejected(tmp_path):
    _expect_invalid(tmp_path, (
        "gestures:\n  g:\n    keyframes:\n"
        "      - {t_ms: 0, joints: {left_eyelid: 0.5}}\n"
    ), "left_eyelid")   # physical joints are not wire joints


def test_keyframe_value_out_of_range_rejected(tmp_path):
    _expect_invalid(tmp_path, (
        "gestures:\n  g:\n    keyframes:\n"
        "      - {t_ms: 0, joints: {head_tilt: 1.5}}\n"
    ), "head_tilt")


def test_keyframes_not_increasing_rejected(tmp_path):
    _expect_invalid(tmp_path, (
        "gestures:\n  g:\n    keyframes:\n"
        "      - {t_ms: 0, joints: {head_tilt: 0.0}}\n"
        "      - {t_ms: 0, joints: {head_tilt: 0.5}}\n"
    ), "strictly increasing")


def test_first_keyframe_must_be_zero(tmp_path):
    _expect_invalid(tmp_path, (
        "gestures:\n  g:\n    keyframes:\n"
        "      - {t_ms: 100, joints: {head_tilt: 0.0}}\n"
    ), "t_ms 0")


def test_step_limit_violation_rejected(tmp_path):
    steps = "\n".join("      - {kind: blink}" for _ in range(65))
    _expect_invalid(
        tmp_path, f"choreographies:\n  e:\n    steps:\n{steps}\n", "64"
    )


# ── Hot reload ──────────────────────────────────────────────────────────────


def _bump_mtime(path):
    st = os.stat(path)
    os.utime(path, (st.st_atime, st.st_mtime + 2))


def test_reload_swaps_valid_file(tmp_choreo):
    c, path = tmp_choreo
    reloaded = []
    c.on_reload = lambda: reloaded.append(True)
    path.write_text(MINIMAL.replace("hello_event", "renamed_event"))
    assert c.try_reload() is True
    assert "renamed_event" in c.event_names
    assert "hello_event" not in c.event_names
    assert reloaded == [True]


def test_reload_keeps_old_on_invalid(tmp_choreo):
    c, path = tmp_choreo
    path.write_text("choreographies:\n  e:\n    steps:\n      - {kind: nope}\n")
    assert c.try_reload() is False
    assert "hello_event" in c.event_names           # old version stays live
    assert c.resolve("hello_event", {}) is not None


async def test_watch_task_reloads_on_mtime_change(tmp_choreo):
    import asyncio

    c, path = tmp_choreo
    c.start_watch(interval_s=0.05)
    try:
        path.write_text(MINIMAL.replace("hello_event", "watched_event"))
        _bump_mtime(path)
        for _ in range(50):
            await asyncio.sleep(0.05)
            if "watched_event" in c.event_names:
                break
        assert "watched_event" in c.event_names
    finally:
        c.stop_watch()
