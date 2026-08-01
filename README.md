# maki_puppet — ROS-free puppet control for the MAKI robot

`maki_puppet` is a single Python process that gives full control of the MAKI robot
(7 Dynamixel servos + 48-pixel LED ring on a Raspberry Pi 5) over one WebSocket.
Clients — the Bookbot Flutter app, Python scripts, debug tools — send semantic events
("the child finished a page") or direct actions ("blink"), and the gateway turns them
into smooth servo motion and LED animation. No ROS, no message broker, no launch files.

It **replaces the vendored ROS 2 stack** (`maki-rpi5-v0.2.13-rc.4/`) at runtime. The
two must **never run at the same time**: both take an exclusive `flock` on the same
lock file (`/tmp/maki_servo_ttyUSB0.lock`), so whichever starts second exits
immediately with a clear lock error instead of fighting over the serial bus. That
fail-fast mutual exclusion is deliberate — if `maki_puppet` won't start because of a
lock, something else is already driving the servos.

## Architecture

```
Flutter app ──ws──┐
Python procs ──ws─┤→ WebSocket server (MPP/1 protocol, port 8765)
                  │      ↓ events → choreographies.yaml lookup
                  │  ActionEngine (channels: MOTION | LED | VOICE; preempt/queue/drop)
                  │      ↓ actions (blink, look, gesture keyframes, led, …)
                  │  IdleBehavior (breathing LED + random blinks when idle)
                  │  VisionService (camera → face detection thread) ──┐
                  │      ↓ joint targets (radians) on named layers  ←──┘ `track` action
                  │  MotionService — 50 Hz thread:
                  │    layer blend → EMA → deadband → load-dampen → S-curve → clamp → SyncWrite
                  │      ↓ ticks                          ↓ pixels
                  └─ ServoBus (Dynamixel, flock'd)    LedRing (SPI, animation thread)
```

Everything above `ServoBus`/`LedRing` is hardware-neutral and runs anywhere; `--sim`
swaps in `SimServoBus`/`SimLedRing`/`SimCamera` so the entire stack (server, engine,
motion loop, face tracking) runs on a Mac with zero hardware dependencies.

## Face tracking

Ported from the ROS `maki_vision` package. **Starts with the gateway** — the camera
opens, detection runs, and `track` becomes available. The only requirement is OpenCV:

```bash
pip install -e ".[vision]"      # or use the Pi's system python3-opencv
```

The YuNet detection model is vendored in [`models/`](models/) and resolved
automatically, so a fresh `deploy_puppet.sh` needs no download step.

If the camera is missing or fails to open, startup logs it and the gateway comes up
anyway; `track` then rejects with `vision_unavailable`. To turn it off deliberately,
set `vision.enabled: false` in [`config/puppet.yaml`](config/puppet.yaml).

Clients drive it with the `track` action (`{"kind": "track", "duration_ms": 8000}`);
the gateway advertises `"vision"` in `welcome.capabilities` only when a camera is
actually running, so clients can feature-detect it. See
[PROTOCOL.md §6](PROTOCOL.md) for the action.

Tuning lives in the `vision.tracking` block of `puppet.yaml`. Those values are the
**robot's deployed ROS tuning**, carried across verbatim — not the ROS node's declared
defaults, which differ and were never what actually ran. The two knobs that matter
most are `camera_hfov_deg`/`camera_vfov_deg` (62/49, measured on this robot's Arducam IMX179 — wrong FOV
makes the head consistently over- or under-shoot, and no gain tuning compensates) and
`head_reanchor_threshold_rad` (lower tracks more tightly but fidgets more).

In `--sim` the camera and detector are replaced by a scripted person who walks back and
forth in front of the robot. It is closed-loop — turning the head really does bring the
face toward the centre of frame, and losing it out of frame really does stop the
detections — so tracking behaviour can be observed and tuned without hardware.

## Quick start — dev machine (no hardware)

```bash
cd maki-puppet
python -m venv .venv && source .venv/bin/activate
pip install -e '.[dev]'

pytest                            # motion math, engine, protocol, sim suites

python -m maki_puppet --sim       # full stack against simulated hardware
```

In a second terminal (same venv):

```bash
python -m maki_client --host localhost demo
```

## Quick start — robot

> **You need the robot's address, SSH user and venv path.** They are deliberately
> not in this repo — they live with the robot's own notes (`bookbot-maki`,
> `README_CONNECT_MAKI.md`). Ask for them, or read them off the robot. Everything
> below takes `<user>@<robot-ip>` as an argument, so nothing here is hardcoded.

From the repo root on your dev machine:

```bash
scripts/deploy_puppet.sh <user>@<robot-ip>     # or: ROBOT_HOST=user@host scripts/deploy_puppet.sh
```

This rsyncs the package to **`~/bookbot-maki`** on the robot (override with
`ROBOT_PUPPET_DIR`), installs it into the robot's venv (default `~/lux_robot_venv`,
override with `ROBOT_VENV`) with the `[robot]` hardware extras, and **stops** any
running gateway. It does not start one again — deploy, then restart:

```bash
ssh <user>@<robot-ip> '~/bookbot-maki/scripts/restart_puppet.sh'
```

`restart_puppet.sh` stops the running gateway with SIGTERM (so servos are torqued
off and locks released cleanly), starts it the same way the boot script does, and
waits until it actually serves — reporting the startup log instead of claiming
success for a process that died on a busy serial port. Use it rather than
rebooting: ~10 s versus ~90 s, and the Flutter app simply reconnects.

The remote directory **must** match the checkout the robot's boot script runs
(`start_bookbot_linux.sh`, default `~/bookbot-maki`). Deploying anywhere else
creates a second tree that nothing executes — a trap that has cost real debugging
time.

To run it by hand instead:

```bash
ssh <user>@<robot-ip>
~/bookbot-maki/scripts/run_puppet.sh
# equivalent to:
#   source ~/lux_robot_venv/bin/activate
#   cd ~/bookbot-maki && python -m maki_puppet --config config/puppet.yaml
```

Bench smoke test from your laptop (blink → look → nod → shake → LED cycle → home):

```bash
python scripts/smoke.py --host <robot-ip>
```

To run at boot, install the systemd user unit in
[`deploy/maki-puppet.service`](deploy/maki-puppet.service) — **only after** the bench
smoke test passes; instructions are in the unit's header comment.

## Where things are

| File | What it is |
|---|---|
| [`PROTOCOL.md`](PROTOCOL.md) | **MPP/1 wire contract — the source of truth.** Message types, action grammar, error codes, arbitration. |
| [`docs/FLUTTER.md`](docs/FLUTTER.md) | Guide for driving MAKI from the Flutter app (no robotics knowledge needed). |
| [`docs/LED_NOISE.md`](docs/LED_NOISE.md) | **Read before touching LED code to chase white flashes.** They are electrical noise from the servo bus, proven not to be a software fault. |
| [`config/puppet.yaml`](config/puppet.yaml) | Runtime config: server port, serial port, joint limits/signs, motion profiles, idle policy. |
| [`config/choreographies.yaml`](config/choreographies.yaml) | Semantic events (`celebrate`, `page_turned`, …) → action sequences, plus gesture keyframes. MAKI's personality lives here — retune without touching app code. |
| [`config/animations.yaml`](config/animations.yaml) | LED ring animation definitions. |
| `maki_puppet/vision/` | Face tracking: detection backends, the control law (`tracker.py`), and the capture thread. |
| [`models/`](models/) | Vendored vision models (YuNet face detection). No download step on deploy. |
| `clients/python/maki_client/` | Python SDK + CLI (`python -m maki_client`). |
| `scripts/smoke.py` | Bench acceptance script (run from the laptop against the robot). |
| `scripts/deploy_puppet.sh` | rsync this checkout to the robot + reinstall into its venv. Stops the gateway; does not restart it. |
| `scripts/restart_puppet.sh` | Stop and restart the gateway on the robot, waiting until it actually serves. Run this after a deploy. |
| `deploy/maki-puppet.service` | Optional systemd user unit for run-at-boot. |

## Making changes

**Where behaviour actually lives.** There are two places, and the split is not
obvious:

| | |
|---|---|
| [`maki_puppet/bridge.py`](maki_puppet/bridge.py) | The **Bookbot app events** — 15 of them (`tap_book`, `book_rate`, `tap_page`, …), one `on_<name>` handler each, plus the colour vocabulary as module constants. This is where the app's reactions live: the tap flourish, ratings holding their colour, the head-down posture while a book is open. |
| [`config/choreographies.yaml`](config/choreographies.yaml) | The **semantic events** (`celebrate`, `page_turned`, …) and the gesture keyframe library. |

**Bridge handlers take precedence**: an event named in `EVENT_PARAMS` shadows a
choreography of the same name. Both are advertised together in `welcome.events`.
So if editing a choreography changes nothing, check whether a bridge handler is
shadowing it.

**Adding an app event** takes two coupled edits — a name in `EVENT_PARAMS` *and*
a matching `async def on_<name>` method. A name without a handler raises at
dispatch; a handler without a name is never called. `tests/test_bridge.py`
asserts the two sets match exactly, so a mismatch fails the suite rather than
the robot.

**What reloads, and what does not.** `choreographies.yaml` is watched and
hot-reloaded on save. Everything else — `puppet.yaml`, `animations.yaml`, and all
Python — needs a gateway restart. After deploying, restart with
`~/bookbot-maki/scripts/restart_puppet.sh`; the gateway log is
`~/run_puppet_log.txt` on the robot (override with `MAKI_PUPPET_LOG`).

**Config beats code.** Values absent from `puppet.yaml` silently fall back to the
constants in the module that reads them, which may differ from what the robot was
tuned with. When comparing against a version that behaved differently, diff for
*absent* keys, not just changed ones — an absent key is not a match.

**Coupled values.** Some settings must be changed together; the config comments
say so at each site (e.g. an animation's `period` and whatever holds it for that
long). Search for "must be matched by" before changing a timing value.

**Verification ladder**, cheapest first:

```bash
pytest                                   # everything hardware-free (motion math, engine, protocol, sim)
python -m maki_puppet --sim              # whole stack, no hardware
python scripts/smoke.py --host <robot>   # on the real robot: blink → look → nod → LED → home
```

Sim cannot tell you about servo load, LED appearance, or timing under real
hardware — anything touching those needs the robot and a person watching.

## Autonomous behaviour

MAKI moves **without any client connected**. Two behaviours do this, and both
surprise people:

- **Idle** — random blinks and the resting LED animation after `idle.delay_s`.
- **Autonomous face-following** — `vision.auto_track.enabled` is **`true` by
  default**: the head follows whoever is in front of it, with no `track` action
  and no client involvement. The client-driven `track` action is a separate,
  explicit override.

Face-following **stands down while a `posture` is held** (`yield_to_posture`), so
opening a book pins the head at the page instead of chasing faces, and closing it
resumes following. If the head "won't stay where I put it", check whether you used
`pose` (momentary, its layer claim expires in ~1 s) rather than `posture`
(sustained until cleared) — see [PROTOCOL.md §6](PROTOCOL.md).

The gateway also runs scripted wake and sleep sequences at start and stop that
physically move the head and take a couple of seconds each.

## Troubleshooting

**"lock" error at startup / exits immediately.**
Something else holds the servo lock (`/tmp/maki_servo_ttyUSB0.lock`) or the LED lock
(`/tmp/maki_led_spi.lock`). Almost always this means the ROS 2 stack
(`ros2 launch maki_bringup …`) or another `maki_puppet` is running — find it with
`pgrep -af 'maki_puppet|servo_node|maki_bringup'` and stop it (Ctrl-C in its
terminal, or `pkill -f maki_puppet`). Stale locks cannot happen: `flock` is released
automatically when the holding process dies, so a lock failure always means a live
process owns the hardware.

**`ModuleNotFoundError: dynamixel_sdk` / `board` / `neopixel_spi`.**
The Pi-only hardware extras are not installed in the environment you're running.
On the robot: `source ~/lux_robot_venv/bin/activate && pip install -e "$HOME/bookbot-maki[robot]"`
(the deploy script does this for you). On a dev machine these packages are neither
needed nor installable — run with `--sim` instead.

**`track` is rejected with `vision_unavailable`.**
The gateway has no running camera, so it never advertised the `vision` capability.
Check `vision.enabled: true` in `puppet.yaml`, then look at the startup log: a camera
that fails to open logs `vision service failed to start` and the gateway continues
without it by design. `state.get` reports which case you're in —
`health.vision` is `"off"` (not configured), `"down"` (configured but not running),
or `"up"`/`"sim"`.

**`ModuleNotFoundError: cv2`.**
Face tracking needs OpenCV: `pip install -e ".[vision]"`, or the Pi's
`python3-opencv`. The detection model itself is vendored in `models/`, so this is the
only missing piece to expect.

**MAKI's head swings past faces and parks at its limit.**
Almost always `camera_hfov_deg`/`camera_vfov_deg` not matching the actual lens: the
gateway converts pixel offsets to angles with those values, so an FOV set too narrow
over-estimates every bearing. Verify against the camera's spec before touching gains.

**Robot is frozen with red LEDs and every command is rejected with `estopped`.**
A client engaged the e-stop: motion freezes and holds pose, queues are flushed, and
the ring shows `alarm_red` until it is released. Any client may disengage by sending
`estop` with `engage: false` (see PROTOCOL.md §5.7) — e.g. via the Python SDK or the
`maki_client` CLI. Normal idle behavior resumes on release. If nothing responds at
all, check the gateway process is still running before assuming e-stop.

**Servos won't move but the server is up.**
Check the `state` push (`python -m maki_client --host <robot> state`): `estop: true`
means see above; `health.servo` not `"up"` means the serial bus is unhappy — check
the USB serial adapter (an FT232H on this robot) on `/dev/ttyUSB0` and the servo power supply, then restart the gateway.
