# MAKI Puppet Protocol — MPP/1

**Status: normative.** This document is the source of truth for the wire contract between
the `maki_puppet` gateway (Python) and every client (Flutter/Dart, Python SDK, debug
tools). Both sides are implemented *against this document*, not against each other.
The JSON fixtures in [`tests/fixtures/`](tests/fixtures/) are the machine-readable
companion: every message type has a fixture that both the Python and Dart test suites
must parse successfully.

- Protocol version: **1** (`MPP/1`)
- Server: `maki_puppet` gateway, one process on the robot (Raspberry Pi 5)
- Reference clients: `maki_flutter_poc/lib/maki_client.dart`, `maki_puppet/clients/python/maki_client/`

---

## 1. Overview & architecture

MAKI is a small desk robot: a head with 7 servos (head pan/tilt, eye pan/tilt, two
eyelids, a mouth) and a 48-pixel LED ring. The gateway exposes it over a single
WebSocket as a **device-neutral puppet**: clients send *semantic events* ("the child
finished a page") or *direct actions* ("blink"), and the gateway turns them into servo
and LED activity.

```
Flutter app ──ws──┐
Python procs ──ws─┤→ WebSocket gateway (this protocol, MPP/1)
                  │      ↓ event {name} → choreographies.yaml lookup → Step tree
                  │  ActionEngine — channels: motion | led | tts; priority, on_busy, locks
                  │      ↓ joint targets (internal units) on named motion layers
                  │  MotionService (50 Hz: blend → smooth → clamp)   LedRing (50 Hz)
                  │      ↓                                              ↓
                  └─ 7 Dynamixel servos                            48 NeoPixels
```

Two things follow from "device-neutral" and are load-bearing for everything below:

1. **No hardware units on the wire.** No servo ticks, no radians, no pixel indices.
   Joints are normalized floats (§1.2). If the device layer is ever swapped (back to
   ROS, to a different robot), the protocol does not change.
2. **Semantic events are the preferred client path.** The mapping from
   `event {name: "celebrate"}` to physical motion lives in `config/choreographies.yaml`
   on the robot (§10) — MAKI's personality can be retuned without touching app code.

### 1.1 Message type registry

| Direction | Types |
|---|---|
| client → gateway | `hello`, `act`, `event`, `mouth`, `cancel`, `estop`, `state.get`, `lock`, `unlock`, `ping`, `pong` |
| gateway → client | `welcome`, `ack`, `state`, `event` (rebroadcast, carries `origin`), `ping`, `pong` |

There is no standalone `error` type: errors are delivered as `ack` with
`status: "error"` (§8). If the offending frame's `id` could not be parsed (e.g. bad
JSON), the error `ack` carries `"ref": null`.

### 1.2 Units convention (normative)

All joint values on the wire are **normalized floats**:

| Wire joint | Range | Meaning |
|---|---|---|
| `head_pan` | −1.0 … 1.0 | −1 = full left, +1 = full right of the safe range; 0 = neutral |
| `head_tilt` | −1.0 … 1.0 | **+1 = full down, −1 = full up**; 0 = neutral. (Matches the hardware sign and the authored gestures: `sleepy` droops to `+0.45`, `wake_up` rises from `+0.3` to `0.0`.) |
| `eyes_pan` | −1.0 … 1.0 | eye direction, same convention as head |
| `eyes_tilt` | −1.0 … 1.0 | |
| `eyelids` | 0.0 … 1.0 | **openness**: 0 = closed, 1 = fully open. One logical joint; the gateway drives both physical eyelids (incl. the mirrored right lid) |
| `mouth` | 0.0 … 1.0 | **openness**: 0 = closed, 1 = fully open |

- `0.0` is the configured neutral pose (defaults to the midpoint of each joint's safe
  range). "Safe range" is the mechanical limit table enforced server-side; clients can
  never command a position outside it.
- The physical joints `left_eyelid` and `right_eyelid` are **not** addressable on the
  wire; only the logical `eyelids` joint is.
- Values outside the documented range are rejected with error code `out_of_range`
  (§8.2). The server rejects rather than clamps: an out-of-range value is a client bug.
- Informative only (never on the wire): internally the gateway converts normalized →
  radians → ticks; the servo tick limit table is reproduced in Appendix C.

### 1.3 Channels

Every action routes to exactly one **channel**: `motion`, `mouth`, `led`, or `tts`.
Channels are the unit of arbitration (§9). A composed act (§7) *claims the union* of the
channels of all actions in its Step tree for its entire duration.

`mouth` is deliberately **separate from `motion`**: speech and gesture are independent
behaviors that must run concurrently. If the mouth shared the `motion` channel, every
viseme would arbitrate against head/eye gestures and one would preempt the other. For
the same reason, high-rate viseme streaming uses the dedicated `mouth` frame (§5.11)
rather than one `act` per sample.

`tts` is **deferred** in the current build: the `welcome.capabilities` list excludes
`"tts"` and any act containing a `say` action is rejected with `tts_unavailable`
(§6, §8.2). The channel is reserved so the protocol does not change when TTS lands.

---

## 2. Transport

- Endpoint: `ws://<robot-host>:8765/ws` (host/port from `config/puppet.yaml`;
  e.g. `ws://192.0.2.10:8765/ws`).
- WebSocket **text frames** only, UTF-8 JSON, exactly **one message per frame**.
- Maximum frame size: **64 KB**. Oversized frames close the connection with WS close
  code **1009**.
- No compression extensions required; TLS not used on the local network.

### 2.1 Close codes

| Code | Meaning |
|---|---|
| 1000 | Normal closure |
| 1001 | Going away — server shutdown, or client missed 2 consecutive heartbeat pings (§4.3) |
| 1009 | Frame exceeded 64 KB |
| 4400 | `hello.protocol` not supported by this server |
| 4401 | A frame other than `hello` arrived before the handshake completed |

---

## 3. Envelope

Every message in both directions is a JSON object with exactly this shape:

```json
{ "type": "act", "id": "c-000042", "ts": 1751421000123, "payload": { } }
```

| Field | Type | Rules |
|---|---|---|
| `type` | string | Required. One of the registry types (§1.1). Unknown type → `ack` `status:"error"`, `code:"unknown_type"`. |
| `id` | string | Required. Unique **per connection**. Client-generated ids: any string; the convention is `c-<counter>`. Server-generated ids are `s-<counter>`. The server does not deduplicate — reusing an id makes your own `ref`s ambiguous. |
| `ts` | integer | Required. Milliseconds since the Unix epoch, sender's clock. Informational; the server never validates it. |
| `payload` | object | Required (may be `{}`). Content per message type (§5). |

Conventions:

- Server messages that respond to a specific client message carry the triggering id in
  `payload.ref`.
- **Unknown fields must be ignored** everywhere, in both directions (forward
  compatibility, §13).
- Every client→gateway message that carries an `id` receives **at least one** response:
  - `act`, `event`, `cancel`, `estop`, `lock`, `unlock` → one or more `ack`s (§8)
  - `hello` → `welcome` (the welcome *is* the ack)
  - `state.get` → a `state` message with `ref` set
  - `ping` → `pong` with `payload.ref` set
  - `pong` → no response (it is itself the response to a server `ping`)

---

## 4. Handshake, versioning, heartbeat

### 4.1 `hello` → `welcome`

The **first frame** on every connection must be `hello`. Any other frame first →
close **4401**. If `payload.protocol` is not supported, the server sends an `ack`
with `code:"unsupported_protocol"` and closes with **4400**.

The server replies with `welcome` (§5.2), which advertises the server's catalogs
(joints, animations, gestures, events). **Clients must not hardcode catalogs** — render
buttons/options from `welcome`.

### 4.2 Versioning & sessions

- `protocol` is a single integer. Rules for what may change without bumping it: §13.
- `welcome.session` is a random string generated at gateway process start. If a
  reconnecting client sees a different `session`, all cached state (catalogs, queue
  contents, lock ownership) is void.
- Connections are stateless from the client's perspective: reconnect = new `hello`.
  Message ids are scoped per connection; acks for a dead connection are discarded.
  In-flight work started by a disconnected client keeps running unless that client set
  `abort_on_disconnect: true` in its `hello`.

### 4.3 Heartbeat

- The server sends `ping` every **10 s**; the client must answer `pong` with
  `payload.ref` = the ping's id. **Two consecutive missed pongs** → server closes 1001.
- Clients may also send `ping` at any time (RTT measurement); the server answers `pong`
  immediately with `payload.ref` set.

---

## 5. Message reference

Every type, with a complete example. All examples are valid MPP/1 frames and are
mirrored 1:1 in `tests/fixtures/`.

### 5.1 `hello` (client → gateway)

```json
{ "type": "hello", "id": "c-1", "ts": 1751421000000, "payload": {
    "protocol": 1,
    "client": { "name": "bookbot-flutter", "kind": "flutter", "version": "0.1.0" },
    "priority": 50,
    "subscribe": ["state", "events"],
    "abort_on_disconnect": false
}}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `protocol` | int | required | Must be `1`. |
| `client.name` | string | required | Human-readable; shows up in `state.clients` and `event.origin`. |
| `client.kind` | string | required | `"flutter"` \| `"python"` \| `"debug"`. |
| `client.version` | string | `""` | Client build version, for logs. |
| `priority` | int | `50` | Default priority for this connection's acts/events, 0–79 (§9.1). |
| `subscribe` | string[] | `[]` | `"state"` → receive `state` pushes (§11); `"events"` → receive rebroadcasts of *other* clients' semantic events. |
| `abort_on_disconnect` | bool | `false` | If true, this client's queued + running acts are cancelled when its socket drops. |

### 5.2 `welcome` (gateway → client)

```json
{ "type": "welcome", "id": "s-1", "ts": 1751421000050, "payload": {
    "ref": "c-1",
    "protocol": 1,
    "server": { "name": "maki-puppet", "version": "0.1.0" },
    "session": "af52c1",
    "capabilities": ["motion", "mouth", "led"],
    "joints": {
      "head_pan":  { "min": -1.0, "max": 1.0, "neutral": 0.0 },
      "head_tilt": { "min": -1.0, "max": 1.0, "neutral": 0.0 },
      "eyes_pan":  { "min": -1.0, "max": 1.0, "neutral": 0.0 },
      "eyes_tilt": { "min": -1.0, "max": 1.0, "neutral": 0.0 },
      "eyelids":   { "min": 0.0, "max": 1.0, "neutral": 1.0 },
      "mouth":     { "min": 0.0, "max": 1.0, "neutral": 0.0 }
    },
    "animations": ["off", "breathing_cyan", "breathing_white", "thinking_pulse_blue",
                   "rainbow_swirl", "chase_rainbow", "attentive_green", "observing_white",
                   "sleeping_dim_blue", "breathing_slow_purple", "sleep_deep_breathe",
                   "breathing_blue", "sparkle_soft", "glow_gold", "amber_idle",
                   "surprise_flash", "warm_pulse", "cool_wave", "attention_sweep",
                   "speaking_pulse", "concurrent_listen", "red_alert", "boot_pulse",
                   "alarm_red", "pulse_yellow", "activity_joinable", "activity_focused",
                   "clear_activity"],
    "gestures": ["nod", "head_shake", "happy_wiggle", "curious_tilt", "wake_up", "sleepy"],
    "events": ["reading_started", "page_turned", "word_read", "word_struggled",
               "sentence_read", "celebrate", "encourage", "attention",
               "reading_finished", "sleep", "wake"]
}}
```

- `capabilities`: subset of `["motion", "mouth", "led", "tts"]`. **Current builds exclude
  `"tts"`** (§1.3). Feature-detect the viseme stream by checking for `"mouth"`.
- `animations` / `gestures` / `events`: the live catalogs. `events` is the union of
  choreography names and `ignored_events` from `choreographies.yaml` — i.e. every name
  the server will accept without `unknown_event`. (In the catalog above,
  `sentence_read` is accepted but currently a deliberate no-op.)
- Catalogs can change while connected (hot-reload of `choreographies.yaml`, §10.4);
  re-fetch with `state.get {fields: ["catalog"]}`.

### 5.3 `act` (client → gateway)

Direct action execution. `payload.do` is one **Step** (grammar in §7).

Single action:

```json
{ "type": "act", "id": "c-7", "ts": 1751421001200, "payload": {
    "do": { "kind": "blink" },
    "priority": "normal",
    "on_busy": "queue",
    "tag": "ui-idle"
}}
```

Composed (nod twice while the LEDs sweep, then settle):

```json
{ "type": "act", "id": "c-8", "ts": 1751421002000, "payload": {
    "priority": "high",
    "on_busy": "replace",
    "do": { "seq": [
        { "par": [
            { "kind": "gesture", "name": "nod", "repeat": 2 },
            { "kind": "led", "animation": "attention_sweep" }
        ]},
        { "kind": "led", "animation": "breathing_cyan" }
    ]}
}}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `do` | Step | required | §7. |
| `priority` | int \| string | connection's `hello.priority` | `"low"`=20, `"normal"`=50, `"high"`=70, or an integer 0–79. Values ≥80 are reserved for the server-internal safety layer → error `out_of_range`. |
| `on_busy` | string | `"queue"` | `"queue"` \| `"replace"` \| `"drop"` — see §9.2. |
| `tag` | string | absent | Free-form label for group cancellation (`cancel {target: "tag:x"}`). |

### 5.4 `event` (client → gateway) — the preferred path

```json
{ "type": "event", "id": "c-9", "ts": 1751421003000, "payload": {
    "name": "word_read",
    "params": { "word": "elephant", "streak": 5 }
}}
```

| Field | Type | Default | Meaning |
|---|---|---|---|
| `name` | string | required | Resolved against `choreographies.yaml` (§10). Unknown name → error `unknown_event`, **unless** listed in `ignored_events`, in which case the server acks `status:"completed"` with `detail:"no-op"`. |
| `params` | object | `{}` | Free-form. Available to `{{param|default}}` templates in the choreography (§10.3) and included verbatim in the rebroadcast to `events` subscribers. |

The resolved choreography runs with the **priority and `on_busy` defined in the YAML**
(per-event, falling back to the file's `defaults`); the client cannot override them —
that tuning belongs to the robot, not the app.

### 5.5 `event` rebroadcast (gateway → client)

Sent to every *other* connection that subscribed to `"events"`, whenever any client's
`event` is accepted (including no-op ignored events):

```json
{ "type": "event", "id": "s-41", "ts": 1751421003020, "payload": {
    "name": "word_read",
    "params": { "word": "elephant", "streak": 5 },
    "origin": "bookbot-flutter"
}}
```

`origin` is the sending client's `hello.client.name`. Rebroadcasts are
informational — they never receive acks.

### 5.6 `cancel` (client → gateway)

```json
{ "type": "cancel", "id": "c-10", "ts": 1751421004000, "payload": { "target": "c-8" } }
```

`target` is one of:

| Form | Effect |
|---|---|
| `"<id>"` | Cancel the act/event with that envelope id (yours only — ids are per-connection). |
| `"all"` | Flush **all** queued and running client work, from every connection. |
| `"tag:<x>"` | Cancel every queued/running act carrying `tag == "x"` (any connection). |

Each victim receives a terminal `ack` `status:"cancelled"` on its own connection. The
`cancel` itself is acked `completed` (with `detail` noting how many items were
cancelled; cancelling an id that is already terminal or unknown is **not** an error —
it acks `completed`, `detail:"no-op"`).

### 5.7 `estop` (client → gateway)

```json
{ "type": "estop", "id": "c-13", "ts": 1751421005000, "payload": {
    "engage": true,
    "reason": "child pressed stop"
}}
```

`engage: true` → motion freezes and holds the current pose, all channel queues are
flushed (every in-flight act acked `cancelled`), the LED ring switches to `alarm_red`,
and all subsequent `act`/`event` messages are rejected with `estopped`.
`engage: false` (from **any** client) releases: motion resumes accepting targets and
the LED returns to the configured default animation. E-stop ignores locks and
priorities — it always wins (§9.4). Both directions ack `completed`.

### 5.8 `state.get` (client → gateway)

```json
{ "type": "state.get", "id": "c-14", "ts": 1751421006000, "payload": {
    "fields": ["pose", "queue"]
}}
```

`fields` (optional; omitted = everything) selects top-level keys of the state payload
(§11), plus the pseudo-field `"catalog"` which returns the `animations`/`gestures`/
`events` lists in `welcome` format. Reply is a `state` message with `ref` set. Works
regardless of whether the client subscribed to `"state"`.

### 5.9 `lock` / `unlock` (client → gateway)

```json
{ "type": "lock", "id": "c-15", "ts": 1751421007000, "payload": {
    "scopes": ["motion", "led"], "ttl_s": 30
}}
```
```json
{ "type": "unlock", "id": "c-16", "ts": 1751421008000, "payload": {
    "scopes": ["motion", "led"]
}}
```

Exclusive per-connection channel locks — see §9.3. `ttl_s` defaults to 30, max **300**
(larger → `out_of_range`). Granted → ack `completed`; contested → ack error `locked`.
`unlock` releases only scopes you hold (releasing a scope you don't hold is a no-op,
acked `completed`).

### 5.10 `ping` / `pong` (both directions)

```json
{ "type": "ping", "id": "s-33", "ts": 1751421012000, "payload": {} }
```
```json
{ "type": "pong", "id": "c-17", "ts": 1751421012040, "payload": { "ref": "s-33" } }
```

§4.3 for the heartbeat rules. A `pong` must carry `payload.ref` = the ping id it
answers.

### 5.11 `ack` (gateway → client)

One `ack` per lifecycle transition of the referenced message (state machine in §8).

```json
{ "type": "ack", "id": "s-12", "ts": 1751421002050, "payload": {
    "ref": "c-8",
    "status": "started",
    "channels": ["motion", "led"],
    "queue_pos": 0,
    "progress": { "step": 1, "of": 2 }
}}
```

```json
{ "type": "ack", "id": "s-19", "ts": 1751421002900, "payload": {
    "ref": "c-9",
    "status": "completed",
    "choreography": "word_read",
    "duration_ms": 850
}}
```

```json
{ "type": "ack", "id": "s-20", "ts": 1751421003100, "payload": {
    "ref": "c-21",
    "status": "error",
    "code": "unknown_animation",
    "detail": "no animation 'sparkle_red'; see welcome.animations"
}}
```

| Field | Type | Present |
|---|---|---|
| `ref` | string \| null | Always. `null` only when the offending frame's id could not be parsed (`bad_json`). |
| `status` | string | Always. §8.1. |
| `code` | string | On `status:"error"`. §8.2. |
| `detail` | string | Optional human-readable elaboration. Never parse it programmatically. |
| `channels` | string[] | On `accepted`/`queued`/`started`: channels the act claims. |
| `queue_pos` | int | On `queued`: 0-based position; may be repeated with decreasing values as the queue drains. |
| `progress` | object | Optional on `started` for multi-step sequences: `{step, of}` (1-based). |
| `choreography` | string | On acks for `event` messages: the resolved choreography name. |
| `duration_ms` | int | Optional on `completed`: wall-clock execution time. |

### 5.12 `state` (gateway → client)

Pushed to `"state"` subscribers on change plus a 1 Hz heartbeat snapshot; also the
reply to `state.get` (then with `ref`). Full field reference in §11.

```json
{ "type": "state", "id": "s-40", "ts": 1751421010000, "payload": {
    "pose": { "head_pan": 0.12, "head_tilt": -0.05, "eyes_pan": 0.0,
              "eyes_tilt": 0.1, "eyelids": 1.0, "mouth": 0.0 },
    "led": { "animation": "breathing_cyan" },
    "queue": { "depth": 2, "active_id": "c-8", "active_client": "bookbot-flutter" },
    "estop": false,
    "lock": { "held_by": null, "scopes": [] },
    "clients": [ { "name": "bookbot-flutter", "kind": "flutter" },
                 { "name": "reading-brain", "kind": "python" } ],
    "health": { "servo": "up", "led": "up", "loop_hz": 49.9 }
}}
```

### 5.13 `mouth` (client → gateway) — the viseme stream

A single mouth sample, for lip-sync during speech. Send these continuously while
audio plays (typically 30–60 Hz):

```json
{ "type": "mouth", "id": "c-77", "ts": 1751421009000, "payload": { "openness": 0.62 } }
```

| Field | Type | Default | Notes |
|---|---|---|---|
| `openness` | number | required | `0.0` = closed, `1.0` = fully open. Out of range → `out_of_range`. |

This frame is **fire-and-forget: there is no ack on success.** A sender streaming at
60 Hz cannot consume one ack per sample, and an ack would not tell it anything
actionable. Only a malformed frame produces an `ack` with `status: "error"`.

It is also the one client frame that **bypasses the action engine** (§9): no
`Performance` is allocated, no channel is claimed, no queue is touched. The sample
goes straight onto the `expression` motion layer, which merges targets — so a
concurrent `blink` keeps its eyelid targets and only the mouth is overwritten.
The cost of one sample is a JSON parse and a dict write.

Consequences worth knowing:

- **Latency** is one motion tick, not one round trip. Loopback transport is ~0.2 ms
  one-way; the `motion.rate_hz` tick (default 50 Hz) adds 0–20 ms. The mouth runs
  with `planning_enabled: false` (passthrough), so no trajectory planner smooths it.
- **Locks and `on_busy` do not apply.** A client holding a `motion` lock cannot block
  another client's mouth stream. If you need exclusivity, lock the `mouth` channel.
- **E-stop still wins.** MotionService freezes output while engaged (§9.4), so mouth
  writes have no effect until it is released.
- **Nothing else commands the mouth.** `neutral` (§6) and the idle behavior both
  exclude it, so a choreography ending mid-word will not snap it shut. The
  `{"kind": "mouth"}` *action* still exists for scripted, non-streaming use and
  claims the `mouth` channel.
- **Stop cleanly.** Send `openness: 0` when speech ends. If the stream simply stops,
  the `expression` layer fades out after its `claim_timeout_s` (1.5 s) instead.

---

## 6. Action kinds reference

Each action is a JSON object with a `kind` plus kind-specific arguments. Unknown
`kind` → error `unknown_type` (with `detail` naming the kind). Unlisted/unknown
arguments are ignored.

| Kind | Channel | Arguments (default) | Behavior |
|---|---|---|---|
| `blink` | motion | `duration_ms` (150) | Close and reopen the eyelids once. |
| `look` | motion | `pan` (0.0), `tilt` (0.0) in [−1,1]; `eyes_only` (false); `duration_ms` (600) | Smoothly orient toward (pan, tilt). `eyes_only:true` moves only the eyes; otherwise head and eyes move together. |
| `eyelids` | motion | `openness` (required) in [0,1]; `duration_ms` (300) | Set eyelid openness (both lids, mirrored). |
| `mouth` | motion | `openness` (required) in [0,1]; `duration_ms` (200) | Set mouth openness. |
| `pose` | motion | `joints` (required): map of wire joints (§1.2) → normalized values; `duration_ms` (800) | Move an arbitrary subset of joints. Unknown joint name → `unknown_joint`; value out of range → `out_of_range`. |
| `gesture` | motion | `name` (required); `repeat` (1, max 10); `intensity` (1.0, range 0–2] | Play a named keyframe gesture from the server library (§10.2). Unknown name → `unknown_gesture`. `intensity` scales joint amplitudes (server clamps the result to safe ranges). |
| `led` | led | exactly one of `animation` (name from `welcome.animations`) or `color` `{r,g,b}` 0–255 each | Switch the ring to a named animation, or a static solid color. Unknown animation → `unknown_animation`; both/neither argument → `out_of_range`. |
| `say` | tts | `text` (required) | **Deferred.** While `"tts"` is absent from `welcome.capabilities`, any wire act containing `say` is rejected whole with `tts_unavailable`. (§1.3; choreography-side `say` steps are instead skipped at runtime — §10.5.) |
| `wait` | — | `duration_ms` (required, 0–60000) | Pause. Occupies no channel by itself; inside a composed act it simply delays within the act's existing channel claim. |
| `neutral` | motion | `duration_ms` (700) | Return all joints to the neutral pose. |

General argument rules:

- All `duration_ms` values: integers, 0–60000; outside → `out_of_range`.
- All normalized values are validated against the ranges in §1.2 and **rejected**, not
  clamped, when out of range.
- Numbers may be sent as JSON ints where a float is expected (`"pan": 0` is fine).

---

## 7. Step composition grammar

```
Step := Action                 -- one object from §6
      | { "seq": [Step, ...] } -- run children in order
      | { "par": [Step, ...] } -- run children concurrently; the par completes when all children complete
```

Limits (violation → error `out_of_range` with detail):

- Maximum nesting depth **4** (a bare Action is depth 1; each `seq`/`par` wrapper adds
  one level).
- Maximum **64** Actions total per act.
- `seq`/`par` arrays must contain at least 1 step.

Semantics:

- The act **claims the union of channels** used anywhere in its tree, from `started`
  until its terminal ack. (So a `seq` of gesture-then-led holds *both* `motion` and
  `led` for the whole sequence — the LED can't be stolen mid-choreography by an
  equal-priority act.)
- Two children of the same `par` that drive the same channel are legal but the later
  targets win per tick on `motion` and last-write-wins on `led`; prefer disjoint
  channels in a `par`.
- Cancellation/supersession takes effect between (and inside long-running) actions;
  motion always hands off smoothly — the planner blends, never snaps.
- An act whose tree contains only `wait` actions claims no channel and simply delays
  before acking `completed`.

---

## 8. Ack lifecycle

### 8.1 Status state machine

```
              (validation failure) ──────────────► error       [terminal]
              (channel busy, on_busy=drop) ──────► dropped     [terminal]
frame ──► accepted ──┬──► started ───────────────► completed   [terminal]
                     │        │
                     └► queued┘  (queued → started when the channel frees)
         any pre-terminal state ────────────────► cancelled    [terminal]  (cancel / estop / abort_on_disconnect)
         accepted | queued | started ───────────► superseded   [terminal]  (replaced under §9.2)
```

Guarantees:

- Exactly **one terminal ack** per `act`/`event`: `completed`, `cancelled`,
  `superseded`, `dropped`, or `error`.
- `dropped` and `error` are sent as a **single** ack (no preceding `accepted`).
- Otherwise `accepted` is always the first ack; `queued` appears only if the act had to
  wait; `started` always precedes `completed`.
- Non-act messages (`cancel`, `estop`, `lock`, `unlock`) use only
  `completed` / `error` — single ack.

### 8.2 Error codes

| `code` | Meaning |
|---|---|
| `bad_json` | Frame was not valid JSON or not a valid envelope. `ref` may be `null`. |
| `unsupported_protocol` | `hello.protocol` not supported; connection then closes 4400. |
| `unknown_type` | Unknown envelope `type`, or unknown action `kind` inside `act.do` (see `detail`). |
| `unknown_event` | `event.name` not in choreographies and not in `ignored_events`. |
| `unknown_animation` | `led.animation` not in the animation catalog. |
| `unknown_gesture` | `gesture.name` not in the gesture library. |
| `unknown_joint` | `pose.joints` used a name not in §1.2. |
| `out_of_range` | A numeric argument outside its documented range; priority ≥80; Step limits exceeded (§7); malformed `led` args; `ttl_s` > 300. |
| `busy` | Channel is held by server-internal safety work that client work cannot preempt or queue behind. Retry later. |
| `locked` | Channel locked by another connection (§9.3). |
| `estopped` | E-stop engaged; `act`/`event` rejected (§9.4). |
| `queue_full` | The per-channel queue is at capacity (16). |
| `tts_unavailable` | Act contains `say` while `"tts"` is not in capabilities. |
| `internal` | Unexpected server error; `detail` has a hint, server logs have the trace. |

Clients must tolerate error codes not in this table (treat as `internal`; §13).

---

## 9. Arbitration

Multiple clients (the Flutter app, Python scripts, a debug console) can be connected at
once. Arbitration is per-channel (`motion`, `led`, `tts`).

### 9.1 Priority

- Effective priority of an act: `payload.priority` if present, else the connection's
  `hello.priority`, else 50. Aliases: `"low"`=20, `"normal"`=50, `"high"`=70.
- Valid client range **0–79**. The server-internal safety layer runs at 100 and can
  never be preempted or locked out by clients; client priority ≥80 → `out_of_range`.
- Events use the priority configured in `choreographies.yaml` (§5.4).

### 9.2 `on_busy` — what happens when a claimed channel is contested

When an incoming act's channel claim intersects the **active** act's channels:

| Condition | `on_busy` of the *incoming* act | Result |
|---|---|---|
| `incoming.priority ≥ active.priority` | `replace` | Active act gets terminal ack `superseded`; incoming starts immediately. |
| `incoming.priority ≥ active.priority` | `queue` | Incoming is enqueued (ack `queued`). |
| `incoming.priority ≥ active.priority` | `drop` | Incoming gets single ack `dropped`. |
| `incoming.priority < active.priority` | `replace` **or** `queue` | Incoming is enqueued — `replace` degrades to `queue`; lower priority never preempts. |
| `incoming.priority < active.priority` | `drop` | Single ack `dropped`. |

- Queues are **per-channel FIFO, max depth 16** (overflow → `queue_full`). Priority
  governs preemption of the *active* act only; the queue itself is strict FIFO — no
  reordering, no starvation surprises.
- An act claiming multiple channels must win/queue on **all** of them atomically; it
  supersedes every active act it overlaps (each victim acked `superseded`).
- When the channels are idle, any `on_busy` value starts immediately.

### 9.3 Locks

- `lock {scopes, ttl_s}` grants the connection **exclusive write access** to those
  channels: other connections' acts/events touching a locked channel are rejected with
  `locked` (regardless of priority).
- Auto-released on disconnect and on TTL expiry (max 300 s). Re-locking scopes you
  already hold refreshes the TTL.
- Locks do not stop the server's own idle behavior or safety layer, and never stop
  e-stop.

### 9.4 E-stop precedence

E-stop beats everything: locks, priorities, queues. On engage — queues flushed
(victims acked `cancelled`), motion freezes and holds, LED → `alarm_red`, subsequent
`act`/`event` → `estopped`. `cancel`, `state.get`, `lock`/`unlock`, `estop` remain
usable while engaged. Any client may disengage. `state.estop` reflects the current
condition.

---

## 10. Semantic events & `choreographies.yaml`

### 10.1 File schema

`config/choreographies.yaml` on the robot (hot-reloaded, §10.4). Actual current file:
[`config/choreographies.yaml`](config/choreographies.yaml).

```yaml
version: 1                       # schema version of this file (not the wire protocol)

defaults:                        # applied to any choreography that omits them
  priority: normal               # low | normal | high | 0..79
  on_busy: replace               # queue | replace | drop

ignored_events: [sentence_read]  # accepted on the wire, acked completed/no-op

choreographies:
  <event_name>:
    description: <string>        # documentation only
    priority: <as defaults>      # optional
    on_busy: <as defaults>       # optional
    steps:                       # a Step LIST == implicit top-level seq;
      - <Step>                   # Steps use EXACTLY the wire grammar (§6, §7)
      - ...

gestures:
  <gesture_name>:
    keyframes:                   # played on the motion service's gesture layer
      - { t_ms: <int>, joints: { <wire_joint>: <normalized float>, ... } }
      - ...                      # t_ms strictly increasing from 0
```

- Steps reuse the **exact wire Step grammar** — one parser serves both the WebSocket
  and the YAML. The same limits apply (depth ≤4, ≤64 actions).
- Gesture keyframes use **wire joints and wire units** (§1.2), including the logical
  `eyelids` joint. Between keyframes the motion service's S-curve planner interpolates
  and smooths; keyframes are targets, not raw servo commands.

### 10.2 Current event catalog (normative for this repo revision)

| Event | Priority | `on_busy` | Physical behavior |
|---|---|---|---|
| `reading_started` | high | replace | Warm LED pulse + wake-up gesture, look at the reader, blink. |
| `page_turned` | normal | replace | Glance down toward the page, blink, settle to neutral. |
| `word_read` | **low** | **drop** | Single blink. Deliberately cheap and droppable — per-word events must never build lag. |
| `word_struggled` | normal | replace | Cool LED wave + curious head tilt (lean-in, no judgement). |
| `sentence_read` | — | — | **Ignored** (valid, acked `completed`/no-op). Reserved. |
| `celebrate` | high | replace | Rainbow sweep + double happy wiggle, then back to breathing cyan. |
| `encourage` | normal | replace | Warm pulse + nod. |
| `attention` | high | replace | Rainbow sweep + wake-up gesture. |
| `reading_finished` | high | replace | Gold glow + double nod, settle, back to breathing cyan. |
| `sleep` | normal | replace | Dim blue breathing + eyes-droop gesture. |
| `wake` | high | replace | Breathing cyan + wake-up gesture. |

Gesture library: `nod`, `head_shake`, `happy_wiggle`, `curious_tilt`, `wake_up`,
`sleepy`.

### 10.3 Parameter templating

Any **string value** inside a choreography's steps may contain `{{param}}` or
`{{param|default}}`. At execution the gateway substitutes `event.params[param]`
(else the default; else empty string). If the *entire* value is a single template and
the parameter is a number/bool, the substituted value keeps its JSON type. The current
file uses no templates; the mechanism is normative and reserved (it exists so future
choreographies like `say: "{{text|Great job!}}"` need no protocol change).

### 10.4 Hot reload

The gateway watches the file's mtime. On change it re-validates (joint names against
§1.2, animation names against `animations.yaml`, gesture references, Step limits) and
swaps atomically — an invalid file is rejected and logged, the old version stays live.
After a successful swap the gateway sends a `state` push so subscribers know to refresh
catalogs via `state.get {fields:["catalog"]}`.

### 10.5 `say` in choreographies while TTS is deferred

A wire `act` containing `say` fails whole with `tts_unavailable` (fail loud — the
client is asking for something the robot advertised it can't do). A **choreography**
containing `say` steps instead has those steps *skipped at execution* with a server-side
log warning — so a personality edit on the robot can never hard-break a reading session.

---

## 11. State pushes

Pushed to `"state"` subscribers **on change** and as a **1 Hz** heartbeat snapshot;
also the reply to `state.get`. Full example in §5.12.

| Field | Type | Meaning |
|---|---|---|
| `pose` | object | Latest measured pose, wire joints/units (§1.2). `eyelids` reports the mean openness of both physical lids. |
| `led` | object | Exactly one of `{"animation": "<name>"}` or `{"color": {"r","g","b"}}`. |
| `queue` | object | `depth` (queued acts, all channels), `active_id` (envelope id of the running act, or `null`), `active_client` (its owner's name, or `null`). |
| `estop` | bool | E-stop engaged. |
| `lock` | object | `held_by` (client name or `null`), `scopes` (locked channels). |
| `clients` | array | Connected clients: `{name, kind}`. |
| `health` | object | `servo`: `"up"` \| `"sim"` \| `"degraded"` (bus errors, holding last-known feedback); `led`: `"up"` \| `"sim"`; `loop_hz`: measured motion-loop rate. Additive fields may appear (§13). |

On-change pushes are rate-limited to the state of the world, not per-ack — clients
needing per-action lifecycle detail should use acks, not state.

---

## 12. Client requirements (summary)

A conforming client must:

1. Send `hello` first; treat anything before `welcome` as handshake failure.
2. Answer server `ping` with `pong` (`payload.ref` set) within 10 s.
3. Ignore unknown message fields; surface unknown error codes as generic failures.
4. Never hardcode animation/gesture/event catalogs — read them from `welcome`.
5. Use per-connection unique ids; correlate acks by `payload.ref`.
6. On reconnect: new `hello`; if `welcome.session` changed, drop all cached state.
7. Reconnect with exponential backoff (reference policy: 0.5 s doubling to 8 s cap,
   ±25 % jitter, reset on successful `welcome`).

Reference client contracts (Dart: [`docs/FLUTTER.md`](docs/FLUTTER.md); Python:
`clients/python/maki_client/`) build on these rules.

---

## 13. Compatibility policy

Within `protocol: 1`:

- **May change without notice (clients must tolerate):** new envelope-payload fields
  anywhere; new action kinds; new error codes; new animations, gestures, events; new
  `state`/`health` fields; new `welcome` fields.
- **Will not change:** the envelope shape; the meaning/order of existing ack statuses
  (no new statuses within protocol 1); existing action kinds' argument names, units and
  defaults; existing error codes' meanings; the units convention (§1.2); close-code
  semantics.
- Anything in the "will not change" list requires `protocol: 2`, negotiated via
  `hello`/`welcome` (a v2 server may still accept v1 hellos).

---

## Appendix A — Animation catalog (from `config/animations.yaml`)

Informative — the normative list is `welcome.animations` at runtime.

| Name | Type | Color (R,G,B) | Period (s) | Intended use |
|---|---|---|---|---|
| `off` | static | 0,0,0 | — | Ring dark |
| `breathing_cyan` | breathing | 0,90,180 | 4.0 | Default idle |
| `breathing_white` | breathing | 150,150,150 | 5.0 | |
| `thinking_pulse_blue` | breathing | 0,100,255 | 3.0 | "Thinking" |
| `rainbow_swirl` | rainbow | — | 5.0 | Playful |
| `chase_rainbow` | chase_rainbow | — | 2.5 | Spatial rainbow chase |
| `attentive_green` | breathing | 0,180,0 | 3.0 | |
| `observing_white` | breathing | 255,255,255 | 3.5 | |
| `sleeping_dim_blue` | breathing | 0,0,60 | 8.0 | Sleep |
| `breathing_slow_purple` | breathing | 60,0,80 | 8.0 | Wind-down |
| `sleep_deep_breathe` | breathing | 30,0,45 | 12.0 | Deep sleep |
| `breathing_blue` | breathing | 0,80,220 | 3.0 | Used by `sleep` event |
| `sparkle_soft` | breathing | 160,70,200 | 2.4 | |
| `glow_gold` | breathing | 200,160,40 | 4.8 | Used by `reading_finished` |
| `amber_idle` | breathing | 120,100,60 | 5.6 | |
| `surprise_flash` | breathing | 200,160,30 | 2.5 | Startle (non-strobing) |
| `warm_pulse` | breathing | 220,130,40 | 3.5 | Encourage / warm |
| `cool_wave` | breathing | 40,160,200 | 3.2 | Calm / empathy |
| `attention_sweep` | rainbow | — | 1.8 | Celebrate / attention |
| `speaking_pulse` | breathing | 150,60,60 | 3.0 | Reserved for TTS |
| `concurrent_listen` | breathing | 0,140,120 | 4.0 | |
| `red_alert` | breathing | 220,0,0 | 1.5 | Safety |
| `boot_pulse` | breathing | 0,120,180 | 4.0 | Startup |
| `alarm_red` | breathing | 255,0,0 | 1.2 | **E-stop indicator** |
| `pulse_yellow` | breathing | 220,180,0 | 3.0 | Warnings |
| `activity_joinable` | breathing | 200,140,20 | 4.0 | |
| `activity_focused` | static | 120,80,10 | — | |
| `clear_activity` | breathing | 0,90,180 | 4.0 | |

## Appendix B — Worked session transcript

Client frames left, server frames right (envelope `ts` omitted for brevity):

```
→ hello   c-1  {protocol:1, client:{name:"bookbot-flutter",kind:"flutter"}, subscribe:["state"]}
←                            welcome s-1  {ref:"c-1", session:"af52c1", capabilities:["motion","led"], ...}
→ event   c-2  {name:"reading_started"}
←                            ack s-2  {ref:"c-2", status:"accepted", channels:["motion","led"]}
←                            ack s-3  {ref:"c-2", status:"started", progress:{step:1,of:3}}
←                            state s-4 {... led:{animation:"warm_pulse"} ...}
←                            ack s-5  {ref:"c-2", status:"completed", choreography:"reading_started", duration_ms:1900}
→ event   c-3  {name:"word_read", params:{word:"the"}}
→ event   c-4  {name:"word_read", params:{word:"cat"}}      (while c-3 still blinking)
←                            ack s-6  {ref:"c-3", status:"accepted", channels:["motion"]}
←                            ack s-7  {ref:"c-3", status:"started"}
←                            ack s-8  {ref:"c-4", status:"dropped"}          (word_read is on_busy: drop)
←                            ack s-9  {ref:"c-3", status:"completed"}
→ event   c-5  {name:"celebrate"}
←                            ack s-10 {ref:"c-5", status:"accepted", channels:["motion","led"]}
←                            ack s-11 {ref:"c-5", status:"started"}
→ estop   c-6  {engage:true, reason:"demo"}
←                            ack s-12 {ref:"c-5", status:"cancelled"}
←                            ack s-13 {ref:"c-6", status:"completed"}
←                            state s-14 {... estop:true, led:{animation:"alarm_red"} ...}
→ estop   c-7  {engage:false}
←                            ack s-15 {ref:"c-7", status:"completed"}
←                            state s-16 {... estop:false, led:{animation:"breathing_cyan"} ...}
```

## Appendix C — Servo tick limits (informative, never on the wire)

Background for maintainers: the safe ranges behind the normalized units, as enforced by
the servo layer (`JOINT_LIMITS_TICKS`; tick = `2048 + sign·rad·2048/π`).

| Physical joint | Tick range |
|---|---|
| `head_pan` | 1348 – 2748 |
| `head_tilt` | 1800 – 2150 |
| `eyes_pan` | 1846 – 2250 |
| `eyes_tilt` | 1600 – 2200 |
| `left_eyelid` | 1448 – 2096 |
| `right_eyelid` | 2000 – 2600 (mirrored sign) |
| `mouth` | 1960 – 2500 |
