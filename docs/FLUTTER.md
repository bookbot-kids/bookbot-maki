# Driving MAKI from Flutter

This is the guide for the Bookbot reading app (or any Flutter app) talking to the MAKI
robot. **You do not need to know anything about robots to use this.** You send small
JSON messages over a WebSocket saying *what just happened in the app* ("the child
finished a page"), and the robot reacts — moves its head, blinks, changes its light
ring. What the robot physically does for each event is configured on the robot itself,
not in your app.

- Wire spec (if you ever need the fine print): [`../PROTOCOL.md`](../PROTOCOL.md)
- Dart client: `maki_flutter_poc/lib/maki_client.dart` (+ `maki_messages.dart`)
- Robot endpoint: `ws://<robot-ip>:8765/ws` — e.g. `ws://192.0.2.10:8765/ws`

**The one-sentence mental model:** call `emit('<event>')` at natural moments of the
reading session, and MAKI comes alive. Everything else in this document is detail.

---

## 1. Quick start — first reaction in under a minute

You don't need the client library to prove the connection works. Add the WebSocket
package:

```yaml
# pubspec.yaml
dependencies:
  web_socket_channel: ^3.0.0
```

Then this is a complete program that makes the robot celebrate:

```dart
import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';

Future<void> main() async {
  final ch = WebSocketChannel.connect(Uri.parse('ws://<robot-ip>:8765/ws'));

  int n = 0;
  void send(String type, Map<String, Object?> payload) => ch.sink.add(jsonEncode({
        'type': type,
        'id': 'c-${++n}',
        'ts': DateTime.now().millisecondsSinceEpoch,
        'payload': payload,
      }));

  // 1. Handshake — must be the first frame.
  send('hello', {
    'protocol': 1,
    'client': {'name': 'quickstart', 'kind': 'flutter', 'version': '0.0.1'},
  });
  await ch.stream.first; // the server's `welcome`

  // 2. Make MAKI celebrate.
  send('event', {'name': 'celebrate', 'params': {}});
}
```

Run it; the robot does a happy wiggle with a rainbow light sweep. That's the whole
protocol in miniature: a `hello`, then `event` frames. For real app code, use
`MakiClient` (§3), which handles reconnect, acks, and heartbeats for you.

---

## 2. The event catalog — what to send, and when

These are the **semantic events** the robot currently understands. Send them at the
matching moment in the reading session; the robot's choreography file decides the
physical reaction, so this table's "what MAKI does" column may be retuned on the robot
without any app change.

| Event | Send it when… | Params | What MAKI physically does (today) |
|---|---|---|---|
| `reading_started` | A reading session begins (book opened, first page shown). | — | Wakes up: reading blue, perks its head up, looks at the reader, blinks. |
| `page_turned` | The reader turns to a new page. | — | Glances down toward the book, blinks, settles back to neutral. |
| `word_read` | The child reads a word correctly. **High-frequency — always fire-and-forget (§4.2).** | `word` (string, optional — for logging/observers) | A quick blink. Deliberately tiny; if MAKI is busy, the event is silently dropped so per-word events can never lag the robot. |
| `word_struggled` | The child hesitates or gets a word wrong. | `word` (string, optional) | Leans in with a curious head tilt, purple light. Deliberately warm — curiosity, not judgement. |
| `sentence_read` | A full sentence completed. | — | **Nothing yet** (accepted, reserved for future choreography). Safe to send now. |
| `celebrate` | Page/chapter finished, streak achieved, goal met. | — | Happy double head wiggle under the reading blue. |
| `encourage` | The child needs a nudge to keep going. | — | A nod. |
| `attention` | The child looked away / app wants eyes back on the book. | — | An energetic wake-up motion. |
| `reading_finished` | Session ends normally. | — | Proud double nod under the book-end green, which stays on. |
| `sleep` | App going to background / long pause. | — | Eyes droop, dim white slow breathing. |
| `wake` | Returning from `sleep`. | — | Perks up, back to the resting white. |

Notes:

- `params` is a free-form JSON map. Unknown params are harmless — they're stored with
  the event and rebroadcast to observer clients. Sending `word` with
  `word_read`/`word_struggled` is recommended for future use even though the robot
  ignores it today.
- Don't invent event names — an unknown name returns an error ack (`unknown_event`).
  The live catalog is in `client.serverInfo!.events` after connecting.

### The exact bytes on the wire

Every event is one JSON text frame. This is what `emit('word_read', params: {'word':
'elephant'})` actually sends:

```json
{ "type": "event", "id": "c-42", "ts": 1751421003000, "payload": {
    "name": "word_read",
    "params": { "word": "elephant" }
}}
```

And the frames for each catalog entry differ only in `payload.name`/`payload.params`:

```json
{ "type": "event", "id": "c-10", "ts": 1751421000000, "payload": { "name": "reading_started", "params": {} } }
{ "type": "event", "id": "c-11", "ts": 1751421001000, "payload": { "name": "page_turned", "params": {} } }
{ "type": "event", "id": "c-12", "ts": 1751421002000, "payload": { "name": "word_read", "params": { "word": "cat" } } }
{ "type": "event", "id": "c-13", "ts": 1751421003000, "payload": { "name": "word_struggled", "params": { "word": "rhinoceros" } } }
{ "type": "event", "id": "c-14", "ts": 1751421004000, "payload": { "name": "sentence_read", "params": {} } }
{ "type": "event", "id": "c-15", "ts": 1751421005000, "payload": { "name": "celebrate", "params": {} } }
{ "type": "event", "id": "c-16", "ts": 1751421006000, "payload": { "name": "encourage", "params": {} } }
{ "type": "event", "id": "c-17", "ts": 1751421007000, "payload": { "name": "attention", "params": {} } }
{ "type": "event", "id": "c-18", "ts": 1751421008000, "payload": { "name": "reading_finished", "params": {} } }
{ "type": "event", "id": "c-19", "ts": 1751421009000, "payload": { "name": "sleep", "params": {} } }
{ "type": "event", "id": "c-20", "ts": 1751421010000, "payload": { "name": "wake", "params": {} } }
```

The server answers each with one or more `ack` frames referencing your `id` — the
client library consumes those for you.

### App events — Bookbot UI events via the bridge

Alongside the choreographed events above, the gateway accepts **app events**: raw
Bookbot UI moments. These route to Python code (`maki_puppet/bridge.py`,
class `AppEventBridge`) instead of `choreographies.yaml` — one `on_<event>` handler
per event, platform-channel style. **All 21 are implemented** — sending them
produces real robot behaviour today (see the table below and PROTOCOL.md §10.2b).
Retuning that behaviour means editing `bridge.py`; the app never changes.

Use the typed sender in `maki_flutter_poc/lib/maki_bridge.dart` rather than
hand-writing names — every method is fire-and-forget and safe to call straight from
a tap handler:

```dart
final events = MakiAppEvents(maki);   // wraps your connected MakiClient

events.tapProfile('Mia');
events.tapBook(book: 'The Big Red Dog', level: '7');
events.practiceCorrect(word: 'elephant');
events.tapPage(12);
events.bookRate(5);
```

| App event | Dart method | Params on the wire |
|---|---|---|
| `tap_profile` | `tapProfile(profile)` | `profile` (string) |
| `tap_category` | `tapCategory(category)` | `category` (string) |
| `tap_series` | `tapSeries(series)` | `series` (string) |
| `tap_book` | `tapBook(book:, level:)` | `book`, `level` (strings) |
| `tap_starred` | `tapStarred(book:, level:)` | `book`, `level` (strings) |
| `close_book` | `closeBook(book:, level:)` | `book`, `level` (strings) |
| `practice_correct` | `practiceCorrect(word:)` | `word` (string) |
| `practice_incorrect` | `practiceIncorrect(word:)` | `word` (string) |
| `focus_word_correct` | `focusWordCorrect(word:)` | `word` (string) |
| `focus_word_incorrect` | `focusWordIncorrect(word:)` | `word` (string) |
| `read_to_me` | `readToMe()` | — |
| `listen` | `listen()` | — |
| `mute` | `mute()` | — |
| `tap_page` | `tapPage(page)` | `page` (int) |
| `book_rate` | `bookRate(rating)` | `rating` (int, 1–5) |
| `show_library` | `showLibrary()` | — |
| `practice_start` | `practiceStart(book:)` | `book` (string) |
| `page_start` | `pageStart(page)` | `page` (int) |
| `reading_word_incorrect` | `readingWordIncorrect(word:)` | `word` (string) |
| `page_end` | `pageEnd(page, errors:)` | `page` (int), `errors` (int) |
| `book_end` | `bookEnd(book:, level:)` | `book`, `level` (strings) |

The last six are new and **not yet in `maki_bridge.dart`** — add them there with
exactly these names and params. When to send each:

- `show_library` — whenever the library screen appears, including the return from a finished book.
- `practice_start` — when the practice-words phase begins, before the first word.
- `page_start` — when the child begins reading a page.
- `reading_word_incorrect` — the moment a word turns purple on screen while reading.
- `page_end` — when the child finishes the page, while it is still on screen; `errors` = words flagged on it.
- `book_end` — when the last page is finished, before the rating screen.

What they do: the ring follows the session — neutral white in the library and on
book open, continuous purple through the practice words, blue while reading,
purple **immediately** for a flagged word (back to blue after 2 s), purple at a page
end with errors, and a brighter green from `book_end` (and `book_rate`) until
`show_library`. A correct word never changes the colour. MAKI follows the child's
face throughout and glances down only at `page_start` and at an error-free
`page_end`. Full table: PROTOCOL.md §10.2b. Colour vocabulary: white = library,
blue = reading, purple = needs practice, green = book end only.

Semantics: a handler acks `completed` (detail `"bridge"`). A handler that raises returns an `error` ack
with code `internal`; the app should treat that as non-fatal (log and move on).
These names appear in `welcome.events` next to the choreographed ones, and bridge
names shadow any same-named choreography.

---

## 3. `MakiClient` — the API you actually use

`maki_flutter_poc/lib/maki_client.dart` implements the protocol; message/value types
live in `maki_flutter_poc/lib/maki_messages.dart`. The full surface:

```dart
enum MakiConnectionStatus { disconnected, connecting, connected }

class MakiClient {
  MakiClient(Uri uri, {String name = 'bookbot-flutter', int priority = 50});

  Future<void> connect();      // opens socket + hello/welcome handshake
  Future<void> disconnect();   // graceful close; cancels auto-reconnect

  ValueListenable<MakiConnectionStatus> get status;
  Welcome? get serverInfo;     // null until connected; catalogs live here

  Stream<RobotState> get states;   // robot state pushes (pose, LED, e-stop, ...)
  Stream<EventMsg> get events;     // semantic events emitted by OTHER clients

  // ---- Semantic events: the primary API ----
  Future<AckStatus> emit(String name, {Map<String, Object?> params = const {}});
  void emitFireAndForget(String name, {Map<String, Object?> params = const {}});

  // ---- Direct actions: the escape hatch (§5) ----
  Future<AckStatus> blink();
  Future<AckStatus> look({double pan = 0, double tilt = 0, int durationMs = 600});
  Future<AckStatus> led({String? animation, Color? color});
  Future<AckStatus> say(String text);              // TTS is deferred — returns AckStatus.error today
  Future<AckStatus> gesture(String name, {int repeat = 1});
  Future<AckStatus> act(Step step, {String priority = 'normal', String onBusy = 'queue'});

  Future<void> cancelAll();
  Future<void> estop({bool engage = true});
}
```

```dart
// maki_messages.dart (the parts you'll touch)
enum AckStatus { accepted, queued, started, completed, cancelled, superseded, dropped, error }

sealed class Step {}
class ActionStep extends Step { ActionStep(String kind, [Map<String, Object?> args]); }
class SeqStep extends Step    { SeqStep(List<Step> steps); }
class ParStep extends Step    { ParStep(List<Step> steps); }

class RobotState { /* pose (Map<String,double>), ledAnimation, queueDepth, estop, ... */ }
class EventMsg   { /* name, params, origin */ }
class Welcome    { /* capabilities, animations, gestures, events, joints, session */ }
```

### 3.1 Setup

```dart
final maki = MakiClient(
  Uri.parse('ws://127.0.0.1:8765/ws'),   // app runs ON the robot — use loopback
  name: 'bookbot-reading-app',
);
await maki.connect();                     // ← required; without it every send no-ops
```

> **Do not skip `connect()`.** Constructing a `MakiClient` does not open the socket.
> Every send path starts with `if (!_connected) return;`, so a client that was built
> but never connected silently discards everything you send it, with no error and no
> log. Use loopback rather than the robot's LAN address, which changes.

Create **one** `MakiClient` for the app's lifetime (e.g. in a provider/singleton).
`connect()` completes after the handshake; after that `maki.serverInfo` holds the
server's catalogs and the client silently maintains the connection (heartbeats,
reconnect) until you call `disconnect()`.

### 3.2 Connection status in the UI

`status` is a `ValueListenable`, so it plugs straight into a `ValueListenableBuilder`:

```dart
ValueListenableBuilder<MakiConnectionStatus>(
  valueListenable: maki.status,
  builder: (context, s, _) => Icon(
    Icons.smart_toy,
    color: switch (s) {
      MakiConnectionStatus.connected => Colors.green,
      MakiConnectionStatus.connecting => Colors.amber,
      MakiConnectionStatus.disconnected => Colors.grey,
    },
  ),
)
```

Design your UI so the robot is an **optional companion**: the reading experience must
work identically when the icon is grey (§6).

---

## 4. Emitting events

### 4.1 `emit` — when you care about the outcome

`emit` returns a `Future<AckStatus>` that resolves when the robot **finishes** (or
rejects) the reaction:

```dart
final result = await maki.emit('celebrate');
// result is one of:
//   AckStatus.completed   — the choreography played to the end
//   AckStatus.superseded  — a newer/higher-priority action replaced it midway
//   AckStatus.cancelled   — someone cancelled it (or e-stop engaged)
//   AckStatus.dropped     — robot was busy and this event is configured to drop
//   AckStatus.error       — rejected (unknown event, e-stop engaged, ...)
```

Treat anything other than `error` as success — `superseded`/`cancelled`/`dropped` are
*normal arbitration outcomes*, not failures. A `celebrate` interrupted by the next
`page_turned` did its job.

```dart
if (await maki.emit('reading_finished') == AckStatus.error) {
  log.warning('robot rejected reading_finished'); // don't surface to the child
}
```

If the client is offline, `emit` **throws `MakiOfflineException` immediately** (it
never hangs waiting for a robot that isn't there) — see §6.

### 4.2 `emitFireAndForget` — for high-frequency events (use this for `word_read`)

```dart
void onWordRecognized(String word) {
  maki.emitFireAndForget('word_read', params: {'word': word});
}
```

`emitFireAndForget` sends the frame and returns immediately — no future, no throw,
and it silently no-ops while offline.

**Rule of thumb: anything you'd call from a scroll handler, tap handler, or per-word
recognizer callback must be `emitFireAndForget`, never `await emit(...)`.** Two
reasons:

1. An awaited future in a hot path stalls your interaction handling on network round
   trips (and on the robot physically moving, since the future resolves at
   *completion*).
2. Reading events are ephemeral. If the robot missed a `word_read` because it was
   offline or busy, the correct behavior is to skip it, not to retry or queue it. The
   robot side agrees: `word_read` is configured `on_busy: drop` in its choreography.

Use `emit` (awaited or not) for the low-frequency milestones where you might want to
know the outcome: `reading_started`, `celebrate`, `reading_finished`. Even for those,
`unawaited(maki.emit(...))` or plain `emitFireAndForget` is fine if you don't care.

### 4.3 Listening to robot state and other clients' events

```dart
// Robot state: pose, current LED animation, e-stop, queue depth.
// Pushed on change + a 1 Hz heartbeat.
final sub = maki.states.listen((s) {
  if (s.estop) showEmergencyBanner();
});

// Events emitted by OTHER clients (e.g. a Python test script poking the robot).
maki.events.listen((e) => log.fine('robot event ${e.name} from ${e.origin}'));
```

Both are broadcast streams — multiple listeners are fine; remember to cancel
subscriptions in `dispose()`.

---

### 4.4 `mouth` — the viseme stream (lip sync)

Mouth motion has its own frame type, separate from events and acts. Send one sample
per animation frame while audio is playing, at whatever rate your viseme source
produces (30–60 Hz is typical):

```dart
/// Call this from your audio/viseme callback while speech is playing.
void onViseme(double openness) {   // 0.0 = closed, 1.0 = fully open
  maki.mouth(openness);
}

/// Always close the mouth when speech ends.
void onSpeechEnd() {
  maki.mouth(0.0);
}
```

If your `MakiClient` predates this, the method is four lines — it follows the same
shape as `emitFireAndForget`:

```dart
/// One viseme sample. Fire-and-forget: no ack, no future, no throw.
void mouth(double openness) {
  if (!_connected) return;
  try {
    _sendEnvelope('mouth', {'openness': openness.clamp(0.0, 1.0)});
  } on Object {
    // fire-and-forget never throws
  }
}
```

**Why this is a separate frame and not an act.** The mouth has its own arbitration
channel, so a viseme stream never contends with head/eye gestures — MAKI can nod and
talk at the same time. And unlike `act`, a `mouth` frame bypasses the action queue
entirely: no ack comes back, because a sender at 60 Hz cannot consume one per sample.
Measured on the robot, a frame costs ~0.35 ms to send and reaches the servo within one
50 Hz motion tick (0–20 ms).

Things to know:

- **Never `await` this** and never send it as `act` — one `{"kind": "mouth"}` act per
  frame would fight every gesture for the motion channel.
- **Send `0.0` when speech ends.** If the stream just stops, the mouth holds its last
  position for ~1.5 s before fading.
- **Nothing else touches the mouth.** Idle behavior and `neutral` both leave it alone,
  so a choreography ending mid-word will not snap it shut.
- **Feature-detect** with `serverInfo.capabilities.contains('mouth')` if you need to
  support older gateways.

---

## 5. Direct actions — the escape hatch

Semantic events are the intended path: they keep the app free of choreography and let
the robot's personality be tuned on the robot. But for debug screens, robot-settings
pages, or one-off effects, `MakiClient` exposes direct actions:

```dart
await maki.blink();
await maki.look(pan: -0.5, tilt: 0.2);        // pan/tilt are normalized -1..1, 0 = straight ahead
await maki.gesture('nod', repeat: 2);          // names from serverInfo!.gestures
await maki.led(animation: 'thinking_pulse_blue'); // names from serverInfo!.animations
await maki.led(color: const Color(0xFF0060B4)); // or a solid color (alpha ignored)
```

`say(text)` exists in the API but **TTS is not enabled on the robot yet** — it returns
`AckStatus.error` (code `tts_unavailable` on the wire). Narration audio belongs in the
app for now.

### Composing with `Step`

For multi-part effects, build a `Step` tree — `SeqStep` runs children in order,
`ParStep` runs them simultaneously (limits: nesting ≤ 4, ≤ 64 actions):

```dart
await maki.act(
  SeqStep([
    ParStep([
      ActionStep('gesture', {'name': 'nod', 'repeat': 2}),
      ActionStep('led', {'animation': 'attention_sweep'}),
    ]),
    ActionStep('led', {'animation': 'breathing_cyan'}),
  ]),
  priority: 'high',
  onBusy: 'replace',
);
```

Action kinds and their arguments are specified in
[`../PROTOCOL.md` §6](../PROTOCOL.md). `priority` is `'low' | 'normal' | 'high'`;
`onBusy` is `'queue' | 'replace' | 'drop'` and controls what happens if the robot is
already doing something.

**If you find yourself composing the same `Step` tree in more than one place, stop:**
that composition should become a named event in the robot's `choreographies.yaml`, and
your app should just `emit` it.

---

## 6. Reconnect, offline behavior, e-stop

### Reconnect (automatic)

After `connect()` succeeds once, `MakiClient` owns the connection: if the socket drops
it retries with exponential backoff (0.5 s doubling to an 8 s cap, with jitter) until
it gets a `welcome` again, updating `status` along the way. You never call `connect()`
in a retry loop yourself. `disconnect()` stops the retrying.

### Offline behavior (deterministic, by design)

While `status` is not `connected`:

| Call | Behavior |
|---|---|
| `emitFireAndForget(...)` | Silently does nothing. |
| `emit`, `blink`, `look`, `led`, `gesture`, `say`, `act`, `cancelAll`, `estop` | Throw `MakiOfflineException` **immediately**. |
| Futures in flight when the connection drops | Complete with `MakiOfflineException`. |

Nothing is ever queued client-side for later delivery — a `celebrate` from two minutes
ago replayed on reconnect would be creepy, not delightful. The practical pattern:

```dart
Future<void> safeEmit(String name) async {
  try {
    await maki.emit(name);
  } on MakiOfflineException {
    /* robot's not here — the reading session continues without it */
  }
}
```

…or just use `emitFireAndForget` and never think about it.

### E-stop

```dart
await maki.estop();              // freeze: motion holds still, lights go red,
                                 // everything queued is cancelled
await maki.estop(engage: false); // release: robot resumes normal behavior
```

If your app has any physical-safety surface (e.g. a grown-ups screen), wire a stop
button to `estop()`. While engaged, every `emit`/action returns `AckStatus.error`
(code `estopped`), and `RobotState.estop` is `true` on the `states` stream — show a
banner and offer the release action.

---

## 7. Worked example — a complete reading session

A thin, app-facing wrapper: the reading flow calls these methods at natural moments and
never touches JSON, acks, or robot concepts. Copy it as a starting point.

```dart
import 'dart:async';
import 'package:flutter/foundation.dart';
import 'maki_client.dart';
import 'maki_messages.dart';

/// The reading app's entire robot integration.
/// Safe to call every method with no robot present: milestone emits swallow
/// offline errors, per-word emits are fire-and-forget no-ops.
class ReadingSessionRobot {
  ReadingSessionRobot(Uri robotUri)
      : _maki = MakiClient(robotUri, name: 'bookbot-reading-app');

  final MakiClient _maki;
  StreamSubscription<RobotState>? _stateSub;

  /// True when the robot is connected and not emergency-stopped.
  final ValueNotifier<bool> robotAvailable = ValueNotifier(false);

  Future<void> start() async {
    _stateSub = _maki.states.listen((s) {
      robotAvailable.value =
          _maki.status.value == MakiConnectionStatus.connected && !s.estop;
    });
    _maki.status.addListener(() {
      if (_maki.status.value != MakiConnectionStatus.connected) {
        robotAvailable.value = false;
      }
    });
    try {
      await _maki.connect(); // keeps retrying in the background after this
    } on Object {
      /* robot unreachable at launch — reconnect loop is already running */
    }
  }

  // ---- Session milestones (low-frequency: emit, swallow offline) ----

  Future<void> sessionStarted() => _milestone('reading_started');
  Future<void> pageTurned() => _milestone('page_turned');
  Future<void> sessionFinished() => _milestone('reading_finished');
  Future<void> celebrate() => _milestone('celebrate');
  Future<void> encourage() => _milestone('encourage');
  Future<void> callAttention() => _milestone('attention');

  Future<void> _milestone(String name) async {
    try {
      final status = await _maki.emit(name);
      if (status == AckStatus.error) {
        debugPrint('maki rejected $name');
      }
    } on MakiOfflineException {
      /* no robot — reading continues without it */
    }
  }

  // ---- Per-word signals (high-frequency: ALWAYS fire-and-forget) ----

  void wordRead(String word) =>
      _maki.emitFireAndForget('word_read', params: {'word': word});

  void wordStruggled(String word) =>
      _maki.emitFireAndForget('word_struggled', params: {'word': word});

  // ---- Lifecycle ----

  Future<void> appPaused() => _milestone('sleep');
  Future<void> appResumed() => _milestone('wake');

  Future<void> emergencyStop() => _maki.estop();
  Future<void> releaseEmergencyStop() => _maki.estop(engage: false);

  Future<void> dispose() async {
    await _stateSub?.cancel();
    await _maki.disconnect();
    robotAvailable.dispose();
  }
}
```

And the reading flow uses it like this:

```dart
final robot = ReadingSessionRobot(Uri.parse('ws://<robot-ip>:8765/ws'));
await robot.start();

await robot.sessionStarted();            // MAKI wakes up and looks at the reader

for (final page in book.pages) {
  await robot.pageTurned();              // glance down at the page
  for (final word in page.words) {
    final ok = await listenForWord(word);      // your speech pipeline
    ok ? robot.wordRead(word.text)             // blink (fire-and-forget)
       : robot.wordStruggled(word.text);       // curious lean-in
  }
  await robot.celebrate();               // page done — rainbow wiggle
}

await robot.sessionFinished();           // proud nod, settle to idle
```

That's a full session: `reading_started` → per-page `page_turned` → a stream of
`word_read`/`word_struggled` → `celebrate` → `reading_finished`.

---

## 8. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `connect()` never completes / times out | Wrong IP/port, robot process not running, or you're not on the robot's network. `curl http://<robot-ip>:8765` should at least refuse politely; the WS endpoint is `/ws`. |
| Socket closes immediately with code 4401 | A frame was sent before `hello`. Using raw sockets? Send `hello` first. |
| Socket closes with code 4400 | Protocol version mismatch — update the client library. |
| `emit` returns `AckStatus.error` for a name in this doc | Robot's `choreographies.yaml` was edited — trust `serverInfo!.events` over this document. |
| Everything returns `AckStatus.error` suddenly | E-stop is engaged (`RobotState.estop == true`). Release via `estop(engage: false)`. |
| `word_read` reactions feel "skipped" | Working as intended: it's `on_busy: drop` so per-word events never queue up lag. |
| Robot moves but light ring doesn't (or vice versa) | Another client holds a channel lock, or a higher-priority action owns that channel. Check `RobotState` (`queue`, `lock`). |
| Frames > 64 KB close the socket (code 1009) | Don't put large blobs in `params`. |

For anything deeper, read [`../PROTOCOL.md`](../PROTOCOL.md) and watch the raw frames —
the protocol is small, and every server response references your message `id` in
`payload.ref`.
