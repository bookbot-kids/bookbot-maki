# MPP/1 wire fixtures — the anti-drift mechanism

One JSON file per protocol message type/variant, each a **complete, valid MPP/1
envelope** exactly as it would appear in a WebSocket text frame. The normative spec is
[`../../PROTOCOL.md`](../../PROTOCOL.md); these files are its machine-readable mirror
— every field name, type, enum value and unit here must match the spec (example values
may differ; shapes may not).

## Why these exist

The Python gateway and the Dart client are implemented independently against
PROTOCOL.md. Both test suites parse **these same files**:

- **Python** (`maki_puppet/tests/`): every fixture must round-trip through
  `maki_puppet.protocol` (parse → validate → serialize → parse) without error; the
  client→gateway fixtures must be *accepted* by the server-side validator, and the
  gateway→client fixtures must match what the server actually emits (field names and
  types).
- **Dart** (`maki_flutter_poc/test/`): every gateway→client fixture must decode into
  the sealed message classes in `maki_messages.dart`; every client→gateway fixture
  must equal (modulo `id`/`ts`) what `MakiClient` encodes for the corresponding call.

If either side changes the wire format, its fixture test breaks *here*, in one place,
before the two implementations can drift apart. **Never fix a failing fixture test by
editing the fixture to match your code** — first decide what the protocol should be,
update PROTOCOL.md, then the fixture, then both implementations.

## Inventory

| File | Direction | Shows |
|---|---|---|
| `hello.json` | client → gw | Handshake with all optional fields present. |
| `welcome.json` | gw → client | Handshake reply with full live catalogs (joints/animations/gestures/events); `capabilities` excludes `tts` (deferred). |
| `act_simple.json` | client → gw | Single bare Action (`blink`) with `priority`/`on_busy`/`tag`. |
| `act_composed.json` | client → gw | `seq`/`par` nesting, `gesture`, `led`, `look`, `wait` — exercises the Step grammar. |
| `event.json` | client → gw | Semantic event with `params` (the primary Flutter path). |
| `event_rebroadcast.json` | gw → client | Same event as rebroadcast: identical payload plus `origin`. |
| `cancel.json` | client → gw | Tag-form target (`"tag:ui-idle"`). |
| `estop.json` | client → gw | Engage with `reason`. |
| `state_get.json` | client → gw | Field-filtered state request. |
| `lock.json` / `unlock.json` | client → gw | Channel lock with `ttl_s` / release. |
| `ping.json` | gw → client | Server heartbeat (empty payload). |
| `pong.json` | client → gw | Heartbeat reply; `payload.ref` = the ping id. |
| `ack_accepted.json` | gw → client | First ack; carries `channels`. |
| `ack_queued.json` | gw → client | Waiting; carries `queue_pos`. |
| `ack_started.json` | gw → client | Running; carries `progress {step, of}`. |
| `ack_completed.json` | gw → client | Terminal success for an `event`; carries `choreography` + `duration_ms`. |
| `ack_dropped.json` | gw → client | Terminal, single-ack (`on_busy: drop` while busy). |
| `ack_superseded.json` | gw → client | Terminal, victim of a `replace`. |
| `ack_cancelled.json` | gw → client | Terminal, via `cancel`/`estop`/`abort_on_disconnect`. |
| `ack_error.json` | gw → client | Terminal, with `code` + `detail`. |
| `state.json` | gw → client | Full state push: `pose`, `led`, `queue`, `estop`, `lock`, `clients`, `health`. |

## Conventions

- Envelope ids follow the spec convention: `c-*` for client-originated, `s-*` for
  server-originated frames; `ts` values are fixed example timestamps (parsers must not
  validate them).
- Fixtures include optional fields on purpose (they document the full shape); parsers
  must also accept the same messages with optional fields absent, and must ignore
  fields they don't know (PROTOCOL.md §13). Consider adding minimal-variant tests in
  code rather than as extra fixtures.
- Adding a message type or variant to the protocol ⇒ add a fixture here in the same
  change, and reference it from PROTOCOL.md §5.
