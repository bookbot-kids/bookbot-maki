# LED ring white flashes — a hardware fault, not a software one

The 48-pixel NeoPixel ring intermittently shows white flashes. This is
**electrical noise from the Dynamixel servo bus coupling into the LED data
line**. No change to `maki_puppet` can fix it. This note records how that was
established, so it does not get re-diagnosed as an animation bug.

## Symptom

Random pixels flash white during any animation that writes the strip
continuously (`chase_rainbow` most visibly). It looks intermittent and
viewing-angle dependent, which invites — and repeatedly survived — wrong
explanations about colour maths, brightness and animation speed.

## What it is not

Ruled out by measurement, in this order:

- **Not the colour maths.** A diagnostic (`MAKI_LED_TRACE=1`, see
  `LedRing._trace_near_white`) logs any frame whose channels sit close together,
  i.e. any grey or white value, with the stack of whoever wrote it. Across full
  rating and page-turn sequences it logged **zero** hits. Every value the code
  writes is a correct, saturated colour.
- **Not the rainbow's parameters.** `_render_chase_rainbow` and `hue_to_rgb` are
  line-for-line identical to the vendored ROS node
  (`maki-rpi5-v0.2.13-rc.4/src/maki_led/maki_led/led_ring_node.py`). The ring is
  clean in isolation at *every* brightness, hue span and period tried, including
  the fast 0.5 s revolution.
- **Not CPU load.** Six CPU-burner processes on four cores: clean. A 50 Hz
  in-process CPU thread: clean.
- **Not servo power.** Servo bus open with torque energised and the head held:
  clean. Head sweeping its full range adds nothing beyond what the serial
  traffic alone already causes.
- **Not USB bandwidth or SPI contention on the RP1.** The USB camera streaming
  640x480@30 — far more USB traffic than servo polling — is **clean**.
- **Not 3.3 V vs 5 V logic levels.** The bare `neopixel_spi` driver, with library
  default and corrected WS2812 bit timings, is clean on solid colour changes.

## What it is

The flashes require **both** active SPI writes **and** active traffic on the
servo serial bus. Neither alone does it.

| condition | SPI writes/s | result |
|---|---|---|
| LED ring alone, incl. `chase_rainbow` | 50 | clean |
| + 6 CPU burners (separate processes) | 50 | clean |
| + 50 Hz in-process CPU thread | 50 | clean |
| + servo bus open, torque on, head still | 50 | clean |
| + USB camera streaming 640x480@30 | 50 | clean |
| static colour held, serial active | **0** | clean |
| **+ servo serial being read** | 50 | **flashes** |
| + servo serial, frames resent 6x | 300 | flashes |
| + servo serial, render throttled to 12 fps | 12 | flashes |

The servo link is half-duplex Dynamixel at 1 Mbaud on an FT232H adapter, sharing
the robot's harness and ground with the LED data line. Its switching corrupts a
WS2812 bit, and a mis-latched pixel reads as white.

## Why no software workaround exists

Three plausible mitigations were implemented and measured. All failed, for
reasons that follow from the mechanism:

- **Lowering the serial read rate** (31 → 16 → 4.5 reads/s): no effect. The
  FT232H polls on its own latency timer (16 ms by default) regardless of how
  often the application calls `read()`.
- **Re-sending each frame** (3x, 6x): no effect. The strip displays whatever the
  *last* write produced, so resending does not lower the odds that the final
  write is the corrupted one. Resends only repair a *static* frame, where a later
  good write can overwrite a bad one and nothing follows to re-corrupt it.
- **Lowering the render rate** (44 → 23 → 12 fps): no effect. Fewer corrupted
  frames, but each persists proportionally longer (83 ms at 12 fps vs 20 ms at
  50 fps), so the perceived flashing is unchanged.

## The fix (hardware)

Cheapest first:

1. **330–470 Ω series resistor** on the LED data line, at the Pi end. Standard
   NeoPixel practice; damps reflections on the data line.
2. **Route the LED data wire away from the servo harness.** Do not bundle them
   or run them parallel over any distance.
3. **Twist the LED data wire with its own ground return** back to the Pi.
4. **Star-ground** the Pi, the LED ring and the servo supply to a single point,
   rather than daisy-chaining grounds.

## Reproducing

`MAKI_LED_TRACE=1` on the gateway logs any washed-out frame with a stack trace —
use it first to confirm the software is still innocent before touching wiring:

```bash
ssh lux@<robot>
cd ~/bookbot-maki && MAKI_LED_TRACE=1 bash scripts/run_puppet.sh
grep "washed-out" ~/run_puppet_log.txt      # expect zero hits
```

To reproduce the fault itself, drive the ring continuously while toggling servo
serial traffic on and off; the flashes track the serial traffic exactly.
